#!/usr/bin/env python3
# Copyright (C) 2026 最小生成树 (Minimum Spanning Tree). All rights reserved.
#
# Author: xianglajituibao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Bridge Node 入口点。

启动流程：
  1. 初始化 rclpy + 创建 ROS2 节点
  2. 创建 RosClient（ROS2 service 封装）
  3. 创建 WsClient（连接后端 /ws/bridge）
  4. 创建 ProgramExecutor（拼图程序解释器）
  5. 在 asyncio 事件循环中并发运行：
     - ROS2 spin（在线程池中）
     - WebSocket 连接主循环
     - 遥测定时上报（1Hz）

用法：
  # 作为 ROS2 节点运行（推荐）
  ros2 run bridge_node bridge_node --ros-args -p backend_url:=ws://192.168.x.x:9200/ws/bridge

  # 直接运行
  python3 -m bridge_node.main --url ws://192.168.x.x:9200/ws/bridge
"""

import argparse
import asyncio
import logging
import sys
import threading

import rclpy
from rclpy.node import Node

from ros_client import RosClient
from executor import ProgramExecutor
from ws_client import WsClient

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(name)s %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger('bridge_node')


class BridgeNode(Node):
    """ROS2 节点壳。持有 RosClient，供 executor 和 telemetry 使用。"""

    def __init__(self):
        super().__init__('bridge_node')
        # 声明参数（ros2 run 时通过 --ros-args -p key:=value 传入）
        self.declare_parameter('backend_url', 'ws://localhost:9200/ws/bridge')
        self.declare_parameter('bridge_id', '')
        self.declare_parameter('telemetry_hz', 1.0)


# ---------- 当前活跃执行器（v1 只支持一个） ----------
_active_executor: ProgramExecutor | None = None
_executor_lock: asyncio.Lock | None = None  # initialized in run_bridge


async def run_bridge(node: BridgeNode):
    """主异步入口：并发运行 WS 连接 + 遥测上报。"""
    global _active_executor, _executor_lock
    _executor_lock = asyncio.Lock()

    # --- 读取参数 ---
    backend_url = node.get_parameter('backend_url').get_parameter_value().string_value
    bridge_id = node.get_parameter('bridge_id').get_parameter_value().string_value or None
    tele_hz = node.get_parameter('telemetry_hz').get_parameter_value().double_value

    logger.info('后端地址: %s', backend_url)

    # --- 初始化组件 ---
    ros_client = RosClient(node)
    ws_client = WsClient(backend_url, bridge_id)

    # --- 事件回调 ---
    async def event_callback(run_id, block_id, event, error=None):
        await ws_client.send_execution_event(run_id, block_id, event, error)

    # --- 收到 program_run ---
    async def on_program_run(run_id: str, program: dict):
        global _active_executor
        logger.info('收到程序: runId=%s, keys=%s', run_id, list(program.keys()) if isinstance(program, dict) else type(program).__name__)
        if isinstance(program, dict):
            entry = program.get('entry')
            blocks = program.get('blocks', {})
            logger.info('entry=%s, blocks数量=%d', entry, len(blocks))
            for bid, b in list(blocks.items())[:10]:
                logger.info('  block[%s]: type=%s, next=%s, params=%s', bid, b.get('type'), b.get('next'), b.get('params'))
        async with _executor_lock:
            if _active_executor is not None:
                logger.warning('已有执行在进行中，拒绝新任务 %s', run_id)
                await ws_client.send_execution_event(run_id, None, 'ERROR', '已有执行在进行中')
                return
            executor = ProgramExecutor(ros_client, event_callback)
            _active_executor = executor
        try:
            await executor.run(run_id, program)
        finally:
            async with _executor_lock:
                _active_executor = None

    # --- 收到 program_stop / program_estop ---
    async def on_program_stop(run_id: str, emergency: bool = False):
        global _active_executor
        async with _executor_lock:
            executor = _active_executor
        if executor is not None:
            logger.info('收到%s信号 runId=%s', '急停' if emergency else '停止', run_id)
            executor.request_stop()
        if emergency:
            ok, msg = await ros_client.emergency_stop()
            if ok:
                logger.warning('急停服务已调用: %s', msg)
            else:
                logger.error('急停服务调用失败: %s', msg)

    ws_client.on_program_run(on_program_run)
    ws_client.on_program_stop(on_program_stop)

    # --- 遥测定时上报 ---
    async def telemetry_loop():
        interval = 1.0 / tele_hz if tele_hz > 0 else 1.0
        try:
            while True:
                await asyncio.sleep(interval)
                if ws_client.connected:
                    snapshot = ros_client.get_telemetry_snapshot()
                    await ws_client.send_telemetry(snapshot)
        except asyncio.CancelledError:
            logger.info('遥测上报已停止')

    # --- 并发，支持优雅关闭 ---
    tasks = [
        asyncio.create_task(ws_client.connect_loop(), name='ws_connect'),
        asyncio.create_task(telemetry_loop(), name='telemetry'),
    ]
    try:
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        pass
    finally:
        # 停止活跃执行器
        async with _executor_lock:
            executor = _active_executor
        if executor is not None:
            executor.request_stop()
        # 取消所有任务
        for t in tasks:
            if not t.done():
                t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def spin_ros2_in_thread(node: Node):
    """在后台线程里跑 rclpy.spin，让主线程留给 asyncio。"""
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    # --- CLI 参数（非 ROS2 方式启动时用） ---
    parser = argparse.ArgumentParser(description='Bridge Node')
    parser.add_argument('--url', default=None, help='后端 WebSocket URL')
    parsed, remaining = parser.parse_known_args(sys.argv[1:])

    # --- ROS2 初始化 ---
    rclpy.init(args=remaining or args)
    node = BridgeNode()

    # 如果通过 --url 传入，覆盖 ROS2 参数
    if parsed.url:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('backend_url', Parameter.Type.STRING, parsed.url)])

    logger.info('Bridge Node 启动中... bridgeId=%s',
                node.get_parameter('bridge_id').get_parameter_value().string_value or '(auto)')

    # --- 后台线程跑 rclpy.spin ---
    spin_thread = threading.Thread(target=spin_ros2_in_thread, args=(node,), daemon=True)
    spin_thread.start()

    # --- 主线程跑 asyncio ---
    try:
        asyncio.run(run_bridge(node))
    except KeyboardInterrupt:
        logger.info('收到 Ctrl+C，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
