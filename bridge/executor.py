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
拼图程序解释执行器。

执行模型：
  1. 从 entry block 开始，沿 next 链顺序执行
  2. 遇到 control_repeat → 压栈 (next, count)，跳入 body
  3. body 末尾 next=null → 弹栈，计数减一，再进 body 或跳到 saved next
  4. 每步发出 BLOCK_ENTER / BLOCK_EXIT 事件
  5. 收到 stop 信号 → 置标志位，当前块执行完后退出

事件上报通过回调函数，由 ws_client 负责发给后端。
"""

import asyncio
import logging

from ros_client import RosClient

logger = logging.getLogger(__name__)

# v1 允许的块类型
V1_BLOCK_TYPES = {
    'event_start',
    'action_takeoff', 'action_land', 'action_rtl',
    'action_move_to', 'action_move_dir',
    'action_arm', 'action_disarm', 'action_set_mode',
    'action_move_relative', 'action_set_yaw',
    'action_set_velocity', 'action_set_max_speed',
    'control_wait', 'control_repeat',
    'sensor_altitude', 'sensor_battery_pct',
}


class ProgramExecutor:
    """
    无状态的拼图程序解释器。每次 run() 创建新的执行上下文。
    """

    def __init__(self, ros_client: RosClient, event_callback):
        """
        Args:
            ros_client: ROS2 服务客户端封装
            event_callback: async def callback(run_id, block_id, event, error=None)
                event ∈ {'STARTED', 'BLOCK_ENTER', 'BLOCK_EXIT', 'FINISHED', 'ERROR'}
        """
        self.ros = ros_client
        self.emit = event_callback
        self._stop_flag = False
        self.command_gap_seconds = 0.5

    def request_stop(self):
        """由外部调用（收到 program_stop 时），置停止标志。"""
        self._stop_flag = True

    async def _interruptible_sleep(self, duration: float, timeout: float):
        """
        分段 sleep，同时检查 _stop_flag 和总超时。

        Args:
            duration: 期望等待的秒数
            timeout: 最大允许等待秒数（超时则返回 False）
        Returns:
            True if completed normally, False if stopped or timed out.
        """
        elapsed = 0.0
        while elapsed < duration:
            if self._stop_flag:
                return False
            step = min(0.5, duration - elapsed, timeout - elapsed)
            if step <= 0:
                return False
            await asyncio.sleep(step)
            elapsed += step
            if elapsed >= timeout:
                return False
        return not self._stop_flag

    async def _post_block_delay(self, block_type: str):
        """Pause briefly between executable commands so PX4/ROS state can settle."""
        if self._stop_flag:
            return
        if block_type in ('event_start', 'control_repeat') or block_type.startswith('sensor_'):
            return
        if self.command_gap_seconds <= 0:
            return
        await self._interruptible_sleep(
            self.command_gap_seconds,
            timeout=self.command_gap_seconds + 0.5,
        )

    async def run(self, run_id: str, program: dict):
        """
        执行一段拼图程序。

        Args:
            run_id: 后端生成的执行 ID
            program: BlockProgram dict（必须包含 entry, blocks）
        """
        self._stop_flag = False
        self.command_gap_seconds = 0.5
        entry = program.get('entry')
        blocks = program.get('blocks', {})

        if not entry or entry not in blocks:
            await self.emit(run_id, None, 'ERROR', f'入口块不存在: {entry}')
            return

        await self.emit(run_id, None, 'STARTED')

        try:
            await self._execute_chain(run_id, entry, blocks)
            if self._stop_flag:
                await self.emit(run_id, None, 'ERROR', '用户中止')
            else:
                await self.emit(run_id, None, 'FINISHED')
        except Exception as e:
            logger.exception('执行异常')
            await self.emit(run_id, None, 'ERROR', str(e))
        finally:
            # Safety measure: attempt to land the drone whenever execution
            # ends due to a stop signal or an unhandled exception, so the
            # aircraft does not remain airborne in an undefined state.
            if self._stop_flag:
                asyncio.create_task(self._safe_land())

    async def _safe_land(self):
        """Attempt to land the drone. Errors are logged but never raised,
        so they cannot mask the original stop/error that triggered this.

        First tries emergency_land (fast, bypasses command lock).
        Falls back to regular land() if emergency_land fails.
        """
        try:
            ok, msg = await self.ros.emergency_land()
            if ok:
                logger.info('_safe_land: 紧急降落成功 (%s)', msg)
                return
            logger.warning('_safe_land: 紧急降落失败 (%s), 回退到普通降落', msg)
        except Exception:
            logger.exception('_safe_land: 紧急降落异常, 回退到普通降落')
        try:
            ok, msg = await self.ros.land()
            if ok:
                logger.info('_safe_land: 普通降落成功')
            else:
                logger.warning('_safe_land: 普通降落也失败 (%s)', msg)
        except Exception:
            logger.exception('_safe_land: 普通降落异常')

    async def _execute_chain(self, run_id: str, start_id: str, blocks: dict):
        """
        从 start_id 开始沿 next 链逐块执行。

        对 control_repeat 类型使用栈帧模拟循环。
        """
        # 栈帧: list of (resume_block_id, remaining_count)
        stack: list[tuple[str, int]] = []
        current = start_id

        while current is not None:
            if self._stop_flag:
                return

            block = blocks.get(current)
            if block is None:
                await self.emit(run_id, current, 'ERROR', f'blockId 不存在: {current}')
                return

            block_type = block.get('type', '')
            params = block.get('params') or {}

            # --- BLOCK_ENTER ---
            await self.emit(run_id, current, 'BLOCK_ENTER')

            # --- 执行块 ---
            try:
                await self._execute_block(run_id, current, block_type, params)
            except Exception as e:
                await self.emit(run_id, current, 'ERROR', str(e))
                return

            # --- BLOCK_EXIT ---
            await self.emit(run_id, current, 'BLOCK_EXIT')

            # --- 电量巡检：飞行中检测低电量 ---
            if block_type not in ('event_start', 'action_takeoff', 'action_land', 'action_rtl'):
                if self.ros.is_low_battery():
                    pct = self.ros.current_battery.get('percentage', '?')
                    logger.warning(f'检测到低电量 ({pct}%)，触发安全降落')
                    self._stop_flag = True

            if not self._stop_flag:
                await self._post_block_delay(block_type)

            # --- 决定下一步 ---
            if block_type == 'control_repeat':
                count = int(params.get('count', 1))
                body = block.get('body')
                next_after = block.get('next')

                if count > 0 and body:
                    # 压栈：循环结束后跳转到 next_after，剩余 count-1 轮
                    stack.append((current, count - 1, next_after, body))
                    current = body
                else:
                    current = next_after
            else:
                next_id = block.get('next')
                if next_id is not None:
                    current = next_id
                else:
                    # next 为 null → 看看栈上有没有循环帧要回去
                    current = self._pop_stack(stack)

    def _pop_stack(self, stack) -> str | None:
        """
        弹栈：如果有循环帧且剩余次数 > 0，回到 body 首块；
        否则弹掉该帧，跳到 next_after；如果栈空，返回 None（程序结束）。
        """
        while stack:
            repeat_block_id, remaining, next_after, body = stack[-1]
            if remaining > 0:
                # 还有循环剩余
                stack[-1] = (repeat_block_id, remaining - 1, next_after, body)
                return body
            else:
                stack.pop()
                if next_after is not None:
                    return next_after
                # next_after 也为 null，继续弹栈
        return None  # 栈空，程序结束

    async def _execute_block(self, run_id: str, block_id: str, block_type: str, params: dict):
        """根据块类型执行对应的动作。"""

        if block_type == 'event_start':
            # 入口块，什么都不做
            pass

        elif block_type == 'action_takeoff':
            # --- 电池预检 ---
            bat = self.ros.get_battery_status()
            if bat['is_unknown']:
                logger.warning('电池数据未知（PX4 SITL 可能不报告电量），允许起飞')
            elif bat['is_low']:
                raise RuntimeError(
                    f"电量过低，无法起飞: {bat['percentage']:.0f}%"
                )

            alt = float(params.get('altitude', 2.0))
            ok, msg = await self.ros.takeoff(alt, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'起飞失败: {msg}')
            # 等待实际到达目标高度（而非固定延时）
            reached, wait_msg = await self.ros.wait_until_altitude(
                target_alt=alt, timeout=30.0, tolerance=0.3,
                stop_check=lambda: self._stop_flag,
            )
            if not reached:
                raise RuntimeError(f'起飞等待失败: {wait_msg}')

        elif block_type == 'action_land':
            ok, msg = await self.ros.land(stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'降落失败: {msg}')
            # 等待实际着陆（ON_GROUND 状态）
            landed, wait_msg = await self.ros.wait_until_landed(
                timeout=30.0, stop_check=lambda: self._stop_flag,
            )
            if not landed:
                raise RuntimeError(f'降落等待失败: {wait_msg}')

        elif block_type == 'action_rtl':
            ok, msg = await self.ros.rtl(stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'返航失败: {msg}')
            # RTL 等待着陆（允许更长超时）
            landed, wait_msg = await self.ros.wait_until_landed(
                timeout=60.0, stop_check=lambda: self._stop_flag,
            )
            if not landed:
                raise RuntimeError(f'返航等待失败: {wait_msg}')

        elif block_type == 'action_move_to':
            x = float(params.get('x', 0.0))
            y = float(params.get('y', 0.0))
            z = float(params.get('z', 2.0))
            yaw = float(params.get('yaw', 0.0))
            ok, msg = await self.ros.move_to(
                x, y, z, yaw, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'移动失败: {msg}')

        elif block_type == 'action_move_dir':
            direction = params.get('direction', 'forward')
            distance = float(params.get('distance', 1.0))
            ok, msg = await self.ros.move_dir(
                direction, distance, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'方向移动失败: {msg}')

        elif block_type == 'action_arm':
            ok, msg = await self.ros.arm(True, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'解锁失败: {msg}')

        elif block_type == 'action_disarm':
            ok, msg = await self.ros.arm(False, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'上锁失败: {msg}')

        elif block_type == 'action_set_mode':
            mode = params.get('mode', 'AUTO.LOITER')
            ok, msg = await self.ros.set_mode(mode, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'模式切换失败: {msg}')
            if hasattr(self.ros, 'wait_until_mode'):
                reached, wait_msg = await self.ros.wait_until_mode(
                    mode, timeout=3.0, stop_check=lambda: self._stop_flag
                )
                if not reached:
                    raise RuntimeError(f'模式切换等待失败: {wait_msg}')

        elif block_type == 'action_move_relative':
            dx = float(params.get('dx', 0.0))
            dy = float(params.get('dy', 0.0))
            dz = float(params.get('dz', 0.0))
            dyaw = float(params.get('dyaw', 0.0))
            ok, msg = await self.ros.move_relative(
                dx, dy, dz, dyaw, stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'相对移动失败: {msg}')

        elif block_type == 'action_set_yaw':
            yaw = float(params.get('yaw', 0.0))
            is_relative = params.get('yaw_mode', 'absolute') == 'relative'
            ok, msg = await self.ros.set_yaw(yaw, is_relative,
                                             stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'偏航设置失败: {msg}')
            await self._interruptible_sleep(0.5, timeout=15.0)

        elif block_type == 'action_set_velocity':
            vx = float(params.get('vx', 0.0))
            vy = float(params.get('vy', 0.0))
            vz = float(params.get('vz', 0.0))
            duration = float(params.get('duration', 1.0))
            ok, msg = await self.ros.set_velocity(vx, vy, vz, duration,
                                                  stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'速度设置失败: {msg}')
            # 等速度命令执行完毕
            await self._interruptible_sleep(duration + 0.5, timeout=15.0)

        elif block_type == 'action_set_max_speed':
            max_h = float(params.get('max_horizontal', 0.0))
            max_v = float(params.get('max_vertical', 0.0))
            ok, msg = await self.ros.set_max_speed(max_h, max_v,
                                                    stop_check=lambda: self._stop_flag)
            if not ok:
                raise RuntimeError(f'最大速度设置失败: {msg}')

        elif block_type == 'control_wait':
            seconds = float(params.get('seconds', 1.0))
            if seconds <= 0:
                raise RuntimeError(f'等待时间必须大于 0 秒: {seconds}')
            logger.info(f'等待 {seconds} 秒开始')
            completed = await self._interruptible_sleep(seconds, timeout=seconds + 1.0)
            if not completed:
                raise RuntimeError(f'等待 {seconds} 秒被中止或超时')
            logger.info(f'等待 {seconds} 秒完成')

        elif block_type == 'control_repeat':
            # repeat 的"执行"不在这里——它的循环逻辑在 _execute_chain 的栈帧里处理。
            # 到这里只是标记进入，实际不做事。
            pass

        elif block_type == 'sensor_altitude':
            # 返回当前相对高度（米），供条件块或显示块使用
            pose = self.ros.current_pose
            altitude = pose.z if pose else None
            logger.info('sensor_altitude → %s', altitude)
            # altitude 值通过事件回报给后端
            await self.emit(run_id, block_id, 'BLOCK_RESULT', altitude)

        elif block_type == 'sensor_battery_pct':
            # 返回当前电池百分比 + 状态摘要
            bat = self.ros.get_battery_status()
            pct = bat['percentage']
            logger.info('sensor_battery_pct → %s (low=%s, unknown=%s)',
                        pct, bat['is_low'], bat['is_unknown'])
            await self.emit(run_id, block_id, 'BLOCK_RESULT', pct)

        else:
            raise RuntimeError(f'未知的块类型: {block_type}')
