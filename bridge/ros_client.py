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
ROS2 服务客户端封装。

封装 ctrl_server 暴露的 5 个 service 为简单的异步方法，
供 executor.py 的逐块执行引擎调用。

Service 定义参照 uav_interfaces/srv:
  - Takeoff:  relative_alt → success, message
  - Land:     timeout      → success, message
  - Move:     x, y, z, yaw → success, message
  - Arm:      arm          → success, message
  - SetMode:  mode         → success, message
  - Rtl:      timeout      → success, message
"""

import asyncio
import math
from rclpy.node import Node
from uav_interfaces.srv import Takeoff, Land, Move, Arm, SetMode, Rtl
from uav_interfaces.srv import MoveRelative, SetYaw, SetVelocity, SetMaxSpeed
from uav_interfaces.srv import EmergencyLand, EmergencyStop
from uav_interfaces.msg import UavPose, UavState
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode as MavrosSetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RosClient:
    """
    薄封装层：把 ROS2 service call 包装成 async 方法。

    所有方法返回 (success: bool, message: str)。
    """

    LOW_BATTERY_THRESHOLD: float = 20.0  # 默认低电量阈值（百分比）

    def __init__(self, node: Node, low_battery_threshold: float = 20.0):
        self.node = node
        self.logger = node.get_logger()
        self._low_battery_threshold = low_battery_threshold

        # --- service clients ---
        self.takeoff_cli = node.create_client(Takeoff, 'uav/takeoff')
        self.land_cli = node.create_client(Land, 'uav/land')
        self.move_cli = node.create_client(Move, 'uav/move')
        self.arm_cli = node.create_client(Arm, 'uav/arm')
        self.set_mode_cli = node.create_client(SetMode, 'uav/set_mode')
        self.rtl_cli = node.create_client(Rtl, 'uav/rtl')
        self.move_relative_cli = node.create_client(MoveRelative, 'uav/move_relative')
        self.set_yaw_cli = node.create_client(SetYaw, 'uav/set_yaw')
        self.set_velocity_cli = node.create_client(SetVelocity, 'uav/set_velocity')
        self.set_max_speed_cli = node.create_client(SetMaxSpeed, 'uav/set_max_speed')
        self.emergency_land_cli = node.create_client(EmergencyLand, 'uav/emergency_land')
        self.emergency_stop_cli = node.create_client(EmergencyStop, 'uav/emergency_stop')

        # --- direct MAVROS clients (bypass ctrl_server async handles) ---
        self.mavros_arm_cli = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.mavros_set_mode_cli = node.create_client(MavrosSetMode, '/mavros/set_mode')

        # --- OFFBOARD setpoint publisher ---
        self._setpoint_pub = node.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self._setpoint_timer = None  # 用于连续发送坐标点的定时器

        # --- telemetry subscriptions ---
        self.current_pose: UavPose | None = None
        self.current_state: UavState | None = None
        self.current_battery: dict = {}  # voltage, current, percentage
        node.create_subscription(UavPose, '/uav/telemetry/pose', self._pose_cb, 10)
        node.create_subscription(UavState, '/uav/telemetry/state', self._state_cb, 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        node.create_subscription(BatteryState, '/mavros/battery', self._battery_cb, qos)

    # ---------- telemetry callbacks ----------

    def _pose_cb(self, msg: UavPose):
        self.current_pose = msg

    def _state_cb(self, msg: UavState):
        self.current_state = msg

    def _battery_cb(self, msg: BatteryState):
        self.current_battery = {
            'voltage': msg.voltage,
            'current': abs(msg.current),
            'percentage': msg.percentage * 100.0,
        }

    def update_battery(self, voltage: float = None, current: float = None,
                       percentage: float = None):
        """由外部（遥测订阅）调用以更新电池数据。"""
        if voltage is not None:
            self.current_battery['voltage'] = voltage
        if current is not None:
            self.current_battery['current'] = current
        if percentage is not None:
            self.current_battery['percentage'] = percentage

    # ---------- battery status helpers ----------

    def is_low_battery(self) -> bool:
        """检查电池电量是否低于阈值。"""
        pct = self.current_battery.get('percentage')
        if pct is None:
            return False
        return pct < self._low_battery_threshold

    def is_battery_unknown(self) -> bool:
        """检查电池数据是否缺失（percentage 为 None）。"""
        return self.current_battery.get('percentage') is None

    def get_battery_status(self) -> dict:
        """返回电池状态摘要。"""
        pct = self.current_battery.get('percentage')
        return {
            'percentage': pct,
            'voltage': self.current_battery.get('voltage'),
            'is_low': self.is_low_battery(),
            'is_unknown': self.is_battery_unknown(),
        }

    # ---------- service calls ----------

    async def takeoff(self, altitude: float, stop_check=None) -> tuple[bool, str]:
        """PX4 AUTO.TAKEOFF: set mode + arm. 直接调 MAVROS 拿真实结果。"""
        ok, msg = await self.set_mode('AUTO.TAKEOFF', stop_check=stop_check)
        if not ok:
            return False, f'set_mode AUTO.TAKEOFF failed: {msg}'
        ok, msg = await self.arm(True, stop_check=stop_check)
        if not ok:
            return False, f'arm failed: {msg}'
        return True, f'takeoff({altitude}) OK'

    async def land(self, timeout: float = 30.0, stop_check=None) -> tuple[bool, str]:
        """直接调 MAVROS set_mode AUTO.LAND，拿真实结果。"""
        ok, msg = await self.set_mode('AUTO.LAND', stop_check=stop_check)
        if not ok:
            return False, f'set_mode AUTO.LAND failed: {msg}'
        return True, 'land OK'

    async def move_to(self, x: float, y: float, z: float, yaw: float = 0.0,
                      stop_check=None) -> tuple[bool, str]:
        req = Move.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        return await self._call(self.move_cli, req, 'move', stop_check=stop_check)

    async def move_dir(self, direction: str, distance: float, stop_check=None) -> tuple[bool, str]:
        """相对移动：根据当前位姿 + 方向计算绝对坐标后调用 move。"""
        pose = self.current_pose
        if pose is None:
            return False, '无遥测数据，无法计算相对坐标'
        yaw = pose.yaw
        dx, dy = 0.0, 0.0
        if direction == 'forward':
            dx = distance * math.cos(yaw)
            dy = distance * math.sin(yaw)
        elif direction == 'backward':
            dx = -distance * math.cos(yaw)
            dy = -distance * math.sin(yaw)
        elif direction == 'left':
            dx = -distance * math.sin(yaw)
            dy = distance * math.cos(yaw)
        elif direction == 'right':
            dx = distance * math.sin(yaw)
            dy = -distance * math.cos(yaw)
        else:
            return False, f'未知方向: {direction}'

        target_x = pose.x + dx
        target_y = pose.y + dy
        self.logger.info(f'move_dir {direction} {distance}m → ({target_x:.2f}, {target_y:.2f}, {pose.z:.2f})')
        return await self.move_to(target_x, target_y, pose.z, yaw, stop_check=stop_check)

    async def arm(self, value: bool = True, stop_check=None) -> tuple[bool, str]:
        req = CommandBool.Request()
        req.value = value
        return await self._call(self.mavros_arm_cli, req, 'mavros/arm' if value else 'mavros/disarm',
                                stop_check=stop_check)

    async def set_mode(self, mode: str, stop_check=None) -> tuple[bool, str]:
        req = MavrosSetMode.Request()
        req.custom_mode = mode
        return await self._call(self.mavros_set_mode_cli, req, f'mavros/set_mode({mode})',
                                stop_check=stop_check)

    # ---------- OFFBOARD 连续坐标点发送 ----------

    def _start_setpoint_stream(self, x: float, y: float, z: float, yaw: float = 0.0):
        """启动 10Hz 定时器，连续发送坐标点到 /mavros/setpoint_position/local。"""
        self._setpoint_target = (x, y, z, yaw)
        if self._setpoint_timer is not None:
            self._setpoint_timer.cancel()
        # 用 rclpy Timer 实现 10Hz 发布
        self._setpoint_timer = self.node.create_timer(0.1, self._publish_setpoint)

    def _publish_setpoint(self):
        """定时器回调：发送当前目标坐标点。"""
        if not hasattr(self, '_setpoint_target') or self._setpoint_target is None:
            return
        x, y, z, yaw = self._setpoint_target
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # yaw → 四元数
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._setpoint_pub.publish(msg)

    def _stop_setpoint_stream(self):
        """停止连续坐标点发送。"""
        if self._setpoint_timer is not None:
            self._setpoint_timer.cancel()
            self._setpoint_timer = None
        self._setpoint_target = None

    async def move_to_offboard(self, x: float, y: float, z: float, yaw: float = 0.0,
                               timeout: float = 30.0, tolerance: float = 0.5,
                               stop_check=None) -> tuple[bool, str]:
        """
        OFFBOARD 模式移动：切换到 OFFBOARD → 连续发送坐标点 → 等待到达目标。
        这是真正的 OFFBOARD 移动，不依赖 ctrl_server。
        """
        # 1. 切换到 OFFBOARD
        ok, msg = await self.set_mode('OFFBOARD', stop_check=stop_check)
        if not ok:
            return False, f'OFFBOARD 模式切换失败: {msg}'
        # 给 PX4 时间接受 OFFBOARD 模式
        await asyncio.sleep(0.5)

        # 2. 启动连续坐标点发送（10Hz）
        self._start_setpoint_stream(x, y, z, yaw)
        self.logger.info(f'OFFBOARD 移动: 目标 ({x:.2f}, {y:.2f}, {z:.2f}), yaw={yaw:.2f}')

        # 3. 等待到达目标位置
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if stop_check is not None and stop_check():
                self._stop_setpoint_stream()
                return False, '移动被用户中止'
            pose = self.current_pose
            if pose is not None:
                dist = math.sqrt(
                    (pose.x - x) ** 2 + (pose.y - y) ** 2 + (pose.z - z) ** 2
                )
                if dist <= tolerance:
                    self._stop_setpoint_stream()
                    self.logger.info(f'OFFBOARD 移动到达: 距离 {dist:.2f}m')
                    return True, f'到达目标位置, 距离 {dist:.2f}m'
            await asyncio.sleep(0.2)

        self._stop_setpoint_stream()
        pose = self.current_pose
        if pose is not None:
            dist = math.sqrt(
                (pose.x - x) ** 2 + (pose.y - y) ** 2 + (pose.z - z) ** 2
            )
            return False, f'OFFBOARD 移动超时({timeout}s): 距离 {dist:.2f}m'
        return False, f'OFFBOARD 移动超时({timeout}s): 无遥测数据'

    async def rtl(self, timeout: float = 60.0, stop_check=None) -> tuple[bool, str]:
        req = Rtl.Request()
        req.timeout = float(timeout)
        return await self._call(self.rtl_cli, req, 'rtl', stop_check=stop_check)

    async def move_relative(self, dx: float, dy: float, dz: float, dyaw_deg: float,
                            stop_check=None) -> tuple[bool, str]:
        """机体坐标相对移动，dyaw 入参为度数，内部转弧度。"""
        req = MoveRelative.Request()
        req.dx = float(dx)
        req.dy = float(dy)
        req.dz = float(dz)
        req.dyaw = float(dyaw_deg) * math.pi / 180.0
        return await self._call(self.move_relative_cli, req, 'move_relative',
                                stop_check=stop_check)

    async def set_yaw(self, yaw_deg: float, is_relative: bool,
                      stop_check=None) -> tuple[bool, str]:
        """设置偏航角，入参为度数，内部转弧度。"""
        req = SetYaw.Request()
        req.yaw = float(yaw_deg) * math.pi / 180.0
        req.is_relative = bool(is_relative)
        return await self._call(self.set_yaw_cli, req, 'set_yaw',
                                stop_check=stop_check)

    async def set_velocity(self, vx: float, vy: float, vz: float, duration: float,
                           stop_check=None) -> tuple[bool, str]:
        req = SetVelocity.Request()
        req.vx = float(vx)
        req.vy = float(vy)
        req.vz = float(vz)
        req.duration = float(duration)
        return await self._call(self.set_velocity_cli, req, 'set_velocity',
                                stop_check=stop_check)

    async def set_max_speed(self, max_horizontal: float, max_vertical: float,
                            stop_check=None) -> tuple[bool, str]:
        req = SetMaxSpeed.Request()
        req.max_horizontal = float(max_horizontal)
        req.max_vertical = float(max_vertical)
        return await self._call(self.set_max_speed_cli, req, 'set_max_speed',
                                stop_check=stop_check)

    async def emergency_land(self) -> tuple[bool, str]:
        """紧急降落：立即切换到 AUTO.LAND 模式，绕过锁，短超时。"""
        req = EmergencyLand.Request()
        return await self._call(self.emergency_land_cli, req, 'emergency_land',
                                timeout=5.0)

    async def emergency_stop(self) -> tuple[bool, str]:
        """紧急停止：尝试悬停(AUTO.LOITER)或降落(AUTO.LAND)，短超时。"""
        req = EmergencyStop.Request()
        return await self._call(self.emergency_stop_cli, req, 'emergency_stop',
                                timeout=5.0)

    # ---------- helper ----------

    async def _call(self, client, request, name: str, timeout: float = 10.0,
                    stop_check=None) -> tuple[bool, str]:
        """异步调用 ROS2 service，返回 (success, message)。

        Args:
            client: ROS2 service client
            request: service request
            name: 服务名称（用于日志）
            timeout: 最大等待秒数
            stop_check: 可选的回调，返回 True 时取消等待并中止调用
        """
        if not client.wait_for_service(timeout_sec=5.0):
            msg = f'服务 {name} 不可用'
            self.logger.error(msg)
            return False, msg

        self.logger.info(f'调用服务: {name}')
        future = client.call_async(request)

        # 将 rclpy future 桥接到 asyncio
        loop = asyncio.get_event_loop()
        result_future = loop.create_future()

        def done_callback(fut):
            try:
                loop.call_soon_threadsafe(result_future.set_result, fut.result())
            except Exception as e:
                loop.call_soon_threadsafe(result_future.set_exception, e)

        future.add_done_callback(done_callback)

        # 如果提供了 stop_check，创建一个轮询任务与服务调用竞争
        if stop_check is not None:
            async def _stop_waiter():
                while True:
                    if stop_check():
                        return
                    await asyncio.sleep(0.1)

            stop_task = asyncio.create_task(_stop_waiter())
            try:
                done, pending = await asyncio.wait(
                    [result_future, stop_task],
                    timeout=timeout,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if stop_task in done:
                    self.logger.warning(f'服务 {name} 被用户中止')
                    return False, f'服务 {name} 被用户中止'
                if not done:
                    self.logger.error(f'服务 {name} 调用超时')
                    return False, f'服务 {name} 调用超时'
                result = result_future.result()
            except Exception as e:
                self.logger.error(f'服务 {name} 调用异常: {e}')
                return False, f'服务 {name} 调用异常: {e}'
            finally:
                stop_task.cancel()
                try:
                    await stop_task
                except asyncio.CancelledError:
                    pass
        else:
            try:
                result = await asyncio.wait_for(result_future, timeout=timeout)
            except asyncio.TimeoutError:
                msg = f'服务 {name} 调用超时'
                self.logger.error(msg)
                return False, msg

        if hasattr(result, 'success'):
            ok = bool(getattr(result, 'success'))
            message = getattr(result, 'message', '')
        elif hasattr(result, 'mode_sent'):
            ok = bool(getattr(result, 'mode_sent'))
            message = f'mode_sent={ok}'
        else:
            ok = False
            message = f'unsupported response type: {type(result).__name__}'

        self.logger.info(f'服务 {name} 返回: success={ok}, message={message}')
        return ok, message

    # ---------- state-based waiting ----------

    async def wait_until_mode(
        self,
        mode: str,
        timeout: float = 3.0,
        stop_check=None,
    ) -> tuple[bool, str]:
        """Wait until telemetry reports the requested flight mode."""
        import time
        deadline = time.monotonic() + timeout
        last_mode = None

        while time.monotonic() < deadline:
            if stop_check is not None and stop_check():
                return False, '等待模式切换被用户中止'
            state = self.current_state
            last_mode = state.mode if state else None
            if last_mode == mode:
                return True, f'模式已切换到 {mode}'
            await asyncio.sleep(0.1)

        return False, f'等待模式 {mode} 超时({timeout}s), 当前模式: {last_mode}'
    async def wait_until_altitude(
        self,
        target_alt: float,
        timeout: float = 30.0,
        tolerance: float = 0.3,
        stop_check=None,
    ) -> tuple[bool, str]:
        """
        等待无人机高度达到目标值（容差范围内）。
        支持两种判断：
          1. 绝对高度：pose.z >= target_alt - tolerance
          2. 相对高度：从起飞点上升了 target_alt 以上（用于气压计有偏移的场景）

        Args:
            target_alt: 目标高度 (m)
            timeout: 最大等待秒数
            tolerance: 高度容差 (m)
            stop_check: 可选的回调，返回 True 时中止等待

        Returns:
            (success, message) - success=True 表示高度达标，False 表示超时/中止/无数据
        """
        import time
        deadline = time.monotonic() + timeout
        check_interval = 0.2  # 200ms polling

        # 记录起始高度，用于相对判断
        start_z = None
        pose = self.current_pose
        if pose is not None:
            start_z = pose.z

        while time.monotonic() < deadline:
            if stop_check is not None and stop_check():
                return False, '等待被用户中止'
            pose = self.current_pose
            if pose is not None:
                # 判断1: 绝对高度达标
                if pose.z >= target_alt - tolerance:
                    self.logger.info(
                        f'wait_until_altitude: 到达高度 {pose.z:.2f}m '
                        f'(目标 {target_alt:.2f}m ± {tolerance}m)'
                    )
                    return True, f'高度 {pose.z:.2f}m 达标'
                # 判断2: 从起始位置上升了 target_alt 以上（气压计偏移补偿）
                if start_z is not None and (pose.z - start_z) >= (target_alt - tolerance):
                    self.logger.info(
                        f'wait_until_altitude: 相对上升 {pose.z - start_z:.2f}m '
                        f'({start_z:.2f}m → {pose.z:.2f}m, 目标上升 {target_alt:.2f}m)'
                    )
                    return True, f'相对上升 {pose.z - start_z:.2f}m 达标'
            await asyncio.sleep(check_interval)

        pose = self.current_pose
        current = pose.z if pose else None
        msg = (
            f'等待高度超时({timeout}s): '
            f'当前 {current:.2f}m, 目标 {target_alt:.2f}m ± {tolerance}m'
            if current is not None
            else f'等待高度超时({timeout}s): 无遥测数据'
        )
        self.logger.warning(f'wait_until_altitude: {msg}')
        return False, msg

    async def wait_until_landed(
        self, timeout: float = 30.0, stop_check=None
    ) -> tuple[bool, str]:
        """
        等待无人机着陆状态变为 ON_GROUND。

        Args:
            timeout: 最大等待秒数
            stop_check: 可选的回调，返回 True 时中止等待

        Returns:
            (success, message) - success=True 表示已着陆，False 表示超时/中止/无数据
        """
        import time
        deadline = time.monotonic() + timeout
        check_interval = 0.2

        while time.monotonic() < deadline:
            if stop_check is not None and stop_check():
                return False, '等待被用户中止'
            state = self.current_state
            if state is not None and state.landed_state == 'ON_GROUND':
                self.logger.info('wait_until_landed: 已确认着陆 (ON_GROUND)')
                return True, '已着陆'
            await asyncio.sleep(check_interval)

        state = self.current_state
        current_ls = state.landed_state if state else None
        msg = (
            f'等待着陆超时({timeout}s): 当前状态 {current_ls}'
            if current_ls is not None
            else f'等待着陆超时({timeout}s): 无遥测数据'
        )
        self.logger.warning(f'wait_until_landed: {msg}')
        return False, msg

    async def wait_until_position(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        timeout: float = 30.0,
        tolerance: float = 0.5,
        stop_check=None,
    ) -> tuple[bool, str]:
        """
        等待无人机到达目标位置（三维距离在容差范围内）。

        Args:
            target_x: 目标 X (m)
            target_y: 目标 Y (m)
            target_z: 目标 Z (m)
            timeout: 最大等待秒数
            tolerance: 距离容差 (m)
            stop_check: 可选的回调，返回 True 时中止等待

        Returns:
            (success, message) - success=True 表示到达目标位置，False 表示超时/中止/无数据
        """
        import time
        deadline = time.monotonic() + timeout
        check_interval = 0.2

        while time.monotonic() < deadline:
            if stop_check is not None and stop_check():
                return False, '等待被用户中止'
            pose = self.current_pose
            if pose is not None:
                dist = math.sqrt(
                    (pose.x - target_x) ** 2
                    + (pose.y - target_y) ** 2
                    + (pose.z - target_z) ** 2
                )
                if dist <= tolerance:
                    self.logger.info(
                        f'wait_until_position: 到达目标 ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})'
                        f', 距离 {dist:.2f}m (容差 {tolerance}m)'
                    )
                    return True, f'到达目标位置, 距离 {dist:.2f}m'
            await asyncio.sleep(check_interval)

        pose = self.current_pose
        if pose is not None:
            dist = math.sqrt(
                (pose.x - target_x) ** 2
                + (pose.y - target_y) ** 2
                + (pose.z - target_z) ** 2
            )
            msg = (
                f'等待位置超时({timeout}s): '
                f'当前 ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}), '
                f'目标 ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
                f'距离 {dist:.2f}m'
            )
        else:
            msg = f'等待位置超时({timeout}s): 无遥测数据'
        self.logger.warning(f'wait_until_position: {msg}')
        return False, msg

    def get_telemetry_snapshot(self) -> dict:
        """构造一帧遥测快照，用于上报给后端。"""
        import time
        pose = self.current_pose
        state = self.current_state
        return {
            'ts': int(time.time() * 1000),
            'pose': {
                'x': pose.x if pose else 0.0,
                'y': pose.y if pose else 0.0,
                'z': pose.z if pose else 0.0,
                'yaw': pose.yaw if pose else 0.0,
            },
            'state': {
                'armed': state.armed if state else False,
                'connected': state.connected if state else False,
                'mode': state.mode if state else 'UNKNOWN',
            },
            'battery': {
                'voltage': self.current_battery.get('voltage'),
                'current': self.current_battery.get('current'),
                'percentage': self.current_battery.get('percentage'),
            },
        }
