#!/usr/bin/env python3
"""
telemetry_printer_node.py
复用 Uav/UavBase 遥测缓存，以低频、可读的方式打印无人机状态
"""

import math
import time
import rclpy
from uav_mavros2.uav import Uav


class TelemetryPrinter(Uav):
    def __init__(self):
        super().__init__()

        self.print_interval_sec = 1.0
        self.heartbeat_sec = 5.0
        self.move_threshold_m = 0.30

        self._last_log_time = 0.0
        self._last_state_key = None
        self._last_pose_key = None
        self._last_battery_key = None

        self.create_timer(self.print_interval_sec, self._on_timer)
        self.get_logger().info("遥测打印节点已启动（状态变化触发 + 5秒心跳，避免刷屏）")

    def timer_cb(self):
        # 遥测打印节点只读状态，不参与 OFFBOARD setpoint 发布，避免与控制服务抢控制。
        return

    @staticmethod
    def _to_bool_text(value):
        return "是" if value else "否"

    @staticmethod
    def _landed_state_to_text(state_code):
        state_map = {
            0: "未知",
            1: "地面",
            2: "空中",
            3: "起飞中",
            4: "降落中",
        }
        return state_map.get(state_code, f"未知({state_code})")

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_snapshot(self):
        pose = self.get_current_pos().pose
        state = self.get_current_state()
        battery = self.get_battery_data()
        home = self.get_home_position()

        yaw_rad = self._yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        yaw_deg = math.degrees(yaw_rad)

        battery_percent = None
        if isinstance(battery.percentage, float) and not math.isnan(battery.percentage):
            battery_percent = battery.percentage * 100.0

        return {
            "connected": bool(state.connected),
            "armed": bool(state.armed),
            "mode": state.mode if state.mode else "UNKNOWN",
            "landed_state": int(self.get_landed_state()),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw_deg": float(yaw_deg),
            "battery_percent": battery_percent,
            "battery_voltage": float(battery.voltage),
            "home_ready": bool(self.is_home_ready()),
            "home_x": float(home.position.x),
            "home_y": float(home.position.y),
            "home_z": float(home.position.z),
        }

    def _on_timer(self):
        snapshot = self._build_snapshot()

        state_key = (
            snapshot["connected"],
            snapshot["armed"],
            snapshot["mode"],
            snapshot["landed_state"],
        )
        pose_key = (
            round(snapshot["x"], 2),
            round(snapshot["y"], 2),
            round(snapshot["z"], 2),
            round(snapshot["yaw_deg"], 1),
        )
        battery_key = (
            None if snapshot["battery_percent"] is None else round(snapshot["battery_percent"], 1),
            round(snapshot["battery_voltage"], 2),
        )

        moved = False
        if self._last_pose_key is not None:
            dx = pose_key[0] - self._last_pose_key[0]
            dy = pose_key[1] - self._last_pose_key[1]
            dz = pose_key[2] - self._last_pose_key[2]
            moved = math.sqrt(dx * dx + dy * dy + dz * dz) >= self.move_threshold_m

        state_changed = self._last_state_key is None or state_key != self._last_state_key
        battery_changed = self._last_battery_key is None or battery_key != self._last_battery_key
        now = time.time()
        heartbeat_due = (now - self._last_log_time) >= self.heartbeat_sec

        if not (state_changed or moved or battery_changed or heartbeat_due):
            return

        battery_text = "未知"
        if snapshot["battery_percent"] is not None:
            battery_text = f"{snapshot['battery_percent']:.1f}%"

        msg = (
            "\n"
            "==================== 无人机遥测 ====================\n"
            f"连接状态 : {self._to_bool_text(snapshot['connected'])}    "
            f"解锁状态 : {self._to_bool_text(snapshot['armed'])}\n"
            f"飞行模式 : {snapshot['mode']}    "
            f"着陆状态 : {self._landed_state_to_text(snapshot['landed_state'])}\n"
            f"当前位置 : x={snapshot['x']:.2f} m, y={snapshot['y']:.2f} m, z={snapshot['z']:.2f} m\n"
            f"Home位置 : x={snapshot['home_x']:.2f} m, y={snapshot['home_y']:.2f} m, z={snapshot['home_z']:.2f} m "
            f"(有效: {self._to_bool_text(snapshot['home_ready'])})\n"
            f"航向角度 : {snapshot['yaw_deg']:.1f}°\n"
            f"电池信息 : {battery_text} / {snapshot['battery_voltage']:.2f} V\n"
            "===================================================="
        )
        self.get_logger().info(msg)

        self._last_log_time = now
        self._last_state_key = state_key
        self._last_pose_key = pose_key
        self._last_battery_key = battery_key


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()