#!/usr/bin/env python3
"""
telemetry_printer_node.py
订阅 UAV 遥测数据并在控制台打印
"""

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavPose, UavState


class TelemetryPrinter(Node):
    def __init__(self):
        super().__init__('telemetry_printer')
        
        # 订阅位姿数据
        self.sub_pose = self.create_subscription(
            UavPose, 
            '/uav/telemetry/pose', 
            self.on_pose, 
            10
        )
        
        # 订阅状态数据
        self.sub_state = self.create_subscription(
            UavState,
            '/uav/telemetry/state',
            self.on_state,
            10
        )
        
        self.get_logger().info('Telemetry printer node started')
    
    def on_pose(self, msg: UavPose):
        """打印位姿信息"""
        self.get_logger().info(
            f'[POSE] x: {msg.x:.3f}, y: {msg.y:.3f}, z: {msg.z:.3f}, '
            f'yaw: {msg.yaw:.3f} rad ({msg.yaw * 180 / 3.14159:.1f}°)'
        )
    
    def on_state(self, msg: UavState):
        """打印状态信息"""
        self.get_logger().info(
            f'[STATE] connected: {msg.connected}, armed: {msg.armed}, '
            f'mode: {msg.mode}, landed: {msg.landed_state}'
        )


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