#!/usr/bin/env python3
"""测试：解锁 → 起飞 → 依次飞航点 → 降落"""
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Arm, Takeoff, Move, Land, SetMode

WAYPOINTS = [
    (1.0, 0.0, 2.0, 0.0),
    (1.0, 1.0, 2.0, 1.57),
    (0.0, 1.0, 2.0, 3.14),
    (0.0, 0.0, 2.0, 0.0),
]


class TestWaypoints(Node):
    def __init__(self):
        super().__init__('test_waypoints')
        self.arm_cli = self.create_client(Arm, 'uav/arm')
        self.takeoff_cli = self.create_client(Takeoff, 'uav/takeoff')
        self.move_cli = self.create_client(Move, 'uav/move')
        self.land_cli = self.create_client(Land, 'uav/land')
        self.mode_cli = self.create_client(SetMode, 'uav/set_mode')

        for cli in [self.arm_cli, self.takeoff_cli, self.move_cli, self.land_cli, self.mode_cli]:
            cli.wait_for_service()
        self.get_logger().info('所有服务就绪')

        self.wp_idx = 0
        self.run()

    def run(self):
        self.call(self.mode_cli, SetMode.Request(mode='OFFBOARD'))
        self.call(self.arm_cli, Arm.Request(arm=True))
        self.call(self.takeoff_cli, Takeoff.Request(relative_alt=2.0))
        self.get_logger().info('起飞完成，开始航点任务')
        self.fly_next()

    def fly_next(self):
        if self.wp_idx >= len(WAYPOINTS):
            self.get_logger().info('航点全部完成，开始降落')
            self.call(self.land_cli, Land.Request(timeout=30.0))
            return

        x, y, z, yaw = WAYPOINTS[self.wp_idx]
        self.get_logger().info(f'航点 {self.wp_idx}: ({x}, {y}, {z}, yaw={yaw})')
        self.call(self.move_cli, Move.Request(x=x, y=y, z=z, yaw=yaw))
        self.wp_idx += 1
        self.create_timer(8.0, self.fly_next)

    def call(self, cli, req):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        return res


def main():
    rclpy.init()
    node = TestWaypoints()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
