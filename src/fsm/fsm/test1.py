#!/usr/bin/env python3
"""测试：解锁 → 起飞 → 悬停 → 降落"""
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Arm, Takeoff, Land, SetMode


class TestTakeoffLand(Node):
    def __init__(self):
        super().__init__('test_takeoff_land')
        self.arm_cli = self.create_client(Arm, 'uav/arm')
        self.takeoff_cli = self.create_client(Takeoff, 'uav/takeoff')
        self.land_cli = self.create_client(Land, 'uav/land')
        self.mode_cli = self.create_client(SetMode, 'uav/set_mode')

        for cli in [self.arm_cli, self.takeoff_cli, self.land_cli, self.mode_cli]:
            cli.wait_for_service()
        self.get_logger().info('所有服务就绪')

        # self.call(self.mode_cli, SetMode.Request(mode='OFFBOARD'))
        self.call(self.arm_cli, Arm.Request(arm=True))
        self.call(self.takeoff_cli, Takeoff.Request(relative_alt=2.0))

        self.get_logger().info('悬停 50 秒...')
        self.create_timer(50.0, self.start_land)

    def start_land(self):
        self.call(self.land_cli, Land.Request(timeout=30.0))
        self.get_logger().info('降落指令已发送')

    def call(self, cli, req):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            res = future.result()
            self.get_logger().info(f'{cli.srv_type.__name__}: success={res.success}, msg={res.message}')
            return res
        self.get_logger().warn(f'{cli.srv_type.__name__}: 服务调用超时')
        return None


def main():
    rclpy.init()
    rclpy.spin(TestTakeoffLand())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
