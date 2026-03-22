#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Land
from uav_interfaces.msg import UavState, UavPose


class TestLandCtrl(Node):
    """测试降落节点：调用降落服务并监测是否落地。"""

    def __init__(self, timeout=60.0):
        super().__init__('test_land_ctrl')
        self.timeout = float(timeout)
        self.current_state = None
        self.current_pose = None

        self.create_subscription(UavState, '/uav/telemetry/state', self.state_cb, 10)
        self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)

        self.get_logger().info('等待遥测/状态...')
        start = time.time()
        while (self.current_state is None or self.current_pose is None) and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_state is None:
            self.get_logger().warn('未收到状态信息，但仍尝试发起降落')

        client = self.create_client(Land, 'uav/land')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/land 服务不可用')
            return

        req = Land.Request()
        req.timeout = float(self.timeout)
        self.get_logger().info(f'调用 uav/land, timeout={req.timeout}')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().error('调用 uav/land 超时')
            return
        res = fut.result()
        self.get_logger().info(f'uav/land 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')

        # 监测落地
        start = time.time()
        while time.time() - start < self.timeout + 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_state and getattr(self.current_state, 'landed_state', None) == 'ON_GROUND':
                self.get_logger().info('检测到已落地')
                return
            # 作为兜底：当高度非常低时也认定为落地
            if self.current_pose and self.current_pose.z < 0.2:
                self.get_logger().info('高度低于 0.2m，认为已落地')
                return
        self.get_logger().error('降落监测超时')

    def state_cb(self, msg: UavState):
        self.current_state = msg

    def pose_cb(self, msg: UavPose):
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TestLandCtrl()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
