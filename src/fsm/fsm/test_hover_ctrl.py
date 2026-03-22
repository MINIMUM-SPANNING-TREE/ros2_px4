#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import SetMode
from uav_interfaces.msg import UavPose


class TestHoverCtrl(Node):
    """测试悬停节点：切换到 LOITER 或 OFFBOARD 以悬停指定时长（默认 5s）。"""

    def __init__(self, duration=5.0, mode='AUTO.LOITER'):
        super().__init__('test_hover_ctrl')
        self.duration = float(duration)
        self.mode = str(mode)
        self.current_pose = None

        self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)

        self.get_logger().info('等待遥测以获取位置基准...')
        start = time.time()
        while self.current_pose is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_pose is None:
            self.get_logger().warn('未收到遥测，仍将尝试设置模式悬停')

        client = self.create_client(SetMode, 'uav/set_mode')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/set_mode 服务不可用')
            return

        req = SetMode.Request()
        req.mode = self.mode
        self.get_logger().info(f'请求切换模式为 {self.mode}，悬停 {self.duration} 秒')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().error('调用 uav/set_mode 超时')
            return
        res = fut.result()
        self.get_logger().info(f'uav/set_mode 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')

        # 悬停期间简单等待并打印遥测
        start = time.time()
        while time.time() - start < self.duration:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current_pose:
                self.get_logger().debug(f'悬停高度 {self.current_pose.z:.2f} m')
            time.sleep(0.1)

        self.get_logger().info('悬停测试完成')

    def pose_cb(self, msg: UavPose):
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TestHoverCtrl()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
