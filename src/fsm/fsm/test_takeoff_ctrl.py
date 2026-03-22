#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Takeoff
from uav_interfaces.msg import UavPose


# 模块说明：
# 简单的起飞测试节点。流程：等待遥测、调用 `uav/takeoff` 服务，然后监测高度是否达到目标。
class TestTakeoffCtrl(Node):
    """测试控制节点：调用起飞服务并监测升高到目标高度。"""

    def __init__(self, relative_alt=2.0, monitor_timeout=90.0):
        super().__init__('test_takeoff_ctrl')
        self.relative_alt = float(relative_alt)
        self.monitor_timeout = float(monitor_timeout)
        self.current_pose = None

        # 订阅遥测位姿，用于获取当前高度
        self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)

        # 等待初始遥测
        self.get_logger().info('等待遥测...')
        start = time.time()
        while self.current_pose is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_pose is None:
            self.get_logger().error('未收到遥测，无法继续测试')
            return

        base_alt = self.current_pose.z
        target_alt = base_alt + self.relative_alt
        self.get_logger().info(f'当前高度 {base_alt:.2f} m, 目标高度 {target_alt:.2f} m')

        # 创建 takeoff 服务客户端并等待可用
        self.client = self.create_client(Takeoff, 'uav/takeoff')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/takeoff 服务不可用')
            return

        # 发起起飞请求
        req = Takeoff.Request()
        req.relative_alt = float(self.relative_alt)
        self.get_logger().info(f'调用 uav/takeoff: relative_alt={req.relative_alt}')
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done():
            self.get_logger().error('调用 uav/takeoff 超时')
            return
        res = fut.result()
        if res is None:
            self.get_logger().error('uav/takeoff 返回空')
        else:
            # 翻译并输出响应内容
            self.get_logger().info(f'uav/takeoff 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')

        # 监控高度直到达到目标或超时
        self.get_logger().info('开始监测高度直到达到目标...')
        start = time.time()
        while time.time() - start < self.monitor_timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current_pose is None:
                continue
            alt = self.current_pose.z
            self.get_logger().debug(f'当前高度 {alt:.2f} m')
            if alt >= target_alt * 0.95:
                self.get_logger().info(f'达到目标高度: {alt:.2f} m')
                return
        self.get_logger().error('监测超时，未达到目标高度')

    def pose_cb(self, msg: UavPose):
        # 位姿回调：更新当前位姿缓存
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TestTakeoffCtrl()
    try:
        # keep node alive briefly to ensure logs are flushed; most work done in __init__
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
