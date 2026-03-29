#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Takeoff


# 模块说明：
# 简单的起飞测试节点。流程：调用 `uav/takeoff`，并以服务返回结果为准。
class TestTakeoffCtrl(Node):
    """测试控制节点：调用起飞服务并输出返回结果。"""

    def __init__(self, relative_alt=3.0, monitor_timeout=90.0):
        super().__init__('test_takeoff_ctrl')
        self.relative_alt = float(relative_alt)
        self.monitor_timeout = float(monitor_timeout)

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
        service_wait = self.monitor_timeout + 10.0
        rclpy.spin_until_future_complete(self, fut, timeout_sec=service_wait)
        if not fut.done():
            self.get_logger().error(f'调用 uav/takeoff 超时（等待 {service_wait:.1f}s）')
            return
        res = fut.result()
        if res is None:
            self.get_logger().error('uav/takeoff 返回空')
        else:
            self.get_logger().info(f'uav/takeoff 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')


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
