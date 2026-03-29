#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Rtl


class TestRtlCtrl(Node):
    """测试返航节点：调用自动返航服务并输出返回结果。"""

    def __init__(self, timeout=90.0):
        super().__init__('test_rtl_ctrl')
        self.timeout = float(timeout)

        client = self.create_client(Rtl, 'uav/rtl')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/rtl 服务不可用')
            return

        req = Rtl.Request()
        req.timeout = float(self.timeout)
        self.get_logger().info(f'调用 uav/rtl, timeout={req.timeout}')
        fut = client.call_async(req)
        # 服务端会等待返航落地后再返回，因此客户端等待时间应不小于返航超时。
        service_wait = req.timeout + 10.0
        rclpy.spin_until_future_complete(self, fut, timeout_sec=service_wait)
        if not fut.done():
            self.get_logger().error(f'调用 uav/rtl 超时（等待 {service_wait:.1f}s）')
            return
        res = fut.result()
        self.get_logger().info(f'uav/rtl 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')


def main(args=None):
    rclpy.init(args=args)
    node = TestRtlCtrl()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
