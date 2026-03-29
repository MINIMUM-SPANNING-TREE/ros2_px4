#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Move


class MoveNode(Node):
    """移动控制节点：仅调用 uav/move 服务，不在本节点做状态监测。"""

    def __init__(self, x=4.0, y=4.0, z=4.0, yaw=0.0, monitor_timeout=30.0):
        super().__init__('move_node')
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)
        self.target_yaw = float(yaw)
        self.monitor_timeout = float(monitor_timeout)

        self.client = self.create_client(Move, 'uav/move')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/move 服务不可用')
            return

        req = Move.Request()
        req.x = self.target_x
        req.y = self.target_y
        req.z = self.target_z
        req.yaw = self.target_yaw

        self.get_logger().info(
            f'调用 uav/move: x={req.x:.2f}, y={req.y:.2f}, z={req.z:.2f}, yaw={req.yaw:.2f}'
        )
        fut = self.client.call_async(req)
        service_wait = self.monitor_timeout + 10.0
        rclpy.spin_until_future_complete(self, fut, timeout_sec=service_wait)
        if not fut.done():
            self.get_logger().error(f'调用 uav/move 超时（等待 {service_wait:.1f}s）')
            return

        res = fut.result()
        if res is None:
            self.get_logger().error('uav/move 返回空')
        else:
            self.get_logger().info(
                f'uav/move 返回: success={getattr(res, "success", None)} '
                f'message="{getattr(res, "message", None)}"'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()

    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
