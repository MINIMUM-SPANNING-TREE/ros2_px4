#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Move
from uav_interfaces.msg import UavPose


class TestMoveCtrl(Node):
    """测试移动节点：将无人机按相对距离沿 X 轴前进（默认 5m）。"""

    def __init__(self, forward=5.0, monitor_timeout=10.0):
        super().__init__('test_move_ctrl')
        self.forward = float(forward)
        self.monitor_timeout = float(monitor_timeout)
        self.current_pose = None

        self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)

        self.get_logger().info('等待遥测...')
        start = time.time()
        while self.current_pose is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_pose is None:
            self.get_logger().error('未收到遥测，无法继续测试')
            return

        sx = self.current_pose.x
        sy = self.current_pose.y
        sz = self.current_pose.z
        tx = sx + self.forward
        ty = sy
        tz = sz
        self.get_logger().info(f'开始移动测试: 从 ({sx:.2f},{sy:.2f},{sz:.2f}) 到 ({tx:.2f},{ty:.2f},{tz:.2f})')

        client = self.create_client(Move, 'uav/move')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/move 服务不可用')
            return

        req = Move.Request()
        req.x = float(tx)
        req.y = float(ty)
        req.z = float(tz)
        req.yaw = float(self.current_pose.yaw if self.current_pose else 0.0)

        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().error('调用 uav/move 超时')
            return
        res = fut.result()
        self.get_logger().info(f'uav/move 返回: success={getattr(res, "success", None)} message="{getattr(res, "message", None)}"')

        # 监测是否到达目标
        start = time.time()
        while time.time() - start < self.monitor_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose is None:
                continue
            dist = ((self.current_pose.x - tx)**2 + (self.current_pose.y - ty)**2 + (self.current_pose.z - tz)**2)**0.5
            self.get_logger().debug(f'当前到目标距离 {dist:.2f} m')
            if dist < 0.5:
                self.get_logger().info('到达目标位置')
                return
        self.get_logger().error('监测超时，未到达目标位置')

    def pose_cb(self, msg: UavPose):
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TestMoveCtrl()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
