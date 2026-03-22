#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Takeoff, Move, Land, SetMode
from uav_interfaces.msg import UavPose, UavState


class TestFullMission(Node):
    """完整飞行任务测试：起飞到 2m -> 前进 5m -> 向上 4m -> 悬停 7s -> 降落"""

    def __init__(self):
        super().__init__('test_full_mission')
        self.pose = None
        self.state = None

        self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)
        self.create_subscription(UavState, '/uav/telemetry/state', self.state_cb, 10)

        self.get_logger().info('等待遥测以获取起始位姿...')
        start = time.time()
        while self.pose is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            self.get_logger().error('未收到遥测，任务中止')
            return

        # 1) 起飞到 2m
        base_alt = self.pose.z
        rel_alt = 2.0
        target_alt = base_alt + rel_alt
        self.get_logger().info(f'起飞：相对 {rel_alt}m（目标高度 {target_alt:.2f}m）')
        t_client = self.create_client(Takeoff, 'uav/takeoff')
        if not t_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/takeoff 服务不可用')
            return
        treq = Takeoff.Request()
        treq.relative_alt = float(rel_alt)
        fut = t_client.call_async(treq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('起飞请求已发送')

        if not self.wait_for_altitude(target_alt, timeout=30.0):
            self.get_logger().error('起飞未能达到目标高度，任务中止')
            return

        # 2) 前进 5m（X 方向）
        start_x = self.pose.x
        target_x = start_x + 5.0
        self.get_logger().info(f'向前移动 5m: X {start_x:.2f} -> {target_x:.2f}')
        m_client = self.create_client(Move, 'uav/move')
        if not m_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/move 服务不可用')
            return
        mreq = Move.Request()
        mreq.x = float(target_x)
        mreq.y = float(self.pose.y)
        mreq.z = float(self.pose.z)
        mreq.yaw = float(self.pose.yaw if self.pose else 0.0)
        fut = m_client.call_async(mreq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('前进请求已发送')
        if not self.wait_for_position(target_x, self.pose.y, self.pose.z, timeout=20.0):
            self.get_logger().error('前进未完成，任务中止')
            return

        # 3) 向上移动 4m
        start_z = self.pose.z
        target_z = start_z + 4.0
        self.get_logger().info(f'向上移动 4m: Z {start_z:.2f} -> {target_z:.2f}')
        mreq2 = Move.Request()
        mreq2.x = float(self.pose.x)
        mreq2.y = float(self.pose.y)
        mreq2.z = float(target_z)
        mreq2.yaw = float(self.pose.yaw if self.pose else 0.0)
        fut = m_client.call_async(mreq2)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('上移请求已发送')
        if not self.wait_for_position(self.pose.x, self.pose.y, target_z, timeout=20.0):
            self.get_logger().error('上移未完成，任务中止')
            return

        # 4) 悬停 7 秒
        hover_sec = 7.0
        self.get_logger().info(f'悬停 {hover_sec} 秒')
        # 使用 set_mode LOITER 保持位置（如果可用），否则简单等待
        sm_client = self.create_client(SetMode, 'uav/set_mode')
        if sm_client.wait_for_service(timeout_sec=2.0):
            sreq = SetMode.Request()
            sreq.mode = 'AUTO.LOITER'
            fut = sm_client.call_async(sreq)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        start = time.time()
        while time.time() - start < hover_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)

        # 5) 降落
        self.get_logger().info('发起降落')
        land_client = self.create_client(Land, 'uav/land')
        if not land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('uav/land 服务不可用')
            return
        lreq = Land.Request()
        lreq.timeout = 60.0
        fut = land_client.call_async(lreq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('降落请求已发送，等待落地...')
        # 等待落地
        start = time.time()
        while time.time() - start < 90.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state and getattr(self.state, 'landed_state', None) == 'ON_GROUND':
                self.get_logger().info('完整任务：已安全落地')
                return
            if self.pose and self.pose.z < 0.2:
                self.get_logger().info('完整任务：高度低于 0.2m，认为已落地')
                return
        self.get_logger().error('降落超时，任务结束')

    def pose_cb(self, msg: UavPose):
        self.pose = msg

    def state_cb(self, msg: UavState):
        self.state = msg

    def wait_for_altitude(self, target, timeout=30.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose and self.pose.z >= target * 0.95:
                self.get_logger().info(f'达到目标高度 {self.pose.z:.2f} m')
                return True
        return False

    def wait_for_position(self, tx, ty, tz, timeout=20.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose:
                dist = ((self.pose.x - tx)**2 + (self.pose.y - ty)**2 + (self.pose.z - tz)**2)**0.5
                if dist < 0.5:
                    self.get_logger().info(f'到达目标位置，距离 {dist:.2f} m')
                    return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TestFullMission()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
