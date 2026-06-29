#!/usr/bin/env python3
"""测试雷达 + 导航功能
流程：检查雷达数据 → 设置目标点 → 添加航点 → 启动导航 → 轮询状态 → 停止 → 清空
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from uav_interfaces.srv import (
    NavSetGoal, NavAddWaypoint, NavStart, NavStop,
    NavGetStatus, NavClearWaypoints, Takeoff, Land,
)


class TestNav(Node):
    def __init__(self):
        super().__init__('test_nav')
        self.got_cloud = False

        # 雷达订阅
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/lslidar_point_cloud', self._cloud_cb, qos)

        # 导航服务客户端
        self.goal_cli = self.create_client(NavSetGoal, 'nav/set_goal')
        self.addwp_cli = self.create_client(NavAddWaypoint, 'nav/add_waypoint')
        self.start_cli = self.create_client(NavStart, 'nav/start')
        self.stop_cli = self.create_client(NavStop, 'nav/stop')
        self.status_cli = self.create_client(NavGetStatus, 'nav/get_status')
        self.clear_cli = self.create_client(NavClearWaypoints, 'nav/clear_waypoints')

        # 起飞服务
        self.takeoff_cli = self.create_client(Takeoff, 'uav/takeoff')
        self.land_cli = self.create_client(Land, 'uav/land')

        self.get_logger().info('等待导航服务...')
        for cli in [self.goal_cli, self.addwp_cli, self.start_cli,
                    self.stop_cli, self.status_cli, self.clear_cli,
                    self.takeoff_cli, self.land_cli]:
            cli.wait_for_service()
        self.get_logger().info('导航服务就绪')

        self.step = 0
        self.create_timer(1.0, self._tick)

    def _cloud_cb(self, msg):
        if not self.got_cloud:
            self.get_logger().info(f'雷达数据: {msg.width} 点, frame={msg.header.frame_id}')
            self.got_cloud = True

    def _call(self, cli, req):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() if future.done() else None

    def _tick(self):
        self.step += 1

        # 1) 检查雷达
        if self.step == 1:
            if not self.got_cloud:
                self.get_logger().warn('未收到雷达数据，检查雷达是否启动')
            return

        # 2) 清空旧航点
        if self.step == 2:
            self.get_logger().info('清空航点...')
            self._call(self.clear_cli, NavClearWaypoints.Request())
            return

        # 3) 起飞
        if self.step == 3:
            self.get_logger().info('起飞...')
            req = Takeoff.Request(relative_alt=2.0)
            res = self._call(self.takeoff_cli, req)
            if res and res.success:
                self.get_logger().info('起飞成功，等待到达高度...')
            else:
                self.get_logger().error(f'起飞失败: {res.message if res else "无响应"}')
            return

        # 4) 等待起飞完成
        if self.step == 4:
            self.get_logger().info('等待5秒让无人机到达高度...')
            return

        # 5) 添加航点
        if self.step == 5:
            for i, (x, y, z, yaw) in enumerate([
                (1.0, 0.0, 2.0, 0.0),
                (1.0, 2.0, 2.0, 1.57),
                (0.0, 2.0, 2.0, 3.14),
                (2.0, 1.0, 2.0, 0.0),
            ]):
                req = NavAddWaypoint.Request(x=x, y=y, z=z, yaw=yaw, hold_time=3.0)
                res = self._call(self.addwp_cli, req)
                if res:
                    self.get_logger().info(f'添加航点 {res.waypoint_index}: ({x},{y},{z})')
            return

        # 6) 启动导航
        if self.step == 6:
            self.get_logger().info('启动导航...')
            res = self._call(self.start_cli, NavStart.Request(auto_start=True))
            if res:
                self.get_logger().info(f'导航已启动, 共 {res.total_waypoints} 个航点')
            return

        # 7) 每 2 秒轮询状态
        if self.step >= 7 and self.step % 2 == 0:
            res = self._call(self.status_cli, NavGetStatus.Request())
            if res:
                self.get_logger().info(
                    f'[{res.state}] 航点 {res.current_waypoint}/{res.total_waypoints} '
                    f'距离 {res.distance_to_goal:.1f}m 障碍物 {res.obstacle_count}')
                if res.state in ('COMPLETED', 'ERROR', 'IDLE'):
                    self.get_logger().info('导航结束')
                    rclpy.shutdown()
            return
        if self.step >  20:
            self.get_logger().info('降落')
            req = Land.Request()
            self._call(self.land_cli, req)




def main():
    rclpy.init()
    rclpy.spin(TestNav())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
