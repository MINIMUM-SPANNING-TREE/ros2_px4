"""
主导航节点
集成障碍物检测、避障规划和航点导航
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import List, Tuple, Optional
import threading

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

from uav_interfaces.msg import UavState, UavPose
from uav_interfaces.srv import Move

from navigation.obstacle_detector import ObstacleDetector, Obstacle
from navigation.local_planner import LocalPlanner, VelocityCommand
from navigation.waypoint_navigator import WaypointNavigator, Waypoint


class NavigationNode(Node):
    """
    导航节点
    
    功能：
    1. 订阅激光雷达点云，检测障碍物
    2. 使用 VFH 算法进行避障规划
    3. 通过航点导航器控制无人机飞行
    4. 发布可视化标记
    
    订阅：
    - /lslidar_point_cloud (PointCloud2): 激光雷达点云
    - /uav/telemetry/pose (UavPose): 无人机位姿
    - /uav/telemetry/state (UavState): 无人机状态
    
    发布：
    - /navigation/markers (MarkerArray): 可视化标记
    - /navigation/obstacles (MarkerArray): 障碍物标记
    
    服务：
    - /navigation/start (通过参数或话题控制)
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self._get_parameters()
        
        # 初始化组件
        self._init_components()
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        
        # 订阅者
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._pointcloud_callback,
            sensor_qos,
        )
        
        self.pose_sub = self.create_subscription(
            UavPose,
            '/uav/telemetry/pose',
            self._pose_callback,
            10,
        )
        
        self.state_sub = self.create_subscription(
            UavState,
            '/uav/telemetry/state',
            self._state_callback,
            10,
        )
        
        # 发布者
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/navigation/markers',
            10,
        )
        
        self.obstacle_pub = self.create_publisher(
            MarkerArray,
            '/navigation/obstacles',
            10,
        )
        
        # 状态
        self.current_pose: Optional[UavPose] = None
        self.current_state: Optional[UavState] = None
        self.current_obstacles: List[Obstacle] = []
        
        # 定时器
        self.planning_timer = self.create_timer(
            1.0 / self.planning_rate,
            self._planning_callback,
        )
        
        self.visualization_timer = self.create_timer(
            1.0 / self.visualization_rate,
            self._visualization_callback,
        )
        
        self.get_logger().info('='*50)
        self.get_logger().info('UAV Navigation Node Started')
        self.get_logger().info('='*50)
        self._log_parameters()
    
    def _declare_parameters(self):
        """声明参数"""
        # 雷达话题
        self.declare_parameter('pointcloud_topic', '/lslidar_point_cloud')
        
        # 障碍物检测参数
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('max_height', 3.0)
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 5)
        
        # 避障参数
        self.declare_parameter('safety_distance', 2.0)
        self.declare_parameter('warning_distance', 3.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 0.5)
        
        # 航点参数
        self.declare_parameter('waypoints', [])
        self.declare_parameter('auto_start', False)
        
        # 频率参数
        self.declare_parameter('planning_rate', 10.0)
        self.declare_parameter('visualization_rate', 5.0)
    
    def _get_parameters(self):
        """获取参数"""
        # 雷达话题
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        
        # 障碍物检测参数
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        
        # 避障参数
        self.safety_distance = self.get_parameter('safety_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        
        # 航点参数
        self.waypoints_param = self.get_parameter('waypoints').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # 频率参数
        self.planning_rate = self.get_parameter('planning_rate').value
        self.visualization_rate = self.get_parameter('visualization_rate').value
    
    def _init_components(self):
        """初始化组件"""
        # 障碍物检测器
        self.obstacle_detector = ObstacleDetector(
            min_height=self.min_height,
            max_height=self.max_height,
            min_distance=self.min_distance,
            max_distance=self.max_distance,
            cluster_tolerance=self.cluster_tolerance,
            min_cluster_size=self.min_cluster_size,
        )
        
        # 局部规划器
        self.local_planner = LocalPlanner(
            safety_distance=self.safety_distance,
            warning_distance=self.warning_distance,
            max_speed=self.max_speed,
            max_yaw_rate=self.max_yaw_rate,
        )
        
        # 航点导航器
        self.waypoint_navigator = WaypointNavigator(self)
        
        # 解析航点参数
        self._parse_waypoints()
    
    def _parse_waypoints(self):
        """解析航点参数"""
        if not self.waypoints_param:
            return
        
        try:
            # 航点格式: [x1, y1, z1, x2, y2, z2, ...]
            coords = [float(v) for v in self.waypoints_param]
            
            for i in range(0, len(coords), 3):
                if i + 2 < len(coords):
                    wp = Waypoint(
                        x=coords[i],
                        y=coords[i+1],
                        z=coords[i+2],
                    )
                    self.waypoint_navigator.add_waypoint(wp)
            
            self.get_logger().info(
                f'解析了 {len(self.waypoint_navigator.waypoints)} 个航点'
            )
        except Exception as e:
            self.get_logger().error(f'航点参数解析失败: {e}')
    
    def _log_parameters(self):
        """打印参数"""
        self.get_logger().info(f'  PointCloud topic: {self.pointcloud_topic}')
        self.get_logger().info(f'  Height range: [{self.min_height}, {self.max_height}]')
        self.get_logger().info(f'  Distance range: [{self.min_distance}, {self.max_distance}]')
        self.get_logger().info(f'  Safety distance: {self.safety_distance}')
        self.get_logger().info(f'  Warning distance: {self.warning_distance}')
        self.get_logger().info(f'  Max speed: {self.max_speed}')
        self.get_logger().info(f'  Planning rate: {self.planning_rate} Hz')
        self.get_logger().info(f'  Auto start: {self.auto_start}')
    
    def _pointcloud_callback(self, msg: PointCloud2):
        """
        点云回调
        
        Args:
            msg: PointCloud2 消息
        """
        try:
            # 解析点云
            points = []
            for p in point_cloud2.read_points(msg, skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if not points:
                return
            
            points_array = np.array(points)
            
            # 获取无人机位置
            drone_pos = (0.0, 0.0, 0.0)
            drone_yaw = 0.0
            
            if self.current_pose:
                drone_pos = (
                    self.current_pose.x,
                    self.current_pose.y,
                    self.current_pose.z,
                )
                drone_yaw = self.current_pose.yaw
            
            # 检测障碍物
            self.current_obstacles = self.obstacle_detector.detect(
                points_array,
                drone_position=drone_pos,
                drone_yaw=drone_yaw,
            )
            
        except Exception as e:
            self.get_logger().warn(f'点云处理异常: {e}')
    
    def _pose_callback(self, msg: UavPose):
        """位姿回调"""
        self.current_pose = msg
    
    def _state_callback(self, msg: UavState):
        """状态回调"""
        self.current_state = msg
    
    def _planning_callback(self):
        """
        规划回调
        
        定期执行避障规划，检查是否需要避障
        """
        if not self.current_pose:
            return
        
        # 检查是否有障碍物需要避障
        if not self.current_obstacles:
            return
        
        # 检查导航状态
        if self.waypoint_navigator.state != WaypointNavigator.STATE_NAVIGATING:
            return
        
        # 获取当前目标航点
        current_wp = self.waypoint_navigator.get_current_waypoint()
        if not current_wp:
            return
        
        # 检查路径是否畅通
        current_pos = (
            self.current_pose.x,
            self.current_pose.y,
            self.current_pose.z,
        )
        goal = (current_wp.x, current_wp.y, current_wp.z)
        
        if not self.local_planner.is_path_clear(
            current_pos, goal, self.current_obstacles
        ):
            self.get_logger().warn('检测到障碍物，启动避障')
            
            # 请求避障
            self.waypoint_navigator.request_avoidance(goal)
            
            # 执行避障规划
            cmd = self.local_planner.plan(
                goal=goal,
                current_pos=current_pos,
                current_yaw=self.current_pose.yaw,
                obstacles=self.current_obstacles,
            )
            
            # 发布避障指令（通过 MAVROS）
            self._publish_velocity_command(cmd)
        else:
            # 路径畅通，如果正在避障则恢复
            if self.waypoint_navigator.state == WaypointNavigator.STATE_AVOIDING:
                self.waypoint_navigator.complete_avoidance()
    
    def _publish_velocity_command(self, cmd: VelocityCommand):
        """
        发布速度指令
        
        Args:
            cmd: 速度指令
        """
        # 创建 Twist 消息
        twist = Twist()
        twist.linear.x = cmd.vx
        twist.linear.y = cmd.vy
        twist.linear.z = cmd.vz
        twist.angular.z = cmd.yaw_rate
        
        # 注意：这里需要通过 MAVROS 发布
        # 实际项目中应该使用 uav/move 服务或直接发布到 MAVROS 话题
        # 这里只是示例
        pass
    
    def _visualization_callback(self):
        """
        可视化回调
        
        发布障碍物和路径可视化标记
        """
        # 发布障碍物标记
        self._publish_obstacle_markers()
    
    def _publish_obstacle_markers(self):
        """发布障碍物可视化标记"""
        marker_array = MarkerArray()
        
        # 清除旧标记
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # 添加新标记
        for i, obs in enumerate(self.current_obstacles):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = obs.x
            marker.pose.position.y = obs.y
            marker.pose.position.z = obs.z
            
            # 大小
            marker.scale.x = obs.radius * 2
            marker.scale.y = obs.radius * 2
            marker.scale.z = 1.0
            
            # 颜色（红色，半透明）
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            
            # 生命周期
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        self.obstacle_pub.publish(marker_array)
    
    def start_navigation(self) -> bool:
        """
        开始导航
        
        Returns:
            success: 是否成功
        """
        return self.waypoint_navigator.start()
    
    def stop_navigation(self):
        """停止导航"""
        self.waypoint_navigator.stop()
    
    def add_waypoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """
        添加航点
        
        Args:
            x: X 坐标
            y: Y 坐标
            z: Z 坐标
            yaw: 偏航角
        """
        wp = Waypoint(x=x, y=y, z=z, yaw=yaw)
        self.waypoint_navigator.add_waypoint(wp)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = NavigationNode()
    
    # 自动开始导航
    if node.auto_start and node.waypoint_navigator.waypoints:
        node.get_logger().info('自动开始导航')
        node.start_navigation()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('导航节点关闭')
    finally:
        node.stop_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
