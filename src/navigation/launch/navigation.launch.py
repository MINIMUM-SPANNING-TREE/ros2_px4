"""
UAV 导航系统 Launch 文件

启动导航节点，使用激光雷达进行避障导航。

使用方法：
  # 默认启动
  ros2 launch navigation navigation.launch.py

  # 指定航点
  ros2 launch navigation navigation.launch.py waypoints:="[0.0, 0.0, 2.0, 5.0, 0.0, 2.0, 5.0, 5.0, 2.0]"

  # 自动开始导航
  ros2 launch navigation navigation.launch.py auto_start:=true

  # 指定雷达话题
  ros2 launch navigation navigation.launch.py pointcloud_topic:=/lslidar_point_cloud
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 雷达话题
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/lslidar_point_cloud',
            description='激光雷达点云话题'
        ),
        
        # 障碍物检测参数
        DeclareLaunchArgument(
            'min_height',
            default_value='-1.0',
            description='最低检测高度（米）'
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='3.0',
            description='最高检测高度（米）'
        ),
        DeclareLaunchArgument(
            'min_distance',
            default_value='0.5',
            description='最近检测距离（米）'
        ),
        DeclareLaunchArgument(
            'max_distance',
            default_value='10.0',
            description='最远检测距离（米）'
        ),
        DeclareLaunchArgument(
            'cluster_tolerance',
            default_value='0.5',
            description='聚类容差（米）'
        ),
        DeclareLaunchArgument(
            'min_cluster_size',
            default_value='5',
            description='最小聚类点数'
        ),
        
        # 避障参数
        DeclareLaunchArgument(
            'safety_distance',
            default_value='2.0',
            description='安全距离（米）'
        ),
        DeclareLaunchArgument(
            'warning_distance',
            default_value='3.0',
            description='警告距离（米）'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='2.0',
            description='最大飞行速度（m/s）'
        ),
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='0.5',
            description='最大偏航角速度（rad/s）'
        ),
        
        # 航点参数
        DeclareLaunchArgument(
            'waypoints',
            default_value='[]',
            description='航点列表，格式: [x1, y1, z1, x2, y2, z2, ...]'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='是否自动开始导航'
        ),
        
        # 频率参数
        DeclareLaunchArgument(
            'planning_rate',
            default_value='10.0',
            description='规划频率（Hz）'
        ),
        DeclareLaunchArgument(
            'visualization_rate',
            default_value='5.0',
            description='可视化频率（Hz）'
        ),
        
        # 启动提示
        LogInfo(msg=['=== UAV 导航系统启动中 ===']),
        LogInfo(msg=['雷达话题: ', LaunchConfiguration('pointcloud_topic')]),
        LogInfo(msg=['安全距离: ', LaunchConfiguration('safety_distance'), 'm']),
        LogInfo(msg=['最大速度: ', LaunchConfiguration('max_speed'), 'm/s']),
        
        # 导航节点
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'min_height': LaunchConfiguration('min_height'),
                'max_height': LaunchConfiguration('max_height'),
                'min_distance': LaunchConfiguration('min_distance'),
                'max_distance': LaunchConfiguration('max_distance'),
                'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                'safety_distance': LaunchConfiguration('safety_distance'),
                'warning_distance': LaunchConfiguration('warning_distance'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                'waypoints': LaunchConfiguration('waypoints'),
                'auto_start': LaunchConfiguration('auto_start'),
                'planning_rate': LaunchConfiguration('planning_rate'),
                'visualization_rate': LaunchConfiguration('visualization_rate'),
            }],
        ),
    ])
