"""
UAV 导航系统 Launch 文件 (C++ 版本)

启动导航节点，提供以下服务：
  nav/set_goal        - 设置目标点
  nav/add_waypoint    - 添加航点
  nav/start           - 开始导航
  nav/stop            - 停止/暂停导航
  nav/get_status      - 获取导航状态
  nav/clear_waypoints - 清空航点

使用方法：
  # 默认启动
  ros2 launch navigation navigation.launch.py

  # 指定参数
  ros2 launch navigation navigation.launch.py safety_distance:=1.5 max_speed:=1.5

  # 设置目标点
  ros2 service call /nav/set_goal uav_interfaces/srv/NavSetGoal "{x: 5.0, y: 0.0, z: 2.0, yaw: 0.0}"

  # 添加航点
  ros2 service call /nav/add_waypoint uav_interfaces/srv/NavAddWaypoint "{x: 5.0, y: 0.0, z: 2.0}"

  # 开始导航
  ros2 service call /nav/start uav_interfaces/srv/NavStart "{auto_start: true}"

  # 获取状态
  ros2 service call /nav/get_status uav_interfaces/srv/NavGetStatus

  # 停止导航
  ros2 service call /nav/stop uav_interfaces/srv/NavStop "{pause: false}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 参数
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/lslidar_point_cloud',
            description='激光雷达点云话题'
        ),
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
            description='最大速度（m/s）'
        ),
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='0.5',
            description='最大偏航角速度（rad/s）'
        ),
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

        # 启动日志
        LogInfo(msg=['=== UAV 导航系统启动 (C++) ===']),
        LogInfo(msg=['点云话题: ', LaunchConfiguration('pointcloud_topic')]),
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
                'safety_distance': LaunchConfiguration('safety_distance'),
                'warning_distance': LaunchConfiguration('warning_distance'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                'planning_rate': LaunchConfiguration('planning_rate'),
                'visualization_rate': LaunchConfiguration('visualization_rate'),
            }],
        ),
    ])
