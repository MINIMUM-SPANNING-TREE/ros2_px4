import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch import LaunchDescription


def generate_launch_description():
    driver_config = os.path.join(
        get_package_share_directory('lslidar_driver'), 'config', 'lslidar_n10_uart_ns.yaml')

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='',
        parameters=[driver_config],
        output='screen',
    )

    return LaunchDescription([driver_node])
