"""
相机节点 Launch 文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # 相机参数
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='相机ID'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1920',
            description='图像宽度'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='1080',
            description='图像高度'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='帧率'
        ),
        DeclareLaunchArgument(
            'use_test_camera',
            default_value='false',
            description='是否使用测试相机节点'
        ),

        # 相机节点 - 真实相机
        Node(
            package='camera',
            executable='camera_node',
            name='dahua_camera',
            output='screen',
            parameters=[{
                'camera_type': 'dahua',
                'camera_id': LaunchConfiguration('camera_id'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': 'camera',
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/camera_info', '/camera/camera_info'),
            ],
            condition=IfCondition(LaunchConfiguration('use_test_camera', default='false')),
        ),

        # 相机节点 - 测试节点
        Node(
            package='camera',
            executable='camera_test_node',
            name='camera_test',
            output='screen',
            parameters=[{
                'mode': 'pattern',
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': 'camera',
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
            ],
            condition=IfCondition(LaunchConfiguration('use_test_camera')),
        ),
    ])
