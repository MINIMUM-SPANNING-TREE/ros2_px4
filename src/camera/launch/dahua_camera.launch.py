"""
大华相机 Launch 文件 - 专用配置
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 大华相机参数
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='大华相机索引（通常为0或1）'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='图像宽度（大华A5131CU210支持1280x720）'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='图像高度'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='15.0',
            description='目标帧率'
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='/dahua_camera/image_raw',
            description='发布的话题名称'
        ),
        
        # 大华相机节点
        Node(
            package='camera',
            executable='camera_test_node',
            name='dahua_camera',
            output='screen',
            parameters=[{
                'mode': 'dahua',
                'camera_id': LaunchConfiguration('camera_id'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': 'dahua_camera',
            }],
            remappings=[
                ('~/image_raw', LaunchConfiguration('topic_name')),
            ],
        ),
    ])
