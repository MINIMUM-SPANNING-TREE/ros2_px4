"""
相机+DeepSORT跟踪系统 Launch 文件
同时启动相机节点和DeepSORT跟踪节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
        
        # DeepSORT参数
        DeclareLaunchArgument(
            'model_name',
            default_value='simple',
            description='DeepSORT特征模型'
        ),
        DeclareLaunchArgument(
            'max_cosine_distance',
            default_value='0.2',
            description='最大余弦距离'
        ),
        DeclareLaunchArgument(
            'max_age',
            default_value='30',
            description='最大丢失帧数'
        ),
        DeclareLaunchArgument(
            'n_init',
            default_value='3',
            description='确认帧数'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='置信度阈值'
        ),
        
        # 使用测试节点
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
            condition=LaunchConfiguration('use_test_camera', default='false'),
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
            condition=LaunchConfiguration('use_test_camera'),
        ),
        
        # DeepSORT跟踪节点
        Node(
            package='vision',
            executable='deepsort_tracker_node',
            name='deepsort_tracker',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'max_cosine_distance': LaunchConfiguration('max_cosine_distance'),
                'max_age': LaunchConfiguration('max_age'),
                'n_init': LaunchConfiguration('n_init'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'show_visualization': True,
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/detections', '/detections'),
                ('~/tracks', '/tracks'),
                ('~/tracks_image', '/tracks_image'),
            ],
        ),
    ])
