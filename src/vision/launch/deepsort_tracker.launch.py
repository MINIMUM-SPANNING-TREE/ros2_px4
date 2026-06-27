"""
DeepSORT Tracker Launch 文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'model_name',
            default_value='simple',
            description='特征提取模型 (simple, osnet, resnet)'
        ),
        DeclareLaunchArgument(
            'max_cosine_distance',
            default_value='0.2',
            description='外观特征最大余弦距离阈值'
        ),
        DeclareLaunchArgument(
            'max_iou_distance',
            default_value='0.7',
            description='IoU 距离阈值'
        ),
        DeclareLaunchArgument(
            'max_age',
            default_value='30',
            description='最大丢失帧数'
        ),
        DeclareLaunchArgument(
            'n_init',
            default_value='3',
            description='确认跟踪所需的连续匹配次数'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='检测置信度阈值'
        ),
        DeclareLaunchArgument(
            'reid_model',
            default_value='',
            description='ReID 模型路径（ONNX 或 BPU，留空使用 model_name）'
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='auto',
            description='推理后端 (auto/onnx/bpu/simple)'
        ),
        DeclareLaunchArgument(
            'show_visualization',
            default_value='true',
            description='是否发布可视化图像'
        ),
        
        # DeepSORT 跟踪节点
        Node(
            package='vision',
            executable='deepsort_tracker_node',
            name='deepsort_tracker',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'reid_model': LaunchConfiguration('reid_model'),
                'backend': LaunchConfiguration('backend'),
                'max_cosine_distance': LaunchConfiguration('max_cosine_distance'),
                'max_iou_distance': LaunchConfiguration('max_iou_distance'),
                'max_age': LaunchConfiguration('max_age'),
                'n_init': LaunchConfiguration('n_init'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'show_visualization': LaunchConfiguration('show_visualization'),
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/detections', '/detections'),
                ('~/tracks', '/tracks'),
                ('~/tracks_image', '/tracks_image'),
            ],
        ),
    ])
