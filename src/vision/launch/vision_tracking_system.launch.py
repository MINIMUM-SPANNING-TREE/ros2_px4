"""
视觉跟踪系统 Launch 文件
YOLOv5 检测 + DeepSORT 跟踪
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # YOLO 参数
        DeclareLaunchArgument(
            'yolo_model',
            default_value='',
            description='YOLOv5 模型路径（留空使用默认）'
        ),
        DeclareLaunchArgument(
            'yolo_conf',
            default_value='0.45',
            description='YOLO 置信度阈值'
        ),
        DeclareLaunchArgument(
            'yolo_iou',
            default_value='0.45',
            description='YOLO IoU 阈值'
        ),

        # DeepSORT 参数
        DeclareLaunchArgument(
            'deepsort_model',
            default_value='simple',
            description='DeepSORT 特征模型'
        ),
        DeclareLaunchArgument(
            'reid_model',
            default_value='',
            description='ReID 模型路径（ONNX 或 BPU，留空使用 deepsort_model）'
        ),
        DeclareLaunchArgument(
            'deepsort_backend',
            default_value='auto',
            description='推理后端 (auto/onnx/bpu/simple)'
        ),
        DeclareLaunchArgument(
            'max_age',
            default_value='30',
            description='最大丢失帧数'
        ),

        # 可视化参数
        DeclareLaunchArgument(
            'show_yolo_detection',
            default_value='true',
            description='显示 YOLO 检测结果'
        ),
        DeclareLaunchArgument(
            'show_deepsort_tracks',
            default_value='true',
            description='显示 DeepSORT 跟踪结果'
        ),

        # YOLOv5 检测节点
        Node(
            package='vision',
            executable='yolo_node',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('yolo_model'),
                'input_size': 640,
                'conf_threshold': LaunchConfiguration('yolo_conf'),
                'iou_threshold': LaunchConfiguration('yolo_iou'),
                'backend': 'auto',
                'show_detection': LaunchConfiguration('show_yolo_detection'),
                'show_labels': True,
                'show_scores': True,
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/detections', '/yolo/detections'),
                ('~/detection_image', '/yolo/detection_image'),
            ],
        ),

        # DeepSORT 跟踪节点
        Node(
            package='vision',
            executable='deepsort_tracker_node',
            name='deepsort_tracker',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('deepsort_model'),
                'reid_model': LaunchConfiguration('reid_model'),
                'backend': LaunchConfiguration('deepsort_backend'),
                'max_cosine_distance': 0.2,
                'max_age': LaunchConfiguration('max_age'),
                'n_init': 3,
                'confidence_threshold': 0.3,
                'show_visualization': LaunchConfiguration('show_deepsort_tracks'),
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/detections', '/yolo/detections'),
                ('~/tracks', '/deepsort/tracks'),
                ('~/tracks_image', '/deepsort/tracks_image'),
            ],
        ),
    ])
