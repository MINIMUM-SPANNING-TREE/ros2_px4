"""Launch FSM helper services."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tracks_topic',
            default_value='/deepsort/tracks',
            description='DeepSORT track topic used by FSM nodes',
        ),
        DeclareLaunchArgument(
            'yolo_topic',
            default_value='/yolo/detections',
            description='YOLO detection topic to save locally',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='',
            description='Directory for saved vision results. Empty uses data/vision_results/<timestamp>.',
        ),
        DeclareLaunchArgument(
            'flush_every',
            default_value='1',
            description='Flush result files after this many writes',
        ),
        DeclareLaunchArgument(
            'target_service_name',
            default_value='fsm/track_target',
            description='Target tracking service name',
        ),
        DeclareLaunchArgument(
            'velocity_service',
            default_value='uav/set_velocity',
            description='UAV velocity service used for target tracking control',
        ),
        DeclareLaunchArgument(
            'control_enabled',
            default_value='true',
            description='Enable velocity control when target tracking is active',
        ),
        DeclareLaunchArgument(
            'tracking_duration_sec',
            default_value='0.0',
            description='Auto-stop tracking after this many seconds. 0 disables the limit.',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640.0',
            description='Tracking image width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480.0',
            description='Tracking image height in pixels',
        ),
        DeclareLaunchArgument(
            'y_kd',
            default_value='0.0',
            description='Derivative gain for horizontal target error to lateral speed',
        ),
        DeclareLaunchArgument(
            'z_kd',
            default_value='0.0',
            description='Derivative gain for vertical target error to vertical speed',
        ),
        DeclareLaunchArgument(
            'x_kd',
            default_value='0.0',
            description='Derivative gain for target area error to forward speed',
        ),

        Node(
            package='fsm',
            executable='track',
            name='target_tracking_service',
            output='screen',
            parameters=[{
                'tracks_topic': LaunchConfiguration('tracks_topic'),
                'service_name': LaunchConfiguration('target_service_name'),
                'velocity_service': LaunchConfiguration('velocity_service'),
                'control_enabled': LaunchConfiguration('control_enabled'),
                'tracking_duration_sec': LaunchConfiguration('tracking_duration_sec'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'y_kd': LaunchConfiguration('y_kd'),
                'z_kd': LaunchConfiguration('z_kd'),
                'x_kd': LaunchConfiguration('x_kd'),
            }],
        ),
        Node(
            package='fsm',
            executable='result',
            name='vision_result_logger',
            output='screen',
            parameters=[{
                'yolo_topic': LaunchConfiguration('yolo_topic'),
                'deepsort_topic': LaunchConfiguration('tracks_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'flush_every': LaunchConfiguration('flush_every'),
            }],
        ),
    ])
