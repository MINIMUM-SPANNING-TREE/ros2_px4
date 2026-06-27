"""
相机系统 Launch 文件

启动相机节点，支持大华相机、OpenCV USB 相机和测试模式。

使用方法：
  # 默认启动（大华相机）
  ros2 launch camera uav_system.launch.py

  # OpenCV USB 相机
  ros2 launch camera uav_system.launch.py camera_type:=opencv camera_id:=0 image_width:=1280 image_height:=720

  # 测试模式
  ros2 launch camera uav_system.launch.py camera_type:=test
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # ========================================================================
    #  相机参数
    # ========================================================================
    camera_type = DeclareLaunchArgument(
        'camera_type', default_value='dahua',
        description='相机类型：dahua / opencv / test')
    camera_id = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='相机设备 ID（索引/序列号/IP）')
    image_width = DeclareLaunchArgument(
        'image_width', default_value='1920',
        description='图像宽度')
    image_height = DeclareLaunchArgument(
        'image_height', default_value='1080',
        description='图像高度')
    camera_fps = DeclareLaunchArgument(
        'camera_fps', default_value='30.0',
        description='相机帧率')
    camera_exposure = DeclareLaunchArgument(
        'camera_exposure', default_value='10000.0',
        description='曝光时间（微秒，仅大华相机）')
    camera_gain = DeclareLaunchArgument(
        'camera_gain', default_value='0.0',
        description='增益 dB（仅大华相机）')
    image_topic = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='相机发布的话题名称')

    # ========================================================================
    #  开关
    # ========================================================================
    enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='是否启动相机节点')

    # ========================================================================
    #  节点定义
    # ========================================================================

    # 大华相机
    dahua_camera_node = Node(
        package='camera',
        executable='camera_node',
        name='dahua_camera',
        output='screen',
        parameters=[{
            'camera_type': 'dahua',
            'camera_id': LaunchConfiguration('camera_id'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'fps': LaunchConfiguration('camera_fps'),
            'exposure': LaunchConfiguration('camera_exposure'),
            'gain': LaunchConfiguration('camera_gain'),
        }],
        remappings=[
            ('~/image_raw', LaunchConfiguration('image_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    # OpenCV USB 相机
    opencv_camera_node = Node(
        package='camera',
        executable='camera_node',
        name='opencv_camera',
        output='screen',
        parameters=[{
            'camera_type': 'opencv',
            'camera_id': LaunchConfiguration('camera_id'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'fps': LaunchConfiguration('camera_fps'),
        }],
        remappings=[
            ('~/image_raw', LaunchConfiguration('image_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    # 测试模式（无真实相机）
    test_camera_node = Node(
        package='camera',
        executable='camera_test_node',
        name='test_camera',
        output='screen',
        parameters=[{
            'mode': 'pattern',
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'fps': LaunchConfiguration('camera_fps'),
        }],
        remappings=[
            ('~/image_raw', LaunchConfiguration('image_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    # ========================================================================
    #  组装 Launch Description
    # ========================================================================
    return LaunchDescription([
        # 开关
        enable_camera,

        # 相机
        camera_type,
        camera_id,
        image_width,
        image_height,
        camera_fps,
        camera_exposure,
        camera_gain,
        image_topic,

        # 启动提示
        LogInfo(msg=['=== 相机系统启动 ===']),
        LogInfo(msg=['相机类型: ', LaunchConfiguration('camera_type')]),

        # 节点
        dahua_camera_node,
        opencv_camera_node,
        test_camera_node,
    ])
