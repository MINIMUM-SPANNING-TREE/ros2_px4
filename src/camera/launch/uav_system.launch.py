"""
UAV 无人机系统 — 整体 Launch 文件

启动顺序（按依赖关系）：
  1. uav_mavros2 遥测节点（转发 MAVROS 数据）
  2. uav_mavros2 控制服务节点（起飞/移动/降落/返航服务）
  3. camera 相机节点（图像采集）
  4. vision yolo_node（YOLOv5 目标检测）
  5. vision deepsort_tracker_node（DeepSORT 多目标跟踪）

使用方法：
  # 默认启动（使用大华相机 + ONNX 模型）
  ros2 launch camera uav_system.launch.py

  # 指定 YOLO 模型路径
  ros2 launch camera uav_system.launch.py yolo_model:=/path/to/yolov5s.onnx

  # 使用 USB 相机 + 自定义分辨率
  ros2 launch camera uav_system.launch.py camera_type:=opencv camera_id:=0 image_width:=1280 image_height:=720

  # 切换 DeepSORT 特征模型（simple / osnet / resnet）
  ros2 launch camera uav_system.launch.py deepsort_model_name:=osnet

  # 跳过视觉模块（仅飞控）
  ros2 launch camera uav_system.launch.py enable_vision:=false

  # 启用激光雷达
  ros2 launch camera uav_system.launch.py enable_lidar:=true

  # 使用 RDK BPU NPU 推理
  ros2 launch camera uav_system.launch.py yolo_backend:=bpu yolo_model:=/path/to/model.bin
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def get_default_model_path(model_name):
    """获取 vision 包内默认模型路径"""
    try:
        vision_share = FindPackageShare('vision')
        return PathJoinSubstitution([vision_share, 'models', model_name])
    except Exception:
        return model_name


def generate_launch_description():

    # ========================================================================
    #  全局开关
    # ========================================================================
    enable_mavros_nodes = DeclareLaunchArgument(
        'enable_mavros_nodes', default_value='true',
        description='是否启动 MAVROS 遥测和控制服务节点')
    enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='是否启动相机节点')
    enable_vision = DeclareLaunchArgument(
        'enable_vision', default_value='true',
        description='是否启动 YOLO + DeepSORT 视觉流水线')
    enable_lidar = DeclareLaunchArgument(
        'enable_lidar', default_value='false',
        description='是否启动激光雷达驱动')

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
        description='相机发布的话题名称（视觉节点会订阅此话题）')

    # ========================================================================
    #  YOLOv5 检测参数
    # ========================================================================
    yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value=get_default_model_path('yolov5s.onnx'),
        description='YOLOv5 模型文件路径（.onnx / .bin）')
    yolo_backend = DeclareLaunchArgument(
        'yolo_backend', default_value='auto',
        description='推理后端：auto / onnx / bpu / opencv')
    yolo_input_size = DeclareLaunchArgument(
        'yolo_input_size', default_value='640',
        description='YOLO 输入尺寸（正方形边长）')
    yolo_conf = DeclareLaunchArgument(
        'yolo_conf', default_value='0.45',
        description='YOLO 置信度阈值')
    yolo_iou = DeclareLaunchArgument(
        'yolo_iou', default_value='0.45',
        description='YOLO NMS IoU 阈值')
    yolo_device = DeclareLaunchArgument(
        'yolo_device', default_value='cpu',
        description='YOLO 推理设备：cpu / cuda:0')
    yolo_classes = DeclareLaunchArgument(
        'yolo_classes', default_value='',
        description='过滤类别（逗号分隔，如 "0,1,2"；空=全部）')
    yolo_show_fps = DeclareLaunchArgument(
        'yolo_show_fps', default_value='true',
        description='是否在日志中显示 FPS')

    # ========================================================================
    #  DeepSORT 跟踪参数
    # ========================================================================
    deepsort_model_name = DeclareLaunchArgument(
        'deepsort_model_name', default_value='simple',
        description='特征提取模型类型：simple / osnet / resnet')
    deepsort_max_cosine = DeclareLaunchArgument(
        'deepsort_max_cosine', default_value='0.2',
        description='外观特征最大余弦距离阈值（越小越严格）')
    deepsort_max_iou = DeclareLaunchArgument(
        'deepsort_max_iou', default_value='0.7',
        description='IoU 匹配阈值')
    deepsort_max_age = DeclareLaunchArgument(
        'deepsort_max_age', default_value='30',
        description='轨迹最大丢失帧数（超过则删除）')
    deepsort_n_init = DeclareLaunchArgument(
        'deepsort_n_init', default_value='3',
        description='轨迹确认所需连续命中帧数')
    deepsort_nn_budget = DeclareLaunchArgument(
        'deepsort_nn_budget', default_value='100',
        description='外观特征 gallery 最大容量')
    deepsort_confidence = DeclareLaunchArgument(
        'deepsort_confidence', default_value='0.5',
        description='跟踪置信度过滤阈值')
    deepsort_device = DeclareLaunchArgument(
        'deepsort_device', default_value='cpu',
        description='DeepSORT 推理设备：cpu / cuda:0')
    show_visualization = DeclareLaunchArgument(
        'show_visualization', default_value='true',
        description='是否发布可视化跟踪图像')
    show_trajectory = DeclareLaunchArgument(
        'show_trajectory', default_value='true',
        description='是否显示轨迹线')
    trajectory_length = DeclareLaunchArgument(
        'trajectory_length', default_value='30',
        description='轨迹显示长度（帧数）')

    # ========================================================================
    #  遥测 & 控制参数
    # ========================================================================
    telemetry_rate = DeclareLaunchArgument(
        'telemetry_rate', default_value='10.0',
        description='遥测发布频率 Hz')

    # ========================================================================
    #  节点定义
    # ========================================================================

    # ------ 1. 遥测节点 ------
    telemetry_node = Node(
        package='uav_mavros2',
        executable='telemetry_node',
        name='mavros_telemetry_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('telemetry_rate'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_mavros_nodes')),
    )

    # ------ 2. 控制服务节点（延迟 2s 等待遥测就绪）------
    ctrl_server_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='uav_mavros2',
                executable='ctrl_server_node',
                name='uav_controller_service_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_mavros_nodes')),
            ),
        ],
    )

    # ------ 3. 相机节点 ------
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

    # ------ 4. YOLOv5 检测节点（延迟 3s 等待相机就绪）------
    yolo_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='vision',
                executable='yolo_node',
                name='yolo_detector',
                output='screen',
                parameters=[{
                    'model_path': LaunchConfiguration('yolo_model'),
                    'backend': LaunchConfiguration('yolo_backend'),
                    'input_size': [
                        LaunchConfiguration('yolo_input_size'),
                        LaunchConfiguration('yolo_input_size'),
                    ],
                    'confidence_threshold': LaunchConfiguration('yolo_conf'),
                    'nms_threshold': LaunchConfiguration('yolo_iou'),
                    'device': LaunchConfiguration('yolo_device'),
                    'classes': LaunchConfiguration('yolo_classes'),
                    'show_fps': LaunchConfiguration('yolo_show_fps'),
                }],
                remappings=[
                    ('~/image_raw', LaunchConfiguration('image_topic')),
                    ('~/detections', '/detections'),
                    ('~/detection_image', '/detection_image'),
                ],
                condition=IfCondition(LaunchConfiguration('enable_vision')),
            ),
        ],
    )

    # ------ 5. DeepSORT 跟踪节点（延迟 5s 等待检测节点就绪）------
    deepsort_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='vision',
                executable='deepsort_tracker_node',
                name='deepsort_tracker',
                output='screen',
                parameters=[{
                    'model_name': LaunchConfiguration('deepsort_model_name'),
                    'max_cosine_distance': LaunchConfiguration('deepsort_max_cosine'),
                    'max_iou_distance': LaunchConfiguration('deepsort_max_iou'),
                    'max_age': LaunchConfiguration('deepsort_max_age'),
                    'n_init': LaunchConfiguration('deepsort_n_init'),
                    'nn_budget': LaunchConfiguration('deepsort_nn_budget'),
                    'confidence_threshold': LaunchConfiguration('deepsort_confidence'),
                    'device': LaunchConfiguration('deepsort_device'),
                    'show_visualization': LaunchConfiguration('show_visualization'),
                    'show_trajectory': LaunchConfiguration('show_trajectory'),
                    'trajectory_length': LaunchConfiguration('trajectory_length'),
                }],
                remappings=[
                    ('~/image_raw', LaunchConfiguration('image_topic')),
                    ('~/detections', '/detections'),
                    ('~/tracks', '/tracks'),
                    ('~/tracks_image', '/tracks_image'),
                ],
                condition=IfCondition(LaunchConfiguration('enable_vision')),
            ),
        ],
    )

    # ------ 6. 激光雷达驱动（可选）------
    lslidar_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'config', 'lslidar_cx.yaml',
            ]),
        ],
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    # ========================================================================
    #  组装 Launch Description
    # ========================================================================
    return LaunchDescription([
        # 开关
        enable_mavros_nodes,
        enable_camera,
        enable_vision,
        enable_lidar,

        # 相机
        camera_type,
        camera_id,
        image_width,
        image_height,
        camera_fps,
        camera_exposure,
        camera_gain,
        image_topic,

        # YOLO
        yolo_model,
        yolo_backend,
        yolo_input_size,
        yolo_conf,
        yolo_iou,
        yolo_device,
        yolo_classes,
        yolo_show_fps,

        # DeepSORT
        deepsort_model_name,
        deepsort_max_cosine,
        deepsort_max_iou,
        deepsort_max_age,
        deepsort_n_init,
        deepsort_nn_budget,
        deepsort_confidence,
        deepsort_device,
        show_visualization,
        show_trajectory,
        trajectory_length,

        # 遥测
        telemetry_rate,

        # 启动提示
        LogInfo(msg=['=== UAV 系统启动中 ===']),
        LogInfo(msg=['相机类型: ', LaunchConfiguration('camera_type')]),
        LogInfo(msg=['YOLO 模型: ', LaunchConfiguration('yolo_model')]),
        LogInfo(msg=['DeepSORT 模型: ', LaunchConfiguration('deepsort_model_name')]),

        # 节点（按启动顺序）
        telemetry_node,      # 1. 遥测（立即）
        ctrl_server_node,    # 2. 控制服务（延迟 2s）
        dahua_camera_node,   # 3a. 大华相机
        opencv_camera_node,  # 3b. OpenCV 相机
        test_camera_node,    # 3c. 测试相机
        yolo_node,           # 4. YOLO 检测（延迟 3s）
        deepsort_node,       # 5. DeepSORT 跟踪（延迟 5s）
        lslidar_node,        # 6. 激光雷达（可选）
    ])
