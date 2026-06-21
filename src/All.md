# UAV 无人机系统 ROS2 工程全景文档

> 工程路径：`~/px4/ros2_px4/src/`  
> 基于 ROS2 Humble，使用 MAVROS 桥接 PX4 飞控

---

## 目录

- [1. 工程总览](#1-工程总览)
- [2. uav_interfaces — 自定义消息/服务接口包](#2-uav_interfaces--自定义消息服务接口包)
- [3. uav_mavros2 — 飞控通信与控制核心包](#3-uav_mavros2--飞控通信与控制核心包)
- [4. camera — 工业相机驱动包](#4-camera--工业相机驱动包)
- [5. vision — 视觉检测与跟踪包](#5-vision--视觉检测与跟踪包)
- [6. mission — 任务规划与交互控制包](#6-mission--任务规划与交互控制包)
- [7. fsm — 飞行任务测试包](#7-fsm--飞行任务测试包)
- [8. lslidar_driver — 雷神激光雷达驱动包](#8-lslidar_driver--雷神激光雷达驱动包)
- [9. 系统架构与节点关系图](#9-系统架构与节点关系图)
- [10. 启动顺序建议](#10-启动顺序建议)

---

## 1. 工程总览

本工程是一个完整的 ROS2 无人机系统，包含 7 个功能包，覆盖从传感器采集、飞控通信、目标检测跟踪到任务执行的完整链路：

| 包名 | 语言 | 类型 | 核心职责 |
|------|------|------|---------|
| `uav_interfaces` | C++ (msg/srv) | 接口包 | 定义所有自定义消息和服务 |
| `uav_mavros2` | Python | 控制包 | MAVROS 桥接、遥测发布、飞控服务 |
| `camera` | Python | 驱动包 | 大华工业相机/USB 相机图像采集 |
| `vision` | Python | 算法包 | YOLOv5 检测 + DeepSORT 多目标跟踪 |
| `mission` | Python | 应用包 | 交互式任务控制台（CLI） |
| `fsm` | Python | 测试包 | 飞行任务功能验证脚本 |
| `lslidar_driver` | C++ | 驱动包 | 雷神激光雷达驱动（点云发布） |

---

## 2. uav_interfaces — 自定义消息/服务接口包

**类型**：C++ ament_cmake 接口包（纯定义，无运行时节点）

### 2.1 消息定义（msg/）

#### `UavState.msg` — 飞控连接与飞行状态

| 字段 | 类型 | 说明 |
|------|------|------|
| header | std_msgs/Header | 时间戳 |
| connected | bool | 是否与飞控建立 MAVLink 连接 |
| armed | bool | 是否已解锁（电机可转动） |
| guided | bool | 是否接受外部控制指令 |
| mode | string | 当前飞行模式（OFFBOARD / AUTO.TAKEOFF / AUTO.LAND / AUTO.RTL 等） |
| landed_state | string | 着陆状态（ON_GROUND / IN_AIR / TAKING_OFF / LANDING） |

#### `UavPose.msg` — 无人机位姿（ENU 坐标系）

| 字段 | 类型 | 说明 |
|------|------|------|
| header | std_msgs/Header | 时间戳 |
| x | float64 | 东向位置（米） |
| y | float64 | 北向位置（米） |
| z | float64 | 高度（米，向上为正） |
| yaw | float64 | 偏航角（弧度，-π ~ +π） |

#### `UavHealth.msg` — 系统健康状态

| 字段 | 类型 | 说明 |
|------|------|------|
| header | std_msgs/Header | 时间戳 |
| level | int8 | 0=OK, 1=WARNING, 2=ERROR |
| message | string | 详细描述（NOMINAL / LOW_BATTERY / POSITION_LOST 等） |
| connected | bool | 通信连接状态 |

#### `LslidarInformation.msg` — 激光雷达设备信息

包含 lidar_ip、destination_ip、mac_address、端口、序列号、FPGA 版本等。

#### `LslidarPacket.msg` — 雷达原始数据包

包含 timestamp 和 1212 字节原始数据。

### 2.2 服务定义（srv/）

| 服务文件 | 请求字段 | 用途 |
|---------|---------|------|
| `Takeoff.srv` | `float64 relative_alt` | 相对高度起飞 |
| `Move.srv` | `float64 x, y, z, yaw` | 绝对位置移动 |
| `Land.srv` | `float64 timeout` | 降落（可设超时） |
| `Rtl.srv` | `float64 timeout` | 返航 |
| `Arm.srv` | `bool arm` | 解锁/上锁电机 |
| `SetMode.srv` | `string mode` | 切换飞行模式 |
| 雷达相关服务 | 角度畸变矫正/帧率/电机控制/功率控制等 | 雷达参数配置 |

所有服务响应格式统一为 `bool success + string message`。

---

## 3. uav_mavros2 — 飞控通信与控制核心包

**类型**：Python ament_python 包  
**依赖**：MAVROS、uav_interfaces

### 3.1 基类架构

```
UavBase（uavbase.py）
  ├── 订阅 MAVROS 原始话题（state/pose/imu/rc/battery/home）
  ├── 缓存最新状态
  ├── 提供 arm() / set_mode() / takeoff_auto() / land_auto() / move_to()
  └── 定时器回调（发布 setpoint，维持 OFFBOARD 心跳）
        │
Uav（uav.py）继承 UavBase
  └── 增加高层封装
        │
UavServer（uavserver.py）继承 Uav
  └── 注册 ROS2 服务，对外暴露控制接口
```

### 3.2 节点清单

#### `telemetry_node`（`TelemetryNode`）— 遥测数据转发节点

**源码**：`uav_mavros2/telemetry.py`

桥接 MAVROS 原始话题，将复杂数据简化为自定义格式发布。

| 订阅（MAVROS） | 发布（简化格式） |
|----------------|-----------------|
| `/mavros/state` | `/uav/telemetry/state`（UavState） |
| `/mavros/extended_state` | `/uav/telemetry/pose`（UavPose） |
| `/mavros/local_position/pose` | |
| `/mavros/local_position/velocity` | |

提取 armed/guided/mode/landed_state，从四元数计算 yaw，10Hz 定时发布。

---

#### `ctrl_server_node`（`UavServer`）— 飞控服务节点

**源码**：`uav_mavros2/uavserver.py` + `uav_mavros2/launch_server.py`

系统的控制中枢，所有飞控指令通过该节点执行。

| 服务名 | 服务类型 | 功能 |
|-------|---------|------|
| `uav/takeoff` | Takeoff | 相对高度起飞（OFFBOARD 模式） |
| `uav/move` | Move | 绝对位置移动（OFFBOARD 模式） |
| `uav/land` | Land | 自动降落（AUTO.LAND 模式） |
| `uav/rtl` | Rtl | 返航（AUTO.RTL 模式） |
| `uav/arm` | Arm | 解锁/上锁电机 |
| `uav/set_mode` | SetMode | 切换飞行模式 |

**控制流程**：
- 起飞：切换 OFFBOARD → 发送 setpoint → 解锁 → 爬升 → 到达后切回 AUTO.LOITER
- 移动：循环发布目标位置，到达容差范围内后悬停
- 使用 ReentrantCallbackGroup + MultiThreadedExecutor(4) 实现并发

---

#### `uav_ctrl_node`（`UavController`）— 独立控制器

**源码**：`uav_mavros2/uav_ctrl.py`

独立的飞控控制器，提供直接方法调用（非服务），主要用于测试脚本。

核心方法：`takeoff_auto()` / `land_auto()` / `move_to()` / `rtl()`

---

#### `print_node`（`TelemetryPrinter`）— 遥测打印节点

**源码**：`uav_mavros2/print.py`

以人类可读格式低频打印状态（模式/位姿/电池/RC），仅状态变化时打印 + 5 秒心跳，避免刷屏。

---

## 4. camera — 工业相机驱动包

**类型**：Python ament_python 包  
**依赖**：rclpy、sensor_msgs、cv_bridge、image_transport

### 4.1 节点清单

#### `camera_node`（`CameraNode`）— 通用相机节点

**源码**：`camera/camera_node.py`

| 发布话题 | 消息类型 | 说明 |
|---------|---------|------|
| `~/image_raw` | sensor_msgs/Image | 原始图像 |
| `~/camera_info` | sensor_msgs/CameraInfo | 相机内参 |

支持：大华工业相机（MVSDK）、OpenCV USB 相机、图像文件/视频流。

参数：`camera_type` / `image_width` / `image_height` / `fps` / `exposure` / `gain`

---

#### `dahua_camera_node`（`DahuaCameraNode`）— 大华相机专用节点

**源码**：`camera/dahua_camera_node.py`

直接使用大华 MVSDK 接口，支持触发模式（连续/软触发/硬触发）。

---

#### `camera_test_node`（`CameraTestNode`）— 测试图像生成节点

**源码**：`camera/camera_test_node.py`

无真实相机时的测试工具，支持三种模式：
- `pattern` — 生成棋盘格/彩色条纹测试图案
- `video` — 从视频文件读取
- `camera` — 使用 OpenCV USB 相机

### 4.2 支撑模块

| 模块 | 功能 |
|------|------|
| `camera_base.py` | 相机抽象基类 |
| `dahua_camera.py` | 大华 MVSDK 封装 |

---

## 5. vision — 视觉检测与跟踪包

**类型**：Python ament_python 包  
**依赖**：rclpy、vision_msgs、numpy、scipy

### 5.1 节点清单

#### `yolo_node`（`YOLONode`）— YOLOv5 目标检测节点

**源码**：`vision/yolo_node.py`

| 订阅 | 发布 |
|------|------|
| `~/image_raw`（Image） | `~/detections`（Detection2DArray） |
| | `~/detection_image`（Image，可视化） |

参数：`model_path` / `model_type`（onnx/rdk_bpu/opencv）/ `input_size` / `confidence_threshold` / `nms_threshold`

支持推理后端：ONNX Runtime、RDK BPU（地平线 NPU）、OpenCV DNN

---

#### `deepsort_node`（`DeepSORTNode`）— DeepSORT 跟踪节点（简化版）

**源码**：`vision/deepsort_node.py`

| 订阅 | 发布 |
|------|------|
| `~/detections` | `~/tracks`（Detection2DArray） |
| `~/image_raw`（可选） | `~/tracks_visualization`（可选） |

参数：`model_name` / `max_cosine_distance` / `max_age` / `n_init`

---

#### `deepsort_tracker_node`（`DeepSORTTrackerNode`）— 完整跟踪节点

**源码**：`vision/deepsort_tracker_node.py`

订阅图像 + 检测结果，发布带跟踪 ID 的彩色可视化图像。

### 5.2 算法模块

| 模块 | 功能 |
|------|------|
| `deepsort.py` | DeepSORT 主类 |
| `tracker.py` | 跟踪器核心（级联匹配 + IoU 匹配） |
| `track.py` | 单条轨迹管理（Tentative → Confirmed → Deleted） |
| `kalman_filter.py` | 卡尔曼滤波器（匀速运动模型） |
| `matching.py` | 匈牙利匹配、级联匹配、IoU 匹配 |
| `feature_extractor.py` | 外观特征提取（颜色直方图/OSNet/ResNet） |
| `yolo_detector.py` | YOLOv5 检测器 |

### 5.3 典型流水线

```
camera/image_raw ──→ yolo_node ──→ /detections ──→ deepsort_tracker_node ──→ /tracks
     │                                                              │
     └──────────────────────────────────────────────────────────────┘
                                                              /tracks_image
```

---

## 6. mission — 任务规划与交互控制包

**类型**：Python ament_python 包  
**依赖**：uav_interfaces

### 6.1 节点

#### `mission_console`（`MissionConsoleLauncher`）— 交互式任务控制台

**源码**：`mission/mission_console_launcher.py`

提供命令行交互界面：

| 命令 | 语法 | 说明 |
|------|------|------|
| 起飞 | `takeoff <相对高度>` | 调用 uav/takeoff |
| 移动 | `move <x> <y> <z> [yaw]` | 绝对坐标移动 |
| 方向移动 | `dir <方向> <距离>` | forward/backward/left/right/up/down |
| 降落 | `land` | 调用 uav/land |
| 返航 | `rtl` | 调用 uav/rtl |

### 6.2 架构分层

```
MissionConsoleLauncher（主节点 + CLI 循环）
  ├── UavStateCache（状态缓存，订阅 MAVROS）
  ├── Action 层
  │     ├── TakeoffClientAction → uav/takeoff
  │     ├── MoveClientAction → uav/move
  │     ├── LandClientAction → uav/land
  │     ├── RtlClientAction → uav/rtl
  │     └── DirectionalMoveAction → 方向转换 + uav/move
  └── MissionConsoleRouter（命令解析 + 分发）
```

---

## 7. fsm — 飞行任务测试包

**类型**：Python ament_python 包  
**依赖**：uav_interfaces、uav_mavros2

### 7.1 脚本清单

| 可执行名 | 源码 | 功能 |
|---------|------|------|
| `test_takeoff` | `test_takeoff.py` | 起飞测试 |
| `test_takeoff_ctrl` | `test_takeoff_ctrl.py` | 起飞控制测试 |
| `land` | `land.py` | 降落测试 |
| `move` | `move.py` | 移动测试（调用 uav/move 服务） |
| `test_move_ctrl` | `test_move_ctrl.py` | 移动控制测试 |
| `test_hover_ctrl` | `test_hover_ctrl.py` | 悬停控制测试 |
| `test_land_ctrl` | `test_land_ctrl.py` | 降落控制测试 |
| `test_rtl_ctrl` | `test_rtl_ctrl.py` | 返航控制测试 |
| `test_full_mission` | `test_full_mission.py` | 完整飞行任务 |
| `fsm_node` | `uav_fsm_node:main` | FSM 状态机节点 |

### 7.2 完整任务（test_full_mission）

1. 订阅遥测获取起始位姿
2. 起飞到 2m 相对高度
3. 前进 5m
4. 爬升到 4m
5. 悬停 7 秒
6. 降落

---

## 8. lslidar_driver — 雷神激光雷达驱动包

**类型**：C++ ament_cmake 包  
**版本**：V5.1.1  
**维护者**：lslidar（雷神智能）

### 8.1 支持的雷达型号

| 类别 | 型号 |
|------|------|
| 单线雷达 | M10, M10GPS, M10P, N10, N10Plus, N301 |
| 机械式雷达 | C16, C32, C1, C1P, C4, C8, CKM8, MSC16 等 |
| 905 混合固态 | CX1S3, CX6S3, CH16X1, CH32A, CH64W, CX126S3 等 |
| 1550 混合固态 | LS25D, LS128S1-S3, LS180S1-S3, LS400S1-S3, MS06 等 |

### 8.2 核心源码

| 文件 | 功能 |
|------|------|
| `lslidar_driver_node.cpp` | 主入口，根据雷达类型创建驱动 |
| `lslidar_cx_driver.cpp` | CX 系列驱动 |
| `lslidar_ch_driver.cpp` | CH 系列驱动 |
| `lslidar_ls_driver.cpp` | LS 系列驱动 |
| `lslidar_x10_driver.cpp` | X10 系列驱动 |
| `lslidar_services.cpp` | 雷达配置服务 |

### 8.3 发布话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `lslidar_point_cloud` | PointCloud2 | 3D 点云 |
| `lslidar_packet` | LslidarPacket | 原始数据包 |
| `lslidar_info` | LslidarInformation | 设备信息 |

---

## 9. 系统架构与节点关系图

```
┌──────────────┐     MAVROS      ┌──────────────────────────────────┐
│  PX4 飞控    │ ◄─────────────► │  uav_mavros2                     │
│  (硬件)      │  MAVLink        │  ├─ telemetry_node               │
└──────────────┘                  │  ├─ ctrl_server_node             │
                                  │  ├─ uav_ctrl_node                │
                                  │  └─ print_node                   │
                                  └──────────┬───────────────────────┘
                                             │
                                  uav_interfaces (消息/服务)
                                             │
          ┌──────────────────────────────────┼───────────────────┐
          │  mission 任务层                  │                   │
          │  mission_console ────────────────┘                   │
          │  (takeoff/move/land/rtl/dir)                         │
          └──────────────────────────────────────────────────────┘

          ┌──────────────────────────────────────────────────────┐
          │  vision 视觉层                                       │
          │  camera ──image_raw──► yolo_node ──detections──►     │
          │                          deepsort_node ──► tracks    │
          └──────────────────────────────────────────────────────┘

          ┌──────────────────────────────────────────────────────┐
          │  lslidar_driver ──► PointCloud2 (点云)               │
          └──────────────────────────────────────────────────────┘

          ┌──────────────────────────────────────────────────────┐
          │  fsm ──► 飞行任务测试脚本                            │
          └──────────────────────────────────────────────────────┘
```

### 数据流摘要

1. **遥测链路**：PX4 → MAVROS → telemetry_node → /uav/telemetry/state,pose → 各节点
2. **控制链路**：mission/fsm → /uav/takeoff,move,land,rtl → ctrl_server_node → MAVROS → PX4
3. **视觉链路**：camera → /camera/image_raw → yolo_node → /detections → deepsort_node → /tracks
4. **雷达链路**：lslidar_driver → /lslidar_point_cloud → 避障/建图模块

---

## 10. 启动顺序建议

### 基础飞行

```bash
ros2 launch mavros px4.launch                    # 1. MAVROS 连接飞控
ros2 run uav_mavros2 telemetry_node              # 2. 遥测转发
ros2 run uav_mavros2 ctrl_server_node            # 3. 控制服务
ros2 run mission mission_console                  # 4. 交互控制台
```

### 视觉跟踪

```bash
ros2 run camera camera_node -p camera_type:=dahua   # 1. 相机
ros2 run vision yolo_node -p model_path:=model.onnx  # 2. 检测
ros2 run vision deepsort_tracker_node                 # 3. 跟踪
```

---

## 附录：话题一览表

| 话题 | 消息类型 | 发布者 | 订阅者 |
|------|---------|--------|--------|
| `/mavros/state` | mavros_msgs/State | MAVROS | telemetry, uavbase |
| `/mavros/local_position/pose` | PoseStamped | MAVROS | telemetry, uavbase |
| `/uav/telemetry/state` | UavState | telemetry_node | mission, fsm |
| `/uav/telemetry/pose` | UavPose | telemetry_node | mission, fsm |
| `/uav/takeoff` | Takeoff (srv) | — | ctrl_server_node |
| `/uav/move` | Move (srv) | — | ctrl_server_node |
| `/uav/land` | Land (srv) | — | ctrl_server_node |
| `/uav/rtl` | Rtl (srv) | — | ctrl_server_node |
| `/camera/image_raw` | Image | camera_node | yolo_node |
| `/detections` | Detection2DArray | yolo_node | deepsort_node |
| `/tracks` | Detection2DArray | deepsort_node | 下游 |
| `/lslidar_point_cloud` | PointCloud2 | lslidar_driver | 避障/建图 |
