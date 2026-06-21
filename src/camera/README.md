# 大华工业相机 ROS2 驱动包

## 简介

这是一个支持大华工业相机的 ROS2 驱动包，可以采集图像并发布到 ROS2 话题，与 DeepSORT 跟踪系统完美兼容。

## 特性

- **大华MVSDK支持**: 支持大华原生SDK接口
- **OpenCV兼容**: 支持OpenCV兼容的USB相机
- **测试模式**: 内置测试图案生成器
- **DeepSORT兼容**: 输出格式与DeepSORT节点完美匹配
- **动态参数**: 支持运行时调整相机参数

## 目录结构

```
camera/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── config/
│   └── dahua_camera.yaml
├── launch/
│   ├── dahua_camera.launch.py
│   └── camera_tracking_system.launch.py
├── camera/
│   ├── __init__.py
│   ├── camera_base.py         # 相机基类
│   ├── dahua_camera.py        # 大华相机驱动
│   ├── camera_node.py         # 通用相机节点
│   ├── dahua_camera_node.py   # 大华相机节点
│   └── camera_test_node.py    # 测试节点
└── test/
    └── __init__.py
```

## 依赖

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- NumPy
- OpenCV (cv2)
- 大华MVSDK (可选，用于真实大华相机)

## 安装

```bash
# 进入工作空间
cd ~/px4/ros2_px4

# 安装依赖
pip3 install numpy opencv-python

# 构建包
colcon build --packages-select camera

# Source 环境
source install/setup.bash
```

## 使用方法

### 1. 使用测试节点

```bash
# 启动测试节点（生成模拟图像）
ros2 run camera camera_test_node --ros-args \
  -p mode:=pattern \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p fps:=30.0
```

### 2. 使用大华相机

```bash
# 启动大华相机节点
ros2 run camera camera_node --ros-args \
  -p camera_type:=dahua \
  -p camera_id:=0 \
  -p image_width:=1920 \
  -p image_height:=1080
```

### 3. 使用Launch文件

```bash
# 只启动相机
ros2 launch camera dahua_camera.launch.py

# 启动相机+DeepSORT跟踪系统
ros2 launch camera camera_tracking_system.launch.py use_test_camera:=true
```

### 4. 话题

**发布的话题:**
| 话题 | 类型 | 描述 |
|------|------|------|
| `~/image_raw` | `sensor_msgs/Image` | 原始图像 |
| `~/camera_info` | `sensor_msgs/CameraInfo` | 相机信息 |

**与DeepSORT兼容的话题映射:**
```
/camera/image_raw → DeepSORT ~/image_raw
```

### 5. 参数

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `camera_type` | string | dahua | 相机类型 |
| `camera_id` | string | 0 | 相机ID |
| `image_width` | int | 1920 | 图像宽度 |
| `image_height` | int | 1080 | 图像高度 |
| `fps` | float | 30.0 | 帧率 |
| `exposure` | float | 10000.0 | 曝光时间(us) |
| `gain` | float | 0.0 | 增益(dB) |
| `trigger_mode` | bool | false | 触发模式 |
| `frame_id` | string | camera | 坐标系ID |

## Python API

```python
from camera.dahua_camera import DahuaCamera

# 创建相机实例
camera = DahuaCamera(
    camera_id="0",
    width=1920,
    height=1080,
    fps=30.0,
)

# 连接相机
if camera.connect():
    # 开始采集
    camera.start_streaming(callback=lambda frame: print(f"Got frame: {frame.shape}"))
    
    # 获取帧
    frame = camera.grab_frame()
    
    # 停止采集
    camera.stop_streaming()
    
    # 断开连接
    camera.disconnect()
```

## DeepSORT集成

### 话题连接

```bash
# 查看话题列表
ros2 topic list | grep -E "camera|image"

# 监听图像话题
ros2 topic echo /camera/image_raw --no-arr

# 查看话题信息
ros2 topic info /camera/image_raw
```

### 系统架构

```
┌─────────────────┐     /camera/image_raw      ┌──────────────────┐
│  Camera Node    │ ──────────────────────────► │ DeepSORT Node    │
│  (大华/测试)    │                             │ (跟踪算法)       │
└─────────────────┘                             └──────────────────┘
                                                        │
                                                        ▼
                                                 /tracks (跟踪结果)
```

## 故障排除

### 1. 相机连接失败

```bash
# 检查相机连接
lsusb | grep -i dahua

# 检查权限
sudo chmod 666 /dev/video*
```

### 2. 图像话题无数据

```bash
# 检查节点状态
ros2 node list
ros2 node info /dahua_camera

# 检查话题
ros2 topic hz /camera/image_raw
```

### 3. DeepSORT无输出

确保图像话题名称匹配：
```bash
# DeepSORT订阅的话题
ros2 topic info /camera/image_raw
```

## 许可证

MIT License

## 作者

MiMo Agent
