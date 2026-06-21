# DeepSORT 多目标跟踪 ROS2 包

## 简介

这是一个基于 DeepSORT 算法的多目标跟踪 ROS2 包。DeepSORT 结合了深度外观特征和运动信息，能够准确地跟踪多个目标。

## 特性

- **DeepSORT 算法**: 完整实现 DeepSORT 多目标跟踪
- **卡尔曼滤波**: 用于状态预测和更新
- **匈牙利匹配**: 用于数据关联
- **外观特征**: 支持简单的颜色直方图或深度学习特征（OSNet, ResNet）
- **级联匹配**: 优先匹配最近更新的跟踪，解决遮挡问题
- **ROS2 集成**: 完整的 ROS2 节点接口

## 目录结构

```
vision/
├── package.xml
├── setup.py
├── setup.cfg
├── launch/
│   └── deepsort_tracker.launch.py
├── vision/
│   ├── __init__.py
│   ├── deepsort.py           # DeepSORT 主类
│   ├── tracker.py            # 跟踪器
│   ├── track.py              # 轨迹管理
│   ├── kalman_filter.py      # 卡尔曼滤波器
│   ├── matching.py           # 匹配算法
│   ├── feature_extractor.py  # 特征提取器
│   ├── deepsort_node.py      # ROS2 节点
│   └── deepsort_tracker_node.py  # 完整 ROS2 节点
└── test/
    └── test_deepsort.py      # 测试脚本
```

## 依赖

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- NumPy
- SciPy
- OpenCV (可选，用于可视化)

## 安装

```bash
# 进入工作空间
cd ~/px4/ros2_px4

# 安装 Python 依赖
pip3 install numpy scipy

# 构建包
colcon build --packages-select vision

# Source 环境
source install/setup.bash
```

## 使用方法

### 1. 命令行启动

```bash
# 启动跟踪节点
ros2 run vision deepsort_tracker_node --ros-args \
  -p model_name:=simple \
  -p max_cosine_distance:=0.2 \
  -p max_age:=30 \
  -p n_init:=3
```

### 2. Launch 文件启动

```bash
ros2 launch vision deepsort_tracker.launch.py
```

### 3. 订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `~/image_raw` | `sensor_msgs/Image` | 输入图像（可选，用于外观特征提取） |
| `~/detections` | `vision_msgs/Detection2DArray` | 2D 检测结果 |

### 4. 发布的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `~/tracks` | `vision_msgs/Detection2DArray` | 跟踪结果 |
| `~/tracks_image` | `sensor_msgs/Image` | 可视化图像 |

### 5. 参数

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `model_name` | string | `simple` | 特征提取模型 (simple, osnet, resnet) |
| `feature_dim` | int | 512 | 特征维度 |
| `max_cosine_distance` | float | 0.2 | 外观特征最大余弦距离阈值 |
| `nn_budget` | int | 100 | 每个跟踪保存的最大特征数量 |
| `max_iou_distance` | float | 0.7 | IoU 距离阈值 |
| `max_age` | int | 30 | 最大丢失帧数 |
| `n_init` | int | 3 | 确认跟踪所需的连续匹配次数 |
| `confidence_threshold` | float | 0.5 | 检测置信度阈值 |
| `show_visualization` | bool | true | 是否发布可视化图像 |

## Python API

```python
from vision.deepsort import DeepSORT

# 创建跟踪器
tracker = DeepSORT(
    model_name="simple",
    max_cosine_distance=0.2,
    max_age=30,
    n_init=3,
)

# 处理每一帧
results = tracker.update(
    image=image,           # 输入图像 (可选)
    bboxes=bboxes,         # 边界框 (N, 4) [x1, y1, x2, y2]
    scores=scores,         # 置信度 (N,)
    classes=classes,       # 类别 (N,)
)

# 获取跟踪结果
for result in results:
    print(f"Track {result['track_id']}: {result['bbox']}")
```

## 测试

```bash
# 运行单元测试
cd ~/px4/ros2_px4/src/vision
python3 test/test_deepsort.py
```

## 算法说明

### DeepSORT 算法流程

1. **检测**: 接收目标检测结果（边界框、置信度、类别）
2. **特征提取**: 从检测区域提取外观特征
3. **卡尔曼预测**: 预测每个跟踪的下一帧状态
4. **级联匹配**:
   - 使用外观特征的余弦距离进行级联匹配
   - 优先匹配最近更新的跟踪
5. **IoU 匹配**: 对未匹配的跟踪使用 IoU 距离匹配
6. **状态更新**: 更新匹配的跟踪，初始化新跟踪，删除丢失的跟踪

### 状态管理

- **TENTATIVE (临时)**: 刚初始化的跟踪，需要连续匹配才能确认
- **CONFIRMED (确认)**: 持续跟踪中的目标
- **DELETED (删除)**: 超过最大丢失帧数的跟踪

## 许可证

MIT License

## 作者

MiMo Agent
