"""
DeepSORT 跟踪器节点 - 完整实现
支持订阅图像和检测结果，发布带跟踪ID的可视化结果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Header, ColorRGBA

import numpy as np
import sys
from typing import Optional, List, Dict, Any, Tuple

from vision.deepsort import DeepSORT


class DeepSORTTrackerNode(Node):
    """
    DeepSORT 跟踪器 ROS2 节点
    
    功能:
        1. 订阅图像和 2D 检测结果
        2. 使用 DeepSORT 算法进行多目标跟踪
        3. 发布跟踪结果和可视化图像
    
    订阅:
        - ~/image_raw (sensor_msgs/Image): 输入图像
        - ~/detections (vision_msgs/Detection2DArray): 检测结果
    
    发布:
        - ~/tracks (vision_msgs/Detection2DArray): 跟踪结果
        - ~/tracks_image (sensor_msgs/Image): 可视化图像
    """
    
    # 颜色映射（用于可视化）
    COLORS = [
        (255, 0, 0),     # 红
        (0, 255, 0),     # 绿
        (0, 0, 255),     # 蓝
        (255, 255, 0),   # 黄
        (255, 0, 255),   # 洋红
        (0, 255, 255),   # 青
        (128, 0, 0),     # 深红
        (0, 128, 0),     # 深绿
        (0, 0, 128),     # 深蓝
        (128, 128, 0),   # 橄榄
        (128, 0, 128),   # 紫
        (0, 128, 128),   # 蓝绿
    ]
    
    def __init__(self):
        super().__init__('deepsort_tracker_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self._get_parameters()
        
        # 初始化 DeepSORT
        self.deepsort = DeepSORT(
            model_name=self.model_name,
            reid_model_path=self.reid_model if self.reid_model else None,
            feature_dim=self.feature_dim,
            max_cosine_distance=self.max_cosine_distance,
            nn_budget=self.nn_budget,
            max_iou_distance=self.max_iou_distance,
            max_age=self.max_age,
            n_init=self.n_init,
            device=self.device,
            backend=self.backend,
        )
        
        # 图像缓存
        self.current_image: Optional[np.ndarray] = None
        self.image_header: Optional[Header] = None
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        
        # 订阅者
        self.image_sub = self.create_subscription(
            Image,
            '~/image_raw',
            self._image_callback,
            sensor_qos,
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '~/detections',
            self._detection_callback,
            sensor_qos,
        )
        
        # 发布者
        self.track_pub = self.create_publisher(
            Detection2DArray,
            '~/tracks',
            10,
        )
        
        self.track_image_pub = self.create_publisher(
            Image,
            '~/tracks_image',
            5,
        )
        
        # 统计
        self.frame_count = 0
        self.total_tracks = 0
        
        self.get_logger().info('='*50)
        self.get_logger().info('DeepSORT Tracker Node Started')
        self.get_logger().info('='*50)
        self._log_parameters()
    
    def _declare_parameters(self):
        """声明参数"""
        # 特征提取参数
        self.declare_parameter('model_name', 'simple')
        self.declare_parameter('reid_model', '')  # ReID 模型路径（ONNX 或 BPU）
        self.declare_parameter('backend', 'auto')  # 推理后端：auto / onnx / bpu / simple
        self.declare_parameter('feature_dim', 512)
        self.declare_parameter('device', 'cpu')
        
        # 匹配参数
        self.declare_parameter('max_cosine_distance', 0.2)
        self.declare_parameter('nn_budget', 100)
        self.declare_parameter('max_iou_distance', 0.7)
        
        # 跟踪参数
        self.declare_parameter('max_age', 30)
        self.declare_parameter('n_init', 3)
        
        # 过滤参数
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 可视化参数
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('show_trajectory', True)
        self.declare_parameter('trajectory_length', 30)
    
    def _get_parameters(self):
        """获取参数"""
        # 特征提取参数
        self.model_name = self.get_parameter('model_name').value
        self.reid_model = self.get_parameter('reid_model').value
        self.backend = self.get_parameter('backend').value
        self.feature_dim = self.get_parameter('feature_dim').value
        self.device = self.get_parameter('device').value
        
        # 匹配参数
        self.max_cosine_distance = self.get_parameter('max_cosine_distance').value
        self.nn_budget = self.get_parameter('nn_budget').value
        self.max_iou_distance = self.get_parameter('max_iou_distance').value
        
        # 跟踪参数
        self.max_age = self.get_parameter('max_age').value
        self.n_init = self.get_parameter('n_init').value
        
        # 过滤参数
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # 可视化参数
        self.show_visualization = self.get_parameter('show_visualization').value
        self.show_trajectory = self.get_parameter('show_trajectory').value
        self.trajectory_length = self.get_parameter('trajectory_length').value
        
        # 轨迹历史
        self.track_trajectories: Dict[int, List[Tuple[float, float]]] = {}
    
    def _log_parameters(self):
        """打印参数"""
        self.get_logger().info(f'  Model: {self.model_name}')
        self.get_logger().info(f'  ReID Model: {self.reid_model if self.reid_model else "None (use model_name)"}')
        self.get_logger().info(f'  Backend: {self.backend}')
        self.get_logger().info(f'  Device: {self.device}')
        self.get_logger().info(f'  Max cosine distance: {self.max_cosine_distance}')
        self.get_logger().info(f'  Max IoU distance: {self.max_iou_distance}')
        self.get_logger().info(f'  Max age: {self.max_age}')
        self.get_logger().info(f'  N init: {self.n_init}')
        self.get_logger().info(f'  Confidence threshold: {self.confidence_threshold}')
    
    def _image_callback(self, msg: Image):
        """
        图像回调
        
        Args:
            msg: 图像消息
        """
        self.image_header = msg.header
        
        try:
            # 尝试使用 cv_bridge
            from cv_bridge import CvBridge
            bridge = CvBridge()
            self.current_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except ImportError:
            # 手动转换
            self.current_image = self._ros_image_to_cv2(msg)
    
    def _ros_image_to_cv2(self, msg: Image) -> np.ndarray:
        """
        手动将 ROS Image 转换为 OpenCV 格式
        
        Args:
            msg: 图像消息
            
        Returns:
            image: OpenCV 图像
        """
        h, w = msg.height, msg.width
        encoding = msg.encoding
        
        if encoding in ['bgr8', 'rgb8']:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if encoding == 'rgb8':
                img = img[:, :, ::-1]
        elif encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            img = np.stack([img, img, img], axis=-1)
        elif encoding == 'bgra8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
            img = img[:, :, :3]
        else:
            # 默认尝试 BGR8
            try:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            except ValueError:
                img = np.zeros((h, w, 3), dtype=np.uint8)
        
        return img
    
    def _detection_callback(self, msg: Detection2DArray):
        """
        检测结果回调
        
        Args:
            msg: 检测结果消息
        """
        self.frame_count += 1
        
        # 解析检测结果
        bboxes, scores, classes = self._parse_detections(msg)
        
        # 更新跟踪器
        results = self.deepsort.update(
            image=self.current_image,
            bboxes=bboxes,
            scores=scores,
            classes=classes,
        )
        
        # 更新轨迹历史
        self._update_trajectories(results)
        
        # 发布跟踪结果
        self._publish_tracks(results, msg.header)
        
        # 发布可视化图像
        if self.show_visualization and self.current_image is not None:
            self._publish_visualization(results, msg.header)
        
        # 统计信息
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'{len(results)} tracks, '
                f'total unique tracks: {self.total_tracks}'
            )
    
    def _parse_detections(self, msg: Detection2DArray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        解析检测消息
        
        Args:
            msg: 检测消息
            
        Returns:
            bboxes: 边界框 (N, 4) [x1, y1, x2, y2]
            scores: 置信度 (N,)
            classes: 类别 (N,)
        """
        bboxes = []
        scores = []
        classes = []
        
        for det in msg.detections:
            if len(det.results) == 0:
                continue
            
            hypothesis = det.results[0]
            score = hypothesis.score
            
            if score < self.confidence_threshold:
                continue
            
            # 提取边界框
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            
            # 计算角点
            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2
            
            bboxes.append([x1, y1, x2, y2])
            scores.append(score)
            classes.append(hypothesis.hypothesis.class_id)
        
        # 转换为 numpy
        if len(bboxes) > 0:
            return (
                np.array(bboxes, dtype=np.float32),
                np.array(scores, dtype=np.float32),
                np.array(classes, dtype=int),
            )
        else:
            return (
                np.array([]).reshape(0, 4),
                np.array([]),
                np.array([]),
            )
    
    def _update_trajectories(self, results: List[Dict[str, Any]]):
        """
        更新轨迹历史
        
        Args:
            results: 跟踪结果
        """
        current_ids = set()
        
        for result in results:
            track_id = result['track_id']
            current_ids.add(track_id)
            
            # 获取中心点
            bbox = result['bbox']
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2
            
            # 更新轨迹
            if track_id not in self.track_trajectories:
                self.track_trajectories[track_id] = []
                self.total_tracks += 1
            
            self.track_trajectories[track_id].append((cx, cy))
            
            # 限制轨迹长度
            if len(self.track_trajectories[track_id]) > self.trajectory_length:
                self.track_trajectories[track_id].pop(0)
        
        # 清理已消失的轨迹
        disappeared_ids = set(self.track_trajectories.keys()) - current_ids
        for track_id in disappeared_ids:
            if len(self.track_trajectories[track_id]) == 0:
                del self.track_trajectories[track_id]
    
    def _publish_tracks(self, results: List[Dict[str, Any]], header: Header):
        """
        发布跟踪结果
        
        Args:
            results: 跟踪结果
            header: 消息头
        """
        track_msg = Detection2DArray()
        track_msg.header = header
        
        for result in results:
            det = Detection2D()
            
            # 边界框
            bbox = result['bbox']
            x1, y1, x2, y2 = bbox
            
            det.bbox.center.position.x = (x1 + x2) / 2
            det.bbox.center.position.y = (y1 + y2) / 2
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            
            # 假设结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = result['class']
            hypothesis.score = result['score']
            
            det.results = [hypothesis]
            
            track_msg.detections.append(det)
        
        self.track_pub.publish(track_msg)
    
    def _publish_visualization(self, results: List[Dict[str, Any]], header: Header):
        """
        发布可视化图像
        
        Args:
            results: 跟踪结果
            header: 消息头
        """
        if self.current_image is None:
            return
        
        try:
            import cv2
        except ImportError:
            return
        
        # 复制图像
        vis_image = self.current_image.copy()
        
        # 绘制跟踪结果
        for result in results:
            track_id = result['track_id']
            bbox = result['bbox']
            score = result['score']
            
            x1, y1, x2, y2 = [int(v) for v in bbox]
            
            # 获取颜色
            color = self.COLORS[track_id % len(self.COLORS)]
            
            # 绘制边界框
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制 ID 和置信度
            label = f'ID:{track_id} {score:.2f}'
            cv2.putText(vis_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制轨迹
            if self.show_trajectory and track_id in self.track_trajectories:
                trajectory = self.track_trajectories[track_id]
                for i in range(1, len(trajectory)):
                    pt1 = (int(trajectory[i-1][0]), int(trajectory[i-1][1]))
                    pt2 = (int(trajectory[i][0]), int(trajectory[i][1]))
                    cv2.line(vis_image, pt1, pt2, color, 2)
        
        # 转换为 ROS Image
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            vis_msg = bridge.cv2_to_imgmsg(vis_image, 'bgr8')
        except ImportError:
            # 手动转换
            vis_msg = Image()
            vis_msg.header = header
            vis_msg.height, vis_msg.width = vis_image.shape[:2]
            vis_msg.encoding = 'bgr8'
            vis_msg.is_bigendian = False
            vis_msg.step = vis_image.shape[1] * 3
            vis_msg.data = vis_image.tobytes()
        
        vis_msg.header = header
        self.track_image_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DeepSORTTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
