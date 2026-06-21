"""
ROS2 YOLOv5 检测节点
订阅图像话题，发布检测结果
兼容 DeepSORT 输入格式
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

import numpy as np
import time
import os
from typing import Optional, List, Dict, Any

from vision.yolo_detector import YOLODetector


class YOLONode(Node):
    """
    YOLOv5 检测 ROS2 节点
    
    功能:
        1. 订阅图像话题
        2. 使用 YOLOv5 进行目标检测
        3. 发布检测结果 (兼容 DeepSORT)
        4. 可选发布检测可视化图像
    
    订阅:
        - ~/image_raw (sensor_msgs/Image): 输入图像
    
    发布:
        - ~/detections (vision_msgs/Detection2DArray): 检测结果
        - ~/detection_image (sensor_msgs/Image): 可视化图像
    """
    
    def __init__(self):
        super().__init__('yolo_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self._get_parameters()
        
        # 初始化检测器
        self._init_detector()
        
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
        
        # 发布者
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '~/detections',
            10,
        )
        
        self.detection_image_pub = self.create_publisher(
            Image,
            '~/detection_image',
            5,
        )
        
        # 统计
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        
        self.get_logger().info('='*50)
        self.get_logger().info('YOLOv5 Detection Node Started')
        self.get_logger().info('='*50)
        self._log_parameters()
    
    def _declare_parameters(self):
        """声明参数"""
        # 模型参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('backend', 'auto')
        
        # 类别过滤
        self.declare_parameter('classes', [])  # 空列表表示所有类别
        
        # 可视化参数
        self.declare_parameter('show_detection', True)
        self.declare_parameter('show_labels', True)
        self.declare_parameter('show_scores', True)
        
        # 性能参数
        self.declare_parameter('skip_frames', 0)  # 跳帧数
    
    def _get_parameters(self):
        """获取参数"""
        # 模型参数
        self.model_path = self.get_parameter('model_path').value
        self.input_size = self.get_parameter('input_size').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.backend = self.get_parameter('backend').value
        
        # 类别过滤
        self.classes = self.get_parameter('classes').value
        if self.classes and len(self.classes) > 0:
            self.classes = [int(c) for c in self.classes]
        else:
            self.classes = None
        
        # 可视化参数
        self.show_detection = self.get_parameter('show_detection').value
        self.show_labels = self.get_parameter('show_labels').value
        self.show_scores = self.get_parameter('show_scores').value
        
        # 性能参数
        self.skip_frames = self.get_parameter('skip_frames').value
        self.skip_count = 0
    
    def _init_detector(self):
        """初始化检测器"""
        # 如果未指定模型路径，使用默认路径
        if not self.model_path:
            # 查找默认模型
            default_paths = [
                os.path.expanduser('~/px4/ros2_px4/src/vision/models/yolov5s.onnx'),
                os.path.expanduser('~/px4/ros2_px4/src/vision/models/yolov5s.bin'),
                '/opt/models/yolov5s.onnx',
            ]
            
            for path in default_paths:
                if os.path.exists(path):
                    self.model_path = path
                    break
        
        if not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(f'模型文件不存在: {self.model_path}')
            self.get_logger().error('请下载 YOLOv5 模型到 src/vision/models/ 目录')
            raise FileNotFoundError(f'模型文件不存在: {self.model_path}')
        
        # 创建检测器
        self.detector = YOLODetector(
            model_path=self.model_path,
            input_size=(self.input_size, self.input_size),
            conf_threshold=self.conf_threshold,
            iou_threshold=self.iou_threshold,
            backend=self.backend,
        )
    
    def _log_parameters(self):
        """打印参数"""
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  Input size: {self.input_size}')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  IoU threshold: {self.iou_threshold}')
        self.get_logger().info(f'  Backend: {self.backend}')
        self.get_logger().info(f'  Classes filter: {self.classes}')
    
    def _image_callback(self, msg: Image):
        """
        图像回调
        
        Args:
            msg: 图像消息
        """
        # 跳帧
        if self.skip_frames > 0:
            self.skip_count += 1
            if self.skip_count <= self.skip_frames:
                return
            self.skip_count = 0
        
        self.frame_count += 1
        
        try:
            # 转换 ROS Image 到 OpenCV
            image = self._ros_image_to_cv2(msg)
            
            if image is None:
                self.get_logger().warn('无法转换图像')
                return
            
            # 执行检测
            start_time = time.time()
            detections = self.detector.detect(image, classes=self.classes)
            inference_time = time.time() - start_time
            
            # 发布检测结果
            self._publish_detections(detections, msg.header)
            
            # 发布可视化图像
            if self.show_detection:
                vis_image = self.detector.draw_detections(
                    image,
                    detections,
                    show_labels=self.show_labels,
                    show_scores=self.show_scores,
                )
                self._publish_detection_image(vis_image, msg.header)
            
            # 统计
            self.detection_count += len(detections)
            self._update_fps(inference_time)
            
        except Exception as e:
            self.get_logger().error(f'检测失败: {e}')
    
    def _ros_image_to_cv2(self, msg: Image) -> Optional[np.ndarray]:
        """
        将 ROS Image 转换为 OpenCV 格式
        
        Args:
            msg: 图像消息
            
        Returns:
            image: OpenCV 图像
        """
        try:
            # 尝试使用 cv_bridge
            from cv_bridge import CvBridge
            bridge = CvBridge()
            return bridge.imgmsg_to_cv2(msg, 'bgr8')
        except ImportError:
            # 手动转换
            h, w = msg.height, msg.width
            encoding = msg.encoding
            
            if encoding in ['bgr8', 'rgb8']:
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                if encoding == 'rgb8':
                    image = image[:, :, ::-1]
            elif encoding == 'mono8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
                image = np.stack([image, image, image], axis=-1)
            elif encoding == 'bgra8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
                image = image[:, :, :3]
            else:
                self.get_logger().warn(f'不支持的编码格式: {encoding}')
                return None
            
            return image.copy()
    
    def _publish_detections(self, detections: List[Dict[str, Any]], header):
        """
        发布检测结果
        
        Args:
            detections: 检测结果列表
            header: 消息头
        """
        msg = Detection2DArray()
        msg.header = header
        
        for det in detections:
            detection = Detection2D()
            
            # 边界框
            bbox = det['bbox']
            x1, y1, x2, y2 = bbox
            
            # 转换为中心点 + 大小格式
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1
            
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            
            # 假设结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.score = det['score']
            
            detection.results = [hypothesis]
            
            msg.detections.append(detection)
        
        self.detection_pub.publish(msg)
    
    def _publish_detection_image(self, image: np.ndarray, header):
        """
        发布检测可视化图像
        
        Args:
            image: 图像
            header: 消息头
        """
        msg = Image()
        msg.header = header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.strides[0]
        msg.data = image.tobytes()
        
        self.detection_image_pub.publish(msg)
    
    def _update_fps(self, inference_time: float):
        """更新 FPS 统计"""
        current_time = time.time()
        if current_time - self.last_fps_time >= 2.0:
            fps = (self.frame_count - self.last_fps_count) / (current_time - self.last_fps_time)
            self.get_logger().info(
                f'FPS: {fps:.1f}, Inference: {inference_time*1000:.1f}ms, '
                f'Detections: {len(detections) if "detections" in dir() else 0}',
                throttle_duration_sec=5.0,
            )
            self.last_fps_time = current_time
            self.last_fps_count = self.frame_count


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLONode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
