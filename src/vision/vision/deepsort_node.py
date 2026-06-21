"""
DeepSORT ROS2 节点
订阅检测结果，发布跟踪结果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header

import numpy as np
from typing import Optional, List, Dict, Any

from vision.deepsort import DeepSORT


class DeepSORTNode(Node):
    """
    DeepSORT ROS2 跟踪节点
    
    订阅:
        - /detections (vision_msgs/Detection2DArray): 2D 检测结果
        - /image_raw (sensor_msgs/Image): 原始图像（可选，用于外观特征提取）
    
    发布:
        - /tracks (vision_msgs/Detection2DArray): 跟踪结果
        - /tracks_visualization (sensor_msgs/Image): 可视化结果（可选）
    """
    
    def __init__(self):
        super().__init__('deepsort_tracker')
        
        # 声明参数
        self.declare_parameter('model_name', 'simple')
        self.declare_parameter('reid_model', '')  # ReID 模型路径（ONNX 或 BPU）
        self.declare_parameter('backend', 'auto')  # 推理后端：auto / onnx / bpu / simple
        self.declare_parameter('max_cosine_distance', 0.2)
        self.declare_parameter('nn_budget', 100)
        self.declare_parameter('max_iou_distance', 0.7)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('n_init', 3)
        self.declare_parameter('use_visualization', False)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 获取参数
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        reid_model = self.get_parameter('reid_model').get_parameter_value().string_value
        backend = self.get_parameter('backend').get_parameter_value().string_value
        max_cosine_distance = self.get_parameter('max_cosine_distance').get_parameter_value().double_value
        nn_budget = self.get_parameter('nn_budget').get_parameter_value().integer_value
        max_iou_distance = self.get_parameter('max_iou_distance').get_parameter_value().double_value
        max_age = self.get_parameter('max_age').get_parameter_value().integer_value
        n_init = self.get_parameter('n_init').get_parameter_value().integer_value
        self.use_visualization = self.get_parameter('use_visualization').get_parameter_value().bool_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
        # 初始化 DeepSORT
        self.deepsort = DeepSORT(
            model_name=model_name,
            reid_model_path=reid_model if reid_model else None,
            max_cosine_distance=max_cosine_distance,
            nn_budget=nn_budget,
            max_iou_distance=max_iou_distance,
            max_age=max_age,
            n_init=n_init,
            backend=backend,
        )
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # 订阅者
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            qos_profile,
        )
        
        self.image_sub = None
        self.current_image = None
        
        if self.use_visualization:
            self.image_sub = self.create_subscription(
                Image,
                '/image_raw',
                self.image_callback,
                qos_profile,
            )
        
        # 发布者
        self.track_pub = self.create_publisher(
            Detection2DArray,
            '/tracks',
            10,
        )
        
        self.get_logger().info('DeepSORT Tracker Node initialized')
        self.get_logger().info(f'  Model: {model_name}')
        self.get_logger().info(f'  Max cosine distance: {max_cosine_distance}')
        self.get_logger().info(f'  Max IoU distance: {max_iou_distance}')
        self.get_logger().info(f'  Max age: {max_age}')
        self.get_logger().info(f'  N init: {n_init}')
    
    def image_callback(self, msg: Image):
        """
        图像回调
        
        Args:
            msg: 图像消息
        """
        try:
            # 转换 ROS Image 到 numpy
            from cv_bridge import CvBridge
            bridge = CvBridge()
            self.current_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except ImportError:
            # 无 cv_bridge 时使用简单转换
            if msg.encoding == 'bgr8':
                self.current_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
            elif msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                self.current_image = img[:, :, ::-1]
    
    def detection_callback(self, msg: Detection2DArray):
        """
        检测结果回调
        
        Args:
            msg: 检测结果消息
        """
        # 解析检测结果
        bboxes = []
        scores = []
        classes = []
        
        for det in msg.detections:
            # 过滤低置信度检测
            if len(det.results) > 0:
                hypothesis = det.results[0]
                score = hypothesis.score
                
                if score < self.confidence_threshold:
                    continue
                
                # 提取边界框 (假设 bbox 中心 + 大小)
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w = det.bbox.size_x
                h = det.bbox.size_y
                
                # 转换为 [x1, y1, x2, y2]
                x1 = cx - w / 2
                y1 = cy - h / 2
                x2 = cx + w / 2
                y2 = cy + h / 2
                
                bboxes.append([x1, y1, x2, y2])
                scores.append(score)
                classes.append(hypothesis.hypothesis.class_id)
        
        # 转换为 numpy 数组
        if len(bboxes) > 0:
            bboxes = np.array(bboxes, dtype=np.float32)
            scores = np.array(scores, dtype=np.float32)
            classes = np.array(classes, dtype=int)
        else:
            bboxes = np.array([]).reshape(0, 4)
            scores = np.array([])
            classes = np.array([])
        
        # 更新跟踪器
        results = self.deepsort.update(
            image=self.current_image,
            bboxes=bboxes,
            scores=scores,
            classes=classes,
        )
        
        # 发布跟踪结果
        self.publish_tracks(results, msg.header)
    
    def publish_tracks(self, results: List[Dict[str, Any]], header: Header):
        """
        发布跟踪结果
        
        Args:
            results: 跟踪结果列表
            header: 消息头
        """
        track_msg = Detection2DArray()
        track_msg.header = header
        track_msg.header.frame_id = 'tracks'
        
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
            
            # 添加自定义 ID（使用 track_id）
            # 注意：标准 Detection2D 没有 track_id 字段
            # 可以使用 id 字段（如果存在）
            
            track_msg.detections.append(det)
        
        self.track_pub.publish(track_msg)
        
        # 日志输出
        if len(results) > 0:
            self.get_logger().info(
                f'Tracked {len(results)} objects: '
                f'IDs={[r["track_id"] for r in results]}',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DeepSORTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
