"""
ROS2 YOLOv5 检测节点
订阅图像话题，发布检测结果
兼容 DeepSORT 输入格式
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from uav_interfaces.msg import Detection, DetectionArray

import contextlib
import io
import numpy as np
import time
import os
import json
import cv2
from typing import Optional, List, Dict, Any

from vision.yolo_detector import YOLODetector


def ros_image_to_cv2(msg: Image) -> Optional[np.ndarray]:
    """将 ROS Image 消息转换为 OpenCV BGR 图像"""
    h, w = msg.height, msg.width
    encoding = msg.encoding
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ('bgr8', 'rgb8'):
        img = data.reshape(h, w, 3)
        if encoding == 'rgb8':
            img = img[:, :, ::-1]
    elif encoding == 'mono8':
        gray = data.reshape(h, w)
        img = np.stack([gray, gray, gray], axis=-1)
    elif encoding in ('bgra8', 'rgba8'):
        img = data.reshape(h, w, 4)[:, :, :3]
        if encoding == 'rgba8':
            img = img[:, :, ::-1]
    elif encoding in ('bgr16', 'rgb16'):
        img = data.reshape(h, w, 3).astype(np.float32) / 65535.0 * 255.0
        img = img.astype(np.uint8)
        if encoding == 'rgb16':
            img = img[:, :, ::-1]
    elif encoding == 'mono16':
        gray = data.reshape(h, w).astype(np.float32) / 65535.0 * 255.0
        gray = gray.astype(np.uint8)
        img = np.stack([gray, gray, gray], axis=-1)
    else:
        return None

    return img.copy()


def cv2_to_ros_image(image: np.ndarray, header=None) -> Image:
    """将 OpenCV BGR 图像转换为 ROS Image 消息"""
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = image.strides[0]
    msg.data = image.tobytes()
    return msg


class YOLONode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self._declare_parameters()
        self._get_parameters()
        self._init_detector()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image, '~/image_raw', self._image_callback, sensor_qos,
        )
        self.detection_pub = self.create_publisher(
            DetectionArray, '~/detections', 10,
        )
        self.detection_image_pub = self.create_publisher(
            Image, '~/detection_image', 5,
        )

        self.frame_count = 0
        self.last_detection_count = 0
        self.last_fps_time = time.time()
        self.last_fps_frame = 0

        pass

    def _declare_parameters(self):
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('classes', '')  # JSON 数组字符串，如 "[0,2,5]"，空串=所有类别
        self.declare_parameter('show_detection', True)
        self.declare_parameter('show_labels', True)
        self.declare_parameter('show_scores', True)
        self.declare_parameter('skip_frames', 0)

    def _get_parameters(self):
        self.model_path = self.get_parameter('model_path').value
        self.input_size = self.get_parameter('input_size').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.backend = self.get_parameter('backend').value

        classes_str = self.get_parameter('classes').value or ''
        try:
            parsed = json.loads(classes_str) if classes_str else []
            self.classes = [int(c) for c in parsed] if parsed else None
        except (json.JSONDecodeError, ValueError):
            self.get_logger().warn(f'无法解析 classes 参数: "{classes_str}"，将检测所有类别')
            self.classes = None

        self.show_detection = self.get_parameter('show_detection').value
        self.show_labels = self.get_parameter('show_labels').value
        self.show_scores = self.get_parameter('show_scores').value
        self.skip_frames = self.get_parameter('skip_frames').value
        self.skip_count = 0

    def _init_detector(self):
        if not self.model_path:
            default_paths = self._default_model_paths()
            for path in default_paths:
                if os.path.exists(path):
                    self.model_path = path
                    break

        if not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(f'模型文件不存在: {self.model_path}')
            raise FileNotFoundError(f'模型文件不存在: {self.model_path}')

        with contextlib.redirect_stdout(io.StringIO()):
            self.detector = YOLODetector(
                model_path=self.model_path,
                input_size=(self.input_size, self.input_size),
                conf_threshold=self.conf_threshold,
                iou_threshold=self.iou_threshold,
                backend=self.backend,
            )

    def _default_model_paths(self):
        repo_model_dirs = [
            os.path.expanduser('~/Desktop/px4/ros2_px4/src/vision/models'),
            os.path.expanduser('~/px4/ros2_px4/src/vision/models'),
            os.path.join(os.getcwd(), 'src/vision/models'),
        ]
        rdk_paths = [
            *(os.path.join(model_dir, 'yolov5s_672x672_nv12.bin') for model_dir in repo_model_dirs),
            *(os.path.join(model_dir, 'yolov5s.bin') for model_dir in repo_model_dirs),
            '/opt/hobot/model/x5/basic/yolov5s_672x672_nv12.bin',
        ]
        onnx_paths = [
            *(os.path.join(model_dir, 'yolov5s.onnx') for model_dir in repo_model_dirs),
            '/opt/models/yolov5s.onnx',
        ]

        try:
            import hobot_dnn  # noqa: F401
            return rdk_paths + onnx_paths
        except ImportError:
            return onnx_paths + rdk_paths

    def _log_parameters(self):
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  Input size: {self.input_size}')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  IoU threshold: {self.iou_threshold}')
        self.get_logger().info(f'  Backend: {self.backend}')
        if hasattr(self, 'detector'):
            self.get_logger().info(f'  Active backend: {self.detector.backend}')
        self.get_logger().info(f'  Classes filter: {self.classes or "all"}')

    def _image_callback(self, msg: Image):
        if self.skip_frames > 0:
            self.skip_count += 1
            if self.skip_count <= self.skip_frames:
                return
            self.skip_count = 0

        self.frame_count += 1

        try:
            image = ros_image_to_cv2(msg)
            if image is None:
                self.get_logger().warn(f'不支持的图像编码: {msg.encoding}')
                return

            start_time = time.time()
            detections = self.detector.detect(image, classes=self.classes)
            inference_time = time.time() - start_time

            self._publish_detections(detections, msg.header)

            if self.show_detection and self.detection_image_pub.get_subscription_count() > 0:
                vis_image = self.detector.draw_detections(
                    image, detections,
                    show_labels=self.show_labels,
                    show_scores=self.show_scores,
                )
                self.detection_image_pub.publish(cv2_to_ros_image(vis_image, msg.header))

            self._update_fps(inference_time, len(detections))

        except Exception as e:
            self.get_logger().error(f'检测失败: {e}')

    def _publish_detections(self, detections: List[Dict[str, Any]], header):
        msg = DetectionArray()
        msg.header = header

        for det in detections:
            detection = Detection()
            x1, y1, x2, y2 = det['bbox']

            detection.class_id = str(det['class_id'])
            detection.score = float(det['score'])
            detection.x_min = float(x1)
            detection.y_min = float(y1)
            detection.x_max = float(x2)
            detection.y_max = float(y2)

            msg.detections.append(detection)

        self.detection_pub.publish(msg)

    def _update_fps(self, inference_time: float, det_count: int):
        current_time = time.time()
        dt = current_time - self.last_fps_time
        if dt >= 2.0:
            self.get_logger().info(f'检测到 {det_count} 个目标')
            self.last_fps_time = current_time
            self.last_fps_frame = self.frame_count


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YOLONode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
