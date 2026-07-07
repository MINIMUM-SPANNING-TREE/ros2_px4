"""
DeepSORT ROS2 节点
订阅检测结果，发布跟踪结果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from uav_interfaces.msg import DetectionArray, TrackDetection, TrackDetectionArray

import numpy as np
import json
import cv2
from typing import Optional, List, Dict, Any

from vision.deepsort import DeepSORT


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


class DeepSORTNode(Node):

    def __init__(self):
        super().__init__('deepsort_tracker')

        self.declare_parameter('model_name', 'simple')
        self.declare_parameter('reid_model', '')
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('max_cosine_distance', 0.2)
        self.declare_parameter('nn_budget', 100)
        self.declare_parameter('max_iou_distance', 0.7)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('n_init', 3)
        self.declare_parameter('use_visualization', False)
        self.declare_parameter('confidence_threshold', 0.5)

        model_name = self.get_parameter('model_name').value
        reid_model = self.get_parameter('reid_model').value or ''
        backend = self.get_parameter('backend').value
        max_cosine_distance = self.get_parameter('max_cosine_distance').value
        nn_budget = self.get_parameter('nn_budget').value
        max_iou_distance = self.get_parameter('max_iou_distance').value
        max_age = self.get_parameter('max_age').value
        n_init = self.get_parameter('n_init').value
        self.use_visualization = self.get_parameter('use_visualization').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.detection_sub = self.create_subscription(
            DetectionArray, '/detections', self.detection_callback, qos_profile,
        )

        self.image_sub = None
        self.current_image = None

        if self.use_visualization:
            self.image_sub = self.create_subscription(
                Image, '/image_raw', self.image_callback, qos_profile,
            )

        self.track_pub = self.create_publisher(TrackDetectionArray, '/tracks', 10)

        self.get_logger().info('DeepSORT Tracker Node initialized')
        self.get_logger().info(f'  Model: {model_name}, Backend: {backend}')
        self.get_logger().info(f'  Max cosine distance: {max_cosine_distance}')

    def image_callback(self, msg: Image):
        try:
            self.current_image = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')

    def detection_callback(self, msg: DetectionArray):
        bboxes = []
        scores = []
        classes = []

        for det in msg.detections:
            score = det.score

            if score < self.confidence_threshold:
                continue

            bboxes.append([det.x_min, det.y_min, det.x_max, det.y_max])
            scores.append(score)
            classes.append(int(det.class_id) if det.class_id else 0)

        if len(bboxes) > 0:
            bboxes = np.array(bboxes, dtype=np.float32)
            scores = np.array(scores, dtype=np.float32)
            classes = np.array(classes, dtype=int)
        else:
            bboxes = np.array([]).reshape(0, 4)
            scores = np.array([], dtype=np.float32)
            classes = np.array([], dtype=int)

        results = self.deepsort.update(
            image=self.current_image,
            bboxes=bboxes,
            scores=scores,
            classes=classes,
        )

        self.publish_tracks(results, msg.header)

    def publish_tracks(self, results: List[Dict[str, Any]], header: Header):
        track_msg = TrackDetectionArray()
        track_msg.header = header

        for result in results:
            det = TrackDetection()
            x1, y1, x2, y2 = result['bbox']

            det.track_id = int(result['track_id'])
            det.class_id = str(result.get('class', 0))
            det.score = result.get('score', 1.0)
            det.x_min = float(x1)
            det.y_min = float(y1)
            det.x_max = float(x2)
            det.y_max = float(y2)
            det.age = int(result.get('age', 0))
            det.hits = int(result.get('hits', 0))
            det.time_since_update = int(result.get('time_since_update', 0))

            track_msg.tracks.append(det)

        self.track_pub.publish(track_msg)

        if results:
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
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
