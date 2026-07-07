"""
DeepSORT 跟踪器节点 - 完整实现
支持订阅图像和检测结果，发布带跟踪ID的可视化结果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from uav_interfaces.msg import DetectionArray, TrackDetection, TrackDetectionArray

import numpy as np
import json
import cv2
from typing import Optional, List, Dict, Any, Tuple

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


class DeepSORTTrackerNode(Node):

    COLORS = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
        (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
        (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
    ]

    def __init__(self):
        super().__init__('deepsort_tracker_node')

        self._declare_parameters()
        self._get_parameters()

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

        self.current_image: Optional[np.ndarray] = None
        self.image_header = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image, '~/image_raw', self._image_callback, sensor_qos,
        )
        self.detection_sub = self.create_subscription(
            DetectionArray, '~/detections', self._detection_callback, sensor_qos,
        )

        self.track_pub = self.create_publisher(TrackDetectionArray, '~/tracks', 10)
        self.track_image_pub = self.create_publisher(
            Image, '~/tracks_image', 5,
        )

        self.frame_count = 0
        self.total_tracks = 0
        self.track_trajectories: Dict[int, List[Tuple[float, float]]] = {}

        self.get_logger().info('=' * 50)
        self.get_logger().info('DeepSORT Tracker Node Started')
        self.get_logger().info('=' * 50)
        self._log_parameters()

    def _declare_parameters(self):
        self.declare_parameter('model_name', 'simple')
        self.declare_parameter('reid_model', '')
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('feature_dim', 512)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_cosine_distance', 0.2)
        self.declare_parameter('nn_budget', 100)
        self.declare_parameter('max_iou_distance', 0.7)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('n_init', 3)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('show_trajectory', True)
        self.declare_parameter('trajectory_length', 30)

    def _get_parameters(self):
        self.model_name = self.get_parameter('model_name').value
        self.reid_model = self.get_parameter('reid_model').value or ''
        self.backend = self.get_parameter('backend').value
        self.feature_dim = self.get_parameter('feature_dim').value
        self.device = self.get_parameter('device').value
        self.max_cosine_distance = self.get_parameter('max_cosine_distance').value
        self.nn_budget = self.get_parameter('nn_budget').value
        self.max_iou_distance = self.get_parameter('max_iou_distance').value
        self.max_age = self.get_parameter('max_age').value
        self.n_init = self.get_parameter('n_init').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.show_visualization = self.get_parameter('show_visualization').value
        self.show_trajectory = self.get_parameter('show_trajectory').value
        self.trajectory_length = self.get_parameter('trajectory_length').value

    def _log_parameters(self):
        self.get_logger().info(f'  Model: {self.model_name}, Backend: {self.backend}')
        self.get_logger().info(f'  ReID: {self.reid_model or "None (simple)"}')
        self.get_logger().info(f'  Max cosine: {self.max_cosine_distance}, Max IoU: {self.max_iou_distance}')
        self.get_logger().info(f'  Max age: {self.max_age}, N init: {self.n_init}')

    def _image_callback(self, msg: Image):
        self.image_header = msg.header
        try:
            self.current_image = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')

    def _detection_callback(self, msg: DetectionArray):
        self.frame_count += 1

        bboxes, scores, classes = self._parse_detections(msg)

        results = self.deepsort.update(
            image=self.current_image,
            bboxes=bboxes,
            scores=scores,
            classes=classes,
        )

        self._update_trajectories(results)
        self._publish_tracks(results, msg.header)

        if self.show_visualization and self.current_image is not None:
            self._publish_visualization(results, msg.header)

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {len(results)} tracks, '
                f'total unique: {self.total_tracks}',
            )

    def _parse_detections(self, msg: DetectionArray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        bboxes = []
        scores = []
        classes = []

        for det in msg.detections:
            if det.score < self.confidence_threshold:
                continue

            bboxes.append([det.x_min, det.y_min, det.x_max, det.y_max])
            scores.append(det.score)
            classes.append(int(det.class_id) if det.class_id else 0)

        if bboxes:
            return (
                np.array(bboxes, dtype=np.float32),
                np.array(scores, dtype=np.float32),
                np.array(classes, dtype=int),
            )
        return (
            np.array([]).reshape(0, 4),
            np.array([], dtype=np.float32),
            np.array([], dtype=int),
        )

    def _update_trajectories(self, results: List[Dict[str, Any]]):
        current_ids = set()

        for result in results:
            track_id = result['track_id']
            current_ids.add(track_id)

            bbox = result['bbox']
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2

            if track_id not in self.track_trajectories:
                self.track_trajectories[track_id] = []
                self.total_tracks += 1

            self.track_trajectories[track_id].append((cx, cy))
            if len(self.track_trajectories[track_id]) > self.trajectory_length:
                self.track_trajectories[track_id].pop(0)

        disappeared = set(self.track_trajectories.keys()) - current_ids
        for tid in disappeared:
            if len(self.track_trajectories[tid]) == 0:
                del self.track_trajectories[tid]

    def _publish_tracks(self, results: List[Dict[str, Any]], header):
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

    def _publish_visualization(self, results: List[Dict[str, Any]], header):
        if self.current_image is None:
            return

        vis_image = self.current_image.copy()

        for result in results:
            track_id = result['track_id']
            x1, y1, x2, y2 = [int(v) for v in result['bbox']]
            score = result['score']
            color = self.COLORS[track_id % len(self.COLORS)]

            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
            label = f'ID:{track_id} {score:.2f}'
            cv2.putText(vis_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if self.show_trajectory and track_id in self.track_trajectories:
                trajectory = self.track_trajectories[track_id]
                for i in range(1, len(trajectory)):
                    pt1 = (int(trajectory[i - 1][0]), int(trajectory[i - 1][1]))
                    pt2 = (int(trajectory[i][0]), int(trajectory[i][1]))
                    cv2.line(vis_image, pt1, pt2, color, 2)

        try:
            self.track_image_pub.publish(cv2_to_ros_image(vis_image, header))
        except Exception as e:
            self.get_logger().warn(f'可视化图像发布失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DeepSORTTrackerNode()
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
