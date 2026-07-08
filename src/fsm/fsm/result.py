#!/usr/bin/env python3
"""Save YOLO detections and DeepSORT tracks to local JSONL files."""

import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_interfaces.msg import DetectionArray, TrackDetectionArray


class VisionResultLogger(Node):
    def __init__(self):
        super().__init__('vision_result_logger')

        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.declare_parameter('deepsort_topic', '/deepsort/tracks')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('flush_every', 1)

        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.deepsort_topic = self.get_parameter('deepsort_topic').value
        self.flush_every = max(1, int(self.get_parameter('flush_every').value))
        self.write_count = 0

        self.output_dir = self._resolve_output_dir(self.get_parameter('output_dir').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.yolo_file = (self.output_dir / 'yolo_detections.jsonl').open('a', encoding='utf-8')
        self.deepsort_file = (self.output_dir / 'deepsort_tracks.jsonl').open('a', encoding='utf-8')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(DetectionArray, self.yolo_topic, self._yolo_cb, qos)
        self.create_subscription(TrackDetectionArray, self.deepsort_topic, self._deepsort_cb, qos)

        self.get_logger().info(f'Saving YOLO results from {self.yolo_topic}')
        self.get_logger().info(f'Saving DeepSORT results from {self.deepsort_topic}')
        self.get_logger().info(f'Output directory: {self.output_dir}')

    def _resolve_output_dir(self, output_dir):
        if output_dir:
            return Path(output_dir).expanduser().resolve()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return (Path.cwd() / 'data' / 'vision_results' / timestamp).resolve()

    def _yolo_cb(self, msg):
        record = {
            'stamp': self._stamp_to_dict(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'count': len(msg.detections),
            'detections': [self._detection_to_dict(det) for det in msg.detections],
        }
        self._write_jsonl(self.yolo_file, record)

    def _deepsort_cb(self, msg):
        record = {
            'stamp': self._stamp_to_dict(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'count': len(msg.tracks),
            'tracks': [self._track_to_dict(track) for track in msg.tracks],
        }
        self._write_jsonl(self.deepsort_file, record)

    def _write_jsonl(self, file_obj, record):
        file_obj.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.write_count += 1
        if self.write_count % self.flush_every == 0:
            file_obj.flush()

    @staticmethod
    def _stamp_to_dict(stamp):
        return {
            'sec': int(stamp.sec),
            'nanosec': int(stamp.nanosec),
        }

    @staticmethod
    def _detection_to_dict(det):
        return {
            'class_id': det.class_id,
            'score': float(det.score),
            'bbox': {
                'x_min': float(det.x_min),
                'y_min': float(det.y_min),
                'x_max': float(det.x_max),
                'y_max': float(det.y_max),
            },
        }

    @staticmethod
    def _track_to_dict(track):
        return {
            'track_id': int(track.track_id),
            'class_id': track.class_id,
            'score': float(track.score),
            'bbox': {
                'x_min': float(track.x_min),
                'y_min': float(track.y_min),
                'x_max': float(track.x_max),
                'y_max': float(track.y_max),
            },
            'age': int(track.age),
            'hits': int(track.hits),
            'time_since_update': int(track.time_since_update),
        }

    def destroy_node(self):
        for file_obj in (getattr(self, 'yolo_file', None), getattr(self, 'deepsort_file', None)):
            if file_obj is not None and not file_obj.closed:
                file_obj.flush()
                file_obj.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionResultLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
