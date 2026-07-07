#!/usr/bin/env python3
"""Save PointCloud2 messages to local files."""

import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudResultLogger(Node):
    def __init__(self):
        super().__init__('pointcloud_result_logger')

        self.declare_parameter('pointcloud_topic', '/lslidar_point_cloud')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('flush_every', 1)
        self.declare_parameter('save_binary', True)

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value                         # 点云话题
        self.flush_every = max(1, int(self.get_parameter('flush_every').value))                      # 刷新间隔
        self.save_binary = bool(self.get_parameter('save_binary').value)                             # 是否保存二进制点云数据
        self.write_count = 0                                                                         # 写入次数
        self.cloud_count = 0                                                                         # 点云帧计数

        self.output_dir = self._resolve_output_dir(self.get_parameter('output_dir').value)           # 输出目录
        self.bin_dir = self.output_dir / 'pointcloud_bins'                                           # 二进制点云目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if self.save_binary:
            self.bin_dir.mkdir(parents=True, exist_ok=True)
        self.index_file = (self.output_dir / 'pointcloud_index.jsonl').open('a', encoding='utf-8')   # 点云索引文件

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._pointcloud_cb, qos)       # 订阅点云话题

        self.get_logger().info(f'Saving pointcloud from {self.pointcloud_topic}')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Save binary data: {self.save_binary}')

    def _resolve_output_dir(self, output_dir): # 解析输出目录
        if output_dir:
            return Path(output_dir).expanduser().resolve()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return (Path.cwd() / 'data' / 'pointcloud_results' / timestamp).resolve()

    def _pointcloud_cb(self, msg): # 处理点云消息的回调函数
        self.cloud_count += 1
        data_file = ''
        if self.save_binary:
            data_file = f'pointcloud_bins/pointcloud_{self.cloud_count:06d}.bin'
            (self.output_dir / data_file).write_bytes(bytes(msg.data))

        record = {
            'stamp': self._stamp_to_dict(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'height': int(msg.height),
            'width': int(msg.width),
            'fields': [self._field_to_dict(field) for field in msg.fields],
            'is_bigendian': bool(msg.is_bigendian),
            'point_step': int(msg.point_step),
            'row_step': int(msg.row_step),
            'is_dense': bool(msg.is_dense),
            'data_size': len(msg.data),
            'data_file': data_file,
        }
        self._write_jsonl(self.index_file, record)

    def _write_jsonl(self, file_obj, record): # 写入一行JSON记录
        file_obj.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.write_count += 1
        if self.write_count % self.flush_every == 0:
            file_obj.flush()

    @staticmethod
    def _stamp_to_dict(stamp): # 时间戳转换为字典
        return {
            'sec': int(stamp.sec),
            'nanosec': int(stamp.nanosec),
        }

    @staticmethod
    def _field_to_dict(field): # 点云字段转换为字典
        return {
            'name': field.name,
            'offset': int(field.offset),
            'datatype': int(field.datatype),
            'count': int(field.count),
        }

    def destroy_node(self): # 关闭文件并销毁节点
        if hasattr(self, 'index_file') and not self.index_file.closed:
            self.index_file.flush()
            self.index_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudResultLogger()
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
