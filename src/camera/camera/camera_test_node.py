"""
相机测试节点
用于测试相机驱动和发布功能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header

import numpy as np
import time
import cv2
from typing import Optional


class CameraTestNode(Node):
    """
    相机测试节点
    
    生成测试图像或从视频文件读取图像
    用于测试相机节点和DeepSORT的兼容性
    """
    
    def __init__(self):
        super().__init__('camera_test_node')
        
        # 声明参数
        self.declare_parameter('mode', 'pattern')  # pattern, video, camera, dahua
        self.declare_parameter('video_path', '')
        self.declare_parameter('camera_id', 0)  # 相机索引
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')
        
        # 获取参数
        self.mode = self.get_parameter('mode').value
        self.video_path = self.get_parameter('video_path').value
        self.camera_id = str(self.get_parameter('camera_id').value)  # 转换为字符串
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # 发布者
        self.image_pub = self.create_publisher(
            Image,
            '~/image_raw',
            sensor_qos,
        )
        
        # 视频捕获
        self.cap: Optional[cv2.VideoCapture] = None
        if self.mode == 'video' and self.video_path:
            self.cap = cv2.VideoCapture(self.video_path)
        elif self.mode == 'dahua':
            # 大华相机模式
            if self.camera_id.isdigit():
                self.cap = cv2.VideoCapture(int(self.camera_id), cv2.CAP_V4L2)
            else:
                self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            
            if self.cap.isOpened():
                # 设置分辨率
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f'大华相机已连接: {self.camera_id}')
            else:
                self.get_logger().error(f'无法连接大华相机: {self.camera_id}')
                self.cap = None
        
        # 定时器
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        
        # 统计
        self.frame_count = 0
        self.start_time = time.time()
        
        # 模拟目标位置
        self.target_positions = [
            {'x': 400, 'y': 300, 'vx': 2, 'vy': 1, 'size': 50},
            {'x': 800, 'y': 500, 'vx': -1, 'vy': 2, 'size': 60},
            {'x': 600, 'y': 400, 'vx': 1, 'vy': -1, 'size': 45},
        ]
        
        self.get_logger().info(f'Camera Test Node Started (mode: {self.mode})')
    
    def publish_frame(self):
        """发布一帧图像"""
        try:
            # 获取图像
            if self.mode == 'pattern':
                frame = self._generate_pattern()
            elif self.mode in ['video', 'dahua'] and self.cap is not None:
                ret, frame = self.cap.read()
                if not ret:
                    if self.mode == 'video':
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = self.cap.read()
                    if not ret:
                        return
                frame = cv2.resize(frame, (self.image_width, self.image_height))
            else:
                frame = self._generate_pattern()
            
            # 创建消息
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height, msg.width, _ = frame.shape
            msg.encoding = 'bgr8'
            msg.is_bigendian = False
            msg.step = frame.strides[0]
            msg.data = frame.tobytes()
            
            # 发布
            self.image_pub.publish(msg)
            self.frame_count += 1
            
            # 日志
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                self.get_logger().info(f'Published {self.frame_count} frames ({fps:.1f} FPS)')
                
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')
    
    def _generate_pattern(self) -> np.ndarray:
        """
        生成测试图案
        
        Returns:
            frame: 图像数组
        """
        # 创建黑色背景
        frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # 绘制网格
        for x in range(0, self.image_width, 100):
            cv2.line(frame, (x, 0), (x, self.image_height), (50, 50, 50), 1)
        for y in range(0, self.image_height, 100):
            cv2.line(frame, (0, y), (self.image_width, y), (50, 50, 50), 1)
        
        # 绘制移动的模拟目标
        current_time = time.time()
        for i, pos in enumerate(self.target_positions):
            # 更新位置
            x = pos['x'] + int(pos['vx'] * self.frame_count * 0.5)
            y = pos['y'] + int(pos['vy'] * self.frame_count * 0.5)
            
            # 边界反弹
            if x < 0 or x > self.image_width:
                pos['vx'] = -pos['vx']
            if y < 0 or y > self.image_height:
                pos['vy'] = -pos['vy']
            
            # 绘制矩形目标
            size = pos['size']
            color = [(0, 0, 255), (0, 255, 0), (255, 0, 0)][i % 3]
            
            cv2.rectangle(frame, 
                         (x - size, y - size), 
                         (x + size, y + size), 
                         color, 2)
            
            # 绘制ID
            cv2.putText(frame, f'T{i+1}', (x - 10, y - size - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 绘制时间戳
        timestamp = time.strftime('%H:%M:%S')
        cv2.putText(frame, f'Time: {timestamp}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f'Frame: {self.frame_count}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def destroy_node(self):
        """销毁节点"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraTestNode()
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
