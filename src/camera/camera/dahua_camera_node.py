"""
ROS2 大华相机节点
发布图像话题，兼容DeepSORT输入
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import numpy as np
import time
import threading
from typing import Optional

from camera.dahua_camera import DahuaCamera


class DahuaCameraNode(Node):
    """
    大华相机 ROS2 节点
    
    功能:
        1. 连接大华工业相机
        2. 采集图像并发布到 ROS2 话题
        3. 支持动态参数调整
    
    发布:
        - ~/image_raw (sensor_msgs/Image): 原始图像
        - ~/camera_info (sensor_msgs/CameraInfo): 相机信息
    """
    
    def __init__(self):
        super().__init__('dahua_camera_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self._get_parameters()
        
        # 创建相机实例
        self.camera = DahuaCamera(
            camera_id=self.camera_id,
            width=self.image_width,
            height=self.image_height,
            fps=self.fps,
            exposure=self.exposure,
            gain=self.gain,
            trigger_mode=self.trigger_mode,
        )
        
        # QoS 配置 - 兼容DeepSORT订阅
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # 发布者
        self.image_pub = self.create_publisher(
            Image,
            '~/image_raw',
            sensor_qos,
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '~/camera_info',
            10,
        )
        
        # 相机信息
        self.camera_info_msg = self._create_camera_info()
        
        # 线程控制
        self.is_running = False
        self.capture_thread: Optional[threading.Thread] = None
        
        # 统计
        self.frame_count = 0
        self.publish_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        
        # 连接相机
        self._connect_camera()
        
        # 创建定时器用于参数更新
        self.create_timer(1.0, self._check_parameters)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Dahua Camera Node Started')
        self.get_logger().info('='*50)
    
    def _declare_parameters(self):
        """声明参数"""
        # 相机参数
        self.declare_parameter('camera_id', '0')
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('exposure', 10000.0)
        self.declare_parameter('gain', 0.0)
        self.declare_parameter('trigger_mode', False)
        
        # 发布参数
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('auto_start', True)
        
        # ROI参数
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_width', 0)
        self.declare_parameter('roi_height', 0)
    
    def _get_parameters(self):
        """获取参数"""
        # 相机参数
        self.camera_id = self.get_parameter('camera_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.exposure = self.get_parameter('exposure').value
        self.gain = self.get_parameter('gain').value
        self.trigger_mode = self.get_parameter('trigger_mode').value
        
        # 发布参数
        self.frame_id = self.get_parameter('frame_id').value
        self.qos_depth = self.get_parameter('qos_depth').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # ROI参数
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_width = self.get_parameter('roi_width').value
        self.roi_height = self.get_parameter('roi_height').value
    
    def _create_camera_info(self) -> CameraInfo:
        """创建相机信息消息"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # 相机内参（需要根据实际标定结果设置）
        # 这里使用默认值
        camera_info.k = [
            self.image_width, 0.0, self.image_width / 2.0,
            0.0, self.image_width, self.image_height / 2.0,
            0.0, 0.0, 1.0
        ]
        
        camera_info.p = [
            self.image_width, 0.0, self.image_width / 2.0, 0.0,
            0.0, self.image_width, self.image_height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return camera_info
    
    def _connect_camera(self):
        """连接相机"""
        self.get_logger().info(f'Connecting to camera: {self.camera_id}')
        
        if self.camera.connect():
            self.get_logger().info(f'Camera connected: {self.camera.sdk_type}')
            
            # 设置ROI
            if self.roi_width > 0 and self.roi_height > 0:
                self.camera.set_roi(self.roi_x, self.roi_y, 
                                   self.roi_width, self.roi_height)
            
            # 自动开始采集
            if self.auto_start:
                self._start_capture()
        else:
            self.get_logger().error('Failed to connect camera')
    
    def _start_capture(self):
        """开始采集"""
        if self.camera.start_streaming(callback=self._frame_callback):
            self.is_running = True
            self.get_logger().info('Streaming started')
        else:
            self.get_logger().error('Failed to start streaming')
    
    def _stop_capture(self):
        """停止采集"""
        self.is_running = False
        self.camera.stop_streaming()
        self.get_logger().info('Streaming stopped')
    
    def _frame_callback(self, frame: np.ndarray):
        """
        帧回调函数
        
        Args:
            frame: 图像数组
        """
        try:
            self.frame_count += 1
            
            # 创建图像消息
            image_msg = self._create_image_msg(frame)
            
            # 发布图像
            self.image_pub.publish(image_msg)
            self.publish_count += 1
            
            # 发布相机信息
            self.camera_info_msg.header.stamp = image_msg.header.stamp
            self.camera_info_pub.publish(self.camera_info_msg)
            
            # 计算并发布FPS
            self._update_fps()
            
        except Exception as e:
            self.get_logger().error(f'Frame callback error: {e}')
    
    def _create_image_msg(self, frame: np.ndarray) -> Image:
        """
        创建ROS Image消息
        
        Args:
            frame: 图像数组
            
        Returns:
            image_msg: Image消息
        """
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # 根据图像格式设置编码
        if len(frame.shape) == 2:
            msg.encoding = 'mono8'
            msg.height, msg.width = frame.shape
        elif frame.shape[2] == 3:
            msg.encoding = 'bgr8'
            msg.height, msg.width, _ = frame.shape
        elif frame.shape[2] == 4:
            msg.encoding = 'bgra8'
            msg.height, msg.width, _ = frame.shape
        else:
            msg.encoding = 'bgr8'
            msg.height, msg.width, _ = frame.shape
        
        msg.is_bigendian = False
        msg.step = frame.strides[0]
        msg.data = frame.tobytes()
        
        return msg
    
    def _update_fps(self):
        """更新FPS统计"""
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            fps = (self.frame_count - self.last_fps_count) / (current_time - self.last_fps_time)
            self.get_logger().info(
                f'FPS: {fps:.1f}, Frames: {self.frame_count}, Published: {self.publish_count}',
                throttle_duration_sec=5.0,
            )
            self.last_fps_time = current_time
            self.last_fps_count = self.frame_count
    
    def _check_parameters(self):
        """检查参数更新"""
        # 这里可以添加动态参数更新逻辑
        pass
    
    def start_streaming_service(self, request, response):
        """开始采集服务"""
        self._start_capture()
        response.success = True
        return response
    
    def stop_streaming_service(self, request, response):
        """停止采集服务"""
        self._stop_capture()
        response.success = True
        return response
    
    def destroy_node(self):
        """销毁节点"""
        self._stop_capture()
        self.camera.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DahuaCameraNode()
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
