"""
ROS2 通用相机节点
支持多种相机类型，兼容DeepSORT输入
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

from array import array
import numpy as np
import time
import threading
from typing import Optional, Union

from camera.camera_base import CameraBase
from camera.dahua_camera import DahuaCamera


class CameraNode(Node):
    """
    通用相机 ROS2 节点
    
    支持:
        - 大华工业相机
        - OpenCV兼容相机
        - 图像文件/视频流
    
    发布:
        - ~/image_raw (sensor_msgs/Image): 原始图像
        - ~/camera_info (sensor_msgs/CameraInfo): 相机信息
    """
    
    # 支持的相机类型
    CAMERA_TYPES = {
        'dahua': DahuaCamera,
    }
    
    def __init__(self):
        super().__init__('camera_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self._get_parameters()
        
        # 创建相机实例
        self.camera = self._create_camera()
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
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
        
        # 控制变量
        self.is_running = False
        self.capture_thread: Optional[threading.Thread] = None
        
        # 统计
        self.frame_count = 0
        self.publish_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        
        # 连接相机
        if self.auto_connect:
            self._connect_and_start()
        
        self.get_logger().info('='*50)
        self.get_logger().info(f'Camera Node Started ({self.camera_type})')
        self.get_logger().info('='*50)
    
    def _declare_parameters(self):
        """声明参数"""
        # 相机类型
        self.declare_parameter('camera_type', 'dahua')
        self.declare_parameter('camera_id', '0')
        
        # 图像参数
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('fps', 30.0)
        
        # 曝光参数
        self.declare_parameter('exposure', 10000.0)
        self.declare_parameter('gain', 0.0)
        self.declare_parameter('software_gain', 1.0)
        self.declare_parameter('software_offset', 0.0)
        
        # 触发参数
        self.declare_parameter('trigger_mode', False)
        
        # 发布参数
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('auto_start', True)
    
    def _get_parameters(self):
        """获取参数"""
        # 相机类型
        self.camera_type = self.get_parameter('camera_type').value
        self.camera_id = self.get_parameter('camera_id').value
        
        # 图像参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        
        # 曝光参数
        self.exposure = self.get_parameter('exposure').value
        self.gain = self.get_parameter('gain').value
        self.software_gain = self.get_parameter('software_gain').value
        self.software_offset = self.get_parameter('software_offset').value
        
        # 触发参数
        self.trigger_mode = self.get_parameter('trigger_mode').value
        
        # 发布参数
        self.frame_id = self.get_parameter('frame_id').value
        self.qos_depth = self.get_parameter('qos_depth').value
        self.auto_connect = self.get_parameter('auto_connect').value
        self.auto_start = self.get_parameter('auto_start').value
    
    def _create_camera(self) -> CameraBase:
        """
        创建相机实例
        
        Returns:
            camera: 相机实例
        """
        camera_class = self.CAMERA_TYPES.get(self.camera_type)
        
        if camera_class is None:
            self.get_logger().warn(
                f'Unknown camera type: {self.camera_type}, using DahuaCamera')
            camera_class = DahuaCamera
        
        return camera_class(
            camera_id=self.camera_id,
            width=self.image_width,
            height=self.image_height,
            fps=self.fps,
            exposure=self.exposure,
            gain=self.gain,
            trigger_mode=self.trigger_mode,
            software_gain=self.software_gain,
            software_offset=self.software_offset,
        )
    
    def _create_camera_info(self) -> CameraInfo:
        """创建相机信息消息"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # 使用默认内参（需要根据实际标定设置）
        fx = self.image_width * 0.8
        fy = self.image_width * 0.8
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return camera_info
    
    def _connect_and_start(self):
        """连接相机并开始采集"""
        if self._connect_camera():
            if self.auto_start:
                self._start_capture()
    
    def _connect_camera(self) -> bool:
        """
        连接相机
        
        Returns:
            success: 是否成功
        """
        self.get_logger().info(f'Connecting to {self.camera_type} camera: {self.camera_id}')
        
        if self.camera.connect():
            self.get_logger().info(f'Camera connected: {self.camera.sdk_type}')
            return True
        else:
            self.get_logger().error('Failed to connect camera')
            return False
    
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
            
            # 创建并发布图像消息
            image_msg = self._create_image_msg(frame)
            self.image_pub.publish(image_msg)
            self.publish_count += 1
            
            # 更新并发布相机信息
            self.camera_info_msg.header.stamp = image_msg.header.stamp
            self.camera_info_pub.publish(self.camera_info_msg)
            
            # 更新FPS
            self._update_fps()
            
        except Exception as e:
            self.get_logger().error(f'Frame callback error: {e}')
    
    def _create_image_msg(self, frame: np.ndarray) -> Image:
        """
        创建ROS Image消息
        
        Args:
            frame: 图像数组 (H, W, C) 或 (H, W)
            
        Returns:
            image_msg: Image消息
        """
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # 处理不同格式
        if len(frame.shape) == 2:
            msg.encoding = 'mono8'
            msg.height, msg.width = frame.shape
        elif len(frame.shape) == 3:
            if frame.shape[2] == 1:
                msg.encoding = 'mono8'
                msg.height, msg.width = frame.shape[:2]
            elif frame.shape[2] == 3:
                msg.encoding = 'bgr8'
                msg.height, msg.width, _ = frame.shape
            elif frame.shape[2] == 4:
                msg.encoding = 'bgra8'
                msg.height, msg.width, _ = frame.shape
        
        msg.is_bigendian = False
        frame = np.ascontiguousarray(frame)
        msg.step = frame.strides[0]
        data = array('B')
        data.frombytes(frame.tobytes())
        msg.data = data
        
        return msg
    
    def _update_fps(self):
        """更新FPS统计"""
        current_time = time.time()
        if current_time - self.last_fps_time >= 2.0:
            fps = (self.frame_count - self.last_fps_count) / (current_time - self.last_fps_time)
            self.get_logger().info(
                f'FPS: {fps:.1f}, Total: {self.frame_count}',
                throttle_duration_sec=5.0,
            )
            self.last_fps_time = current_time
            self.last_fps_count = self.frame_count
    
    # 公共接口
    def start_streaming(self) -> bool:
        """开始采集"""
        self._start_capture()
        return self.is_running
    
    def stop_streaming(self):
        """停止采集"""
        self._stop_capture()
    
    def grab_frame(self) -> Optional[np.ndarray]:
        """获取一帧"""
        return self.camera.grab_frame()
    
    def set_exposure(self, exposure_us: float) -> bool:
        """设置曝光"""
        return self.camera.set_exposure(exposure_us)
    
    def set_gain(self, gain_db: float) -> bool:
        """设置增益"""
        return self.camera.set_gain(gain_db)
    
    def destroy_node(self):
        """销毁节点"""
        self._stop_capture()
        self.camera.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
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
