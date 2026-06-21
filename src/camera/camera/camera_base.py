"""
相机基类
提供通用的相机接口
"""

import numpy as np
import time
from typing import Optional, Tuple, Callable
from enum import Enum
import threading


class CameraState(Enum):
    """相机状态枚举"""
    DISCONNECTED = 0
    CONNECTED = 1
    STREAMING = 2
    ERROR = 3


class CameraBase:
    """
    相机基类
    
    所有相机驱动的基类，提供统一的接口
    """
    
    def __init__(self,
                 camera_id: str = "0",
                 width: int = 1920,
                 height: int = 1080,
                 fps: float = 30.0):
        """
        初始化相机基类
        
        Args:
            camera_id: 相机ID
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        
        # 状态
        self.state = CameraState.DISCONNECTED
        self.is_streaming = False
        
        # 帧缓存
        self.current_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        self.last_frame_time = 0
        
        # 回调函数
        self.frame_callback: Optional[Callable] = None
        
    def connect(self) -> bool:
        """连接相机"""
        raise NotImplementedError
    
    def disconnect(self):
        """断开连接"""
        raise NotImplementedError
    
    def start_streaming(self, callback: Optional[Callable] = None) -> bool:
        """开始采集"""
        raise NotImplementedError
    
    def stop_streaming(self):
        """停止采集"""
        raise NotImplementedError
    
    def grab_frame(self) -> Optional[np.ndarray]:
        """获取一帧"""
        raise NotImplementedError
    
    def get_frame(self) -> Tuple[Optional[np.ndarray], float]:
        """获取帧和时间戳"""
        frame = self.grab_frame()
        timestamp = self.last_frame_time
        return frame, timestamp
    
    def set_exposure(self, exposure_us: float) -> bool:
        """设置曝光"""
        return False
    
    def set_gain(self, gain_db: float) -> bool:
        """设置增益"""
        return False
    
    def set_roi(self, x: int, y: int, width: int, height: int) -> bool:
        """设置ROI"""
        return False
    
    def trigger_software(self) -> bool:
        """软触发"""
        return False
    
    def get_camera_info(self) -> dict:
        """获取相机信息"""
        return {
            "camera_id": self.camera_id,
            "state": self.state.name,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "frame_count": self.frame_count,
            "is_streaming": self.is_streaming,
        }
