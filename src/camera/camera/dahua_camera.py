"""
大华工业相机驱动
支持大华MVSDK和GenICam标准接口
"""

import numpy as np
import time
from typing import Optional, Tuple, Callable
import threading

from camera.camera_base import CameraBase, CameraState


class DahuaCamera(CameraBase):
    """
    大华工业相机驱动类
    
    支持：
    - 大华MVSDK原生接口
    - OpenCV VideoCapture后备方案
    """
    
    def __init__(self, 
                 camera_id: str = "0",
                 width: int = 1920,
                 height: int = 1080,
                 fps: float = 30.0,
                 exposure: float = 10000.0,
                 gain: float = 0.0,
                 trigger_mode: bool = False,
                 pixel_format: str = "BGR8"):
        """
        初始化大华相机
        
        Args:
            camera_id: 相机ID（序列号、IP或索引）
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            exposure: 曝光时间（微秒）
            gain: 增益（dB）
            trigger_mode: 是否使用触发模式
            pixel_format: 像素格式 (BGR8, MONO8, RGB8)
        """
        super().__init__(camera_id, width, height, fps)
        
        self.exposure = exposure
        self.gain = gain
        self.trigger_mode = trigger_mode
        self.pixel_format = pixel_format
        
        # 相机句柄
        self.camera_handle = None
        self.sdk_type = None
        
    def connect(self) -> bool:
        """
        连接相机
        
        Returns:
            success: 是否连接成功
        """
        # 尝试大华MVSDK
        if self._try_connect_mvsdk():
            self.sdk_type = "mvsdk"
            self.state = CameraState.CONNECTED
            return True
        
        # 尝试OpenCV作为后备
        if self._try_connect_opencv():
            self.sdk_type = "opencv"
            self.state = CameraState.CONNECTED
            return True
        
        self.state = CameraState.ERROR
        return False
    
    def _try_connect_mvsdk(self) -> bool:
        """尝试使用大华MVSDK连接"""
        try:
            # 大华MVSDK导入
            from MvCameraControl_class import MvCamera, MV_CC_DEVICE_INFO_LIST, \
                MV_GIGE_DEVICE, MV_USB_DEVICE, MV_ACCESS_Exclusive
            
            self.camera_handle = MvCamera()
            
            # 枚举设备
            device_list = MV_CC_DEVICE_INFO_LIST()
            ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
            
            if ret != 0 or device_list.nDeviceNum == 0:
                self.camera_handle = None
                return False
            
            # 查找指定相机
            device_info = None
            for i in range(device_list.nDeviceNum):
                info = device_list.pDeviceInfo[i]
                # 这里需要根据实际SDK结构获取序列号
                if self.camera_id == "0":
                    device_info = info
                    break
            
            if device_info is None:
                self.camera_handle = None
                return False
            
            # 创建句柄并打开设备
            ret = self.camera_handle.MV_CC_CreateHandle(device_info)
            if ret != 0:
                self.camera_handle = None
                return False
            
            ret = self.camera_handle.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                self.camera_handle.MV_CC_DestroyHandle()
                self.camera_handle = None
                return False
            
            # 设置参数
            self._set_mvsdk_params()
            
            return True
            
        except ImportError:
            return False
        except Exception as e:
            print(f"MVSDK连接失败: {e}")
            return False
    
    def _set_mvsdk_params(self):
        """设置MVSDK相机参数"""
        if self.camera_handle is None:
            return
        
        try:
            # 设置分辨率
            self.camera_handle.MV_CC_SetIntValue("Width", self.width)
            self.camera_handle.MV_CC_SetIntValue("Height", self.height)
            
            # 设置帧率
            self.camera_handle.MV_CC_SetFloatValue("AcquisitionFrameRate", self.fps)
            
            # 设置曝光
            self.camera_handle.MV_CC_SetFloatValue("ExposureTime", self.exposure)
            
            # 设置增益
            self.camera_handle.MV_CC_SetFloatValue("Gain", self.gain)
            
            # 设置触发模式
            if self.trigger_mode:
                self.camera_handle.MV_CC_SetEnumValue("TriggerMode", 1)
            else:
                self.camera_handle.MV_CC_SetEnumValue("TriggerMode", 0)
        except Exception as e:
            print(f"设置参数失败: {e}")
    
    def _try_connect_opencv(self) -> bool:
        """尝试使用OpenCV连接（主要用于USB相机或测试）"""
        try:
            import cv2
            
            # 尝试打开相机
            if self.camera_id.isdigit():
                # 尝试使用V4L2后端
                cap = cv2.VideoCapture(int(self.camera_id), cv2.CAP_V4L2)
            else:
                # 尝试设备路径
                if self.camera_id.startswith('/dev/video'):
                    cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
                else:
                    cap = cv2.VideoCapture(self.camera_id)
            
            if not cap.isOpened():
                return False
            
            # 设置分辨率
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 测试读取一帧
            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                return False
            
            self.camera_handle = cap
            return True
            
        except ImportError:
            return False
        except Exception as e:
            print(f"OpenCV连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开相机连接"""
        if self.is_streaming:
            self.stop_streaming()
        
        if self.camera_handle is not None:
            if self.sdk_type == "mvsdk":
                self.camera_handle.MV_CC_CloseDevice()
                self.camera_handle.MV_CC_DestroyHandle()
            elif self.sdk_type == "opencv":
                self.camera_handle.release()
            
            self.camera_handle = None
        
        self.state = CameraState.DISCONNECTED
    
    def start_streaming(self, callback: Optional[Callable] = None) -> bool:
        """
        开始采集图像
        
        Args:
            callback: 帧回调函数 callback(frame: np.ndarray)
            
        Returns:
            success: 是否成功
        """
        if self.state != CameraState.CONNECTED:
            return False
        
        self.frame_callback = callback
        
        if self.sdk_type == "mvsdk":
            return self._start_mvsdk_streaming()
        elif self.sdk_type == "opencv":
            self.is_streaming = True
            self.state = CameraState.STREAMING
            return True
        
        return False
    
    def _start_mvsdk_streaming(self) -> bool:
        """开始MVSDK采集"""
        try:
            # 注册回调
            if self.frame_callback:
                self.camera_handle.MV_CC_RegisterImageCallBackEx(
                    self._mvsdk_frame_callback, None)
            
            # 开始采集
            ret = self.camera_handle.MV_CC_StartGrabbing()
            if ret != 0:
                return False
            
            self.is_streaming = True
            self.state = CameraState.STREAMING
            return True
            
        except Exception as e:
            print(f"开始采集失败: {e}")
            return False
    
    def _mvsdk_frame_callback(self, frame_info, user_data):
        """MVSDK帧回调"""
        try:
            if frame_info is None:
                return
            
            # 转换图像格式
            frame = self._convert_mvsdk_frame(frame_info)
            
            if frame is not None:
                with self.frame_lock:
                    self.current_frame = frame
                    self.frame_count += 1
                    self.last_frame_time = time.time()
                
                if self.frame_callback:
                    self.frame_callback(frame)
                    
        except Exception as e:
            print(f"帧回调处理失败: {e}")
    
    def _convert_mvsdk_frame(self, frame_info) -> Optional[np.ndarray]:
        """转换MVSDK帧为numpy数组"""
        try:
            # 这里需要根据实际SDK结构实现
            # 示例代码，需要根据大华SDK文档调整
            
            # 获取图像数据
            frame_data = frame_info.pBufAddr
            frame_len = frame_info.stFrameInfo.nFrameLen
            width = frame_info.stFrameInfo.nWidth
            height = frame_info.stFrameInfo.nHeight
            
            # 根据像素格式转换
            pixel_type = frame_info.stFrameInfo.enPixelType
            
            if pixel_type == 0x02180001:  # BGR8
                frame = np.frombuffer(frame_data, dtype=np.uint8, count=frame_len)
                frame = frame.reshape((height, width, 3))
            elif pixel_type == 0x01080001:  # Mono8
                frame = np.frombuffer(frame_data, dtype=np.uint8, count=frame_len)
                frame = frame.reshape((height, width))
            else:
                frame = np.frombuffer(frame_data, dtype=np.uint8, count=frame_len)
                frame = frame.reshape((height, width, -1))
            
            return frame.copy()
            
        except Exception as e:
            print(f"帧转换失败: {e}")
            return None
    
    def stop_streaming(self):
        """停止采集"""
        self.is_streaming = False
        
        if self.camera_handle is not None:
            if self.sdk_type == "mvsdk":
                self.camera_handle.MV_CC_StopGrabbing()
        
        self.state = CameraState.CONNECTED
    
    def grab_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧图像
        
        Returns:
            frame: 图像数组，失败返回None
        """
        if not self.is_streaming:
            return None
        
        if self.sdk_type == "opencv":
            ret, frame = self.camera_handle.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame
                    self.frame_count += 1
                    self.last_frame_time = time.time()
                return frame
            return None
        
        # MVSDK使用回调模式，直接返回缓存的帧
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def set_exposure(self, exposure_us: float) -> bool:
        """
        设置曝光时间
        
        Args:
            exposure_us: 曝光时间（微秒）
            
        Returns:
            success: 是否成功
        """
        self.exposure = exposure_us
        
        if self.camera_handle is None:
            return False
        
        if self.sdk_type == "mvsdk":
            return self.camera_handle.MV_CC_SetFloatValue("ExposureTime", exposure_us) == 0
        elif self.sdk_type == "opencv":
            import cv2
            return self.camera_handle.set(cv2.CAP_PROP_EXPOSURE, exposure_us)
        
        return False
    
    def set_gain(self, gain_db: float) -> bool:
        """
        设置增益
        
        Args:
            gain_db: 增益（dB）
            
        Returns:
            success: 是否成功
        """
        self.gain = gain_db
        
        if self.camera_handle is None:
            return False
        
        if self.sdk_type == "mvsdk":
            return self.camera_handle.MV_CC_SetFloatValue("Gain", gain_db) == 0
        elif self.sdk_type == "opencv":
            import cv2
            return self.camera_handle.set(cv2.CAP_PROP_GAIN, gain_db)
        
        return False
    
    def trigger_software(self) -> bool:
        """
        软触发一次采集
        
        Returns:
            success: 是否成功
        """
        if not self.trigger_mode:
            return False
        
        if self.camera_handle is None:
            return False
        
        if self.sdk_type == "mvsdk":
            return self.camera_handle.MV_CC_SetCommandValue("TriggerSoftware") == 0
        
        return False
    
    def get_camera_info(self) -> dict:
        """
        获取相机信息
        
        Returns:
            info: 相机信息字典
        """
        info = super().get_camera_info()
        info.update({
            "sdk_type": self.sdk_type,
            "exposure": self.exposure,
            "gain": self.gain,
            "trigger_mode": self.trigger_mode,
        })
        return info


def list_dahua_cameras() -> list:
    """
    枚举所有连接的大华相机
    
    Returns:
        cameras: 相机信息列表
    """
    cameras = []
    
    # 尝试MVSDK枚举
    try:
        from MvCameraControl_class import MvCamera, MV_CC_DEVICE_INFO_LIST, \
            MV_GIGE_DEVICE, MV_USB_DEVICE
        
        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
        
        if ret == 0:
            for i in range(device_list.nDeviceNum):
                info = device_list.pDeviceInfo[i]
                cameras.append({
                    "index": i,
                    "serial": "unknown",
                    "model": "Dahua",
                    "type": "GigE/USB",
                })
    except ImportError:
        pass
    
    # 尝试OpenCV枚举
    try:
        import cv2
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    cameras.append({
                        "index": i,
                        "serial": f"opencv_{i}",
                        "model": "Unknown",
                        "type": "OpenCV",
                    })
                cap.release()
    except:
        pass
    
    return cameras
