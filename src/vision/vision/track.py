"""
DeepSORT 跟踪轨迹管理
"""

import numpy as np
from typing import Optional, List
from vision.kalman_filter import KalmanFilter


class TrackState:
    """跟踪状态枚举"""
    TENTATIVE = 1   # 临时状态（刚初始化）
    CONFIRMED = 2   # 确认状态（持续跟踪中）
    DELETED = 3     # 删除状态（丢失）


class Track:
    """
    单个目标的跟踪轨迹
    
    包含卡尔曼滤波器状态、外观特征、生命周期管理
    """
    
    def __init__(
        self,
        mean: np.ndarray,
        covariance: np.ndarray,
        track_id: int,
        n_init: int = 3,
        max_age: int = 30,
        feature: Optional[np.ndarray] = None,
        class_id: Optional[int] = None,
        confidence: Optional[float] = None,
    ):
        """
        初始化跟踪轨迹
        
        Args:
            mean: 卡尔曼滤波器初始状态均值
            covariance: 卡尔曼滤波器初始状态协方差
            track_id: 跟踪ID
            n_init: 确认为有效跟踪所需的连续匹配次数
            max_age: 最大丢失帧数（超过则删除）
            feature: 外观特征向量
            class_id: 类别ID
            confidence: 检测置信度
        """
        self.track_id = track_id
        self.mean = mean
        self.covariance = covariance
        self.n_init = n_init
        self.max_age = max_age
        
        # 状态
        self.state = TrackState.TENTATIVE
        self.hits = 1          # 连续匹配次数
        self.age = 1           # 总帧数
        self.time_since_update = 0  # 自上次更新以来的帧数
        
        # 外观特征
        self.features: List[np.ndarray] = []
        if feature is not None:
            self.features.append(feature)
        
        # 类别和置信度
        self.class_id = class_id
        self.confidence = confidence
        
    def predict(self, kf: KalmanFilter) -> None:
        """
        使用卡尔曼滤波器预测下一帧状态
        
        Args:
            kf: 卡尔曼滤波器实例
        """
        self.mean, self.covariance = kf.predict(self.mean, self.covariance)
        self.age += 1
        self.time_since_update += 1
        
    def update(self, kf: KalmanFilter, detection, feature: Optional[np.ndarray] = None) -> None:
        """
        使用检测结果更新跟踪状态
        
        Args:
            kf: 卡尔曼滤波器实例
            detection: 检测结果（边界框 [x, y, a, h]）
            feature: 外观特征
        """
        self.mean, self.covariance = kf.update(
            self.mean, self.covariance, detection.to_xyah()
        )
        
        self.hits += 1
        self.time_since_update = 0
        self.state = TrackState.CONFIRMED
        
        # 更新外观特征
        if feature is not None:
            self.features.append(feature)
            
        # 更新类别和置信度
        if hasattr(detection, 'class_id'):
            self.class_id = detection.class_id
        if hasattr(detection, 'confidence'):
            self.confidence = detection.confidence
    
    def mark_missed(self) -> None:
        """标记为未匹配（丢失）"""
        if self.state == TrackState.TENTATIVE:
            self.state = TrackState.DELETED
        elif self.time_since_update > self.max_age:
            self.state = TrackState.DELETED
    
    def is_tentative(self) -> bool:
        """是否为临时状态"""
        return self.state == TrackState.TENTATIVE
    
    def is_confirmed(self) -> bool:
        """是否为确认状态"""
        return self.state == TrackState.CONFIRMED
    
    def is_deleted(self) -> bool:
        """是否已删除"""
        return self.state == TrackState.DELETED
    
    @property
    def feature(self) -> Optional[np.ndarray]:
        """获取最新的外观特征"""
        if len(self.features) > 0:
            return self.features[-1]
        return None
    
    def get_state(self) -> np.ndarray:
        """
        获取当前跟踪状态（边界框）
        
        Returns:
            bbox: 边界框 [x, y, a, h]
        """
        return self.mean[:4].copy()
    
    def to_tlwh(self) -> np.ndarray:
        """
        转换为 [top_left_x, top_left_y, width, height] 格式
        
        Returns:
            tlwh: 边界框
        """
        ret = self.mean[:4].copy()
        ret[2] *= ret[3]  # a * h = w
        ret[:2] -= ret[2:] / 2  # 中心 -> 左上角
        return ret
    
    def to_tlbr(self) -> np.ndarray:
        """
        转换为 [top_left_x, top_left_y, bottom_right_x, bottom_right_y] 格式
        
        Returns:
            tlbr: 边界框
        """
        ret = self.to_tlwh()
        ret[2:] += ret[:2]  # [x, y, w, h] -> [x, y, x+w, y+h]
        return ret
    
    def __repr__(self) -> str:
        return f"Track(id={self.track_id}, state={self.state}, hits={self.hits}, age={self.age})"


class Detection:
    """
    检测结果
    """
    
    def __init__(
        self,
        tlwh: np.ndarray,
        confidence: float,
        feature: Optional[np.ndarray] = None,
        class_id: Optional[int] = None,
    ):
        """
        初始化检测结果
        
        Args:
            tlwh: 边界框 [top_left_x, top_left_y, width, height]
            confidence: 检测置信度
            feature: 外观特征向量
            class_id: 类别ID
        """
        self.tlwh = np.asarray(tlwh, dtype=np.float32)
        self.confidence = confidence
        self.feature = feature
        self.class_id = class_id
        
    def to_xyah(self) -> np.ndarray:
        """
        转换为卡尔曼滤波器使用的 [center_x, center_y, aspect_ratio, height] 格式
        
        Returns:
            xyah: 边界框
        """
        ret = self.tlwh.copy()
        ret[:2] += ret[2:] / 2  # 左上角 -> 中心
        ret[2] /= ret[3]  # width / height = aspect_ratio
        return ret
    
    def to_tlbr(self) -> np.ndarray:
        """
        转换为 [top_left_x, top_left_y, bottom_right_x, bottom_right_y] 格式
        
        Returns:
            tlbr: 边界框
        """
        ret = self.tlwh.copy()
        ret[2:] += ret[:2]
        return ret
    
    def __repr__(self) -> str:
        return f"Detection(conf={self.confidence:.2f}, class={self.class_id})"
