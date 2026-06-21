"""
DeepSORT 卡尔曼滤波器实现
用于目标状态预测和更新
"""

import numpy as np
from typing import Tuple


class KalmanFilter:
    """
    卡尔曼滤波器，用于跟踪目标的状态
    
    状态向量: [x, y, a, h, vx, vy, va, vh]
    - x, y: 边界框中心坐标
    - a: 宽高比 (aspect ratio)
    - h: 高度
    - vx, vy, va, vh: 对应的速度分量
    """
    
    def __init__(self):
        """初始化卡尔曼滤波器参数"""
        # 状态维度
        self.dim_state = 8
        # 观测维度
        self.dim_obs = 4
        
        # 状态转移矩阵 F
        self.F = np.eye(self.dim_state)
        self.F[:4, 4:] = np.eye(4)  # 位置 += 速度 * dt
        
        # 观测矩阵 H
        self.H = np.eye(self.dim_obs, self.dim_state)
        
        # 观测噪声协方差 R
        self.R = np.eye(self.dim_obs)
        self.R[2, 2] *= 10.0
        self.R[3, 3] *= 10.0
        
        # 过程噪声协方差 Q
        self.Q = np.eye(self.dim_state)
        self.Q[4:, 4:] *= 0.01
        self.Q[3, 3] *= 0.01
        
        # 初始不确定度 P
        self.P = np.eye(self.dim_state) * 10.0
        self.P[4:, 4:] *= 100.0
        
        # 位置和速度的标准差权重
        self._std_weight_position = 1. / 20
        self._std_weight_velocity = 1. / 160
    
    def initiate(self, measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        从测量值初始化状态
        
        Args:
            measurement: 边界框 [x, y, a, h]
            
        Returns:
            mean: 状态均值
            covariance: 状态协方差
        """
        mean = np.zeros(self.dim_state)
        mean[:4] = measurement
        
        covariance = np.eye(self.dim_state) * 10.0
        
        # 根据测量值调整不确定度
        std = [
            2 * self._std_weight_position * measurement[3],  # x
            2 * self._std_weight_position * measurement[3],  # y
            1e-2,  # a
            2 * self._std_weight_position * measurement[3],  # h
            10 * self._std_weight_velocity * measurement[3],  # vx
            10 * self._std_weight_velocity * measurement[3],  # vy
            1e-5,  # va
            10 * self._std_weight_velocity * measurement[3],  # vh
        ]
        
        covariance = np.diag(np.square(std))
        
        return mean, covariance
    
    def predict(self, mean: np.ndarray, covariance: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        预测下一时刻的状态
        
        Args:
            mean: 当前状态均值
            covariance: 当前状态协方差
            
        Returns:
            mean: 预测状态均值
            covariance: 预测状态协方差
        """
        # 根据速度调整过程噪声
        std = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-2,
            self._std_weight_position * mean[3],
            self._std_weight_velocity * mean[3],
            self._std_weight_velocity * mean[3],
            1e-5,
            self._std_weight_velocity * mean[3],
        ]
        
        Q = np.diag(np.square(std))
        
        # 预测
        mean = np.dot(self.F, mean)
        covariance = np.linalg.multi_dot([self.F, covariance, self.F.T]) + Q
        
        # 确保协方差矩阵对称
        covariance = (covariance + covariance.T) / 2
        
        return mean, covariance
    
    def update(self, mean: np.ndarray, covariance: np.ndarray, 
               measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        使用测量值更新状态
        
        Args:
            mean: 预测状态均值
            covariance: 预测状态协方差
            measurement: 测量值 [x, y, a, h]
            
        Returns:
            mean: 更新后状态均值
            covariance: 更新后状态协方差
        """
        # 计算卡尔曼增益
        S = np.linalg.multi_dot([self.H, covariance, self.H.T]) + self.R
        K = np.linalg.multi_dot([covariance, self.H.T, np.linalg.inv(S)])
        
        # 计算残差
        y = measurement - np.dot(self.H, mean)
        
        # 更新状态
        mean = mean + np.dot(K, y)
        covariance = covariance - np.linalg.multi_dot([K, S, K.T])
        
        # 确保协方差矩阵对称
        covariance = (covariance + covariance.T) / 2
        
        return mean, covariance
    
    def get_state(self, mean: np.ndarray) -> np.ndarray:
        """
        获取当前状态的边界框表示
        
        Args:
            mean: 状态均值
            
        Returns:
            边界框 [x, y, a, h]
        """
        return mean[:4]
