"""
DeepSORT 跟踪器主类
整合卡尔曼滤波、外观特征匹配和轨迹管理
"""

import numpy as np
from typing import List, Tuple, Optional, Callable

from vision.kalman_filter import KalmanFilter
from vision.track import Track, TrackState, Detection
from vision.matching import (
    iou_batch,
    cosine_distance,
    matching_cascade,
    min_cost_matching,
)


class Tracker:
    """
    DeepSORT 跟踪器
    
    使用级联匹配策略，优先匹配最近更新的跟踪
    结合外观特征和运动信息进行数据关联
    """
    
    def __init__(
        self,
        max_cosine_distance: float = 0.2,
        nn_budget: int = 100,
        max_iou_distance: float = 0.7,
        max_age: int = 30,
        n_init: int = 3,
        lambda_iou: float = 0.98,
    ):
        """
        初始化跟踪器
        
        Args:
            max_cosine_distance: 外观特征最大余弦距离阈值
            nn_budget: 每个跟踪保存的最大特征数量
            max_iou_distance: IoU 距离阈值（用于级联匹配失败后的备选）
            max_age: 最大丢失帧数
            n_init: 确认为有效跟踪所需的连续匹配次数
            lambda_iou: IoU 权重（当同时使用外观和运动信息时）
        """
        self.max_cosine_distance = max_cosine_distance
        self.nn_budget = nn_budget
        self.max_iou_distance = max_iou_distance
        self.max_age = max_age
        self.n_init = n_init
        self.lambda_iou = lambda_iou
        
        # 卡尔曼滤波器
        self.kf = KalmanFilter()
        
        # 跟踪列表
        self.tracks: List[Track] = []
        
        # 跟踪ID计数器
        self._next_id = 1
        
    def predict(self) -> None:
        """对所有跟踪执行卡尔曼预测"""
        for track in self.tracks:
            track.predict(self.kf)
    
    def update(
        self,
        detections: List[Detection],
        classes: Optional[List[int]] = None,
        confidences: Optional[List[float]] = None,
    ) -> Tuple[List[Track], List[Track], List[Track]]:
        """
        使用检测结果更新跟踪器
        
        Args:
            detections: 检测结果列表
            classes: 类别ID列表
            confidences: 置信度列表
            
        Returns:
            tracked: 成功跟踪的目标列表
            untracked: 未匹配的跟踪列表
            detections_matched: 匹配到的检测索引
        """
        # 运行匹配级联
        matches, unmatched_tracks, unmatched_detections = self._match(detections)
        
        # 更新匹配的跟踪
        for track_idx, detection_idx in matches:
            self.tracks[track_idx].update(
                self.kf,
                detections[detection_idx],
                feature=detections[detection_idx].feature,
            )
        
        # 标记未匹配的跟踪
        for track_idx in unmatched_tracks:
            self.tracks[track_idx].mark_missed()
        
        # 为未匹配的检测创建新跟踪
        for detection_idx in unmatched_detections:
            self._initiate_track(detections[detection_idx])
        
        # 删除已失效的跟踪
        self.tracks = [t for t in self.tracks if not t.is_deleted()]
        
        # 返回结果
        tracked = [t for t in self.tracks if t.is_confirmed()]
        untracked = [t for t in self.tracks if t.is_tentative()]
        
        return tracked, untracked, matches
    
    def _match(self, detections: List[Detection]) -> Tuple[
        List[Tuple[int, int]], List[int], List[int]
    ]:
        """
        执行数据关联
        
        Args:
            detections: 检测结果列表
            
        Returns:
            matches: 匹配对
            unmatched_tracks: 未匹配的跟踪
            unmatched_detections: 未匹配的检测
        """
        # 按状态分组跟踪
        confirmed_tracks = [i for i, t in enumerate(self.tracks) if t.is_confirmed()]
        unconfirmed_tracks = [i for i, t in enumerate(self.tracks) if t.is_tentative()]
        
        # 第一阶段：使用外观特征的级联匹配
        matches_a, unmatched_tracks_a, unmatched_detections = matching_cascade(
            distance_metric=self._nn_distance,
            max_distance=self.max_cosine_distance,
            cascade_depth=self.max_age,
            tracks=self.tracks,
            detections=detections,
            track_indices=confirmed_tracks,
            detection_indices=list(range(len(detections))),
        )
        
        # 第二阶段：对未匹配的跟踪使用 IoU 匹配
        # 包括未确认的跟踪和第一阶段未匹配的确认跟踪
        iou_track_candidates = unconfirmed_tracks + [
            k for k in unmatched_tracks_a if self.tracks[k].time_since_update == 1
        ]
        
        matches_b, unmatched_tracks_b, unmatched_detections = min_cost_matching(
            distance_metric=iou_distance,
            max_distance=self.max_iou_distance,
            tracks=self.tracks,
            detections=detections,
            track_indices=iou_track_candidates,
            detection_indices=unmatched_detections,
        )
        
        # 合并匹配结果
        matches = matches_a + matches_b
        matched_tracks_b = {k for k, _ in matches_b}
        unmatched_tracks = list((set(unmatched_tracks_a) - matched_tracks_b) | set(unmatched_tracks_b))
        
        return matches, unmatched_tracks, unmatched_detections
    
    def _nn_distance(self, tracks: List[Track], detections: List[Detection]) -> np.ndarray:
        """
        计算最近邻余弦距离
        
        Args:
            tracks: 跟踪列表
            detections: 检测列表
            
        Returns:
            cost_matrix: 代价矩阵 (len(tracks), len(detections))
        """
        cost_matrix = np.full((len(tracks), len(detections)), self.max_cosine_distance + 1e-6)

        track_features_list = [t.feature for t in tracks]
        detection_features_list = [d.feature for d in detections]

        track_has_feature = [f is not None for f in track_features_list]
        detection_has_feature = [f is not None for f in detection_features_list]

        track_idx = [i for i, f in enumerate(track_features_list) if f is not None]
        detection_idx = [i for i, f in enumerate(detection_features_list) if f is not None]

        if not track_idx or not detection_idx:
            return cost_matrix

        track_features = np.array([track_features_list[i] for i in track_idx])
        detection_features = np.array([detection_features_list[i] for i in detection_idx])

        # 计算余弦距离
        sub_matrix = cosine_distance(detection_features, track_features)
        sub_matrix = sub_matrix.T  # (len(track_idx), len(detection_idx))

        # 将子矩阵填入完整矩阵的对应位置
        for i, ti in enumerate(track_idx):
            for j, dj in enumerate(detection_idx):
                cost_matrix[ti, dj] = sub_matrix[i, j]

        return cost_matrix
    
    def _initiate_track(self, detection: Detection) -> None:
        """
        为检测创建新跟踪
        
        Args:
            detection: 检测结果
        """
        mean, covariance = self.kf.initiate(detection.to_xyah())
        
        track = Track(
            mean=mean,
            covariance=covariance,
            track_id=self._next_id,
            n_init=self.n_init,
            max_age=self.max_age,
            feature=detection.feature,
            class_id=detection.class_id,
            confidence=detection.confidence,
        )
        
        self.tracks.append(track)
        self._next_id += 1
    
    def get_tracks(self) -> List[Track]:
        """
        获取所有活跃跟踪
        
        Returns:
            tracks: 跟踪列表
        """
        return [t for t in self.tracks if t.is_confirmed()]
    
    def get_track_by_id(self, track_id: int) -> Optional[Track]:
        """
        根据ID获取跟踪
        
        Args:
            track_id: 跟踪ID
            
        Returns:
            track: 跟踪对象，未找到返回 None
        """
        for track in self.tracks:
            if track.track_id == track_id:
                return track
        return None
    
    def reset(self) -> None:
        """重置跟踪器"""
        self.tracks.clear()
        self._next_id = 1


def iou_distance(tracks: List[Track], detections: List[Detection]) -> np.ndarray:
    """
    计算跟踪和检测之间的 IoU 距离
    
    Args:
        tracks: 跟踪列表
        detections: 检测列表
        
    Returns:
        cost_matrix: IoU 距离矩阵 (1 - IoU)
    """
    if len(tracks) == 0 or len(detections) == 0:
        return np.zeros((len(tracks), len(detections)))
    
    # 获取边界框
    track_boxes = np.array([t.to_tlbr() for t in tracks])
    detection_boxes = np.array([d.to_tlbr() for d in detections])
    
    # 计算 IoU
    iou_matrix = iou_batch(track_boxes, detection_boxes)
    
    # 转换为距离
    distance_matrix = 1.0 - iou_matrix
    
    return distance_matrix
