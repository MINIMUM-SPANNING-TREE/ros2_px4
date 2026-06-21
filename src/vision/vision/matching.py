"""
DeepSORT 距离度量和匹配算法
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from typing import List, Tuple, Optional


def iou(bbox_a: np.ndarray, bbox_b: np.ndarray) -> float:
    """
    计算两个边界框的 IoU (Intersection over Union)
    
    Args:
        bbox_a: 边界框 [x1, y1, x2, y2]
        bbox_b: 边界框 [x1, y1, x2, y2]
        
    Returns:
        iou: IoU 值
    """
    x1 = max(bbox_a[0], bbox_b[0])
    y1 = max(bbox_a[1], bbox_b[1])
    x2 = min(bbox_a[2], bbox_b[2])
    y2 = min(bbox_a[3], bbox_b[3])
    
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    
    area_a = (bbox_a[2] - bbox_a[0]) * (bbox_a[3] - bbox_a[1])
    area_b = (bbox_b[2] - bbox_b[0]) * (bbox_b[3] - bbox_b[1])
    
    union = area_a + area_b - intersection
    
    if union <= 0:
        return 0.0
    
    return intersection / union


def iou_batch(bbox_a: np.ndarray, bbox_b: np.ndarray) -> np.ndarray:
    """
    批量计算 IoU 矩阵
    
    Args:
        bbox_a: 边界框数组 (N, 4) [x1, y1, x2, y2]
        bbox_b: 边界框数组 (M, 4) [x1, y1, x2, y2]
        
    Returns:
        iou_matrix: IoU 矩阵 (N, M)
    """
    if len(bbox_a) == 0 or len(bbox_b) == 0:
        return np.zeros((len(bbox_a), len(bbox_b)))
    
    # 计算交集
    x1 = np.maximum(bbox_a[:, 0:1], bbox_b[:, 0].T)
    y1 = np.maximum(bbox_a[:, 1:2], bbox_b[:, 1].T)
    x2 = np.minimum(bbox_a[:, 2:3], bbox_b[:, 2].T)
    y2 = np.minimum(bbox_a[:, 3:4], bbox_b[:, 3].T)
    
    intersection = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    
    # 计算面积
    area_a = (bbox_a[:, 2] - bbox_a[:, 0]) * (bbox_a[:, 3] - bbox_a[:, 1])
    area_b = (bbox_b[:, 2] - bbox_b[:, 0]) * (bbox_b[:, 3] - bbox_b[:, 1])
    
    union = area_a[:, np.newaxis] + area_b[np.newaxis, :] - intersection
    
    # 避免除以零
    iou_matrix = np.where(union > 0, intersection / union, 0.0)
    
    return iou_matrix


def cosine_distance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    计算余弦距离矩阵
    
    Args:
        a: 特征向量数组 (N, D)
        b: 特征向量数组 (M, D)
        
    Returns:
        distance: 余弦距离矩阵 (N, M)
    """
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    
    # 归一化
    a_norm = a / np.linalg.norm(a, axis=1, keepdims=True)
    b_norm = b / np.linalg.norm(b, axis=1, keepdims=True)
    
    # 计算余弦相似度
    similarity = np.dot(a_norm, b_norm.T)
    
    # 转换为距离
    distance = 1.0 - similarity
    
    return distance


def nn_cosine_distance(detections: np.ndarray, tracks: np.ndarray) -> np.ndarray:
    """
    计算检测和跟踪之间的最近邻余弦距离
    
    Args:
        detections: 检测特征 (N, D)
        tracks: 跟踪特征 (M, D)
        
    Returns:
        distances: 最小余弦距离 (N,)
    """
    if len(tracks) == 0:
        return np.zeros(len(detections))
    
    distances = cosine_distance(detections, tracks)
    
    return distances.min(axis=1)


def matching_cascade(
    distance_metric,
    max_distance: float,
    cascade_depth: int,
    tracks: List,
    detections: List,
    track_indices: Optional[List[int]] = None,
    detection_indices: Optional[List[int]] = None,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """
    匹配级联算法
    
    优先匹配最近更新的跟踪，解决长时间遮挡的目标被新目标抢占的问题
    
    Args:
        distance_metric: 距离度量函数
        max_distance: 最大匹配距离阈值
        cascade_depth: 级联深度
        tracks: 跟踪列表
        detections: 检测列表
        track_indices: 要匹配的跟踪索引
        detection_indices: 要匹配的检测索引
        
    Returns:
        matches: 匹配对列表 [(track_idx, detection_idx), ...]
        unmatched_tracks: 未匹配的跟踪索引
        unmatched_detections: 未匹配的检测索引
    """
    if track_indices is None:
        track_indices = list(range(len(tracks)))
    if detection_indices is None:
        detection_indices = list(range(len(detections)))
    
    unmatched_tracks = list(track_indices)
    unmatched_detections = list(detection_indices)
    matches = []
    
    # 按级联深度分组
    for level in range(cascade_depth):
        if len(unmatched_tracks) == 0 or len(unmatched_detections) == 0:
            break
        
        # 获取当前深度的跟踪
        track_indices_level = [
            k for k in unmatched_tracks 
            if tracks[k].time_since_update == level + 1
        ]
        
        if len(track_indices_level) == 0:
            continue
        
        # 计算距离矩阵
        cost_matrix = distance_metric(
            [tracks[i] for i in track_indices_level],
            [detections[i] for i in unmatched_detections]
        )
        
        # 使用匈牙利算法求解最优匹配
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        # 筛选有效匹配（先收集，再批量更新）
        matched_track_indices = []
        matched_det_indices = []
        for row, col in zip(row_indices, col_indices):
            if cost_matrix[row, col] <= max_distance:
                track_idx = track_indices_level[row]
                det_idx = unmatched_detections[col]
                
                matches.append((track_idx, det_idx))
                matched_track_indices.append(track_idx)
                matched_det_indices.append(det_idx)
        
        # 批量移除已匹配的索引
        for idx in matched_track_indices:
            unmatched_tracks.remove(idx)
        for idx in matched_det_indices:
            unmatched_detections.remove(idx)
    
    return matches, unmatched_tracks, unmatched_detections


def min_cost_matching(
    distance_metric,
    max_distance: float,
    tracks: List,
    detections: List,
    track_indices: Optional[List[int]] = None,
    detection_indices: Optional[List[int]] = None,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """
    最小代价匹配
    
    Args:
        distance_metric: 距离度量函数
        max_distance: 最大匹配距离阈值
        tracks: 跟踪列表
        detections: 检测列表
        track_indices: 要匹配的跟踪索引
        detection_indices: 要匹配的检测索引
        
    Returns:
        matches: 匹配对列表
        unmatched_tracks: 未匹配的跟踪索引
        unmatched_detections: 未匹配的检测索引
    """
    if track_indices is None:
        track_indices = list(range(len(tracks)))
    if detection_indices is None:
        detection_indices = list(range(len(detections)))
    
    if len(track_indices) == 0 or len(detection_indices) == 0:
        return [], list(track_indices), list(detection_indices)
    
    # 计算距离矩阵
    cost_matrix = distance_metric(
        [tracks[i] for i in track_indices],
        [detections[i] for i in detection_indices]
    )
    
    # 使用匈牙利算法
    row_indices, col_indices = linear_sum_assignment(cost_matrix)
    
    matches = []
    unmatched_tracks = list(track_indices)
    unmatched_detections = list(detection_indices)
    
    matched_track_indices = []
    matched_det_indices = []
    for row, col in zip(row_indices, col_indices):
        if cost_matrix[row, col] <= max_distance:
            track_idx = track_indices[row]
            det_idx = detection_indices[col]
            
            matches.append((track_idx, det_idx))
            matched_track_indices.append(track_idx)
            matched_det_indices.append(det_idx)
    
    for idx in matched_track_indices:
        unmatched_tracks.remove(idx)
    for idx in matched_det_indices:
        unmatched_detections.remove(idx)
    
    return matches, unmatched_tracks, unmatched_detections
