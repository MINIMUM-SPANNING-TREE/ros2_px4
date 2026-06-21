"""
DeepSORT 主类
整合跟踪器、特征提取器和数据关联
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any

from vision.tracker import Tracker, Detection
from vision.track import Track
from vision.feature_extractor import FeatureExtractor


class DeepSORT:
    """
    DeepSORT 目标跟踪器
    
    结合深度外观特征和运动信息的多目标跟踪算法
    
    典型用法:
        deepsort = DeepSORT()
        
        # 处理每一帧
        detections = deepsort.update(image, bboxes, scores, classes)
        
        # 获取跟踪结果
        for track in deepsort.get_tracks():
            print(f"Track {track.track_id}: {track.to_tlbr()}")
    """
    
    def __init__(
        self,
        # 外观特征参数
        model_name: str = "simple",
        reid_model_path: Optional[str] = None,
        feature_dim: int = 512,
        max_cosine_distance: float = 0.2,
        nn_budget: int = 100,
        # 运动参数
        max_iou_distance: float = 0.7,
        max_age: int = 30,
        n_init: int = 3,
        # 混合参数
        lambda_iou: float = 0.98,
        # 设备
        device: str = "cpu",
        # 后端
        backend: str = "auto",
    ):
        """
        初始化 DeepSORT 跟踪器
        
        Args:
            model_name: 特征提取模型名称 ("simple", "osnet", "resnet")
            feature_dim: 特征维度
            max_cosine_distance: 外观特征最大余弦距离阈值
            nn_budget: 每个跟踪保存的最大特征数量
            max_iou_distance: IoU 距离阈值
            max_age: 最大丢失帧数
            n_init: 确认为有效跟踪所需的连续匹配次数
            lambda_iou: IoU 权重
            device: 计算设备 ("cpu" 或 "cuda")
        """
        # 特征提取器
        self.feature_extractor = FeatureExtractor(
            model_path=reid_model_path,
            input_size=(256, 128),
            feature_dim=feature_dim,
            backend=backend,
            device=device,
        )
        
        # 跟踪器
        self.tracker = Tracker(
            max_cosine_distance=max_cosine_distance,
            nn_budget=nn_budget,
            max_iou_distance=max_iou_distance,
            max_age=max_age,
            n_init=n_init,
            lambda_iou=lambda_iou,
        )
        
        # 当前帧跟踪结果
        self._tracks: List[Track] = []
        
    def update(
        self,
        image: Optional[np.ndarray] = None,
        bboxes: Optional[np.ndarray] = None,
        scores: Optional[np.ndarray] = None,
        classes: Optional[np.ndarray] = None,
        detections: Optional[List[Dict[str, Any]]] = None,
    ) -> List[Dict[str, Any]]:
        """
        更新跟踪器
        
        Args:
            image: 输入图像 (H, W, 3), BGR 格式
            bboxes: 边界框数组 (N, 4) [x1, y1, x2, y2]
            scores: 置信度数组 (N,)
            classes: 类别数组 (N,)
            detections: 检测结果字典列表（替代参数）
                - bbox: [x1, y1, x2, y2]
                - score: 置信度
                - class: 类别ID
            
        Returns:
            results: 跟踪结果列表
                - track_id: 跟踪ID
                - bbox: [x1, y1, x2, y2]
                - score: 置信度
                - class: 类别ID
                - age: 跟踪年龄
                - hits: 匹配次数
        """
        # 从字典列表解析输入
        if detections is not None and bboxes is None:
            if len(detections) > 0:
                bboxes = np.array([d['bbox'] for d in detections])
                scores = np.array([d.get('score', 1.0) for d in detections])
                classes = np.array([d.get('class', 0) for d in detections])
            else:
                bboxes = np.array([]).reshape(0, 4)
                scores = np.array([])
                classes = np.array([])
        
        # 处理空输入
        if bboxes is None or len(bboxes) == 0:
            self.tracker.predict()
            self.tracker.update([])
            self._tracks = self.tracker.get_tracks()
            return self._format_results()
        
        # 确保输入格式正确
        bboxes = np.asarray(bboxes, dtype=np.float32)
        if scores is None:
            scores = np.ones(len(bboxes))
        if classes is None:
            classes = np.zeros(len(bboxes), dtype=int)
            
        # 提取外观特征
        if image is not None and len(bboxes) > 0:
            features = self.feature_extractor.extract(image, bboxes)
        else:
            # 无图像时使用随机特征（仅IoU匹配）
            features = np.random.randn(len(bboxes), self.feature_extractor.feature_dim)
            features = features / np.linalg.norm(features, axis=1, keepdims=True)
        
        # 创建检测对象
        detections = []
        for i in range(len(bboxes)):
            tlwh = self._xyxy_to_tlwh(bboxes[i])
            detection = Detection(
                tlwh=tlwh,
                confidence=scores[i],
                feature=features[i] if i < len(features) else None,
                class_id=int(classes[i]),
            )
            detections.append(detection)
        
        # 预测
        self.tracker.predict()
        
        # 更新
        tracked, untracked, matches = self.tracker.update(detections)
        self._tracks = self.tracker.get_tracks()
        
        return self._format_results()
    
    def _format_results(self) -> List[Dict[str, Any]]:
        """
        格式化跟踪结果
        
        Returns:
            results: 结果字典列表
        """
        results = []
        
        for track in self._tracks:
            result = {
                'track_id': track.track_id,
                'bbox': track.to_tlbr().tolist(),
                'bbox_tlwh': track.to_tlwh().tolist(),
                'score': float(track.confidence) if track.confidence else 1.0,
                'class': int(track.class_id) if track.class_id is not None else 0,
                'age': track.age,
                'hits': track.hits,
                'time_since_update': track.time_since_update,
                'state': track.state,
            }
            results.append(result)
        
        return results
    
    def _xyxy_to_tlwh(self, bbox: np.ndarray) -> np.ndarray:
        """
        转换边界框格式: [x1, y1, x2, y2] -> [x1, y1, w, h]
        
        Args:
            bbox: [x1, y1, x2, y2]
            
        Returns:
            tlwh: [x1, y1, w, h]
        """
        ret = bbox.copy()
        ret[2:] -= ret[:2]  # [x2, y2] - [x1, y1] = [w, h]
        return ret
    
    def get_tracks(self) -> List[Dict[str, Any]]:
        """
        获取当前所有活跃跟踪
        
        Returns:
            tracks: 跟踪结果列表
        """
        return self._format_results()
    
    def get_track_by_id(self, track_id: int) -> Optional[Dict[str, Any]]:
        """
        根据ID获取跟踪
        
        Args:
            track_id: 跟踪ID
            
        Returns:
            track: 跟踪结果，未找到返回 None
        """
        track = self.tracker.get_track_by_id(track_id)
        if track is None:
            return None
            
        return {
            'track_id': track.track_id,
            'bbox': track.to_tlbr().tolist(),
            'bbox_tlwh': track.to_tlwh().tolist(),
            'score': float(track.confidence) if track.confidence else 1.0,
            'class': int(track.class_id) if track.class_id is not None else 0,
            'age': track.age,
            'hits': track.hits,
            'time_since_update': track.time_since_update,
        }
    
    def reset(self) -> None:
        """重置跟踪器"""
        self.tracker.reset()
        self._tracks = []
    
    @property
    def tracks(self) -> List[Dict[str, Any]]:
        """获取当前跟踪结果"""
        return self._format_results()
    
    @property
    def num_tracks(self) -> int:
        """获取活跃跟踪数量"""
        return len(self._tracks)
    
    @property
    def next_id(self) -> int:
        """获取下一个跟踪ID"""
        return self.tracker._next_id
