"""
障碍物检测模块
从激光雷达点云中检测障碍物，输出障碍物信息
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class Obstacle:
    """障碍物数据"""
    x: float           # 障碍物中心 X（ENU，米）
    y: float           # 障碍物中心 Y（ENU，米）
    z: float           # 障碍物中心 Z（ENU，米）
    distance: float    # 到无人机的距离（米）
    angle: float       # 相对于无人机航向的角度（弧度）
    radius: float      # 障碍物近似半径（米）
    point_count: int   # 构成障碍物的点数


class ObstacleDetector:
    """
    障碍物检测器
    
    从 PointCloud2 数据中检测障碍物：
    1. 点云滤波（去除地面点和噪声）
    2. 障碍物聚类（基于欧氏距离）
    3. 障碍物提取（计算中心和半径）
    
    使用方法:
        detector = ObstacleDetector(
            min_height=-1.0,    # 最低高度（去除地面）
            max_height=3.0,     # 最高高度
            min_distance=0.5,   # 最近距离
            max_distance=10.0,  # 最远距离
            cluster_tolerance=0.5,  # 聚类容差
            min_cluster_size=5,     # 最小聚类点数
        )
        
        obstacles = detector.detect(points, drone_pos, drone_yaw)
    """
    
    def __init__(
        self,
        min_height: float = -1.0,
        max_height: float = 3.0,
        min_distance: float = 0.5,
        max_distance: float = 10.0,
        cluster_tolerance: float = 0.5,
        min_cluster_size: int = 5,
        max_cluster_size: int = 1000,
    ):
        """
        初始化障碍物检测器
        
        Args:
            min_height: 最低高度（低于此值视为地面点）
            max_height: 最高高度（高于此值视为噪声）
            min_distance: 最近检测距离（避免检测自身）
            max_distance: 最远检测距离
            cluster_tolerance: 欧氏聚类容差（米）
            min_cluster_size: 最小聚类点数
            max_cluster_size: 最大聚类点数
        """
        self.min_height = min_height
        self.max_height = max_height
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.cluster_tolerance = cluster_tolerance
        self.min_cluster_size = min_cluster_size
        self.max_cluster_size = max_cluster_size
    
    def detect(
        self,
        points: np.ndarray,
        drone_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        drone_yaw: float = 0.0,
    ) -> List[Obstacle]:
        """
        检测障碍物
        
        Args:
            points: 点云数组 (N, 3) 或 (N, 4)，格式 [x, y, z, intensity?]
            drone_position: 无人机位置 (x, y, z)
            drone_yaw: 无人机偏航角（弧度）
            
        Returns:
            obstacles: 障碍物列表
        """
        if len(points) == 0:
            return []
        
        # 确保点云格式正确
        if points.shape[1] >= 3:
            xyz = points[:, :3]
        else:
            return []
        
        # 步骤1: 滤波
        filtered = self._filter_points(xyz, drone_position)
        if len(filtered) == 0:
            return []
        
        # 步骤2: 聚类
        clusters = self._euclidean_clustering(filtered)
        
        # 步骤3: 提取障碍物信息
        obstacles = self._extract_obstacles(clusters, drone_position, drone_yaw)
        
        return obstacles
    
    def _filter_points(
        self,
        points: np.ndarray,
        drone_position: Tuple[float, float, float],
    ) -> np.ndarray:
        """
        点云滤波：去除地面点、过高点、过远点、过近点
        
        Args:
            points: 点云 (N, 3)
            drone_position: 无人机位置
            
        Returns:
            filtered: 滤波后的点云
        """
        dx, dy, dz = drone_position
        
        # 相对高度
        relative_z = points[:, 2] - dz
        
        # 距离计算
        distances = np.sqrt(
            (points[:, 0] - dx) ** 2 +
            (points[:, 1] - dy) ** 2
        )
        
        # 距离过滤
        mask = (
            (distances >= self.min_distance) &
            (distances <= self.max_distance)
        )
        
        # 高度过滤（相对高度）
        mask &= (relative_z >= self.min_height) & (relative_z <= self.max_height)
        
        return points[mask]
    
    def _euclidean_clustering(self, points: np.ndarray) -> List[np.ndarray]:
        """
        欧氏距离聚类
        
        简单的 BFS 聚类算法，将相近的点归为同一障碍物
        
        Args:
            points: 滤波后的点云 (N, 3)
            
        Returns:
            clusters: 聚类列表，每个元素是一个点集
        """
        if len(points) == 0:
            return []
        
        n = len(points)
        visited = np.zeros(n, dtype=bool)
        clusters = []
        
        # 使用 KD-Tree 加速（如果可用）
        try:
            from scipy.spatial import cKDTree
            tree = cKDTree(points)
            use_kdtree = True
        except ImportError:
            use_kdtree = False
        
        for i in range(n):
            if visited[i]:
                continue
            
            # BFS 聚类
            cluster_indices = []
            queue = [i]
            visited[i] = True
            
            while queue:
                idx = queue.pop(0)
                cluster_indices.append(idx)
                
                if use_kdtree:
                    # KD-Tree 查询
                    neighbors = tree.query_ball_point(
                        points[idx], self.cluster_tolerance
                    )
                    for ni in neighbors:
                        if not visited[ni]:
                            visited[ni] = True
                            queue.append(ni)
                else:
                    # 暴力搜索
                    for j in range(n):
                        if not visited[j]:
                            dist = np.linalg.norm(points[idx] - points[j])
                            if dist <= self.cluster_tolerance:
                                visited[j] = True
                                queue.append(j)
                
                # 防止聚类过大
                if len(cluster_indices) > self.max_cluster_size:
                    break
            
            # 过滤太小的聚类
            if self.min_cluster_size <= len(cluster_indices) <= self.max_cluster_size:
                clusters.append(points[cluster_indices])
        
        return clusters
    
    def _extract_obstacles(
        self,
        clusters: List[np.ndarray],
        drone_position: Tuple[float, float, float],
        drone_yaw: float,
    ) -> List[Obstacle]:
        """
        从聚类中提取障碍物信息
        
        Args:
            clusters: 聚类列表
            drone_position: 无人机位置
            drone_yaw: 无人机偏航角
            
        Returns:
            obstacles: 障碍物列表
        """
        obstacles = []
        dx, dy, dz = drone_position
        
        for cluster in clusters:
            # 计算聚类中心
            center = np.mean(cluster, axis=0)
            cx, cy, cz = center
            
            # 计算到无人机的距离
            dist = np.sqrt((cx - dx) ** 2 + (cy - dy) ** 2)
            
            # 计算相对于无人机的角度
            angle = np.arctan2(cy - dy, cx - dx) - drone_yaw
            # 归一化到 [-pi, pi]
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            
            # 计算障碍物半径（点到中心的最大距离）
            if len(cluster) > 1:
                radii = np.sqrt(
                    (cluster[:, 0] - cx) ** 2 +
                    (cluster[:, 1] - cy) ** 2
                )
                radius = np.max(radii)
            else:
                radius = 0.3  # 单点默认半径
            
            obstacle = Obstacle(
                x=cx,
                y=cy,
                z=cz,
                distance=dist,
                angle=angle,
                radius=radius,
                point_count=len(cluster),
            )
            obstacles.append(obstacle)
        
        # 按距离排序
        obstacles.sort(key=lambda o: o.distance)
        
        return obstacles
    
    def get_nearest_obstacle(
        self,
        obstacles: List[Obstacle],
        angle_range: Optional[Tuple[float, float]] = None,
    ) -> Optional[Obstacle]:
        """
        获取最近的障碍物
        
        Args:
            obstacles: 障碍物列表
            angle_range: 角度范围 (min_angle, max_angle)，弧度
            
        Returns:
            nearest: 最近的障碍物，无则返回 None
        """
        if not obstacles:
            return None
        
        if angle_range is not None:
            min_a, max_a = angle_range
            filtered = [
                o for o in obstacles
                if min_a <= o.angle <= max_a
            ]
            if filtered:
                return min(filtered, key=lambda o: o.distance)
        
        return min(obstacles, key=lambda o: o.distance)
    
    def get_obstacles_in_sector(
        self,
        obstacles: List[Obstacle],
        center_angle: float,
        half_angle: float,
    ) -> List[Obstacle]:
        """
        获取指定扇区内的障碍物
        
        Args:
            obstacles: 障碍物列表
            center_angle: 扇区中心角度（弧度）
            half_angle: 扇区半角（弧度）
            
        Returns:
            sector_obstacles: 扇区内的障碍物
        """
        min_a = center_angle - half_angle
        max_a = center_angle + half_angle
        
        return [
            o for o in obstacles
            if min_a <= o.angle <= max_a
        ]
