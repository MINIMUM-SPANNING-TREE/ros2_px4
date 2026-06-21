"""
局部避障规划器
基于 VFH (Vector Field Histogram) 算法的 UAV 避障
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from navigation.obstacle_detector import Obstacle


@dataclass
class VelocityCommand:
    """速度指令"""
    vx: float      # 前向速度（m/s）
    vy: float      # 右向速度（m/s）
    vz: float      # 上向速度（m/s）
    yaw_rate: float  # 偏航角速度（rad/s）


class LocalPlanner:
    """
    局部避障规划器
    
    基于 VFH 算法的简化版本：
    1. 将周围空间划分为扇区
    2. 计算每个扇区的障碍物密度
    3. 选择最优安全方向
    4. 生成速度指令
    
    使用方法:
        planner = LocalPlanner(
            safety_distance=2.0,    # 安全距离
            sector_count=36,        # 扇区数量
            max_speed=2.0,          # 最大速度
        )
        
        cmd = planner.plan(
            goal=(5.0, 3.0, 2.0),
            current_pos=(0.0, 0.0, 2.0),
            current_yaw=0.0,
            obstacles=obstacles,
        )
    """
    
    def __init__(
        self,
        safety_distance: float = 2.0,
        warning_distance: float = 3.0,
        sector_count: int = 36,
        max_speed: float = 2.0,
        max_yaw_rate: float = 0.5,
        goal_weight: float = 1.0,
        safety_weight: float = 2.0,
        smooth_weight: float = 0.5,
    ):
        """
        初始化局部规划器
        
        Args:
            safety_distance: 安全距离（米），低于此距离触发紧急避障
            warning_distance: 警告距离（米），开始避障
            sector_count: 扇区数量（角度分辨率 = 360/sector_count）
            max_speed: 最大速度（m/s）
            max_yaw_rate: 最大偏航角速度（rad/s）
            goal_weight: 目标方向权重
            safety_weight: 安全方向权重
            smooth_weight: 平滑权重
        """
        self.safety_distance = safety_distance
        self.warning_distance = warning_distance
        self.sector_count = sector_count
        self.sector_angle = 2 * np.pi / sector_count
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.goal_weight = goal_weight
        self.safety_weight = safety_weight
        self.smooth_weight = smooth_weight
        
        # 扇区中心角度
        self.sector_centers = np.linspace(
            -np.pi, np.pi, sector_count, endpoint=False
        )
        
        # 上一次的指令（用于平滑）
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_yaw_rate = 0.0
    
    def plan(
        self,
        goal: Tuple[float, float, float],
        current_pos: Tuple[float, float, float],
        current_yaw: float,
        obstacles: List[Obstacle],
        current_velocity: Optional[Tuple[float, float, float]] = None,
    ) -> VelocityCommand:
        """
        规划避障速度指令
        
        Args:
            goal: 目标位置 (x, y, z)
            current_pos: 当前位置 (x, y, z)
            current_yaw: 当前偏航角（弧度）
            obstacles: 障碍物列表
            current_velocity: 当前速度 (vx, vy, vz)，可选
            
        Returns:
            cmd: 速度指令
        """
        gx, gy, gz = goal
        cx, cy, cz = current_pos
        
        # 计算到目标的距离和角度
        dx = gx - cx
        dy = gy - cy
        dz = gz - cz
        
        dist_to_goal = np.sqrt(dx ** 2 + dy ** 2)
        goal_angle = np.arctan2(dy, dx)
        
        # 计算目标方向相对于当前航向的角度
        goal_relative_angle = goal_angle - current_yaw
        goal_relative_angle = (goal_relative_angle + np.pi) % (2 * np.pi) - np.pi
        
        # 计算扇区障碍物代价
        sector_costs = self._compute_sector_costs(obstacles, current_pos, current_yaw)
        
        # 计算每个扇区的总代价（目标方向 + 安全代价）
        total_costs = np.zeros(self.sector_count)
        
        for i in range(self.sector_count):
            sector_angle = self.sector_centers[i]
            
            # 目标方向代价（越接近目标方向，代价越低）
            angle_diff = abs(sector_angle - goal_relative_angle)
            angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
            goal_cost = angle_diff / np.pi
            
            # 总代价
            total_costs[i] = (
                self.goal_weight * goal_cost +
                self.safety_weight * sector_costs[i]
            )
        
        # 选择最优扇区
        best_sector_idx = np.argmin(total_costs)
        best_angle = self.sector_centers[best_sector_idx]
        
        # 检查是否需要紧急避障
        nearest_obstacle = self._get_nearest_obstacle_in_path(
            obstacles, current_pos, current_yaw, best_angle
        )
        
        if nearest_obstacle and nearest_obstacle.distance < self.safety_distance:
            # 紧急避障：停止前进，侧向躲避
            return self._emergency_avoid(
                nearest_obstacle, current_pos, current_yaw
            )
        
        # 计算速度指令
        cmd = self._compute_velocity(
            best_angle, dist_to_goal, dz,
            nearest_obstacle, current_yaw
        )
        
        # 平滑处理
        cmd = self._smooth_command(cmd)
        
        return cmd
    
    def _compute_sector_costs(
        self,
        obstacles: List[Obstacle],
        current_pos: Tuple[float, float, float],
        current_yaw: float,
    ) -> np.ndarray:
        """
        计算每个扇区的障碍物代价
        
        Args:
            obstacles: 障碍物列表
            current_pos: 当前位置
            current_yaw: 当前偏航角
            
        Returns:
            costs: 扇区代价数组
        """
        costs = np.zeros(self.sector_count)
        
        for obs in obstacles:
            # 障碍物角度（相对航向）
            obs_angle = obs.angle
            
            # 距离越近，代价越高
            if obs.distance < self.safety_distance:
                dist_cost = 1.0  # 最高代价
            elif obs.distance < self.warning_distance:
                dist_cost = (self.warning_distance - obs.distance) / \
                           (self.warning_distance - self.safety_distance)
            else:
                dist_cost = 0.0
            
            # 将障碍物代价分配到对应扇区
            for i in range(self.sector_count):
                sector_angle = self.sector_centers[i]
                angle_diff = abs(sector_angle - obs_angle)
                angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
                
                # 障碍物半径对应的角宽度
                if obs.distance > 0:
                    half_angular_width = np.arctan2(
                        obs.radius + self.safety_distance * 0.5,
                        obs.distance
                    )
                else:
                    half_angular_width = np.pi
                
                # 在角宽度内分配代价
                if angle_diff <= half_angular_width:
                    costs[i] = max(costs[i], dist_cost)
        
        return costs
    
    def _get_nearest_obstacle_in_path(
        self,
        obstacles: List[Obstacle],
        current_pos: Tuple[float, float, float],
        current_yaw: float,
        target_angle: float,
    ) -> Optional[Obstacle]:
        """
        获取目标方向上最近的障碍物
        
        Args:
            obstacles: 障碍物列表
            current_pos: 当前位置
            current_yaw: 当前偏航角
            target_angle: 目标方向角度
            
        Returns:
            nearest: 最近的障碍物
        """
        half_angle = np.pi / 6  # 30度范围
        
        nearest = None
        min_dist = float('inf')
        
        for obs in obstacles:
            angle_diff = abs(obs.angle - target_angle)
            angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
            
            if angle_diff <= half_angle and obs.distance < min_dist:
                min_dist = obs.distance
                nearest = obs
        
        return nearest
    
    def _emergency_avoid(
        self,
        obstacle: Obstacle,
        current_pos: Tuple[float, float, float],
        current_yaw: float,
    ) -> VelocityCommand:
        """
        紧急避障：停止前进，侧向躲避
        
        Args:
            obstacle: 最近的障碍物
            current_pos: 当前位置
            current_yaw: 当前偏航角
            
        Returns:
            cmd: 速度指令
        """
        # 障碍物角度
        obs_angle = obstacle.angle
        
        # 选择躲避方向（左侧或右侧，选择空间较大的一侧）
        if obs_angle > 0:
            # 障碍物在左侧，向右躲避
            avoid_yaw_rate = -self.max_yaw_rate
        else:
            # 障碍物在右侧，向左躲避
            avoid_yaw_rate = self.max_yaw_rate
        
        # 低速后退
        vx = -0.3
        vy = 0.0
        vz = 0.0
        
        return VelocityCommand(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=avoid_yaw_rate,
        )
    
    def _compute_velocity(
        self,
        best_angle: float,
        dist_to_goal: float,
        dz: float,
        nearest_obstacle: Optional[Obstacle],
        current_yaw: float,
    ) -> VelocityCommand:
        """
        计算速度指令
        
        Args:
            best_angle: 最优方向角度
            dist_to_goal: 到目标的距离
            dz: 高度差
            nearest_obstacle: 最近障碍物
            current_yaw: 当前偏航角
            
        Returns:
            cmd: 速度指令
        """
        # 速度大小（根据距离调整）
        if dist_to_goal > 1.0:
            speed = self.max_speed
        else:
            # 接近目标时减速
            speed = self.max_speed * dist_to_goal
        
        # 如果附近有障碍物，降低速度
        if nearest_obstacle and nearest_obstacle.distance < self.warning_distance:
            speed_factor = nearest_obstacle.distance / self.warning_distance
            speed *= max(speed_factor, 0.3)
        
        # 分解到 x, y 方向
        vx = speed * np.cos(best_angle)
        vy = speed * np.sin(best_angle)
        
        # 垂直速度（保持高度或调整高度）
        vz = np.clip(dz * 0.5, -0.5, 0.5)
        
        # 偏航角速度（转向目标方向）
        yaw_rate = np.clip(best_angle * 0.5, -self.max_yaw_rate, self.max_yaw_rate)
        
        return VelocityCommand(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
        )
    
    def _smooth_command(self, cmd: VelocityCommand) -> VelocityCommand:
        """
        平滑处理速度指令
        
        Args:
            cmd: 原始速度指令
            
        Returns:
            smoothed: 平滑后的速度指令
        """
        alpha = self.smooth_weight
        
        smoothed = VelocityCommand(
            vx=cmd.vx * (1 - alpha) + self._last_vx * alpha,
            vy=cmd.vy * (1 - alpha) + self._last_vy * alpha,
            vz=cmd.vz,
            yaw_rate=cmd.yaw_rate * (1 - alpha) + self._last_yaw_rate * alpha,
        )
        
        self._last_vx = smoothed.vx
        self._last_vy = smoothed.vy
        self._last_yaw_rate = smoothed.yaw_rate
        
        return smoothed
    
    def is_path_clear(
        self,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        obstacles: List[Obstacle],
    ) -> bool:
        """
        检查路径是否畅通
        
        Args:
            start: 起点
            end: 终点
            obstacles: 障碍物列表
            
        Returns:
            clear: 是否畅通
        """
        sx, sy, sz = start
        ex, ey, ez = end
        
        # 路径方向
        dx = ex - sx
        dy = ey - sy
        path_length = np.sqrt(dx ** 2 + dy ** 2)
        
        if path_length < 0.1:
            return True
        
        # 单位方向向量
        ux = dx / path_length
        uy = dy / path_length
        
        for obs in obstacles:
            # 障碍物到路径的距离
            ox = obs.x - sx
            oy = obs.y - sy
            
            # 投影到路径方向
            proj = ox * ux + oy * uy
            
            # 如果投影在路径范围内
            if 0 <= proj <= path_length:
                # 计算垂直距离
                perp_dist = abs(ox * uy - oy * ux)
                
                if perp_dist < obs.radius + self.safety_distance:
                    return False
        
        return True
