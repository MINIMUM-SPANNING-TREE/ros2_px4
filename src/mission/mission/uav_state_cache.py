#!/usr/bin/env python3
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, HomePosition, State
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class UavStateCache(Node):
    """订阅 MAVROS 原始话题并缓存最新状态。"""

    def __init__(self):
        super().__init__('mission_state_cache')

        self.current_pose: Optional[PoseStamped] = None
        self.current_state: Optional[State] = None
        self.current_extended_state: Optional[ExtendedState] = None
        self.current_home: Optional[HomePosition] = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self._extended_state_cb, qos)
        self.create_subscription(HomePosition, '/mavros/home_position/home', self._home_cb, qos)

    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _extended_state_cb(self, msg: ExtendedState):
        self.current_extended_state = msg

    def _home_cb(self, msg: HomePosition):
        self.current_home = msg

    def has_pose(self) -> bool:
        return self.current_pose is not None

    def pose_xyz(self):
        if self.current_pose is None:
            return None
        p = self.current_pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def yaw_from_pose(self) -> Optional[float]:
        if self.current_pose is None:
            return None
        q = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_connected(self) -> bool:
        return bool(self.current_state.connected) if self.current_state is not None else False
