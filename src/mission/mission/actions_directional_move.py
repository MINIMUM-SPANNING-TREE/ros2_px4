#!/usr/bin/env python3

from mission.actions_move import MoveClientAction
from mission.uav_state_cache import UavStateCache


class DirectionalMoveAction:
    DIRECTION_MAP = {
        'forward': (1.0, 0.0, 0.0),
        'backward': (-1.0, 0.0, 0.0),
        'left': (0.0, 1.0, 0.0),
        'right': (0.0, -1.0, 0.0),
        'up': (0.0, 0.0, 1.0),
        'down': (0.0, 0.0, -1.0),
    }

    def __init__(self, state_cache: UavStateCache, move_action: MoveClientAction):
        self.state_cache = state_cache
        self.move_action = move_action

    def execute(self, direction: str, distance: float, yaw: float = None, timeout: float = None):
        direction = direction.lower().strip()
        if direction not in self.DIRECTION_MAP:
            return False, f'不支持的方向: {direction}'

        pose = self.state_cache.pose_xyz()
        if pose is None:
            return False, '尚未收到位置数据，无法执行方向移动'

        if distance <= 0.0:
            return False, '移动距离必须大于 0'

        x, y, z = pose
        dx, dy, dz = self.DIRECTION_MAP[direction]
        tx = x + dx * float(distance)
        ty = y + dy * float(distance)
        tz = z + dz * float(distance)

        final_yaw = self.state_cache.yaw_from_pose() if yaw is None else float(yaw)
        if final_yaw is None:
            final_yaw = 0.0

        return self.move_action.execute(tx, ty, tz, yaw=final_yaw, timeout=timeout)
