# Lazy imports to avoid ROS dependency at module level
# Users should import specific modules directly:
#   from navigation.obstacle_detector import ObstacleDetector
#   from navigation.local_planner import LocalPlanner

__all__ = [
    'ObstacleDetector',
    'LocalPlanner',
    'WaypointNavigator',
    'NavigationNode',
]
