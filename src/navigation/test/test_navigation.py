"""
navigation 包单元测试
"""

import sys
import numpy as np
import importlib.util

# 直接加载模块
def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod

# 加载模块
obstacle_mod = load_module('navigation.obstacle_detector', 'navigation/obstacle_detector.py')
planner_mod = load_module('navigation.local_planner', 'navigation/local_planner.py')

ObstacleDetector = obstacle_mod.ObstacleDetector
Obstacle = obstacle_mod.Obstacle
LocalPlanner = planner_mod.LocalPlanner
VelocityCommand = planner_mod.VelocityCommand


def run_tests():
    """运行所有测试"""
    print('='*50)
    print('运行 navigation 包单元测试')
    print('='*50)

    passed = 0
    failed = 0

    # ==================== ObstacleDetector 测试 ====================
    print('\n--- ObstacleDetector 测试 ---')

    # 测试初始化
    try:
        d = ObstacleDetector()
        assert d.min_height == -1.0 and d.max_height == 3.0
        assert d.min_distance == 0.5 and d.max_distance == 10.0
        print('✅ test_init')
        passed += 1
    except Exception as e:
        print(f'❌ test_init: {e}')
        failed += 1

    # 测试空点云滤波
    try:
        d = ObstacleDetector()
        result = d._filter_points(np.array([]).reshape(0, 3), (0, 0, 0))
        assert len(result) == 0
        print('✅ test_filter_empty')
        passed += 1
    except Exception as e:
        print(f'❌ test_filter_empty: {e}')
        failed += 1

    # 测试高度滤波
    try:
        d = ObstacleDetector(min_height=0.0, max_height=2.0)
        pts = np.array([[1, 0, 0.5], [1, 0, -0.5], [1, 0, 3.0], [1, 0, 1.5]])
        result = d._filter_points(pts, (0, 0, 0))
        assert len(result) == 2
        print('✅ test_filter_height')
        passed += 1
    except Exception as e:
        print(f'❌ test_filter_height: {e}')
        failed += 1

    # 测试距离滤波
    try:
        d = ObstacleDetector(min_distance=1.0, max_distance=5.0)
        pts = np.array([[2, 0, 0], [0.3, 0, 0], [8, 0, 0], [4, 0, 0]])
        result = d._filter_points(pts, (0, 0, 0))
        assert len(result) == 2
        print('✅ test_filter_distance')
        passed += 1
    except Exception as e:
        print(f'❌ test_filter_distance: {e}')
        failed += 1

    # 测试空点云检测
    try:
        obstacles = ObstacleDetector().detect(np.array([]).reshape(0, 3))
        assert len(obstacles) == 0
        print('✅ test_detect_empty')
        passed += 1
    except Exception as e:
        print(f'❌ test_detect_empty: {e}')
        failed += 1

    # 测试单个障碍物聚类
    try:
        d = ObstacleDetector(min_distance=0.1, max_distance=10.0, cluster_tolerance=0.5, min_cluster_size=3)
        pts = np.array([[3, 0, 0], [3.1, 0.1, 0], [3, 0.1, 0], [3.1, 0, 0]])
        obs = d.detect(pts, (0, 0, 0), 0.0)
        assert len(obs) >= 1 and obs[0].distance > 0
        print('✅ test_detect_cluster')
        passed += 1
    except Exception as e:
        print(f'❌ test_detect_cluster: {e}')
        failed += 1

    # 测试多个障碍物检测
    try:
        d = ObstacleDetector(min_distance=0.1, max_distance=10.0, cluster_tolerance=0.5, min_cluster_size=2)
        # 两个分离的障碍物
        pts = np.array([
            [3, 0, 0], [3.1, 0.1, 0],  # 障碍物1
            [0, 5, 0], [0.1, 5.1, 0],  # 障碍物2
        ])
        obs = d.detect(pts, (0, 0, 0), 0.0)
        assert len(obs) >= 2
        print('✅ test_detect_multiple')
        passed += 1
    except Exception as e:
        print(f'❌ test_detect_multiple: {e}')
        failed += 1

    # 测试获取最近障碍物
    try:
        obs_list = [
            Obstacle(x=5, y=0, z=0, distance=5, angle=0, radius=0.5, point_count=10),
            Obstacle(x=2, y=0, z=0, distance=2, angle=0, radius=0.3, point_count=5),
            Obstacle(x=8, y=0, z=0, distance=8, angle=0, radius=0.8, point_count=20),
        ]
        nearest = ObstacleDetector().get_nearest_obstacle(obs_list)
        assert nearest is not None and nearest.distance == 2
        print('✅ test_get_nearest')
        passed += 1
    except Exception as e:
        print(f'❌ test_get_nearest: {e}')
        failed += 1

    # 测试空列表
    try:
        assert ObstacleDetector().get_nearest_obstacle([]) is None
        print('✅ test_get_nearest_empty')
        passed += 1
    except Exception as e:
        print(f'❌ test_get_nearest_empty: {e}')
        failed += 1

    # 测试扇区查询
    try:
        obs_list = [
            Obstacle(x=1, y=1, z=0, distance=1.4, angle=0.78, radius=0.3, point_count=5),
            Obstacle(x=1, y=-1, z=0, distance=1.4, angle=-0.78, radius=0.3, point_count=5),
        ]
        sector = ObstacleDetector().get_obstacles_in_sector(obs_list, 0.78, 0.2)
        assert len(sector) >= 1
        print('✅ test_get_sector')
        passed += 1
    except Exception as e:
        print(f'❌ test_get_sector: {e}')
        failed += 1

    # ==================== LocalPlanner 测试 ====================
    print('\n--- LocalPlanner 测试 ---')

    # 测试初始化
    try:
        p = LocalPlanner()
        assert p.safety_distance == 2.0
        assert p.warning_distance == 3.0
        assert p.max_speed == 2.0
        assert p.sector_count == 36
        print('✅ test_planner_init')
        passed += 1
    except Exception as e:
        print(f'❌ test_planner_init: {e}')
        failed += 1

    # 测试无障碍物规划
    try:
        p = LocalPlanner()
        cmd = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        assert isinstance(cmd, VelocityCommand)
        assert cmd.vx > 0  # 应该向前飞
        print('✅ test_plan_no_obstacles')
        passed += 1
    except Exception as e:
        print(f'❌ test_plan_no_obstacles: {e}')
        failed += 1

    # 测试有障碍物时规划（选择绕行方向）
    try:
        p = LocalPlanner(safety_distance=2.0, warning_distance=3.0)
        obs = [Obstacle(x=3, y=0, z=2, distance=3, angle=0, radius=0.5, point_count=10)]
        cmd = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=obs)
        assert isinstance(cmd, VelocityCommand)
        # 规划器应该选择绕行方向
        print('✅ test_plan_with_obstacle')
        passed += 1
    except Exception as e:
        print(f'❌ test_plan_with_obstacle: {e}')
        failed += 1

    # 测试紧急避障（障碍物在正前方且很近）
    try:
        p = LocalPlanner(safety_distance=2.0, warning_distance=3.0)
        # 障碍物在正前方很近处
        obs = [Obstacle(x=1.0, y=0, z=2, distance=1.0, angle=0, radius=0.5, point_count=10)]
        cmd = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=obs)
        assert isinstance(cmd, VelocityCommand)
        # 紧急避障时应该有明显的转向
        assert abs(cmd.yaw_rate) > 0.1
        print('✅ test_plan_emergency')
        passed += 1
    except Exception as e:
        print(f'❌ test_plan_emergency: {e}')
        failed += 1

    # 测试路径检查 - 畅通
    try:
        p = LocalPlanner(safety_distance=1.0)
        assert p.is_path_clear((0, 0, 0), (5, 0, 0), []) == True
        print('✅ test_path_clear_empty')
        passed += 1
    except Exception as e:
        print(f'❌ test_path_clear_empty: {e}')
        failed += 1

    # 测试路径检查 - 障碍物不在路径上
    try:
        p = LocalPlanner(safety_distance=1.0)
        obs_far = [Obstacle(x=2, y=5, z=0, distance=5, angle=0, radius=0.5, point_count=10)]
        assert p.is_path_clear((0, 0, 0), (5, 0, 0), obs_far) == True
        print('✅ test_path_clear_far')
        passed += 1
    except Exception as e:
        print(f'❌ test_path_clear_far: {e}')
        failed += 1

    # 测试路径检查 - 障碍物在路径上
    try:
        p = LocalPlanner(safety_distance=1.0)
        obs_on_path = [Obstacle(x=2.5, y=0, z=0, distance=2.5, angle=0, radius=0.5, point_count=10)]
        assert p.is_path_clear((0, 0, 0), (5, 0, 0), obs_on_path) == False
        print('✅ test_path_blocked')
        passed += 1
    except Exception as e:
        print(f'❌ test_path_blocked: {e}')
        failed += 1

    # 测试紧急避障方向（左侧障碍物 -> 向右转）
    try:
        p = LocalPlanner()
        obs_left = Obstacle(x=2, y=1, z=0, distance=2.2, angle=0.5, radius=0.5, point_count=10)
        cmd = p._emergency_avoid(obs_left, (0, 0, 0), 0)
        assert cmd.yaw_rate < 0  # 应该向右转
        print('✅ test_emergency_left')
        passed += 1
    except Exception as e:
        print(f'❌ test_emergency_left: {e}')
        failed += 1

    # 测试紧急避障方向（右侧障碍物 -> 向左转）
    try:
        p = LocalPlanner()
        obs_right = Obstacle(x=2, y=-1, z=0, distance=2.2, angle=-0.5, radius=0.5, point_count=10)
        cmd = p._emergency_avoid(obs_right, (0, 0, 0), 0)
        assert cmd.yaw_rate > 0  # 应该向左转
        print('✅ test_emergency_right')
        passed += 1
    except Exception as e:
        print(f'❌ test_emergency_right: {e}')
        failed += 1

    # 测试速度限制
    try:
        p = LocalPlanner(max_speed=1.5)
        cmd = p.plan(goal=(10, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        speed = np.sqrt(cmd.vx**2 + cmd.vy**2)
        assert speed <= 1.6  # 允许小误差
        print('✅ test_speed_limit')
        passed += 1
    except Exception as e:
        print(f'❌ test_speed_limit: {e}')
        failed += 1

    # 测试接近目标时减速
    try:
        p = LocalPlanner(max_speed=2.0)
        # 目标很近
        cmd = p.plan(goal=(0.5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        speed = np.sqrt(cmd.vx**2 + cmd.vy**2)
        assert speed < 1.5  # 应该减速
        print('✅ test_slow_near_goal')
        passed += 1
    except Exception as e:
        print(f'❌ test_slow_near_goal: {e}')
        failed += 1

    # 测试平滑效果
    try:
        p = LocalPlanner(smooth_weight=0.3)
        cmd1 = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        cmd2 = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        cmd3 = p.plan(goal=(5, 0, 2), current_pos=(0, 0, 2), current_yaw=0, obstacles=[])
        # 平滑后应该逐渐趋近目标值
        assert abs(cmd3.vx - cmd2.vx) < abs(cmd2.vx - cmd1.vx) + 0.01
        print('✅ test_smooth_convergence')
        passed += 1
    except Exception as e:
        print(f'❌ test_smooth_convergence: {e}')
        failed += 1

    # ==================== 结果汇总 ====================
    print(f'\n{"="*50}')
    print(f'测试结果: {passed} 通过, {failed} 失败')
    if failed == 0:
        print('🎉 全部通过!')
    return failed == 0


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
