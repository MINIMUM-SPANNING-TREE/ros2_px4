"""
航点导航器
管理航点序列，调用 uav/move 服务实现导航
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from typing import List, Tuple, Optional
from dataclasses import dataclass
import time
import threading

from uav_interfaces.srv import Move, Takeoff, Land, Rtl
from uav_interfaces.msg import UavState, UavPose


@dataclass
class Waypoint:
    """航点数据"""
    x: float           # 目标 X 坐标（ENU，米）
    y: float           # 目标 Y 坐标（ENU，米）
    z: float           # 目标 Z 坐标（ENU，米）
    yaw: float = 0.0   # 目标偏航角（弧度）
    speed: float = 1.0  # 飞行速度（m/s）
    tolerance: float = 0.3  # 到达容差（米）
    hold_time: float = 0.0  # 到达后悬停时间（秒）


class WaypointNavigator:
    """
    航点导航器
    
    管理航点队列，通过调用 uav/move 服务控制无人机飞行。
    支持：
    - 航点队列管理
    - 避障中断和恢复
    - 导航状态跟踪
    
    使用方法:
        navigator = WaypointNavigator(node)
        
        # 添加航点
        navigator.add_waypoint(Waypoint(x=5.0, y=0.0, z=2.0))
        navigator.add_waypoint(Waypoint(x=5.0, y=5.0, z=2.0))
        
        # 开始导航
        navigator.start()
        
        # 暂停/恢复
        navigator.pause()
        navigator.resume()
    """
    
    # 导航状态
    STATE_IDLE = 'idle'
    STATE_NAVIGATING = 'navigating'
    STATE_PAUSED = 'paused'
    STATE_AVOIDING = 'avoiding'
    STATE_COMPLETED = 'completed'
    STATE_ERROR = 'error'
    
    def __init__(
        self,
        node: Node,
        default_timeout: float = 60.0,
    ):
        """
        初始化航点导航器
        
        Args:
            node: ROS2 节点
            default_timeout: 默认超时时间（秒）
        """
        self.node = node
        self.logger = node.get_logger()
        self.default_timeout = default_timeout
        
        # 航点队列
        self.waypoints: List[Waypoint] = []
        self.current_index = 0
        
        # 状态
        self.state = self.STATE_IDLE
        self.current_pose: Optional[UavPose] = None
        self.current_state: Optional[UavState] = None
        
        # 避障相关
        self.avoidance_active = False
        self.avoidance_target: Optional[Tuple[float, float, float]] = None
        
        # 服务客户端
        self.move_client = node.create_client(Move, 'uav/move')
        self.takeoff_client = node.create_client(Takeoff, 'uav/takeoff')
        self.land_client = node.create_client(Land, 'uav/land')
        self.rtl_client = node.create_client(Rtl, 'uav/rtl')
        
        # 订阅遥测
        self.pose_sub = node.create_subscription(
            UavPose,
            '/uav/telemetry/pose',
            self._pose_callback,
            10,
        )
        self.state_sub = node.create_subscription(
            UavState,
            '/uav/telemetry/state',
            self._state_callback,
            10,
        )
        
        # 导航线程
        self._nav_thread: Optional[threading.Thread] = None
        self._stop_flag = False
        
        self.logger.info('航点导航器已初始化')
    
    def _pose_callback(self, msg: UavPose):
        """位姿回调"""
        self.current_pose = msg
    
    def _state_callback(self, msg: UavState):
        """状态回调"""
        self.current_state = msg
    
    def add_waypoint(self, waypoint: Waypoint):
        """
        添加航点
        
        Args:
            waypoint: 航点数据
        """
        self.waypoints.append(waypoint)
        self.logger.info(
            f'添加航点 [{len(self.waypoints)}]: '
            f'({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})'
        )
    
    def set_waypoints(self, waypoints: List[Waypoint]):
        """
        设置航点列表
        
        Args:
            waypoints: 航点列表
        """
        self.waypoints = waypoints
        self.current_index = 0
        self.logger.info(f'设置 {len(waypoints)} 个航点')
    
    def clear_waypoints(self):
        """清空航点"""
        self.waypoints.clear()
        self.current_index = 0
        self.logger.info('已清空航点')
    
    def start(self) -> bool:
        """
        开始导航
        
        Returns:
            success: 是否成功启动
        """
        if self.state == self.STATE_NAVIGATING:
            self.logger.warn('导航已在进行中')
            return False
        
        if not self.waypoints:
            self.logger.error('没有航点，无法开始导航')
            return False
        
        if not self.move_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('uav/move 服务不可用')
            return False
        
        self.state = self.STATE_NAVIGATING
        self.current_index = 0
        self._stop_flag = False
        
        # 启动导航线程
        self._nav_thread = threading.Thread(target=self._navigation_loop)
        self._nav_thread.daemon = True
        self._nav_thread.start()
        
        self.logger.info('开始航点导航')
        return True
    
    def stop(self):
        """停止导航"""
        self._stop_flag = True
        self.state = self.STATE_IDLE
        
        if self._nav_thread and self._nav_thread.is_alive():
            self._nav_thread.join(timeout=2.0)
        
        self.logger.info('导航已停止')
    
    def pause(self):
        """暂停导航"""
        if self.state == self.STATE_NAVIGATING:
            self.state = self.STATE_PAUSED
            self.logger.info('导航已暂停')
    
    def resume(self):
        """恢复导航"""
        if self.state == self.STATE_PAUSED:
            self.state = self.STATE_NAVIGATING
            self.logger.info('导航已恢复')
    
    def request_avoidance(self, target: Tuple[float, float, float]):
        """
        请求避障（由外部避障模块调用）
        
        Args:
            target: 避障目标位置
        """
        self.avoidance_active = True
        self.avoidance_target = target
        self.state = self.STATE_AVOIDING
        self.logger.info(f'避障请求: 目标 ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})')
    
    def complete_avoidance(self):
        """完成避障，恢复导航"""
        self.avoidance_active = False
        self.avoidance_target = None
        self.state = self.STATE_NAVIGATING
        self.logger.info('避障完成，恢复导航')
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """获取当前航点"""
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None
    
    def get_progress(self) -> Tuple[int, int]:
        """
        获取导航进度
        
        Returns:
            (current, total): 当前航点索引和总航点数
        """
        return (self.current_index, len(self.waypoints))
    
    def _navigation_loop(self):
        """导航主循环"""
        while not self._stop_flag and self.current_index < len(self.waypoints):
            # 检查状态
            if self.state == self.STATE_PAUSED:
                time.sleep(0.1)
                continue
            
            if self.state == self.STATE_AVOIDING:
                # 避障模式：飞向避障目标
                if self.avoidance_target:
                    self._fly_to_position(self.avoidance_target)
                time.sleep(0.1)
                continue
            
            # 获取当前航点
            waypoint = self.waypoints[self.current_index]
            
            self.logger.info(
                f'导航到航点 [{self.current_index + 1}/{len(self.waypoints)}]: '
                f'({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})'
            )
            
            # 飞向航点
            success = self._fly_to_waypoint(waypoint)
            
            if success:
                self.logger.info(
                    f'到达航点 [{self.current_index + 1}]'
                )
                
                # 悬停等待
                if waypoint.hold_time > 0:
                    self.logger.info(f'悬停 {waypoint.hold_time:.1f} 秒')
                    time.sleep(waypoint.hold_time)
                
                # 下一个航点
                self.current_index += 1
            else:
                self.logger.warn(
                    f'航点 [{self.current_index + 1}] 导航失败'
                )
                # 可以选择跳过或重试
                self.current_index += 1
        
        # 导航完成
        if self.current_index >= len(self.waypoints):
            self.state = self.STATE_COMPLETED
            self.logger.info('所有航点导航完成')
        else:
            self.state = self.STATE_IDLE
    
    def _fly_to_waypoint(self, waypoint: Waypoint) -> bool:
        """
        飞向航点
        
        Args:
            waypoint: 目标航点
            
        Returns:
            success: 是否成功到达
        """
        target = (waypoint.x, waypoint.y, waypoint.z)
        return self._fly_to_position(target, waypoint.yaw, waypoint.tolerance)
    
    def _fly_to_position(
        self,
        target: Tuple[float, float, float],
        yaw: float = 0.0,
        tolerance: float = 0.3,
    ) -> bool:
        """
        飞向目标位置
        
        Args:
            target: 目标位置 (x, y, z)
            yaw: 目标偏航角
            tolerance: 到达容差
            
        Returns:
            success: 是否成功到达
        """
        # 创建服务请求
        req = Move.Request()
        req.x = float(target[0])
        req.y = float(target[1])
        req.z = float(target[2])
        req.yaw = float(yaw)
        
        # 调用服务
        future = self.move_client.call_async(req)
        
        # 等待服务完成
        timeout = self.default_timeout
        start_time = time.time()
        
        while not future.done() and time.time() - start_time < timeout:
            if self._stop_flag:
                return False
            time.sleep(0.1)
        
        if not future.done():
            self.logger.error('uav/move 服务调用超时')
            return False
        
        result = future.result()
        if result and result.success:
            return True
        else:
            msg = result.message if result else '无响应'
            self.logger.warn(f'移动失败: {msg}')
            return False
    
    def takeoff(self, altitude: float = 2.0) -> bool:
        """
        起飞
        
        Args:
            altitude: 目标高度（相对高度，米）
            
        Returns:
            success: 是否成功
        """
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('uav/takeoff 服务不可用')
            return False
        
        req = Takeoff.Request()
        req.relative_alt = float(altitude)
        
        future = self.takeoff_client.call_async(req)
        
        start_time = time.time()
        while not future.done() and time.time() - start_time < 30.0:
            time.sleep(0.1)
        
        if future.done():
            result = future.result()
            if result and result.success:
                self.logger.info(f'起飞成功，高度: {altitude:.1f}m')
                return True
        
        self.logger.error('起飞失败')
        return False
    
    def land(self) -> bool:
        """
        降落
        
        Returns:
            success: 是否成功
        """
        self.stop()
        
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('uav/land 服务不可用')
            return False
        
        req = Land.Request()
        req.timeout = 30.0
        
        future = self.land_client.call_async(req)
        
        start_time = time.time()
        while not future.done() and time.time() - start_time < 35.0:
            time.sleep(0.1)
        
        if future.done():
            result = future.result()
            if result and result.success:
                self.logger.info('降落成功')
                return True
        
        self.logger.error('降落失败')
        return False
    
    def rtl(self) -> bool:
        """
        返航
        
        Returns:
            success: 是否成功
        """
        self.stop()
        
        if not self.rtl_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('uav/rtl 服务不可用')
            return False
        
        req = Rtl.Request()
        req.timeout = 60.0
        
        future = self.rtl_client.call_async(req)
        
        start_time = time.time()
        while not future.done() and time.time() - start_time < 65.0:
            time.sleep(0.1)
        
        if future.done():
            result = future.result()
            if result and result.success:
                self.logger.info('返航成功')
                return True
        
        self.logger.error('返航失败')
        return False


def main(args=None):
    """独立运行测试"""
    rclpy.init(args=args)
    node = Node('waypoint_navigator_test')
    
    navigator = WaypointNavigator(node)
    
    # 等待服务
    node.get_logger().info('等待 uav/move 服务...')
    if not navigator.move_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('uav/move 服务不可用')
        return
    
    # 测试航点
    navigator.add_waypoint(Waypoint(x=0.0, y=0.0, z=2.0))
    navigator.add_waypoint(Waypoint(x=3.0, y=0.0, z=2.0))
    navigator.add_waypoint(Waypoint(x=3.0, y=3.0, z=2.0))
    navigator.add_waypoint(Waypoint(x=0.0, y=3.0, z=2.0))
    navigator.add_waypoint(Waypoint(x=0.0, y=0.0, z=2.0))
    
    # 开始导航
    navigator.start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        navigator.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
