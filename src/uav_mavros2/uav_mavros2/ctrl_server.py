#!/usr/bin/env python3
"""
UAV 控制服务节点
提供起飞/降落/移动/解锁/切模式 服务接口
直接复用 UavBase 的 MAVROS 交互（订阅、发布、arm、set_mode）
"""
import rclpy
import time
import math
import threading
import uuid
from geometry_msgs.msg import PoseStamped
from uav_interfaces.srv import Takeoff, Land, Move, Arm, SetMode
from uav_mavros2 import uavbase


class UavServer(uavbase.UavBase):
    def __init__(self):
        super().__init__('uav_controller_service_node')

        self.is_taking_off = False
        self.is_in_air = False
        self.is_on_ground = False
        self._tasks = {}

        # 对外服务
        self.create_service(Takeoff, 'uav/takeoff', self.handle_takeoff)
        self.create_service(Land, 'uav/land', self.handle_land)
        self.create_service(Move, 'uav/move', self.handle_move)
        self.create_service(Arm, 'uav/arm', self.handle_arm)
        self.create_service(SetMode, 'uav/set_mode', self.handle_set_mode)

        self.get_logger().info('UAV 控制服务已就绪：uav/takeoff uav/land uav/move uav/arm uav/set_mode')

    # ====================== 工具 ======================

    def _get_yaw(self):
        q = self.current_pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _get_pos(self):
        p = self.current_pose.pose.position
        return p.x, p.y, p.z

    def _make_pose(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        return msg

    # ====================== 核心控制 ======================

    def do_takeoff(self, alt=2.0) -> bool:
        x, y, z = self._get_pos()
        target = z + alt
        self.get_logger().info(f'起飞: {z:.1f}m → {target:.1f}m')

        if not self.set_mode('AUTO.TAKEOFF'):
            return False
        if not self.arm(True):
            return False

        deadline = time.time() + 20.0
        while time.time() < deadline:
            time.sleep(0.2)
            if self.current_pose.pose.position.z >= target * 0.95:
                self.get_logger().info('起飞完成')
                return True
        self.get_logger().error('起飞超时')
        return False

    def do_move(self, x, y, z, yaw=None) -> bool:
        x0, y0, z0 = self._get_pos()
        target_yaw = yaw if yaw is not None else self._get_yaw()
        self.get_logger().info(f'移动: ({x0:.1f},{y0:.1f},{z0:.1f}) → ({x},{y},{z})')

        # 预热: 直接发布 50 个 setpoint（OFFBOARD 切换前必须有 setpoint 流）
        warmup_pose = self._make_pose(x0, y0, z0, target_yaw)
        for _ in range(50):
            warmup_pose.header.stamp = self.get_clock().now().to_msg()
            self.local_pos_pub.publish(warmup_pose)
            time.sleep(0.02)

        if not self.set_mode('OFFBOARD'):
            self.get_logger().error('OFFBOARD 切换失败')
            return False

        # 到达循环: 更新 target_pose，由 uavbase timer_cb 持续发布
        deadline = time.time() + 30.0
        while time.time() < deadline:
            self.target_pose = self._make_pose(x, y, z, target_yaw)
            time.sleep(0.05)
            cx, cy, cz = self._get_pos()
            dist = math.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
            if dist < 0.3:
                self.get_logger().info(f'到达 ({x},{y},{z})')
                return True

        self.get_logger().error(f'移动超时 ({x},{y},{z})')
        return False

    def do_land(self, timeout=30.0) -> bool:
        if not self.set_mode('AUTO.LAND'):
            return False
        self.get_logger().info('降落中...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.2)
            if self.current_extended_state.landed_state == 1:
                self.get_logger().info('降落完成')
                return True
        self.get_logger().error('降落超时')
        return False

    # ====================== 后台任务 ======================

    def _run_task(self, name, func, *args):
        task_id = str(uuid.uuid4())[:8]
        self._tasks[task_id] = {'name': name, 'status': 'running'}
        self.get_logger().info(f'[{task_id}] 启动 {name}')
        try:
            ok = func(*args)
            self._tasks[task_id]['status'] = 'succeeded' if ok else 'failed'
            self.get_logger().info(f'[{task_id}] {name} {"成功" if ok else "失败"}')
        except Exception as e:
            self._tasks[task_id]['status'] = 'failed'
            self.get_logger().error(f'[{task_id}] {name} 异常: {e}')

    def _spawn(self, name, func, *args):
        threading.Thread(target=self._run_task, args=(name, func) + args, daemon=True).start()

    # ====================== 服务回调 ======================

    def handle_takeoff(self, req, resp):
        self.get_logger().info(f'起飞请求: alt={req.relative_alt}')
        self._spawn('takeoff', self.do_takeoff, req.relative_alt)
        resp.success = True
        resp.message = '起飞已启动'
        return resp

    def handle_land(self, req, resp):
        self.get_logger().info(f'降落请求: timeout={req.timeout}')
        self._spawn('land', self.do_land, req.timeout)
        resp.success = True
        resp.message = '降落已启动'
        return resp

    def handle_move(self, req, resp):
        self.get_logger().info(f'移动请求: ({req.x},{req.y},{req.z})')
        self._spawn('move', self.do_move, req.x, req.y, req.z, req.yaw)
        resp.success = True
        resp.message = '移动已启动'
        return resp

    def handle_arm(self, req, resp):
        self.get_logger().info(f'解锁请求: arm={req.arm}')
        self._spawn('arm', self.arm, req.arm)
        resp.success = True
        resp.message = '解锁已启动'
        return resp

    def handle_set_mode(self, req, resp):
        self.get_logger().info(f'切模式请求: {req.mode}')
        self._spawn('set_mode', self.set_mode, req.mode)
        resp.success = True
        resp.message = '模式切换已启动'
        return resp
    def update(self):
        self.target_pose = self.current_pose

def main(args=None):
    rclpy.init(args=args)
    node = UavServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
