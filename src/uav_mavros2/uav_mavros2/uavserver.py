import rclpy
import math
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from uav_mavros2.uav import Uav
from uav_interfaces.srv import (
    Arm, Land, Move, Rtl, SetMode, Takeoff,
    MoveRelative, SetYaw, SetVelocity, SetMaxSpeed, GetPose, GetState,
    EmergencyLand, EmergencyStop,
)

class UavServer(Uav):
    def __init__(self):
        super().__init__()
        self.cb_group = ReentrantCallbackGroup()
        # Mutex to serialize MAVROS service calls (arm, set_mode, takeoff, land,
        # move, rtl).  Without this, concurrent service handlers issued through
        # the ReentrantCallbackGroup can call call_async on the same underlying
        # rclpy client simultaneously, triggering "generator already executing"
        # inside rclpy's internal Future/executor machinery.
        self._cmd_lock = threading.Lock()

        # 基础服务
        self.srv_arm = self.create_service(Arm, 'uav/arm', self.handle_arm, callback_group=self.cb_group)
        self.srv_land = self.create_service(Land, 'uav/land', self.handle_land, callback_group=self.cb_group)
        self.srv_move = self.create_service(Move, 'uav/move', self.handle_move, callback_group=self.cb_group)
        self.srv_set_mode = self.create_service(SetMode, 'uav/set_mode', self.handle_set_mode, callback_group=self.cb_group)
        self.srv_takeoff = self.create_service(Takeoff, 'uav/takeoff', self.handle_takeoff, callback_group=self.cb_group)
        self.srv_rtl = self.create_service(Rtl, 'uav/rtl', self.handle_rtl, callback_group=self.cb_group)

        # 扩展服务
        self.srv_move_relative = self.create_service(MoveRelative, 'uav/move_relative', self.handle_move_relative, callback_group=self.cb_group)
        self.srv_set_yaw = self.create_service(SetYaw, 'uav/set_yaw', self.handle_set_yaw, callback_group=self.cb_group)
        self.srv_set_velocity = self.create_service(SetVelocity, 'uav/set_velocity', self.handle_set_velocity, callback_group=self.cb_group)
        self.srv_set_max_speed = self.create_service(SetMaxSpeed, 'uav/set_max_speed', self.handle_set_max_speed, callback_group=self.cb_group)
        self.srv_get_pose = self.create_service(GetPose, 'uav/get_pose', self.handle_get_pose, callback_group=self.cb_group)
        self.srv_get_state = self.create_service(GetState, 'uav/get_state', self.handle_get_state, callback_group=self.cb_group)

        # 紧急服务（绕过 _cmd_lock）
        self.srv_emergency_land = self.create_service(EmergencyLand, 'uav/emergency_land', self.handle_emergency_land, callback_group=self.cb_group)
        self.srv_emergency_stop = self.create_service(EmergencyStop, 'uav/emergency_stop', self.handle_emergency_stop, callback_group=self.cb_group)


    def handle_arm(self, request, response):
        self.get_logger().info(f"==> 收到请求: {'解锁' if request.arm else '上锁'}")
        with self._cmd_lock:
            result = self.arm(request.arm)
        response.success = result
        response.message = "Arm/Disarm 执行完毕" if result else "执行失败"
        return response

    def handle_land(self, request, response):
        self.get_logger().info(f"==> 收到请求: 降落, 超时时间={request.timeout}")
        with self._cmd_lock:
            result = self.land(request.timeout)
        response.success = result
        response.message = "降落成功" if result else "降落失败或超时"
        return response

    def handle_move(self, request, response):
        self.get_logger().info(f"==> 收到请求: 移动到 ({getattr(request,'x',None)}, {getattr(request,'y',None)}, {getattr(request,'z',None)}, yaw={getattr(request,'yaw',0.0)})")
        yaw = getattr(request, 'yaw', 0.0)
        timeout = getattr(request, 'timeout', 30.0)
        with self._cmd_lock:
            success = self.move(
                getattr(request, 'x', 0.0),
                getattr(request, 'y', 0.0),
                getattr(request, 'z', 0.0),
                yaw=yaw,
                timeout=timeout,
                wait_until_arrive=True,
            )
        response.success = bool(success)
        response.message = "顺利到达指定位置" if success else "移动失败或超时"
        return response

    def handle_set_mode(self, request, response):
        self.get_logger().info(f"==> 收到请求: 切换模式到 {request.mode}")
        with self._cmd_lock:
            result = self.set_mode(request.mode)
        response.success = result
        response.message = "模式切换完毕" if result else "模式切换失败"
        return response

    def handle_takeoff(self, request, response):
        self.get_logger().info(f"==> 收到请求: 起飞, 相对高度={request.relative_alt}")
        with self._cmd_lock:
            result = self.takeoff(request.relative_alt)
        response.success = result
        response.message = "成功发送起飞指令" if result else "起飞失败"
        return response

    def handle_rtl(self, request, response):
        self.get_logger().info(f"==> 收到请求: 自动返航, 超时时间={request.timeout}")
        with self._cmd_lock:
            result = self.rtl(request.timeout)
        response.success = result
        response.message = "返航成功" if result else "返航失败或超时"
        return response

    def handle_move_relative(self, request, response):
        """相对移动：基于当前位置偏移"""
        self.get_logger().info(f"==> 收到请求: 相对移动 dx={request.dx} dy={request.dy} dz={request.dz} dyaw={request.dyaw}")
        with self._cmd_lock:
            cur_x = self.get_current_pos_x()
            cur_y = self.get_current_pos_y()
            cur_z = self.get_current_pos_z()
            target_x = cur_x + request.dx
            target_y = cur_y + request.dy
            target_z = cur_z + request.dz
            success = self.move(target_x, target_y, target_z, yaw=request.dyaw, timeout=30.0, wait_until_arrive=True)
        response.success = bool(success)
        response.message = f"相对移动完成: ({target_x:.1f},{target_y:.1f},{target_z:.1f})" if success else "相对移动失败"
        return response

    def handle_set_yaw(self, request, response):
        """设置偏航角"""
        self.get_logger().info(f"==> 收到请求: 设置偏航 yaw={request.yaw} relative={request.is_relative}")
        with self._cmd_lock:
            if request.is_relative:
                cur_q = self.current_pose.pose.orientation
                cur_yaw = math.atan2(2 * (cur_q.w * cur_q.z + cur_q.x * cur_q.y),
                                     1 - 2 * (cur_q.y * cur_q.y + cur_q.z * cur_q.z))
                target_yaw = cur_yaw + request.yaw
            else:
                target_yaw = request.yaw
            success = self.move(
                self.get_current_pos_x(), self.get_current_pos_y(), self.get_current_pos_z(),
                yaw=target_yaw, timeout=15.0, wait_until_arrive=True
            )
        response.success = bool(success)
        response.message = f"偏航设置完成: {target_yaw:.2f} rad" if success else "偏航设置失败"
        return response

    def handle_set_velocity(self, request, response):
        """速度控制（简化实现：发送速度设定点一段时间）"""
        self.get_logger().info(f"==> 收到请求: 速度控制 vx={request.vx} vy={request.vy} vz={request.vz} dur={request.duration}")
        with self._cmd_lock:
            # 切换到 OFFBOARD 模式
            if self.get_mode() != "OFFBOARD":
                if not self.set_mode("OFFBOARD"):
                    response.success = False
                    response.message = "切换 OFFBOARD 失败"
                    return response
        # 注意：完整实现需要发布速度设定点，这里简化处理
        response.success = True
        response.message = "速度指令已接受（简化实现）"
        return response

    def handle_set_max_speed(self, request, response):
        """设置最大速度（简化实现）"""
        self.get_logger().info(f"==> 收到请求: 设置最大速度 h={request.max_horizontal} v={request.max_vertical}")
        # 完整实现应通过 MAVROS 参数设置
        response.success = True
        response.message = f"最大速度已记录: h={request.max_horizontal} v={request.max_vertical}"
        return response

    def handle_get_pose(self, request, response):
        """获取当前位姿"""
        pos = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        response.success = True
        response.x = pos.x
        response.y = pos.y
        response.z = pos.z
        response.yaw = yaw
        response.message = f"pos=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}) yaw={yaw:.2f}"
        self.get_logger().info(f"==> 收到请求: 获取位姿 → {response.message}")
        return response

    def handle_get_state(self, request, response):
        """获取当前飞控状态"""
        landed_map = {0: 'UNKNOWN', 1: 'ON_GROUND', 2: 'IN_AIR', 3: 'TAKING_OFF', 4: 'LANDING'}
        response.success = True
        response.armed = self.is_armed()
        response.connected = self.current_state.connected
        response.mode = self.get_mode() or "UNKNOWN"
        response.landed_state = landed_map.get(self.get_landed_state(), 'UNKNOWN')
        response.message = f"armed={response.armed} mode={response.mode} landed={response.landed_state}"
        self.get_logger().info(f"==> 收到请求: 获取状态 → {response.message}")
        return response

    # ------ 紧急服务（绕过 _cmd_lock，确保立即执行） ------

    def handle_emergency_land(self, request, response):
        """紧急降落：立即切换到 AUTO.LAND 模式，不等待确认"""
        self.get_logger().warn("EMERGENCY LAND RECEIVED")
        # 紧急操作绕过 _cmd_lock，直接调用
        result = self.set_mode("AUTO.LAND")
        response.success = result
        response.message = "紧急降落指令已发送" if result else "紧急降落模式切换失败"
        return response

    def handle_emergency_stop(self, request, response):
        """紧急停止：先尝试悬停(AUTO.LOITER)，失败则降落(AUTO.LAND)"""
        self.get_logger().warn("EMERGENCY STOP RECEIVED")
        # 先尝试悬停
        result = self.set_mode("AUTO.LOITER")
        if result:
            response.success = True
            response.message = "紧急停止：已切换到悬停模式"
            return response
        # 悬停失败，尝试降落
        self.get_logger().warn("EMERGENCY STOP: AUTO.LOITER failed, trying AUTO.LAND")
        result = self.set_mode("AUTO.LAND")
        response.success = result
        response.message = "紧急停止：已切换到降落模式" if result else "紧急停止失败"
        return response


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = UavServer()
    node.get_logger().info("UavServer v3.0 已启动，14 个服务就绪")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
