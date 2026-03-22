#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import threading
import uuid
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode as MavrosSetMode
from uav_interfaces.srv import Takeoff, Land, Move, Arm, SetMode
from uav_interfaces.msg import UavState, UavPose

# 模块说明：
# 该文件实现 `UavServer` ROS2 节点，提供无人机控制的服务接口（起飞/降落/移动/上锁-解锁/切换模式），
# 并通过 MAVROS 与飞控交互，订阅遥测以维持内部状态，用于决策与发布 setpoint。
# 注意：服务处理器中部分耗时操作会在后台线程执行以避免阻塞节点主循环。

class UavServer(Node):
    def __init__(self):
        super().__init__('uav_controller_service_node')

        # ---- 订阅 TelemetryNode 发布的简化状态 ----
        # 用于接收来自 Telemetry 节点的飞行状态和位姿，用作控制与判断条件
        self.state_sub = self.create_subscription(UavState, '/uav/telemetry/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)
        self.current_state = None
        self.current_pose = None

        # ---- 发布 setpoint ----
        # 发布位置/速度目标给 MAVROS（用于 OFFBOARD 控制）
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ---- MAVROS 服务客户端 ----
        # 创建 MAVROS 服务客户端以执行解锁/上锁与模式切换等命令
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(MavrosSetMode, '/mavros/set_mode')

        self.get_logger().info("UavController 服务节点已就绪")
        while not self.arm_client.service_is_ready():
            self.get_logger().info("等待 Arming 服务可用...")
            time.sleep(0.5)
        while not self.mode_client.service_is_ready():
            self.get_logger().info("等待 SetMode 服务可用...")
            time.sleep(0.5)
        self.get_logger().info("MAVROS 服务已就绪")

        # ---- 内部状态缓存 ----
        # 快速缓存飞机关键状态，减少对遥测的直接查询
        self.is_taking_off = False
        self.is_in_air = False
        self.is_on_ground = False
        self.mode = None
        # 后台任务跟踪: {task_id: {'name':str,'status':'running'|'succeeded'|'failed','result':bool,'message':str}}
        self._tasks = {}

        # 创建对外服务：起飞、降落、移动、上/解锁、切换模式（服务回调内部会在后台线程执行实际动作）
        self.create_service(Takeoff, 'uav/takeoff', self.handle_takeoff)
        self.create_service(Land, 'uav/land', self.handle_land)
        self.create_service(Move, 'uav/move', self.handle_move)
        self.create_service(Arm, 'uav/arm', self.handle_arm)
        self.create_service(SetMode, 'uav/set_mode', self.handle_set_mode)

        self.get_logger().info('UAV 控制服务已就绪：uav/takeoff、uav/land、uav/move、uav/arm、uav/set_mode')

    # ---------------- 回调 ----------------
    def state_cb(self, msg: UavState):
        # 更新内部状态缓存（由 Telemetry 提供），供控制逻辑判断使用
        self.current_state = msg
        self.is_taking_off = (msg.landed_state == 'TAKING_OFF')
        self.is_in_air = (msg.landed_state == 'IN_AIR')
        self.is_on_ground = (msg.landed_state == 'ON_GROUND')
        self.mode = msg.mode

    def pose_cb(self, msg: UavPose):
        # 保存最新位姿（x,y,z,yaw），供 move/hover 等函数使用
        self.current_pose = msg

    # ---------------- 基本 MAVROS 接口 ----------------
    def arm(self, arm: bool) -> bool:
        # 通过 MAVROS 服务发送上锁/解锁命令
        if not self.arm_client.service_is_ready():
            self.get_logger().error("解锁服务不可用")
            return False

        req = CommandBool.Request()
        req.value = arm

        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"{('解锁' if arm else '上锁')} 成功")
            return True
        else:
            self.get_logger().error("解锁命令失败")
            return False

    def set_mode(self, mode: str) -> bool:
        # 使用 MAVROS 的 set_mode 服务切换飞控工作模式
        if not self.mode_client.service_is_ready():
            self.get_logger().error("模式设定服务不可用")
            return False

        req = MavrosSetMode.Request()
        req.custom_mode = mode

        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"模式切换至 {mode}")
            return True
        else:
            self.get_logger().error(f"切换 {mode}模式失败")
            return False

    # ---------------- 高级接口 ----------------
    def hover(self, duration: float = 5.0, mode: str = "OFFBOARD"):
        """
        让无人机悬停一段时间（次代码块没有debug，暂时没有用处）
        :param duration: 悬停时长 (秒)
        :param mode: "OFFBOARD" 或 "LOITER"
        """
        # 悬停策略：LOITER 由飞控保持位置；OFFBOARD 通过本节点持续下发位置 setpoint
        if mode.upper() == "LOITER":
            if self.set_mode("AUTO.LOITER"):
                self.get_logger().info(f"切换到 LOITER 模式，悬停 {duration} 秒")
            else:
                self.get_logger().warn("切换到 LOITER 失败，悬停中止")
                return
            start_time = time.time()
            while time.time() - start_time < duration:
                time.sleep(0.1)

        elif mode.upper() == "OFFBOARD":
            if not self.current_pose:
                self.get_logger().warn("尚无遥测，无法悬停")
                return
            if not self.set_mode("OFFBOARD"):
                self.get_logger().warn("切换到 OFFBOARD 失败")
                return

            self.get_logger().info(f"在 OFFBOARD 模式悬停 {duration} 秒")
            start_time = time.time()
            while time.time() - start_time < duration:
                self.move_to(self.current_pose.x, self.current_pose.y, self.current_pose.z,
                             yaw=self.current_pose.yaw)

        else:
            self.get_logger().warn(f"未知的悬停模式: {mode}")

    def takeoff_auto(self, relative_altitude: float = 2.0):
        """
        使用 AUTO.TAKEOFF 起飞到基于当前高度的目标高度
        :param relative_altitude: 期望相对于当前高度的上升高度
        """
        if self.is_taking_off:
            self.get_logger().warn("起飞已在进行中")
            return False

        if self.is_in_air:
            self.get_logger().warn("已经在空中")
            return False
        # 1️⃣ 等待 telemetry 中获得当前有效高度
        while self.current_pose is None or self.current_pose.z is None:
            self.get_logger().info("等待接收有效高度作为基准...")
            rclpy.spin_once(self, timeout_sec=0.1)

        base_altitude = self.current_pose.z
        target_altitude = base_altitude + relative_altitude
        self.get_logger().info(f"当前基准高度: {base_altitude:.2f} m, 起飞目标: {target_altitude:.2f} m")
        # 2️⃣ 设置模式和解锁
        if self.mode != "AUTO.TAKEOFF":
            if not self.set_mode("AUTO.TAKEOFF"):
                self.get_logger().error("无法切换到 AUTO.TAKEOFF 模式")
                return False
            self.get_logger().info("切换到 AUTO.TAKEOFF 模式")

        if not self.arm(True):
            self.get_logger().error("解锁失败")
            return False
        # 3️⃣ 持续等待升空到目标高度
        while self.current_pose.z < target_altitude * 0.95:
            self.get_logger().info(f"当前高度: {self.current_pose.z:.2f} m, 等待达到目标高度...")
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("✅ 起飞完成，达到目标高度")
        return True

    def move_offboard(self, x: float, y: float, z: float, yaw: float = None) -> bool:
        """
        OFFBOARD 模式移动到目标点
        :param x: 目标 X 坐标
        :param y: 目标 Y 坐标
        :param z: 目标 Z 坐标
        :param yaw: 目标航向（弧度），None 则保持当前航向
        :return: 是否成功到达目标点
        """
        # OFFBOARD 移动流程：等待遥测、预热 setpoint、切换模式、持续下发目标点直至接近
        timeout = 5.0
        # 确保有遥测
        start_time = time.time()
        while self.current_pose is None:
            if time.time() - start_time > timeout:
                self.get_logger().error("移动失败：等待遥测超时")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_pose:
            self.get_logger().warn("无遥测信息")
            return False
        # 预热 OFFBOARD
        for _ in range(50):
            self.move_to(self.current_pose.x, self.current_pose.y, self.current_pose.z,
                         self.current_pose.yaw)
        # 进入 OFFBOARD
        if not self.set_mode("OFFBOARD"):
            self.get_logger().error("无法切换 OFFBOARD 模式")
            return False
        # 持续发布目标点直到达到或超时
        while True:
            self.move_to(float(x), float(y), float(z), yaw)
            rclpy.spin_once(self, timeout_sec=0.1)
            # 计算当前距离
            pose = self.current_pose
            dist = math.sqrt((x - pose.x)**2 + (y - pose.y)**2 + (z - pose.z)**2)
            if dist < 0.3:
                self.get_logger().info(f"✅ 到达目标点 ({x},{y},{z})")
                return True

            if time.time() - start_time > timeout:
                self.get_logger().error(f"❌ 移动超时，未到达目标 ({x},{y},{z})")
                return False

    def move_to(self, x: float, y: float, z: float, yaw: float = None):
        # 构建位置 setpoint（包括由 yaw 计算的四元数）并发布到 MAVROS
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 直接用 TelemetryNode 给的 yaw
        final_yaw = yaw if yaw is not None else (self.current_pose.yaw if self.current_pose else 0.0)
        # 简单 yaw -> 四元数转换（roll=pitch=0）
        qx = 0.0
        qy = 0.0
        qz = math.sin(final_yaw / 2)
        qw = math.cos(final_yaw / 2)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)

    def _run_task(self, name: str, func, *args, **kwargs):
        """在后台线程中运行给定函数并记录状态。
        返回 task_id，可通过内部的 _tasks 字典查询执行状态和结果。
        """
        task_id = str(uuid.uuid4())
        self._tasks[task_id] = {'name': name, 'status': 'running', 'result': None, 'message': ''}
        self.get_logger().info(f"启动后台任务 {name} id={task_id}")
        try:
            ok = func(*args, **kwargs)
            self._tasks[task_id]['result'] = bool(ok)
            self._tasks[task_id]['status'] = 'succeeded' if ok else 'failed'
            self._tasks[task_id]['message'] = f"{name} {('成功' if ok else '失败')}"
            self.get_logger().info(f"后台任务 {name} id={task_id} 完成: {self._tasks[task_id]['message']}")
        except Exception as e:
            self._tasks[task_id]['result'] = False
            self._tasks[task_id]['status'] = 'failed'
            self._tasks[task_id]['message'] = f"异常: {e}"
            self.get_logger().error(f"后台任务 {name} id={task_id} 异常: {e}")
        return task_id

    def land_auto(self, timeout: float = 30.0) -> bool:
        """
        使用 AUTO.LAND 降落并等待落地
        :param timeout: 最大等待时间 (秒)
        :return: 是否成功降落
        """
        # 降落流程：切换到 AUTO.LAND 并等待飞控反馈已落地或超时
        if not self.set_mode("AUTO.LAND"):
            self.get_logger().error("模式切换至 AUTO.LAND 失败")
            return False

        self.get_logger().info("模式切换至 AUTO.LAND，等待降落...")

        start_time = self.get_clock().now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # 检查是否落地
            if self.is_on_ground:
                self.get_logger().info("✅ 无人机已安全落地")
                return True
            # 超时判断
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error("❌ 降落超时")
                return False

    # ---------------- 服务处理回调 ----------------
    def handle_takeoff(self, request, response):
        """服务处理器：uav/takeoff
        请求: Takeoff.Request.relative_alt (float64)
        响应: Takeoff.Response.success (bool), message (string)
        """
        # 接收到起飞服务请求后，异步在后台线程中执行起飞操作
        self.get_logger().info(f"收到起飞请求: relative_alt={request.relative_alt}")
        # 非阻塞：在后台线程执行起飞
        t = threading.Thread(target=self._run_task, args=("takeoff", self.takeoff_auto, request.relative_alt), daemon=True)
        t.start()
        response.success = True
        response.message = "起飞已启动"
        return response

    def handle_land(self, request, response):
        """服务处理器：uav/land
        请求: Land.Request.timeout (float64)
        """
        self.get_logger().info(f"收到降落请求: timeout={request.timeout}")
        t = threading.Thread(target=self._run_task, args=("land", self.land_auto, request.timeout), daemon=True)
        t.start()
        response.success = True
        response.message = "降落已启动"
        return response

    def handle_move(self, request, response):
        """服务处理器：uav/move
        请求: Move.Request.x, y, z, yaw
        """
        self.get_logger().info(f"收到移动请求: x={request.x}, y={request.y}, z={request.z}, yaw={request.yaw}")
        t = threading.Thread(target=self._run_task, args=("move", self.move_offboard, request.x, request.y, request.z, request.yaw), daemon=True)
        t.start()
        response.success = True
        response.message = "移动已启动"
        return response

    def handle_arm(self, request, response):
        """服务处理器：uav/arm
        请求: Arm.Request.arm (bool)
        """
        self.get_logger().info(f"收到上/解锁请求: arm={request.arm}")
        t = threading.Thread(target=self._run_task, args=("arm", self.arm, request.arm), daemon=True)
        t.start()
        response.success = True
        response.message = "上/解锁操作已启动"
        return response

    def handle_set_mode(self, request, response):
        """服务处理器：uav/set_mode
        请求: SetMode.Request.mode (string)
        """
        self.get_logger().info(f"收到设置模式请求: mode={request.mode}")
        t = threading.Thread(target=self._run_task, args=("set_mode", self.set_mode, request.mode), daemon=True)
        t.start()
        response.success = True
        response.message = "设置模式已启动"
        return response


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
