#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode
from uav_interfaces.msg import UavState, UavPose



class UavController(Node):
    def __init__(self):
        super().__init__('uav_controller_node')

        # ---- 订阅 TelemetryNode 发布的简化状态 ----
        self.state_sub = self.create_subscription(UavState, '/uav/telemetry/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(UavPose, '/uav/telemetry/pose', self.pose_cb, 10)
        self.current_state = None
        self.current_pose = None

        # ---- 发布 setpoint ----
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ---- MAVROS 服务客户端 ----
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info("UavController 节点已就绪")
        
        while not self.arm_client.service_is_ready():
            self.get_logger().info("等待 Arming 服务可用...")
            time.sleep(0.5)
        while not self.mode_client.service_is_ready():
            self.get_logger().info("等待 SetMode 服务可用...")
            time.sleep(0.5)
        self.get_logger().info("MAVROS 服务已就绪")
        
        # ----内部状态缓存----
        self.is_taking_off = False
        self.is_in_air = False  
        self.is_on_ground = False
        self.mode = None

    # ---------------- 回调 ----------------
    def state_cb(self, msg: UavState):
        self.current_state = msg
        self.is_taking_off = (msg.landed_state == 'TAKING_OFF')
        self.is_in_air = (msg.landed_state == 'IN_AIR')
        self.is_on_ground = (msg.landed_state == 'ON_GROUND')
        self.mode = msg.mode
    def pose_cb(self, msg: UavPose):
        self.current_pose = msg
        

    # ---------------- 基本 MAVROS 接口 ----------------
    def arm(self, arm: bool) -> bool:
        if not self.arm_client.service_is_ready():
            self.get_logger().error("解锁服务不可用")
            return False

        req = CommandBool.Request()
        req.value = arm

        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"{'Armed' if arm else 'Disarmed'} 成功")
            return True
        else:
            self.get_logger().error("解锁命令失败")
            return False

    def set_mode(self, mode: str) -> bool:
        if not self.mode_client.service_is_ready():
            self.get_logger().error("模式设定服务不可用")
            return False

        req = SetMode.Request()
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
        if mode.upper() == "LOITER":
            # 切换到 LOITER / AUTO.LOITER 模式
            if self.set_mode("AUTO.LOITER"):
                self.get_logger().info(f"Switched to LOITER mode for hovering {duration} sec")
            else:
                self.get_logger().warn("Failed to switch to LOITER, hover aborted")
                return
            # 等待悬停
            start_time = time.time()
            while time.time() - start_time < duration:
                time.sleep(0.1)

        elif mode.upper() == "OFFBOARD":
            # OFFBOARD模式悬停，需要持续发布当前位置
            if not self.current_pose:
                self.get_logger().warn("No telemetry yet, cannot hover")
                return
            if not self.set_mode("OFFBOARD"):
                self.get_logger().warn("Failed to switch to OFFBOARD mode")
                return

            self.get_logger().info(f"Hovering in OFFBOARD mode for {duration} sec")
            start_time = time.time()
            while time.time() - start_time < duration:
                # 发布当前位置保持悬停
                self.move_to(self.current_pose.x, self.current_pose.y, self.current_pose.z,
                             yaw=self.current_pose.yaw)

        else:
            self.get_logger().warn(f"Unknown hover mode: {mode}")

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

    def land_auto(self, timeout: float = 30.0) -> bool:
        """
        使用 AUTO.LAND 降落并等待落地
        :param timeout: 最大等待时间 (秒)
        :return: 是否成功降落
        """
        if not self.set_mode("AUTO.LAND"):
            self.get_logger().error("模式切换至 AUTO.LAND 失败")
            return False

        self.get_logger().info("模式切换至 AUTO.LAND，等待降落...")

        start_time = self.get_clock().now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)  # 持续接收 Telemetry

            # 检查是否落地
            if self.is_on_ground:
                self.get_logger().info("✅ 无人机已安全落地")
                return True

            # 超时判断
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error("❌ 降落超时")
                return False


# ---------------- 节点入口 ----------------
def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = UavController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
