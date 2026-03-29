import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import State, ExtendedState, RCIn, HomePosition
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

class UavBase(Node):
    def __init__(self, node_name='uavbase_node'):
        super().__init__(node_name)
        self.common_group = ReentrantCallbackGroup()  # 共享的回调组，允许并行处理多个请求
        self.current_pose = PoseStamped()
        self.current_state = State() 
        self.current_imu = Imu()
        self.target_pose = PoseStamped()        
        self.current_extended_state = ExtendedState()
        self.current_rc = RCIn()
        self.current_battery = BatteryState()
        self.current_home = HomePosition()
        self.home_received = False

        
        # 明确指定与 MAVROS 端兼容的 QoS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(State,         '/mavros/state',               self.state_cb, qos)
        self.create_subscription(PoseStamped,   '/mavros/local_position/pose', self.pose_cb,  qos)
        self.create_subscription(Imu,           '/mavros/imu/data',            self.imu_cb,   qos)
        self.create_subscription(ExtendedState, '/mavros/extended_state',      self.extended_state_cb, qos)
        self.create_subscription(RCIn,          '/mavros/rc/in',               self.rc_cb,             qos)
        self.create_subscription(BatteryState,  '/mavros/battery',             self.battery_cb,        qos)
        self.create_subscription(HomePosition,  '/mavros/home_position/home',  self.home_cb,           qos)

        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.create_timer(0.1, self.timer_cb)

    def _wait_future(self, future, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok() and not future.done() and (time.time() - start) < float(timeout_sec):
            time.sleep(0.05)
        return future.done()
    #回调更新当前状态、位置和IMU数据
    def state_cb(self, msg):  
        self.current_state = msg
    def pose_cb(self,  msg):  
        self.current_pose = msg
    def imu_cb(self,   msg):  
        self.current_imu  = msg  
    def extended_state_cb(self, msg):
        self.current_extended_state = msg
    def rc_cb(self, msg):
        self.current_rc = msg
    def battery_cb(self, msg):
        self.current_battery = msg
    def home_cb(self, msg):
        self.current_home = msg
        self.home_received = True
    def timer_cb(self):
        if self.get_mode() == "OFFBOARD":
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.local_pos_pub.publish(self.target_pose)
    def wait_for_connection(self):
        self.get_logger().info("等待飞控连接...")
        while rclpy.ok() and not self.current_state.connected:
            time.sleep(0.05)
        self.get_logger().info("飞控已连接！")

    #上锁或解锁的方法
    def arm(self, state=True):
        if self.current_state.armed == state:
            self.get_logger().info(f"已经是 {'解锁' if state else '上锁'} 状态")
            return True
        if not self.arming_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("/mavros/cmd/arming 服务不可用")
            return False
        req = CommandBool.Request()
        req.value = state
        future = self.arming_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self.get_logger().error(f"{'解锁' if state else '上锁'}失败: 服务响应超时")
            return False
        result = future.result()
        if result is None:
            self.get_logger().error(f"{'解锁' if state else '上锁'}失败: 无返回结果")
            return False
        if result.success:
            self.get_logger().info(f"{'解锁' if state else '上锁'}成功")
            return True
        else:
            self.get_logger().error(f"{'解锁' if state else '上锁'}失败: result={getattr(result, 'result', None)}")
            return False

    #设置飞行模式的方法
    def set_mode(self, custom_mode):
        if self.current_state.mode == custom_mode:
            self.get_logger().info(f"已经是 {custom_mode} 模式")
            return True
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self.get_logger().error(f"设置 {custom_mode} 模式失败: 服务响应超时")
            return False
        result = future.result()
        if result is not None and result.mode_sent:
            self.get_logger().info(f"设置 {custom_mode} 模式成功")
            return True
        else:
            self.get_logger().error(f"设置 {custom_mode} 模式失败")
            return False
        
    #获得当前状态、位置和IMU数据的方法
    def get_current_pos(self):
        return self.current_pose
    def get_current_pos_x(self):
        return self.current_pose.pose.position.x
    def get_current_pos_y(self):
        return self.current_pose.pose.position.y
    def get_current_pos_z(self):
        return self.current_pose.pose.position.z
    def get_target_pos(self):
        return self.target_pose
    def get_target_pos_x(self):
        return self.target_pose.pose.position.x
    def get_target_pos_y(self):
        return self.target_pose.pose.position.y
    def get_target_pos_z(self):
        return self.target_pose.pose.position.z
    def set_target_pos_x(self, x):
        self.target_pose.pose.position.x = float(x)
    def set_target_pos_y(self, y):
        self.target_pose.pose.position.y = float(y)
    def set_target_pos_z(self, z):
        self.target_pose.pose.position.z = float(z)
    def set_target_pos(self, x, y, z):
        self.target_pose.pose.position.x = float(x)
        self.target_pose.pose.position.y = float(y)
        self.target_pose.pose.position.z = float(z)
    def get_current_state(self):
        return self.current_state
    def get_current_imu(self):
        return self.current_imu
    def get_flight_mode(self):
        return self.current_state.mode
    def is_armed(self):
        return self.current_state.armed
    def get_extended_state(self):
        return self.current_extended_state
    def get_landed_state(self):
        #获取着陆探测器状态: 0:UNDEFINED, 1:ON_GROUND, 2:IN_AIR, 3:TAKEOFF, 4:LANDING
        return self.current_extended_state.landed_state
    def get_vtol_state(self):
        #获取垂直起降状态: 0:UNDEFINED, 1:TRANSITION_TO_FW, 2:TRANSITION_TO_MC, 3:MC, 4:FW
        return self.current_extended_state.vtol_state
    def get_rc_data(self):
        return self.current_rc
    def get_battery_data(self):
        return self.current_battery
    #获取 home 位置信息的方法
    def get_home_position(self):
        return self.current_home
    def is_home_ready(self):
        return self.home_received
    def get_home_pos_x(self):
        return self.current_home.position.x
    def get_home_pos_y(self):
        return self.current_home.position.y
    def get_home_pos_z(self):
        return self.current_home.position.z
    def get_mode(self):
        return self.current_state.mode