import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, ExtendedState, RCIn
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class UavBase(Node):
    def __init__(self, node_name='uavbase_node'):
        super().__init__(node_name)
        self.current_pose = PoseStamped()
        self.current_state = State() 
        self.current_imu = Imu()
        self.target_pose = PoseStamped()        
        self.current_extended_state = ExtendedState()
        self.current_rc = RCIn()
        self.current_battery = BatteryState()

        
        self.create_subscription(State,         '/mavros/state',               self.state_cb, 10)
        self.create_subscription(PoseStamped,   '/mavros/local_position/pose', self.pose_cb,  10)
        self.create_subscription(Imu,           '/mavros/imu/data',            self.imu_cb,   10)
        self.create_subscription(ExtendedState, '/mavros/extended_state',      self.extended_state_cb, 10)
        self.create_subscription(RCIn,          '/mavros/rc/in',               self.rc_cb,             10)
        self.create_subscription(BatteryState,  '/mavros/battery',             self.battery_cb,        10)



        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.create_timer(0.1, self.timer_cb)
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
    def timer_cb(self):
        if self.get_mode() == "OFFBOARD":
            self.local_pos_pub.publish(self.target_pose)
    def wait_for_connection(self):
        self.get_logger().info("等待飞控连接...")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("飞控已连接！")

    #上锁或解锁的方法
    def arm(self, state=True):
        if self.current_state.armed == state:
            self.get_logger().info(f"已经是 {'解锁' if state else '上锁'} 状态")
            return True
        req = CommandBool.Request()
        req.value = state
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"{'解锁' if state else '上锁'}成功")
            return True
        else:
            self.get_logger().error(f"{'解锁' if state else '上锁'}失败")
            return False

    #设置飞行模式的方法
    def set_mode(self, custom_mode):
        if self.current_state.mode == custom_mode:
            self.get_logger().info(f"已经是 {custom_mode} 模式")
            return True
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
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
    def get_mode(self):
        return self.current_state.mode