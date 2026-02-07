#!/usr/bin/env python3
"""
mavros_telemetry_node.py
持续监听MAVROS状态,发布简化遥测数据
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# MAVROS消息
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import PoseStamped, TwistStamped

# 自定义消息（你需要创建这些消息类型）
from uav_interfaces.msg import UavState, UavPose


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('mavros_telemetry_node')
        
        # QoS配置：传感器数据用BestEffort更实时，但这里用Reliable确保不丢
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ========== 订阅MAVROS原始话题 ==========
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.on_state, qos)
        self.sub_extended = self.create_subscription(
            ExtendedState, '/mavros/extended_state', self.on_extended, qos)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose, qos)
        self.sub_vel = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity', self.on_velocity, qos)
        
        # ========== 发布简化遥测数据 ==========
        self.pub_state = self.create_publisher(UavState, '/uav/telemetry/state', 10)
        self.pub_pose = self.create_publisher(UavPose, '/uav/telemetry/pose', 10)    
        
        # ========== 内部状态缓存 ==========
        self.raw_state = None
        self.raw_extended = None
        self.current_pose = None
        self.get_logger().info('Telemetry node initialized')
        self.timer = self.create_timer(0.1, self.publish_telemetry)
    
    # ========== MAVROS回调：原始数据存储 ==========
    def on_state(self, msg: State):
        """飞控连接状态、解锁状态、飞行模式"""
        self.raw_state = msg
        # 可在这里做连接断开告警
    
    def on_extended(self, msg: ExtendedState):
        """扩展状态：起飞/降落状态"""
        self.raw_extended = msg
    
    def on_pose(self, msg: PoseStamped):
        """本地位置 ENU坐标系"""
        self.current_pose = msg
        
        # 实时发布位姿（高频，给控制节点用）
        uav_pose = UavPose()
        uav_pose.header = msg.header
        uav_pose.x = msg.pose.position.x
        uav_pose.y = msg.pose.position.y
        uav_pose.z = msg.pose.position.z
        
        # 四元数转偏航角（简化，可用tf_transformations）
        q = msg.pose.orientation
        uav_pose.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        self.pub_pose.publish(uav_pose)
    
    def on_velocity(self, msg: TwistStamped):
        """本地速度"""
        self.current_velocity = msg
    
    
    # ========== 定时发布融合状态 ==========
    def publish_telemetry(self):
        """10Hz发布综合状态和健康信息"""
        if self.raw_state is None:
            return  # 还未收到数据
        
        # 发布简化状态
        state_msg = UavState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.connected = self.raw_state.connected
        state_msg.armed = self.raw_state.armed
        state_msg.mode = self.raw_state.mode  # 字符串如"OFFBOARD"
        state_msg.guided = self.raw_state.guided
        
        # 着陆状态转换
        if self.raw_extended:
            landed_map = {
                0: 'UNKNOWN',
                1: 'ON_GROUND',
                2: 'IN_AIR', 
                3: 'TAKING_OFF',
                4: 'LANDING'
            }
            state_msg.landed_state = landed_map.get(
                self.raw_extended.landed_state, 'UNKNOWN')
        
        self.pub_state.publish(state_msg)
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """四元数转偏航角（简化版）"""
        import math
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()