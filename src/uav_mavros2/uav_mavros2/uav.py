#!/usr/bin/env python3
import rclpy
import math
import time
from geometry_msgs.msg import PoseStamped
from uav_mavros2.uavbase import UavBase
from mavros_msgs.srv import CommandTOL
class Uav(UavBase):
    def __init__(self):
        super().__init__('uav_node')
        self.DIRECTION_MAP = {
        "forward":  ( 1,  0,  0),
        "backward": (-1,  0,  0),
        "left":     ( 0,  1,  0),
        "right":    ( 0, -1,  0),
        "up":       ( 0,  0,  1),
        "down":     ( 0,  0, -1),
    }

    def takeoff(self, altitude=2.5):
        self.wait_for_connection()
        target_alt = self.current_pose.pose.position.z + altitude
        tar_x = self.get_current_pos_x()
        tar_y = self.get_current_pos_y()
        tar_z = target_alt
        self.set_target_pos(tar_x, tar_y, tar_z)
        if not self.arm(True):
            self.get_logger().error("解锁失败")
            return False
        if self.current_state.mode != "OFFBOARD":
            if not self.set_mode("OFFBOARD"):
                self.get_logger().error("设置 OFFBOARD 模式失败")
                return False
        return True


    def land(self, timeout=30.0):
        
        # 将目标高度更新为0或者当前平面的Z=0高度
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        self.set_target_pos(curr_x, curr_y, 0.0)
        if not self.set_mode("OFFBOARD"):
            self.get_logger().error("设置 OFFBOARD 模式失败")
            return False
        return True

    def move(self, x, y, z, yaw=0.0):
        # 更新基类中记录的目标值参数
        self.set_target_pos(x, y, z)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 把构建好的位姿保存为类属性
        self.target_pose = pose

    def move_direction(self, direction, distance, yaw=0.0):
        if direction not in self.DIRECTION_MAP:
            return False
        target_x = self.get_target_pos_x()
        target_y = self.get_target_pos_y()
        target_z = self.get_target_pos_z()
        dx, dy, dz = self.DIRECTION_MAP[direction]
        target_x = target_x + dx * distance
        target_y = target_y + dy * distance
        target_z = target_z + dz * distance
        return self.move(target_x, target_y, target_z)
    def is_arrive(self):
        current_x = self.get_current_pos_x()
        current_y = self.get_current_pos_y()
        current_z = self.get_current_pos_z()
        target_x = self.get_target_pos_x()
        target_y = self.get_target_pos_y()
        target_z = self.get_target_pos_z()
        dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2 + (target_z - current_z)**2)
        return dist < 0.3