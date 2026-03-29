#!/usr/bin/env python3
import rclpy
import math
import time
from geometry_msgs.msg import PoseStamped
from uav_mavros2.uavbase import UavBase
from mavros_msgs.srv import CommandTOL

class Uav(UavBase):
    def __init__(self):
        super().__init__()
        self.DIRECTION_MAP = {
        "forward":  ( 1,  0,  0),
        "backward": (-1,  0,  0),
        "left":     ( 0,  1,  0),
        "right":    ( 0, -1,  0),
        "up":       ( 0,  0,  1),
        "down":     ( 0,  0, -1),
    }

    def takeoff(self, altitude=2.0):
        self.wait_for_connection()

        if altitude <= 0.0:
            self.get_logger().error("起飞高度必须大于 0")
            return False

        target_alt = self.current_pose.pose.position.z + altitude
        self.get_logger().info(f"起飞到 {target_alt:.2f} 米（当前模式: {self.get_mode()}）")

        if not self.takeoff_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("/mavros/cmd/takeoff 服务不可用")
            return False

        armed = False
        for _ in range(3):
            if self.arm(True):
                armed = True
                break
            time.sleep(0.3)

        if not armed:
            self.get_logger().error("解锁失败")
            return False

        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        # 传 NaN 代表使用当前位置，避免被误解为飞往(0,0)。
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.altitude = float(target_alt)

        future = self.takeoff_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self.get_logger().error("起飞命令发送失败: 服务响应超时")
            return False
        result = future.result()
        if result is None or not result.success:
            result_code = getattr(result, "result", None) if result is not None else None
            self.get_logger().error(f"起飞命令发送失败: success={getattr(result, 'success', None)} result={result_code}")
            return False

        self.get_logger().info(f"起飞命令已接受: success={result.success} result={result.result}")

        # 等待高度接近目标值，避免“命令发送成功但未实际爬升”。
        timeout = 30.0
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            current_alt = self.current_pose.pose.position.z
            if current_alt >= target_alt - 0.25:
                self.get_logger().info(f"已到达目标高度附近: {current_alt:.2f} 米")
                return True
            time.sleep(0.05)

        self.get_logger().error("起飞超时，未到达目标高度")
        return False


    def land(self, timeout=30.0):
        """
        仅切换到 AUTO.LAND 并在本层等待落地完成。
        """
        self.wait_for_connection()

        if timeout <= 0.0:
            self.get_logger().error("降落超时时间必须大于 0")
            return False

        self.get_logger().info(f"开始降落，超时时间={timeout:.1f} 秒（当前模式: {self.get_mode()} 已解锁: {self.is_armed()}）")

        if self.get_landed_state() == 1:
            self.get_logger().info("当前已在地面，无需重复降落")
            return True

        if not self.set_mode("AUTO.LAND"):
            self.get_logger().error("切换 AUTO.LAND 失败")
            return False

        self.get_logger().info("已切换到 AUTO.LAND")

        # 在服务层监测落地状态，避免上层重复实现。
        start = time.time()
        while rclpy.ok() and (time.time() - start) < float(timeout):
            current_alt = self.current_pose.pose.position.z
            if self.get_landed_state() == 1 or current_alt < 0.20:
                self.get_logger().info(f"已落地: landed_state={self.get_landed_state()} alt={current_alt:.2f}m")
                return True
            time.sleep(0.05)

        current_alt = self.current_pose.pose.position.z
        self.get_logger().error(f"降落超时: landed_state={self.get_landed_state()} alt={current_alt:.2f}m")
        return False

    def rtl(self, timeout=60.0):
        """
        切换到 AUTO.RTL 并在本层等待返航落地完成。
        """
        self.wait_for_connection()

        if timeout <= 0.0:
            self.get_logger().error("返航超时时间必须大于 0")
            return False

        if self.get_landed_state() == 1:
            self.get_logger().info("当前已在地面，无需执行返航")
            return True

        if not self.set_mode("AUTO.RTL"):
            self.get_logger().error("切换 AUTO.RTL 失败")
            return False

        self.get_logger().info("已切换到 AUTO.RTL，等待返航落地")

        start = time.time()
        while rclpy.ok() and (time.time() - start) < float(timeout):
            current_alt = self.current_pose.pose.position.z
            landed = self.get_landed_state() == 1
            near_ground = current_alt < 0.20
            auto_disarmed = (not self.is_armed()) and (current_alt < 0.50)
            if landed or near_ground or auto_disarmed:
                self.get_logger().info(
                    f"返航完成: landed_state={self.get_landed_state()} alt={current_alt:.2f}m armed={self.is_armed()}"
                )
                return True
            time.sleep(0.05)

        current_alt = self.current_pose.pose.position.z
        self.get_logger().error(f"返航超时: landed_state={self.get_landed_state()} alt={current_alt:.2f}m")
        return False

    def move(self, x, y, z, yaw=0.0, timeout=30.0, wait_until_arrive=True):
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

        # move 使用位置 setpoint 控制，需先进入 OFFBOARD。
        if self.get_mode() != "OFFBOARD":
            for _ in range(20):
                self.target_pose.header.stamp = self.get_clock().now().to_msg()
                self.local_pos_pub.publish(self.target_pose)
                time.sleep(0.05)

            if not self.set_mode("OFFBOARD"):
                self.get_logger().error("切换 OFFBOARD 失败，无法执行移动")
                return False

            self.get_logger().info("已切换到 OFFBOARD，开始执行移动")

        if not wait_until_arrive:
            return True

        # 在业务层等待到位，避免在上层重复实现检测逻辑。
        start = time.time()
        while rclpy.ok() and (time.time() - start) < float(timeout):
            if self.is_arrive():
                return True
            time.sleep(0.05)

        self.get_logger().error("移动超时，未到达目标位置")
        return False

    def move_direction(self, direction, distance, yaw=0.0, timeout=30.0, wait_until_arrive=True):
        if direction not in self.DIRECTION_MAP:
            return False
        target_x = self.get_target_pos_x()
        target_y = self.get_target_pos_y()
        target_z = self.get_target_pos_z()
        dx, dy, dz = self.DIRECTION_MAP[direction]
        target_x = target_x + dx * distance
        target_y = target_y + dy * distance
        target_z = target_z + dz * distance
        return self.move(target_x, target_y, target_z, yaw=yaw, timeout=timeout, wait_until_arrive=wait_until_arrive)
    def is_arrive(self):
        current_x = self.get_current_pos_x()
        current_y = self.get_current_pos_y()
        current_z = self.get_current_pos_z()
        target_x = self.get_target_pos_x()
        target_y = self.get_target_pos_y()
        target_z = self.get_target_pos_z()
        dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2 + (target_z - current_z)**2)
        return dist < 0.3