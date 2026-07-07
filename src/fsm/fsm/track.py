#!/usr/bin/env python3
"""Target tracking service for selecting the latest DeepSORT track."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_interfaces.msg import TrackDetection, TrackDetectionArray
from uav_interfaces.srv import SetVelocity, TrackTarget


class TargetTrackingService(Node):
    def __init__(self):
        super().__init__('target_tracking_service')

        self.declare_parameter('tracks_topic', '/deepsort/tracks')
        self.declare_parameter('service_name', 'fsm/track_target')
        self.declare_parameter('velocity_service', 'uav/set_velocity')
        self.declare_parameter('default_min_score', 0.3)
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('tracking_duration_sec', 10.0)
        self.declare_parameter('control_enabled', True)
        self.declare_parameter('control_rate_hz', 5.0)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('deadband_x', 0.08)
        self.declare_parameter('deadband_y', 0.08)
        self.declare_parameter('target_area_ratio', 0.08)
        self.declare_parameter('area_deadband', 0.02)
        self.declare_parameter('y_kp', 0.8)
        self.declare_parameter('z_kp', 0.4)
        self.declare_parameter('x_kp', 0.6)
        self.declare_parameter('y_kd', 0.0)
        self.declare_parameter('z_kd', 0.0)
        self.declare_parameter('x_kd', 0.0)
        self.declare_parameter('max_vy', 0.6)
        self.declare_parameter('max_vz', 0.4)
        self.declare_parameter('max_vx', 0.6)

        self.tracks_topic = self.get_parameter('tracks_topic').value                                   # 跟踪话题
        self.velocity_service_name = self.get_parameter('velocity_service').value                      # 速度服务名称
        self.default_min_score = float(self.get_parameter('default_min_score').value)                  # 默认最小分数
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)                # 目标超时时间
        self.default_tracking_duration_sec = float(self.get_parameter('tracking_duration_sec').value)  # 默认跟踪持续时间
        self.control_enabled = bool(self.get_parameter('control_enabled').value)                       # 控制是否启用
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))            # 控制频率
        self.image_width = max(1.0, float(self.get_parameter('image_width').value))                    # 图像宽度
        self.image_height = max(1.0, float(self.get_parameter('image_height').value))                  # 图像高度
        self.deadband_x = float(self.get_parameter('deadband_x').value)                                # 死区范围X
        self.deadband_y = float(self.get_parameter('deadband_y').value)                                # 死区范围Y
        self.target_area_ratio = float(self.get_parameter('target_area_ratio').value)                  # 目标面积比例
        self.area_deadband = float(self.get_parameter('area_deadband').value)                          # 面积死区
        self.y_kp = float(self.get_parameter('y_kp').value)                                            # Y轴比例增益
        self.z_kp = float(self.get_parameter('z_kp').value)                                            # Z轴比例增益
        self.x_kp = float(self.get_parameter('x_kp').value)                                            # X轴比例增益
        self.y_kd = float(self.get_parameter('y_kd').value)                                            # Y轴微分增益
        self.z_kd = float(self.get_parameter('z_kd').value)                                            # Z轴微分增益
        self.x_kd = float(self.get_parameter('x_kd').value)                                            # X轴微分增益
        self.max_vy = abs(float(self.get_parameter('max_vy').value))                                   # Y轴最大速度
        self.max_vz = abs(float(self.get_parameter('max_vz').value))                                   # Z轴最大速度
        self.max_vx = abs(float(self.get_parameter('max_vx').value))                                   # X轴最大速度

        self.tracking = False                                                                          # 是否正在跟踪
        self.target_class = ''                                                                         # 目标类别
        self.min_score = self.default_min_score                                                        # 最小分数
        self.latest_target = None                                                                      # 最新目标
        self.latest_target_time = None                                                                 # 最新目标时间
        self.tracking_start_time = None                                                                # 跟踪开始时间
        self.tracking_duration_sec = self.default_tracking_duration_sec                                # 跟踪持续时间
        self.velocity_pending = False                                                                  # 速度请求是否挂起
        self.prev_error_x = None                                                                       # 上一次的X误差
        self.prev_error_y = None                                                                       # 上一次的Y误差
        self.prev_area_error = None                                                                    # 上一次的面积误差
        self.prev_control_time = None                                                                  # 上一次的控制时间

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(TrackDetectionArray, self.tracks_topic, self._tracks_cb, qos)        # 订阅跟踪检测数组话题
        self.velocity_cli = self.create_client(SetVelocity, self.velocity_service_name)               # 创建速度服务客户端
        self.create_timer(1.0 / self.control_rate_hz, self._control_tick)                             # 创建定时器以控制频率调用控制函数

        service_name = self.get_parameter('service_name').value                                       # 目标跟踪服务名称
        self.create_service(TrackTarget, service_name, self._handle_track_target)                     # 创建目标跟踪服务
        self.get_logger().info(f'Target tracking service ready: /{service_name}')                     # 记录目标跟踪服务已准备好
        self.get_logger().info(f'Subscribed tracks topic: {self.tracks_topic}')                       # 记录已订阅的跟踪话题
        self.get_logger().info(f'Control enabled: {self.control_enabled}, velocity service: {self.velocity_service_name}') # 记录控制是否启用以及速度服务名称

    def _tracks_cb(self, msg):  # 处理跟踪检测数组消息的回调函数
        if not self.tracking:
            return

        target = self._select_target(msg.tracks)
        if target is None:
            return

        self.latest_target = target
        self.latest_target_time = self.get_clock().now()

    def _select_target(self, detections):  # 从检测结果中选择目标
        candidates = []
        for detection in detections:
            if detection.score < self.min_score:
                continue
            if self.target_class and detection.class_id != self.target_class:
                continue
            candidates.append(detection)

        if not candidates:
            return None
        return max(candidates, key=lambda detection: detection.score) # 选择分数最高的检测结果作为目标

    def _handle_track_target(self, request, response):   # 处理目标跟踪服务请求的回调函数
        command = request.command.strip().lower()
        if command not in ('start', 'stop', 'status'):
            response.success = False
            response.message = 'command must be one of: start, stop, status'
            return self._fill_status(response)

        if command == 'start':
            self.tracking = True
            self.tracking_start_time = self.get_clock().now()
            self.target_class = request.target_class.strip()
            self.min_score = request.min_score if request.min_score > 0.0 else self.default_min_score
            if request.timeout_sec > 0.0:
                self.target_timeout_sec = request.timeout_sec
            self.tracking_duration_sec = (
                request.tracking_duration_sec
                if request.tracking_duration_sec > 0.0
                else self.default_tracking_duration_sec
            )
            self.latest_target = None
            self.latest_target_time = None
            self._reset_controller_state()
            response.success = True
            response.message = 'target tracking started'
        elif command == 'stop':
            self._stop_tracking()
            response.success = True
            response.message = 'target tracking stopped'
        else:
            response.success = True
            response.message = 'target tracking status'

        return self._fill_status(response)

    def _fill_status(self, response): # 填充目标跟踪服务响应的状态信息
        response.tracking = self.tracking
        response.target_found = self._target_is_fresh()
        response.tracking_elapsed_sec = self._tracking_elapsed_sec()

        if response.target_found:
            response.target = self.latest_target
            response.track_id = self.latest_target.track_id
            response.center_x = (self.latest_target.x_min + self.latest_target.x_max) / 2.0
            response.center_y = (self.latest_target.y_min + self.latest_target.y_max) / 2.0
            response.age_sec = self._target_age_sec()
        else:
            response.target = TrackDetection()
            response.track_id = 0
            response.center_x = 0.0
            response.center_y = 0.0
            response.age_sec = 0.0

        return response

    def _target_is_fresh(self): # 检查最新目标是过期
        if self.latest_target is None or self.latest_target_time is None:
            return False
        return self._target_age_sec() <= self.target_timeout_sec

    def _target_age_sec(self): # 计算最新目标的时间
        if self.latest_target_time is None:
            return 0.0
        age = self.get_clock().now() - self.latest_target_time
        return age.nanoseconds / 1e9

    def _tracking_elapsed_sec(self): # 计算跟踪已经持续的时间
        if self.tracking_start_time is None:
            return 0.0
        elapsed = self.get_clock().now() - self.tracking_start_time
        return elapsed.nanoseconds / 1e9

    def _tracking_time_expired(self): # 检查跟踪时间是否已经到达设定的持续时间
        return self.tracking_duration_sec > 0.0 and self._tracking_elapsed_sec() >= self.tracking_duration_sec

    def _control_tick(self): # 定时器回调函数，用于控制无人机的速度
        if not self.control_enabled or not self.tracking:
            return
        if self._tracking_time_expired():
            self.get_logger().info(f'跟踪时间到达 {self.tracking_duration_sec:.1f}s，停止跟踪')
            self._stop_tracking()
            return
        if not self._target_is_fresh():
            self._reset_controller_state()
            self._send_velocity(0.0, 0.0, 0.0, 0.0)
            return

        vx, vy, vz, yaw_rate = self._make_velocity_command(self.latest_target)
        self._send_velocity(vx, vy, vz, yaw_rate)

    def _make_velocity_command(self, target): # 根据最新目标计算无人机的速度命令
        center_x = (target.x_min + target.x_max) / 2.0
        center_y = (target.y_min + target.y_max) / 2.0
        width = max(0.0, target.x_max - target.x_min)
        height = max(0.0, target.y_max - target.y_min)

        error_x = (center_x - self.image_width / 2.0) / (self.image_width / 2.0)
        error_y = (center_y - self.image_height / 2.0) / (self.image_height / 2.0)
        area_ratio = (width * height) / (self.image_width * self.image_height)
        area_error = self.target_area_ratio - area_ratio
        dt = self._control_dt()
        error_x_d = self._derivative(error_x, self.prev_error_x, dt)
        error_y_d = self._derivative(error_y, self.prev_error_y, dt)
        area_error_d = self._derivative(area_error, self.prev_area_error, dt)

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_area_error = area_error

        vy = 0.0 if abs(error_x) < self.deadband_x else -(self.y_kp * error_x + self.y_kd * error_x_d)
        vz = 0.0 if abs(error_y) < self.deadband_y else -(self.z_kp * error_y + self.z_kd * error_y_d)
        vx = 0.0 if abs(area_error) < self.area_deadband else self.x_kp * area_error + self.x_kd * area_error_d

        return (
            self._clamp(vx, -self.max_vx, self.max_vx),
            self._clamp(vy, -self.max_vy, self.max_vy),
            self._clamp(vz, -self.max_vz, self.max_vz),
            0.0,
        )

    def _send_velocity(self, vx, vy, vz, yaw_rate): # 发送速度命令到无人机
        if self.velocity_pending:
            return
        if not self.velocity_cli.service_is_ready():
            self.velocity_cli.wait_for_service(timeout_sec=0.0)
            if not self.velocity_cli.service_is_ready():
                return

        req = SetVelocity.Request(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        self.velocity_pending = True
        future = self.velocity_cli.call_async(req)
        future.add_done_callback(self._velocity_done)

    def _velocity_done(self, future): # 处理速度服务调用完成的回调函数
        self.velocity_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'速度控制服务调用失败: {exc}')
            return
        if result is not None and not result.success:
            self.get_logger().warn(f'速度控制被拒绝: {result.message}')

    @staticmethod # 将值限制在指定范围内
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def _control_dt(self):
        now = self.get_clock().now()
        if self.prev_control_time is None:
            self.prev_control_time = now
            return None
        dt = (now - self.prev_control_time).nanoseconds / 1e9
        self.prev_control_time = now
        return dt if dt > 1e-3 else None

    @staticmethod # 计算误差的导数
    def _derivative(error, previous_error, dt):
        if previous_error is None or dt is None:
            return 0.0
        return (error - previous_error) / dt

    def _reset_controller_state(self):
        self.prev_error_x = None
        self.prev_error_y = None
        self.prev_area_error = None
        self.prev_control_time = None

    def _stop_tracking(self):
        self.tracking = False
        self.latest_target = None
        self.latest_target_time = None
        self.tracking_start_time = None
        self._reset_controller_state()
        self._send_velocity(0.0, 0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackingService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
