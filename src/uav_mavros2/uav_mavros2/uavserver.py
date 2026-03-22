import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from uav_mavros2.uav import Uav
from uav_interfaces.srv import Arm, Land, Move, SetMode, Takeoff

class UavServer(Uav):
    def __init__(self):
        super().__init__()
        
        
        self.srv_arm = self.create_service(Arm, 'uav/arm', self.handle_arm, callback_group=self.cb_group)
        self.srv_land = self.create_service(Land, 'uav/land', self.handle_land, callback_group=self.cb_group)
        self.srv_move = self.create_service(Move, 'uav/move', self.handle_move, callback_group=self.cb_group)
        self.srv_set_mode = self.create_service(SetMode, 'uav/set_mode', self.handle_set_mode, callback_group=self.cb_group)
        self.srv_takeoff = self.create_service(Takeoff, 'uav/takeoff', self.handle_takeoff, callback_group=self.cb_group)


    def handle_arm(self, request, response):
        self.get_logger().info(f"==> 收到请求: {'解锁' if request.arm else '上锁'}")
        result = self.arm(request.arm)
        response.success = result
        response.message = "Arm/Disarm 执行完毕" if result else "执行失败"
        return response

    def handle_land(self, request, response):
        self.get_logger().info(f"==> 收到请求: 降落, 超时时间={request.timeout}")
        # 这里直接调用自身（继承自Uav）的方法
        result = self.land(request.timeout)
        response.success = result
        response.message = "降落成功" if result else "降落失败或超时"
        return response

    def handle_move(self, request, response):
        self.get_logger().info(f"==> 收到请求: 移动到 ({request.x}, {request.y}, {request.z}, yaw={request.yaw})")
        result = self.move(request.x, request.y, request.z, request.yaw)
        response.success = result
        response.message = "顺利到达指定位置" if result else "移动失败或超时"
        return response

    def handle_set_mode(self, request, response):
        self.get_logger().info(f"==> 收到请求: 切换模式到 {request.mode}")
        result = self.set_mode(request.mode)
        response.success = result
        response.message = "模式切换完毕" if result else "模式切换失败"
        return response

    def handle_takeoff(self, request, response):
        self.get_logger().info(f"==> 收到请求: 起飞, 相对高度={request.relative_alt}")
        result = self.takeoff(request.relative_alt)
        response.success = result
        response.message = "成功到达起飞指定高度" if result else "起飞失败"
        return response
