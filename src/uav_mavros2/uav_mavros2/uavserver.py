import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from uav_mavros2.uav import Uav
from uav_interfaces.srv import Arm, Land, Move, Rtl, SetMode, Takeoff

class UavServer(Uav):
    def __init__(self):
        super().__init__()
        self.cb_group = ReentrantCallbackGroup()
        
        self.srv_arm = self.create_service(Arm, 'uav/arm', self.handle_arm, callback_group=self.cb_group)
        self.srv_land = self.create_service(Land, 'uav/land', self.handle_land, callback_group=self.cb_group)
        self.srv_move = self.create_service(Move, 'uav/move', self.handle_move, callback_group=self.cb_group)
        self.srv_set_mode = self.create_service(SetMode, 'uav/set_mode', self.handle_set_mode, callback_group=self.cb_group)
        self.srv_takeoff = self.create_service(Takeoff, 'uav/takeoff', self.handle_takeoff, callback_group=self.cb_group)
        self.srv_rtl = self.create_service(Rtl, 'uav/rtl', self.handle_rtl, callback_group=self.cb_group)


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
        self.get_logger().info(f"==> 收到请求: 移动到 ({getattr(request,'x',None)}, {getattr(request,'y',None)}, {getattr(request,'z',None)}, yaw={getattr(request,'yaw',0.0)})")
        # 触发移动并在业务层等待到位
        yaw = getattr(request, 'yaw', 0.0)
        timeout = getattr(request, 'timeout', 30.0)
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
        result = self.set_mode(request.mode)
        response.success = result
        response.message = "模式切换完毕" if result else "模式切换失败"
        return response

    def handle_takeoff(self, request, response):
        self.get_logger().info(f"==> 收到请求: 起飞, 相对高度={request.relative_alt}")
        result = self.takeoff(request.relative_alt)
        response.success = result
        response.message = "成功发送起飞指令" if result else "起飞失败"
        return response

    def handle_rtl(self, request, response):
        self.get_logger().info(f"==> 收到请求: 自动返航, 超时时间={request.timeout}")
        result = self.rtl(request.timeout)
        response.success = result
        response.message = "返航成功" if result else "返航失败或超时"
        return response
