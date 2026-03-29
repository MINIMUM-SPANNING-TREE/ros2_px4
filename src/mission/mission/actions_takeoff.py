#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.srv import Takeoff


class TakeoffClientAction:
    def __init__(self, node: Node, default_timeout: float = 90.0):
        self.node = node
        self.default_timeout = float(default_timeout)
        self.client = self.node.create_client(Takeoff, 'uav/takeoff')

    def execute(self, relative_alt: float, timeout: float = None):
        if not self.client.wait_for_service(timeout_sec=5.0):
            return False, 'uav/takeoff 服务不可用'

        req = Takeoff.Request()
        req.relative_alt = float(relative_alt)
        fut = self.client.call_async(req)

        service_wait = (self.default_timeout if timeout is None else float(timeout)) + 10.0
        start = time.time()
        while rclpy.ok() and (time.time() - start) < service_wait:
            if fut.done():
                break
            time.sleep(0.05)

        if not fut.done():
            return False, f'调用 uav/takeoff 超时（等待 {service_wait:.1f}s）'

        res = fut.result()
        if res is None:
            return False, 'uav/takeoff 返回空'
        return bool(getattr(res, 'success', False)), str(getattr(res, 'message', ''))
