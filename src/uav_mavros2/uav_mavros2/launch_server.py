#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from uav_mavros2.uavserver import UavServer

def main(args=None):
    rclpy.init(args=args)
    node = UavServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()