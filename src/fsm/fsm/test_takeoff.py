#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_mavros2.uav_ctrl import UavController  # 假设你的 UavController 在 uav_mavros2 包里

class TestTakeoffNode(Node):
    def __init__(self):
        super().__init__('test_takeoff_node')
        self.get_logger().info("TestTakeoffNode 初始化")
        
        # 创建 UavController 节点实例
        self.uav = UavController()
        self.get_logger().info("UavController 已创建")

        # 启动测试
        self.test_takeoff()

    def test_takeoff(self):
        self.get_logger().info("开始起飞测试...")
        success = self.uav.takeoff_auto()  # 默认目标高度 2 米
        if success:
            self.get_logger().info("✅ 起飞测试完成，达到目标高度")
        else:
            self.get_logger().error("❌ 起飞测试失败")


def main(args=None):
    rclpy.init(args=args)
    node = TestTakeoffNode()

    try:
        # 持续 spin 保证节点接收 Telemetry
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("测试中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
