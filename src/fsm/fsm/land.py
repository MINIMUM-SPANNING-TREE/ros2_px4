#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_mavros2.uav_ctrl import UavController  # 假设你的 UavController 在 uav_mavros2 包里

class TestlandNode(Node):
    def __init__(self):
        super().__init__('test_land_node')
        self.get_logger().info("TestlandNode 初始化")
        
        # 创建 UavController 节点实例
        self.uav = UavController()
        self.get_logger().info("UavController 已创建")

        # 启动测试
        self.test_land()

    def test_land(self):
        self.get_logger().info("开始降落测试...")
        success = self.uav.land_auto()  # 自动降落
        if success:
            self.get_logger().info("✅ 降落测试完成，无人机已安全着陆")
        else:
            self.get_logger().error("❌ 降落测试失败")


def main(args=None):
    rclpy.init(args=args)
    node = TestlandNode()

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
