#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_mavros2.uav_ctrl import UavController  
class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        self.get_logger().info("MoveNode 初始化")
        
        # 创建 UavController 节点实例
        self.uav = UavController()
        self.get_logger().info("UavController 已创建")

        # 启动测试
        self.move()

    def move(self):
        self.get_logger().info("开始移动测试...")
        success = self.uav.move_offboard(4.0,4.0,4.0)  
        if success:
            self.get_logger().info("✅ 移动测试完成，到达目标位置")
            self.uav.set_mode("AUTO.LOITER")  # 切换回自动巡航模式
        else:
            self.get_logger().error("❌ 移动测试失败")
    


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()

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
