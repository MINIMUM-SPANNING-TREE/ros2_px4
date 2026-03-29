#!/usr/bin/env python3
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission.actions_directional_move import DirectionalMoveAction
from mission.actions_land import LandClientAction
from mission.actions_move import MoveClientAction
from mission.actions_rtl import RtlClientAction
from mission.actions_takeoff import TakeoffClientAction
from mission.mission_console_router import MissionConsoleRouter
from mission.uav_state_cache import UavStateCache


class MissionConsoleLauncher(Node):
    """mission 交互入口：组装动作层与路由层。"""

    def __init__(self, state_cache: UavStateCache):
        super().__init__('mission_console_launcher')
        self.state_cache = state_cache

        self.takeoff_action = TakeoffClientAction(self)
        self.move_action = MoveClientAction(self)
        self.land_action = LandClientAction(self)
        self.rtl_action = RtlClientAction(self)
        self.directional_move_action = DirectionalMoveAction(self.state_cache, self.move_action)

        self.router = MissionConsoleRouter(
            takeoff_action=self.takeoff_action,
            move_action=self.move_action,
            land_action=self.land_action,
            rtl_action=self.rtl_action,
            directional_move_action=self.directional_move_action,
            logger=self.get_logger(),
        )

    def run_console_loop(self):
        self.get_logger().info('Mission 控制台已启动，输入 help 查看命令')
        while rclpy.ok():
            try:
                line = input('mission> ')
            except EOFError:
                break
            except KeyboardInterrupt:
                self.get_logger().info('检测到 Ctrl+C，退出 mission 控制台')
                break

            should_continue = self.router.execute_line(line)
            if not should_continue:
                break


def main(args=None):
    rclpy.init(args=args)

    state_cache = UavStateCache()
    launcher = MissionConsoleLauncher(state_cache)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(state_cache)
    executor.add_node(launcher)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        launcher.run_console_loop()
    finally:
        executor.shutdown()
        state_cache.destroy_node()
        launcher.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
