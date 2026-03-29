#!/usr/bin/env python3

from mission.actions_directional_move import DirectionalMoveAction
from mission.actions_land import LandClientAction
from mission.actions_move import MoveClientAction
from mission.actions_rtl import RtlClientAction
from mission.actions_takeoff import TakeoffClientAction


class MissionConsoleRouter:
    def __init__(
        self,
        takeoff_action: TakeoffClientAction,
        move_action: MoveClientAction,
        land_action: LandClientAction,
        rtl_action: RtlClientAction,
        directional_move_action: DirectionalMoveAction,
        logger,
    ):
        self.takeoff_action = takeoff_action
        self.move_action = move_action
        self.land_action = land_action
        self.rtl_action = rtl_action
        self.directional_move_action = directional_move_action
        self.logger = logger

    def print_help(self):
        self.logger.info('可用命令:')
        self.logger.info('  takeoff <relative_alt>')
        self.logger.info('  move <x> <y> <z> [yaw] [timeout]')
        self.logger.info('  dir <forward|backward|left|right|up|down> <distance> [yaw] [timeout]')
        self.logger.info('  land [timeout]')
        self.logger.info('  rtl [timeout]')
        self.logger.info('  help')
        self.logger.info('  exit')

    def execute_line(self, line: str) -> bool:
        text = line.strip()
        if not text:
            return True

        parts = text.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd == 'help':
                self.print_help()
                return True

            if cmd == 'exit':
                self.logger.info('收到退出命令，准备结束 mission 控制台')
                return False

            if cmd == 'takeoff':
                if len(args) != 1:
                    self.logger.error('用法: takeoff <relative_alt>')
                    return True
                ok, msg = self.takeoff_action.execute(float(args[0]))
                self.logger.info(f'takeoff 结果: success={ok} message="{msg}"')
                return True

            if cmd == 'move':
                if len(args) not in (3, 4, 5):
                    self.logger.error('用法: move <x> <y> <z> [yaw] [timeout]')
                    return True
                x = float(args[0])
                y = float(args[1])
                z = float(args[2])
                yaw = float(args[3]) if len(args) >= 4 else 0.0
                timeout = float(args[4]) if len(args) == 5 else None
                ok, msg = self.move_action.execute(x, y, z, yaw=yaw, timeout=timeout)
                self.logger.info(f'move 结果: success={ok} message="{msg}"')
                return True

            if cmd == 'dir':
                if len(args) not in (2, 3, 4):
                    self.logger.error('用法: dir <direction> <distance> [yaw] [timeout]')
                    return True
                direction = args[0]
                distance = float(args[1])
                yaw = float(args[2]) if len(args) >= 3 else None
                timeout = float(args[3]) if len(args) == 4 else None
                ok, msg = self.directional_move_action.execute(
                    direction=direction,
                    distance=distance,
                    yaw=yaw,
                    timeout=timeout,
                )
                self.logger.info(f'dir 结果: success={ok} message="{msg}"')
                return True

            if cmd == 'land':
                if len(args) > 1:
                    self.logger.error('用法: land [timeout]')
                    return True
                timeout = float(args[0]) if args else None
                ok, msg = self.land_action.execute(timeout=timeout)
                self.logger.info(f'land 结果: success={ok} message="{msg}"')
                return True

            if cmd == 'rtl':
                if len(args) > 1:
                    self.logger.error('用法: rtl [timeout]')
                    return True
                timeout = float(args[0]) if args else None
                ok, msg = self.rtl_action.execute(timeout=timeout)
                self.logger.info(f'rtl 结果: success={ok} message="{msg}"')
                return True

            self.logger.error(f'未知命令: {cmd}，输入 help 查看可用命令')
            return True
        except ValueError as ex:
            self.logger.error(f'参数解析失败: {ex}')
            return True
