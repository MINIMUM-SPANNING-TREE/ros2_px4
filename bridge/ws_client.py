#!/usr/bin/env python3
# Copyright (C) 2026 最小生成树 (Minimum Spanning Tree). All rights reserved.
#
# Author: xianglajituibao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
WebSocket 客户端。

职责：
  - 连接到 Spring Boot 后端 /ws/bridge
  - 发送 bridge_hello + 定期 heartbeat
  - 接收 program_run / program_stop 消息并转给 executor
  - 发送 execution_event / telemetry 回后端
"""

import asyncio
import json
import logging
import time
import uuid

import websockets

logger = logging.getLogger(__name__)


class WsClient:
    """WebSocket 客户端，管理与后端的双向通信。"""

    def __init__(self, url: str, bridge_id: str = None, token: str = None):
        """
        Args:
            url: 后端 WebSocket 地址，如 ws://192.168.x.x:8880/ws/bridge
            bridge_id: 可选，标识此 Bridge；缺省自动生成
            token: 可选，认证令牌；非空时自动追加到连接 URL 的 ?token= 参数
        """
        if token:
            sep = '&' if '?' in url else '?'
            self.url = f'{url}{sep}token={token}'
        else:
            self.url = url
        self.bridge_id = bridge_id or f'bridge-{uuid.uuid4().hex[:6]}'
        self.ws = None
        self._on_program_run = None   # async callback(run_id, program)
        self._on_program_stop = None  # async callback(run_id)
        self._connected = False
        self._tasks: list[asyncio.Task] = []  # prevent GC of fire-and-forget tasks

    def on_program_run(self, callback):
        self._on_program_run = callback

    def on_program_stop(self, callback):
        self._on_program_stop = callback

    # ---------- 发送消息 ----------

    async def send_envelope(self, msg_type: str, payload: dict):
        if self.ws is None or not self._connected:
            logger.warning('WebSocket 未连接，无法发送 %s', msg_type)
            return
        envelope = {
            'type': msg_type,
            'id': uuid.uuid4().hex[:8],
            'ts': int(time.time() * 1000),
            'payload': payload,
        }
        try:
            await self.ws.send(json.dumps(envelope))
        except Exception as e:
            logger.warning('发送 %s 失败: %s', msg_type, e)
            self._connected = False

    async def send_hello(self):
        await self.send_envelope('bridge_hello', {
            'bridgeId': self.bridge_id,
            'version': '0.1.0',
            'capabilities': ['execute_v1'],
        })

    async def send_heartbeat(self):
        await self.send_envelope('heartbeat', {})

    async def send_execution_event(self, run_id: str, block_id: str | None,
                                    event: str, error: str | None = None):
        payload = {'runId': run_id, 'blockId': block_id, 'event': event}
        if error:
            payload['error'] = error
        await self.send_envelope('execution_event', payload)

    async def send_telemetry(self, telemetry: dict):
        await self.send_envelope('telemetry', telemetry)

    # ---------- 接收消息 ----------

    async def _handle_message(self, raw: str):
        try:
            env = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning('无法解析消息: %s', raw[:200])
            return

        msg_type = env.get('type', '')
        payload = env.get('payload', {})

        if msg_type == 'program_run':
            run_id = payload.get('runId')
            program = payload.get('program')
            logger.info('收到 program_run: runId=%s', run_id)
            if self._on_program_run:
                task = asyncio.create_task(self._on_program_run(run_id, program))
                self._tasks.append(task)
                task.add_done_callback(self._tasks.remove)

        elif msg_type in ('program_stop', 'program_estop'):
            run_id = payload.get('runId')
            logger.info('收到 program_stop: runId=%s', run_id)
            if self._on_program_stop:
                task = asyncio.create_task(self._on_program_stop(run_id))
                self._tasks.append(task)
                task.add_done_callback(self._tasks.remove)

        elif msg_type == 'hello_ack':
            logger.info('后端已确认连接')

        elif msg_type == 'ping':
            await self.send_envelope('pong', {})

        else:
            logger.debug('忽略未知消息类型: %s', msg_type)

    # ---------- 连接主循环 ----------

    async def connect_loop(self):
        """持续连接后端，断线自动重连（指数退避，2s 起步，最大 30s）。"""
        backoff = 2.0
        max_backoff = 30.0
        while True:
            try:
                logger.info('正在连接 %s ...', self.url)
                ws = await websockets.connect(self.url)
                self.ws = ws
                self._connected = True
                backoff = 2.0  # 连接成功，重置退避
                logger.info('已连接到后端')
                await self.send_hello()

                # 启动心跳协程
                heartbeat_task = asyncio.create_task(self._heartbeat_loop())

                try:
                    async for message in ws:
                        await self._handle_message(message)
                finally:
                    heartbeat_task.cancel()
                    self._connected = False
                    self.ws = None
                    await ws.close()
                    logger.warning('WebSocket 连接断开')

            except asyncio.CancelledError:
                logger.info('连接循环被取消')
                raise
            except (ConnectionRefusedError, OSError) as e:
                logger.warning('连接失败: %s，%.1f 秒后重试', e, backoff)
            except Exception as e:
                logger.error('WebSocket 异常: %s，%.1f 秒后重试', e, backoff)

            self.ws = None
            self._connected = False
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, max_backoff)

    async def _heartbeat_loop(self):
        """每 5 秒发送一次心跳。"""
        try:
            while True:
                await asyncio.sleep(5)
                await self.send_heartbeat()
        except asyncio.CancelledError:
            pass

    @property
    def connected(self) -> bool:
        return self._connected
