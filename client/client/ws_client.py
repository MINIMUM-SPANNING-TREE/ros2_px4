import asyncio
import json
import logging
import time
import uuid
import websockets

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# The Java backend BoardWebSocketHandler expects *flat* SensorData JSON:
#   { "type": "sensor_data", "temperature": 25.1, "humidity": 60.2, ... }
#
# It does NOT understand the envelope format
#   { "type": "sensor_data", "id": "...", "ts": ..., "payload": {...} }
#
# Board registration happens automatically when the WebSocket connects
# (in afterConnectionEstablished), so "board_hello" is implicit.
#
# Commands from the backend arrive as CommandMessage JSON:
#   { "type": "command", "command": "led_on", "parameters": {...}, "id": "..." }
# ---------------------------------------------------------------------------


class BoardWsClient:
    def __init__(self, url: str, board_id: str = None, token: str = None):
        if token:
            sep = '&' if '?' in url else '?'
            self.url = f'{url}{sep}token={token}'
        else:
            self.url = url
        self.board_id = board_id or f"rdk-x5-{uuid.uuid4().hex[:6]}"
        self.ws = None
        self._connected = False
        self._on_command = None

    def on_command(self, callback):
        self._on_command = callback

    async def send_raw(self, payload: dict):
        """Send a flat JSON message that the backend can parse."""
        if not self._connected or self.ws is None:
            return
        try:
            await self.ws.send(json.dumps(payload))
        except Exception as e:
            logger.warning("Send failed: %s", e)
            self._connected = False

    async def send_sensor_data(self, data: dict):
        """Send sensor data in the flat format the backend expects.

        The backend SensorData model has fields:
          type, temperature, humidity, pressure, light, status, timestamp
        We include the board_id as 'status' so the frontend can identify us.
        """
        payload = {
            "type": "sensor_data",
            "temperature": data.get("temperature"),
            "humidity": data.get("humidity"),
            "pressure": data.get("pressure"),
            "light": data.get("light"),
            "status": f"board:{self.board_id}",
            "timestamp": int(time.time() * 1000),
        }
        # Strip None values so Jackson doesn't choke on missing fields
        payload = {k: v for k, v in payload.items() if v is not None}
        await self.send_raw(payload)

    async def send_envelope(self, msg_type: str, payload: dict, src: str = "board"):
        """Send a message wrapped in the unified envelope format.

        Envelope schema:
            { "id": "xxxxxxxx", "type": "domain.action", "ts": <ms>,
              "src": "board", "payload": {...} }

        This allows the board to send any message type (sensor.data,
        board.heartbeat, telem.pose, etc.) in a consistent format that
        the backend can route by ``type``.  The backend's
        BoardWebSocketHandler will automatically unwrap the ``payload``
        field for backward-compatible SensorData processing.
        """
        envelope = {
            "id": str(uuid.uuid4())[:8],
            "type": msg_type,
            "ts": int(time.time() * 1000),
            "src": src,
            "payload": payload,
        }
        await self.send_raw(envelope)

    async def send_hello(self):
        """Log the board identity.  The backend registers the board by
        WebSocket IP on connection, so no protocol message is needed.
        """
        logger.info("Board '%s' registered by backend on connection", self.board_id)

    async def _handle_message(self, raw: str):
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("Received non-JSON message: %s", raw[:200])
            return

        msg_type = msg.get("type", "")

        if msg_type == "connection":
            logger.info("Backend connection ack: %s", msg.get("message", ""))
            return

        if msg_type == "ack":
            logger.debug("Backend ack: %s", msg.get("message", ""))
            return

        if msg_type == "error":
            logger.warning("Backend error: %s", msg.get("message", ""))
            return

        # Command messages from the backend
        if msg_type == "command" and self._on_command:
            command_name = msg.get("command", "")
            parameters = msg.get("parameters", {}) or {}
            command_id = msg.get("id", "")
            logger.info("Received command: %s (id=%s)", command_name, command_id)
            result = await self._on_command({
                "commandId": command_id,
                "command": command_name,
                "parameters": parameters,
            })
            logger.info("Command result: %s", result)
            return

        logger.debug("Unhandled message type '%s': %s", msg_type, raw[:200])

    async def connect_loop(self):
        backoff = 2.0
        while True:
            try:
                logger.info("Connecting to %s ...", self.url)
                ws = await websockets.connect(self.url)
                self.ws = ws
                self._connected = True
                backoff = 2.0
                await self.send_hello()
                logger.info("Board connected and registered")
                try:
                    async for msg in ws:
                        await self._handle_message(msg)
                finally:
                    self._connected = False
                    self.ws = None
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.warning("Connection failed: %s", e)
            self.ws = None
            self._connected = False
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30.0)

    @property
    def connected(self):
        return self._connected
