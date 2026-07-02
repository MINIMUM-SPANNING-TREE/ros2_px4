import pytest
import json
import time
from unittest.mock import AsyncMock, MagicMock, patch
from client.ws_client import BoardWsClient


class TestBoardWsClient:
    def test_init_default_board_id(self):
        client = BoardWsClient("ws://localhost:9200/ws/board")
        assert client.url == "ws://localhost:9200/ws/board"
        assert client.board_id.startswith("rdk-x5-")
        assert client.connected == False

    def test_init_custom_board_id(self):
        client = BoardWsClient("ws://test", board_id="my-board")
        assert client.board_id == "my-board"

    def test_on_command_sets_callback(self):
        client = BoardWsClient("ws://test")
        cb = lambda x: x
        client.on_command(cb)
        assert client._on_command == cb

    @pytest.mark.asyncio
    async def test_send_sensor_data_format(self):
        client = BoardWsClient("ws://test", board_id="test-board")
        client._connected = True
        client.ws = AsyncMock()

        await client.send_sensor_data({"temperature": 25.0, "humidity": 60.0})

        client.ws.send.assert_awaited_once()
        sent = json.loads(client.ws.send.call_args[0][0])
        assert sent["type"] == "sensor_data"
        assert sent["temperature"] == 25.0
        assert sent["humidity"] == 60.0
        assert "test-board" in sent["status"]
        assert "timestamp" in sent

    @pytest.mark.asyncio
    async def test_send_sensor_data_strips_none(self):
        client = BoardWsClient("ws://test", board_id="test-board")
        client._connected = True
        client.ws = AsyncMock()

        await client.send_sensor_data({"temperature": 25.0, "humidity": None})

        sent = json.loads(client.ws.send.call_args[0][0])
        assert "temperature" in sent
        assert "humidity" not in sent  # None values stripped

    @pytest.mark.asyncio
    async def test_send_raw_when_disconnected(self):
        client = BoardWsClient("ws://test")
        client._connected = False
        client.ws = None
        # Should not raise
        await client.send_raw({"type": "test"})

    @pytest.mark.asyncio
    async def test_send_envelope_format(self):
        client = BoardWsClient("ws://test", board_id="test-board")
        client._connected = True
        client.ws = AsyncMock()

        await client.send_envelope("sensor.data", {"temp": 25.0}, src="board")

        sent = json.loads(client.ws.send.call_args[0][0])
        assert sent["type"] == "sensor.data"
        assert sent["payload"] == {"temp": 25.0}
        assert sent["src"] == "board"
        assert "id" in sent
        assert "ts" in sent


class TestHalMocks:
    def test_mock_gpio(self):
        from hal.mock.mock_gpio import MockGpio
        gpio = MockGpio()
        assert gpio.set_pin(1, True) == True
        assert gpio.read_pin(1) == True
        assert gpio.read_pin(99) == False

    def test_mock_camera(self):
        from hal.mock.mock_camera import MockCamera
        cam = MockCamera()
        assert cam.capture() == b"mock_frame"
        assert cam.start_stream(lambda x: None) == True
        cam.stop_stream()  # should not raise

    def test_mock_led(self):
        from hal.mock.mock_led import MockLed
        led = MockLed()
        assert led.set_color(255, 0, 0) == True
        assert led.blink(1.0) == True
        assert led.off() == True

    def test_mock_buzzer(self):
        from hal.mock.mock_buzzer import MockBuzzer
        buzzer = MockBuzzer()
        assert buzzer.beep(0.5, 1000) == True
        assert buzzer.off() == True
