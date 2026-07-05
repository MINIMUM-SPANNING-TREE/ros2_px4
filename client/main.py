#!/usr/bin/env python3
"""RDK X5 Mock Edge Client"""

import argparse
import asyncio
import logging
import yaml
from client.ws_client import BoardWsClient
from client.sensor_reporter import SensorReporter
from hal.mock.mock_gpio import MockGpio
from hal.mock.mock_camera import MockCamera
from hal.mock.mock_led import MockLed
from hal.mock.mock_buzzer import MockBuzzer

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(name)s %(levelname)s: %(message)s", datefmt="%H:%M:%S")
logger = logging.getLogger("rdk_x5")

async def main():
    parser = argparse.ArgumentParser(description="RDK X5 Mock Edge Client")
    parser.add_argument("--url", default="ws://192.168.3.1:9200/ws/board")
    parser.add_argument("--config", default="config/default.yaml")
    args = parser.parse_args()

    try:
        with open(args.config) as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        config = {}

    url = args.url
    if url == "ws://localhost:9200/ws/board" and config.get("backend", {}).get("url"):
        url = config["backend"]["url"]

    gpio, camera, led, buzzer = MockGpio(), MockCamera(), MockLed(), MockBuzzer()
    ws_client = BoardWsClient(url)

    async def on_command(cmd):
        # The backend sends CommandMessage with fields:
        #   { "type":"command", "command":"led_on", "parameters":{...}, "id":"..." }
        command_name = cmd.get("command", "")
        params = cmd.get("parameters", {}) or {}
        logger.info("Executing command: %s with params %s", command_name, params)

        if command_name.startswith("led"):
            if "color" in command_name or "set_color" in command_name:
                led.set_color(params.get("r", 0), params.get("g", 0), params.get("b", 0))
            elif "blink" in command_name:
                led.blink(params.get("frequency", 1.0))
            else:
                led.off()
            return {"success": True}
        elif command_name.startswith("buzzer"):
            if "beep" in command_name:
                buzzer.beep(params.get("duration", 0.5), params.get("frequency", 1000))
            else:
                buzzer.off()
            return {"success": True}
        elif command_name.startswith("gpio"):
            pin = params.get("pin", 0)
            value = params.get("value", False)
            gpio.set_pin(pin, value)
            return {"success": True}
        elif command_name.startswith("camera"):
            if "capture" in command_name:
                camera.capture()
            return {"success": True}
        else:
            logger.warning("Unknown command: %s", command_name)
            return {"success": False, "error": f"Unknown command: {command_name}"}

    ws_client.on_command(on_command)
    reporter = SensorReporter(ws_client, config)

    logger.info("RDK X5 Mock Client starting -> %s", url)
    tasks = [asyncio.create_task(ws_client.connect_loop()), asyncio.create_task(reporter.run())]
    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        pass
    finally:
        for t in tasks: t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: pass
