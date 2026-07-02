import asyncio
import logging
import random

logger = logging.getLogger(__name__)

class SensorReporter:
    def __init__(self, ws_client, config: dict):
        self.ws = ws_client
        self.config = config

    async def run(self):
        while True:
            await asyncio.sleep(5)
            if not self.ws.connected:
                continue
            data = {}
            sensors = self.config.get("sensors", {})
            if sensors.get("temperature", {}).get("enabled", True):
                base = sensors["temperature"].get("base_value", 25.0)
                var = sensors["temperature"].get("variance", 2.0)
                data["temperature"] = round(base + random.uniform(-var, var), 1)
            if sensors.get("humidity", {}).get("enabled", True):
                base = sensors["humidity"].get("base_value", 60.0)
                var = sensors["humidity"].get("variance", 5.0)
                data["humidity"] = round(base + random.uniform(-var, var), 1)
            # The backend SensorData model has "pressure" and "light" fields.
            # Map our mock sensors to those fields.
            if sensors.get("distance", {}).get("enabled", True):
                # Map ultrasonic distance to a pseudo-pressure value (kPa)
                base = sensors["distance"].get("base_value", 2.0)
                var = sensors["distance"].get("variance", 0.5)
                distance = round(max(0.1, base + random.uniform(-var, var)), 2)
                data["pressure"] = round(101.3 + distance * 0.5, 1)  # mock atmospheric pressure
            if data:
                logger.info("Sensor data: %s", data)
                await self.ws.send_sensor_data(data)
