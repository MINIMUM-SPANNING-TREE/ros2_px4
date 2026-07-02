import logging
from hal.base import IGpio

logger = logging.getLogger(__name__)

class MockGpio(IGpio):
    def __init__(self):
        self._pins = {}
    def set_pin(self, pin: int, value: bool) -> bool:
        self._pins[pin] = value
        logger.info(f"[MOCK] GPIO pin {pin} -> {value}")
        return True
    def read_pin(self, pin: int) -> bool:
        return self._pins.get(pin, False)
