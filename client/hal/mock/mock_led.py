import logging
from hal.base import ILed

logger = logging.getLogger(__name__)

class MockLed(ILed):
    def set_color(self, r: int, g: int, b: int) -> bool:
        logger.info(f"[MOCK] LED -> ({r},{g},{b})")
        return True
    def blink(self, frequency: float) -> bool:
        logger.info(f"[MOCK] LED blink {frequency}Hz")
        return True
    def off(self) -> bool:
        logger.info("[MOCK] LED off")
        return True
