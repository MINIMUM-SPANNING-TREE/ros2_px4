import logging
from hal.base import IBuzzer

logger = logging.getLogger(__name__)

class MockBuzzer(IBuzzer):
    def beep(self, duration: float, frequency: float) -> bool:
        logger.info(f"[MOCK] Buzzer {duration}s {frequency}Hz")
        return True
    def off(self) -> bool:
        logger.info("[MOCK] Buzzer off")
        return True
