import logging
from hal.base import ICamera

logger = logging.getLogger(__name__)

class MockCamera(ICamera):
    def capture(self) -> bytes:
        logger.info("[MOCK] Camera capture")
        return b"mock_frame"
    def start_stream(self, callback) -> bool:
        logger.info("[MOCK] Camera stream started")
        return True
    def stop_stream(self) -> None:
        logger.info("[MOCK] Camera stream stopped")
