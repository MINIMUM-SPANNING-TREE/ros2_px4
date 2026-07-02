#!/usr/bin/env python3
"""
Integration test for the RDK X5 mock client WebSocket connection.

Connects to the backend WebSocket, sends a sensor_data message,
waits for a response, and exits with an appropriate status code.
"""

import argparse
import asyncio
import json
import logging
import sys
import time

import websockets

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


async def run_test(url: str, timeout: float) -> int:
    """Connect, send a sensor_data message, wait for a response.

    Returns 0 on success, 1 on any failure.
    """
    try:
        logger.info("Connecting to %s ...", url)
        async with websockets.connect(url, open_timeout=timeout) as ws:
            logger.info("Connected to backend")

            payload = {
                "type": "sensor_data",
                "temperature": 25.0,
                "humidity": 60.0,
                "pressure": 101.3,
                "status": "test-board",
                "timestamp": int(time.time() * 1000),
            }
            await ws.send(json.dumps(payload))
            logger.info("Sent sensor_data message")

            response = await asyncio.wait_for(ws.recv(), timeout=5)
            logger.info("Received response: %s", response)

            logger.info("Test PASSED")
            return 0

    except asyncio.TimeoutError:
        logger.error("Timed out waiting for a response (5 s)")
        return 1

    except websockets.exceptions.ConnectionClosed as exc:
        logger.error("Connection closed unexpectedly: %s", exc)
        return 1

    except Exception as exc:
        logger.error("Test FAILED: %s", exc)
        return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Integration test for the RDK X5 mock client WebSocket connection.",
    )
    parser.add_argument(
        "--url",
        default="ws://localhost:9200/ws/board",
        help="WebSocket URL to connect to (default: ws://localhost:9200/ws/board)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Connection / overall timeout in seconds (default: 10)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exit_code = asyncio.run(run_test(args.url, args.timeout))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
