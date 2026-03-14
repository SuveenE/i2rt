"""GPIO satellite server for linear rail brake and limit switch control.

Run this on the Raspberry Pi to expose GPIO operations over the network
via portal RPC.  The mini PC connects using ``RemoteGPIOBackend``.

Usage::

    python gpio_satellite_server.py --port 8765
"""

import argparse
import atexit
import logging
import sys
import threading

import portal
from RPi import GPIO

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("GPIOSatelliteServer")

# Pin definitions — must match gpio_backend.py
BRAKE_CONTROL_GPIO = 12
UPPER_LIMIT_GPIO = 5
LOWER_LIMIT_GPIO = 6

DEFAULT_PORT = 8765

# Shared state ----------------------------------------------------------------
_lock = threading.Lock()
_upper_limit = False
_lower_limit = False
_limit_switches_initialized = False
_brake_initialized = False


def _ensure_gpio_mode() -> None:
    """(Re-)set BCM pin numbering if it was cleared by GPIO.cleanup()."""
    try:
        GPIO.setmode(GPIO.BCM)
    except RuntimeError as e:
        if "already" not in str(e).lower():
            raise


# GPIO interrupt callbacks -----------------------------------------------------


def _upper_callback(channel: int) -> None:
    global _upper_limit
    state = GPIO.input(UPPER_LIMIT_GPIO) == GPIO.HIGH
    with _lock:
        _upper_limit = state
    if state:
        logger.warning("Upper limit switch triggered")
    else:
        logger.info("Upper limit switch released")


def _lower_callback(channel: int) -> None:
    global _lower_limit
    state = GPIO.input(LOWER_LIMIT_GPIO) == GPIO.HIGH
    with _lock:
        _lower_limit = state
    if state:
        logger.warning("Lower limit switch triggered")
    else:
        logger.info("Lower limit switch released")


# RPC handlers -----------------------------------------------------------------


def initialize_brake(input_dict=None):
    global _brake_initialized
    _ensure_gpio_mode()
    GPIO.setup(BRAKE_CONTROL_GPIO, GPIO.OUT)
    _brake_initialized = True
    logger.info("Brake GPIO initialized")


def set_brake(input_dict):
    global _brake_initialized
    if not _brake_initialized:
        initialize_brake()
    _ensure_gpio_mode()
    engaged = input_dict["engaged"]
    GPIO.output(BRAKE_CONTROL_GPIO, GPIO.LOW if engaged else GPIO.HIGH)
    logger.info(f"Brake {'engaged' if engaged else 'released'}")


def initialize_limit_switches(input_dict=None):
    global _limit_switches_initialized, _upper_limit, _lower_limit

    if _limit_switches_initialized:
        logger.debug("Limit switches already initialized")
        return

    _ensure_gpio_mode()
    GPIO.setup(UPPER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LOWER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    with _lock:
        _upper_limit = GPIO.input(UPPER_LIMIT_GPIO) == GPIO.HIGH
        _lower_limit = GPIO.input(LOWER_LIMIT_GPIO) == GPIO.HIGH

    GPIO.add_event_detect(
        UPPER_LIMIT_GPIO, GPIO.BOTH, callback=_upper_callback, bouncetime=50
    )
    GPIO.add_event_detect(
        LOWER_LIMIT_GPIO, GPIO.BOTH, callback=_lower_callback, bouncetime=50
    )
    _limit_switches_initialized = True
    logger.info("Limit switch GPIO initialized with event callbacks")


def get_limit_states(input_dict=None):
    with _lock:
        return {"upper": _upper_limit, "lower": _lower_limit}


def cleanup(input_dict=None):
    global _limit_switches_initialized, _brake_initialized
    try:
        GPIO.remove_event_detect(UPPER_LIMIT_GPIO)
        GPIO.remove_event_detect(LOWER_LIMIT_GPIO)
        GPIO.cleanup()
        _limit_switches_initialized = False
        _brake_initialized = False
        logger.info("GPIO cleaned up")
    except Exception as e:
        logger.warning(f"GPIO cleanup error: {e}")


# Entry point ------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="GPIO Satellite Server for linear rail brake and limit switches"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Server port (default: {DEFAULT_PORT})",
    )
    args = parser.parse_args()

    atexit.register(cleanup)

    server = portal.Server(args.port)
    server.bind("initialize_brake", initialize_brake)
    server.bind("set_brake", set_brake)
    server.bind("initialize_limit_switches", initialize_limit_switches)
    server.bind("get_limit_states", get_limit_states)
    server.bind("cleanup", cleanup)

    logger.info(f"GPIO Satellite Server starting on port {args.port}")
    server.start(block=True)


if __name__ == "__main__":
    main()
