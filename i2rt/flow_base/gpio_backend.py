import abc
import logging
import sys
import threading
import time
from typing import Callable, Optional, Tuple

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("GPIOBackend")

BRAKE_CONTROL_GPIO = 12
UPPER_LIMIT_GPIO = 5
LOWER_LIMIT_GPIO = 6

DEFAULT_GPIO_SATELLITE_PORT = 8765


class GPIOBackend(abc.ABC):
    """Abstract interface for GPIO operations (brake + limit switches)."""

    @abc.abstractmethod
    def initialize_brake(self) -> None:
        """Set up the brake control output."""

    @abc.abstractmethod
    def set_brake(self, engaged: bool) -> None:
        """Engage (*True*) or release (*False*) the brake."""

    @abc.abstractmethod
    def initialize_limit_switches(
        self, on_limit_change: Callable[[bool, bool], None]
    ) -> None:
        """Set up limit switch monitoring and register a state-change callback.

        Args:
            on_limit_change: Called with ``(upper_triggered, lower_triggered)``
                whenever either limit switch changes state.
        """

    @abc.abstractmethod
    def get_limit_states(self) -> Tuple[bool, bool]:
        """Return current ``(upper_triggered, lower_triggered)``."""

    @abc.abstractmethod
    def cleanup(self) -> None:
        """Release all resources (GPIO pins, threads, connections)."""


# ---------------------------------------------------------------------------
# Local backend — direct RPi.GPIO on the same machine
# ---------------------------------------------------------------------------


class LocalGPIOBackend(GPIOBackend):
    """Backend that talks directly to RPi.GPIO on the local machine."""

    def __init__(self):
        from RPi import GPIO  # noqa: N811

        self._GPIO = GPIO
        self._gpio_mode_set = False
        self._on_limit_change: Optional[Callable] = None
        self._upper = False
        self._lower = False
        self._lock = threading.Lock()

    def _ensure_gpio_mode(self) -> None:
        if not self._gpio_mode_set:
            try:
                self._GPIO.setmode(self._GPIO.BCM)
                self._gpio_mode_set = True
            except RuntimeError as e:
                if "mode" in str(e).lower() or "already" in str(e).lower():
                    self._gpio_mode_set = True
                else:
                    raise

    # -- brake --

    def initialize_brake(self) -> None:
        try:
            self._ensure_gpio_mode()
            self._GPIO.setup(BRAKE_CONTROL_GPIO, self._GPIO.OUT)
            logger.info("Brake GPIO initialized (local)")
        except RuntimeError as e:
            if "mode" in str(e).lower() or "already" in str(e).lower():
                try:
                    self._GPIO.setup(BRAKE_CONTROL_GPIO, self._GPIO.OUT)
                except Exception as setup_err:
                    logger.warning(f"Brake GPIO setup failed: {setup_err}")
            else:
                logger.warning(f"Failed to initialize brake GPIO: {e}")

    def set_brake(self, engaged: bool) -> None:
        try:
            self._GPIO.output(
                BRAKE_CONTROL_GPIO,
                self._GPIO.LOW if engaged else self._GPIO.HIGH,
            )
            logger.info(f"Brake {'engaged' if engaged else 'released'}")
        except Exception as e:
            logger.error(f"Failed to {'engage' if engaged else 'release'} brake: {e}")
            raise

    # -- limit switches --

    def initialize_limit_switches(
        self, on_limit_change: Callable[[bool, bool], None]
    ) -> None:
        self._on_limit_change = on_limit_change
        self._ensure_gpio_mode()

        GPIO = self._GPIO
        GPIO.setup(UPPER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LOWER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._upper = GPIO.input(UPPER_LIMIT_GPIO) == GPIO.HIGH
        self._lower = GPIO.input(LOWER_LIMIT_GPIO) == GPIO.HIGH

        if self._on_limit_change:
            self._on_limit_change(self._upper, self._lower)

        GPIO.add_event_detect(
            UPPER_LIMIT_GPIO,
            GPIO.BOTH,
            callback=self._gpio_callback,
            bouncetime=50,
        )
        GPIO.add_event_detect(
            LOWER_LIMIT_GPIO,
            GPIO.BOTH,
            callback=self._gpio_callback,
            bouncetime=50,
        )
        logger.info("Limit switch GPIO initialized (local)")

    def _gpio_callback(self, channel: int) -> None:
        GPIO = self._GPIO
        try:
            upper = GPIO.input(UPPER_LIMIT_GPIO) == GPIO.HIGH
            lower = GPIO.input(LOWER_LIMIT_GPIO) == GPIO.HIGH
            with self._lock:
                self._upper = upper
                self._lower = lower
            if self._on_limit_change:
                self._on_limit_change(upper, lower)
        except Exception as e:
            logger.error(f"Error in GPIO callback: {e}")

    def get_limit_states(self) -> Tuple[bool, bool]:
        with self._lock:
            return self._upper, self._lower

    # -- cleanup --

    def cleanup(self) -> None:
        try:
            self._ensure_gpio_mode()
            self._GPIO.remove_event_detect(UPPER_LIMIT_GPIO)
            self._GPIO.remove_event_detect(LOWER_LIMIT_GPIO)
            self._GPIO.cleanup()
            logger.info("Local GPIO cleaned up")
        except Exception as e:
            logger.warning(f"GPIO cleanup error: {e}")


# ---------------------------------------------------------------------------
# Remote backend — talks to gpio_satellite_server over portal RPC
# ---------------------------------------------------------------------------


class RemoteGPIOBackend(GPIOBackend):
    """Backend that talks to a ``gpio_satellite_server`` via portal RPC."""

    def __init__(self, host: str, poll_interval: float = 0.01):
        import portal

        self._client = portal.Client(host)
        self._poll_interval = poll_interval
        self._on_limit_change: Optional[Callable] = None
        self._upper = False
        self._lower = False
        self._lock = threading.Lock()
        self._polling = False
        self._poll_thread: Optional[threading.Thread] = None
        logger.info(f"RemoteGPIOBackend connected to {host}")

    # -- brake --

    def initialize_brake(self) -> None:
        self._client.initialize_brake({}).result()
        logger.info("Brake initialized (remote)")

    def set_brake(self, engaged: bool) -> None:
        self._client.set_brake({"engaged": engaged}).result()
        logger.info(f"Brake {'engaged' if engaged else 'released'} (remote)")

    # -- limit switches --

    def initialize_limit_switches(
        self, on_limit_change: Callable[[bool, bool], None]
    ) -> None:
        self._on_limit_change = on_limit_change
        self._client.initialize_limit_switches({}).result()

        states = self._client.get_limit_states({}).result()
        with self._lock:
            self._upper = states["upper"]
            self._lower = states["lower"]

        if self._on_limit_change:
            self._on_limit_change(self._upper, self._lower)

        self._polling = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        logger.info("Limit switch monitoring started (remote, polling)")

    def _poll_loop(self) -> None:
        while self._polling:
            try:
                states = self._client.get_limit_states({}).result()
                upper = states["upper"]
                lower = states["lower"]

                changed = False
                with self._lock:
                    if upper != self._upper or lower != self._lower:
                        self._upper = upper
                        self._lower = lower
                        changed = True

                if changed and self._on_limit_change:
                    self._on_limit_change(upper, lower)
            except Exception as e:
                logger.error(f"Remote limit state poll error: {e}")

            time.sleep(self._poll_interval)

    def get_limit_states(self) -> Tuple[bool, bool]:
        with self._lock:
            return self._upper, self._lower

    # -- cleanup --

    def cleanup(self) -> None:
        self._polling = False
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        try:
            self._client.cleanup({}).result()
        except Exception as e:
            logger.warning(f"Remote cleanup error: {e}")
        logger.info("Remote GPIO backend cleaned up")


# ---------------------------------------------------------------------------
# No-op backend — used when GPIO is unavailable and no remote host given
# ---------------------------------------------------------------------------


class NoopGPIOBackend(GPIOBackend):
    """No-op backend when GPIO is unavailable and no remote host is configured."""

    def __init__(self):
        logger.warning(
            "Using NoopGPIOBackend — brake control and limit switches will not function."
        )

    def initialize_brake(self) -> None:
        logger.warning("Brake initialization skipped (no GPIO backend)")

    def set_brake(self, engaged: bool) -> None:
        logger.debug(f"Brake {'engaged' if engaged else 'released'} (no-op)")

    def initialize_limit_switches(
        self, on_limit_change: Callable[[bool, bool], None]
    ) -> None:
        logger.warning("Limit switch initialization skipped (no GPIO backend)")

    def get_limit_states(self) -> Tuple[bool, bool]:
        return False, False

    def cleanup(self) -> None:
        pass
