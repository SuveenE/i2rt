"""Typed i2rt exceptions with stable, machine-readable error codes."""

import os
import sys
from enum import Enum, unique

_BRIGHT_YELLOW = "\033[93m"
_RESET_COLOR = "\033[0m"


def _supports_color() -> bool:
    """Return whether operator guidance should use ANSI terminal colors."""
    if "NO_COLOR" in os.environ:
        return False
    force_color = os.environ.get("FORCE_COLOR")
    if force_color is not None:
        return force_color.lower() not in {"", "0", "false", "no"}
    return os.environ.get("TERM") != "dumb" and bool(getattr(sys.stderr, "isatty", lambda: False)())


@unique
class HardwareErrorCode(str, Enum):
    """Stable identifiers for hardware failures.

    Enum member names and values are part of the public API. Troubleshooting
    tools should match these codes instead of exception message text.
    """

    CAN_DEVICE_NOT_FOUND = "I2RT-CAN-001"
    MOTOR_COMMUNICATION_FAILED = "I2RT-CAN-002"
    GPIO_SERIAL_DEVICE_UNAVAILABLE = "I2RT-GPIO-001"

    def __str__(self) -> str:
        return self.value


class I2RTHardwareError(RuntimeError):
    """Base class for hardware failures that operators can act on."""

    code: HardwareErrorCode

    def __init__(
        self,
        message: str,
        *,
        debug_steps: tuple[str, ...] = (),
        **context: object,
    ) -> None:
        self.message = message
        self.debug_steps = debug_steps
        self.context = context
        rendered_message = f"[{self.code.value}] {message}"
        if debug_steps:
            steps = "\n".join(f"  {index}. {step}" for index, step in enumerate(debug_steps, start=1))
            debug_message = f"Debug steps:\n{steps}"
            if _supports_color():
                debug_message = f"{_BRIGHT_YELLOW}{debug_message}{_RESET_COLOR}"
            rendered_message = f"{rendered_message}\n\n{debug_message}"
        super().__init__(rendered_message)

    def to_dict(self) -> dict[str, object]:
        """Return a serializable representation for logs and RPC boundaries."""
        return {
            "code": self.code.value,
            "message": self.message,
            "debug_steps": list(self.debug_steps),
            "context": dict(self.context),
        }


class I2RTHardwareIOError(I2RTHardwareError, OSError):
    """Base for hardware failures raised in place of an ``OSError`` from a driver.

    These replace an OSError that callers may already guard with ``except OSError``
    (SocketCAN's ``ENODEV``, pyserial's ``SerialException``), so ``OSError`` stays in
    the MRO to keep those handlers matching. ``errno`` is not populated; the original
    exception remains available as ``__cause__``.
    """


class CanDeviceNotFoundError(I2RTHardwareIOError):
    """Raised when the operating system cannot find a configured CAN device."""

    code = HardwareErrorCode.CAN_DEVICE_NOT_FOUND

    def __init__(self, *, channel: str, bustype: str) -> None:
        self.channel = channel
        self.bustype = bustype
        super().__init__(
            f"CAN device {channel!r} could not be opened with interface {bustype!r}.",
            debug_steps=(
                "Make sure the Linearbot CAN cable is firmly connected.",
                "After reconnecting it, rerun robot:cans (window 0), robot:servers "
                "(window 1), and robot:flowbase (window 2), in that order.",
            ),
            channel=channel,
            bustype=bustype,
        )


class GpioSerialDeviceUnavailableError(I2RTHardwareIOError):
    """Raised when the Pi's USB GPIO serial gadget cannot be opened on this host."""

    code = HardwareErrorCode.GPIO_SERIAL_DEVICE_UNAVAILABLE

    def __init__(self, *, device: str) -> None:
        self.device = device
        super().__init__(
            f"Pi USB GPIO serial device {device!r} could not be opened.",
            debug_steps=(
                "Check the status light on the Raspberry Pi. If it is red, press the reset "
                "button on the Pi and wait until the light is yellow and no longer blinking; "
                "the serial gadget is only re-exported once the Pi has finished booting.",
                "Run 'lerobot-doctor linearbot --gpio-mode pi-usb' and confirm the Pi USB "
                "GPIO serial check passes.",
                "If the device still does not appear after a reset, check the USB-C cable "
                "between the Pi and the Linearbot PC.",
                f"If {device} exists but cannot be opened, check that your user is in the "
                "'dialout' group.",
                "Once the check passes, rerun robot:flowbase (window 2).",
            ),
            device=device,
        )


class MotorCommunicationError(I2RTHardwareError):
    """Raised after all attempts to communicate with a CAN motor fail."""

    code = HardwareErrorCode.MOTOR_COMMUNICATION_FAILED

    def __init__(
        self,
        *,
        motor_id: int,
        interface_name: str,
        channel: str,
        attempts: int,
    ) -> None:
        self.motor_id = motor_id
        self.interface_name = interface_name
        self.channel = channel
        self.attempts = attempts
        super().__init__(
            f"Failed to communicate with motor {motor_id} on {interface_name!r} "
            f"at CAN channel {channel!r} after {attempts} attempts.",
            debug_steps=(
                "Make sure the emergency stop (E-stop) was not pressed accidentally; "
                "release or reset it if necessary.",
                "Check that the Linearbot and motor power are on.",
                "Check and reseat the Linearbot CAN cable.",
                "Rerun robot:cans (window 0), robot:servers (window 1), and robot:flowbase (window 2), in that order.",
            ),
            motor_id=motor_id,
            interface_name=interface_name,
            channel=channel,
            attempts=attempts,
        )
