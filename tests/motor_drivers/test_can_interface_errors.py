import errno
from unittest.mock import MagicMock, patch

import pytest

from i2rt.exceptions import (
    CanDeviceNotFoundError,
    HardwareErrorCode,
    MotorCommunicationError,
)
from i2rt.motor_config_tool.utils import RawCanInterface
from i2rt.motor_drivers.can_interface import CanInterface


@pytest.mark.parametrize(
    ("interface_class", "bus_factory"),
    [
        (CanInterface, "i2rt.motor_drivers.can_interface.can.interface.Bus"),
        (RawCanInterface, "i2rt.motor_config_tool.utils.can.interface.Bus"),
    ],
)
def test_missing_can_device_has_stable_typed_error(
    interface_class: type[CanInterface] | type[RawCanInterface],
    bus_factory: str,
) -> None:
    os_error = OSError(errno.ENODEV, "No such device")

    with patch(bus_factory, side_effect=os_error):
        with pytest.raises(CanDeviceNotFoundError) as exc_info:
            interface_class(channel="can_linearbot", bustype="socketcan")

    error = exc_info.value
    assert error.code is HardwareErrorCode.CAN_DEVICE_NOT_FOUND
    assert str(error).startswith("[I2RT-CAN-001]")
    assert error.channel == "can_linearbot"
    assert error.to_dict() == {
        "code": "I2RT-CAN-001",
        "message": "CAN device 'can_linearbot' could not be opened with interface 'socketcan'.",
        "debug_steps": [
            "Make sure the Linearbot CAN cable is firmly connected.",
            "After reconnecting it, rerun robot:cans (window 0), robot:servers "
            "(window 1), and robot:flowbase (window 2), in that order.",
        ],
        "context": {
            "channel": "can_linearbot",
            "bustype": "socketcan",
        },
    }
    assert "Debug steps:" in str(error)
    assert "robot:flowbase (window 2)" in str(error)
    assert error.__cause__ is os_error


def test_debug_steps_are_yellow_when_terminal_supports_color() -> None:
    with patch("i2rt.exceptions._supports_color", return_value=True):
        error = CanDeviceNotFoundError(channel="can_linearbot", bustype="socketcan")

    rendered = str(error)
    assert "\033[93mDebug steps:\n" in rendered
    assert rendered.endswith("\033[0m")
    assert all("\033[" not in step for step in error.to_dict()["debug_steps"])


def test_other_can_open_errors_are_not_misclassified() -> None:
    os_error = OSError(errno.EACCES, "Permission denied")

    with patch("i2rt.motor_drivers.can_interface.can.interface.Bus", side_effect=os_error):
        with pytest.raises(OSError) as exc_info:
            CanInterface(channel="can_linearbot", bustype="socketcan")

    assert exc_info.value is os_error
    assert not isinstance(exc_info.value, CanDeviceNotFoundError)


def test_motor_nonresponse_has_stable_typed_error() -> None:
    interface = CanInterface.__new__(CanInterface)
    interface.channel = "can_linearbot"
    interface.name = "linear_rail_vehicle"
    interface.bus = MagicMock()
    interface._receive_message = MagicMock(return_value=None)

    with pytest.raises(MotorCommunicationError) as exc_info:
        interface._send_message_get_response(
            id=3,
            motor_id=3,
            data=[0xFF] * 7 + [0xFC],
            max_retry=2,
            expected_id=3,
        )

    error = exc_info.value
    assert error.code is HardwareErrorCode.MOTOR_COMMUNICATION_FAILED
    assert str(error).startswith("[I2RT-CAN-002]")
    assert error.motor_id == 3
    assert error.interface_name == "linear_rail_vehicle"
    assert error.channel == "can_linearbot"
    assert error.attempts == 2
    assert "emergency stop (E-stop)" in str(error)
    assert "Check and reseat the Linearbot CAN cable." in str(error)
    assert error.to_dict()["context"] == {
        "motor_id": 3,
        "interface_name": "linear_rail_vehicle",
        "channel": "can_linearbot",
        "attempts": 2,
    }
    assert error.__cause__ is None
