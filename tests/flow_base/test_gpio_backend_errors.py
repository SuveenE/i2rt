import sys
import types

import pytest

from i2rt.exceptions import GpioSerialDeviceUnavailableError, HardwareErrorCode
from i2rt.flow_base.gpio_backend import SerialSatelliteBackend


@pytest.fixture
def fake_serial(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    """Stand in for pyserial so port-open failures can be raised on demand.

    ``SerialSatelliteBackend`` imports ``serial`` lazily inside ``__init__``, so
    replacing the module entry is enough and pyserial itself is not needed here.
    """
    module = types.ModuleType("serial")

    class SerialException(IOError):
        pass

    def _serial(device: str, baudrate: int | None = None, timeout: float | None = None) -> None:
        raise module.open_error

    module.SerialException = SerialException
    module.Serial = _serial
    module.open_error = None
    monkeypatch.setitem(sys.modules, "serial", module)
    return module


def test_missing_gpio_serial_device_has_stable_typed_error(fake_serial: types.ModuleType) -> None:
    open_error = fake_serial.SerialException(
        "[Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'"
    )
    fake_serial.open_error = open_error

    with pytest.raises(GpioSerialDeviceUnavailableError) as exc_info:
        SerialSatelliteBackend("/dev/ttyACM0")

    error = exc_info.value
    assert error.code is HardwareErrorCode.GPIO_SERIAL_DEVICE_UNAVAILABLE
    assert str(error).startswith("[I2RT-GPIO-001]")
    assert error.device == "/dev/ttyACM0"
    assert error.to_dict()["context"] == {"device": "/dev/ttyACM0"}
    assert "Debug steps:" in str(error)
    assert "press the reset button" in str(error)
    assert "lerobot-doctor linearbot --gpio-mode pi-usb" in str(error)
    assert "robot:flowbase (window 2)" in str(error)
    assert error.__cause__ is open_error
    # It replaces pyserial's SerialException, so except OSError must keep matching.
    assert isinstance(error, OSError)


def test_other_serial_open_errors_are_not_misclassified(fake_serial: types.ModuleType) -> None:
    value_error = ValueError("invalid baudrate")
    fake_serial.open_error = value_error

    with pytest.raises(ValueError) as exc_info:
        SerialSatelliteBackend("/dev/ttyACM0")

    assert exc_info.value is value_error
    assert not isinstance(exc_info.value, GpioSerialDeviceUnavailableError)
