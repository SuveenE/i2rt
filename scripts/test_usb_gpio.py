"""Standalone bring-up test for the USB-to-GPIO converter (Pi-bypass linear-rail GPIO).

Use this to verify the bestep USB-to-16-channel GPIO converter is wired and working
BEFORE running flow_base_controller.py with --device. It exercises the stack bottom-up:

  Phase 1 (--probe, default): open the port and read the firmware version.
  Phase 2 (--read,  default): poll the two limit switches so you can press each by
                              hand and confirm the reading flips (read-only, safe).
  Phase 3 (--test-brake):     toggle the brake channel. DANGEROUS: releasing the brake
                              with the rail motor unpowered lets the carriage drop under
                              gravity. Support the carriage / remove any payload first.
  Phase 4 (--test-backend):   drive the fork's SerialGpioBackend end-to-end (brake init +
                              limit-switch event callbacks), i.e. exactly what
                              flow_base_controller uses.

Channel wiring (converter channel <- controller BCM pin):
    ch1 = upper limit switch (BCM 5)
    ch2 = lower limit switch (BCM 6)
    ch3 = brake control      (BCM 12)

Examples:
    python i2rt/scripts/test_usb_gpio.py --device /dev/ttyUSB0
    python i2rt/scripts/test_usb_gpio.py --device /dev/ttyUSB0 --test-backend
    python i2rt/scripts/test_usb_gpio.py --device /dev/ttyUSB0 --test-brake   # CAUTION
"""

import argparse
import logging
import time

from i2rt.flow_base.gpio_backend import (
    BRAKE_CONTROL_GPIO,
    LOWER_LIMIT_GPIO,
    UPPER_LIMIT_GPIO,
    USB_GPIO_CHANNEL_MAP,
    SerialGpioBackend,
)
from i2rt.utils.usb_gpio_driver import PullMode, UsbToGpio

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("test_usb_gpio")

UPPER_CH = USB_GPIO_CHANNEL_MAP[UPPER_LIMIT_GPIO]
LOWER_CH = USB_GPIO_CHANNEL_MAP[LOWER_LIMIT_GPIO]
BRAKE_CH = USB_GPIO_CHANNEL_MAP[BRAKE_CONTROL_GPIO]


def phase_probe(dev: UsbToGpio) -> None:
    print("\n=== Phase 1: probe ===")
    try:
        version = dev.get_version()
        print(f"[OK] Board responded. Firmware/version: {version!r}")
    except Exception as e:
        print(f"[FAIL] Board did not respond to 'ver': {e}")
        print("       Check the cable, that the port is correct, and that nothing else")
        print("       (another process) already holds the serial port.")
        raise


def phase_read_limits(dev: UsbToGpio, seconds: float) -> None:
    print("\n=== Phase 2: read limit switches (read-only) ===")
    print(f"Reading ch{UPPER_CH} (upper) and ch{LOWER_CH} (lower) for {seconds:.0f}s.")
    print("Press each limit switch by hand and watch the value flip.\n")
    end = time.time() + seconds
    last = None
    while time.time() < end:
        readings = dev.read_inputs(min(UPPER_CH, LOWER_CH), max(UPPER_CH, LOWER_CH), pull=PullMode.PULL_UP)
        upper = readings.get(UPPER_CH)
        lower = readings.get(LOWER_CH)
        state = (upper, lower)
        if state != last:
            print(f"  upper(ch{UPPER_CH})={'HIGH' if upper else 'LOW '}   lower(ch{LOWER_CH})={'HIGH' if lower else 'LOW '}")
            last = state
        time.sleep(0.05)
    print("[done] If the values changed when you pressed the switches, wiring is good.")


def phase_test_brake(dev: UsbToGpio) -> None:
    print("\n=== Phase 3: brake toggle (DANGEROUS) ===")
    print("!! Releasing the brake with the rail motor unpowered can let the carriage drop.")
    print("!! Support the carriage and remove any payload before continuing.")
    if input("Type 'yes' to continue: ").strip().lower() != "yes":
        print("[skip] brake test skipped.")
        return
    try:
        print(f"Engaging brake (ch{BRAKE_CH} -> LOW) ...")
        dev.set_pins({BRAKE_CH: False})  # LOW == engaged
        time.sleep(1.0)
        input("Brake should be ENGAGED (holding). Press Enter to RELEASE ...")
        print(f"Releasing brake (ch{BRAKE_CH} -> HIGH) ...")
        dev.set_pins({BRAKE_CH: True})  # HIGH == released
        time.sleep(1.0)
        input("Brake should be RELEASED. Press Enter to re-engage and finish ...")
        dev.set_pins({BRAKE_CH: False})
        print("[done] Brake re-engaged.")
    except Exception as e:
        print(f"[FAIL] Brake toggle failed: {e}")
        raise


def phase_test_backend(device: str, seconds: float) -> None:
    print("\n=== Phase 4: SerialGpioBackend end-to-end ===")
    print("This is exactly what flow_base_controller uses on an x86 host.")
    backend = SerialGpioBackend(device)
    try:
        backend.initialize_brake()
        print("[OK] initialize_brake() succeeded.")

        def on_change(upper: bool, lower: bool) -> None:
            print(f"  limit change -> upper={upper} lower={lower}")

        backend.initialize_limit_switches(on_change)
        print(f"[OK] initialize_limit_switches() succeeded. Watching for {seconds:.0f}s.")
        print("Press each limit switch by hand; changes should print above.\n")
        end = time.time() + seconds
        while time.time() < end:
            time.sleep(0.1)
        print(f"[state] get_limit_states() -> {backend.get_limit_states()}")
    finally:
        backend.cleanup()
        print("[done] backend cleaned up.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device (default: /dev/ttyUSB0).")
    parser.add_argument("--seconds", type=float, default=15.0, help="Duration for the read/watch phases (default: 15).")
    parser.add_argument("--test-brake", action="store_true", help="Also toggle the brake channel (DANGEROUS).")
    parser.add_argument(
        "--test-backend",
        action="store_true",
        help="Also run the fork's SerialGpioBackend end-to-end (opens its own port).",
    )
    args = parser.parse_args()

    print(f"Using device: {args.device}")
    print(f"Channel map (BCM -> converter channel): {USB_GPIO_CHANNEL_MAP}")

    # Phases 1-3 share one low-level driver instance.
    with UsbToGpio(args.device) as dev:
        phase_probe(dev)
        phase_read_limits(dev, args.seconds)
        if args.test_brake:
            phase_test_brake(dev)

    # Phase 4 opens its own port via the backend, so it runs after the driver is closed.
    if args.test_backend:
        phase_test_backend(args.device, args.seconds)

    print("\nAll requested phases finished.")


if __name__ == "__main__":
    main()
