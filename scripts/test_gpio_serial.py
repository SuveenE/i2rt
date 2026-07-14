"""Standalone bring-up test for the Pi GPIO satellite over a direct USB serial link.

Use this to verify the USB serial path (Pi ``gpio_serial_satellite_server`` <-> PC
``SerialSatelliteBackend``) is working BEFORE running flow_base_controller.py with
``--gpio-serial``. It exercises the stack bottom-up:

  Phase 1 (--probe, default):   send a raw ``GET`` and confirm the satellite replies
                                ``L <upper> <lower>`` (the port and server are alive).
  Phase 2 (--read,  default):   poll the two limit switches so you can press each by
                                hand and confirm the reading flips (read-only, safe).
  Phase 3 (--test-brake):       toggle the brake. DANGEROUS: releasing the brake with
                                the rail motor unpowered lets the carriage drop under
                                gravity. Support the carriage / remove any payload first.
  Phase 4 (--test-backend):     drive SerialSatelliteBackend end-to-end (brake init +
                                limit-switch event callbacks), i.e. exactly what
                                flow_base_controller uses with --gpio-serial.

On a Pi 5 configured as a CDC-ACM gadget, the Pi's ``/dev/ttyGS0`` enumerates as
``/dev/ttyACM0`` on this PC (see scripts/setup_pi_usb_gpio_gadget.sh).

Examples:
    python i2rt/scripts/test_gpio_serial.py --device /dev/ttyACM0
    python i2rt/scripts/test_gpio_serial.py --device /dev/ttyACM0 --test-backend
    python i2rt/scripts/test_gpio_serial.py --device /dev/ttyACM0 --test-brake   # CAUTION
"""

import argparse
import logging
import time

from i2rt.flow_base.gpio_backend import SerialSatelliteBackend

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("test_gpio_serial")


def phase_probe(backend: SerialSatelliteBackend) -> None:
    print("\n=== Phase 1: probe ===")
    try:
        upper, lower = backend._query_limits()
        print(f"[OK] Satellite responded. Limits: upper={upper} lower={lower}")
    except Exception as e:
        print(f"[FAIL] Satellite did not respond to 'GET': {e}")
        print("       Check the USB cable, that --device is correct (/dev/ttyACM0), and")
        print("       that gpio_serial_satellite_server.py is running on the Pi (start it")
        print("       over SSH: python i2rt/flow_base/gpio_serial_satellite_server.py --port /dev/ttyGS0).")
        raise


def phase_read_limits(backend: SerialSatelliteBackend, seconds: float) -> None:
    print("\n=== Phase 2: read limit switches (read-only) ===")
    print(f"Polling upper/lower limits for {seconds:.0f}s.")
    print("Press each limit switch by hand and watch the value flip.\n")
    end = time.time() + seconds
    last = None
    while time.time() < end:
        upper, lower = backend._query_limits()
        state = (upper, lower)
        if state != last:
            print(f"  upper={'HIGH' if upper else 'LOW '}   lower={'HIGH' if lower else 'LOW '}")
            last = state
        time.sleep(0.05)
    print("[done] If the values changed when you pressed the switches, the link is good.")


def phase_test_brake(backend: SerialSatelliteBackend) -> None:
    print("\n=== Phase 3: brake toggle (DANGEROUS) ===")
    print("!! Releasing the brake with the rail motor unpowered can let the carriage drop.")
    print("!! Support the carriage and remove any payload before continuing.")
    if input("Type 'yes' to continue: ").strip().lower() != "yes":
        print("[skip] brake test skipped.")
        return
    try:
        backend.initialize_brake()
        print("Engaging brake ...")
        backend.set_brake(True)  # True == engaged
        time.sleep(1.0)
        input("Brake should be ENGAGED (holding). Press Enter to RELEASE ...")
        print("Releasing brake ...")
        backend.set_brake(False)
        time.sleep(1.0)
        input("Brake should be RELEASED. Press Enter to re-engage and finish ...")
        backend.set_brake(True)
        print("[done] Brake re-engaged.")
    except Exception as e:
        print(f"[FAIL] Brake toggle failed: {e}")
        raise


def phase_test_backend(backend: SerialSatelliteBackend, seconds: float) -> None:
    print("\n=== Phase 4: SerialSatelliteBackend end-to-end ===")
    print("This is exactly what flow_base_controller uses with --gpio-serial.")
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/ttyACM0", help="Serial device (default: /dev/ttyACM0).")
    parser.add_argument("--seconds", type=float, default=15.0, help="Duration for the read/watch phases (default: 15).")
    parser.add_argument("--test-brake", action="store_true", help="Also toggle the brake channel (DANGEROUS).")
    parser.add_argument(
        "--test-backend",
        action="store_true",
        help="Also run SerialSatelliteBackend end-to-end (brake + limit-switch callbacks).",
    )
    args = parser.parse_args()

    print(f"Using device: {args.device}")

    backend = SerialSatelliteBackend(args.device)
    try:
        phase_probe(backend)
        phase_read_limits(backend, args.seconds)
        if args.test_brake:
            phase_test_brake(backend)
        if args.test_backend:
            phase_test_backend(backend, args.seconds)
    finally:
        backend.cleanup()
        print("[done] backend cleaned up.")

    print("\nAll requested phases finished.")


if __name__ == "__main__":
    main()
