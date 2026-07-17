"""GPIO satellite server that talks to the control host over a USB serial link.

This is the USB-cable counterpart of the network ``gpio_satellite_server``: it runs
on the Raspberry Pi, drives the linear rail's brake + limit switches through the Pi's
native GPIO (via :class:`~i2rt.flow_base.gpio_backend.LocalGPIOBackend`), and exposes
them over a simple newline-delimited ASCII protocol on a serial device.

On a Pi 5 configured as a USB CDC-ACM gadget (see the flow_base README), the serial
device is ``/dev/ttyGS0`` on the Pi and enumerates as ``/dev/ttyACM0`` on the host PC,
so the host's :class:`~i2rt.flow_base.gpio_backend.SerialSatelliteBackend` can drive
the rail over the USB cable with no network / SSH in the loop.

Protocol (one ``<command>\\n`` request -> one ``<reply>\\n`` line):
    ``INIT_BRAKE``          -> ``OK``
    ``BRAKE 1`` / ``BRAKE 0`` -> ``OK`` (1 = engaged)
    ``INIT_LIMITS``         -> ``OK``
    ``GET``                 -> ``L <upper> <lower>`` (e.g. ``L 1 0``)
    ``CLEANUP``             -> ``OK``
    (anything else / errors) -> ``ERR <message>``

Run on the Pi:
    python i2rt/flow_base/gpio_serial_satellite_server.py --port /dev/ttyGS0
"""

import argparse
import logging

from i2rt.flow_base.gpio_backend import LocalGPIOBackend

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("gpio_serial_satellite")


def _handle_command(backend: LocalGPIOBackend, line: str) -> str:
    """Dispatch one protocol line to the local GPIO backend and return the reply text."""
    parts = line.split()
    if not parts:
        return "ERR empty command"
    cmd = parts[0].upper()

    if cmd == "INIT_BRAKE":
        backend.initialize_brake()
        return "OK"

    if cmd == "BRAKE":
        if len(parts) != 2 or parts[1] not in ("0", "1"):
            return "ERR usage: BRAKE <0|1>"
        backend.set_brake(parts[1] == "1")
        return "OK"

    if cmd == "INIT_LIMITS":
        # LocalGPIOBackend tracks state internally; the host polls it via GET, so a
        # no-op callback is enough here (the callback path is exercised host-side).
        backend.initialize_limit_switches(lambda upper, lower: None)
        return "OK"

    if cmd == "GET":
        upper, lower = backend.get_limit_states()
        return f"L {int(upper)} {int(lower)}"

    if cmd == "CLEANUP":
        backend.cleanup()
        return "OK"

    return f"ERR unknown command {cmd!r}"


def serve(port: str, baudrate: int = 115200) -> None:
    import serial  # local import: pyserial is only needed to run the server

    backend = LocalGPIOBackend()
    # readline() blocks up to timeout; a modest timeout keeps the loop responsive to
    # shutdown without busy-spinning.
    conn = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    logger.info(f"GPIO serial satellite listening on {port} (baud={baudrate})")
    try:
        while True:
            raw = conn.readline()
            if not raw:
                continue  # read timeout, no command pending
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            try:
                reply = _handle_command(backend, line)
            except Exception as e:  # never let one bad command kill the server
                logger.exception(f"Command {line!r} failed")
                reply = f"ERR {e}"
            conn.write((reply + "\n").encode("ascii"))
            conn.flush()
    except KeyboardInterrupt:
        logger.info("Interrupted, shutting down")
    finally:
        try:
            backend.cleanup()
        except Exception as e:
            logger.warning(f"GPIO cleanup error: {e}")
        if conn.is_open:
            conn.close()
        logger.info("GPIO serial satellite stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyGS0", help="Serial gadget device (default: /dev/ttyGS0).")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate (default: 115200).")
    args = parser.parse_args()
    serve(args.port, args.baudrate)


if __name__ == "__main__":
    main()
