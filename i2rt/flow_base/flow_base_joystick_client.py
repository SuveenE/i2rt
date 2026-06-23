#!/usr/bin/env python3
"""Network joystick bridge for the FlowBase / LinearBot.

Reads a gamepad connected to *this* machine (ideally over a reliable USB cable)
and streams velocity commands over the network to a ``flow_base_controller``
running with ``--no-gamepad``. This lets the joystick live at the operator
station on any PC on the robot network, instead of relying on a flaky Bluetooth
pairing to the PC riding on the base.

It reuses the controller's existing remote-command RPC (``set_target_velocity``
on ``BASE_DEFAULT_PORT``) and the same ``Gamepad`` helper the controller uses,
so the behavior matches the local-joystick path:

  * Base axes ``[x, y, theta]`` are sent NORMALISED ``[-1, 1]``; the controller
    scales them by its internal ``max_vel``.
  * The linear rail is sent in physical m/s (the controller converts it to
    motor rad/s server-side via ``meters_per_rad``).
  * Left-stick = translation, right-stick X = rotation, right-stick Y = rail.
  * ``key_mode`` toggles local/global frame; ``key_left_1`` resets odometry.

The controller's remote-command path has a 0.2 s timeout: if this bridge stops
sending (Ctrl+C, crash, or network drop), the base stops on its own.

Example
-------
On the controller PC (the base):
    python3 i2rt/i2rt/flow_base/flow_base_controller.py \
        --channel can_linearbot --gpio-host 172.16.0.67:8765 --no-gamepad

On the operator PC with the joystick plugged in via USB:
    python3 i2rt/i2rt/flow_base/flow_base_joystick_client.py --host 172.16.0.67
"""

import argparse
import sys
import time

import numpy as np
import portal

from i2rt.flow_base.flow_base_controller import BASE_DEFAULT_PORT
from i2rt.utils.gamepad_utils import Gamepad

# Mirror the constants used by flow_base_controller.py's local joystick loop.
RAIL_DEADZONE = 0.15  # Larger deadzone for the rail to prevent unwanted movement.
DEFAULT_LIFT_MAX_VEL_MS = 0.5  # Right-stick Y full deflection -> m/s for the rail.
DEFAULT_SEND_HZ = 50.0  # Command rate. Stay well under the controller's 0.2 s timeout.


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="Host (IP or hostname) of the flow_base_controller RPC server.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=BASE_DEFAULT_PORT,
        help=f"Port of the flow_base_controller RPC server (default {BASE_DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--no-linear-rail",
        action="store_true",
        help="Send 3-DOF base-only commands (omit the linear rail axis).",
    )
    parser.add_argument(
        "--lift-max-vel-ms",
        type=float,
        default=DEFAULT_LIFT_MAX_VEL_MS,
        help="Right-stick Y full deflection -> linear rail speed in m/s.",
    )
    parser.add_argument(
        "--send-hz",
        type=float,
        default=DEFAULT_SEND_HZ,
        help="Command send rate in Hz (must stay above ~5 Hz to beat the controller timeout).",
    )
    args = parser.parse_args()

    # Connect to the controller's RPC server first, so a bad host fails fast
    # before we bother initializing the joystick.
    client = portal.Client(f"{args.host}:{args.port}")
    print(f"Connected to flow_base_controller at {args.host}:{args.port}")

    # Gamepad() waits (forever, by default) until a joystick is detected and
    # prints its name / axis / button counts.
    gamepad = Gamepad()
    joy = gamepad.joy

    num_dofs = 3 if args.no_linear_rail else 4
    period = 1.0 / args.send_hz

    frame = "local"
    last_mode_toggled = False
    count = 0

    print("Streaming joystick -> base. Press Ctrl+C to stop (base stops on disconnect).")
    try:
        while True:
            base_cmd = gamepad.get_user_cmd()  # normalised [x, y, theta], deadzone applied
            buttons = gamepad.get_button_reading()

            # Toggle local/global frame on the rising edge of key_mode.
            if buttons["key_mode"] and not last_mode_toggled:
                last_mode_toggled = True
                frame = "global" if frame == "local" else "local"
                print(f"\nFrame -> {frame}")
            elif not buttons["key_mode"]:
                last_mode_toggled = False

            # Reset odometry while key_left_1 is held.
            if buttons["key_left_1"]:
                client.reset_odometry({}).result()
                print("\nOdometry reset")

            if args.no_linear_rail:
                target = base_cmd  # 3D: [x, y, theta]
            else:
                rail_mps = 0.0
                if joy.get_numaxes() > 3:
                    right_stick_y = joy.get_axis(3)
                    # Up (negative axis value) -> positive (upward) rail velocity.
                    if np.abs(right_stick_y) > RAIL_DEADZONE:
                        rail_mps = -right_stick_y * args.lift_max_vel_ms
                target = np.append(base_cmd, rail_mps)  # 4D: rail in physical m/s

            client.set_target_velocity({"target_velocity": target, "frame": frame}).result()

            if count % 10 == 0:
                rail_str = "" if args.no_linear_rail else f" rail:{target[3]:+.2f}m/s"
                sys.stdout.write(
                    f"\r[{frame}] x:{target[0]:+.2f} y:{target[1]:+.2f} th:{target[2]:+.2f}{rail_str}    "
                )
                sys.stdout.flush()
            count += 1

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping, sending zero velocity...")
    finally:
        # Best-effort explicit stop; the controller's timeout would stop the
        # base anyway, but sending zeros makes the halt immediate and clean.
        try:
            client.set_target_velocity({"target_velocity": np.zeros(num_dofs), "frame": "local"}).result()
        except Exception:
            pass
        try:
            gamepad.close()
        except Exception:
            pass
        try:
            client.close(timeout=2.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
