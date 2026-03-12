#!/usr/bin/env python3
"""Keyboard control for the Flow Base.

Controls:
    W/S     - Forward/Backward
    A/D     - Strafe Left/Right
    Q/E     - Rotate Left/Right
    U/J     - Linear rail Up/Down (if enabled)
    F       - Toggle local/global frame
    R       - Reset odometry
    O       - Print odometry
    ESC/X   - Quit
"""

import argparse
import sys
import termios
import time
import tty
from select import select

import numpy as np

from i2rt.flow_base.flow_base_client import FlowBaseClient

SPEED_STEP = 0.05
ROTATION_STEP = 0.1
RAIL_SPEED_STEP = 0.5

KEY_BINDINGS = {
    "w": ("vx", +1),
    "s": ("vx", -1),
    "a": ("vy", +1),
    "d": ("vy", -1),
    "q": ("vtheta", +1),
    "e": ("vtheta", -1),
    "u": ("rail", +1),
    "j": ("rail", -1),
}

MAX_VX = 0.5
MAX_VY = 0.5
MAX_VTHETA = 1.57
MAX_RAIL = 14.0


def get_key(timeout: float = 0.02) -> str:
    """Non-blocking key read from stdin."""
    if select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return ""


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def main() -> None:
    parser = argparse.ArgumentParser(description="Keyboard control for Flow Base")
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--with-linear-rail", action="store_true", help="Enable linear rail support")
    args = parser.parse_args()

    client = FlowBaseClient(host=args.host, with_linear_rail=args.with_linear_rail)

    vx = 0.0
    vy = 0.0
    vtheta = 0.0
    rail_vel = 0.0
    frame = "local"

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print(__doc__)
        print(f"Connected to {args.host}")
        print(f"Frame: {frame} | Linear rail: {'ON' if args.with_linear_rail else 'OFF'}")
        print(f"Speed step: {SPEED_STEP} m/s | Rotation step: {ROTATION_STEP} rad/s")
        print("-" * 60)

        while True:
            key = get_key()

            if key in ("\x1b", "x"):
                print("\nExiting...")
                break

            if key in KEY_BINDINGS:
                axis, direction = KEY_BINDINGS[key]
                if axis == "vx":
                    vx = clamp(vx + direction * SPEED_STEP, -MAX_VX, MAX_VX)
                elif axis == "vy":
                    vy = clamp(vy + direction * SPEED_STEP, -MAX_VY, MAX_VY)
                elif axis == "vtheta":
                    vtheta = clamp(vtheta + direction * ROTATION_STEP, -MAX_VTHETA, MAX_VTHETA)
                elif axis == "rail" and args.with_linear_rail:
                    rail_vel = clamp(rail_vel + direction * RAIL_SPEED_STEP, -MAX_RAIL, MAX_RAIL)
            elif key == " ":
                vx = vy = vtheta = rail_vel = 0.0
            elif key == "f":
                frame = "global" if frame == "local" else "local"
            elif key == "r":
                client.reset_odometry()
                print("\n[Odometry reset]")
            elif key == "o":
                odo = client.get_odometry()
                print(f"\n[Odometry] x={odo['translation'][0]:.4f} y={odo['translation'][1]:.4f} θ={odo['rotation']:.4f}")

            if args.with_linear_rail:
                vel = np.array([vx, vy, vtheta, rail_vel])
            else:
                vel = np.array([vx, vy, vtheta])

            client.set_target_velocity(vel, frame=frame)

            if args.with_linear_rail:
                status = (
                    f"\r[{frame:6s}] vx={vx:+.2f} vy={vy:+.2f} vθ={vtheta:+.2f} rail={rail_vel:+.1f}"
                    f"  (SPACE=stop)"
                )
            else:
                status = f"\r[{frame:6s}] vx={vx:+.2f} vy={vy:+.2f} vθ={vtheta:+.2f}  (SPACE=stop)"

            sys.stdout.write(status)
            sys.stdout.flush()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        client.set_target_velocity(np.zeros(client.num_dofs), frame="local")
        time.sleep(0.1)
        client.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Stopped.")


if __name__ == "__main__":
    main()
