import argparse
import sys
import threading
import time
from typing import Any

import numpy as np
import portal

from i2rt.flow_base.flow_base_controller import BASE_DEFAULT_PORT


class FlowBaseClient:
    def __init__(self, host: str = "localhost", with_linear_rail: bool = False):
        self.with_linear_rail = with_linear_rail
        self.client = portal.Client(f"{host}:{BASE_DEFAULT_PORT}")
        self.num_dofs = 3 if not self.with_linear_rail else 4
        self.command = {"target_velocity": np.zeros(self.num_dofs), "frame": "local"}
        self._lock = threading.Lock()
        self.running = True
        self._thread = threading.Thread(target=self._update_command)
        self._thread.start()

    def _update_command(self) -> None:
        while self.running:
            with self._lock:
                self.client.set_target_velocity(self.command).result()
            time.sleep(0.02)

    def get_odometry(self) -> Any:
        return self.client.get_odometry({}).result()

    def reset_odometry(self) -> Any:
        return self.client.reset_odometry({}).result()

    def set_target_velocity(self, target_velocity: np.ndarray, frame: str = "local") -> None:
        """Set target velocity for base and optionally linear rail.

        Args:
            target_velocity: [x, y, theta] or [x, y, theta, linear_rail_vel]
            frame: "local" or "global"
        """
        assert target_velocity.shape == (self.num_dofs,), f"Target velocity must have shape ({self.num_dofs},)"
        assert frame in ["local", "global"], "Frame must be either local or global"

        with self._lock:
            self.command["target_velocity"] = target_velocity
            self.command["frame"] = frame

    def get_linear_rail_state(self) -> Any:
        """Get the current state of the linear rail."""
        if not self.with_linear_rail:
            raise ValueError("Linear rail not enabled. Initialize FlowBaseClient with with_linear_rail=True")
        return self.client.get_linear_rail_state({}).result()

    def set_linear_rail_velocity(self, velocity: float) -> None:
        """Set the velocity of the linear rail.

        Args:
            velocity (float): Target velocity in rad/s
        """
        if not self.with_linear_rail:
            raise ValueError("Linear rail not enabled. Initialize FlowBaseClient with with_linear_rail=True")
        with self._lock:
            if len(self.command["target_velocity"]) < 4:
                self.command["target_velocity"] = np.append(self.command["target_velocity"], 0.0)
            self.command["target_velocity"][3] = velocity

    def close(self) -> None:
        """Stop the client and clean up resources."""
        self.running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def _format_linear_rail_state(rail_state: Any) -> str:
    """Format linear rail state defensively for CLI output."""
    if not isinstance(rail_state, dict):
        return f"unexpected response: {rail_state!r}"

    position = rail_state.get("position")
    velocity = rail_state.get("velocity")
    upper_limit = rail_state.get("upper_limit_triggered")
    lower_limit = rail_state.get("lower_limit_triggered")

    if position is None or velocity is None:
        return f"linear rail response: {rail_state}"

    return (
        f"position: {position:.4f} velocity: {velocity:.4f} "
        f"upper_limit: {upper_limit} lower_limit: {lower_limit}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--command", type=str, default="get_odometry")
    parser.add_argument("--with-linear-rail", action="store_true", help="Enable linear rail support")
    args = parser.parse_args()

    # Auto-enable linear rail for linear rail commands
    linear_rail_commands = ["test_linear_rail", "get_linear_rail_state"]
    use_linear_rail = args.with_linear_rail or args.command in linear_rail_commands

    client = FlowBaseClient(args.host, with_linear_rail=use_linear_rail)

    try:
        if args.command == "get_odometry":
            print(client.get_odometry())
            sys.exit(0)
        elif args.command == "reset_odometry":
            client.reset_odometry()
            sys.exit(0)
        elif args.command == "test_command":
            client.set_target_velocity(np.array([0.0, 0.0, 0.1]), "local")
            while True:
                odo_reading = client.get_odometry()
                sys.stdout.write(f"\r translation: {odo_reading['translation']} rotation: {odo_reading['rotation']}")
                sys.stdout.flush()
                time.sleep(0.02)
        elif args.command == "test_linear_rail":
            try:
                client.set_linear_rail_velocity(0.5)
                while True:
                    rail_state = client.get_linear_rail_state()
                    sys.stdout.write(f"\r {_format_linear_rail_state(rail_state)}")
                    sys.stdout.flush()
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nStopping...")
                client.set_linear_rail_velocity(0.0)
                time.sleep(0.5)
        elif args.command == "get_linear_rail_state":
            print("Monitoring linear rail state (Press Ctrl+C to exit)")
            try:
                while True:
                    rail_state = client.get_linear_rail_state()
                    sys.stdout.write(f"\r {_format_linear_rail_state(rail_state)}")
                    sys.stdout.flush()
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\nExiting")
        else:
            sys.exit(1)
    finally:
        client.close()
