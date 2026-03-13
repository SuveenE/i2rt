import argparse
import portal
from i2rt.flow_base.flow_base_controller import BASE_DEFAULT_PORT
import numpy as np
import time
import threading
import sys

class FlowBaseClient:
    def __init__(self, host: str = "localhost"):
        self.client = portal.Client(f"{host}:{BASE_DEFAULT_PORT}")
        self.command = {'target_velocity': np.array([0.0, 0.0, 0.0]), 'frame': 'local'}
        self._lock = threading.Lock()
        self.running = True
        self._thread = threading.Thread(target=self._update_velocity)
        self._thread.start()

    def _update_velocity(self):
        while self.running:
            with self._lock:
                self.client.set_target_velocity(self.command).result()
            time.sleep(0.02)

    def get_odometry(self):
        return self.client.get_odometry({}).result()


    def reset_odometry(self):
        return self.client.reset_odometry({}).result()

    def set_target_velocity(self, target_velocity: np.ndarray, frame: str = "local"):
        with self._lock:
            self.command["target_velocity"] = target_velocity
            self.command["frame"] = frame

    def get_linear_rail_state(self) -> Any:
        """Get the current state of the linear rail."""
        if not self.with_linear_rail:
            raise ValueError("Linear rail not enabled. Initialize FlowBaseClient with with_linear_rail=True")
        return self.client.get_linear_rail_state({}).result()

    def get_current_command(self) -> dict[str, Any]:
        """Return the resolved command currently being executed by the controller.

        Returns a dict with keys:
            velocity: list of 4 floats [x, y, theta, rail] in physical units
            frame: "local" or "global"
            source: "gamepad", "remote", or "none"
        """
        return self.client.get_current_command({}).result()

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
        self._thread.join()
        self.client.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--command", type=str, default="get_odometry")
    args = parser.parse_args()

    client = FlowBaseClient(args.host)

    if args.command == "get_odometry":
        print(client.get_odometry())
        client.close()
        exit()
    elif args.command == "reset_odometry":
        client.reset_odometry()
        client.close()
        exit()
    elif args.command == "test_command":
        client.set_target_velocity(np.array([0.0, 0.0, 0.1]), "local")
        while True:
            odo_reading = client.get_odometry()
            sys.stdout.write(
                f"\r translation: {odo_reading['translation']} rotation: {odo_reading['rotation']}"
            )
            sys.stdout.flush()
            time.sleep(0.02)
