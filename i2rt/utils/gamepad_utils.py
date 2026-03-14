#!/usr/bin/env python3
import ctypes
import os

import numpy as np
import pygame


def _load_sdl2():
    """Load SDL2 shared library for direct joystick polling."""
    for name in ("libSDL2-2.0.so.0", "libSDL2-2.0.dylib", "SDL2.dll", "libSDL2.so"):
        try:
            lib = ctypes.CDLL(name)
            lib.SDL_JoystickUpdate.argtypes = []
            lib.SDL_JoystickUpdate.restype = None
            return lib
        except OSError:
            continue
    return None


class Gamepad:
    def __init__(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick/gamepad connected!")
            exit()
        else:
            print(f"Detected {pygame.joystick.get_count()} joystick(s).")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        # SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS via env var is unreliable
        # when other threads are running (motor control, RPC server, etc.).
        # Bypass the event system entirely: disable SDL joystick events and
        # poll the hardware directly with SDL_JoystickUpdate().
        self._sdl2 = _load_sdl2()
        if self._sdl2:
            SDL_DISABLE = 0
            self._sdl2.SDL_JoystickEventState(SDL_DISABLE)

        print(f"Joystick Name: {self.joy.get_name()}")
        print(f"Number of Axes: {self.joy.get_numaxes()}")
        print(f"Number of Buttons: {self.joy.get_numbuttons()}")

    def _poll(self) -> None:
        """Update joystick state by polling hardware directly."""
        if self._sdl2:
            self._sdl2.SDL_JoystickUpdate()
        else:
            pygame.event.pump()

    def get_button_reading(self) -> dict[str, int]:
        self._poll()
        key_mode = self.joy.get_button(12)
        key_left_2 = self.joy.get_button(8)
        key_left_1 = self.joy.get_button(6)
        return dict(
            key_mode=key_mode,
            key_left_2=key_left_2,
            key_left_1=key_left_1,
        )

    def get_user_cmd(self) -> np.ndarray:
        self._poll()
        x = self.joy.get_axis(1)  # Left stick Y-axis
        y = self.joy.get_axis(0)  # Left stick X-axis
        th = self.joy.get_axis(2)  # Right stick X-axis

        user_cmd = np.array([-x, y, th])
        user_cmd[np.abs(user_cmd) < 0.05] = 0
        return user_cmd

    def close(self) -> None:
        pygame.quit()


if __name__ == "__main__":
    pad = Gamepad()
    try:
        while True:
            cmd = pad.get_user_cmd()
            print(f"user_cmd = {cmd}")
    except KeyboardInterrupt:
        pad.close()
