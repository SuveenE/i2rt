#!/usr/bin/env python3
import ctypes
import glob
import os
import sys

import numpy as np
import pygame


def _pygame_sdl2_paths():
    """Yield candidate paths to the *exact* SDL2 library that pygame loaded.

    A pip-installed pygame bundles its own SDL2 (under ``pygame.libs`` on
    Linux or ``pygame/.dylibs`` on macOS). That bundled copy is a different
    library instance than the system ``libSDL2`` reachable by name, and they
    do NOT share state. We must poll the instance pygame actually opened the
    joystick on, otherwise ``joy.get_axis()`` reads never update.
    """
    # 1. The library already mapped into this process (most reliable on Linux).
    if sys.platform.startswith("linux"):
        try:
            with open("/proc/self/maps", encoding="utf-8") as f:
                for line in f:
                    path = line.rstrip().split(" ", maxsplit=5)[-1]
                    if "libSDL2" in path and os.path.isfile(path):
                        yield path
        except OSError:
            pass

    # 2. Libraries bundled inside the pygame package directory.
    pkg_dir = os.path.dirname(pygame.__file__)
    for pattern in (
        os.path.join(pkg_dir, "..", "pygame.libs", "libSDL2*"),
        os.path.join(pkg_dir, ".dylibs", "libSDL2*"),
        os.path.join(pkg_dir, "libSDL2*"),
        os.path.join(pkg_dir, "SDL2*.dll"),
    ):
        yield from glob.glob(pattern)


def _load_sdl2():
    """Load the SDL2 shared library pygame uses, for direct joystick polling."""
    candidates = list(_pygame_sdl2_paths())
    # Fall back to system-wide names only if pygame's own copy can't be found.
    candidates += ["libSDL2-2.0.so.0", "libSDL2-2.0.dylib", "SDL2.dll", "libSDL2.so"]
    for name in candidates:
        try:
            lib = ctypes.CDLL(name)
            lib.SDL_JoystickUpdate.argtypes = []
            lib.SDL_JoystickUpdate.restype = None
            return lib
        except (OSError, AttributeError):
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

        # Keep SDL joystick events ENABLED so that pygame.event.pump() auto-
        # refreshes axis state (this is what the working test_gamepad.py does).
        # We additionally call SDL_JoystickUpdate() directly in _poll() on the
        # SAME SDL2 instance pygame loaded, as event-system-independent
        # insurance when background threads (motor control, RPC) are running.
        # NOTE: do NOT disable joystick events here — doing so stops pump() from
        # updating axes and was the cause of user_cmd reading all zeros.
        self._sdl2 = _load_sdl2()

        print(f"Joystick Name: {self.joy.get_name()}")
        print(f"Number of Axes: {self.joy.get_numaxes()}")
        print(f"Number of Buttons: {self.joy.get_numbuttons()}")

    def _poll(self) -> None:
        """Update joystick state by polling hardware directly.

        ``pygame.event.pump()`` keeps pygame's own SDL instance refreshed (and
        works as long as background events are allowed). The direct
        ``SDL_JoystickUpdate()`` call is an additional, event-system-independent
        refresh of the same instance pygame uses.
        """
        pygame.event.pump()
        if self._sdl2:
            self._sdl2.SDL_JoystickUpdate()

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
