#!/usr/bin/env python3
import ctypes
import glob
import math
import os
import sys
import time

import numpy as np
import pygame


def apply_axis_dominance(x: float, y: float, cone_ratio: float) -> tuple[float, float]:
    """Snap a 2D stick reading onto a cardinal axis when one axis dominates.

    Two perpendicular axes of the same stick are prone to cross-talk: when the
    user intends a pure left/right (or up/down) push but tilts the stick a few
    degrees off-axis, the minor component can still clear a per-axis deadzone and
    trigger an unwanted perpendicular motion.

    If the minor component is smaller than ``cone_ratio`` times the major one
    (i.e. the input falls within a cone around the dominant cardinal axis), the
    minor component is zeroed. Deliberate diagonals outside that cone are left
    untouched. ``cone_ratio`` is ``tan(half_angle)``; e.g. ~0.466 for a 25 deg
    cone. A non-positive ratio disables the filter.
    """
    if cone_ratio <= 0.0:
        return x, y
    ax, ay = abs(x), abs(y)
    if ax >= ay:
        if ay < cone_ratio * ax:
            y = 0.0
    elif ax < cone_ratio * ay:
        x = 0.0
    return x, y


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
    def __init__(
        self,
        connect_timeout: float | None = None,
        retry_delay: float = 0.5,
        cross_axis_cone_deg: float = 25.0,
    ):
        """Initialize the gamepad, polling at startup until one is connected.

        Args:
            connect_timeout: How long to wait for a joystick to appear at
                startup before giving up. ``None`` (the default) means wait
                forever, so the controller comes up the moment the gamepad is
                plugged in instead of exiting and requiring a manual restart.
            retry_delay: Delay between connection scans during the initial wait.
            cross_axis_cone_deg: Half-angle (degrees) of the cone around each
                cardinal direction within which the perpendicular axis of the
                left stick is suppressed. This stops a slightly-angled "right"
                push from also triggering a "forward" motion. Set to 0 to
                disable.
        """
        self._cross_axis_cone_ratio = math.tan(math.radians(cross_axis_cone_deg))
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
        # By default SDL installs SIGINT/SIGTERM handlers that swallow Ctrl+C
        # (it posts an internal SDL_QUIT event instead of letting Python raise
        # KeyboardInterrupt). Running headless we never consume that event, so
        # Ctrl+C appears to hang. Disable SDL's signal handlers so the normal
        # KeyboardInterrupt path works. Must be set before pygame.init().
        os.environ["SDL_NO_SIGNAL_HANDLERS"] = "1"
        pygame.init()
        pygame.joystick.init()

        # SDL often fails to enumerate the joystick on the very first scan,
        # especially headless / right after CAN/GPIO init races the HID probe.
        # Instead of giving up after a fixed timeout (which forced a manual
        # restart), re-scan the joystick subsystem in a loop until one appears.
        # By default we wait forever so the controller connects on first launch.
        deadline = None if connect_timeout is None else time.time() + connect_timeout
        last_msg = 0.0
        while pygame.joystick.get_count() == 0:
            now = time.time()
            if deadline is not None and now >= deadline:
                print(f"No joystick/gamepad connected after {connect_timeout:.0f}s!")
                raise RuntimeError("No joystick/gamepad detected")
            if now - last_msg >= 5.0:
                print("Waiting for joystick/gamepad to connect...")
                last_msg = now
            pygame.event.pump()  # let SDL process pending device-added events
            pygame.joystick.quit()
            time.sleep(retry_delay)
            pygame.joystick.init()
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

        # Suppress off-axis cross-talk on the left stick so a near-cardinal
        # push (e.g. "right") doesn't bleed into the perpendicular axis
        # (e.g. "forward"). Done before the per-axis deadzone below.
        x, y = apply_axis_dominance(x, y, self._cross_axis_cone_ratio)

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
