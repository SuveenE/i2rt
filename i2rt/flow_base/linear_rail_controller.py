import logging
import sys
import threading
import time
from typing import Any, Dict, Literal, Optional

import numpy as np

from i2rt.flow_base.gpio_backend import (
    BRAKE_CONTROL_GPIO,  # noqa: F401 — re-exported for backward compatibility
    LOWER_LIMIT_GPIO,  # noqa: F401
    UPPER_LIMIT_GPIO,  # noqa: F401
    GPIOBackend,
    LocalGPIOBackend,
    NoopGPIOBackend,
)
from i2rt.motor_drivers.dm_driver import DMChainCanInterface
from i2rt.motor_drivers.utils import MotorInfo

try:
    from RPi import GPIO  # noqa: F401 — re-exported for backward compatibility
except (ImportError, RuntimeError):
    GPIO = None  # type: ignore[assignment]

# configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("LinearRailController")

HOMING_SPEED_RATIO = 0.5
HOMING_TIMEOUT = 30.0  # Timeout for homing procedure in seconds
COMMAND_TIMEOUT = 0.25  # Timeout for command stream (2.5 * POLICY_CONTROL_PERIOD, where POLICY_CONTROL_PERIOD = 0.1s)


def _default_gpio_backend() -> GPIOBackend:
    """Create a default GPIO backend: local RPi.GPIO if available, else no-op."""
    try:
        return LocalGPIOBackend()
    except (ImportError, RuntimeError):
        return NoopGPIOBackend()


def initialize_brake_gpio() -> None:
    """Initialize brake control GPIO pin as an independent function.

    Legacy helper kept for backward compatibility. Prefer passing a
    ``GPIOBackend`` to ``LinearRailController`` instead.
    """
    _default_gpio_backend().initialize_brake()


def set_brake_gpio(engaged: bool) -> None:
    """Set brake GPIO state (engaged=True: brake on, engaged=False: brake off).

    Legacy helper kept for backward compatibility.
    """
    _default_gpio_backend().set_brake(engaged)


class SingleMotorControlInterface:
    """Single motor control interface for motor chains"""

    def __init__(self, motor_chain: DMChainCanInterface, target_motor_idx: int = -1):
        """Initialize single motor control interface"""
        if len(motor_chain) == 0:
            raise ValueError(f"Motor chain must contain at least 1 motor, got {len(motor_chain)}")

        self.motor_chain = motor_chain
        self.target_motor_idx = target_motor_idx

        if target_motor_idx < 0 or target_motor_idx >= len(motor_chain):
            raise ValueError(f"Motor index {target_motor_idx} out of range [0, {len(motor_chain)})")

        self.motor_id = motor_chain.motor_list[target_motor_idx][0]

    def set_velocity(self, vel: float) -> None:
        """Set motor velocity"""
        num_motors = len(self.motor_chain)

        velocities = np.zeros(num_motors)
        velocities[self.target_motor_idx] = vel

        # Preserve velocities of other motors (e.g., base motors) by reading current commands
        with self.motor_chain.command_lock:
            current_commands = self.motor_chain.commands
            if current_commands and len(current_commands) == num_motors:
                # Preserve velocities of other motors
                for idx in range(num_motors):
                    if idx != self.target_motor_idx:
                        velocities[idx] = current_commands[idx].vel

        torques = np.zeros(num_motors)
        self.motor_chain.set_commands(torques=torques, vel=velocities, pos=None, kp=None, kd=None, get_state=False)

    def get_state(self) -> MotorInfo:
        """Get motor state"""
        return self.motor_chain.read_states()[self.target_motor_idx]

    def set_zero_position(self) -> None:
        """Set the current motor position as the zero reference"""
        self.motor_chain.set_zero_position(self.target_motor_idx)

    @classmethod
    def from_multi_motor_chain(
        cls, motor_chain: DMChainCanInterface, target_motor_idx: int
    ) -> "SingleMotorControlInterface":
        """Create SingleMotorControlInterface from an existing multi-motor chain using motor index"""
        return cls(motor_chain, target_motor_idx=target_motor_idx)


class LinearRailController:
    def __init__(
        self,
        single_motor_control_interface: SingleMotorControlInterface,
        gpio_backend: Optional[GPIOBackend] = None,
        rail_speed: float = 14.0,
        auto_home: bool = True,
        homing_timeout: float = HOMING_TIMEOUT,
        total_stroke_m: float = 1.0,
        max_vel_mps: Optional[float] = 0.5,
        meters_per_rad_override: Optional[float] = None,
    ):
        """Initialize linear rail controller

        Args:
            single_motor_control_interface: Motor control interface (required)
            gpio_backend: GPIO backend for brake and limit switch operations.
                If *None*, auto-detects local RPi.GPIO or falls back to no-op.
            rail_speed: Maximum rail speed in rad/s
            auto_home: Whether to automatically home after initialization
            homing_timeout: Timeout for homing procedure in seconds
            total_stroke_m: Physical stroke between upper and lower limit switches, in meters.
                Used during the top-then-bottom startup calibration to convert motor radians
                into linear meters: meters_per_rad = total_stroke_m / (theta_upper - theta_lower).
            max_vel_mps: Hard cap on the rail's linear speed, in m/s. Enforced in
                ``set_velocity`` once ``meters_per_rad`` is known (after calibration) so the
                rail's top speed is independent of motor gearing and identical on both the
                joystick and remote command paths. ``None`` disables the m/s cap (only the
                ``rail_speed`` rad/s limit applies). Homing moves bypass ``set_velocity`` so
                they are never throttled by this.
            meters_per_rad_override: If provided, skip startup limit-switch calibration
                and use this (fixed, signed) value as ``meters_per_rad`` instead of measuring
                it. ``meters_per_rad`` is a mechanical constant (lead-screw
                pitch / gearing), so once measured it can be reused. No startup rail motion
                is performed; the current position is set as the software zero instead.
                Useful when a payload blocks travel to either limit switch. The value must
                carry the same sign as the originally calibrated one (copy it from a previous
                "Linear rail calibrated: ... meters_per_rad=..." log line).
        """
        self.single_motor_control_interface = single_motor_control_interface
        self.gpio_backend = gpio_backend if gpio_backend is not None else _default_gpio_backend()
        self.rail_speed = rail_speed
        self.auto_home = auto_home
        self.homing_timeout = homing_timeout
        self.total_stroke_m = total_stroke_m
        self.max_vel_mps = max_vel_mps
        self.meters_per_rad_override = meters_per_rad_override
        self.meters_per_rad: Optional[float] = None

        self.initialized = False
        self.brake_on = True

        self._lock = threading.Lock()
        self.upper_limit_triggered = False
        self.lower_limit_triggered = False
        self._gpio_initialized = False
        self._homing_event = threading.Event()
        self._homing_start_time = None
        self.homing_speed_ratio = HOMING_SPEED_RATIO

        # Command timeout tracking (similar to base control)
        self.last_command_time = time.time() - 1000000  # Initialize to far past
        self.command_timeout = COMMAND_TIMEOUT

        if self.auto_home:
            self._initialize_linear_rail()
        else:
            with self._lock:
                self.initialized = True
            logger.info("Linear rail initialized without auto-homing (GPIO will be initialized separately)")

    def initialize_gpio(self) -> None:
        """Initialize GPIO via the backend. Call after construction to avoid
        blocking during ``__init__``.
        """
        if self._gpio_initialized:
            logger.debug("GPIO already initialized, skipping")
            return

        self.gpio_backend.initialize_limit_switches(self._on_limit_change)
        self._gpio_initialized = True
        logger.info("GPIO initialized via backend")

    def _on_limit_change(self, upper_triggered: bool, lower_triggered: bool) -> None:
        """Callback invoked by the GPIO backend when limit switch state changes."""
        prev_upper = prev_lower = False
        with self._lock:
            prev_upper = self.upper_limit_triggered
            prev_lower = self.lower_limit_triggered
            self.upper_limit_triggered = upper_triggered
            self.lower_limit_triggered = lower_triggered

        if upper_triggered and not prev_upper:
            logger.warning("Upper limit switch triggered!")
            self.single_motor_control_interface.set_velocity(0.0)
        elif not upper_triggered and prev_upper:
            logger.info("Upper limit switch released")

        if lower_triggered and not prev_lower:
            logger.warning("Lower limit switch triggered!")
            self.single_motor_control_interface.set_velocity(0.0)
        elif not lower_triggered and prev_lower:
            logger.info("Lower limit switch released")

    def set_brake(self, engaged: bool) -> None:
        """Set brake state (engaged=True: brake on, engaged=False: brake off)"""
        try:
            self.gpio_backend.set_brake(engaged)
            with self._lock:
                self.brake_on = engaged
        except Exception as e:
            action = "engage" if engaged else "release"
            logger.error(f"Failed to {action} brake: {e}")

    def _initialize_linear_rail(self) -> None:
        """Initialize the rail using a fixed scale or a full limit-switch calibration.

        With ``meters_per_rad_override``, performs no startup motion and makes the
        current position zero. Otherwise, captures the motor angle at each limit,
        computes meters_per_rad from ``self.total_stroke_m``, and zeroes the encoder
        at the lower limit.
        """
        try:
            # Release brake before any motion
            self.set_brake(engaged=False)

            # Override path: reuse the fixed mechanical conversion without moving
            # toward either limit. The startup position becomes this session's zero.
            if self.meters_per_rad_override is not None:
                with self._lock:
                    self.meters_per_rad = self.meters_per_rad_override
                self._set_home_zero(at_lower_limit=False)
                with self._lock:
                    self.initialized = True
                logger.info(
                    f"Linear rail using provided meters_per_rad={self.meters_per_rad:.6f} m/rad "
                    f"(startup motion skipped; current position set to zero)"
                )
                return

            # Phase 1: drive to upper limit (skip if already triggered there)
            with self._lock:
                already_at_upper = self.upper_limit_triggered
            if not already_at_upper:
                self._move_until_limit(direction="up")
            time.sleep(0.2)  # settle so the encoder reading is stable
            theta_upper = self.single_motor_control_interface.get_state().pos

            # Phase 2: drive to lower limit (skip if already triggered there)
            with self._lock:
                already_at_lower = self.lower_limit_triggered
            if not already_at_lower:
                self._move_until_limit(direction="down")
            time.sleep(0.2)
            theta_lower = self.single_motor_control_interface.get_state().pos

            # Phase 3: compute meters-per-radian from the captured motor angles.
            # Carrying the sign of delta through gives a signed conversion factor so
            # linear_pos = motor_pos * meters_per_rad always grows from 0 (bottom) to
            # total_stroke_m (top) regardless of motor direction.
            delta = theta_upper - theta_lower
            if abs(delta) < 1e-3:
                raise RuntimeError(
                    f"Linear-rail calibration failed: |theta_upper - theta_lower| = "
                    f"{abs(delta):.6f} rad is too small to calibrate "
                    f"(theta_upper={theta_upper:.3f}, theta_lower={theta_lower:.3f})"
                )
            with self._lock:
                self.meters_per_rad = self.total_stroke_m / delta
            logger.info(
                f"Linear rail calibrated: theta_upper={theta_upper:.3f} rad, "
                f"theta_lower={theta_lower:.3f} rad, delta={delta:.3f} rad, "
                f"meters_per_rad={self.meters_per_rad:.6f} m/rad "
                f"(stroke={self.total_stroke_m:.3f} m)"
            )

            # Phase 4: zero encoder at lower limit so encoder 0 == bottom of travel
            self._set_home_zero()
            with self._lock:
                self.initialized = True

        except Exception as e:
            logger.error(f"Linear rail initialization failed: {e}")
            self.initialized = False
            try:
                self.single_motor_control_interface.set_velocity(0.0)
            except Exception as stop_error:
                logger.error(f"Failed to stop linear-rail motor during cleanup: {stop_error}")
            with self._lock:
                self._homing_event.clear()
                self._homing_start_time = None
            raise

    def _move_until_limit(self, direction: Literal["up", "down"]) -> None:
        """Drive the rail until the corresponding limit switch triggers, or time out.

        Continuously re-applies the homing velocity (every 50 ms) so the base controller's
        own ``set_commands`` calls don't race-overwrite the rail motor's velocity. Sets
        ``_homing_event`` while running so ``is_homing()`` reflects the in-progress state.
        """
        if direction == "up":
            motor_velocity = self.rail_speed * HOMING_SPEED_RATIO
        elif direction == "down":
            motor_velocity = -self.rail_speed * HOMING_SPEED_RATIO
        else:
            raise ValueError(f"direction must be 'up' or 'down', got {direction!r}")

        logger.info(f"Homing started with velocity {motor_velocity:.3f} rad/s (moving {direction})")

        with self._lock:
            self._homing_event.set()
            self._homing_start_time = time.time()

        start_time = time.time()
        last_velocity_set_time = start_time
        velocity_set_interval = 0.05  # Re-apply every 50 ms

        self.single_motor_control_interface.set_velocity(motor_velocity)

        try:
            while time.time() - start_time < self.homing_timeout:
                current_time = time.time()

                if current_time - last_velocity_set_time >= velocity_set_interval:
                    self.single_motor_control_interface.set_velocity(motor_velocity)
                    last_velocity_set_time = current_time

                with self._lock:
                    triggered = self.upper_limit_triggered if direction == "up" else self.lower_limit_triggered
                if triggered:
                    self.single_motor_control_interface.set_velocity(0.0)
                    elapsed = current_time - start_time
                    logger.info(f"{direction.capitalize()} limit reached in {elapsed:.1f}s")
                    return

                time.sleep(0.01)

            self.single_motor_control_interface.set_velocity(0.0)
            raise RuntimeError(f"Homing timed out after {self.homing_timeout}s moving {direction}")
        finally:
            with self._lock:
                self._homing_event.clear()
                self._homing_start_time = None

    def _stop_homing(self) -> None:
        """Stop homing procedure and reset state (assumes lock is held)"""
        self.single_motor_control_interface.set_velocity(0.0)
        self._homing_event.clear()
        self._homing_start_time = None

    def _set_home_zero(self, *, at_lower_limit: bool = True) -> None:
        """Zero the encoder at the current rail position."""
        self.single_motor_control_interface.set_zero_position()
        if at_lower_limit:
            logger.info("Linear rail encoder zeroed at lower limit (encoder 0 = home)")
        else:
            logger.info("Linear rail encoder zeroed at startup position (no limit-switch homing)")

    def is_homing(self) -> bool:
        """Check if linear rail is currently homing"""
        with self._lock:
            return self._homing_event.is_set()

    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the linear rail.

        ``position``/``velocity`` are the motor encoder value in rad / rad/s (scalar,
        unchanged for backward compatibility). ``position_linear``/``velocity_linear``
        are the same quantities in m / m/s, derived from ``meters_per_rad`` captured
        during startup calibration; they are ``None`` until the controller is calibrated
        (e.g. when ``auto_home=False`` and homing has not run).
        """
        motor_state = self.single_motor_control_interface.get_state()

        with self._lock:
            brake_on = self.brake_on
            initialized = self.initialized
            upper_limit = self.upper_limit_triggered
            lower_limit = self.lower_limit_triggered
            meters_per_rad = self.meters_per_rad

        if meters_per_rad is not None:
            linear_pos: Optional[float] = motor_state.pos * meters_per_rad
            linear_vel: Optional[float] = motor_state.vel * meters_per_rad
        else:
            linear_pos = None
            linear_vel = None

        return {
            "position": motor_state.pos,
            "velocity": motor_state.vel,
            "position_linear": linear_pos,
            "velocity_linear": linear_vel,
            "meters_per_rad": meters_per_rad,
            # Single source of truth for the rail's linear speed cap (m/s). Exposed
            # here so remote clients (e.g. the lerobot bi_yam_linear_bot) can inherit
            # the controller's cap instead of configuring it independently. ``None``
            # means no m/s cap is enforced (only the rad/s ceiling applies).
            "max_vel_mps": self.max_vel_mps,
            "brake_on": brake_on,
            "initialized": initialized,
            "upper_limit_triggered": upper_limit,
            "lower_limit_triggered": lower_limit,
            "motor_state": motor_state,
        }

    def set_velocity(self, vel: float) -> None:
        """Set the velocity of the linear rail, unit in rad/s (motor velocity)"""
        assert self.initialized, "Linear rail must be initialized before setting velocity"
        assert not self.brake_on, "Brake must be released before setting velocity"

        # Clamp the commanded motor velocity. Once calibrated we cap by the linear
        # speed limit (m/s), which makes the rail's top speed a fixed physical value
        # independent of motor gearing and applies to every caller (joystick +
        # remote). This is authoritative — we do NOT also clip to rail_speed, since a
        # high-gearing rail can need >rail_speed rad/s to reach max_vel_mps. Before
        # calibration (meters_per_rad is None) we fall back to the rad/s ceiling so a
        # command is never unbounded. Homing moves bypass set_velocity, so they are
        # never throttled here.
        mpr = self.meters_per_rad
        if self.max_vel_mps is not None and mpr:
            max_rad_s = abs(self.max_vel_mps / mpr)
            vel = float(np.clip(vel, -max_rad_s, max_rad_s))
        else:
            vel = float(np.clip(vel, -self.rail_speed, self.rail_speed))

        with self._lock:
            current_time = time.time()
            if current_time - self.last_command_time > self.command_timeout:
                try:
                    self.single_motor_control_interface.set_velocity(0.0)
                    logger.warning(
                        f"Linear rail command timeout ({self.command_timeout:.2f}s) detected, "
                        "but new command received (command stream recovered)"
                    )
                except Exception as e:
                    logger.error(f"Failed to stop linear rail on timeout: {e}")

            self.last_command_time = current_time

            if vel > 0.0 and self.upper_limit_triggered:
                logger.warning("Upper limit triggered, cannot move forward")
                self.single_motor_control_interface.set_velocity(0.0)
                return
            if vel < 0.0 and self.lower_limit_triggered:
                logger.warning("Lower limit triggered, cannot move backward")
                self.single_motor_control_interface.set_velocity(0.0)
                if self._homing_event.is_set():
                    self._set_home_zero()
                    elapsed_time = time.time() - self._homing_start_time if self._homing_start_time else 0.0
                    logger.info(f"Homing success! Zero position found in {elapsed_time:.1f}s")
                    self._stop_homing()
                return

            try:
                self.single_motor_control_interface.set_velocity(vel)
                if vel != 0.0:
                    logger.info(f"Linear rail velocity set to {vel:.3f} rad/s")
            except Exception as e:
                logger.error(f"Failed to set linear rail velocity: {e}")
                raise

    def cleanup(self) -> None:
        """Clean up resources"""
        try:
            self.single_motor_control_interface.set_velocity(0.0)
            if self.initialized:
                self.set_brake(engaged=True)
                self.gpio_backend.cleanup()
            logger.info("Linear rail controller cleaned up successfully")
        except Exception as e:
            logger.error(f"Cleanup error: {e}")
