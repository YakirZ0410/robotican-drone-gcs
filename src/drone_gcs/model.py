"""Pure, deterministic motion and battery model for the simulated drone."""

from dataclasses import dataclass, replace
import math


# Percentage points drained per second. These are illustrative simulation rates.
IDLE_BATTERY_DRAIN_PER_SECOND = 0.002
MOTION_BATTERY_DRAIN_PER_SECOND = 0.01


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    """Normalized body-frame velocity: +vx forward, +vy left (ROS convention)."""

    vx: float
    vy: float
    seq: int


@dataclass(frozen=True, slots=True)
class DroneState:
    """World position in robot-aligned axes: +x forward, +y left."""

    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    battery_pct: float = 100.0
    last_seq: int = -1


def _normalized_axis(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("velocity must be a finite number")
    return max(-1.0, min(1.0, float(value)))


def apply_command(state: DroneState, command: VelocityCommand) -> DroneState:
    """Update both commanded axes together; clamp each to [-1, 1]."""

    vx = _normalized_axis(command.vx)
    vy = _normalized_axis(command.vy)
    return replace(state, vx=vx, vy=vy, last_seq=command.seq)


def current_speed(state: DroneState) -> float:
    """Magnitude of the current velocity in world-units per second."""

    return math.hypot(state.vx, state.vy)


def advance_state(state: DroneState, dt_s: float) -> DroneState:
    """Advance the drone using elapsed seconds, independent of frame rate."""

    if isinstance(dt_s, bool) or not isinstance(dt_s, (int, float)) or not math.isfinite(dt_s) or dt_s < 0:
        raise ValueError("elapsed time must be a finite nonnegative number")

    battery_drain = dt_s * (
        IDLE_BATTERY_DRAIN_PER_SECOND
        + MOTION_BATTERY_DRAIN_PER_SECOND * current_speed(state)
    )
    return replace(
        state,
        x=state.x + state.vx * dt_s,
        y=state.y + state.vy * dt_s,
        battery_pct=max(0.0, state.battery_pct - battery_drain),
    )
