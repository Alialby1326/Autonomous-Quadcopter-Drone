"""Attitude estimators, run host-side for comparison against each other.

This is the scratchpad for estimator math. Nothing here flies -- the real
estimator will run on the STM32 -- but the host is where it's cheap to try one
against another on the same recorded data.

Each estimator takes `ImuSample`s one at a time and returns an `Attitude`.
`visualize_attitude.py` runs every entry in `ESTIMATORS` side by side, so trying
a new idea means writing a class and adding one line to that dict.

Conventions (see CLAUDE.md "Frames"):

    body frame   X forward, Y left, Z up (the MPU9250's)
    angles       Z-Y-X Euler, radians: R = Rz(yaw) @ Ry(pitch) @ Rx(roll),
                 body -> world
    roll  > 0    about +X: carries +Y toward +Z, i.e. a RIGHT bank
    pitch > 0    about +Y: carries +Z toward +X, i.e. NOSE DOWN

Level and at rest the accelerometer reads (0, 0, +g). In a right bank it reads
positive Y; nose down, negative X.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from .telemetry import ImuSample


@dataclass(frozen=True)
class Attitude:
    roll: float  # rad
    pitch: float  # rad
    yaw: Optional[float] = None  # rad; None when the estimator can't observe it


def rotation_matrix(att: Attitude) -> np.ndarray:
    """Body -> world rotation for an attitude. Unobserved yaw is drawn as 0."""
    cr, sr = math.cos(att.roll), math.sin(att.roll)
    cp, sp = math.cos(att.pitch), math.sin(att.pitch)
    yaw = att.yaw or 0.0
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# -- math --------------------------------------------------------------------

ROLL_MAX_RAD = math.radians(85)
PITCH_MAX_RAD = math.radians(85)


def accel_to_pitch_roll(ax: float, ay: float, az: float) -> tuple[float, float]:
    eps = 1e-9
    pitch = math.atan(-ax / (math.sqrt(ay * ay + az * az) + eps))
    roll = math.atan(ay / (math.sqrt(ax * ax + az * az) + eps))
    pitch = max(-PITCH_MAX_RAD, min(PITCH_MAX_RAD, pitch))
    roll = max(-ROLL_MAX_RAD, min(ROLL_MAX_RAD, roll))
    return pitch, roll


# -- estimators --------------------------------------------------------------


class AccelTilt:

    def reset(self) -> None:
        pass

    def update(self, s: ImuSample, dt: float) -> Attitude:
        pitch, roll = accel_to_pitch_roll(s.ax, s.ay, s.az)
        return Attitude(roll, pitch)


class Smoothed:
    """First-order low-pass on another estimator's roll/pitch.

    The original viewer smoothed with a fixed alpha per *rendered frame*, which
    made the filter depend on the render rate. This uses a time constant and
    the device's dt instead, so it behaves the same at any loop rate.
    """

    def __init__(self, inner, tau_s: float):
        self.inner = inner
        self.tau_s = tau_s
        self.state: Optional[Attitude] = None

    def reset(self) -> None:
        self.inner.reset()
        self.state = None

    def update(self, s: ImuSample, dt: float) -> Attitude:
        raw = self.inner.update(s, dt)
        if self.state is None or dt <= 0:
            self.state = raw
        else:
            a = dt / (self.tau_s + dt)
            self.state = Attitude(
                self.state.roll + a * (raw.roll - self.state.roll),
                self.state.pitch + a * (raw.pitch - self.state.pitch),
                raw.yaw,
            )
        return self.state


# Display name -> factory. Insertion order sets the plot colors, and the first
# entry is the one drawn solid in the 3D view.
ESTIMATORS: dict[str, Callable[[], object]] = {
    "accel": AccelTilt,
    "accel lpf 0.1s": lambda: Smoothed(AccelTilt(), tau_s=0.1),
}
