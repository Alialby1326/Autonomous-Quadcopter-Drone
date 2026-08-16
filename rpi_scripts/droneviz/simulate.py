"""Synthetic IMU data with known ground truth.

This is the piece that makes the estimator testable. A recorded log can show
that the filter *runs*; only a trajectory whose true attitude you already know
can show that it is *right*. Every sample this module produces carries the
exact (roll, pitch, yaw) that generated it, so the visualizer can draw the
error and the tests can assert a bound on it.

How it works: define the attitude trajectory analytically, differentiate it
analytically to get body rates, then rotate the known gravity and magnetic
field vectors into the body frame to get what an ideal accelerometer and
magnetometer would read. Corrupt all three with the noise, bias, and
quantization a real MPU9250 exhibits. The filter then has to recover what we
started from.

Frames and units match droneviz.attitude and droneviz.telemetry:
body X forward / Y left / Z up, world NWU, accel m/s^2, gyro deg/s, mag uT.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, List, Tuple

import numpy as np

from .attitude import G, euler_to_quat, quat_to_matrix
from .telemetry import FLAG_MAG_STALE, ImuSample


# --------------------------------------------------------------------------
# Trajectory primitives
#
# Both return value and derivative, and both are C1 continuous -- they start
# and end with zero slope. That matters: a trajectory with a rate discontinuity
# would demand infinite angular acceleration, and the "error" the filter shows
# at that instant would be an artifact of the trajectory rather than a real
# property of the filter.
# --------------------------------------------------------------------------


def _bump(t: float, t0: float, dur: float, amp: float) -> Tuple[float, float]:
    """Rise to `amp` and return to zero over `dur` seconds."""
    if t < t0 or t >= t0 + dur:
        return 0.0, 0.0
    u = (t - t0) / dur
    value = amp * np.sin(np.pi * u) ** 2
    rate = amp * np.pi / dur * np.sin(2 * np.pi * u)
    return float(value), float(rate)


def _turn(t: float, t0: float, dur: float, amp: float) -> Tuple[float, float]:
    """Move permanently from 0 to `amp` over `dur` seconds, then hold."""
    if t < t0:
        return 0.0, 0.0
    if t >= t0 + dur:
        return float(amp), 0.0
    u = (t - t0) / dur
    value = amp * (u - np.sin(2 * np.pi * u) / (2 * np.pi))
    rate = amp / dur * (1 - np.cos(2 * np.pi * u))
    return float(value), float(rate)


@dataclass
class Segment:
    """One trajectory component: which axis, when, how long, how far."""

    axis: str  # "roll" | "pitch" | "yaw"
    kind: str  # "bump" (there and back) | "turn" (and stay)
    t0: float
    duration: float
    amplitude_deg: float


# A scripted maneuver that exercises each axis alone and then all three at
# once. The isolated segments make it obvious which axis is wrong when one is;
# the combined segment is where convention errors that survive the isolated
# tests finally show up.
DEFAULT_MANEUVER: List[Segment] = [
    Segment("roll", "bump", 2.0, 4.0, 40.0),
    Segment("roll", "bump", 6.5, 4.0, -40.0),
    Segment("pitch", "bump", 11.5, 4.0, 30.0),
    Segment("pitch", "bump", 16.0, 4.0, -30.0),
    Segment("yaw", "turn", 21.0, 6.0, 360.0),
    Segment("roll", "bump", 28.0, 3.0, 25.0),
    Segment("pitch", "bump", 28.5, 3.0, -20.0),
    Segment("yaw", "turn", 28.0, 3.0, 90.0),
]

DEFAULT_DURATION = 32.0


def evaluate(segments: List[Segment], t: float) -> Tuple[np.ndarray, np.ndarray]:
    """Ground-truth (euler, euler_rate) in radians and rad/s at time t."""
    index = {"roll": 0, "pitch": 1, "yaw": 2}
    euler = np.zeros(3)
    rate = np.zeros(3)
    for seg in segments:
        fn = _bump if seg.kind == "bump" else _turn
        value, deriv = fn(t, seg.t0, seg.duration, np.radians(seg.amplitude_deg))
        euler[index[seg.axis]] += value
        rate[index[seg.axis]] += deriv
    return euler, rate


def euler_rate_to_body_rate(euler: np.ndarray, euler_rate: np.ndarray) -> np.ndarray:
    """Convert (roll_dot, pitch_dot, yaw_dot) to body angular rates p, q, r.

    Euler rates are measured about three *different* axes -- roll about the
    body X, pitch about an intermediate axis, yaw about the world Z -- so they
    are not what a gyro strapped to the body measures. This is the ZYX
    transform that relates the two. `test_attitude.py` checks it against a
    numerical quaternion derivative, which is the real guard against getting a
    sign wrong here.
    """
    roll, pitch, _ = euler
    rd, pd, yd = euler_rate
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    return np.array([
        rd - yd * sp,
        pd * cr + yd * cp * sr,
        -pd * sr + yd * cp * cr,
    ])


@dataclass
class SensorModel:
    """Imperfections to layer onto the ideal readings.

    Defaults are in the neighbourhood of an MPU9250 on a bench, not a flying
    quad -- a real airframe adds vibration well beyond this. Turn the noise up
    when you want to know how the filter behaves on a bad day.
    """

    accel_noise: float = 0.08          # m/s^2, 1-sigma per axis
    gyro_noise: float = 0.25           # deg/s, 1-sigma per axis
    mag_noise: float = 0.6             # uT, 1-sigma per axis

    gyro_bias_dps: Tuple[float, float, float] = (1.5, -0.8, 0.4)
    accel_bias: Tuple[float, float, float] = (0.05, -0.03, 0.02)
    mag_hard_iron: Tuple[float, float, float] = (3.0, -2.0, 1.5)

    # Quantization steps, mimicking the sensor LSB. None disables.
    accel_lsb: float = G / 4096.0      # +-8 g range
    gyro_lsb: float = 1.0 / 32.8       # +-1000 deg/s range
    mag_lsb: float = 0.15              # AK8963 16-bit mode

    mag_field_uT: float = 50.0
    mag_inclination_deg: float = 65.0

    seed: int = 0

    def world_field(self) -> np.ndarray:
        """Earth's field in the NWU world frame.

        Points north and, in the northern hemisphere, downward -- so it has a
        positive X and a negative Z component.
        """
        incl = np.radians(self.mag_inclination_deg)
        return self.mag_field_uT * np.array([np.cos(incl), 0.0, -np.sin(incl)])


def _quantize(value: np.ndarray, step: float) -> np.ndarray:
    if not step:
        return value
    return np.round(value / step) * step


def simulate(
    duration: float = DEFAULT_DURATION,
    rate_hz: float = 200.0,
    mag_rate_hz: float = 100.0,
    segments: List[Segment] = None,
    model: SensorModel = None,
    t_start: float = 0.0,
) -> Iterator[ImuSample]:
    """Yield ImuSamples with ground truth attached.

    `mag_rate_hz` below `rate_hz` reproduces the real timing: the AK8963 in
    continuous mode 2 updates at 100 Hz while the accel/gyro loop runs faster,
    so some samples carry no fresh magnetometer data and are marked
    FLAG_MAG_STALE. The filter has to cope with that, so the simulator had
    better produce it.
    """
    segments = DEFAULT_MANEUVER if segments is None else segments
    model = SensorModel() if model is None else model

    rng = np.random.default_rng(model.seed)
    dt = 1.0 / rate_hz
    n = int(round(duration * rate_hz))

    world_field = model.world_field()
    gyro_bias = np.array(model.gyro_bias_dps, dtype=float)
    accel_bias = np.array(model.accel_bias, dtype=float)
    hard_iron = np.array(model.mag_hard_iron, dtype=float)

    mag_period = 1.0 / mag_rate_hz if mag_rate_hz > 0 else 0.0
    next_mag_t = 0.0
    last_mag = None

    for i in range(n):
        t = i * dt

        euler, euler_rate = evaluate(segments, t)
        R = quat_to_matrix(euler_to_quat(*euler))  # body -> world

        # Accelerometer: specific force. Stationary in translation, so the only
        # specific force is the reaction to gravity, +G along world up.
        accel = R.T @ np.array([0.0, 0.0, G])
        accel = accel + accel_bias + rng.normal(0.0, model.accel_noise, 3)
        accel = _quantize(accel, model.accel_lsb)

        # Gyroscope, in deg/s to match the wire format.
        body_rate = euler_rate_to_body_rate(euler, euler_rate)
        gyro = np.degrees(body_rate) + gyro_bias + rng.normal(0.0, model.gyro_noise, 3)
        gyro = _quantize(gyro, model.gyro_lsb)

        flags = 0
        if mag_period and t + 1e-9 < next_mag_t:
            flags |= FLAG_MAG_STALE
            mag = last_mag if last_mag is not None else np.zeros(3)
        else:
            next_mag_t = t + mag_period
            mag = R.T @ world_field
            mag = mag + hard_iron + rng.normal(0.0, model.mag_noise, 3)
            mag = _quantize(mag, model.mag_lsb)
            last_mag = mag

        yield ImuSample(
            t=t_start + t,
            ax=float(accel[0]), ay=float(accel[1]), az=float(accel[2]),
            gx=float(gyro[0]), gy=float(gyro[1]), gz=float(gyro[2]),
            mx=float(mag[0]), my=float(mag[1]), mz=float(mag[2]),
            flags=flags,
            truth=(float(euler[0]), float(euler[1]), float(euler[2])),
        )
