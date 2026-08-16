"""Quaternion complementary filter (Mahony-style) for 9-axis attitude.

Deliberately written to be portable to C: fixed-size state, no allocation in
the update path, no library calls beyond arithmetic and one square root. The
plan is to validate it here against the simulator's ground truth, then port it
to the STM32 -- so resist the urge to reach for scipy.

Frames
------
Body:  X forward, Y left, Z up. At rest and level the accelerometer reads
       (0, 0, +9.81): it measures specific force, so the up axis reads +g.
World: NWU -- X magnetic north, Y west, Z up. Right-handed.

The quaternion `q` rotates *body vectors into the world frame*:
`v_world = R(q) @ v_body`.

Euler angles use the aerospace ZYX convention (yaw, then pitch, then roll).
Yaw is measured from magnetic north, positive counter-clockwise seen from
above -- i.e. toward west, not the compass convention. Convert with
`heading_deg()` if you want degrees clockwise from north.

Why a complementary filter and not an EKF
-----------------------------------------
The gyro is accurate over short intervals but its integral drifts; the
accelerometer and magnetometer are noisy and disturbed by linear acceleration
and nearby iron, but have no long-term drift. This filter integrates the gyro
and steers that integration toward the accel/mag reference with a proportional
term, while a slow integral term learns the gyro bias. That is most of what an
EKF buys you here, in about forty lines, with two tunable numbers instead of a
covariance matrix. The EKF is a later refinement, not a prerequisite.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# Gravity used to normalize the accelerometer's magnitude check.
G = 9.80665


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R such that v_world = R @ v_body."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """(roll, pitch, yaw) in radians, aerospace ZYX convention."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # Clip guards against a sin argument drifting a hair outside [-1, 1] through
    # accumulated rounding, which would make arcsin return NaN at exactly the
    # attitudes (straight up, straight down) where you least want a NaN.
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Inverse of quat_to_euler. Used by the simulator to build ground truth."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def heading_deg(yaw_rad: float) -> float:
    """Yaw in radians (CCW from north) as a compass heading in [0, 360)."""
    return float(np.degrees(-yaw_rad) % 360.0)


def angle_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b, wrapped to (-pi, pi].

    Needed everywhere attitude error is measured: the naive difference between
    179 deg and -179 deg is 358 deg when the real error is 2 deg.
    """
    return float((a - b + np.pi) % (2 * np.pi) - np.pi)


class ComplementaryFilter:
    """Mahony-style attitude filter with online gyro-bias estimation.

    Parameters
    ----------
    kp: proportional gain on the accel/mag error. Higher trusts the reference
        vectors more and converges faster, at the cost of letting linear
        acceleration and magnetic disturbance into the estimate.
    ki: integral gain, which is what actually learns the gyro bias. Keep it
        small; too large and it fights the proportional term.
    accel_weight, mag_weight: relative trust in each reference. Setting
        mag_weight to 0 gives a 6-axis filter with unobservable yaw -- useful
        while the magnetometer is uncalibrated.
    accel_tolerance: fraction of g by which |accel| may deviate before the
        sample is rejected as a gravity reference. Under linear acceleration
        the accelerometer no longer points down, and using it anyway is how a
        quad's attitude estimate tips over during aggressive maneuvers.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.05,
        accel_weight: float = 1.0,
        mag_weight: float = 0.5,
        accel_tolerance: float = 0.25,
    ):
        self.kp = kp
        self.ki = ki
        self.accel_weight = accel_weight
        self.mag_weight = mag_weight
        self.accel_tolerance = accel_tolerance

        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)  # estimated gyro bias, rad/s
        self._initialized = False

        # Diagnostics, for the visualizer's HUD.
        self.accel_rejected = False
        self.mag_used = False

    # -- state accessors ---------------------------------------------------

    @property
    def euler(self) -> np.ndarray:
        return quat_to_euler(self.q)

    @property
    def roll(self) -> float:
        return float(self.euler[0])

    @property
    def pitch(self) -> float:
        return float(self.euler[1])

    @property
    def yaw(self) -> float:
        return float(self.euler[2])

    def reset(self) -> None:
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self._initialized = False

    # -- initialization ----------------------------------------------------

    def initialize_from(self, accel: np.ndarray, mag: Optional[np.ndarray] = None) -> None:
        """Snap straight to the attitude implied by accel (and mag, if given).

        Without this the filter starts level and has to converge from whatever
        attitude the vehicle is actually sitting at, which takes seconds and
        looks like a bug. One measurement is enough to get within noise.
        """
        a = accel / (np.linalg.norm(accel) + 1e-12)
        # Roll and pitch that put the measured gravity direction on +Z.
        roll = np.arctan2(a[1], a[2])
        pitch = np.arctan2(-a[0], np.hypot(a[1], a[2]))

        yaw = 0.0
        if mag is not None and np.linalg.norm(mag) > 1e-6:
            # Tilt-compensated heading. Undo roll and pitch to get the field in
            # a level frame, which by construction equals Rz(-yaw) applied to
            # the earth field (Bx, 0, Bz) -- so its horizontal part is
            # (Bx*cos(yaw), -Bx*sin(yaw)) and yaw falls straight out.
            #
            # Undoing the tilt is essential: the field has a large vertical
            # component (~65 degrees of inclination at mid-latitudes), and
            # without this any roll or pitch swings that component into the
            # horizontal axes and shows up as a heading error.
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            mx, my, mz = mag / np.linalg.norm(mag)
            north = cp * mx + sp * sr * my + sp * cr * mz
            west = cr * my - sr * mz
            yaw = np.arctan2(-west, north)

        self.q = euler_to_quat(roll, pitch, yaw)
        self.bias = np.zeros(3)
        self._initialized = True

    # -- update ------------------------------------------------------------

    def update(
        self,
        gyro: np.ndarray,
        accel: np.ndarray,
        mag: Optional[np.ndarray],
        dt: float,
    ) -> np.ndarray:
        """Advance the estimate by dt seconds. Returns (roll, pitch, yaw).

        `gyro` is rad/s, `accel` is m/s^2, `mag` is uT (or None to skip the
        magnetometer correction this step). All in the body frame.
        """
        if dt <= 0.0 or dt > 1.0:
            # A nonsensical dt means a dropped frame or a restarted device.
            # Integrating it would fling the estimate somewhere arbitrary.
            return self.euler

        if not self._initialized:
            self.initialize_from(accel, mag)
            return self.euler

        error = np.zeros(3)
        self.accel_rejected = False
        self.mag_used = False

        # -- gravity reference --
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            if abs(a_norm - G) <= self.accel_tolerance * G:
                a = accel / a_norm
                # Where the filter currently believes "up" lies in body axes:
                # the third row of R(q), i.e. R^T @ (0,0,1).
                w, x, y, z = self.q
                up_body = np.array([
                    2 * (x * z - w * y),
                    2 * (y * z + w * x),
                    1 - 2 * (x * x + y * y),
                ])
                error += self.accel_weight * np.cross(a, up_body)
            else:
                self.accel_rejected = True

        # -- magnetic reference --
        if mag is not None and self.mag_weight > 0.0:
            m_norm = np.linalg.norm(mag)
            if m_norm > 1e-6:
                m = mag / m_norm
                R = quat_to_matrix(self.q)
                # Rotate the measured field into the world frame, then flatten
                # it onto the north axis. Whatever east/west component it has
                # is heading error -- that is precisely the signal we want.
                h = R @ m
                b = np.array([np.hypot(h[0], h[1]), 0.0, h[2]])
                mag_body = R.T @ b
                error += self.mag_weight * np.cross(m, mag_body)
                self.mag_used = True

        # Integral term learns the gyro bias. Note the sign: `error` is the
        # correction to add to the rate, so the bias it implies is its negative.
        if self.ki > 0.0:
            self.bias -= self.ki * error * dt

        rate = gyro - self.bias + self.kp * error

        # Quaternion integration: q_dot = 0.5 * q (x) (0, rate).
        w, x, y, z = self.q
        wx, wy, wz = rate
        q_dot = 0.5 * np.array([
            -x * wx - y * wy - z * wz,
            w * wx + y * wz - z * wy,
            w * wy - x * wz + z * wx,
            w * wz + x * wy - y * wx,
        ])
        self.q = quat_normalize(self.q + q_dot * dt)

        return self.euler
