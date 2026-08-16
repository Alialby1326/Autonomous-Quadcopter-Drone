"""Tests for the attitude filter and the simulator that validates it.

Most of these are convention tests rather than algorithm tests. The algorithm
is short and well known; what actually goes wrong in practice is a sign, an
axis order, or a frame mismatch between two files that each look correct on
their own. So the tests assert the relationships *between* modules: that the
simulator's gyro matches the derivative of the simulator's own attitude, that
the filter recovers what the simulator started from.
"""

import numpy as np
import pytest

from droneviz import attitude, simulate
from droneviz.attitude import ComplementaryFilter, euler_to_quat, quat_to_euler
from droneviz.telemetry import (
    FLAG_MAG_STALE,
    ImuSample,
    format_line,
    parse_line,
)


# -- quaternion / Euler round trips ----------------------------------------


@pytest.mark.parametrize(
    "roll,pitch,yaw",
    [
        (0.0, 0.0, 0.0),
        (0.3, 0.0, 0.0),
        (0.0, -0.4, 0.0),
        (0.0, 0.0, 1.2),
        (0.3, -0.4, 1.2),
        (-1.1, 0.7, -2.5),
    ],
)
def test_euler_quat_roundtrip(roll, pitch, yaw):
    q = euler_to_quat(roll, pitch, yaw)
    assert np.allclose(quat_to_euler(q), [roll, pitch, yaw], atol=1e-9)


def test_rotation_matrix_is_orthonormal():
    q = euler_to_quat(0.4, -0.2, 2.0)
    R = attitude.quat_to_matrix(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)


def test_level_accel_reads_gravity_on_z():
    """Sanity-check the sign convention the whole stack depends on."""
    R = attitude.quat_to_matrix(euler_to_quat(0.0, 0.0, 0.0))
    accel = R.T @ np.array([0.0, 0.0, attitude.G])
    assert np.allclose(accel, [0.0, 0.0, attitude.G])


def test_positive_roll_puts_gravity_on_positive_y():
    """Pin down the roll sign, which is easy to get backwards.

    Body is X forward, Y left, Z up. A positive rotation about X carries +Y
    toward +Z -- the left side rises, which is a bank to the right. World "up"
    then has a positive projection on body Y, so the accelerometer reads
    +G*sin(roll) there. Flip either the frame or the rotation sense and this
    test fails, which is the point.
    """
    R = attitude.quat_to_matrix(euler_to_quat(np.radians(30), 0.0, 0.0))
    accel = R.T @ np.array([0.0, 0.0, attitude.G])
    assert accel[1] == pytest.approx(attitude.G * 0.5)
    assert accel[2] == pytest.approx(attitude.G * np.cos(np.radians(30)))
    assert np.isclose(np.linalg.norm(accel), attitude.G)


# -- the simulator's internal consistency ----------------------------------


def test_body_rate_matches_numerical_quaternion_derivative():
    """The euler-rate -> body-rate transform must agree with the quaternions.

    This is the test that catches a wrong sign in euler_rate_to_body_rate. It
    computes body rates two independent ways -- analytically from the Euler
    rates, and numerically by differencing the attitude quaternion -- and
    requires them to agree. A convention error breaks one but not the other.
    """
    segments = simulate.DEFAULT_MANEUVER
    h = 1e-6
    for t in np.linspace(0.5, 30.0, 60):
        euler, euler_rate = simulate.evaluate(segments, t)
        analytic = simulate.euler_rate_to_body_rate(euler, euler_rate)

        q0 = euler_to_quat(*simulate.evaluate(segments, t - h)[0])
        q1 = euler_to_quat(*simulate.evaluate(segments, t + h)[0])
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q_dot = (q1 - q0) / (2 * h)

        # omega_body = 2 * q^-1 (x) q_dot, vector part.
        w, x, y, z = q0
        q_conj = np.array([w, -x, -y, -z])
        numeric = 2 * np.array([
            q_conj[0] * q_dot[1] + q_conj[1] * q_dot[0]
            + q_conj[2] * q_dot[3] - q_conj[3] * q_dot[2],
            q_conj[0] * q_dot[2] - q_conj[1] * q_dot[3]
            + q_conj[2] * q_dot[0] + q_conj[3] * q_dot[1],
            q_conj[0] * q_dot[3] + q_conj[1] * q_dot[2]
            - q_conj[2] * q_dot[1] + q_conj[3] * q_dot[0],
        ])

        assert np.allclose(analytic, numeric, atol=1e-4), f"mismatch at t={t}"


def test_trajectory_is_continuous():
    """No attitude or rate jumps, which would be trajectory artifacts."""
    ts = np.linspace(0, simulate.DEFAULT_DURATION, 20000)
    values = np.array([simulate.evaluate(simulate.DEFAULT_MANEUVER, t) for t in ts])
    euler, rate = values[:, 0, :], values[:, 1, :]
    assert np.max(np.abs(np.diff(euler, axis=0))) < 0.01
    assert np.max(np.abs(np.diff(rate, axis=0))) < 0.01


def test_simulator_marks_stale_mag_samples():
    samples = list(simulate.simulate(duration=1.0, rate_hz=200.0, mag_rate_hz=100.0))
    stale = sum(1 for s in samples if s.flags & FLAG_MAG_STALE)
    # Half the samples fall between magnetometer updates.
    assert 0.4 * len(samples) < stale < 0.6 * len(samples)


# -- the filter ------------------------------------------------------------


def run_filter(samples, **kwargs):
    """Feed samples through the filter, returning estimates and truth."""
    filt = ComplementaryFilter(**kwargs)
    est, truth, times = [], [], []
    prev_t = None
    for s in samples:
        dt = 0.0 if prev_t is None else s.t - prev_t
        prev_t = s.t
        angles = filt.update(s.gyro, s.accel, s.mag if s.mag_valid else None, dt)
        est.append(angles)
        truth.append(s.truth)
        times.append(s.t)
    return np.array(est), np.array(truth), np.array(times), filt


def max_error_deg(est, truth, times, settle=3.0):
    """Worst angular error in degrees, ignoring the initial convergence."""
    mask = times >= settle
    err = np.array([
        [attitude.angle_diff(e[i], t[i]) for i in range(3)]
        for e, t in zip(est[mask], truth[mask])
    ])
    return np.degrees(np.abs(err)).max(axis=0)


def test_filter_tracks_the_full_maneuver():
    samples = list(simulate.simulate(duration=simulate.DEFAULT_DURATION, rate_hz=200.0))
    est, truth, times, _ = run_filter(samples)
    roll_err, pitch_err, yaw_err = max_error_deg(est, truth, times)
    assert roll_err < 5.0, f"roll error {roll_err:.2f} deg"
    assert pitch_err < 5.0, f"pitch error {pitch_err:.2f} deg"
    assert yaw_err < 10.0, f"yaw error {yaw_err:.2f} deg"


def test_filter_converges_when_stationary_and_level():
    model = simulate.SensorModel(seed=3)
    samples = list(
        simulate.simulate(duration=8.0, rate_hz=200.0, segments=[], model=model)
    )
    est, truth, times, _ = run_filter(samples)
    roll_err, pitch_err, _ = max_error_deg(est, truth, times, settle=2.0)
    assert roll_err < 1.5
    assert pitch_err < 1.5


def test_filter_estimates_gyro_bias():
    """The integral term should recover the bias the simulator injected."""
    injected = (2.0, -1.5, 1.0)  # deg/s
    model = simulate.SensorModel(gyro_bias_dps=injected, seed=7)
    samples = list(
        simulate.simulate(duration=40.0, rate_hz=200.0, segments=[], model=model)
    )
    _, _, _, filt = run_filter(samples)
    estimated = np.degrees(filt.bias)
    assert np.allclose(estimated, injected, atol=0.6), (
        f"estimated bias {estimated} vs injected {injected}"
    )


def test_uncorrected_gyro_bias_would_have_drifted():
    """Establish that the previous test is actually testing something.

    Integrating the biased gyro with no correction drifts tens of degrees over
    the same interval, so the bounds above are not passing by accident.
    """
    model = simulate.SensorModel(gyro_bias_dps=(2.0, -1.5, 1.0), seed=7)
    samples = list(
        simulate.simulate(duration=40.0, rate_hz=200.0, segments=[], model=model)
    )
    est, truth, times, _ = run_filter(samples, kp=0.0, ki=0.0)
    assert max_error_deg(est, truth, times).max() > 20.0


def test_yaw_rotation_leaves_roll_and_pitch_alone():
    segments = [simulate.Segment("yaw", "turn", 2.0, 6.0, 360.0)]
    samples = list(simulate.simulate(duration=12.0, rate_hz=200.0, segments=segments))
    est, truth, times, _ = run_filter(samples)
    roll_err, pitch_err, yaw_err = max_error_deg(est, truth, times)
    assert roll_err < 3.0
    assert pitch_err < 3.0
    assert yaw_err < 8.0


def test_filter_rejects_accelerometer_under_linear_acceleration():
    """A shaken accelerometer must not be believed as a gravity reference."""
    filt = ComplementaryFilter()
    level = np.array([0.0, 0.0, attitude.G])
    for _ in range(400):
        filt.update(np.zeros(3), level, None, 0.005)
    assert not filt.accel_rejected

    # Now hand it 3 g sideways -- a real vehicle accelerating, not a tilt.
    filt.update(np.zeros(3), np.array([3 * attitude.G, 0.0, attitude.G]), None, 0.005)
    assert filt.accel_rejected
    assert abs(filt.roll) < np.radians(1.0)
    assert abs(filt.pitch) < np.radians(1.0)


def test_filter_ignores_absurd_dt():
    filt = ComplementaryFilter()
    filt.update(np.zeros(3), np.array([0.0, 0.0, attitude.G]), None, 0.005)
    before = filt.q.copy()
    filt.update(np.array([5.0, 5.0, 5.0]), np.array([0.0, 0.0, attitude.G]), None, 90.0)
    assert np.allclose(filt.q, before)


def test_initialize_from_snaps_to_measured_attitude():
    roll, pitch, yaw = np.radians(25.0), np.radians(-15.0), np.radians(70.0)
    R = attitude.quat_to_matrix(euler_to_quat(roll, pitch, yaw))
    accel = R.T @ np.array([0.0, 0.0, attitude.G])
    mag = R.T @ simulate.SensorModel().world_field()

    filt = ComplementaryFilter()
    filt.initialize_from(accel, mag)
    assert np.allclose(filt.euler, [roll, pitch, yaw], atol=1e-6)


# -- wire format -----------------------------------------------------------


def test_wire_format_roundtrip():
    original = ImuSample(
        t=1.234567, ax=0.5, ay=-1.25, az=9.81,
        gx=10.0, gy=-20.5, gz=0.25,
        mx=12.3, my=-45.6, mz=7.89, flags=FLAG_MAG_STALE,
    )
    parsed = parse_line(format_line(original))
    assert parsed is not None
    assert parsed.t == pytest.approx(original.t, abs=1e-6)
    assert parsed.accel == pytest.approx(original.accel)
    assert parsed.gyro_dps == pytest.approx(original.gyro_dps)
    assert parsed.mag == pytest.approx(original.mag)
    assert parsed.flags == original.flags


@pytest.mark.parametrize(
    "line",
    [
        "",
        "hello",
        "I,123",                              # truncated
        "I,123,1,2,3,4,5,6,7,8,9",            # missing flags
        "I,123,1,2,3,4,5,6,7,8,9,0,extra",    # too many fields
        "I,abc,1,2,3,4,5,6,7,8,9,0",          # bad timestamp
        "ACCELEROMETER: X: 0, Y: 0, Z: 9",    # legacy format
        "\x00\xff garbage",                   # baud mismatch
    ],
)
def test_parse_line_rejects_junk(line):
    assert parse_line(line) is None


def test_mag_valid_reflects_flags():
    base = dict(t=0.0, ax=0, ay=0, az=9.81, gx=0, gy=0, gz=0, mx=1, my=2, mz=3)
    assert ImuSample(**base).mag_valid
    assert not ImuSample(**base, flags=FLAG_MAG_STALE).mag_valid
