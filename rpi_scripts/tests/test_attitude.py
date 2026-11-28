"""Sign conventions every estimator must honour, whatever its math.

CLAUDE.md "Frames": X forward, Y left, Z up. A right bank puts gravity on
*positive* body Y; nose down puts it on *negative* body X. This has already
caused one wrong test, so it is pinned here against each registered estimator.
"""

import math

import numpy as np
import pytest

from droneviz.attitude import ESTIMATORS, Attitude, rotation_matrix
from droneviz.telemetry import ImuSample

G = 9.80665


def at_rest(roll_deg: float, pitch_deg: float) -> ImuSample:
    """What a stationary IMU reads at this attitude: world-up in body frame."""
    r = rotation_matrix(Attitude(math.radians(roll_deg), math.radians(pitch_deg)))
    ax, ay, az = G * r.T @ np.array([0.0, 0.0, 1.0])
    return ImuSample(0.0, ax, ay, az, 0, 0, 0, 0, 0, 0)


def settle(name: str, sample: ImuSample) -> Attitude:
    est = ESTIMATORS[name]()
    att = None
    for _ in range(200):  # 2 s at 100 Hz, long enough for any smoothing
        att = est.update(sample, 0.01)
    return att


@pytest.mark.parametrize("name", list(ESTIMATORS))
def test_level_is_zero(name):
    att = settle(name, at_rest(0, 0))
    assert att.roll == pytest.approx(0, abs=1e-6)
    assert att.pitch == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize("name", list(ESTIMATORS))
def test_right_bank_is_positive_roll(name):
    sample = at_rest(20, 0)
    assert sample.ay > 0  # gravity on +Y, per CLAUDE.md
    att = settle(name, sample)
    assert math.degrees(att.roll) == pytest.approx(20, abs=0.5)
    assert abs(math.degrees(att.pitch)) < 0.5


@pytest.mark.parametrize("name", list(ESTIMATORS))
def test_nose_down_is_positive_pitch(name):
    sample = at_rest(0, 20)
    assert sample.ax < 0
    att = settle(name, sample)
    assert math.degrees(att.pitch) == pytest.approx(20, abs=0.5)
    assert abs(math.degrees(att.roll)) < 0.5


# Earth's field at a mid-northern latitude: 20 uT toward north (world +X),
# 45 uT down. The large dip is what makes tilt compensation matter.
FIELD_WORLD = np.array([20.0, 0.0, -45.0])


def at_rest_facing(roll_deg: float, pitch_deg: float, yaw_deg: float) -> ImuSample:
    """Accel and mag readings of a stationary IMU at this attitude."""
    att = Attitude(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    r = rotation_matrix(att)
    ax, ay, az = G * r.T @ np.array([0.0, 0.0, 1.0])
    mx, my, mz = r.T @ FIELD_WORLD
    return ImuSample(0.0, ax, ay, az, 0, 0, 0, mx, my, mz)


def test_mag_yaw_nose_left_is_positive():
    sample = at_rest_facing(0, 0, 30)
    assert sample.my < 0  # nose left of north: north is off to the right, -Y
    att = ESTIMATORS["accel + mag yaw"]().update(sample, 0.01)
    assert math.degrees(att.yaw) == pytest.approx(30, abs=0.01)


@pytest.mark.parametrize("roll, pitch, yaw", [
    (0, 0, 0), (0, 0, -60), (0, 0, 170),
    (20, 0, 45), (-30, 0, -120),   # banked
    (0, 20, 45), (0, -30, 100),    # pitched
])
def test_mag_yaw_is_tilt_compensated(roll, pitch, yaw):
    att = ESTIMATORS["accel + mag yaw"]().update(at_rest_facing(roll, pitch, yaw), 0.01)
    assert math.degrees(att.yaw) == pytest.approx(yaw, abs=0.01)


def test_mag_yaw_without_mag_data_is_none():
    att = ESTIMATORS["accel + mag yaw"]().update(at_rest(10, 5), 0.01)
    assert att.yaw is None


@pytest.mark.parametrize("name", list(ESTIMATORS))
@pytest.mark.parametrize("roll, pitch", [(30, 30), (-40, 25), (60, -20)])
def test_combined_roll_and_pitch(name, roll, pitch):
    # Single-axis tilts can't catch a formula that mixes pitch into roll.
    att = settle(name, at_rest(roll, pitch))
    assert math.degrees(att.roll) == pytest.approx(roll, abs=0.5)
    assert math.degrees(att.pitch) == pytest.approx(pitch, abs=0.5)


def test_log_replay_splices_resets_and_corrupt_stamps(tmp_path):
    """imu.log's shape: steady 10 ms, a board reset, then one garbled stamp."""
    from droneviz.sources import iter_log
    from droneviz.telemetry import format_line

    stamps = [600.00, 600.01, 600.02, 2.00, 2.01, 42478.0, 2.03, 2.04]
    log = tmp_path / "x.log"
    log.write_text("\n".join(format_line(ImuSample(t, 0, 0, G, 0, 0, 0, 0, 0, 0)) for t in stamps))

    times = [s.t for s in iter_log(str(log), realtime=False)]
    # The garbled stamp is dropped by parse_line; the reset becomes one
    # typical 10 ms step. Before, replay slept ~11.6 h at the garbled line.
    assert np.diff(times) == pytest.approx([0.01, 0.01, 0.01, 0.01, 0.02, 0.01])
