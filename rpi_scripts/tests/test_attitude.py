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
