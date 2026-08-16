"""Host-side tooling for the quadcopter's IMU stream.

Development and validation only -- the flight-critical attitude estimator and
PID loop run on the STM32. What lives here is the harness that lets you check
those algorithms against ground truth before committing them to firmware, plus
the tools to record and watch real hardware.

    telemetry  wire format, ImuSample, ZeroMQ transport
    sources    serial / log / simulator, all yielding the same sample stream
    simulate   synthetic trajectories with known ground truth
    attitude   quaternion complementary filter, written to port to C
"""

from .attitude import ComplementaryFilter, quat_to_euler, euler_to_quat
from .telemetry import ImuSample, Publisher, Subscriber, format_line, parse_line

__all__ = [
    "ComplementaryFilter",
    "ImuSample",
    "Publisher",
    "Subscriber",
    "euler_to_quat",
    "format_line",
    "parse_line",
    "quat_to_euler",
]
