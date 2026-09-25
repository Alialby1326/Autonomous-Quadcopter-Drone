"""Host-side tooling for the quadcopter's IMU stream.

Recording and watching what the board actually sends. Nothing here is in the
flight path -- the control loop runs on the STM32.

    telemetry  wire format and ImuSample
    sources    serial / log, both yielding the same sample stream
"""

from .telemetry import ImuSample, format_line, parse_line

__all__ = [
    "ImuSample",
    "format_line",
    "parse_line",
]
