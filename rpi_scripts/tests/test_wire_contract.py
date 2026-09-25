"""Check that the firmware and the host agree on the telemetry contract.

The wire format and the health flags are defined twice -- once in C for the
STM32, once in Python here -- and nothing at build time connects the two. Edit
one without the other and the failure is silent and confusing: fields shift by
one, or a flag decodes as the wrong condition. These tests read the actual
firmware headers and fail if the definitions have drifted apart.

They are pure text inspection, so they need no toolchain and no hardware.
"""

import re
from pathlib import Path

import pytest

from droneviz import telemetry

REPO = Path(__file__).resolve().parents[2]
IMU_H = REPO / "Core" / "Inc" / "imu.h"
TELEMETRY_C = REPO / "Core" / "Src" / "telemetry.c"

pytestmark = pytest.mark.skipif(
    not IMU_H.exists() or not TELEMETRY_C.exists(),
    reason="firmware sources not present",
)


def firmware_flags() -> dict:
    """Pull the TLM_FLAG_* definitions out of Core/Inc/imu.h."""
    pattern = re.compile(r"#define\s+TLM_FLAG_(\w+)\s+\(1u\s*<<\s*(\d+)\)")
    return {
        name: 1 << int(shift)
        for name, shift in pattern.findall(IMU_H.read_text())
    }


def test_flag_definitions_match():
    firmware = firmware_flags()
    host = {name: bit for bit, name in telemetry.FLAG_NAMES.items()}
    assert firmware, "no TLM_FLAG_* definitions found in imu.h"
    assert firmware == {name: bit for name, bit in host.items()}, (
        f"firmware {firmware} != host {host}"
    )


def firmware_format() -> str:
    """Pull the telemetry frame's printf format out of Core/Src/telemetry.c."""
    text = TELEMETRY_C.read_text()
    match = re.search(r'"(I,%lu(?:,[^"]*)*)\\n"', text)
    assert match, "could not find the telemetry frame format in telemetry.c"
    return match.group(1)


def test_firmware_format_has_the_expected_field_count():
    fmt = firmware_format()
    assert fmt.count("%") == telemetry.FRAME_FIELDS - 1  # prefix is a literal
    assert fmt.startswith(telemetry.FRAME_PREFIX + ",")


def test_a_line_the_firmware_would_emit_parses_here():
    """Render the firmware's own format string and parse it with the host code."""
    fmt = firmware_format()
    # C's %lu and %u are Python's %d; the float conversions are identical.
    py_fmt = fmt.replace("%lu", "%d").replace("%u", "%d")

    t_us = 1234567
    values = [0.5, -1.25, 9.8067, 10.0, -20.5, 0.25, 12.3, -45.6, 7.89]
    flags = telemetry.FLAG_MAG_STALE

    line = py_fmt % (t_us, *values, flags)
    sample = telemetry.parse_line(line)

    assert sample is not None, f"host could not parse firmware output: {line!r}"
    assert sample.t == pytest.approx(t_us / 1e6)
    assert list(sample.accel) == pytest.approx(values[0:3])
    assert list(sample.gyro_dps) == pytest.approx(values[3:6])
    assert list(sample.mag) == pytest.approx(values[6:9])
    assert sample.flags == flags


def test_firmware_buffer_is_large_enough_for_a_worst_case_frame():
    """A truncated frame would be dropped, so the buffer has to fit the worst one.

    Worst case is every field at full width with a negative sign: the +-8 g and
    +-1000 deg/s ranges configured in imu.c, a saturated magnetometer, and a
    timestamp just before the 32-bit microsecond counter wraps.
    """
    declared = re.search(r"char\s+line\[(\d+)\]", TELEMETRY_C.read_text())
    assert declared, "could not find the frame buffer declaration"
    size = int(declared.group(1))

    py_fmt = firmware_format().replace("%lu", "%d").replace("%u", "%d")
    worst = py_fmt % (
        4294967295,
        -78.4532, -78.4532, -78.4532,      # +-8 g in m/s^2
        -1000.0, -1000.0, -1000.0,          # +-1000 deg/s
        -4912.0, -4912.0, -4912.0,          # AK8963 full scale, uT
        255,
    )
    needed = len(worst) + 1 + 1  # newline + NUL
    assert size >= needed, f"line[{size}] is too small; worst case needs {needed}"


# -- host-side parser --------------------------------------------------------


def test_wire_format_roundtrip():
    original = telemetry.ImuSample(
        t=1.234567, ax=0.5, ay=-1.25, az=9.81,
        gx=10.0, gy=-20.5, gz=0.25,
        mx=12.3, my=-45.6, mz=7.89, flags=telemetry.FLAG_MAG_STALE,
    )
    parsed = telemetry.parse_line(telemetry.format_line(original))
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
    assert telemetry.parse_line(line) is None


def test_mag_valid_reflects_flags():
    base = dict(t=0.0, ax=0, ay=0, az=9.81, gx=0, gy=0, gz=0, mx=1, my=2, mz=3)
    assert telemetry.ImuSample(**base).mag_valid
    assert not telemetry.ImuSample(**base, flags=telemetry.FLAG_MAG_STALE).mag_valid
