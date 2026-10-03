"""Sample sources: live serial or recorded log.

Both yield the same `ImuSample` stream, so the visualizer is written once and
works against either -- watching the board live and replaying a capture of it
differ by one command-line flag.
"""

from __future__ import annotations

import re
import time
from typing import Iterator

from .telemetry import ImuSample, parse_line


def iter_serial(
    port: str,
    baud: int = 460800,
    timeout: float = 1.0,
) -> Iterator[ImuSample]:
    """Live samples from the STM32.

    Lines that do not parse are skipped rather than raised on: opening a serial
    port mid-stream reliably yields one partial line, and a baud mismatch
    yields nothing but garbage. Neither is worth a traceback.
    """
    try:
        import serial  # pyserial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required to read the serial port.\n"
            "    pip install -r rpi_scripts/requirements.txt"
        ) from exc

    with serial.Serial(port, baud, timeout=timeout) as link:
        link.reset_input_buffer()
        while True:
            raw = link.readline()
            if not raw:
                continue
            sample = parse_line(raw.decode("ascii", errors="replace"))
            if sample is not None:
                yield sample


def iter_log(
    path: str,
    loop: bool = False,
    realtime: bool = True,
    speed: float = 1.0,
) -> Iterator[ImuSample]:
    """Replay a recorded log, paced by the timestamps inside it.

    Handles both the current wire format and the older three-line
    `ACCELEROMETER:/GYROSCOPE:/MAGNETOMETER:` capture (see `_iter_legacy_log`).
    """
    with open(path, "r") as handle:
        text = handle.read()

    samples = [s for s in (parse_line(line) for line in text.splitlines()) if s]
    if not samples:
        samples = list(_iter_legacy_log(text))
    if not samples:
        raise SystemExit(f"no IMU samples found in {path}")

    t_offset = 0.0
    while True:
        wall_start = time.monotonic()
        base = samples[0].t
        for sample in samples:
            elapsed = sample.t - base
            if realtime and speed > 0:
                delay = wall_start + elapsed / speed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            yield ImuSample(
                t=t_offset + elapsed,
                ax=sample.ax, ay=sample.ay, az=sample.az,
                gx=sample.gx, gy=sample.gy, gz=sample.gz,
                mx=sample.mx, my=sample.my, mz=sample.mz,
                flags=sample.flags,
            )
        if not loop:
            return
        t_offset += samples[-1].t - base

