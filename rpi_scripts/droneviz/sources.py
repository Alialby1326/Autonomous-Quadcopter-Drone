"""Sample sources: live serial or recorded log.

Both yield the same `ImuSample` stream, so the visualizer is written once and
works against either -- watching the board live and replaying a capture of it
differ by one command-line flag.
"""

from __future__ import annotations

import sys
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

    Replayed time starts at 0 and only moves forward; see `_continuous_times`.
    """
    with open(path, "r") as handle:
        text = handle.read()

    samples = [s for s in (parse_line(line) for line in text.splitlines()) if s]
    if not samples:
        raise SystemExit(f"no IMU samples found in {path}")

    times, splices = _continuous_times(samples)
    if splices:
        print(f"{path}: spliced {splices} timestamp discontinuities "
              "(board reset, counter wrap or corrupt line)", file=sys.stderr)

    t_offset = 0.0
    while True:
        wall_start = time.monotonic()
        for sample, elapsed in zip(samples, times):
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
        t_offset += times[-1]


MAX_GAP_S = 1.0  # longer than any plausible loop period; anything beyond is a break


def _continuous_times(samples: list[ImuSample]) -> tuple[list[float], int]:
    """Replay times for `samples`, starting at 0, with discontinuities spliced out.

    A log can contain a board reset (time jumps back), a TIM5 wrap every
    ~71.6 min (also back), or a line garbled on the wire that still parses
    (one wild stamp, then a jump back). Pacing against raw timestamps sleeps
    for hours at a forward jump and plays everything after a backward one
    instantly. Each such step is replaced with the log's typical step instead,
    so playback stays continuous and the sample data is kept.
    """
    steps = [b.t - a.t for a, b in zip(samples, samples[1:])]
    good = sorted(d for d in steps if 0 <= d <= MAX_GAP_S)
    nominal = good[len(good) // 2] if good else 0.01

    times = [0.0]
    splices = 0
    for step in steps:
        if not 0 <= step <= MAX_GAP_S:
            step = nominal
            splices += 1
        times.append(times[-1] + step)
    return times, splices

