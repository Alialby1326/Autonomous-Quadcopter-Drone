#!/usr/bin/env python3
"""Record the STM32's telemetry stream to a replayable log.

    python3 record_log.py --port /dev/ttyUSB0 --output flight.log

Writes the wire format verbatim, so `publish_imu.py --source log` can replay
the recording with the device's own timing rather than a guess at it.

This is also the tool that captures data for magnetometer hard-iron
calibration: record while slowly rotating the board through every orientation,
and fit the offsets offline from the result.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

from droneviz.telemetry import FLAG_NAMES, format_line, parse_line


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--output", default="imu_capture.log")
    p.add_argument("--seconds", type=float, default=0.0, help="stop after N seconds (0 = until ctrl-c)")
    args = p.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        print(
            "pyserial is required.\n    pip install -r rpi_scripts/requirements.txt",
            file=sys.stderr,
        )
        return 1

    flag_counts = Counter()
    count = 0
    junk = 0
    first_t = None
    last_t = None
    start = time.monotonic()

    print(f"recording {args.port} @ {args.baud} -> {args.output} (ctrl-c to stop)")

    try:
        with serial.Serial(args.port, args.baud, timeout=1.0) as link, \
                open(args.output, "w") as out:
            link.reset_input_buffer()
            while True:
                raw = link.readline()
                if not raw:
                    continue

                sample = parse_line(raw.decode("ascii", errors="replace"))
                if sample is None:
                    junk += 1
                    continue

                out.write(format_line(sample) + "\n")
                count += 1
                last_t = sample.t
                if first_t is None:
                    first_t = sample.t

                for bit in FLAG_NAMES:
                    if sample.flags & bit:
                        flag_counts[bit] += 1

                if count % 100 == 0:
                    out.flush()
                    elapsed = time.monotonic() - start
                    print(
                        f"\r{count:8d} samples  {count / max(elapsed, 1e-9):6.1f} Hz  "
                        f"{junk:5d} unparsed",
                        end="",
                        flush=True,
                    )

                if args.seconds and time.monotonic() - start >= args.seconds:
                    break
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"\nserial error: {exc}", file=sys.stderr)
        return 1

    print(f"\n\nwrote {count} samples to {args.output}")
    if count and first_t is not None and last_t is not None and last_t > first_t:
        span = last_t - first_t
        # Rate computed from the device's own timestamps, not the host clock --
        # this is the number that tells you whether the firmware loop is
        # actually keeping up.
        print(f"device timespan {span:.2f}s -> {(count - 1) / span:.1f} Hz on-device")
    if junk:
        print(f"{junk} unparsed lines (expect one or two at connect; many means a baud mismatch)")
    if flag_counts:
        print("flags seen:")
        for bit, n in flag_counts.most_common():
            print(f"  {FLAG_NAMES[bit]:<14} {n:8d}  ({100.0 * n / max(count, 1):.1f}%)")
    else:
        print("no health flags raised")
    return 0


if __name__ == "__main__":
    sys.exit(main())
