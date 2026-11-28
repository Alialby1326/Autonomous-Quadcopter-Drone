#!/usr/bin/env python3
"""Record the STM32's telemetry stream to a replayable log.

    python3 record_log.py --port /dev/ttyUSB0 --output flight.log
    python3 record_log.py --port /dev/ttyUSB0 --monitor     # just watch, save nothing
    python3 record_log.py --port /dev/ttyUSB0 --monitor 250 # at most one sample per 250 ms

--monitor prints lines as they arrive, including the firmware's "#" status
messages (boot, IMU errors, calibration), and writes no file. An optional value
in milliseconds throttles the sample lines to at most one per interval; status
messages and garbage lines always print, and the end-of-run statistics still
count every sample.

Recording writes the wire format verbatim, so `visualize_attitude.py --log` can replay
the recording with the device's own timing rather than a guess at it.

This is also the tool that captures data for magnetometer hard-iron
calibration: record while slowly rotating the board through every orientation,
and fit the offsets offline from the result.
"""

from __future__ import annotations

import argparse
import contextlib
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
    p.add_argument(
        "--monitor", nargs="?", type=float, const=0.0, default=None, metavar="MS",
        help="print lines instead of saving to --output; optional MS shows at most one sample per MS ms",
    )
    args = p.parse_args()
    monitor = args.monitor is not None
    if monitor and args.monitor < 0:
        p.error("--monitor interval must be >= 0 ms")

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
    interval_s = (args.monitor or 0.0) / 1000.0
    next_print = 0.0

    if monitor:
        every = f", one sample per {args.monitor:g} ms" if interval_s else ""
        print(f"monitoring {args.port} @ {args.baud}{every} (ctrl-c to stop)")
    else:
        print(f"recording {args.port} @ {args.baud} -> {args.output} (ctrl-c to stop)")

    try:
        # The port opens first, so a bad --port leaves no empty log behind.
        with serial.Serial(args.port, args.baud, timeout=1.0) as link, \
                (contextlib.nullcontext() if monitor else open(args.output, "w")) as out:
            link.reset_input_buffer()
            while True:
                raw = link.readline()
                if not raw:
                    continue

                text = raw.decode("ascii", errors="replace")
                sample = parse_line(text)

                if monitor:
                    # Status and garbage lines are rare and always worth seeing;
                    # only the sample stream is throttled.
                    now = time.monotonic()
                    if sample is None or now >= next_print:
                        print(text.rstrip("\r\n"), flush=True)
                        if sample is not None:
                            next_print = now + interval_s

                if sample is None:
                    # "#" lines are the firmware's status messages, not noise.
                    if not text.startswith("#"):
                        junk += 1
                    continue

                if out is not None:
                    out.write(format_line(sample) + "\n")
                count += 1
                last_t = sample.t
                if first_t is None:
                    first_t = sample.t

                for bit in FLAG_NAMES:
                    if sample.flags & bit:
                        flag_counts[bit] += 1

                if count % 100 == 0 and out is not None:
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

    if monitor:
        print(f"\n\nsaw {count} samples")
    else:
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
