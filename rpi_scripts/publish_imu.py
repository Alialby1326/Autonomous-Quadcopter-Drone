#!/usr/bin/env python3
"""Publish IMU samples onto ZeroMQ from the simulator, a log, or the board.

    python3 publish_imu.py --source sim              # synthetic + ground truth
    python3 publish_imu.py --source log --loop       # replay a recording
    python3 publish_imu.py --source serial --port /dev/ttyUSB0

Everything downstream sees the same stream regardless of which one you pick,
so you can develop the visualizer and the filter against `sim` -- where the
true attitude is known -- and then point the identical pipeline at hardware.
"""

from __future__ import annotations

import argparse
import sys
import time

from droneviz.sources import iter_log, iter_serial, iter_sim
from droneviz.telemetry import DEFAULT_ENDPOINT, Publisher, describe_flags


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=["sim", "log", "serial"], default="sim")
    p.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help="ZeroMQ bind address")
    p.add_argument("--loop", action="store_true", help="restart when the data runs out")
    p.add_argument("--quiet", action="store_true", help="suppress the status line")

    sim = p.add_argument_group("sim")
    sim.add_argument("--rate", type=float, default=200.0, help="sample rate, Hz")
    sim.add_argument("--duration", type=float, default=32.0, help="seconds per pass")
    sim.add_argument("--seed", type=int, default=0)

    log = p.add_argument_group("log")
    log.add_argument("--log", default="sim_imu.log", help="path to replay")
    log.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")

    serial = p.add_argument_group("serial")
    serial.add_argument("--port", default="/dev/ttyUSB0")
    serial.add_argument("--baud", type=int, default=460800)

    return p


def make_stream(args):
    if args.source == "sim":
        from droneviz.simulate import SensorModel

        return iter_sim(
            duration=args.duration,
            rate_hz=args.rate,
            loop=args.loop,
            model=SensorModel(seed=args.seed),
        )
    if args.source == "log":
        return iter_log(args.log, loop=args.loop, speed=args.speed)
    return iter_serial(args.port, args.baud)


def main() -> int:
    args = build_parser().parse_args()

    publisher = Publisher(args.endpoint)
    print(f"publishing {args.source} samples on {args.endpoint} (ctrl-c to stop)")

    # PUB sockets drop messages sent before a subscriber has finished
    # connecting. A short pause here is the difference between the visualizer
    # showing data immediately and it appearing to hang on startup.
    time.sleep(0.3)

    count = 0
    last_report = time.monotonic()
    last_count = 0

    try:
        for sample in make_stream(args):
            publisher.send(sample)
            count += 1

            now = time.monotonic()
            if not args.quiet and now - last_report >= 1.0:
                rate = (count - last_count) / (now - last_report)
                print(
                    f"\r{count:8d} samples  {rate:6.1f} Hz  "
                    f"t={sample.t:8.2f}s  {describe_flags(sample.flags):<24}",
                    end="",
                    flush=True,
                )
                last_report, last_count = now, count
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()

    print(f"\nstopped after {count} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
