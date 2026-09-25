#!/usr/bin/env python3
"""Live plots of the raw IMU stream, straight from the board or a recording.

    python3 visualize_imu.py --port /dev/ttyUSB0          # live
    python3 visualize_imu.py --log flight.log --loop      # replay a capture

Shows accelerometer, gyroscope and magnetometer as they arrive -- no filtering,
no attitude estimate. The point is to see what the hardware is actually doing:
whether the accelerometer reads ~9.81 on +Z at rest, whether the gyro sits near
zero after calibration, whether the magnetometer produces anything at all.

Keys: space pause, c clear, q quit.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
from collections import Counter, deque

import numpy as np

import matplotlib

# TkAgg is the most reliable interactive backend under WSLg, but it needs
# python3-tk installed. Fall back rather than dying on import, so an
# environment without it still gets whatever backend it does have.
try:
    matplotlib.use("TkAgg")
except ImportError:  # pragma: no cover - depends on the host's Tk install
    pass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from droneviz.sources import iter_log, iter_serial
from droneviz.telemetry import FLAG_NAMES, describe_flags

G = 9.80665
COLOR_AXIS = ("#e8663d", "#3d8ce8", "#5cb85c")  # x, y, z


class Viewer:
    def __init__(self, samples, window_s: float):
        self.window_s = window_s
        self.paused = False
        self.latest = None
        self.error = None
        self.sample_count = 0
        self.flag_counts = Counter()

        # Unbounded here; trim() cuts by device time rather than sample count,
        # so the window stays the same length whatever the loop rate is.
        self.t = deque()
        self.accel = deque()
        self.gyro = deque()
        # The magnetometer updates slower than the loop, and stale samples
        # repeat the last reading. Only fresh ones are plotted.
        self.mag_t = deque()
        self.mag = deque()

        # Reading happens on its own thread: a serial readline() blocks, and
        # log replay sleeps to reproduce the device's timing. Neither can run
        # inside matplotlib's animation callback.
        self.inbox = queue.Queue(maxsize=10000)
        self.reader = threading.Thread(target=self._read, args=(samples,), daemon=True)
        self.reader.start()

        self._build_figure()

    def _read(self, samples):
        try:
            for sample in samples:
                self.inbox.put(sample)
        except BaseException as exc:  # SystemExit from a missing pyserial, SerialException, ...
            self.error = exc

    # -- figure construction ----------------------------------------------

    def _build_figure(self):
        self.fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
        self.fig.canvas.manager.set_window_title("IMU stream")
        self.fig.subplots_adjust(top=0.84, hspace=0.25)
        self.ax_accel, self.ax_gyro, self.ax_mag = axes

        self.accel_lines = self._setup_axes(self.ax_accel, "Accelerometer", "m/s²")
        self.gyro_lines = self._setup_axes(self.ax_gyro, "Gyroscope", "deg/s")
        self.mag_lines = self._setup_axes(self.ax_mag, "Magnetometer (fresh samples only)", "µT")
        self.ax_mag.set_xlabel("device time (s)")

        self.hud = self.fig.text(0.015, 0.975, "", va="top", family="monospace", fontsize=9.5)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    @staticmethod
    def _setup_axes(ax, title, unit):
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_ylabel(unit)
        ax.grid(alpha=0.25)
        ax.axhline(0.0, color="#999999", lw=0.8)
        lines = [
            ax.plot([], [], color=COLOR_AXIS[i], lw=1.1, label=label)[0]
            for i, label in enumerate("xyz")
        ]
        ax.legend(loc="upper left", fontsize=8, ncol=3, framealpha=0.6)
        return lines

    # -- input -------------------------------------------------------------

    def on_key(self, event):
        if event.key is None:
            return
        key = event.key.lower()
        if key == "q":
            plt.close(self.fig)
        elif key == " ":
            self.paused = not self.paused
        elif key == "c":
            for buf in (self.t, self.accel, self.gyro, self.mag_t, self.mag):
                buf.clear()
            self.flag_counts.clear()
            self.sample_count = 0

    # -- per-frame work ----------------------------------------------------

    def consume(self):
        while True:
            try:
                sample = self.inbox.get_nowait()
            except queue.Empty:
                break
            if self.paused:
                continue  # discard, so resuming shows live data, not a backlog

            if self.t and sample.t < self.t[-1]:
                # Board reset or the log looped: time went backwards, and a
                # plot spanning both would be meaningless.
                for buf in (self.t, self.accel, self.gyro, self.mag_t, self.mag):
                    buf.clear()

            self.t.append(sample.t)
            self.accel.append(sample.accel)
            self.gyro.append(sample.gyro_dps)
            if sample.mag_valid:
                self.mag_t.append(sample.t)
                self.mag.append(sample.mag)

            for bit in FLAG_NAMES:
                if sample.flags & bit:
                    self.flag_counts[bit] += 1
            self.sample_count += 1
            self.latest = sample

        self.trim()

    def trim(self):
        if not self.t:
            return
        cutoff = self.t[-1] - self.window_s
        while self.t and self.t[0] < cutoff:
            self.t.popleft()
            self.accel.popleft()
            self.gyro.popleft()
        while self.mag_t and self.mag_t[0] < cutoff:
            self.mag_t.popleft()
            self.mag.popleft()

    def update_plots(self):
        if len(self.t) < 2:
            return
        t = np.array(self.t)
        for lines, ax, times, values in (
            (self.accel_lines, self.ax_accel, t, self.accel),
            (self.gyro_lines, self.ax_gyro, t, self.gyro),
            (self.mag_lines, self.ax_mag, np.array(self.mag_t), self.mag),
        ):
            data = np.array(values) if values else np.empty((0, 3))
            for i in range(3):
                lines[i].set_data(times, data[:, i] if len(data) else [])
            if len(data):
                lo, hi = float(data.min()), float(data.max())
                pad = max(0.5, 0.1 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
        self.ax_mag.set_xlim(t[-1] - self.window_s, t[-1])

    def update_hud(self):
        if self.error is not None:
            self.hud.set_text(f"source stopped: {self.error}")
            return
        if self.latest is None:
            self.hud.set_text("waiting for samples...")
            return

        s = self.latest
        span = self.t[-1] - self.t[0] if len(self.t) > 1 else 0.0
        rate = (len(self.t) - 1) / span if span > 0 else 0.0
        a_norm = float(np.linalg.norm(s.accel))
        text = [
            f"accel {s.ax:8.3f} {s.ay:8.3f} {s.az:8.3f} m/s²   |a| {a_norm:6.3f} ({a_norm / G:5.3f} g)",
            f"gyro  {s.gx:8.3f} {s.gy:8.3f} {s.gz:8.3f} deg/s",
            f"mag   {s.mx:8.3f} {s.my:8.3f} {s.mz:8.3f} µT"
            f"      {rate:6.1f} Hz on-device   n={self.sample_count}",
            f"flags now: {describe_flags(s.flags):<24} seen: "
            + (", ".join(f"{FLAG_NAMES[b]} {n}" for b, n in self.flag_counts.most_common()) or "none"),
        ]
        if self.paused:
            text.append("PAUSED (space to resume)")
        self.hud.set_text("\n".join(text))

    def frame(self, _idx):
        self.consume()
        self.update_plots()
        self.update_hud()
        return self.accel_lines + self.gyro_lines + self.mag_lines


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port the board is on, e.g. /dev/ttyUSB0")
    source.add_argument("--log", help="recording from record_log.py to replay")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--loop", action="store_true", help="restart the log when it ends")
    p.add_argument("--speed", type=float, default=1.0, help="log replay speed multiplier")
    p.add_argument("--window", type=float, default=10.0, help="seconds of history shown")
    p.add_argument("--interval", type=int, default=50, help="render period, ms")
    args = p.parse_args()

    if args.port:
        samples = iter_serial(args.port, args.baud)
    else:
        samples = iter_log(args.log, loop=args.loop, speed=args.speed)

    viewer = Viewer(samples, args.window)
    print("keys: space pause | c clear | q quit")

    # Held in a local so the animation is not garbage collected mid-run.
    _anim = FuncAnimation(
        viewer.fig, viewer.frame, interval=args.interval, blit=False, cache_frame_data=False
    )
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
