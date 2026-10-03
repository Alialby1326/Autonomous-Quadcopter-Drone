#!/usr/bin/env python3
"""Live attitude from the IMU stream, for comparing estimators side by side.

    python3 visualize_attitude.py --port /dev/ttyUSB0          # live
    python3 visualize_attitude.py --log flight.log --loop      # replay a capture
    python3 visualize_attitude.py --log flight.log --estimators accel

Every estimator in `droneviz.attitude.ESTIMATORS` sees the same samples. The
3D view draws the first one solid and the rest as ghosts; the strips on the
right overlay their roll and pitch over time. The bottom strip is |a|/g -- when
it leaves the shaded band the accelerometer is measuring more than gravity, and
any accel-derived tilt is wrong by construction.

Angle signs follow the body frame (X forward, Y left, Z up): roll > 0 is a
right bank, pitch > 0 is nose down.

Keys: space pause, c clear (also resets estimators), q quit.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
from collections import deque

import numpy as np

import matplotlib

# TkAgg is the most reliable interactive backend under WSLg, but it needs
# python3-tk installed. Fall back rather than dying on import.
try:
    matplotlib.use("TkAgg")
except ImportError:  # pragma: no cover - depends on the host's Tk install
    pass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from droneviz.attitude import ESTIMATORS, Attitude, rotation_matrix
from droneviz.sources import iter_log, iter_serial
from droneviz.telemetry import FLAG_IMU_ERROR, describe_flags

G = 9.80665
# Categorical slots in fixed order; estimator N always gets color N.
SERIES_COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
INK_MUTED = "#8a8a85"
ACCEL_TRUST_BAND = 0.1  # |a|/g within 1 +- this is "probably just gravity"


# -- quadcopter model, body frame --------------------------------------------


def _quad_model(arm: float = 1.0, rotor: float = 0.32) -> tuple[np.ndarray, np.ndarray]:
    """Frame polyline and nose arrow, each an (N, 3) array with NaN breaks.

    X-configuration: arms on the diagonals, so "forward" is between the two
    front rotors -- the nose arrow makes that unambiguous.
    """
    nan = np.full((1, 3), np.nan)
    d = arm / math.sqrt(2)
    tips = np.array([[d, d, 0], [d, -d, 0], [-d, -d, 0], [-d, d, 0]])
    theta = np.linspace(0, 2 * math.pi, 25)
    ring = np.column_stack([rotor * np.cos(theta), rotor * np.sin(theta), np.zeros_like(theta)])

    parts = [np.array([tips[0], tips[2]]), nan, np.array([tips[1], tips[3]]), nan]
    for tip in tips:
        parts += [ring + tip, nan]
    parts += [np.array([[0, 0, 0], [0, 0, 0.35 * arm]])]  # body +Z, so upside-down is obvious
    frame = np.vstack(parts)

    nose = np.array([[0, 0, 0], [1.25 * arm, 0, 0], [np.nan] * 3,
                     [1.25 * arm, 0, 0], [1.05 * arm, 0.12 * arm, 0], [np.nan] * 3,
                     [1.25 * arm, 0, 0], [1.05 * arm, -0.12 * arm, 0]])
    return frame, nose


FRAME, NOSE = _quad_model()
GROUND_Z = -1.0


class Viewer:
    def __init__(self, samples, names: list[str], window_s: float):
        self.names = names
        self.estimators = [ESTIMATORS[n]() for n in names]
        self.colors = {n: SERIES_COLORS[list(ESTIMATORS).index(n) % len(SERIES_COLORS)] for n in names}
        self.window_s = window_s
        self.paused = False
        self.latest = None
        self.error = None
        self.sample_count = 0
        self.prev_t = None
        self.current = [Attitude(0.0, 0.0) for _ in names]

        # Trimmed by device time, so the window is the same length at any rate.
        self.t = deque()
        self.a_norm = deque()
        self.roll = [deque() for _ in names]
        self.pitch = [deque() for _ in names]

        # serial readline() blocks and log replay sleeps; neither belongs in
        # matplotlib's animation callback.
        self.inbox = queue.Queue(maxsize=10000)
        threading.Thread(target=self._read, args=(samples,), daemon=True).start()

        self._build_figure()

    def _read(self, samples):
        try:
            for sample in samples:
                self.inbox.put(sample)
        except BaseException as exc:  # SystemExit from a missing pyserial, SerialException, ...
            self.error = exc

    # -- figure construction ----------------------------------------------

    def _build_figure(self):
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("Attitude")
        gs = self.fig.add_gridspec(
            3, 2, width_ratios=(1.05, 1), height_ratios=(1, 1, 0.55),
            left=0.02, right=0.97, top=0.86, bottom=0.07, wspace=0.12, hspace=0.3,
        )
        self.ax3d = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_roll = self.fig.add_subplot(gs[0, 1])
        self.ax_pitch = self.fig.add_subplot(gs[1, 1], sharex=self.ax_roll)
        self.ax_norm = self.fig.add_subplot(gs[2, 1], sharex=self.ax_roll)

        self._build_3d()

        self.roll_lines = self._setup_strip(self.ax_roll, "Roll  (+ = right bank)", "deg")
        self.pitch_lines = self._setup_strip(self.ax_pitch, "Pitch  (+ = nose down)", "deg")
        self.ax_roll.legend(loc="upper left", fontsize=8, ncol=len(self.names), framealpha=0.7)
        for ax in (self.ax_roll, self.ax_pitch):
            ax.tick_params(labelbottom=False)

        ax = self.ax_norm
        ax.set_title("|a| / g  (shaded: accel tilt trustworthy)", fontsize=10, loc="left")
        ax.axhspan(1 - ACCEL_TRUST_BAND, 1 + ACCEL_TRUST_BAND, color=INK_MUTED, alpha=0.15, lw=0)
        ax.axhline(1.0, color=INK_MUTED, lw=0.8)
        ax.grid(alpha=0.25)
        ax.set_xlabel("device time (s)")
        (self.norm_line,) = ax.plot([], [], color="#444444", lw=1.2)

        self.hud = self.fig.text(0.015, 0.975, "", va="top", family="monospace", fontsize=9.5)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def _build_3d(self):
        ax = self.ax3d
        lim = 1.3
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(GROUND_Z, lim + GROUND_Z + 1.3)
        ax.set_box_aspect((1, 1, 0.9))
        # Camera behind and slightly right of the aircraft, so its left (+Y)
        # is on the screen's left and a right bank tips to the right.
        ax.view_init(elev=22, azim=-155)
        ax.set_xlabel("X fwd")
        ax.set_ylabel("Y left")
        ax.set_zlabel("Z up")
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((1, 1, 1, 0))
            axis.set_tick_params(labelsize=7, colors=INK_MUTED)

        # Level reference: a ring in the world XY plane through the origin.
        theta = np.linspace(0, 2 * math.pi, 73)
        ax.plot(1.15 * np.cos(theta), 1.15 * np.sin(theta), 0 * theta, color=INK_MUTED, lw=0.8, ls=":")
        # Ground grid for the shadow to land on.
        for v in np.linspace(-lim, lim, 7):
            ax.plot([v, v], [-lim, lim], [GROUND_Z] * 2, color=INK_MUTED, lw=0.5, alpha=0.4)
            ax.plot([-lim, lim], [v, v], [GROUND_Z] * 2, color=INK_MUTED, lw=0.5, alpha=0.4)

        (self.shadow,) = ax.plot([], [], [], color="#000000", lw=1.5, alpha=0.18)
        self.bodies = []
        # Reverse so the primary estimator is drawn last, on top.
        for i, name in reversed(list(enumerate(self.names))):
            primary = i == 0
            style = dict(color=self.colors[name], lw=2.2 if primary else 1.3,
                         alpha=1.0 if primary else 0.6, ls="-" if primary else "--")
            (frame,) = ax.plot([], [], [], **style)
            (nose,) = ax.plot([], [], [], **{**style, "lw": style["lw"] + 0.8, "ls": "-"})
            self.bodies.insert(0, (frame, nose))

    def _setup_strip(self, ax, title, unit):
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_ylabel(unit)
        ax.grid(alpha=0.25)
        ax.axhline(0.0, color=INK_MUTED, lw=0.8)
        return [ax.plot([], [], color=self.colors[n], lw=1.6, label=n)[0] for n in self.names]

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
            self.reset()

    def reset(self):
        for buf in (self.t, self.a_norm, *self.roll, *self.pitch):
            buf.clear()
        for est in self.estimators:
            est.reset()
        self.prev_t = None
        self.sample_count = 0

    # -- per-frame work ----------------------------------------------------

    def consume(self):
        while True:
            try:
                s = self.inbox.get_nowait()
            except queue.Empty:
                break
            if self.paused:
                continue  # discard, so resuming shows live data, not a backlog

            if self.prev_t is not None and s.t < self.prev_t:
                # Board reset or the log looped: time went backwards, and
                # neither the plot nor any estimator state survives that.
                self.reset()
            dt = 0.0 if self.prev_t is None else s.t - self.prev_t
            self.prev_t = s.t
            self.latest = s
            self.sample_count += 1

            if s.flags & FLAG_IMU_ERROR:
                continue  # stale accel/gyro; don't feed it to the estimators

            self.current = [est.update(s, dt) for est in self.estimators]
            self.t.append(s.t)
            self.a_norm.append(float(np.linalg.norm(s.accel)) / G)
            for i, att in enumerate(self.current):
                self.roll[i].append(math.degrees(att.roll))
                self.pitch[i].append(math.degrees(att.pitch))

        if self.t:
            cutoff = self.t[-1] - self.window_s
            while self.t[0] < cutoff:
                for buf in (self.t, self.a_norm, *self.roll, *self.pitch):
                    buf.popleft()

    def update_3d(self):
        for (frame, nose), att in zip(self.bodies, self.current):
            r = rotation_matrix(att)
            f = FRAME @ r.T
            n = NOSE @ r.T
            frame.set_data_3d(f[:, 0], f[:, 1], f[:, 2])
            nose.set_data_3d(n[:, 0], n[:, 1], n[:, 2])
            if frame is self.bodies[0][0]:
                self.shadow.set_data_3d(f[:, 0], f[:, 1], np.where(np.isnan(f[:, 2]), np.nan, GROUND_Z))

    def update_strips(self):
        if len(self.t) < 2:
            return
        t = np.array(self.t)
        for lines, ax, series in ((self.roll_lines, self.ax_roll, self.roll),
                                  (self.pitch_lines, self.ax_pitch, self.pitch)):
            lo, hi = -5.0, 5.0  # never zoom in so far that sensor noise looks like motion
            for line, buf in zip(lines, series):
                y = np.array(buf)
                line.set_data(t, y)
                lo, hi = min(lo, float(y.min())), max(hi, float(y.max()))
            pad = 0.1 * (hi - lo)
            ax.set_ylim(lo - pad, hi + pad)

        norm = np.array(self.a_norm)
        self.norm_line.set_data(t, norm)
        lo = min(1 - 2 * ACCEL_TRUST_BAND, float(norm.min()))
        hi = max(1 + 2 * ACCEL_TRUST_BAND, float(norm.max()))
        self.ax_norm.set_ylim(lo - 0.05, hi + 0.05)
        self.ax_norm.set_xlim(t[-1] - self.window_s, t[-1])

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
        width = max(len(n) for n in self.names)
        text = [f"{n:<{width}}  roll {math.degrees(a.roll):7.2f}°  pitch {math.degrees(a.pitch):7.2f}°"
                + (f"  yaw {math.degrees(a.yaw):7.2f}°" if a.yaw is not None else "")
                for n, a in zip(self.names, self.current)]
        text.append(f"{rate:6.1f} Hz on-device   n={self.sample_count}   flags: {describe_flags(s.flags)}"
                    + ("   PAUSED (space to resume)" if self.paused else ""))
        self.hud.set_text("\n".join(text))

    def frame(self, _idx):
        self.consume()
        self.update_3d()
        self.update_strips()
        self.update_hud()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port the board is on, e.g. /dev/ttyUSB0")
    source.add_argument("--log", help="recording from record_log.py to replay")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--loop", action="store_true", help="restart the log when it ends")
    p.add_argument("--speed", type=float, default=1.0, help="log replay speed multiplier")
    p.add_argument("--window", type=float, default=10.0, help="seconds of history shown")
    p.add_argument("--interval", type=int, default=40, help="render period, ms")
    p.add_argument("--estimators", nargs="+", choices=list(ESTIMATORS), default=list(ESTIMATORS),
                   metavar="NAME", help=f"which to run, first is drawn solid (default: all of {list(ESTIMATORS)})")
    args = p.parse_args()

    if args.port:
        samples = iter_serial(args.port, args.baud)
    else:
        samples = iter_log(args.log, loop=args.loop, speed=args.speed)

    viewer = Viewer(samples, args.estimators, args.window)
    print("keys: space pause | c clear | q quit")

    # Held in a local so the animation is not garbage collected mid-run.
    _anim = FuncAnimation(
        viewer.fig, viewer.frame, interval=args.interval, blit=False, cache_frame_data=False
    )
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
