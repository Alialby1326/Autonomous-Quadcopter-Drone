#!/usr/bin/env python3
"""Live 3D attitude view driven by the ZeroMQ IMU stream.

    python3 publish_imu.py --source sim      # in one terminal
    python3 visualize_attitude.py            # in another

Shows the estimated attitude as a quad body, the three angles over time, and
-- when the source is the simulator -- the true attitude as a ghost outline
plus the estimation error. That error trace is the point of the whole tool: it
turns "the cube moves, looks about right" into a number you can watch while
tuning the filter.

Keys: space pause, r reset the filter, g toggle the ground-truth ghost, q quit.
"""

from __future__ import annotations

import argparse
import sys
from collections import deque

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
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from droneviz.attitude import ComplementaryFilter, angle_diff, euler_to_quat, heading_deg, quat_to_matrix
from droneviz.telemetry import DEFAULT_ENDPOINT, Subscriber, describe_flags

HISTORY = 1500  # samples retained in the time-series plots

ARM = 0.9        # arm half-length, body units
ROTOR_R = 0.26
BODY = 0.16

COLOR_FRONT = "#e8663d"
COLOR_REAR = "#3d8ce8"
COLOR_TRUTH = "#888888"
COLOR_ANGLE = ("#e8663d", "#3d8ce8", "#5cb85c")


def quad_geometry():
    """Line segments for a quad in X configuration, in the body frame.

    Front arms are drawn in a different colour from the rear ones, which is
    what makes yaw readable at a glance -- a symmetric shape gives you no way
    to tell 0 degrees from 180.
    """
    c = ARM / np.sqrt(2.0)
    arms = [
        (np.array([0.0, 0.0, 0.0]), np.array([c, c, 0.0]), COLOR_FRONT),    # front-left
        (np.array([0.0, 0.0, 0.0]), np.array([c, -c, 0.0]), COLOR_FRONT),   # front-right
        (np.array([0.0, 0.0, 0.0]), np.array([-c, c, 0.0]), COLOR_REAR),    # rear-left
        (np.array([0.0, 0.0, 0.0]), np.array([-c, -c, 0.0]), COLOR_REAR),   # rear-right
    ]

    segments = []
    for start, end, color in arms:
        segments.append((np.array([start, end]), color, 2.5))

    # Rotor discs at the arm tips.
    theta = np.linspace(0, 2 * np.pi, 25)
    circle = np.stack([ROTOR_R * np.cos(theta), ROTOR_R * np.sin(theta), np.zeros_like(theta)], axis=1)
    for _, end, color in arms:
        segments.append((circle + end, color, 1.4))

    # Nose marker, so "forward" is unambiguous.
    segments.append((np.array([[0.0, 0.0, 0.0], [ARM * 1.15, 0.0, 0.0]]), "#222222", 2.0))
    # Small body box outline.
    box = np.array([
        [BODY, BODY, 0.0], [BODY, -BODY, 0.0],
        [-BODY, -BODY, 0.0], [-BODY, BODY, 0.0], [BODY, BODY, 0.0],
    ])
    segments.append((box, "#222222", 1.5))

    return segments


class Viewer:
    def __init__(self, endpoint: str, kp: float, ki: float, mag_weight: float):
        self.subscriber = Subscriber(endpoint)
        self.filter = ComplementaryFilter(kp=kp, ki=ki, mag_weight=mag_weight)

        self.geometry = quad_geometry()
        self.paused = False
        self.show_truth = True
        self.have_truth = False
        self.latest = None
        self.prev_t = None
        self.sample_count = 0
        self.rate_estimate = 0.0

        self.times = deque(maxlen=HISTORY)
        self.est = deque(maxlen=HISTORY)
        self.truth = deque(maxlen=HISTORY)

        self._build_figure()

    # -- figure construction ----------------------------------------------

    def _build_figure(self):
        self.fig = plt.figure(figsize=(13, 7))
        self.fig.canvas.manager.set_window_title("Quadcopter attitude")
        # `top` leaves room for the HUD text block above the axes; without it
        # the 3D subplot's title lands underneath it.
        grid = self.fig.add_gridspec(
            2, 2, width_ratios=[1.35, 1], hspace=0.35, wspace=0.18, top=0.86
        )

        self.ax3d = self.fig.add_subplot(grid[:, 0], projection="3d")
        self.ax_angles = self.fig.add_subplot(grid[0, 1])
        self.ax_error = self.fig.add_subplot(grid[1, 1])

        self._setup_3d()
        self._setup_timeseries()

        self.hud = self.fig.text(
            0.015, 0.965, "", va="top", family="monospace", fontsize=9.5
        )
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def _setup_3d(self):
        ax = self.ax3d
        ax.set_box_aspect([1, 1, 0.75])
        lim = 1.15
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_xlabel("N")
        ax.set_ylabel("W")
        ax.set_zlabel("Up")
        ax.view_init(elev=22, azim=-135)

        # World reference axes, drawn once and left alone.
        for vec, label, color in (
            ([1, 0, 0], "N", "#c0392b"),
            ([0, 1, 0], "W", "#27ae60"),
            ([0, 0, 1], "Up", "#2980b9"),
        ):
            v = np.array(vec) * 1.05
            ax.plot([0, v[0]], [0, v[1]], [0, v[2]], color=color, lw=1.0, alpha=0.35)
            ax.text(v[0], v[1], v[2], label, color=color, fontsize=8, alpha=0.7)

        self.lines = [
            ax.plot([], [], [], color=color, lw=lw)[0]
            for _, color, lw in self.geometry
        ]
        self.truth_lines = [
            ax.plot([], [], [], color=COLOR_TRUTH, lw=1.0, ls="--", alpha=0.65)[0]
            for _ in self.geometry
        ]

    def _setup_timeseries(self):
        labels = ("roll", "pitch", "yaw")

        self.ax_angles.set_title("Attitude", fontsize=10)
        self.ax_angles.set_ylabel("degrees")
        self.ax_angles.set_ylim(-190, 190)
        self.ax_angles.grid(alpha=0.25)
        self.angle_lines = [
            self.ax_angles.plot([], [], color=COLOR_ANGLE[i], lw=1.3, label=labels[i])[0]
            for i in range(3)
        ]
        self.truth_angle_lines = [
            self.ax_angles.plot([], [], color=COLOR_ANGLE[i], lw=1.0, ls=":", alpha=0.55)[0]
            for i in range(3)
        ]
        self.ax_angles.legend(loc="upper left", fontsize=8, ncol=3, framealpha=0.6)

        self.ax_error.set_title("Estimation error (simulator only)", fontsize=10)
        self.ax_error.set_ylabel("degrees")
        self.ax_error.set_xlabel("time (s)")
        self.ax_error.grid(alpha=0.25)
        self.ax_error.axhline(0.0, color="#999999", lw=0.8)
        self.error_lines = [
            self.ax_error.plot([], [], color=COLOR_ANGLE[i], lw=1.2, label=labels[i])[0]
            for i in range(3)
        ]
        self.ax_error.legend(loc="upper left", fontsize=8, ncol=3, framealpha=0.6)

    # -- input -------------------------------------------------------------

    def on_key(self, event):
        if event.key is None:
            return
        key = event.key.lower()
        if key == "q":
            plt.close(self.fig)
        elif key == " ":
            self.paused = not self.paused
        elif key == "r":
            self.filter.reset()
            self.times.clear()
            self.est.clear()
            self.truth.clear()
        elif key == "g":
            self.show_truth = not self.show_truth

    # -- per-frame work ----------------------------------------------------

    def consume(self):
        """Run every queued sample through the filter.

        The stream arrives faster than the display refreshes, so each frame
        has a backlog waiting. It all has to go through the filter -- skipping
        samples to "catch up" would corrupt the integration, which is the one
        thing an attitude estimate cannot survive.
        """
        samples = self.subscriber.drain(max_samples=512)
        if not samples:
            return

        for sample in samples:
            if self.prev_t is None or sample.t < self.prev_t:
                # First sample, or the publisher restarted and time went
                # backwards. Either way there is no valid dt to integrate.
                self.prev_t = sample.t
                continue

            dt = sample.t - self.prev_t
            self.prev_t = sample.t
            if dt <= 0.0:
                continue

            angles = self.filter.update(
                sample.gyro, sample.accel, sample.mag if sample.mag_valid else None, dt
            )

            self.times.append(sample.t)
            self.est.append(angles.copy())
            if sample.truth is not None:
                self.have_truth = True
                self.truth.append(np.array(sample.truth))
            else:
                self.truth.append(np.full(3, np.nan))

            self.sample_count += 1

        self.latest = samples[-1]
        span = self.times[-1] - self.times[0] if len(self.times) > 1 else 0.0
        if span > 0:
            self.rate_estimate = (len(self.times) - 1) / span

    def draw_body(self, lines, euler, visible=True):
        R = quat_to_matrix(euler_to_quat(*euler))
        for line, (points, _, _) in zip(lines, self.geometry):
            if not visible:
                line.set_data([], [])
                line.set_3d_properties([])
                continue
            rotated = points @ R.T
            line.set_data(rotated[:, 0], rotated[:, 1])
            line.set_3d_properties(rotated[:, 2])

    def update_hud(self):
        if self.latest is None:
            self.hud.set_text("waiting for samples on the IMU topic...")
            return

        roll, pitch, yaw = np.degrees(self.filter.euler)
        bias = np.degrees(self.filter.bias)
        text = [
            f"roll {roll:7.2f}   pitch {pitch:7.2f}   yaw {yaw:7.2f}   "
            f"heading {heading_deg(self.filter.yaw):6.1f}",
            f"gyro bias  {bias[0]:6.2f} {bias[1]:6.2f} {bias[2]:6.2f} deg/s"
            f"    stream {self.rate_estimate:6.1f} Hz   n={self.sample_count}",
            f"flags {describe_flags(self.latest.flags):<28}"
            f"accel {'REJECTED' if self.filter.accel_rejected else 'ok':<10}"
            f"mag {'used' if self.filter.mag_used else 'skipped'}",
        ]

        if self.have_truth and len(self.truth):
            err = np.array([
                angle_diff(self.filter.euler[i], self.truth[-1][i]) for i in range(3)
            ])
            if not np.isnan(err).any():
                e = np.degrees(err)
                text.append(f"error      {e[0]:6.2f} {e[1]:6.2f} {e[2]:6.2f} deg")
        if self.paused:
            text.append("PAUSED (space to resume)")

        self.hud.set_text("\n".join(text))

    def update_timeseries(self):
        if len(self.times) < 2:
            return

        t = np.array(self.times)
        est = np.degrees(np.array(self.est))
        truth = np.degrees(np.array(self.truth))

        for i in range(3):
            self.angle_lines[i].set_data(t, est[:, i])
            if self.have_truth and self.show_truth:
                self.truth_angle_lines[i].set_data(t, truth[:, i])
            else:
                self.truth_angle_lines[i].set_data([], [])

        self.ax_angles.set_xlim(t[0], max(t[-1], t[0] + 1.0))

        if self.have_truth:
            err = np.degrees(np.array([
                [angle_diff(e[i], v[i]) for i in range(3)]
                for e, v in zip(self.est, self.truth)
            ]))
            for i in range(3):
                self.error_lines[i].set_data(t, err[:, i])
            self.ax_error.set_xlim(t[0], max(t[-1], t[0] + 1.0))
            finite = err[np.isfinite(err)]
            bound = max(2.0, float(np.abs(finite).max()) * 1.25) if finite.size else 2.0
            self.ax_error.set_ylim(-bound, bound)
        else:
            self.ax_error.set_xlim(t[0], max(t[-1], t[0] + 1.0))

    def frame(self, _idx):
        if not self.paused:
            self.consume()
        else:
            self.subscriber.drain(max_samples=512)  # discard, don't queue up

        self.draw_body(self.lines, self.filter.euler)
        if self.have_truth and self.show_truth and len(self.truth) and not np.isnan(self.truth[-1]).any():
            self.draw_body(self.truth_lines, self.truth[-1])
        else:
            self.draw_body(self.truth_lines, np.zeros(3), visible=False)

        self.update_timeseries()
        self.update_hud()
        return self.lines + self.truth_lines


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    p.add_argument("--kp", type=float, default=1.0, help="filter proportional gain")
    p.add_argument("--ki", type=float, default=0.05, help="filter integral gain (gyro bias)")
    p.add_argument("--mag-weight", type=float, default=0.5,
                   help="magnetometer trust; 0 disables it and leaves yaw unobservable")
    p.add_argument("--interval", type=int, default=33, help="render period, ms")
    args = p.parse_args()

    viewer = Viewer(args.endpoint, args.kp, args.ki, args.mag_weight)
    print("keys: space pause | r reset filter | g toggle ground truth | q quit")

    # Held in a local so the animation is not garbage collected mid-run.
    _anim = FuncAnimation(
        viewer.fig, viewer.frame, interval=args.interval, blit=False, cache_frame_data=False
    )
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
