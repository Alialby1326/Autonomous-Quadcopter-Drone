# visualize_attitude.py
# Real-time 3D visualization of roll (aroll) and pitch (apitch) from accelerometer-only attitude
# Assumes serial output lines like:
#   ACCELEROMETER: ax: <g>, ay: <g>, az: <g>
# and (optionally) GYROSCOPE/MAGNETOMETER lines which are ignored here.

import serial
import numpy as np
import sys
import time
import math
from collections import deque

import matplotlib
matplotlib.use("TkAgg")  # 'Qt5Agg' also fine; pick one that works on your system
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed by mpl)

# --------------------
# Serial configuration
# --------------------
PORT = "COM4"         # change if needed
BAUD = 115200
TIMEOUT = 0.1         # seconds

# -----------------------------------
# Smoothing (helps with accel jitter)
# -----------------------------------
EMA_ALPHA = 0.15      # 0..1, higher = less smoothing
# optional: clamp angles to reasonable range to avoid spikes
ROLL_MAX_RAD  = math.radians(85)
PITCH_MAX_RAD = math.radians(85)

def ema_update(prev, new, alpha=EMA_ALPHA):
    if prev is None:
        return new
    return (1 - alpha) * prev + alpha * new

def parse_accel_line(s):
    """
    Parse a line like: "ACCELEROMETER: ax: 0.01, ay: -0.02, az: 0.99"
    Returns ax, ay, az as floats in g's.
    """
    try:
        # tolerate extra spaces/CRLF
        s = s.strip().replace("\\r", " ").replace("\\n", " ")
        # Strip label and split
        parts = s.replace("ACCELEROMETER:", "").split(",")
        ax = float(parts[0].split(":")[1].strip())
        ay = float(parts[1].split(":")[1].strip())
        az = float(parts[2].split(":")[1].strip())
        return ax, ay, az
    except Exception:
        return None

def accel_to_pitch_roll(ax, ay, az):
    """
    Compute pitch (rotation about Y) and roll (rotation about X) from accelerometer in g's.
    Uses common tilt formulas (gravity reference). Returns radians.
    apitch = atan( -ax / sqrt(ay^2 + az^2) )
    aroll  = atan(  ay / sqrt(ax^2 + az^2) )
    """
    # tiny epsilon to avoid div/0
    eps = 1e-9
    denom_pitch = math.sqrt(ay * ay + az * az) + eps
    denom_roll  = math.sqrt(ax * ax + az * az) + eps

    apitch = math.atan(-ax / denom_pitch)
    aroll  = math.atan( ay / denom_roll)

    # clamp to keep visuals sane on transient spikes
    apitch = max(-PITCH_MAX_RAD, min(PITCH_MAX_RAD, apitch))
    aroll  = max(-ROLL_MAX_RAD,  min(ROLL_MAX_RAD,  aroll))
    return apitch, aroll

# -------------------------
# 3D Object: a wireframe cube
# -------------------------
def make_cube(size=1.0):
    s = size / 2.0
    # 8 vertices of a cube centered at origin
    V = np.array([
        [-s, -s, -s],
        [ s, -s, -s],
        [ s,  s, -s],
        [-s,  s, -s],
        [-s, -s,  s],
        [ s, -s,  s],
        [ s,  s,  s],
        [-s,  s,  s],
    ])
    # edges (pairs of vertex indices)
    edges = [
        (0,1), (1,2), (2,3), (3,0),   # bottom square
        (4,5), (5,6), (6,7), (7,4),   # top square
        (0,4), (1,5), (2,6), (3,7)    # verticals
    ]
    return V, edges

def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def apply_rotation(V, roll_x, pitch_y):
    # Convention: first roll about X, then pitch about Y (R = Ry * Rx)
    R = rot_y(pitch_y) @ rot_x(roll_x)
    return (R @ V.T).T

# -------------------------
# Live plot/animation setup
# -------------------------
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect([1,1,0.7])  # aesth. squash Z slightly

# world frame axes (just for reference)
axis_len = 1.0
origin = np.array([0,0,0])
axes_lines = [
    (origin, np.array([axis_len,0,0])),  # X
    (origin, np.array([0,axis_len,0])),  # Y
    (origin, np.array([0,0,axis_len]))   # Z
]
axes_plots = [ax.plot([],[],[], lw=2)[0] for _ in axes_lines]

# drone body (cube) lines
V0, E = make_cube(size=1.2)
cube_lines = [ax.plot([],[],[], lw=2)[0] for _ in E]

# label & bounds
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
lim = 1.2
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-lim, lim)
ax.view_init(elev=20, azim=35)

# text overlay for angles
text_roll = ax.text2D(0.02, 0.92, "", transform=ax.transAxes)
text_pitch = ax.text2D(0.02, 0.88, "", transform=ax.transAxes)

# Smoothing state
roll_ema = None
pitch_ema = None

# Serial (open lazily; tolerate disconnects)
ser = None
def ensure_serial():
    global ser
    if ser is None:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
            ser.reset_input_buffer()
            print(f"Opened {PORT} @ {BAUD}")
        except Exception as e:
            print(f"Could not open {PORT}: {e}")
            ser = None

def read_latest_accel():
    """
    Read lines until we get an ACCELEROMETER line, or timeout returns None.
    Keeps up if other sensors chat in-between.
    """
    ensure_serial()
    if ser is None:
        return None
    t0 = time.time()
    while time.time() - t0 < 0.05:  # ~50ms budget per animation frame
        try:
            raw = ser.readline()
            if not raw:
                break
            s = raw.decode(errors="ignore").strip()
            if "ACCELEROMETER" in s:
                return parse_accel_line(s)
        except Exception:
            break
    return None

def update(frame_idx):
    global roll_ema, pitch_ema

    data = read_latest_accel()
    if data is not None:
        ax_g, ay_g, az_g = data
        apitch, aroll = accel_to_pitch_roll(ax_g, ay_g, az_g)

        # smooth
        roll_ema  = ema_update(roll_ema,  aroll)
        pitch_ema = ema_update(pitch_ema, apitch)

    # if we don't have any data yet, just keep identity orientation
    roll = roll_ema if roll_ema is not None else 0.0
    pitch = pitch_ema if pitch_ema is not None else 0.0

    # rotate cube
    V = apply_rotation(V0, roll, pitch)

    # draw cube edges
    for line, (i, j) in zip(cube_lines, E):
        xs = [V[i,0], V[j,0]]
        ys = [V[i,1], V[j,1]]
        zs = [V[i,2], V[j,2]]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    # draw world axes
    for l, (p0, p1) in zip(axes_plots, axes_lines):
        l.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        l.set_3d_properties([p0[2], p1[2]])

    # update labels (degrees for readability)
    text_roll.set_text(f"Roll (X):  {np.degrees(roll):6.1f}°")
    text_pitch.set_text(f"Pitch (Y): {np.degrees(pitch):6.1f}°")

    return cube_lines + axes_plots + [text_roll, text_pitch]

ani = None
def on_key(event):
    if event.key.lower() == 'q':
        plt.close(fig)

fig.canvas.mpl_connect('key_press_event', on_key)

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, interval=30, blit=False)

plt.title("Accelerometer-based Attitude (Roll/Pitch)")
print("Press 'q' in the figure window to quit.")
plt.show()
