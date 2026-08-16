"""Wire format and transport for IMU samples.

The STM32 emits one line per sample over UART:

    I,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<flags>

Units are the sensor's natural ones, already scaled on the MCU:

    accel  m/s^2      body frame
    gyro   deg/s      body frame
    mag    uT         body frame, already remapped out of the AK8963's own frame

`t_us` is microseconds since MCU boot, from the free-running TIM5 counter --
not a wall clock. It is what makes honest log replay possible: the recorded
timing is the *device's*, not the host's guess at it.

The body frame is X forward, Y left, Z up. At rest and level the accelerometer
reads roughly (0, 0, +9.81) -- it measures specific force, so the axis pointing
up reads +g.

Keep FLAG_* in sync with Core/Inc/imu.h.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import zmq

# Telemetry health flags. Mirror of the TLM_FLAG_* defines in Core/Inc/imu.h.
FLAG_IMU_ERROR = 1 << 0  # accel/gyro I2C transfer failed; values are stale
FLAG_MAG_STALE = 1 << 1  # no new magnetometer sample this cycle (mag runs slower)
FLAG_MAG_OVERFLOW = 1 << 2  # AK8963 HOFL set; field saturated, sample discarded
FLAG_LOOP_OVERRUN = 1 << 3  # the control loop missed its deadline
FLAG_UNCALIBRATED = 1 << 4  # gyro bias calibration has not completed

FLAG_NAMES = {
    FLAG_IMU_ERROR: "IMU_ERROR",
    FLAG_MAG_STALE: "MAG_STALE",
    FLAG_MAG_OVERFLOW: "MAG_OVERFLOW",
    FLAG_LOOP_OVERRUN: "LOOP_OVERRUN",
    FLAG_UNCALIBRATED: "UNCALIBRATED",
}

FRAME_PREFIX = "I"
FRAME_FIELDS = 12  # prefix + timestamp + 9 axes + flags

DEFAULT_ENDPOINT = "tcp://127.0.0.1:5555"
TOPIC = "IMU"


def describe_flags(flags: int) -> str:
    """Render a flags word as a readable string, for logs and the HUD."""
    if not flags:
        return "ok"
    names = [name for bit, name in FLAG_NAMES.items() if flags & bit]
    known = 0
    for bit in FLAG_NAMES:
        known |= bit
    unknown = flags & ~known
    if unknown:
        names.append(f"UNKNOWN(0x{unknown:x})")
    return "|".join(names)


@dataclass(frozen=True)
class ImuSample:
    """One synchronized 9-axis reading.

    `truth` carries ground-truth (roll, pitch, yaw) in radians and is populated
    only by the simulator. Real hardware has no such thing, which is exactly
    why the simulator is worth having.
    """

    t: float  # seconds since device boot
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    flags: int = 0
    truth: Optional[Sequence[float]] = field(default=None)

    @property
    def accel(self) -> np.ndarray:
        """Specific force, m/s^2."""
        return np.array([self.ax, self.ay, self.az], dtype=float)

    @property
    def gyro_dps(self) -> np.ndarray:
        """Angular rate as it arrives on the wire, deg/s."""
        return np.array([self.gx, self.gy, self.gz], dtype=float)

    @property
    def gyro(self) -> np.ndarray:
        """Angular rate in rad/s, which is what the filter wants."""
        return np.radians(self.gyro_dps)

    @property
    def mag(self) -> np.ndarray:
        """Magnetic field, uT, body frame."""
        return np.array([self.mx, self.my, self.mz], dtype=float)

    @property
    def mag_valid(self) -> bool:
        """False when the magnetometer had nothing new or useful to say."""
        return not (self.flags & (FLAG_MAG_STALE | FLAG_MAG_OVERFLOW))

    def to_dict(self) -> dict:
        d = {
            "t": self.t,
            "ax": self.ax, "ay": self.ay, "az": self.az,
            "gx": self.gx, "gy": self.gy, "gz": self.gz,
            "mx": self.mx, "my": self.my, "mz": self.mz,
            "flags": self.flags,
        }
        if self.truth is not None:
            d["truth"] = [float(v) for v in self.truth]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "ImuSample":
        truth = d.get("truth")
        return cls(
            t=d["t"],
            ax=d["ax"], ay=d["ay"], az=d["az"],
            gx=d["gx"], gy=d["gy"], gz=d["gz"],
            mx=d["mx"], my=d["my"], mz=d["mz"],
            flags=d.get("flags", 0),
            truth=tuple(truth) if truth is not None else None,
        )


def format_line(sample: ImuSample) -> str:
    """Render a sample in the MCU's wire format.

    Used by record_log.py and by the tests; keeps one definition of the format
    rather than letting the writer and the parser drift apart.
    """
    return (
        f"{FRAME_PREFIX},{int(round(sample.t * 1e6))},"
        f"{sample.ax:.4f},{sample.ay:.4f},{sample.az:.4f},"
        f"{sample.gx:.4f},{sample.gy:.4f},{sample.gz:.4f},"
        f"{sample.mx:.3f},{sample.my:.3f},{sample.mz:.3f},"
        f"{sample.flags}"
    )


def parse_line(line: str) -> Optional[ImuSample]:
    """Parse one wire-format line, or return None if it isn't one.

    Returns None rather than raising: a serial link picks up partial lines on
    connect and garbage on a baud mismatch, and neither should stop the stream.
    Anything that isn't a well-formed frame is simply not a frame.
    """
    line = line.strip()
    if not line.startswith(FRAME_PREFIX + ","):
        return None

    parts = line.split(",")
    if len(parts) != FRAME_FIELDS:
        return None

    try:
        t_us = int(parts[1])
        values = [float(p) for p in parts[2:11]]
        flags = int(parts[11])
    except ValueError:
        return None

    return ImuSample(t_us / 1e6, *values, flags=flags)


class Publisher:
    """ZeroMQ PUB side: one message per sample, all nine axes together.

    The previous design published ACCEL/GYRO/MAG as three separate messages on
    three topics, which left the subscriber with no way to know which readings
    belonged to the same instant -- and in practice it only ever subscribed to
    one of them. A sample is one thing; it travels as one message.
    """

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT):
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(endpoint)
        self.endpoint = endpoint

    def send(self, sample: ImuSample) -> None:
        self.socket.send_string(TOPIC, flags=zmq.SNDMORE)
        self.socket.send_string(json.dumps(sample.to_dict()))

    def close(self) -> None:
        self.socket.close()


class Subscriber:
    """ZeroMQ SUB side."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT):
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, TOPIC)
        # Deliberately not CONFLATE: the filter needs every sample to integrate
        # correctly, and drain() below hands the whole backlog to it each frame.
        # The high-water mark bounds memory if the consumer stalls outright.
        self.socket.setsockopt(zmq.RCVHWM, 1000)

    def poll(self, timeout_ms: int = 0) -> Optional[ImuSample]:
        """Return the next sample, or None if none arrived within the timeout."""
        if not self.socket.poll(timeout_ms):
            return None
        self.socket.recv_string()  # topic
        payload = self.socket.recv_string()
        return ImuSample.from_dict(json.loads(payload))

    def drain(self, max_samples: int = 64) -> list:
        """Return every sample currently queued, up to a cap.

        The visualizer runs at ~30 fps against a 100+ Hz stream, so each frame
        it must consume several samples -- feeding them all through the filter
        rather than dropping them is what keeps the estimate correct.
        """
        out = []
        while len(out) < max_samples:
            sample = self.poll(0)
            if sample is None:
                break
            out.append(sample)
        return out

    def close(self) -> None:
        self.socket.close()
