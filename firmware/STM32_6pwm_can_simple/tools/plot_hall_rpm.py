#!/usr/bin/env python3
"""Serial RPM plotter for the STM32_6pwm_can_simple debug output.

Install dependencies once:
  python -m pip install pyserial matplotlib

Run example:
  python tools/plot_hall_rpm.py COM7 --baud 115200
"""

from __future__ import annotations

import argparse
import re
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=(-?\d+(?:\.\d+)?)")


def parse_values(line: str) -> dict[str, float]:
    values: dict[str, float] = {}
    for key, value in VALUE_RE.findall(line):
        try:
            values[key] = float(value)
        except ValueError:
            pass
    return values


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Hall-measured motor RPM from serial debug.")
    parser.add_argument("port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=20.0, help="Visible time window in seconds")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    start = time.monotonic()
    times: deque[float] = deque()
    measured_rpm: deque[float] = deque()
    target_rpm: deque[float] = deque()

    fig, ax = plt.subplots()
    (measured_line,) = ax.plot([], [], label="Hall measured RPM")
    (target_line,) = ax.plot([], [], label="Target RPM", linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("RPM")
    ax.grid(True)
    ax.legend(loc="upper left")

    def update(_frame: int):
        while ser.in_waiting:
            raw = ser.readline().decode(errors="replace").strip()
            values = parse_values(raw)
            if "measured_rpm" not in values:
                continue

            now = time.monotonic() - start
            times.append(now)
            measured_rpm.append(values["measured_rpm"])
            target_rpm.append(values.get("target_rpm", 0.0))

            while times and now - times[0] > args.window:
                times.popleft()
                measured_rpm.popleft()
                target_rpm.popleft()

        if not times:
            return measured_line, target_line

        measured_line.set_data(times, measured_rpm)
        target_line.set_data(times, target_rpm)
        ax.set_xlim(max(0.0, times[-1] - args.window), max(args.window, times[-1]))

        y_values = list(measured_rpm) + list(target_rpm)
        y_min = min(y_values)
        y_max = max(y_values)
        margin = max(20.0, (y_max - y_min) * 0.15)
        ax.set_ylim(y_min - margin, y_max + margin)
        return measured_line, target_line

    animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
