#!/usr/bin/env python3
"""Browser-based serial RPM plotter.

This version does not need matplotlib or tkinter. It reads the STM32 serial
debug output and serves a live graph at http://127.0.0.1:8765.

Run example:
  python tools/plot_hall_rpm_web.py COM7 --baud 115200
"""

from __future__ import annotations

import argparse
import json
import re
import threading
import time
import webbrowser
from collections import deque
from typing import Any
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial


VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=(-?\d+(?:\.\d+)?)")
SAMPLES: deque[dict[str, Any]] = deque(maxlen=2000)
SAMPLES_LOCK = threading.Lock()
STOP = threading.Event()


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Hall RPM plot</title>
  <style>
    body { margin: 0; font: 26px system-ui, sans-serif; background: #111; color: #eee; }
    header {
      min-height: 120px;
      padding: 16px 22px;
      border-bottom: 1px solid #333;
      display: flex;
      align-items: center;
      gap: 38px;
      flex-wrap: wrap;
      box-sizing: border-box;
    }
    b { font-size: 32px; }
    button {
      border: 1px solid #555;
      background: #222;
      color: #eee;
      font: 700 28px system-ui, sans-serif;
      padding: 10px 18px;
      cursor: pointer;
    }
    button.active {
      background: #f9c74f;
      color: #111;
      border-color: #f9c74f;
    }
    canvas { display: block; width: 100vw; height: calc(100vh - 120px); }
    .measured { color: #4cc9f0; }
    .target { color: #f9c74f; }
    .hall { color: #00f5d4; }
    .bad { color: #ff5c8a; }
    .mode { color: #fff; }
  </style>
</head>
<body>
  <header>
    <button id="pauseBtn" type="button">Pause</button>
    <span class="mode">Mode: <b id="mode">-</b></span>
    <span class="measured">Hall measured RPM: <b id="measured">-</b></span>
    <span style="color:#90be6d">Avg RPM: <b id="avg">-</b></span>
    <span class="target">Target RPM: <b id="target">-</b></span>
    <span style="color:#f9844a">Voltage limit: <b id="voltage">-</b></span>
    <span style="color:#c77dff">Ia/Ib/Ic: <b id="current">-</b></span>
    <span style="color:#aaa">Iref: <b id="iref">-</b></span>
    <span>PWM Hz: <b id="pwm">-</b></span>
    <span class="hall">Hall ABC: <b id="hall">---</b></span>
    <span>Hall state: <b id="hallState">-</b></span>
  </header>
  <canvas id="plot"></canvas>
  <script>
    const canvas = document.getElementById("plot");
    const ctx = canvas.getContext("2d");
    const pauseBtn = document.getElementById("pauseBtn");
    let samples = [];
    let paused = false;
    const windowSec = 20;

    pauseBtn.addEventListener("click", () => {
      paused = !paused;
      pauseBtn.textContent = paused ? "Resume" : "Pause";
      pauseBtn.classList.toggle("active", paused);
    });

    function resize() {
      canvas.width = Math.floor(canvas.clientWidth * devicePixelRatio);
      canvas.height = Math.floor(canvas.clientHeight * devicePixelRatio);
    }
    addEventListener("resize", resize);
    resize();

    function drawLine(points, color, minY, maxY, minT, maxT) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 4 * devicePixelRatio;
      ctx.beginPath();
      for (let i = 0; i < points.length; i++) {
        const p = points[i];
        const x = ((p.t - minT) / Math.max(0.001, maxT - minT)) * canvas.width;
        const y = canvas.height - ((p.y - minY) / Math.max(1, maxY - minY)) * canvas.height;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    function drawHallTrace(bit, color, minT, maxT, lane) {
      const base = canvas.height - (28 + lane * 34) * devicePixelRatio;
      const high = base - 22 * devicePixelRatio;
      const low = base;
      ctx.strokeStyle = color;
      ctx.lineWidth = 3 * devicePixelRatio;
      ctx.beginPath();
      for (let i = 0; i < samples.length; i++) {
        const s = samples[i];
        const x = ((s.t - minT) / Math.max(0.001, maxT - minT)) * canvas.width;
        const y = ((s.hall_state >> bit) & 1) ? high : low;
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          const prev = samples[i - 1];
          const prevY = ((prev.hall_state >> bit) & 1) ? high : low;
          ctx.lineTo(x, prevY);
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      if (!samples.length) {
        requestAnimationFrame(draw);
        return;
      }

      const now = samples[samples.length - 1].t;
      samples = samples.filter(s => now - s.t <= windowSec);
      const minT = Math.max(0, now - windowSec);
      const maxT = Math.max(windowSec, now);
      const ys = samples.flatMap(s => [s.measured_rpm, s.target_rpm]);
      let minY = Math.min(...ys);
      let maxY = Math.max(...ys);
      const margin = Math.max(20, (maxY - minY) * 0.15);
      minY -= margin;
      maxY += margin;

      ctx.strokeStyle = "#333";
      ctx.lineWidth = devicePixelRatio;
      for (let i = 0; i <= 5; i++) {
        const y = (i / 5) * canvas.height;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      const avg = [];
      const avgWindow = 10;
      for (let i = 0; i < samples.length; i++) {
        const from = Math.max(0, i - avgWindow + 1);
        const part = samples.slice(from, i + 1);
        avg.push({
          t: samples[i].t,
          y: part.reduce((sum, s) => sum + s.measured_rpm, 0) / part.length
        });
      }

      drawLine(samples.map(s => ({t: s.t, y: s.target_rpm})), "#f9c74f", minY, maxY, minT, maxT);
      drawLine(avg, "#90be6d", minY, maxY, minT, maxT);
      drawLine(samples.map(s => ({t: s.t, y: s.measured_rpm})), "#4cc9f0", minY, maxY, minT, maxT);
      drawHallTrace(2, "#00f5d4", minT, maxT, 2);
      drawHallTrace(1, "#f15bb5", minT, maxT, 1);
      drawHallTrace(0, "#fee440", minT, maxT, 0);

      requestAnimationFrame(draw);
    }

    async function poll() {
      if (paused) return;
      const res = await fetch("/samples");
      samples = await res.json();
      const last = samples[samples.length - 1];
      if (last) {
        document.getElementById("measured").textContent = last.measured_rpm.toFixed(1);
        document.getElementById("mode").textContent = last.mode;
        document.getElementById("target").textContent = last.target_rpm.toFixed(1);
        document.getElementById("voltage").textContent = last.voltage_limit.toFixed(2) + " V";
        document.getElementById("current").textContent =
          `${last.ia.toFixed(2)} / ${last.ib.toFixed(2)} / ${last.ic.toFixed(2)} A`;
        document.getElementById("iref").textContent = last.iref_v.toFixed(3) + " V";
        document.getElementById("pwm").textContent = last.tim1_est_pwm_hz || "-";
        const hall = last.hall_state | 0;
        const hallBits = hall.toString(2).padStart(3, "0");
        const hallValid = hall > 0 && hall < 7;
        const hallEl = document.getElementById("hall");
        const hallStateEl = document.getElementById("hallState");
        hallEl.textContent = hallBits;
        hallStateEl.textContent = hallValid ? "OK" : "INVALID";
        hallStateEl.className = hallValid ? "" : "bad";
        const lastSamples = samples.slice(-10);
        const avg = lastSamples.reduce((sum, s) => sum + s.measured_rpm, 0) / lastSamples.length;
        document.getElementById("avg").textContent = avg.toFixed(1);
      }
    }
    setInterval(poll, 100);
    draw();
  </script>
</body>
</html>
"""


def parse_values(line: str) -> dict[str, Any]:
    csv_parts = line.split(";")
    if len(csv_parts) >= 6 and csv_parts[0] in ("O", "C"):
        try:
            return {
                "mode": "OPEN LOOP" if csv_parts[0] == "O" else "CLOSED LOOP",
                "target_rpm": float(csv_parts[1]),
                "measured_rpm": float(csv_parts[2]),
                "voltage_limit": float(csv_parts[3]) / 100.0,
                "tim1_est_pwm_hz": float(csv_parts[4]),
                "hall_state": float(csv_parts[5]),
                "startup_assist": float(csv_parts[6]) if len(csv_parts) >= 7 else 0.0,
            }
        except ValueError:
            pass

    if len(csv_parts) >= 4:
        try:
            return {
                "mode": "LEGACY",
                "target_rpm": float(csv_parts[0]),
                "measured_rpm": float(csv_parts[1]),
                "voltage_limit": float(csv_parts[2]) / 100.0,
                "tim1_est_pwm_hz": float(csv_parts[3]),
                "hall_state": float(csv_parts[4]) if len(csv_parts) >= 5 else 0.0,
                "startup_assist": 0.0,
            }
        except ValueError:
            pass

    values: dict[str, Any] = {}
    for key, value in VALUE_RE.findall(line):
        values[key] = float(value)
    return values


def serial_reader(port: str, baud: int) -> None:
    start = time.monotonic()
    with serial.Serial(port, baud, timeout=0.1) as ser:
        while not STOP.is_set():
            line = ser.readline().decode(errors="replace").strip()
            values = parse_values(line)
            if "measured_rpm" not in values:
                continue
            sample = {
                "t": time.monotonic() - start,
                "mode": values.get("mode", "UNKNOWN"),
                "measured_rpm": values.get("measured_rpm", 0.0),
                "target_rpm": values.get("target_rpm", 0.0),
                "voltage_limit": values.get("voltage_limit", 0.0),
                "ia": values.get("ia", 0.0),
                "ib": values.get("ib", 0.0),
                "ic": values.get("ic", 0.0),
                "iref_v": values.get("iref_v", 0.0),
                "tim1_est_pwm_hz": values.get("tim1_est_pwm_hz", 0.0),
                "hall_state": values.get("hall_state", 0.0),
                "startup_assist": values.get("startup_assist", 0.0),
            }
            with SAMPLES_LOCK:
                SAMPLES.append(sample)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/" or self.path == "/index.html":
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/samples":
            with SAMPLES_LOCK:
                body = json.dumps(list(SAMPLES)).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, _fmt: str, *_args: object) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Hall-measured RPM in a browser.")
    parser.add_argument("port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--http-port", type=int, default=8765)
    args = parser.parse_args()

    thread = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    thread.start()

    url = f"http://127.0.0.1:{args.http_port}"
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"Reading {args.port} at {args.baud} baud")
    print(f"Open {url}")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()
        server.server_close()


if __name__ == "__main__":
    main()
