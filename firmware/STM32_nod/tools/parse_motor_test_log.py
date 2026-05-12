#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def parse_value(raw: str):
    lowered = raw.lower()
    if lowered in {"yes", "true"}:
        return True
    if lowered in {"no", "false"}:
        return False
    if raw.startswith("0x"):
        try:
            return int(raw, 16)
        except ValueError:
            return raw
    try:
        if any(ch in raw for ch in ".eE"):
            return float(raw)
        return int(raw)
    except ValueError:
        return raw


def parse_motor_test_line(line: str) -> dict[str, object] | None:
    line = line.strip()
    if not line.startswith("motor_test "):
        return None
    result: dict[str, object] = {}
    for token in line.split()[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        result[key] = parse_value(value)
    return result


def read_lines(path: str | None) -> list[str]:
    if path:
        return Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    return sys.stdin.read().splitlines()


def numeric_series(rows: list[dict[str, object]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def bool_count(rows: list[dict[str, object]], key: str, expected: bool) -> int:
    return sum(1 for row in rows if row.get(key) is expected)


def fmt_float(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}"


def summarize(lines: list[str]) -> str:
    rows = [row for line in lines if (row := parse_motor_test_line(line)) is not None]
    if not rows:
        return "Nem talaltam egyetlen 'motor_test ...' sort sem."

    reasons = Counter(str(row.get("reason", "?")) for row in rows)
    periodic_rows = [row for row in rows if row.get("reason") == "periodic"]
    latest = rows[-1]
    latest_periodic = periodic_rows[-1] if periodic_rows else latest

    target_rad_s = latest_periodic.get("target_rad_s")
    target_rad_s_f = float(target_rad_s) if isinstance(target_rad_s, (int, float)) else None

    measured = numeric_series(periodic_rows, "measured_rad_s")
    raw_sensor = numeric_series(periodic_rows, "raw_sensor_rad_s")
    motor_velocity = numeric_series(periodic_rows, "motor_shaft_velocity")

    stall_sector_totals = {
        sector: int(latest_periodic.get(f"stall_s{sector}", 0))
        for sector in range(6)
    }
    dominant_sector = max(stall_sector_totals, key=stall_sector_totals.get)
    dominant_value = stall_sector_totals[dominant_sector]

    within_target = 0
    low_speed = 0
    overspeed = 0
    if target_rad_s_f and measured:
        for value in measured:
            if abs(value - target_rad_s_f) <= abs(target_rad_s_f) * 0.10:
                within_target += 1
            if abs(value) < abs(target_rad_s_f) * 0.60:
                low_speed += 1
            if abs(value) > abs(target_rad_s_f) * 1.40:
                overspeed += 1

    status_lines: list[str] = []
    status_lines.append("Motor Test Log Summary")
    status_lines.append(f"Osszes motor_test sor: {len(rows)}")
    status_lines.append(
        "Reason countok: "
        + ", ".join(f"{reason}={count}" for reason, count in sorted(reasons.items()))
    )
    status_lines.append(
        "Utolso allapot: "
        f"mode={latest.get('mode', '-')}, pwm_mode={latest.get('pwm_mode', '-')}, "
        f"driver_type={latest.get('driver_type', '-')}, ctrl1={latest.get('ctrl1', '-')}, "
        f"expected_ctrl1={latest.get('expected_ctrl1', '-')}, ctrl_match={latest.get('ctrl_match', '-')}"
    )
    status_lines.append(
        "DRV/Fault summary: "
        f"frame_fault_yes={bool_count(rows, 'frame_fault', True)}, "
        f"ctrl_match_no={bool_count(rows, 'ctrl_match', False)}, "
        f"fault_active={sum(1 for row in rows if row.get('fault') == 'ACTIVE')}"
    )

    if periodic_rows:
        status_lines.append(
            "Measured speed: "
            f"min={fmt_float(min(measured) if measured else None)}, "
            f"avg={fmt_float(statistics.fmean(measured) if measured else None)}, "
            f"max={fmt_float(max(measured) if measured else None)}, "
            f"last={fmt_float(measured[-1] if measured else None)}"
        )
        status_lines.append(
            "Raw Hall speed: "
            f"min={fmt_float(min(raw_sensor) if raw_sensor else None)}, "
            f"avg={fmt_float(statistics.fmean(raw_sensor) if raw_sensor else None)}, "
            f"max={fmt_float(max(raw_sensor) if raw_sensor else None)}, "
            f"last={fmt_float(raw_sensor[-1] if raw_sensor else None)}"
        )
        status_lines.append(
            "Motor shaft_velocity: "
            f"min={fmt_float(min(motor_velocity) if motor_velocity else None)}, "
            f"avg={fmt_float(statistics.fmean(motor_velocity) if motor_velocity else None)}, "
            f"max={fmt_float(max(motor_velocity) if motor_velocity else None)}, "
            f"last={fmt_float(motor_velocity[-1] if motor_velocity else None)}"
        )
        if target_rad_s_f:
            status_lines.append(
                "Target tracking: "
                f"within_10pct={within_target}/{len(measured)}, "
                f"below_60pct={low_speed}/{len(measured)}, "
                f"above_140pct={overspeed}/{len(measured)}"
            )
        status_lines.append(
            "Hall/raw behaviour: "
            f"raw_zero_detected_yes={bool_count(periodic_rows, 'raw_zero_detected', True)}, "
            f"stall_detected_yes={bool_count(periodic_rows, 'stall_detected', True)}, "
            f"hall_jump_sum={int(sum(float(row.get('hall_jump', 0)) for row in periodic_rows))}"
        )
        status_lines.append(
            "Dominans stall szektor: "
            f"s{dominant_sector}={dominant_value} "
            f"(s0={stall_sector_totals[0]}, s1={stall_sector_totals[1]}, s2={stall_sector_totals[2]}, "
            f"s3={stall_sector_totals[3]}, s4={stall_sector_totals[4]}, s5={stall_sector_totals[5]})"
        )
        status_lines.append(
            "Utolso stall minta: "
            f"last_stall_sector={latest_periodic.get('last_stall_sector', '-')}, "
            f"last_stall_measured_rad_s={fmt_float(float(latest_periodic['last_stall_measured_rad_s'])) if isinstance(latest_periodic.get('last_stall_measured_rad_s'), (int, float)) else None}, "
            f"last_stall_raw_sensor_rad_s={fmt_float(float(latest_periodic['last_stall_raw_sensor_rad_s'])) if isinstance(latest_periodic.get('last_stall_raw_sensor_rad_s'), (int, float)) else None}"
        )

    return "\n".join(status_lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="motor_test sorok emberibb osszegzese STM32_nod closed-loop logokhoz"
    )
    parser.add_argument(
        "logfile",
        nargs="?",
        help="Log fajl eleresi utja. Ha nincs megadva, stdin-rol olvas.",
    )
    args = parser.parse_args()
    print(summarize(read_lines(args.logfile)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
