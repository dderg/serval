"""Calibrate event-mode capture against busy-waited F401 steps.

uv run python tools/bench/buzz.py --motor a --steps 2000 --period-us 100

DIAG_STEPPER_BUZZ makes the F401 toggle one STEP pin at a fixed period from
a busy-wait on its own timer, bypassing the motion engine. The capture board
records it in event mode; the report gives the exact step count, the period
ratio between the two crystals (ppm) and the spread of every interval around
the median in nanoseconds. Writes buzz.json next to the raw capture.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import analyze
import capture_events
import lock
import numpy as np
import pi
from capture import EVENT_TICK_HZ, Capture

SETTLE_S = 0.5
NS_PER_TICK = 1e9 / EVENT_TICK_HZ


def motor_channel(name: str) -> int:
    sections = analyze._config_sections(pi.REPO_CFG.read_text())
    for motor in analyze.read_motors(sections):
        if motor.name == name:
            return motor.channel
    raise pi.BenchError(f"no captured motor named {name!r} in {pi.REPO_CFG}")


def buzz(motor: str, steps: int, period_us: int, out_dir: Path) -> dict:
    channel = motor_channel(motor)
    pi.prepare_printer()
    capture = Capture(out_dir, both_edges_mask=0, events=True)
    capture.start()
    try:
        time.sleep(SETTLE_S)
        pi.gcode_script(
            f"DIAG_STEPPER_BUZZ STEPPER={motor} STEPS={steps} "
            f"PERIOD_US={period_us} DIR=1"
        )
        time.sleep(SETTLE_S)
        capture.check()
    finally:
        capture.stop()
    decoded = capture_events.load(out_dir)
    ticks = decoded.channels[channel].step_ticks
    if ticks.size != steps:
        raise pi.BenchError(
            f"captured {ticks.size} steps on channel {channel}, buzz sent {steps}"
        )
    intervals = np.diff(ticks)
    median = float(np.median(intervals))
    deviation_ns = (intervals - median) * NS_PER_TICK
    nominal_ticks = period_us * EVENT_TICK_HZ / 1e6
    return {
        "motor": motor,
        "channel": channel,
        "steps": int(ticks.size),
        "period_us": period_us,
        "median_interval_ticks": median,
        "ppm_vs_nominal": (float(intervals.mean()) / nominal_ticks - 1.0) * 1e6,
        "deviation_ns": {
            "std": float(deviation_ns.std()),
            "p1": float(np.percentile(deviation_ns, 1)),
            "p50": float(np.percentile(deviation_ns, 50)),
            "p99": float(np.percentile(deviation_ns, 99)),
            "abs_max": float(np.abs(deviation_ns).max()),
            "within_24ns": float(np.mean(np.abs(deviation_ns) <= 24.0)),
        },
        "interval_histogram_ticks": [
            [int(v), int(c)]
            for v, c in zip(*np.unique(intervals, return_counts=True))
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--motor", default="a")
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument("--period-us", type=int, default=100)
    parser.add_argument(
        "--runs-dir", type=Path, default=Path("~/bench/runs").expanduser()
    )
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    out_dir = args.runs_dir / f"buzz-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    with lock.bench_lock(args.lock_timeout):
        report = buzz(args.motor, args.steps, args.period_us, out_dir)
    (out_dir / "buzz.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
