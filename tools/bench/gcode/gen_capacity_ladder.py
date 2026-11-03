"""Generate the capacity ladders: all four motors stepping at once, in rungs
of rising aggregate step rate, for measuring the MCU's step-rate ceiling.

uv run python tools/bench/gcode/gen_capacity_ladder.py

Each move goes 200 mm in X with 40 mm of Z and E, so with printer.cfg's
80 steps/mm on a/b, 400 on z and 414 on e every motor steps at about
78 * v Hz and the four together at about 314 * v Hz (v = path speed).
capacity_ladder.gcode (to ~283 kHz) fits the event-mode capture's USB
budget; capacity_ladder_high.gcode (to ~630 kHz) only runs in count mode.
"""

from __future__ import annotations

from pathlib import Path

OUT_DIR = Path(__file__).resolve().parent
LADDERS = {
    "capacity_ladder.gcode": range(100, 901, 50),
    "capacity_ladder_high.gcode": range(900, 2001, 100),
}
ROUND_TRIPS = 3
X_ENDS = (40, 240)
Z_ENDS = (10, 50)
E_PER_MOVE_MM = 38.6
Y = 150
TRAVEL_FEED = 6000


def rung(speed_mm_s: int) -> list[str]:
    feed = speed_mm_s * 60
    lines = []
    for _ in range(ROUND_TRIPS):
        for x, z in ((X_ENDS[1], Z_ENDS[1]), (X_ENDS[0], Z_ENDS[0])):
            lines.append(f"G1 X{x} Y{Y} Z{z} E{E_PER_MOVE_MM} F{feed}")
    return lines


def capacity_ladder(speeds_mm_s: range) -> list[str]:
    lines = ["G90", "M83", f"G1 X{X_ENDS[0]} Y{Y} Z{Z_ENDS[0]} F{TRAVEL_FEED}"]
    for speed in speeds_mm_s:
        lines += rung(speed)
    lines.append("M400")
    return lines


if __name__ == "__main__":
    for name, speeds in LADDERS.items():
        (OUT_DIR / name).write_text("\n".join(capacity_ladder(speeds)) + "\n")
