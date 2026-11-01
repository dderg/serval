"""Generate speed_ladder.gcode and extrude_ladder.gcode.

uv run python tools/bench/gcode/gen_speed_ladder.py
"""

from __future__ import annotations

from pathlib import Path

OUT_DIR = Path(__file__).resolve().parent
TRAVEL_FEED = 6000
ROUND_TRIPS = 2
SPEEDS_MM_S = range(100, 1001, 100)
X_RANGE = (40, 260)
Y_STRAIGHT = 150
DIAGONAL_PLUS = ((80, 80), (220, 220))
DIAGONAL_MINUS = ((80, 220), (220, 80))
E_SPEEDS_MM_S = range(5, 51, 5)
E_MOVE_MM = 20
E_ROUND_TRIPS = 2


def feed(speed_mm_s: int) -> int:
    return speed_mm_s * 60


def travel(x: float, y: float) -> str:
    return f"G1 X{x} Y{y} F{TRAVEL_FEED}"


def back_and_forth(
    start: tuple[int, int], end: tuple[int, int], speed_mm_s: int
) -> list[str]:
    lines = [travel(*start)]
    for _ in range(ROUND_TRIPS):
        lines.append(f"G1 X{end[0]} Y{end[1]} F{feed(speed_mm_s)}")
        lines.append(f"G1 X{start[0]} Y{start[1]} F{feed(speed_mm_s)}")
    return lines


def speed_ladder() -> list[str]:
    lines = ["G90", "M83"]
    straight = ((X_RANGE[0], Y_STRAIGHT), (X_RANGE[1], Y_STRAIGHT))
    for path in (straight, DIAGONAL_PLUS, DIAGONAL_MINUS):
        for speed in SPEEDS_MM_S:
            lines += back_and_forth(path[0], path[1], speed)
    lines.append(travel(X_RANGE[0], Y_STRAIGHT))
    lines.append("M400")
    return lines


def extrude_ladder() -> list[str]:
    lines = ["G90", "M83"]
    for speed in E_SPEEDS_MM_S:
        for _ in range(E_ROUND_TRIPS):
            lines.append(f"G1 E{E_MOVE_MM} F{feed(speed)}")
            lines.append(f"G1 E-{E_MOVE_MM} F{feed(speed)}")
    lines.append("M400")
    return lines


def write(name: str, lines: list[str]) -> None:
    (OUT_DIR / name).write_text("\n".join(lines) + "\n")


def main() -> None:
    write("speed_ladder.gcode", speed_ladder())
    write("extrude_ladder.gcode", extrude_ladder())


if __name__ == "__main__":
    main()
