"""The exact gcode a bench run prints and its batch reference plans.

The program starts from SET_KINEMATIC_POSITION (the bench has no endstops)
and ends with M400. Commands outside the allowlist fail; heater, fan and
homing commands are dropped since the bench has no such hardware.
"""

from __future__ import annotations

import dataclasses
import re
from pathlib import Path

ALLOWED = {
    "G0",
    "G1",
    "G2",
    "G3",
    "G4",
    "G90",
    "G91",
    "G92",
    "G21",
    "M82",
    "M83",
    "M400",
    "SET_VELOCITY_LIMIT",
}
DROPPED = {"M104", "M109", "M140", "M190", "M106", "M107", "M73", "G28"}
START_HEADER = re.compile(r"^; motion_eval start=([-\d.]+),([-\d.]+),([-\d.]+)")
ORIGIN = (0.0, 0.0, 0.0)


class ProgramError(ValueError):
    pass


@dataclasses.dataclass(frozen=True)
class Program:
    name: str
    lines: tuple[str, ...]
    start_xyz: tuple[float, float, float]

    def text(self) -> str:
        return "\n".join(self.lines) + "\n"


def start_of(source: str) -> tuple[float, float, float]:
    match = START_HEADER.match(source)
    if match is None:
        return ORIGIN
    x, y, z = (float(v) for v in match.groups())
    return (x, y, z)


def body_of(path: Path) -> list[str]:
    body = []
    for number, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.split(";", 1)[0].strip()
        if not line:
            continue
        command = line.split()[0].upper()
        if command in DROPPED:
            continue
        if command not in ALLOWED:
            raise ProgramError(f"{path}:{number}: unsupported command {raw!r}")
        body.append(line)
    return body


def travel_to(start: tuple[float, float, float]) -> list[str]:
    x, y, z = start
    return ["G90", f"G1 X{x!r} Y{y!r} Z{z!r} F6000", "G92 E0"]


def load(path: Path, repeat: int = 1) -> Program:
    if repeat < 1:
        raise ProgramError(f"repeat must be >= 1, got {repeat}")
    start = start_of(path.read_text())
    body = body_of(path)
    x, y, z = start
    lines = [f"SET_KINEMATIC_POSITION X={x!r} Y={y!r} Z={z!r}", "G92 E0"]
    for index in range(repeat):
        if index:
            lines += travel_to(start)[:2]
        lines += body
    lines.append("M400")
    name = path.stem if repeat == 1 else f"{path.stem}_x{repeat}"
    return Program(name, tuple(lines), start)


def concat(paths: list[Path], cycles: int, name: str) -> Program:
    """`cycles` passes over the slices, each slice entered by a travel to
    its start position (all slices are absolute-XYZ, relative-E programs)."""
    if cycles < 1:
        raise ProgramError(f"cycles must be >= 1, got {cycles}")
    slices = [(start_of(p.read_text()), body_of(p)) for p in paths]
    x, y, z = slices[0][0]
    lines = [f"SET_KINEMATIC_POSITION X={x!r} Y={y!r} Z={z!r}", "G92 E0"]
    for _ in range(cycles):
        for start, body in slices:
            lines += travel_to(start) + body
    lines.append("M400")
    return Program(f"{name}_x{cycles}", tuple(lines), slices[0][0])
