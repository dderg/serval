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


def load(path: Path, repeat: int = 1) -> Program:
    source = path.read_text()
    start = start_of(source)
    body = []
    for number, raw in enumerate(source.splitlines(), 1):
        line = raw.split(";", 1)[0].strip()
        if not line:
            continue
        command = line.split()[0].upper()
        if command in DROPPED:
            continue
        if command not in ALLOWED:
            raise ProgramError(f"{path}:{number}: unsupported command {raw!r}")
        body.append(line)
    if repeat < 1:
        raise ProgramError(f"repeat must be >= 1, got {repeat}")
    x, y, z = start
    preamble = [f"SET_KINEMATIC_POSITION X={x!r} Y={y!r} Z={z!r}", "G92 E0"]
    back_to_start = ["G90", f"G1 X{x!r} Y{y!r} Z{z!r} F6000"]
    lines = list(preamble)
    for index in range(repeat):
        if index:
            lines += back_to_start
        lines += body
    lines.append("M400")
    name = path.stem if repeat == 1 else f"{path.stem}_x{repeat}"
    return Program(name, tuple(lines), start)
