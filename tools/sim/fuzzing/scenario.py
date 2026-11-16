"""A scenario: printer, programs, the session's prints and the timeline,
all drawn from one seed and stored as a replayable directory:

    scenario.json   seed, printer knobs, programs (segments), prints
    timeline.txt    one action per line (timeline.py)
    printer.cfg     rendered config (view; ptys as {h7_pty}, {f4_pty})
    print_<P>.gcode rendered print files (view)
"""

from __future__ import annotations

import dataclasses
import enum
import json
import math
import pathlib
import random
import typing
from typing import Optional

from tools.sim.fuzzing import printer, program, timeline
from tools.sim.fuzzing.printer import PrinterSpec
from tools.sim.fuzzing.program import Model, Program
from tools.sim.fuzzing.timeline import Action

SCENARIO_FILE = "scenario.json"
TIMELINE_FILE = "timeline.txt"
CONFIG_VIEW = "printer.cfg"
PROGRAM_NAMES = ("A", "B")
SESSIONS = (
    (("A",), 45),
    (("A", "A"), 25),
    (("A", "B"), 15),
    (("A", "A", "A"), 15),
)
START_POSITION = (100.0, 100.0, 10.0)
DEFAULT_FEED_MM_S = 25.0


@dataclasses.dataclass(frozen=True)
class Scenario:
    seed: int
    printer: PrinterSpec
    programs: tuple[Program, ...]
    prints: tuple[str, ...]
    timeline: tuple[Action, ...]

    def program(self, name: str) -> Program:
        return next(p for p in self.programs if p.name == name)

    def actions_for(self, print_index: int) -> list[Action]:
        chosen = [a for a in self.timeline if a.print_index == print_index]
        return sorted(chosen, key=lambda action: action.at_s)


def _child(seed: int, part: str) -> random.Random:
    return random.Random(f"{seed}:{part}")


def estimate_s(lines, max_velocity: float) -> float:
    """Print time at cruise speed: a floor, used only to place marks."""
    model = Model(START_POSITION + (0.0,))
    feed = DEFAULT_FEED_MM_S
    total = 0.0
    for line in lines:
        before = list(model.position)
        model.step(line)
        words = program.gcode_words(line) if line[:2] in ("G0", "G1") else {}
        feed = words.get("F", feed * 60) / 60
        distance = math.dist(before[:3], model.position[:3]) or abs(
            model.position[3] - before[3]
        )
        total += distance / max(min(feed, max_velocity), 1e-3)
        if line.startswith("G4 P"):
            total += float(line[4:]) / 1000
    return total


def _session(rng: random.Random) -> tuple[str, ...]:
    sessions, weights = zip(*SESSIONS)
    return rng.choices(sessions, weights)[0]


def generate(seed: int) -> Scenario:
    spec = printer.draw_printer(_child(seed, "printer"))
    programs = tuple(
        program.draw_program(_child(seed, f"program{name}"), spec, name)
        for name in PROGRAM_NAMES
    )
    prints = _session(_child(seed, "session"))
    rng = _child(seed, "timeline")
    actions = []
    for index, name in enumerate(prints):
        lines = program.render(next(p for p in programs if p.name == name))
        duration = estimate_s(lines, spec.max_velocity)
        actions += timeline.draw_print_actions(rng, spec, index, duration)
    used = tuple(p for p in programs if p.name in prints)
    return Scenario(seed, spec, used, prints, tuple(actions))


def _encode(value):
    if dataclasses.is_dataclass(value):
        return {
            f.name: _encode(getattr(value, f.name))
            for f in dataclasses.fields(value)
        }
    if isinstance(value, enum.Enum):
        return value.value
    if isinstance(value, (tuple, list)):
        return [_encode(item) for item in value]
    return value


def _decode(kind, data):
    origin = typing.get_origin(kind)
    if origin is typing.Union:
        inner = [arg for arg in typing.get_args(kind) if arg is not type(None)][
            0
        ]
        return None if data is None else _decode(inner, data)
    if origin is tuple:
        args = typing.get_args(kind)
        item = args[0]
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_decode(item, entry) for entry in data)
        return tuple(_decode(arg, entry) for arg, entry in zip(args, data))
    if isinstance(kind, type) and issubclass(kind, enum.Enum):
        return kind(data)
    if dataclasses.is_dataclass(kind):
        hints = typing.get_type_hints(kind)
        return kind(**{name: _decode(hints[name], data[name]) for name in data})
    return data


@dataclasses.dataclass(frozen=True)
class _Stored:
    seed: int
    printer: PrinterSpec
    programs: tuple[Program, ...]
    prints: tuple[str, ...]


def save(scenario: Scenario, directory: pathlib.Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    stored = _Stored(
        scenario.seed, scenario.printer, scenario.programs, scenario.prints
    )
    (directory / SCENARIO_FILE).write_text(
        json.dumps(_encode(stored), indent=1) + "\n"
    )
    (directory / TIMELINE_FILE).write_text(timeline.to_text(scenario.timeline))
    config = printer.render_config(
        scenario.printer, "{h7_pty}", "{f4_pty}", "{gcode_dir}"
    )
    (directory / CONFIG_VIEW).write_text(config)
    for prog in scenario.programs:
        text = "\n".join(program.render(prog)) + "\n"
        (directory / f"print_{prog.name}.gcode").write_text(text)


def load(directory: pathlib.Path) -> Scenario:
    stored = _decode(
        _Stored, json.loads((directory / SCENARIO_FILE).read_text())
    )
    actions = timeline.from_text((directory / TIMELINE_FILE).read_text())
    return Scenario(
        stored.seed, stored.printer, stored.programs, stored.prints, actions
    )


def describe(scenario: Scenario) -> str:
    spec = scenario.printer
    shapers = "/".join(
        s.kernel.value if s else "none" for s in (spec.shaper_x, spec.shaper_y)
    )
    extruder = "no_pa" if spec.pa_k is None else "pa"
    if spec.extruder_kernel is not None:
        extruder += "+" + spec.extruder_kernel.kernel.value
    acts = ",".join(action.act.value for action in scenario.timeline) or "-"
    return (
        f"{spec.base.value} {spec.topology.value} shapers={shapers} "
        f"e={extruder} mesh={spec.mesh.value} heaters={int(spec.heaters)} "
        f"prints={''.join(scenario.prints)} actions={acts}"
    )


def with_parts(
    scenario: Scenario,
    programs: Optional[tuple[Program, ...]] = None,
    actions: Optional[tuple[Action, ...]] = None,
) -> Scenario:
    return dataclasses.replace(
        scenario,
        programs=scenario.programs if programs is None else programs,
        timeline=scenario.timeline if actions is None else actions,
    )
