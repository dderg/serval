"""Bench session scenarios: prints on one klippy session with actions on a
timeline (pause, resume, cancel, another print, firmware restart, e-stop,
gcode), each judged by the checks session_check.py evaluates.

session.py runs a scenario; suite.py --group session runs them all.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from pathlib import Path

import run

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parents[1]
PRINTER_CFG = BENCH / "printer.cfg"
SESSION_CFG = BENCH / "configs" / "session"
PRINT_MACROS = SESSION_CFG / "print_macros.cfg"
BED_MESH_WAVY = SESSION_CFG / "bed_mesh_wavy.cfg"
FIRST_LAYERS = REPO / "tools" / "sim" / "gcode" / "voron_first_layers.gcode"
AUTOSAVE_PREFIX = "#*#"

SHORT_PRINT_LINES = 1500
DENSE_PREFIX_LINES = 200
ZIGZAG_STROKES = 120
LOAD_WAVY_MESH = "BED_MESH_PROFILE LOAD=wavy"
DWELL_LINE = "G4 P1"

INTERRUPT_AFTER_S = 20.0
NEXT_PRINT_AFTER_S = 5.0
PAUSE_HELD_S = 10.0
PARKED_PAUSE_HELD_S = 12.0
PAUSE_THEN_CANCEL_S = 8.0
DWELL_PENDING_S = 2.0
ESTOP_RECOVERY_AFTER_S = 5.5
RESTART_WATCH_S = 10.0


class Unit(enum.Enum):
    SECONDS = " s"
    RATIO = ""


@dataclass(frozen=True)
class Bound:
    limit: float
    issue: str
    unit: Unit = Unit.SECONDS

    def holds(self, value: float) -> bool:
        return value <= self.limit

    def judge(self, what: str, value: float) -> str:
        relation = "<=" if self.holds(value) else ">"
        unit = self.unit.value
        return (
            f"{what} {value:.3f}{unit} {relation} {self.limit:g}{unit} "
            f"({self.issue})"
        )


MID_PRINT_GAP_S = Bound(1.0, "#843")
SECOND_PRINT_SLOWDOWN = Bound(0.10, "#843", Unit.RATIO)
PACER_RESERVE_S = Bound(2.0, "#843")
PAUSE_CANCEL_STOP_S = Bound(6.0, "#844")
FIRMWARE_RESTART_STOP_S = Bound(6.0, "#844")
EMERGENCY_STOP_S = Bound(0.1, "#406")
REACTOR_STALL_ISSUE = "#406"


class Act(enum.Enum):
    START_PRINT = "start_print"
    PAUSE = "pause"
    RESUME = "resume"
    CANCEL = "cancel"
    FIRMWARE_RESTART = "firmware_restart"
    EMERGENCY_STOP = "emergency_stop"
    GCODE = "gcode"
    WAIT = "wait"


TERMINATING_ACTS = frozenset(
    {Act.CANCEL, Act.FIRMWARE_RESTART, Act.EMERGENCY_STOP}
)


class After(enum.Enum):
    PREVIOUS = "previous"
    PRINT_END = "print_end"
    LINE_READ = "line_read"


@dataclass(frozen=True)
class Action:
    act: Act
    delay_s: float = 0.0
    after: After = After.PREVIOUS
    script: str = ""
    marker: str = ""


class Outcome(enum.Enum):
    COMPLETE = "complete"
    CANCELLED = "cancelled"
    ERROR = "error"
    RESET = "reset"


class NotAnalyzed(enum.Enum):
    MESH = "the analyzer plans its reference without a bed mesh"
    INTERRUPTED = "interrupted: the reference plans the file uninterrupted"
    SEVERAL_PRINTS = "several prints in one capture; the reference plans one"


@dataclass(frozen=True)
class StopsWithin:
    act: Act
    bound: Bound


@dataclass(frozen=True)
class SecondPrintLikeFirst:
    bound: Bound = SECOND_PRINT_SLOWDOWN


@dataclass(frozen=True)
class ReserveBounded:
    bound: Bound = PACER_RESERVE_S


SpecificCheck = StopsWithin | SecondPrintLikeFirst | ReserveBounded


@dataclass(frozen=True)
class Config:
    base: Path = PRINTER_CFG
    fragments: tuple[Path, ...] = ()

    def text(self) -> str:
        parts = [self.base, *self.fragments]
        for part in parts[:-1]:
            if any(
                line.startswith(AUTOSAVE_PREFIX)
                for line in part.read_text().splitlines()
            ):
                raise run.BenchError(
                    f"{part} holds a {AUTOSAVE_PREFIX} block; klippy reads it "
                    "only at the end of printer.cfg, so it must come last"
                )
        return "\n".join(part.read_text().rstrip() + "\n" for part in parts)

    def describe(self) -> list[str]:
        return [
            str(part.relative_to(REPO)) for part in (self.base, *self.fragments)
        ]


@dataclass(frozen=True)
class Gcode:
    source: Path = FIRST_LAYERS
    limit: int | None = None
    setup: tuple[str, ...] = ()
    tail: tuple[str, ...] = ()
    ending: tuple[str, ...] = tuple(run.POSTAMBLE)

    def lines(self) -> list[str]:
        sanitized, _ = run.sanitize(self.source.read_text())
        body = sanitized[: -len(run.POSTAMBLE)][: self.limit]
        preamble = len(run.PREAMBLE)
        return [
            *body[:preamble],
            *self.setup,
            *body[preamble:],
            *self.tail,
            *self.ending,
        ]

    def describe(self) -> str:
        cut = "" if self.limit is None else f"[:{self.limit}]"
        return f"{self.source.relative_to(REPO)}{cut}"


@dataclass(frozen=True)
class Scenario:
    name: str
    guards: str
    gcode: Gcode
    timeline: tuple[Action, ...]
    outcomes: tuple[Outcome, ...]
    not_analyzed: NotAnalyzed
    config: Config = Config()
    checks: tuple[SpecificCheck, ...] = ()


def zigzag_moves() -> tuple[str, ...]:
    moves = ["G90", "M83", "G1 Z0.3 F600", "G1 X60 Y20 F12000"]
    for index in range(ZIGZAG_STROKES):
        x = 140.0 if index % 2 == 0 else 60.0
        y = 20.0 + (index // 2) * 0.4
        moves.append(f"G1 X{x:.1f} Y{y:.2f} E0.3 F6000")
    return tuple(moves)


MESH_CONFIG = Config(fragments=(PRINT_MACROS, BED_MESH_WAVY))
MESH_PRINT = Gcode(setup=(LOAD_WAVY_MESH,))
SHORT_PRINT = Gcode(limit=SHORT_PRINT_LINES)
START = Action(Act.START_PRINT)
NEXT_PRINT = Action(Act.START_PRINT, NEXT_PRINT_AFTER_S, After.PRINT_END)
ESTOP_RECOVERY = (
    Action(Act.FIRMWARE_RESTART, ESTOP_RECOVERY_AFTER_S),
    Action(Act.WAIT, RESTART_WATCH_S),
)
PAUSE_STOP = StopsWithin(Act.PAUSE, PAUSE_CANCEL_STOP_S)

SCENARIOS = (
    Scenario(
        "mesh_full_print",
        "#842",
        MESH_PRINT,
        (START,),
        (Outcome.COMPLETE,),
        NotAnalyzed.MESH,
        MESH_CONFIG,
    ),
    Scenario(
        "pause_park_resume_mesh",
        "#842",
        MESH_PRINT,
        (
            START,
            Action(Act.PAUSE, INTERRUPT_AFTER_S),
            Action(Act.RESUME, PARKED_PAUSE_HELD_S),
        ),
        (Outcome.COMPLETE,),
        NotAnalyzed.MESH,
        MESH_CONFIG,
    ),
    Scenario(
        "print_twice",
        "#843",
        SHORT_PRINT,
        (START, NEXT_PRINT),
        (Outcome.COMPLETE, Outcome.COMPLETE),
        NotAnalyzed.SEVERAL_PRINTS,
        checks=(SecondPrintLikeFirst(), ReserveBounded()),
    ),
    Scenario(
        "cancel_then_print",
        "#843",
        SHORT_PRINT,
        (
            START,
            NEXT_PRINT,
            Action(Act.CANCEL, INTERRUPT_AFTER_S),
            NEXT_PRINT,
        ),
        (Outcome.COMPLETE, Outcome.CANCELLED, Outcome.COMPLETE),
        NotAnalyzed.SEVERAL_PRINTS,
        checks=(SecondPrintLikeFirst(), ReserveBounded()),
    ),
    Scenario(
        "pause_resume",
        "#844",
        Gcode(),
        (
            START,
            Action(Act.PAUSE, INTERRUPT_AFTER_S),
            Action(Act.RESUME, PAUSE_HELD_S),
        ),
        (Outcome.COMPLETE,),
        NotAnalyzed.INTERRUPTED,
        checks=(PAUSE_STOP,),
    ),
    Scenario(
        "pause_cancel",
        "#844",
        Gcode(),
        (
            START,
            Action(Act.PAUSE, INTERRUPT_AFTER_S),
            Action(Act.CANCEL, PAUSE_THEN_CANCEL_S),
        ),
        (Outcome.CANCELLED,),
        NotAnalyzed.INTERRUPTED,
        checks=(PAUSE_STOP,),
    ),
    Scenario(
        "cancel_plain",
        "#844",
        Gcode(),
        (START, Action(Act.CANCEL, INTERRUPT_AFTER_S)),
        (Outcome.CANCELLED,),
        NotAnalyzed.INTERRUPTED,
        checks=(StopsWithin(Act.CANCEL, PAUSE_CANCEL_STOP_S),),
    ),
    Scenario(
        "cancel_macro_mesh",
        REACTOR_STALL_ISSUE,
        MESH_PRINT,
        (START, Action(Act.CANCEL, INTERRUPT_AFTER_S)),
        (Outcome.CANCELLED,),
        NotAnalyzed.MESH,
        MESH_CONFIG,
    ),
    Scenario(
        "end_print_mesh",
        REACTOR_STALL_ISSUE,
        Gcode(
            limit=SHORT_PRINT_LINES,
            setup=(LOAD_WAVY_MESH,),
            ending=("END_PRINT",),
        ),
        (START,),
        (Outcome.COMPLETE,),
        NotAnalyzed.MESH,
        MESH_CONFIG,
    ),
    Scenario(
        "firmware_restart",
        "#844",
        Gcode(),
        (START, Action(Act.FIRMWARE_RESTART, INTERRUPT_AFTER_S)),
        (Outcome.RESET,),
        NotAnalyzed.INTERRUPTED,
        checks=(StopsWithin(Act.FIRMWARE_RESTART, FIRMWARE_RESTART_STOP_S),),
    ),
    Scenario(
        "estop",
        "#855",
        Gcode(),
        (
            START,
            Action(Act.EMERGENCY_STOP, INTERRUPT_AFTER_S),
            *ESTOP_RECOVERY,
        ),
        (Outcome.ERROR,),
        NotAnalyzed.INTERRUPTED,
        checks=(StopsWithin(Act.EMERGENCY_STOP, EMERGENCY_STOP_S),),
    ),
    Scenario(
        "estop_g4",
        REACTOR_STALL_ISSUE,
        Gcode(limit=DENSE_PREFIX_LINES, tail=(*zigzag_moves(), DWELL_LINE)),
        (
            START,
            Action(
                Act.EMERGENCY_STOP,
                DWELL_PENDING_S,
                After.LINE_READ,
                marker=DWELL_LINE,
            ),
            *ESTOP_RECOVERY,
        ),
        (Outcome.ERROR,),
        NotAnalyzed.INTERRUPTED,
        checks=(StopsWithin(Act.EMERGENCY_STOP, EMERGENCY_STOP_S),),
    ),
)
SCENARIO_BY_NAME = {scenario.name: scenario for scenario in SCENARIOS}
