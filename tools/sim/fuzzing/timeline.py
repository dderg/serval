"""User actions at print-time marks, one per line of a plain text file:

print=0 at=3.250 pause_resume hold_s=1.50 jog=5.0,-3.0,1.0
print=1 at=0.800 estop_restart delay_s=5.40
"""

from __future__ import annotations

import dataclasses
import enum
import random
from typing import Optional

from tools.sim.fuzzing.printer import MeshKind, PrinterSpec

EARLIEST_MARK_S = 0.5
LATEST_MARK_FRACTION = 0.8
ACTIONS_PER_PRINT = ((0, 40), (1, 35), (2, 15), (3, 10))
ENDPOINT_DEATH_WINDOW_S = (4.5, 6.5)
MAX_ESTOP_DELAY_S = 10.0
MAX_JOG_MM = 10.0
MAX_JOG_Z_MM = 5.0
MAX_FAULT_S = 0.1
FAULT_COMMAND = "get_clock"


class Act(enum.Enum):
    PAUSE_RESUME = "pause_resume"
    CANCEL = "cancel"
    FIRMWARE_RESTART = "firmware_restart"
    HOST_RESTART = "host_restart"
    ESTOP_RESTART = "estop_restart"
    SET_PA = "set_pa"
    SET_POST_PROCESSOR = "set_post_processor"
    SET_INPUT_SHAPER = "set_input_shaper"
    MESH_LOAD = "mesh_load"
    MESH_CLEAR = "mesh_clear"
    HEATER = "heater"
    LINK_FAULT = "link_fault"


ENDS_PRINT = (
    Act.CANCEL,
    Act.FIRMWARE_RESTART,
    Act.HOST_RESTART,
    Act.ESTOP_RESTART,
)
RESTARTS = (Act.FIRMWARE_RESTART, Act.HOST_RESTART, Act.ESTOP_RESTART)
STOPS_MOTION = ENDS_PRINT + (Act.PAUSE_RESUME,)
CHANGES_STATE = (
    Act.SET_PA,
    Act.SET_POST_PROCESSOR,
    Act.MESH_LOAD,
    Act.MESH_CLEAR,
)


@dataclasses.dataclass(frozen=True)
class Action:
    print_index: int
    at_s: float
    act: Act
    params: tuple[tuple[str, str], ...] = ()

    def param(self, key: str, default: Optional[str] = None) -> Optional[str]:
        return dict(self.params).get(key, default)

    def to_line(self) -> str:
        words = [
            f"print={self.print_index}",
            f"at={self.at_s:.3f}",
            self.act.value,
        ]
        words += [f"{key}={value}" for key, value in self.params]
        return " ".join(words)

    @classmethod
    def from_line(cls, line: str) -> "Action":
        words = line.split()
        print_index = int(words[0].removeprefix("print="))
        at_s = float(words[1].removeprefix("at="))
        params = tuple(tuple(word.split("=", 1)) for word in words[3:])
        return cls(print_index, at_s, Act(words[2]), params)


def to_text(actions) -> str:
    return "".join(action.to_line() + "\n" for action in actions)


def from_text(text: str) -> tuple[Action, ...]:
    lines = (line.strip() for line in text.splitlines())
    return tuple(
        Action.from_line(line)
        for line in lines
        if line and not line.startswith("#")
    )


def _fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def _pause_params(rng: random.Random) -> tuple:
    params = [("hold_s", _fmt(rng.uniform(0.2, 3.0), 2))]
    if rng.random() < 0.5:
        jog = (
            rng.uniform(-MAX_JOG_MM, MAX_JOG_MM),
            rng.uniform(-MAX_JOG_MM, MAX_JOG_MM),
            rng.uniform(0.0, MAX_JOG_Z_MM),
        )
        params.append(("jog", ",".join(_fmt(value, 2) for value in jog)))
    return tuple(params)


def _estop_params(rng: random.Random) -> tuple:
    if rng.random() < 0.3:
        delay = rng.uniform(*ENDPOINT_DEATH_WINDOW_S)
    else:
        delay = rng.uniform(0.1, MAX_ESTOP_DELAY_S)
    return (("delay_s", _fmt(delay, 2)),)


def _set_pa_params(rng: random.Random) -> tuple:
    params = [("advance", _fmt(rng.uniform(0.0, 0.1), 4))]
    if rng.random() < 0.3:
        params.append(("smooth_time", _fmt(rng.uniform(0.0, 0.04), 4)))
    return tuple(params)


def _post_processor_targets(
    spec: PrinterSpec,
) -> list[tuple[str, str, float, float]]:
    targets = []
    for name, smoothing in (
        ("fz_shaper_x", spec.shaper_x),
        ("fz_shaper_y", spec.shaper_y),
        ("fz_kernel", spec.extruder_kernel),
    ):
        if smoothing is None:
            continue
        if smoothing.param == "smooth_time":
            targets.append((name, smoothing.param, 0.006, 0.04))
        else:
            targets.append((name, smoothing.param, 25.0, 200.0))
    if spec.pa_k is not None:
        targets.append(("fz_pa", "k", 0.0, 0.08))
    return targets


def _post_processor_params(rng: random.Random, spec: PrinterSpec) -> tuple:
    name, param, low, high = rng.choice(_post_processor_targets(spec))
    return (
        ("name", name),
        ("param", param),
        ("value", _fmt(rng.uniform(low, high), 4)),
    )


def _heater_params(rng: random.Random) -> tuple:
    style = rng.choice(("m104", "set_heater_temperature"))
    return (("style", style), ("target", str(rng.randint(0, 200))))


def _fault_params(rng: random.Random, spec: PrinterSpec) -> tuple:
    mcu = rng.choice(("h7", "f4") if spec.dual_mcu else ("h7",))
    kind = rng.choice(("drop", "delay", "stall"))
    if kind == "drop":
        return (("mcu", mcu), ("kind", kind), ("count", str(rng.randint(1, 2))))
    return (
        ("mcu", mcu),
        ("kind", kind),
        ("by_s", _fmt(rng.uniform(0.01, MAX_FAULT_S))),
    )


def _candidates(spec: PrinterSpec) -> dict:
    """Each act the printer supports, with its parameter draw."""
    acts = {
        Act.PAUSE_RESUME: lambda rng: _pause_params(rng),
        Act.CANCEL: lambda rng: (),
        Act.FIRMWARE_RESTART: lambda rng: (),
        Act.HOST_RESTART: lambda rng: (),
        Act.ESTOP_RESTART: _estop_params,
        Act.SET_PA: _set_pa_params,
        Act.SET_INPUT_SHAPER: lambda rng: (
            ("freq", _fmt(rng.uniform(25, 200), 1)),
        ),
        Act.HEATER: _heater_params,
        Act.LINK_FAULT: lambda rng: _fault_params(rng, spec),
    }
    if _post_processor_targets(spec):
        acts[Act.SET_POST_PROCESSOR] = lambda rng: _post_processor_params(
            rng, spec
        )
    if spec.mesh is not MeshKind.NONE:
        acts[Act.MESH_LOAD] = lambda rng: ()
        acts[Act.MESH_CLEAR] = lambda rng: ()
    return acts


def _count(rng: random.Random) -> int:
    counts, weights = zip(*ACTIONS_PER_PRINT)
    return rng.choices(counts, weights)[0]


def draw_print_actions(
    rng: random.Random, spec: PrinterSpec, print_index: int, duration_s: float
) -> list[Action]:
    acts = _candidates(spec)
    latest = max(EARLIEST_MARK_S, LATEST_MARK_FRACTION * duration_s)
    marks = sorted(
        rng.uniform(EARLIEST_MARK_S, latest) for _ in range(_count(rng))
    )
    actions = []
    for at_s in marks:
        act = rng.choice(sorted(acts, key=lambda a: a.value))
        actions.append(Action(print_index, round(at_s, 3), act, acts[act](rng)))
        if act in ENDS_PRINT:
            break
    return actions
