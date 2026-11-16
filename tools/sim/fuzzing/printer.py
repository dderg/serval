"""The printer a scenario runs on: one of the configs.py generators with
its limits, shapers, extruder chain, bed mesh, heaters, macros and idle
timeout drawn from a seed, rendered to printer.cfg text."""

from __future__ import annotations

import dataclasses
import enum
import math
import random
from typing import Callable, Optional

from tools.sim import configs

AUX_MCU = "aux"
MARK_PIN = "fuzz_mark"
DONE_PIN = "fuzz_done"
MARK_PIN_LINE = 50
DONE_PIN_LINE = 51
FULL_STEPS_PER_REV = 200
PARK_XY = 20.0
MESH_PROFILE = "fz"
MESH_EDGE_MM = 20.0
HEATER_CHECK_GAIN_S = 3600
EXTRUDER_TARGET_C = 150.0
CHAMBER_TARGET_C = 100.0
XY_PIN_LINES = range(0, 6)


class Base(enum.Enum):
    HEATERS = "heaters"
    NEPTUNE = "neptune"
    BENCH = "bench"
    TRIDENT_KERNEL = "trident_kernel"


class Kinematics(enum.Enum):
    CARTESIAN = "cartesian"
    COREXY = "corexy"


@dataclasses.dataclass(frozen=True)
class BaseShape:
    render: Callable[[str, str], str]
    kinematics: Kinematics
    bed_mm: float
    xy_endstops: tuple[str, str]
    has_chamber: bool


BASES = {
    Base.HEATERS: BaseShape(
        configs.heaters_config,
        Kinematics.CARTESIAN,
        200.0,
        ("gpio200", "gpio201"),
        True,
    ),
    Base.NEPTUNE: BaseShape(
        configs.neptune_print_config,
        Kinematics.CARTESIAN,
        250.0,
        ("gpio10", "gpio11"),
        False,
    ),
    Base.BENCH: BaseShape(
        configs.bench_print_config,
        Kinematics.COREXY,
        300.0,
        ("gpio10", "gpio11"),
        False,
    ),
    Base.TRIDENT_KERNEL: BaseShape(
        configs.bench_trident_kernel_config,
        Kinematics.COREXY,
        300.0,
        ("gpio10", "gpio11"),
        False,
    ),
}


class Topology(enum.Enum):
    ONE_MCU = "one_mcu"
    XY_ON_AUX = "xy_on_aux"


class Corner(enum.Enum):
    DEVIATION = "corner_deviation"
    SQUARE_VELOCITY = "square_corner_velocity"


class Kernel(enum.Enum):
    BELL = "smooth_bell"
    TRIANGLE = "smooth_triangle"
    ZV = "smooth_zv"
    MZV = "smooth_mzv"


KERNEL_PARAM = {
    Kernel.BELL: "smooth_time",
    Kernel.TRIANGLE: "smooth_time",
    Kernel.ZV: "frequency_hz",
    Kernel.MZV: "frequency_hz",
}


@dataclasses.dataclass(frozen=True)
class Smoothing:
    kernel: Kernel
    value: float

    @property
    def param(self) -> str:
        return KERNEL_PARAM[self.kernel]


class MeshKind(enum.Enum):
    NONE = "none"
    FLAT = "flat"
    WAVY = "wavy"
    STEEP = "steep"


MESH_AMPLITUDE_MM = {
    MeshKind.FLAT: 0.02,
    MeshKind.WAVY: 0.1,
    MeshKind.STEEP: 1.0,
}


class PauseMacro(enum.Enum):
    BASE = "base"
    PARK = "park"


class CancelMacro(enum.Enum):
    BASE = "base"
    REPORTER_ORDER = "reporter_order"
    STOCK_ORDER = "stock_order"


CANCEL_END_STATE = {
    CancelMacro.BASE: "cancelled",
    CancelMacro.REPORTER_ORDER: "standby",
    CancelMacro.STOCK_ORDER: "cancelled",
}


class EndStep(enum.Enum):
    RETRACT_LIFT = "retract_lift"
    PARK = "park"
    M400 = "m400"
    FAN_OFF = "fan_off"
    MESH_CLEAR = "mesh_clear"
    HEATERS_OFF = "heaters_off"
    MOTORS_OFF = "motors_off"


@dataclasses.dataclass(frozen=True)
class PrinterSpec:
    base: Base
    topology: Topology
    max_velocity: float
    max_accel: float
    corner: Corner
    corner_value: float
    max_z_velocity: float
    max_z_accel: float
    shaper_x: Optional[Smoothing]
    shaper_y: Optional[Smoothing]
    pa_k: Optional[float]
    extruder_kernel: Optional[Smoothing]
    mesh: MeshKind
    mesh_points: tuple[tuple[float, ...], ...]
    mesh_fade: bool
    mesh_at_start: bool
    heaters: bool
    pause_macro: PauseMacro
    cancel_macro: CancelMacro
    end_steps: tuple[EndStep, ...]
    idle_timeout_s: Optional[float]

    @property
    def shape(self) -> BaseShape:
        return BASES[self.base]

    @property
    def dual_mcu(self) -> bool:
        return self.topology is Topology.XY_ON_AUX


@dataclasses.dataclass(frozen=True)
class Motor:
    name: str
    axis: str
    process: str
    step_line: int
    steps_per_mm: float


def _rounded(rng: random.Random, low: float, high: float, digits: int) -> float:
    return round(rng.uniform(low, high), digits)


def _log_uniform(rng: random.Random, low: float, high: float) -> float:
    return round(math.exp(rng.uniform(math.log(low), math.log(high))))


def _draw_smoothing(rng: random.Random, kernels=tuple(Kernel)) -> Smoothing:
    kernel = rng.choice(kernels)
    if KERNEL_PARAM[kernel] == "smooth_time":
        return Smoothing(kernel, _rounded(rng, 0.006, 0.04, 4))
    return Smoothing(kernel, _rounded(rng, 25.0, 200.0, 1))


def _maybe(rng: random.Random, chance: float, draw):
    return draw() if rng.random() < chance else None


def _draw_mesh_points(rng: random.Random, kind: MeshKind):
    if kind is MeshKind.NONE:
        return ()
    count = rng.choice((3, 5))
    amplitude = MESH_AMPLITUDE_MM[kind]
    return tuple(
        tuple(_rounded(rng, -amplitude, amplitude, 3) for _ in range(count))
        for _ in range(count)
    )


def _draw_end_steps(rng: random.Random, mesh: MeshKind) -> tuple:
    candidates = [step for step in EndStep if rng.random() < 0.5]
    if mesh is MeshKind.NONE and EndStep.MESH_CLEAR in candidates:
        candidates.remove(EndStep.MESH_CLEAR)
    return tuple(candidates)


def _draw_limits(rng: random.Random) -> dict:
    if rng.random() < 0.25:
        return dict(
            max_velocity=1000.0,
            max_accel=10000.0,
            corner=Corner.DEVIATION,
            corner_value=0.21,
            max_z_velocity=25.0,
            max_z_accel=100.0,
        )
    corner = rng.choice(tuple(Corner))
    corner_value = (
        _rounded(rng, 0.02, 0.7, 3)
        if corner is Corner.DEVIATION
        else _rounded(rng, 2.0, 30.0, 1)
    )
    return dict(
        max_velocity=float(rng.choice((150, 300, 500, 700, 1000))),
        max_accel=float(_log_uniform(rng, 1500.0, 70000.0)),
        corner=corner,
        corner_value=corner_value,
        max_z_velocity=float(rng.choice((5, 10, 15, 25))),
        max_z_accel=float(rng.choice((30, 100, 200))),
    )


def draw_printer(rng: random.Random) -> PrinterSpec:
    base = rng.choice(tuple(Base))
    mesh = rng.choice(tuple(MeshKind))
    pa_k = _maybe(rng, 0.8, lambda: _rounded(rng, 0.0, 0.08, 4))
    kernel = None
    if pa_k is not None:
        kernel = _maybe(rng, 0.5, lambda: _draw_smoothing(rng))
    return PrinterSpec(
        base=base,
        topology=rng.choice(tuple(Topology)),
        **_draw_limits(rng),
        shaper_x=_maybe(rng, 0.75, lambda: _draw_smoothing(rng)),
        shaper_y=_maybe(rng, 0.75, lambda: _draw_smoothing(rng)),
        pa_k=pa_k,
        extruder_kernel=kernel,
        mesh=mesh,
        mesh_points=_draw_mesh_points(rng, mesh),
        mesh_fade=rng.random() < 0.3,
        mesh_at_start=mesh is not MeshKind.NONE and rng.random() < 0.7,
        heaters=rng.random() < 0.5,
        pause_macro=rng.choice(tuple(PauseMacro)),
        cancel_macro=rng.choice(tuple(CancelMacro)),
        end_steps=_draw_end_steps(rng, mesh),
        idle_timeout_s=_maybe(rng, 0.2, lambda: _rounded(rng, 6.0, 10.0, 1)),
    )


class _Sections:
    """printer.cfg as ordered sections, so knobs replace options and drop
    sections without matching on generator text."""

    def __init__(self, text: str):
        self._order: list[str] = []
        self._lines: dict[str, list[str]] = {}
        current = None
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("[") and stripped.endswith("]"):
                current = stripped[1:-1]
                self._order.append(current)
                self._lines[current] = []
                continue
            if current is not None and stripped:
                self._lines[current].append(line)

    def names(self) -> list[str]:
        return list(self._order)

    def drop_section(self, name: str) -> None:
        self._order.remove(name)
        del self._lines[name]

    def set(self, section: str, key: str, value) -> None:
        self.unset(section, key)
        self._lines[section].append(f"{key}: {value}")

    def unset(self, section: str, key: str) -> None:
        self._lines[section] = [
            line
            for line in self._lines[section]
            if line.split(":", 1)[0].strip() != key
        ]

    def get(self, section: str, key: str) -> Optional[str]:
        for line in self._lines[section]:
            name, _, value = line.partition(":")
            if name.strip() == key:
                return value.strip()
        return None

    def add(self, section: str, options: dict) -> None:
        if section in self._lines:
            self.drop_section(section)
        self._order.append(section)
        self._lines[section] = [
            f"{key}: {value}" for key, value in options.items()
        ]

    def add_raw(self, section: str, body: str) -> None:
        self._order.append(section)
        self._lines[section] = body.splitlines()

    def render(self) -> str:
        blocks = [
            "\n".join([f"[{name}]", *self._lines[name]]) for name in self._order
        ]
        return "\n\n".join(blocks) + "\n"


def _xy_motor_names(spec: PrinterSpec) -> tuple[str, str]:
    if spec.shape.kinematics is Kinematics.COREXY:
        return ("a", "b")
    return ("x", "y")


def _printer_section(spec: PrinterSpec) -> dict:
    return {
        "max_velocity": spec.max_velocity,
        "max_accel": spec.max_accel,
        "max_jerk": 0,
        spec.corner.value: spec.corner_value,
        "max_z_velocity": spec.max_z_velocity,
        "max_z_accel": spec.max_z_accel,
    }


def _smoothing_section(smoothing: Smoothing) -> dict:
    return {"type": smoothing.kernel.value, smoothing.param: smoothing.value}


def _apply_chains(spec: PrinterSpec, cfg: _Sections) -> None:
    for name in cfg.names():
        if name.startswith("post_processor "):
            cfg.drop_section(name)
    for axis, shaper in (("x", spec.shaper_x), ("y", spec.shaper_y)):
        cfg.unset(f"axis {axis}", "post_processors")
        if shaper is None:
            continue
        cfg.add(f"post_processor fz_shaper_{axis}", _smoothing_section(shaper))
        cfg.set(f"axis {axis}", "post_processors", f"fz_shaper_{axis}")
    cfg.unset("axis e", "post_processors")
    if spec.pa_k is None:
        return
    cfg.add(
        "post_processor fz_pa",
        {"type": "linear_pressure_advance", "k": spec.pa_k},
    )
    chain = ["fz_pa"]
    if spec.extruder_kernel is not None:
        cfg.add(
            "post_processor fz_kernel", _smoothing_section(spec.extruder_kernel)
        )
        chain.append("fz_kernel")
    cfg.set("axis e", "post_processors", ", ".join(chain))
    if cfg.get("axis e", "max_accel") is None:
        cfg.set("axis e", "max_accel", 20000)
        cfg.set("axis e", "max_velocity", 120)


def _apply_topology(spec: PrinterSpec, cfg: _Sections, f4_pty: str) -> None:
    if spec.topology is Topology.ONE_MCU:
        return
    cfg.add(f"mcu {AUX_MCU}", {"serial": f4_pty})
    for motor in _xy_motor_names(spec):
        section = f"motor {motor}"
        for key in ("step_pin", "dir_pin", "enable_pin"):
            pin = cfg.get(section, key)
            invert = pin[: len(pin) - len(pin.lstrip("!^~"))]
            cfg.set(section, key, f"{invert}{AUX_MCU}:{pin.lstrip('!^~')}")


def _apply_heaters(spec: PrinterSpec, cfg: _Sections) -> None:
    heaters = ["extruder"] + (["chamber"] if spec.shape.has_chamber else [])
    for heater in heaters:
        cfg.add(
            f"verify_heater {heater}", {"check_gain_time": HEATER_CHECK_GAIN_S}
        )


def _mesh_section(spec: PrinterSpec) -> dict:
    bed = spec.shape.bed_mm
    count = len(spec.mesh_points)
    options = {
        "mesh_min": f"{MESH_EDGE_MM}, {MESH_EDGE_MM}",
        "mesh_max": f"{bed - MESH_EDGE_MM}, {bed - MESH_EDGE_MM}",
        "probe_count": f"{count}, {count}",
        "zero_reference_position": f"{bed / 2}, {bed / 2}",
    }
    if spec.mesh_fade:
        options.update(fade_start=1, fade_end=10, fade_target=0)
    return options


def saved_mesh_block(spec: PrinterSpec) -> str:
    bed = spec.shape.bed_mm
    count = len(spec.mesh_points)
    rows = "\n".join(
        "#*#   " + ", ".join(f"{value:.3f}" for value in row)
        for row in spec.mesh_points
    )
    algo = "lagrange" if count == 3 else "bicubic"
    return f"""\
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh {MESH_PROFILE}]
#*# version = 1
#*# points =
{rows}
#*# min_x = {MESH_EDGE_MM}
#*# max_x = {bed - MESH_EDGE_MM}
#*# min_y = {MESH_EDGE_MM}
#*# max_y = {bed - MESH_EDGE_MM}
#*# x_count = {count}
#*# y_count = {count}
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = {algo}
#*# tension = 0.2
"""


PAUSE_PARK_MACROS = {
    "PAUSE": (
        "BASE_PAUSE",
        "G91",
        "G1 E-1 F2400",
        "G1 Z2 F600",
        "G90",
        f"G1 X{PARK_XY} Y{PARK_XY} F12000",
    ),
    "RESUME": ("G91", "G1 E1 F2400", "G90", "BASE_RESUME"),
}

CANCEL_BODIES = {
    CancelMacro.REPORTER_ORDER: ("SDCARD_RESET_FILE", "BASE_CANCEL_PRINT"),
    CancelMacro.STOCK_ORDER: ("BASE_CANCEL_PRINT",),
}

CANCEL_PARK = (
    "G91",
    "G0 Z5 F4500",
    "G90",
    f"G0 X{PARK_XY} Y{PARK_XY} F6000",
    "M107",
    "SET_STEPPER_ENABLE STEPPER={x} ENABLE=0",
    "SET_STEPPER_ENABLE STEPPER={y} ENABLE=0",
)

END_STEP_LINES = {
    EndStep.RETRACT_LIFT: ("G91", "G1 E-0.5 F1800", "G0 Z5 F5000", "G90"),
    EndStep.PARK: (f"G0 X{PARK_XY} Y{PARK_XY} F6000",),
    EndStep.M400: ("M400",),
    EndStep.FAN_OFF: ("M107",),
    EndStep.MESH_CLEAR: ("BED_MESH_CLEAR",),
    EndStep.HEATERS_OFF: ("TURN_OFF_HEATERS",),
    EndStep.MOTORS_OFF: ("M84",),
}


def macros(spec: PrinterSpec) -> dict[str, tuple[str, ...]]:
    """The gcode_macro bodies the config defines, by name."""
    bodies: dict[str, tuple[str, ...]] = {}
    if spec.pause_macro is PauseMacro.PARK:
        bodies.update(PAUSE_PARK_MACROS)
    if spec.cancel_macro is not CancelMacro.BASE:
        x_motor, y_motor = _xy_motor_names(spec)
        park = tuple(line.format(x=x_motor, y=y_motor) for line in CANCEL_PARK)
        bodies["CANCEL_PRINT"] = (
            ("CLEAR_PAUSE",) + CANCEL_BODIES[spec.cancel_macro] + park
        )
    bodies["END_PRINT"] = tuple(
        line for step in spec.end_steps for line in END_STEP_LINES[step]
    )
    return bodies


RENAMED = {
    "PAUSE": "BASE_PAUSE",
    "RESUME": "BASE_RESUME",
    "CANCEL_PRINT": "BASE_CANCEL_PRINT",
}


def _apply_macros(spec: PrinterSpec, cfg: _Sections) -> None:
    if "pause_resume" not in cfg.names():
        cfg.add("pause_resume", {})
    if "display_status" not in cfg.names():
        cfg.add("display_status", {})
    for name, body in macros(spec).items():
        header = (
            f"rename_existing: {RENAMED[name]}\n" if name in RENAMED else ""
        )
        lines = "\n".join(f"  {line}" for line in body) or "  M117"
        cfg.add_raw(f"gcode_macro {name}", f"{header}gcode:\n{lines}")


def _apply_marks(cfg: _Sections) -> None:
    for name, line in ((MARK_PIN, MARK_PIN_LINE), (DONE_PIN, DONE_PIN_LINE)):
        cfg.add(
            f"output_pin {name}",
            {"pin": f"gpiochip0/gpio{line}", "value": 0, "shutdown_value": 0},
        )


def render_config(
    spec: PrinterSpec, h7_pty: str, f4_pty: str, gcode_dir: str
) -> str:
    cfg = _Sections(spec.shape.render(h7_pty, gcode_dir))
    cfg.add("printer", _printer_section(spec))
    _apply_chains(spec, cfg)
    _apply_topology(spec, cfg, f4_pty)
    _apply_heaters(spec, cfg)
    _apply_macros(spec, cfg)
    _apply_marks(cfg)
    if spec.idle_timeout_s is not None:
        cfg.add("idle_timeout", {"timeout": spec.idle_timeout_s})
    if spec.mesh is MeshKind.NONE:
        return cfg.render()
    cfg.add("bed_mesh", _mesh_section(spec))
    return cfg.render() + "\n" + saved_mesh_block(spec)


def heater_targets(spec: PrinterSpec) -> tuple[str, ...]:
    if not spec.heaters:
        return ()
    commands = [
        f"SET_HEATER_TEMPERATURE HEATER=extruder TARGET={EXTRUDER_TARGET_C}"
    ]
    if spec.shape.has_chamber:
        commands.append(
            f"SET_HEATER_TEMPERATURE HEATER=chamber TARGET={CHAMBER_TARGET_C}"
        )
    return tuple(commands)


def motors(spec: PrinterSpec, config_text: str) -> tuple[Motor, ...]:
    """Every stepper the step tracker follows, with the kinematic axis
    whose position it carries (x/y/z/e, or a/b for CoreXY belts)."""
    cfg = _Sections(config_text)
    x_motor, y_motor = _xy_motor_names(spec)
    axes = {x_motor: x_motor, y_motor: y_motor, "z": "z", "extruder": "e"}
    found = []
    for motor, axis in axes.items():
        section = f"motor {motor}"
        pin = cfg.get(section, "step_pin").lstrip("!^~")
        process = "f4" if pin.startswith(f"{AUX_MCU}:") else "h7"
        line = int(pin.rsplit("gpio", 1)[1])
        steps_per_rev = FULL_STEPS_PER_REV * int(cfg.get(section, "microsteps"))
        rotation = float(cfg.get(section, "rotation_distance"))
        found.append(
            Motor(motor, axis, process, line, steps_per_rev / rotation)
        )
    return tuple(found)
