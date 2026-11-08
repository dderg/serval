"""A printer's parts list: one config file per machine (parts/*.cfg).

Electrical and mass values come from the file; the resonance targets and
the step geometry come from the klippy printer.cfg the gcode is planned
with, so the two never disagree. Every value in a parts file needs a line
in its [sources] section.
"""

from __future__ import annotations

import configparser
import dataclasses
import math
import pathlib

from tools.motion_eval import accel_residual

HERE = pathlib.Path(__file__).resolve().parent
PARTS_DIR = HERE / "parts"
KLIPPER_DEFAULT_FULL_STEPS = 200


class PartsError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class Motor:
    resistance: float
    inductance: float
    holding_torque: float
    rated_current: float
    detent_torque: float
    rotor_inertia: float
    full_steps_per_rotation: int

    @property
    def pole_pairs(self) -> int:
        """Rotor teeth: electrical cycles per revolution (4 full steps each)."""
        return self.full_steps_per_rotation // 4

    @property
    def torque_constant(self) -> float:
        """N m per A of current-vector magnitude: the holding torque is
        taken with both phases at the rated current, a vector of
        sqrt(2) * rated_current."""
        return self.holding_torque / (math.sqrt(2.0) * self.rated_current)


@dataclasses.dataclass(frozen=True)
class Driver:
    supply_voltage: float
    run_current: float

    @property
    def current_amplitude(self) -> float:
        """Peak phase current of the sine table for an RMS run_current."""
        return math.sqrt(2.0) * self.run_current


@dataclasses.dataclass(frozen=True)
class Local:
    """Interpretation B: the toolhead sits on a local spring (hotend and
    mount compliance, gantry twist) on a belt-driven carriage, and the
    measured resonances are that local mode. carriage_mass is the part
    moving rigidly with the belts; the rest of the toolhead mass is on the
    spring."""

    carriage_mass: float


@dataclasses.dataclass(frozen=True)
class Parts:
    name: str
    motor: Motor
    driver: Driver
    toolhead_mass: float | None
    belt_stiffness: float | None
    gantry_mass: float | None
    local: Local | None
    rotation_distance: float
    resonance_x: float
    resonance_y: float
    damping_ratio: float
    sources: dict

    @property
    def pulley_radius(self) -> float:
        """Metres of belt per radian of rotor."""
        return self.rotation_distance * 1e-3 / (2.0 * math.pi)


def _float(cfg, section, option, path):
    if not cfg.has_option(section, option):
        raise PartsError("%s: [%s] %s is missing" % (path, section, option))
    value = cfg.getfloat(section, option)
    if not math.isfinite(value) or value <= 0.0:
        raise PartsError(
            "%s: [%s] %s = %r must be positive" % (path, section, option, value)
        )
    return value


def _optional(cfg, section, option, path):
    if not cfg.has_option(section, option):
        return None
    return _float(cfg, section, option, path)


def _printer_option(printer_cfg, section, option):
    value = accel_residual.config_option(printer_cfg, section, option)
    if value is None:
        raise PartsError("%s has no [%s] %s" % (printer_cfg, section, option))
    return float(value)


def _check_sources(cfg, path):
    sources = dict(cfg.items("sources")) if cfg.has_section("sources") else {}
    for section in cfg.sections():
        if section == "sources":
            continue
        for option in cfg.options(section):
            if "%s.%s" % (section, option) not in sources:
                raise PartsError(
                    "%s: [%s] %s has no [sources] %s.%s line"
                    % (path, section, option, section, option)
                )
    return sources


def load(path, printer_cfg) -> Parts:
    path = pathlib.Path(path)
    cfg = configparser.ConfigParser(interpolation=None)
    if not cfg.read(path):
        raise PartsError("cannot read %s" % (path,))
    sources = _check_sources(cfg, path)
    steps = [
        float(
            accel_residual.config_option(
                printer_cfg, s, "full_steps_per_rotation"
            )
            or KLIPPER_DEFAULT_FULL_STEPS
        )
        for s in ("stepper_x", "stepper_y")
    ]
    distances = [
        _printer_option(printer_cfg, s, "rotation_distance")
        for s in ("stepper_x", "stepper_y")
    ]
    if steps[0] != steps[1] or distances[0] != distances[1]:
        raise PartsError(
            "%s: stepper_x and stepper_y differ (full steps %r, rotation"
            " distance %r); the corexy model needs identical A/B drives"
            % (printer_cfg, steps, distances)
        )
    if steps[0] % 4:
        raise PartsError("full_steps_per_rotation %r is not 4k" % (steps[0],))
    motor = Motor(
        resistance=_float(cfg, "motor", "resistance", path),
        inductance=_float(cfg, "motor", "inductance", path),
        holding_torque=_float(cfg, "motor", "holding_torque", path),
        rated_current=_float(cfg, "motor", "rated_current", path),
        detent_torque=cfg.getfloat("motor", "detent_torque"),
        rotor_inertia=_float(cfg, "motor", "rotor_inertia", path),
        full_steps_per_rotation=int(steps[0]),
    )
    if motor.detent_torque < 0.0:
        raise PartsError("%s: detent_torque must not be negative" % (path,))
    toolhead_mass = _optional(cfg, "toolhead", "mass", path)
    belt_stiffness = _optional(cfg, "belts", "stiffness", path)
    gantry_mass = _optional(cfg, "gantry", "mass", path)
    local = None
    if cfg.has_section("local"):
        local = Local(carriage_mass=_float(cfg, "local", "carriage_mass", path))
        if None in (toolhead_mass, belt_stiffness, gantry_mass):
            raise PartsError(
                "%s: with [local] the resonances fit the local springs, so"
                " [toolhead] mass, [belts] stiffness and [gantry] mass are"
                " all required" % (path,)
            )
        if local.carriage_mass >= toolhead_mass:
            raise PartsError(
                "%s: [local] carriage_mass %r must be part of toolhead mass %r"
                % (path, local.carriage_mass, toolhead_mass)
            )
    else:
        if (toolhead_mass is None) == (belt_stiffness is None):
            raise PartsError(
                "%s: give exactly one of [toolhead] mass and [belts]"
                " stiffness; the X resonance fits the other" % (path,)
            )
        if gantry_mass is not None:
            raise PartsError(
                "%s: [gantry] mass is fitted to the Y resonance unless"
                " [local] is given" % (path,)
            )
    return Parts(
        name=path.stem,
        motor=motor,
        driver=Driver(
            supply_voltage=_float(cfg, "driver", "supply_voltage", path),
            run_current=_float(cfg, "driver", "run_current", path),
        ),
        toolhead_mass=toolhead_mass,
        belt_stiffness=belt_stiffness,
        gantry_mass=gantry_mass,
        local=local,
        rotation_distance=distances[0],
        resonance_x=_printer_option(
            printer_cfg, "input_shaper", "shaper_freq_x"
        ),
        resonance_y=_printer_option(
            printer_cfg, "input_shaper", "shaper_freq_y"
        ),
        damping_ratio=_float(cfg, "calibration", "damping_ratio", path),
        sources=sources,
    )
