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
class Extrusion:
    """Module 3. lever: metres of filament per radian of the extruder
    rotor (printer.cfg rotation_distance / gear_ratio). The melt drains its
    compression with time_constant (what pressure advance compensates); in
    steady flow the filament pushes with back_pressure * flow (N per
    mm^3/s), so the filament-plus-melt spring is back_pressure *
    filament_area / time_constant. max_flow caps the volumetric flow
    (mm^3/s); max_bead_width caps the bead where the nozzle all but stops."""

    motor: Motor
    driver: Driver
    damping_ratio: float
    lever: float
    filament_area: float
    time_constant: float
    back_pressure: float
    max_flow: float
    max_bead_width: float

    @property
    def stiffness(self) -> float:
        """N per metre of filament compression."""
        return (
            self.back_pressure * self.filament_area / self.time_constant * 1e3
        )


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
    extrusion: Extrusion | None
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
    motor = _motor(cfg, "motor", path, steps[0])
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
        extrusion=_extrusion(cfg, path, printer_cfg),
        sources=sources,
    )


def _motor(cfg, section, path, full_steps):
    if full_steps % 4:
        raise PartsError("full_steps_per_rotation %r is not 4k" % (full_steps,))
    detent = cfg.getfloat(section, "detent_torque")
    if detent < 0.0:
        raise PartsError(
            "%s: [%s] detent_torque must not be negative" % (path, section)
        )
    return Motor(
        resistance=_float(cfg, section, "resistance", path),
        inductance=_float(cfg, section, "inductance", path),
        holding_torque=_float(cfg, section, "holding_torque", path),
        rated_current=_float(cfg, section, "rated_current", path),
        detent_torque=detent,
        rotor_inertia=_float(cfg, section, "rotor_inertia", path),
        full_steps_per_rotation=int(full_steps),
    )


def _gear_ratio(text):
    if text is None:
        return 1.0
    ratio = 1.0
    for pair in text.split(","):
        driven, driver = (float(v) for v in pair.split(":"))
        ratio *= driven / driver
    return ratio


def _extrusion(cfg, path, printer_cfg):
    if not cfg.has_section("melt"):
        return None
    full_steps = float(
        accel_residual.config_option(
            printer_cfg, "extruder", "full_steps_per_rotation"
        )
        or KLIPPER_DEFAULT_FULL_STEPS
    )
    rotation_distance = _printer_option(
        printer_cfg, "extruder", "rotation_distance"
    )
    ratio = _gear_ratio(
        accel_residual.config_option(printer_cfg, "extruder", "gear_ratio")
    )
    diameter = _printer_option(printer_cfg, "extruder", "filament_diameter")
    return Extrusion(
        motor=_motor(cfg, "extruder_motor", path, full_steps),
        driver=Driver(
            supply_voltage=_float(
                cfg, "extruder_driver", "supply_voltage", path
            ),
            run_current=_float(cfg, "extruder_driver", "run_current", path),
        ),
        damping_ratio=_float(cfg, "extruder_motor", "damping_ratio", path),
        lever=rotation_distance / ratio * 1e-3 / (2.0 * math.pi),
        filament_area=math.pi * (diameter / 2.0) ** 2,
        time_constant=_float(cfg, "melt", "time_constant", path),
        back_pressure=_float(cfg, "melt", "back_pressure", path),
        max_flow=_float(cfg, "melt", "max_flow", path),
        max_bead_width=_float(cfg, "bead", "max_width", path),
    )
