"""CoreXY mechanics as a linear mass-spring-damper network (module 1).

Generalized coordinates q (SI: rad for rotors, m for positions):
rotor_a, rotor_b, carriage_x (belt-driven carriage along the gantry),
gantry_y and, with a [local] section, toolhead_x / toolhead_y (the
toolhead on a local spring to the carriage). Klipper's corexy drives
stepper_x with a = x + y and stepper_y with b = x - y, so belt A stretches
by e_a = r*rotor_a - (x + y) and belt B by e_b = r*rotor_b - (x - y). The
mass matrix is diagonal; every spring is an elongation g.q adding k*g*g'
to K. Each motor holds its rotor to the commanded angle through a
magnetic spring (motor.py; linearized() adds its small-angle stiffness).

Damping is stiffness-proportional: every spring, the magnetic one
included, has a dashpot of beta times its stiffness (material and
eddy-current losses in the compliant parts, no drag on steady motion).
The motor realizes its share as a current-vector lead (sim.py), which
linearizes to this dashpot.
Such a system keeps its undamped mode shapes and damps mode i by
zeta_i = beta * w_i / 2, so one measured damping ratio fixes beta and
the other modes follow.

The measured resonances (printer.cfg shaper_freq_x/y, SHAPER_CALIBRATE's
peaks) calibrate the model under one of two readings:
  A (no [local]): they are the belt modes, the accelerometer's response
    peaks. X fits the belt stiffness or the toolhead mass, whichever the
    parts list leaves out; Y fits the gantry mass.
  B ([local]): they are local toolhead modes. Belts, masses and gantry
    come from the parts list; X and Y fit the local springs so that the
    measured frequencies are undamped modes. Whether they are also the
    largest response peaks, as a shaper tuned to them assumes, is a
    consistency check (response_peaks), not a fit.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
from scipy import optimize, signal

from . import parts as parts_mod

ROTOR_A, ROTOR_B, CARRIAGE_X, GANTRY_Y, TOOLHEAD_X, TOOLHEAD_Y = range(6)
FRF_GRID_HZ = (1.0, 5000.0, 6000)
BELT_STIFFNESS_RANGE = (1e2, 1e10)
GANTRY_MASS_RANGE = (0.0, 100.0)
TOOLHEAD_MASS_RANGE = (1e-3, 100.0)
CALIBRATION_RTOL = 1e-9
PEAK_RTOL = 1e-6
MAX_CALIBRATION_ROUNDS = 50


class CalibrationError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class Calibrated:
    belt_stiffness: float
    toolhead_mass: float
    gantry_mass: float
    damping_time: float
    local_stiffness_x: float = 0.0
    local_stiffness_y: float = 0.0


@dataclasses.dataclass(frozen=True)
class System:
    mass: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    radius: float
    rotors: tuple
    magnetic_stiffness: float
    motor_damping: float
    sensor: np.ndarray
    nozzle: np.ndarray

    @property
    def n(self) -> int:
        return len(self.mass)


def _elongation(n, coefficients):
    g = np.zeros(n)
    for i, w in coefficients.items():
        g[i] = w
    return g


def build(
    parts: parts_mod.Parts, cal: Calibrated, magnetic_stiffness: float
) -> System:
    local = parts.local
    n = 4 if local is None else 6
    r = parts.pulley_radius
    carriage_mass = cal.toolhead_mass if local is None else local.carriage_mass
    mass = np.zeros(n)
    mass[ROTOR_A] = mass[ROTOR_B] = parts.motor.rotor_inertia
    mass[CARRIAGE_X] = carriage_mass
    mass[GANTRY_Y] = carriage_mass + cal.gantry_mass
    stiffness = np.zeros((n, n))
    springs = [
        (
            _elongation(n, {ROTOR_A: r, CARRIAGE_X: -1.0, GANTRY_Y: -1.0}),
            cal.belt_stiffness,
        ),
        (
            _elongation(n, {ROTOR_B: r, CARRIAGE_X: -1.0, GANTRY_Y: 1.0}),
            cal.belt_stiffness,
        ),
    ]
    rows = np.zeros((2, n))
    rows[0, CARRIAGE_X] = rows[1, GANTRY_Y] = 1.0
    if local is not None:
        mass[TOOLHEAD_X] = mass[TOOLHEAD_Y] = (
            cal.toolhead_mass - local.carriage_mass
        )
        springs.append(
            (
                _elongation(n, {TOOLHEAD_X: 1.0, CARRIAGE_X: -1.0}),
                cal.local_stiffness_x,
            )
        )
        springs.append(
            (
                _elongation(n, {TOOLHEAD_Y: 1.0, GANTRY_Y: -1.0}),
                cal.local_stiffness_y,
            )
        )
        rows = np.zeros((2, n))
        rows[0, TOOLHEAD_X] = rows[1, TOOLHEAD_Y] = 1.0
    for g, k in springs:
        stiffness += k * np.outer(g, g)
    return System(
        mass=mass,
        stiffness=stiffness,
        damping=cal.damping_time * stiffness,
        radius=r,
        rotors=(ROTOR_A, ROTOR_B),
        magnetic_stiffness=magnetic_stiffness,
        motor_damping=cal.damping_time * magnetic_stiffness,
        sensor=rows,
        nozzle=rows.copy(),
    )


def command_direction(system: System, axis: str) -> np.ndarray:
    """Rotor angles (rad) per metre of commanded carriage motion."""
    sign = {"x": 1.0, "y": -1.0}[axis]
    out = np.zeros(system.n)
    out[ROTOR_A] = 1.0 / system.radius
    out[ROTOR_B] = sign / system.radius
    return out


def linearized(system: System):
    """K and C with each motor's magnetic spring and its dashpot (rotor to
    the commanded angle) added."""
    k = system.stiffness.copy()
    c = system.damping.copy()
    for rotor in system.rotors:
        k[rotor, rotor] += system.magnetic_stiffness
        c[rotor, rotor] += system.motor_damping
    return k, c


def frf(system: System, freqs, axis: str):
    """Accelerometer motion per unit commanded carriage motion along
    `axis`: SHAPER_CALIBRATE's response, accel over commanded accel."""
    k, c = linearized(system)
    direction = command_direction(system, axis)
    row = system.sensor[0 if axis == "x" else 1]
    m = np.diag(system.mass)
    out = np.empty(len(freqs), dtype=complex)
    for i, f in enumerate(freqs):
        w = 2.0 * math.pi * f
        drive = (
            system.magnetic_stiffness + 1j * w * system.motor_damping
        ) * direction
        out[i] = row @ np.linalg.solve(k - w * w * m + 1j * w * c, drive)
    return out


def _grid():
    lo, hi, count = FRF_GRID_HZ
    return np.geomspace(lo, hi, count)


def _refine(system, axis, lo, hi, centre):
    res = optimize.minimize_scalar(
        lambda f: -abs(frf(system, [f], axis)[0]),
        bounds=(lo, hi),
        method="bounded",
        options={"xatol": 1e-9 * centre},
    )
    return float(res.x), float(-res.fun)


def peak_frequency(system: System, axis: str) -> float:
    grid = _grid()
    mag = np.abs(frf(system, grid, axis))
    i = int(np.argmax(mag))
    if i == 0 or i == len(grid) - 1:
        raise CalibrationError(
            "%s response peaks at the edge of %r-%r Hz"
            % (axis, grid[0], grid[-1])
        )
    return _refine(system, axis, grid[i - 1], grid[i + 1], grid[i])[0]


def response_peaks(system: System, axis: str):
    """Every local maximum of the accelerometer response, (Hz, gain),
    largest first."""
    grid = _grid()
    mag = np.abs(frf(system, grid, axis))
    found = [
        _refine(system, axis, grid[i - 1], grid[i + 1], grid[i])
        for i in signal.find_peaks(mag)[0]
    ]
    return sorted(found, key=lambda p: -p[1])


def modes(system: System):
    """Undamped mode frequencies (Hz) of the linearized system, ascending."""
    k, _ = linearized(system)
    inv_sqrt_m = 1.0 / np.sqrt(system.mass)
    w2 = np.linalg.eigvalsh(inv_sqrt_m[:, None] * k * inv_sqrt_m[None, :])
    return np.sqrt(np.maximum(w2, 0.0)) / (2.0 * math.pi)


def damping_ratio(system: System, freq: float) -> float:
    return system.motor_damping / system.magnetic_stiffness * math.pi * freq


def _solve(fn, lo, hi, what):
    flo, fhi = fn(lo), fn(hi)
    if flo * fhi > 0.0:
        raise CalibrationError(
            "%s: no root in [%r, %r] (residuals %r, %r)"
            % (what, lo, hi, flo, fhi)
        )
    return optimize.brentq(
        fn, lo, hi, rtol=CALIBRATION_RTOL * 1e-3, xtol=1e-300
    )


def _fit_peak(parts, k_mag, cal, field, bounds, axis, target):
    def peak(value):
        system = build(parts, dataclasses.replace(cal, **{field: value}), k_mag)
        return peak_frequency(system, axis)

    value = _solve(
        lambda v: peak(v) - target,
        *bounds,
        "%s for the %s response peak at %r Hz" % (field, axis.upper(), target),
    )
    reached = peak(value)
    if abs(reached - target) > PEAK_RTOL * target:
        raise CalibrationError(
            "%s: the %s peak reaches %r for %r Hz; it jumps between modes"
            " there" % (field, axis.upper(), reached, target)
        )
    return dataclasses.replace(cal, **{field: value})


def _nearest_mode(system, freq):
    f = modes(system)
    return float(f[np.argmin(np.abs(f - freq))])


def _beta(zeta, freq):
    return 2.0 * zeta / (2.0 * math.pi * freq)


def _calibrate_belt_modes(parts, k_mag):
    if parts.toolhead_mass is not None:
        field, bounds = "belt_stiffness", BELT_STIFFNESS_RANGE
        cal = Calibrated(0.0, parts.toolhead_mass, 0.0, 0.0)
    else:
        field, bounds = "toolhead_mass", TOOLHEAD_MASS_RANGE
        cal = Calibrated(parts.belt_stiffness, 0.0, 0.0, 0.0)
    cal = dataclasses.replace(
        cal, damping_time=_beta(parts.damping_ratio, parts.resonance_x)
    )
    for _ in range(MAX_CALIBRATION_ROUNDS):
        prev = cal
        cal = _fit_peak(
            parts, k_mag, cal, field, bounds, "x", parts.resonance_x
        )
        mode = _nearest_mode(build(parts, cal, k_mag), parts.resonance_x)
        cal = dataclasses.replace(
            cal, damping_time=_beta(parts.damping_ratio, mode)
        )
        if all(
            math.isclose(
                getattr(prev, f), getattr(cal, f), rel_tol=CALIBRATION_RTOL
            )
            for f in (field, "damping_time")
        ):
            break
    else:
        raise CalibrationError("X calibration did not converge: %r" % (cal,))
    return _fit_peak(
        parts,
        k_mag,
        cal,
        "gantry_mass",
        GANTRY_MASS_RANGE,
        "y",
        parts.resonance_y,
    )


def _local_stiffness_for_mode(parts, k_mag, cal, axis, freq):
    """The local spring k that makes freq an undamped mode: K + k g g' -
    w^2 M is singular, a rank-one update of A = K - w^2 M (local spring
    off), so k = -1 / (g' A^-1 g)."""
    head, carriage = (
        (TOOLHEAD_X, CARRIAGE_X) if axis == "x" else (TOOLHEAD_Y, GANTRY_Y)
    )
    system = build(
        parts,
        dataclasses.replace(cal, **{"local_stiffness_" + axis: 0.0}),
        k_mag,
    )
    k_lin, _ = linearized(system)
    w = 2.0 * math.pi * freq
    g = _elongation(system.n, {head: 1.0, carriage: -1.0})
    k = -1.0 / (g @ np.linalg.solve(k_lin - w * w * np.diag(system.mass), g))
    if not k > 0.0:
        raise CalibrationError(
            "no positive %s local spring puts a mode at %r Hz (k = %r): the"
            " frequency lies above the rigid-toolhead modes"
            % (axis.upper(), freq, k)
        )
    return k


def _calibrate_local_modes(parts, k_mag):
    cal = Calibrated(
        parts.belt_stiffness,
        parts.toolhead_mass,
        parts.gantry_mass,
        _beta(parts.damping_ratio, parts.resonance_x),
    )
    for axis, target in (("x", parts.resonance_x), ("y", parts.resonance_y)):
        cal = dataclasses.replace(
            cal,
            **{
                "local_stiffness_" + axis: _local_stiffness_for_mode(
                    parts, k_mag, cal, axis, target
                )
            },
        )
    return cal


def calibrate(parts: parts_mod.Parts, magnetic_stiffness: float) -> Calibrated:
    """Damping: damping_ratio at the X resonance (A: the X peak's mode;
    B: the X local mode)."""
    if parts.local is None:
        return _calibrate_belt_modes(parts, magnetic_stiffness)
    return _calibrate_local_modes(parts, magnetic_stiffness)
