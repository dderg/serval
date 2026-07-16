"""Servo calibration toolkit (A6-EC over EtherCAT).

Loaded only when a printer.cfg contains a [servo_calibration] section
(typically on the EtherCAT bench, so no config in this repo references it);
run-invariant values (motor datasheet, stroke window, drive names,
excitation grid) live in the config section and every command reads them as
overridable defaults. Command and option reference:
docs/rewrite/servo-calibration.md.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import subprocess
import time

try:
    import tomllib
except ImportError:
    tomllib = None
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Callable, overload

from .. import structured_log
from . import servo_param, servo_strain_comp, servo_strokes

ApplyResult = tuple[Mapping[str, float], list[dict[str, Any]]]
VERDICT_ABORT_FLAGS = frozenset({"torque_saturated", "resonance_detected"})

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
DEFAULT_CAPTURES_ROOT = "~/printer_data/logs/servo_captures"
DEFAULT_DYNAMICS_DIR = "~/printer_data/config/servo_dynamics"

_git_rev_cache: str | None = None


def _git_rev() -> str:
    global _git_rev_cache
    if _git_rev_cache is None:
        try:
            _git_rev_cache = (
                subprocess.check_output(
                    ["git", "rev-parse", "--short", "HEAD"],
                    cwd=REPO_ROOT,
                    stderr=subprocess.DEVNULL,
                )
                .decode()
                .strip()
            )
        except Exception:
            _git_rev_cache = "unknown"
    return _git_rev_cache


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


GAIN_PARAMS = {
    "position": (
        "0x2001.0x01",
        1,
        20000,
        "C01.00 position loop gain",
        "0.1 rad/s",
        10.0,
    ),
    "speed": (
        "0x2001.0x02",
        1,
        20000,
        "C01.01 speed loop gain",
        "0.1 Hz",
        10.0,
    ),
    "integral": (
        "0x2001.0x03",
        15,
        51200,
        "C01.02 speed integral time",
        "0.01 ms",
        100.0,
    ),
}

SPEED_GAIN_MIN = 100
SPEED_GAIN_MAX = min(
    GAIN_PARAMS["speed"][2], int(GAIN_PARAMS["position"][2] / 1.6)
)


INERTIA_RATIO_ADDR = "0x2000.0x07"
C00_06_INERTIA_RATIO_MAX = 12000

C00_05_STIFFNESS_LEVEL_MIN = 1
C00_05_STIFFNESS_LEVEL_MAX = 31

SYNC_LOSS_COUNT_ADDR = "0x2013.0x05"
SYNC_LOSS_THRESHOLD_ADDR = "0x2013.0x03"

NOTCH_MODE_ADDR = "0x2001.0x31"
NOTCH_READBACK: tuple[tuple[str, tuple[str, str, str]], ...] = (
    ("notch1", ("0x2001.0x41", "0x2001.0x42", "0x2001.0x43")),
    ("notch2", ("0x2001.0x44", "0x2001.0x45", "0x2001.0x46")),
    ("notch3", ("0x2001.0x47", "0x2001.0x48", "0x2001.0x49")),
    ("notch4", ("0x2001.0x4a", "0x2001.0x4b", "0x2001.0x4c")),
    ("notch5", ("0x2001.0x4d", "0x2001.0x4e", "0x2001.0x4f")),
)
LADDER_STOP_FLAGS = frozenset(
    {"resonance_detected", "torque_saturated", "settle_window_truncated"}
)


def refine_values(
    current: float,
    values_text: str | None,
    span: float | None,
    steps: int,
) -> list[int]:
    if values_text is not None:
        vals = [
            int(round(float(v))) for v in values_text.split(",") if v.strip()
        ]
        if not vals:
            raise ValueError("VALUES lists no usable numbers")
    else:
        if steps < 2:
            raise ValueError("STEPS must be at least 2")
        if span is None or not 0.0 < span < 1.0:
            raise ValueError(
                "SPAN must be between 0 and 1 (fraction of current)"
            )
        lo, hi = 1.0 - span, 1.0 + span
        vals = [
            int(round(current * (lo + (hi - lo) * i / (steps - 1))))
            for i in range(steps)
        ]
        vals.append(int(round(current)))
    return sorted(set(vals))


def validate_gain_values(values: list[int], param: str) -> list[int]:
    if param not in GAIN_PARAMS:
        raise ValueError(
            "PARAM must be position, speed or integral (got %r)" % (param,)
        )
    _addr, lo, hi, _desc, _unit, _scale = GAIN_PARAMS[param]
    for v in values:
        if v <= 0:
            raise ValueError(
                "%s value %d is not a positive integer" % (param, v)
            )
        if not lo <= v <= hi:
            raise ValueError(
                "%s value %d outside drive range %d..%d" % (param, v, lo, hi)
            )
    return values


DYNAMICS_METRIC_BY_TERM = {
    "MASS": "ferr_peak",
    "VISCOUS": "ferr_rms",
    "COULOMB": "ferr_peak",
}
DYNAMICS_TERM_KEYS = {
    "MASS": "mass",
    "VISCOUS": "viscous",
    "COULOMB": "coulomb",
}
GOLDEN_RATIO_CONJ = (math.sqrt(5.0) - 1.0) / 2.0


def parse_dynamics_profile(text: str) -> dict[str, Any]:
    if tomllib is None:
        raise ValueError(
            "parsing dynamics profiles requires Python 3.11+ (tomllib)"
        )
    data = tomllib.loads(text)
    if data.get("version") != 6:
        raise ValueError(
            "dynamics profile version must be 6 (got %r) - refit with "
            "SERVO_FIT_DYNAMICS" % (data.get("version"),)
        )
    if "pair" in data:
        raise ValueError(
            "dynamics profile contains [[pair]] tables - the belt pair "
            "load-share split was removed; refit with SERVO_FIT_DYNAMICS"
        )
    axes = data.get("axes")
    if not isinstance(axes, list) or not axes:
        raise ValueError("profile axes must be a non-empty list")
    n_slots = len(axes)
    modes = data.get("modes")
    if not isinstance(modes, list) or not modes:
        raise ValueError("profile modes must be a non-empty list")
    n_modes = len(modes)
    frame = data.get("frame")
    if (
        not isinstance(frame, list)
        or len(frame) != n_modes
        or any(
            not isinstance(row, list) or len(row) != n_slots for row in frame
        )
    ):
        raise ValueError(
            "profile frame must be %d modes x %d slots" % (n_modes, n_slots)
        )
    for key in ("mass", "viscous", "coulomb"):
        vec = data.get(key)
        if not isinstance(vec, list) or len(vec) != n_modes:
            raise ValueError(
                "profile %s must list %d per-mode values" % (key, n_modes)
            )
    numbers = [v for row in frame for v in row]
    for key in ("mass", "viscous", "coulomb"):
        numbers += data[key]
    for v in numbers:
        if (
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
        ):
            raise ValueError(
                "profile contains a non-numeric or non-finite value: %r" % (v,)
            )
    return {
        "axes": [str(a) for a in axes],
        "modes": [str(m) for m in modes],
        "frame": [[float(v) for v in row] for row in frame],
        "mass": [float(v) for v in data["mass"]],
        "viscous": [float(v) for v in data["viscous"]],
        "coulomb": [float(v) for v in data["coulomb"]],
    }


def _copy_dynamics(profile: dict[str, Any]) -> dict[str, Any]:
    return {
        "axes": list(profile["axes"]),
        "modes": list(profile["modes"]),
        "frame": [list(row) for row in profile["frame"]],
        "mass": list(profile["mass"]),
        "viscous": list(profile["viscous"]),
        "coulomb": list(profile["coulomb"]),
    }


def scale_dynamics(
    profile: dict[str, Any], term: str, scale: float
) -> dict[str, Any]:
    key = DYNAMICS_TERM_KEYS.get(term)
    if key is None:
        raise ValueError("unknown dynamics term %r" % (term,))
    scaled = _copy_dynamics(profile)
    scaled[key] = [v * scale for v in profile[key]]
    return scaled


def scale_dynamics_mode(
    profile: dict[str, Any], term: str, mode_index: int, scale: float
) -> dict[str, Any]:
    key = DYNAMICS_TERM_KEYS.get(term)
    if key is None:
        raise ValueError("unknown dynamics term %r" % (term,))
    scaled = _copy_dynamics(profile)
    values = list(profile[key])
    values[mode_index] = values[mode_index] * scale
    scaled[key] = values
    return scaled


def render_dynamics_toml(
    profile: dict[str, Any],
    source: str,
    term: str,
    scales: dict[str, float],
    run_dir: str,
) -> str:
    def num(v: float) -> str:
        if not math.isfinite(v):
            raise ValueError("refusing to render non-finite value %r" % (v,))
        return repr(float(v))

    def vec(values: list[float]) -> str:
        return "[%s]" % (", ".join(num(v) for v in values),)

    lines = [
        "version = 6",
        "axes = %s" % (json.dumps(profile["axes"]),),
        "modes = %s" % (json.dumps(profile["modes"]),),
        "frame = [%s]" % (", ".join(vec(row) for row in profile["frame"]),),
        "mass = %s" % (vec(profile["mass"]),),
        "viscous = %s" % (vec(profile["viscous"]),),
        "coulomb = %s" % (vec(profile["coulomb"]),),
        "refined_source = %s" % (json.dumps(source),),
        "refined_term = %s" % (json.dumps(term.lower()),),
    ]
    for suffix, scale in sorted(scales.items()):
        lines.append(
            "refined_scale%s = %s"
            % ("_%s" % (suffix,) if suffix else "", num(scale))
        )
    lines.append("refined_run = %s" % (json.dumps(run_dir),))
    return "\n".join(lines) + "\n"


class _GssBudgetExhausted(Exception):
    pass


def golden_section_search(
    evaluate: Callable[[float], float],
    lo: float,
    hi: float,
    tol: float,
    max_evals: int,
) -> tuple[float, float, list[tuple[float, float]]]:
    """Minimize evaluate() over [lo, hi]; probes are cached on round(x, 4)
    so re-probes are free, and the search stops once the bracket is
    narrower than tol or max_evals distinct probes have run. Returns the
    measured best probe (argmin over the cache), not the bracket midpoint -
    under measurement noise the point actually measured best is the only
    defensible pick."""
    if not 0.0 < lo < hi:
        raise ValueError("bracket must satisfy 0 < LO < HI")
    if tol <= 0.0:
        raise ValueError("TOL must be > 0")
    if max_evals < 3:
        raise ValueError("MAX_EVALS must be at least 3")
    cache: dict[float, float] = {}

    def probe(x: float) -> float:
        key = round(x, 4)
        if key in cache:
            return cache[key]
        if len(cache) >= max_evals:
            raise _GssBudgetExhausted()
        cache[key] = evaluate(key)
        return cache[key]

    a, b = lo, hi
    try:
        c = b - GOLDEN_RATIO_CONJ * (b - a)
        d = a + GOLDEN_RATIO_CONJ * (b - a)
        fc, fd = probe(c), probe(d)
        while b - a > tol:
            if fc <= fd:
                b, d, fd = d, c, fc
                c = b - GOLDEN_RATIO_CONJ * (b - a)
                fc = probe(c)
            else:
                a, c, fc = c, d, fd
                d = a + GOLDEN_RATIO_CONJ * (b - a)
                fd = probe(d)
    except _GssBudgetExhausted:
        pass
    best_scale, best_score = min(cache.items(), key=lambda kv: (kv[1], kv[0]))
    return best_scale, best_score, sorted(cache.items())


def _applied(servo: str, addr: str, value: int) -> dict[str, Any]:
    return {"servo": servo, "addr": addr, "type": "u16", "value": value}


_C0006_RE = re.compile(r"recommended C00\.06 \(light direction\):\s*(-?\d+)%")


def _parse_c0006_recommendation(text: str) -> int | None:
    """servo-cal fit prints the C00.06 pick to stdout/stderr (no JSON
    field carries it - profile_out::render_profile never emits it); the
    console stream servo_calibration already captures is the cleanest
    existing seam to recover it programmatically."""
    m = _C0006_RE.search(text)
    return int(m.group(1)) if m else None


class _OverrideGcmd:
    """Wraps a gcmd, forcing specific parameter values so a stage can drive
    another SERVO_* command's implementation directly - SERVO_AUTOTUNE's
    stages are the real command bodies, not a reimplementation of them."""

    def __init__(self, base: Any, overrides: dict[str, Any]):
        self._base = base
        self._overrides = overrides
        self.error = base.error
        self.respond_info = base.respond_info
        self.get_commandline = base.get_commandline

    def get(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get(name, default, **kw)

    def get_int(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get_int(name, default, **kw)

    def get_float(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get_float(name, default, **kw)


@dataclass
class SweepStep:
    name: str
    swept: dict[str, float]
    applied: list[dict[str, Any]]
    accel: str | None = None


@dataclass
class ExperimentRun:
    """One experiment's run directory and its manifest, rewritten as steps
    complete so a crashed run keeps partial truth on disk."""

    run_dir: str
    stamp: str
    manifest: dict[str, Any]
    started_s: float = field(default_factory=time.time)

    @property
    def manifest_path(self) -> str:
        return os.path.join(self.run_dir, "manifest.json")

    def step_scap(self, name: str) -> str:
        return os.path.join(self.run_dir, "step_%s.scap" % (name,))

    def step_accel_csv(self, name: str) -> str:
        return os.path.join(self.run_dir, "step_%s_accel.csv" % (name,))

    def write(self) -> None:
        tmp = self.manifest_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.manifest, f, indent=2)
        os.replace(tmp, self.manifest_path)

    def record_step(self, step: SweepStep) -> None:
        self.manifest["steps"].append(
            {
                "name": step.name,
                "swept": step.swept,
                "applied": step.applied,
                "capture": os.path.basename(self.step_scap(step.name)),
                "accel": step.accel,
            }
        )
        self.write()


class GainSetAdapter:
    """Speed-gain sweep: derives position/integral, writes gain set 1."""

    def __init__(
        self, calibration: "ServoCalibration", servos: list[str], tag: str
    ):
        self._cal = calibration
        self.servos = servos
        self.tag = tag

    @staticmethod
    def derive(speed_gain: int) -> tuple[int, int]:
        return round(speed_gain * 1.6), round(1250000 / speed_gain)

    def step_name(self, speed_gain: int) -> str:
        pos_gain, integral = self.derive(speed_gain)
        return "%s_p%d_s%d_i%d" % (self.tag, pos_gain, speed_gain, integral)

    def describe(
        self, i: int, speed_gain: int, total: int, servos: list[str]
    ) -> str:
        pos_gain, integral = self.derive(speed_gain)
        return (
            "gain step %d/%d: pos %.1f rad/s, speed %.1f Hz, Ti %.2f ms on %s"
            % (
                i + 1,
                total,
                pos_gain / 10.0,
                speed_gain / 10.0,
                integral / 100.0,
                ", ".join(servos),
            )
        )

    def apply(self, speed_gain: int) -> ApplyResult:
        pos_gain, integral = self.derive(speed_gain)
        self._cal._write_gains(self.servos, pos_gain, speed_gain, integral)
        swept = {
            "position": pos_gain,
            "speed": speed_gain,
            "integral": integral,
        }
        applied = self._cal._gain_write_records(
            self.servos, pos_gain, speed_gain, integral
        )
        return swept, applied

    def revert(self, speed_gain: int) -> None:
        pos_gain, integral = self.derive(speed_gain)
        self._cal._write_gains(self.servos, pos_gain, speed_gain, integral)


class SingleGainAdapter:
    """SERVO_REFINE_GAIN: sweeps one gain, holding the other two fixed."""

    def __init__(
        self,
        calibration: "ServoCalibration",
        servos: list[str],
        param: str,
        tag: str,
        original: dict[str, int],
        current: int,
    ):
        self._cal = calibration
        self.servos = servos
        self.param = param
        self.tag = tag
        self._original = original
        self.current = current

    def step_name(self, value: int) -> str:
        return "%s_%s_v%d" % (self.tag, self.param, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        _addr, _lo, _hi, desc, unit, scale = GAIN_PARAMS[self.param]
        marker = "  <- current" if value == self.current else ""
        return "refine %s step %d/%d: %s = %d (%.4g %s)%s on %s" % (
            self.param,
            i + 1,
            total,
            desc,
            value,
            value / scale,
            unit,
            marker,
            ", ".join(servos),
        )

    def apply(self, value: int) -> ApplyResult:
        triple = dict(self._original)
        triple[self.param] = value
        self._cal._write_gains(
            self.servos, triple["position"], triple["speed"], triple["integral"]
        )
        swept = {self.param: value}
        applied = self._cal._gain_write_records(
            self.servos, triple["position"], triple["speed"], triple["integral"]
        )
        return swept, applied

    def revert(self) -> None:
        self._cal._write_gains(
            self.servos,
            self._original["position"],
            self._original["speed"],
            self._original["integral"],
        )


class InertiaRatioAdapter:
    """SERVO_SWEEP_INERTIA: sweeps C00.06 load inertia ratio."""

    ADDR = INERTIA_RATIO_ADDR

    def __init__(
        self,
        calibration: "ServoCalibration",
        servos: list[str],
        tag: str,
        original: int,
    ):
        self._cal = calibration
        self.servos = servos
        self.tag = tag
        self.original = original

    def step_name(self, value: int) -> str:
        return "%s_r%d" % (self.tag, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        return "inertia step %d/%d: C00.06 ratio %d%% on %s" % (
            i + 1,
            total,
            value,
            ", ".join(servos),
        )

    def _write(self, value: int) -> None:
        with servo_param.suppress_write_log():
            self._cal.gcode.run_script_from_command(
                "\n".join(
                    "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
                    % (servo, self.ADDR, value)
                    for servo in self.servos
                )
            )

    def apply(self, value: int) -> ApplyResult:
        self._write(value)
        swept = {"inertia_ratio": value}
        applied = [_applied(servo, self.ADDR, value) for servo in self.servos]
        return swept, applied

    def revert(self) -> None:
        self._write(self.original)


class MotionAccelAdapter:
    """SERVO_SWEEP_ACCEL: no SDO write, varies the stroke plan's accel."""

    def __init__(self, tag: str):
        self.tag = tag

    def step_name(self, value: int) -> str:
        return "%s_a%d" % (self.tag, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        return "accel step %d/%d: %d mm/s^2 on %s" % (
            i + 1,
            total,
            value,
            ", ".join(servos),
        )

    def apply(self, value: int) -> ApplyResult:
        return {"accel": value}, []

    def revert(self) -> None:
        pass


class DynamicsModelAdapter:
    """SERVO_REFINE_DYNAMICS: streams a scaled copy of the baseline dynamics
    model into the running endpoint per step; revert re-sends the baseline
    (the message is an idempotent full replacement)."""

    def __init__(
        self,
        engine: Any,
        handle: int,
        baseline: dict[str, Any],
        scale_fn: Callable[[dict[str, Any], float], dict[str, Any]],
        label: str,
        tag: str,
    ):
        self._engine = engine
        self._handle = handle
        self.baseline = baseline
        self._scale_fn = scale_fn
        self.label = label
        self.tag = tag
        self.applied = False

    def step_name(self, scale: float) -> str:
        return "%s_%s_s%04d" % (self.tag, self.label, round(scale * 1000))

    def describe(
        self, i: int, scale: float, total: int, servos: list[str]
    ) -> str:
        return "dynamics %s eval %d: scale %.4f on %s" % (
            self.label,
            i + 1,
            scale,
            ", ".join(servos),
        )

    def scaled(self, scale: float) -> dict[str, Any]:
        return self._scale_fn(self.baseline, scale)

    def apply(self, scale: float) -> ApplyResult:
        self._send(self.scaled(scale))
        self.applied = True
        return {"scale": scale}, []

    def revert(self) -> None:
        self._send(self.baseline)

    def _send(self, profile: dict[str, Any]) -> None:
        self._engine.set_dynamics_model(
            self._handle,
            [float(f) for row in profile["frame"] for f in row],
            [float(v) for v in profile["mass"]],
            [float(v) for v in profile["viscous"]],
            [float(v) for v in profile["coulomb"]],
        )


class SweepEngine:
    """for each value: adapter.apply -> capture -> run strokes -> capture."""

    def __init__(self, calibration: "ServoCalibration"):
        self._cal = calibration

    def run_one(
        self,
        adapter: Any,
        i: int,
        value: Any,
        total: int,
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        accel_chip: Any = None,
        accel_chip_name: str | None = None,
    ) -> SweepStep:
        name = adapter.step_name(value)
        swept, applied = adapter.apply(value)
        gcmd.respond_info(adapter.describe(i, value, total, servos))
        self._cal._start_capture(name, servos)
        aclient = (
            None if accel_chip is None else accel_chip.start_internal_client()
        )
        try:
            run_step(value)
            self._cal._stop_capture()
        finally:
            if aclient is not None:
                aclient.finish_measurements()
        step = SweepStep(name, swept, applied)
        if aclient is not None:
            assert accel_chip_name is not None, (
                "accel client exists without a chip name"
            )
            accel_path = self._cal._write_accel_csv(
                gcmd, aclient, accel_chip_name, name
            )
            step.accel = os.path.basename(accel_path)
        self._cal._on_step_complete(step)
        return step

    def run(
        self,
        adapter: Any,
        values: list[Any],
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        accel_chip: Any = None,
        accel_chip_name: str | None = None,
    ) -> list[SweepStep]:
        return [
            self.run_one(
                adapter,
                i,
                value,
                len(values),
                servos,
                run_step,
                gcmd,
                accel_chip,
                accel_chip_name,
            )
            for i, value in enumerate(values)
        ]


COARSE_GAINS = {"position": 400, "speed": 250, "integral": 3184}


@dataclass
class AutotuneContext:
    """State threaded through SERVO_AUTOTUNE's stage list - one instance per
    invocation, mutated in place as each stage records what it found."""

    gcmd: Any
    axis: str
    apply: bool
    torque_nm: float | None
    inertia_kgm2: float | None
    speed_gains: str | None
    dwell_ms: int
    baseline_run: ExperimentRun | None = None
    baseline_results: dict[str, Any] | None = None
    recommended_ratio: int | None = None

    def overrides(self, **extra: Any) -> dict[str, Any]:
        merged: dict[str, Any] = {"AXIS": self.axis, "DWELL_MS": self.dwell_ms}
        merged.update(extra)
        return merged

    def gcmd_for(self, **extra: Any) -> Any:
        return _OverrideGcmd(self.gcmd, self.overrides(**extra))


class AutotuneStage:
    name = "unnamed"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        raise NotImplementedError


class BaselineTrackingStage(AutotuneStage):
    name = "baseline"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        run, results = cal._measure_tracking(
            ctx.gcmd_for(), ctx.axis, "autotune_baseline"
        )
        ctx.baseline_run = run
        ctx.baseline_results = results
        return {"outcome": "ran", "run_dir": run.run_dir}


class InertiaRatioIdentifyStage(AutotuneStage):
    name = "inertia_ratio"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        run, text, _out_path = cal._run_fit(
            ctx.gcmd_for(), "autotune_inertia", ctx.torque_nm, ctx.inertia_kgm2
        )
        ratio = _parse_c0006_recommendation(text)
        if ratio is None:
            raise ctx.gcmd.error(
                "SERVO_AUTOTUNE: aborting at stage %r (run %s): could not "
                "parse a C00.06 recommendation from servo-cal fit output"
                % (self.name, run.run_dir)
            )
        ctx.recommended_ratio = ratio
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "recommended_ratio": ratio,
        }


class ApplyInertiaRatioStage(AutotuneStage):
    name = "apply_inertia_ratio"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        servos = cal._servos(ctx.gcmd, ctx.axis)
        if not ctx.apply:
            return {
                "outcome": "would_run",
                "ratio": ctx.recommended_ratio,
                "servos": servos,
            }
        assert ctx.recommended_ratio is not None
        applied = [
            _applied(s, INERTIA_RATIO_ADDR, ctx.recommended_ratio)
            for s in servos
        ]
        cal._issue_apply_writes(ctx.gcmd, applied)
        return {
            "outcome": "ran",
            "ratio": ctx.recommended_ratio,
            "servos": servos,
        }


class CoarseGainsStage(AutotuneStage):
    name = "coarse_gains"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        if not ctx.apply:
            return {"outcome": "would_run", "gains": COARSE_GAINS}
        cal.cmd_SERVO_APPLY_GAINS(ctx.gcmd_for())
        return {"outcome": "ran", "gains": COARSE_GAINS}


class GainSweepStage(AutotuneStage):
    name = "gain_sweep"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        extra: dict[str, Any] = {
            "TAG": "autotune_gain",
            "APPLY": 1 if ctx.apply else 0,
        }
        if ctx.speed_gains is not None:
            extra["SPEED_GAINS"] = ctx.speed_gains
        cal.cmd_SERVO_CALIBRATE_GAINS(ctx.gcmd_for(**extra))
        run, results = cal._last_sweep_run, cal._last_sweep_results
        assert run is not None and results is not None
        verdict = cal._check_clean_verdict(
            ctx.gcmd, self.name, run, results, require_step=ctx.apply
        )
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "recommended_step": verdict.get("recommended_step"),
        }


class RefineGainStage(AutotuneStage):
    name = "refine_gain"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        if not ctx.apply:
            return {
                "outcome": "would_run",
                "detail": "refine PARAM=speed around the gain-sweep winner",
            }
        extra = {"PARAM": "speed", "TAG": "autotune_refine", "APPLY": 1}
        cal.cmd_SERVO_REFINE_GAIN(ctx.gcmd_for(**extra))
        run, results = cal._last_sweep_run, cal._last_sweep_results
        assert run is not None and results is not None
        verdict = cal._check_clean_verdict(
            ctx.gcmd, self.name, run, results, require_step=True
        )
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "recommended_step": verdict.get("recommended_step"),
        }


class FitDynamicsStage(AutotuneStage):
    name = "fit_dynamics"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        if not ctx.apply:
            return {
                "outcome": "would_run",
                "detail": "fit dynamics at the final tuned gains",
            }
        run, _text, out_path = cal._run_fit(
            ctx.gcmd_for(), "autotune_dynamics", ctx.torque_nm, ctx.inertia_kgm2
        )
        return {"outcome": "ran", "run_dir": run.run_dir, "profile": out_path}


class VerifyStage(AutotuneStage):
    name = "verify"

    def run(
        self, cal: "ServoCalibration", ctx: AutotuneContext
    ) -> dict[str, Any]:
        if not ctx.apply:
            return {
                "outcome": "skipped",
                "reason": "dry run - nothing was applied",
            }
        assert ctx.baseline_run is not None and ctx.baseline_results is not None
        run, results = cal._measure_tracking(
            ctx.gcmd_for(), ctx.axis, "autotune_verify"
        )
        base_name = ctx.baseline_results["steps"][0]["name"]
        final_name = results["steps"][0]["name"]
        base_ferr, _base_overshoot = cal._step_headline(
            ctx.baseline_results, base_name
        )
        final_ferr, _final_overshoot = cal._step_headline(results, final_name)
        if base_ferr > 0.0:
            pct = 100.0 * (final_ferr - base_ferr) / base_ferr
            if pct > 20.0:
                raise ctx.gcmd.error(
                    "SERVO_AUTOTUNE: aborting at stage 'verify' (run %s): "
                    "ferr peak regressed %.0f%% vs baseline (run %s): "
                    "%.0f -> %.0f counts"
                    % (
                        run.run_dir,
                        pct,
                        ctx.baseline_run.run_dir,
                        base_ferr,
                        final_ferr,
                    )
                )
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "baseline_ferr_peak": base_ferr,
            "final_ferr_peak": final_ferr,
        }


AUTOTUNE_STAGES: tuple[AutotuneStage, ...] = (
    BaselineTrackingStage(),
    InertiaRatioIdentifyStage(),
    ApplyInertiaRatioStage(),
    CoarseGainsStage(),
    GainSweepStage(),
    RefineGainStage(),
    FitDynamicsStage(),
    VerifyStage(),
)


class ServoCalibration:
    def __init__(self, config: Any):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.servos = config.getlist("servos", ["stepper_x", "stepper_y"])
        self.rated_torque_nm = config.getfloat(
            "rated_torque_nm", None, above=0.0
        )
        self.rotor_inertia_kgm2 = config.getfloat(
            "rotor_inertia_kgm2", None, above=0.0
        )
        self.bounds: servo_strokes.Bounds = {
            "X": (
                config.getfloat("x_start", 20.0),
                config.getfloat("x_end", 200.0),
            ),
            "Y": (
                config.getfloat("y_start", 20.0),
                config.getfloat("y_end", 200.0),
            ),
        }
        self.accels = config.getfloatlist("accels", [5000.0, 10000.0, 20000.0])
        self.speeds = config.getfloatlist("speeds", [100.0, 400.0])
        self.iterations = config.getint("iterations", 3, minval=1)
        self.accel_chip_name = config.get("accel_chip", None)
        self.dwell_ms = config.getint("dwell_ms", 700, minval=0)
        self.travel_speed = config.getfloat("travel_speed", 100.0, above=0.0)
        self.captures_root = config.get("captures_root", DEFAULT_CAPTURES_ROOT)
        self.dynamics_dir = os.path.expanduser(DEFAULT_DYNAMICS_DIR)
        self.servo_cal_binary = config.get(
            "servo_cal_binary",
            os.path.join(REPO_ROOT, "rust", "target", "snapshot", "servo-cal"),
        )
        self.journal_params = self._parse_journal_params(config)
        self._active_run: ExperimentRun | None = None
        self._capture_sync_loss: (
            tuple[str, list[str], dict[str, int]] | None
        ) = None
        self._last_sweep_run: ExperimentRun | None = None
        self._last_sweep_results: dict[str, Any] | None = None
        self._engine = SweepEngine(self)
        for name in (
            "SERVO_MEASURE_TRACKING",
            "SERVO_MEASURE_DIFFERENTIAL",
            "SERVO_DIFF_DAMPER",
            "SERVO_DIFF_TRIM",
            "SERVO_MEASURE_STRAIN_MAP",
            "SERVO_MEASURE_STRAIN_RESPONSE",
            "SERVO_STRAIN_COMP_TUNE",
            "SERVO_MEASURE_INERTIA",
            "SERVO_FIT_DYNAMICS",
            "SERVO_CALIBRATE_INERTIA_RATIO",
            "SERVO_SHOW_TUNING",
            "SERVO_SET_INERTIA_RATIO",
            "SERVO_APPLY_GAINS",
            "SERVO_CALIBRATE_GAINS",
            "SERVO_GAIN_LADDER",
            "SERVO_REFINE_GAIN",
            "SERVO_REFINE_DYNAMICS",
            "SERVO_HARVEST_NOTCHES",
            "SERVO_SWEEP_INERTIA",
            "SERVO_SWEEP_ACCEL",
            "SERVO_SET_STIFFNESS",
            "SERVO_AUTOTUNE",
        ):
            self.gcode.register_command(
                name,
                getattr(self, "cmd_" + name),
                desc=getattr(self, "cmd_" + name + "_help"),
            )

    def _kin(self) -> Any:
        return self.printer.lookup_object("toolhead").get_kinematics()

    def _parse_journal_params(
        self, config: Any
    ) -> list[tuple[str, str | None]]:
        entries: list[tuple[str, str | None]] = []
        for raw in config.getlist("journal_params", []):
            addr, _sep, type_token = raw.partition(":")
            addr = addr.strip()
            type_token = type_token.strip() or None
            if (
                type_token is not None
                and type_token not in servo_param.TYPE_TOKENS
            ):
                raise config.error(
                    "[servo_calibration] journal_params: unknown type %r "
                    "(use u8/u16/u32/i8/i16/i32)" % (type_token,)
                )
            entries.append((addr, type_token))
        return entries

    def _servo_capture(self) -> Any:
        return self.printer.lookup_object("servo_capture")

    def _run_dir(self, tag: str) -> tuple[str, str]:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        root = os.path.expanduser(self.captures_root)
        run_dir = os.path.join(root, "%s_%s" % (tag, stamp))
        os.makedirs(run_dir, exist_ok=True)
        return run_dir, stamp

    def _resolve_motor(self, servo: str) -> Any:
        from . import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        return motor

    def _motor_manifest(self, motor: Any) -> dict[str, Any]:
        return {
            "name": motor.get_motor_name(),
            "invert": motor.get_invert_direction(),
            "rotation_distance": motor.get_rotation_distance(),
            "counts_per_mm": motor.get_counts_per_mm(),
        }

    def _ff_lead_cycles(self, gcmd: Any, motors: list[Any]) -> int:
        leads = {getattr(m, "ff_lead_cycles", 0) for m in motors}
        if len(leads) > 1:
            raise gcmd.error(
                "motors disagree on ff_lead_cycles (%s); the analyzer "
                "needs a single per-run value" % (sorted(leads),)
            )
        return leads.pop() if leads else 0

    def _belts(self, rails: list[Any] | None) -> str | None:
        if not rails:
            return None
        return ",".join(
            "+".join(
                "%s:%d"
                % (
                    m.get_motor_name(),
                    -1 if m.get_invert_direction() else 1,
                )
                for m in servo_strokes.rail_motors_in_slot_order(r)
            )
            for r in rails
        )

    def _read_journal(
        self, servo: str, addr: str, type_token: str | None
    ) -> int:
        node, slot = self._resolve_node_slot(servo)
        index, subindex = servo_param.parse_address(addr)
        size, raw = servo_param.read_param(
            self.printer, node, slot, index, subindex
        )
        if type_token is not None:
            return servo_param.decode_typed(raw, size, type_token)
        return raw

    def _ambient(self, gcmd: Any, servos: list[str]) -> dict[str, Any]:
        journal: dict[str, dict[str, int]] = {}
        for servo in servos:
            readings: dict[str, int] = {}
            for addr, type_token in self.journal_params:
                try:
                    readings[addr] = self._read_journal(servo, addr, type_token)
                except (RuntimeError, ValueError) as e:
                    raise gcmd.error(
                        "journal_params readback failed for %s %s: %s"
                        % (servo, addr, e)
                    )
            journal[servo] = readings
        return {
            "journal_params": journal,
            "notches": {
                servo: self._notch_state(gcmd, servo) for servo in servos
            },
            "param_writes_since_last_run": servo_param.drain_param_writes(),
        }

    def _begin_run(
        self,
        gcmd: Any,
        experiment: str,
        tag: str,
        axis: str,
        servos: list[str],
        stroke_plan: dict[str, Any],
        belts_rails: list[Any] | None = None,
    ) -> ExperimentRun:
        run_dir, stamp = self._run_dir(tag)
        kin = self._kin()
        motors = [self._resolve_motor(s) for s in servos]
        manifest = {
            "version": 1,
            "experiment": experiment,
            "command": gcmd.get_commandline(),
            "tag": tag,
            "created_utc": _utc_now(),
            "axis": axis,
            "kinematics": getattr(kin, "kind", None),
            "git_rev": _git_rev(),
            "session_id": structured_log.get_session(),
            "stroke_plan": stroke_plan,
            "motors": [self._motor_manifest(m) for m in motors],
            "ff_lead_cycles": self._ff_lead_cycles(gcmd, motors),
            "belts": self._belts(belts_rails),
            "steps": [],
            "ambient": self._ambient(gcmd, servos),
        }
        run = ExperimentRun(run_dir, stamp, manifest)
        run.write()
        structured_log.event(
            "calibration",
            "run_start",
            run_dir=run_dir,
            experiment=experiment,
            tag=tag,
            axis=axis,
        )
        self._active_run = run
        return run

    def _on_step_complete(self, step: SweepStep) -> None:
        if self._active_run is not None:
            self._active_run.record_step(step)

    def _servo_cal(self, gcmd: Any) -> str:
        if not os.path.exists(self.servo_cal_binary):
            raise gcmd.error(
                "servo-cal binary not found at %s - build it with: "
                "cargo build --profile snapshot -p servo-ident"
                % (self.servo_cal_binary,)
            )
        return self.servo_cal_binary

    def _read_results(self, gcmd: Any, run_dir: str) -> dict[str, Any]:
        path = os.path.join(run_dir, "results.json")
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "failed to read results.json from %s: %s" % (run_dir, e)
            )

    def _run_analyze(
        self, gcmd: Any, run: ExperimentRun, incremental: bool = False
    ) -> dict[str, Any]:
        binary = self._servo_cal(gcmd)
        argv = [binary, "analyze", run.run_dir]
        if incremental:
            argv.append("--incremental")
        self._run(gcmd, argv, 120.0)
        return self._read_results(gcmd, run.run_dir)

    def _analyze_and_report(
        self, gcmd: Any, run: ExperimentRun
    ) -> dict[str, Any]:
        results = self._run_analyze(gcmd, run)
        verdict = results.get("verdict") or {}
        step = verdict.get("recommended_step")
        reason = verdict.get("reason") or "no reason given"
        flags = verdict.get("flags") or []
        duration_s = round(time.time() - run.started_s, 3)
        gcmd.respond_info(
            "verdict: %s (%s) | run %s"
            % (step if step else "no step", reason, run.run_dir)
        )
        structured_log.event(
            "calibration",
            "run_done",
            run_dir=run.run_dir,
            recommended_step=step,
            flags=flags,
            duration_s=duration_s,
        )
        return results

    def _step_headline(
        self, results: dict[str, Any], step_name: str
    ) -> tuple[float, float]:
        """(ferr_peak, overshoot) in encoder counts, maxed over every drive
        and move of the named step - the before/after APPLY verification
        reads off this, not the mm-scaled `combined` block, so it works
        identically on a single-drive step and a CoreXY one."""
        for step in results.get("steps") or []:
            if step.get("name") != step_name:
                continue
            ferr_peak = 0.0
            overshoot = 0.0
            for drive in (step.get("drives") or {}).values():
                for move in (drive.get("metrics") or {}).get("moves") or []:
                    ferr_peak = max(ferr_peak, move.get("ferr_peak", 0.0))
                    overshoot = max(overshoot, move.get("overshoot", 0.0))
            return ferr_peak, overshoot
        raise self.printer.command_error(
            "step %r missing from results.json" % (step_name,)
        )

    def _step_metric_mean(
        self,
        gcmd: Any,
        results: dict[str, Any],
        step_name: str,
        metric: str,
    ) -> float:
        """Mean of one per-move metric over the named step's drives - the
        refinement objective, so mean (not max): lower variance under stroke
        noise, and constant per-drive offsets do not move the argmin."""
        for step in results.get("steps") or []:
            if step.get("name") != step_name:
                continue
            step_drives = step.get("drives") or {}
            values = [
                move[metric]
                for drive in step_drives.values()
                for move in (drive.get("metrics") or {}).get("moves") or []
                if metric in move
            ]
            if not values:
                raise gcmd.error(
                    "step %s carries no %r move metrics in results.json"
                    % (step_name, metric)
                )
            return sum(values) / len(values)
        raise gcmd.error("step %r missing from results.json" % (step_name,))

    def _step_flags(self, results: dict[str, Any], step_name: str) -> list[str]:
        for step in results.get("steps") or []:
            if step.get("name") == step_name:
                return list(step.get("flags") or [])
        return []

    def _check_clean_verdict(
        self,
        gcmd: Any,
        stage: str,
        run: ExperimentRun,
        results: dict[str, Any],
        require_step: bool,
    ) -> dict[str, Any]:
        """SERVO_AUTOTUNE's shared abort gate: a null recommendation is only
        fatal when this stage's job is to promote one (require_step); a
        torque/resonance flag on the chosen step is always fatal, dry run
        or not - continuing past a flagged step is unsafe regardless of
        whether anything gets written."""
        verdict = results.get("verdict") or {}
        step_name = verdict.get("recommended_step")
        if require_step and step_name is None:
            raise gcmd.error(
                "SERVO_AUTOTUNE: aborting at stage %r (run %s): no "
                "recommendation - %s"
                % (
                    stage,
                    run.run_dir,
                    verdict.get("reason") or "no reason given",
                )
            )
        if step_name is not None:
            flags = set(verdict.get("flags") or [])
            flags |= set(self._step_flags(results, step_name))
            bad = sorted(flags & VERDICT_ABORT_FLAGS)
            if bad:
                raise gcmd.error(
                    "SERVO_AUTOTUNE: aborting at stage %r (run %s): verdict "
                    "flags %s on step %r" % (stage, run.run_dir, bad, step_name)
                )
        return verdict

    def _issue_apply_writes(
        self, gcmd: Any, applies: list[dict[str, Any]]
    ) -> None:
        if not applies:
            return
        lines = [
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=%s"
            % (a["servo"], a["addr"], a["value"], a["type"])
            for a in applies
        ]
        with servo_param.suppress_write_log():
            self.gcode.run_script_from_command("\n".join(lines))
        for a in applies:
            node, slot = self._resolve_node_slot(a["servo"])
            index, subindex = servo_param.parse_address(a["addr"])
            size, raw = servo_param.read_param(
                self.printer, node, slot, index, subindex
            )
            value = servo_param.decode_typed(raw, size, a["type"])
            if value != a["value"]:
                raise gcmd.error(
                    "APPLY readback mismatch on %s %s: wrote %d, read %d"
                    % (a["servo"], a["addr"], a["value"], value)
                )

    def _chosen_swept(
        self, run: ExperimentRun, step_name: str
    ) -> dict[str, Any]:
        for step in run.manifest["steps"]:
            if step["name"] == step_name:
                return step["swept"]
        raise self.printer.command_error(
            "step %r missing from manifest %s" % (step_name, run.manifest_path)
        )

    def _apply_verdict(
        self,
        gcmd: Any,
        run: ExperimentRun,
        results: dict[str, Any],
        axis: str,
    ) -> None:
        verdict = results.get("verdict") or {}
        step_name = verdict.get("recommended_step")
        apply = verdict.get("apply")
        if step_name is None or apply is None:
            raise gcmd.error(
                "APPLY=1: nothing to apply - verdict on run %s: %s"
                % (run.run_dir, verdict.get("reason") or "no reason given")
            )
        self._issue_apply_writes(gcmd, apply)
        before = self._step_headline(results, step_name)
        swept = self._chosen_swept(run, step_name)
        overrides = {"ACCEL": swept["accel"]} if "accel" in swept else {}
        verify_gcmd = _OverrideGcmd(gcmd, overrides) if overrides else gcmd
        verify_run, verify_results = self._measure_tracking(
            verify_gcmd, axis, "verify_%s" % (run.stamp,)
        )
        verify_step_name = verify_results["steps"][0]["name"]
        after = self._step_headline(verify_results, verify_step_name)
        gcmd.respond_info(
            "APPLY verified (%s): ferr_peak %.0f -> %.0f counts, "
            "overshoot %.0f -> %.0f counts | sweep %s -> verify %s"
            % (
                step_name,
                before[0],
                after[0],
                before[1],
                after[1],
                run.run_dir,
                verify_run.run_dir,
            )
        )

    @overload
    def _floats(self, text: str) -> list[float]: ...
    @overload
    def _floats(self, text: None) -> None: ...
    def _floats(self, text: str | None) -> list[float] | None:
        return servo_strokes.parse_floats(text)

    def _motor(
        self, gcmd: Any, required: bool
    ) -> tuple[float | None, float | None]:
        torque = gcmd.get_float("TORQUE_NM", self.rated_torque_nm)
        inertia = gcmd.get_float("INERTIA_KGM2", self.rotor_inertia_kgm2)
        if required:
            if torque is None:
                raise gcmd.error(
                    "TORQUE_NM required - set rated_torque_nm in "
                    "[servo_calibration] or pass TORQUE_NM= (motor rated torque, N*m)"
                )
            if inertia is None:
                raise gcmd.error(
                    "INERTIA_KGM2 required - set rotor_inertia_kgm2 in "
                    "[servo_calibration] or pass INERTIA_KGM2= (rotor inertia, kg*m^2)"
                )
        elif (torque is None) != (inertia is None):
            raise gcmd.error(
                "TORQUE_NM and INERTIA_KGM2 must be given together"
            )
        return torque, inertia

    def _servo(self, gcmd: Any) -> str:
        default = self.servos[0] if len(self.servos) == 1 else None
        servo = gcmd.get("SERVO", default)
        if servo is None:
            raise gcmd.error(
                "SERVO= is required - name the drive explicitly (e.g. SERVO=motor_a)"
            )
        return servo

    def _servos(self, gcmd: Any, axis: str | None = None) -> list[str]:
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            return [s.strip() for s in servo.split(",") if s.strip()]
        if axis is None:
            axis = gcmd.get("AXIS", None)
        if axis is not None:
            return servo_strokes.axis_servos(gcmd, self._kin(), axis.upper())
        if len(self.servos) == 1:
            return [self.servos[0]]
        raise gcmd.error(
            "AXIS= or SERVO= is required (SERVO= accepts a comma list)"
        )

    def _reject_corexy_only_params(self, gcmd: Any) -> None:
        bad = [
            p
            for p in ("SERVOS", "X_START", "X_END", "Y_START", "Y_END")
            if gcmd.get(p, None) is not None
        ]
        if bad:
            raise gcmd.error(
                "%s require coupled_xy kinematics - the active kinematics "
                "is not CoreXY" % (", ".join(bad),)
            )

    def _strokes(
        self,
        axis: str,
        start: float,
        end: float,
        speed: float,
        accel: float,
        iterations: int,
        dwell: int,
    ) -> None:
        servo_strokes.emit_strokes(
            self.gcode,
            lambda u: "%s%.3f" % (axis, u),
            start,
            end,
            1.0,
            speed,
            accel,
            iterations,
            dwell,
        )

    def _goto_xy(self, x: float, y: float, dwell: int) -> None:
        servo_strokes.goto_xy(self.gcode, self.travel_speed, x, y, dwell)

    def _prep(self, axis: str, dwell: int) -> None:
        servo_strokes.prep(self.printer, self.gcode, axis, dwell)

    def _restore(self) -> None:
        self.gcode.run_script_from_command("RESET_VELOCITY_LIMIT")

    def _start_capture(self, name: str, servos: list[str]) -> None:
        if self._active_run is None:
            raise self.printer.command_error(
                "servo capture requested without an active experiment run"
            )
        self._capture_sync_loss = (
            name,
            list(servos),
            self._sync_loss_counts(servos),
        )
        self._servo_capture().start_capture_to(
            self._active_run.step_scap(name), servos
        )

    def _stop_capture(self) -> None:
        self._servo_capture().stop_capture()
        self._check_sync_loss()

    def _sync_loss_counts(self, servos: list[str]) -> dict[str, int]:
        """C13.04 per drive - the drive's own EtherCAT sync loss counter.
        The drive silently tolerates up to C13.02 (default 8) consecutive
        lost/late sync events before faulting, so this counter is the only
        way to see the tolerated ones. A failed read aborts the command
        (not the printer): the counter is diagnostics, and a CoE abort here
        means the drive does not expose it where expected."""
        counts = {}
        for servo in servos:
            try:
                counts[servo] = self._read_param(servo, SYNC_LOSS_COUNT_ADDR)
            except Exception as e:
                raise self.printer.command_error(
                    "reading EtherCAT sync loss counter C13.04 (%s) failed "
                    "for %s: %s" % (SYNC_LOSS_COUNT_ADDR, servo, e)
                )
        return counts

    def _check_sync_loss(self) -> None:
        if self._capture_sync_loss is None:
            return
        name, servos, before = self._capture_sync_loss
        self._capture_sync_loss = None
        after = self._sync_loss_counts(servos)
        deltas = {
            servo: (after[servo] - before[servo]) & 0xFFFF for servo in servos
        }
        hits = {servo: d for servo, d in deltas.items() if d}
        if not hits:
            return
        detail = ", ".join(
            "%s +%d" % (servo, d) for servo, d in sorted(hits.items())
        )
        self.gcode.respond_info(
            "WARNING step %s: EtherCAT sync loss count (C13.04) incremented "
            "during the capture: %s. The drive tolerated lost/late sync "
            "cycles without faulting (it only faults after C13.02 "
            "consecutive losses) - this step's tracking metrics are "
            "contaminated." % (name, detail)
        )
        structured_log.event(
            "calibration",
            "sync_loss",
            step=name,
            drives=detail,
            total=sum(hits.values()),
        )

    def _accel_chip(self, gcmd: Any) -> tuple[Any, str | None]:
        chip_name = gcmd.get("ACCEL_CHIP", self.accel_chip_name)
        if chip_name is None:
            return None, None
        return self.printer.lookup_object(chip_name.strip()), chip_name

    def _write_accel_csv(
        self, gcmd: Any, aclient: Any, chip_name: str, step_name: str
    ) -> str:
        if not aclient.has_valid_samples():
            raise gcmd.error(
                "accelerometer %r measured no data for step %s"
                % (chip_name, step_name)
            )
        assert self._active_run is not None, "accel CSV written outside a run"
        path = self._active_run.step_accel_csv(step_name)
        with open(path, "w") as f:
            f.write("#time,accel_x,accel_y,accel_z\n")
            for t, accel_x, accel_y, accel_z in aclient.get_samples():
                f.write(
                    "%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z)
                )
        gcmd.respond_info("Accelerometer data written to %s" % (path,))
        return path

    def _run(self, gcmd: Any, argv: list[str], timeout: float) -> str:
        reactor = self.printer.get_reactor()
        label = os.path.basename(argv[0])
        try:
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=lambda: os.nice(10),
            )
        except Exception:
            logging.exception("servo_calibration: failed to launch %s", label)
            raise gcmd.error("Error launching %s" % (label,))
        assert proc.stdout is not None, "Popen was given stdout=PIPE"
        fd = proc.stdout.fileno()
        buf = [""]
        output: list[str] = []

        def emit(data: str) -> None:
            buf[0] += data
            if "\n" in buf[0]:
                head, _, buf[0] = buf[0].rpartition("\n")
                gcmd.respond_info(head)
                output.append(head)

        def on_readable(eventtime: float) -> None:
            try:
                emit(os.read(fd, 4096).decode())
            except Exception:
                pass

        hdl = reactor.register_fd(fd, on_readable)
        gcmd.respond_info("Running %s ..." % (label,))
        eventtime = reactor.monotonic()
        endtime = eventtime + timeout
        complete = False
        while eventtime < endtime:
            eventtime = reactor.pause(eventtime + 0.05)
            if proc.poll() is not None:
                complete = True
                break
        reactor.unregister_fd(hdl)
        if not complete:
            proc.terminate()
            raise gcmd.error("%s timed out after %.0fs" % (label, timeout))
        while True:
            data = os.read(fd, 4096).decode()
            if not data:
                break
            emit(data)
        if buf[0]:
            gcmd.respond_info(buf[0])
            output.append(buf[0])
        if proc.returncode:
            raise gcmd.error(
                "%s exited with code %d" % (label, proc.returncode)
            )
        return "\n".join(output)

    cmd_SERVO_MEASURE_TRACKING_help = (
        "Single accel/speed stroke run with capture - the before/after check "
        "for any tuning change. AXIS=X/Y records every motor driving the axis "
        "(both lanes on CoreXY) into a run directory that servo-cal analyzes "
        "into results.json (per-motor + combined tracking metrics). "
        "AXIS=A/B run a CoreXY 45-degree diagonal that exercises one motor "
        "alone (A=+45 x&y up, motor A; B=-45 x up y down, motor B); SPEED is "
        "the toolhead feedrate, so belt speed is sqrt(2)x SPEED on a diagonal. "
        "Params AXIS START END SPEED ACCEL ITERATIONS DWELL_MS NAME"
    )

    def _measure_tracking(
        self, gcmd: Any, axis: str, name: str
    ) -> tuple[ExperimentRun, dict[str, Any]]:
        """The SERVO_MEASURE_TRACKING body - shared with APPLY=1's
        verification stroke and every SERVO_AUTOTUNE tracking stage."""
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        servos = plan.servos
        rails = plan.rails
        belts_rails = (
            rails
            if not plan.diagonal and len(rails) == 2 and axis in ("X", "Y")
            else None
        )
        stroke_plan = {
            "start": plan.start,
            "end": plan.end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd, "tracking", name, axis, servos, stroke_plan, belts_rails
        )
        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            self._start_capture(name, servos)
            servo_strokes.emit_strokes(
                self.gcode,
                plan.coord,
                plan.start,
                plan.end,
                plan.th_per_unit,
                speed,
                accel,
                iterations,
                dwell,
            )
            self._stop_capture()
            self._restore()
            run.record_step(SweepStep(name, {}, []))
            results = self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None
        return run, results

    def cmd_SERVO_MEASURE_TRACKING(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        name = gcmd.get("NAME", "track")
        self._measure_tracking(gcmd, axis, name)

    MAX_DIFFERENTIAL_AMPLITUDE_MM = 0.5
    MAX_BUZZ_FREQ_HZ = 2000.0
    MAX_BUZZ_DURATION_S = 300.0

    cmd_SERVO_MEASURE_DIFFERENTIAL_help = (
        "Anti-phase chirp on one AWD belt pair via the engine buzz "
        "generator - the carriage holds still while the two drives strain "
        "the belt against each other, so the capture isolates the "
        "rotor-vs-rotor (differential) modes. servo-cal analyzes the run "
        "into a differential FRF with mode frequency, damping and "
        "coherence. Belt strain is twice AMPLITUDE. Params BELT=A|B "
        "FREQ_START FREQ_END HZ_PER_SEC DURATION AMPLITUDE RAMP DWELL_MS "
        "NAME"
    )

    def _belt_pair(self, gcmd, belt, cmd_name):
        layout = servo_strokes.corexy_fit_layout(gcmd, self._kin())
        if layout["pairs"] is None:
            raise gcmd.error(
                "%s needs two drives per belt "
                "(AWD); this printer has one drive per belt" % (cmd_name,)
            )
        pair_names = layout["pairs"].split(";")["AB".index(belt)].split(",")
        motors = [self._resolve_motor(name) for name in pair_names]
        node = self.printer.lookup_object(
            "ethercat_node " + motors[0].get_node_name()
        )
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "belt %s drives have no live EtherCAT engine handle "
                "(node not claimed)" % (belt,)
            )
        slots = [node.get_slot_for_motor(m.get_motor_name()) for m in motors]
        return pair_names, motors, handle, slots

    def cmd_SERVO_MEASURE_DIFFERENTIAL(self, gcmd):
        belt = gcmd.get("BELT", "A").upper()
        if belt not in ("A", "B"):
            raise gcmd.error("BELT must be A or B (got %r)" % (belt,))
        pair_names, motors, handle, slots = self._belt_pair(
            gcmd, belt, "SERVO_MEASURE_DIFFERENTIAL"
        )
        freq_start = gcmd.get_float("FREQ_START", 20.0, above=0.0)
        freq_end = gcmd.get_float("FREQ_END", 250.0, above=0.0)
        if max(freq_start, freq_end) > self.MAX_BUZZ_FREQ_HZ:
            raise gcmd.error(
                "buzz frequencies must stay at or below %.0f Hz"
                % (self.MAX_BUZZ_FREQ_HZ,)
            )
        amplitude = gcmd.get_float("AMPLITUDE", 0.05, above=0.0)
        if amplitude > self.MAX_DIFFERENTIAL_AMPLITUDE_MM:
            raise gcmd.error(
                "AMPLITUDE %.3f mm exceeds the %.1f mm differential ceiling "
                "(belt strain between the pair is twice the amplitude)"
                % (amplitude, self.MAX_DIFFERENTIAL_AMPLITUDE_MM)
            )
        hz_per_sec = gcmd.get_float("HZ_PER_SEC", 5.0, above=0.0)
        duration = gcmd.get_float("DURATION", 0.0, minval=0.0)
        if duration <= 0.0:
            duration = max(abs(freq_end - freq_start) / hz_per_sec, 0.5)
        if duration > self.MAX_BUZZ_DURATION_S:
            raise gcmd.error(
                "sweep duration %.0f s exceeds the %.0f s buzz ceiling; "
                "raise HZ_PER_SEC or narrow the frequency band"
                % (duration, self.MAX_BUZZ_DURATION_S)
            )
        ramp = gcmd.get_float(
            "RAMP",
            min(0.1 * duration, 3.0 / min(freq_start, freq_end)),
            above=0.0,
        )
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "diff")
        engine = self.printer.lookup_object("motion_engine")
        stroke_plan = {
            "belt": belt,
            "freq_start": freq_start,
            "freq_end": freq_end,
            "hz_per_sec": hz_per_sec,
            "duration": duration,
            "ramp": ramp,
            "amplitude": amplitude,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd, "differential", name, belt, pair_names, stroke_plan
        )
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            gcmd.respond_info(
                "differential sweep on belt %s (%s anti-phase %s): "
                "%.1f->%.1f Hz over %.1f s, amplitude %.3f mm"
                % (
                    belt,
                    pair_names[0],
                    pair_names[1],
                    freq_start,
                    freq_end,
                    duration,
                    amplitude,
                )
            )
            self._start_capture(name, pair_names)
            try:
                engine.resonance_buzz(
                    handle,
                    (1 << slots[0]) | (1 << slots[1]),
                    1 << slots[1],
                    int(round(freq_start * 1000.0)),
                    int(round(freq_end * 1000.0)),
                    int(round(amplitude * 1e6)),
                    int(round(duration * 1000.0)),
                    int(round(ramp * 1000.0)),
                )
                reactor = self.printer.get_reactor()
                reactor.pause(reactor.monotonic() + duration + 0.2)
            finally:
                self._stop_capture()
            run.record_step(SweepStep(name, {}, []))
            self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None

    MAX_DAMPER_CLAMP_TENTHS = 300.0
    MAX_DAMPER_LEAD_US = 5000.0

    cmd_SERVO_DIFF_DAMPER_help = (
        "Arm or disarm the differential belt-pair damper: the engine adds "
        "an antisymmetric torque offset (60B2h) proportional to the "
        "low-passed velocity difference between the two drives of a belt "
        "- a virtual dashpot between the rotors that damps the inter-motor "
        "belt mode at whatever frequency it sits. Zero on synchronized "
        "motion, so it costs no torque during printing. GAIN is in 0.1% "
        "rated torque per mm/s of differential velocity; GAIN=0 disarms. "
        "LEAD_US adds first-order phase lead to compensate the loop's "
        "transport and torque-path lag. Params BELT=A|B|AB GAIN CLAMP "
        "LPF_HZ LEAD_US"
    )

    def cmd_SERVO_DIFF_DAMPER(self, gcmd):
        belts = gcmd.get("BELT", "AB").upper()
        if belts not in ("A", "B", "AB"):
            raise gcmd.error("BELT must be A, B or AB (got %r)" % (belts,))
        gain = gcmd.get_float("GAIN", minval=0.0)
        clamp = gcmd.get_float("CLAMP", 50.0, above=0.0)
        if clamp > self.MAX_DAMPER_CLAMP_TENTHS:
            raise gcmd.error(
                "CLAMP %.0f exceeds the %.0f x0.1%%-rated-torque ceiling"
                % (clamp, self.MAX_DAMPER_CLAMP_TENTHS)
            )
        lpf_hz = gcmd.get_float("LPF_HZ", 300.0, above=0.0)
        lead_us = gcmd.get_float(
            "LEAD_US", 0.0, minval=0.0, maxval=self.MAX_DAMPER_LEAD_US
        )
        engine = self.printer.lookup_object("motion_engine")
        for belt in belts:
            pair_names, _motors, handle, slots = self._belt_pair(
                gcmd, belt, "SERVO_DIFF_DAMPER"
            )
            engine.set_diff_damper(
                handle,
                slots[0],
                slots[1],
                int(round(gain * 1000.0)),
                int(round(clamp)),
                int(round(lpf_hz * 1000.0)),
                int(round(lead_us)),
            )
            if gain > 0.0:
                gcmd.respond_info(
                    "belt %s damper armed (%s vs %s): gain %.3f "
                    "x0.1%%/(mm/s), clamp %.0f x0.1%%, lpf %.0f Hz, "
                    "lead %.0f us"
                    % (
                        belt,
                        pair_names[0],
                        pair_names[1],
                        gain,
                        clamp,
                        lpf_hz,
                        lead_us,
                    )
                )
            else:
                gcmd.respond_info("belt %s damper disarmed" % (belt,))

    MAX_TRIM_GAIN = 2.0
    MAX_TRIM_CLAMP_UM = 500.0
    cmd_SERVO_DIFF_TRIM_help = (
        "Arm or disarm the differential belt-pair trim: the engine "
        "integrates each pair's low-passed differential torque into a "
        "small antisymmetric position offset, continuously nulling the "
        "static fight (homing preload, thermal drift) during motion - the "
        "always-on version of SERVO_SYNC. Loop bandwidth is a few Hz, far "
        "below the belt resonances. GAIN is in mm/s of offset slew per 1% "
        "differential torque; GAIN=0 disarms. CLAMP_UM bounds the offset "
        "(hitting it logs a warning). Params BELT=A|B|AB GAIN CLAMP_UM "
        "LPF_HZ"
    )

    def cmd_SERVO_DIFF_TRIM(self, gcmd):
        belts = gcmd.get("BELT", "AB").upper()
        if belts not in ("A", "B", "AB"):
            raise gcmd.error("BELT must be A, B or AB (got %r)" % (belts,))
        gain = gcmd.get_float("GAIN", minval=0.0, maxval=self.MAX_TRIM_GAIN)
        clamp_um = gcmd.get_float(
            "CLAMP_UM", 150.0, above=0.0, maxval=self.MAX_TRIM_CLAMP_UM
        )
        lpf_hz = gcmd.get_float("LPF_HZ", 25.0, above=0.0)
        engine = self.printer.lookup_object("motion_engine")
        for belt in belts:
            pair_names, _motors, handle, slots = self._belt_pair(
                gcmd, belt, "SERVO_DIFF_TRIM"
            )
            engine.set_diff_trim(
                handle,
                slots[0],
                slots[1],
                int(round(gain * 1e6)),
                int(round(clamp_um)),
                int(round(lpf_hz * 1000.0)),
            )
            if gain > 0.0:
                gcmd.respond_info(
                    "belt %s trim armed (%s vs %s): gain %.3f (mm/s)/%%, "
                    "clamp %.0f um, lpf %.1f Hz"
                    % (
                        belt,
                        pair_names[0],
                        pair_names[1],
                        gain,
                        clamp_um,
                        lpf_hz,
                    )
                )
            else:
                gcmd.respond_info("belt %s trim disarmed" % (belt,))

    STRAIN_MAP_MIN_LINE_SPACING_MM = servo_strain_comp.MIN_LINE_SPACING_MM

    cmd_SERVO_MEASURE_STRAIN_MAP_help = (
        "Raster the bed with slow constant-speed strokes, one capture per "
        "line - the measurement half of the belt strain map. Differential "
        "pair torque vs (x, y) separates trapped preload, pulley/idler "
        "runout (periodic in travel) and geometry (smooth) when the run is "
        "analyzed. Serpentine X sweeps stepped along Y by LINE_SPACING, "
        "then Y sweeps stepped along X; every line strokes forward and "
        "back so friction asymmetry averages out. Before rastering the "
        "carriage parks at the region center and SERVO_SYNC releases the "
        "trapped preload, so every map shares the same zero (SYNC=0 "
        "skips). CoreXY only. Params SPEED (50) ACCEL (1000) LINE_SPACING "
        "(10) X_START X_END Y_START Y_END DWELL_MS TAG SYNC"
    )

    @staticmethod
    def _raster_levels(start: float, end: float, spacing: float) -> list[float]:
        n = max(1, int(round((end - start) / spacing)))
        return [start + (end - start) * i / n for i in range(n + 1)]

    def cmd_SERVO_MEASURE_STRAIN_MAP(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_MAP requires coupled XY (CoreXY) "
                "kinematics - the strain map is a belt-pair measurement"
            )
        x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        spacing = gcmd.get_float(
            "LINE_SPACING", 10.0, minval=self.STRAIN_MAP_MIN_LINE_SPACING_MM
        )
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "strain")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        # The zero point must be reproducible when the map is APPLIED, not
        # just when it is measured: always the center of the configured
        # calibration area, never the (run-specific) raster region.
        zero_x = (self.bounds["X"][0] + self.bounds["X"][1]) / 2.0
        zero_y = (self.bounds["Y"][0] + self.bounds["Y"][1]) / 2.0
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y_start": y_start,
            "y_end": y_end,
            "speed": speed,
            "accel": accel,
            "line_spacing": spacing,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": [zero_x, zero_y],
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_MEASURE_STRAIN_MAP: [servo_sync] is not "
                    "configured - add it so every map shares a preload "
                    "zero, or pass SYNC=0 to raster without one"
                )
            self._goto_xy(zero_x, zero_y, dwell)
            gcmd.respond_info(
                "strain map zero point: SERVO_SYNC at (%.1f, %.1f) — the "
                "calibration area center; repeat there when applying the "
                "map" % (zero_x, zero_y)
            )
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_map",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        lines = [
            ("X", x_start, x_end, "y", level)
            for level in self._raster_levels(y_start, y_end, spacing)
        ] + [
            ("Y", y_start, y_end, "x", level)
            for level in self._raster_levels(x_start, x_end, spacing)
        ]
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            for i, (axis, start, end, fixed_axis, level) in enumerate(lines):
                if axis == "X":
                    self._goto_xy(start, level, dwell)
                else:
                    self._goto_xy(level, start, dwell)
                name = "%sline_%s%03d" % (
                    axis.lower(),
                    fixed_axis,
                    int(round(level)),
                )
                gcmd.respond_info(
                    "strain map line %d/%d: %s sweep at %s=%.1f"
                    % (i + 1, len(lines), axis, fixed_axis.upper(), level)
                )
                self._start_capture(name, servos)
                self._strokes(axis, start, end, speed, accel, 1, dwell)
                self._stop_capture()
                run.record_step(SweepStep(name, {fixed_axis: level}, []))
            self._restore()
            gcmd.respond_info(
                "strain map raster complete: %d lines in %s"
                % (len(lines), run.run_dir)
            )
        finally:
            self._active_run = None

    STRAIN_RESPONSE_STEPS = (0.0, 1.0, -1.0, 2.0, -2.0)
    MAX_STRAIN_STEP_UM = servo_strain_comp.MAX_STRAIN_STEP_UM

    cmd_SERVO_MEASURE_STRAIN_RESPONSE_help = (
        "Measure the belt stiffness matrix in the rolling regime — the one "
        "the strain map and its compensation operate in (a parked belt "
        "reads ~20% stiffer). Strokes ONE X line forward and back while "
        "stepping a constant antisymmetric offset through each pair's "
        "compensation bank; the line's own strain field is identical on "
        "every pass and cancels out of the offset-response slope, so no "
        "baseline raster is needed. Both pairs' responses are captured per "
        "step, so the direct and cross terms come from the same strokes. "
        "The fitted matrix is stored for SERVO_STRAIN_COMP_BUILD. CoreXY "
        "only, needs [servo_strain_comp]. Params SPEED (50) ACCEL (1000) "
        "STEP_UM (50) SETTLE (0.8) Y (area center) X_START X_END DWELL_MS "
        "TAG SYNC"
    )

    def cmd_SERVO_MEASURE_STRAIN_RESPONSE(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_RESPONSE requires coupled XY "
                "(CoreXY) kinematics - the response is a belt-pair "
                "measurement"
            )
        comp = self.printer.lookup_object("servo_strain_comp", None)
        if comp is None:
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_RESPONSE needs [servo_strain_comp] "
                "configured - it drives the compensation bank"
            )
        x_start, x_end, _y_start, _y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        step_um = gcmd.get_float(
            "STEP_UM", 50.0, above=0.0, maxval=self.MAX_STRAIN_STEP_UM
        )
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "strainresp")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        zero_x = (self.bounds["X"][0] + self.bounds["X"][1]) / 2.0
        zero_y = (self.bounds["Y"][0] + self.bounds["Y"][1]) / 2.0
        line_y = gcmd.get_float("Y", zero_y)
        session = comp.begin_constant_offsets(gcmd)
        steps_um = [k * step_um for k in self.STRAIN_RESPONSE_STEPS]
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y": line_y,
            "speed": speed,
            "accel": accel,
            "step_um": step_um,
            "offset_steps_um": steps_um,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": [zero_x, zero_y],
            "response_pairs": session.pair_motor_names(),
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_MEASURE_STRAIN_RESPONSE: [servo_sync] is not "
                    "configured - add it so the line shares the maps' "
                    "preload zero, or pass SYNC=0"
                )
            self._goto_xy(zero_x, zero_y, dwell)
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_response",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        reactor = self.printer.get_reactor()
        total = session.pair_count() * len(steps_um)
        try:
            self._prep("X", dwell)
            self._goto_xy(x_start, line_y, dwell)
            for belt_idx in range(session.pair_count()):
                for step_idx, value_um in enumerate(steps_um):
                    slew_s = session.apply(belt_idx, value_um)
                    reactor.pause(reactor.monotonic() + settle + slew_s)
                    name = "belt%s_step%d" % ("ab"[belt_idx], step_idx)
                    gcmd.respond_info(
                        "strain response %d/%d: belt %s at %+.0f um"
                        % (
                            belt_idx * len(steps_um) + step_idx + 1,
                            total,
                            "AB"[belt_idx],
                            value_um,
                        )
                    )
                    self._start_capture(name, servos)
                    self._strokes("X", x_start, x_end, speed, accel, 1, dwell)
                    self._stop_capture()
                    run.record_step(
                        SweepStep(
                            name,
                            {"belt": float(belt_idx), "offset_um": value_um},
                            [],
                        )
                    )
                slew_s = session.apply(belt_idx, 0.0)
                reactor.pause(reactor.monotonic() + slew_s)
            self._restore()
        finally:
            session.clear()
            self._active_run = None
        comp.fit_strain_response(gcmd, run.run_dir)

    TUNE_MAX_ITERS = 5

    cmd_SERVO_STRAIN_COMP_TUNE_help = (
        "Converge the strain map's stiffness matrix against reality: "
        "rebuild the FULL map from RUN=<baseline raster> at the trial "
        "matrix (starting from the probe's values or "
        "STIFFNESS_A/B+CROSS_AB/BA), enable it, sweep an X and a Y "
        "verification line, and refit every belt's direct AND cross "
        "stiffness from the measured response to the applied offsets - "
        "the two sweeps swap the belts' roles, so all four matrix "
        "elements are measured independently. Repeat until the measured "
        "matrix reproduces the applied one (per element, within TOL of "
        "the row's direct stiffness). Each iteration costs two line "
        "sweeps, not a raster, and the converged map is already on disk "
        "and enabled - it is the same full-bed map that was being "
        "verified. The residuals cover only the smooth elastic field: "
        "direction-dependent friction asymmetry and short-wavelength "
        "ripple are invisible to a position-keyed map. Fails loudly if "
        "MAX_ITERS passes don't converge. CoreXY only, needs "
        "[servo_strain_comp] and [servo_sync]. Params RUN (required) "
        "SPACING TOL (0.05) MAX_ITERS (5) Y (map zero) X (map zero) "
        "SPEED (50) ACCEL (1000) SETTLE (0.8) DWELL_MS TAG SYNC"
    )

    def cmd_SERVO_STRAIN_COMP_TUNE(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_STRAIN_COMP_TUNE requires coupled XY (CoreXY) "
                "kinematics - the strain map is a belt-pair measurement"
            )
        comp = self.printer.lookup_object("servo_strain_comp", None)
        if comp is None:
            raise gcmd.error(
                "SERVO_STRAIN_COMP_TUNE needs [servo_strain_comp] "
                "configured - it builds and applies the map"
            )
        spacing = gcmd.get_float(
            "SPACING", None, minval=self.STRAIN_MAP_MIN_LINE_SPACING_MM
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0)
        tol = gcmd.get_float("TOL", 0.05, above=0.0, below=0.5)
        max_iters = gcmd.get_int("MAX_ITERS", self.TUNE_MAX_ITERS, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "straintune")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        tuner = comp.begin_tune(gcmd, gcmd.get("RUN"), spacing)
        x_start = tuner.plan["x_start"]
        x_end = tuner.plan["x_end"]
        y_start = tuner.plan["y_start"]
        y_end = tuner.plan["y_end"]
        zero_xy = tuner.plan["zero_xy"]
        line_y = gcmd.get_float("Y", zero_xy[1])
        line_x = gcmd.get_float("X", zero_xy[0])
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y_start": y_start,
            "y_end": y_end,
            "y": line_y,
            "x": line_x,
            "speed": speed,
            "accel": accel,
            "tol": tol,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": list(zero_xy),
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_STRAIN_COMP_TUNE: [servo_sync] is not "
                    "configured - add it so the verification line shares "
                    "the map's preload zero, or pass SYNC=0"
                )
            self._goto_xy(zero_xy[0], zero_xy[1], dwell)
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_tune",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        reactor = self.printer.get_reactor()
        converged = False
        results = None
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            for iteration in range(max_iters):
                tuner.rebuild_and_enable(gcmd)
                reactor.pause(
                    reactor.monotonic() + settle + tuner.enable_ramp_s()
                )
                name_x = "iter%d_x" % iteration
                self._goto_xy(x_start, line_y, dwell)
                self._start_capture(name_x, servos)
                self._strokes("X", x_start, x_end, speed, accel, 1, dwell)
                self._stop_capture()
                name_y = "iter%d_y" % iteration
                self._goto_xy(line_x, y_start, dwell)
                self._start_capture(name_y, servos)
                self._strokes("Y", y_start, y_end, speed, accel, 1, dwell)
                self._stop_capture()
                results = tuner.score_lines(
                    gcmd,
                    run.run_dir,
                    [(name_x, "y", line_y), (name_y, "x", line_x)],
                )
                (kaa, kab), (kba, kbb) = tuner.matrix_rows()
                swept = {
                    "y": line_y,
                    "x": line_x,
                    "kaa": kaa,
                    "kab": kab,
                    "kba": kba,
                    "kbb": kbb,
                }
                for belt_idx, result in enumerate(results):
                    belt = "ab"[belt_idx]
                    swept["s_own_%s" % belt] = result["s_own"]
                    swept["s_cross_%s" % belt] = result["s_cross"]
                    for axis, (rms, base) in result["lines"].items():
                        swept["rms_%s_%s" % (belt, axis)] = rms
                        swept["base_rms_%s_%s" % (belt, axis)] = base
                run.record_step(SweepStep("iter%d" % iteration, swept, []))
                for belt_idx, result in enumerate(results):
                    k_own = tuner.k_matrix[belt_idx][belt_idx]
                    k_cross = tuner.k_matrix[belt_idx][1 - belt_idx]
                    lines = ", ".join(
                        "%s-line residual %.2f%% rms (smooth field was "
                        "%.2f%%)" % (axis.upper(), rms, base)
                        for axis, (rms, base) in sorted(result["lines"].items())
                    )
                    gcmd.respond_info(
                        "tune %d/%d belt %s: measured direct %.1f (map "
                        "used %.1f), cross %.1f (map used %.1f) %%/mm; %s"
                        % (
                            iteration + 1,
                            max_iters,
                            "AB"[belt_idx],
                            result["s_own"],
                            k_own,
                            result["s_cross"],
                            k_cross,
                            lines,
                        )
                    )
                if tuner.converged(results, tol):
                    converged = True
                    break
                tuner.apply(results)
            self._restore()
        finally:
            self._active_run = None
        if not converged:
            raise gcmd.error(
                "did not converge within %d iterations — last measured %s; "
                "the map from the final pass is still enabled"
                % (
                    max_iters,
                    ", ".join(
                        "belt %s direct %.1f cross %.1f"
                        % ("AB"[i], r["s_own"], r["s_cross"])
                        for i, r in enumerate(results)
                    ),
                )
            )
        tuner.store_matrix()
        (kaa, kab), (kba, kbb) = tuner.matrix_rows()
        gcmd.respond_info(
            "converged: stiffness A %.1f B %.1f, cross AB %.1f BA %.1f "
            "%%/mm - all four measured independently on the X and Y "
            "verification lines. Full-bed map rebuilt, written and "
            "ENABLED; the matrix is stored for future builds. The "
            "residuals above only cover the smooth elastic field: "
            "direction-dependent friction asymmetry and sub-%.0fmm ripple "
            "are invisible to a position-keyed map and remain in raw "
            "measurements."
            % (kaa, kbb, kab, kba, servo_strain_comp.FIELD_2D_PITCH_MM)
        )

    cmd_SERVO_MEASURE_INERTIA_help = (
        "Excitation grid for the inertia/friction fit (servo-ident). "
        "coupled_xy kinematics run the X+Y belt grid (SERVOS=/X_START etc "
        "override; travel_speed centers the idle axis between strokes); "
        "cartesian kinematics run a single AXIS grid and reject SERVOS/"
        "X_START/X_END/Y_START/Y_END. Params AXIS START END X_START X_END "
        "Y_START Y_END ACCELS SPEEDS ITERATIONS DWELL_MS NAME SERVOS"
    )

    def _grid_stroke_plan(self, gcmd: Any) -> dict[str, Any]:
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        return {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }

    def _grid_servos(
        self, gcmd: Any, kin: Any
    ) -> tuple[list[str], list[Any] | None, str]:
        if kin.coupled_xy():
            override = gcmd.get("SERVOS", None)
            if override is None:
                servos = servo_strokes.axis_servos(gcmd, kin, "X")
            else:
                servos = [s.strip() for s in override.split(",") if s.strip()]
            return servos, servo_strokes.axis_rails(gcmd, kin, "X"), "X"
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        return servo_strokes.axis_servos(gcmd, kin, axis), None, axis

    def cmd_SERVO_MEASURE_INERTIA(self, gcmd: Any) -> None:
        name = gcmd.get("NAME", "ident")
        kin = self._kin()
        servos, belts_rails, axis = self._grid_servos(gcmd, kin)
        self._begin_run(
            gcmd,
            "inertia_grid",
            name,
            axis,
            servos,
            self._grid_stroke_plan(gcmd),
            belts_rails,
        )
        try:
            self._measure_inertia(gcmd, name)
            run = self._active_run
            assert run is not None, "inertia grid ran outside its run"
            run.record_step(SweepStep(name, {}, []))
        finally:
            self._active_run = None

    def _measure_inertia(self, gcmd: Any, name: str) -> None:
        kin = self._kin()
        if kin.coupled_xy():
            self._measure_inertia_corexy(gcmd, name)
            return
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        self._prep(axis, dwell)
        self._start_capture(name, servos)
        for accel in accels:
            for speed in speeds:
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self._stop_capture()
        self._restore()

    def _measure_inertia_corexy(
        self, gcmd: Any, name: str, servos: str | list[str] | None = None
    ) -> None:
        kin = self._kin()
        if servos is None:
            servos = gcmd.get("SERVOS", None)
        if servos is None:
            servo_list = servo_strokes.axis_servos(gcmd, kin, "X")
        elif isinstance(servos, str):
            servo_list = [s.strip() for s in servos.split(",") if s.strip()]
        else:
            servo_list = list(servos)
        x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        x_center = (x_start + x_end) / 2.0
        y_center = (y_start + y_end) / 2.0
        self._prep("X", dwell)
        self._prep("Y", dwell)
        self._start_capture(name, servo_list)
        for accel in accels:
            for speed in speeds:
                self._goto_xy(x_start, y_center, dwell)
                self._strokes(
                    "X", x_start, x_end, speed, accel, iterations, dwell
                )
                self._goto_xy(x_center, y_start, dwell)
                self._strokes(
                    "Y", y_start, y_end, speed, accel, iterations, dwell
                )
        self._stop_capture()
        self._restore()

    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward - runs the "
        "SERVO_MEASURE_INERTIA grid, fits mass/viscous/coulomb, and writes "
        "a timestamped profile (node-level on coupled_xy, a Cartesian "
        "mode-space model with the frame built from the kinematics; per-axis "
        "otherwise, DRIVE= picking the scalar fit on a multi-drive axis). "
        "Optional TORQUE_NM + INERTIA_KGM2 add the C00.06 recommendation. "
        "Params as SERVO_MEASURE_INERTIA plus TORQUE_NM INERTIA_KGM2 DRIVE"
    )

    def _corexy_frame(
        self, gcmd: Any, kin: Any
    ) -> tuple[list[str], list[str], list[list[float]]]:
        rails = servo_strokes.axis_rails(gcmd, kin, "X")
        slots: list[tuple[str, int, float, int]] = []
        for belt_index, rail in enumerate(rails):
            motors = servo_strokes.rail_motors_in_slot_order(rail)
            drives = len(motors)
            for m in motors:
                sign = -1.0 if m.get_invert_direction() else 1.0
                slots.append((m.get_motor_name(), belt_index, sign, drives))
        axes = [name for name, _b, _s, _d in slots]
        frame_x = [sign / (2.0 * drives) for _n, _b, sign, drives in slots]
        frame_y = [
            (sign if belt == 0 else -sign) / (2.0 * drives)
            for _n, belt, sign, drives in slots
        ]
        return axes, ["x", "y"], [frame_x, frame_y]

    def _fit_plan(self, gcmd: Any) -> dict[str, Any]:
        kin = self._kin()
        if kin.coupled_xy():
            layout = servo_strokes.corexy_fit_layout(gcmd, kin)
            servo_strokes.check_servos_override(gcmd, layout)
            axes, modes, frame = self._corexy_frame(gcmd, kin)
            return {
                "corexy": True,
                "servos": layout["servos"],
                "axes": axes,
                "modes": modes,
                "frame": frame,
                "axis": "X",
                "rails": servo_strokes.axis_rails(gcmd, kin, "X"),
            }
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        drive = servo_strokes.scalar_fit_drive(gcmd, kin)
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        axes = [drive if drive is not None else servos[0]]
        return {
            "corexy": False,
            "servos": servos,
            "axes": axes,
            "modes": list(axes),
            "frame": [[1.0]],
            "axis": axis,
            "rails": None,
        }

    def _rotation_distance(self, gcmd: Any, servos: list[str]) -> float:
        distances = {
            self._resolve_motor(s).get_rotation_distance() for s in servos
        }
        if len(distances) != 1:
            raise gcmd.error(
                "drives disagree on rotation_distance (%s); cannot fit"
                % (sorted(distances),)
            )
        return distances.pop()

    def _dynamics_out_path(
        self, gcmd: Any, run: ExperimentRun, name: str
    ) -> str:
        os.makedirs(self.dynamics_dir, exist_ok=True)
        path = os.path.join(
            self.dynamics_dir, "dynamics_%s_%s.toml" % (name, run.stamp)
        )
        if os.path.exists(path):
            raise gcmd.error(
                "dynamics profile %s already exists (never overwritten)"
                % (path,)
            )
        return path

    def _run_fit(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        plan = self._fit_plan(gcmd)
        run = self._begin_run(
            gcmd,
            "inertia_grid",
            name,
            plan["axis"],
            plan["servos"],
            self._grid_stroke_plan(gcmd),
            plan["rails"],
        )
        try:
            if plan["corexy"]:
                self._measure_inertia_corexy(gcmd, name, servos=plan["servos"])
            else:
                self._measure_inertia(gcmd, name)
            run.record_step(SweepStep(name, {}, []))
            out_path = self._dynamics_out_path(gcmd, run, name)
            argv = [
                self._servo_cal(gcmd),
                "fit",
                "--capture",
                run.step_scap(name),
                "--frame",
                ";".join(
                    ",".join("%g" % (f,) for f in row) for row in plan["frame"]
                ),
                "--modes",
                ",".join(plan["modes"]),
                "--axes",
                ",".join(plan["axes"]),
                "--out",
                out_path,
                "--rotation-distance-mm",
                "%g" % (self._rotation_distance(gcmd, plan["servos"]),),
            ]
            if torque is not None:
                argv += [
                    "--rated-torque-nm",
                    "%g" % (torque,),
                    "--rotor-inertia-kgm2",
                    "%g" % (inertia,),
                ]
            text = self._run(gcmd, argv, 120.0)
            gcmd.respond_info(
                "dynamics profile: %s | run %s" % (out_path, run.run_dir)
            )
        finally:
            self._active_run = None
        return run, text, out_path

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=False)
        self._run_fit(gcmd, gcmd.get("NAME", "ident"), torque, inertia)

    cmd_SERVO_CALIBRATE_INERTIA_RATIO_help = (
        "Step 2 of servo tuning - identify the load inertia and print the "
        "recommended C00.06 (on coupled_xy kinematics: for both belt "
        "directions, via the coupled X+Y grid and mode-space fit; the "
        "drive takes one scalar, so start from the light-direction number "
        "and confirm with SERVO_SWEEP_INERTIA). TORQUE_NM and INERTIA_KGM2 "
        "required (config or param). Params as SERVO_MEASURE_INERTIA plus "
        "TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=True)
        self._run_fit(gcmd, gcmd.get("NAME", "inertia"), torque, inertia)

    def _write_gains(
        self, servos: list[str], pos_gain: int, speed_gain: int, integral: int
    ) -> None:
        lines = []
        for servo in servos:
            lines += [
                "SERVO_PARAM SERVO=%s SET=0x2001.0x01 VALUE=%d TYPE=u16"
                % (servo, pos_gain),
                "SERVO_PARAM SERVO=%s SET=0x2001.0x02 VALUE=%d TYPE=u16"
                % (servo, speed_gain),
                "SERVO_PARAM SERVO=%s SET=0x2001.0x03 VALUE=%d TYPE=u16"
                % (servo, integral),
            ]
        with servo_param.suppress_write_log():
            self.gcode.run_script_from_command("\n".join(lines))

    def _gain_write_records(
        self, servos: list[str], pos_gain: int, speed_gain: int, integral: int
    ) -> list[dict[str, Any]]:
        values = {
            "position": pos_gain,
            "speed": speed_gain,
            "integral": integral,
        }
        return [
            _applied(servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in ("position", "speed", "integral")
        ]

    def _resolve_node_slot(self, servo: str) -> tuple[Any, int]:
        from . import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _read_param(self, servo: str, addr: str) -> int:
        from . import servo_param

        node, slot = self._resolve_node_slot(servo)
        handle = node.get_engine_handle()
        if handle is None:
            raise self.printer.command_error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        index, subindex = servo_param.parse_address(addr)
        _size, raw = engine.sdo_read(handle, slot, index, subindex)
        return raw

    def _read_gains(self, servo: str) -> dict[str, int]:
        return {
            name: self._read_param(servo, GAIN_PARAMS[name][0])
            for name in ("position", "speed", "integral")
        }

    def _set_manual_tuning(self, servos: list[str]) -> None:
        self.gcode.run_script_from_command(
            "\n".join(
                "SERVO_PARAM SERVO=%s SET=0x2000.0x05 VALUE=0 TYPE=u16"
                % (servo,)
                for servo in servos
            )
        )

    cmd_SERVO_SHOW_TUNING_help = (
        "Read back tuning mode, inertia ratio, gain set 1 and feedforward "
        "params from the drive(s). Params SERVO (comma list) or AXIS"
    )

    def cmd_SERVO_SHOW_TUNING(self, gcmd: Any) -> None:
        for servo in self._servos(gcmd):
            self._show_tuning(servo)

    def _show_tuning(self, servo: str) -> None:
        reads = [
            (
                "C00.04 auto-tuning mode (0=manual 1=stiffness 2=positioning):",
                ["0x2000.0x05"],
            ),
            (
                "C00.05 stiffness level (1..31, used in mode 1):",
                ["0x2000.0x06"],
            ),
            ("C00.06 load inertia ratio (%):", ["0x2000.0x07"]),
            ("C01.00 position loop gain (0.1 rad/s):", ["0x2001.0x01"]),
            ("C01.01 speed loop gain (0.1 Hz):", ["0x2001.0x02"]),
            ("C01.02 speed integral time (0.01 ms):", ["0x2001.0x03"]),
            (
                "C01.13 velocity FF source / C01.14 pct / C01.15 filter:",
                ["0x2001.0x14", "0x2001.0x15", "0x2001.0x16"],
            ),
            (
                "C01.16 torque FF source / C01.17 pct / C01.18 filter:",
                ["0x2001.0x17", "0x2001.0x18", "0x2001.0x19"],
            ),
            (
                "C13.02 sync loss fault threshold / C13.04 sync loss count:",
                [SYNC_LOSS_THRESHOLD_ADDR, SYNC_LOSS_COUNT_ADDR],
            ),
        ]
        script = ['RESPOND MSG="=== %s ==="' % (servo,)]
        for msg, addrs in reads:
            script.append('RESPOND MSG="%s"' % (msg,))
            for addr in addrs:
                script.append("SERVO_PARAM SERVO=%s GET=%s" % (servo, addr))
        self.gcode.run_script_from_command("\n".join(script))

    cmd_SERVO_SET_INERTIA_RATIO_help = (
        "Write C00.06 load inertia ratio in percent. Params RATIO SERVO"
    )

    def cmd_SERVO_SET_INERTIA_RATIO(self, gcmd: Any) -> None:
        servo = self._servo(gcmd)
        ratio = gcmd.get_int("RATIO", minval=0, maxval=C00_06_INERTIA_RATIO_MAX)
        self.gcode.run_script_from_command(
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
            % (servo, INERTIA_RATIO_ADDR, ratio)
        )

    cmd_SERVO_APPLY_GAINS_help = (
        "Switch the drive(s) to manual tuning (C00.04=0) and write gain set "
        "1 to every servo driving the axis. POS_GAIN 0.1 rad/s, SPEED_GAIN "
        "0.1 Hz, INTEGRAL 0.01 ms. Params AXIS or SERVO (comma list)"
    )

    def cmd_SERVO_APPLY_GAINS(self, gcmd: Any) -> None:
        servos = self._servos(gcmd)
        pos_gain = gcmd.get_int("POS_GAIN", 400)
        speed_gain = gcmd.get_int("SPEED_GAIN", 250)
        integral = gcmd.get_int("INTEGRAL", 3184)
        self._set_manual_tuning(servos)
        self._write_gains(servos, pos_gain, speed_gain, integral)
        for servo in servos:
            self._show_tuning(servo)

    def _corexy_rails(self, gcmd: Any, axis: str) -> list[Any] | None:
        kin = self._kin()
        if kin.coupled_xy() and axis in ("X", "Y"):
            return servo_strokes.axis_rails(gcmd, kin, axis)
        return None

    def _run_sweep_with_revert(
        self,
        adapter: Any,
        values: list[Any],
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        on_revert: Callable[[], None],
    ) -> list[SweepStep]:
        try:
            steps = self._engine.run(adapter, values, servos, run_step, gcmd)
        finally:
            on_revert()
            adapter.revert()
            self._restore()
        return steps

    cmd_SERVO_CALIBRATE_GAINS_help = (
        "Gain sweep, shaper-calibrate style. Resolves every servo driving "
        "AXIS (both drives on CoreXY), writes each SPEED_GAINS entry (0.1 Hz "
        "units, comma list) to all of them, one capture per step of all "
        "drives into a run directory, then servo-cal analyzes it into "
        "results.json with a typed verdict (the recommended step). "
        "With an accelerometer (accel_chip config option or ACCEL_CHIP=) "
        "each step also records vibration data next to its capture. Always "
        "restores the gains that were active before the sweep (also on "
        "failure). APPLY=1 writes the verdict's "
        "recommended gains after the restore, reads them back, and runs one "
        "SERVO_MEASURE_TRACKING to report before/after tracking metrics "
        "(default APPLY=0, report-only). SERVO= (comma list) restricts the "
        "sweep to a subset of the axis servos; BASE_SPEED_GAIN then pins "
        "every non-swept axis servo at that gain (same 1.6x/Ti derivation) "
        "for an asymmetric-gain experiment; those servos are restored too. "
        "Params SPEED_GAINS AXIS START "
        "END SPEED ACCEL ITERATIONS DWELL_MS TAG ACCEL_CHIP APPLY SERVO "
        "BASE_SPEED_GAIN"
    )

    def cmd_SERVO_CALIBRATE_GAINS(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "cal")
        sgains = self._floats(gcmd.get("SPEED_GAINS", "500,650,800,1000"))
        for sg in sgains:
            if not SPEED_GAIN_MIN <= sg <= SPEED_GAIN_MAX:
                raise gcmd.error(
                    "SPEED_GAIN %d outside %d..%d (0.1 Hz units; max keeps "
                    "the derived 1.6x position gain in drive range)"
                    % (sg, SPEED_GAIN_MIN, SPEED_GAIN_MAX)
                )
        sgains = [int(sg) for sg in sgains]
        if gcmd.get("REVERT_GAIN", None) is not None:
            raise gcmd.error(
                "REVERT_GAIN was removed - the sweep always restores the "
                "gains that were active before it ran; keep a result with "
                "APPLY=1 or SERVO_APPLY_GAINS"
            )
        base_sg = gcmd.get("BASE_SPEED_GAIN", None)
        base_servos: list[str] = []
        if base_sg is not None:
            base_sg = int(base_sg)
            if not SPEED_GAIN_MIN <= base_sg <= SPEED_GAIN_MAX:
                raise gcmd.error(
                    "BASE_SPEED_GAIN %d outside %d..%d (0.1 Hz units)"
                    % (base_sg, SPEED_GAIN_MIN, SPEED_GAIN_MAX)
                )
            axis_servos = servo_strokes.axis_servos(gcmd, self._kin(), axis)
            base_servos = [s for s in axis_servos if s not in servos]
            if not base_servos:
                raise gcmd.error(
                    "BASE_SPEED_GAIN needs SERVO= to name a subset of the "
                    "axis servos - every servo on axis %s is already in the "
                    "sweep" % (axis,)
                )
        apply = gcmd.get_int("APPLY", 0)
        chip, chip_name = self._accel_chip(gcmd)
        affected = list(dict.fromkeys(servos + base_servos))
        prior = {s: self._read_gains(s) for s in affected}

        def restore_prior() -> None:
            for s, g in prior.items():
                self._write_gains([s], g["position"], g["speed"], g["integral"])

        stroke_plan = {
            "start": start,
            "end": end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "gain_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = GainSetAdapter(self, servos, tag)
        restored = False
        try:
            self._prep(axis, dwell)
            servo_strokes.goto(
                self.gcode,
                self.travel_speed,
                "%s%.3f" % (axis, start),
                dwell,
            )
            self._set_manual_tuning(servos)
            if base_servos:
                base_pos, base_integral = GainSetAdapter.derive(base_sg)
                self._set_manual_tuning(base_servos)
                self._write_gains(base_servos, base_pos, base_sg, base_integral)
                run.manifest["base_gains"] = {
                    "servos": base_servos,
                    "position": base_pos,
                    "speed": base_sg,
                    "integral": base_integral,
                }
                run.write()
                gcmd.respond_info(
                    "base gains on %s: pos %.1f rad/s, speed %.1f Hz, "
                    "Ti %.2f ms (held for the whole sweep)"
                    % (
                        ", ".join(base_servos),
                        base_pos / 10.0,
                        base_sg / 10.0,
                        base_integral / 100.0,
                    )
                )
            steps = self._engine.run(
                adapter,
                sgains,
                servos,
                lambda sg: self._strokes(
                    axis, start, end, speed, accel, iterations, dwell
                ),
                gcmd,
                accel_chip=chip,
                accel_chip_name=chip_name,
            )
            gcmd.respond_info(
                "sweep done - restoring the pre-sweep gains until you "
                "apply the recommendation"
            )
            restore_prior()
            restored = True
            self._restore()
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            if not restored:
                restore_prior()
            self._active_run = None
        return steps

    def _stroke_motion(self, gcmd: Any) -> tuple[float, float, int, int]:
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        return speed, accel, iterations, dwell

    def _config_bounds(self, gcmd: Any, axis: str) -> tuple[float, float]:
        lo, hi = self.bounds.get(axis, (None, None))
        if lo is None or hi is None:
            raise gcmd.error(
                "no stroke bounds configured for axis %s" % (axis,)
            )
        return lo, hi

    cmd_SERVO_HARVEST_NOTCHES_help = (
        "Let the drive's adaptive notch tuning find the axis resonances "
        "during motion, then lock and read back what it chose (manual 7.10). "
        "Writes C01.30 adaptive_notch_mode = MODE (1 = 1st notch adaptive, "
        "2 = 1st+2nd adaptive) to every servo driving AXIS, strokes so the "
        "tuner sees motion, reads back notch 1 and notch 2 center frequency / "
        "width / depth, then writes C01.30 = 0 to lock. The mode writes and "
        "the lock are journaled (no run directory). Any SDO read/write "
        "failure aborts naming the motor and address, before the lock. "
        "Params AXIS (X) MODE (2) START END SPEED ACCEL ITERATIONS DWELL_MS"
    )

    def _write_notch_mode(self, servos: list[str], mode: int) -> None:
        self.gcode.run_script_from_command(
            "\n".join(
                "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
                % (servo, NOTCH_MODE_ADDR, mode)
                for servo in servos
            )
        )

    def _read_notch_param(self, gcmd: Any, servo: str, addr: str) -> int:
        try:
            return self._read_param(servo, addr)
        except (RuntimeError, ValueError) as e:
            raise gcmd.error(
                "notch readback failed for %s %s: %s" % (servo, addr, e)
            )

    def _read_notches(
        self, gcmd: Any, servo: str
    ) -> list[tuple[int, int, int]]:
        return [
            (
                self._read_notch_param(gcmd, servo, addrs[0]),
                self._read_notch_param(gcmd, servo, addrs[1]),
                self._read_notch_param(gcmd, servo, addrs[2]),
            )
            for _label, addrs in NOTCH_READBACK
        ]

    def _notch_state(self, gcmd: Any, servo: str) -> dict[str, Any]:
        state: dict[str, Any] = {
            "mode": self._read_notch_param(gcmd, servo, NOTCH_MODE_ADDR)
        }
        for (label, _addrs), (freq, width, depth) in zip(
            NOTCH_READBACK, self._read_notches(gcmd, servo)
        ):
            state[label] = {"freq_hz": freq, "width": width, "depth": depth}
        return state

    def cmd_SERVO_HARVEST_NOTCHES(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        mode = gcmd.get_int("MODE", 2)
        if mode not in (1, 2):
            raise gcmd.error(
                "MODE must be 1 (1st notch adaptive) or 2 (1st+2nd "
                "adaptive), got %d" % (mode,)
            )
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed, accel, iterations, dwell = self._stroke_motion(gcmd)
        self._prep(axis, dwell)
        self._write_notch_mode(servos, mode)
        self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self.gcode.run_script_from_command("M400")
        harvested = {servo: self._read_notches(gcmd, servo) for servo in servos}
        self._write_notch_mode(servos, 0)
        self._restore()
        for servo in servos:
            n1, n2 = harvested[servo][:2]
            gcmd.respond_info(
                "%s notch1 %d Hz w%d d%d | notch2 %d Hz w%d d%d"
                % (servo, n1[0], n1[1], n1[2], n2[0], n2[1], n2[2])
            )
        gcmd.respond_info(
            "adaptive notch tuning locked (C01.30 = 0) on %s"
            % (", ".join(servos),)
        )

    cmd_SERVO_GAIN_LADDER_help = (
        "Gain sweep that climbs until analysis flags trouble instead of "
        "a fixed list. Runs [SAFE, START, START+STEP, ... <= MAX]. Without "
        "PARAM it climbs the speed gain with position/integral derived from "
        "it (1.6x / Ti); with PARAM=position|speed|integral it climbs that "
        "one gain in its device units, holding the other two at their "
        "pre-ladder values (the drives must agree on the current gains). "
        "After every rung at or above START, servo-cal analyzes "
        "the run so far; the first rung whose step carries a resonance, "
        "torque-rail or truncated-settle flag stops the climb (higher rungs "
        "are never executed). The SAFE baseline never stops the climb. "
        "Always restores the pre-ladder gains at the end (also on failure) "
        "- keep a rung with SERVO_APPLY_GAINS. Prints the verdict one-liner "
        "and, on an early "
        "stop, the rung and flags that stopped it. START names the first "
        "climb value, not a stroke bound - the stroke window comes from the "
        "configured axis bounds. Params SAFE START STEP (50) MAX PARAM "
        "AXIS (X) SPEED ACCEL ITERATIONS DWELL_MS TAG (ladder) SERVO"
    )

    def _ladder_values(
        self, gcmd: Any, param: str | None
    ) -> tuple[int, list[int]]:
        safe_g = gcmd.get_int("SAFE")
        start_g = gcmd.get_int("START")
        step_g = gcmd.get_int("STEP", 50)
        max_g = gcmd.get_int("MAX")
        if step_g <= 0:
            raise gcmd.error("STEP must be > 0 (got %d)" % (step_g,))
        if max_g < start_g:
            raise gcmd.error(
                "MAX (%d) must be >= START (%d)" % (max_g, start_g)
            )
        values = [safe_g] + list(range(start_g, max_g + 1, step_g))
        if param is None:
            for sg in values:
                if not SPEED_GAIN_MIN <= sg <= SPEED_GAIN_MAX:
                    raise gcmd.error(
                        "speed gain %d outside %d..%d (0.1 Hz units)"
                        % (sg, SPEED_GAIN_MIN, SPEED_GAIN_MAX)
                    )
            return start_g, values
        try:
            validate_gain_values(values, param)
        except ValueError as e:
            raise gcmd.error("SERVO_GAIN_LADDER: %s" % (e,))
        return start_g, values

    def cmd_SERVO_GAIN_LADDER(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        param = gcmd.get("PARAM", None)
        if param is not None:
            param = param.lower()
            if param not in GAIN_PARAMS:
                raise gcmd.error(
                    "PARAM must be position, speed or integral (got %r)"
                    % (gcmd.get("PARAM"),)
                )
        start_g, values = self._ladder_values(gcmd, param)
        start, end = self._config_bounds(gcmd, axis)
        speed, accel, iterations, dwell = self._stroke_motion(gcmd)
        tag = gcmd.get("TAG", "ladder")
        prior = {s: self._read_gains(s) for s in servos}
        if param is not None:
            first = prior[servos[0]]
            for s, g in prior.items():
                if g != first:
                    raise gcmd.error(
                        "servos disagree on the current gains (%s=%s vs "
                        "%s=%s) - a single-parameter ladder holds the other "
                        "two at one shared value; align the drives first "
                        "(SERVO_APPLY_GAINS)" % (servos[0], first, s, g)
                    )
            adapter: Any = SingleGainAdapter(
                self, servos, param, tag, dict(first), first[param]
            )
        else:
            adapter = GainSetAdapter(self, servos, tag)

        def restore_prior() -> None:
            for s, g in prior.items():
                self._write_gains([s], g["position"], g["speed"], g["integral"])

        stroke_plan = {
            "start": start,
            "end": end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "gain_ladder",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        stopped: tuple[int, str, list[str]] | None = None
        steps: list[SweepStep] = []
        restored = False
        try:
            self._prep(axis, dwell)
            self._set_manual_tuning(servos)
            for i, value in enumerate(values):
                step = self._engine.run_one(
                    adapter,
                    i,
                    value,
                    len(values),
                    servos,
                    lambda _v: self._strokes(
                        axis, start, end, speed, accel, iterations, dwell
                    ),
                    gcmd,
                )
                steps.append(step)
                if value < start_g:
                    continue
                results = self._run_analyze(gcmd, run, incremental=True)
                flags = sorted(
                    set(self._step_flags(results, step.name))
                    & LADDER_STOP_FLAGS
                )
                if flags:
                    stopped = (value, step.name, flags)
                    break
            gcmd.respond_info(
                "ladder done - restoring the pre-ladder gains on %s; keep "
                "a rung with SERVO_APPLY_GAINS" % (", ".join(servos),)
            )
            restore_prior()
            restored = True
            self._restore()
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
        finally:
            if not restored:
                restore_prior()
            self._active_run = None
        if stopped is not None:
            gcmd.respond_info(
                "climb stopped at %s (step %s): flags %s"
                % (stopped[0], stopped[1], stopped[2])
            )
        return steps

    cmd_SERVO_REFINE_GAIN_help = (
        "1-D sensitivity sweep of a single drive gain around the current "
        "operating point, holding the other two fixed. PARAM=position|speed|"
        "integral. Reads the current gains from the drive; sweeps either an "
        "explicit VALUES= list or the current value +-SPAN over STEPS points "
        "(default +-30%% in 5 steps, always including the current value). "
        "Writes each step to EVERY drive on AXIS (both CoreXY lanes), one "
        "capture per step, restores the original gains afterwards (also on "
        "failure), then servo-cal analyzes the run into results.json. "
        "APPLY=1 writes the verdict's recommended value after the restore, "
        "reads it back, and runs one SERVO_MEASURE_TRACKING to report "
        "before/after tracking metrics (default APPLY=0, report-only). "
        "Params PARAM AXIS VALUES SPAN "
        "STEPS CURRENT START END SPEED ACCEL ITERATIONS DWELL_MS TAG APPLY "
        "SERVO (comma list override)"
    )

    def cmd_SERVO_REFINE_GAIN(self, gcmd: Any) -> list[SweepStep]:
        param = gcmd.get("PARAM", "").lower()
        if param not in GAIN_PARAMS:
            raise gcmd.error(
                "PARAM must be position, speed or integral (got %r)"
                % (gcmd.get("PARAM", ""),)
            )
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "refine")
        gains = self._read_gains(servos[0])
        current = gcmd.get_int("CURRENT", gains[param], minval=1)
        span = gcmd.get_float("SPAN", 0.3, above=0.0, below=1.0)
        stepcount = gcmd.get_int("STEPS", 5, minval=2)
        values_text = gcmd.get("VALUES", None)
        apply = gcmd.get_int("APPLY", 0)
        try:
            values = refine_values(current, values_text, span, stepcount)
            validate_gain_values(values, param)
        except ValueError as e:
            raise gcmd.error("SERVO_REFINE_GAIN: %s" % (e,))
        stroke_plan = {
            "start": start,
            "end": end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "refine_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = SingleGainAdapter(self, servos, param, tag, gains, current)

        def on_revert() -> None:
            gcmd.respond_info(
                "restoring original gains pos %d / speed %d / integral %d on %s"
                % (
                    gains["position"],
                    gains["speed"],
                    gains["integral"],
                    ", ".join(servos),
                )
            )

        try:
            self._prep(axis, dwell)
            self._set_manual_tuning(servos)
            steps = self._run_sweep_with_revert(
                adapter,
                values,
                servos,
                lambda v: self._strokes(
                    axis, start, end, speed, accel, iterations, dwell
                ),
                gcmd,
                on_revert,
            )
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            self._active_run = None
        return steps

    cmd_SERVO_REFINE_DYNAMICS_help = (
        "Empirically refine the torque-feedforward dynamics profile on the "
        "RUNNING endpoint: golden-section search over a scale factor on the "
        "baseline profile's mass matrix (TERM=MASS, scored on mean per-move "
        "ferr_peak - the error window runs from move start through settle, "
        "so it covers in-move tracking and endpoint overshoot alike), "
        "viscous vector (TERM=VISCOUS, scored on mean ferr_rms) or "
        "coulomb vector (TERM=COULOMB, "
        "scored on mean ferr_peak - friction error peaks at breakaway). On "
        "coupled_xy every term refines the two modes sequentially - "
        "X strokes scaling only the x-mode entry, then Y strokes scaling "
        "the y mode on top of the X winner - since the modes are "
        "independent physical quantities (moved mass, rail friction) and "
        "an axis stroke leaves the other mode's velocity at exactly zero. "
        "Each candidate runs the full "
        "SERVO_MEASURE_INERTIA ACCELS x SPEEDS grid in one tracking "
        "capture, so the score averages over every operating point. The "
        "baseline is PROFILE= or the "
        "node-level [ethercat_node] dynamics_profile (per-motor profiles "
        "are not supported). The live model is ALWAYS restored to the "
        "baseline afterwards (also on failure; if klippy dies mid-run the "
        "endpoint keeps the last candidate until restart). A torque-rail "
        "flag on any step aborts (clipped strokes cannot score a "
        "candidate); the resonance flag is ignored here - scaling a "
        "feedforward term does not move the loop's resonances. When a scale "
        "beats 1.0 the scaled profile is written to a new TOML - pointing "
        "dynamics_profile at it (then RESTART) is the only way to keep it. "
        "Params TERM (MASS) AXIS (X) SERVOS PROFILE LO (0.7) HI (1.3) TOL "
        "(0.02) MAX_EVALS (10) START END X_START X_END Y_START Y_END "
        "ACCELS SPEEDS ITERATIONS DWELL_MS TAG (refdyn) NAME"
    )

    def _refine_dynamics_node(self, gcmd: Any, servos: list[str]) -> Any:
        nodes = {}
        for servo in servos:
            node, _slot = self._resolve_node_slot(servo)
            nodes[node.name] = node
        if len(nodes) != 1:
            raise gcmd.error(
                "servos %s span multiple ethercat nodes (%s) - the dynamics "
                "model is per-node" % (servos, sorted(nodes))
            )
        return nodes.popitem()[1]

    def _load_baseline_dynamics(
        self, gcmd: Any, node: Any
    ) -> tuple[str, dict[str, Any]]:
        profile_path = gcmd.get("PROFILE", None) or node.get_dynamics_profile()
        if profile_path is None:
            raise gcmd.error(
                "no baseline dynamics profile - set dynamics_profile on "
                "[ethercat_node %s] or pass PROFILE= (per-motor profiles "
                "are not supported by SERVO_REFINE_DYNAMICS)" % (node.name,)
            )
        profile_path = os.path.expanduser(profile_path)
        try:
            with open(profile_path) as f:
                baseline = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "failed to load dynamics profile %s: %s" % (profile_path, e)
            )
        if len(baseline["axes"]) != node.get_drive_count():
            raise gcmd.error(
                "profile %s describes %d axes but node %s has %d drives"
                % (
                    profile_path,
                    len(baseline["axes"]),
                    node.name,
                    node.get_drive_count(),
                )
            )
        return profile_path, baseline

    def cmd_SERVO_REFINE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_REFINE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        term = gcmd.get("TERM", "MASS").upper()
        if term not in DYNAMICS_METRIC_BY_TERM:
            raise gcmd.error(
                "TERM must be MASS, VISCOUS or COULOMB (got %r)"
                % (gcmd.get("TERM", ""),)
            )
        kin = self._kin()
        servos, rails, axis = self._grid_servos(gcmd, kin)
        node = self._refine_dynamics_node(gcmd, servos)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )

        def axis_grid(
            ax: str, a_start: float, a_end: float, goto: tuple[float, float]
        ) -> Callable[[], None]:
            def run_grid() -> None:
                self._goto_xy(goto[0], goto[1], dwell)
                for accel in accels:
                    for speed in speeds:
                        self._strokes(
                            ax, a_start, a_end, speed, accel, iterations, dwell
                        )

            return run_grid

        def term_scale_fn(
            profile: dict[str, Any], scale: float
        ) -> dict[str, Any]:
            return scale_dynamics(profile, term, scale)

        if kin.coupled_xy():
            x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
                gcmd, self.bounds
            )
            x_center = (x_start + x_end) / 2.0
            y_center = (y_start + y_end) / 2.0

            def prep_axes() -> None:
                self._prep("X", dwell)
                self._prep("Y", dwell)

            x_grid = axis_grid("X", x_start, x_end, (x_start, y_center))
            y_grid = axis_grid("Y", y_start, y_end, (x_center, y_start))

            modes = baseline["modes"]
            if len(modes) != 2 or not {"x", "y"} <= set(modes):
                raise gcmd.error(
                    "coupled_xy TERM=%s refine needs a 2-mode profile "
                    "with x and y modes; profile %s has modes %s"
                    % (term, profile_path, modes)
                )

            def mode_scale_fn(
                index: int,
            ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                def scale_fn(
                    profile: dict[str, Any], scale: float
                ) -> dict[str, Any]:
                    return scale_dynamics_mode(profile, term, index, scale)

                return scale_fn

            phases = [
                (
                    "%s_x" % (term.lower(),),
                    "x",
                    mode_scale_fn(modes.index("x")),
                    x_grid,
                ),
                (
                    "%s_y" % (term.lower(),),
                    "y",
                    mode_scale_fn(modes.index("y")),
                    y_grid,
                ),
            ]
        else:
            start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)

            def prep_axes() -> None:
                self._prep(axis, dwell)

            def cart_grid() -> None:
                for accel in accels:
                    for speed in speeds:
                        self._strokes(
                            axis, start, end, speed, accel, iterations, dwell
                        )

            phases = [(term.lower(), "", term_scale_fn, cart_grid)]

        tag = gcmd.get("TAG", "refdyn")
        name = gcmd.get("NAME", "refined_%s" % (term.lower(),))
        lo = gcmd.get_float("LO", 0.7, above=0.0)
        hi = gcmd.get_float("HI", 1.3)
        tol = gcmd.get_float("TOL", 0.02, above=0.0)
        max_evals = gcmd.get_int("MAX_EVALS", 10, minval=3)
        if not lo < 1.0 < hi:
            raise gcmd.error(
                "bracket [LO, HI] = [%g, %g] must contain 1.0 strictly - "
                "the search is centered on the baseline" % (lo, hi)
            )
        metric = DYNAMICS_METRIC_BY_TERM[term]
        stroke_plan = {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "dynamics_refine",
            tag,
            axis,
            servos,
            stroke_plan,
            rails,
        )
        run.manifest["dynamics_refine"] = {
            "baseline_profile": profile_path,
            "term": term.lower(),
            "metric": metric,
            "phases": [label for label, _s, _fn, _g in phases],
            "bracket": [lo, hi],
            "tol": tol,
            "max_evals": max_evals,
        }
        run.write()
        report_metrics = ("overshoot", "ferr_rms", "ferr_peak")

        def metrics_line(values: dict[str, float]) -> str:
            return ", ".join("%s %.1f" % kv for kv in values.items())

        def run_phase(
            adapter: DynamicsModelAdapter,
            run_grid: Callable[[], None],
        ) -> tuple[float, float, float, dict[float, dict[str, float]]]:
            scores: dict[float, float] = {}
            reports: dict[float, dict[str, float]] = {}

            def gate_torque_rail(
                step_name: str, results: dict[str, Any]
            ) -> None:
                if "torque_saturated" in self._step_flags(results, step_name):
                    raise gcmd.error(
                        "step %s hit the torque rail - clipped strokes "
                        "cannot score a candidate, aborting refinement"
                        % (step_name,)
                    )

            def evaluate(scale: float) -> float:
                key = round(scale, 4)
                if key in scores:
                    return scores[key]
                step = self._engine.run_one(
                    adapter,
                    len(scores),
                    key,
                    max_evals + 1,
                    servos,
                    lambda _s: run_grid(),
                    gcmd,
                )
                results = self._run_analyze(gcmd, run, incremental=True)
                gate_torque_rail(step.name, results)
                reports[key] = {
                    m: self._step_metric_mean(gcmd, results, step.name, m)
                    for m in report_metrics
                }
                gcmd.respond_info(
                    "%s scale %.4f -> %s (counts, mean per move)"
                    % (adapter.label, key, metrics_line(reports[key]))
                )
                scores[key] = reports[key][metric]
                return scores[key]

            baseline_score = evaluate(1.0)
            best, best_score, _probes = golden_section_search(
                evaluate, lo, hi, tol, max_evals
            )
            if baseline_score <= best_score:
                best, best_score = 1.0, baseline_score
            return best, best_score, baseline_score, reports

        phase_out: list[tuple[str, str, float, float, float, dict]] = []
        adapters: list[DynamicsModelAdapter] = []
        refined = baseline
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            prep_axes()
            for label, suffix, scale_fn, run_grid in phases:
                adapter = DynamicsModelAdapter(
                    engine, handle, refined, scale_fn, label, tag
                )
                adapters.append(adapter)
                best, best_score, baseline_score, reports = run_phase(
                    adapter, run_grid
                )
                phase_out.append(
                    (label, suffix, best, best_score, baseline_score, reports)
                )
                refined = adapter.scaled(best)
        finally:
            try:
                if any(a.applied for a in adapters):
                    adapters[0].revert()
                    gcmd.respond_info(
                        "live dynamics model restored to baseline %s"
                        % (profile_path,)
                    )
            finally:
                self._restore()
                self._active_run = None
        for (
            label,
            _suffix,
            best,
            best_score,
            baseline_score,
            reports,
        ) in phase_out:
            for scale in sorted(reports):
                marker = "  <- best" if scale == round(best, 4) else ""
                gcmd.respond_info(
                    "  %s scale %.4f: %s%s"
                    % (label, scale, metrics_line(reports[scale]), marker)
                )
            structured_log.event(
                "calibration",
                "dynamics_refined",
                run_dir=run.run_dir,
                term=label,
                metric=metric,
                best_scale=best,
                best_score=best_score,
                baseline_score=baseline_score,
                evals=len(reports),
            )
        if all(best == 1.0 for _l, _s, best, _bs, _bl, _r in phase_out):
            gcmd.respond_info(
                "baseline already optimal within the bracket - no profile "
                "written | run %s" % (run.run_dir,)
            )
            return
        scales = {suffix: best for _l, suffix, best, _bs, _bl, _r in phase_out}
        with open(out_path, "w") as f:
            f.write(
                render_dynamics_toml(
                    refined, profile_path, term, scales, run.run_dir
                )
            )
        gcmd.respond_info(
            "%s | refined profile: %s | point [ethercat_node %s] "
            "dynamics_profile at it and RESTART | run %s"
            % (
                "; ".join(
                    "%s scale %.4f (%s %.1f -> %.1f)"
                    % (label, best, metric, baseline_score, best_score)
                    for label, _s, best, best_score, baseline_score, _r in (
                        phase_out
                    )
                ),
                out_path,
                node.name,
                run.run_dir,
            )
        )

    cmd_SERVO_SWEEP_INERTIA_help = (
        "Empirical inertia sweep, gain-sweep style. Resolves every servo "
        "driving AXIS (both drives on CoreXY), writes each C00.06 ratio in "
        "RATIOS (percent, comma list) identically to all of them, one capture "
        "per step of all drives into a run directory, then servo-cal analyzes "
        "it into results.json. Restores the "
        "original ratio afterwards (also on failure). No automated pick "
        "(read the overshoot trend across steps), so APPLY=1 always errors "
        "here - use SERVO_SET_INERTIA_RATIO once you have chosen a value. "
        "Params RATIOS AXIS "
        "START END SPEED ACCEL ITERATIONS DWELL_MS TAG APPLY SERVO (comma "
        "list override)"
    )

    def cmd_SERVO_SWEEP_INERTIA(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "inertia")
        apply = gcmd.get_int("APPLY", 0)
        ratios: list[int] = []
        for r in self._floats(gcmd.get("RATIOS", "40,70,100,130")):
            rv = int(r)
            if not 0 <= rv <= C00_06_INERTIA_RATIO_MAX:
                raise gcmd.error(
                    "ratio %d outside C00.06 range 0..%d (%%)"
                    % (rv, C00_06_INERTIA_RATIO_MAX)
                )
            if rv not in ratios:
                ratios.append(rv)
        ratios.sort()
        original = self._read_param(servos[0], INERTIA_RATIO_ADDR)
        stroke_plan = {
            "start": start,
            "end": end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "inertia_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = InertiaRatioAdapter(self, servos, tag, original)

        def on_revert() -> None:
            gcmd.respond_info(
                "restoring C00.06 ratio %d%% on %s"
                % (original, ", ".join(servos))
            )

        try:
            self._prep(axis, dwell)
            steps = self._run_sweep_with_revert(
                adapter,
                ratios,
                servos,
                lambda rv: self._strokes(
                    axis, start, end, speed, accel, iterations, dwell
                ),
                gcmd,
                on_revert,
            )
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            self._active_run = None
        return steps

    cmd_SERVO_SWEEP_ACCEL_help = (
        "Accel sweep to find the max non-saturating acceleration. Runs one "
        "capture of strokes per ACCELS entry (mm/s^2, comma list, toolhead "
        "frame) named step_<TAG>_a<ACCEL>, then servo-cal analyzes the run "
        "into results.json (verdict: the highest non-railing accel). "
        "AXIS=X/Y strokes a single axis; AXIS=A/B strokes a CoreXY diagonal so "
        "one motor carries the whole load (belt accel is sqrt(2)x on a "
        "diagonal). Restores the velocity limit afterwards (also on failure). "
        "servo-cal flags samples at/above its 1400 per-mille torque ceiling "
        "as railed. APPLY=1 has no register to write (ACCEL is a stroke-plan "
        "parameter, not an SDO), so it runs the verification stroke at the "
        "recommended accel and reports before/after tracking metrics "
        "(default APPLY=0, report-only). "
        "Params ACCELS AXIS SPEED START END ITERATIONS DWELL_MS TAG APPLY"
    )

    def cmd_SERVO_SWEEP_ACCEL(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "accel")
        apply = gcmd.get_int("APPLY", 0)
        raw = self._floats(gcmd.get("ACCELS", None))
        if not raw:
            raise gcmd.error("ACCELS= required (comma list of mm/s^2)")
        accels: list[int] = []
        for a in raw:
            av = int(a)
            if av <= 0:
                raise gcmd.error("accel %d must be positive (mm/s^2)" % (av,))
            if av not in accels:
                accels.append(av)
        accels.sort()
        servos = plan.servos
        stroke_plan = {
            "start": plan.start,
            "end": plan.end,
            "speed": speed,
            "accel": None,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "accel_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = MotionAccelAdapter(tag)

        def run_step(av: int) -> None:
            servo_strokes.emit_strokes(
                self.gcode,
                plan.coord,
                plan.start,
                plan.end,
                plan.th_per_unit,
                speed,
                float(av),
                iterations,
                dwell,
            )

        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            try:
                steps = self._engine.run(
                    adapter, accels, servos, run_step, gcmd
                )
            finally:
                self._restore()
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            self._active_run = None
        return steps

    cmd_SERVO_SET_STIFFNESS_help = (
        "Vendor-table tuning - switch to standard mode (C00.04=1) and set "
        "C00.05 stiffness level 1..31. Params LEVEL SERVO"
    )

    def cmd_SERVO_SET_STIFFNESS(self, gcmd: Any) -> None:
        servo = self._servo(gcmd)
        level = gcmd.get_int(
            "LEVEL",
            minval=C00_05_STIFFNESS_LEVEL_MIN,
            maxval=C00_05_STIFFNESS_LEVEL_MAX,
        )
        self.gcode.run_script_from_command(
            "\n".join(
                [
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x05 VALUE=1 TYPE=u16"
                    % servo,
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x06 VALUE=%d TYPE=u16"
                    % (servo, level),
                ]
            )
        )
        self.cmd_SERVO_SHOW_TUNING(gcmd)

    cmd_SERVO_AUTOTUNE_help = (
        "Packaged tuning sequence: baseline tracking -> inertia ratio "
        "identify -> apply C00.06 -> coarse gains (SERVO_APPLY_GAINS "
        "defaults) -> gain sweep (apply winner) -> refine speed gain "
        "(apply winner) -> fit dynamics -> verify vs baseline. APPLY=0 "
        "(default) is a dry run: it still measures the baseline and "
        "identifies the inertia ratio, then walks every remaining stage "
        "reporting what it WOULD write instead of touching the drive. "
        "APPLY=1 performs every stage for real and aborts loudly, naming "
        "the stage and run directory, on a torque/resonance flag on the "
        "chosen step, a null recommendation, or a final following-error "
        "regression over 20%% vs baseline. Never persists the result - "
        "run SERVO_SAVE_TUNING SERVO=... NAME=... afterwards. Params AXIS "
        "APPLY TORQUE_NM INERTIA_KGM2 SPEED_GAINS DWELL_MS"
    )

    def cmd_SERVO_AUTOTUNE(self, gcmd: Any) -> list[dict[str, Any]]:
        axis = gcmd.get("AXIS", "X").upper()
        apply = bool(gcmd.get_int("APPLY", 0))
        torque, inertia = self._motor(gcmd, required=False)
        if apply and (torque is None or inertia is None):
            raise gcmd.error(
                "SERVO_AUTOTUNE APPLY=1 requires rated_torque_nm/"
                "rotor_inertia_kgm2 (config or TORQUE_NM=/INERTIA_KGM2=) "
                "before the inertia_ratio stage runs"
            )
        ctx = AutotuneContext(
            gcmd=gcmd,
            axis=axis,
            apply=apply,
            torque_nm=torque,
            inertia_kgm2=inertia,
            speed_gains=gcmd.get("SPEED_GAINS", None),
            dwell_ms=gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0),
        )
        outcomes: list[dict[str, Any]] = []
        for stage in AUTOTUNE_STAGES:
            outcome = stage.run(self, ctx)
            structured_log.event(
                "calibration",
                "autotune_stage",
                stage=stage.name,
                run_dir=outcome.get("run_dir"),
                outcome=outcome.get("outcome"),
            )
            gcmd.respond_info(
                "autotune stage %s: %s" % (stage.name, outcome.get("outcome"))
            )
            outcomes.append({"stage": stage.name, **outcome})
        gcmd.respond_info(
            "\n".join(
                ["SERVO_AUTOTUNE summary:"]
                + ["  %-20s %s" % (o["stage"], o["outcome"]) for o in outcomes]
            )
        )
        if apply:
            gcmd.respond_info(
                "nothing persisted - run SERVO_SAVE_TUNING SERVO=... "
                "NAME=... to keep this result"
            )
        return outcomes


def load_config(config: Any) -> ServoCalibration:
    config.get_printer().load_object(config, "servo_tuning")
    return ServoCalibration(config)
