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
    "torque_filter": (
        "0x2001.0x19",
        5,
        16000,
        "C01.18 torque feedforward filter cutoff",
        "Hz",
        1.0,
    ),
}

GAIN_LIST_PARAMS = {
    "POS_GAINS": "position",
    "SPEED_GAINS": "speed",
    "INTEGRALS": "integral",
    "TORQUE_FILTERS": "torque_filter",
}

INERTIA_RATIO_ADDR = "0x2000.0x07"
C00_06_INERTIA_RATIO_MAX = 12000

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


def validate_gain_values(values: list[int], param: str) -> list[int]:
    if param not in GAIN_PARAMS:
        raise ValueError(
            "PARAM must be one of %s (got %r)" % (", ".join(GAIN_PARAMS), param)
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
    "DIRECTION_SPLIT": "ferr_mean_direction_imbalance",
}
DYNAMICS_TERM_KEYS = {
    "MASS": "mass",
    "VISCOUS": "viscous",
    "COULOMB": "coulomb",
}
TUNE_RELATIVE_CLAMP = 0.4
TUNE_MASS_FLOOR_FRACTION = 0.10
TUNE_ZERO_FLOOR_STEPS = {"VISCOUS": 0.05, "COULOMB": 5.0}
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
    for key in ("direction_split", "orientation"):
        if key in data:
            raise ValueError(
                "profile %s is not a global field; direction split is only "
                "defined by ordered [[pair]] records" % (key,)
            )
    axes = data.get("axes")
    if not isinstance(axes, list) or not axes:
        raise ValueError("profile axes must be a non-empty list")
    if any(not isinstance(axis, str) or not axis.strip() for axis in axes):
        raise ValueError("profile axes must contain only non-empty strings")
    if len(set(axes)) != len(axes):
        raise ValueError("profile axes must be unique (got %s)" % (axes,))
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
    axis_names = list(axes)
    parsed_frame = [[float(v) for v in row] for row in frame]
    pairs = _parse_dynamics_pairs(data, axis_names)
    columns = [list(col) for col in zip(*parsed_frame)]
    axis_index = {name: i for i, name in enumerate(axis_names)}
    for pair in pairs:
        first, second = (axis_index[name] for name in pair["slots"])
        if not _equal_or_opposite_columns(columns[first], columns[second]):
            raise ValueError(
                "pair slots %s must have exact equal or opposite frame columns"
                % (pair["slots"],)
            )
    return {
        "axes": axis_names,
        "modes": [str(m) for m in modes],
        "frame": parsed_frame,
        "mass": [float(v) for v in data["mass"]],
        "viscous": [float(v) for v in data["viscous"]],
        "coulomb": [float(v) for v in data["coulomb"]],
        "pairs": pairs,
    }


def _parse_dynamics_pairs(
    data: Mapping[str, Any], axis_names: list[str]
) -> list[dict[str, Any]]:
    raw = data.get("pair", [])
    if not isinstance(raw, list):
        raise ValueError("profile pair must be an array of tables")
    axis_set = set(axis_names)
    claimed: set[str] = set()
    pairs: list[dict[str, Any]] = []
    for entry in raw:
        if not isinstance(entry, Mapping):
            raise ValueError("each pair must be a table")
        if "orientation" in entry:
            raise ValueError(
                "pair orientation is not supported; slots order defines the sign"
            )
        slots = entry.get("slots")
        if (
            not isinstance(slots, list)
            or len(slots) != 2
            or not all(isinstance(s, str) for s in slots)
        ):
            raise ValueError(
                "pair slots must be a list of two motor names (got %r)"
                % (slots,)
            )
        first, second = slots
        if first == second:
            raise ValueError(
                "pair slots must name two distinct motors (got %r)" % (slots,)
            )
        for name in slots:
            if name not in axis_set:
                raise ValueError(
                    "pair slot %r is not among profile axes %s"
                    % (name, axis_names)
                )
            if name in claimed:
                raise ValueError(
                    "motor %r appears in more than one pair" % (name,)
                )
        value = entry.get("direction_split")
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError(
                "pair direction_split must be a finite number (got %r)"
                % (value,)
            )
        if abs(value) >= 0.5:
            raise ValueError(
                "pair direction_split must satisfy abs(value) < 0.5 (got %r)"
                % (value,)
            )
        claimed.update(slots)
        pairs.append(
            {
                "slots": [str(first), str(second)],
                "direction_split": float(value),
            }
        )
    return pairs


def _copy_dynamics(profile: dict[str, Any]) -> dict[str, Any]:
    return {
        "axes": list(profile["axes"]),
        "modes": list(profile["modes"]),
        "frame": [list(row) for row in profile["frame"]],
        "mass": list(profile["mass"]),
        "viscous": list(profile["viscous"]),
        "coulomb": list(profile["coulomb"]),
        "pairs": [
            {
                "slots": list(pair["slots"]),
                "direction_split": float(pair["direction_split"]),
            }
            for pair in profile.get("pairs", [])
        ],
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


def add_dynamics_direction_split(
    profile: dict[str, Any], pair_index: int, delta: float
) -> dict[str, Any]:
    refined = _copy_dynamics(profile)
    pair = refined["pairs"][pair_index]
    value = pair["direction_split"] + delta
    if not math.isfinite(value) or abs(value) >= 0.5:
        raise ValueError(
            "direction_split candidate must satisfy abs(value) < 0.5 "
            "(got %r)" % (value,)
        )
    pair["direction_split"] = value
    return refined


def send_dynamics_model(
    engine: Any, handle: int, profile: dict[str, Any]
) -> None:
    axis_index = {name: i for i, name in enumerate(profile["axes"])}
    pair_slots: list[int] = []
    direction_split: list[float] = []
    for pair in profile.get("pairs", []):
        first, second = pair["slots"]
        pair_slots += [axis_index[first], axis_index[second]]
        direction_split.append(float(pair["direction_split"]))
    engine.set_dynamics_model(
        handle,
        [float(f) for row in profile["frame"] for f in row],
        [float(v) for v in profile["mass"]],
        [float(v) for v in profile["viscous"]],
        [float(v) for v in profile["coulomb"]],
        pair_slots,
        direction_split,
    )


def dynamics_torque_changes(
    prev: dict[str, Any],
    new: dict[str, Any],
    accel_mm_s2: float,
    speed_mm_s: float,
) -> list[float]:
    """Per-mode relative parameter change, weighted into torque units at the
    excitation ceiling (0.1% rated): |dm|*a + |db|*v + |dc| over the previous
    model's total feedforward there. A raw per-term ratio would flag a
    physically negligible flap of a near-zero viscous term as divergence."""
    if prev["modes"] != new["modes"]:
        raise ValueError(
            "fit rounds disagree on modes: %s vs %s"
            % (prev["modes"], new["modes"])
        )
    changes = []
    for k in range(len(prev["modes"])):
        ref = (
            abs(prev["mass"][k]) * accel_mm_s2
            + abs(prev["viscous"][k]) * speed_mm_s
            + abs(prev["coulomb"][k])
        )
        if ref <= 0.0:
            raise ValueError(
                "mode %s fitted to zero feedforward torque at accel %g / "
                "speed %g - degenerate fit"
                % (prev["modes"][k], accel_mm_s2, speed_mm_s)
            )
        delta = (
            abs(new["mass"][k] - prev["mass"][k]) * accel_mm_s2
            + abs(new["viscous"][k] - prev["viscous"][k]) * speed_mm_s
            + abs(new["coulomb"][k] - prev["coulomb"][k])
        )
        changes.append(delta / ref)
    return changes


def dynamics_tune_step(
    value: float,
    gradient: float,
    step_frac: float,
    prev: tuple[float, float] | None,
    zero_floor_step: float = 0.0,
) -> float:
    """One SERVO_TUNE_DYNAMICS update for a single (term, mode) ferr-fit
    coefficient: a positive gradient means the term is too low (ferr grows
    WITH the regressor, so the feedforward under-feeds), so the first
    probe steps by step_frac*|value| in sign(gradient) (or
    zero_floor_step, when value is exactly zero, so a zero-valued term
    can still probe a direction); every later probe secants to the
    empirical (value, gradient) zero-crossing between this probe and the
    previous one. Callers own clamping the result."""
    if prev is None:
        magnitude = zero_floor_step if value == 0.0 else step_frac * abs(value)
        sign = 1.0 if gradient > 0.0 else -1.0 if gradient < 0.0 else 0.0
        return value + sign * magnitude
    prev_value, prev_gradient = prev
    if gradient == prev_gradient:
        raise ValueError(
            "degenerate secant sensitivity: gradient stayed %.6g across "
            "probes %.6g -> %.6g" % (gradient, prev_value, value)
        )
    return value - gradient * (value - prev_value) / (gradient - prev_gradient)


def _frame_column_lambda(first: list[float], second: list[float]) -> int:
    if len(first) != len(second) or not any(value != 0.0 for value in first):
        raise ValueError("frame columns must be nonzero and equal length")
    if all(a == b for a, b in zip(first, second)):
        return 1
    if all(b == -a for a, b in zip(first, second)):
        return -1
    raise ValueError("frame columns are not exactly equal or opposite")


def _equal_or_opposite_columns(a: list[float], b: list[float]) -> bool:
    try:
        _frame_column_lambda(a, b)
    except ValueError:
        return False
    return True


def _direction_split_lambda(profile: dict[str, Any], slots: list[str]) -> int:
    axis_index = {name: i for i, name in enumerate(profile["axes"])}
    try:
        first_index, second_index = (axis_index[name] for name in slots)
    except (KeyError, ValueError) as e:
        raise ValueError(
            "direction-split pair must name two profile slots (got %r)"
            % (slots,)
        ) from e
    columns = [list(column) for column in zip(*profile["frame"])]
    try:
        return _frame_column_lambda(columns[first_index], columns[second_index])
    except ValueError as e:
        raise ValueError(
            "direction-split pair %s has invalid frame columns: %s" % (slots, e)
        ) from e


def _direction_split_moves(
    step_name: str, drive_name: str, step_drives: Mapping[str, Any]
) -> dict[Any, Mapping[str, Any]]:
    if drive_name not in step_drives:
        raise ValueError(
            "step %s is missing direction-split drive %s"
            % (step_name, drive_name)
        )
    moves = (step_drives[drive_name].get("metrics") or {}).get("moves") or []
    indexed = {}
    for move in moves:
        if not isinstance(move, Mapping):
            raise ValueError(
                "step %s drive %s has a malformed move entry %r"
                % (step_name, drive_name, move)
            )
        window = []
        for window_key in ("start_ms", "end_ms"):
            value = move.get(window_key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(
                    "step %s drive %s move %r has invalid %s %r"
                    % (
                        step_name,
                        drive_name,
                        move.get("move"),
                        window_key,
                        value,
                    )
                )
            window.append(value)
        window = tuple(window)
        if window in indexed:
            raise ValueError(
                "step %s drive %s repeats move window %r"
                % (step_name, drive_name, window)
            )
        indexed[window] = move
    return indexed


def direction_split_candidate_metrics(
    profile: dict[str, Any], step: Mapping[str, Any], slots: list[str]
) -> dict[str, float]:
    if len(slots) != 2:
        raise ValueError(
            "direction-split scoring requires two slots (got %r)" % (slots,)
        )
    step_name = str(step.get("name", "<unnamed>"))
    pair_lambda = _direction_split_lambda(profile, slots)
    step_drives = step.get("drives") or {}
    first_moves = _direction_split_moves(step_name, slots[0], step_drives)
    second_moves = _direction_split_moves(step_name, slots[1], step_drives)
    shared_windows = sorted(first_moves.keys() & second_moves.keys())
    directional_q: dict[int, list[float]] = {1: [], -1: []}
    for window in shared_windows:
        first = first_moves[window]
        second = second_moves[window]
        first_direction = first.get("direction")
        second_direction = second.get("direction")
        if first_direction not in (-1, 1):
            raise ValueError(
                "step %s drive %s move window %r has nonmoving direction %r"
                % (step_name, slots[0], window, first_direction)
            )
        if second_direction not in (-1, 1):
            raise ValueError(
                "step %s drive %s move window %r has nonmoving direction %r"
                % (step_name, slots[1], window, second_direction)
            )
        expected_direction = pair_lambda * first_direction
        if second_direction != expected_direction:
            raise ValueError(
                "step %s pair %s move window %r directions do not match lambda %d: "
                "%r vs %r"
                % (
                    step_name,
                    slots,
                    window,
                    pair_lambda,
                    first_direction,
                    second_direction,
                )
            )
        for drive_name, move in zip(slots, (first, second)):
            value = move.get("ferr_mean_moving")
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(
                    "step %s drive %s move window %r has invalid "
                    "ferr_mean_moving %r"
                    % (step_name, drive_name, window, value)
                )
        q = first["ferr_mean_moving"] - pair_lambda * second["ferr_mean_moving"]
        directional_q[first_direction].append(q)
    missing = [direction for direction, q in directional_q.items() if not q]
    if missing:
        raise ValueError(
            "step %s pair %s needs moves in both first-drive directions; "
            "missing %s" % (step_name, slots, missing)
        )
    q_plus = sum(directional_q[1]) / len(directional_q[1])
    q_minus = sum(directional_q[-1]) / len(directional_q[-1])
    return {
        "q_plus": q_plus,
        "q_minus": q_minus,
        "ferr_mean_direction_imbalance": abs(q_plus + q_minus) / 2.0,
    }


def discover_dynamics_pairs(profile: dict[str, Any]) -> list[dict[str, Any]]:
    columns = [list(col) for col in zip(*profile["frame"])]
    remaining = set(range(len(columns)))
    pairs = []
    for first in range(len(columns)):
        if first not in remaining:
            continue
        if not any(value != 0.0 for value in columns[first]):
            continue
        group = [
            i
            for i in sorted(remaining)
            if _equal_or_opposite_columns(columns[first], columns[i])
        ]
        if len(group) > 2:
            names = [profile["axes"][i] for i in group]
            raise ValueError(
                "ambiguous equal/opposite frame column group %s; expected "
                "exactly two slots per pair" % (names,)
            )
        if len(group) == 2:
            remaining.difference_update(group)
            pairs.append(
                {
                    "slots": [profile["axes"][i] for i in group],
                    "direction_split": 0.0,
                }
            )
    return pairs


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
    provenance_key = (
        "refined_delta" if term == "DIRECTION_SPLIT" else "refined_scale"
    )
    for suffix, scale in sorted(scales.items()):
        lines.append(
            "%s%s = %s"
            % (
                provenance_key,
                "_%s" % (suffix,) if suffix else "",
                num(scale),
            )
        )
    lines.append("refined_run = %s" % (json.dumps(run_dir),))
    for pair in profile.get("pairs", []):
        lines += [
            "",
            "[[pair]]",
            "slots = %s" % (json.dumps(pair["slots"]),),
            "direction_split = %s" % (num(pair["direction_split"]),),
        ]
    return "\n".join(lines) + "\n"


def render_fit_dynamics_toml(
    applied: dict[str, Any],
    fitted: dict[str, Any],
    terms: list[str],
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
        "axes = %s" % (json.dumps(applied["axes"]),),
        "modes = %s" % (json.dumps(applied["modes"]),),
        "frame = [%s]" % (", ".join(vec(row) for row in applied["frame"]),),
        "mass = %s" % (vec(applied["mass"]),),
        "viscous = %s" % (vec(applied["viscous"]),),
        "coulomb = %s" % (vec(applied["coulomb"]),),
        "applied_terms = %s" % (json.dumps([t.lower() for t in terms]),),
    ]
    for key in ("mass", "viscous", "coulomb"):
        if applied[key] != fitted[key]:
            lines.append("fitted_%s = %s" % (key, vec(fitted[key])))
    lines.append("fit_run = %s" % (json.dumps(run_dir),))
    for pair in applied.get("pairs", []):
        lines += [
            "",
            "[[pair]]",
            "slots = %s" % (json.dumps(pair["slots"]),),
            "direction_split = %s" % (num(pair["direction_split"]),),
        ]
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
    if not math.isfinite(lo) or not math.isfinite(hi) or not lo < hi:
        raise ValueError("bracket must satisfy finite LO < HI")
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
    stops: list[float] | None = None


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
        entry = {
            "name": step.name,
            "swept": step.swept,
            "applied": step.applied,
            "capture": os.path.basename(self.step_scap(step.name)),
            "accel": step.accel,
        }
        if step.stops is not None:
            entry["stops"] = step.stops
        self.manifest["steps"].append(entry)
        self.write()


class SingleGainAdapter:
    """SERVO_CALIBRATE_GAINS: sweeps one gain, holding the others fixed."""

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
        return "sweep %s step %d/%d: %s = %d (%.4g %s)%s on %s" % (
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
        values = dict(self._original)
        values[self.param] = value
        self._cal._write_gains(self.servos, values)
        swept = {self.param: value}
        applied = self._cal._gain_write_records(self.servos, values)
        return swept, applied

    def revert(self) -> None:
        self._cal._write_gains(self.servos, self._original)


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
        value_name: str = "scale",
    ):
        self._engine = engine
        self._handle = handle
        self.baseline = baseline
        self._scale_fn = scale_fn
        self.label = label
        self.tag = tag
        self.value_name = value_name
        self.applied = False

    def step_name(self, scale: float) -> str:
        return "%s_%s_s%04d" % (self.tag, self.label, round(scale * 1000))

    def describe(
        self, i: int, scale: float, total: int, servos: list[str]
    ) -> str:
        return "dynamics %s eval %d: %s %.4f on %s" % (
            self.label,
            i + 1,
            self.value_name,
            scale,
            ", ".join(servos),
        )

    def scaled(self, scale: float) -> dict[str, Any]:
        return self._scale_fn(self.baseline, scale)

    def apply(self, scale: float) -> ApplyResult:
        self._send(self.scaled(scale))
        self.applied = True
        return {self.value_name: scale}, []

    def revert(self) -> None:
        self._send(self.baseline)

    def _send(self, profile: dict[str, Any]) -> None:
        send_dynamics_model(self._engine, self._handle, profile)


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
            "SERVO_MEASURE_RINGDOWN",
            "SERVO_DIFF_DAMPER",
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
            "SERVO_REFINE_DYNAMICS",
            "SERVO_TUNE_DYNAMICS",
            "SERVO_SWEEP_INERTIA",
            "SERVO_SWEEP_ACCEL",
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
            "spatial": servo_strokes.spatial_frame(kin),
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
        return servo_strokes.belt_pair(
            self.printer, gcmd, self._kin(), belt, cmd_name
        )

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

    RINGDOWN_MIN_DWELL_MS = 500
    RINGDOWN_DEFAULT_DWELL_MS = 1500
    RINGDOWN_DEFAULT_CRUISE_MS = 200

    cmd_SERVO_MEASURE_RINGDOWN_help = (
        "Ring-down resonance measurement: short strokes centered on the "
        "axis - accelerate to speed, cruise CRUISE_MS so the accel "
        "transient settles, then a full stop - with post-processors "
        "bypassed and jerk limiting lifted so the stop excites the raw "
        "closed-loop plant. One step per SPEEDS entry; each stroke's "
        "commanded-stop time is recorded. servo-cal fits the post-stop "
        "residual vibration (servo encoders + optional accelerometer) for "
        "per-mode frequency and damping ratio - the free decay a drive "
        "cannot compensate the way it fights a steady sweep. Params "
        "AXIS=X|Y|A|B SPEEDS ACCEL ITERATIONS DWELL_MS CRUISE_MS "
        "ACCEL_CHIP TAG"
    )

    def _ringdown_dynamics(self, gcmd: Any, engine: Any) -> tuple[float, float]:
        """(accel, max_velocity). ACCEL defaults to the printer's effective
        max accel — the sharpest stop excites the widest band (the decel
        pulse's spectral null sits at a/v). Asking for more than the
        machine allows fails loudly: SET_VELOCITY_LIMIT is a cap that
        silently min()s with [printer] max_accel, which would shallow the
        decel AND break the stroke-length math."""
        max_velocity, max_accel, _deviation = engine.effective_limits()
        accel = gcmd.get_float("ACCEL", max_accel, above=0.0)
        if accel > max_accel:
            raise gcmd.error(
                "ACCEL %.0f exceeds the printer's max accel %.0f - the "
                "runtime cap can only lower it, so the strokes would "
                "silently run shallower; raise [printer] max_accel instead"
                % (accel, max_accel)
            )
        return accel, max_velocity

    def _ringdown_strokes(
        self,
        gcmd: Any,
        plan: Any,
        accel: float,
        max_velocity: float,
        cruise_ms: int,
    ) -> list[tuple[int, float, float, float]]:
        """(speed, start_u, end_u, length_mm) per step: the shortest
        centered stroke that reaches cruise speed and holds it for
        `cruise_ms` before the stop."""
        speeds_raw = self._floats(gcmd.get("SPEEDS", None)) or list(self.speeds)
        speeds: list[int] = []
        for s in speeds_raw:
            sv = int(round(s))
            if sv <= 0:
                raise gcmd.error("speed %d must be positive (mm/s)" % (sv,))
            if sv > max_velocity:
                raise gcmd.error(
                    "speed %d exceeds the printer's max velocity %.0f - "
                    "the stroke would silently cruise slower than the "
                    "step claims" % (sv, max_velocity)
                )
            if sv not in speeds:
                speeds.append(sv)
        speeds.sort()
        center_u = (plan.start + plan.end) / 2.0
        avail_half_u = (plan.end - plan.start) / 2.0
        strokes = []
        for speed in speeds:
            length_mm = speed * speed / accel + speed * cruise_ms / 1000.0
            half_u = length_mm / (2.0 * plan.th_per_unit)
            if half_u > avail_half_u:
                raise gcmd.error(
                    "%d mm/s needs a %.1f mm stroke (%.1f mm accel+decel + "
                    "%.1f mm cruise) but only %.1f mm fit around the center "
                    "- lower SPEEDS or CRUISE_MS, or widen START/END"
                    % (
                        speed,
                        length_mm,
                        speed * speed / accel,
                        speed * cruise_ms / 1000.0,
                        2.0 * avail_half_u * plan.th_per_unit,
                    )
                )
            strokes.append(
                (speed, center_u - half_u, center_u + half_u, length_mm)
            )
        return strokes

    def cmd_SERVO_MEASURE_RINGDOWN(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        engine = self.printer.lookup_object("motion_engine")
        accel, max_velocity = self._ringdown_dynamics(gcmd, engine)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int(
            "DWELL_MS",
            max(self.dwell_ms, self.RINGDOWN_DEFAULT_DWELL_MS),
            minval=self.RINGDOWN_MIN_DWELL_MS,
        )
        cruise_ms = gcmd.get_int(
            "CRUISE_MS", self.RINGDOWN_DEFAULT_CRUISE_MS, minval=0
        )
        strokes = self._ringdown_strokes(
            gcmd, plan, accel, max_velocity, cruise_ms
        )
        tag = gcmd.get("TAG", "ringdown")
        chip, chip_name = self._accel_chip(gcmd)
        servos = plan.servos
        stroke_plan = {
            "center": (plan.start + plan.end) / 2.0,
            "speed": None,
            "speeds": [s for s, _, _, _ in strokes],
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
            "cruise_ms": cruise_ms,
            "accel_chip": chip_name,
        }
        run = self._begin_run(
            gcmd,
            "ringdown",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            engine.set_post_processor_bypass(True)
            try:
                engine.set_jerk_override(float("inf"))
                try:
                    for i, (speed, start_u, end_u, length_mm) in enumerate(
                        strokes
                    ):
                        name = "%s_v%d" % (tag, speed)
                        gcmd.respond_info(
                            "ringdown %d/%d: %s at %d mm/s, accel %.0f "
                            "mm/s^2, %.1f mm stroke, %d stops"
                            % (
                                i + 1,
                                len(strokes),
                                axis,
                                speed,
                                accel,
                                length_mm,
                                iterations * 2,
                            )
                        )
                        servo_strokes.goto(
                            self.gcode,
                            self.travel_speed,
                            plan.coord(start_u),
                            dwell,
                        )
                        self._start_capture(name, servos)
                        aclient = (
                            None
                            if chip is None
                            else chip.start_internal_client()
                        )
                        try:
                            stops = servo_strokes.emit_strokes_with_stop_times(
                                self.printer,
                                self.gcode,
                                plan.coord,
                                start_u,
                                end_u,
                                plan.th_per_unit,
                                float(speed),
                                accel,
                                iterations,
                                dwell,
                            )
                            self._stop_capture()
                        finally:
                            if aclient is not None:
                                aclient.finish_measurements()
                        step = SweepStep(
                            name,
                            {"speed": float(speed), "stroke_mm": length_mm},
                            [],
                            stops=stops,
                        )
                        if aclient is not None:
                            assert chip_name is not None, (
                                "accel client exists without a chip name"
                            )
                            step.accel = os.path.basename(
                                self._write_accel_csv(
                                    gcmd, aclient, chip_name, name
                                )
                            )
                        run.record_step(step)
                finally:
                    engine.set_jerk_override(None)
            finally:
                engine.set_post_processor_bypass(False)
                self._restore()
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
        "X_START/X_END/Y_START/Y_END. PATTERN=1 runs each ACCELS x SPEEDS "
        "cell as a TEST_SPEED-style XY pattern over the configured XY "
        "bounds inset by BOUND (plus a SMALL_SIZE box at center) exciting "
        "every XY servo; segments too short to reach SPEED run triangular "
        "profiles on purpose and are reported by achieved peak velocity, "
        "and it rejects START/END/X_START/X_END/Y_START/Y_END. Params AXIS "
        "START END X_START X_END Y_START Y_END ACCELS SPEEDS ITERATIONS "
        "DWELL_MS NAME SERVOS PATTERN BOUND SMALL_SIZE"
    )

    def _grid_stroke_plan(self, gcmd: Any) -> dict[str, Any]:
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        plan = {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if gcmd.get_int("PATTERN", 0):
            _points, _sx, _sy, pattern_plan = self._pattern_geometry_params(
                gcmd
            )
            plan.update(pattern_plan)
        return plan

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
        if gcmd.get_int("PATTERN", 0):
            self._reject_pattern_stroke_bounds(gcmd)
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
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        if gcmd.get_int("PATTERN", 0):
            points, start_x, start_y, _plan = self._pattern_geometry_params(
                gcmd
            )
            self._pattern_grid(gcmd, name, servos, points, start_x, start_y)
            return
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
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
        if gcmd.get_int("PATTERN", 0):
            points, start_x, start_y, _plan = self._pattern_geometry_params(
                gcmd
            )
            self._pattern_grid(gcmd, name, servo_list, points, start_x, start_y)
            return
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

    def _reject_pattern_stroke_bounds(self, gcmd: Any) -> None:
        bad = [
            p
            for p in ("START", "END", "X_START", "X_END", "Y_START", "Y_END")
            if gcmd.get(p, None) is not None
        ]
        if bad:
            raise gcmd.error(
                "%s are single-axis stroke bounds - PATTERN=1 uses the "
                "configured XY bounds with BOUND= inset" % (", ".join(bad),)
            )

    def _pattern_reach_report(
        self,
        gcmd: Any,
        points: list[tuple[float, float]],
        start_x: float,
        start_y: float,
        accels: list[float],
        speeds: list[float],
    ) -> None:
        for accel in accels:
            for speed in speeds:
                moves = servo_strokes.pattern_moves(
                    self.gcode, points, start_x, start_y, speed, accel
                )
                gcmd.respond_info(
                    "accel %.0f speed %.0f: %s"
                    % (
                        accel,
                        speed,
                        servo_strokes.pattern_reach_summary(moves, speed),
                    )
                )

    def _pattern_grid(
        self,
        gcmd: Any,
        name: str,
        servos: list[str],
        points: list[tuple[float, float]],
        start_x: float,
        start_y: float,
    ) -> None:
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        self._prep("X", dwell)
        self._prep("Y", dwell)
        self._pattern_reach_report(
            gcmd, points, start_x, start_y, accels, speeds
        )
        self._goto_xy(start_x, start_y, dwell)
        self._start_capture(name, servos)
        for accel in accels:
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    accel,
                    iterations,
                    dwell,
                )
        self._stop_capture()
        self._restore()

    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward. On coupled_xy this "
        "is an iterative closed-loop identification: it runs the "
        "TEST_SPEED-style XY pattern (always - there is no PATTERN option), "
        "fits mass/viscous/coulomb (all three always regressed - the "
        "friction columns keep the mass estimate unbiased), streams the "
        "APPLIED model into the running endpoint, and re-captures with the "
        "feedforward active - "
        "with FF in the loop the drives track the command, so regressing "
        "measured torque against commanded kinematics loses its bias - "
        "until the parameters move less than TOL (torque-weighted, at the "
        "excitation ceiling) between rounds. It then re-identifies once at "
        "MAX_ACCEL: a converged model that shifts more than DRIFT there is "
        "a fit artifact, not physics, and the command aborts with the "
        "numbers. No SPEEDS matrix: give the calibration envelope as "
        "MAX_ACCEL/MAX_SPEED limits (e.g. capped below ringing; defaults "
        "are the config grid maxima) - convergence rounds run at half "
        "MAX_ACCEL, speeds at half and full MAX_SPEED. ACCELS=<comma list> "
        "runs an identify-only sweep instead: one capture + fit per accel "
        "under whatever model is currently live (nothing is streamed or "
        "applied), reporting mass per accel and the torque-weighted change "
        "between neighbours - the m(accel) curve that says whether the "
        "model extrapolates. The live model is "
        "restored to the configured dynamics_profile afterwards (also on "
        "failure); without one the last fitted model stays live until "
        "RESTART. TERMS picks what the applied/written model keeps "
        "(default MASS: with velocity_ff on, the speed-loop integrator "
        "already supplies friction torque at all but reversal transients, "
        "and a wrong friction FF is worse than none; fitted-but-dropped "
        "values are reported and recorded as fitted_* keys so enabling "
        "TERMS=MASS,COULOMB later is a data-driven call). Writes a "
        "timestamped node-level profile from the "
        "MAX_ACCEL verification fit. On non-coupled kinematics the "
        "single-shot per-axis grid fit remains (a per-motor candidate "
        "cannot be streamed into a multi-drive node), with params as "
        "SERVO_MEASURE_INERTIA plus DRIVE. Optional TORQUE_NM + "
        "INERTIA_KGM2 add the C00.06 recommendation. Params TERMS (MASS) "
        "MAX_ACCEL "
        "MAX_SPEED TOL (0.05) DRIFT (0.15) MAX_ROUNDS (4) ITERATIONS "
        "DWELL_MS BOUND SMALL_SIZE NAME SERVOS TORQUE_NM INERTIA_KGM2"
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

    def _fit_argv_for(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        scap: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
        response: str | None = None,
    ) -> list[str]:
        argv = [
            self._servo_cal(gcmd),
            "fit",
            "--capture",
            scap,
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
        if response is not None:
            argv += ["--response", response]
        return argv

    def _run_fit(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if self._kin().coupled_xy():
            if gcmd.get("ACCELS", None) is not None:
                return self._run_fit_sweep(gcmd, name, torque, inertia)
            return self._run_fit_iterative(gcmd, name, torque, inertia)
        return self._run_fit_grid(gcmd, name, torque, inertia)

    def _run_fit_grid(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if gcmd.get_int("PATTERN", 0):
            self._reject_pattern_stroke_bounds(gcmd)
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
            self._measure_inertia(gcmd, name)
            run.record_step(SweepStep(name, {}, []))
            out_path = self._dynamics_out_path(gcmd, run, name)
            argv = self._fit_argv_for(
                gcmd, plan, run.step_scap(name), out_path, torque, inertia
            )
            text = self._run(gcmd, argv, 120.0)
            gcmd.respond_info(
                "dynamics profile: %s | run %s" % (out_path, run.run_dir)
            )
        finally:
            self._active_run = None
        return run, text, out_path

    def _reject_fit_grid_params(self, gcmd: Any) -> None:
        stale = [
            p for p in ("SPEEDS", "PATTERN") if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: the iterative fit has no excitation matrix and always "
                "runs the XY pattern - give the calibration envelope as "
                "MAX_ACCEL/MAX_SPEED limits, or ACCELS=<comma list> for an "
                "identify-only sweep" % (", ".join(stale),)
            )

    def _validate_fit_slots(
        self, gcmd: Any, node: Any, profile: dict[str, Any]
    ) -> None:
        for slot, motor in enumerate(profile["axes"]):
            if node.get_slot_for_motor(motor) != slot:
                raise gcmd.error(
                    "fitted profile axis %r is at slot %d but node %s maps "
                    "it to %s - cannot stream the candidate model"
                    % (motor, slot, node.name, node.get_slot_for_motor(motor))
                )

    def _fit_round(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        run: ExperimentRun,
        step: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[dict[str, Any], str]:
        argv = self._fit_argv_for(
            gcmd, plan, run.step_scap(step), out_path, torque, inertia
        )
        text = self._run(gcmd, argv, 120.0)
        try:
            with open(out_path) as f:
                fitted = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit for step %s produced an unusable profile "
                "%s: %s" % (step, out_path, e)
            )
        return fitted, text

    def _dynamics_params_line(self, profile: dict[str, Any]) -> str:
        return " | ".join(
            "%s mass %.5g viscous %.5g coulomb %.5g"
            % (
                mode,
                profile["mass"][k],
                profile["viscous"][k],
                profile["coulomb"][k],
            )
            for k, mode in enumerate(profile["modes"])
        )

    def _run_fit_iterative(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_fit_grid_params(gcmd)
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        node = self._refine_dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        restore = None
        if node.get_dynamics_profile() is not None:
            _path, restore = self._load_baseline_dynamics(gcmd, node)
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        tol = gcmd.get_float("TOL", 0.05, above=0.0)
        drift = gcmd.get_float("DRIFT", 0.15, above=0.0)
        max_rounds = gcmd.get_int("MAX_ROUNDS", 4, minval=2)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS").split(",")
            if t.strip()
        ]
        if (
            not terms
            or any(t not in DYNAMICS_TERM_KEYS for t in terms)
            or "MASS" not in terms
        ):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB and include MASS (got %r)" % (gcmd.get("TERMS", ""),)
            )
        dropped = [
            key for term, key in DYNAMICS_TERM_KEYS.items() if term not in terms
        ]

        def applied_model(full: dict[str, Any]) -> dict[str, Any]:
            trimmed = _copy_dynamics(full)
            for key in dropped:
                trimmed[key] = [0.0] * len(full[key])
            return trimmed

        def round_line(full: dict[str, Any], trimmed: dict[str, Any]) -> str:
            line = self._dynamics_params_line(trimmed)
            if dropped:
                line += " | fitted but not applied: " + ", ".join(
                    "%s [%s]"
                    % (key, ", ".join("%.5g" % (v,) for v in full[key]))
                    for key in dropped
                )
            return line

        converge_accel = max_accel / 2.0
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "converge_accel": converge_accel,
            "speeds": speeds,
            "tol": tol,
            "drift": drift,
            "max_rounds": max_rounds,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_fit",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str, accel: float) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    accel,
                    iterations,
                    dwell,
                )
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": accel}, []))

        def torque_changes(
            prev: dict[str, Any],
            new: dict[str, Any],
            accel: float,
        ) -> list[float]:
            try:
                return dynamics_torque_changes(prev, new, accel, max_speed)
            except ValueError as e:
                raise gcmd.error(str(e))

        applied = False
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd,
                points,
                start_x,
                start_y,
                [converge_accel, max_accel],
                speeds,
            )
            prev = None
            fitted = None
            converged = False
            last_change = None
            rounds_run = 0
            for round_i in range(max_rounds):
                step = "fit_r%d" % (round_i,)
                capture_round(step, converge_accel)
                fitted_full, _text = self._fit_round(
                    gcmd,
                    plan,
                    run,
                    step,
                    os.path.join(run.run_dir, "dynamics_%s.toml" % (step,)),
                    None,
                    None,
                )
                fitted = applied_model(fitted_full)
                rounds_run = round_i + 1
                if round_i == 0:
                    self._validate_fit_slots(gcmd, node, fitted)
                send_dynamics_model(engine, handle, fitted)
                applied = True
                if prev is None:
                    gcmd.respond_info(
                        "round %d: %s (feedforward now live for the next "
                        "round)" % (round_i, round_line(fitted_full, fitted))
                    )
                else:
                    last_change = max(
                        torque_changes(prev, fitted, converge_accel)
                    )
                    gcmd.respond_info(
                        "round %d: %s | torque-weighted change %.1f%% "
                        "(TOL %.1f%%)"
                        % (
                            round_i,
                            round_line(fitted_full, fitted),
                            100.0 * last_change,
                            100.0 * tol,
                        )
                    )
                    if last_change <= tol:
                        converged = True
                        break
                prev = fitted
            if not converged:
                raise gcmd.error(
                    "dynamics fit did not converge in %d rounds at accel "
                    "%.0f (last torque-weighted change %.1f%% > TOL %.1f%%) "
                    "- the identification is not settling; inspect run %s"
                    % (
                        max_rounds,
                        converge_accel,
                        100.0
                        * (last_change if last_change is not None else 1.0),
                        100.0 * tol,
                        run.run_dir,
                    )
                )
            capture_round("fit_verify", max_accel)
            verified_full, text = self._fit_round(
                gcmd,
                plan,
                run,
                "fit_verify",
                os.path.join(run.run_dir, "dynamics_fit_verify.toml"),
                torque,
                inertia,
            )
            verified = applied_model(verified_full)
            shift = max(torque_changes(fitted, verified, max_accel))
            if shift > drift:
                raise gcmd.error(
                    "converged model does not hold at MAX_ACCEL %.0f: "
                    "re-identification shifted the parameters %.1f%% "
                    "(DRIFT %.1f%%) - converged %s vs verify %s | the fit "
                    "at accel %.0f was an artifact of that operating "
                    "point, not physics; lower MAX_ACCEL below the "
                    "regime change or investigate | run %s"
                    % (
                        max_accel,
                        100.0 * shift,
                        100.0 * drift,
                        self._dynamics_params_line(fitted),
                        self._dynamics_params_line(verified),
                        converge_accel,
                        run.run_dir,
                    )
                )
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        verified, verified_full, terms, run.run_dir
                    )
                )
            run.manifest["dynamics_fit"] = {
                "rounds": rounds_run,
                "converged_change": last_change,
                "verify_shift": shift,
                "terms": [t.lower() for t in terms],
                "fitted_not_applied": {
                    key: verified_full[key] for key in dropped
                },
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_fit",
                run_dir=run.run_dir,
                rounds=rounds_run,
                converged_change=last_change,
                verify_shift=shift,
                profile=out_path,
            )
            gcmd.respond_info(
                "converged in %d rounds (change %.1f%%), holds at MAX_ACCEL "
                "%.0f (shift %.1f%% <= DRIFT %.1f%%) | dynamics profile: %s "
                "| run %s"
                % (
                    rounds_run,
                    100.0 * (last_change or 0.0),
                    max_accel,
                    100.0 * shift,
                    100.0 * drift,
                    out_path,
                    run.run_dir,
                )
            )
        finally:
            try:
                if applied:
                    if restore is not None:
                        send_dynamics_model(engine, handle, restore)
                        gcmd.respond_info(
                            "live dynamics model restored to configured "
                            "baseline"
                        )
                    else:
                        gcmd.respond_info(
                            "WARNING: no dynamics_profile configured - the "
                            "last fitted model stays live until RESTART"
                        )
            finally:
                self._restore()
                self._active_run = None
        return run, text, out_path

    def _run_fit_sweep(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        """Identify-only m(accel) curve: one pattern capture + fit per
        ACCELS entry, run under whatever dynamics model is currently live
        (nothing is streamed), so the points differ only in accel."""
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        stale = [
            p
            for p in (
                "SPEEDS",
                "PATTERN",
                "TOL",
                "DRIFT",
                "MAX_ROUNDS",
                "MAX_ACCEL",
                "TERMS",
            )
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: ACCELS runs an identify-only sweep - it takes only "
                "MAX_SPEED, ITERATIONS, DWELL_MS, NAME, SERVOS and the "
                "pattern geometry" % (", ".join(stale),)
            )
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        raw = gcmd.get("ACCELS")
        try:
            accels = [float(v) for v in raw.split(",") if v.strip()]
        except ValueError:
            raise gcmd.error(
                "ACCELS must be a comma list of accelerations (got %r)" % (raw,)
            )
        if (
            len(accels) < 2
            or any(a <= 0.0 for a in accels)
            or sorted(accels) != accels
            or len(set(accels)) != len(accels)
        ):
            raise gcmd.error(
                "ACCELS wants at least two distinct ascending positive "
                "accelerations (got %r)" % (raw,)
            )
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "accels": accels,
            "max_speed": max_speed,
            "speeds": speeds,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_sweep",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )
        text = ""
        out_path = ""
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, accels, speeds
            )
            fits: list[tuple[float, dict[str, Any]]] = []
            for accel in accels:
                step = "fit_a%d" % (round(accel),)
                self._start_capture(step, plan["servos"])
                self._goto_xy(start_x, start_y, dwell)
                for speed in speeds:
                    servo_strokes.emit_pattern(
                        self.gcode,
                        points,
                        start_x,
                        start_y,
                        speed,
                        accel,
                        iterations,
                        dwell,
                    )
                self._stop_capture()
                run.record_step(SweepStep(step, {"accel": accel}, []))
                out_path = os.path.join(
                    run.run_dir, "dynamics_%s.toml" % (step,)
                )
                fitted, text = self._fit_round(
                    gcmd, plan, run, step, out_path, torque, inertia
                )
                fits.append((accel, fitted))
                gcmd.respond_info(
                    "accel %.0f: %s"
                    % (accel, self._dynamics_params_line(fitted))
                )
            for (a0, f0), (a1, f1) in zip(fits, fits[1:]):
                try:
                    change = max(dynamics_torque_changes(f0, f1, a1, max_speed))
                except ValueError as e:
                    raise gcmd.error(str(e))
                gcmd.respond_info(
                    "accel %.0f -> %.0f: torque-weighted change %.1f%%"
                    % (a0, a1, 100.0 * change)
                )
            modes = fits[0][1]["modes"]
            curve = {
                mode: [f[1]["mass"][k] for f in fits]
                for k, mode in enumerate(modes)
            }
            for mode in modes:
                masses = curve[mode]
                lo, hi = min(masses), max(masses)
                gcmd.respond_info(
                    "mode %s mass(accel): %s | spread %.1f%%"
                    % (
                        mode,
                        ", ".join(
                            "%.0f: %.5g" % (a, m)
                            for (a, _f), m in zip(fits, masses)
                        ),
                        200.0 * (hi - lo) / (hi + lo),
                    )
                )
            run.manifest["dynamics_sweep"] = {
                "accels": accels,
                "mass": curve,
                "max_speed": max_speed,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_sweep",
                run_dir=run.run_dir,
                accels=accels,
            )
            gcmd.respond_info(
                "identify-only sweep done - nothing was applied | run %s"
                % (run.run_dir,)
            )
        finally:
            self._restore()
            self._active_run = None
        return run, text, out_path

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=False)
        self._run_fit(gcmd, gcmd.get("NAME", "ident"), torque, inertia)

    def _reject_tune_dynamics_params(self, gcmd: Any) -> None:
        stale = [
            p
            for p in ("ACCELS", "SPEEDS", "PATTERN")
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: SERVO_TUNE_DYNAMICS always drives the XY pattern "
                "excitation at MAX_ACCEL/MAX_SPEED - it has no excitation "
                "matrix and no PATTERN toggle to override" % (", ".join(stale),)
            )

    def _load_ferr_fit(self, gcmd: Any, path: str) -> dict[str, Any]:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit --response ferr produced an unusable result "
                "%s: %s" % (path, e)
            )
        if data.get("version") != 1:
            raise gcmd.error(
                "ferr fit %s: unsupported version %r (expected 1)"
                % (path, data.get("version"))
            )
        return data

    cmd_SERVO_TUNE_DYNAMICS_help = (
        "Iterative, signed, closed-loop dynamics tuner on coupled_xy: each "
        "round captures one XY pattern run at MAX_ACCEL/MAX_SPEED, then "
        "servo-cal regresses the per-mode FOLLOWING ERROR (not the "
        "drive-reported torque) against the same commanded-kinematics "
        "regressor [accel, vel, sign(vel)] SERVO_FIT_DYNAMICS uses. The "
        "sign of each coefficient says which way the command path is "
        "wrong as seen by the closed loop: a positive mass coefficient "
        "means ferr grows WITH commanded accel, i.e. the feedforward "
        "under-feeds during accel (mass too low on the command path); "
        "viscous/coulomb read the same way against vel/sign(vel). Each "
        "round steps every TERMS coefficient still outside SIGMA stderrs "
        "of zero - sign-probe on the first round, secant on the empirical "
        "sensitivity afterwards (relative change clamped to 40%% per "
        "round; mass is floored at 10%% of its baseline value and a "
        "second push below that floor aborts as a degenerate fit) - and "
        "streams the updated model straight to the running endpoint "
        "(no restart) before the next round. Stops and writes a new "
        "dynamics TOML the moment every coefficient is statistically "
        "zero, leaving the tuned model LIVE (point [ethercat_node] "
        "dynamics_profile at it and RESTART to keep it past a klippy "
        "restart). If ROUNDS runs out unconverged the live model is "
        "restored to the baseline and the command fails loudly with the "
        "last coefficient table - no partially-tuned profile is ever "
        "written. A torque_saturated verdict flag aborts and restores "
        "the baseline the same way; resonance_detected only warns (a "
        "feedforward retune does not move the loop's resonances). The "
        "baseline is PROFILE= or the node-level [ethercat_node] "
        "dynamics_profile (per-motor profiles are not supported). Params "
        "MAX_ACCEL MAX_SPEED ROUNDS (6) STEP (0.15) SIGMA (3.0) TERMS "
        "(mass,viscous,coulomb) NAME (tune) PROFILE SERVOS BOUND SMALL_SIZE"
    )

    def cmd_SERVO_TUNE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_tune_dynamics_params(gcmd)
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires coupled_xy kinematics - the "
                "ferr regression needs the mode-space frame"
            )
        plan = self._fit_plan(gcmd)
        node = self._refine_dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        baseline_modes = baseline["modes"]
        if len(baseline_modes) != 2 or not {"x", "y"} <= set(baseline_modes):
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS needs a 2-mode profile with x and y "
                "modes; profile %s has modes %s"
                % (profile_path, baseline_modes)
            )
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS,VISCOUS,COULOMB").split(",")
            if t.strip()
        ]
        if not terms or any(t not in DYNAMICS_TERM_KEYS for t in terms):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB (got %r)" % (gcmd.get("TERMS", ""),)
            )
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        rounds = gcmd.get_int("ROUNDS", 6, minval=2)
        step_frac = gcmd.get_float("STEP", 0.15, minval=0.02, maxval=0.5)
        sigma = gcmd.get_float("SIGMA", 3.0, above=0.0)
        name = gcmd.get("NAME", "tune")
        dwell = self.dwell_ms
        iterations = self.iterations
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "speeds": speeds,
            "rounds": rounds,
            "step": step_frac,
            "sigma": sigma,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_tune",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    max_accel,
                    iterations,
                    dwell,
                )
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": max_accel}, []))

        current = _copy_dynamics(baseline)
        probes: dict[str, dict[str, list[tuple[float, float]]]] = {
            term: {mode: [] for mode in plan["modes"]} for term in terms
        }
        rounds_history: list[dict[str, Any]] = []
        applied = False
        success = False
        last_report = ""
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, [max_accel], speeds
            )
            for round_i in range(rounds):
                send_dynamics_model(engine, handle, current)
                applied = True
                step = "tune_r%d" % (round_i,)
                capture_round(step)
                results = self._run_analyze(gcmd, run, incremental=True)
                flags = set(self._step_flags(results, step))
                if "torque_saturated" in flags:
                    raise gcmd.error(
                        "step %s hit the torque rail - clipped strokes "
                        "cannot score a ferr fit, aborting "
                        "SERVO_TUNE_DYNAMICS" % (step,)
                    )
                if "resonance_detected" in flags:
                    gcmd.respond_info(
                        "WARNING step %s flagged resonance_detected - "
                        "continuing (feedforward tuning does not move the "
                        "loop's resonances)" % (step,)
                    )
                ferr_out = os.path.join(
                    run.run_dir, "ferr_r%d.json" % (round_i,)
                )
                argv = self._fit_argv_for(
                    gcmd,
                    plan,
                    run.step_scap(step),
                    ferr_out,
                    None,
                    None,
                    response="ferr",
                )
                self._run(gcmd, argv, 120.0)
                ferr = self._load_ferr_fit(gcmd, ferr_out)
                if ferr.get("modes") != plan["modes"]:
                    raise gcmd.error(
                        "servo-cal fit --response ferr modes %s do not "
                        "match the requested modes %s"
                        % (ferr.get("modes"), plan["modes"])
                    )
                coef = ferr["coef"]
                stderr_map = ferr["stderr"]
                values_after = {
                    DYNAMICS_TERM_KEYS[t]: list(current[DYNAMICS_TERM_KEYS[t]])
                    for t in terms
                }
                all_converged = True
                round_lines = []
                for term in terms:
                    key = DYNAMICS_TERM_KEYS[term]
                    for fit_idx, mode in enumerate(plan["modes"]):
                        baseline_idx = baseline_modes.index(mode)
                        g = coef[key][fit_idx]
                        se = stderr_map[key][fit_idx]
                        before = current[key][baseline_idx]
                        note = ""
                        if abs(g) <= sigma * se:
                            after = before
                        elif term != "MASS" and before == 0.0 and g < 0.0:
                            after = before
                            note = " (bounded at 0: loop wants negative %s)" % (
                                term.lower(),
                            )
                        else:
                            all_converged = False
                            history = probes[term][mode]
                            prev_probe = history[-1] if history else None
                            floor_step = TUNE_ZERO_FLOOR_STEPS.get(term, 0.0)
                            try:
                                candidate = dynamics_tune_step(
                                    before, g, step_frac, prev_probe, floor_step
                                )
                            except ValueError as e:
                                raise gcmd.error(str(e))
                            history.append((before, g))
                            if before != 0.0:
                                span = TUNE_RELATIVE_CLAMP * abs(before)
                                candidate = min(
                                    max(candidate, before - span), before + span
                                )
                            if term == "MASS":
                                floor = (
                                    TUNE_MASS_FLOOR_FRACTION
                                    * baseline[key][baseline_idx]
                                )
                                if candidate < floor:
                                    if before <= floor * (1.0 + 1e-9):
                                        raise gcmd.error(
                                            "mode %s mass update collapses "
                                            "below the %.0f%% baseline floor "
                                            "(%.6g) for the second time - "
                                            "the secant sensitivity is "
                                            "degenerate"
                                            % (
                                                mode,
                                                100.0
                                                * TUNE_MASS_FLOOR_FRACTION,
                                                floor,
                                            )
                                        )
                                    candidate = floor
                            elif candidate < 0.0:
                                candidate = 0.0
                            after = candidate
                            values_after[key][baseline_idx] = after
                        round_lines.append(
                            "mode %s %s: g=%+.4g (se %.4g) %.6g -> %.6g%s"
                            % (mode, term.lower(), g, se, before, after, note)
                        )
                report = " | ".join(round_lines)
                gcmd.respond_info("round %d: %s" % (round_i, report))
                last_report = report
                rounds_history.append(
                    {
                        "round": round_i,
                        "coef": {
                            DYNAMICS_TERM_KEYS[t]: list(
                                coef[DYNAMICS_TERM_KEYS[t]]
                            )
                            for t in terms
                        },
                        "stderr": {
                            DYNAMICS_TERM_KEYS[t]: list(
                                stderr_map[DYNAMICS_TERM_KEYS[t]]
                            )
                            for t in terms
                        },
                        "ferr_rms": list(ferr.get("ferr_rms", [])),
                        "samples": ferr.get("samples"),
                        "values": {
                            DYNAMICS_TERM_KEYS[t]: list(
                                values_after[DYNAMICS_TERM_KEYS[t]]
                            )
                            for t in terms
                        },
                    }
                )
                for term in terms:
                    key = DYNAMICS_TERM_KEYS[term]
                    current[key] = list(values_after[key])
                if all_converged:
                    success = True
                    break
            if not success:
                raise gcmd.error(
                    "SERVO_TUNE_DYNAMICS did not converge in %d rounds - "
                    "coefficients remain outside %.1f-sigma of zero: %s"
                    % (rounds, sigma, last_report)
                )
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        current,
                        current,
                        [t.lower() for t in terms],
                        run.run_dir,
                    )
                )
            run.manifest["dynamics_tune"] = {
                "terms": [t.lower() for t in terms],
                "max_accel": max_accel,
                "max_speed": max_speed,
                "sigma": sigma,
                "step": step_frac,
                "rounds": rounds_history,
                "converged": True,
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_tune",
                run_dir=run.run_dir,
                rounds=len(rounds_history),
                profile=out_path,
            )
            gcmd.respond_info(
                "SERVO_TUNE_DYNAMICS converged in %d rounds | tuned "
                "dynamics profile: %s | tuned model stays live until "
                "RESTART - point [ethercat_node %s] dynamics_profile at "
                "it to keep it | run %s"
                % (len(rounds_history), out_path, node.name, run.run_dir)
            )
        finally:
            try:
                if applied and not success:
                    send_dynamics_model(engine, handle, baseline)
                    gcmd.respond_info(
                        "live dynamics model restored to baseline %s"
                        % (profile_path,)
                    )
            finally:
                self._restore()
                self._active_run = None

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

    def _write_gains(self, servos: list[str], values: dict[str, int]) -> None:
        lines = [
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
            % (servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in values
        ]
        with servo_param.suppress_write_log():
            self.gcode.run_script_from_command("\n".join(lines))

    def _gain_write_records(
        self, servos: list[str], values: dict[str, int]
    ) -> list[dict[str, Any]]:
        return [
            _applied(servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in values
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
            for name in GAIN_PARAMS
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
        "0.1 Hz, INTEGRAL 0.01 ms, TORQUE_FILTER Hz (C01.18, only written "
        "when given). Params AXIS or SERVO (comma list)"
    )

    def cmd_SERVO_APPLY_GAINS(self, gcmd: Any) -> None:
        servos = self._servos(gcmd)
        values = {
            "position": gcmd.get_int("POS_GAIN", 400),
            "speed": gcmd.get_int("SPEED_GAIN", 250),
            "integral": gcmd.get_int("INTEGRAL", 3184),
        }
        torque_filter = gcmd.get("TORQUE_FILTER", None)
        if torque_filter is not None:
            values["torque_filter"] = int(torque_filter)
            validate_gain_values([values["torque_filter"]], "torque_filter")
        self._set_manual_tuning(servos)
        self._write_gains(servos, values)
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
        "Sweep of exactly one drive gain, shaper-calibrate style. Give one "
        "of POS_GAINS (0.1 rad/s units), SPEED_GAINS (0.1 Hz units, default "
        "500,650,800,1000), INTEGRALS (0.01 ms units) or TORQUE_FILTERS "
        "(C01.18 torque feedforward filter cutoff, Hz) as a comma list; "
        "the other params stay at their current drive values, so tune "
        "each one individually. Resolves every servo driving AXIS (both "
        "drives on CoreXY; they must agree on the current gains), writes "
        "each entry to all of them, one capture per step of all "
        "drives into a run directory, then servo-cal analyzes it into "
        "results.json with a typed verdict (the recommended step). "
        "With an accelerometer (accel_chip config option or ACCEL_CHIP=) "
        "each step also records vibration data next to its capture. Always "
        "restores the gains that were active before the sweep (also on "
        "failure). APPLY=1 writes the verdict's "
        "recommended gains after the restore, reads them back, and runs one "
        "SERVO_MEASURE_TRACKING to report before/after tracking metrics "
        "(default APPLY=0, report-only). SERVO= (comma list) restricts the "
        "sweep to a subset of the axis servos; BASE_GAIN then pins the "
        "swept gain on every non-swept axis servo at that value "
        "for an asymmetric-gain experiment; those servos are restored too. "
        "PATTERN=1 replaces the single-axis strokes with a TEST_SPEED-style "
        "XY pattern (diagonals + box over the configured XY bounds inset by "
        "BOUND, then over a SMALL_SIZE box at center) exciting every XY "
        "servo; segments too short to reach SPEED run triangular profiles "
        "on purpose and are reported with their achieved peak velocity, and "
        "the per-step settle/overshoot metrics are not meaningful "
        "(continuous motion, no rest windows) - the verdict gates on "
        "resonance and torque saturation only. "
        "Params POS_GAINS SPEED_GAINS INTEGRALS TORQUE_FILTERS AXIS START "
        "END SPEED ACCEL ITERATIONS DWELL_MS TAG ACCEL_CHIP APPLY SERVO "
        "BASE_GAIN PATTERN BOUND SMALL_SIZE"
    )

    def _pattern_geometry_params(
        self, gcmd: Any
    ) -> tuple[list[tuple[float, float]], float, float, dict[str, Any]]:
        inset = gcmd.get_float("BOUND", 20.0, minval=0.0)
        small = gcmd.get_float("SMALL_SIZE", 20.0, above=0.0)
        x_lo, x_hi = self._config_bounds(gcmd, "X")
        y_lo, y_hi = self._config_bounds(gcmd, "Y")
        points, start_x, start_y = servo_strokes.pattern_geometry(
            gcmd, x_lo, x_hi, y_lo, y_hi, inset, small
        )
        plan = {
            "pattern": {
                "x_bounds": [x_lo, x_hi],
                "y_bounds": [y_lo, y_hi],
                "inset": inset,
                "small_size": small,
                "segments": len(points),
            }
        }
        return points, start_x, start_y, plan

    def _pattern_setup(
        self, gcmd: Any
    ) -> tuple[list[str], list[tuple[float, float]], float, float, dict]:
        if gcmd.get("BASE_GAIN", None) is not None:
            raise gcmd.error(
                "BASE_GAIN pins gains on the non-swept servos of one axis - "
                "not supported with PATTERN=1, which sweeps every XY servo"
            )
        if (
            gcmd.get("START", None) is not None
            or gcmd.get("END", None) is not None
        ):
            raise gcmd.error(
                "START/END are single-axis stroke bounds - PATTERN=1 uses "
                "the configured XY bounds with BOUND= inset"
            )
        points, start_x, start_y, plan = self._pattern_geometry_params(gcmd)
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            servos = [s.strip() for s in servo.split(",") if s.strip()]
        else:
            kin = self._kin()
            servos = list(
                dict.fromkeys(
                    servo_strokes.axis_servos(gcmd, kin, "X")
                    + servo_strokes.axis_servos(gcmd, kin, "Y")
                )
            )
        return servos, points, start_x, start_y, plan

    def _swept_gain_values(self, gcmd: Any) -> tuple[str, list[int]]:
        given = {p: gcmd.get(p, None) for p in GAIN_LIST_PARAMS}
        named = [p for p, text in given.items() if text is not None]
        if len(named) > 1:
            raise gcmd.error(
                "give exactly one of %s (got %s)"
                % (", ".join(GAIN_LIST_PARAMS), ", ".join(named))
            )
        chosen = named[0] if named else "SPEED_GAINS"
        param = GAIN_LIST_PARAMS[chosen]
        text = given.get(chosen) or "500,650,800,1000"
        values = [int(round(v)) for v in self._floats(text)]
        try:
            validate_gain_values(values, param)
        except ValueError as e:
            raise gcmd.error("SERVO_CALIBRATE_GAINS: %s" % (e,))
        return param, values

    def cmd_SERVO_CALIBRATE_GAINS(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        pattern = gcmd.get_int("PATTERN", 0)
        if pattern:
            servos, points, start_x, start_y, pattern_plan = (
                self._pattern_setup(gcmd)
            )
            axis = "XY"
            start = end = None
        else:
            servos = self._servos(gcmd, axis)
            start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "cal")
        param, values = self._swept_gain_values(gcmd)
        if gcmd.get("REVERT_GAIN", None) is not None:
            raise gcmd.error(
                "REVERT_GAIN was removed - the sweep always restores the "
                "gains that were active before it ran; keep a result with "
                "APPLY=1 or SERVO_APPLY_GAINS"
            )
        if gcmd.get("BASE_SPEED_GAIN", None) is not None:
            raise gcmd.error(
                "BASE_SPEED_GAIN was removed - the sweep no longer derives "
                "position/integral from the speed gain; use BASE_GAIN= to "
                "pin the swept gain on the non-swept axis servos"
            )
        base_gain = gcmd.get("BASE_GAIN", None)
        base_servos: list[str] = []
        if base_gain is not None:
            base_gain = int(base_gain)
            try:
                validate_gain_values([base_gain], param)
            except ValueError as e:
                raise gcmd.error("BASE_GAIN: %s" % (e,))
            axis_servos = servo_strokes.axis_servos(gcmd, self._kin(), axis)
            base_servos = [s for s in axis_servos if s not in servos]
            if not base_servos:
                raise gcmd.error(
                    "BASE_GAIN needs SERVO= to name a subset of the "
                    "axis servos - every servo on axis %s is already in the "
                    "sweep" % (axis,)
                )
        apply = gcmd.get_int("APPLY", 0)
        chip, chip_name = self._accel_chip(gcmd)
        affected = list(dict.fromkeys(servos + base_servos))
        prior = {s: self._read_gains(s) for s in affected}
        first = prior[servos[0]]
        for s in servos:
            if prior[s] != first:
                raise gcmd.error(
                    "servos disagree on the current gains (%s=%s vs %s=%s) "
                    "- the sweep holds the non-swept gains at one shared "
                    "value; align the drives first (SERVO_APPLY_GAINS)"
                    % (servos[0], first, s, prior[s])
                )

        def restore_prior() -> None:
            for s, g in prior.items():
                self._write_gains([s], g)

        stroke_plan = {
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if pattern:
            stroke_plan.update(pattern_plan)
        else:
            stroke_plan.update({"start": start, "end": end})
        run = self._begin_run(
            gcmd,
            "gain_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = SingleGainAdapter(
            self, servos, param, tag, dict(first), first[param]
        )
        restored = False
        try:
            if pattern:
                self._prep("X", dwell)
                self._prep("Y", dwell)
                moves = servo_strokes.pattern_moves(
                    self.gcode, points, start_x, start_y, speed, accel
                )
                gcmd.respond_info(
                    servo_strokes.pattern_reach_summary(moves, speed)
                )
                self._goto_xy(start_x, start_y, dwell)
            else:
                self._prep(axis, dwell)
                servo_strokes.goto(
                    self.gcode,
                    self.travel_speed,
                    "%s%.3f" % (axis, start),
                    dwell,
                )
            self._set_manual_tuning(servos)
            if base_servos:
                self._set_manual_tuning(base_servos)
                for s in base_servos:
                    pinned = dict(prior[s])
                    pinned[param] = base_gain
                    self._write_gains([s], pinned)
                run.manifest["base_gains"] = {
                    "servos": base_servos,
                    "param": param,
                    "value": base_gain,
                }
                run.write()
                _addr, _lo, _hi, desc, unit, scale = GAIN_PARAMS[param]
                gcmd.respond_info(
                    "base %s pinned at %d (%.4g %s) on %s (held for the "
                    "whole sweep)"
                    % (
                        desc,
                        base_gain,
                        base_gain / scale,
                        unit,
                        ", ".join(base_servos),
                    )
                )

            def run_step(sg: Any) -> None:
                if pattern:
                    servo_strokes.emit_pattern(
                        self.gcode,
                        points,
                        start_x,
                        start_y,
                        speed,
                        accel,
                        iterations,
                        dwell,
                    )
                else:
                    self._strokes(
                        axis, start, end, speed, accel, iterations, dwell
                    )

            steps = self._engine.run(
                adapter,
                values,
                servos,
                run_step,
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
                if pattern:
                    gcmd.respond_info(
                        "PATTERN=1: APPLY verification runs single-axis X "
                        "strokes (the tracking measurement is per-axis)"
                    )
                self._apply_verdict(
                    gcmd, run, results, "X" if pattern else axis
                )
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

    cmd_SERVO_REFINE_DYNAMICS_help = (
        "Empirically refine the torque-feedforward dynamics profile on the "
        "RUNNING endpoint: golden-section search over a scale factor on the "
        "baseline profile's mass matrix (TERM=MASS, scored on mean per-move "
        "ferr_peak - the error window runs from move start through settle, "
        "so it covers in-move tracking and endpoint overshoot alike), "
        "viscous vector (TERM=VISCOUS, scored on mean ferr_rms) or "
        "coulomb vector (TERM=COULOMB, "
        "scored on mean ferr_peak - friction error peaks at breakaway), or "
        "an additive signed coefficient for each AWD pair "
        "(TERM=DIRECTION_SPLIT, scored on the even directional differential "
        "of signed moving-error means). On "
        "coupled_xy every vector term refines the two modes sequentially - "
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
        "feedforward term does not move the loop's resonances. When a "
        "candidate beats the baseline, the refined profile is written to a "
        "new TOML - pointing "
        "dynamics_profile at it (then RESTART) is the only way to keep it. "
        "PATTERN=1 replaces the per-axis stroke grids with the "
        "TEST_SPEED-style XY pattern over the configured XY bounds inset "
        "by BOUND (plus a SMALL_SIZE box at center); short segments run "
        "triangular profiles on purpose, and TERM=DIRECTION_SPLIT is not "
        "supported with PATTERN=1 (direction metrics need rest-to-rest "
        "strokes). Params TERM (MASS) AXIS (X) SERVOS PROFILE LO HI TOL "
        "(0.02) MAX_EVALS (10) START END X_START X_END Y_START Y_END "
        "ACCELS SPEEDS ITERATIONS DWELL_MS TAG (refdyn) NAME PATTERN BOUND "
        "SMALL_SIZE"
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
        for profile_slot, motor in enumerate(baseline["axes"]):
            node_slot = node.get_slot_for_motor(motor)
            if node_slot is None:
                raise gcmd.error(
                    "profile %s axis %r is not a motor on node %s"
                    % (profile_path, motor, node.name)
                )
            if node_slot != profile_slot:
                raise gcmd.error(
                    "profile %s axis %r is at slot %d, but node %s maps it "
                    "to slot %d"
                    % (profile_path, motor, profile_slot, node.name, node_slot)
                )
        return profile_path, baseline

    def _direction_split_baseline(
        self, gcmd: Any, kin: Any, baseline: dict[str, Any]
    ) -> dict[str, Any]:
        if baseline.get("pairs"):
            return baseline
        pair_slots = None
        if kin.coupled_xy():
            layout = servo_strokes.corexy_fit_layout(gcmd, kin)
            pair_slots = layout["pairs"]
        derived = _copy_dynamics(baseline)
        if pair_slots is not None:
            pairs = [part.split(",") for part in pair_slots.split(";") if part]
            axis_index = {name: i for i, name in enumerate(baseline["axes"])}
            columns = [list(col) for col in zip(*baseline["frame"])]
            claimed: set[str] = set()
            for slots in pairs:
                if len(slots) != 2:
                    raise gcmd.error(
                        "kinematic AWD pair must contain exactly two slots "
                        "(got %s)" % (slots,)
                    )
                if slots[0] == slots[1]:
                    raise gcmd.error(
                        "kinematic AWD pair slots must be distinct (got %s)"
                        % (slots,)
                    )
                overlap = claimed.intersection(slots)
                if overlap:
                    raise gcmd.error(
                        "kinematic AWD pairs overlap at slots %s"
                        % (sorted(overlap),)
                    )
                if any(s not in axis_index for s in slots):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not match profile axes %s"
                        % (slots, baseline["axes"])
                    )
                claimed.update(slots)
                first, second = (axis_index[s] for s in slots)
                if not _equal_or_opposite_columns(
                    columns[first], columns[second]
                ):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not have equal parallel "
                        "or antiparallel frame columns" % (slots,)
                    )
            derived["pairs"] = [
                {"slots": slots, "direction_split": 0.0} for slots in pairs
            ]
        else:
            try:
                derived["pairs"] = discover_dynamics_pairs(baseline)
            except ValueError as e:
                raise gcmd.error("cannot derive dynamics pairs: %s" % (e,))
        if not derived["pairs"]:
            raise gcmd.error(
                "TERM=DIRECTION_SPLIT found no explicit [[pair]] tables, "
                "kinematic AWD pairs, or groups of exactly two equal "
                "parallel/antiparallel frame columns"
            )
        return derived

    def cmd_SERVO_REFINE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_REFINE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        term = gcmd.get("TERM", "MASS").upper()
        if term not in DYNAMICS_METRIC_BY_TERM:
            raise gcmd.error(
                "TERM must be MASS, VISCOUS, COULOMB or DIRECTION_SPLIT "
                "(got %r)" % (gcmd.get("TERM", ""),)
            )
        pattern = gcmd.get_int("PATTERN", 0)
        if pattern:
            if term == "DIRECTION_SPLIT":
                raise gcmd.error(
                    "TERM=DIRECTION_SPLIT needs rest-to-rest single-axis "
                    "strokes for per-move direction metrics - not "
                    "supported with PATTERN=1"
                )
            self._reject_pattern_stroke_bounds(gcmd)
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
        if term == "DIRECTION_SPLIT":
            baseline = self._direction_split_baseline(gcmd, kin, baseline)
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        pattern_plan: dict[str, Any] = {}
        if pattern:
            points, start_x, start_y, pattern_plan = (
                self._pattern_geometry_params(gcmd)
            )

            def pattern_grid() -> None:
                self._goto_xy(start_x, start_y, dwell)
                for accel in accels:
                    for speed in speeds:
                        servo_strokes.emit_pattern(
                            self.gcode,
                            points,
                            start_x,
                            start_y,
                            speed,
                            accel,
                            iterations,
                            dwell,
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

            def both_grids() -> None:
                x_grid()
                y_grid()

            if term == "DIRECTION_SPLIT":

                def pair_add_fn(
                    index: int,
                ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                    def add_fn(
                        profile: dict[str, Any], delta: float
                    ) -> dict[str, Any]:
                        return add_dynamics_direction_split(
                            profile, index, delta
                        )

                    return add_fn

                phases = [
                    (
                        "direction_split_%s" % (pair["slots"][0],),
                        pair["slots"][0],
                        pair_add_fn(i),
                        both_grids,
                        list(pair["slots"]),
                    )
                    for i, pair in enumerate(baseline["pairs"])
                ]
            else:
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

                grid_x = pattern_grid if pattern else x_grid
                grid_y = pattern_grid if pattern else y_grid
                phases = [
                    (
                        "%s_x" % (term.lower(),),
                        "x",
                        mode_scale_fn(modes.index("x")),
                        grid_x,
                        None,
                    ),
                    (
                        "%s_y" % (term.lower(),),
                        "y",
                        mode_scale_fn(modes.index("y")),
                        grid_y,
                        None,
                    ),
                ]
        else:
            start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)

            def prep_axes() -> None:
                if pattern:
                    self._prep("X", dwell)
                    self._prep("Y", dwell)
                else:
                    self._prep(axis, dwell)

            def cart_grid() -> None:
                for accel in accels:
                    for speed in speeds:
                        self._strokes(
                            axis, start, end, speed, accel, iterations, dwell
                        )

            if term == "DIRECTION_SPLIT":

                def pair_add_fn(
                    index: int,
                ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                    return lambda profile, delta: add_dynamics_direction_split(
                        profile, index, delta
                    )

                phases = [
                    (
                        "direction_split_%s" % (pair["slots"][0],),
                        pair["slots"][0],
                        pair_add_fn(i),
                        cart_grid,
                        list(pair["slots"]),
                    )
                    for i, pair in enumerate(baseline["pairs"])
                ]
            else:
                phases = [
                    (
                        term.lower(),
                        "",
                        term_scale_fn,
                        pattern_grid if pattern else cart_grid,
                        None,
                    )
                ]

        tag = gcmd.get("TAG", "refdyn")
        name = gcmd.get("NAME", "refined_%s" % (term.lower(),))
        if term == "DIRECTION_SPLIT":
            span = min(
                0.25,
                min(
                    0.9 * (0.5 - abs(pair["direction_split"]))
                    for pair in baseline["pairs"]
                ),
            )
            lo = gcmd.get_float("LO", -span)
            hi = gcmd.get_float("HI", span)
            tol = gcmd.get_float("TOL", 0.01, above=0.0)
        else:
            lo = gcmd.get_float("LO", 0.7, above=0.0)
            hi = gcmd.get_float("HI", 1.3)
            tol = gcmd.get_float("TOL", 0.02, above=0.0)
        max_evals = gcmd.get_int("MAX_EVALS", 10, minval=3)
        baseline_candidate = 0.0 if term == "DIRECTION_SPLIT" else 1.0
        if not lo < baseline_candidate < hi:
            raise gcmd.error(
                "bracket [LO, HI] = [%g, %g] must contain %g strictly - "
                "the search is centered on the baseline"
                % (lo, hi, baseline_candidate)
            )
        if term == "DIRECTION_SPLIT":
            for pair in baseline["pairs"]:
                base = pair["direction_split"]
                if abs(base + lo) >= 0.5 or abs(base + hi) >= 0.5:
                    raise gcmd.error(
                        "direction split delta bracket [%g, %g] takes pair "
                        "%s from %g outside abs(w) < 0.5"
                        % (lo, hi, pair["slots"], base)
                    )
        metric = DYNAMICS_METRIC_BY_TERM[term]
        stroke_plan = {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if pattern:
            stroke_plan.update(pattern_plan)
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
            "phases": [label for label, _s, _fn, _g, _d in phases],
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
            drives: list[str] | None,
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
                if drives is not None:
                    result_step = next(
                        (
                            item
                            for item in results.get("steps") or []
                            if item.get("name") == step.name
                        ),
                        None,
                    )
                    if result_step is None:
                        raise gcmd.error(
                            "step %r missing from results.json" % (step.name,)
                        )
                    try:
                        reports[key].update(
                            direction_split_candidate_metrics(
                                adapter.baseline, result_step, drives
                            )
                        )
                    except ValueError as e:
                        raise gcmd.error(str(e))
                gcmd.respond_info(
                    "%s %s %.4f -> %s (counts, mean per move)"
                    % (
                        adapter.label,
                        adapter.value_name,
                        key,
                        metrics_line(reports[key]),
                    )
                )
                scores[key] = reports[key][metric]
                return scores[key]

            baseline_score = evaluate(baseline_candidate)
            best, best_score, _probes = golden_section_search(
                evaluate, lo, hi, tol, max_evals
            )
            if baseline_score <= best_score:
                best, best_score = baseline_candidate, baseline_score
            return best, best_score, baseline_score, reports

        phase_out: list[tuple[str, str, float, float, float, dict]] = []
        adapters: list[DynamicsModelAdapter] = []
        refined = baseline
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            prep_axes()
            if pattern:
                self._pattern_reach_report(
                    gcmd, points, start_x, start_y, accels, speeds
                )
            for label, suffix, scale_fn, run_grid, drives in phases:
                adapter = DynamicsModelAdapter(
                    engine,
                    handle,
                    refined,
                    scale_fn,
                    label,
                    tag,
                    "delta" if term == "DIRECTION_SPLIT" else "scale",
                )
                adapters.append(adapter)
                best, best_score, baseline_score, reports = run_phase(
                    adapter, run_grid, drives
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
                    "  %s %s %.4f: %s%s"
                    % (
                        label,
                        "delta" if term == "DIRECTION_SPLIT" else "scale",
                        scale,
                        metrics_line(reports[scale]),
                        marker,
                    )
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
        if all(
            best == baseline_candidate
            for _l, _s, best, _bs, _bl, _r in phase_out
        ):
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
                    "%s %s %.4f (%s %.1f -> %.1f)"
                    % (
                        label,
                        "delta" if term == "DIRECTION_SPLIT" else "scale",
                        best,
                        metric,
                        baseline_score,
                        best_score,
                    )
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

    cmd_SERVO_AUTOTUNE_help = (
        "Packaged tuning sequence: baseline tracking -> inertia ratio "
        "identify -> apply C00.06 -> coarse gains (SERVO_APPLY_GAINS "
        "defaults) -> gain sweep (apply winner) "
        "-> fit dynamics -> verify vs baseline. APPLY=0 "
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
