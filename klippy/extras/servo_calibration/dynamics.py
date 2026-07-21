from __future__ import annotations

import json
import math

try:
    import tomllib
except ImportError:
    tomllib = None

from collections.abc import Mapping
from typing import Any

DYNAMICS_TERM_KEYS = {
    "MASS": "mass",
    "VISCOUS": "viscous",
    "COULOMB": "coulomb",
}
TUNE_RELATIVE_CLAMP = 0.4
TUNE_MASS_FLOOR_FRACTION = 0.10
TUNE_ZERO_FLOOR_STEPS = {"VISCOUS": 0.05, "COULOMB": 5.0}
FF_LEAD_US_MAX = 10_000.0


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
    ff_lead_us = data.get("ff_lead_us", 0.0)
    if (
        isinstance(ff_lead_us, bool)
        or not isinstance(ff_lead_us, (int, float))
        or not math.isfinite(ff_lead_us)
        or not (0.0 <= ff_lead_us <= FF_LEAD_US_MAX)
    ):
        raise ValueError(
            "profile ff_lead_us must be a finite number in [0, %g] (got %r)"
            % (FF_LEAD_US_MAX, ff_lead_us)
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
        "ff_lead_us": float(ff_lead_us),
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


def send_ff_lead(
    engine: Any, handle: int, node: Any, servos: list[str], lead_s: float
) -> None:
    lead_ns = int(round(lead_s * 1e9))
    for servo in servos:
        engine.set_ff_lead(handle, node.get_slot_for_motor(servo), lead_ns)


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


def render_fit_dynamics_toml(
    applied: dict[str, Any],
    fitted: dict[str, Any],
    terms: list[str],
    run_dir: str,
    lead_us: float,
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
        "ff_lead_us = %s" % (num(lead_us),),
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
