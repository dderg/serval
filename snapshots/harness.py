"""Snapshot-testing harness for the motion planner.

A *case* is a directory holding ``case.gcode`` + ``printer.cfg``. Running it
drives the real ``_motion_engine.pipeline_snapshot`` and records the full raw
trajectory dict as a checked-in ``baseline.json.gz`` under ``baselines/``
(deterministic gzip). ``run.py`` flags a re-run that deviates; the web review
re-baselines only on an explicit accept.

Comparison is structural with a float tolerance (see ``snapshots_match``):
segment shape and integer counts must match exactly, floats within
``FLOAT_ATOL``/``FLOAT_RTOL``. The planner is deterministic, but its
transcendental math bottoms out in the host libm, whose last ulp differs
between a macOS dev box and the Linux CI runner — so a baseline generated on
one host is validated on another within tolerance rather than bit-for-bit.
Baselines are still stored as canonical JSON (``allow_nan=False`` poisons a
non-finite sample on write).
"""

from __future__ import annotations

import enum
import gzip
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
# scripts/ for viz_pipeline (the VISUALIZE tool, reused as-is); repo root and
# klippy/ for read_printer_config's `from klippy import ...` and _motion_engine.
for _p in (_REPO_ROOT / "scripts", _REPO_ROOT / "klippy", _REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import viz_pipeline  # noqa: E402

CASES_DIR = Path(__file__).resolve().parent / "cases"
BASELINES_DIR = Path(__file__).resolve().parent / "baselines"
CONFIG_NAME = "printer.cfg"
BASELINE_SUFFIX = ".baseline.json.gz"

# Floats are compared with a tolerance, not bit-exactly: the planner's
# transcendental math (Fresnel/clothoid sampling, atan2, the RK4 integrator)
# bottoms out in the host libm, whose last ulp differs between macOS dev boxes
# and the Linux CI runner. A sample passes if it is within the absolute OR the
# relative tolerance — far below any meaningful trajectory change, above libm
# noise. Structure and integer counts still compare exactly. The snapshot stores
# only primary planner state (geometry + s, v, heading, kappa); numerically
# differentiated diagnostics are not stored, so there is no near-zero
# finite-difference noise to absorb here.
FLOAT_ATOL = 1e-7
FLOAT_RTOL = 1e-7


class Status(enum.Enum):
    EXACT = "exact"
    CHANGED = "changed"
    NEW = "new"


@dataclass(frozen=True)
class Case:
    """One G-code file. The folder it lives in is a group sharing one
    printer.cfg; `name` is `<group>/<gcode stem>`."""

    name: str
    gcode_path: Path
    config_path: Path
    baseline_path: Path


def discover_cases(
    cases_dir: Path = CASES_DIR, baselines_dir: Path = BASELINES_DIR
) -> list[Case]:
    if not cases_dir.is_dir():
        return []
    cases = []
    for group in sorted(p for p in cases_dir.iterdir() if p.is_dir()):
        config = group / CONFIG_NAME
        for gcode in sorted(group.glob("*.gcode")):
            if not gcode.read_text().strip():
                continue
            stem = gcode.stem
            cases.append(
                Case(
                    name=f"{group.name}/{stem}",
                    gcode_path=gcode,
                    config_path=config,
                    baseline_path=(
                        baselines_dir / group.name / f"{stem}{BASELINE_SUFFIX}"
                    ),
                )
            )
    return cases


def discover_baselines(baselines_dir: Path = BASELINES_DIR) -> list[Path]:
    if not baselines_dir.is_dir():
        return []
    return sorted(baselines_dir.rglob(f"*{BASELINE_SUFFIX}"))


def orphan_baselines(
    cases: list[Case], baselines_dir: Path = BASELINES_DIR
) -> list[Path]:
    live = {case.baseline_path.resolve() for case in cases}
    return [
        b for b in discover_baselines(baselines_dir) if b.resolve() not in live
    ]


def prune_orphan_baselines(
    cases: list[Case], baselines_dir: Path = BASELINES_DIR
) -> list[Path]:
    """Delete baselines whose case is gone (gcode deleted, renamed, or skipped).

    `cases` must be the full discovered set, never a -k-filtered subset, or a
    filtered run would delete the excluded cases' still-live baselines.
    """
    orphans = orphan_baselines(cases, baselines_dir)
    for baseline in orphans:
        baseline.unlink()
    _remove_empty_dirs(baselines_dir)
    return orphans


def _remove_empty_dirs(root: Path) -> None:
    if not root.is_dir():
        return
    for sub in sorted(root.rglob("*"), reverse=True):
        if sub.is_dir() and not any(sub.iterdir()):
            sub.rmdir()


def _import_engine():
    try:
        import _motion_engine
    except ModuleNotFoundError as exc:
        raise ImportError(
            "_motion_engine not built — build it with: "
            "make -f Makefile.rust motion-engine"
        ) from exc
    return _motion_engine


def run_case(case: Case) -> dict:
    if not case.gcode_path.exists():
        raise ValueError(f"case '{case.name}': missing {case.gcode_path.name}")
    if not case.config_path.exists():
        raise ValueError(
            f"case '{case.name}': no {CONFIG_NAME} in its group folder"
        )

    max_velocity, max_accel, scv, max_jerk, arc_fit = (
        viz_pipeline.read_printer_config(case.config_path)
    )
    waypoints = viz_pipeline.parse_gcode(case.gcode_path, max_velocity)
    if len(waypoints) < 2:
        raise ValueError(
            f"case '{case.name}': fewer than two spatial moves in "
            f"{case.gcode_path.name}"
        )

    engine = _import_engine()
    return engine.pipeline_snapshot(
        waypoints,
        max_velocity,
        max_accel,
        scv,
        max_jerk,
        arc_fit=arc_fit,
    )


def canonical_json(snapshot: dict) -> str:
    # sort_keys makes field order irrelevant; Python's float repr round-trips
    # exactly, so a re-run of a deterministic planner reproduces this string.
    # allow_nan=False raises on a non-finite sample instead of emitting bare
    # NaN/Infinity that would self-compare green and poison the baseline.
    return json.dumps(
        snapshot, sort_keys=True, separators=(",", ":"), allow_nan=False
    )


def read_baseline(case: Case) -> str | None:
    if not case.baseline_path.exists():
        return None
    return gzip.decompress(case.baseline_path.read_bytes()).decode()


def write_baseline(case: Case, snapshot: dict) -> None:
    data = (canonical_json(snapshot) + "\n").encode()
    case.baseline_path.parent.mkdir(parents=True, exist_ok=True)
    case.baseline_path.write_bytes(
        gzip.compress(data, compresslevel=9, mtime=0)
    )


def baseline_snapshot(case: Case) -> dict | None:
    text = read_baseline(case)
    if text is None:
        return None
    return json.loads(text)


def _floats_match(a: float, b: float, atol: float, rtol: float) -> bool:
    if a == b:
        return True
    if not (math.isfinite(a) and math.isfinite(b)):
        return False
    diff = abs(a - b)
    return diff <= atol or diff <= rtol * max(abs(a), abs(b))


def snapshots_match(
    a: object,
    b: object,
    atol: float = FLOAT_ATOL,
    rtol: float = FLOAT_RTOL,
) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, bool) or isinstance(b, bool):
            return a is b
        if not (isinstance(a, (int, float)) and isinstance(b, (int, float))):
            return False
        return _floats_match(float(a), float(b), atol, rtol)
    if isinstance(a, dict):
        return (
            isinstance(b, dict)
            and a.keys() == b.keys()
            and all(snapshots_match(a[k], b[k], atol, rtol) for k in a)
        )
    if isinstance(a, list):
        return (
            isinstance(b, list)
            and len(a) == len(b)
            and all(snapshots_match(x, y, atol, rtol) for x, y in zip(a, b))
        )
    return a == b


def describe_mismatches(
    a: object,
    b: object,
    atol: float = FLOAT_ATOL,
    rtol: float = FLOAT_RTOL,
    path: str = "",
    out: list[str] | None = None,
    limit: int = 12,
) -> list[str]:
    if out is None:
        out = []
    if len(out) >= limit:
        return out
    numeric = (int, float)
    if isinstance(a, float) or isinstance(b, float):
        ok = (
            isinstance(a, numeric)
            and isinstance(b, numeric)
            and not (isinstance(a, bool) or isinstance(b, bool))
        )
        if not ok:
            out.append(f"{path}: {a!r} != {b!r} (type/NaN)")
        elif not _floats_match(float(a), float(b), atol, rtol):
            d = abs(float(a) - float(b))
            rel = d / max(abs(a), abs(b), 1e-300)
            out.append(f"{path}: {a!r} != {b!r} (abs={d:.3e} rel={rel:.3e})")
        return out
    if isinstance(a, dict):
        if not isinstance(b, dict) or a.keys() != b.keys():
            out.append(f"{path}: keys differ")
            return out
        for k in a:
            sub = f"{path}.{k}" if path else str(k)
            describe_mismatches(a[k], b[k], atol, rtol, sub, out, limit)
        return out
    if isinstance(a, list):
        if not isinstance(b, list):
            out.append(f"{path}: list vs {type(b).__name__}")
        elif len(a) != len(b):
            out.append(f"{path}: length {len(a)} != {len(b)}")
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                describe_mismatches(
                    x, y, atol, rtol, f"{path}[{i}]", out, limit
                )
        return out
    if a != b:
        out.append(f"{path}: {a!r} != {b!r}")
    return out


def drift_envelope(a: object, b: object, tiny: float = 1e-3) -> dict:
    """Worst relative drift on |value|>tiny and worst absolute drift on the
    near-zero (|value|<=tiny) values, each with the field path that hit it.

    Splits the two regimes so a single changed run names both the rtol the large
    values need and the atol the near-zero values (where rtol is useless) need —
    and points at the exact field, not just a magnitude.
    """
    worst = {"rel": 0.0, "rel_at": "", "abs": 0.0, "abs_at": ""}
    numeric = (int, float)

    def walk(a: object, b: object, path: str) -> None:
        if isinstance(a, float) or isinstance(b, float):
            if (
                isinstance(a, numeric)
                and isinstance(b, numeric)
                and not (isinstance(a, bool) or isinstance(b, bool))
                and math.isfinite(a)
                and math.isfinite(b)
            ):
                d = abs(float(a) - float(b))
                m = max(abs(a), abs(b))
                if m > tiny:
                    if d / m > worst["rel"]:
                        worst["rel"] = d / m
                        worst["rel_at"] = f"{path} ({a!r} vs {b!r})"
                elif d > worst["abs"]:
                    worst["abs"] = d
                    worst["abs_at"] = f"{path} ({a!r} vs {b!r})"
            return
        if isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys():
            for k in a:
                walk(a[k], b[k], f"{path}.{k}" if path else str(k))
        elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
            for i, (x, y) in enumerate(zip(a, b)):
                walk(x, y, f"{path}[{i}]")

    walk(a, b, "")
    return worst


def compare(case: Case, snapshot: dict) -> Status:
    baseline = baseline_snapshot(case)
    if baseline is None:
        return Status.NEW
    if snapshots_match(baseline, snapshot):
        return Status.EXACT
    return Status.CHANGED
