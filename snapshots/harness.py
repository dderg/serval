"""Snapshot-testing harness for the motion planner.

A *case* is one (config, G-code) pair: every ``*.cfg`` in a group folder runs
against every ``*.gcode`` in it (a matrix). Running a case drives the real
``_motion_engine.pipeline_snapshot`` and records the full raw trajectory dict as
a checked-in ``baseline.json.gz`` under ``baselines/`` (deterministic gzip).
``run.py`` flags a re-run that deviates; the web review re-baselines only on an
explicit accept.

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

import concurrent.futures
import enum
import gzip
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
# klippy/ for read_printer_config's `from klippy import ...`; repo root for
# _motion_engine.
for _p in (_REPO_ROOT / "klippy", _REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

CASES_DIR = Path(__file__).resolve().parent / "cases"
BASELINES_DIR = Path(__file__).resolve().parent / "baselines"
BASELINE_SUFFIX = ".baseline.json.gz"

# Floats are compared with a tolerance, not bit-exactly: the planner's
# transcendental math (Fresnel/clothoid sampling, atan2, the RK4 integrator)
# bottoms out in the host libm, whose last ulp differs between macOS dev boxes
# and the Linux CI runner. A sample passes if it is within the absolute OR the
# relative tolerance — far below any meaningful trajectory change, above libm
# noise. Structure and integer counts still compare exactly.
FLOAT_ATOL = 1e-6
FLOAT_RTOL = 1e-7


@dataclass
class PrinterConfigData:
    max_velocity: float
    max_accel: float
    square_corner_velocity: float | None
    corner_deviation: float | None
    max_jerk: float
    max_path_deviation: float
    max_accel_deviation: float
    max_extrude_only_velocity: float | None
    max_extrude_only_accel: float | None
    config_text: str


def read_printer_config(cfg_path: Path) -> PrinterConfigData:
    # Parse through klippy's own loader so includes resolve exactly like the
    # live printer; the engine's pipeline_snapshot re-reads the motion
    # sections from the serialized document with the same Rust reader
    # (defaults, bounds, scv conversion) init_planner uses.
    from klippy import configfile

    loader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
    loader.printer = None
    config = loader.read_config(str(cfg_path))
    printer = config.getsection("printer")
    extruder = (
        config.getsection("extruder")
        if config.has_section("extruder")
        else None
    )
    return PrinterConfigData(
        max_velocity=printer.getfloat("max_velocity", above=0.0),
        max_accel=printer.getfloat("max_accel", above=0.0),
        square_corner_velocity=printer.getfloat(
            "square_corner_velocity", None, minval=0.0
        ),
        corner_deviation=printer.getfloat("corner_deviation", None, minval=0.0),
        max_jerk=printer.getfloat("max_jerk", 0.0, minval=0.0),
        max_path_deviation=printer.getfloat(
            "max_path_deviation", 0.005, above=0.0
        ),
        max_accel_deviation=printer.getfloat(
            "max_accel_deviation", 50.0, above=0.0
        ),
        max_extrude_only_velocity=(
            extruder.getfloat("max_extrude_only_velocity", None, above=0.0)
            if extruder
            else None
        ),
        max_extrude_only_accel=(
            extruder.getfloat("max_extrude_only_accel", None, above=0.0)
            if extruder
            else None
        ),
        config_text=config.fileconfig.write_string(),
    )


def parse_gcode(
    path: Path, max_velocity: float, max_accel: float
) -> list[tuple[float, float, float, float, float, float]]:
    # Waypoints carry absolute (x, y, z, e, feedrate, accel). E rides as a
    # fifth coordinate so retracts (E-only moves) and extruding moves flow
    # through the pipeline as followers; the engine differences consecutive E
    # to a per-move delta. The sixth coordinate is the acceleration limit in
    # force for the move ending at that waypoint — max_accel until a
    # `SET_VELOCITY_LIMIT ACCEL=` line changes it. Extruder mode is M82
    # (absolute) / M83 (relative), independent of the G90/G91 flag that
    # governs X/Y/Z; under G91 an undeclared extruder rides along as
    # relative, and an E word with no mode declared at all is refused rather
    # than guessed. G92 resets any axis's position (commonly `G92 E0`)
    # without emitting a move.
    waypoints: list[tuple[float, float, float, float, float, float]] = []
    x, y, z, e = 0.0, 0.0, 0.0, 0.0
    feedrate = max_velocity
    accel = max_accel
    relative = False
    e_relative: bool | None = None
    motion_cmd = re.compile(r"^G0?([0-3])\b", re.IGNORECASE)
    mode_cmd = re.compile(r"^G(90|91)\b", re.IGNORECASE)
    set_pos_cmd = re.compile(r"^G92\b", re.IGNORECASE)
    e_mode_cmd = re.compile(r"^M(82|83)\b", re.IGNORECASE)
    velocity_limit_cmd = re.compile(
        r"^SET_VELOCITY_LIMIT\b(.*)$", re.IGNORECASE
    )
    coord = re.compile(r"([XYZEFIJ])([-+]?[0-9]*\.?[0-9]+)", re.IGNORECASE)

    def params_of(line: str) -> dict[str, float]:
        return {
            c.group(1).upper(): float(c.group(2)) for c in coord.finditer(line)
        }

    for line in path.read_text().splitlines():
        line = line.split(";", 1)[0].strip()

        mm = mode_cmd.match(line)
        if mm:
            relative = mm.group(1) == "91"
            continue

        em = e_mode_cmd.match(line)
        if em:
            e_relative = em.group(1) == "83"
            continue

        if set_pos_cmd.match(line):
            params = params_of(line)
            x = params.get("X", x)
            y = params.get("Y", y)
            z = params.get("Z", z)
            e = params.get("E", e)
            continue

        vl = velocity_limit_cmd.match(line)
        if vl:
            for arg in vl.group(1).split():
                key, sep, value = arg.partition("=")
                if not sep:
                    raise ValueError(
                        f"{path.name}: malformed SET_VELOCITY_LIMIT argument "
                        f"{arg!r} — expected KEY=VALUE"
                    )
                if key.upper() != "ACCEL":
                    raise ValueError(
                        f"{path.name}: SET_VELOCITY_LIMIT {key}=… is not "
                        "supported here — only ACCEL is wired through the "
                        "snapshot waypoints; silently ignoring the parameter "
                        "would let a case claim limits it never exercised"
                    )
                accel = float(value)
                if not (accel > 0.0 and accel != float("inf")):
                    raise ValueError(
                        f"{path.name}: SET_VELOCITY_LIMIT ACCEL={value} must "
                        "be a positive finite number"
                    )
            continue

        m = motion_cmd.match(line)
        if not m:
            continue
        cmd = int(m.group(1))
        params = params_of(line)
        has_position = any(axis in params for axis in ("X", "Y", "Z"))
        has_extrusion = "E" in params

        if relative:
            nx = x + params.get("X", 0.0)
            ny = y + params.get("Y", 0.0)
            nz = z + params.get("Z", 0.0)
        else:
            nx = params.get("X", x)
            ny = params.get("Y", y)
            nz = params.get("Z", z)

        if has_extrusion:
            if e_relative is None and not relative:
                raise ValueError(
                    f"{path.name}: E word before any M82/M83 (or G91) — the "
                    "extruder mode is ambiguous, and guessing absolute turns "
                    "relative-E slicer output into garbage extrusion ratios. "
                    "Declare the mode (slicer excerpts printed with relative "
                    "extrusion need an 'M83' line at the top)."
                )
            e_is_relative = True if e_relative is None else e_relative
            ne = e + params["E"] if e_is_relative else params["E"]
        else:
            ne = e

        if cmd in (2, 3):
            raise ValueError(
                f"G{cmd} arc command is not supported: the motion engine has no "
                "native arc ingestion yet, and silently linearizing it here would "
                "let a snapshot claim to exercise an arc while feeding the engine "
                "straight segments"
            )
        if cmd == 1:
            feedrate = params.get("F", feedrate * 60.0) / 60.0
        if not (has_position or ne != e):
            continue
        x, y, z, e = nx, ny, nz, ne
        if not waypoints and not has_position:
            # A prime or retract before any positional command: there is no
            # known toolhead position yet, so anchoring a waypoint would invent
            # a move from the parser's arbitrary origin. Fold the E change into
            # the state; the first positional waypoint carries it.
            continue
        move_feedrate = max_velocity if cmd == 0 else feedrate
        waypoints.append((x, y, z, e, move_feedrate, accel))

    return waypoints


class Status(enum.Enum):
    EXACT = "exact"
    CHANGED = "changed"
    NEW = "new"


@dataclass(frozen=True)
class Case:
    """One (config, G-code) pair from a group folder. Every `*.cfg` in the
    group runs against every `*.gcode`; `name` is
    `<group>/<cfg stem>/<gcode stem>`."""

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
        gcodes = [
            g for g in sorted(group.glob("*.gcode")) if g.read_text().strip()
        ]
        configs = sorted(group.glob("*.cfg"))
        if gcodes and not configs:
            raise ValueError(
                f"group '{group.name}': has .gcode files but no .cfg"
            )
        for config in configs:
            for gcode in gcodes:
                cases.append(
                    Case(
                        name=f"{group.name}/{config.stem}/{gcode.stem}",
                        gcode_path=gcode,
                        config_path=config,
                        baseline_path=(
                            baselines_dir
                            / group.name
                            / config.stem
                            / f"{gcode.stem}{BASELINE_SUFFIX}"
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
            "make -f Makefile.rust motion-engine-fast"
        ) from exc
    if not hasattr(_motion_engine, "pipeline_snapshot"):
        raise ImportError(
            "_motion_engine was built without the `snapshot` cargo feature — "
            "pipeline_snapshot is unavailable. Rebuild with: "
            "make -f Makefile.rust motion-engine-fast"
        )
    return _motion_engine


def run_case(case: Case) -> dict:
    if not case.gcode_path.exists():
        raise ValueError(f"case '{case.name}': missing {case.gcode_path.name}")
    if not case.config_path.exists():
        raise ValueError(
            f"case '{case.name}': missing config {case.config_path.name}"
        )

    cfg = read_printer_config(case.config_path)
    waypoints = parse_gcode(case.gcode_path, cfg.max_velocity, cfg.max_accel)
    if len(waypoints) < 2:
        raise ValueError(
            f"case '{case.name}': fewer than two spatial moves in "
            f"{case.gcode_path.name}"
        )

    engine = _import_engine()
    return engine.pipeline_snapshot(waypoints, cfg.config_text)


def _run_case_named(case: Case) -> tuple[str, dict]:
    return case.name, run_case(case)


def run_cases_parallel(cases: list[Case], max_workers: int | None = None):
    """Snapshot every case, one worker process per core: the engine call is
    single-threaded Rust holding the GIL, so in-process threads would
    serialize. Yields `(case, snapshot)` in completion order; a worker's
    ImportError/ValueError propagates to the consumer.
    """
    if len(cases) == 1:
        yield cases[0], run_case(cases[0])
        return
    if max_workers is None:
        max_workers = min(len(cases), os.cpu_count() or 1)
    with concurrent.futures.ProcessPoolExecutor(
        max_workers=max_workers
    ) as pool:
        futures = {pool.submit(_run_case_named, c): c for c in cases}
        for fut in concurrent.futures.as_completed(futures):
            _, snapshot = fut.result()
            yield futures[fut], snapshot


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
    worst = {
        "rel": 0.0,
        "rel_at": "",
        "abs": 0.0,
        "abs_at": "",
        "schema_at": "",
    }
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
        if isinstance(a, dict) and isinstance(b, dict):
            if a.keys() != b.keys():
                if not worst["schema_at"]:
                    worst["schema_at"] = path or "<root>"
                return
            for k in a:
                walk(a[k], b[k], f"{path}.{k}" if path else str(k))
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                if not worst["schema_at"]:
                    worst["schema_at"] = path or "<root>"
                return
            for i, (x, y) in enumerate(zip(a, b)):
                walk(x, y, f"{path}[{i}]")
        elif type(a) is not type(b) and not worst["schema_at"]:
            worst["schema_at"] = path or "<root>"

    walk(a, b, "")
    return worst


def compare(case: Case, snapshot: dict) -> Status:
    baseline = baseline_snapshot(case)
    if baseline is None:
        return Status.NEW
    if snapshots_match(baseline, snapshot):
        return Status.EXACT
    return Status.CHANGED
