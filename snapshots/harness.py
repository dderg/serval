"""Snapshot-testing harness for the motion planner.

A *case* is a directory holding ``case.gcode`` + ``printer.cfg``. Running it
drives the real ``_motion_engine.pipeline_snapshot`` and records the full raw
trajectory dict as a checked-in ``baseline.json.gz`` under ``baselines/``
(deterministic gzip). ``run.py`` flags a re-run that deviates; the web review
re-baselines only on an explicit accept.

Comparison is exact equality on a canonical JSON serialization. The planner is
deterministic, so this is stable run-to-run on one machine. Bit-reproducibility
across machines (dev Mac vs Pi vs CI) is an open question — if baselines are
generated and checked on different hosts, sub-ulp drift could false-fail; pin a
canonical generate/compare host until that is settled.
"""

from __future__ import annotations

import enum
import gzip
import json
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


def compare(case: Case, snapshot: dict) -> Status:
    baseline = read_baseline(case)
    if baseline is None:
        return Status.NEW
    # Re-canonicalize the stored baseline so a reformatted (e.g. jq-pretty)
    # baseline still compares by value, not by stored byte layout.
    if canonical_json(json.loads(baseline)) == canonical_json(snapshot):
        return Status.EXACT
    return Status.CHANGED
