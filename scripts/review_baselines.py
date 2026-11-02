#!/usr/bin/env python3
"""Physical review of the snapshot cases a change moves.

    uv run python scripts/review_baselines.py --pr N [--expect SPEC ...]
    uv run python scripts/review_baselines.py --base REV --head REV [--expect SPEC ...]
        [--json OUT] [--no-audit] [-k SUBSTRING]

Every snapshot case whose engine output differs between the base and the
head is reviewed against physics rather than against its old numbers:

- timing: traversal time change;
- motion: per-axis max |dp|, |dv|, |da| on a 1 ms grid, where it happens and
  whether that is within SEAM_WINDOW_S of a segment seam;
- seams: the snapshot's own per-axis seam discontinuities, base and head;
- limits: the planned phases against each span's velocity and acceleration
  caps (which carry per-move, Z and extrude-only caps), the shaped toolhead
  speed against max_velocity, extrude-only E motion against its caps;
- fit tolerance: the exact-signal fit audit (`fit_tolerance` with the
  `fit-audit` feature) where the commit has it, worst position and
  acceleration over max_path_deviation / max_accel_deviation;
- material: final E against the base and against the G-code's own total;
- pieces: spline piece count and degree per axis;
- non-finite values anywhere in the head output.

Each case is classified:

- REGRESSION: a limit or tolerance violated that the base did not violate,
  material gained or lost beyond what the expectation allows, a seam
  position or velocity discontinuity grown past its physical threshold,
  non-finite output, or a case that plans at the base and fails at the head;
- SUSPICIOUS: the change falls outside the expectation (an axis it should
  not touch, a magnitude beyond the stated bound, a timing or piece-count
  change it did not claim), a seam acceleration step grown past the
  acceleration tolerance (a knot may carry the signal's own acceleration
  step, so only reading the case tells), or no expectation covers it;
- EXPECTED: inside the expectation, no red flags.

An expectation is `key=value;...` with keys
  cases=GLOB     which cases it covers (fnmatch; default every case)
  axes=x,y,z,e   axes allowed to change (default all)
  dp= dv= da=    bounds on the per-axis max |dp| (mm), |dv| (mm/s), |da| (mm/s^2)
  duration=S     allowed |traversal time change| (default 0)
  material=MM    allowed |final E change| (default 0)
  pieces=F       allowed piece-count ratio either way, or `any` (default 2)
The first expectation whose `cases` matches applies. --pr reads them from a
```snapshot-expect fenced block in the PR body; --expect on the command line
replaces the body's. Engines and samples are built and cached exactly as
pending_baselines.py does; the fit audit is cached per commit under the same
cache. Exit status is 1 when any case is a REGRESSION.
"""

from __future__ import annotations

import argparse
import configparser
import fnmatch
import json
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pending_baselines as pending

ROOT = pending.ROOT
AUDIT_DIR = pending.CACHE / "audit"
AUDIT_TARGET_DIR = pending.CACHE / "target-audit"
SEAM_WINDOW_S = 2e-3
LIMIT_SLACK = 1e-9
SHAPED_LIMIT_SLACK = 1e-6
CHANGE_FLOOR = {"p": 1e-9, "v": 1e-6, "a": 1e-3}
MATERIAL_FLOOR_MM = 1e-9
SEAM_GROWTH = 1e-3
AUDIT_GROWTH = 1.25
REST_SPEED_MM_S = 1e-6
AUDIT_LINE = re.compile(
    r"^(?P<case>\S+): (?P<context>\S+) axis (?P<axis>\d+): (?P<count>\d+) pieces, "
    r"worst accel (?P<accel>[\d.eE+-]+)x, worst position (?P<position>[\d.eE+-]+)x$"
)


class ReviewError(RuntimeError):
    pass


@dataclass
class Expectation:
    cases: str = "*"
    axes: tuple[str, ...] = pending.AXES
    bounds: dict = field(
        default_factory=lambda: {"p": math.inf, "v": math.inf, "a": math.inf}
    )
    duration: float = 0.0
    material: float = 0.0
    pieces: float = 2.0

    @classmethod
    def parse(cls, spec: str) -> Expectation:
        expectation = cls()
        for item in filter(None, (part.strip() for part in spec.split(";"))):
            key, sep, value = item.partition("=")
            if not sep:
                raise ReviewError(f"expectation item '{item}' has no '='")
            key, value = key.strip(), value.strip()
            if key == "cases":
                expectation.cases = value
            elif key == "axes":
                axes = tuple(a.strip() for a in value.split(",") if a.strip())
                unknown = set(axes) - set(pending.AXES)
                if unknown:
                    raise ReviewError(
                        f"unknown axes {sorted(unknown)} in '{spec}'"
                    )
                expectation.axes = axes
            elif key in ("dp", "dv", "da"):
                expectation.bounds[key[1]] = float(value)
            elif key == "duration":
                expectation.duration = float(value)
            elif key == "material":
                expectation.material = float(value)
            elif key == "pieces":
                expectation.pieces = (
                    math.inf if value == "any" else float(value)
                )
            else:
                raise ReviewError(
                    f"unknown expectation key '{key}' in '{spec}'"
                )
        return expectation


def expectations_from_body(body: str) -> list[str]:
    match = re.search(r"```snapshot-expect\s*\n(.*?)```", body or "", re.S)
    if not match:
        return []
    return [
        line.strip() for line in match.group(1).splitlines() if line.strip()
    ]


def expectation_for(
    name: str, expectations: list[Expectation]
) -> Expectation | None:
    return next(
        (e for e in expectations if fnmatch.fnmatch(name, e.cases)), None
    )


def resolve(rev: str) -> str:
    return pending.run(["git", "rev-parse", f"{rev}^{{commit}}"]).strip()


def pr_endpoints(number: int) -> tuple[str, str, str]:
    raw = json.loads(
        pending.run(
            [
                "gh",
                "pr",
                "view",
                str(number),
                "--json",
                "headRefOid,baseRefName,body",
            ]
        )
    )
    base = resolve(f"origin/{raw['baseRefName']}")
    return base, raw["headRefOid"], raw["body"] or ""


def show(sha: str, path: str) -> str:
    return pending.run(["git", "show", f"{sha}:{path}"])


def gcode_extrusion(text: str) -> float:
    absolute_e, e, total = True, 0.0, 0.0
    for raw_line in text.splitlines():
        line = raw_line.split(";", 1)[0].strip().upper()
        if not line:
            continue
        words = line.split()
        command = words[0]
        if command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G90":
            absolute_e = True
        elif command == "G91":
            absolute_e = False
        elif command == "G92":
            for word in words[1:]:
                if word.startswith("E"):
                    e = float(word[1:])
        elif command in ("G0", "G1", "G2", "G3"):
            for word in words[1:]:
                if word.startswith("E"):
                    value = float(word[1:])
                    delta = value - e if absolute_e else value
                    total += delta
                    e = value if absolute_e else e + value
    return total


def config_caps(text: str) -> dict:
    parser = configparser.RawConfigParser(
        strict=False, inline_comment_prefixes=("#", ";")
    )
    parser.read_string(text)

    def get(section: str, key: str) -> float | None:
        if parser.has_option(section, key):
            return float(parser.get(section, key).split()[0])
        return None

    return {
        "max_velocity": get("printer", "max_velocity"),
        "max_extrude_only_velocity": get(
            "extruder", "max_extrude_only_velocity"
        ),
        "max_extrude_only_accel": get("extruder", "max_extrude_only_accel"),
        "max_path_deviation": get("printer", "max_path_deviation") or 0.005,
        "max_accel_deviation": get("printer", "max_accel_deviation") or 50.0,
    }


def case_paths(sha: str, name: str) -> tuple[str, str]:
    group, config, gcode = name.split("/")
    return (
        f"snapshots/cases/{group}/{config}.cfg",
        f"snapshots/cases/{group}/{gcode}.gcode",
    )


def finite_everywhere(value: object) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(finite_everywhere(v) for v in value.values())
    if isinstance(value, list):
        return all(finite_everywhere(v) for v in value)
    return True


def planned_limit_excess(snapshot: dict) -> float:
    worst = 0.0
    for span in snapshot["trajectory"]["spans"]:
        limits = span["limits"]
        for phase in span["phases"]:
            speed = max(abs(phase["v0"]), abs(phase["v1"]))
            accel = abs(phase.get("a0", phase.get("accel", 0.0)))
            worst = max(
                worst,
                speed / limits["max_velocity_mm_s"] - 1.0,
                accel / limits["accel_mm_s2"] - 1.0,
            )
    return worst


def worst_seam_time(snapshot: dict, axis: int, key: str) -> float | None:
    seams = [s for s in snapshot.get("worst_seams", []) if s["axis"] == axis]
    worst = max(seams, key=lambda s: s[key], default=None)
    return None if worst is None else worst["t"]


def seam_times(snapshot: dict) -> list[float]:
    times = set()
    for span in snapshot["trajectory"]["spans"]:
        times.update((span["t_start"], span["t_end"]))
    return sorted(times)


def pieces_per_axis(snapshot: dict) -> dict:
    curves = snapshot["trajectory"]["curves"]
    out = {}
    for axis, carriers in zip(pending.AXES, snapshot["trajectory"]["axes"]):
        used: set[int] = set()

        def collect(node: object) -> None:
            if isinstance(node, dict):
                for key, value in node.items():
                    if key == "curve" and isinstance(value, int):
                        used.add(value)
                    collect(value)
            elif isinstance(node, list):
                for value in node:
                    collect(value)

        collect(carriers)
        out[axis] = {
            "pieces": sum(len(set(curves[i]["knots"])) - 1 for i in used),
            "max_degree": max((curves[i]["degree"] for i in used), default=0),
        }
    return out


def shaped_limits(pva, duration: float, caps: dict) -> dict:
    import numpy as np

    velocity = pva[:3, :, 1]
    speed = np.sqrt((velocity**2).sum(axis=0))
    out = {"toolhead_speed_ratio": float(speed.max() / caps["max_velocity"])}
    resting = speed <= REST_SPEED_MM_S
    if resting.any():
        e_velocity = np.abs(pva[3, resting, 1])
        e_accel = np.abs(pva[3, resting, 2])
        if caps["max_extrude_only_velocity"]:
            out["extrude_only_velocity_ratio"] = float(
                e_velocity.max() / caps["max_extrude_only_velocity"]
            )
        if caps["max_extrude_only_accel"]:
            out["extrude_only_accel_ratio"] = float(
                e_accel.max() / caps["max_extrude_only_accel"]
            )
    return out


def start_shift(base: dict, head: dict) -> float:
    base_spans, head_spans = (
        base["trajectory"]["spans"],
        head["trajectory"]["spans"],
    )
    if not base_spans or not head_spans:
        return 0.0
    return head_spans[0]["t_start"] - base_spans[0]["t_start"]


def motion_deltas(
    base_pva, base_duration, head_pva, head_duration, seams, shift
):
    import numpy as np

    common = min(base_duration, head_duration - shift)
    grid = np.arange(0.0, common + pending.SAMPLE_DT_S / 2, pending.SAMPLE_DT_S)
    grid = grid[(grid <= common) & (grid + shift >= 0.0)]
    base_t = np.linspace(0.0, base_duration, base_pva.shape[1])
    head_t = np.linspace(0.0, head_duration, head_pva.shape[1])
    spacing = max(base_t[1] - base_t[0], head_t[1] - head_t[0])
    seam_array = np.asarray(seams)
    out = {}
    for index, axis in enumerate(pending.AXES):
        peak_accel = float(np.abs(base_pva[index, :, 2]).max())
        sampling_floor = (
            {
                "p": spacing * spacing / 8.0 * peak_accel,
                "v": spacing / 2.0 * peak_accel,
                "a": 2.0 * peak_accel,
            }
            if shift
            else {"p": 0.0, "v": 0.0, "a": 0.0}
        )
        worst = {}
        for column, key in enumerate("pva"):
            delta = np.abs(
                np.interp(grid + shift, head_t, head_pva[index, :, column])
                - np.interp(grid, base_t, base_pva[index, :, column])
            )
            at = int(delta.argmax())
            t = float(grid[at] + shift)
            nearest = (
                float(np.abs(seam_array - t).min())
                if len(seam_array)
                else math.inf
            )
            worst[key] = {
                "max": float(delta[at]),
                "beyond_sampling": max(
                    0.0, float(delta[at]) - sampling_floor[key]
                ),
                "t": t,
                "near_seam": nearest <= SEAM_WINDOW_S,
            }
        out[axis] = worst
    return out


def fit_audit(sha: str) -> dict | None:
    cached = AUDIT_DIR / f"{sha}.json"
    if cached.exists():
        return json.loads(cached.read_text())
    probe = subprocess.run(
        [
            "git",
            "cat-file",
            "-e",
            f"{sha}:rust/pipeline-snapshot/tests/fit_tolerance.rs",
        ],
        cwd=ROOT,
    )
    if probe.returncode != 0:
        return None
    tree = pending.CACHE / "wt-audit" / pending.short(sha)
    tree.parent.mkdir(parents=True, exist_ok=True)
    pending.run(["git", "worktree", "prune"])
    pending.run(
        ["git", "worktree", "add", "--detach", "--force", str(tree), sha]
    )
    try:
        print(
            f"fit audit at {pending.short(sha)} ...",
            file=sys.stderr,
            flush=True,
        )
        result = subprocess.run(
            [
                "cargo",
                "test",
                "--release",
                "-p",
                "pipeline-snapshot",
                "--features",
                "fit-audit",
                "--test",
                "fit_tolerance",
                "every_accepted_fit_piece",
                "--",
                "--ignored",
                "--exact",
                "every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal",
            ],
            cwd=tree / "rust",
            env={**os.environ, "CARGO_TARGET_DIR": str(AUDIT_TARGET_DIR)},
            capture_output=True,
            text=True,
        )
    finally:
        pending.run(["git", "worktree", "remove", "--force", str(tree)])
    rows: dict = {}
    for line in (result.stdout + result.stderr).splitlines():
        match = AUDIT_LINE.match(line.strip())
        if match:
            key = f"{match['context']}/axis{match['axis']}"
            rows.setdefault(match["case"], {})[key] = {
                "pieces": int(match["count"]),
                "accel": float(match["accel"]),
                "position": float(match["position"]),
            }
    if result.returncode != 0 and not rows:
        raise ReviewError(
            f"fit audit at {pending.short(sha)} failed without a report:\n"
            f"{result.stdout[-1500:]}{result.stderr[-1500:]}"
        )
    AUDIT_DIR.mkdir(parents=True, exist_ok=True)
    cached.write_text(json.dumps(rows, indent=1))
    return rows


def worst_audit(rows: dict | None, name: str) -> dict | None:
    if rows is None:
        return None
    entries = rows.get(name, {})
    return {
        "accel": max((v["accel"] for v in entries.values()), default=0.0),
        "position": max((v["position"] for v in entries.values()), default=0.0),
    }


@dataclass
class Review:
    name: str
    verdict: str = "EXPECTED"
    regressions: list[str] = field(default_factory=list)
    suspicions: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)


def review_case(
    name: str,
    base_dir: Path,
    head_dir: Path,
    shas: tuple[str, str],
    expectation: Expectation | None,
    audits: tuple[dict | None, dict | None],
) -> Review:
    base_sha, head_sha = shas
    review = Review(name)
    base = pending.load_snapshot(base_dir, name)
    head = pending.load_snapshot(head_dir, name)
    base_pva, base_duration = pending.load_case(base_dir, name)
    head_pva, head_duration = pending.load_case(head_dir, name)
    config_path, gcode_path = case_paths(head_sha, name)
    caps = config_caps(show(head_sha, config_path))
    gcode_e = gcode_extrusion(show(head_sha, gcode_path))
    metrics = review.metrics

    import numpy as np

    if not (finite_everywhere(head) and np.isfinite(head_pva).all()):
        review.regressions.append("non-finite values in the head output")
        review.verdict = "REGRESSION"
        return review

    metrics["duration_s"] = {"base": base_duration, "head": head_duration}
    duration_delta = head_duration - base_duration
    shift = start_shift(base, head)
    metrics["start_shift_s"] = shift
    if shift:
        review.notes.append(
            f"motion starts {shift * 1e3:+.4g} ms later; compared after aligning "
            "the starts, within the 1 ms sampling's interpolation error"
        )
    deltas = motion_deltas(
        base_pva,
        base_duration,
        head_pva,
        head_duration,
        seam_times(head),
        shift,
    )
    metrics["deltas"] = deltas

    seams = {}
    for key in ("dp", "dv", "da"):
        base_seam, head_seam = base[f"seam_max_{key}"], head[f"seam_max_{key}"]
        seams[key] = {"base": base_seam, "head": head_seam}
        threshold = {
            "dp": caps["max_path_deviation"],
            "dv": caps["max_accel_deviation"] * pending.SAMPLE_DT_S,
            "da": caps["max_accel_deviation"],
        }[key]
        for index, (axis, before, after) in enumerate(
            zip(pending.AXES, base_seam, head_seam)
        ):
            grown = after > before * (1.0 + SEAM_GROWTH) + 1e-12
            where = worst_seam_time(head, index, key)
            message = (
                f"seam {key} on {axis} grew {before:.3g} -> {after:.3g}"
                f"{'' if where is None else f' (worst at t={where:.4f})'}"
            )
            if grown and after > threshold and key == "da":
                review.suspicions.append(
                    f"{message}, past {threshold:.3g}: a knot acceleration step "
                    "is the fit's or the signal's own"
                )
            elif grown and after > threshold:
                review.regressions.append(f"{message}, past {threshold:.3g}")
            elif grown and after > 1e-9:
                review.notes.append(message)
    metrics["seams"] = seams

    planned = {
        "base": planned_limit_excess(base),
        "head": planned_limit_excess(head),
    }
    metrics["planned_limit_excess"] = planned
    if (
        planned["head"] > LIMIT_SLACK
        and planned["head"] > planned["base"] + LIMIT_SLACK
    ):
        review.regressions.append(
            f"planned phases exceed their span caps by {planned['head']:.3g} "
            f"(base {planned['base']:.3g})"
        )
    shaped = {
        "base": shaped_limits(base_pva, base_duration, caps),
        "head": shaped_limits(head_pva, head_duration, caps),
    }
    metrics["shaped_limits"] = shaped
    for key, after in shaped["head"].items():
        before = shaped["base"].get(key, 0.0)
        if (
            after > 1.0 + SHAPED_LIMIT_SLACK
            and after > before + SHAPED_LIMIT_SLACK
        ):
            review.regressions.append(f"{key} {before:.4g} -> {after:.4g}")

    base_e = float(base_pva[3, -1, 0] - base_pva[3, 0, 0])
    head_e = float(head_pva[3, -1, 0] - head_pva[3, 0, 0])
    material = head_e - base_e
    metrics["material"] = {
        "gcode": gcode_e,
        "base": base_e,
        "head": head_e,
        "delta": material,
        "head_vs_gcode": head_e - gcode_e,
    }
    allowed_material = expectation.material if expectation else 0.0
    if abs(material) > max(allowed_material, MATERIAL_FLOOR_MM):
        review.regressions.append(
            f"material changed by {material:+.3g} mm (allowed {allowed_material:.3g})"
        )

    pieces = {"base": pieces_per_axis(base), "head": pieces_per_axis(head)}
    metrics["pieces"] = pieces

    base_audit, head_audit = (worst_audit(rows, name) for rows in audits)
    metrics["fit_audit"] = {"base": base_audit, "head": head_audit}
    if head_audit is not None:
        for key in ("accel", "position"):
            after = head_audit[key]
            before = base_audit[key] if base_audit else None
            if after <= 1.0:
                continue
            if before is not None and before <= 1.0:
                review.regressions.append(
                    f"fit audit {key} {after:.3g}x over tolerance (base within it)"
                )
            elif before is None:
                review.notes.append(
                    f"fit audit {key} {after:.3g}x over tolerance (no base audit)"
                )
            elif after > before * AUDIT_GROWTH:
                review.suspicions.append(
                    f"fit audit {key} {before:.3g}x -> {after:.3g}x"
                )

    if expectation is None:
        review.suspicions.append("no expectation covers this case")
    else:
        for axis, worst in deltas.items():
            moved = any(
                worst[k]["beyond_sampling"] > CHANGE_FLOOR[k] for k in "pva"
            )
            if moved and axis not in expectation.axes:
                review.suspicions.append(
                    f"{axis} moved (dp {worst['p']['max']:.3g}, dv {worst['v']['max']:.3g}, "
                    f"da {worst['a']['max']:.3g}) but only {','.join(expectation.axes)} may"
                )
            for k in "pva":
                if worst[k]["beyond_sampling"] > expectation.bounds[k]:
                    review.suspicions.append(
                        f"{axis} d{k} {worst[k]['max']:.3g} at t={worst[k]['t']:.4f}"
                        f"{' near a seam' if worst[k]['near_seam'] else ''} "
                        f"beyond {expectation.bounds[k]:.3g}"
                    )
        if abs(shift) > expectation.duration:
            review.suspicions.append(
                f"motion start {shift:+.3g} s beyond {expectation.duration:.3g}"
            )
        if abs(duration_delta) > expectation.duration:
            review.suspicions.append(
                f"traversal time {duration_delta:+.3g} s beyond {expectation.duration:.3g}"
            )
        for axis in pending.AXES:
            before = pieces["base"][axis]
            after = pieces["head"][axis]
            if (
                before["max_degree"] != after["max_degree"]
                and expectation.pieces != math.inf
            ):
                review.suspicions.append(
                    f"{axis} max degree {before['max_degree']} -> {after['max_degree']}"
                )
            low, high = sorted((before["pieces"], after["pieces"]))
            if high > max(low, 1) * expectation.pieces:
                review.suspicions.append(
                    f"{axis} pieces {before['pieces']} -> {after['pieces']}"
                )
    if review.regressions:
        review.verdict = "REGRESSION"
    elif review.suspicions:
        review.verdict = "SUSPICIOUS"
    return review


def compare(
    base_sha: str,
    head_sha: str,
    expectations: list[Expectation],
    audit: bool,
    only: str | None,
) -> tuple[list[Review], dict]:
    base_dir = pending.sample_commit(base_sha)
    head_dir = pending.sample_commit(head_sha)
    base_meta = json.loads((base_dir / "meta.json").read_text())
    head_meta = json.loads((head_dir / "meta.json").read_text())
    audits = (
        (fit_audit(base_sha), fit_audit(head_sha)) if audit else (None, None)
    )
    harness = pending.snapshot_harness()
    reviews, summary = [], {"unchanged": 0, "new": [], "removed": []}
    for name in sorted(set(base_meta) | set(head_meta)):
        if only and only not in name:
            continue
        base_info, head_info = base_meta.get(name), head_meta.get(name)
        if head_info is None:
            summary["removed"].append(name)
            continue
        if head_info["status"] == "error":
            review = Review(name, verdict="REGRESSION")
            if base_info and base_info["status"] == "error":
                review.verdict = "SUSPICIOUS"
                review.suspicions.append(
                    f"fails at base and head: {head_info['error']}"
                )
            else:
                review.regressions.append(
                    f"fails to plan: {head_info['error']}"
                )
            reviews.append(review)
            continue
        if base_info is None or base_info["status"] == "error":
            summary["new"].append(name)
            continue
        if harness.snapshots_match(
            pending.load_snapshot(base_dir, name),
            pending.load_snapshot(head_dir, name),
        ):
            summary["unchanged"] += 1
            continue
        reviews.append(
            review_case(
                name,
                base_dir,
                head_dir,
                (base_sha, head_sha),
                expectation_for(name, expectations),
                audits,
            )
        )
    if audits[1] is not None:
        snapshot_names = set(head_meta)
        extra = {}
        for name in set(audits[1]) - snapshot_names:
            before, after = (
                worst_audit(audits[0], name),
                worst_audit(audits[1], name),
            )
            assert after is not None, f"{name} vanished from the head audit"
            extra[name] = {"base": before, "head": after}
            regressed = any(
                after[k] > 1.0 and before is not None and before[k] <= 1.0
                for k in ("accel", "position")
            )
            if regressed:
                review = Review(name, verdict="REGRESSION")
                review.regressions.append(
                    f"fit audit over tolerance: {after} (base {before})"
                )
                review.metrics["fit_audit"] = extra[name]
                reviews.append(review)
        summary["audited_outside_snapshots"] = extra
    return reviews, summary


def headline(review: Review) -> str:
    deltas = review.metrics.get("deltas")
    if not deltas:
        return ""
    cells = []
    for k in "pva":
        axis, worst = max(deltas.items(), key=lambda item: item[1][k]["max"])
        cells.append(f"{worst[k]['max']:.1e}({axis})")
    return "/".join(cells)


def print_table(reviews: list[Review], summary: dict) -> None:
    order = {"REGRESSION": 0, "SUSPICIOUS": 1, "EXPECTED": 2}
    print(
        f"{'case':<52}{'verdict':<12}{'dT s':>10}  {'max dp/dv/da (axis)':<34}{'dE mm':>10}"
    )
    for review in sorted(reviews, key=lambda r: (order[r.verdict], r.name)):
        duration = review.metrics.get("duration_s")
        d_t = f"{duration['head'] - duration['base']:+.2e}" if duration else "-"
        material = review.metrics.get("material")
        d_e = f"{material['delta']:+.1e}" if material else "-"
        print(
            f"{review.name[:51]:<52}{review.verdict:<12}{d_t:>10}  {headline(review):<34}{d_e:>10}"
        )
        for reason in review.regressions:
            print(f"    ! {reason}")
        for reason in review.suspicions:
            print(f"    ? {reason}")
        for reason in review.notes:
            print(f"    . {reason}")
    counts = {v: sum(r.verdict == v for r in reviews) for v in order}
    print(
        f"\n{len(reviews)} changed: {counts['EXPECTED']} expected, "
        f"{counts['SUSPICIOUS']} suspicious, {counts['REGRESSION']} regression; "
        f"{summary['unchanged']} unchanged, {len(summary['new'])} new, "
        f"{len(summary['removed'])} removed"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--pr", type=int)
    parser.add_argument("--base")
    parser.add_argument("--head")
    parser.add_argument("--expect", action="append", default=[])
    parser.add_argument("--json", type=Path)
    parser.add_argument("--no-audit", action="store_true")
    parser.add_argument("-k", dest="only")
    args = parser.parse_args()
    pending.run(["git", "fetch", "origin", "-q"])
    if args.pr is not None:
        base_sha, head_sha, body = pr_endpoints(args.pr)
        specs = args.expect or expectations_from_body(body)
    elif args.base and args.head:
        base_sha, head_sha = resolve(args.base), resolve(args.head)
        specs = args.expect
    else:
        parser.error("give --pr N or both --base and --head")
    if args.base and args.pr is not None:
        base_sha = resolve(args.base)
    if args.head and args.pr is not None:
        head_sha = resolve(args.head)
    expectations = [Expectation.parse(spec) for spec in specs]
    print(
        f"reviewing {pending.short(head_sha)} against {pending.short(base_sha)}; "
        f"expectations: {specs or 'none'}",
        flush=True,
    )
    reviews, summary = compare(
        base_sha, head_sha, expectations, not args.no_audit, args.only
    )
    print_table(reviews, summary)
    if args.json:
        args.json.write_text(
            json.dumps(
                {
                    "base": base_sha,
                    "head": head_sha,
                    "expectations": specs,
                    "summary": summary,
                    "cases": [vars(review) for review in reviews],
                },
                indent=1,
            )
        )
    return 1 if any(r.verdict == "REGRESSION" for r in reviews) else 0


if __name__ == "__main__":
    sys.exit(main())
