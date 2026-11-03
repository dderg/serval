#!/usr/bin/env python3
"""Which open PRs wait on snapshot baselines, and by how much.

    uv run python scripts/pending_baselines.py [--pr N ...] [-v] [--all]
    uv run python scripts/pending_baselines.py --review N [--dry-run]

For every open PR it reports the stack relationship, mergeability, CI state
and the snapshot cases whose output changes: each case's engine output at the
PR head is compared with the output at its base (the base branch tip, which
for a stacked PR is the parent PR's head) as max |dp|, |dv|, |da| per axis
on a 1 ms grid, and with the baselines committed at the head (what the
snapshot CI job fails on). Engines are built in throwaway worktrees against a
shared cargo target dir; results are cached by commit under
~/.cache/pending-baselines.

--review N checks the PR head out in a worktree on branch review-pr-N and starts
snapshots/snapshot-tests.sh there, so the changed cases can be inspected and
accepted; commit the written baselines there and push them to the PR branch.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CACHE = Path(
    os.environ.get("PENDING_BASELINES_CACHE", "~/.cache/pending-baselines")
).expanduser()
TARGET_DIR = CACHE / "target"
SAMPLE_DT_S = 1e-3
AXES = ("x", "y", "z", "e")
DETAIL_CASES = 6
PR_FIELDS = (
    "number,title,headRefName,headRefOid,baseRefName,mergeable,isDraft,"
    "statusCheckRollup"
)


class PendingBaselinesError(RuntimeError):
    pass


def run(args: list[str], cwd: Path = ROOT, env: dict | None = None) -> str:
    result = subprocess.run(
        args,
        cwd=cwd,
        env={**os.environ, **(env or {})},
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise PendingBaselinesError(
            f"{' '.join(args)} failed ({result.returncode}):\n"
            f"{result.stdout[-2000:]}{result.stderr[-2000:]}"
        )
    return result.stdout


@dataclass
class Pr:
    number: int
    title: str
    branch: str
    head: str
    base_ref: str
    mergeable: str
    draft: bool
    checks: list[dict]
    parent: int | None = None
    base: str = ""
    comparison: dict = field(default_factory=dict)
    error: str = ""


def load_prs(only: list[int]) -> list[Pr]:
    raw = json.loads(
        run(
            [
                "gh",
                "pr",
                "list",
                "--state",
                "open",
                "--limit",
                "100",
                "--json",
                PR_FIELDS,
            ]
        )
    )
    prs = [
        Pr(
            r["number"],
            r["title"],
            r["headRefName"],
            r["headRefOid"],
            r["baseRefName"],
            r["mergeable"],
            r["isDraft"],
            r["statusCheckRollup"] or [],
        )
        for r in raw
        if not only or r["number"] in only
    ]
    by_branch = {r["headRefName"]: r for r in raw}
    main = run(["git", "rev-parse", "origin/main"]).strip()
    for pr in prs:
        parent = by_branch.get(pr.base_ref)
        if pr.base_ref == "main":
            pr.base = main
        elif parent:
            pr.parent = parent["number"]
            pr.base = parent["headRefOid"]
        else:
            pr.base = run(["git", "rev-parse", f"origin/{pr.base_ref}"]).strip()
    return sorted(prs, key=lambda p: p.number)


def ensure_commit(sha: str) -> None:
    probe = subprocess.run(
        ["git", "cat-file", "-e", f"{sha}^{{commit}}"], cwd=ROOT
    )
    if probe.returncode != 0:
        run(["git", "fetch", "origin", sha])


def ci_state(pr: Pr) -> tuple[str, list[str]]:
    failed, pending = [], 0
    for check in pr.checks:
        name = check.get("name") or check.get("context") or "?"
        conclusion = check.get("conclusion") or check.get("state") or ""
        status = check.get("status") or "COMPLETED"
        if status != "COMPLETED" or conclusion in ("PENDING", "EXPECTED"):
            pending += 1
        elif conclusion in ("FAILURE", "ERROR", "TIMED_OUT", "CANCELLED"):
            failed.append(name)
    if failed:
        only_snapshot = all(n == "snapshot" for n in failed)
        label = "red:snapshot" if only_snapshot else f"red:{len(failed)}"
        return label, failed
    return ("pending" if pending else "green"), []


def short(sha: str) -> str:
    return sha[:10]


def sample_dir(sha: str) -> Path:
    return CACHE / "samples" / sha


def sample_commit(sha: str) -> Path:
    out = sample_dir(sha)
    if (out / "meta.json").exists():
        return out
    ensure_commit(sha)
    tree = CACHE / "wt" / short(sha)
    tree.parent.mkdir(parents=True, exist_ok=True)
    run(["git", "worktree", "prune"])
    shutil.rmtree(tree, ignore_errors=True)
    run(["git", "worktree", "add", "--detach", "--force", str(tree), sha])
    try:
        env = {"CARGO_TARGET_DIR": str(TARGET_DIR)}
        print(
            f"building engine at {short(sha)} ...", file=sys.stderr, flush=True
        )
        run(["make", "-f", "Makefile.rust", "motion-engine-fast"], tree, env)
        out.mkdir(parents=True, exist_ok=True)
        run_sample_worker(tree, out)
    finally:
        run(["git", "worktree", "remove", "--force", str(tree)])
    return out


def read_progress(out: Path) -> tuple[dict, str | None]:
    done, running = {}, None
    path = out / "progress.jsonl"
    for line in path.read_text().splitlines() if path.exists() else []:
        entry = json.loads(line)
        if "meta" in entry:
            done[entry["case"]] = entry["meta"]
            running = None
        else:
            running = entry["case"]
    return done, running


def run_sample_worker(tree: Path, out: Path) -> None:
    (out / "progress.jsonl").unlink(missing_ok=True)
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--sample-worker",
        str(tree),
        str(out),
    ]
    while True:
        result = subprocess.run(
            command, cwd=tree, capture_output=True, text=True
        )
        done, crashed = read_progress(out)
        if result.returncode == 0:
            break
        if crashed is None:
            raise PendingBaselinesError(
                f"sample worker died outside a case ({result.returncode}):\n"
                f"{result.stderr[-2000:]}"
            )
        panic = [ln for ln in result.stderr.splitlines() if ln.strip()]
        message = next(
            (ln for ln in reversed(panic) if ln.startswith("shaper:")),
            panic[-1] if panic else "worker aborted",
        )
        append_progress(
            out,
            {
                "case": crashed,
                "meta": {"status": "error", "error": message[:400]},
            },
        )
    done, _ = read_progress(out)
    errors = {
        info["error"] for info in done.values() if info["status"] == "error"
    }
    if (
        done
        and len(errors) == 1
        and all(info["status"] == "error" for info in done.values())
    ):
        shutil.rmtree(out)
        raise PendingBaselinesError(
            f"every case failed the same way, not caching: {errors.pop()}"
        )
    (out / "meta.json").write_text(json.dumps(done, indent=1))


def append_progress(out: Path, entry: dict) -> None:
    with (out / "progress.jsonl").open("a") as handle:
        handle.write(json.dumps(entry) + "\n")


def sample_worker(tree: Path, out: Path) -> None:
    import importlib

    for sub in ("klippy", "snapshots", "."):
        sys.path.insert(0, str(tree / sub))
    harness = importlib.import_module("harness")
    engine = harness._import_engine()
    done, _ = read_progress(out)
    for case in harness.discover_cases():
        if case.name in done:
            continue
        append_progress(out, {"case": case.name})
        try:
            meta = sample_case(harness, engine, case, out)
        except (KeyboardInterrupt, SystemExit):
            raise
        except BaseException as exc:
            meta = {"status": "error", "error": str(exc)[:400]}
        append_progress(out, {"case": case.name, "meta": meta})


def sample_case(harness, engine, case, out: Path) -> dict:
    import gzip

    import numpy as np

    cfg = harness.read_printer_config(case.config_path)
    waypoints = engine.parse_gcode(
        case.gcode_path.read_text(), cfg.max_velocity, cfg.max_accel
    )
    snapshot = harness.run_case(case)
    duration = float(snapshot["traversal_time_s"])
    count = max(2, math.ceil(duration / SAMPLE_DT_S) + 1)
    _, rows = engine.pipeline_snapshot_axis_samples(
        waypoints, cfg.config_text, [0, 1, 2, 3], count
    )
    stem = case.name.replace("/", "__")
    np.savez_compressed(
        out / f"{stem}.npz",
        pva=np.asarray(rows, dtype=np.float64),
        duration=duration,
    )
    (out / f"{stem}.json.gz").write_bytes(
        gzip.compress((harness.canonical_json(snapshot) + "\n").encode(), 1)
    )
    return {
        "status": harness.compare(case, snapshot).value,
        "duration_s": duration,
    }


def load_snapshot(directory: Path, name: str) -> dict:
    import gzip

    path = directory / f"{name.replace('/', '__')}.json.gz"
    return json.loads(gzip.decompress(path.read_bytes()))


def load_case(directory: Path, name: str):
    import numpy as np

    path = directory / f"{name.replace('/', '__')}.npz"
    with np.load(path) as data:
        return data["pva"], float(data["duration"])


def compare_case(base_dir: Path, head_dir: Path, name: str) -> dict | None:
    import numpy as np

    harness = snapshot_harness()
    base_snapshot = load_snapshot(base_dir, name)
    head_snapshot = load_snapshot(head_dir, name)
    if harness.snapshots_match(base_snapshot, head_snapshot):
        return None
    drift = harness.drift_envelope(base_snapshot, head_snapshot)

    base_pva, base_duration = load_case(base_dir, name)
    head_pva, head_duration = load_case(head_dir, name)
    common = min(base_duration, head_duration)
    grid = np.arange(0.0, common + SAMPLE_DT_S / 2, SAMPLE_DT_S)
    grid = grid[grid <= common]
    axes = {}
    for index, axis in enumerate(AXES):
        worst = {}
        for column, key in enumerate("pva"):
            base_t = np.linspace(0.0, base_duration, base_pva.shape[1])
            head_t = np.linspace(0.0, head_duration, head_pva.shape[1])
            delta = np.interp(grid, head_t, head_pva[index, :, column]) - (
                np.interp(grid, base_t, base_pva[index, :, column])
            )
            worst[key] = float(np.abs(delta).max())
        axes[axis] = worst
    return {
        "axes": axes,
        "duration_delta_s": head_duration - base_duration,
        "drift": drift,
    }


def snapshot_harness():
    import importlib

    sys.path.insert(0, str(ROOT / "snapshots"))
    return importlib.import_module("harness")


def analyze(pr: Pr) -> None:
    head_dir = sample_commit(pr.head)
    base_dir = sample_commit(pr.base)
    head_meta = json.loads((head_dir / "meta.json").read_text())
    base_meta = json.loads((base_dir / "meta.json").read_text())
    against_baselines = {
        name: info["status"]
        for name, info in head_meta.items()
        if info["status"] != "exact"
    }
    changed, new, errors = {}, [], {}
    for name, info in head_meta.items():
        base_info = base_meta.get(name)
        if info["status"] == "error" or (
            base_info and base_info["status"] == "error"
        ):
            errors[name] = info.get("error") or "fails at its base"
        elif base_info is None:
            new.append(name)
        else:
            result = compare_case(base_dir, head_dir, name)
            if result:
                changed[name] = result
    pr.comparison = {
        "against_baselines": against_baselines,
        "changed": changed,
        "new": new,
        "errors": errors,
    }


def worst(changed: dict) -> dict:
    out = {key: ("", 0.0) for key in "pva"}
    for name, result in changed.items():
        for axis, values in result["axes"].items():
            for key in "pva":
                if values[key] > out[key][1]:
                    out[key] = (axis, values[key])
    return out


def merge_order(prs: list[Pr]) -> list[Pr]:
    numbers = {p.number for p in prs}
    remaining = {p.number: p for p in prs}
    done: set[int] = set()
    order = []

    def key(pr: Pr):
        pending = len(pr.comparison.get("against_baselines", {}))
        return (pending > 0, pr.mergeable == "CONFLICTING", pending, pr.number)

    while remaining:
        ready = [
            p
            for p in remaining.values()
            if p.parent is None or p.parent in done or p.parent not in numbers
        ]
        if not ready:
            raise PendingBaselinesError("stack cycle among open PRs")
        pick = min(ready, key=key)
        order.append(pick)
        done.add(pick.number)
        del remaining[pick.number]
    return order


def print_table(prs: list[Pr]) -> None:
    print(
        f"{'PR':<5}{'branch':<28}{'base':<26}{'merge':<12}{'CI':<14}"
        f"{'vs base':<9}{'vs baselines':<14}{'errors':<8}worst dp / dv / da (axis)"
    )
    for pr in prs:
        ci, _ = ci_state(pr)
        base = f"#{pr.parent}" if pr.parent else pr.base_ref
        comparison = pr.comparison
        w = worst(comparison["changed"])
        deltas = " / ".join(f"{w[k][1]:.2e}({w[k][0] or '-'})" for k in "pva")
        pending = len(comparison["against_baselines"])
        print(
            f"{pr.number:<5}{pr.branch[:26]:<28}{base[:24]:<26}"
            f"{pr.mergeable.lower():<12}{ci:<14}"
            f"{len(comparison['changed']):<9}{pending:<14}"
            f"{len(comparison['errors']):<8}{deltas}"
        )


def case_rank(item: tuple) -> tuple:
    result = item[1]
    peak = max(v["p"] for v in result["axes"].values())
    return (-peak, -result["drift"]["rel"], -result["drift"]["abs"])


def print_details(prs: list[Pr], limit: int | None) -> None:
    for pr in prs:
        comparison = pr.comparison
        print(f"\n#{pr.number} {pr.title}")
        _, failed = ci_state(pr)
        if failed:
            print(f"  failing checks: {', '.join(failed)}")
        print(
            f"  cases differing from the baselines committed at its head: "
            f"{len(comparison['against_baselines'])}; "
            f"new cases vs base: {len(comparison['new'])}"
        )
        by_message: dict[str, list[str]] = {}
        for name, message in comparison["errors"].items():
            by_message.setdefault(message[:160], []).append(name)
        for message, names in by_message.items():
            print(
                f"  PLAN ERROR in {len(names)} case(s), e.g. {names[0]}: {message}"
            )
        ranked = sorted(comparison["changed"].items(), key=case_rank)
        for name, result in ranked[: limit or len(ranked)]:
            cells = " ".join(
                f"{axis}:{v['p']:.1e}/{v['v']:.1e}/{v['a']:.1e}"
                for axis, v in result["axes"].items()
            )
            print(f"  {name:<46} dT={result['duration_delta_s']:+.2e}s {cells}")
            drift = result["drift"]
            if drift["schema_at"]:
                print(f"    structure differs at {drift['schema_at']}")
            elif all(
                v["p"] == v["v"] == v["a"] == 0 for v in result["axes"].values()
            ):
                where = drift["rel_at"] or drift["abs_at"]
                print(f"    trajectory samples identical; differs at {where}")
        if limit and len(ranked) > limit:
            print(f"  ... {len(ranked) - limit} more (use --all)")


def overlap_with_earlier(order: list[Pr]) -> dict[int, list[str]]:
    notes: dict[int, list[str]] = {}
    for index, pr in enumerate(order):
        mine = set(pr.comparison["against_baselines"])
        notes[pr.number] = [
            f"#{earlier.number}({len(mine & set(earlier.comparison['against_baselines']))})"
            for earlier in order[:index]
            if mine & set(earlier.comparison["against_baselines"])
        ]
    return notes


def print_order(prs: list[Pr]) -> None:
    order = merge_order(prs)
    overlaps = overlap_with_earlier(order)
    print(
        "\nSuggested merge order (parents first; PRs needing no baselines, "
        "then fewest pending cases):"
    )
    for position, pr in enumerate(order, 1):
        pending = len(pr.comparison["against_baselines"])
        ci, _ = ci_state(pr)
        notes = []
        if pr.parent:
            notes.append(f"after #{pr.parent}")
        if pr.mergeable == "CONFLICTING":
            notes.append("rebase first (conflicts)")
        if pending:
            notes.append(f"accept {pending} baseline case(s)")
        elif ci != "green":
            notes.append(f"CI {ci}")
        if overlaps[pr.number]:
            notes.append(
                "rebase + re-accept shared cases after merging "
                + ", ".join(overlaps[pr.number])
            )
        suffix = f"  [{'; '.join(notes)}]" if notes else ""
        print(f"  {position}. #{pr.number} {pr.title}{suffix}")


def review(number: int, dry_run: bool) -> None:
    prs = load_prs([number])
    if not prs:
        raise PendingBaselinesError(f"no open PR #{number}")
    pr = prs[0]
    tree = CACHE / "review" / str(number)
    local = f"review-pr-{number}"
    commands = [
        ["git", "fetch", "origin", pr.branch],
        [
            "git",
            "worktree",
            "add",
            "-B",
            local,
            str(tree),
            f"origin/{pr.branch}",
        ],
    ]
    launch = [str(tree / "snapshots" / "snapshot-tests.sh")]
    env = {"CARGO_TARGET_DIR": str(TARGET_DIR)}
    if dry_run:
        for command in commands:
            print("+", " ".join(command))
        print(f"+ (cwd {tree}) CARGO_TARGET_DIR={TARGET_DIR} {launch[0]}")
        return
    if tree.exists():
        raise PendingBaselinesError(
            f"{tree} exists; finish or remove it: git worktree remove {tree}"
        )
    for command in commands:
        run(command)
    print(
        f"reviewing #{number} in {tree}\n"
        f"after accepting: cd {tree} && git add snapshots/baselines && "
        f"git commit && git push origin {local}:{pr.branch}",
        flush=True,
    )
    os.chdir(tree)
    os.execve(launch[0], launch, {**os.environ, **env})


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--pr", type=int, action="append", default=[])
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--review", type=int)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--sample-worker", nargs=2, metavar=("TREE", "OUT"))
    args = parser.parse_args()
    if args.sample_worker:
        sample_worker(*map(Path, args.sample_worker))
        return 0
    run(["git", "fetch", "origin", "-q"])
    if args.review is not None:
        review(args.review, args.dry_run)
        return 0
    prs = load_prs(args.pr)
    for pr in prs:
        try:
            analyze(pr)
        except PendingBaselinesError as exc:
            lines = [
                ln for ln in exc.args[0].strip().splitlines() if ln.strip()
            ]
            pr.error = " | ".join(lines[-3:])
    analyzed = [pr for pr in prs if not pr.error]
    print_table(analyzed)
    if args.verbose or args.all:
        print_details(analyzed, None if args.all else DETAIL_CASES)
    print_order(analyzed)
    failed = [pr for pr in prs if pr.error]
    for pr in failed:
        print(f"\n#{pr.number} {pr.branch}: NOT ANALYZED: {pr.error}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
