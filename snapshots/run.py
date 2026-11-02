#!/usr/bin/env python3
"""Standalone snapshot test runner.

Compares every case's current trajectory to its committed baseline and reports
EXACT / CHANGED / PENDING. Exit 0 when everything matches, 1 when a case changed
or is still pending (no baseline yet), 2 on a setup error (malformed case or the
cdylib not built). This is the snapshot test pillar — it does not use pytest and
runs only the case comparisons; the harness's own unit tests live in
`test_harness.py` and run under the `py` job.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-k", dest="filter", help="only run cases whose name contains this"
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        help="write per-case statuses and non-matching snapshots here, so the "
        "review server can serve them without re-running every case",
    )
    parser.add_argument(
        "--accept",
        nargs="+",
        metavar="CASE",
        default=[],
        help="write the baseline of each named CHANGED or NEW case",
    )
    parser.add_argument(
        "--accept-all",
        action="store_true",
        help="write the baseline of every CHANGED or NEW case",
    )
    args = parser.parse_args()

    try:
        all_cases = harness.discover_cases()
    except ValueError as exc:
        print(f"  ERROR   {exc}")
        return 2
    for baseline in harness.prune_orphan_baselines(all_cases):
        print(f"  PRUNED   {baseline.relative_to(harness.BASELINES_DIR)}")

    if args.accept and args.accept_all:
        print("  ERROR   --accept and --accept-all are exclusive")
        return 2
    known = {c.name for c in all_cases}
    unknown = sorted(set(args.accept) - known)
    if unknown:
        print(f"  ERROR   unknown case(s): {', '.join(unknown)}")
        return 2

    cases = all_cases
    if args.accept:
        cases = [c for c in all_cases if c.name in args.accept]
    if args.filter:
        cases = [c for c in cases if args.filter in c.name]
    if not cases:
        print("no snapshot cases found under cases/")
        return 1

    buckets = {
        harness.Status.EXACT: [],
        harness.Status.CHANGED: [],
        harness.Status.NEW: [],
    }
    statuses: dict[str, str] = {}
    accepting = bool(args.accept) or args.accept_all
    written: list[str] = []
    try:
        for case, snapshot in harness.run_cases_parallel(cases):
            status = harness.compare(case, snapshot)
            buckets[status].append(case.name)
            statuses[case.name] = status.value
            label = (
                "PENDING"
                if status is harness.Status.NEW
                else status.value.upper()
            )
            print(f"  {label:8} {case.name}")
            if status is not harness.Status.EXACT and args.results_dir:
                out = (
                    args.results_dir / f"{case.name.replace('/', '__')}.json.gz"
                )
                out.parent.mkdir(parents=True, exist_ok=True)
                data = (harness.canonical_json(snapshot) + "\n").encode()
                out.write_bytes(gzip.compress(data, compresslevel=1))
            if accepting and status is not harness.Status.EXACT:
                harness.write_baseline(case, snapshot)
                written.append(case.name)
                print(
                    f"  WROTE    {case.baseline_path.relative_to(harness.BASELINES_DIR)}"
                )
                continue
            if status is harness.Status.CHANGED:
                baseline = harness.baseline_snapshot(case)
                d = harness.drift_envelope(baseline, snapshot)
                if d["schema_at"]:
                    print(f"             schema changed at {d['schema_at']}")
                else:
                    print(
                        f"             worst rel {d['rel']:.2e} at {d['rel_at']}"
                    )
                    print(
                        f"             worst abs {d['abs']:.2e} at {d['abs_at']}"
                    )
    except (ImportError, ValueError) as exc:
        print(f"  ERROR   {exc}")
        return 2

    if args.results_dir:
        args.results_dir.mkdir(parents=True, exist_ok=True)
        (args.results_dir / "status.json").write_text(json.dumps(statuses))

    not_written = set(written)
    ok = buckets[harness.Status.EXACT]
    changed = [
        n for n in buckets[harness.Status.CHANGED] if n not in not_written
    ]
    pending = [n for n in buckets[harness.Status.NEW] if n not in not_written]
    print(
        f"\n{len(ok)} ok, {len(changed)} changed, {len(pending)} pending"
        + (f", {len(written)} accepted" if accepting else "")
    )
    missed = [
        n
        for n in args.accept
        if n not in written and statuses.get(n) == "exact"
    ]
    if missed:
        print(f"  ERROR   nothing to accept for: {', '.join(missed)}")
        return 2
    return 0 if not changed and not pending else 1


if __name__ == "__main__":
    raise SystemExit(main())
