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
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-k", dest="filter", help="only run cases whose name contains this"
    )
    args = parser.parse_args()

    try:
        all_cases = harness.discover_cases()
    except ValueError as exc:
        print(f"  ERROR   {exc}")
        return 2
    for baseline in harness.prune_orphan_baselines(all_cases):
        print(f"  PRUNED   {baseline.relative_to(harness.BASELINES_DIR)}")

    cases = all_cases
    if args.filter:
        cases = [c for c in all_cases if args.filter in c.name]
    if not cases:
        print("no snapshot cases found under cases/")
        return 1

    buckets = {
        harness.Status.EXACT: [],
        harness.Status.CHANGED: [],
        harness.Status.NEW: [],
    }
    for case in cases:
        try:
            snapshot = harness.run_case(case)
        except (ImportError, ValueError) as exc:
            print(f"  ERROR   {case.name}: {exc}")
            return 2
        status = harness.compare(case, snapshot)
        buckets[status].append(case.name)
        label = (
            "PENDING" if status is harness.Status.NEW else status.value.upper()
        )
        print(f"  {label:8} {case.name}")
        if status is harness.Status.CHANGED:
            baseline = harness.baseline_snapshot(case)
            d = harness.drift_envelope(baseline, snapshot)
            print(f"             worst rel {d['rel']:.2e} at {d['rel_at']}")
            print(f"             worst abs {d['abs']:.2e} at {d['abs_at']}")
            dump_dir = os.environ.get("SNAPSHOT_DUMP_DIR")
            if dump_dir:
                out = Path(dump_dir) / f"{case.name}.json.gz"
                out.parent.mkdir(parents=True, exist_ok=True)
                data = (harness.canonical_json(snapshot) + "\n").encode()
                out.write_bytes(gzip.compress(data, compresslevel=9, mtime=0))

    ok = buckets[harness.Status.EXACT]
    changed = buckets[harness.Status.CHANGED]
    pending = buckets[harness.Status.NEW]
    print(f"\n{len(ok)} ok, {len(changed)} changed, {len(pending)} pending")
    return 0 if not changed and not pending else 1


if __name__ == "__main__":
    raise SystemExit(main())
