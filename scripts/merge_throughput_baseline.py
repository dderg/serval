#!/usr/bin/env python3
"""Git merge driver for tools/bench/throughput_baseline.json.

    merge_throughput_baseline.py %O %A %B %P      (registered by install-git-guards.sh)

Every accepted PR rewrites this file at "the merge with main", so two PRs
accepting at once always touched the same lines: 6 of the ~12 merge conflicts
in one audit window. Merged field by field instead:

- a field changed on one side only takes that side;
- `configs.<config>`: both sides changed -> the entry with the lower Ir (its
  stream_secs stays with it). The gate fails when head Ir exceeds the baseline
  by more than 2 %, so the lower of two accepted baselines is the fail-safe
  choice: the merged tree is checked against the stricter of the two, never the
  looser, and a wrong guess shows up as a red gate rather than a silent pass;
- `commit` and `reason` (only printed in the report): ours, with both reasons
  kept when they differ;
- anything else changed on both sides (the Pi calibration, slice_lines) is a
  real conflict: exit 1 and leave the file for a human.

Exit 0 and the merged JSON in %A on success.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


class Unresolvable(Exception):
    pass


def lower_ir(ours: dict, theirs: dict) -> dict:
    return ours if ours["ir"] <= theirs["ir"] else theirs


def merge_configs(base: dict, ours: dict, theirs: dict) -> dict:
    merged = {}
    for name in sorted(set(ours) | set(theirs) | set(base)):
        o, t, b = ours.get(name), theirs.get(name), base.get(name)
        if o == t:
            choice = o
        elif o == b:
            choice = t
        elif t == b:
            choice = o
        else:
            choice = lower_ir(o, t) if o and t else None
            if choice is None:
                raise Unresolvable(
                    f"configs.{name}: one side removed it, the other changed it"
                )
        if choice is not None:
            merged[name] = choice
    return merged


def merge_reason(ours: str, theirs: str) -> str:
    if not theirs or theirs == ours:
        return ours
    if not ours:
        return theirs
    return f"{ours}; {theirs}"


def merge(base: dict, ours: dict, theirs: dict) -> dict:
    merged = {}
    for key in sorted(
        set(ours) | set(theirs) | set(base),
        key=lambda k: list(ours).index(k) if k in ours else 10**6,
    ):
        o, t, b = ours.get(key), theirs.get(key), base.get(key)
        if o == t:
            value = o
        elif o == b:
            value = t
        elif t == b:
            value = o
        elif key == "configs":
            value = merge_configs(b or {}, o or {}, t or {})
        elif key == "commit":
            value = o
        elif key == "reason":
            value = merge_reason(o or "", t or "")
        else:
            raise Unresolvable(f"{key}: changed differently on both sides")
        if value is not None:
            merged[key] = value
    return merged


def main(argv: list[str]) -> int:
    base_path, ours_path, theirs_path = (Path(a) for a in argv[1:4])
    try:
        base = json.loads(base_path.read_text() or "{}")
        ours = json.loads(ours_path.read_text())
        theirs = json.loads(theirs_path.read_text())
        merged = merge(base, ours, theirs)
    except (Unresolvable, ValueError, KeyError) as exc:
        print(f"merge_throughput_baseline: {exc}", file=sys.stderr)
        return 1
    ours_path.write_text(json.dumps(merged, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
