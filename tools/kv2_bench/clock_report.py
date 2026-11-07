"""Clock-sync outcome of `--gates clocktrials` runs.

    uv run python -m tools.kv2_bench.clock_report ~/bench/kv2/<stamp>

Per run, from the probe's record of the main MCU's ClockSync: the
frequency estimate after the connect-time calibration and its worst
departure from the dictionary CLOCK_FREQ afterwards, how many samples the
outlier filter ignored, and whether klippy shut down. A run is "skewed"
when the estimate is off by more than SKEW_PPM at any point after connect.
"""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

from . import headroom

SKEW_PPM = 1000.0


def main_sync(probe: dict) -> dict:
    syncs = [
        log
        for name, log in probe["clocksync"].items()
        if name.startswith("ClockSync@")
    ]
    if len(syncs) != 1:
        raise ValueError(f"expected one main ClockSync, found {len(syncs)}")
    return syncs[0]


def run_outcome(record: dict) -> dict:
    run_dir = Path(record["run_dir"])
    probe = json.loads((run_dir / "probe.json").read_text())
    sync = main_sync(probe)
    nominal = sync["connect"]["mcu_freq"]
    samples = sync["samples"]
    after = [s for s in samples if not s["in_connect"]]
    calibrated = [s for s in samples if s["in_connect"]][-1]["freq"]
    ppm = [1e6 * (s["freq"] / nominal - 1.0) for s in after] or [0.0]
    log = (run_dir / "klippy.log").read_text()
    return {
        "run_dir": str(run_dir),
        "kind": record["kind"],
        "failure": record["failure"],
        "fatal_lines": headroom.fatal_lines(log)[:2],
        "calibrated_ppm": 1e6 * (calibrated / nominal - 1.0),
        "worst_ppm": max(ppm, key=abs),
        "skewed": abs(1e6 * (calibrated / nominal - 1.0)) > SKEW_PPM
        or max(abs(p) for p in ppm) > SKEW_PPM,
        "ignored": sum(s["decision"] == "ignored" for s in after),
        "resets": sum(s["decision"] == "reset" for s in after),
        "connect_samples": [
            {
                "sent": s["sent"],
                "rtt_ms": None
                if not s["sent"]
                else 1e3 * (s["received"] - s["sent"]),
                "clock": s["clock"],
                "decision": s["decision"],
                "freq_ppm": 1e6 * (s["freq"] / nominal - 1.0),
            }
            for s in samples
            if s["in_connect"]
        ],
    }


def summarize(session: Path) -> dict:
    records = [
        r
        for r in json.loads((session / "session.json").read_text())["runs"]
        if r.get("gate") == "clocktrials"
    ]
    by_kind: dict[str, list[dict]] = collections.defaultdict(list)
    for record in records:
        by_kind[record["kind"]].append(run_outcome(record))
    return {
        kind: {
            "runs": len(runs),
            "failures": sum(r["failure"] is not None for r in runs),
            "skewed": sum(r["skewed"] for r in runs),
            "calibrated_ppm_range": [
                min(r["calibrated_ppm"] for r in runs),
                max(r["calibrated_ppm"] for r in runs),
            ],
            "bad_runs": [r for r in runs if r["skewed"] or r["failure"]],
        }
        for kind, runs in by_kind.items()
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    result = summarize(args.session.expanduser())
    (args.session.expanduser() / "clock_trials.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
