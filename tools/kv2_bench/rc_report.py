"""Release-candidate verdict table from one session directory.

    uv run python -m tools.kv2_bench.rc_report ~/bench/kv2/<stamp>

One row per gate and config: PASS when every run of it completed and every
check the analyzer enforces held, with the row's key number.
"""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

from . import clock_report, trials_report


def analysis_of(run_dir: Path) -> dict:
    for path in (
        run_dir / "analysis.json",
        run_dir / "replay" / "replay_judgement.json",
    ):
        if path.exists():
            return json.loads(path.read_text())
    return {}


def batch_row(runs: list[dict]) -> tuple[bool, str]:
    worst = []
    ok = True
    for r in runs:
        a = analysis_of(Path(r["run_dir"])) if r.get("run_dir") else {}
        ok &= "error" not in r and bool(a.get("ok"))
        timing = a.get("capture_vs_reference", {}).get("timing") or a.get(
            "capture_vs_replay", {}
        ).get("timing", {})
        if "abs_max_us" in timing:
            worst.append(
                (timing["abs_max_us"], timing.get("bound_us", float("nan")))
            )
    if not worst:
        return ok, f"{len(runs)} runs"
    cap, bound = max(worst, key=lambda pair: pair[0] / pair[1])
    return (
        ok,
        f"{len(runs)} runs, worst capture−ref {cap:.1f} / bound {bound:.1f} µs",
    )


def g4_row(
    session: Path, records: list[dict]
) -> list[tuple[str, str, bool, str]]:
    rows = []
    by_program = collections.defaultdict(dict)
    for r in records:
        if (
            r.get("config") == "owner_like"
            and r.get("tap")
            and r.get("gate") in ("g2", "g4")
        ):
            by_program[r["program"]].setdefault(r["tree"], r)
    from . import compare

    for program, pair in sorted(by_program.items()):
        if (
            set(pair) != {"kv2", "upstream"}
            or "error" in pair["kv2"]
            or "error" in pair["upstream"]
        ):
            continue
        refs = [
            json.loads(
                (
                    session
                    / "refs"
                    / tree
                    / "owner_like"
                    / program
                    / "reference.json"
                ).read_text()
            )["messages_sha256"]
            for tree in ("kv2", "upstream")
        ]
        captured = compare.sequences(
            compare.load_capture(Path(pair["kv2"]["run_dir"])),
            compare.load_capture(Path(pair["upstream"]["run_dir"])),
        )["all_steps_paired"]
        ok = (
            refs[0] == refs[1]
            and captured
            and analysis_of(Path(pair["upstream"]["run_dir"])).get("ok", False)
        )
        rows.append(
            (
                "G4 upstream identity",
                f"owner_like {program}",
                ok,
                "batch messages identical"
                if refs[0] == refs[1]
                else "batch messages DIFFER",
            )
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.expanduser()
    data = json.loads((session / "session.json").read_text())
    records = data["runs"]
    rows: list[tuple[str, str, bool, str]] = []
    grouped = collections.defaultdict(list)
    for r in records:
        if r.get("gate") in ("g1", "g2", "soak"):
            label = {
                "g1": "G1 live print",
                "g2": "G2 executed = planned",
                "soak": "Soak (replay judge)",
            }[r["gate"]]
            host = " 1 cpu/1500" if r.get("host", {}).get("cpus") else ""
            grouped[(label, r["config"] + host)].append(r)
    for (label, config), runs in grouped.items():
        ok, key = batch_row(runs)
        rows.append((label, config, ok, key))
    rows += g4_row(session, records)
    trials = trials_report.summarize(session)
    for config, v in trials.items():
        rows.append(
            (
                "Trials arcs_x30 1 cpu/1500",
                config,
                v["failures"] == 0,
                f"{v['failures']}/{v['runs']} failed, lateness p99 {v['pooled_late_ge_2ms']['p99_ms']:.1f} ms max {v['pooled_late_ge_2ms']['max_ms']:.1f} ms",
            )
        )
    clocks = clock_report.summarize(session)
    for kind, v in clocks.items():
        low, high = v["calibrated_ppm_range"]
        rows.append(
            (
                "Clock trials 1 cpu/1500",
                kind,
                v["failures"] == 0,
                f"{v['failures']}/{v['runs']} failed, calibration {low:+.0f}…{high:+.0f} ppm",
            )
        )
    restoration = json.loads((session / "restoration.json").read_text())
    rows.append(
        (
            "Restoration",
            "serval state",
            not restoration["differences"],
            f"differences {restoration['differences']}",
        )
    )
    print("| gate | config | result | key number |\n|---|---|---|---|")
    for gate, config, ok, key in rows:
        print(f"| {gate} | {config} | {'PASS' if ok else 'FAIL'} | {key} |")


if __name__ == "__main__":
    main()
