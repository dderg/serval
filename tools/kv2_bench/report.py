"""Markdown tables from a session directory (session.json + run dirs).

    uv run python -m tools.kv2_bench.report ~/bench/kv2/<stamp> [...]

Several session dirs may be given; their runs are concatenated.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from . import compare
from .reference import Reference

RATE_WINDOW_STEPS = 100
AGGREGATE_WINDOW_STEPS = 400


def peak_rates(ref_dir: Path) -> dict:
    reference = Reference.load(ref_dir)
    freq = reference.clock_freq
    out = {}
    merged = []
    for name, steps in reference.steppers.items():
        t = steps.clocks / freq
        merged.append(t)
        k = RATE_WINDOW_STEPS
        out[name] = float((k / (t[k:] - t[:-k])).max()) if len(t) > k else 0.0
    t = np.sort(np.concatenate(merged))
    k = AGGREGATE_WINDOW_STEPS
    out["aggregate"] = float((k / (t[k:] - t[:-k])).max())
    out["total_steps"] = int(len(t))
    return out


def host_label(host: dict) -> str:
    parts = []
    if "cpus" in host:
        parts.append(f"{host['cpus']} cpu")
    if "max_mhz" in host:
        parts.append(f"{host['max_mhz']} MHz")
    if "quota_pct" in host:
        parts.append(f"quota {host['quota_pct']}%")
    return ", ".join(parts) or "4 cpu, 2400 MHz"


def load_runs(sessions: list[Path]) -> list[dict]:
    runs = []
    for session in sessions:
        data = json.loads((session / "session.json").read_text())
        for record in data["runs"]:
            record["session"] = str(session)
            runs.append(record)
    return runs


def analysis(record: dict) -> dict:
    run_dir = record.get("run_dir")
    path = Path(run_dir) / "analysis.json" if run_dir else None
    return json.loads(path.read_text()) if path and path.exists() else {}


def headroom(record: dict) -> dict:
    run_dir = record.get("run_dir")
    path = Path(run_dir) / "headroom.json" if run_dir else None
    return json.loads(path.read_text()) if path and path.exists() else {}


def fmt(value, spec=".1f") -> str:
    return "–" if value is None else format(value, spec)


def ref_dir(record: dict) -> Path:
    return (
        Path(record["session"])
        / "refs"
        / record["tree"]
        / record["config"]
        / record["program"]
    )


def correctness_table(runs: list[dict]) -> list[str]:
    rows = [
        "| gate | tree | config | program | print | net | paired | MCU ran every commanded step | capture−ref abs max µs | bound µs | MCU lateness p99 / max µs | tap = ref after offset | tap−ref abs max µs | epochs | slope ppm |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in runs:
        if not r["tap"]:
            continue
        a = analysis(r)
        checks = a.get("checks", {})
        timing = a.get("capture_vs_reference", {}).get("timing", {})
        late = a.get("capture_vs_tap", {}).get("mcu_lateness", {})
        tapref = a.get("tap_vs_reference", {}).get("identity", {})
        identical = tapref.get("steps_identical")
        rows.append(
            f"| {r['gate']} | {r['tree']} | {r['config']} | {r['program']} "
            f"| {r.get('print_state', 'FAILED')} "
            f"| {checks.get('net_steps_match')} | {checks.get('all_steps_paired')} "
            f"| {checks.get('mcu_executed_every_commanded_step')} "
            f"| {fmt(timing.get('abs_max_us'), '.2f')} | {fmt(timing.get('bound_us'), '.2f')} "
            f"| {fmt(late.get('p99_us'), '.2f')} / {fmt(late.get('max_us'), '.2f')} "
            f"| {fmt(None if identical is None else 100 * identical, '.3f')}% "
            f"| {fmt(tapref.get('abs_max_us'), '.2f')} "
            f"| {timing.get('epochs', '–')} | {fmt(timing.get('slope_ppm'), '.2f')} |"
        )
    return rows


def rates_table(runs: list[dict]) -> list[str]:
    rows = [
        "| config | program | steps | peak stepper_x | peak stepper_y | peak stepper_z | peak extruder | peak aggregate |",
        "|---|---|---|---|---|---|---|---|",
    ]
    seen = set()
    for r in runs:
        key = (r["config"], r["program"])
        if key in seen or r["tree"] != "kv2" or not ref_dir(r).exists():
            continue
        seen.add(key)
        p = peak_rates(ref_dir(r))
        rows.append(
            f"| {r['config']} | {r['program']} | {p['total_steps']} "
            + " ".join(f"| {p[n] / 1e3:.1f} k" for n in compare.CHANNELS)
            + f" | {p['aggregate'] / 1e3:.1f} k |"
        )
    return rows


def headroom_table(runs: list[dict]) -> list[str]:
    rows = [
        "| gate | config | program | tap | host | print | fatal lines | klippy CPU mean % | peak 1 s % | busiest threads (mean %) | buffer_time min / median s | stalls |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in runs:
        h = headroom(r)
        cpu, buf = h.get("cpu", {}), h.get("buffer", {})
        threads = ", ".join(
            f"{name} {pct:.0f}"
            for name, pct in list(cpu.get("threads_mean_pct", {}).items())[:3]
        )
        state = r.get("print_state") or ("FAILED: " + r.get("error", "")[:80])
        rows.append(
            f"| {r['gate']} | {r['config']} | {r['program']} | {r['tap']} "
            f"| {host_label(r['host'])} | {state} | {len(r.get('fatal_lines', []))} "
            f"| {fmt(cpu.get('process_mean_pct'))} | {fmt(cpu.get('process_peak_1s_pct'))} "
            f"| {threads or '–'} | {fmt(buf.get('buffer_time_min_s'), '.3f')} / "
            f"{fmt(buf.get('buffer_time_median_s'), '.3f')} | {buf.get('print_stall_delta', '–')} |"
        )
    return rows


def g4_table(runs: list[dict]) -> list[str]:
    rows = [
        "| program | batch messages kv2 = upstream | captured sequences kv2 = upstream | upstream tap = kv2 tap after offset | tap kv2−upstream abs max µs |",
        "|---|---|---|---|---|",
    ]
    by_key = {}
    for r in runs:
        if r["tap"] and r["config"] == "owner_like" and r.get("print_state"):
            by_key.setdefault(r["program"], {})[r["tree"]] = r
    for name, pair in sorted(by_key.items()):
        if set(pair) != {"kv2", "upstream"}:
            continue
        kv2, up = pair["kv2"], pair["upstream"]
        same_messages = (
            json.loads((ref_dir(kv2) / "reference.json").read_text())[
                "messages_sha256"
            ]
            == json.loads((ref_dir(up) / "reference.json").read_text())[
                "messages_sha256"
            ]
        )
        cap_kv2 = compare.load_capture(Path(kv2["run_dir"]))
        cap_up = compare.load_capture(Path(up["run_dir"]))
        seq = compare.sequences(cap_kv2, cap_up)
        tap_kv2 = compare.load_tap(Path(kv2["run_dir"]))
        tap_up = compare.load_tap(Path(up["run_dir"]))
        tap_seq = compare.sequences(tap_kv2, tap_up)
        identical = abs_max = None
        if tap_seq["all_steps_paired"]:
            identity = compare.schedule_identity(tap_kv2, tap_up, 84e6)
            identical = identity["steps_identical"]
            abs_max = identity["abs_max_us"]
        rows.append(
            f"| {name} | {same_messages} | {seq['all_steps_paired']} "
            f"| {fmt(None if identical is None else 100 * identical, '.3f')}% "
            f"| {fmt(abs_max, '.2f')} |"
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", type=Path, nargs="+")
    args = parser.parse_args()
    runs = load_runs([s.expanduser() for s in args.sessions])
    for title, table in (
        ("Executed vs planned (tap runs)", correctness_table(runs)),
        ("Reference peak step rates (steps/s)", rates_table(runs)),
        ("Headroom", headroom_table(runs)),
        ("Upstream vs kv2", g4_table(runs)),
    ):
        print(f"### {title}\n")
        print("\n".join(table))
        print()


if __name__ == "__main__":
    main()
