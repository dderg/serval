"""One bench session for kv2: record, install, flash, run the gates, restore.

    uv run python ~/Developer/kv2-bench-serval/tools/bench/lock.py hold -- \
        uv run python -m tools.kv2_bench.session [--gates g1,g2,g3,g4]

Run from the kv2-bench tree. Writes ~/bench/kv2/<stamp>/session.json (one
record per run: gate, tree, config, program, host limits, print result,
analysis.json checks and headroom.json) and restoration.json. The bench is
restored in a finally block, also after a failed run.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import traceback
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path

from . import (
    bench,
    compare,
    firmware,
    headroom,
    live,
    program,
    reference,
    replay,
    restore,
)
from .serval_tools import TREE as SERVAL_TREE
from .serval_tools import head as serval_head
from .serval_tools import lock, throttle

HERE = Path(__file__).resolve().parent
KV2_ROOT = HERE.parents[1]
CONFIGS = HERE / "configs"
CORPUS = KV2_ROOT / "tools" / "motion_eval" / "corpus"
SPEED_LADDER = SERVAL_TREE / "tools" / "bench" / "gcode" / "speed_ladder.gcode"
SESSIONS = Path("~/bench/kv2").expanduser()
UPSTREAM = "1ee39ff2c"
FAST_PROGRAMS = ("speed_ladder", "voron_top_layers", "arcs")
DENSE = {"arcs": 30, "voron_top_layers": 15}
SOAK_SLICES = (
    "voron_top_layers",
    "arcs",
    "circles",
    "zigzag_infill",
    "corners_turn90",
)
SOAK_SECONDS = 1800.0


def load_program(name: str, repeat: int = 1) -> program.Program:
    path = SPEED_LADDER if name == "speed_ladder" else CORPUS / f"{name}.gcode"
    loaded = program.load(path, repeat)
    return loaded


class Session:
    def __init__(self, out: Path, fw: firmware.Firmware):
        self.out = out
        self.fw = fw
        self.records: list[dict] = []
        self.trees: dict[str, str] = {}

    def save(self) -> None:
        (self.out / "session.json").write_text(
            json.dumps(
                {
                    "firmware": self.fw.record(),
                    "trees": self.trees,
                    "runs": self.records,
                },
                indent=2,
            )
        )

    def reference(self, tree_name: str, commit: str, config: str, prog) -> Path:
        ref_dir = self.out / "refs" / tree_name / config / prog.name
        if not (ref_dir / "reference.json").exists():
            worktree = (
                firmware.FW_WORKTREE
                if tree_name == "kv2"
                else UPSTREAM_WORKTREE
            )
            firmware.checkout(worktree, commit)
            reference.build(
                worktree,
                CONFIGS / f"{config}.cfg",
                prog,
                self.fw.dictionary,
                ref_dir,
            )
        return ref_dir

    def run(
        self,
        gate: str,
        tree_name: str,
        commit: str,
        config: str,
        prog: program.Program,
        tap: bool,
        host: dict | None = None,
        judge: str = "batch",
        **options,
    ) -> dict:
        record = {
            "gate": gate,
            "tree": tree_name,
            "commit": commit,
            "config": config,
            "program": prog.name,
            "tap": tap,
            "host": host or {},
            "options": options,
            "judge": judge,
        }
        print(
            f"[{gate}] {tree_name} {config} {prog.name} tap={tap} {host or ''}",
            flush=True,
        )
        try:
            ref_dir = (
                self.reference(tree_name, commit, config, prog)
                if judge == "batch"
                else None
            )
            run_dir = live.run(
                prog,
                CONFIGS / f"{config}.cfg",
                self.out / "runs",
                f"{gate}-{tree_name}-{config}-{prog.name}",
                tap,
                extra={"record": record},
                **options,
            )
            record["run_dir"] = str(run_dir)
            record["print_state"] = "complete"
        except Exception as exc:
            record["error"] = f"{exc!r}"
            record["traceback"] = traceback.format_exc()
            run_dirs = sorted(
                (self.out / "runs").glob(
                    f"*-{gate}-{tree_name}-{config}-{prog.name}"
                )
            )
            run_dir = run_dirs[-1] if run_dirs else None
            record["run_dir"] = str(run_dir) if run_dir else None
            print(f"  FAILED: {exc!r}", flush=True)
        if record.get("run_dir"):
            run_dir = Path(record["run_dir"])
            log = run_dir / "klippy.log"
            if log.exists():
                record["fatal_lines"] = headroom.fatal_lines(log.read_text())
            try:
                record["headroom"] = headroom.analyze(run_dir)
            except Exception as exc:
                record["headroom_error"] = repr(exc)
            if (run_dir / "events.bin").exists():
                try:
                    if judge == "batch":
                        analysis = compare.analyze(run_dir, ref_dir)
                    else:
                        analysis = self.replay_judgement(
                            tree_name, commit, run_dir
                        )
                    record["analysis_ok"] = analysis["ok"]
                    record["checks"] = analysis["checks"]
                except Exception as exc:
                    record["analysis_error"] = repr(exc)
        self.records.append(record)
        self.save()
        print(
            f"  -> {record.get('print_state', 'FAILED')} analysis_ok={record.get('analysis_ok')} "
            f"{record.get('checks', '')}",
            flush=True,
        )
        return record

    def replay_judgement(
        self, tree_name: str, commit: str, run_dir: Path
    ) -> dict:
        worktree = (
            firmware.FW_WORKTREE if tree_name == "kv2" else UPSTREAM_WORKTREE
        )
        firmware.checkout(worktree, commit)
        reference = replay.build(
            run_dir, worktree, self.fw.dictionary, run_dir / "replay"
        )
        result = replay.judge(run_dir, reference)
        (run_dir / "replay" / "replay_judgement.json").write_text(
            json.dumps(result, indent=2)
        )
        return result


UPSTREAM_WORKTREE = Path("~/Developer/kv2-bench-upstream").expanduser()


@contextlib.contextmanager
def quota(percent: int | None) -> Iterator[None]:
    bench.set_quota(percent)
    try:
        yield
    finally:
        bench.set_quota(None)


def gate_g1_g2(
    s: Session, kv2: str, gates: set[str], configs: set[str]
) -> None:
    for config in ("default", "owner_like", "owner_like_cd"):
        if config not in configs:
            continue
        for name in FAST_PROGRAMS:
            gate = "g1" if config == "default" and name != "arcs" else "g2"
            if gate in gates or "g2" in gates:
                s.run(gate, "kv2", kv2, config, load_program(name), tap=True)
        if config != "default" and "g2" in gates:
            dense = load_program("voron_top_layers", DENSE["voron_top_layers"])
            s.run(
                "g2", "kv2", kv2, config, dense, tap=True, judge="replay",
                trapq=True,
            )  # fmt: skip


def gate_g3(s: Session, kv2: str, configs: set[str]) -> None:
    conditions = [
        {},
        {"cpus": 1},
        {"max_mhz": 1500},
        {"cpus": 1, "max_mhz": 1500},
    ]
    for config in ("owner_like", "owner_like_cd"):
        if config not in configs:
            continue
        for name, repeat in DENSE.items():
            prog = load_program(name, repeat)
            for host in conditions:
                with throttle.throttled(
                    throttle.Throttle(host.get("cpus"), host.get("max_mhz"))
                ):
                    s.run("g3", "kv2", kv2, config, prog, tap=False, host=host)
            with throttle.throttled(throttle.Throttle(1, 1500)):
                for percent in (50, 25, 12):
                    with quota(percent):
                        record = s.run(
                            "g3",
                            "kv2",
                            kv2,
                            config,
                            prog,
                            tap=False,
                            host={
                                "cpus": 1,
                                "max_mhz": 1500,
                                "quota_pct": percent,
                            },
                        )
                    if "error" in record:
                        break


def gate_g4(s: Session, kv2: str, upstream: str) -> None:
    remote = bench.install_tree(upstream)
    s.trees["upstream"] = remote
    bench.activate(remote, CONFIGS / "owner_like.cfg")
    bench.restart_ready()
    for name in FAST_PROGRAMS:
        s.run(
            "g4",
            "upstream",
            upstream,
            "owner_like",
            load_program(name),
            tap=True,
        )
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg")
    bench.restart_ready()


def soak_program(s: Session, kv2: str, config: str) -> program.Program:
    """SOAK_SLICES concatenated for SOAK_SECONDS of motion at `config`, the
    cycle count taken from one cycle's batch plan."""
    paths = [CORPUS / f"{name}.gcode" for name in SOAK_SLICES]
    cycle = program.concat(paths, 1, "soak")
    firmware.checkout(firmware.FW_WORKTREE, kv2)
    planned = reference.build(
        firmware.FW_WORKTREE,
        CONFIGS / f"{config}.cfg",
        cycle,
        s.fw.dictionary,
        s.out / "refs" / "soak_cycle" / config,
    )
    clocks = [st.clocks for st in planned.steppers.values() if len(st.clocks)]
    first = min(int(c[0]) for c in clocks)
    last = max(int(c[-1]) for c in clocks)
    seconds = (last - first) / planned.clock_freq
    return program.concat(paths, math.ceil(SOAK_SECONDS / seconds), "soak")


def gate_soak(s: Session, kv2: str, configs: set[str]) -> None:
    """30 min of dense corpus slices per config and host, event capture,
    judged against the live-trapq replay, with the timer-lateness probe."""
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg", probe=True)
    bench.restart_ready()
    for config in ("owner_like", "owner_like_cd"):
        if config not in configs:
            continue
        prog = soak_program(s, kv2, config)
        for host in ({}, {"cpus": 1, "max_mhz": 1500}):
            with throttle.throttled(
                throttle.Throttle(host.get("cpus"), host.get("max_mhz"))
            ):
                s.run(
                    "soak", "kv2", kv2, config, prog, tap=True, host=host,
                    judge="replay", trapq=True, probe=True,
                    timeout_s=2 * SOAK_SECONDS,
                )  # fmt: skip
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg")
    bench.restart_ready()


TRIALS = 10
TRIAL_CONFIGS = ("owner_like_cd", "owner_like")


def gate_trials(s: Session, kv2: str, trials: int) -> None:
    """arcs_x30 at 1 cpu / 1500 MHz, `trials` runs per config, interleaved,
    with the stall-catching probe (stacks and py-spy dumps of stalls)."""
    bench.ssh(f"{bench.PI_ENV}/bin/pip install -q py-spy")
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg", probe=True)
    bench.restart_ready()
    arcs = load_program("arcs", DENSE["arcs"])
    with throttle.throttled(throttle.Throttle(1, 1500)):
        cold_done = False
        for trial in range(trials):
            for config in TRIAL_CONFIGS:
                host = {"cpus": 1, "max_mhz": 1500, "trial": trial}
                if config == "owner_like_cd":
                    host["table_cache"] = "warm" if cold_done else "cold"
                    if not cold_done:
                        host["cleared_cache_files"] = bench.clear_table_cache()
                        cold_done = True
                s.run(
                    "trials", "kv2", kv2, config, arcs, tap=False, host=host,
                    probe=True,
                )  # fmt: skip
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg")
    bench.restart_ready()


CLOCK_TRIAL_KINDS = (
    ("owner_like_cd", True),
    ("owner_like", True),
    ("owner_like", False),
)
CLOCK_TRIAL_WATCH_S = 15.0


def gate_clocktrials(s: Session, kv2: str, trials: int) -> None:
    """Startup + 15 s, at 1 cpu / 1500 MHz, `trials` rounds of: owner_like_cd
    printing arcs_x15, owner_like printing it, owner_like idle. The probe
    records every clocksync sample; the outcome is klippy's state."""
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg", probe=True)
    bench.restart_ready()
    arcs = load_program("arcs", 15)
    with throttle.throttled(throttle.Throttle(1, 1500)):
        for trial in range(trials):
            for config, printing in CLOCK_TRIAL_KINDS:
                kind = f"{config}-{'print' if printing else 'idle'}"
                print(f"[clocktrials] {trial} {kind}", flush=True)
                record = live.startup_trial(
                    arcs if printing else None,
                    CONFIGS / f"{config}.cfg",
                    s.out / "runs",
                    f"clock-{kind}",
                    CLOCK_TRIAL_WATCH_S,
                    {"trial": trial, "kind": kind, "commit": kv2},
                )
                record["gate"] = "clocktrials"
                s.records.append(record)
                s.save()
                print(f"  -> {record['failure']}", flush=True)
    bench.activate(s.trees["kv2"], CONFIGS / "owner_like.cfg")
    bench.restart_ready()


def gate_investigate(s: Session, kv2: str, upstream: str) -> None:
    """Live-vs-batch trapq on the dense x15 case (upstream and kv2), and a
    probed, py-spy profiled arcs_x30 with and without corner_deviation."""
    bench.ssh(f"{bench.PI_ENV}/bin/pip install -q py-spy")
    owner = CONFIGS / "owner_like.cfg"
    dense = load_program("voron_top_layers", DENSE["voron_top_layers"])
    s.trees["upstream"] = bench.install_tree(upstream)
    bench.activate(s.trees["upstream"], owner)
    bench.restart_ready()
    s.run(
        "inv", "upstream", upstream, "owner_like", dense, tap=True, trapq=True
    )
    bench.activate(s.trees["kv2"], owner, probe=True)
    bench.restart_ready()
    s.run(
        "inv", "kv2", kv2, "owner_like", dense, tap=True, trapq=True, probe=True
    )
    arcs = load_program("arcs", DENSE["arcs"])
    slow = {"cpus": 1, "max_mhz": 1500}
    for config in ("owner_like_cd", "owner_like"):
        for host in ({}, slow):
            with throttle.throttled(
                throttle.Throttle(host.get("cpus"), host.get("max_mhz"))
            ):
                s.run(
                    "inv",
                    "kv2",
                    kv2,
                    config,
                    arcs,
                    tap=False,
                    host=host,
                    profile=True,
                    probe=True,
                )
    with throttle.throttled(throttle.Throttle(1, 1500)), quota(50):
        s.run(
            "inv",
            "kv2",
            kv2,
            "owner_like_cd",
            arcs,
            tap=False,
            host={**slow, "quota_pct": 50},
            profile=True,
            probe=True,
        )
    bench.activate(s.trees["kv2"], owner)
    bench.restart_ready()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gates", default="g1,g2,g3,g4")
    parser.add_argument(
        "--configs",
        default="default,owner_like,owner_like_cd",
        help="configs the g1-g3 rows run with",
    )
    parser.add_argument(
        "--g3-configs", help="configs of the g3 rows (default: --configs)"
    )
    parser.add_argument("--kv2-ref", default="kv2/main")
    parser.add_argument("--upstream-ref", default=UPSTREAM)
    parser.add_argument("--lock-timeout", type=float, default=7200.0)
    parser.add_argument("--trials", type=int, default=TRIALS)
    args = parser.parse_args()
    if lock.TOKEN_ENV not in os.environ:
        raise SystemExit(
            "run under the bench lock: uv run python "
            f"{SERVAL_TREE}/tools/bench/lock.py hold -- <this command>"
        )
    gates = set(args.gates.split(","))
    kv2 = bench.resolve(args.kv2_ref)
    upstream = bench.resolve(args.upstream_ref)
    out = SESSIONS / datetime.now().strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True)
    print(f"session {out}", flush=True)
    fw = firmware.build(kv2)
    with lock.bench_lock(args.lock_timeout):
        pre = bench.snapshot_state()
        (out / "pre_state.json").write_text(json.dumps(pre, indent=2))
        (out / "pre_state_py.txt").write_text(restore.state_report())
        restore_tree = restore.prepare(pre)
        session = Session(out, fw)
        session.trees["serval_tools_head"] = serval_head()
        failure: BaseException | None = None
        try:
            bench.install_agent()
            session.trees["kv2"] = bench.install_tree(kv2)
            bench.activate(session.trees["kv2"], CONFIGS / "default.cfg")
            session.trees["firmware"] = bench.flash_kv2_firmware(
                fw.elf, fw.version
            )
            session.save()
            bench.write_kv2_deploy_record(
                {
                    "kv2_commit": kv2,
                    "tree": session.trees["kv2"],
                    "firmware": session.trees["firmware"],
                    "table_cache_dir": bench.PI_TABLE_CACHE,
                    "session": str(out),
                }
            )
            session.trees["table_cache_dir"] = bench.PI_TABLE_CACHE
            configs = set(args.configs.split(","))
            if gates & {"g1", "g2"}:
                gate_g1_g2(session, kv2, gates, configs)
            if "g3" in gates:
                gate_g3(
                    session,
                    kv2,
                    set((args.g3_configs or args.configs).split(",")),
                )
            if "g4" in gates:
                gate_g4(session, kv2, upstream)
            if "inv" in gates:
                gate_investigate(session, kv2, upstream)
            if "trials" in gates:
                gate_trials(session, kv2, args.trials)
            if "clocktrials" in gates:
                gate_clocktrials(session, kv2, args.trials)
            if "soak" in gates:
                gate_soak(session, kv2, set(args.configs.split(",")))
        except BaseException as exc:
            failure = exc
            traceback.print_exc()
        finally:
            restoration = restore.restore(
                pre, restore_tree, out / "restore.log"
            )
            (out / "restoration.json").write_text(
                json.dumps(restoration, indent=2)
            )
            print(restoration["state_py"], flush=True)
            print(
                f"restoration differences: {restoration['differences']}",
                flush=True,
            )
        session.save()
        if failure is not None:
            raise failure
        if restoration["differences"]:
            raise SystemExit(
                f"bench not restored: {restoration['differences']}"
            )


if __name__ == "__main__":
    main()
