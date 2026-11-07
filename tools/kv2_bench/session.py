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
    ) -> dict:
        record = {
            "gate": gate,
            "tree": tree_name,
            "commit": commit,
            "config": config,
            "program": prog.name,
            "tap": tap,
            "host": host or {},
        }
        print(
            f"[{gate}] {tree_name} {config} {prog.name} tap={tap} {host or ''}",
            flush=True,
        )
        try:
            ref_dir = self.reference(tree_name, commit, config, prog)
            run_dir = live.run(
                prog,
                CONFIGS / f"{config}.cfg",
                self.out / "runs",
                f"{gate}-{tree_name}-{config}-{prog.name}",
                tap,
                extra={"record": record},
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
            with contextlib.suppress(Exception):
                record["headroom"] = headroom.analyze(run_dir)
            if (run_dir / "events.bin").exists():
                try:
                    analysis = compare.analyze(run_dir, ref_dir)
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


UPSTREAM_WORKTREE = Path("~/Developer/kv2-bench-upstream").expanduser()


@contextlib.contextmanager
def quota(percent: int | None) -> Iterator[None]:
    bench.set_quota(percent)
    try:
        yield
    finally:
        bench.set_quota(None)


def gate_g1_g2(s: Session, kv2: str, gates: set[str]) -> None:
    for config in ("default", "owner_like", "owner_like_cd"):
        for name in FAST_PROGRAMS:
            gate = "g1" if config == "default" and name != "arcs" else "g2"
            if gate in gates or "g2" in gates:
                s.run(gate, "kv2", kv2, config, load_program(name), tap=True)


def gate_g3(s: Session, kv2: str) -> None:
    conditions = [
        {},
        {"cpus": 1},
        {"max_mhz": 1500},
        {"cpus": 1, "max_mhz": 1500},
    ]
    for config in ("owner_like", "owner_like_cd"):
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gates", default="g1,g2,g3,g4")
    parser.add_argument("--kv2-ref", default="kv2/main")
    parser.add_argument("--upstream-ref", default=UPSTREAM)
    parser.add_argument("--lock-timeout", type=float, default=7200.0)
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
            if gates & {"g1", "g2"}:
                gate_g1_g2(session, kv2, gates)
            if "g3" in gates:
                gate_g3(session, kv2)
            if "g4" in gates:
                gate_g4(session, kv2, upstream)
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
