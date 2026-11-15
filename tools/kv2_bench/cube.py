"""The voron0 cube on the bench: the real printer's A/B, live, captured.

    uv run python -m tools.kv2_bench.cube plan OUT_DIR [--runs A,B,...]
    uv run python ~/Developer/kv2-bench-serval/tools/bench/lock.py hold -- \
        uv run python -m tools.kv2_bench.cube session [--runs A,B,...]

`plan` makes the offline batch plans (trapq_runner format) of every run on
the bench config and on voron0-motion.cfg, for the microstep-independence
diff. `session` prints the whole, byte-identical cube file once per run via
virtual_sdcard with the live queue_step and trapq taps and event capture,
and restores the bench in a finally block. Judging and timing are
cube_report's, offline.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import subprocess
import sys
import traceback
from datetime import datetime
from pathlib import Path

from . import bench, firmware, live, program, reference, restore
from .serval_tools import TREE as SERVAL_TREE
from .serval_tools import head as serval_head
from .serval_tools import lock

HERE = Path(__file__).resolve().parent
BASE_CONFIG = HERE / "configs" / "voron0.cfg"
NOTES = Path("~/Developer/kv2-notes/voron0").expanduser()
GCODES = NOTES / "cube-gcode"
STOCK = GCODES / "Voron_Design_Cube_v7_PLA_11m58s.gcode"
STRIPPED = GCODES / "Voron_Design_Cube_v7_PLA_11m58s_cd.gcode"
STOCK_SHA256 = (
    "aa5e978c9a4a0b9886f893da2aec3c286f1d49a7feda247d995a0b8adcd67169"
)
STRIPPED_SHA256 = (
    "a2b09edb422fe122a6861617db25d7999375fec8fb69f3f0ae3162b747e9fdd2"
)
AB = NOTES / "ab"
MOTION_CONFIG = AB / "voron0-motion.cfg"
TRAPQ_RUNNER = AB / "trapq_runner.py"
SESSIONS = Path("~/bench/kv2").expanduser()
KV2_REF = "95b409a49"
RUN_TIMEOUT_S = 1800.0
SERVAL_SLOTS = Path("~/Developer/serval/scripts/slots.py").expanduser()
MOTION_DICTIONARY = NOTES / "evidence" / "f103-ab9fa12b.dict"
UNCAPPED = 0
SCV_LINE = "square_corner_velocity: 5.0\n"
MCR_LINE = "minimum_cruise_ratio: 0.1\n"


@dataclasses.dataclass(frozen=True)
class Setting:
    gcode: Path
    cruise_ratio: float = 0.1
    corner_cap: int | None = None

    def corner_section(self) -> str | None:
        if self.corner_cap is None:
            return None
        cap = (
            ""
            if self.corner_cap == UNCAPPED
            else f"max_corner_accel: {self.corner_cap}\n"
        )
        return f"[corner_deviation]\n{cap}"

    def apply(self, config: str) -> str:
        if config.count(SCV_LINE) != 1 or config.count(MCR_LINE) != 1:
            raise ValueError("config lacks the [printer] lines a setting edits")
        config = config.replace(
            MCR_LINE, f"minimum_cruise_ratio: {self.cruise_ratio}\n"
        )
        section = self.corner_section()
        if section is None:
            return config
        return config.replace(SCV_LINE, "") + "\n" + section


SETTINGS = {
    "A": Setting(STOCK, cruise_ratio=0.2),
    "B": Setting(STOCK),
    "C": Setting(STOCK, corner_cap=21000),
    "D": Setting(STRIPPED, corner_cap=21000),
    "E": Setting(STOCK, corner_cap=24000),
    "F": Setting(STOCK, corner_cap=28000),
    "G": Setting(STOCK, corner_cap=UNCAPPED),
}


def cube_program(path: Path) -> program.Program:
    """The file as sliced, line for line: SET_VELOCITY_LIMIT, M106, M73 and
    every other line reach klippy unchanged and sd_pos is a file offset."""
    data = path.read_bytes()
    expected = STOCK_SHA256 if path == STOCK else STRIPPED_SHA256
    if bench.sha256(path) != expected:
        raise ValueError(f"{path} is not the recorded file (sha256 {expected})")
    prog = program.Program(
        path.stem, tuple(data.decode().splitlines()), (121.0, 120.0, 116.1)
    )
    if prog.text().encode() != data:
        raise ValueError(f"{path} does not round-trip through Program")
    return prog


def bench_config(name: str) -> str:
    return SETTINGS[name].apply(BASE_CONFIG.read_text())


def motion_config(name: str) -> str:
    """voron0-motion.cfg with the macros and no-ops voron0/ab/analyze.py
    adds, and the same setting."""
    sys.path.insert(0, str(AB))
    import analyze  # noqa: PLC0415

    setting = SETTINGS[name]
    noops = analyze.needed_noops(setting.gcode.read_text())
    section = setting.corner_section()
    text = analyze.config_text(
        None if section else SCV_LINE.strip(), section, noops
    )
    if text.count(MCR_LINE) != 1:
        raise ValueError(
            "voron0-motion.cfg lacks the minimum_cruise_ratio line"
        )
    return text.replace(
        MCR_LINE, f"minimum_cruise_ratio: {setting.cruise_ratio}\n"
    )


def run_trapq_runner(
    tree: Path, config: str, gcode: Path, dictionary: Path, out: Path
) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    (out / "printer.cfg").write_text(config)
    plan = out / "plan.json"
    subprocess.run(
        [
            "python3", str(SERVAL_SLOTS), "run", "--",
            sys.executable, str(TRAPQ_RUNNER), str(tree), str(plan), str(out / "printer.cfg"),
            "-i", str(gcode), "-o", str(out / "mcu.out"), "-d", str(dictionary),
            "-l", str(out / "klippy.log"),
        ],
        cwd=tree, check=True, capture_output=True,
    )  # fmt: skip
    if not plan.exists():
        raise RuntimeError(f"trapq_runner wrote no plan in {out}")
    return plan


def plan(out: Path, names: list[str], kv2: str) -> None:
    fw = firmware.build(kv2)
    tree = firmware.FW_WORKTREE
    for name in names:
        setting = SETTINGS[name]
        bench_dir = out / name / "bench"
        bench_dir.mkdir(parents=True, exist_ok=True)
        cube_program(setting.gcode)
        run_trapq_runner(
            tree,
            reference.batch_config(bench_config(name)),
            setting.gcode,
            fw.dictionary,
            bench_dir,
        )
        motion_dir = out / name / "motion"
        motion_dir.mkdir(parents=True, exist_ok=True)
        wrapped = motion_dir / "input.gcode"
        wrapped.write_text(
            "SET_KINEMATIC_POSITION X=121.0 Y=120.0 Z=116.1\n%s\nM400\n"
            % setting.gcode.read_text()
        )
        run_trapq_runner(
            tree, motion_config(name), wrapped, MOTION_DICTIONARY, motion_dir
        )
        print(name, "planned", flush=True)


def session(names: list[str], kv2_ref: str, lock_timeout: float) -> None:
    if lock.TOKEN_ENV not in os.environ:
        raise SystemExit(
            f"run under the bench lock: uv run python {SERVAL_TREE}/tools/bench/lock.py hold -- <this command>"
        )
    kv2 = bench.resolve(kv2_ref)
    out = SESSIONS / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-cube"
    out.mkdir(parents=True)
    print(f"session {out}", flush=True)
    fw = firmware.build(kv2)
    configs = out / "configs"
    configs.mkdir()
    runs = []
    for name in names:
        path = configs / f"voron0_{name}.cfg"
        path.write_text(bench_config(name))
        runs.append((name, path, cube_program(SETTINGS[name].gcode)))
    record: dict = {"kv2": kv2, "firmware": fw.record(), "runs": []}

    def save() -> None:
        (out / "session.json").write_text(json.dumps(record, indent=2))

    with lock.bench_lock(lock_timeout):
        pre = bench.snapshot_state()
        (out / "pre_state.json").write_text(json.dumps(pre, indent=2))
        (out / "pre_state_py.txt").write_text(restore.state_report())
        restore_tree = restore.prepare(pre)
        record["serval_tools_head"] = serval_head()
        failure: BaseException | None = None
        try:
            bench.install_agent()
            record["tree"] = bench.install_tree(kv2)
            bench.activate(record["tree"], runs[0][1])
            record["flashed"] = bench.flash_kv2_firmware(fw.elf, fw.version)
            bench.write_kv2_deploy_record(
                {
                    "kv2_commit": kv2,
                    "tree": record["tree"],
                    "firmware": record["flashed"],
                    "table_cache_dir": bench.PI_TABLE_CACHE,
                    "session": str(out),
                }
            )
            record["upload_identical"] = upload_identical(runs[0][2])
            save()
            for name, config, prog in runs:
                print(f"[{name}] {config.name} {prog.name}", flush=True)
                entry = {
                    "run": name,
                    "config": str(config),
                    "gcode": str(SETTINGS[name].gcode),
                }
                try:
                    run_dir = live.run(
                        prog, config, out / "runs", f"cube-{name}", tap=True, trapq=True,
                        timeout_s=RUN_TIMEOUT_S, extra={"run": name},
                    )  # fmt: skip
                    entry["run_dir"] = str(run_dir)
                except Exception as exc:
                    entry["error"] = repr(exc)
                    entry["traceback"] = traceback.format_exc()
                    print(f"  FAILED: {exc!r}", flush=True)
                record["runs"].append(entry)
                save()
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
        save()
        if failure is not None:
            raise failure
        if restoration["differences"]:
            raise SystemExit(
                f"bench not restored: {restoration['differences']}"
            )


def upload_identical(prog: program.Program) -> bool:
    """Moonraker stores the upload byte for byte (no object processing)."""
    name = "kv2bench_upload_check.gcode"
    bench.upload(name, prog.text())
    try:
        remote = bench.ssh(f"sha256sum {bench.PI_GCODES}/{name}").split()[0]
    finally:
        bench.remove_upload(name)
    if remote != STOCK_SHA256 and remote != STRIPPED_SHA256:
        raise bench.BenchError(f"uploaded cube has sha256 {remote}")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plan", "session"))
    parser.add_argument("out", type=Path, nargs="?")
    parser.add_argument("--runs", default=",".join(SETTINGS))
    parser.add_argument("--kv2-ref", default=KV2_REF)
    parser.add_argument("--lock-timeout", type=float, default=7200.0)
    args = parser.parse_args()
    names = args.runs.split(",")
    unknown = sorted(set(names) - set(SETTINGS))
    if unknown:
        raise SystemExit(f"unknown runs {unknown}")
    if args.action == "plan":
        plan(args.out, names, bench.resolve(args.kv2_ref))
    else:
        session(names, args.kv2_ref, args.lock_timeout)


if __name__ == "__main__":
    main()
