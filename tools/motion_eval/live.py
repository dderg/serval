"""klippy live, in real time, against Klipper's linux-process MCU.

The MCU (KLIPPER_LINUX_MCU, out/klipper.elf built from
test/configs/linuxprocess.config; kv2-notes/gate.sh builds and exports it)
runs with step_recorder.c preloaded: its GPIO chip exists only in memory,
and every step it executes is logged with its CLOCK_MONOTONIC time. klippy
runs under capture.py with an API socket; the gcode goes in as one
gcode/script, MOTION_EVAL_EXIT ends the run. The MCU runs SCHED_FIFO (its
-r flag) where RLIMIT_RTPRIO allows, as klipper-mcu does in production.

configs/owner_linux.cfg is owner.cfg with its pins on the simulated chip,
a host temperature sensor, and X/Y at 4 microsteps instead of 16: at 16 the
linux MCU process misses step deadlines ("Stepper too far in past") on the
55 000 steps/s of voron_top_layers travel when several live runs share
the box.
"""

from __future__ import annotations

import dataclasses
import json
import os
import pathlib
import re
import resource
import socket
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from klippy.stepper import MAX_STEPCOMPRESS_ERROR

from . import run

HERE = pathlib.Path(__file__).resolve().parent
LINUX_MCU = os.environ.get("KLIPPER_LINUX_MCU")
OWNER_LINUX_CFG = HERE / "configs" / "owner_linux.cfg"
GPIO_CHIP = 15
READY_TIMEOUT_S = 30.0
API_TIMEOUT_S = 600.0
CLOCK_TICK_S = "1/CLOCK_FREQ"


class LiveRunError(RuntimeError):
    pass


@dataclasses.dataclass
class LiveRun:
    capture: dict
    executed: dict
    klippy_cpu_s: float
    print_wall_s: float
    state: str
    ready_s: float
    table_ready_s: float
    status_polls: int
    status_errors: list


def linux_dictionary():
    if not LINUX_MCU:
        raise LiveRunError("KLIPPER_LINUX_MCU is not set")
    return pathlib.Path(LINUX_MCU).parent / "out" / "klipper.dict"


def recorded_steppers(config_text):
    """{stepper name: (step line, dir line)} for every [stepper_*] on the
    simulated chip."""
    out = {}
    for name, body in re.findall(
        r"^\[(stepper_\w+)\]\n(.*?)(?=^\[|\Z)", config_text, re.S | re.M
    ):
        lines = {}
        for key in ("step_pin", "dir_pin"):
            m = re.search(
                r"^%s:\s*!?gpiochip%d/gpio(\d+)" % (key, GPIO_CHIP), body, re.M
            )
            if m is None:
                raise LiveRunError(
                    "%s %s is not on gpiochip%d" % (name, key, GPIO_CHIP)
                )
            lines[key] = int(m.group(1))
        out[name] = (lines["step_pin"], lines["dir_pin"])
    return out


_recorder = None


def recorder_library():
    global _recorder
    if _recorder is None:
        lib = pathlib.Path(tempfile.mkdtemp(prefix="step_recorder")) / "rec.so"
        subprocess.run(
            ["cc", "-shared", "-fPIC", "-O2", "-Wall", "-Werror", "-o", str(lib),
             str(HERE / "step_recorder.c"), "-ldl"],
            check=True,
        )  # fmt: skip
        _recorder = lib
    return _recorder


def realtime_flag():
    wanted = os.sched_get_priority_max(os.SCHED_FIFO) // 2
    soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
    return ["-r"] if soft >= wanted else []


def _wait_for(predicate, what, timeout=READY_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise LiveRunError("timed out waiting for " + what)
        time.sleep(0.02)


class _Api:
    def __init__(self, path):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(str(path))
        self.sock.settimeout(API_TIMEOUT_S)
        self.buf = b""
        self.next_id = 0

    def call(self, method, **params):
        self.next_id += 1
        req = {"id": self.next_id, "method": method, "params": params}
        self.sock.sendall(json.dumps(req).encode() + b"\x03")
        while True:
            while b"\x03" not in self.buf:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError("klippy closed the API socket")
                self.buf += chunk
            msg, self.buf = self.buf.split(b"\x03", 1)
            resp = json.loads(msg)
            if resp.get("id") == req["id"]:
                if "error" in resp:
                    raise LiveRunError(
                        "%s failed: %r" % (method, resp["error"])
                    )
                return resp["result"]


def _cpu_seconds(pid):
    fields = (
        pathlib.Path("/proc/%d/stat" % pid)
        .read_text()
        .rsplit(")", 1)[1]
        .split()
    )
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def _read_steps(path, names):
    rec = np.frombuffer(
        path.read_bytes(),
        dtype=[("ns", "<i8"), ("stepper", "<i4"), ("dir", "<i4")],
    )
    return {
        name: {"ns": rec["ns"][rec["stepper"] == i].copy(),
               "dir": rec["dir"][rec["stepper"] == i].copy()}
        for i, name in enumerate(names)
    }  # fmt: skip


def run_live(
    config_path,
    gcode,
    start_xyz,
    cpu=None,
    quota=None,
    env=None,
    wait_table=True,
):
    """Plan and execute gcode in real time. cpu: pin klippy to that core.
    quota: CPU percent klippy may use, enforced by the kernel in
    QUOTA_PERIOD slices (a slower host, without the long gaps of the
    default 100 ms period). wait_table: start the print only once the
    corner_deviation table is ready."""
    if not LINUX_MCU:
        raise LiveRunError("KLIPPER_LINUX_MCU is not set")
    steppers = recorded_steppers(_expanded(config_path))
    with tempfile.TemporaryDirectory(prefix="mel") as tmp:
        tmp = pathlib.Path(tmp)
        tty, api_path, log = tmp / "mcu", tmp / "api", tmp / "klippy.log"
        steps_path, capture_path = tmp / "steps.bin", tmp / "capture.json"
        mcu = subprocess.Popen(
            [LINUX_MCU, "-I", str(tty)] + realtime_flag(),
            env=dict(
                os.environ,
                LD_PRELOAD=str(recorder_library()),
                KV2_STEP_REC_CHIP=str(GPIO_CHIP),
                KV2_STEP_REC_LINES=",".join(
                    "%d:%d" % p for p in steppers.values()
                ),
                KV2_STEP_REC_OUT=str(steps_path),
            ),
        )
        klippy = None
        try:
            _wait_for(tty.exists, "the linux MCU tty")
            printer_cfg = tmp / "printer.cfg"
            printer_cfg.write_text(
                "[include %s]\n\n[mcu]\nserial: %s\n" % (config_path, tty)
            )
            cmd = [sys.executable, "-m", "tools.motion_eval.capture",
                   str(capture_path), str(printer_cfg), "-l", str(log),
                   "-a", str(api_path), "-I", str(tmp / "printer")]  # fmt: skip
            if cpu is not None:
                cmd = ["taskset", "-c", str(cpu)] + cmd
            if quota is not None:
                cmd = ["systemd-run", "--user", "--scope", "-q",
                       "-p", "CPUQuota=%d%%" % (quota,),
                       "-p", "CPUQuotaPeriodSec=%s" % (QUOTA_PERIOD,)] + cmd  # fmt: skip
            launched = time.monotonic()
            klippy = subprocess.Popen(
                cmd,
                cwd=run.REPO_ROOT,
                env=dict(
                    os.environ, PYTHONPATH=str(run.REPO_ROOT), **(env or {})
                ),
            )
            _wait_for(api_path.exists, "the klippy API socket")
            api = _Api(api_path)
            _wait_for(
                lambda: (
                    api.call("info")["state"] in ("ready", "shutdown", "error")
                ),
                "klippy to start",
            )
            info = api.call("info")
            ready_s = time.monotonic() - launched
            table_ready_s = 0.0
            if info["state"] != "ready":
                raise LiveRunError(
                    "klippy %s: %s" % (info["state"], info["state_message"])
                )
            if (
                wait_table
                and "corner_deviation" in api.call("objects/list")["objects"]
            ):
                _wait_for(
                    lambda: api.call(
                        "objects/query", objects={"corner_deviation": None}
                    )["status"]["corner_deviation"]["table_ready"],
                    "the corner_deviation table",
                )
            table_ready_s = time.monotonic() - launched - ready_s
            poller = _StatusPoller(api_path)
            cpu0, wall0 = _cpu_seconds(klippy.pid), time.monotonic()
            poller.start()
            api.call("gcode/script", script=run.wrap_gcode(gcode, start_xyz))
            cpu_s, wall_s = (
                _cpu_seconds(klippy.pid) - cpu0,
                time.monotonic() - wall0,
            )
            time.sleep(STATS_PERIOD_S)
            poller.stop()
            state = api.call("info")["state"]
            try:
                api.call("gcode/script", script="MOTION_EVAL_EXIT")
            except ConnectionError:
                pass
            klippy.wait(READY_TIMEOUT_S)
            if klippy.returncode or not capture_path.exists():
                raise LiveRunError(
                    "klippy exit %r\n%s"
                    % (klippy.returncode, log.read_text()[-6000:])
                )
        finally:
            for proc in (klippy, mcu):
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    proc.wait(10)
        return LiveRun(
            capture=json.loads(capture_path.read_text()),
            executed=_read_steps(steps_path, list(steppers)),
            klippy_cpu_s=cpu_s,
            print_wall_s=wall_s,
            state=state,
            ready_s=ready_s,
            table_ready_s=table_ready_s,
            status_polls=poller.polls,
            status_errors=poller.errors,
        )


STATS_PERIOD_S = 1.2
QUOTA_PERIOD = "5ms"
POLL_PERIOD_S = 0.25


class _StatusPoller(threading.Thread):
    """Queries every printer object's status over a second API connection
    while the print runs, as Moonraker does; the run waits one
    statistics.py period after the print so its timer fires on the
    finished state too."""

    def __init__(self, api_path):
        super().__init__(daemon=True)
        self.api = _Api(api_path)
        self.objects = self.api.call("objects/list")["objects"]
        self.polls = 0
        self.errors = []
        self.done = threading.Event()

    def run(self):
        while not self.done.wait(POLL_PERIOD_S):
            try:
                status = self.api.call(
                    "objects/query", objects={o: None for o in self.objects}
                )["status"]
                json.dumps(status)
                self.polls += 1
            except Exception as e:
                self.errors.append("%s: %s" % (type(e).__name__, e))

    def stop(self):
        self.done.set()
        self.join(API_TIMEOUT_S)


def _expanded(config_path):
    text = pathlib.Path(config_path).read_text()
    for inc in re.findall(r"^\[include (.+)\]", text, re.M):
        path = (pathlib.Path(config_path).parent / inc.strip()).resolve()
        text += "\n" + _expanded(path)
    return text


def commanded_steps(stepper):
    """Step clocks and positions from a capture's stepcompress history (the
    count is signed by direction)."""
    clocks, positions, position = [], [], 0
    for first, signed_count, interval, add in stepper["step_history"]:
        count, direction = abs(signed_count), (1 if signed_count > 0 else -1)
        k = np.arange(count, dtype=np.int64)
        clocks.append(first + k * interval + add * k * (k + 1) // 2)
        positions.append(position + direction * (k + 1))
        position += direction * count
    if not clocks:
        empty = np.zeros(0, dtype=np.int64)
        return empty, empty
    return np.concatenate(clocks), np.concatenate(positions)


def compare_streams(batch, live):
    """Per stepper: identical step positions in order, and step times equal
    within two stepcompress bounds (each stream lies within one of the
    planned step time) after the best affine map between the two clocks."""
    out = {}
    for name, b in batch["steppers"].items():
        lv = live["steppers"][name]
        bc, bp = commanded_steps(b)
        lc, lp = commanded_steps(lv)
        same_positions = len(bp) == len(lp) and bool(np.array_equal(bp, lp))
        entry = {"steps": int(len(bp)), "live_steps": int(len(lp)),
                 "same_positions": same_positions}  # fmt: skip
        if same_positions and len(bc) > 1:
            tb = bc / b["clock_freq"]
            tl = lc / lv["clock_freq"]
            scale, offset = np.polyfit(tb, tl, 1)
            resid = tl - (scale * tb + offset)
            entry.update(
                clock_scale_ppm=float((scale - 1.0) * 1e6),
                max_time_error_s=float(np.abs(resid).max()),
                allowed_s=2.0 * MAX_STEPCOMPRESS_ERROR + 2.0 / b["clock_freq"],
            )
        out[name] = entry
    return out


def compare_execution(capture, executed):
    """Steps the MCU executed (GPIO edges) against the commanded stream:
    counts and net position exactly, and the timing jitter."""
    out = {}
    for name, ex in executed.items():
        st = capture["steppers"][name]
        clocks, positions = commanded_steps(st)
        sign = np.where(ex["dir"] == (0 if st["invert_dir"] else 1), 1, -1)
        entry = {"commanded": int(len(clocks)), "executed": int(len(ex["ns"])),
                 "commanded_net": int(positions[-1]) if len(positions) else 0,
                 "executed_net": int(sign.sum())}  # fmt: skip
        if len(clocks) == len(ex["ns"]) and len(clocks) > 1:
            t_cmd = clocks / st["clock_freq"]
            t_ex = ex["ns"] * 1e-9
            scale, offset = np.polyfit(t_cmd, t_ex, 1)
            late = t_ex - (scale * t_cmd + offset)
            late -= late.min()
            entry.update(jitter_p99_us=float(np.percentile(late, 99) * 1e6),
                         jitter_max_us=float(late.max() * 1e6))  # fmt: skip
        out[name] = entry
    return out


def compare_trapq(batch_moves, live_moves):
    """Moves equal in every field, print times by one constant offset."""
    if len(batch_moves) != len(live_moves):
        return {
            "same": False,
            "moves": len(batch_moves),
            "live_moves": len(live_moves),
        }
    offsets = np.array(
        [
            lm["print_time"] - bm["print_time"]
            for bm, lm in zip(batch_moves, live_moves)
        ]
    )
    fields_equal = all(
        {k: v for k, v in bm.items() if k != "print_time"}
        == {k: v for k, v in lm.items() if k != "print_time"}
        for bm, lm in zip(batch_moves, live_moves)
    )
    spread = float(offsets.max() - offsets.min())
    return {"same": fields_equal and spread <= 1e-9 * max(1.0, abs(offsets).max()),
            "moves": len(batch_moves), "offset_spread_s": spread, "fields_equal": fields_equal}  # fmt: skip


PI5_SLOWDOWN = 2.5


def cpu_report(case, repeat, core, quota=None, cold=False, wait_table=True):
    """klippy at real time on one pinned core (and a CPU quota): the case
    printed repeat times (returning to its start in between), without and
    with [corner_deviation]; cold builds the corner table at startup."""
    from . import sweep

    text = (sweep.CORPUS / (case + ".gcode")).read_text()
    start = sweep.case_start(text)
    back = "G1 X%r Y%r Z%r F30000\n" % tuple(start)
    gcode = (text + back) * repeat
    rows, details = [], []
    for limiter in ("default", "corner_deviation"):
        with tempfile.TemporaryDirectory() as tmp:
            section = ""
            if limiter == "corner_deviation":
                section = "[corner_deviation]\n"
                if cold:
                    section += "table_cache_dir: %s\n" % (tmp,)
            config = pathlib.Path(tmp) / "printer.cfg"
            config.write_text("[include %s]\n\n%s" % (OWNER_LINUX_CFG, section))
            try:
                r = run_live(
                    config,
                    gcode,
                    start,
                    cpu=core,
                    quota=quota,
                    wait_table=wait_table,
                )
            except LiveRunError as e:
                rows.append(
                    "| %s | %s | failed: %s |"
                    % (limiter, case, str(e)[:160].replace("\n", " "))
                )
                continue
        moves = r.capture["moves"]
        span = (
            moves[-1]["print_time"]
            + moves[-1]["accel_t"]
            + moves[-1]["cruise_t"]
            + moves[-1]["decel_t"]
            - moves[0]["print_time"]
        )
        timing = r.capture["timing"]
        reactor = r.capture["reactor"]
        n = r.capture["added_moves"]
        share = r.klippy_cpu_s / r.print_wall_s
        work = r.capture["flush_work"]
        status = r.capture["corner_deviation"]
        if status is not None:
            details.append(
                "%s table built until %.2f s after ready, worst late while"
                " building %s; admitted corner accel X %.0f Y %.0f"
                % (limiter, reactor["building_until_s"],
                   _ms(reactor["worst_building"][:3]),
                   status["corner_accel_x"], status["corner_accel_y"])
            )  # fmt: skip
        details.append(
            "%s worst [s after ready, cpu ms, late ms]: cpu %s late %s;"
            " lookahead flushes %d, max queued %d, max flush %.1f ms"
            % (limiter, _ms(reactor["worst_cpu"][:4]),
               _ms(reactor["worst_late"][:4]), len(work),
               max(w[0] for w in work), 1e3 * max(w[2] for w in work))
        )  # fmt: skip
        rows.append(
            "| %s | %s | %s | %.2f | %.2f | %d | %.1f | %.1f | %.1f%% | %.1f | %.1f |"
            % (limiter, case, r.state, r.ready_s, r.table_ready_s, n / span,
               1e6 * timing["lookahead_s"] / n,
               1e6 * timing["junction_limiter_s"] / n, 100.0 * share,
               1e3 * reactor["max_cpu_s"], 1e3 * reactor["max_late_s"])
        )  # fmt: skip
    head = (
        "| limiter | case | state | to ready s | table after ready s |"
        " moves/s of print | lookahead us/move | of which limiter |"
        " klippy CPU | longest callback ms (CPU) | worst timer lateness ms |"
        "\n|%s" % ("---|" * 11,)
    )
    return head + "\n" + "\n".join(rows + [""] + details)


def _ms(samples):
    return [
        [round(t, 2), round(1e3 * cpu, 1), round(1e3 * late, 1)]
        for t, cpu, late in samples
    ]


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(prog="python -m tools.motion_eval.live")
    ap.add_argument("--case", default="arcs")
    ap.add_argument("--repeat", type=int, default=10)
    ap.add_argument("--core", type=int, default=os.cpu_count() - 1)
    ap.add_argument("--quota", type=int)
    ap.add_argument("--cold", action="store_true")
    ap.add_argument("--no-table-wait", action="store_true")
    args = ap.parse_args(argv)
    print(
        cpu_report(
            args.case,
            args.repeat,
            args.core,
            args.quota,
            args.cold,
            not args.no_table_wait,
        )
    )


if __name__ == "__main__":
    main()
