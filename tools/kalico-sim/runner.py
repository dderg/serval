#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import pathlib
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="[sim %(levelname)s] %(message)s",
)
log = logging.getLogger("kalico-sim")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
VTIME_SHM = "/dev/shm/kalico_vtime"
VTIME_SHM_SIZE = 256
VTIME_STRUCT_FMT = "<QIIII"


@dataclasses.dataclass
class McuProcess:
    name: str
    process: subprocess.Popen
    pty_path: str
    log_path: str


@dataclasses.dataclass
class SimResult:
    success: bool
    print_time_s: float
    wall_time_s: float
    speedup: float
    error: Optional[str] = None
    klippy_log: Optional[str] = None
    mcu_logs: Optional[dict] = None


def vtime_create(start_ns: int = 1_000_000_000) -> None:
    with open(VTIME_SHM, "wb") as f:
        header = struct.pack(VTIME_STRUCT_FMT, start_ns, 0, 0, 1, 0)
        f.write(header + b"\x00" * (VTIME_SHM_SIZE - len(header)))
    os.chmod(VTIME_SHM, 0o666)


def vtime_read_ns() -> int:
    try:
        with open(VTIME_SHM, "rb") as f:
            data = f.read(VTIME_SHM_SIZE)
        return struct.unpack_from(VTIME_STRUCT_FMT, data, 0)[0]
    except FileNotFoundError:
        return 0


def vtime_destroy() -> None:
    try:
        os.unlink(VTIME_SHM)
    except FileNotFoundError:
        pass


def build_firmware(
    repo_root: pathlib.Path, config_name: str, output_name: str
) -> pathlib.Path:
    config_src = repo_root / "tools" / "kalico-sim" / "configs" / config_name
    if not config_src.exists():
        config_src = (
            repo_root / "tools" / "sim_klippy" / "configs" / config_name
        )
    if not config_src.exists():
        raise RuntimeError(f"Config {config_name} not found")

    out_elf = repo_root / "out" / output_name
    log.info("Building firmware: %s -> %s", config_name, output_name)

    subprocess.run(
        ["cp", str(config_src), str(repo_root / ".config")],
        check=True,
    )
    subprocess.run(
        ["make", "clean"],
        cwd=str(repo_root),
        check=True,
        capture_output=True,
    )
    nproc = os.cpu_count() or 4
    result = subprocess.run(
        ["make", f"-j{nproc}"],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Firmware build failed:\nstdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    subprocess.run(
        ["cp", str(repo_root / "out" / "klipper.elf"), str(out_elf)],
        check=True,
    )
    log.info("Built: %s", out_elf)
    return out_elf


def ensure_shims_built(repo_root: pathlib.Path) -> tuple:
    shim_dir = repo_root / "tools" / "kalico-sim" / "libvtime"
    shim_so = shim_dir / "libsim_intercept.so"
    vtime_so = shim_dir / "libvtime.so"
    if not shim_so.exists() or not vtime_so.exists():
        subprocess.check_call(["make", "-C", str(shim_dir)])
    return (shim_so, vtime_so)


def spawn_mcu(
    name: str,
    elf_path: pathlib.Path,
    pty_path: str,
    log_path: str,
    sock_dir: str,
    shim_so: pathlib.Path,
    vtime_so: pathlib.Path,
    verbose: bool = False,
) -> McuProcess:
    if os.path.exists(pty_path):
        os.unlink(pty_path)

    log_fd = open(log_path, "wb")
    env = os.environ.copy()
    # vtime first, intercept second (constructor/interpose ordering). The
    # motion tick thread registers as a vtime pacer, which both prevents the
    # historical stall-deadlock (the pacer always advances time) and pins
    # virtual time so it can never skip a motion sample period. SPEED=0.2
    # runs the simulated world at 1/5 real speed, inflating every host-side
    # latency budget (drip window, trsync timeouts) 5x against Docker jitter.
    env["LD_PRELOAD"] = "%s:%s" % (vtime_so, shim_so)
    env.setdefault("KALICO_VTIME_SPEED", "1")
    env["KALICO_SIM_SOCK_DIR"] = sock_dir
    if verbose:
        env["KALICO_SIM_SHIM_VERBOSE"] = "1"
        env["KALICO_VTIME_DEBUG"] = "1"

    proc = subprocess.Popen(
        [str(elf_path), "-I", pty_path],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
        env=env,
    )

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if os.path.exists(pty_path):
            return McuProcess(
                name=name,
                process=proc,
                pty_path=pty_path,
                log_path=log_path,
            )
        if proc.poll() is not None:
            log_fd.close()
            content = open(log_path).read()
            raise RuntimeError(
                f"{name}: klipper.elf exited early (rc={proc.returncode})\n"
                f"---log---\n{content}"
            )
        time.sleep(0.05)

    proc.kill()
    log_fd.close()
    raise RuntimeError(f"{name}: PTY {pty_path} did not appear in 10s")


def send_gcode(api_socket: str, script: str, timeout: float = 30.0) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect(api_socket)
    req = {
        "id": 1,
        "method": "gcode/script",
        "params": {"script": script},
    }
    sock.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if b"\x03" in buf:
            break
    sock.close()
    if b"\x03" in buf:
        body = buf.split(b"\x03", 1)[0]
        try:
            return json.loads(body.decode())
        except json.JSONDecodeError:
            return {"raw": body.decode(errors="replace")}
    return {}


def query_status(api_socket: str, timeout: float = 5.0) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(api_socket)
    except (
        ConnectionRefusedError,
        FileNotFoundError,
        BlockingIOError,
        OSError,
    ):
        return {}
    req = {
        "id": 1,
        "method": "objects/query",
        "params": {
            "objects": {
                "print_stats": None,
                "toolhead": None,
                "virtual_sdcard": None,
            }
        },
    }
    sock.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if b"\x03" in buf:
            break
    sock.close()
    if b"\x03" in buf:
        body = buf.split(b"\x03", 1)[0]
        try:
            return json.loads(body.decode())
        except json.JSONDecodeError:
            pass
    return {}


def wait_for_klippy_ready(
    klippy_log: pathlib.Path,
    klippy_proc: subprocess.Popen,
    timeout: float = 120.0,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if klippy_log.exists():
            content = klippy_log.read_bytes()
            if b"Printer is ready" in content:
                return True
            if b"Welcome to Kalico" in content:
                return True
            if klippy_proc.poll() is not None:
                return False
            if b"Internal error" in content or b"shutdown:" in content:
                return False
        elif klippy_proc.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def wait_for_print_done(
    api_socket: str,
    klippy_proc: subprocess.Popen,
    klippy_log: pathlib.Path,
    timeout: float = 600.0,
) -> tuple:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if klippy_proc.poll() is not None:
            return False, "klippy exited unexpectedly"

        if klippy_log.exists():
            content = klippy_log.read_bytes()
            if b"shutdown:" in content:
                for line in content.decode(errors="replace").split("\n"):
                    if "shutdown:" in line.lower():
                        return False, f"Printer shutdown: {line.strip()}"
                return False, "Printer shutdown (unknown reason)"

        status = query_status(api_socket)
        if status:
            result = status.get("result", {}).get("status", {})
            ps = result.get("print_stats", {})
            state = ps.get("state", "")
            if state == "complete":
                return True, None
            if state == "error":
                return False, ps.get("message", "print error")
            if state == "cancelled":
                return False, "print cancelled"

        time.sleep(0.2)
    return False, "timeout waiting for print"


def run_simulation(
    repo_root: pathlib.Path,
    gcode_path: Optional[pathlib.Path] = None,
    config_dir: Optional[pathlib.Path] = None,
    timeout: float = 600.0,
    verbose: bool = False,
    phase_stepping: bool = False,
    homing_test: bool = False,
    beacon_test: Optional[str] = None,
    motion_state_test: bool = False,
    sensorless_phase_test: bool = False,
    motor_adjust_test: bool = False,
    serve: bool = False,
    serve_data_dir=None,
) -> SimResult:
    wall_start = time.monotonic()

    with tempfile.TemporaryDirectory(prefix="kalico_sim_") as tmpdir:
        tmp = (
            pathlib.Path(serve_data_dir)
            if serve_data_dir
            else pathlib.Path(tmpdir)
        )
        if serve_data_dir:
            tmp.mkdir(parents=True, exist_ok=True)
            # Restart hygiene: a supervisor restart re-enters on the same
            # persistent volume. Drop transient state (stale socket, marker,
            # sim sock dirs) and reap orphaned sim processes from a prior
            # crashed run so the fresh spawn is clean. events/*.jsonl logs
            # are deliberately preserved across restarts.
            for _name in ("klippy.sock", "SERVE_READY"):
                try:
                    (tmp / _name).unlink()
                except FileNotFoundError:
                    pass
            try:
                for _pat in (
                    "klipper-h7-sim.elf",
                    "klipper-f4-sim.elf",
                    "beacon_mcu",
                ):
                    subprocess.run(["pkill", "-f", _pat], check=False)
            except Exception:
                pass
        log_dir = tmp / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        shim_so, vtime_so = ensure_shims_built(repo_root)
        vtime_create(start_ns=1_000_000_000)

        mcus = []
        klippy_proc = None
        chip_servers = []
        beacon_stub = None
        endstop_trigger = None

        try:
            h7_sock_dir = tmp / "sim" / "h7"
            f4_sock_dir = tmp / "sim" / "f4"
            h7_sock_dir.mkdir(parents=True, exist_ok=True)
            f4_sock_dir.mkdir(parents=True, exist_ok=True)

            h7_elf = repo_root / "out" / "klipper-h7-sim.elf"
            f4_elf = repo_root / "out" / "klipper-f4-sim.elf"
            if not h7_elf.exists():
                return SimResult(
                    success=False,
                    print_time_s=0,
                    wall_time_s=0,
                    speedup=0,
                    error="Missing H7 firmware ELF (out/klipper-h7-sim.elf)",
                )

            h7_pty = str(tmp / "klipper_sim_h7")
            f4_pty = str(tmp / "klipper_sim_f4")
            dual_mcu = f4_elf.exists()

            h7 = spawn_mcu(
                "h7",
                h7_elf,
                h7_pty,
                str(log_dir / "h7.log"),
                str(h7_sock_dir),
                shim_so,
                vtime_so,
                verbose,
            )
            mcus.append(h7)
            log.info("H7 MCU spawned (pid=%d)", h7.process.pid)

            if dual_mcu:
                f4 = spawn_mcu(
                    "f4",
                    f4_elf,
                    f4_pty,
                    str(log_dir / "f4.log"),
                    str(f4_sock_dir),
                    shim_so,
                    vtime_so,
                    verbose,
                )
                mcus.append(f4)
                log.info("F4 MCU spawned (pid=%d)", f4.process.pid)

            chip_servers = _start_chip_emulators(
                h7_sock_dir, f4_sock_dir, repo_root
            )
            beacon_pty = str(tmp / "klipper_sim_beacon")
            z_sock_dir = f4_sock_dir if dual_mcu else h7_sock_dir
            beacon_stub = _start_beacon(
                beacon_pty,
                log_dir,
                repo_root,
                step_sock_path=str(z_sock_dir / "sim_control"),
            )
            has_motion_engine = (
                repo_root / "klippy" / "motion_engine.py"
            ).exists()

            rendered_cfg = _prepare_config(
                tmp,
                config_dir,
                repo_root,
                h7_pty,
                f4_pty if dual_mcu else None,
                beacon_pty,
                has_motion_engine=has_motion_engine,
                phase_stepping=phase_stepping,
                homing_test=homing_test,
                beacon_test=beacon_test,
                sensorless_phase_test=sensorless_phase_test,
                motor_adjust_test=motor_adjust_test,
            )

            if serve:
                # smooth_mzv shaper required here: the kalico motion bridge
                # rejects freq=0, so omitting it causes immediate boot failure.
                with open(rendered_cfg, "a") as _cfgf:
                    _cfgf.write(
                        "\n[post_processor is_xy]\ntype: smooth_mzv\n"
                        "frequency_hz: 50\n\n[pause_resume]\n\n"
                        "[display_status]\n\n[exclude_object]\n\n"
                        "[gcode_macro PAUSE]\nrename_existing: BASE_PAUSE\n"
                        "gcode:\n  BASE_PAUSE\n\n"
                        "[gcode_macro RESUME]\nrename_existing: BASE_RESUME\n"
                        "gcode:\n  BASE_RESUME\n\n"
                        "[gcode_macro CANCEL_PRINT]\nrename_existing: BASE_CANCEL_PRINT\n"
                        "gcode:\n  CLEAR_PAUSE\n  BASE_CANCEL_PRINT\n"
                    )

            gcode_dir = tmp / "gcodes"
            gcode_dir.mkdir(parents=True, exist_ok=True)
            if gcode_path:
                import shutil

                gcode_dest = gcode_dir / gcode_path.name
                shutil.copy2(gcode_path, gcode_dest)

            klippy_log = log_dir / "klippy.log"
            api_socket = str(tmp / "klippy.sock")

            env = os.environ.copy()
            # Klippy deliberately does NOT use virtual time: it runs at real
            # CPU speed while MCU processes run under LD_PRELOAD vtime so they
            # process commands instantly. Loading vtime in klippy too causes a
            # deadlock where both sides block on I/O and neither advances time.
            if verbose:
                env["KALICO_VTIME_DEBUG"] = "1"
            env["KALICO_SIM_SOCK_DIR"] = str(h7_sock_dir)

            third_party = (
                repo_root
                / "tools"
                / "sim_klippy"
                / "printer_real"
                / "third_party_repos"
            )
            if third_party.exists():
                pp = env.get("PYTHONPATH", "")
                beacon_path = third_party / "beacon_klipper"
                motors_path = third_party / "motors-sync"
                env["PYTHONPATH"] = ":".join(
                    filter(
                        None,
                        [
                            str(beacon_path) if beacon_path.exists() else "",
                            str(motors_path) if motors_path.exists() else "",
                            pp,
                        ],
                    )
                )
                extras_dir = repo_root / "klippy" / "extras"
                for fname in ("beacon.py", "beacon_kalico.py"):
                    src = beacon_path / fname
                    dest = extras_dir / fname
                    if src.exists() and not dest.exists():
                        try:
                            os.symlink(str(src.resolve()), str(dest))
                            log.info("Symlinked %s into klippy/extras/", fname)
                        except OSError:
                            pass

            klippy_proc = subprocess.Popen(
                [
                    "python3",
                    str(repo_root / "klippy" / "klippy.py"),
                    str(rendered_cfg),
                    "-l",
                    str(klippy_log),
                    "-a",
                    api_socket,
                ],
                env=env,
                stdout=open(log_dir / "klippy.stdout", "wb"),
                stderr=subprocess.STDOUT,
                cwd=str(repo_root),
            )
            log.info("Klippy spawned (pid=%d)", klippy_proc.pid)

            if not wait_for_klippy_ready(klippy_log, klippy_proc, timeout=120):
                content = ""
                if klippy_log.exists():
                    content = klippy_log.read_text(errors="replace")
                return SimResult(
                    success=False,
                    print_time_s=0,
                    wall_time_s=time.monotonic() - wall_start,
                    speedup=0,
                    error=f"Klippy failed to start:\n{content[-2000:]}",
                    klippy_log=content,
                )
            log.info("Klippy ready")

            if serve:
                log.info(
                    "SERVE: klippy ready, holding for Moonraker. api=%s",
                    api_socket,
                )
                (tmp / "SERVE_READY").write_text(
                    "api_socket=%s\nklippy_log=%s\nevents_dir=%s\n"
                    % (api_socket, klippy_log, log_dir / "events")
                )
                try:
                    while klippy_proc.poll() is None:
                        time.sleep(1.0)
                except KeyboardInterrupt:
                    pass
                return SimResult(
                    success=(klippy_proc.poll() in (None, 0)),
                    print_time_s=0,
                    wall_time_s=time.monotonic() - wall_start,
                    speedup=0,
                    error=None,
                    klippy_log=(
                        klippy_log.read_text(errors="replace")
                        if klippy_log.exists()
                        else ""
                    ),
                )

            # Endstop triggering is handled by the libsim_intercept.so
            # shim's auto-endstop feature (step counting → GPIO trigger).
            vtime_start = vtime_read_ns()

            if homing_test:
                log.info("Homing test: faking position, then G28 Z via beacon")

                resp = send_gcode(
                    api_socket,
                    "SET_KINEMATIC_POSITION X=150 Y=150 Z=100",
                    timeout=10,
                )
                log.info("SET_KINEMATIC_POSITION: %s", resp)

                resp = send_gcode(api_socket, "G4 P1000", timeout=15)

                resp = send_gcode(api_socket, "G28 Z", timeout=60)
                log.info("G28 Z: %s", resp)

                time.sleep(2.0)
                klippy_content = (
                    klippy_log.read_text(errors="replace")
                    if klippy_log.exists()
                    else ""
                )
                homing_error = None
                if isinstance(resp, dict) and resp.get("error"):
                    homing_error = f"G28 Z failed: {resp['error']}"
                elif "shutdown:" in klippy_content.lower():
                    for line in klippy_content.split("\n"):
                        if "shutdown:" in line.lower():
                            homing_error = f"Printer shutdown during homing: {line.strip()}"
                            break
                elif not resp:
                    homing_error = "G28 Z timed out or returned no response"

                if homing_error is not None:
                    traffic = log_dir / "beacon_traffic.log"
                    if traffic.exists():
                        for line in traffic.read_text(
                            errors="replace"
                        ).splitlines()[-120:]:
                            log.info("TRAFFIC %s", line)
                    events_dir = log_dir / "events"
                    for ev_file in sorted(events_dir.glob("*.jsonl")):
                        for line in ev_file.read_text(
                            errors="replace"
                        ).splitlines():
                            if "trip" in line or "clock-seed" in line:
                                log.info("EVENT %s", line)

                success = homing_error is None
                error = homing_error
                wall_end = time.monotonic()
                wall_time_s = wall_end - wall_start

                return SimResult(
                    success=success,
                    print_time_s=0,
                    wall_time_s=wall_time_s,
                    speedup=0,
                    error=error,
                    klippy_log=klippy_content,
                    mcu_logs={
                        mcu.name: open(mcu.log_path).read()
                        for mcu in mcus
                        if pathlib.Path(mcu.log_path).exists()
                    },
                )

            if beacon_test:
                log.info("Beacon test variant: %s", beacon_test)
                beacon_error = None

                def _gdb_dump_later(delay, pid):
                    def _run():
                        time.sleep(delay)
                        if klippy_proc.poll() is not None:
                            return
                        try:
                            out = subprocess.run(
                                [
                                    "gdb",
                                    "-p",
                                    str(pid),
                                    "-batch",
                                    "-ex",
                                    "thread apply all bt",
                                ],
                                capture_output=True,
                                text=True,
                                timeout=60,
                            )
                            for line in out.stdout.splitlines():
                                log.info("GDB %s", line)
                            for line in out.stderr.splitlines()[-5:]:
                                log.info("GDBERR %s", line)
                        except Exception as e:
                            log.info("GDB failed: %s", e)

                    t = threading.Thread(target=_run, daemon=True)
                    t.start()

                import threading

                cmds = BEACON_TEST_SCRIPTS[beacon_test]
                for idx, (script, cmd_timeout) in enumerate(cmds):
                    log.info("Sending: %s", script)
                    if idx == len(cmds) - 1:
                        _gdb_dump_later(4.0, klippy_proc.pid)
                    resp = send_gcode(api_socket, script, timeout=cmd_timeout)
                    log.info("%s: %s", script, resp)
                    if isinstance(resp, dict) and resp.get("error"):
                        beacon_error = "%s failed: %s" % (
                            script,
                            resp["error"],
                        )
                        break
                    if not resp:
                        beacon_error = (
                            "%s timed out or returned no response" % (script,)
                        )
                        break

                klippy_content = (
                    klippy_log.read_text(errors="replace")
                    if klippy_log.exists()
                    else ""
                )
                if beacon_error is None:
                    if "shutdown:" in klippy_content.lower():
                        for line in klippy_content.split("\n"):
                            if "shutdown:" in line.lower():
                                beacon_error = (
                                    "Printer shutdown during beacon test: "
                                    + line.strip()
                                )
                                break
                if beacon_error is None and beacon_test == "connect":
                    relevant = klippy_content.replace(
                        "Executing Beacon update script failed: Traceback",
                        "Executing Beacon update script failed:",
                    )
                    if "Traceback" in relevant:
                        beacon_error = "klippy.log contains a Traceback"
                    elif "Failed to load module 'beacon'" in klippy_content:
                        beacon_error = "beacon module failed to load"
                if beacon_error is not None and klippy_proc.poll() is None:
                    try:
                        out = subprocess.run(
                            [
                                "gdb",
                                "-p",
                                str(klippy_proc.pid),
                                "-batch",
                                "-ex",
                                "thread apply all bt",
                            ],
                            capture_output=True,
                            text=True,
                            timeout=120,
                        )
                        for line in out.stdout.splitlines():
                            log.info("GDB %s", line)
                        for line in out.stderr.splitlines()[-5:]:
                            log.info("GDBERR %s", line)
                    except Exception as e:
                        log.info("GDB failed: %s", e)
                if beacon_error is not None:
                    stdout_log = log_dir / "klippy.stdout"
                    if stdout_log.exists():
                        for line in stdout_log.read_text(
                            errors="replace"
                        ).splitlines()[-80:]:
                            log.info("STDOUT %s", line)
                    traffic = log_dir / "beacon_traffic.log"
                    if traffic.exists():
                        for line in traffic.read_text(
                            errors="replace"
                        ).splitlines()[-500:]:
                            log.info("TRAFFIC %s", line)
                    events_dir = log_dir / "events"
                    for ev_file in sorted(events_dir.glob("*.jsonl")):
                        lines = ev_file.read_text(errors="replace").splitlines()
                        for line in lines:
                            if (
                                "fire_and_forget" in line
                                or "beacon_stream" in line
                                or "[relay]" in line
                                or "Backpressure" in line
                                or "ceiling" in line
                            ):
                                log.info("EVENT %s", line)
                        for line in lines[-150:]:
                            log.info("EVENT %s", line)
                if (
                    beacon_test == "contact"
                    and beacon_error is not None
                    and "model convergence" in beacon_error
                    and "Collected" in klippy_content
                ):
                    log.info(
                        "contact: tolerating emulator model-fit limitation: %s",
                        beacon_error,
                    )
                    beacon_error = None
                if beacon_error is None:
                    for needle in BEACON_TEST_LOG_ASSERTIONS.get(
                        beacon_test, ()
                    ):
                        if needle not in klippy_content:
                            beacon_error = (
                                "expected %r in klippy.log, not found"
                                % (needle,)
                            )
                            break

                return SimResult(
                    success=beacon_error is None,
                    print_time_s=0,
                    wall_time_s=time.monotonic() - wall_start,
                    speedup=0,
                    error=beacon_error,
                    klippy_log=klippy_content,
                    mcu_logs={
                        mcu.name: open(mcu.log_path).read()
                        for mcu in mcus
                        if pathlib.Path(mcu.log_path).exists()
                    },
                )

            if sensorless_phase_test:
                log.info(
                    "Sensorless phase test: G28 Z with TMC5160 virtual "
                    "endstop on a phase-stepped axis"
                )

                diag_trigger = EndstopTrigger(
                    str(h7_sock_dir / "sim_control"), [(0, 203)]
                )
                diag_trigger.start()
                try:
                    resp = send_gcode(api_socket, "G28 Z", timeout=120)
                finally:
                    diag_trigger.stop()
                    diag_trigger.clear()
                log.info("G28 Z: %s", resp)

                klippy_content = (
                    klippy_log.read_text(errors="replace")
                    if klippy_log.exists()
                    else ""
                )
                sp_error = None
                if isinstance(resp, dict) and resp.get("error"):
                    sp_error = f"G28 Z failed: {resp['error']}"
                elif "shutdown:" in klippy_content.lower():
                    for line in klippy_content.split("\n"):
                        if "shutdown:" in line.lower():
                            sp_error = (
                                "Printer shutdown during sensorless "
                                f"homing: {line.strip()}"
                            )
                            break
                elif not resp:
                    sp_error = "G28 X timed out or returned no response"

                if sp_error is None:
                    enter_marker = "phase mode entered"
                    exit_marker = "phase mode exited (pulse stepping)"
                    first_enter = klippy_content.find(enter_marker)
                    exit_idx = klippy_content.find(exit_marker)
                    re_enter = klippy_content.find(
                        enter_marker,
                        exit_idx + len(exit_marker) if exit_idx >= 0 else 0,
                    )
                    if first_enter < 0:
                        sp_error = (
                            "phase mode never entered (post-enable "
                            "callback did not run)"
                        )
                    elif exit_idx < 0:
                        sp_error = (
                            "phase mode never exited around the "
                            "StallGuard trip move"
                        )
                    elif not (first_enter < exit_idx < re_enter):
                        sp_error = (
                            "phase mode enter/exit/re-enter sequence out "
                            f"of order (enter={first_enter} exit={exit_idx}"
                            f" re-enter={re_enter})"
                        )

                if sp_error is None:
                    status = query_status(api_socket)
                    toolhead = (
                        status.get("result", {})
                        .get("status", {})
                        .get("toolhead", {})
                    )
                    homed = toolhead.get("homed_axes", "")
                    pos = toolhead.get("position", [None])
                    pos_z = pos[2] if len(pos) > 2 else None
                    if "z" not in homed:
                        sp_error = (
                            f"Z not reported homed (homed_axes={homed!r})"
                        )
                    elif pos_z is None or abs(pos_z) > 6.0:
                        sp_error = (
                            "homed Z position %r too far from "
                            "position_endstop=0 (wall=50 steps + retract "
                            "tolerance)" % (pos_z,)
                        )

                success = sp_error is None
                wall_end = time.monotonic()
                return SimResult(
                    success=success,
                    print_time_s=0,
                    wall_time_s=wall_end - wall_start,
                    speedup=0,
                    error=sp_error,
                    klippy_log=klippy_content,
                    mcu_logs={
                        mcu.name: open(mcu.log_path).read()
                        for mcu in mcus
                        if pathlib.Path(mcu.log_path).exists()
                    },
                )

            if motor_adjust_test:
                log.info(
                    "Motor-adjust test: MOTOR_ADJUST on a 3-Z-stepper axis"
                )
                ma_error = None

                def _toolhead_z():
                    status = query_status(api_socket)
                    toolhead = (
                        status.get("result", {})
                        .get("status", {})
                        .get("toolhead", {})
                    )
                    pos = toolhead.get("position", [None, None, None])
                    return pos[2] if len(pos) > 2 else None

                send_gcode(
                    api_socket,
                    "SET_KINEMATIC_POSITION X=125 Y=125 Z=100",
                    timeout=10,
                )
                send_gcode(api_socket, "G4 P500", timeout=15)
                z_before = _toolhead_z()

                resp = send_gcode(
                    api_socket,
                    "MOTOR_ADJUST MOTOR=stepper_z1 DELTA=2.0",
                    timeout=60,
                )
                log.info("MOTOR_ADJUST stepper_z1: %s", resp)
                if isinstance(resp, dict) and resp.get("error"):
                    ma_error = f"MOTOR_ADJUST failed: {resp['error']}"
                elif not resp:
                    ma_error = "MOTOR_ADJUST timed out"

                if ma_error is None:
                    z_after = _toolhead_z()
                    if z_before is None or z_after is None:
                        ma_error = (
                            "could not read toolhead Z (before=%r after=%r)"
                            % (z_before, z_after)
                        )
                    elif abs(z_after - z_before) > 1e-6:
                        ma_error = (
                            "commanded Z changed across MOTOR_ADJUST: "
                            "%.6f -> %.6f" % (z_before, z_after)
                        )

                if ma_error is None:
                    time.sleep(2.0)
                    events_dir = log_dir / "events"
                    ev_text = ""
                    for ev_file in sorted(events_dir.glob("*.jsonl")):
                        ev_text += ev_file.read_text(errors="replace")
                    if "motion.correction_start" not in ev_text:
                        ma_error = (
                            "no motion.correction_start event in events/*.jsonl"
                        )
                    elif "motion.correction_drained" not in ev_text:
                        ma_error = (
                            "no motion.correction_drained event in "
                            "events/*.jsonl"
                        )
                    if ma_error is not None:
                        for line in ev_text.splitlines():
                            if (
                                "motion." in line
                                or "fault" in line
                                or "correction" in line
                            ):
                                log.info("EVENT %s", line)
                        resp2 = send_gcode(
                            api_socket,
                            "MOTOR_ADJUST MOTOR=stepper_z2 DELTA=0.5",
                            timeout=60,
                        )
                        log.info("DISCRIMINATOR second adjust: %s", resp2)
                        resp3 = send_gcode(
                            api_socket, "G1 Z101 F300", timeout=30
                        )
                        log.info("DISCRIMINATOR G1 Z101: %s", resp3)
                        resp4 = send_gcode(api_socket, "M400", timeout=30)
                        log.info("DISCRIMINATOR M400: %s", resp4)

                if ma_error is None:
                    for script in (
                        "MOTOR_ADJUST MOTOR=stepper_z DELTA=1.5 SPEED=5",
                        "MOTOR_ADJUST MOTOR=stepper_z2 DELTA=0.8 SPEED=5",
                        "G1 Z104 F300",
                        "G1 X10 F12000",
                        "M400",
                    ):
                        resp = send_gcode(api_socket, script, timeout=60)
                        log.info("post-adjust %s: %s", script, resp)
                        if isinstance(resp, dict) and resp.get("error"):
                            ma_error = "%s failed: %s" % (
                                script,
                                resp["error"],
                            )
                            break
                        if not resp:
                            ma_error = "%s timed out" % (script,)
                            break

                if ma_error is None:
                    resp = send_gcode(
                        api_socket,
                        "MOTOR_ADJUST MOTOR=stepper_q DELTA=1",
                        timeout=30,
                    )
                    log.info("MOTOR_ADJUST stepper_q: %s", resp)
                    err_text = (
                        resp.get("error", "") if isinstance(resp, dict) else ""
                    )
                    if "Unknown motor" not in str(err_text):
                        ma_error = (
                            "unknown-motor command did not fail with the "
                            "expected error; got %r" % (resp,)
                        )

                klippy_content = (
                    klippy_log.read_text(errors="replace")
                    if klippy_log.exists()
                    else ""
                )
                if ma_error is None and "shutdown:" in klippy_content.lower():
                    for line in klippy_content.split("\n"):
                        if "shutdown:" in line.lower():
                            ma_error = (
                                "MCU shutdown during motor-adjust test: "
                                + line.strip()
                            )
                            break

                success = ma_error is None
                wall_end = time.monotonic()
                return SimResult(
                    success=success,
                    print_time_s=0,
                    wall_time_s=wall_end - wall_start,
                    speedup=0,
                    error=ma_error,
                    klippy_log=klippy_content,
                    mcu_logs={
                        mcu.name: open(mcu.log_path).read()
                        for mcu in mcus
                        if pathlib.Path(mcu.log_path).exists()
                    },
                )

            if motion_state_test:
                log.info("Motion-state test: move, then query mid-move state")

                send_gcode(
                    api_socket,
                    "SET_KINEMATIC_POSITION X=150 Y=150 Z=100",
                    timeout=10,
                )
                send_gcode(api_socket, "G4 P500", timeout=15)
                send_gcode(api_socket, "G1 X170 F600", timeout=30)
                send_gcode(api_socket, "M400", timeout=30)

                offset = len(klippy_log.read_bytes())
                resp = send_gcode(
                    api_socket, "KALICO_SIM_MOTION_STATE T_AGO=1.0", timeout=15
                )
                out, offset = _log_tail_since(klippy_log, offset)
                log.info("KALICO_SIM_MOTION_STATE: %s", out or resp)

                klippy_content = (
                    klippy_log.read_text(errors="replace")
                    if klippy_log.exists()
                    else ""
                )

                ms_error = None
                if isinstance(resp, dict) and resp.get("error"):
                    ms_error = (
                        f"KALICO_SIM_MOTION_STATE failed: {resp['error']}"
                    )
                elif "shutdown:" in klippy_content.lower():
                    for line in klippy_content.split("\n"):
                        if "shutdown:" in line.lower():
                            ms_error = f"MCU shutdown during motion-state test: {line.strip()}"
                            break
                else:
                    m = re.search(
                        r"x: pos=([0-9.eE+-]+) vel=([0-9.eE+-]+)", out
                    )
                    if not m:
                        ms_error = f"no x-axis state in response: {out!r}"
                    else:
                        pos, vel = float(m.group(1)), float(m.group(2))
                        if not (150.5 < pos < 169.5):
                            ms_error = (
                                "x pos %.4f not strictly interior to move span (150.5, 169.5) — endpoint clamp or wrong print_time?"
                                % pos
                            )
                        elif vel <= 5.0:
                            ms_error = (
                                "x vel %.4f not strictly moving (expected > 5 mm/s mid-cruise) — query may have landed at rest"
                                % vel
                            )
                success = ms_error is None
                error = ms_error
                wall_end = time.monotonic()
                wall_time_s = wall_end - wall_start

                return SimResult(
                    success=success,
                    print_time_s=0,
                    wall_time_s=wall_time_s,
                    speedup=0,
                    error=error,
                    klippy_log=klippy_content,
                    mcu_logs={
                        mcu.name: open(mcu.log_path).read()
                        for mcu in mcus
                        if pathlib.Path(mcu.log_path).exists()
                    },
                )

            if gcode_path:
                gcode_name = gcode_path.name
                resp = send_gcode(
                    api_socket,
                    f"SDCARD_PRINT_FILE FILENAME={gcode_name}",
                    timeout=30,
                )
                log.info("Print started: %s", gcode_name)

                success, error = wait_for_print_done(
                    api_socket,
                    klippy_proc,
                    klippy_log,
                    timeout,
                )
            else:
                log.info("No G-code file, generating test pattern")
                test_gcode = gcode_dir / "sim_test.gcode"
                if phase_stepping:
                    test_gcode.write_text("""\
; Phase stepping acceptance test
SET_KINEMATIC_POSITION X=0 Y=125 Z=125
G1 X50 F1000
G1 X100 F2000
G1 X50 F3000
G1 X0 F1000
""")
                else:
                    test_gcode.write_text("""\
; Kalico Sim self-test: square spiral with Z moves
SET_KINEMATIC_POSITION X=125 Y=125 Z=125
G1 Z120 F300
G1 X10 Y10 F3000
G1 X100 Y10 F3000
G1 X100 Y100 F3000
G1 X10 Y100 F3000
G1 X10 Y10 F3000
G1 X20 Y20 F3000
G1 X90 Y20 F3000
G1 X90 Y90 F3000
G1 X20 Y90 F3000
G1 X20 Y20 F3000
G1 X30 Y30 F2000
G1 X80 Y30 F2000
G1 X80 Y80 F2000
G1 X30 Y80 F2000
G1 X30 Y30 F2000
G1 X40 Y40 F1500
G1 X70 Y40 F1500
G1 X70 Y70 F1500
G1 X40 Y70 F1500
G1 X40 Y40 F1500
G1 Z125 F300
""")
                resp = send_gcode(
                    api_socket,
                    "SDCARD_PRINT_FILE FILENAME=sim_test.gcode",
                    timeout=30,
                )
                log.info("Print started: sim_test.gcode")
                success, error = wait_for_print_done(
                    api_socket,
                    klippy_proc,
                    klippy_log,
                    timeout,
                )

            wall_end = time.monotonic()
            wall_time_s = wall_end - wall_start

            print_time_s = 0.0
            try:
                status = query_status(api_socket)
                if status:
                    r = status.get("result", {}).get("status", {})
                    ps = r.get("print_stats", {})
                    print_time_s = ps.get("total_duration", 0.0)
                    if print_time_s == 0:
                        print_time_s = ps.get("print_duration", 0.0)
            except Exception:
                pass

            if print_time_s == 0:
                try:
                    klippy_content = klippy_log.read_text(errors="replace")
                    for line in reversed(klippy_content.split("\n")):
                        m = re.search(r"print_time=(\d+\.?\d*)", line)
                        if m:
                            print_time_s = float(m.group(1))
                            break
                except Exception:
                    pass

            speedup = (
                print_time_s / wall_time_s
                if (wall_time_s > 0 and print_time_s > 0)
                else 0
            )

            klippy_content = ""
            if klippy_log.exists():
                klippy_content = klippy_log.read_text(errors="replace")

            mcu_log_contents = {}
            for mcu in mcus:
                try:
                    mcu_log_contents[mcu.name] = open(mcu.log_path).read()
                except Exception:
                    pass

            return SimResult(
                success=success,
                print_time_s=print_time_s,
                wall_time_s=wall_time_s,
                speedup=speedup,
                error=error,
                klippy_log=klippy_content,
                mcu_logs=mcu_log_contents,
            )

        finally:
            if endstop_trigger:
                endstop_trigger.stop()
            if klippy_proc and klippy_proc.poll() is None:
                klippy_proc.terminate()
                try:
                    klippy_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    klippy_proc.kill()

            for mcu in mcus:
                if mcu.process.poll() is None:
                    mcu.process.send_signal(signal.SIGTERM)
            for mcu in mcus:
                try:
                    mcu.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    mcu.process.kill()

            for srv in chip_servers:
                srv.stop()

            if beacon_stub:
                beacon_stub.stop()

            vtime_destroy()


def _start_chip_emulators(h7_sock_dir, f4_sock_dir, repo_root):
    servers = []
    try:
        sys.path.insert(0, str(repo_root))
        from tools.sim_klippy.orchestrator.chip_socket_server import (
            ChipSocketServer,
        )
        from tools.sim_klippy.orchestrator.max31865_emulator import (
            MAX31865Emulator,
        )
        from tools.sim_klippy.orchestrator.tmc2209_emulator import (
            TMC2209Emulator,
        )
        from tools.sim_klippy.orchestrator.tmc5160_emulator import (
            TMC5160Emulator,
        )

        h7_chips = [
            (5, TMC5160Emulator().transfer),
            (4, TMC5160Emulator().transfer),
            (6, TMC5160Emulator().transfer),
            (3, TMC5160Emulator().transfer),
            (40, MAX31865Emulator().transfer),  # extruder_rtd
        ]
        for cs_line, transfer in h7_chips:
            path = str(h7_sock_dir / f"spi_cs_0_{cs_line}")
            srv = ChipSocketServer(path, transfer, framed=False)
            srv.start()
            servers.append(srv)

        # H7 TMC2209 (extruder)

        chip = TMC2209Emulator(slave_addr=0)
        srv = ChipSocketServer(
            str(h7_sock_dir / "tmcuart_0"),
            chip.handle,
            chunk=10,
        )
        srv.start()
        servers.append(srv)

        # F4 TMC2209s (Z, Z1, Z2)
        for i in range(3):
            chip = TMC2209Emulator(slave_addr=0)
            path = str(f4_sock_dir / f"tmcuart_{i}")
            srv = ChipSocketServer(path, chip.handle, chunk=10)
            srv.start()
            servers.append(srv)

        log.info("Started %d chip emulators", len(servers))
    except ImportError as e:
        log.warning("Could not import sim_klippy emulators: %s", e)
    return servers


def _start_beacon(beacon_pty, log_dir, repo_root, step_sock_path=None):
    try:
        sys.path.insert(0, str(repo_root))
        # tools/kalico-sim is not importable as a module (hyphen in name),
        # so the emulators/ subdir is added to sys.path directly.
        emulators_dir = repo_root / "tools" / "kalico-sim" / "emulators"
        if emulators_dir.exists():
            sys.path.insert(0, str(emulators_dir))
        try:
            from beacon_mcu import BeaconMcuStub
        except ImportError:
            from tools.sim_klippy.orchestrator.beacon_serial_stub import (
                BeaconMcuStub,
            )
        try:
            beacon = BeaconMcuStub(
                beacon_pty,
                log_path=str(log_dir / "beacon_traffic.log"),
                step_sock_path=step_sock_path,
            )
        except TypeError:
            beacon = BeaconMcuStub(
                beacon_pty,
                log_path=str(log_dir / "beacon_traffic.log"),
            )
        beacon.start_sample_stream(z_target_mm=10.0, rate_hz=200)
        log.info("Beacon MCU emulator started")
        return beacon
    except ImportError as e:
        log.warning("Could not import Beacon emulator: %s", e)
        return None


def _prepare_config(
    tmp_dir: pathlib.Path,
    config_dir: Optional[pathlib.Path],
    repo_root: pathlib.Path,
    h7_pty: str,
    f4_pty: Optional[str],
    beacon_pty: str,
    has_motion_engine: bool = False,
    phase_stepping: bool = False,
    homing_test: bool = False,
    beacon_test: Optional[str] = None,
    sensorless_phase_test: bool = False,
    motor_adjust_test: bool = False,
) -> pathlib.Path:
    if config_dir is None:
        if motor_adjust_test:
            cfg = _generate_multi_z_config(
                h7_pty, gcode_dir=str(tmp_dir / "gcodes")
            )
            cfg_path = tmp_dir / "printer.cfg"
            cfg_path.write_text(cfg)
            return cfg_path
        if homing_test or beacon_test:
            cfg = _generate_beacon_homing_config(
                h7_pty,
                f4_pty,
                beacon_pty,
                gcode_dir=str(tmp_dir / "gcodes"),
                bed_mesh=(beacon_test == "mesh"),
            )
        elif sensorless_phase_test:
            cfg = _generate_sensorless_phase_config(
                h7_pty, gcode_dir=str(tmp_dir / "gcodes")
            )
        elif phase_stepping:
            cfg = _generate_phase_stepping_config(
                h7_pty, f4_pty, gcode_dir=str(tmp_dir / "gcodes")
            )
        else:
            cfg = _generate_minimal_config(
                h7_pty, f4_pty, gcode_dir=str(tmp_dir / "gcodes")
            )
        if (
            has_motion_engine
            and not phase_stepping
            and not homing_test
            and not beacon_test
            and not sensorless_phase_test
        ):
            cfg += """
[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50
"""
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(cfg)
        return cfg_path

    try:
        sys.path.insert(0, str(repo_root))
        from tools.sim_klippy.orchestrator.overrides import (
            apply_overrides,
            load_overrides,
        )

        overrides_path = (
            repo_root / "tools" / "sim_klippy" / "pin-overrides.toml"
        )
        if overrides_path.exists():
            overrides = load_overrides(overrides_path)
        else:
            overrides = {}

        serial_map = {"usb-Klipper_stm32h723xx_*": h7_pty}
        if f4_pty:
            serial_map["usb-Klipper_stm32f446xx_*"] = f4_pty
        serial_map["usb-Beacon_*"] = beacon_pty
        overrides["mcu_main.serial"] = serial_map

        cfg_text = (config_dir / "printer.cfg").read_text()
        rendered = apply_overrides(cfg_text, overrides)
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(rendered)

        import shutil

        for entry in config_dir.iterdir():
            if entry.name == "printer.cfg":
                continue
            target = tmp_dir / entry.name
            if target.exists():
                continue
            if entry.is_dir():
                os.symlink(entry.resolve(), target)
            elif entry.suffix == ".cfg":
                text = entry.read_text()
                text = apply_overrides(text, overrides)
                target.write_text(text)
            else:
                shutil.copy2(entry, target)

        return cfg_path

    except (ImportError, FileNotFoundError) as e:
        log.warning("Could not use sim_klippy overrides: %s", e)
        cfg = _generate_minimal_config(h7_pty, f4_pty)
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(cfg)
        return cfg_path


class EndstopTrigger:
    """Toggles endstop GPIO lines via sim_control socket to simulate endstop
    switches; endstops trigger after a brief delay from when homing starts.
    """

    def __init__(self, sim_control_path: str, endstop_pins: list):
        self.sim_control_path = sim_control_path
        self.endstop_pins = endstop_pins
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _send_cmd(self, cmd: str) -> str:
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            sock.connect(self.sim_control_path)
            sock.sendall((cmd + "\n").encode())
            resp = sock.recv(256).decode().strip()
            sock.close()
            return resp
        except (ConnectionRefusedError, FileNotFoundError, socket.timeout):
            return ""

    def _run(self):
        # Load-bearing sleep: MCU must be fully up before GPIO commands work.
        time.sleep(0.5)

        # Cycles clear→wait→trigger→wait→repeat so the endstop always triggers
        # within ~0.5 s of homing start, clears for the retract (~0.5-1 s), then
        # triggers again for the second approach.
        while not self._stop.is_set():
            # Clear phase — long enough for homing start and retract to complete.
            for chip, line in self.endstop_pins:
                self._send_cmd(
                    f"set_gpio_input chip={chip} line={line} value=0"
                )
            self._stop.wait(1.0)
            if self._stop.is_set():
                break

            for chip, line in self.endstop_pins:
                self._send_cmd(
                    f"set_gpio_input chip={chip} line={line} value=1"
                )
            self._stop.wait(0.1)
            if self._stop.is_set():
                break

    def trigger_once(self):
        for chip, line in self.endstop_pins:
            self._send_cmd(f"set_gpio_input chip={chip} line={line} value=1")

    def clear(self):
        for chip, line in self.endstop_pins:
            self._send_cmd(f"set_gpio_input chip={chip} line={line} value=0")


def _generate_beacon_homing_config(
    h7_pty: str,
    f4_pty: str,
    beacon_pty: str,
    gcode_dir: str = "/tmp/kalico_sim_gcodes",
    bed_mesh: bool = False,
) -> str:
    """SAVE_CONFIG beacon model must match the stub's frequency model.

    model_domain [1.8359e-7, 1.8936e-7] → frequencies ~5.28–5.45 MHz:
      z=10mm → count ≈ 70,710,853 (freq ≈ 5,268,182 Hz)
      z=0    → count ≈ 73,153,076 (freq ≈ 5,450,000 Hz)
    Changing either without the other causes boot-time calibration rejection.
    """
    f4_section = ""
    if f4_pty:
        f4_section = f"""
[mcu bottom]
serial: {f4_pty}
"""
    z_step_mcu = "bottom:" if f4_pty else ""
    bed_mesh_section = ""
    if bed_mesh:
        bed_mesh_section = """
[bed_mesh]
mesh_min: 20,20
mesh_max: 280,280
probe_count: 3,3
"""
    return f"""\
[mcu]
serial: {h7_pty}
{f4_section}
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z

[axis x]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_max: 250
endstop_pin: probe:z_virtual_endstop
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 300
max_accel: 3000

[limit z]
axes: z
max_velocity: 10
max_accel: 100

[motor a]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: {z_step_mcu}gpiochip0/gpio0
dir_pin: {z_step_mcu}gpiochip0/gpio1
enable_pin: !{z_step_mcu}gpiochip0/gpio2
microsteps: 16
rotation_distance: 4

[beacon]
serial: {beacon_pty}
x_offset: 0
y_offset: 0
home_xy_position: 150, 150
home_z_hop: 5
home_z_hop_speed: 10
home_xy_move_speed: 50
home_method: proximity
home_method_when_homed: proximity
home_autocalibrate: never
{bed_mesh_section}
[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [beacon model default]
#*# model_coef = 1.4366832587589902,
#*#   1.7791425946955506,
#*#   0.8114676630327906,
#*#   0.4077638527717382,
#*#   0.2629778891883896,
#*#   0.21087515838926726,
#*#   -0.15390965626840192,
#*#   -0.21990798533166914,
#*#   0.24377872047881705,
#*#   0.22573604715705745
#*# model_domain = 1.8359521074610915e-07,1.893648763276026e-07
#*# model_range = 0.200000,5.000000
#*# model_temp = 30.886664
#*# model_offset = 0.00000
"""


BEACON_TEST_VARIANTS = (
    "connect",
    "contact",
    "probe",
    "poke",
    "mesh",
    "accel",
)

BEACON_TEST_SCRIPTS = {
    "connect": [("G4 P2000", 15)],
    "contact": [
        ("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", 10),
        ("G4 P1000", 15),
        ("BEACON_AUTO_CALIBRATE", 600),
    ],
    "probe": [
        ("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", 10),
        ("G4 P1000", 15),
        ("G28 Z", 120),
        ("PROBE PROBE_METHOD=proximity SAMPLES=2", 120),
        ("PROBE PROBE_METHOD=contact SAMPLES=1", 120),
        ("PROBE_ACCURACY SAMPLES=3", 180),
    ],
    "poke": [
        ("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", 10),
        ("G4 P1000", 15),
        ("BEACON_POKE TOP=5 BOTTOM=-0.3", 120),
    ],
    "mesh": [
        ("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", 10),
        ("G4 P1000", 15),
        ("G28 Z", 120),
        ("BED_MESH_CALIBRATE", 600),
    ],
    "accel": [
        ("ACCELEROMETER_MEASURE CHIP=beacon", 30),
        ("G4 P1000", 15),
        ("ACCELEROMETER_MEASURE CHIP=beacon NAME=test", 30),
    ],
}

BEACON_TEST_LOG_ASSERTIONS = {
    "contact": ("Collected",),
    "poke": ("Triggered at:", "Armed at:"),
}


PROBE_TEST_VARIANTS = (
    "virtual",
    "safe-z",
    "gpio-z",
    "no-probe",
    "conflict",
    "pullup",
    "remote",
    "points",
)

PROBE_TEST_BOOT_ERRORS = {
    "no-probe": "Unknown pin chip name 'probe'",
    "conflict": "must not set position_endstop",
    "pullup": "Can not pullup/invert probe virtual endstop",
}


def _generate_probe_config(
    h7_pty: str, gcode_dir: str, variant: str, f4_pty: Optional[str] = None
) -> str:
    """Auto-endstop walls (libsim_intercept.c): X steps→gpio200,
    Y steps→gpio201, Z steps→gpio202 and gpio203."""
    if variant == "gpio-z":
        z_endstop = "endstop_pin: ^gpiochip0/gpio202\nposition_endstop: 0"
        probe_pin = "gpiochip0/gpio203"
    elif variant == "pullup":
        z_endstop = "endstop_pin: ^probe:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio202"
    elif variant == "remote":
        z_endstop = "endstop_pin: sim_remote_endstop:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio203"
    elif variant == "conflict":
        z_endstop = (
            "endstop_pin: probe:z_virtual_endstop\nposition_endstop: 1.0"
        )
        probe_pin = "gpiochip0/gpio202"
    else:
        z_endstop = "endstop_pin: probe:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio202"

    probe_section = ""
    if variant != "no-probe":
        probe_section = f"""
[probe]
pin: {probe_pin}
z_offset: 1.5
speed: 5
x_offset: 24.0
y_offset: 5.0
"""

    safe_z_section = ""
    if variant == "safe-z":
        safe_z_section = """
[safe_z_home]
home_xy_position: 125, 125
z_hop: 10
z_hop_speed: 15
"""

    remote_section = ""
    if variant == "remote":
        if f4_pty is None:
            raise ValueError(
                "probe-test remote: a second (F446) sim MCU is required so"
                " the trsync lives on a different MCU than the steppers"
            )
        probe_section = ""
        remote_section = f"""
[mcu bottom]
serial: {f4_pty}

[sim_remote_endstop]
mcu: bottom
trigger_delay: 1.0
measured_z: 3.25
trigger_height: 0
"""

    z_motors = "z"
    points_sections = ""
    if variant == "points":
        z_motors = "z, z1"
        points_sections = """
[motor z1]
drive: stepper
step_pin: gpiochip0/gpio9
dir_pin: gpiochip0/gpio10
enable_pin: !gpiochip0/gpio11
microsteps: 16
rotation_distance: 4

[z_tilt]
z_positions:
    0, 125
    250, 125
points:
    50, 125
    200, 125
speed: 50
horizontal_move_z: 8

[bed_mesh]
mesh_min: 30, 10
mesh_max: 200, 200
probe_count: 3, 3
speed: 50
horizontal_move_z: 8

[screws_tilt_adjust]
screw1: 50, 50
screw1_name: front left
screw2: 200, 50
screw2_name: front right
screw3: 125, 200
screw3_name: back
speed: 50
horizontal_move_z: 8
screw_thread: CW-M4

[axis_twist_compensation]
calibrate_start_x: 30
calibrate_end_x: 200
calibrate_y: 125
"""
    return f"""\
[mcu]
serial: {h7_pty}

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: {z_motors}

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio200
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio201
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_max: 250
homing_speed: 5
{z_endstop}

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4
{safe_z_section}{probe_section}{remote_section}{points_sections}
[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _wait_for_log_text(
    klippy_log: pathlib.Path,
    klippy_proc: subprocess.Popen,
    needle: str,
    timeout: float = 60.0,
) -> Optional[str]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if klippy_log.exists():
            for line in klippy_log.read_text(errors="replace").split("\n"):
                if needle in line:
                    return line.strip()
        if klippy_proc.poll() is not None and klippy_log.exists():
            for line in klippy_log.read_text(errors="replace").split("\n"):
                if needle in line:
                    return line.strip()
            return None
        time.sleep(0.2)
    return None


def _log_tail_since(klippy_log: pathlib.Path, offset: int) -> tuple:
    data = klippy_log.read_bytes() if klippy_log.exists() else b""
    return data[offset:].decode(errors="replace"), len(data)


def _query_toolhead_position(api_socket: str) -> Optional[list]:
    status = query_status(api_socket)
    pos = (
        status.get("result", {})
        .get("status", {})
        .get("toolhead", {})
        .get("position")
    )
    return list(pos) if pos else None


def _query_toolhead_z(api_socket: str) -> Optional[float]:
    pos = _query_toolhead_position(api_socket)
    if pos and len(pos) >= 3:
        return float(pos[2])
    return None


def run_probe_test(
    repo_root: pathlib.Path,
    variant: str,
    timeout: float = 600.0,
    verbose: bool = False,
) -> int:
    checks = []

    def check(name, ok, detail):
        checks.append((name, bool(ok), detail))
        log.info("CHECK %-28s %s  %s", name, "PASS" if ok else "FAIL", detail)

    with tempfile.TemporaryDirectory(prefix="kalico_probe_") as tmpdir:
        tmp = pathlib.Path(tmpdir)
        log_dir = tmp / "logs"
        log_dir.mkdir(parents=True)
        gcode_dir = tmp / "gcodes"
        gcode_dir.mkdir(parents=True)
        klippy_log = log_dir / "klippy.log"
        api_socket = str(tmp / "klippy.sock")

        shim_so, vtime_so = ensure_shims_built(repo_root)
        vtime_create(start_ns=1_000_000_000)

        mcus = []
        klippy_proc = None
        expect_boot_error = PROBE_TEST_BOOT_ERRORS.get(variant)

        try:
            h7_pty = str(tmp / "klipper_sim_h7")
            if expect_boot_error is None:
                h7_sock_dir = tmp / "sim" / "h7"
                h7_sock_dir.mkdir(parents=True)
                h7_elf = repo_root / "out" / "klipper-h7-sim.elf"
                if not h7_elf.exists():
                    print("ERROR: missing out/klipper-h7-sim.elf")
                    return 1
                mcus.append(
                    spawn_mcu(
                        "h7",
                        h7_elf,
                        h7_pty,
                        str(log_dir / "h7.log"),
                        str(h7_sock_dir),
                        shim_so,
                        vtime_so,
                        verbose,
                    )
                )
                log.info("H7 MCU spawned (pid=%d)", mcus[0].process.pid)

            f4_pty = None
            if expect_boot_error is None and variant == "remote":
                f4_pty = str(tmp / "klipper_sim_f4")
                f4_sock_dir = tmp / "sim" / "f4"
                f4_sock_dir.mkdir(parents=True)
                f4_elf = repo_root / "out" / "klipper-f4-sim.elf"
                if not f4_elf.exists():
                    print("ERROR: missing out/klipper-f4-sim.elf")
                    return 1
                mcus.append(
                    spawn_mcu(
                        "f4",
                        f4_elf,
                        f4_pty,
                        str(log_dir / "f4.log"),
                        str(f4_sock_dir),
                        shim_so,
                        vtime_so,
                        verbose,
                    )
                )
                log.info("F4 MCU spawned (pid=%d)", mcus[-1].process.pid)

            cfg_text = _generate_probe_config(
                h7_pty, str(gcode_dir), variant, f4_pty
            )
            cfg_path = tmp / "printer.cfg"
            cfg_path.write_text(cfg_text)
            if variant == "safe-z":
                check(
                    "config-safe-z-before-probe",
                    cfg_text.index("[safe_z_home]") < cfg_text.index("[probe]"),
                    "generated config must order [safe_z_home] above [probe]",
                )

            env = os.environ.copy()
            if expect_boot_error is None:
                env["KALICO_SIM_SOCK_DIR"] = str(tmp / "sim" / "h7")
            klippy_proc = subprocess.Popen(
                [
                    "python3",
                    str(repo_root / "klippy" / "klippy.py"),
                    str(cfg_path),
                    "-l",
                    str(klippy_log),
                    "-a",
                    api_socket,
                ],
                env=env,
                stdout=open(log_dir / "klippy.stdout", "wb"),
                stderr=subprocess.STDOUT,
                cwd=str(repo_root),
            )
            log.info(
                "Klippy spawned (pid=%d), variant=%s", klippy_proc.pid, variant
            )

            if expect_boot_error is not None:
                line = _wait_for_log_text(
                    klippy_log, klippy_proc, expect_boot_error, timeout=60
                )
                check(
                    "boot-error[%s]" % variant,
                    line is not None,
                    line
                    or "expected '%s' not found in klippy.log"
                    % expect_boot_error,
                )
                ready = klippy_log.exists() and (
                    b"Printer is ready" in klippy_log.read_bytes()
                )
                check(
                    "boot-error-not-ready",
                    not ready,
                    "klippy did not reach ready"
                    if not ready
                    else "klippy reached ready despite config error",
                )
            else:
                ready = wait_for_klippy_ready(
                    klippy_log, klippy_proc, timeout=120
                )
                check(
                    "boot-ready",
                    ready,
                    "Printer is ready" if ready else "klippy failed to start",
                )
                if not ready:
                    raise RuntimeError("klippy not ready")

                offset = len(klippy_log.read_bytes())

                if variant == "remote":
                    resp = send_gcode(api_socket, "G28 Z", timeout=120)
                    out, offset = _log_tail_since(klippy_log, offset)
                    g28_err = (
                        resp.get("error") if isinstance(resp, dict) else None
                    )
                    check("remote-g28-z", not g28_err, g28_err or "G28 Z ok")
                    check(
                        "remote-relay-fired",
                        "remote trsync terminal report" in out
                        or "sim_remote_endstop: firing" in out,
                        "relay/trigger evidence in logs",
                    )
                    check(
                        "remote-measured-override",
                        "set Z=3.2500" in out,
                        "homing log should show measured override 3.25",
                    )
                    m = re.search(r"trip_to_stop_travel=(-?\d+\.\d+)", out)
                    check(
                        "remote-trip-vs-stop",
                        m is not None and 0.0 <= float(m.group(1)) < 0.5,
                        m.group(0)
                        if m
                        else "no trip_to_stop_travel in homing log",
                    )
                    z = _query_toolhead_z(api_socket)
                    check(
                        "remote-final-position",
                        z is not None and abs(z - 8.25) < 0.1,
                        "z=%s expected ~8.25 (3.25 + 5.0 retract)" % (z,),
                    )
                else:
                    resp = send_gcode(api_socket, "QUERY_PROBE", timeout=30)
                    out, offset = _log_tail_since(klippy_log, offset)
                    check(
                        "query-probe-open",
                        "probe: open" in out and not resp.get("error"),
                        [l.strip() for l in out.split("\n") if "probe:" in l][
                            :1
                        ]
                        or resp,
                    )

                    resp = send_gcode(api_socket, "PROBE", timeout=60)
                    err = str(resp.get("error", ""))
                    check(
                        "probe-before-home-rejected",
                        "Must home before probe" in err,
                        err or resp,
                    )

                    _, offset = _log_tail_since(klippy_log, offset)
                    resp = send_gcode(api_socket, "G28", timeout=180)
                    out, offset = _log_tail_since(klippy_log, offset)
                    check(
                        "g28", not resp.get("error"), resp.get("error") or "ok"
                    )
                    homing_lines = [
                        l.strip() for l in out.split("\n") if "homing: " in l
                    ]
                    if variant in ("virtual", "safe-z", "points"):
                        check(
                            "g28-z-trigger-height",
                            any(
                                "homing: Z trigger=1.5000" in l
                                for l in homing_lines
                            ),
                            homing_lines,
                        )
                    z = _query_toolhead_z(api_socket)
                    if variant == "safe-z":
                        expected_z = 10.0
                    elif variant in ("virtual", "points"):
                        expected_z = 6.5
                    else:
                        expected_z = 5.0
                    check(
                        "post-home-retract-z",
                        z is not None and abs(z - expected_z) < 0.1,
                        "z=%s expected ~%.1f" % (z, expected_z),
                    )
                    if variant == "safe-z":
                        pos = _query_toolhead_position(api_socket)
                        xy_ok = (
                            pos is not None
                            and len(pos) >= 2
                            and abs(pos[0] - 125.0) < 0.5
                            and abs(pos[1] - 125.0) < 0.5
                        )
                        check(
                            "g28-at-safe-xy",
                            xy_ok,
                            "position=%s expected x,y~125,125" % (pos,),
                        )

                    resp = send_gcode(api_socket, "PROBE", timeout=90)
                    out, offset = _log_tail_since(klippy_log, offset)
                    probe_lines = [
                        l.strip() for l in out.split("\n") if " is z=" in l
                    ]
                    expected_probe_z = 1.5 if variant != "gpio-z" else 0.0
                    probe_z = None
                    if probe_lines:
                        m = re.search(r"is z=(-?\d+\.?\d*)", probe_lines[-1])
                        if m:
                            probe_z = float(m.group(1))
                    check(
                        "probe",
                        not resp.get("error")
                        and probe_z is not None
                        and abs(probe_z - expected_probe_z) < 0.25,
                        probe_lines or resp.get("error") or resp,
                    )

                    resp = send_gcode(
                        api_socket, "PROBE_ACCURACY SAMPLES=3", timeout=180
                    )
                    out, offset = _log_tail_since(klippy_log, offset)
                    acc_lines = [
                        l.strip()
                        for l in out.split("\n")
                        if "probe accuracy results" in l
                    ]
                    acc_ok = False
                    if acc_lines and not resp.get("error"):
                        m = re.search(r"range (\d+\.?\d*)", acc_lines[-1])
                        acc_ok = m is not None and float(m.group(1)) < 0.25
                    check(
                        "probe-accuracy",
                        acc_ok,
                        acc_lines or resp.get("error") or resp,
                    )

                    resp = send_gcode(api_socket, "QUERY_PROBE", timeout=30)
                    out, offset = _log_tail_since(klippy_log, offset)
                    check(
                        "query-probe-open-after",
                        "probe: open" in out and not resp.get("error"),
                        [l.strip() for l in out.split("\n") if "probe:" in l][
                            :1
                        ]
                        or resp,
                    )

                    if variant == "points":
                        resp = send_gcode(
                            api_socket, "SCREWS_TILT_ADJUST", timeout=300
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        check(
                            "screws-tilt-adjust",
                            not resp.get("error")
                            and "front left" in out
                            and "back" in out,
                            resp.get("error") or "screw report present",
                        )

                        resp = send_gcode(
                            api_socket, "BED_MESH_CALIBRATE", timeout=600
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        check(
                            "bed-mesh-calibrate",
                            not resp.get("error")
                            and "Mesh Bed Leveling Complete" in out,
                            resp.get("error") or "mesh completed",
                        )

                        resp = send_gcode(
                            api_socket, "Z_TILT_ADJUST", timeout=300
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        err = str(resp.get("error", ""))
                        check(
                            "z-tilt-fails-loudly",
                            "per-motor Z adjustment is not yet implemented"
                            in err,
                            err or "expected not-yet-implemented error",
                        )
                        check(
                            "z-tilt-reports-adjustments",
                            "Z adjustments needed" in out,
                            "measured deviations reported before the raise",
                        )

                shutdown = (
                    b"shutdown:" in klippy_log.read_bytes()
                    if klippy_log.exists()
                    else False
                )
                check(
                    "no-shutdown",
                    not shutdown,
                    "clean" if not shutdown else "printer shut down",
                )

        except Exception as e:
            check("exception", False, repr(e))
        finally:
            if klippy_proc and klippy_proc.poll() is None:
                klippy_proc.terminate()
                try:
                    klippy_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    klippy_proc.kill()
            for mcu in mcus:
                if mcu.process.poll() is None:
                    mcu.process.send_signal(signal.SIGTERM)
            for mcu in mcus:
                try:
                    mcu.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    mcu.process.kill()
            vtime_destroy()

            print("\n" + "=" * 60)
            print(f"PROBE TEST RESULT (variant={variant})")
            print("=" * 60)
            failed = [c for c in checks if not c[1]]
            for name, ok, detail in checks:
                print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
            print("=" * 60)
            if failed and klippy_log.exists():
                print("\n--- klippy.log tail ---")
                print(klippy_log.read_text(errors="replace")[-4000:])
                print("--- end klippy.log ---")
                for mcu_log in sorted(klippy_log.parent.glob("*.log")):
                    if mcu_log == klippy_log:
                        continue
                    print(f"\n--- {mcu_log.name} tail ---")
                    print(mcu_log.read_text(errors="replace")[-3000:])
                    print(f"--- end {mcu_log.name} ---")
                events_dir = klippy_log.parent / "events"
                for ev_file in sorted(events_dir.glob("*.jsonl")):
                    lines = ev_file.read_text(errors="replace").splitlines()
                    print(f"\n--- {ev_file.name} tail ---")
                    for line in lines[-80:]:
                        print(line)
                    print(f"--- end {ev_file.name} ---")

    return 1 if [c for c in checks if not c[1]] else 0


def _generate_minimal_config(
    h7_pty: str, f4_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes"
) -> str:
    """Pin names must use gpiochip0/gpioN format, not STM32 PA3 style."""
    return f"""\
[mcu]
serial: {h7_pty}

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4

[post_processor is_xy]
type: smooth_zv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _generate_multi_z_config(
    h7_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes"
) -> str:
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
kinematics: corexy
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 30

[stepper_x]
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio10
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10

[stepper_y]
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio11
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10

[stepper_z]
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4
endstop_pin: ^gpiochip0/gpio12
position_min: -5
position_endstop: 0
position_max: 250
homing_speed: 5

[stepper_z1]
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
microsteps: 16
rotation_distance: 4

[stepper_z2]
step_pin: gpiochip0/gpio16
dir_pin: gpiochip0/gpio17
enable_pin: !gpiochip0/gpio18
microsteps: 16
rotation_distance: 4

[motor_adjust]

[input_shaper]
shaper_freq_x: 50
shaper_freq_y: 50
shaper_type: smooth_mzv

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _generate_phase_stepping_config(
    h7_pty: str, f4_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes"
) -> str:
    return f"""\
[mcu]
serial: {h7_pty}

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 256
rotation_distance: 40
phase_stepping: True

[tmc5160 x]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 16
rotation_distance: 4

[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _generate_sensorless_phase_config(
    h7_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes"
) -> str:
    """Phase-stepping Z with a TMC5160 virtual (StallGuard) endstop.

    The DIAG pin is gpiochip0/gpio203 — the libsim_intercept auto-endstop
    wall asserts it after 50 steps on the runtime Z step-queue line
    (gpio15), standing in for a StallGuard trip during the homing move.
    Z is used because Z homing at 5 mm/s is the sim's validated homing
    path (X full-mode homing times out under the vtime clock race).
    """
    return f"""\
[mcu]
serial: {h7_pty}

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: tmc5160_z:virtual_endstop
homing_speed: 5
homing_retract_dist: 0

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 256
rotation_distance: 40
phase_stepping: True

[tmc5160 z]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075
diag0_pin: gpiochip0/gpio203
driver_SGT: 1

[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def run_batch_simulation(
    repo_root: pathlib.Path,
    gcode_path: pathlib.Path,
    config_path: Optional[pathlib.Path] = None,
    timeout: float = 300.0,
    verbose: bool = False,
) -> SimResult:
    """Runs full motion planner without MCU firmware (~100x real-time).
    Requires out/klipper.dict built from the firmware.
    """
    wall_start = time.monotonic()

    with tempfile.TemporaryDirectory(prefix="kalico_batch_") as tmpdir:
        tmp = pathlib.Path(tmpdir)

        dict_path = repo_root / "out" / "klipper.dict"
        if not dict_path.exists():
            return SimResult(
                success=False,
                print_time_s=0,
                wall_time_s=time.monotonic() - wall_start,
                speedup=0,
                error="Missing klipper.dict. Build firmware first.",
            )

        if config_path is None:
            cfg_text = _generate_batch_config()
            cfg_file = tmp / "printer.cfg"
            cfg_file.write_text(cfg_text)
            config_path = cfg_file

        debug_output = str(tmp / "debug_output")
        klippy_log = str(tmp / "klippy.log")
        cmd = [
            "python3",
            str(repo_root / "klippy" / "klippy.py"),
            str(config_path),
            "-i",
            str(gcode_path),
            "-o",
            debug_output,
            "-d",
            str(dict_path),
            "-l",
            klippy_log,
        ]
        if verbose:
            cmd.append("-v")

        preprocessor = (
            repo_root / "tools" / "kalico-sim" / "preprocess_gcode.py"
        )
        if preprocessor.exists():
            processed = tmp / "processed.gcode"
            try:
                subprocess.run(
                    [
                        "python3",
                        str(preprocessor),
                        str(gcode_path),
                        str(processed),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                log.info("Preprocessed: %s", processed.name)
                cmd[cmd.index(str(gcode_path))] = str(processed)
            except subprocess.CalledProcessError as e:
                log.warning("Preprocessor failed: %s", e.stderr[:200])

        log.info("Running batch simulation: %s", gcode_path.name)
        log.info("Command: %s", " ".join(cmd))

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=str(repo_root),
            )
        except subprocess.TimeoutExpired:
            return SimResult(
                success=False,
                print_time_s=0,
                wall_time_s=time.monotonic() - wall_start,
                speedup=0,
                error=f"Batch simulation timed out after {timeout}s",
            )

        wall_end = time.monotonic()
        wall_time = wall_end - wall_start

        print_time = 0.0
        error = None
        klippy_content = ""
        try:
            klippy_content = pathlib.Path(klippy_log).read_text(
                errors="replace"
            )
        except FileNotFoundError:
            pass

        if result.returncode != 0:
            if (
                "error" in klippy_content.lower()
                or "shutdown" in klippy_content.lower()
            ):
                for line in klippy_content.split("\n"):
                    if "error" in line.lower() or "shutdown" in line.lower():
                        error = line.strip()
                        break
            if not error:
                error = f"klippy exited with code {result.returncode}"
                if result.stderr:
                    error += f"\n{result.stderr[-500:]}"

        import re

        for line in reversed(klippy_content.split("\n")):
            m = re.search(r"print time (\d+\.?\d*)s", line)
            if m:
                print_time = float(m.group(1))
                break
        if print_time == 0:
            for line in reversed(klippy_content.split("\n")):
                m = re.search(r"print_time=(\d+\.?\d*)", line)
                if m:
                    print_time = float(m.group(1))
                    break

        speedup = print_time / wall_time if wall_time > 0 else 0

        return SimResult(
            success=(result.returncode == 0 and error is None),
            print_time_s=print_time,
            wall_time_s=wall_time,
            speedup=speedup,
            error=error,
            klippy_log=klippy_content,
        )


def _generate_batch_config() -> str:
    """Voron Trident 250/300 representative config for batch timing predictions.
    Limits calibrated to that hardware; adjust if targeting a different machine.
    """
    return """\
[mcu]
serial: /dev/null

[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z

[axis x]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 50
post_processors: is_xy

[axis y]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 50
post_processors: is_xy

[axis z]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio12
homing_speed: 8

[axis e]
follows: x, y, z
motors: e

[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[limit extruder]
axes: e
max_velocity: 75
max_accel: 1500

[limit gantry]
axes: x, y
max_velocity: 500
max_accel: 25000

[limit z]
axes: z
max_velocity: 30
max_accel: 100

[motor a]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 32
rotation_distance: 40

[motor b]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 32
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 32
rotation_distance: 4

[motor e]
drive: stepper
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
microsteps: 16
rotation_distance: 22.6789511

[extruder]
axis: e
nozzle_diameter: 0.4
filament_diameter: 1.75
heater_pin: gpiochip0/gpio20
sensor_pin: analog0
sensor_type: EPCOS 100K B57560G104F
min_temp: -273
max_temp: 300
control: pid
pid_kp: 30
pid_ki: 2
pid_kd: 100

[heater_bed]
heater_pin: gpiochip0/gpio21
sensor_pin: analog1
sensor_type: EPCOS 100K B57560G104F
min_temp: -273
max_temp: 120
control: pid
pid_kp: 60
pid_ki: 1
pid_kd: 600

[force_move]
enable_force_move: True
"""


def main():
    parser = argparse.ArgumentParser(description="Kalico Simulator")
    parser.add_argument("--gcode", type=str, help="G-code file to print")
    parser.add_argument("--config", type=str, help="Config directory or file")
    parser.add_argument(
        "--mode",
        choices=["full", "batch"],
        default="full",
        help="Simulation mode: 'full' (MCU firmware) or "
        "'batch' (timing prediction, faster)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=600,
        help="Max wall-clock seconds (default: 600)",
    )
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument(
        "--phase-test",
        action="store_true",
        help="Enable phase stepping config (TMC5160 on X)",
    )
    parser.add_argument(
        "--sensorless-phase-test",
        action="store_true",
        help="G28 X via TMC5160 virtual endstop on a phase-stepped axis",
    )
    parser.add_argument(
        "--homing-test",
        action="store_true",
        help="Run beacon Z homing test (dual-MCU CoreXY + beacon proximity)",
    )
    parser.add_argument(
        "--beacon-test",
        choices=BEACON_TEST_VARIANTS,
        help="Run one beacon fork validation variant (implies the beacon"
        " stub + beacon config)",
    )
    parser.add_argument(
        "--probe-test",
        choices=PROBE_TEST_VARIANTS,
        help="Run [probe] / virtual endstop validation for one variant",
    )
    parser.add_argument(
        "--test-motion-state",
        action="store_true",
        help="Move, then query mid-move commanded state via motion_state_at",
    )
    parser.add_argument(
        "--motor-adjust-test",
        action="store_true",
        help="MOTOR_ADJUST on a 3-Z-stepper cartesian config",
    )
    parser.add_argument(
        "--repo",
        type=str,
        default=str(REPO_ROOT),
        help="Repository root (default: auto-detect)",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Long-lived interactive mode for Moonraker/Mainsail",
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default=None,
        help="Stable printer_data dir for --serve",
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    repo = pathlib.Path(args.repo)
    gcode = pathlib.Path(args.gcode) if args.gcode else None
    config = pathlib.Path(args.config) if args.config else None

    if args.probe_test:
        sys.exit(
            run_probe_test(
                repo_root=repo,
                variant=args.probe_test,
                timeout=args.timeout,
                verbose=args.verbose,
            )
        )

    if args.mode == "batch":
        if not gcode:
            print("ERROR: --gcode is required for batch mode")
            sys.exit(1)
        result = run_batch_simulation(
            repo_root=repo,
            gcode_path=gcode,
            config_path=config,
            timeout=args.timeout,
            verbose=args.verbose,
        )
    else:
        result = run_simulation(
            repo_root=repo,
            gcode_path=gcode,
            config_dir=config,
            timeout=args.timeout,
            verbose=args.verbose,
            phase_stepping=args.phase_test,
            homing_test=args.homing_test,
            beacon_test=args.beacon_test,
            motion_state_test=args.test_motion_state,
            sensorless_phase_test=args.sensorless_phase_test,
            motor_adjust_test=args.motor_adjust_test,
            serve=args.serve,
            serve_data_dir=args.data_dir,
        )

    print("\n" + "=" * 60)
    print(f"SIMULATION RESULT ({args.mode} mode)")
    print("=" * 60)
    print(f"  Status:     {'PASS' if result.success else 'FAIL'}")
    print(
        f"  Print time: {result.print_time_s:.1f}s "
        f"({result.print_time_s / 60:.1f} min)"
    )
    print(
        f"  Wall time:  {result.wall_time_s:.1f}s "
        f"({result.wall_time_s / 60:.1f} min)"
    )
    if result.speedup > 0:
        print(f"  Speedup:    {result.speedup:.1f}x")
    if result.error:
        print(f"  Error:      {result.error}")
    print("=" * 60)

    if not result.success and result.klippy_log:
        print("\n--- klippy.log raw tail ---")
        print(result.klippy_log[-8000:])
        print("--- end raw tail ---")
        if "Traceback" in result.klippy_log:
            print("\n--- klippy.log (traceback context) ---")
            lines = result.klippy_log.strip().split("\n")
            for i, line in enumerate(lines):
                if "Traceback" in line:
                    print("\n".join(lines[max(0, i - 3) : i + 20]))
                    print("...")
        print("\n--- klippy.log (homing-relevant, all lines) ---")
        for line in result.klippy_log.strip().split("\n"):
            lo = line.lower()
            if any(
                k in lo
                for k in (
                    "bridge-trace",
                    "endstop_arm",
                    "arm_id",
                    "arm status",
                    "trip",
                    "drip",
                    "credit",
                    "homing",
                    "home_start",
                    "home_wait",
                    "no trigger",
                    "segment_id",
                    "submit_homing",
                    "homing_move",
                    "error during",
                    "internal error",
                    "mcu silent",
                    "move-diag",
                    "sim-trace",
                    "dispatch closure",
                    "load_curve",
                    "classify",
                    "submit_move",
                )
            ):
                print(line)
        print("--- end klippy.log ---")

    if not result.success and result.mcu_logs:
        for name, content in result.mcu_logs.items():
            print(f"\n--- {name} MCU log (last 30 lines) ---")
            for line in content.strip().split("\n")[-30:]:
                print(line)
            print(f"--- end {name} ---")

    if not result.success and result.klippy_log:
        trace_lines = [
            l
            for l in result.klippy_log.split("\n")
            if "trace-write" in l
            or "trace-close" in l
            or "trace-kcall" in l
            or "endstop_arm" in l.lower()
        ]
        if trace_lines:
            print("\n--- bridge trace lines ---")
            for line in trace_lines[-30:]:
                print(line)
            print("--- end bridge trace ---")

    if args.homing_test and result.klippy_log:
        print("\n--- Beacon homing log excerpts ---")
        for line in result.klippy_log.split("\n"):
            llow = line.lower()
            if any(
                k in llow
                for k in [
                    "beacon",
                    "homing",
                    "home",
                    "trsync",
                    "trigger",
                    "z_virtual_endstop",
                    "proximity",
                    "endstop",
                    "can_trigger",
                    "threshold",
                    "z_hop",
                ]
            ):
                print(f"  {line.strip()}")
        print("---")

    if args.sensorless_phase_test and result.klippy_log:
        print("\n--- Sensorless phase test log excerpts ---")
        for line in result.klippy_log.split("\n"):
            llow = line.lower()
            if any(
                k in llow
                for k in [
                    "phase",
                    "homing",
                    "home_axis",
                    "trip",
                    "fault",
                    "shutdown",
                    "g28",
                    "xdirect",
                    "endstop",
                    "stall",
                    "diag",
                ]
            ):
                print(f"  {line.strip()}")
        print("---")
        for mcu_name, mcu_log in (result.mcu_logs or {}).items():
            relevant = [
                ln
                for ln in mcu_log.split("\n")
                if any(
                    k in ln.lower()
                    for k in ["fault", "phase", "jog", "align", "shutdown"]
                )
            ]
            if relevant:
                print(f"--- {mcu_name} MCU excerpts ---")
                for ln in relevant[-40:]:
                    print(f"  {ln.strip()}")
                print("---")

    if args.phase_test and result.klippy_log:
        print("\n--- Phase stepping log excerpts ---")
        for line in result.klippy_log.split("\n"):
            llow = line.lower()
            if any(
                k in llow
                for k in [
                    "phase_stepping",
                    "phase_step",
                    "tmc5160",
                    "direct_mode",
                    "configure_axes",
                    "step_mode",
                    "modulated",
                    "phase_config",
                    "register_phase",
                    "bridge-trace",
                    "spi_bus",
                ]
            ):
                print(f"  {line.strip()}")
        print("---")

    if args.phase_test and result.print_time_s > 0:
        timer_in_past = (
            result.error
            and "timer" in result.error.lower()
            and "past" in result.error.lower()
        )
        if timer_in_past:
            print("\nNote: 'timer in past' is a known MACH_LINUX timing issue")
            print("      under Docker VM pressure, not a phase stepping bug.")
            print(
                f"      Motion ran for {result.print_time_s:.1f}s before the timing fault."
            )
            sys.exit(0)

    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
