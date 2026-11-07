"""Pi and board operations for kv2 bench sessions.

kv2 never touches the serval install: klippy trees live under ~/kv2/trees on
the Pi with their own venv (~/kv2/env), config (~/kv2/printer.cfg) and log
(~/kv2/logs/klippy.log). A systemd drop-in (KV2_DROPIN) points the klipper
service at them; Moonraker keeps talking to the same API socket.
"""

from __future__ import annotations

import hashlib
import json
import re
import shlex
import subprocess
import time
import urllib.parse
from pathlib import Path

from .serval_tools import BENCH as SERVAL_BENCH
from .serval_tools import pi

KV2_ROOT = Path(__file__).resolve().parents[2]
REPO = Path("~/Developer/serval").expanduser()
PI_HOME = "/home/pi"
PI_KV2 = f"{PI_HOME}/kv2"
PI_ENV = f"{PI_KV2}/env"
PI_CONFIG = f"{PI_KV2}/printer.cfg"
PI_LOG = f"{PI_KV2}/logs/klippy.log"
PI_AGENT = f"{PI_KV2}/pi_agent.py"
PI_OPENOCD_CFG = f"{PI_KV2}/openocd-target.cfg"
PI_FIRMWARE_ELF = f"{PI_KV2}/kv2-f401.elf"
PI_GCODES = f"{PI_HOME}/printer_data/gcodes"
DROPIN_DIR = "/etc/systemd/system/klipper.service.d"
KV2_DROPIN = f"{DROPIN_DIR}/kv2-host.conf"
QUOTA_DROPIN = f"{DROPIN_DIR}/kv2-quota.conf"
SERIAL_GLOB = "/dev/serial/by-id/usb-Klipper_stm32f401xc_*"
READY_TIMEOUT_S = 120.0
SERIAL_TIMEOUT_S = 30.0
EXEC_ARGV = re.compile(r"argv\[\]=([^;]*);")


class BenchError(RuntimeError):
    pass


def ssh(command: str) -> str:
    return pi.ssh(command)


def git(*args: str, cwd: Path = REPO) -> str:
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def resolve(ref: str) -> str:
    return git("rev-parse", "--verify", f"{ref}^{{commit}}")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_dir(commit: str) -> str:
    return f"{PI_KV2}/trees/{commit[:12]}"


def install_tree(commit: str) -> str:
    """Extract the committed tree of `commit` (no worktree edits) on the Pi
    and build its chelper there; idempotent per commit."""
    remote = tree_dir(commit)
    marker = f"{remote}/.kv2-bench-installed"
    if ssh(f"test -e {marker} && echo yes || true").strip() == "yes":
        return remote
    archive = subprocess.run(
        ["git", "archive", "--format=tar", commit],
        cwd=REPO,
        check=True,
        capture_output=True,
    ).stdout
    subprocess.run(
        [
            "ssh",
            pi.PI_SSH,
            f"rm -rf {remote} && mkdir -p {remote} && tar -x -C {remote}",
        ],
        input=archive,
        check=True,
    )
    ensure_env(remote)
    ssh(
        f"cd {remote} && {PI_ENV}/bin/python -c "
        "'from klippy import chelper; chelper.get_ffi()'"
    )
    ssh(f"echo {commit} > {marker}")
    return remote


def ensure_env(remote_tree: str) -> None:
    requirements = f"{remote_tree}/scripts/klippy-requirements.txt"
    digest = ssh(f"sha256sum {requirements}").split()[0]
    marker = f"{PI_ENV}/.requirements-{digest[:16]}"
    if ssh(f"test -e {marker} && echo yes || true").strip() == "yes":
        return
    ssh(
        f"test -x {PI_ENV}/bin/python || python3 -m venv {PI_ENV}; "
        f"{PI_ENV}/bin/pip install -q -r {requirements} && touch {marker}"
    )


def install_agent() -> None:
    ssh(f"rm -rf {PI_KV2}/runs && mkdir -p {PI_KV2}/logs {PI_KV2}/runs")
    pi.scp_to_pi(Path(__file__).with_name("pi_agent.py"), PI_AGENT)
    pi.scp_to_pi(SERVAL_BENCH / "openocd-target.cfg", PI_OPENOCD_CFG)


def klipper_cmdline() -> str:
    pid = pi.klipper_pid()
    if not pid:
        raise BenchError("klipper service has no main pid")
    return ssh(f"tr '\\0' ' ' < /proc/{pid}/cmdline")


def wait_state(
    accepted: tuple[str, ...], timeout_s: float = READY_TIMEOUT_S
) -> str:
    deadline = time.monotonic() + timeout_s
    state = "unreachable"
    while time.monotonic() < deadline:
        try:
            state = pi.printer_state()
        except OSError:
            state = "unreachable"
        if state in accepted:
            return state
        time.sleep(1)
    info = ""
    try:
        info = pi.http("GET", "/printer/info").get("state_message", "")
    except OSError:
        pass
    raise BenchError(
        f"printer not {'/'.join(accepted)} within {timeout_s:.0f} s "
        f"(state {state}): {info}"
    )


def wait_ready(timeout_s: float = READY_TIMEOUT_S) -> None:
    wait_state(("ready",), timeout_s)


def restart_service() -> None:
    ssh("sudo systemctl restart klipper")
    time.sleep(2)


def restart_ready() -> str:
    """Restart klippy; an MCU latched in shutdown by an earlier run is reset
    with FIRMWARE_RESTART. Returns the state klippy first settled in."""
    restart_service()
    settled = wait_state(("ready", "shutdown", "error"))
    if settled != "ready":
        pi.firmware_restart()
        wait_ready()
    return settled


def activate(remote_tree: str, config: Path) -> None:
    """Point the klipper service at `remote_tree` with `config` and restart it
    with a fresh log. Readiness is the caller's: the board may still run
    other firmware."""
    pi.scp_to_pi(config, PI_CONFIG)
    exec_start = (
        f"{PI_ENV}/bin/python {remote_tree}/klippy/klippy.py {PI_CONFIG} "
        f"-I {PI_HOME}/printer_data/comms/klippy.serial -l {PI_LOG} "
        "-a /tmp/klippy_uds"
    )
    dropin = f"[Service]\\nExecStart=\\nExecStart={exec_start}\\n"
    ssh(
        f"sudo mkdir -p {DROPIN_DIR} && printf '{dropin}' "
        f"| sudo tee {KV2_DROPIN} >/dev/null && sudo systemctl daemon-reload "
        f"&& : > {PI_LOG}"
    )
    restart_service()
    cmdline = klipper_cmdline()
    if f"{remote_tree}/klippy/klippy.py" not in cmdline:
        raise BenchError(f"klipper runs {cmdline!r}, not {remote_tree}")


def push_config(config: Path) -> None:
    pi.scp_to_pi(config, PI_CONFIG)


def deactivate() -> None:
    ssh(
        f"sudo rm -f {KV2_DROPIN} {QUOTA_DROPIN} && sudo systemctl daemon-reload"
    )
    restart_service()


def set_quota(percent: int | None) -> None:
    if percent is None:
        ssh(f"sudo rm -f {QUOTA_DROPIN} && sudo systemctl daemon-reload")
        return
    ssh(
        f"printf '[Service]\\nCPUQuota={percent}%%\\n' | sudo tee {QUOTA_DROPIN} "
        ">/dev/null && sudo systemctl daemon-reload"
    )


def mcu_version() -> str:
    return pi.mcu_version()


def flash_kv2_firmware(elf: Path, expected_version: str) -> dict:
    """Program the F401 over the Pi's SWD with the stock kv2 build and require
    the running kv2 klippy to report its version afterwards."""
    pi.scp_to_pi(elf, PI_FIRMWARE_ELF)
    output = ssh(
        f"openocd -f {PI_OPENOCD_CFG} "
        f'-c "program {PI_FIRMWARE_ELF} verify reset exit" 2>&1'
    )
    if "Verified OK" not in output:
        raise BenchError(f"openocd did not verify the image:\n{output}")
    deadline = time.monotonic() + SERIAL_TIMEOUT_S
    while not ssh(f"ls {SERIAL_GLOB} 2>/dev/null || true").strip():
        if time.monotonic() > deadline:
            raise BenchError(f"{SERIAL_GLOB} did not appear after flashing")
        time.sleep(1)
    restart_ready()
    reported = mcu_version()
    if reported != expected_version:
        raise BenchError(
            f"F401 reports {reported!r}, flashed build is {expected_version!r}"
        )
    return {"elf_sha256": sha256(elf), "version": reported}


def upload(name: str, text: str) -> None:
    pi.upload(name, text.encode())


def remove_upload(name: str) -> None:
    ssh(f"rm -f {PI_GCODES}/{shlex.quote(name)}")


def start_print(name: str) -> None:
    pi.http("POST", f"/printer/print/start?filename={urllib.parse.quote(name)}")


def print_stats() -> dict:
    return pi.http("GET", "/printer/objects/query?print_stats")["status"][
        "print_stats"
    ]


def snapshot_state() -> dict:
    """Everything a session must leave as it found it."""
    policies = ssh("ls -d /sys/devices/system/cpu/cpufreq/policy*").split()
    info = pi.http("GET", "/printer/info")
    return {
        "deploy_record": json.loads(ssh("cat ~/klipper/.bench-deploy.json")),
        "service_active": ssh("systemctl is-active klipper || true").strip(),
        "exec_start": EXEC_ARGV.search(
            ssh("systemctl show -p ExecStart --value klipper")
        )
        .group(1)
        .strip(),
        "dropins": sorted(ssh(f"ls {DROPIN_DIR} 2>/dev/null || true").split()),
        "klipper_cmdline": klipper_cmdline(),
        "printer_state": info["state"],
        "mcu_version": mcu_version() if info["state"] == "ready" else None,
        "printer_cfg_sha256": ssh(
            "sha256sum ~/printer_data/config/printer.cfg"
        ).split()[0],
        "scaling_max_freq": {
            p: ssh(f"cat {p}/scaling_max_freq").strip() for p in policies
        },
    }
