"""Deploy and flash tooling for the bench.

uv run python tools/bench/deploy.py sync
uv run python tools/bench/deploy.py flash-target
uv run python tools/bench/deploy.py flash-capture
uv run python tools/bench/deploy.py config
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import lock
import pi
from capture import DEVICE as CAPTURE_DEVICE

REPO = Path(__file__).resolve().parents[2]
BENCH = REPO / "tools" / "bench"
TARGET_CONFIG = BENCH / "f401.config"
TARGET_OUT = Path("~/bench/fw/f401").expanduser()
CAPTURE_FW = BENCH / "capture-fw"
CAPTURE_ELF = (
    CAPTURE_FW / "target/thumbv7em-none-eabihf/release/bench-capture-fw"
)
CARGO_BIN = Path("~/.cargo/bin").expanduser()
TARGET_SERIAL_GLOB = "/dev/serial/by-id/usb-Klipper_stm32f401xc_*"
PI_ELF = "/tmp/bench-target.elf"
SERIAL_TIMEOUT_S = 30.0
VERIFIED = "Verified OK"


def run_checked(
    args: list[str], cwd: Path | None = None, env: dict | None = None
) -> str:
    print("+", " ".join(args), flush=True)
    result = subprocess.run(
        args, cwd=cwd, env=env, capture_output=True, text=True
    )
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)
    if result.returncode != 0:
        raise pi.BenchError(
            f"{args[0]} failed with exit code {result.returncode}"
        )
    return result.stdout + result.stderr


def require_verified(output: str) -> None:
    if VERIFIED not in output:
        raise pi.BenchError(f"flash output does not contain {VERIFIED!r}")


def local_files() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z", "-co", "--exclude-standard"],
        cwd=REPO,
        check=True,
        capture_output=True,
    ).stdout
    names = out.decode().split("\0")
    return sorted({n for n in names if n and os.path.lexists(REPO / n)})


def rsync_changes(files: list[str]) -> list[str]:
    result = subprocess.run(
        [
            "rsync",
            "-a",
            "-i",
            "--from0",
            "--files-from=-",
            "./",
            f"{pi.PI_SSH}:klipper/",
        ],
        cwd=REPO,
        input="\0".join(files).encode(),
        capture_output=True,
    )
    if result.returncode != 0:
        raise pi.BenchError(f"rsync failed:\n{result.stderr.decode()}")
    changed = []
    for line in result.stdout.decode().splitlines():
        flags, _, name = line.partition(" ")
        if flags[1:2] == "f" and flags[0] in "<>c":
            changed.append(name)
    return changed


def cmd_sync() -> None:
    files = local_files()
    changed = rsync_changes(files)
    stale = sorted(
        set(pi.ssh("cd klipper && git ls-files").splitlines()) - set(files)
    )
    if stale:
        quoted = " ".join(f"'{name}'" for name in stale)
        pi.ssh(f"cd klipper && rm -f -- {quoted}")
    print(f"synced: {len(changed)} files changed, {len(stale)} stale removed")
    if any(name.startswith("rust/") for name in changed + stale):
        print("rust/ changed: building native on the Pi", flush=True)
        build = subprocess.run(
            [
                "ssh",
                pi.PI_SSH,
                "cd klipper && . ~/.cargo/env && ./scripts/build-native.sh",
            ],
        )
        if build.returncode != 0:
            raise pi.BenchError(
                f"build-native.sh failed with exit code {build.returncode}"
            )
    if changed or stale:
        before = pi.klipper_pid()
        pi.restart_klipper()
        print(f"klipper restarted: pid {before} -> {pi.klipper_pid()}")


def wait_pi_serial() -> None:
    deadline = time.monotonic() + SERIAL_TIMEOUT_S
    while time.monotonic() < deadline:
        if pi.ssh(f"ls {TARGET_SERIAL_GLOB} 2>/dev/null || true").strip():
            return
        time.sleep(1)
    raise pi.BenchError(f"{TARGET_SERIAL_GLOB} did not appear on the Pi")


def cmd_flash_target() -> None:
    TARGET_OUT.mkdir(parents=True, exist_ok=True)
    make = ["make", f"KCONFIG_CONFIG={TARGET_CONFIG}", f"OUT={TARGET_OUT}/"]
    run_checked([*make, "olddefconfig"], cwd=REPO)
    run_checked([*make, f"-j{os.cpu_count()}"], cwd=REPO)
    pi.scp_to_pi(TARGET_OUT / "klipper.elf", PI_ELF)
    output = pi.ssh(
        f"openocd -f ~/klipper/tools/bench/openocd-target.cfg"
        f' -c "program {PI_ELF} verify reset exit" 2>&1'
    )
    print(output)
    require_verified(output)
    wait_pi_serial()
    pi.firmware_restart()
    print("target flashed, printer ready")


def cmd_flash_capture() -> None:
    env = {**os.environ, "PATH": f"{CARGO_BIN}:{os.environ['PATH']}"}
    run_checked(["cargo", "build", "--release"], cwd=CAPTURE_FW, env=env)
    output = run_checked(
        [
            "openocd",
            "-f",
            str(BENCH / "openocd-capture.cfg"),
            "-c",
            f"program {CAPTURE_ELF} verify reset exit",
        ],
        env=env,
    )
    require_verified(output)
    deadline = time.monotonic() + SERIAL_TIMEOUT_S
    while not os.path.exists(CAPTURE_DEVICE):
        if time.monotonic() > deadline:
            raise pi.BenchError(f"{CAPTURE_DEVICE} did not appear")
        time.sleep(0.5)
    print("capture board flashed, device present")


def cmd_config() -> None:
    pi.validate_config(pi.REPO_CFG.read_text())
    print("config valid")
    print("config pushed" if pi.push_config() else "config unchanged on Pi")
    pi.firmware_restart()
    print("printer ready")


COMMANDS = {
    "sync": cmd_sync,
    "flash-target": cmd_flash_target,
    "flash-capture": cmd_flash_capture,
    "config": cmd_config,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=COMMANDS)
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    with lock.bench_lock(args.lock_timeout):
        COMMANDS[args.command]()


if __name__ == "__main__":
    main()
