"""Deploy and flash tooling for the bench.

uv run python tools/bench/deploy.py sync [--tree DIR]
uv run python tools/bench/deploy.py flash-target [--tree DIR]
uv run python tools/bench/deploy.py flash-capture [--tree DIR]
uv run python tools/bench/deploy.py config [--config CFG]
uv run python tools/bench/deploy.py reset-target
uv run python tools/bench/deploy.py mco

Firmware builds inside the tree they come from (out/bench-f401 for the F401,
tools/bench/capture-fw/target for the F407), never in a directory another
tree shares. Each flash bakes the tree's firmware build id
(firmware_build_id.py) into the image, requires the board to report it back,
and records it under "firmware" in the deploy record, which run.py copies
into every run.json; a sync keeps the boards' entries.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import capture
import firmware_build_id
import lock
import pi

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "snapshots"))
import engine_build_id  # noqa: E402

BENCH = Path("tools/bench")
TARGET_CONFIG = BENCH / "f401.config"
TARGET_OUT = Path("out/bench-f401")
TARGET_BOARD = "target"
CAPTURE_FW = BENCH / "capture-fw"
CAPTURE_TARGET_DIR = CAPTURE_FW / "target"
CAPTURE_ELF = Path("thumbv7em-none-eabihf/release/bench-capture-fw")
CAPTURE_BOARD = "capture"
CAPTURE_BUILD_ID_ENV = "CAPTURE_FW_BUILD_ID"
TARGET_VERSION_TAG = "-fw"
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


def git_output(tree: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=tree, check=True, capture_output=True, text=True
    ).stdout


def deploy_identity(tree: Path) -> dict:
    diff = subprocess.run(
        ["git", "diff", "HEAD"], cwd=tree, check=True, capture_output=True
    ).stdout
    dirty = [
        line[3:]
        for line in git_output(tree, "status", "--porcelain").splitlines()
    ]
    return {
        "head": git_output(tree, "rev-parse", "HEAD").strip(),
        "branch": git_output(tree, "rev-parse", "--abbrev-ref", "HEAD").strip(),
        "diff_sha256": hashlib.sha256(diff).hexdigest(),
        "rust_build_id": engine_build_id.tree_build_id(tree),
        "dirty_paths": dirty,
        "timestamp": datetime.now().astimezone().isoformat(),
        "worktree": str(tree),
    }


def local_files(tree: Path) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z", "-co", "--exclude-standard"],
        cwd=tree,
        check=True,
        capture_output=True,
    ).stdout
    names = out.decode().split("\0")
    return sorted({n for n in names if n and os.path.lexists(tree / n)})


def rsync_changes(tree: Path, files: list[str]) -> list[str]:
    result = subprocess.run(
        [
            "rsync",
            "-rlpgoD",
            "--checksum",
            "-i",
            "--from0",
            "--files-from=-",
            "./",
            f"{pi.PI_SSH}:klipper/",
        ],
        cwd=tree,
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


BUILD_OUTPUT_DIRS = ("rust/target/", ".git/")
BUILD_OUTPUT_SUFFIXES = (".so", ".pyc")
BUILD_OUTPUT_FILES = (".bench-deploy.json",)
REMOTE_TREE_LISTING = (
    "cd klipper && find . -path ./.git -prune -o -path ./rust/target -prune "
    "-o \\( -type f -o -type l \\) -print"
)


def is_build_output(name: str) -> bool:
    return (
        name in BUILD_OUTPUT_FILES
        or name.startswith(BUILD_OUTPUT_DIRS)
        or name.endswith(BUILD_OUTPUT_SUFFIXES)
        or "/__pycache__/" in name
    )


def stale_remote_files(remote: list[str], local: list[str]) -> list[str]:
    return sorted(
        name for name in set(remote) - set(local) if not is_build_output(name)
    )


def cmd_sync(tree: Path) -> None:
    files = local_files(tree)
    changed = rsync_changes(tree, files)
    remote = [
        line.removeprefix("./")
        for line in pi.ssh(REMOTE_TREE_LISTING).splitlines()
        if line
    ]
    stale = stale_remote_files(remote, files)
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
    pi.write_deployed(
        {**deploy_identity(tree), "firmware": pi.flashed_firmware()}
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


def firmware_identity(tree: Path, build_id: str, elf: Path) -> dict:
    return {
        "build_id": build_id,
        "worktree": str(tree),
        "head": git_output(tree, "rev-parse", "HEAD").strip(),
        "elf": str(elf),
        "elf_sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
        "timestamp": datetime.now().astimezone().isoformat(),
    }


def require_reported(board: str, reported: str, build_id: str) -> None:
    if not reported.endswith(build_id):
        raise pi.BenchError(
            f"the {board} board reports firmware {reported!r} after flashing, "
            f"not the build {build_id} this tree just made"
        )


def cmd_flash_target(tree: Path) -> None:
    """Build the F401 firmware in this tree's own output directory (another
    tree's objects and dependency files never mix in), flash it, and require
    the mcu to report the build id baked into its version."""
    out = tree / TARGET_OUT
    out.mkdir(parents=True, exist_ok=True)
    make = ["make", f"KCONFIG_CONFIG={tree / TARGET_CONFIG}", f"OUT={out}/"]
    run_checked([*make, "olddefconfig"], cwd=tree)
    build_id = firmware_build_id.target_build_id(tree, tree / TARGET_CONFIG)
    (out / "compile_time_request.o").unlink(missing_ok=True)
    run_checked(
        [
            *make,
            f"VERSION_EXTRA={TARGET_VERSION_TAG}{build_id}",
            f"-j{os.cpu_count()}",
        ],
        cwd=tree,
    )
    elf = out / "klipper.elf"
    pi.scp_to_pi(elf, PI_ELF)
    output = pi.ssh(
        f"openocd -f ~/klipper/tools/bench/openocd-target.cfg"
        f' -c "program {PI_ELF} verify reset exit" 2>&1'
    )
    print(output)
    require_verified(output)
    wait_pi_serial()
    pi.firmware_restart()
    require_reported(TARGET_BOARD, pi.mcu_version(), build_id)
    pi.record_firmware(TARGET_BOARD, firmware_identity(tree, build_id, elf))
    pi.enable_mco()
    print(f"target flashed with firmware {build_id}, MCO1 on, printer ready")


def cmd_reset_target() -> None:
    output = pi.ssh(
        "openocd -f ~/klipper/tools/bench/openocd-target.cfg"
        ' -c "init; reset; exit" 2>&1'
    )
    print(output)
    wait_pi_serial()
    pi.restart_klipper()
    pi.enable_mco()
    print("target reset, MCO1 on, printer ready")


def cmd_mco() -> None:
    pi.enable_mco()
    print("F401 MCO1 = HSE/4 on PA8")


def cmd_flash_capture(tree: Path) -> None:
    """Build the F407 capture firmware in this tree's own cargo target dir,
    whatever CARGO_TARGET_DIR the caller exports, flash it, and require the
    board to report the build id it was built with."""
    build_id = firmware_build_id.capture_build_id(tree)
    target_dir = tree / CAPTURE_TARGET_DIR
    env = {
        **os.environ,
        "PATH": f"{CARGO_BIN}:{os.environ['PATH']}",
        "CARGO_TARGET_DIR": str(target_dir),
        CAPTURE_BUILD_ID_ENV: build_id,
    }
    run_checked(["cargo", "build", "--release"], cwd=tree / CAPTURE_FW, env=env)
    elf = target_dir / CAPTURE_ELF
    output = run_checked(
        [
            "openocd",
            "-f",
            str(tree / BENCH / "openocd-capture.cfg"),
            "-c",
            f"program {elf} verify reset exit",
        ],
        env=env,
    )
    require_verified(output)
    deadline = time.monotonic() + SERIAL_TIMEOUT_S
    while not os.path.exists(capture.DEVICE):
        if time.monotonic() > deadline:
            raise pi.BenchError(f"{capture.DEVICE} did not appear")
        time.sleep(0.5)
    require_reported(CAPTURE_BOARD, capture.identity(), build_id)
    pi.record_firmware(CAPTURE_BOARD, firmware_identity(tree, build_id, elf))
    print(f"capture board flashed with firmware {build_id}, device present")


def cmd_config(config: Path) -> None:
    pi.validate_config(config.read_text())
    print("config valid")
    print(
        "config pushed" if pi.push_config(config) else "config unchanged on Pi"
    )
    pi.firmware_restart()
    print("printer ready")


TREE_COMMANDS = {
    "flash-target": cmd_flash_target,
    "flash-capture": cmd_flash_capture,
}
COMMANDS = {
    "reset-target": cmd_reset_target,
    "mco": cmd_mco,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=["sync", "config", *TREE_COMMANDS, *COMMANDS]
    )
    parser.add_argument("--tree", type=Path, default=REPO)
    parser.add_argument(
        "--config",
        type=Path,
        default=pi.REPO_CFG,
        help="printer config the config command pushes (e.g. configs/full.cfg)",
    )
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    with lock.bench_lock(args.lock_timeout):
        if args.command == "sync":
            cmd_sync(args.tree.resolve())
        elif args.command == "config":
            cmd_config(args.config)
        elif args.command in TREE_COMMANDS:
            TREE_COMMANDS[args.command](args.tree.resolve())
        else:
            COMMANDS[args.command]()


if __name__ == "__main__":
    main()
