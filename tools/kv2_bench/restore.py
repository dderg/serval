"""Leave the bench as the session found it, and prove it.

The serval F401 firmware is rebuilt from the commit the Pi's deploy record
names (a detached worktree, RESTORE_WORKTREE) and flashed with serval's own
`deploy.py flash-target`, which refuses a board that does not report the
build id; the kv2 drop-ins go away and the serval klippy runs again.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from . import bench, firmware
from .serval_tools import BENCH, TREE, firmware_build_id

RESTORE_WORKTREE = Path("~/Developer/kv2-bench-fw").expanduser()
IDENTITY_KEYS = (
    "head",
    "branch",
    "diff_sha256",
    "rust_build_id",
    "dirty_paths",
    "worktree",
)


def prepare(pre: dict) -> Path:
    """Check out the recorded firmware's commit and require it to hash to the
    recorded build id before anything on the bench changes."""
    target = pre["deploy_record"]["firmware"]["target"]
    firmware.checkout(RESTORE_WORKTREE, target["head"])
    built = firmware_build_id.target_build_id(
        RESTORE_WORKTREE, RESTORE_WORKTREE / "tools" / "bench" / "f401.config"
    )
    if built != target["build_id"]:
        raise bench.BenchError(
            f"{RESTORE_WORKTREE} at {target['head']} hashes to {built}, the Pi "
            f"records firmware {target['build_id']}: cannot restore it"
        )
    return RESTORE_WORKTREE


def state_report() -> str:
    return subprocess.run(
        ["uv", "run", "python", str(BENCH / "state.py")],
        cwd=TREE,
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def differences(pre: dict, post: dict) -> list[str]:
    problems = []
    for key in IDENTITY_KEYS:
        if pre["deploy_record"].get(key) != post["deploy_record"].get(key):
            problems.append(f"deploy record {key}")
    for board in ("target", "capture"):
        before = pre["deploy_record"]["firmware"][board]["build_id"]
        after = post["deploy_record"]["firmware"][board]["build_id"]
        if before != after:
            problems.append(f"{board} firmware {before} -> {after}")
    for key in (
        "service_active",
        "exec_start",
        "dropins",
        "klipper_cmdline",
        "printer_state",
        "mcu_version",
        "printer_cfg_sha256",
        "scaling_max_freq",
    ):
        if pre[key] != post[key]:
            problems.append(f"{key}: {pre[key]!r} -> {post[key]!r}")
    return problems


def restore(pre: dict, tree: Path, log: Path) -> dict:
    bench.deactivate()
    with open(log, "w") as out:
        subprocess.run(
            [
                "python3",
                str(firmware.SLOTS),
                "run",
                "--",
                "uv",
                "run",
                "python",
                str(BENCH / "deploy.py"),
                "flash-target",
                "--tree",
                str(tree),
            ],
            cwd=TREE,
            check=True,
            stdout=out,
            stderr=subprocess.STDOUT,
        )
    post = bench.snapshot_state()
    problems = differences(pre, post)
    return {
        "post_state": post,
        "differences": problems,
        "state_py": state_report(),
    }
