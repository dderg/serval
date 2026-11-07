"""The serval bench instrument tools, imported from a serval worktree.

The F407 capture firmware, its host reader and decoder, the bench lock and
the Pi helpers are independent of kalico, so kv2 reuses them as they are.
KV2_BENCH_SERVAL selects the worktree (default ~/Developer/kv2-bench-serval).
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

TREE = Path(
    os.environ.get("KV2_BENCH_SERVAL", "~/Developer/kv2-bench-serval")
).expanduser()
BENCH = TREE / "tools" / "bench"
sys.path.insert(0, str(BENCH))

import capture  # noqa: E402
import capture_events  # noqa: E402
import firmware_build_id  # noqa: E402
import lock  # noqa: E402
import pi  # noqa: E402
import throttle  # noqa: E402

__all__ = [
    "capture",
    "capture_events",
    "firmware_build_id",
    "lock",
    "pi",
    "throttle",
    "head",
]


TRANSIENT_SSH = ("Could not resolve hostname", "Connection timed out")
SSH_ATTEMPTS = 5
SSH_RETRY_S = 3.0
_pi_ssh = pi.ssh


def _ssh_with_retry(command: str) -> str:
    """bench-pi resolves over mDNS (bench-pi.local), which failed once
    transiently; only those failures are retried, any other is raised."""
    for attempt in range(SSH_ATTEMPTS):
        try:
            return _pi_ssh(command)
        except pi.BenchError as exc:
            if attempt + 1 == SSH_ATTEMPTS or not any(
                text in str(exc) for text in TRANSIENT_SSH
            ):
                raise
            time.sleep(SSH_RETRY_S)
    raise AssertionError("unreachable")


pi.ssh = _ssh_with_retry


def head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=TREE,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
