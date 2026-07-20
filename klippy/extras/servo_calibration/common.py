from __future__ import annotations

import os
import re
import subprocess
import time
from collections.abc import Mapping
from typing import Any

ApplyResult = tuple[Mapping[str, float], list[dict[str, Any]]]
VERDICT_ABORT_FLAGS = frozenset({"torque_saturated", "resonance_detected"})


REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
DEFAULT_CAPTURES_ROOT = "~/printer_data/logs/servo_captures"
DEFAULT_DYNAMICS_DIR = "~/printer_data/config/servo_dynamics"

_git_rev_cache: str | None = None


def _git_rev() -> str:
    global _git_rev_cache
    if _git_rev_cache is None:
        try:
            _git_rev_cache = (
                subprocess.check_output(
                    ["git", "rev-parse", "--short", "HEAD"],
                    cwd=REPO_ROOT,
                    stderr=subprocess.DEVNULL,
                )
                .decode()
                .strip()
            )
        except Exception:
            _git_rev_cache = "unknown"
    return _git_rev_cache


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _applied(servo: str, addr: str, value: int) -> dict[str, Any]:
    return {"servo": servo, "addr": addr, "type": "u16", "value": value}


_C0006_RE = re.compile(r"recommended C00\.06 \(light direction\):\s*(-?\d+)%")


def _parse_c0006_recommendation(text: str) -> int | None:
    """servo-cal fit prints the C00.06 pick to stdout/stderr (no JSON
    field carries it - profile_out::render_profile never emits it); the
    console stream servo_calibration already captures is the cleanest
    existing seam to recover it programmatically."""
    m = _C0006_RE.search(text)
    return int(m.group(1)) if m else None
