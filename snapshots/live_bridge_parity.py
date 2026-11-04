"""The live/snapshot parity gate.

The live side is the real host path minus MCU I/O: klippy's own G-code
handlers (``live_replay.drive``) submit to the real ``MotionEngine`` bridge,
whose detached planner (``init_planner_detached``) runs the production
ingress and pipeline with nothing executing the trajectory, and records every
pipeline input and shaped segment. The snapshot side parses the same G-code
with ``_motion_engine.parse_gcode`` and plans it through the snapshot path.
Both records must agree item for item, bit for bit; the first difference is
reported as the diverging pipeline input (the move, rest or control token,
with the field that differs) or, when every input agrees, the first diverging
shaped segment.

    uv run python snapshots/live_bridge_parity.py GCODE CONFIG
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import harness  # noqa: E402
import live_replay  # noqa: E402

_FIELD = re.compile(r"^\s*(?:(\w+): )?(.*?)[{\[(]?,?$")
_SEQ = re.compile(r"start_line: (\d+),")


class DetachedEngine:
    """The bridge as klippy drives it, with the MCU clock caught up: the
    host never waits on an MCU, so every clock query answers "now"."""

    def __init__(self, config_text: str):
        self._engine = harness._import_engine().MotionEngine()
        self._engine.init_planner_detached(config_text)
        self.config_text = config_text
        self.frontier = 0.0

    def __getattr__(self, name):
        return getattr(self._engine, name)

    def frontier_print_time(self, mcu_handle):
        return self.frontier

    def dwell_until(self, print_time, mcu_handle):
        raise RuntimeError(
            "dwell_until needs an MCU clock; the parity gate has none"
        )

    def fence_print_time_poll(self, fence_id, mcu_handle):
        end = self._engine.detached_fence_poll(fence_id)
        if end is not None:
            self.frontier = max(self.frontier, end)
        return end

    def drained_record(self):
        while not self._engine.motion_drain_poll():
            pass
        self._engine.motion_drain_finalize()
        record = self._engine.detached_record()
        self._engine.shutdown()
        return record


@dataclass(frozen=True)
class Divergence:
    kind: str
    index: int
    seq: int | None
    path: str
    live: str
    snapshot: str

    def __str__(self) -> str:
        move = "" if self.seq is None else f" (move {self.seq})"
        return (
            f"{self.kind} {self.index}{move} differs at {self.path}:\n"
            f"  live:     {self.live}\n  snapshot: {self.snapshot}"
        )


def _field_path(live_debug: str, snapshot_debug: str) -> tuple[str, str, str]:
    live_lines = live_debug.splitlines()
    snapshot_lines = snapshot_debug.splitlines()
    stack: list[tuple[int, str]] = []
    for live_line, snapshot_line in zip(live_lines, snapshot_lines):
        indent = len(live_line) - len(live_line.lstrip())
        while stack and stack[-1][0] >= indent:
            stack.pop()
        name = _FIELD.match(live_line).group(1) or live_line.strip(" ,{[(")
        if live_line != snapshot_line:
            path = ".".join([*(label for _, label in stack), name])
            return path, live_line.strip(), snapshot_line.strip()
        if live_line.rstrip().endswith(("{", "[", "(")):
            stack.append((indent, name))
    return (
        "length",
        f"{len(live_lines)} lines",
        f"{len(snapshot_lines)} lines",
    )


def _first_difference(kind, live, snapshot):
    live_digests = getattr(live, f"{kind}_digests")()
    snapshot_digests = getattr(snapshot, f"{kind}_digests")()
    for index, (a, b) in enumerate(zip(live_digests, snapshot_digests)):
        if a != b:
            live_debug = getattr(live, f"{kind}_debug")(index)
            path, live_value, snapshot_value = _field_path(
                live_debug, getattr(snapshot, f"{kind}_debug")(index)
            )
            seq = _SEQ.search(live_debug)
            return Divergence(
                kind,
                index,
                int(seq.group(1)) if seq else None,
                path,
                live_value,
                snapshot_value,
            )
    if len(live_digests) != len(snapshot_digests):
        return Divergence(
            kind,
            min(len(live_digests), len(snapshot_digests)),
            None,
            "count",
            f"{len(live_digests)} {kind}s",
            f"{len(snapshot_digests)} {kind}s",
        )
    return None


def first_divergence(live, snapshot) -> Divergence | None:
    """The first pipeline input the two records disagree on, or when every
    input agrees, the first shaped segment."""
    return _first_difference("input", live, snapshot) or _first_difference(
        "segment", live, snapshot
    )


def live_record(gcode_text: str, cfg_path: Path):
    """What the live host feeds its pipeline for ``gcode_text`` and what the
    pipeline shapes, with the config document klippy serialized for it."""
    detached = live_replay.drive(gcode_text, cfg_path, DetachedEngine)
    return detached.drained_record(), detached.config_text


def snapshot_record(gcode_text: str, config_text: str):
    engine = harness._import_engine()
    return engine.snapshot_parity_record(
        engine.parse_gcode(gcode_text, config_text), config_text
    )


def compare(gcode_text: str, cfg_path: Path) -> Divergence | None:
    live, config_text = live_record(gcode_text, cfg_path)
    return first_divergence(live, snapshot_record(gcode_text, config_text))


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    gcode, cfg = Path(argv[0]), Path(argv[1])
    divergence = compare(gcode.read_text(), cfg)
    if divergence is None:
        print(f"parity: {gcode} on {cfg}")
        return 0
    print(f"{gcode} on {cfg}: {divergence}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
