# Destinations for servo lane runs
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

from . import wire
from .stream import SlotRing


class FileEndpoint:
    """Batch-mode endpoint: writes the exact PushSampleRuns frames a live
    endpoint would receive to a file. Nothing plays in real time, so only
    the ring's continuity rules are checked."""

    def __init__(self, path, cycle_ns):
        self.path = path
        self.cycle_ns = cycle_ns
        self._file = open(path, "wb")
        self._rings = {}
        self._correlation_id = 0

    def now_index(self):
        return None

    def push(self, lanes: list[wire.LaneRun]):
        for run in lanes:
            ring = self._rings.get(run.slot_idx)
            if ring is None:
                ring = self._rings[run.slot_idx] = SlotRing(
                    run.slot_idx, self.cycle_ns, None, 0
                )
            ring.admit(run, self.now_index())
        self._correlation_id += 1
        body = wire.encode_push_sample_runs(lanes)
        payload = wire.encode_message(
            wire.Kind.PUSH_SAMPLE_RUNS, self._correlation_id, body
        )
        self._file.write(wire.encode_frame(wire.CHANNEL_PIECES, payload))

    def close(self):
        self._file.close()
