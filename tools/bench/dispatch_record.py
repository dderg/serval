"""Read a live host's dispatch record (dispatch_record_p<pid>_s<n>.bin).

The live host writes one when SERVAL_DISPATCH_RECORD_DIR is set: every shaped
segment it dispatched, sampled per axis as (position, velocity, acceleration)
with both ends included, every breakpoint read once from each side and no two
samples of one breakpoint interval more than `sample_interval_s` apart,
together with the host-time origin it was anchored on, and every drain to rest
with its cause.
The engine's `pipeline_snapshot_dispatch_record` writes the offline plan in
the same format, so the two compare segment for segment.

Layout (little-endian): magic, u32 axes, f64 sample interval; then records,
each a u8 kind followed by
    SEGMENT  f64 t0, f64 t_start, f64 t_end, u32 source_line, u8 flags,
             u32 n, n x (f64 t, axes x 3 f64)
    DRAIN    u8 cause, u64 moves_sent
    NUDGE    f64 t0, f64 t_start, f64 t_end, u32 mcu_id, u8 axis
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"SVDREC2\0"
HEADER = struct.Struct("<8sId")
KIND_SEGMENT = 1
KIND_DRAIN = 2
KIND_NUDGE = 3
SEGMENT = struct.Struct("<dddIBI")
DRAIN = struct.Struct("<BQ")
NUDGE = struct.Struct("<dddIB")
FLAG_FRESH_EPOCH = 1
FLAG_REST_AT_END = 2
CAUSE_REQUESTED = 0
CAUSE_PACER_BRAKE = 1
CAUSES = (CAUSE_REQUESTED, CAUSE_PACER_BRAKE)


class RecordError(RuntimeError):
    pass


@dataclass(frozen=True)
class Drain:
    after_segments: int
    cause: int
    moves_sent: int

    @property
    def pacer_brake(self) -> bool:
        return self.cause == CAUSE_PACER_BRAKE


@dataclass
class DispatchRecord:
    axes: int
    sample_interval_s: float
    t0: np.ndarray
    t_start: np.ndarray
    t_end: np.ndarray
    source_line: np.ndarray
    flags: np.ndarray
    sample_start: np.ndarray
    times: np.ndarray
    samples: np.ndarray
    drains: list[Drain]
    nudges: int

    def __len__(self) -> int:
        return self.t0.size

    def segment_samples(self, index: int) -> np.ndarray:
        return self.samples[
            self.sample_start[index] : self.sample_start[index + 1]
        ]

    def segment_times(self, index: int) -> np.ndarray:
        return self.times[
            self.sample_start[index] : self.sample_start[index + 1]
        ]

    def host_start(self) -> np.ndarray:
        return self.t0 + self.t_start

    def host_end(self) -> np.ndarray:
        return self.t0 + self.t_end

    def pacer_brakes(self) -> list[Drain]:
        return [d for d in self.drains if d.pacer_brake]


def parse(data: bytes, name: str = "dispatch record") -> DispatchRecord:
    if len(data) < HEADER.size or data[: len(MAGIC)] != MAGIC:
        raise RecordError(f"{name}: not a dispatch record (magic {data[:8]!r})")
    _, axes, interval = HEADER.unpack_from(data)
    if axes < 1 or not interval > 0.0:
        raise RecordError(
            f"{name}: header claims {axes} axes sampled {interval} s apart"
        )
    heads: list[tuple] = []
    sample_starts = [0]
    time_blobs: list[np.ndarray] = []
    blobs: list[np.ndarray] = []
    row = 1 + axes * 3
    drains: list[Drain] = []
    nudges = 0
    offset = HEADER.size
    end = len(data)
    while offset < end:
        kind = data[offset]
        offset += 1
        if kind == KIND_SEGMENT:
            _need(data, offset, SEGMENT.size, name, "segment")
            t0, t_start, t_end, line, flags, n = SEGMENT.unpack_from(
                data, offset
            )
            offset += SEGMENT.size
            width = n * row * 8
            _need(data, offset, width, name, "segment samples")
            if n < 2 or not t_end > t_start:
                raise RecordError(
                    f"{name}: segment {len(heads)} at t={t_start} spans "
                    f"[{t_start}, {t_end}] with {n} samples"
                )
            rows = np.frombuffer(data, "<f8", n * row, offset).reshape(n, row)
            times = rows[:, 0]
            gaps = np.diff(times)
            if (
                times[0] != t_start
                or times[-1] != t_end
                or gaps.min() < 0.0
                or gaps.max() > interval * (1.0 + 1e-9)
            ):
                raise RecordError(
                    f"{name}: segment {len(heads)} over [{t_start}, {t_end}] "
                    f"is sampled from {times[0]} to {times[-1]} with gaps "
                    f"{gaps.min()}..{gaps.max()} s (at most {interval})"
                )
            time_blobs.append(times)
            blobs.append(rows[:, 1:].reshape(n, axes, 3))
            offset += width
            heads.append((t0, t_start, t_end, line, flags))
            sample_starts.append(sample_starts[-1] + n)
        elif kind == KIND_DRAIN:
            _need(data, offset, DRAIN.size, name, "drain")
            cause, moves_sent = DRAIN.unpack_from(data, offset)
            offset += DRAIN.size
            if cause not in CAUSES:
                raise RecordError(f"{name}: drain with unknown cause {cause}")
            drains.append(Drain(len(heads), cause, moves_sent))
        elif kind == KIND_NUDGE:
            _need(data, offset, NUDGE.size, name, "nudge")
            offset += NUDGE.size
            nudges += 1
        else:
            raise RecordError(
                f"{name}: unknown record kind {kind} at byte {offset - 1}"
            )
    table = np.array(heads, dtype=np.float64).reshape(-1, 5)
    return DispatchRecord(
        axes=axes,
        sample_interval_s=interval,
        t0=table[:, 0],
        t_start=table[:, 1],
        t_end=table[:, 2],
        source_line=table[:, 3].astype(np.int64),
        flags=table[:, 4].astype(np.int64),
        sample_start=np.asarray(sample_starts, np.int64),
        times=(np.concatenate(time_blobs) if time_blobs else np.empty(0)),
        samples=(
            np.concatenate(blobs)
            if blobs
            else np.empty((0, axes, 3), np.float64)
        ),
        drains=drains,
        nudges=nudges,
    )


def _need(data: bytes, offset: int, size: int, name: str, what: str) -> None:
    if offset + size > len(data):
        raise RecordError(
            f"{name}: truncated {what} at byte {offset} "
            f"({len(data) - offset} of {size} bytes)"
        )


def load(path: Path) -> DispatchRecord:
    return parse(path.read_bytes(), path.name)
