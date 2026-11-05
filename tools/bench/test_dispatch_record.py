from __future__ import annotations

import struct

import dispatch_record
import numpy as np
import pytest

AXES = 4
INTERVAL_S = 0.5e-3


def header() -> bytes:
    return dispatch_record.HEADER.pack(dispatch_record.MAGIC, AXES, INTERVAL_S)


def segment(t0, t_start, t_end, line=7, flags=0, samples=2) -> bytes:
    values = np.arange(samples * AXES * 3, dtype="<f8")
    return (
        bytes([dispatch_record.KIND_SEGMENT])
        + dispatch_record.SEGMENT.pack(t0, t_start, t_end, line, flags, samples)
        + values.tobytes()
    )


def drain(cause, moves_sent) -> bytes:
    return bytes([dispatch_record.KIND_DRAIN]) + dispatch_record.DRAIN.pack(
        cause, moves_sent
    )


def nudge() -> bytes:
    return bytes([dispatch_record.KIND_NUDGE]) + dispatch_record.NUDGE.pack(
        5.0, 1.0, 1.2, 0, 2
    )


def test_segments_drains_and_nudges_parse_in_dispatch_order():
    record = dispatch_record.parse(
        header()
        + segment(10.0, 0.0, 0.1, line=3, flags=1, samples=3)
        + drain(dispatch_record.CAUSE_PACER_BRAKE, 12)
        + segment(11.5, 0.1, 0.2, line=4, samples=2)
        + drain(dispatch_record.CAUSE_REQUESTED, 20)
        + nudge()
    )
    assert len(record) == 2
    assert record.source_line.tolist() == [3, 4]
    assert record.host_start().tolist() == [10.0, 11.6]
    assert record.flags.tolist() == [1, 0]
    assert record.segment_samples(0).shape == (3, AXES, 3)
    assert record.segment_samples(1)[0, 0, 0] == 0.0
    assert record.drains == [
        dispatch_record.Drain(1, dispatch_record.CAUSE_PACER_BRAKE, 12),
        dispatch_record.Drain(2, dispatch_record.CAUSE_REQUESTED, 20),
    ]
    assert record.pacer_brakes() == record.drains[:1]
    assert record.nudges == 1


@pytest.mark.parametrize(
    "data, problem",
    [
        (b"SVTAP2\0\0" + bytes(12), "not a dispatch record"),
        (header() + segment(0.0, 0.0, 0.1)[:-1], "truncated segment samples"),
        (header() + drain(1, 3)[:5], "truncated drain"),
        (header() + drain(7, 3), "unknown cause 7"),
        (header() + bytes([9]), "unknown record kind 9"),
        (header() + segment(0.0, 0.1, 0.1), "with 2 samples"),
        (header() + segment(0.0, 0.0, 0.1, samples=1), "with 1 samples"),
        (
            dispatch_record.HEADER.pack(dispatch_record.MAGIC, 0, INTERVAL_S),
            "0 axes",
        ),
    ],
)
def test_a_malformed_record_is_refused(data, problem):
    with pytest.raises(dispatch_record.RecordError, match=problem):
        dispatch_record.parse(data)


def test_the_layout_is_the_little_endian_contract():
    assert dispatch_record.HEADER.size == 20
    assert dispatch_record.SEGMENT.size == 33
    assert dispatch_record.DRAIN.size == 9
    assert dispatch_record.NUDGE.size == 29
    assert struct.calcsize("<dddIBI") == dispatch_record.SEGMENT.size
