from __future__ import annotations

import capture
import pytest


def _identity_frame(build_id: bytes) -> bytes:
    return (
        capture.HEADER.pack(capture.IDENTITY_MAGIC, 0, 0, len(build_id), 0, 0)
        + build_id
    )


def test_an_identity_frame_carries_the_build_id():
    frame = _identity_frame(b"a" * 40)
    assert capture.parse_identity(frame) == "a" * 40


def test_a_partial_identity_frame_waits_for_the_rest():
    frame = _identity_frame(b"a" * 40)
    assert capture.parse_identity(frame[:10]) is None
    assert capture.parse_identity(frame[:-1]) is None


def test_a_capture_frame_is_not_an_identity():
    frame = capture.HEADER.pack(capture.COUNT_MAGIC, 0, 0, 0, 0, 0)
    with pytest.raises(
        capture.CaptureError, match="expected an identity frame"
    ):
        capture.parse_identity(frame)
