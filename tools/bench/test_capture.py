from __future__ import annotations

import capture
import pytest

ENDSTOP_OUTPUTS_LOW = (0b01_01_01_01 << 16, 0, 0)


def _identity_frame(build_id: bytes, registers=ENDSTOP_OUTPUTS_LOW) -> bytes:
    payload = build_id + capture.IDENTITY_REGISTERS.pack(*registers)
    return (
        capture.HEADER.pack(capture.IDENTITY_MAGIC, 0, 0, len(payload), 0, 0)
        + payload
    )


def test_an_identity_frame_carries_the_build_id_and_the_endstop_registers():
    found = capture.parse_identity(_identity_frame(b"a" * 40))
    assert found == capture.Identity("a" * 40, *ENDSTOP_OUTPUTS_LOW)
    assert found.endstop_line_problems() == []
    capture.require_endstop_lines_idle(found)


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


def test_firmware_that_reports_no_pin_registers_is_refused():
    frame = (
        capture.HEADER.pack(capture.IDENTITY_MAGIC, 0, 0, 7, 0, 0) + b"unknown"
    )
    with pytest.raises(capture.CaptureError, match="predates the endstop-line"):
        capture.parse_identity(frame)


@pytest.mark.parametrize(
    ("registers", "problem"),
    [
        ((0, 0, 0), "PD8 (x endstop) is in mode 00, not a driven output"),
        ((0b01_01_01_00 << 16, 0, 0), "PD8 (x endstop) is in mode 00"),
        ((0b10_01_01_01 << 16, 0, 0), "PD11 (probe) is in mode 10"),
        ((ENDSTOP_OUTPUTS_LOW[0], 1 << 9, 0), "PD9 (y endstop) is open-drain"),
        ((ENDSTOP_OUTPUTS_LOW[0], 0, 1 << 10), "PD10 (z endstop) drives high"),
    ],
)
def test_an_endstop_line_not_driven_low_push_pull_is_refused(
    registers, problem
):
    found = capture.parse_identity(_identity_frame(b"b" * 40, registers))
    with pytest.raises(
        capture.CaptureError,
        match=problem.replace("(", r"\(").replace(")", r"\)"),
    ):
        capture.require_endstop_lines_idle(found)


def test_other_gpiod_pins_do_not_matter():
    moder = ENDSTOP_OUTPUTS_LOW[0] | (0b10 << 24) | 0b11
    odr = (1 << 14) | (1 << 3)
    found = capture.parse_identity(
        _identity_frame(b"c" * 40, (moder, 0xFFFF00FF, odr))
    )
    assert found.endstop_line_problems() == []
