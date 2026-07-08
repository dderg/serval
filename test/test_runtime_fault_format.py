from klippy.mcu import format_runtime_fault


def test_steps_per_sample_decodes_axis_and_steps():
    # The trident bench crash: wire u16 65226 = -310, detail 3<<16 | 120
    msg = format_runtime_fault(65226, (3 << 16) | 120, 0)
    assert msg == (
        "StepsPerSampleExceeded (-310): axis 3 demanded 120 steps in one "
        "sample, more than its motor's per-sample step budget"
    )


def test_saturated_step_count_reads_as_at_least():
    msg = format_runtime_fault(65226, (3 << 16) | 0xFFFF, 0)
    assert "at least 65535 steps" in msg


def test_piece_start_in_past_decodes_axis():
    msg = format_runtime_fault(65228, (2 << 16) | 2603, 0)
    assert msg == "PieceStartInPast (-308): axis 2, detail 2603"


def test_tick_interval_reports_blocker_pc():
    msg = format_runtime_fault(65225, 0, 0x0801_2345)
    assert msg == "TickIntervalExceeded (-311): tick blocker pc=0x08012345"


def test_unknown_code_is_still_reported():
    msg = format_runtime_fault(65535, 7, 0)
    assert msg == "unknown fault (-1): detail 7"
