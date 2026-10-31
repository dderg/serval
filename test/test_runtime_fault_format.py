from klippy.mcu import format_runtime_fault


def wire(fault_code):
    return fault_code + 0x10000


def test_phase_motor_unmapped_decodes_axis():
    msg = format_runtime_fault(wire(-313), (2 << 16) | 7, 0)
    assert msg == "PhaseMotorUnmapped (-313): axis 2, detail 7"


def test_a_deleted_piece_fault_code_no_longer_decodes():
    msg = format_runtime_fault(wire(-308), (2 << 16) | 2603, 0)
    assert msg == "unknown fault (-308): detail 133675"


def test_tick_interval_reports_blocker_pc():
    msg = format_runtime_fault(wire(-311), 0, 0x0801_2345)
    assert msg == "TickIntervalExceeded (-311): tick blocker pc=0x08012345"


def test_ring_full_names_the_lane():
    msg = format_runtime_fault(wire(-319), 2 << 16, 0)
    assert msg == "SampleRingFull (-319): lane 2"


def test_run_late_decodes_the_deficit():
    msg = format_runtime_fault(wire(-317), (1 << 16) | 4200, 0)
    assert msg == "SampleRunLate (-317): lane 1, deficit_ticks 4200"


def test_ring_underrun_decodes_the_tail_delta():
    msg = format_runtime_fault(wire(-318), (3 << 16) | 17, 0)
    assert msg == "SampleRingUnderrun (-318): lane 3, tail_delta_quanta 17"


def test_run_rejected_decodes_the_inner_fault():
    msg = format_runtime_fault(wire(-321), (0 << 16) | 2603, 0)
    assert msg == "SampleRunRejected (-321): lane 0, run_fault 2603"


def test_barrier_overflow_names_the_lane():
    msg = format_runtime_fault(wire(-322), 5 << 16, 0)
    assert msg == "SampleBarrierOverflow (-322): lane 5"


def test_lane_unknown_reports_the_oid():
    msg = format_runtime_fault(wire(-320), (0xFF << 16) | 9, 0)
    assert msg == "SampleLaneUnknown (-320): oid 9"


def test_steps_per_sample_decodes_axis_and_steps():
    msg = format_runtime_fault(wire(-310), (1 << 16) | 640, 0)
    assert msg == "StepsPerSampleExceeded (-310): axis 1, detail 640"


def test_sample_rate_misconfigured_is_named():
    msg = format_runtime_fault(wire(-304), 0, 0)
    assert msg == "SampleRateMisconfigured (-304): detail 0"


def test_unknown_code_is_still_reported():
    msg = format_runtime_fault(wire(-1), 7, 0)
    assert msg == "unknown fault (-1): detail 7"
