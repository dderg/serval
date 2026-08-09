from fakes import FakeStepperEnable, FakeToolhead


def test_debug_enable_resyncs_before_energize():
    th = FakeToolhead(last_move_time=100.0, resync_delay=1.0)
    se = FakeStepperEnable(toolhead=th, names=["servo_z"], real_methods=True)
    se.motor_debug_enable("servo_z", True)
    call_names = [c[0] for c in th.calls]
    assert "resync_parked_servos" in call_names
    enabled_at = se.enable_lines["servo_z"].enabled_at
    assert enabled_at, "motor was energized"
    resync_idx = call_names.index("resync_parked_servos")
    enable_idx = call_names.index("get_last_move_time")
    assert resync_idx < enable_idx


def test_debug_disable_does_not_resync():
    th = FakeToolhead(last_move_time=100.0, resync_delay=1.0)
    se = FakeStepperEnable(toolhead=th, names=["servo_z"], real_methods=True)
    se.motor_debug_enable("servo_z", False)
    assert all(c[0] != "resync_parked_servos" for c in th.calls)
    assert se.enable_lines["servo_z"].disabled_at


def test_group_enable_resyncs_before_energize():
    th = FakeToolhead(last_move_time=100.0, resync_delay=1.0)
    se = FakeStepperEnable(
        toolhead=th, names=["motor_a", "servo_z"], real_methods=True
    )
    se.motor_enable_group(["motor_a", "servo_z"])
    call_names = [c[0] for c in th.calls]
    assert "resync_parked_servos" in call_names
    resync_idx = call_names.index("resync_parked_servos")
    enable_idx = call_names.index("get_last_move_time")
    assert resync_idx < enable_idx
    shared_times = {el.enabled_at[0] for el in se.enable_lines.values()}
    assert len(shared_times) == 1
