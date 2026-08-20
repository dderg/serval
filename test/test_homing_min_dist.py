import pytest
from fakes import FakeEngine, FakeGcmd, FakeToolhead

from klippy.extras import homing as homing_mod


def test_trigger_too_early_short_with_margin():
    assert homing_mod._trigger_too_early(2.0, 15.0, 0.5) is True


def test_trigger_too_early_at_tolerance_edge_is_early():
    # 15 - 14.5 = 0.5 >= 0.5 -> early
    assert homing_mod._trigger_too_early(14.5, 15.0, 0.5) is True


def test_trigger_too_early_within_tolerance_band_not_early():
    # 15 - 14.6 = 0.4 < 0.5 -> not early
    assert homing_mod._trigger_too_early(14.6, 15.0, 0.5) is False


def test_trigger_too_early_beyond_min_not_early():
    assert homing_mod._trigger_too_early(100.0, 15.0, 0.5) is False


def test_trigger_too_early_disabled_when_min_zero():
    assert homing_mod._trigger_too_early(0.0, 0.0, 0.5) is False


def _hi_for_travel(position_endstop, positive_dir):
    from klippy.rail import HomingInfo

    return HomingInfo(
        speed=50.0,
        position_endstop=position_endstop,
        retract_speed=25.0,
        retract_dist=5.0,
        positive_dir=positive_dir,
        second_homing_speed=50.0,
        use_sensorless_homing=False,
        min_home_dist=0.0,
        accel=None,
    )


def test_homing_max_travel_positive_dir_pads_span():
    hi = _hi_for_travel(position_endstop=300.0, positive_dir=True)
    assert homing_mod._homing_max_travel(hi, 0.0, 300.0) == pytest.approx(450.0)


def test_homing_max_travel_negative_dir_pads_span():
    hi = _hi_for_travel(position_endstop=0.0, positive_dir=False)
    assert homing_mod._homing_max_travel(hi, 0.0, 300.0) == pytest.approx(450.0)


def test_homing_max_travel_endstop_inside_range():
    hi = _hi_for_travel(position_endstop=-2.0, positive_dir=False)
    assert homing_mod._homing_max_travel(hi, -5.0, 250.0) == pytest.approx(
        1.5 * 252.0
    )


def _hi(min_home_dist=15.0, speed=50.0, retract_speed=25.0, retract_dist=5.0):
    from klippy.rail import HomingInfo

    return HomingInfo(
        speed=speed,
        position_endstop=20.0,
        retract_speed=retract_speed,
        retract_dist=retract_dist,
        positive_dir=True,
        second_homing_speed=speed,
        use_sensorless_homing=False,
        min_home_dist=min_home_dist,
        accel=None,
    )


def test_commit_and_seed_seeds_post_retract_position():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    engine = FakeEngine()
    hi = _hi(retract_dist=5.0)
    homing_mod._commit_and_seed(
        toolhead,
        engine,
        axis,
        1.0,
        hi,
        trip_pos=[20.0, 0.0, 0.0],
        final_pos=[20.0, 0.0, 0.0],
        trigger_position=20.0,
        provider=None,
        servo_handle="h",
    )
    assert toolhead.get_position()[axis] == 15.0
    assert engine.calls == [("finalize_homed_axis", "h", 0, [15.0, 0.0, 0.0])]


def test_commit_and_seed_no_servo_does_not_seed():
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    engine = FakeEngine()
    homing_mod._commit_and_seed(
        toolhead,
        engine,
        0,
        1.0,
        _hi(),
        trip_pos=[20.0, 0.0, 0.0],
        final_pos=[20.0, 0.0, 0.0],
        trigger_position=20.0,
        provider=None,
        servo_handle=None,
    )
    assert engine.calls == []


def _approach_script(toolhead, axis, traveled_per_call, overshoot=0.0):
    state = {"i": 0}
    calls = []

    def approach(speed, max_travel):
        i = state["i"]
        state["i"] += 1
        calls.append((speed, max_travel))
        cur = toolhead.get_position()
        trip = list(cur)
        trip[axis] = cur[axis] + traveled_per_call[i]
        final = list(trip)
        final[axis] = trip[axis] + overshoot
        return trip, final

    return approach, calls


def test_no_rehome_when_first_travel_exceeds_min():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [100.0])
    trip, final = homing_mod._run_homing_attempts(
        FakeGcmd(error=RuntimeError),
        toolhead,
        axis,
        1.0,
        _hi(min_home_dist=15.0),
        speed=50.0,
        first_max_travel=200.0,
        tolerance=0.5,
        trigger_position=20.0,
        approach=approach,
    )
    assert len(calls) == 1
    assert trip[axis] == 100.0


def test_rehome_backoff_stays_within_axis_bounds():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [2.0, 20.0])
    trip, final = homing_mod._run_homing_attempts(
        FakeGcmd(error=RuntimeError),
        toolhead,
        axis,
        1.0,
        _hi(min_home_dist=15.0),
        speed=50.0,
        first_max_travel=200.0,
        tolerance=0.5,
        trigger_position=20.0,
        approach=approach,
    )
    assert len(calls) == 2
    # The re-approach gets the full travel budget: an early trip may have
    # been a spurious mid-travel blip with the real switch far beyond a
    # 2*min_home_dist window.
    assert calls[1][1] == 200.0
    assert calls[0][0] == 50.0
    assert calls[1][0] == 50.0
    # Backoff is computed from the endstop position (20) minus min_home_dist
    # (15), landing at +5 inside the axis range rather than a negative
    # raw-frame coordinate.
    assert ("move", [5.0, 0.0, 0.0], 25.0) in toolhead.calls
    assert ("set_position", [20.0, 0.0, 0.0], (0,)) in toolhead.calls
    assert trip[axis] == 25.0


def test_rehome_backoff_within_bounds_for_min_endstop():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [-2.0, -20.0])
    hi = _hi(min_home_dist=15.0)
    hi = hi._replace(positive_dir=False, position_endstop=0.0)
    homing_mod._run_homing_attempts(
        FakeGcmd(error=RuntimeError),
        toolhead,
        axis,
        -1.0,
        hi,
        speed=50.0,
        first_max_travel=200.0,
        tolerance=0.5,
        trigger_position=0.0,
        approach=approach,
    )
    # Min endstop at 0, direction -1: backoff = 0 - (-1)*15 = +15, in range.
    assert ("move", [15.0, 0.0, 0.0], 25.0) in toolhead.calls


def test_rehome_then_still_early_raises():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [2.0, 1.0])
    with pytest.raises(RuntimeError, match="early homing trigger"):
        homing_mod._run_homing_attempts(
            FakeGcmd(error=RuntimeError),
            toolhead,
            axis,
            1.0,
            _hi(min_home_dist=15.0),
            speed=50.0,
            first_max_travel=200.0,
            tolerance=0.5,
            trigger_position=20.0,
            approach=approach,
        )
    assert len(calls) == 2


def test_min_home_dist_zero_never_rehomes():
    axis = 0
    toolhead = FakeToolhead(position=[0.0, 0.0, 0.0])
    approach, calls = _approach_script(toolhead, axis, [0.0])
    homing_mod._run_homing_attempts(
        FakeGcmd(error=RuntimeError),
        toolhead,
        axis,
        1.0,
        _hi(min_home_dist=0.0),
        speed=50.0,
        first_max_travel=200.0,
        tolerance=0.5,
        trigger_position=20.0,
        approach=approach,
    )
    assert len(calls) == 1
