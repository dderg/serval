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


class FakeToolhead:
    def __init__(self, pos):
        self.pos = list(pos)
        self.events = []

    def get_position(self):
        return list(self.pos)

    def set_position(self, newpos, homing_axes=None):
        self.pos = list(newpos)
        self.events.append(("set_position", list(newpos)))

    def move(self, newpos, speed):
        self.pos = list(newpos)
        self.events.append(("move", list(newpos), speed))

    def wait_moves(self):
        self.events.append(("wait_moves",))


class FakeBridge:
    def __init__(self):
        self.finalize_calls = []

    def finalize_homed_axis(self, handle, axis, pos):
        self.finalize_calls.append((handle, axis, pos))


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
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    bridge = FakeBridge()
    hi = _hi(retract_dist=5.0)
    homing_mod._commit_and_seed(
        toolhead,
        bridge,
        axis,
        1.0,
        hi,
        trip_pos=[20.0, 0.0, 0.0],
        final_pos=[20.0, 0.0, 0.0],
        trigger_height=20.0,
        provider=None,
        servo_handle="h",
    )
    assert toolhead.get_position()[axis] == 15.0
    assert bridge.finalize_calls == [("h", 0, 15.0)]


def test_commit_and_seed_no_servo_does_not_seed():
    toolhead = FakeToolhead([0.0, 0.0, 0.0])
    bridge = FakeBridge()
    homing_mod._commit_and_seed(
        toolhead,
        bridge,
        0,
        1.0,
        _hi(),
        trip_pos=[20.0, 0.0, 0.0],
        final_pos=[20.0, 0.0, 0.0],
        trigger_height=20.0,
        provider=None,
        servo_handle=None,
    )
    assert bridge.finalize_calls == []
