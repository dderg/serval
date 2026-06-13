from klippy.motion import Motion
from klippy.motion_kinematics import _LinearKinematics


class FakeStepper:
    def __init__(self, name):
        self._name = name
        self._active_callbacks = []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_name(self, short=False):
        return self._name


class FakeRail:
    def __init__(self, name, steppers):
        self._name = name
        self._steppers = steppers

    def get_name(self, short=False):
        return self._name if short else "stepper_" + self._name

    def get_steppers(self):
        return self._steppers


class FakeKin:
    active_rails = _LinearKinematics.active_rails

    def __init__(self, kind, rails):
        self.kind = kind
        self.rails = rails
        self._lanes = [(0, "x", []), (1, "y", []), (2, "z", [])]

    def coupled_xy(self):
        return self.kind == "corexy"

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]


def make_kin(kind):
    rails = [
        FakeRail("x", [FakeStepper("stepper_x"), FakeStepper("stepper_x1")]),
        FakeRail("y", [FakeStepper("stepper_y"), FakeStepper("stepper_y1")]),
        FakeRail("z", [FakeStepper("stepper_z"), FakeStepper("stepper_z1")]),
    ]
    return FakeKin(kind, rails)


def rail_names(rails):
    return [r.get_name(short=True) for r in rails]


def test_corexy_x_move_couples_both_gantry_rails_not_z():
    kin = make_kin("corexy")
    assert rail_names(kin.active_rails(5.0, 0.0, 0.0)) == ["x", "y"]
    assert rail_names(kin.active_rails(0.0, 5.0, 0.0)) == ["x", "y"]
    assert rail_names(kin.active_rails(0.0, 0.0, 5.0)) == ["z"]
    assert rail_names(kin.active_rails(0.0, 0.0, 0.0)) == []


def test_cartesian_rails_are_independent():
    kin = make_kin("cartesian")
    assert rail_names(kin.active_rails(5.0, 0.0, 0.0)) == ["x"]
    assert rail_names(kin.active_rails(0.0, 5.0, 0.0)) == ["y"]
    assert rail_names(kin.active_rails(0.0, 0.0, 5.0)) == ["z"]


class FakeToolhead:
    _fire_active_callbacks = Motion._fire_active_callbacks

    def __init__(self, kin, follower_steppers=()):
        self.kin = kin
        self.follower_steppers = list(follower_steppers)
        self._clock = 100.0

    def get_last_move_time(self):
        self._clock += 0.090
        return self._clock


def arm_callbacks(steppers):
    fired = []
    for s in steppers:
        s.add_active_callback(lambda pt, n=s.get_name(): fired.append(n))
    return fired


def all_steppers(kin):
    return [s for rail in kin.rails for s in rail.get_steppers()]


def test_each_enable_callback_gets_fresh_print_time():
    kin = make_kin("corexy")
    fired = []
    for s in all_steppers(kin):
        s.add_active_callback(fired.append)
    th = FakeToolhead(kin)
    th._fire_active_callbacks((5.0, 5.0, 5.0, 0.0))
    assert len(fired) == 6
    assert len(set(fired)) == 6, "print_time must be recomputed per callback"
    assert fired == sorted(fired)


def test_cartesian_move_enables_only_the_moving_axis():
    kin = make_kin("cartesian")
    fired = arm_callbacks(all_steppers(kin))
    FakeToolhead(kin)._fire_active_callbacks((5.0, 0.0, 0.0, 0.0))
    assert sorted(fired) == ["stepper_x", "stepper_x1"]


def test_corexy_x_move_enables_both_gantry_steppers_not_z():
    kin = make_kin("corexy")
    fired = arm_callbacks(all_steppers(kin))
    FakeToolhead(kin)._fire_active_callbacks((5.0, 0.0, 0.0, 0.0))
    assert sorted(fired) == [
        "stepper_x",
        "stepper_x1",
        "stepper_y",
        "stepper_y1",
    ]


def test_extruder_move_enables_follower_not_kinematic_steppers():
    kin = make_kin("cartesian")
    follower = FakeStepper("motor_e")
    fired = arm_callbacks(all_steppers(kin) + [follower])
    FakeToolhead(kin, follower_steppers=[follower])._fire_active_callbacks(
        (0.0, 0.0, 0.0, 4.0)
    )
    assert fired == ["motor_e"]


def test_pure_kinematic_move_leaves_follower_disabled():
    kin = make_kin("cartesian")
    follower = FakeStepper("motor_e")
    fired = arm_callbacks(all_steppers(kin) + [follower])
    FakeToolhead(kin, follower_steppers=[follower])._fire_active_callbacks(
        (0.0, 0.0, 5.0, 0.0)
    )
    assert fired == ["stepper_z", "stepper_z1"]
