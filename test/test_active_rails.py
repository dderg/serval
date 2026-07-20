from fakes import FakeKin as FakeKinBase
from fakes import FakeRail, FakeStepper
from fakes import FakeToolhead as FakeToolheadBase

from klippy.motion import Motion
from klippy.motion_kinematics import _LinearKinematics


class FakeKin(FakeKinBase):
    active_rails = _LinearKinematics.active_rails

    def __init__(self, kind, rails):
        super().__init__(rails=rails, kind=kind)


def make_kin(kind):
    rails = [
        FakeRail(
            name="stepper_x",
            steppers=[
                FakeStepper(name="stepper_x"),
                FakeStepper(name="stepper_x1"),
            ],
        ),
        FakeRail(
            name="stepper_y",
            steppers=[
                FakeStepper(name="stepper_y"),
                FakeStepper(name="stepper_y1"),
            ],
        ),
        FakeRail(
            name="stepper_z",
            steppers=[
                FakeStepper(name="stepper_z"),
                FakeStepper(name="stepper_z1"),
            ],
        ),
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


class FakeToolhead(FakeToolheadBase):
    _fire_active_callbacks = Motion._fire_active_callbacks

    def __init__(self, kin, follower_steppers=()):
        super().__init__(
            kin=kin,
            follower_steppers=follower_steppers,
            last_move_time=100.0,
            move_time_step=0.090,
        )

    @property
    def move_time_calls(self):
        return sum(1 for c in self.calls if c[0] == "get_last_move_time")


def arm_callbacks(steppers):
    fired = []
    for s in steppers:
        s.add_active_callback(lambda pt, n=s.get_name(): fired.append(n))
    return fired


def all_steppers(kin):
    return [s for rail in kin.rails for s in rail.get_steppers()]


def test_all_enable_callbacks_share_one_print_time():
    kin = make_kin("corexy")
    fired = []
    for s in all_steppers(kin):
        s.add_active_callback(fired.append)
    th = FakeToolhead(kin)
    th._fire_active_callbacks((5.0, 5.0, 5.0, 0.0))
    assert len(fired) == 6
    assert len(set(fired)) == 1, (
        "every motor of a move energizes at one print_time"
    )
    assert th.move_time_calls == 1, "print_time is sampled once, not per motor"


def test_no_active_callbacks_does_not_sample_print_time():
    kin = make_kin("corexy")
    th = FakeToolhead(kin)
    assert th._fire_active_callbacks((5.0, 5.0, 5.0, 0.0)) is False
    assert th.move_time_calls == 0, "no enable to schedule, no clock read"


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
    follower = FakeStepper(name="motor_e")
    fired = arm_callbacks(all_steppers(kin) + [follower])
    FakeToolhead(kin, follower_steppers=[follower])._fire_active_callbacks(
        (0.0, 0.0, 0.0, 4.0)
    )
    assert fired == ["motor_e"]


def test_pure_kinematic_move_leaves_follower_disabled():
    kin = make_kin("cartesian")
    follower = FakeStepper(name="motor_e")
    fired = arm_callbacks(all_steppers(kin) + [follower])
    FakeToolhead(kin, follower_steppers=[follower])._fire_active_callbacks(
        (0.0, 0.0, 5.0, 0.0)
    )
    assert fired == ["stepper_z", "stepper_z1"]
