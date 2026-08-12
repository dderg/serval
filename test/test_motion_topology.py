from fakes import (
    FakeKin as FakeKinBase,
)
from fakes import (
    FakeMcu,
    FakePrinter,
    FakeRail,
    FakeStepper,
)

from klippy.motion import Motion
from klippy.motion_kinematics import _LinearKinematics

FAKE_STEPPER_VELOCITY_CEILING = 0.5 / (0.000001 + 0.000002) * 0.0125
FAKE_STEP_DIST = 0.0125


def piece_topology(handle, axes, kin):
    n = len(axes)
    return (
        handle,
        axes,
        kin,
        [FAKE_STEPPER_VELOCITY_CEILING] * n,
        0,
        [FAKE_STEP_DIST] * n,
        [False] * n,
        [0] * n,
        0.0,
        0,
        [2e-06] * n,
    )


class FakeKin(FakeKinBase):
    coupled_xy = _LinearKinematics.coupled_xy
    mcu_tag = _LinearKinematics.mcu_tag
    claimed_axes = _LinearKinematics.claimed_axes

    def __init__(self, kind, lane_handles):
        lanes = [
            (i, axis, ["m_" + axis])
            for i, (axis, _h) in enumerate(lane_handles)
        ]
        rails = [
            FakeRail(steppers=[FakeStepper(name="stepper_" + axis, handle=h)])
            for axis, h in lane_handles
        ]
        super().__init__(rails=rails, kind=kind, lanes=lanes)


class FakeForceMove:
    def __init__(self, steppers):
        self.steppers = steppers


SPATIAL_AXES = [("x", 11), ("y", 11), ("z", 11)]


def make_motion(kind, lane_handles, follower=None, fm_present=True):
    motion = Motion.__new__(Motion)
    motion.kin = FakeKin(kind, lane_handles)
    motion.axis_sections = [
        (axis, [], ["m_" + axis], []) for axis, _h in lane_handles
    ]
    lanes = [
        (i, axis, ["m_" + axis], "stepper")
        for i, (axis, _h) in enumerate(lane_handles)
    ]
    followers = []
    steppers = {}
    if follower is not None:
        name, motor_name, handle = follower
        motion.axis_sections.append((name, ["x"], [motor_name], []))
        followers.append((name, [motor_name], 3))
        steppers[motor_name] = FakeStepper(name=motor_name, handle=handle)
    motion.kinematics_decl = (kind, lanes, followers)
    fm = FakeForceMove(steppers) if fm_present else None
    objs = {} if fm is None else {"force_move": fm}
    motion.printer = FakePrinter(objects=objs)
    motion.reactor = motion.printer.get_reactor()
    return motion


def test_one_mcu_corexy_topology():
    motion = make_motion("corexy", SPATIAL_AXES, follower=("e", "extruder", 11))
    a2h = motion._build_axis_to_handle()
    assert a2h == {0: 11, 1: 11, 2: 11, 3: 11}
    assert motion._derive_mcu_topology(a2h) == [
        piece_topology(11, [0, 1, 2, 3], 0)
    ]


def test_two_mcu_corexy_topology():
    lanes = [("x", 100), ("y", 100), ("z", 200)]
    motion = make_motion("corexy", lanes, follower=("e", "extruder", 200))
    a2h = motion._build_axis_to_handle()
    assert a2h == {0: 100, 1: 100, 2: 200, 3: 200}
    assert motion._derive_mcu_topology(a2h) == [
        piece_topology(100, [0, 1], 0),
        piece_topology(200, [2, 3], 1),
    ]


def test_cartesian_topology_tag_is_cartesian():
    motion = make_motion(
        "cartesian", SPATIAL_AXES, follower=("e", "extruder", 11)
    )
    a2h = motion._build_axis_to_handle()
    assert motion._derive_mcu_topology(a2h) == [
        piece_topology(11, [0, 1, 2, 3], 1)
    ]


def test_follower_slot_sourced_from_force_move_extruder():
    motion = make_motion(
        "cartesian", SPATIAL_AXES, follower=("e", "extruder", 42)
    )
    a2h = motion._build_axis_to_handle()
    assert a2h[3] == 42
    slot_steppers = motion._build_slot_steppers()
    assert [name for name, _ in slot_steppers[3]] == ["extruder"]


def test_follower_declared_before_spatial_axes_does_not_clobber_lane_slot():
    # A follower [axis e] declared before the spatial axes (e.g. via an
    # [include] processed first) gets declared-order index 0, which must NOT
    # overwrite corexy lane slot 0 (motor A) with the extruder. Doing so makes
    # the engine apply the extruder's fine step distance to motor A, so a normal
    # X move demands >16 microsteps/sample and faults -310 StepsPerSampleExceeded.
    motion = Motion.__new__(Motion)
    motion.kin = FakeKin("corexy", SPATIAL_AXES)
    motion.kinematics_decl = _corexy_decl_with_follower()
    fm = FakeForceMove({"extruder": FakeStepper(name="extruder", handle=11)})
    motion.printer = FakePrinter(objects={"force_move": fm})

    slot_steppers = motion._build_slot_steppers()

    assert [name for name, _ in slot_steppers[0]] == ["stepper_x"]


def _corexy_decl_with_follower():
    lanes = [
        (i, axis, ["m_" + axis], "stepper")
        for i, (axis, _h) in enumerate(SPATIAL_AXES)
    ]
    return ("corexy", lanes, [("e", ["extruder"], 3)])


def _motion_with_follower_first(follower_handle):
    motion = Motion.__new__(Motion)
    motion.kin = FakeKin("corexy", SPATIAL_AXES)
    motion.kinematics_decl = _corexy_decl_with_follower()
    fm = FakeForceMove(
        {"extruder": FakeStepper(name="extruder", handle=follower_handle)}
    )
    motion.printer = FakePrinter(objects={"force_move": fm})
    return motion


def test_follower_declared_before_spatial_axes_maps_handle_to_free_slot():
    # _build_axis_to_handle must agree with _build_slot_steppers: a follower
    # declared first lands in the free slot (3), not lane slot 0.
    motion = _motion_with_follower_first(42)
    a2h = motion._build_axis_to_handle()
    assert a2h == {0: 11, 1: 11, 2: 11, 3: 42}


class CaptureEngine:
    def __init__(self):
        self.init_planner_args = None

    def init_planner(self, config_text, topology):
        self.init_planner_args = {
            "config_text": config_text,
            "topology": topology,
        }


def test_init_planner_passes_config_text_and_topology():
    motion = make_motion("corexy", SPATIAL_AXES, follower=("e", "extruder", 11))
    motion._motion_config_text = (
        "[printer]\nmax_velocity: 300\nmax_accel: 3000\n"
    )
    motion._planner_ready = False
    engine = CaptureEngine()
    motion.engine = engine

    mcu = FakeMcu(handle=11)

    def lookup_objects(module=None):
        if module == "mcu":
            return [("mcu", mcu)]
        return []

    motion.printer.lookup_objects = lookup_objects
    motion._configure_axes_per_mcu = lambda engine_mcus: None
    motion._register_engine_wakeup = lambda: None

    motion._init_planner()
    assert engine.init_planner_args["config_text"] == motion._motion_config_text
    assert engine.init_planner_args["topology"] == [
        piece_topology(11, [0, 1, 2, 3], 0)
    ]
