from klippy.motion import Motion
from klippy.motion_kinematics import _LinearKinematics


class FakeMcu:
    def __init__(self, handle):
        self._bridge_handle = handle


class FakeStepper:
    def __init__(self, name, handle):
        self._name = name
        self._mcu = FakeMcu(handle)

    def get_name(self, short=False):
        return self._name

    def get_mcu(self):
        return self._mcu


class FakeRail:
    def __init__(self, steppers):
        self._steppers = steppers

    def get_steppers(self):
        return list(self._steppers)


class FakeKin:
    coupled_xy = _LinearKinematics.coupled_xy
    mcu_tag = _LinearKinematics.mcu_tag
    claimed_axes = _LinearKinematics.claimed_axes

    def __init__(self, kind, lane_handles):
        self.kind = kind
        self._lanes = [
            (i, axis, ["m_" + axis])
            for i, (axis, _h) in enumerate(lane_handles)
        ]
        self.rails = [
            FakeRail([FakeStepper("stepper_" + axis, h)])
            for axis, h in lane_handles
        ]

    def lanes(self):
        return self._lanes


class FakeForceMove:
    def __init__(self, steppers):
        self.steppers = steppers


class FakePrinter:
    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)


SPATIAL_AXES = [("x", 11), ("y", 11), ("z", 11)]


def make_motion(kind, lane_handles, follower=None, fm_present=True):
    motion = Motion.__new__(Motion)
    motion.kin = FakeKin(kind, lane_handles)
    motion.axis_sections = [
        (axis, [], ["m_" + axis], []) for axis, _h in lane_handles
    ]
    steppers = {}
    if follower is not None:
        name, motor_name, handle = follower
        motion.axis_sections.append((name, ["x"], [motor_name], []))
        steppers[motor_name] = FakeStepper(motor_name, handle)
    fm = FakeForceMove(steppers) if fm_present else None
    objs = {} if fm is None else {"force_move": fm}
    motion.printer = FakePrinter(objs)
    return motion


def test_one_mcu_corexy_topology():
    motion = make_motion("corexy", SPATIAL_AXES, follower=("e", "extruder", 11))
    a2h = motion._build_axis_to_handle()
    assert a2h == {0: 11, 1: 11, 2: 11, 3: 11}
    assert motion._derive_mcu_topology(a2h) == [(11, [0, 1, 2, 3], 0)]


def test_two_mcu_corexy_topology():
    lanes = [("x", 100), ("y", 100), ("z", 200)]
    motion = make_motion("corexy", lanes, follower=("e", "extruder", 200))
    a2h = motion._build_axis_to_handle()
    assert a2h == {0: 100, 1: 100, 2: 200, 3: 200}
    assert motion._derive_mcu_topology(a2h) == [
        (100, [0, 1], 0),
        (200, [2, 3], 1),
    ]


def test_cartesian_topology_tag_is_cartesian():
    motion = make_motion(
        "cartesian", SPATIAL_AXES, follower=("e", "extruder", 11)
    )
    a2h = motion._build_axis_to_handle()
    assert motion._derive_mcu_topology(a2h) == [(11, [0, 1, 2, 3], 1)]


def test_follower_slot_sourced_from_force_move_extruder():
    motion = make_motion(
        "cartesian", SPATIAL_AXES, follower=("e", "extruder", 42)
    )
    a2h = motion._build_axis_to_handle()
    assert a2h[3] == 42
    slot_steppers = motion._build_slot_steppers()
    assert [name for name, _ in slot_steppers[3]] == ["extruder"]


class CaptureBridge:
    def __init__(self):
        self.init_planner_args = None

    def init_planner(
        self, axis_sections, limit_sections, pp_sections, topology, kin_axes
    ):
        self.init_planner_args = {
            "topology": topology,
            "kinematics_axes": kin_axes,
        }


def test_init_planner_passes_claimed_axes():
    motion = make_motion("corexy", SPATIAL_AXES, follower=("e", "extruder", 11))
    motion.limit_sections = []
    motion.post_processor_sections = []
    bridge = CaptureBridge()
    motion.bridge = bridge

    mcu = FakeMcu(11)
    motion.printer._objs["__mcus"] = [("mcu", mcu)]

    def lookup_objects(module=None):
        if module == "mcu":
            return [("mcu", mcu)]
        return []

    motion.printer.lookup_objects = lookup_objects
    motion._configure_axes_per_mcu = lambda bridge_mcus: None

    motion._init_planner()
    assert bridge.init_planner_args["kinematics_axes"] == ["x", "y", "z"]
    assert bridge.init_planner_args["topology"] == [(11, [0, 1, 2, 3], 0)]
