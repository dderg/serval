import pytest

from klippy.extras import resonance_buzz as resonance_buzz_module
from klippy.extras import servo_axis
from klippy.extras.resonance_buzz import (
    ResonanceBuzz,
    buzz_axis_to_motor_mask,
)


def test_corexy_x_drives_both_motors_in_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("x", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b000


def test_corexy_y_drives_both_motors_anti_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("y", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b010


def test_corexy_z_is_single_slot():
    assert buzz_axis_to_motor_mask("z", coupled=True) == (0b100, 0b000)


def test_cartesian_axes_map_one_to_one():
    assert buzz_axis_to_motor_mask("x", coupled=False) == (0b001, 0b000)
    assert buzz_axis_to_motor_mask("y", coupled=False) == (0b010, 0b000)
    assert buzz_axis_to_motor_mask("z", coupled=False) == (0b100, 0b000)


def test_case_insensitive():
    assert buzz_axis_to_motor_mask(
        "X", coupled=True
    ) == buzz_axis_to_motor_mask("x", coupled=True)


def test_unsupported_axis_raises():
    with pytest.raises(ValueError, match="unsupported buzz axis"):
        buzz_axis_to_motor_mask("e", coupled=False)


class FakeBuzzToolhead:
    def __init__(self):
        self.homed_axes = "xyz"

    def get_kinematics(self):
        return object()

    def get_status(self, eventtime):
        return {"homed_axes": self.homed_axes}

    def wait_moves(self):
        pass


class FakeBuzzMotion:
    def __init__(self):
        self.calls = []

    def submit_resonance_buzz(self, *args):
        self.calls.append(args)

    def resonance_buzz_done(self):
        return True


class FakeBuzzReactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


class FakeBuzzPrinter:
    command_error = RuntimeError

    def __init__(self):
        self.motion = FakeBuzzMotion()
        self._objs = {"toolhead": FakeBuzzToolhead(), "motion": self.motion}

    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)

    def get_reactor(self):
        return FakeBuzzReactor()


class FakeBuzzGcmd:
    error = RuntimeError

    def __init__(self):
        self.infos = []

    def respond_info(self, msg):
        self.infos.append(msg)


def _resonance_buzz(max_peak_accel=200000.0, max_amplitude=5.0):
    buzz = ResonanceBuzz.__new__(ResonanceBuzz)
    buzz.printer = FakeBuzzPrinter()
    buzz.max_peak_accel = max_peak_accel
    buzz.max_amplitude = max_amplitude
    return buzz


def test_unhomed_axis_rejects_buzz_before_motion_submission():
    buzz = _resonance_buzz()
    buzz.printer._objs["toolhead"].homed_axes = "yz"
    with pytest.raises(RuntimeError, match="home X"):
        buzz.run_sweep(FakeBuzzGcmd(), "x", 40.0, 40.0, 1.0, 0.05, 75.0, 0.05)
    assert buzz.printer.motion.calls == []


def test_over_ceiling_accel_per_hz_fails_loud_instead_of_clamping():
    buzz = _resonance_buzz()
    with pytest.raises(RuntimeError, match="largest ACCEL_PER_HZ"):
        buzz.run_sweep(
            FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 600.0, 0.0
        )
    assert buzz.printer.motion.calls == []


def test_accel_per_hz_at_ceiling_runs():
    buzz = _resonance_buzz()
    amplitude = buzz.run_sweep(
        FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 500.0, 0.0
    )
    assert buzz.printer.motion.calls
    assert amplitude > 0.0


def test_configured_max_peak_accel_bounds_the_sweep():
    buzz = _resonance_buzz(max_peak_accel=10000.0)
    with pytest.raises(RuntimeError, match="max_peak_accel"):
        buzz.run_sweep(FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 50.0, 0.0)
    assert buzz.printer.motion.calls == []


def test_configured_max_amplitude_bounds_explicit_amplitude():
    buzz = _resonance_buzz(max_amplitude=0.5)
    with pytest.raises(RuntimeError, match="max_amplitude"):
        buzz.run_sweep(FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 50.0, 1.0)
    assert buzz.printer.motion.calls == []


class FakeBuzzMotor:
    def __init__(self, motor_name, node_name):
        self.motor_name = motor_name
        self.node_name = node_name

    def get_motor_name(self):
        return self.motor_name

    def get_node_name(self):
        return self.node_name


class FakeBuzzNode:
    def __init__(self, handle, slots):
        self.handle = handle
        self.slots = slots

    def get_engine_handle(self):
        return self.handle

    def get_slot_for_motor(self, motor_name):
        return self.slots.get(motor_name)


class FakeBuzzKinematics:
    def __init__(self, rails):
        self.rails = rails

    def lanes(self):
        return [(i, "xyz"[i], []) for i in range(len(self.rails))]


class FakeBuzzEngine:
    def __init__(self):
        self.calls = []

    def resonance_buzz(self, routes, wave):
        self.calls.append((tuple(routes), tuple(wave)))


class FakeBuzzMotionTarget:
    def __init__(self, rails, nodes):
        self.printer = FakeBuzzPrinter()
        for node_name, node in nodes.items():
            self.printer._objs["ethercat_node " + node_name] = node
        self.kin = FakeBuzzKinematics(rails)
        self.engine = FakeBuzzEngine()


def _servo_rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.axis = axis
    rail.motors = motors
    return rail


WAVE = (40000, 40000, 250000, 1000, 50)


def test_mixed_topology_submits_one_atomic_request():
    motion = FakeBuzzMotionTarget(
        [_servo_rail("x", [FakeBuzzMotor("servo_x", "node0")]), object()],
        {"node0": FakeBuzzNode(7, {"servo_x": 2})},
    )
    resonance_buzz_module.submit_buzz(motion, 0b011, 0b010, WAVE)
    assert motion.engine.calls == [
        (
            (("ethercat", 7, 0b100, 0b000), ("stepper", 0b010, 0b010)),
            WAVE,
        )
    ]


def test_servo_only_topology_emits_no_stepper_route():
    motion = FakeBuzzMotionTarget(
        [
            _servo_rail("x", [FakeBuzzMotor("servo_x", "node0")]),
            _servo_rail("y", [FakeBuzzMotor("servo_y", "node0")]),
        ],
        {"node0": FakeBuzzNode(3, {"servo_x": 0, "servo_y": 1})},
    )
    resonance_buzz_module.submit_buzz(motion, 0b011, 0b010, WAVE)
    assert motion.engine.calls == [((("ethercat", 3, 0b011, 0b010),), WAVE)]


def test_empty_axis_mask_refuses_before_any_engine_call():
    motion = FakeBuzzMotionTarget([object()], {})
    with pytest.raises(RuntimeError, match="no target engine"):
        resonance_buzz_module.submit_buzz(motion, 0b000, 0b000, WAVE)
    assert motion.engine.calls == []
