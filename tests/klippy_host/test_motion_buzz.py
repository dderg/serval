import pytest

from klippy.extras import servo_axis
from klippy.extras.resonance_buzz import MOTOR_A, MOTOR_B
from klippy.motion import Motion


class FakeCommandError(Exception):
    pass


class FakeNode:
    def __init__(self, handle, slot_by_motor):
        self._handle = handle
        self._slot_by_motor = slot_by_motor

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slot_by_motor.get(motor_name)


class FakePrinter:
    command_error = FakeCommandError

    def __init__(self, nodes):
        self._nodes = nodes

    def lookup_object(self, name, default=None):
        return self._nodes.get(name, default)


class FakeEngine:
    def __init__(self):
        self.buzz_calls = []

    def resonance_buzz(self, handle, slot_mask, slot_sign_mask, *_args):
        self.buzz_calls.append((handle, slot_mask, slot_sign_mask))


class FakeServoMotor:
    def __init__(self, name, node_name, chain_index):
        self._name = name
        self._node_name = node_name
        self._chain_index = chain_index

    def get_motor_name(self):
        return self._name

    def get_node_name(self):
        return self._node_name

    def get_chain_index(self):
        return self._chain_index


class FakeKin:
    def __init__(self, rails):
        self.rails = rails

    def lanes(self):
        return [(idx, rail.axis, None) for idx, rail in enumerate(self.rails)]


def make_servo_rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.axis = axis
    rail.motors = motors
    return rail


def make_motion(rails, node_handles, slot_by_motor):
    motion = Motion.__new__(Motion)
    motion.printer = FakePrinter(
        {
            "ethercat_node " + name: FakeNode(handle, slot_by_motor)
            for name, handle in node_handles.items()
        }
    )
    motion.kin = FakeKin(rails)
    motion.engine = FakeEngine()
    return motion


def submit_buzz(motion, axis_mask, sign_mask):
    motion.submit_resonance_buzz(
        axis_mask, sign_mask, 5000, 133000, 100000, 20000, 1000
    )


def test_single_motor_rail_buzzes_its_claim_slot_not_chain_index():
    rail = make_servo_rail("x", [FakeServoMotor("motor x", "node0", 2)])
    motion = make_motion([rail], {"node0": 7}, {"motor x": 1})
    submit_buzz(motion, MOTOR_A, 0)
    assert motion.engine.buzz_calls == [(7, 0b010, 0)]


def test_corexy_pair_sends_one_anti_phase_frame():
    rail_a = make_servo_rail("x", [FakeServoMotor("motor a", "node0", 0)])
    rail_b = make_servo_rail("y", [FakeServoMotor("motor b", "node0", 1)])
    motion = make_motion(
        [rail_a, rail_b], {"node0": 7}, {"motor a": 0, "motor b": 1}
    )
    submit_buzz(motion, MOTOR_A | MOTOR_B, MOTOR_B)
    assert motion.engine.buzz_calls == [(7, 0b11, 0b10)]


def test_multi_motor_rail_sets_a_slot_bit_per_motor():
    rail = make_servo_rail(
        "x",
        [
            FakeServoMotor("motor x", "node0", 0),
            FakeServoMotor("motor x1", "node0", 3),
        ],
    )
    motion = make_motion([rail], {"node0": 7}, {"motor x": 0, "motor x1": 1})
    submit_buzz(motion, MOTOR_A, MOTOR_A)
    assert motion.engine.buzz_calls == [(7, 0b011, 0b011)]


def test_awd_corexy_y_signs_belt_b_claim_slots():
    rail_a = make_servo_rail(
        "x",
        [
            FakeServoMotor("motor_a", "node0", 0),
            FakeServoMotor("motor_a1", "node0", 2),
        ],
    )
    rail_b = make_servo_rail(
        "y",
        [
            FakeServoMotor("motor_b", "node0", 1),
            FakeServoMotor("motor_b1", "node0", 3),
        ],
    )
    claim_slots = {"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3}
    motion = make_motion([rail_a, rail_b], {"node0": 7}, claim_slots)
    submit_buzz(motion, MOTOR_A | MOTOR_B, MOTOR_B)
    assert motion.engine.buzz_calls == [(7, 0b1111, 0b1100)]


def test_duplicate_slot_on_one_node_raises():
    rail_a = make_servo_rail("x", [FakeServoMotor("motor a", "node0", 1)])
    rail_b = make_servo_rail("y", [FakeServoMotor("motor b", "node0", 2)])
    motion = make_motion(
        [rail_a, rail_b], {"node0": 7}, {"motor a": 1, "motor b": 1}
    )
    with pytest.raises(FakeCommandError, match="already claimed"):
        submit_buzz(motion, MOTOR_A | MOTOR_B, 0)


def test_motor_missing_from_claim_map_raises():
    rail = make_servo_rail("x", [FakeServoMotor("motor x", "node0", 0)])
    motion = make_motion([rail], {"node0": 7}, {})
    with pytest.raises(FakeCommandError, match="no claim slot"):
        submit_buzz(motion, MOTOR_A, 0)
    assert motion.engine.buzz_calls == []


def test_missing_engine_handle_raises():
    rail = make_servo_rail("x", [FakeServoMotor("motor x", "node0", 0)])
    motion = make_motion([rail], {}, {"motor x": 0})
    with pytest.raises(FakeCommandError, match="no live EtherCAT"):
        submit_buzz(motion, MOTOR_A, 0)
