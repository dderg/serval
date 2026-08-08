import pytest
from fakes import (
    FakeConfig,
    FakeError,
    FakeMcu,
    FakeMotion,
)
from fakes import (
    FakePins as _FakePinsBase,
)
from fakes import (
    FakePrinter as _FakePrinterBase,
)

from klippy import motion_kinematics, motion_setup, stepper
from klippy.extras import servo_axis
from klippy.gcode import Coord


class FakeMCUEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []

    def add_stepper(self, stepper):
        self.steppers.append(stepper)


class FakePins(_FakePinsBase):
    def setup_pin(self, pin_type, pin_desc):
        assert pin_type == "endstop"
        return FakeMCUEndstop(pin_desc)


class FakeRegistrar:
    def register_stepper(self, config, mcu_stepper):
        pass


class FakeHoming:
    def resolve_endstops(self, kin):
        pass


class FakePrinter(_FakePrinterBase):
    config_error = FakeError

    def __init__(self):
        super().__init__(objects={"homing": FakeHoming()})
        self.add_object("pins", FakePins(chip=FakeMcu(printer=self)))

    def load_object(self, config, name):
        return self.objects.setdefault(name, FakeRegistrar())


def motor_section(**extra):
    values = {
        "drive": "stepper",
        "step_pin": "PF0",
        "dir_pin": "PF1",
        "rotation_distance": 40.0,
        "microsteps": 16,
    }
    values.update(extra)
    return values


def axis_section(**extra):
    values = {
        "position_min": 0.0,
        "position_max": 300.0,
        "position_endstop": 0.0,
        "endstop_pin": "^PE5",
        "homing_speed": 50.0,
    }
    values.update(extra)
    return values


def corexy_sections():
    return {
        "printer": {"max_velocity": 300, "max_accel": 3000},
        "kinematics": {
            "type": "corexy",
            "axis_x": "x",
            "axis_y": "y",
            "axis_z": "z",
            "a_motors": "a",
            "b_motors": "b",
            "z_motors": "z0, z1",
        },
        "axis x": axis_section(),
        "axis y": axis_section(),
        "axis z": axis_section(position_max=200.0),
        "motor a": motor_section(),
        "motor b": motor_section(),
        "motor z0": motor_section(),
        "motor z1": motor_section(),
    }


def cartesian_sections():
    return {
        "printer": {"max_velocity": 300, "max_accel": 3000},
        "kinematics": {
            "type": "cartesian",
            "axis_x": "x",
            "axis_y": "y",
            "axis_z": "z",
            "x_motors": "x",
            "y_motors": "y",
            "z_motors": "z",
        },
        "axis x": axis_section(),
        "axis y": axis_section(),
        "axis z": axis_section(position_max=200.0),
        "motor x": motor_section(),
        "motor y": motor_section(),
        "motor z": motor_section(),
    }


def sections_to_text(sections):
    lines = []
    for name, options in sections.items():
        lines.append("[%s]" % name)
        lines.extend("%s: %s" % (k, v) for k, v in options.items())
    return "\n".join(lines) + "\n"


def read_native_decl(sections):
    """Parse the topology with the native reader, surfacing its errors the
    way Motion._load_motion_config does."""
    from klippy import configfile

    try:
        _limits, _axes, kinematics_decl, _consumed = (
            configfile._config_doc.read_motion_settings(
                sections_to_text(sections)
            )
        )
    except configfile.error as e:
        raise FakeError(str(e))
    return kinematics_decl


def make_kin(sections):
    printer = FakePrinter()
    config = FakeConfig(printer, sections=sections, error=FakeError)
    motion = FakeMotion()
    motion.kinematics_decl = read_native_decl(sections)
    return motion_kinematics.load_kinematics(config, motion)


def test_corexy_section_parses_roles_and_motors():
    kin = make_kin(corexy_sections())
    assert kin.kind == "corexy"
    assert kin.claimed_axes() == ["x", "y", "z"]
    assert kin.lanes()[0] == (0, "x", ["a"])
    assert kin.lanes()[1] == (1, "y", ["b"])
    assert kin.lanes()[2] == (2, "z", ["z0", "z1"])
    assert len(kin.rails) == 3
    assert len(kin.rails[2].get_steppers()) == 2


def test_cartesian_uses_xyz_motor_roles():
    kin = make_kin(cartesian_sections())
    assert kin.kind == "cartesian"
    assert kin.claimed_axes() == ["x", "y", "z"]
    assert kin.lanes()[0] == (0, "x", ["x"])
    assert kin.lanes()[1] == (1, "y", ["y"])
    assert kin.lanes()[2] == (2, "z", ["z"])


def test_mcu_tag_corexy_needs_both_xy():
    corexy = make_kin(corexy_sections())
    assert corexy.mcu_tag([0, 1, 3]) == 0
    assert corexy.mcu_tag([2]) == 1
    assert corexy.mcu_tag([0]) == 1
    cartesian = make_kin(cartesian_sections())
    assert cartesian.mcu_tag([0, 1, 2]) == 1


def test_coupled_xy_true_for_corexy_false_for_cartesian():
    assert make_kin(corexy_sections()).coupled_xy() is True
    assert make_kin(cartesian_sections()).coupled_xy() is False


def test_cartesian_steppers_project_their_axis_coord():
    kin = make_kin(cartesian_sections())
    coord = Coord(4.0, 5.0, 6.0, 0.0)
    assert kin.rails[0].calc_position_from_coord(coord) == 4.0
    assert kin.rails[1].calc_position_from_coord(coord) == 5.0
    assert kin.rails[2].calc_position_from_coord(coord) == 6.0


def test_corexy_steppers_project_xy_sum_difference_and_z():
    kin = make_kin(corexy_sections())
    coord = Coord(4.0, 5.0, 6.0, 0.0)
    assert kin.rails[0].calc_position_from_coord(coord) == 9.0
    assert kin.rails[1].calc_position_from_coord(coord) == -1.0
    assert kin.rails[2].calc_position_from_coord(coord) == 6.0


def test_corexy_steppers_accept_sequence_coords():
    kin = make_kin(corexy_sections())
    assert kin.rails[0].calc_position_from_coord([4.0, 5.0]) == 9.0
    assert kin.rails[1].calc_position_from_coord((4.0, 5.0, 6.0)) == -1.0
    assert kin.rails[2].calc_position_from_coord([4.0, 5.0, 6.0]) == 6.0


def _motor_positions(kin, coord):
    return {
        name: rail.calc_position_from_coord(coord)
        for rail in kin.rails
        for name in (s.get_name() for s in rail.get_steppers())
    }


def test_cartesian_calc_position_reads_rail_positions_directly():
    kin = make_kin(cartesian_sections())
    assert kin.calc_position({"x": 4.0, "y": 5.0, "z": 6.0}) == [4.0, 5.0, 6.0]


def test_corexy_calc_position_inverts_xy_half_sum_difference():
    kin = make_kin(corexy_sections())
    assert kin.calc_position({"a": 9.0, "b": -1.0, "z0": 6.0, "z1": 6.0}) == [
        4.0,
        5.0,
        6.0,
    ]


def test_corexy_calc_position_round_trips_from_coord():
    kin = make_kin(corexy_sections())
    coord = Coord(4.0, 5.0, 6.0, 0.0)
    motor_pos = _motor_positions(kin, coord)
    assert motor_pos == {"a": 9.0, "b": -1.0, "z0": 6.0, "z1": 6.0}
    assert kin.calc_position(motor_pos) == [4.0, 5.0, 6.0]


def test_cartesian_calc_position_round_trips_from_coord():
    kin = make_kin(cartesian_sections())
    coord = Coord(4.0, 5.0, 6.0, 0.0)
    motor_pos = _motor_positions(kin, coord)
    assert kin.calc_position(motor_pos) == [4.0, 5.0, 6.0]


def test_cartesian_motors_active_on_their_own_axis():
    kin = make_kin(cartesian_sections())
    for lane_idx, axis in {0: "x", 1: "y", 2: "z"}.items():
        for motor in kin.rails[lane_idx].get_steppers():
            assert motor.is_active_axis(axis) is True
            for other in set("xyz") - {axis}:
                assert motor.is_active_axis(other) is False


def test_corexy_xy_motors_active_on_both_axes():
    kin = make_kin(corexy_sections())
    for lane_idx in (0, 1):
        for motor in kin.rails[lane_idx].get_steppers():
            assert motor.is_active_axis("x") is True
            assert motor.is_active_axis("y") is True
            assert motor.is_active_axis("z") is False
    for motor in kin.rails[2].get_steppers():
        assert motor.is_active_axis("z") is True
        assert motor.is_active_axis("x") is False
        assert motor.is_active_axis("y") is False


def test_lane_follower_motors_share_the_lane_projector():
    kin = make_kin(corexy_sections())
    coord = Coord(4.0, 5.0, 6.0, 0.0)
    z0, z1 = kin.rails[2].get_steppers()
    assert z0.calc_position_from_coord(coord) == 6.0
    assert z1.calc_position_from_coord(coord) == 6.0
    assert z0.is_active_axis("z") is True
    assert z1.is_active_axis("z") is True


def test_follower_axis_stepper_fails_loud_without_projector():
    sections = cartesian_sections()
    sections["motor e"] = motor_section()
    sections["axis e"] = {"follows": "x", "motors": "e"}
    printer = FakePrinter()
    config = FakeConfig(printer, sections=sections, error=FakeError)
    motion = FakeMotion()
    motion.kinematics_decl = read_native_decl(sections)
    motion_setup.build_follower_steppers(motion, config)
    assert [s.get_name() for s in motion.follower_steppers] == ["e"]
    with pytest.raises(stepper.error):
        motion.follower_steppers[0].calc_position_from_coord(
            Coord(1.0, 2.0, 3.0, 0.0)
        )


def test_unknown_type_rejected():
    sections = corexy_sections()
    sections["kinematics"]["type"] = "hybrid_corexy"
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "cartesian" in str(exc.value)
    assert "corexy" in str(exc.value)


def test_role_binding_to_undeclared_axis_rejected():
    sections = corexy_sections()
    sections["kinematics"]["axis_x"] = "w"
    with pytest.raises(FakeError):
        make_kin(sections)


def test_missing_kinematics_section_rejected():
    sections = corexy_sections()
    del sections["kinematics"]
    with pytest.raises(FakeError):
        make_kin(sections)


def test_printer_kinematics_key_rejected():
    sections = corexy_sections()
    sections["printer"]["kinematics"] = "corexy"
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "kinematics" in str(exc.value)


def test_lane_without_motors_rejected():
    sections = corexy_sections()
    sections["kinematics"]["a_motors"] = ""
    with pytest.raises(FakeError):
        make_kin(sections)


def test_active_rails_couples_xy_for_corexy():
    kin = make_kin(corexy_sections())
    active = kin.active_rails(1.0, 0.0, 0.0)
    assert kin.rails[0] in active
    assert kin.rails[1] in active
    assert kin.rails[2] not in active


def test_active_rails_independent_for_cartesian():
    kin = make_kin(cartesian_sections())
    active = kin.active_rails(1.0, 0.0, 0.0)
    assert kin.rails[0] in active
    assert kin.rails[1] not in active


def test_note_z_not_homed_clears_only_z():
    kin = make_kin(corexy_sections())
    kin.limits = [(0.0, 300.0), (0.0, 300.0), (0.0, 200.0)]
    kin.note_z_not_homed()
    assert kin.limits[0][0] <= kin.limits[0][1]
    assert kin.limits[1][0] <= kin.limits[1][1]
    assert kin.limits[2] == (1.0, -1.0)


def reject_legacy(extra_sections):
    """Legacy role-section rejection lives in the native motion-config
    reader; feed it the sections as config text and surface its error like
    Motion._load_motion_config does."""
    from klippy import configfile

    lines = ["[printer]", "max_velocity: 300", "max_accel: 3000"]
    for name, options in extra_sections.items():
        lines.append("[%s]" % name)
        lines.extend("%s: %s" % (k, v) for k, v in options.items())
    try:
        configfile._config_doc.read_motion_settings("\n".join(lines) + "\n")
    except configfile.error as e:
        raise FakeError(str(e))


def test_stepper_x_section_rejected():
    with pytest.raises(FakeError) as exc:
        reject_legacy({"stepper_x": motor_section()})
    assert "[kinematics]" in str(exc.value)
    assert "[motor a]" in str(exc.value)


def test_stepper_z2_section_rejected():
    with pytest.raises(FakeError) as exc:
        reject_legacy({"stepper_z2": motor_section()})
    assert "role-encoding motor sections" in str(exc.value)


def test_servo_x_section_rejected():
    with pytest.raises(FakeError) as exc:
        reject_legacy({"servo_x": {}})
    assert "drive: servo" in str(exc.value)


def test_arbitrary_motor_section_not_rejected():
    reject_legacy({"motor_a": motor_section()})


def test_stepper_enable_section_not_rejected():
    reject_legacy({"stepper_enable": {}})


def test_arbitrary_motor_name_builds_short_named_stepper():
    sections = cartesian_sections()
    del sections["motor x"]
    sections["motor front_left"] = motor_section()
    sections["kinematics"]["x_motors"] = "front_left"
    kin = make_kin(sections)
    assert kin.lanes()[0] == (0, "x", ["front_left"])
    assert kin.rails[0].get_steppers()[0].get_name() == "front_left"


def test_orphan_motor_rejected():
    sections = cartesian_sections()
    sections["motor spare"] = motor_section()
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "[motor spare]" in str(exc.value)
    assert "not assigned" in str(exc.value)


def test_follower_motor_not_orphan_and_gets_the_free_slot():
    sections = cartesian_sections()
    sections["motor e"] = motor_section()
    sections["axis e"] = {"follows": "x", "motors": "e"}
    kin = make_kin(sections)
    assert kin.kind == "cartesian"
    decl = read_native_decl(sections)
    assert decl[2] == [("e", ["e"], 3)]


def test_follower_with_servo_motor_rejected():
    sections = cartesian_sections()
    sections["motor e"] = {"drive": "servo"}
    sections["axis e"] = {"follows": "x", "motors": "e"}
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "follower axes support stepper motors only" in str(exc.value)


def test_two_followers_overflow_the_free_slots():
    sections = cartesian_sections()
    for name in ("e", "f"):
        sections["motor " + name] = motor_section()
        sections["axis " + name] = {"follows": "x", "motors": name}
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "motion slot(s) free of kinematics lanes" in str(exc.value)


def test_missing_drive_rejected():
    sections = cartesian_sections()
    del sections["motor x"]["drive"]
    with pytest.raises(FakeError):
        make_kin(sections)


def test_mixed_drive_lane_rejected():
    sections = corexy_sections()
    sections["motor z1"] = {"drive": "servo"}
    with pytest.raises(FakeError) as exc:
        make_kin(sections)
    assert "all-stepper or all-servo" in str(exc.value)


def _servo_rail():
    axis_opts = {
        "position_min": -6.0,
        "position_max": 235.0,
        "endstop_pin": "ec_z:endstop",
        "position_endstop": -6.0,
    }
    motor_opts = {
        "protocol": "ethercat",
        "node": "z_drive",
        "ethercat_chain_index": 1,
        "rotation_distance": 40.0,
        "encoder_counts_per_rev": 131072,
    }
    from test_servo_homing import FakeRailConfig

    return servo_axis.ServoRail(
        FakeRailConfig("axis z", axis_opts),
        [FakeRailConfig("motor z_drive", motor_opts)],
    )


def _homed_cartesian_with_servo_z():
    kin = make_kin(cartesian_sections())
    kin.rails[2] = _servo_rail()
    kin.limits = [(0.0, 300.0), (0.0, 300.0), (-6.0, 235.0)]
    return kin


def test_motor_off_keeps_homed_servo_axis_and_marks_dirty():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.limits[2] == (-6.0, 235.0)
    assert kin.parked_dirty_axes() == [2]


def test_motor_off_clears_stepper_axes_and_never_dirties_them():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.limits[0] == (1.0, -1.0)
    assert kin.limits[1] == (1.0, -1.0)
    assert 0 not in kin.parked_dirty_axes()
    assert 1 not in kin.parked_dirty_axes()


def test_motor_off_does_not_dirty_unhomed_servo_axis():
    kin = _homed_cartesian_with_servo_z()
    kin.limits[2] = (1.0, -1.0)
    kin._handle_motor_off(0.0)
    assert kin.parked_dirty_axes() == []


def test_set_position_clears_parked_dirty_for_homed_axes():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.parked_dirty_axes() == [2]
    kin.set_position([0.0, 0.0, 100.0, 0.0], homing_axes=[2])
    assert kin.parked_dirty_axes() == []


def test_set_position_without_homing_axes_keeps_dirty():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    kin.set_position([0.0, 0.0, 100.0, 0.0])
    assert kin.parked_dirty_axes() == [2]


def test_clear_homing_state_clears_parked_dirty():
    kin = _homed_cartesian_with_servo_z()
    kin._handle_motor_off(0.0)
    assert kin.parked_dirty_axes() == [2]
    kin.clear_homing_state([2])
    assert kin.parked_dirty_axes() == []
    assert kin.limits[2] == (1.0, -1.0)


def test_clear_parked_dirty_subset():
    kin = _homed_cartesian_with_servo_z()
    kin._parked_dirty = [True, False, True]
    kin.clear_parked_dirty([0])
    assert kin.parked_dirty_axes() == [2]
