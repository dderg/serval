import pytest

from klippy import motion, motion_kinematics
from klippy.extras import servo_axis


class FakeError(Exception):
    pass


class FakePinParams:
    def __init__(self, pin, chip):
        self.pin = pin
        self.chip = chip

    def __getitem__(self, key):
        return {"pin": self.pin, "invert": False, "chip": self.chip}[key]


class FakeMCUEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []

    def add_stepper(self, stepper):
        self.steppers.append(stepper)


class FakePins:
    def __init__(self, chip):
        self.chip = chip

    def lookup_pin(self, pin, can_invert=False, can_pullup=False):
        return FakePinParams(pin, self.chip)

    def setup_pin(self, pin_type, pin_desc):
        assert pin_type == "endstop"
        return FakeMCUEndstop(pin_desc)


class FakeMCU:
    def __init__(self, printer):
        self._printer = printer
        self._oid = 0

    def create_oid(self):
        self._oid += 1
        return self._oid

    def register_config_callback(self, cb):
        pass

    def get_printer(self):
        return self._printer


class FakeRegistrar:
    def register_stepper(self, config, mcu_stepper):
        pass


class FakeHoming:
    def resolve_endstops(self):
        pass


class FakePrinter:
    def __init__(self):
        self.mcu = FakeMCU(self)
        self.pins = FakePins(self.mcu)
        self._objects = {"pins": self.pins, "homing": FakeHoming()}
        self.event_handlers = []

    def lookup_object(self, name):
        return self._objects[name]

    def load_object(self, config, name):
        return self._objects.setdefault(name, FakeRegistrar())

    def register_event_handler(self, event, handler):
        self.event_handlers.append((event, handler))

    config_error = FakeError


_UNSET = object()


class FakeConfig:
    def __init__(self, printer, sections):
        self._printer = printer
        self._sections = sections
        self.error = FakeError

    def get_printer(self):
        return self._printer

    def has_section(self, name):
        return name in self._sections

    def get_prefix_sections(self, prefix):
        return [
            FakeSection(self._printer, name, self._sections[name])
            for name in self._sections
            if name.startswith(prefix)
        ]

    def getsection(self, name):
        if name not in self._sections:
            raise FakeError("no section [%s]" % name)
        return FakeSection(self._printer, name, self._sections[name])


class FakeSection:
    def __init__(self, printer, name, values):
        self._printer = printer
        self._name = name
        self._values = values
        self.error = FakeError

    def get_name(self):
        return self._name

    def get_printer(self):
        return self._printer

    def lookup_object(self, name):
        return self._printer.lookup_object(name)

    def _raw(self, option, default):
        if option in self._values:
            return self._values[option]
        if default is _UNSET:
            raise FakeError(
                "Option '%s' missing in [%s]" % (option, self._name)
            )
        return default

    def get(self, option, default=_UNSET, note_valid=True):
        return self._raw(option, default)

    def getfloat(
        self,
        option,
        default=_UNSET,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        val = self._raw(option, default)
        if val is None:
            return None
        return float(val)

    def getint(self, option, default=_UNSET, minval=None, note_valid=True):
        val = self._raw(option, default)
        if val is None:
            return None
        return int(val)

    def getboolean(self, option, default=_UNSET, note_valid=True):
        val = self._raw(option, default)
        if val is None or isinstance(val, bool):
            return val
        return str(val).strip().lower() in ("1", "true", "yes", "on")

    def getchoice(self, option, choices, default=_UNSET, note_valid=True):
        val = self._raw(option, default)
        if val not in choices:
            raise FakeError(
                "Choice '%s' for option '%s' in [%s] is not valid"
                % (val, option, self._name)
            )
        return choices[val]

    def getlist(
        self,
        option,
        default=_UNSET,
        seps=(",",),
        count=None,
        note_valid=True,
    ):
        val = self._raw(option, default)
        if val is None:
            return None
        if isinstance(val, (list, tuple)):
            return list(val)
        return [p.strip() for p in val.split(",") if p.strip()]

    def getlists(
        self,
        option,
        default=_UNSET,
        seps=(",",),
        count=None,
        parser=str,
        note_valid=True,
    ):
        return self._raw(option, default)


class FakeBridge:
    def __init__(self):
        self.set_position_calls = []

    def set_position(self, x, y, z):
        self.set_position_calls.append((x, y, z))


class FakeMotion:
    def __init__(self, axis_sections=()):
        self.bridge = FakeBridge()
        self.axis_sections = list(axis_sections)
        self._limits = {
            ("z", "max_velocity"): 10.0,
            ("z", "max_accel"): 100.0,
        }

    def _axis_limit(self, axis, kind):
        return self._limits[(axis, kind)]


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
        "printer": {},
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
        "printer": {},
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


def make_kin(sections, axis_sections=()):
    printer = FakePrinter()
    config = FakeConfig(printer, sections)
    motion = FakeMotion(axis_sections)
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
    sections = cartesian_sections()
    sections.update(extra_sections)
    printer = FakePrinter()
    config = FakeConfig(printer, sections)
    motion.reject_legacy_role_sections(config)


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


def test_follower_motor_not_orphan():
    sections = cartesian_sections()
    sections["motor e"] = motor_section()
    axis_sections = [("e", ["x"], ["e"], [])]
    kin = make_kin(sections, axis_sections=axis_sections)
    assert kin.kind == "cartesian"


def test_missing_drive_rejected():
    sections = cartesian_sections()
    del sections["motor x"]["drive"]
    with pytest.raises(FakeError):
        make_kin(sections)


def test_read_claimed_axes_returns_role_bound_names():
    printer = FakePrinter()
    assert motion_kinematics.read_claimed_axes(
        FakeConfig(printer, corexy_sections())
    ) == ["x", "y", "z"]
    assert motion_kinematics.read_claimed_axes(
        FakeConfig(printer, cartesian_sections())
    ) == ["x", "y", "z"]


def test_read_claimed_axes_unknown_type_rejected():
    sections = corexy_sections()
    sections["kinematics"]["type"] = "bogus"
    with pytest.raises(FakeError):
        motion_kinematics.read_claimed_axes(FakeConfig(FakePrinter(), sections))


def test_read_claimed_axes_missing_section_rejected():
    sections = corexy_sections()
    del sections["kinematics"]
    with pytest.raises(FakeError):
        motion_kinematics.read_claimed_axes(FakeConfig(FakePrinter(), sections))


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
        "rotation_distance": 40.0,
        "encoder_counts_per_rev": 131072,
    }
    from test_servo_homing import FakeRailConfig

    return servo_axis.ServoRail(
        FakeRailConfig("axis z", axis_opts),
        FakeRailConfig("z_drive", motor_opts),
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


def test_clear_parked_dirty_subset():
    kin = _homed_cartesian_with_servo_z()
    kin._parked_dirty = [True, False, True]
    kin.clear_parked_dirty([0])
    assert kin.parked_dirty_axes() == [2]
