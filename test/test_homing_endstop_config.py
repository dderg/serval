import pytest
from fakes import (
    FakeConfig,
    FakeConfigError,
    FakeKin,
    FakeMcu,
    FakePrinter,
    FakeRail,
    FakeStepper,
)

from klippy.extras.homing import Homing
from klippy.mcu import STEPPING_MODE_PIECE, STEPPING_MODE_STEPCOMPRESS
from klippy.motion_endstop import entry_endstops


class RecordingQueryEndstops:
    def __init__(self):
        self.registered = []

    def register_endstop(self, endstop, name):
        self.registered.append((endstop, name))


class Pins:
    def __init__(self, chips):
        self._chips = chips

    def parse_pin(self, pin_desc, can_invert=False, can_pullup=False):
        chip, chip_name = self._chips[pin_desc]
        return {
            "chip": chip,
            "chip_name": chip_name,
            "pin": pin_desc,
            "invert": 0,
            "pullup": 0,
        }


class VirtualChip:
    def setup_motion_endstop(self, pin_params, axis_index):
        return VirtualEndstop()

    def get_position_endstop(self):
        return 1.75


class VirtualEndstop:
    endstop_id = 9

    def engine_mcu_handle(self):
        return 0


def _mcu(printer, name="mcu", stepping_mode=STEPPING_MODE_PIECE):
    return FakeMcu(printer=printer, name=name, stepping_mode=stepping_mode)


def _kin(mcu, motor_names, axis="x", kind="cartesian", second_mcu=None):
    mcus = [mcu] * len(motor_names)
    if second_mcu is not None:
        mcus[-1] = second_mcu
    steppers = [FakeStepper(name=n, mcu=m) for n, m in zip(motor_names, mcus)]
    axis_index = "xyz".index(axis)
    rails = [FakeRail(name="rail_" + a) for a in "xyz"]
    rails[axis_index] = FakeRail(name="rail_" + axis, steppers=steppers)
    lanes = [
        (i, a, list(motor_names) if a == axis else [])
        for i, a in enumerate("xyz")
    ]
    return FakeKin(rails=rails, kind=kind, lanes=lanes)


def _resolve(endstop_pin, kin, pins, printer=None):
    printer = printer if printer is not None else FakePrinter()
    query_endstops = RecordingQueryEndstops()
    printer.add_object("pins", pins)
    printer.add_object("query_endstops", query_endstops)
    axis_config = FakeConfig(
        printer=printer, name="axis x", values={"endstop_pin": endstop_pin}
    )
    config = FakeConfig(
        printer=printer, name="printer", sections={"axis x": axis_config}
    )
    homing = Homing.__new__(Homing)
    homing.printer = printer
    homing._config = config
    homing.resolve_endstops(kin)
    return homing, query_endstops


def _keyed_setup(
    endstop_pin,
    motor_names=("stepper_x", "stepper_x1"),
    stepping_mode=STEPPING_MODE_PIECE,
    kind="cartesian",
    pin_chips=None,
    lane_second_mcu=False,
):
    printer = FakePrinter()
    mcu = _mcu(printer, stepping_mode=stepping_mode)
    second = _mcu(printer, name="mcu2") if lane_second_mcu else None
    kin = _kin(mcu, list(motor_names), kind=kind, second_mcu=second)
    chips = pin_chips if pin_chips is not None else {}
    chips.setdefault("PA0", (mcu, "mcu"))
    chips.setdefault("PA1", (mcu, "mcu"))
    return printer, mcu, kin, Pins(chips), endstop_pin


KEYED_PINS = "\nstepper_x: PA0\nstepper_x1: PA1\n"


def test_keyed_endstops_bind_each_motor_to_its_lane_slot():
    printer, mcu, kin, pins, pin_text = _keyed_setup(KEYED_PINS)
    homing, query_endstops = _resolve(pin_text, kin, pins, printer)
    entry = homing._axes[0]
    endstops = entry_endstops(entry)
    assert [e.motor_name for e in endstops] == ["stepper_x", "stepper_x1"]
    assert [e.binding.stepper_idx for e in endstops] == [0, 1]
    assert all(e.binding.lane_idx == 0 for e in endstops)
    assert [name for _, name in query_endstops.registered] == [
        "x:stepper_x",
        "x:stepper_x1",
    ]
    assert "endstop" not in entry


def test_keyed_endstop_missing_a_motor_is_rejected():
    printer, mcu, kin, pins, _ = _keyed_setup(KEYED_PINS)
    with pytest.raises(FakeConfigError, match="is missing motor"):
        _resolve("\nstepper_x: PA0\n", kin, pins, printer)


def test_keyed_endstop_naming_an_unknown_motor_is_rejected():
    printer, mcu, kin, pins, _ = _keyed_setup(KEYED_PINS)
    pins._chips["PA2"] = (mcu, "mcu")
    with pytest.raises(FakeConfigError, match="does not drive this axis"):
        _resolve(
            "\nstepper_x: PA0\nstepper_x1: PA1\nstepper_z: PA2\n",
            kin,
            pins,
            printer,
        )


def test_keyed_endstop_duplicate_motor_key_is_rejected():
    printer, mcu, kin, pins, _ = _keyed_setup(KEYED_PINS)
    with pytest.raises(FakeConfigError, match="twice"):
        _resolve("\nstepper_x: PA0\nstepper_x: PA1\n", kin, pins, printer)


def test_keyed_endstop_on_corexy_shared_lane_is_rejected():
    printer, mcu, kin, pins, pin_text = _keyed_setup(KEYED_PINS, kind="corexy")
    with pytest.raises(FakeConfigError, match="shared lane"):
        _resolve(pin_text, kin, pins, printer)


def test_keyed_endstop_pin_on_a_foreign_mcu_is_rejected():
    printer = FakePrinter()
    mcu = _mcu(printer)
    other = _mcu(printer, name="mcu2")
    kin = _kin(mcu, ["stepper_x", "stepper_x1"])
    pins = Pins({"PA0": (mcu, "mcu"), "PA1": (other, "mcu2")})
    with pytest.raises(
        FakeConfigError, match="must be wired to its own motor's MCU"
    ):
        _resolve(KEYED_PINS, kin, pins, printer)


def test_keyed_endstop_lane_split_across_mcus_is_rejected():
    printer, mcu, kin, pins, pin_text = _keyed_setup(
        KEYED_PINS, lane_second_mcu=True
    )
    with pytest.raises(FakeConfigError, match="must live on one MCU"):
        _resolve(pin_text, kin, pins, printer)


def test_keyed_endstop_with_virtual_provider_chip_is_rejected():
    printer, mcu, kin, pins, pin_text = _keyed_setup(
        KEYED_PINS, pin_chips={"PA1": (VirtualChip(), "probe")}
    )
    with pytest.raises(
        FakeConfigError, match="virtual endstops drive one switch per axis"
    ):
        _resolve(pin_text, kin, pins, printer)


def test_keyed_endstop_on_classic_stepcompress_mcu_is_rejected():
    printer, mcu, kin, pins, pin_text = _keyed_setup(
        KEYED_PINS, stepping_mode=STEPPING_MODE_STEPCOMPRESS
    )
    with pytest.raises(
        FakeConfigError, match="requires motion-runtime stepping"
    ):
        _resolve(pin_text, kin, pins, printer)
    assert mcu.config_cmds == []
    assert mcu.config_callbacks == []


def test_single_endstop_entry_carries_both_shapes():
    printer = FakePrinter()
    mcu = _mcu(printer)
    kin = _kin(mcu, ["stepper_x"])
    homing, _ = _resolve("PA0", kin, Pins({"PA0": (mcu, "mcu")}), printer)
    entry = homing._axes[0]
    assert entry["endstop"] is entry["endstops"][0]
    assert entry_endstops(entry) == entry["endstops"]


def test_provider_entry_carries_both_shapes_and_trigger_position():
    printer = FakePrinter()
    chip = VirtualChip()
    kin = _kin(_mcu(printer), ["stepper_x"])
    homing, _ = _resolve(
        "virtual_endstop",
        kin,
        Pins({"virtual_endstop": (chip, "probe")}),
        printer,
    )
    entry = homing._axes[0]
    assert entry["provider"] is chip
    assert entry["trigger_position"] == 1.75
    assert entry["endstop"] is entry["endstops"][0]


def test_entry_endstops_accepts_the_legacy_single_endstop_shape():
    endstop = VirtualEndstop()
    assert entry_endstops({"endstop": endstop, "provider": None}) == [endstop]
