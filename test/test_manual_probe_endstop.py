import types

import pytest
from klippy_testing import PrinterShim, Restart

from klippy.extras import manual_probe

POSITION_ENDSTOP = 0.5
GCODE_OFFSET = 0.2

PRINTER_CFG = """\
[mcu]
serial: /dev/ttyACM0

[axis z]
endstop_pin: ^PE5
position_endstop: {position_endstop}
position_max: 200

[manual_probe]
"""


class FakeGcodeMove:
    def __init__(self, z_offset):
        self.homing_origin = types.SimpleNamespace(z=z_offset)

    def get_status(self, eventtime=None):
        return {"homing_origin": self.homing_origin}


class ManualProbeShim(PrinterShim):
    def load_object(self, config, name):
        return self.lookup_object(name)


def _manual_probe(tmp_path, position_endstop):
    config_file = tmp_path / "printer.cfg"
    config_file.write_text(
        PRINTER_CFG.format(position_endstop=position_endstop)
        if position_endstop is not None
        else PRINTER_CFG.replace("position_endstop: {position_endstop}\n", "")
    )
    printer = ManualProbeShim({"config_file": str(config_file)})
    printer.add_object("gcode_move", FakeGcodeMove(GCODE_OFFSET))
    config = printer.load_config()
    probe = manual_probe.ManualProbe(config.getsection("manual_probe"))
    return printer, config_file, probe


def test_z_offset_apply_endstop_is_registered_for_a_z_position_endstop(
    tmp_path,
):
    printer, _, _ = _manual_probe(tmp_path, POSITION_ENDSTOP)

    commands = printer.lookup_object("gcode").ready_gcode_handlers

    assert "Z_OFFSET_APPLY_ENDSTOP" in commands
    assert "Z_ENDSTOP_CALIBRATE" in commands


def test_z_offset_apply_endstop_is_absent_without_a_z_position_endstop(
    tmp_path,
):
    printer, _, _ = _manual_probe(tmp_path, None)

    commands = printer.lookup_object("gcode").ready_gcode_handlers

    assert "Z_OFFSET_APPLY_ENDSTOP" not in commands
    assert "Z_ENDSTOP_CALIBRATE" not in commands


def test_z_offset_apply_endstop_saves_the_axis_z_section(tmp_path):
    printer, config_file, _ = _manual_probe(tmp_path, POSITION_ENDSTOP)
    configfile = printer.lookup_object("configfile")

    printer.call("Z_OFFSET_APPLY_ENDSTOP")

    expected = "%.3f" % (POSITION_ENDSTOP - GCODE_OFFSET)
    assert configfile.status_save_pending == {
        "axis z": {"position_endstop": expected}
    }
    with pytest.raises(Restart):
        printer.call("SAVE_CONFIG")
    saved = config_file.read_text()
    assert "#*# [axis z]" in saved
    assert "#*# position_endstop = %s" % (expected,) in saved
    assert "stepper_z" not in saved


def test_z_endstop_calibrate_saves_the_axis_z_section(tmp_path):
    printer, _, probe = _manual_probe(tmp_path, POSITION_ENDSTOP)
    configfile = printer.lookup_object("configfile")
    kinematic_z = 0.1

    probe.z_endstop_finalize([0.0, 0.0, kinematic_z])

    expected = "%.3f" % (POSITION_ENDSTOP - kinematic_z)
    assert configfile.status_save_pending == {
        "axis z": {"position_endstop": expected}
    }
