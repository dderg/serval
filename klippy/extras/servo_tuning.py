import os
import re
import time

from . import servo_axis, servo_calibration, servo_param

INERTIA_RATIO_ADDR = "0x2000.0x07"
GAIN_NAMES = ("position", "speed", "integral")
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class ServoTuning:
    cmd_SERVO_SAVE_TUNING_help = (
        "Read gain set, inertia ratio and any ADDRS back from a servo drive "
        "and write ~/printer_data/config/servo_tuning/<NAME>.params. Params "
        "SERVO NAME [ADDRS=addr[:type],...] (never overwrites NAME)"
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_SAVE_TUNING",
            self.cmd_SERVO_SAVE_TUNING,
            desc=self.cmd_SERVO_SAVE_TUNING_help,
        )

    def _resolve_node_slot(self, servo_name):
        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo_name, "SERVO_SAVE_TUNING"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _read_typed(self, node, slot, addr_text, type_token):
        index, subindex = servo_param.parse_address(addr_text)
        size, raw = servo_param.read_param(
            self.printer, node, slot, index, subindex
        )
        expected_size = servo_param.TYPE_TOKENS[type_token][0]
        if size != expected_size:
            raise ValueError(
                "0x%04x.%d: drive reports a %d-byte object, expected %d "
                "bytes for type %s"
                % (index, subindex, size, expected_size, type_token)
            )
        value = servo_param.decode_typed(raw, size, type_token)
        servo_param.check_value(value, type_token)
        return index, subindex, type_token, value

    def _parse_addrs(self, addrs_text):
        entries = []
        if not addrs_text:
            return entries
        for item in addrs_text.split(","):
            item = item.strip()
            if not item:
                continue
            addr_text, sep, type_text = item.partition(":")
            type_token = type_text.strip() if sep else "u16"
            if type_token not in servo_param.TYPE_TOKENS:
                raise ValueError(
                    "ADDRS %r: unknown type %r (use u8/u16/u32/i8/i16/i32)"
                    % (item, type_token)
                )
            entries.append((addr_text.strip(), type_token))
        return entries

    def cmd_SERVO_SAVE_TUNING(self, gcmd):
        servo_name = gcmd.get("SERVO")
        name = gcmd.get("NAME")
        if not NAME_RE.match(name):
            raise gcmd.error(
                "SERVO_SAVE_TUNING: NAME %r must match [A-Za-z0-9_-]+" % (name,)
            )
        path = servo_param.tuning_profile_path(name)
        if os.path.exists(path):
            raise gcmd.error(
                "SERVO_SAVE_TUNING: profile %r already exists at %s — pick "
                "a new NAME" % (name, path)
            )
        try:
            extra_addrs = self._parse_addrs(gcmd.get("ADDRS", None))
        except ValueError as e:
            raise gcmd.error("SERVO_SAVE_TUNING: %s" % (e,))
        node, slot = self._resolve_node_slot(servo_name)
        reads = (
            [
                (servo_calibration.GAIN_PARAMS[gain_name][0], "u16")
                for gain_name in GAIN_NAMES
            ]
            + [(INERTIA_RATIO_ADDR, "u16")]
            + extra_addrs
        )
        lines = []
        try:
            for addr_text, type_token in reads:
                index, subindex, type_token, value = self._read_typed(
                    node, slot, addr_text, type_token
                )
                lines.append(
                    "0x%04x.%d: %s %d" % (index, subindex, type_token, value)
                )
        except (ValueError, RuntimeError) as e:
            raise gcmd.error("SERVO_SAVE_TUNING: %s" % (e,))
        header = [
            "# tuning profile: %s" % (name,),
            "# created_utc: %s"
            % (time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),),
            "# servo: %s" % (servo_name,),
            "# source: drive readback (SERVO_SAVE_TUNING)",
        ]
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(header + lines) + "\n")
        gcmd.respond_info("SERVO_SAVE_TUNING: wrote %s" % (path,))


def load_config(config):
    return ServoTuning(config)
