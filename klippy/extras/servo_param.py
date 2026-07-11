import contextlib
import os
import time

TYPE_TOKENS = {
    "u8": (1, 0, 0xFF),
    "u16": (2, 0, 0xFFFF),
    "u32": (4, 0, 0xFFFFFFFF),
    "i8": (1, -(1 << 7), (1 << 7) - 1),
    "i16": (2, -(1 << 15), (1 << 15) - 1),
    "i32": (4, -(1 << 31), (1 << 31) - 1),
}
PROBED_MIN = -(1 << 31)
PROBED_MAX = (1 << 32) - 1


def _parse_int(text):
    t = text.strip().lower()
    if t.startswith("0x") or t.startswith("-0x"):
        return int(t, 16)
    return int(t, 10)


def parse_address(text):
    parts = text.strip().split(".")
    if len(parts) != 2 or not parts[0].lower().startswith("0x"):
        raise ValueError("address %r: expected 0xINDEX.SUB" % (text,))
    try:
        index = int(parts[0], 16)
        subindex = _parse_int(parts[1])
    except ValueError:
        raise ValueError("address %r: expected 0xINDEX.SUB" % (text,))
    if not 0 <= index <= 0xFFFF:
        raise ValueError("address %r: index out of 16-bit range" % (text,))
    if not 0 <= subindex <= 0xFF:
        raise ValueError("address %r: subindex out of 8-bit range" % (text,))
    return index, subindex


def check_value(value, type_token):
    if type_token is None:
        if not PROBED_MIN <= value <= PROBED_MAX:
            raise ValueError("value %d out of 32-bit range" % (value,))
        return 0
    size, vmin, vmax = TYPE_TOKENS[type_token]
    if not vmin <= value <= vmax:
        raise ValueError(
            "value %d out of range for %s [%d..%d]"
            % (value, type_token, vmin, vmax)
        )
    return size


def parse_param_entry(line):
    addr_text, sep, rest = line.partition(":")
    if not sep:
        raise ValueError(
            "param %r: expected '0xINDEX.SUB: [type] value'" % (line,)
        )
    index, subindex = parse_address(addr_text)
    fields = rest.split()
    if len(fields) == 1:
        type_token = None
        value_text = fields[0]
    elif len(fields) == 2:
        type_token = fields[0]
        if type_token not in TYPE_TOKENS:
            raise ValueError(
                "param %r: unknown type %r (use u8/u16/u32/i8/i16/i32)"
                % (line, type_token)
            )
        value_text = fields[1]
    else:
        raise ValueError(
            "param %r: expected '0xINDEX.SUB: [type] value'" % (line,)
        )
    try:
        value = _parse_int(value_text)
    except ValueError:
        raise ValueError("param %r: bad value %r" % (line, value_text))
    try:
        size = check_value(value, type_token)
    except ValueError as e:
        raise ValueError("param %r: %s" % (line, e))
    return index, subindex, size, value


def parse_params_block(text):
    entries = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        entries.append(parse_param_entry(line))
    return entries


TUNING_PROFILE_DIR = "~/printer_data/config/servo_tuning"


def tuning_profile_path(name):
    return os.path.join(
        os.path.expanduser(TUNING_PROFILE_DIR), name + ".params"
    )


def load_tuning_profile(name):
    """Parse a [motor] tuning_profile: file, same syntax as params:."""
    path = tuning_profile_path(name)
    if not os.path.isfile(path):
        raise ValueError(
            "tuning_profile %r not found (expected %s)" % (name, path)
        )
    with open(path) as f:
        text = f.read()
    return parse_params_block(text), path


def decode_typed(raw, size, type_token):
    bits = 8 * size
    unsigned = raw & ((1 << bits) - 1)
    if type_token.startswith("i"):
        return unsigned - (1 << bits) if unsigned >> (bits - 1) else unsigned
    return unsigned


def format_value(index, subindex, size, raw, type_token):
    if not 1 <= size <= 4:
        raise ValueError("SDO object size %d outside 1..4" % (size,))
    bits = 8 * size
    unsigned = raw & ((1 << bits) - 1)
    signed = unsigned - (1 << bits) if unsigned >> (bits - 1) else unsigned
    hex_text = "0x%0*x" % (size * 2, unsigned)
    if type_token is not None:
        shown = signed if type_token.startswith("i") else unsigned
        return "0x%04x.%d = %s (%s: %d)" % (
            index,
            subindex,
            hex_text,
            type_token,
            shown,
        )
    return "0x%04x.%d = %s (u%d: %d, i%d: %d)" % (
        index,
        subindex,
        hex_text,
        bits,
        unsigned,
        bits,
        signed,
    )


_PARAM_WRITES = []
_WRITE_LOG_SUPPRESSED = 0


@contextlib.contextmanager
def suppress_write_log():
    """Machinery-issued SERVO_PARAM writes (sweep adapters applying their own
    step values) are part of the run that issues them, not a change the user
    made between runs — keep them out of the between-runs journal."""
    global _WRITE_LOG_SUPPRESSED
    _WRITE_LOG_SUPPRESSED += 1
    try:
        yield
    finally:
        _WRITE_LOG_SUPPRESSED -= 1


def record_param_write(servo, addr, value):
    """Log a successful SERVO_PARAM SET so the next calibration run can record
    what changed since it last ran (session-scoped, drained at run start)."""
    if _WRITE_LOG_SUPPRESSED:
        return
    _PARAM_WRITES.append(
        {
            "servo": servo,
            "addr": addr,
            "value": value,
            "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
    )


def drain_param_writes():
    writes = list(_PARAM_WRITES)
    _PARAM_WRITES.clear()
    return writes


def read_param(printer, node, slot, index, subindex):
    """Resolve node's engine handle and read one SDO object. Shared by
    cmd_SERVO_PARAM and any other extra that needs a drive readback."""
    handle = node.get_engine_handle()
    if handle is None:
        raise RuntimeError(
            "ethercat_node %s has no engine handle" % (node.name,)
        )
    engine = printer.lookup_object("motion_engine")
    return engine.sdo_read(handle, slot, index, subindex)


class ServoParam:
    cmd_SERVO_PARAM_help = (
        "Read/write a raw CoE SDO object on an EtherCAT servo drive"
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_PARAM",
            self.cmd_SERVO_PARAM,
            desc=self.cmd_SERVO_PARAM_help,
        )

    def _resolve_node(self, servo_name):
        from . import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo_name, "SERVO_PARAM"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def cmd_SERVO_PARAM(self, gcmd):
        servo_name = gcmd.get("SERVO")
        node, slot = self._resolve_node(servo_name)
        get_addr = gcmd.get("GET", None)
        set_addr = gcmd.get("SET", None)
        if (get_addr is None) == (set_addr is None):
            raise gcmd.error("SERVO_PARAM: specify exactly one of GET or SET")
        type_token = gcmd.get("TYPE", None)
        if type_token is not None and type_token not in TYPE_TOKENS:
            raise gcmd.error(
                "SERVO_PARAM: unknown TYPE %r (use u8/u16/u32/i8/i16/i32)"
                % (type_token,)
            )
        try:
            if get_addr is not None:
                index, subindex = parse_address(get_addr)
                size, raw = read_param(
                    self.printer, node, slot, index, subindex
                )
                gcmd.respond_info(
                    format_value(index, subindex, size, raw, type_token)
                )
            else:
                handle = node.get_engine_handle()
                if handle is None:
                    raise RuntimeError(
                        "ethercat_node %s has no engine handle" % (node.name,)
                    )
                engine = self.printer.lookup_object("motion_engine")
                index, subindex = parse_address(set_addr)
                value = _parse_int(gcmd.get("VALUE"))
                size = check_value(value, type_token)
                rb_size, rb_raw = engine.sdo_write(
                    handle, slot, index, subindex, size, value
                )
                settled = format_value(
                    index, subindex, rb_size, rb_raw, type_token
                )
                record_param_write(servo_name, set_addr, value)
                gcmd.respond_info("set " + settled)
        except ValueError as e:
            raise gcmd.error("SERVO_PARAM: %s" % (e,))
        except RuntimeError as e:
            raise gcmd.error("SERVO_PARAM: %s" % (e,))


def load_config(config):
    return ServoParam(config)
