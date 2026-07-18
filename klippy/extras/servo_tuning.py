"""Servo drive tuning: profile capture/replay plus the tuning panel backend
(curated parameter map, SERVO_DUMP_TUNING, SERVO_TUNE). See
docs/rewrite/servo-tuning-profiles.md.
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass, field
from typing import Any

from . import servo_axis, servo_calibration, servo_param, servo_strokes

INERTIA_RATIO_ADDR = "0x2000.0x07"
GAIN_NAMES = ("position", "speed", "integral")
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")

_CCODE_RE = re.compile(r"^[Cc](\d{2})\.([0-9A-Fa-f]{2})$")


def c_code_to_addr(c_code: str) -> str:
    """Drive datasheet CGG.NN -> "0xINDEX.SUB": index is 0x2000 + the group
    number, subindex is the code digits read as hex, plus one (the drive's
    SDO objects are 1-based where the datasheet code is 0-based). NN is a
    hex byte in the A6-EC manual (the notch bank runs C01.49, C01.4A,
    C01.4B, ...)."""
    m = _CCODE_RE.match(c_code.strip())
    if not m:
        raise ValueError("C-code %r: expected CGG.NN (e.g. C01.30)" % (c_code,))
    group_text, code_text = m.groups()
    index = 0x2000 + int(group_text, 10)
    subindex = int(code_text, 16) + 1
    return "0x%04x.0x%02x" % (index, subindex)


def _addr_key(addr_text: str) -> tuple[int, int]:
    return servo_param.parse_address(addr_text)


@dataclass
class PanelParam:
    """One curated drive register the tuning panel can read or write."""

    name: str
    c_code: str
    unit: str
    group: str
    description: str
    type_token: str = "u16"
    options: dict[int, str] | None = None
    addr: str = field(init=False, default="")

    def __post_init__(self) -> None:
        self.addr = c_code_to_addr(self.c_code)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "c_code": self.c_code,
            "addr": self.addr,
            "type": self.type_token,
            "unit": self.unit,
            "group": self.group,
            "description": self.description,
            "options": (
                None
                if self.options is None
                else {str(k): v for k, v in self.options.items()}
            ),
        }


PANEL_PARAMS: tuple[PanelParam, ...] = (
    PanelParam(
        name="position_gain",
        c_code="C01.00",
        unit="0.1 rad/s",
        group="gains",
        description="C01.00 position loop gain",
    ),
    PanelParam(
        name="speed_gain",
        c_code="C01.01",
        unit="0.1 Hz",
        group="gains",
        description="C01.01 speed loop gain",
    ),
    PanelParam(
        name="integral_time",
        c_code="C01.02",
        unit="0.01 ms",
        group="gains",
        description="C01.02 speed integral time",
    ),
    PanelParam(
        name="torque_filter_cutoff",
        c_code="C01.03",
        unit="Hz",
        group="filters",
        description=(
            "C01.03 1st torque reference filter cutoff frequency; lower "
            "filters more but adds delay (manual 7.3, range 5-16000, "
            "drive default 200)"
        ),
    ),
    *(
        PanelParam(
            name="notch_%d_%s" % (n, kind),
            c_code="C01.%02X" % (0x40 + (n - 1) * 3 + kind_offset),
            unit="Hz" if kind == "freq" else "0.1%",
            group="notch",
            description=(
                "C01.%02X %s of the %d%s notch (manual 7.10; notches 1-2 "
                "are overwritten by the drive while adaptive_notch_mode "
                "is 1 or 2)"
                % (
                    0x40 + (n - 1) * 3 + kind_offset,
                    {
                        "freq": "center frequency",
                        "width": "width level",
                        "depth": "depth level",
                    }[kind],
                    n,
                    {1: "st", 2: "nd", 3: "rd"}.get(n, "th"),
                )
            ),
        )
        for n in range(1, 6)
        for kind_offset, kind in enumerate(("freq", "width", "depth"))
    ),
    PanelParam(
        name="adaptive_notch_mode",
        c_code="C01.30",
        unit="",
        group="notch",
        description=(
            "C01.30 adaptive notch mode (manual 7.10): 0=disabled, "
            "1=1st notch adaptive, 2=1st+2nd notches adaptive, "
            "3=reset notch parameters, 4=test resonance frequency only"
        ),
        options={
            0: "disabled",
            1: "1 adaptive notch",
            2: "2 adaptive notches",
            3: "reset notch params",
            4: "test resonance only",
        },
    ),
    PanelParam(
        name="speed_feedback_filter",
        c_code="C01.10",
        unit="",
        group="speed_observer",
        description=(
            "C01.10 speed feedback filter (manual 7.11); 3 enables the "
            "speed observer; the drive only accepts changes at stop"
        ),
        options={
            0: "internal setting",
            1: "low-pass filter",
            2: "overlapping average",
            3: "speed observer",
            4: "no filter",
        },
    ),
    PanelParam(
        name="speed_observer_gain",
        c_code="C02.30",
        unit="0.1 Hz",
        group="speed_observer",
        description=(
            "C02.30 speed observer gain; higher observes faster, too "
            "high oscillates (manual 7.11)"
        ),
    ),
    PanelParam(
        name="speed_observer_inertia",
        c_code="C02.31",
        unit="0.1%",
        group="speed_observer",
        description=(
            "C02.31 speed observer inertia correction; corrects for an "
            "inaccurate inertia_ratio (manual 7.11, default 1000 = 100%)"
        ),
    ),
    PanelParam(
        name="speed_observer_cutoff",
        c_code="C02.32",
        unit="Hz",
        group="speed_observer",
        description=(
            "C02.32 speed observer speed feedback low-pass cutoff "
            "frequency (manual 7.11)"
        ),
    ),
    PanelParam(
        name="disturbance_gain",
        c_code="C02.60",
        unit="0.1 Hz",
        group="disturbance_observer",
        description=(
            "C02.60 disturbance observer gain; higher responds to "
            "disturbances faster, too high vibrates (manual 7.12)"
        ),
    ),
    PanelParam(
        name="disturbance_inertia",
        c_code="C02.61",
        unit="0.1%",
        group="disturbance_observer",
        description=(
            "C02.61 disturbance observer inertia correction coefficient "
            "(manual 7.12, default 1000 = 100%)"
        ),
    ),
    PanelParam(
        name="disturbance_cutoff",
        c_code="C02.62",
        unit="Hz",
        group="disturbance_observer",
        description=(
            "C02.62 disturbance observer low-pass cutoff frequency "
            "(manual 7.12)"
        ),
    ),
    PanelParam(
        name="disturbance_comp_torque",
        c_code="C02.63",
        unit="0.1%",
        group="disturbance_observer",
        description=(
            "C02.63 disturbance observer compensation torque percentage "
            "(manual 7.12)"
        ),
    ),
    PanelParam(
        name="gain_mode",
        c_code="C00.04",
        unit="",
        group="load",
        description=(
            "C00.04 auto-tuning mode: 0=manual, 1=standard/stiffness table"
        ),
        options={0: "manual", 1: "stiffness table"},
    ),
    PanelParam(
        name="stiffness_level",
        c_code="C00.05",
        unit="",
        group="load",
        description=(
            "C00.05 stiffness level 1-31, used when gain_mode is the "
            "stiffness table (manual 7.2, default 12)"
        ),
    ),
    PanelParam(
        name="inertia_ratio",
        c_code="C00.06",
        unit="%",
        group="load",
        description="C00.06 load inertia ratio",
    ),
)


def validate_param_map(params: list[PanelParam]) -> None:
    """Fail loud on a broken map: every type_token must be a real SDO type,
    every name and every resolved address must be unique."""
    names: dict[str, PanelParam] = {}
    addrs: dict[tuple[int, int], PanelParam] = {}
    for p in params:
        if p.type_token not in servo_param.TYPE_TOKENS:
            raise ValueError(
                "param %r: unknown type %r (use u8/u16/u32/i8/i16/i32)"
                % (p.name, p.type_token)
            )
        if p.name in names:
            raise ValueError("duplicate param name %r" % (p.name,))
        key = _addr_key(p.addr)
        if key in addrs:
            raise ValueError(
                "params %r and %r both target %s"
                % (addrs[key].name, p.name, p.addr)
            )
        names[p.name] = p
        addrs[key] = p


validate_param_map(list(PANEL_PARAMS))


def _parse_extra_params(config: Any) -> list[PanelParam]:
    text = config.get("extra_params", "")
    entries: list[PanelParam] = []
    for lineno, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 5:
            raise config.error(
                "[servo_tuning] extra_params line %d: expected 'name "
                "C-code type unit group', got %r" % (lineno, line)
            )
        name, c_code, type_token, unit, group = fields
        if type_token not in servo_param.TYPE_TOKENS:
            raise config.error(
                "[servo_tuning] extra_params line %d: unknown type %r "
                "(use u8/u16/u32/i8/i16/i32)" % (lineno, type_token)
            )
        try:
            entries.append(
                PanelParam(
                    name=name,
                    c_code=c_code,
                    unit=unit,
                    group=group,
                    description="",
                    type_token=type_token,
                )
            )
        except ValueError as e:
            raise config.error(
                "[servo_tuning] extra_params line %d: %s" % (lineno, e)
            )
    return entries


def _build_param_map(config: Any) -> list[PanelParam]:
    params = list(PANEL_PARAMS) + _parse_extra_params(config)
    try:
        validate_param_map(params)
    except ValueError as e:
        raise config.error("[servo_tuning] %s" % (e,))
    return params


def _resolve_captures_root(config: Any) -> str:
    root = servo_calibration.DEFAULT_CAPTURES_ROOT
    if config.has_section("servo_calibration"):
        root = config.getsection("servo_calibration").get("captures_root", root)
    return os.path.expanduser(root)


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class ServoTuning:
    cmd_SERVO_SAVE_TUNING_help = (
        "Read gain set, inertia ratio and any ADDRS back from a servo drive "
        "and write ~/printer_data/config/servo_tuning/<NAME>.params. Params "
        "SERVO NAME [ADDRS=addr[:type],...] (never overwrites NAME)"
    )
    cmd_SERVO_DUMP_TUNING_help = (
        "Read every mapped drive-tuning parameter from one or more servo "
        "motors and write <captures_root>/drive_state.json (atomic "
        "tmp+rename). Params MOTORS=all|<list> (default: every servo "
        "motor)"
    )
    cmd_SERVO_TUNE_help = (
        "Write one drive register on one or more servo motors, journal the "
        "write like SERVO_PARAM SET, and verify it by readback. PARAM "
        "resolves a panel name, a C-code (CGG.NN), or a raw 0xINDEX.SUB "
        "address; an unmapped raw address needs TYPE= (default u16). "
        "Params PARAM VALUE [MOTORS=all|<list>] [TYPE=u8/u16/u32/i8/i16/"
        "i32]"
    )

    def __init__(self, config: Any) -> None:
        self.printer = config.get_printer()
        self.params = _build_param_map(config)
        self._by_name = {p.name: p for p in self.params}
        self._by_addr = {_addr_key(p.addr): p for p in self.params}
        self.captures_root = _resolve_captures_root(config)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_SAVE_TUNING",
            self.cmd_SERVO_SAVE_TUNING,
            desc=self.cmd_SERVO_SAVE_TUNING_help,
        )
        gcode.register_command(
            "SERVO_DUMP_TUNING",
            self.cmd_SERVO_DUMP_TUNING,
            desc=self.cmd_SERVO_DUMP_TUNING_help,
        )
        gcode.register_command(
            "SERVO_TUNE",
            self.cmd_SERVO_TUNE,
            desc=self.cmd_SERVO_TUNE_help,
        )

    def _resolve_node_slot(self, servo_name: str) -> tuple[Any, int]:
        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo_name, "SERVO_SAVE_TUNING"
        )
        return self._node_slot_for_motor(motor)

    def _read_typed(
        self, node: Any, slot: int, addr_text: str, type_token: str
    ) -> tuple[int, int, str, int]:
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

    def _parse_addrs(self, addrs_text: str | None) -> list[tuple[str, str]]:
        entries: list[tuple[str, str]] = []
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

    def cmd_SERVO_SAVE_TUNING(self, gcmd: Any) -> None:
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

    def _node_slot_for_motor(
        self, motor: servo_axis.ServoMotor
    ) -> tuple[Any, int]:
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _all_servo_motors(
        self,
    ) -> list[tuple[servo_axis.ServoRail, servo_axis.ServoMotor]]:
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        return list(servo_axis.iter_servo_motors(kin))

    def _resolve_motors(
        self, gcmd: Any, motors_text: str | None
    ) -> list[tuple[servo_axis.ServoRail, servo_axis.ServoMotor]]:
        pairs = self._all_servo_motors()
        if not pairs:
            raise gcmd.error("no servo motors configured")
        if motors_text is None or motors_text.strip().lower() == "all":
            return pairs
        by_name = {m.get_motor_name(): (r, m) for r, m in pairs}
        wanted = [t.strip() for t in motors_text.split(",") if t.strip()]
        if not wanted:
            raise gcmd.error("MOTORS= lists no usable names")
        result = []
        for name in wanted:
            if name not in by_name:
                raise gcmd.error(
                    "MOTORS: no servo motor named %r (known: %s)"
                    % (name, ", ".join(sorted(by_name)))
                )
            result.append(by_name[name])
        return result

    def _resolve_param(
        self, gcmd: Any, param_text: str, type_text: str | None
    ) -> tuple[str, str]:
        mapped = self._by_name.get(param_text)
        if mapped is not None:
            return mapped.addr, mapped.type_token
        addr_text: str | None
        try:
            addr_text = c_code_to_addr(param_text)
        except ValueError:
            addr_text = None
        if addr_text is None:
            try:
                servo_param.parse_address(param_text)
            except ValueError:
                raise gcmd.error(
                    "SERVO_TUNE: PARAM %r is not a mapped name, a C-code "
                    "(CGG.NN), or a raw address (0xINDEX.SUB)" % (param_text,)
                )
            addr_text = param_text
        mapped = self._by_addr.get(_addr_key(addr_text))
        if mapped is not None:
            return mapped.addr, mapped.type_token
        type_token = type_text if type_text is not None else "u16"
        if type_token not in servo_param.TYPE_TOKENS:
            raise gcmd.error(
                "SERVO_TUNE: unknown TYPE %r (use u8/u16/u32/i8/i16/i32)"
                % (type_token,)
            )
        return addr_text, type_token

    def cmd_SERVO_DUMP_TUNING(self, gcmd: Any) -> None:
        targets = self._resolve_motors(gcmd, gcmd.get("MOTORS", None))
        motors_out: dict[str, dict[str, int]] = {}
        config_pins_out: dict[str, dict[str, int]] = {}
        slots_out: dict[str, int] = {}
        for _rail, motor in targets:
            node, slot = self._node_slot_for_motor(motor)
            motor_name = motor.get_motor_name()
            slots_out[motor_name] = slot
            readings: dict[str, int] = {}
            for p in self.params:
                index, subindex = _addr_key(p.addr)
                try:
                    size, raw = servo_param.read_param(
                        self.printer, node, slot, index, subindex
                    )
                except (RuntimeError, ValueError) as e:
                    raise gcmd.error(
                        "SERVO_DUMP_TUNING: readback failed for %s %s "
                        "(%s): %s" % (motor_name, p.name, p.c_code, e)
                    )
                expected_size = servo_param.TYPE_TOKENS[p.type_token][0]
                if size != expected_size:
                    raise gcmd.error(
                        "SERVO_DUMP_TUNING: %s %s: drive reports a %d-byte "
                        "object, expected %d bytes for %s"
                        % (
                            motor_name,
                            p.c_code,
                            size,
                            expected_size,
                            p.type_token,
                        )
                    )
                readings[p.c_code] = servo_param.decode_typed(
                    raw, size, p.type_token
                )
            motors_out[motor_name] = readings
            sdo_keys = {(i, s) for i, s, _sz, _v in motor.get_sdo_params()}
            config_pins_out[motor_name] = {
                p.c_code: readings[p.c_code]
                for p in self.params
                if _addr_key(p.addr) in sdo_keys
            }
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        payload = {
            "version": 1,
            "created_utc": _utc_now(),
            "params": [p.as_dict() for p in self.params],
            "motors": motors_out,
            "config_pins": config_pins_out,
            "slots": slots_out,
            "spatial": servo_strokes.spatial_frame(kin),
        }
        os.makedirs(self.captures_root, exist_ok=True)
        path = os.path.join(self.captures_root, "drive_state.json")
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp, path)
        gcmd.respond_info(
            "SERVO_DUMP_TUNING: wrote %s (%d motors, %d params)"
            % (path, len(motors_out), len(self.params))
        )

    def _patch_drive_state(
        self, gcmd: Any, motor_names: list[str], addr: str, value: int
    ) -> None:
        """SERVO_TUNE just readback-verified `value` on every motor, so
        drive_state.json can absorb it in place - the tuning panel reloads
        the file instead of paying for a full SERVO_DUMP_TUNING re-read of
        the drives after every apply."""
        mapped = self._by_addr.get(_addr_key(addr))
        if mapped is None:
            return
        path = os.path.join(self.captures_root, "drive_state.json")
        if not os.path.exists(path):
            return
        try:
            with open(path) as f:
                payload = json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "SERVO_TUNE: writes verified, but drive_state.json at %s "
                "is unreadable (%s) - run SERVO_DUMP_TUNING to rebuild it"
                % (path, e)
            )
        for motor_name in motor_names:
            motors = payload.get("motors") or {}
            if motor_name in motors:
                motors[motor_name][mapped.c_code] = value
            pins = (payload.get("config_pins") or {}).get(motor_name)
            if pins is not None and mapped.c_code in pins:
                pins[mapped.c_code] = value
        payload["created_utc"] = _utc_now()
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp, path)

    def cmd_SERVO_TUNE(self, gcmd: Any) -> None:
        param_text = gcmd.get("PARAM")
        value = gcmd.get_int("VALUE")
        type_text = gcmd.get("TYPE", None)
        if type_text is not None and type_text not in servo_param.TYPE_TOKENS:
            raise gcmd.error(
                "SERVO_TUNE: unknown TYPE %r (use u8/u16/u32/i8/i16/i32)"
                % (type_text,)
            )
        addr, type_token = self._resolve_param(gcmd, param_text, type_text)
        try:
            size = servo_param.check_value(value, type_token)
        except ValueError as e:
            raise gcmd.error("SERVO_TUNE: %s" % (e,))
        index, subindex = _addr_key(addr)
        targets = self._resolve_motors(gcmd, gcmd.get("MOTORS", None))
        engine = self.printer.lookup_object("motion_engine")
        motor_names: list[str] = []
        for _rail, motor in targets:
            node, slot = self._node_slot_for_motor(motor)
            handle = node.get_engine_handle()
            if handle is None:
                raise gcmd.error(
                    "SERVO_TUNE: ethercat_node %s has no engine handle"
                    % (node.name,)
                )
            try:
                rb_size, rb_raw = engine.sdo_write(
                    handle, slot, index, subindex, size, value
                )
            except (RuntimeError, ValueError) as e:
                raise gcmd.error(
                    "SERVO_TUNE: write failed for %s %s: %s"
                    % (motor.get_motor_name(), addr, e)
                )
            servo_param.record_param_write(motor.get_motor_name(), addr, value)
            if rb_size != size:
                raise gcmd.error(
                    "SERVO_TUNE: %s %s: drive reports a %d-byte object, "
                    "expected %d bytes"
                    % (motor.get_motor_name(), addr, rb_size, size)
                )
            rb_value = servo_param.decode_typed(rb_raw, rb_size, type_token)
            if rb_value != value:
                raise gcmd.error(
                    "SERVO_TUNE: readback mismatch on %s %s: wrote %d, "
                    "read %d" % (motor.get_motor_name(), addr, value, rb_value)
                )
            motor_names.append(motor.get_motor_name())
        self._patch_drive_state(gcmd, motor_names, addr, value)
        gcmd.respond_info(
            "SERVO_TUNE: %s = %d on %s"
            % (param_text, value, ", ".join(motor_names))
        )


def load_config(config: Any) -> ServoTuning:
    return ServoTuning(config)
