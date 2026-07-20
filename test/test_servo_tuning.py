import json

import pytest
from fakes import (
    FakeConfigError,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd
from fakes import FakePrinter as _FakePrinter

from klippy.extras import servo_axis, servo_param, servo_tuning


@pytest.fixture(autouse=True)
def _drain_writes():
    servo_param.drain_param_writes()
    yield
    servo_param.drain_param_writes()


class FakeMotorConfig:
    def __init__(self, name, values):
        self._name = name
        self._values = values

    def get_name(self):
        return self._name

    def error(self, msg):
        return FakeConfigError(msg)

    def get(self, key, default=None):
        return self._values.get(key, default)

    def getint(self, key, default=None, **kw):
        return self._values.get(key, default)

    def getfloat(self, key, default=None, **kw):
        return self._values.get(key, default)

    def getboolean(self, key, default=None, **kw):
        return self._values.get(key, default)


def _motor_config(name="motor motor_x", **overrides):
    values = {
        "protocol": "ethercat",
        "node": "node_x",
        "rotation_distance": 40.0,
        "encoder_counts_per_rev": 3200,
        "ethercat_chain_index": 0,
    }
    values.update(overrides)
    return FakeMotorConfig(name, values)


@pytest.fixture(autouse=True)
def _tuning_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(servo_param, "TUNING_PROFILE_DIR", str(tmp_path))
    return tmp_path


def test_parse_params_block_skips_comment_lines():
    text = (
        "# provenance line\n"
        "0x2001.0x01: u16 700\n"
        "# another comment\n"
        "0x2000.0x07: u16 150\n"
    )
    assert servo_param.parse_params_block(text) == [
        (0x2001, 1, 2, 700),
        (0x2000, 7, 2, 150),
    ]


def test_no_tuning_profile_leaves_params_untouched():
    motor_config = _motor_config(params="0x2010.0: u16 5")
    motor = servo_axis.ServoMotor(motor_config, False)
    assert motor.get_sdo_params() == [(0x2010, 0, 2, 5)]


def test_tuning_profile_applied_before_params_as_single_list(tmp_path):
    (tmp_path / "gainsA.params").write_text(
        "# promoted from run cal_20260710\n0x2001.0x01: u16 700\n"
    )
    motor_config = _motor_config(
        params="0x2010.0: u16 5",
        tuning_profile="gainsA",
    )
    motor = servo_axis.ServoMotor(motor_config, False)
    assert motor.get_sdo_params() == [
        (0x2001, 1, 2, 700),
        (0x2010, 0, 2, 5),
    ]


def test_tuning_profile_overlap_with_params_is_config_error(tmp_path):
    (tmp_path / "gainsA.params").write_text("0x2001.0x01: u16 700\n")
    motor_config = _motor_config(
        params="0x2001.0x01: u16 650",
        tuning_profile="gainsA",
    )
    with pytest.raises(FakeConfigError) as e:
        servo_axis.ServoMotor(motor_config, False)
    msg = str(e.value)
    assert "0x2001.1" in msg
    assert "gainsA" in msg
    assert "params" in msg


def test_tuning_profile_missing_file_is_config_error():
    motor_config = _motor_config(tuning_profile="does_not_exist")
    with pytest.raises(FakeConfigError) as e:
        servo_axis.ServoMotor(motor_config, False)
    msg = str(e.value)
    assert "does_not_exist" in msg
    assert "not found" in msg


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


class FakeReadEngine(_FakeEngine):
    def __init__(self, values, fail_addrs=None):
        super().__init__()
        self.values = values
        self.fail_addrs = fail_addrs or set()

    def sdo_read(self, handle, slot, index, subindex):
        if (index, subindex) in self.fail_addrs:
            raise RuntimeError("simulated SDO read failure")
        return self.values[(index, subindex)]


class FakePrinter(_FakePrinter):
    def __init__(self, objs):
        super().__init__(objs)
        self.objects = objs


READBACK = {
    (0x2001, 1): (2, 700),
    (0x2001, 2): (2, 550),
    (0x2001, 3): (2, 2273),
    (0x2000, 7): (2, 150),
    (0x2001, 0x31): (2, 2),
}


def _make_servo_tuning(engine, node):
    motor = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    motor.motor_name = "motor_x"
    motor.node_name = "node_x"
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis x"
    rail.axis = "x"
    rail.motors = [motor]
    printer = FakePrinter(
        {
            "toolhead": FakeToolhead(FakeKin([rail])),
            "ethercat_node node_x": node,
            "motion_engine": engine,
        }
    )
    st = servo_tuning.ServoTuning.__new__(servo_tuning.ServoTuning)
    st.printer = printer
    return st


def test_save_tuning_writes_expected_content(tmp_path):
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win1")
    st.cmd_SERVO_SAVE_TUNING(gcmd)
    text = (tmp_path / "win1.params").read_text()
    value_lines = [
        line for line in text.splitlines() if line and not line.startswith("#")
    ]
    assert value_lines == [
        "0x2001.1: u16 700",
        "0x2001.2: u16 550",
        "0x2001.3: u16 2273",
        "0x2000.7: u16 150",
    ]
    header_lines = [line for line in text.splitlines() if line.startswith("#")]
    assert any("created_utc" in line for line in header_lines)
    assert any("motor_x" in line for line in header_lines)
    assert any("drive readback" in line for line in header_lines)
    assert servo_param.parse_params_block(text) == [
        (0x2001, 1, 2, 700),
        (0x2001, 2, 2, 550),
        (0x2001, 3, 2, 2273),
        (0x2000, 7, 2, 150),
    ]
    assert any("win1.params" in r for r in gcmd.responses)


def test_save_tuning_with_addrs_extra(tmp_path):
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win2", ADDRS="0x2001.0x31")
    st.cmd_SERVO_SAVE_TUNING(gcmd)
    text = (tmp_path / "win2.params").read_text()
    assert "0x2001.49: u16 2" in text


def test_save_tuning_addrs_type_override(tmp_path):
    values = dict(READBACK)
    values[(0x2001, 0x31)] = (1, 2)
    engine = FakeReadEngine(values)
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win3", ADDRS="0x2001.0x31:u8")
    st.cmd_SERVO_SAVE_TUNING(gcmd)
    text = (tmp_path / "win3.params").read_text()
    assert "0x2001.49: u8 2" in text


def test_save_tuning_refuses_overwrite(tmp_path):
    (tmp_path / "win4.params").write_text("0x2001.0x01: u16 700\n")
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win4")
    with pytest.raises(RuntimeError, match="already exists"):
        st.cmd_SERVO_SAVE_TUNING(gcmd)


def test_save_tuning_rejects_bad_name(tmp_path):
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="../win5")
    with pytest.raises(RuntimeError, match="A-Za-z0-9"):
        st.cmd_SERVO_SAVE_TUNING(gcmd)


def test_save_tuning_no_engine_handle_is_command_error(tmp_path):
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(None))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win6")
    with pytest.raises(RuntimeError, match="no engine handle"):
        st.cmd_SERVO_SAVE_TUNING(gcmd)


def test_save_tuning_addr_size_mismatch_is_error(tmp_path):
    values = dict(READBACK)
    values[(0x2001, 0x31)] = (4, 2)
    engine = FakeReadEngine(values)
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win7", ADDRS="0x2001.0x31")
    with pytest.raises(RuntimeError, match="expected 2 bytes"):
        st.cmd_SERVO_SAVE_TUNING(gcmd)


def test_save_tuning_bad_addrs_type_is_error(tmp_path):
    engine = FakeReadEngine(dict(READBACK))
    st = _make_servo_tuning(engine, FakeNode(7))
    gcmd = FakeGcmd(SERVO="motor_x", NAME="win8", ADDRS="0x2001.0x31:q16")
    with pytest.raises(RuntimeError, match="unknown type"):
        st.cmd_SERVO_SAVE_TUNING(gcmd)


# ---- tuning panel backend: c_code_to_addr, PANEL_PARAMS, DUMP/TUNE ----

CCODE_ADDR_PAIRS = [
    ("C00.04", "0x2000.0x05"),
    ("C00.06", "0x2000.0x07"),
    ("C01.00", "0x2001.0x01"),
    ("C01.01", "0x2001.0x02"),
    ("C01.02", "0x2001.0x03"),
    ("C01.30", "0x2001.0x31"),
    ("C02.60", "0x2002.0x61"),
]


@pytest.mark.parametrize("c_code,addr", CCODE_ADDR_PAIRS)
def test_c_code_to_addr_verified_pairs(c_code, addr):
    assert servo_tuning.c_code_to_addr(c_code) == addr


def test_c_code_to_addr_rejects_bad_format():
    with pytest.raises(ValueError, match="CGG.NN"):
        servo_tuning.c_code_to_addr("C1.30")


def test_panel_params_addr_matches_c_code_rule():
    for p in servo_tuning.PANEL_PARAMS:
        assert p.addr == servo_tuning.c_code_to_addr(p.c_code)


def test_panel_params_names_are_unique():
    names = [p.name for p in servo_tuning.PANEL_PARAMS]
    assert len(names) == len(set(names))


def test_panel_params_addresses_are_unique():
    addrs = [p.addr for p in servo_tuning.PANEL_PARAMS]
    assert len(addrs) == len(set(addrs))


def test_panel_params_types_are_valid():
    for p in servo_tuning.PANEL_PARAMS:
        assert p.type_token in servo_param.TYPE_TOKENS


NOTCH_BANK_ADDR_PAIRS = [
    ("notch_1_freq", "0x2001.0x41"),
    ("notch_1_width", "0x2001.0x42"),
    ("notch_1_depth", "0x2001.0x43"),
    ("notch_2_freq", "0x2001.0x44"),
    ("notch_3_freq", "0x2001.0x47"),
    ("notch_4_freq", "0x2001.0x4a"),
    ("notch_4_width", "0x2001.0x4b"),
    ("notch_5_freq", "0x2001.0x4d"),
    ("notch_5_depth", "0x2001.0x4f"),
]


@pytest.mark.parametrize("name,addr", NOTCH_BANK_ADDR_PAIRS)
def test_manual_notch_bank_addresses(name, addr):
    """A6-EC manual 7.10: notch n occupies C01.40+3(n-1) .. +2. The bench
    config's own notes record SDO subindexes (C-code + 1), so notch 1's
    frequency C01.40 lands at 0x2001.0x41."""
    by_name = {p.name: p for p in servo_tuning.PANEL_PARAMS}
    assert by_name[name].addr == addr
    assert by_name[name].group == "notch"


def test_c_code_to_addr_accepts_hex_codes():
    assert servo_tuning.c_code_to_addr("C01.4A") == "0x2001.0x4b"
    assert servo_tuning.c_code_to_addr("C01.4e") == "0x2001.0x4f"


OBSERVER_ADDR_PAIRS = [
    ("speed_feedback_filter", "0x2001.0x11", "speed_observer"),
    ("speed_observer_gain", "0x2002.0x31", "speed_observer"),
    ("speed_observer_inertia", "0x2002.0x32", "speed_observer"),
    ("speed_observer_cutoff", "0x2002.0x33", "speed_observer"),
    ("disturbance_gain", "0x2002.0x61", "disturbance_observer"),
    ("disturbance_inertia", "0x2002.0x62", "disturbance_observer"),
    ("disturbance_cutoff", "0x2002.0x63", "disturbance_observer"),
    ("disturbance_comp_torque", "0x2002.0x64", "disturbance_observer"),
]


@pytest.mark.parametrize("name,addr,group", OBSERVER_ADDR_PAIRS)
def test_observer_addresses(name, addr, group):
    by_name = {p.name: p for p in servo_tuning.PANEL_PARAMS}
    assert by_name[name].addr == addr
    assert by_name[name].group == group


def test_options_serialize_with_string_keys():
    by_name = {p.name: p for p in servo_tuning.PANEL_PARAMS}
    d = by_name["speed_feedback_filter"].as_dict()
    assert d["options"] == {
        "0": "internal setting",
        "1": "low-pass filter",
        "2": "overlapping average",
        "3": "speed observer",
        "4": "no filter",
    }
    assert by_name["speed_gain"].as_dict()["options"] is None


def test_validate_param_map_rejects_duplicate_name():
    params = [
        servo_tuning.PanelParam(
            name="dup", c_code="C01.00", unit="", group="g", description=""
        ),
        servo_tuning.PanelParam(
            name="dup", c_code="C01.01", unit="", group="g", description=""
        ),
    ]
    with pytest.raises(ValueError, match="duplicate param name"):
        servo_tuning.validate_param_map(params)


def test_validate_param_map_rejects_duplicate_address():
    params = [
        servo_tuning.PanelParam(
            name="a", c_code="C01.00", unit="", group="g", description=""
        ),
        servo_tuning.PanelParam(
            name="b", c_code="C01.00", unit="", group="g", description=""
        ),
    ]
    with pytest.raises(ValueError, match="both target"):
        servo_tuning.validate_param_map(params)


def test_validate_param_map_rejects_bad_type():
    params = [
        servo_tuning.PanelParam(
            name="a",
            c_code="C01.00",
            unit="",
            group="g",
            description="",
            type_token="q16",
        )
    ]
    with pytest.raises(ValueError, match="unknown type"):
        servo_tuning.validate_param_map(params)


class FakeCalibrationSection:
    def __init__(self, captures_root):
        self._captures_root = captures_root

    def get(self, key, default=None):
        if key == "captures_root":
            return self._captures_root
        return default


class FakeTuningConfig:
    def __init__(self, printer, values=None, sections=None):
        self._printer = printer
        self._values = values or {}
        self._sections = sections or {}

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._values.get(key, default)

    def has_section(self, name):
        return name in self._sections

    def getsection(self, name):
        return self._sections[name]

    def error(self, msg):
        return FakeConfigError(msg)


class FakeMotor:
    def __init__(self, motor_name, node_name, sdo_params=(), chain_index=0):
        self._motor_name = motor_name
        self._node_name = node_name
        self._sdo_params = list(sdo_params)
        self._chain_index = chain_index

    def get_motor_name(self):
        return self._motor_name

    def get_node_name(self):
        return self._node_name

    def get_sdo_params(self):
        return self._sdo_params

    def get_chain_index(self):
        return self._chain_index

    def get_invert_direction(self):
        return False


def _fake_rail(motors, axis="x"):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def _c_code_key(c_code):
    index, subindex = servo_param.parse_address(
        servo_tuning.c_code_to_addr(c_code)
    )
    return index, subindex


def _full_readback(overrides=None):
    values = {
        _c_code_key("C01.00"): (2, 700),
        _c_code_key("C01.01"): (2, 550),
        _c_code_key("C01.02"): (2, 2273),
        _c_code_key("C01.03"): (2, 220),
        _c_code_key("C01.18"): (2, 318),
        _c_code_key("C01.15"): (2, 318),
        _c_code_key("C01.30"): (2, 2),
        _c_code_key("C01.10"): (2, 3),
        _c_code_key("C02.30"): (2, 8000),
        _c_code_key("C02.31"): (2, 1000),
        _c_code_key("C02.32"): (2, 0),
        _c_code_key("C02.60"): (2, 2000),
        _c_code_key("C02.61"): (2, 1000),
        _c_code_key("C02.62"): (2, 30),
        _c_code_key("C02.63"): (2, 150),
        _c_code_key("C00.04"): (2, 0),
        _c_code_key("C00.05"): (2, 12),
        _c_code_key("C00.06"): (2, 150),
        **{
            _c_code_key("C01.%02X" % (0x40 + i)): (2, v)
            for i, v in enumerate(
                [345, 160, 200, 225, 200, 120, 140, 100, 350]
                + [8000, 0, 1000] * 2
            )
        },
    }
    if overrides:
        values.update(overrides)
    return values


class FakeWriteEngine(_FakeEngine):
    def __init__(self, mismatch_addr=None, fail_addrs=None):
        super().__init__()
        self.writes = []
        self.mismatch_addr = mismatch_addr
        self.fail_addrs = fail_addrs or set()

    def sdo_write(self, handle, slot, index, subindex, size, value):
        if (index, subindex) in self.fail_addrs:
            raise RuntimeError("simulated SDO write failure")
        self.writes.append((handle, slot, index, subindex, size, value))
        if self.mismatch_addr == (index, subindex):
            return size, value + 1
        return size, value


def _make_full_servo_tuning(
    printer_objs, config_values=None, config_sections=None
):
    printer = FakePrinter(printer_objs)
    config = FakeTuningConfig(
        printer, values=config_values, sections=config_sections
    )
    return servo_tuning.ServoTuning(config)


def _two_motor_printer(engine, node=None, sdo_params_by_motor=None):
    sdo_params_by_motor = sdo_params_by_motor or {}
    motor_a = FakeMotor(
        "motor_a", "node_x", sdo_params_by_motor.get("motor_a", ())
    )
    motor_b = FakeMotor(
        "motor_b", "node_x", sdo_params_by_motor.get("motor_b", ())
    )
    rail = _fake_rail([motor_a, motor_b])
    node = node if node is not None else FakeNode(7)
    objs = {
        "toolhead": FakeToolhead(FakeKin([rail])),
        "ethercat_node node_x": node,
        "motion_engine": engine,
        "gcode": FakeGcode(),
    }
    return objs, motor_a, motor_b


def test_dump_tuning_writes_expected_json(tmp_path):
    engine = FakeReadEngine(_full_readback())
    objs, motor_a, motor_b = _two_motor_printer(
        engine,
        node=FakeNode(7, slots={"motor_a": 0, "motor_b": 1}),
        sdo_params_by_motor={
            "motor_a": [(0x2000, 7, 2, 870)],
            "motor_b": [(0x2001, 1, 2, 700)],
        },
    )
    st = _make_full_servo_tuning(
        objs,
        config_sections={
            "servo_calibration": FakeCalibrationSection(str(tmp_path))
        },
    )
    gcmd = FakeGcmd()
    st.cmd_SERVO_DUMP_TUNING(gcmd)
    path = tmp_path / "drive_state.json"
    payload = json.loads(path.read_text())
    assert payload["version"] == 1
    assert "created_utc" in payload
    assert len(payload["params"]) == len(servo_tuning.PANEL_PARAMS)
    param_names = {p["name"] for p in payload["params"]}
    assert "position_gain" in param_names
    assert "inertia_ratio" not in param_names
    assert payload["motors"]["motor_a"]["C01.01"] == 550
    assert "C00.06" not in payload["motors"]["motor_b"]
    assert payload["config_pins"]["motor_a"] == {}
    assert payload["config_pins"]["motor_b"] == {"C01.00": 700}
    assert payload["slots"] == {"motor_a": 0, "motor_b": 1}
    assert payload["spatial"] == {
        "modes": ["x"],
        "axes": ["motor_a", "motor_b"],
        "frame": [[0.5, 0.5]],
    }
    assert any(str(path) in r for r in gcmd.responses)
    assert any("2" in r and "motors" in r for r in gcmd.responses)


def test_dump_tuning_default_motors_is_all(tmp_path):
    engine = FakeReadEngine(_full_readback())
    objs, _a, _b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(
        objs,
        config_sections={
            "servo_calibration": FakeCalibrationSection(str(tmp_path))
        },
    )
    gcmd = FakeGcmd()
    st.cmd_SERVO_DUMP_TUNING(gcmd)
    payload = json.loads((tmp_path / "drive_state.json").read_text())
    assert set(payload["motors"]) == {"motor_a", "motor_b"}


def test_dump_tuning_motors_filter(tmp_path):
    engine = FakeReadEngine(_full_readback())
    objs, _a, _b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(
        objs,
        config_sections={
            "servo_calibration": FakeCalibrationSection(str(tmp_path))
        },
    )
    gcmd = FakeGcmd(MOTORS="motor_a")
    st.cmd_SERVO_DUMP_TUNING(gcmd)
    payload = json.loads((tmp_path / "drive_state.json").read_text())
    assert set(payload["motors"]) == {"motor_a"}
    assert set(payload["slots"]) == {"motor_a"}


def test_dump_tuning_readback_failure_names_motor_and_param(tmp_path):
    engine = FakeReadEngine(
        _full_readback(), fail_addrs={_c_code_key("C01.01")}
    )
    objs, _a, _b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(
        objs,
        config_sections={
            "servo_calibration": FakeCalibrationSection(str(tmp_path))
        },
    )
    gcmd = FakeGcmd(MOTORS="motor_a")
    with pytest.raises(RuntimeError, match="motor_a"):
        st.cmd_SERVO_DUMP_TUNING(gcmd)


def test_dump_tuning_default_captures_root(monkeypatch, tmp_path):
    engine = FakeReadEngine(_full_readback())
    objs, _a, _b = _two_motor_printer(engine)
    monkeypatch.setattr(
        servo_tuning.servo_calibration,
        "DEFAULT_CAPTURES_ROOT",
        str(tmp_path) + "/nested",
    )
    st = _make_full_servo_tuning(objs)
    assert st.captures_root == str(tmp_path) + "/nested"


def test_tune_by_name_writes_and_records(tmp_path):
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(
        objs,
        config_sections={
            "servo_calibration": FakeCalibrationSection(str(tmp_path))
        },
    )
    gcmd = FakeGcmd(PARAM="speed_gain", VALUE=600, MOTORS="all")
    st.cmd_SERVO_TUNE(gcmd)
    assert len(engine.writes) == 2
    _handle, _slot, index, subindex, size, value = engine.writes[0]
    assert (index, subindex) == (0x2001, 2)
    assert size == 2
    assert value == 600
    writes = servo_param.drain_param_writes()
    assert {w["servo"] for w in writes} == {"motor_a", "motor_b"}
    assert all(w["value"] == 600 for w in writes)
    assert all(w["addr"] == "0x2001.0x02" for w in writes)
    assert any("speed_gain" in r and "600" in r for r in gcmd.responses)


def _tmp_calibration_sections(tmp_path):
    return {"servo_calibration": FakeCalibrationSection(str(tmp_path))}


def test_tune_patches_drive_state_in_place(tmp_path):
    objs, _a, _b = _two_motor_printer(
        FakeReadEngine(_full_readback()),
        node=FakeNode(7, slots={"motor_a": 0, "motor_b": 1}),
        sdo_params_by_motor={"motor_b": [(0x2001, 2, 2, 550)]},
    )
    st = _make_full_servo_tuning(
        objs, config_sections=_tmp_calibration_sections(tmp_path)
    )
    st.cmd_SERVO_DUMP_TUNING(FakeGcmd())
    objs["motion_engine"] = FakeWriteEngine()
    st.cmd_SERVO_TUNE(FakeGcmd(PARAM="speed_gain", VALUE=640, MOTORS="motor_a"))
    payload = json.loads((tmp_path / "drive_state.json").read_text())
    assert payload["motors"]["motor_a"]["C01.01"] == 640
    assert payload["motors"]["motor_b"]["C01.01"] == 550
    assert payload["config_pins"]["motor_b"]["C01.01"] == 550
    st.cmd_SERVO_TUNE(FakeGcmd(PARAM="speed_gain", VALUE=700, MOTORS="motor_b"))
    payload = json.loads((tmp_path / "drive_state.json").read_text())
    assert payload["motors"]["motor_b"]["C01.01"] == 700
    assert payload["config_pins"]["motor_b"]["C01.01"] == 700
    servo_param.drain_param_writes()


def test_tune_without_drive_state_writes_no_file(tmp_path):
    objs, _a, _b = _two_motor_printer(FakeWriteEngine())
    st = _make_full_servo_tuning(
        objs, config_sections=_tmp_calibration_sections(tmp_path)
    )
    st.cmd_SERVO_TUNE(FakeGcmd(PARAM="speed_gain", VALUE=600, MOTORS="motor_a"))
    assert not (tmp_path / "drive_state.json").exists()
    servo_param.drain_param_writes()


def test_tune_unmapped_addr_leaves_drive_state_untouched(tmp_path):
    objs, _a, _b = _two_motor_printer(
        FakeReadEngine(_full_readback()),
        node=FakeNode(7, slots={"motor_a": 0, "motor_b": 1}),
    )
    st = _make_full_servo_tuning(
        objs, config_sections=_tmp_calibration_sections(tmp_path)
    )
    st.cmd_SERVO_DUMP_TUNING(FakeGcmd())
    before = (tmp_path / "drive_state.json").read_text()
    objs["motion_engine"] = FakeWriteEngine()
    st.cmd_SERVO_TUNE(FakeGcmd(PARAM="0x2005.0x03", VALUE=42, MOTORS="motor_a"))
    assert (tmp_path / "drive_state.json").read_text() == before
    servo_param.drain_param_writes()


def test_tune_corrupt_drive_state_is_command_error(tmp_path):
    (tmp_path / "drive_state.json").write_text("{not json")
    objs, _a, _b = _two_motor_printer(FakeWriteEngine())
    st = _make_full_servo_tuning(
        objs, config_sections=_tmp_calibration_sections(tmp_path)
    )
    gcmd = FakeGcmd(PARAM="speed_gain", VALUE=600, MOTORS="motor_a")
    with pytest.raises(RuntimeError, match="SERVO_DUMP_TUNING to rebuild"):
        st.cmd_SERVO_TUNE(gcmd)
    servo_param.drain_param_writes()


def test_tune_by_c_code(tmp_path):
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="C00.06", VALUE=870, MOTORS="motor_a")
    st.cmd_SERVO_TUNE(gcmd)
    assert len(engine.writes) == 1
    _handle, _slot, index, subindex, _size, value = engine.writes[0]
    assert (index, subindex) == (0x2000, 7)
    assert value == 870


def test_tune_by_raw_addr_unmapped_defaults_u16(tmp_path):
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="0x2005.0x03", VALUE=42, MOTORS="motor_a")
    st.cmd_SERVO_TUNE(gcmd)
    _handle, _slot, index, subindex, size, value = engine.writes[0]
    assert (index, subindex) == (0x2005, 3)
    assert size == 2
    assert value == 42


def test_tune_by_raw_addr_with_type_override(tmp_path):
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="0x2005.0x03", VALUE=42, MOTORS="motor_a", TYPE="u8")
    st.cmd_SERVO_TUNE(gcmd)
    _handle, _slot, _index, _subindex, size, _value = engine.writes[0]
    assert size == 1


def test_tune_unresolvable_param_is_error():
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="not_a_param", VALUE=1, MOTORS="motor_a")
    with pytest.raises(RuntimeError, match="not a mapped name"):
        st.cmd_SERVO_TUNE(gcmd)


def test_tune_readback_mismatch_is_error():
    engine = FakeWriteEngine(mismatch_addr=(0x2001, 2))
    objs, motor_a, motor_b = _two_motor_printer(engine)
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="speed_gain", VALUE=600, MOTORS="motor_a")
    with pytest.raises(RuntimeError, match="readback mismatch"):
        st.cmd_SERVO_TUNE(gcmd)
    writes = servo_param.drain_param_writes()
    assert len(writes) == 1


def test_tune_no_engine_handle_is_error():
    engine = FakeWriteEngine()
    objs, motor_a, motor_b = _two_motor_printer(engine, node=FakeNode(None))
    st = _make_full_servo_tuning(objs)
    gcmd = FakeGcmd(PARAM="speed_gain", VALUE=600, MOTORS="motor_a")
    with pytest.raises(RuntimeError, match="no engine handle"):
        st.cmd_SERVO_TUNE(gcmd)


def test_extra_params_parsed_and_appended(tmp_path):
    printer = FakePrinter(
        {"gcode": FakeGcode(), "toolhead": FakeToolhead(FakeKin([]))}
    )
    config = FakeTuningConfig(
        printer,
        values={"extra_params": "notch_freq2 C01.31 u16 Hz notch\n"},
    )
    st = servo_tuning.ServoTuning(config)
    extra = st._by_name["notch_freq2"]
    assert extra.addr == "0x2001.0x32"
    assert extra.unit == "Hz"
    assert extra.group == "notch"
    assert len(st.params) == len(servo_tuning.PANEL_PARAMS) + 1


def test_extra_params_bad_line_is_config_error():
    printer = FakePrinter(
        {"gcode": FakeGcode(), "toolhead": FakeToolhead(FakeKin([]))}
    )
    config = FakeTuningConfig(
        printer, values={"extra_params": "bad_line_too_few_fields\n"}
    )
    with pytest.raises(FakeConfigError, match="expected 'name"):
        servo_tuning.ServoTuning(config)


def test_extra_params_bad_type_is_config_error():
    printer = FakePrinter(
        {"gcode": FakeGcode(), "toolhead": FakeToolhead(FakeKin([]))}
    )
    config = FakeTuningConfig(
        printer,
        values={"extra_params": "bad C01.31 q16 Hz notch\n"},
    )
    with pytest.raises(FakeConfigError, match="unknown type"):
        servo_tuning.ServoTuning(config)


def test_extra_params_duplicate_name_is_config_error():
    printer = FakePrinter(
        {"gcode": FakeGcode(), "toolhead": FakeToolhead(FakeKin([]))}
    )
    config = FakeTuningConfig(
        printer,
        values={"extra_params": "speed_gain C01.31 u16 Hz notch\n"},
    )
    with pytest.raises(FakeConfigError, match="duplicate param name"):
        servo_tuning.ServoTuning(config)
