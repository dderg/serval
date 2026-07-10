import pytest

from klippy.extras import servo_axis, servo_param, servo_tuning


class FakeConfigError(Exception):
    pass


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


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeReadEngine:
    def __init__(self, values):
        self.values = values

    def sdo_read(self, handle, slot, index, subindex):
        return self.values[(index, subindex)]


class FakeNode:
    name = "node_x"

    def __init__(self, handle=7):
        self._h = handle

    def get_engine_handle(self):
        return self._h

    def get_slot_for_motor(self, motor_name):
        return 0


class FakeKin:
    def __init__(self, rails):
        self.rails = rails


class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin

    def get_kinematics(self):
        return self.kin


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


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
