import pytest

from klippy.extras import servo_axis, servo_calibration


class FakeGcode:
    error = RuntimeError

    def __init__(self):
        self.commands = {}
        self.scripts = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run_script_from_command(self, script):
        self.scripts.append(script)


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self._params.get(name, default))

    def get_float(
        self,
        name,
        default=None,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        value = self._params.get(name, default)
        return None if value is None else float(value)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeKin:
    def __init__(self, rails, coupled=True):
        self.rails = rails
        self._coupled = coupled

    def coupled_xy(self):
        return self._coupled


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


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def getlist(self, name, default=None):
        return default

    def getfloat(self, name, default=None, **kw):
        return default

    def getfloatlist(self, name, default=None):
        return default

    def getint(self, name, default=None, **kw):
        return default


def _motor(name, node_name, chain_index, invert=False):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.chain_index = chain_index
    m.invert_direction = invert
    return m


def _rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def make_calibration(rails, coupled=True):
    gcode = FakeGcode()
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled)),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc._prep = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None
    sc._restore = lambda *a, **k: None
    sc._run = lambda *a, **k: gcode.scripts.append(("RUN",) + tuple(a[1:]))
    return sc, gcode


def awd_rails(node="xy_drives", node_b=None):
    return [
        _rail(
            "x",
            [
                _motor("motor_a1", node, 1),
                _motor("motor_a", node, 0),
            ],
        ),
        _rail(
            "y",
            [
                _motor("motor_b", node_b or node, 2, invert=True),
                _motor("motor_b1", node_b or node, 3),
            ],
        ),
    ]


def single_drive_rails():
    return [
        _rail("x", [_motor("motor_a", "xy_drives", 0)]),
        _rail("y", [_motor("motor_b", "xy_drives", 1)]),
    ]


def test_layout_single_drive_pairs_is_none():
    sc, _ = make_calibration(single_drive_rails())
    layout = sc._corexy_fit_layout(FakeGcmd())
    assert layout == {"servos": ["motor_a", "motor_b"], "pairs": None}


def test_layout_awd_orders_pairs_by_chain_index():
    sc, _ = make_calibration(awd_rails())
    layout = sc._corexy_fit_layout(FakeGcmd())
    assert layout["servos"] == [
        "motor_a",
        "motor_a1",
        "motor_b",
        "motor_b1",
    ]
    assert layout["pairs"] == "motor_a,motor_a1;motor_b,motor_b1"


def test_layout_awd_rejects_split_nodes():
    sc, _ = make_calibration(awd_rails(node_b="other_node"))
    with pytest.raises(RuntimeError, match="one ethercat node"):
        sc._corexy_fit_layout(FakeGcmd())


def test_layout_rejects_mixed_drive_counts():
    rails = [
        _rail(
            "x",
            [
                _motor("motor_a", "xy_drives", 0),
                _motor("motor_a1", "xy_drives", 1),
            ],
        ),
        _rail("y", [_motor("motor_b", "xy_drives", 2)]),
    ]
    sc, _ = make_calibration(rails)
    with pytest.raises(RuntimeError, match="one or two drives per belt"):
        sc._corexy_fit_layout(FakeGcmd())


def test_layout_requires_coupled_xy():
    sc, _ = make_calibration(single_drive_rails(), coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy"):
        sc._corexy_fit_layout(FakeGcmd())


def test_servos_override_must_match_kinematics():
    sc, _ = make_calibration(awd_rails())
    layout = sc._corexy_fit_layout(FakeGcmd())
    with pytest.raises(RuntimeError, match="does not match"):
        sc._check_servos_override(FakeGcmd(SERVOS="motor_a,motor_b"), layout)
    sc._check_servos_override(
        FakeGcmd(SERVOS="motor_b1, motor_a, motor_b, motor_a1"), layout
    )


def test_fit_dynamics_corexy_captures_all_four_and_passes_pairs():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS_COREXY(FakeGcmd())
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    assert "SERVO=motor_a,motor_a1,motor_b,motor_b1" in starts[0]
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    _tag, script, args, _timeout = runs[-1]
    assert script == "servo_fit_dynamics.py"
    assert args[args.index("--pairs") + 1] == (
        "motor_a,motor_a1;motor_b,motor_b1"
    )


def test_fit_dynamics_corexy_two_drives_has_no_pairs():
    sc, gcode = make_calibration(single_drive_rails())
    sc.cmd_SERVO_FIT_DYNAMICS_COREXY(FakeGcmd())
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    assert "--pairs" not in runs[-1][2]


def test_scalar_fit_requires_drive_on_multi_drive_axis():
    sc, _ = make_calibration(awd_rails())
    with pytest.raises(RuntimeError, match="pass DRIVE="):
        sc._scalar_fit_drive(FakeGcmd(AXIS="X"))
    assert sc._scalar_fit_drive(FakeGcmd(AXIS="X", DRIVE="motor_a1")) == (
        "motor_a1"
    )
    with pytest.raises(RuntimeError, match="not among"):
        sc._scalar_fit_drive(FakeGcmd(AXIS="X", DRIVE="motor_z"))


def test_fit_dynamics_passes_drive_to_script():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(AXIS="X", DRIVE="motor_a"))
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    args = runs[-1][2]
    assert args[args.index("--drive") + 1] == "motor_a"


def test_tracking_combined_view_lists_every_motor_per_belt():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    assert "SERVO=motor_a1,motor_a,motor_b,motor_b1" in starts[0]
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    args = runs[-1][2]
    assert args[args.index("--combine-corexy") + 1] == (
        "motor_a:1+motor_a1:1,motor_b:-1+motor_b1:1"
    )


def test_tracking_single_drive_belts_keep_plain_terms():
    sc, gcode = make_calibration(single_drive_rails())
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    args = runs[-1][2]
    assert args[args.index("--combine-corexy") + 1] == "motor_a:1,motor_b:1"


def _capture_starts(gcode):
    return [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]


def test_measure_inertia_captures_every_motor_moving_the_axis():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X"))
    assert (
        "SERVO=motor_a1,motor_a,motor_b,motor_b1" in _capture_starts(gcode)[0]
    )


def test_measure_friction_captures_every_motor_moving_the_axis():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_FRICTION(FakeGcmd(AXIS="X"))
    assert (
        "SERVO=motor_a1,motor_a,motor_b,motor_b1" in _capture_starts(gcode)[0]
    )


def test_measure_inertia_cartesian_captures_only_its_rail():
    rails = [
        _rail(
            "x",
            [_motor("motor_x", "node", 0), _motor("motor_x1", "node", 1)],
        ),
        _rail("y", [_motor("motor_y", "node", 2)]),
    ]
    sc, gcode = make_calibration(rails, coupled=False)
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X"))
    assert "SERVO=motor_x,motor_x1" in _capture_starts(gcode)[0]


def test_measure_inertia_corexy_defaults_to_kinematics_servos():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_INERTIA_COREXY(FakeGcmd())
    assert (
        "SERVO=motor_a1,motor_a,motor_b,motor_b1" in _capture_starts(gcode)[0]
    )


def test_tracking_single_rail_dual_motor_axis_gets_no_combine():
    rails = [
        _rail(
            "x",
            [_motor("motor_x", "x_node", 0), _motor("motor_x1", "x_node", 1)],
        ),
    ]
    sc, gcode = make_calibration(rails, coupled=False)
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    assert "--combine-corexy" not in runs[-1][2]
