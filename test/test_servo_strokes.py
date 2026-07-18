import pytest

from klippy.extras import servo_axis, servo_strokes


class FakeGcode:
    error = RuntimeError

    def __init__(self):
        self.scripts = []

    def run_script_from_command(self, script):
        self.scripts.append(script)


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params

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


class FakeKin:
    def __init__(self, rails, coupled=True):
        self.rails = rails
        self._coupled = coupled

    def coupled_xy(self):
        return self._coupled


def _motor(name, node_name="node", chain_index=0):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.chain_index = chain_index
    m.invert_direction = False
    return m


def _rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


BOUNDS = {"X": (30.0, 220.0), "Y": (30.0, 220.0)}


def cartesian_kin():
    return FakeKin(
        [
            _rail("x", [_motor("motor_x")]),
            _rail("y", [_motor("motor_y")]),
        ],
        coupled=False,
    )


def corexy_kin():
    return FakeKin(
        [
            _rail("x", [_motor("motor_a")]),
            _rail("y", [_motor("motor_b")]),
        ],
        coupled=True,
    )


def test_check_reachable_passes_when_speed_fits_the_stroke():
    gcode = FakeGcode()
    servo_strokes.check_reachable(gcode, length=100.0, speed=100.0, accel=200.0)


def test_check_reachable_rejects_speed_too_high_for_stroke():
    gcode = FakeGcode()
    with pytest.raises(RuntimeError, match="too short to reach"):
        servo_strokes.check_reachable(
            gcode, length=1.0, speed=100.0, accel=100.0
        )


def test_emit_strokes_rejects_end_not_exceeding_start():
    gcode = FakeGcode()
    with pytest.raises(RuntimeError, match="must exceed START"):
        servo_strokes.emit_strokes(
            gcode,
            lambda u: "X%.3f" % (u,),
            100.0,
            100.0,
            1.0,
            100.0,
            3000.0,
            1,
            0,
        )


def test_emit_strokes_builds_iteration_pairs_and_feed():
    gcode = FakeGcode()
    servo_strokes.emit_strokes(
        gcode, lambda u: "X%.3f" % (u,), 0.0, 10.0, 1.0, 100.0, 3000.0, 2, 250
    )
    (script,) = gcode.scripts
    lines = script.splitlines()
    assert lines[0] == "SET_VELOCITY_LIMIT ACCEL=3000"
    assert lines[1] == "G90"
    g1_lines = [ln for ln in lines if ln.startswith("G1")]
    assert g1_lines == [
        "G1 X10.000 F6000",
        "G1 X0.000 F6000",
        "G1 X10.000 F6000",
        "G1 X0.000 F6000",
    ]
    assert lines.count("G4 P250") == 4


def test_axis_bounds_default_and_override():
    gcmd = FakeGcmd()
    assert servo_strokes.axis_bounds(gcmd, BOUNDS, "X") == (30.0, 220.0)
    gcmd = FakeGcmd(START=10.0, END=50.0)
    assert servo_strokes.axis_bounds(gcmd, BOUNDS, "X") == (10.0, 50.0)


def test_axis_bounds_requires_configured_bounds():
    with pytest.raises(RuntimeError, match="START/END required"):
        servo_strokes.axis_bounds(FakeGcmd(), {}, "Z")


def test_xy_bounds_default_and_override():
    gcmd = FakeGcmd(X_START=1.0)
    assert servo_strokes.xy_bounds(gcmd, BOUNDS) == (1.0, 220.0, 30.0, 220.0)


def test_grid_defaults_and_overrides():
    gcmd = FakeGcmd()
    accels, speeds, iterations, dwell = servo_strokes.grid(
        gcmd, [5000.0, 10000.0], [100.0, 400.0], 3, 700
    )
    assert accels == [5000.0, 10000.0]
    assert speeds == [100.0, 400.0]
    assert iterations == 3
    assert dwell == 700

    gcmd = FakeGcmd(ACCELS="1000,2000", SPEEDS="50", ITERATIONS=1, DWELL_MS=0)
    accels, speeds, iterations, dwell = servo_strokes.grid(
        gcmd, [5000.0], [100.0], 3, 700
    )
    assert accels == [1000.0, 2000.0]
    assert speeds == [50.0]
    assert iterations == 1
    assert dwell == 0


def test_axis_rails_expands_to_both_lanes_on_corexy():
    kin = corexy_kin()
    rails = servo_strokes.axis_rails(FakeGcmd(), kin, "X")
    assert [r.axis for r in rails] == ["x", "y"]


def test_axis_rails_stays_single_lane_off_corexy():
    kin = cartesian_kin()
    rails = servo_strokes.axis_rails(FakeGcmd(), kin, "X")
    assert [r.axis for r in rails] == ["x"]


def test_axis_rails_rejects_bad_axis_name():
    with pytest.raises(RuntimeError, match="AXIS must be"):
        servo_strokes.axis_rails(FakeGcmd(), cartesian_kin(), "A")


def test_diagonal_rail_requires_corexy():
    with pytest.raises(RuntimeError, match="not coupled_xy"):
        servo_strokes.diagonal_rail(FakeGcmd(), cartesian_kin(), "A")


def test_build_plan_axis_case_uses_bounds_and_unit_th_per_unit():
    plan = servo_strokes.build_plan(FakeGcmd(), cartesian_kin(), BOUNDS, "X")
    assert (plan.start, plan.end) == (30.0, 220.0)
    assert plan.th_per_unit == 1.0
    assert plan.diagonal is False
    assert plan.servos == ["motor_x"]
    assert plan.prep == ("X",)
    assert plan.coord(12.5) == "X12.500"


def test_build_plan_diagonal_a_is_plus_45_centered_on_bounds():
    plan = servo_strokes.build_plan(FakeGcmd(), corexy_kin(), BOUNDS, "A")
    assert plan.diagonal is True
    assert plan.th_per_unit == pytest.approx(2.0**0.5)
    assert plan.prep == ("X", "Y")
    assert plan.servos == ["motor_a"]
    # center (125, 125), half 95 -> start=-95, end=95
    assert (plan.start, plan.end) == (-95.0, 95.0)
    assert plan.coord(10.0) == "X135.000 Y135.000"


def test_build_plan_diagonal_b_moves_x_up_y_down():
    plan = servo_strokes.build_plan(FakeGcmd(), corexy_kin(), BOUNDS, "B")
    assert plan.servos == ["motor_b"]
    assert plan.coord(10.0) == "X135.000 Y115.000"


def test_build_plan_diagonal_start_end_override():
    plan = servo_strokes.build_plan(
        FakeGcmd(START=-10.0, END=10.0), corexy_kin(), BOUNDS, "A"
    )
    assert (plan.start, plan.end) == (-10.0, 10.0)


def test_rail_motors_in_slot_order_sorts_by_chain_index():
    rail = _rail(
        "x", [_motor("m1", chain_index=1), _motor("m0", chain_index=0)]
    )
    ordered = servo_strokes.rail_motors_in_slot_order(rail)
    assert [m.get_motor_name() for m in ordered] == ["m0", "m1"]


def test_corexy_fit_layout_single_drive_per_belt():
    layout = servo_strokes.corexy_fit_layout(FakeGcmd(), corexy_kin())
    assert layout == {"servos": ["motor_a", "motor_b"], "pairs": None}


def test_scalar_fit_drive_returns_none_for_single_drive_axis():
    assert servo_strokes.scalar_fit_drive(FakeGcmd(), cartesian_kin()) is None


def test_spatial_frame_corexy_folds_invert_and_halves_belts():
    inverted_b = _motor("motor_b")
    inverted_b.invert_direction = True
    kin = FakeKin(
        [
            _rail("x", [_motor("motor_a")]),
            _rail("y", [inverted_b]),
        ],
        coupled=True,
    )
    assert servo_strokes.spatial_frame(kin) == {
        "modes": ["x", "y"],
        "axes": ["motor_a", "motor_b"],
        "frame": [[0.5, -0.5], [0.5, 0.5]],
    }


def test_spatial_frame_corexy_awd_scales_by_drives_per_belt():
    kin = FakeKin(
        [
            _rail(
                "x",
                [
                    _motor("motor_a", chain_index=0),
                    _motor("motor_a1", chain_index=1),
                ],
            ),
            _rail("y", [_motor("motor_b")]),
        ],
        coupled=True,
    )
    assert servo_strokes.spatial_frame(kin) == {
        "modes": ["x", "y"],
        "axes": ["motor_a", "motor_a1", "motor_b"],
        "frame": [[0.25, 0.25, 0.5], [0.25, 0.25, -0.5]],
    }


def test_spatial_frame_cartesian_maps_each_rail_to_its_mode():
    assert servo_strokes.spatial_frame(cartesian_kin()) == {
        "modes": ["x", "y"],
        "axes": ["motor_x", "motor_y"],
        "frame": [[1.0, 0.0], [0.0, 1.0]],
    }


def test_spatial_frame_cartesian_skips_non_servo_lanes():
    kin = FakeKin(
        [object(), _rail("y", [_motor("motor_y")])],
        coupled=False,
    )
    assert servo_strokes.spatial_frame(kin) == {
        "modes": ["y"],
        "axes": ["motor_y"],
        "frame": [[1.0]],
    }


def test_spatial_frame_none_without_servo_xy_rails():
    assert servo_strokes.spatial_frame(FakeKin([], coupled=False)) is None
    assert (
        servo_strokes.spatial_frame(FakeKin([object(), object()], coupled=True))
        is None
    )
