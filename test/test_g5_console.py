import types


class FakeKin:
    def __init__(self):
        self.checked = []

    def parked_dirty_axes(self):
        return []

    def check_move(self, move):
        self.checked.append(tuple(move.end_pos))
        # simulate a bed of +/-100 in X/Y, +/-50 in Z
        ep = move.end_pos
        if not (
            -100 <= ep[0] <= 100 and -100 <= ep[1] <= 100 and -50 <= ep[2] <= 50
        ):
            raise RuntimeError("out of range")


def make_motion():
    import klippy.motion as motion

    m = motion.Motion.__new__(motion.Motion)
    m.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    m._max_velocity = 300.0
    m._max_accel = 3000.0
    m._square_corner_velocity = 5.0
    m._planner_ready = False
    m.kin = FakeKin()
    m.extruder = types.SimpleNamespace(check_move=lambda mv: None)
    m.engine = types.SimpleNamespace(
        calls=[],
        get_last_move_time=lambda: 0.0,
        submit_bezier=lambda *a: m.engine.calls.append(("bezier", a)),
    )
    m.mcu = None
    m._mcu_pending_end_time = 0.0
    m._fire_active_callbacks = lambda axes_d: None
    m._sync_print_time = lambda: None
    m._axis_limit = lambda axis, kind: 100.0
    return m


def test_move_curve_rejects_out_of_range_control_point():
    m = make_motion()

    # endpoints in range, but P1 control point at Y=500 bulges off the bed
    def submit(dx, dy, dz, de, fr):
        m.engine.submit_bezier(dx, dy, dz, de, fr)
        return True

    interior = [[10.0, 500.0, 0.0], [10.0, 0.0, 0.0]]
    try:
        m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
        assert False, "expected out-of-range rejection"
    except RuntimeError as e:
        assert "out of range" in str(e)


def test_move_curve_submits_and_advances_when_in_range():
    m = make_motion()

    def submit(dx, dy, dz, de, fr):
        m.engine.submit_bezier(dx, dy, dz, de, fr)
        return True

    interior = [[10.0, 5.0, 0.0], [10.0, -5.0, 0.0]]
    m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
    assert m.engine.calls and m.engine.calls[0][0] == "bezier"
    assert m.commanded_pos[0] == 20.0


def make_gcode_move():
    import klippy.extras.gcode_move as gm

    g = gm.GCodeMove.__new__(gm.GCodeMove)
    g.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: g._toolhead
    )
    g._toolhead = types.SimpleNamespace(
        get_position=lambda: [0.0, 0.0, 0.0, 0.0]
    )
    g.position_with_transform = lambda: [0.0, 0.0, 0.0, 0.0]
    return g


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


def test_transform_gate_passes_when_identity():
    g = make_gcode_move()
    # identity: transformed == raw -> no raise
    g._reject_curve_if_transform_active(FakeGcmd())


def test_transform_gate_rejects_when_active():
    g = make_gcode_move()
    g.position_with_transform = lambda: [0.0, 2.0, 0.0, 0.0]  # bent in Y
    try:
        g._reject_curve_if_transform_active(FakeGcmd())
        assert False, "expected rejection"
    except RuntimeError as e:
        assert "active move transform" in str(e)


class ParamGcmd:
    def __init__(self, params):
        self._p = params

    def get_command_parameters(self):
        return self._p

    def get_commandline(self):
        return "G5 " + " ".join("%s%s" % kv for kv in self._p.items())

    def error(self, msg):
        return RuntimeError(msg)


def make_full_gcode_move():
    import klippy.extras.gcode_move as gm

    g = gm.GCodeMove.__new__(gm.GCodeMove)
    g.absolute_coord = True
    g.absolute_extrude = True
    g.base_position = [0.0, 0.0, 0.0, 0.0]
    g.last_position = [0.0, 0.0, 0.0, 0.0]
    g.extrude_factor = 1.0
    g.speed = 50.0
    g.speed_factor = 1.0 / 60.0
    g.curve_calls = []
    g._toolhead = types.SimpleNamespace(
        get_position=lambda: [0.0, 0.0, 0.0, 0.0],
        move_curve=lambda *a, **k: g.curve_calls.append((a, k)),
        resync_parked_servos=lambda: None,
    )
    g.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: g._toolhead
    )
    g.position_with_transform = lambda: [0.0, 0.0, 0.0, 0.0]
    return g


def test_cmd_g5_requires_p_and_q():
    g = make_full_gcode_move()
    try:
        g.cmd_G5(ParamGcmd({"X": "10", "Y": "0", "I": "2", "J": "2"}))
        assert False
    except RuntimeError as e:
        assert "P and Q" in str(e)


def test_cmd_g5_calls_move_curve_with_interior_points():
    g = make_full_gcode_move()
    g.cmd_G5(
        ParamGcmd(
            {"X": "10", "Y": "0", "I": "2", "J": "4", "P": "-3", "Q": "4"}
        )
    )
    assert g.curve_calls, "move_curve should be invoked"
    (args, _kwargs) = g.curve_calls[0]
    newpos, interior, _submit, _speed = args
    assert newpos[0] == 10.0 and newpos[1] == 0.0
    # P1 = start+(I,J) = (2,4); P2 = end+(P,Q) = (7,4)
    assert interior[0][:2] == [2.0, 4.0]
    assert interior[1][:2] == [7.0, 4.0]


def test_cmd_g5_chained_omits_ij_and_forwards_none():
    g = make_full_gcode_move()
    bezier_calls = []
    fake_motion = types.SimpleNamespace(
        engine=types.SimpleNamespace(
            submit_bezier=lambda *a: bezier_calls.append(a)
        )
    )
    objs = {"toolhead": g._toolhead, "motion": fake_motion}
    g.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: objs[name]
    )
    g.cmd_G5(ParamGcmd({"X": "10", "Y": "0", "P": "-3", "Q": "4"}))
    (args, _kwargs) = g.curve_calls[0]
    _newpos, interior, submit, _speed = args
    # chained: only P2, no P1
    assert len(interior) == 1
    assert interior[0][:2] == [7.0, 4.0]
    # invoke the submit closure and confirm i,j forwarded as None
    submit(10.0, 0.0, 0.0, 0.0, 50.0)
    assert bezier_calls, "submit_bezier should be called"
    i, j, p, q = (
        bezier_calls[0][0],
        bezier_calls[0][1],
        bezier_calls[0][2],
        bezier_calls[0][3],
    )
    assert i is None and j is None
    assert p == -3.0 and q == 4.0


def test_cmd_g5_rejects_i_without_j():
    g = make_full_gcode_move()
    try:
        g.cmd_G5(ParamGcmd({"X": "10", "Y": "0", "I": "1", "P": "0", "Q": "0"}))
        assert False
    except RuntimeError as e:
        assert "both be present or both omitted" in str(e)


def test_cmd_g5_1_requires_i_or_j():
    g = make_full_gcode_move()
    try:
        g.cmd_G5_1(ParamGcmd({"X": "10", "Y": "0"}))
        assert False
    except RuntimeError as e:
        assert "I and/or J" in str(e)


def test_cmd_g5_1_dispatches_quadratic():
    g = make_full_gcode_move()
    g.cmd_G5_1(ParamGcmd({"X": "10", "Y": "0", "I": "5", "J": "5"}))
    assert g.curve_calls
    (args, _kwargs) = g.curve_calls[0]
    newpos, interior, _submit, _speed = args
    # single quadratic control point Q1 = start + (I,J) = (5,5)
    assert interior[0][:2] == [5.0, 5.0]
