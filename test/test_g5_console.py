import types


class FakeKin:
    def __init__(self):
        self.checked = []

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
    m.max_velocity = 300.0
    m.max_accel = 3000.0
    m.kin = FakeKin()
    m.extruder = types.SimpleNamespace(check_move=lambda mv: None)
    m.bridge = types.SimpleNamespace(
        calls=[],
        get_last_move_time=lambda: 0.0,
        submit_bezier=lambda *a: m.bridge.calls.append(("bezier", a)),
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
        m.bridge.submit_bezier(dx, dy, dz, de, fr)

    interior = [[10.0, 500.0, 0.0], [10.0, 0.0, 0.0]]
    try:
        m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
        assert False, "expected out-of-range rejection"
    except RuntimeError as e:
        assert "out of range" in str(e)


def test_move_curve_submits_and_advances_when_in_range():
    m = make_motion()

    def submit(dx, dy, dz, de, fr):
        m.bridge.submit_bezier(dx, dy, dz, de, fr)

    interior = [[10.0, 5.0, 0.0], [10.0, -5.0, 0.0]]
    m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
    assert m.bridge.calls and m.bridge.calls[0][0] == "bezier"
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
