import pytest

from klippy import motion


class FakeKin:
    def __init__(self, dirty):
        self._dirty = list(dirty)
        self.cleared = []

    def parked_dirty_axes(self):
        return list(self._dirty)

    def clear_parked_dirty(self, axes):
        self.cleared.append(list(axes))


class FakeBridge:
    def __init__(self, measured, raises=None):
        self._measured = measured
        self._raises = raises
        self.queries = 0

    def query_motor_positions(self):
        self.queries += 1
        if self._raises is not None:
            raise self._raises
        return self._measured


class FakeMotion:
    resync_parked_servos = motion.Motion.resync_parked_servos

    def __init__(self, dirty, measured, raises=None):
        self.kin = FakeKin(dirty)
        self.bridge = FakeBridge(measured, raises)
        self.commanded_pos = [10.0, 20.0, 30.0, 4.0]
        self.set_position_calls = []

    def set_position(self, newpos, homing_axes=()):
        self.set_position_calls.append((list(newpos), tuple(homing_axes)))
        self.commanded_pos[:] = newpos


def test_resync_no_dirty_axes_does_not_query():
    m = FakeMotion(dirty=[], measured={})
    m.resync_parked_servos()
    assert m.bridge.queries == 0
    assert m.set_position_calls == []


def test_resync_dirty_z_reseats_only_z():
    m = FakeMotion(dirty=[2], measured={"z": (123.5, 0.0)})
    m.resync_parked_servos()
    assert m.bridge.queries == 1
    newpos, homing_axes = m.set_position_calls[0]
    assert newpos == [10.0, 20.0, 123.5, 4.0]
    assert homing_axes == ()
    assert m.kin.cleared == [[2]]


def test_resync_dirty_xy_reseats_both():
    m = FakeMotion(dirty=[0, 1], measured={"x": (1.0, 0.0), "y": (2.0, 0.0)})
    m.resync_parked_servos()
    newpos, _ = m.set_position_calls[0]
    assert newpos == [1.0, 2.0, 30.0, 4.0]


def test_resync_query_error_does_not_move():
    err = RuntimeError("ec-rt timeout")
    m = FakeMotion(dirty=[2], measured={}, raises=err)
    with pytest.raises(RuntimeError, match="ec-rt timeout"):
        m.resync_parked_servos()
    assert m.set_position_calls == []
    assert m.kin.cleared == []


class _MoveKin(FakeKin):
    def check_move(self, move):
        pass

    def active_rails(self, dx, dy, dz):
        return []


class FakeExtruder:
    def check_move(self, move):
        pass


class _SubmitBridge(FakeBridge):
    def __init__(self, measured):
        super().__init__(measured)
        self.moves = []

    def get_last_move_time(self):
        return 0.0

    def submit_move(self, dx, dy, dz, de, feedrate):
        self.moves.append((dx, dy, dz, de, feedrate))


class MoveMotion(FakeMotion):
    move = motion.Motion.move
    move_curve = motion.Motion.move_curve
    _fire_active_callbacks = motion.Motion._fire_active_callbacks

    max_accel = 1000.0
    max_velocity = 100.0

    def __init__(self, dirty, measured):
        super().__init__(dirty, measured)
        self.kin = _MoveKin(dirty)
        self.bridge = _SubmitBridge(measured)
        self.extruder = FakeExtruder()

    def _axis_limit(self, axis, kind):
        return 100.0

    def get_last_move_time(self):
        return 0.0

    def _bump_pending_end_time(self, dt):
        pass

    def _sync_print_time(self):
        pass


def test_move_resyncs_before_computing_deltas():
    m = MoveMotion(dirty=[2], measured={"z": (123.5, 0.0)})
    m.commanded_pos = [10.0, 20.0, 30.0, 4.0]
    m.move([10.0, 20.0, 140.0, 4.0], 50.0)
    assert m.bridge.queries == 1
    dx, dy, dz, de, _feedrate = m.bridge.moves[0]
    assert (dx, dy) == (0.0, 0.0)
    assert dz == pytest.approx(140.0 - 123.5)
    assert de == 0.0


def test_move_curve_resyncs_before_computing_deltas():
    m = MoveMotion(dirty=[2], measured={"z": (123.5, 0.0)})
    m.commanded_pos = [10.0, 20.0, 30.0, 4.0]
    submitted = []

    def submit(dx, dy, dz, de, feedrate):
        submitted.append((dx, dy, dz, de, feedrate))

    m.move_curve([10.0, 20.0, 140.0, 4.0], [], submit, 50.0)
    assert m.bridge.queries == 1
    dx, dy, dz, de, _feedrate = submitted[0]
    assert (dx, dy) == (0.0, 0.0)
    assert dz == pytest.approx(140.0 - 123.5)
    assert de == 0.0
