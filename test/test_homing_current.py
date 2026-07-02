from klippy.extras.homing import Homing


class _FakeToolhead:
    def get_last_move_time(self):
        return 7.5


class _FakeReactor:
    def __init__(self):
        self.pauses = []

    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        self.pauses.append(waketime)


class _FakePrinter:
    def __init__(self, reactor):
        self._reactor = reactor

    def get_reactor(self):
        return self._reactor


class _FakeCurrentHelper:
    def __init__(self, dwell_time):
        self._dwell_time = dwell_time
        self.calls = []

    def set_current_for_homing(self, print_time, pre_homing):
        self.calls.append((print_time, pre_homing))
        return self._dwell_time


class _FakeRail:
    def __init__(self, helpers):
        self._helpers = helpers

    def get_tmc_current_helpers(self):
        return self._helpers


def _homing(reactor):
    h = Homing.__new__(Homing)
    h.printer = _FakePrinter(reactor)
    return h


def test_applies_to_every_helper_and_waits_for_the_slowest():
    fast = _FakeCurrentHelper(0.5)
    slow = _FakeCurrentHelper(1.0)
    reactor = _FakeReactor()

    _homing(reactor)._set_homing_current(
        _FakeToolhead(), [_FakeRail([fast, slow])], pre_homing=True
    )

    assert fast.calls == [(7.5, True)]
    assert slow.calls == [(7.5, True)]
    assert reactor.pauses == [1.0]


def test_skips_steppers_without_tmc_drivers():
    helper = _FakeCurrentHelper(0.5)
    reactor = _FakeReactor()

    _homing(reactor)._set_homing_current(
        _FakeToolhead(), [_FakeRail([None, helper])], pre_homing=False
    )

    assert helper.calls == [(7.5, False)]
    assert reactor.pauses == [0.5]


def test_no_wait_when_no_current_change_needed():
    reactor = _FakeReactor()

    _homing(reactor)._set_homing_current(
        _FakeToolhead(), [_FakeRail([_FakeCurrentHelper(0.0)])], pre_homing=True
    )

    assert reactor.pauses == []


def test_applies_across_every_coupled_rail():
    homed = _FakeCurrentHelper(0.3)
    partner = _FakeCurrentHelper(0.8)
    reactor = _FakeReactor()

    _homing(reactor)._set_homing_current(
        _FakeToolhead(),
        [_FakeRail([homed]), _FakeRail([partner])],
        pre_homing=True,
    )

    assert homed.calls == [(7.5, True)]
    assert partner.calls == [(7.5, True)]
    assert reactor.pauses == [0.8]


def test_helper_shared_between_rails_is_switched_once():
    shared = _FakeCurrentHelper(0.5)
    reactor = _FakeReactor()

    _homing(reactor)._set_homing_current(
        _FakeToolhead(),
        [_FakeRail([shared]), _FakeRail([shared])],
        pre_homing=False,
    )

    assert shared.calls == [(7.5, False)]
    assert reactor.pauses == [0.5]
