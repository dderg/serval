"""Every clocksync estimate klippy publishes must reach the motion engine for
the whole life of a connection — not just the first one. A record the router
stops receiving keeps projecting off a dead estimate, which lands the first
step volley in the MCU's past.
"""

import pytest

from klippy import clocksync, mcu


class StubEstimator:
    """The published-estimate half of the native ClockSyncEstimator: every
    stamped sample publishes, unstamped samples are dropped."""

    DECAY = 1.0 / 30.0

    def __init__(self, *_args):
        self.last_clock = 0
        self.time_avg = 0.0
        self.clock_avg = 0.0
        self.time_variance = 0.0
        self.clock_covariance = 0.0
        self.prediction_variance = 0.0
        self.last_prediction_time = 0.0
        self.min_half_rtt = 0.0
        self.min_rtt_time = 0.0
        self.sync_stable_count = 0
        self.synced = False
        self.get_clock_period_secs = 0.9839
        self.published = 0

    def handle_clock(
        self, raw_clock_low, sent_time, receive_time, mcu_freq, prev_freq
    ):
        self.last_clock = raw_clock_low
        if sent_time == 0.0:
            return None
        self.time_avg = sent_time
        self.clock_avg = float(raw_clock_low)
        self.synced = True
        self.published += 1
        return (mcu_freq, sent_time, float(raw_clock_low))


class FakeReactor:
    NOW = 0.0
    NEVER = 9e99

    def __init__(self):
        self.now = 100.0
        self.timers = []

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = max(self.now, waketime)
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        self.timers.append(callback)
        return callback

    def update_timer(self, timer, waketime):
        return timer


class FakeMsgParser:
    def get_constant_float(self, name):
        assert name == "CLOCK_FREQ"
        return 168000000.0

    def get_raw_data_dictionary(self):
        return b"{}"


class FakeSerial:
    def __init__(self, reactor):
        self.reactor = reactor
        self.handlers = {}
        self.clock = 1000000
        self.async_queries = 0

    def get_msgparser(self):
        return FakeMsgParser()

    @property
    def msgparser(self):
        return FakeMsgParser()

    def register_response(self, callback, name, oid=None):
        self.handlers[name] = callback

    def send_with_response(self, msg, response):
        return self.emit_clock()

    def engine_get_clock_async(self):
        self.async_queries += 1

    def emit_clock(self):
        self.reactor.now += 0.9839
        self.clock += 165000000
        return {
            "clock": self.clock,
            "high": 0,
            "#sent_time": self.reactor.now,
            "#receive_time": self.reactor.now + 0.0002,
        }

    def deliver_clock(self):
        self.handlers["clock"](self.emit_clock())


class FakeEngineMcu:
    def __init__(self):
        self.handle = 0
        self.estimates = []
        self.invalidations = 0
        self.nominal_freqs = []
        self.claims = []

    def available(self):
        return True

    def set_msgproto_dict(self, raw_dict):
        pass

    def claim(self, serial_path, baud):
        self.claims.append((serial_path, baud))
        return self.handle

    def invalidate_clock_est(self):
        self.invalidations += 1
        self.estimates.append(None)

    def set_nominal_clock_freq(self, freq_hz):
        self.nominal_freqs.append(freq_hz)

    def set_clock_est(self, freq, offset, last_clock, converged, host_now_raw):
        self.estimates.append(
            (self.handle, freq, offset, last_clock, converged)
        )


class FakeMcu:
    _identify_setup_motion_engine = mcu.MCU._identify_setup_motion_engine

    def __init__(self, reactor, serial, engine_mcu, sync):
        self._name = "mcu"
        self._reactor = reactor
        self._serial = serial
        self.engine_mcu = engine_mcu
        self._clocksync = sync
        self._serialport = "/dev/fake"
        self._baud = 250000
        self._mcu_freq = 168000000.0


@pytest.fixture
def connected(monkeypatch):
    monkeypatch.setattr(clocksync, "native_class", lambda name: StubEstimator)
    monkeypatch.setattr(
        clocksync,
        "get_danger_options",
        lambda: type("Danger", (), {"clock_sync_stable_ppm": 1.0})(),
    )
    reactor = FakeReactor()
    serial = FakeSerial(reactor)
    sync = clocksync.ClockSync(reactor)
    sync.connect(serial)
    engine = FakeEngineMcu()
    board = FakeMcu(reactor, serial, engine, sync)
    board._identify_setup_motion_engine()
    return board, serial, engine, sync


def published_estimates(engine):
    return [e for e in engine.estimates if e is not None]


def test_identify_invalidates_then_seeds_the_record(connected):
    _board, _serial, engine, sync = connected

    assert engine.invalidations == 1
    assert engine.estimates[0] is None, (
        "the record must be dropped before the callback re-seeds it, or the"
        " previous boot epoch's numbers survive the reconnect"
    )
    assert len(published_estimates(engine)) == 1
    _handle, freq, offset, last_clock, _converged = published_estimates(engine)[
        -1
    ]
    assert (offset, last_clock, freq) == (
        sync.clock_est[0],
        int(sync.clock_est[1]),
        sync.clock_est[2],
    )


def test_every_estimate_of_the_connection_reaches_the_engine(connected):
    _board, serial, engine, sync = connected
    seeded = len(published_estimates(engine))

    for _ in range(40):
        serial.deliver_clock()

    delivered = published_estimates(engine)
    assert len(delivered) == seeded + 40, (
        "every published estimate must reach the engine; the router record"
        " freezes on the first one otherwise"
    )
    assert {e[0] for e in delivered} == {0}, "all estimates on one live handle"
    _handle, freq, offset, last_clock, converged = delivered[-1]
    assert (offset, last_clock, freq) == (
        sync.clock_est[0],
        int(sync.clock_est[1]),
        sync.clock_est[2],
    )
    assert converged is True


def test_unstamped_samples_publish_nothing_and_break_nothing(connected):
    _board, serial, engine, _sync = connected
    before = len(published_estimates(engine))

    serial.handlers["clock"](
        {"clock": 12345, "#sent_time": 0.0, "#receive_time": 100.0}
    )
    assert len(published_estimates(engine)) == before

    serial.deliver_clock()
    assert len(published_estimates(engine)) == before + 1


def test_a_reconnect_reinvalidates_and_keeps_the_flow_alive(connected):
    board, serial, engine, _sync = connected
    serial.deliver_clock()

    board._clocksync.connect(serial)
    board._identify_setup_motion_engine()

    assert engine.invalidations == 2
    after_reconnect = len(published_estimates(engine))
    for _ in range(10):
        serial.deliver_clock()
    assert len(published_estimates(engine)) == after_reconnect + 10, (
        "the re-identify must leave a live callback, not a dead handle"
    )


def test_the_get_clock_cadence_comes_from_the_shared_estimator(connected):
    _board, serial, _engine, sync = connected
    start = 500.0

    next_wake = sync._get_clock_event(start)

    assert serial.async_queries == 1
    assert next_wake == start + sync._est.get_clock_period_secs
