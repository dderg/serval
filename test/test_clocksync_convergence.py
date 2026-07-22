import types

import klippy.clocksync as clocksync

MCU_FREQ = 400e6


class _FakeReactor:
    NOW = 0.0
    NEVER = 9e99

    def register_timer(self, cb):
        return cb

    def update_timer(self, timer, when):
        pass

    def monotonic(self):
        return 0.0


def make_sync(freq=MCU_FREQ):
    cs = clocksync.ClockSync(_FakeReactor())
    cs.mcu_freq = freq
    cs.last_clock = 0
    cs.clock_avg = 0.0
    cs.time_avg = 0.0
    cs.clock_est = (0.0, 0.0, freq)
    cs.prediction_variance = (0.001 * freq) ** 2
    return cs


def feed(cs, sent_time, clock):
    cs._handle_clock(
        {
            "clock": int(clock) & 0xFFFFFFFF,
            "#sent_time": sent_time,
            "#receive_time": sent_time + 0.0001,
        }
    )


def feed_exact(cs, sent_time):
    feed(cs, sent_time, cs.mcu_freq * sent_time)


def test_starts_unsynced():
    assert not make_sync().is_synced()


def test_syncs_after_consecutive_stable_freq_samples():
    cs = make_sync()
    for i in range(clocksync.SYNC_STABLE_SAMPLES):
        assert not cs.is_synced()
        feed_exact(cs, 1.0 + i)
    assert cs.is_synced()


def test_unstable_freq_resets_the_stability_count():
    cs = make_sync()
    feed_exact(cs, 1.0)
    feed_exact(cs, 2.0)
    drift = 200e-6 * MCU_FREQ
    feed(cs, 3.0, MCU_FREQ * 3.0 + drift * 3.0)
    feed(cs, 4.0, MCU_FREQ * 4.0 + drift * 4.0)
    assert not cs.is_synced()


def test_sync_latches_once_reached():
    cs = make_sync()
    for i in range(clocksync.SYNC_STABLE_SAMPLES):
        feed_exact(cs, 1.0 + i)
    assert cs.is_synced()
    t = 1.0 + clocksync.SYNC_STABLE_SAMPLES
    feed(cs, t, MCU_FREQ * t + 500e-6 * MCU_FREQ * t)
    assert cs.is_synced()


def test_secondary_requires_main_sync_too():
    main = make_sync()
    secondary = clocksync.SecondarySync(_FakeReactor(), main)
    secondary.mcu_freq = MCU_FREQ
    secondary._synced = True
    assert not secondary.is_synced()
    main._synced = True
    assert secondary.is_synced()


def test_connect_file_is_always_synced():
    cs = clocksync.ClockSync(_FakeReactor())
    serial = types.SimpleNamespace(
        msgparser=types.SimpleNamespace(
            get_constant_float=lambda name: MCU_FREQ
        ),
        set_clock_est=lambda *args: None,
    )
    cs.connect_file(serial)
    assert cs.is_synced()
