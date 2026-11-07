import json
from pathlib import Path

import pytest

from klippy import clocksync

DATA = Path(__file__).with_name("data") / "clocksync_late_uptime.json"
CLOCK_MASK = 0xFFFFFFFF
# Eight calibration samples span 0.35 s with up to ~0.8 ms of send jitter, so
# their slope is good to a few 1000 ppm; the running regression refines it.
MAX_CALIBRATION_PPM = 3000.0
MAX_PREDICTION_ERROR_S = 0.005


class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0

    def register_timer(self, callback, waketime=NEVER):
        return callback

    def update_timer(self, timer, waketime):
        pass

    def pause(self, waketime):
        return waketime

    def monotonic(self):
        return 0.0


class FakeMsgParser:
    def __init__(self, freq):
        self.freq = freq

    def get_constant_float(self, name):
        assert name == "CLOCK_FREQ"
        return self.freq

    def create_command(self, msg):
        return msg


class FakeSerial:
    """Answers connect()'s queries from a recorded session."""

    def __init__(self, freq, uptime, connect_samples):
        self.msgparser = FakeMsgParser(freq)
        self.uptime = uptime
        self.connect_samples = list(connect_samples)

    def send_with_response(self, msg, response):
        if msg == "get_uptime":
            clock = self.uptime["clock"]
            return {
                "high": clock >> 32,
                "clock": clock & CLOCK_MASK,
                "#sent_time": self.uptime["sent"],
                "#receive_time": self.uptime["sent"],
            }
        assert msg == "get_clock"
        return clock_params(self.connect_samples.pop(0))

    def get_msgparser(self):
        return self.msgparser

    def alloc_command_queue(self):
        return None

    def register_response(self, callback, name):
        pass

    def set_clock_est(self, *args):
        pass


def clock_params(sample):
    return {
        "clock": sample["clock"] & CLOCK_MASK,
        "#sent_time": sample["sent"],
        "#receive_time": sample["received"],
    }


def connected(data, connect_count=8):
    sync = clocksync.ClockSync(FakeReactor())
    samples = data["samples"]
    sync.connect(
        FakeSerial(data["mcu_freq"], data["uptime"], samples[:connect_count])
    )
    return sync, samples[connect_count:]


def true_frequency(samples):
    n = len(samples)
    t = [s["sent"] for s in samples]
    c = [s["clock"] for s in samples]
    t_mean, c_mean = sum(t) / n, sum(c) / n
    cov = sum((a - t_mean) * (b - c_mean) for a, b in zip(t, c))
    var = sum((a - t_mean) ** 2 for a in t)
    return cov / var


def test_late_uptime_reply_does_not_skew_the_calibration():
    data = json.loads(DATA.read_text())
    sync, _ = connected(data)
    truth = true_frequency(data["samples"])
    error_ppm = 1e6 * (sync.clock_est[2] / truth - 1.0)
    assert abs(error_ppm) < MAX_CALIBRATION_PPM, error_ppm


def test_estimate_follows_samples_after_a_bad_start():
    data = json.loads(DATA.read_text())
    sync, later = connected(data)
    freq = data["mcu_freq"]
    for index, sample in enumerate(later):
        half_rtt = 0.5 * (sample["received"] - sample["sent"])
        predicted = sync.get_clock(sample["sent"] + half_rtt)
        error_s = (sample["clock"] - predicted) / freq
        assert abs(error_s) < MAX_PREDICTION_ERROR_S, (index, error_s)
        sync._handle_clock(clock_params(sample))


def test_implausible_frequency_fails_at_connect():
    nominal = 84e6
    actual = nominal * 1.08
    samples = [
        {"sent": 10.0 + 0.05 * i, "received": 10.0002 + 0.05 * i,
         "clock": int(5e8 + actual * 0.05 * i)}
        for i in range(8)
    ]  # fmt: skip
    data = {
        "mcu_freq": nominal,
        "uptime": {"sent": 9.95, "clock": int(5e8 - actual * 0.05)},
        "samples": samples,
    }
    with pytest.raises(clocksync.ClockSyncError, match="CLOCK_FREQ"):
        connected(data)


def test_secondary_mcu_uses_the_same_calibration():
    data = json.loads(DATA.read_text())
    main, _ = connected(data)
    secondary = clocksync.SecondarySync(FakeReactor(), main)
    samples = data["samples"]
    secondary.connect(FakeSerial(data["mcu_freq"], data["uptime"], samples[:8]))
    truth = true_frequency(samples)
    error_ppm = 1e6 * (secondary.clock_est[2] / truth - 1.0)
    assert abs(error_ppm) < MAX_CALIBRATION_PPM, error_ppm
