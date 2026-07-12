import pytest

from klippy import configfile, motion_setup
from klippy.motion import Motion


class ConfigError(Exception):
    pass


def cartesian_limits(extra="", max_accel=3000.0):
    limits, _axes, _limit_sections, _kinematics, _consumed = (
        configfile._config_doc.read_motion_settings(
            "[printer]\nmax_velocity: 300\nmax_accel: %s\n%s"
            % (max_accel, extra)
        )
    )
    return limits


def corner_deviation(extra=""):
    return cartesian_limits(extra)[5]


def test_corner_deviation_only_is_taken_verbatim():
    assert corner_deviation("corner_deviation: 0.02\n") == 0.02


def test_scv_only_converts_at_max_accel():
    assert corner_deviation("square_corner_velocity: 8.0\n") == pytest.approx(
        motion_setup.corner_deviation_from_scv(8.0, 3000.0)
    )


def test_both_corner_keys_is_a_config_error():
    with pytest.raises(configfile.error, match="corner_deviation"):
        corner_deviation(
            "square_corner_velocity: 8.0\ncorner_deviation: 0.02\n"
        )


def test_neither_key_defaults_to_converted_default_scv():
    assert corner_deviation() == pytest.approx(
        motion_setup.corner_deviation_from_scv(
            motion_setup.DEFAULT_SQUARE_CORNER_VELOCITY, 3000.0
        )
    )


def test_scv_deviation_conversion_round_trips():
    d = motion_setup.corner_deviation_from_scv(8.0, 3000.0)
    assert motion_setup.scv_from_corner_deviation(d, 3000.0) == pytest.approx(
        8.0
    )


class CaptureEngine:
    def __init__(self):
        self.corner_deviation_calls = []
        self.velocity_caps = []
        self.accel_caps = []

    def set_corner_deviation(self, value):
        self.corner_deviation_calls.append(value)

    def set_velocity_cap(self, value):
        self.velocity_caps.append(value)

    def set_accel_cap(self, value):
        self.accel_caps.append(value)

    def effective_limits(self):
        return (300.0, 3000.0, 0.02)


class FakeGcmd:
    error = ConfigError

    def __init__(self, **params):
        self.params = params
        self.responses = []

    def get_float(self, key, default=None, **kwargs):
        return self.params.get(key, default)

    def respond_info(self, msg):
        self.responses.append(msg)


def make_motion():
    m = Motion.__new__(Motion)
    m._max_velocity = 300.0
    m._max_accel = 3000.0
    m._corner_deviation = 0.01
    m._planner_ready = True
    m.engine = CaptureEngine()
    return m


def test_set_velocity_limit_corner_deviation_passes_through():
    m = make_motion()
    m.cmd_SET_VELOCITY_LIMIT(FakeGcmd(CORNER_DEVIATION=0.05))
    assert m.engine.corner_deviation_calls == [0.05]


def test_set_velocity_limit_scv_converts_at_configured_max_accel():
    m = make_motion()
    m.cmd_SET_VELOCITY_LIMIT(FakeGcmd(SQUARE_CORNER_VELOCITY=8.0))
    assert m.engine.corner_deviation_calls == [
        pytest.approx(motion_setup.corner_deviation_from_scv(8.0, 3000.0))
    ]


def test_set_velocity_limit_rejects_both_corner_keys():
    m = make_motion()
    with pytest.raises(ConfigError, match="exactly one"):
        m.cmd_SET_VELOCITY_LIMIT(
            FakeGcmd(SQUARE_CORNER_VELOCITY=8.0, CORNER_DEVIATION=0.05)
        )
    assert m.engine.corner_deviation_calls == []


def test_set_velocity_limit_report_includes_both_corner_values():
    m = make_motion()
    gcmd = FakeGcmd()
    m.cmd_SET_VELOCITY_LIMIT(gcmd)
    assert len(gcmd.responses) == 1
    assert "corner_deviation=0.02" in gcmd.responses[0]
    expected_scv = motion_setup.scv_from_corner_deviation(0.02, 3000.0)
    assert "square_corner_velocity=%s" % expected_scv in gcmd.responses[0]


def test_square_corner_velocity_property_is_derived_from_effective_limits():
    m = make_motion()
    assert m.corner_deviation == 0.02
    assert m.square_corner_velocity == pytest.approx(
        motion_setup.scv_from_corner_deviation(0.02, 3000.0)
    )


def test_square_corner_velocity_property_before_planner_ready():
    m = make_motion()
    m._planner_ready = False
    assert m.corner_deviation == 0.01
    assert m.square_corner_velocity == pytest.approx(
        motion_setup.scv_from_corner_deviation(0.01, 3000.0)
    )
