import pytest
from fakes import FakeGcmd

from klippy import configfile, motion_setup
from klippy.motion import Motion


class ConfigError(Exception):
    pass


def cartesian_limits(extra="", max_accel=3000.0):
    limits, _axes, _kinematics, _consumed = (
        configfile._config_doc.read_motion_settings(
            "[printer]\nmax_velocity: 300\nmax_accel: %s\n%s"
            % (max_accel, extra)
        )
    )
    return limits


def corner_budget(extra=""):
    return cartesian_limits(extra)[5]


def test_corner_deviation_is_taken_verbatim():
    assert corner_budget("corner_deviation: 0.02\n") == (
        "corner_deviation",
        0.02,
    )


def test_scv_stays_a_speed():
    assert corner_budget("square_corner_velocity: 8.0\n") == (
        "square_corner_velocity",
        8.0,
    )


def test_both_corner_keys_is_a_config_error():
    with pytest.raises(configfile.error, match="corner_deviation"):
        corner_budget("square_corner_velocity: 8.0\ncorner_deviation: 0.02\n")


@pytest.mark.parametrize("max_accel", [1000.0, 3000.0, 70000.0])
def test_neither_key_defaults_to_the_0_04_allowance(max_accel):
    assert cartesian_limits(max_accel=max_accel)[5] == ("default", 0.04)


def test_scv_deviation_conversion_round_trips():
    d = motion_setup.corner_deviation_from_scv(8.0, 3000.0)
    assert motion_setup.scv_from_corner_deviation(d, 3000.0) == pytest.approx(
        8.0
    )


@pytest.mark.parametrize(
    "corner, expected",
    [
        (
            ("corner_deviation", 0.02),
            (0.02, motion_setup.scv_from_corner_deviation(0.02, 3000.0)),
        ),
        (
            ("default", 0.04),
            (0.04, motion_setup.scv_from_corner_deviation(0.04, 3000.0)),
        ),
        (
            ("square_corner_velocity", 8.0),
            (motion_setup.corner_deviation_from_scv(8.0, 3000.0), 8.0),
        ),
    ],
)
def test_corner_values_report_the_set_kind_verbatim(corner, expected):
    assert motion_setup.corner_values(corner, 3000.0) == expected


def test_corner_values_reject_an_unknown_kind():
    with pytest.raises(ValueError, match="unknown corner budget kind"):
        motion_setup.corner_values(("bogus", 1.0), 3000.0)


class CaptureEngine:
    def __init__(self, corner=("corner_deviation", 0.02)):
        self.corner = corner
        self.corner_deviation_calls = []
        self.square_corner_velocity_calls = []
        self.velocity_caps = []
        self.accel_caps = []

    def set_corner_deviation(self, value):
        self.corner_deviation_calls.append(value)

    def set_square_corner_velocity(self, value):
        self.square_corner_velocity_calls.append(value)

    def set_velocity_cap(self, value):
        self.velocity_caps.append(value)

    def set_accel_cap(self, value):
        self.accel_caps.append(value)

    def effective_limits(self):
        return (300.0, 3000.0, self.corner)


def make_motion(corner=("corner_deviation", 0.02)):
    m = Motion.__new__(Motion)
    m._max_velocity = 300.0
    m._max_accel = 3000.0
    m._corner = ("corner_deviation", 0.01)
    m._planner_ready = True
    m.engine = CaptureEngine(corner)
    return m


def test_set_velocity_limit_corner_deviation_passes_through():
    m = make_motion()
    m.cmd_SET_VELOCITY_LIMIT(FakeGcmd(CORNER_DEVIATION=0.05, error=ConfigError))
    assert m.engine.corner_deviation_calls == [0.05]
    assert m.engine.square_corner_velocity_calls == []


def test_set_velocity_limit_scv_passes_through_unconverted():
    m = make_motion()
    m.cmd_SET_VELOCITY_LIMIT(
        FakeGcmd(SQUARE_CORNER_VELOCITY=8.0, error=ConfigError)
    )
    assert m.engine.square_corner_velocity_calls == [8.0]
    assert m.engine.corner_deviation_calls == []


def test_set_velocity_limit_rejects_both_corner_keys():
    m = make_motion()
    with pytest.raises(ConfigError, match="exactly one"):
        m.cmd_SET_VELOCITY_LIMIT(
            FakeGcmd(
                SQUARE_CORNER_VELOCITY=8.0,
                CORNER_DEVIATION=0.05,
                error=ConfigError,
            )
        )
    assert m.engine.corner_deviation_calls == []
    assert m.engine.square_corner_velocity_calls == []


def test_set_velocity_limit_report_includes_the_kind_and_both_values():
    m = make_motion()
    gcmd = FakeGcmd(error=ConfigError)
    m.cmd_SET_VELOCITY_LIMIT(gcmd)
    assert len(gcmd.responses) == 1
    assert "corner_budget=corner_deviation:0.02" in gcmd.responses[0]
    assert "corner_deviation=0.02" in gcmd.responses[0]
    expected_scv = motion_setup.scv_from_corner_deviation(0.02, 3000.0)
    assert "square_corner_velocity=%s" % expected_scv in gcmd.responses[0]


def test_set_velocity_limit_report_of_an_scv_budget():
    m = make_motion(("square_corner_velocity", 8.0))
    gcmd = FakeGcmd(error=ConfigError)
    m.cmd_SET_VELOCITY_LIMIT(gcmd)
    assert "corner_budget=square_corner_velocity:8.0" in gcmd.responses[0]
    assert "square_corner_velocity=8.0" in gcmd.responses[0]


def test_properties_derive_the_unset_kind_from_effective_limits():
    m = make_motion()
    assert m.corner_deviation == 0.02
    assert m.square_corner_velocity == pytest.approx(
        motion_setup.scv_from_corner_deviation(0.02, 3000.0)
    )
    m = make_motion(("square_corner_velocity", 8.0))
    assert m.square_corner_velocity == 8.0
    assert m.corner_deviation == pytest.approx(
        motion_setup.corner_deviation_from_scv(8.0, 3000.0)
    )


def test_properties_before_planner_ready():
    m = make_motion()
    m._planner_ready = False
    assert m.corner_deviation == 0.01
    assert m.square_corner_velocity == pytest.approx(
        motion_setup.scv_from_corner_deviation(0.01, 3000.0)
    )
