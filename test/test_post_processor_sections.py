"""The [axis]/[post_processor] section parsing itself is tested in Rust
(planner-config from_doc_tests); these tests exercise the same reader
end-to-end through the engine's load_motion_config, plus the Python-side
SET_POST_PROCESSOR error path."""

import pytest

from klippy import configfile
from klippy.motion import Motion

MINIMAL = "[printer]\nmax_velocity: 300\nmax_accel: 3000\n"
SPATIAL = "[axis x]\n[axis y]\n[axis z]\n"


def load(extra):
    _limits, axes, _limit_sections, _kinematics, _consumed = (
        configfile._config_doc.read_motion_settings(MINIMAL + SPATIAL + extra)
    )
    return axes


def test_input_shaper_section_rejected_pointing_at_post_processor():
    with pytest.raises(configfile.error, match=r"\[post_processor"):
        load("[input_shaper]\n")


def test_post_processor_missing_type_fails():
    with pytest.raises(configfile.error, match="type"):
        load("[post_processor is]\nfrequency_hz: 50.0\n")


def test_axis_referencing_undeclared_post_processor_fails():
    with pytest.raises(configfile.error, match="pa"):
        load("[axis e]\nfollows: x,y,z\npost_processors: pa\n")


def test_happy_path_parses_sections_for_init_planner():
    sections = load(
        "[axis e]\nfollows: x,y,z\npost_processors: is,pa\n"
        "[post_processor is]\ntype: smooth_bell\nsmooth_time: 0.0182\n"
        "[post_processor pa]\ntype: linear_pressure_advance\nk: 0.04\n"
    )
    assert ("e", ["x", "y", "z"], [], ["is", "pa"]) in sections


def make_toolhead():
    return Motion.__new__(Motion)


class CommandError(Exception):
    pass


class StubGcmd:
    error = CommandError

    def __init__(self, params):
        self.params = params

    def get(self, key):
        return self.params[key]

    def get_command_parameters(self):
        return dict(self.params)


class RaisingEngine:
    def update_post_processor(self, name, key, value):
        raise ValueError("unknown post_processor '%s'" % name)


def test_set_post_processor_engine_error_becomes_command_error():
    th = make_toolhead()
    th.engine = RaisingEngine()
    gcmd = StubGcmd({"NAME": "ghost", "K": "0.1"})
    with pytest.raises(CommandError, match="ghost"):
        th.cmd_SET_POST_PROCESSOR(gcmd)
