import pytest
from fakes import FakeConfig, FakeGcmd, FakeGcode, FakePrinter

from klippy.extras.heaters.profiles import (
    PID_PROFILE_VERSION,
    ProfileManager,
    saved_profile_options,
)


class FakeConfigfile:
    def __init__(self):
        self.values = {}
        self.removed = []

    def set(self, section, option, value):
        self.values.setdefault(section, {})[option] = value

    def remove_section(self, section):
        self.removed.append(section)


class FakeControl:
    def __init__(self, profile):
        self.profile = profile

    def get_profile(self):
        return self.profile


class FakeProfileHeater:
    def __init__(self, config, control_profile=None):
        self.config = config
        self.short_name = "extruder"
        self.printer = FakePrinter()
        self.gcode = FakeGcode()
        self.gcode.error = FakeGcmd.error
        self.configfile = FakeConfigfile()
        self.smooth_time = 1.0
        self._control = FakeControl(control_profile or {})
        self.set_controls = []

    def get_control(self):
        return self._control

    def set_control(self, control, keep_target=True):
        self.set_controls.append((control, keep_target))
        self._control = control

    def lookup_control(self, profile, load_clean=False):
        return FakeControl(profile)

    def get_smooth_time(self):
        return self.smooth_time


def gcode_error():
    return FakeGcmd.error


def make_config(values=None, sections=None):
    return FakeConfig(name="extruder", values=values or {}, sections=sections)


def test_default_pid_profile_loads_from_heater_section():
    config = make_config(
        {
            "control": "pid",
            "pid_kp": "30.0",
            "pid_ki": "2.0",
            "pid_kd": "100.0",
        }
    )
    heater = FakeProfileHeater(config)
    heater.gcode.error = FakeGcmd("").error
    pmgr = ProfileManager(heater)
    profile = pmgr.init_default_profile()
    assert profile["control"] == "pid"
    assert profile["name"] == "default"
    assert profile["pid_kp"] == 30.0
    assert profile["smooth_time"] is None


def test_unknown_control_type_fails_loudly():
    config = make_config({"control": "quantum"})
    heater = FakeProfileHeater(config)
    pmgr = ProfileManager(heater)
    with pytest.raises(FakePrinter.config_error):
        pmgr.init_default_profile()


def test_dual_loop_profile_requires_inner_gains():
    config = make_config(
        {
            "control": "dual_loop_pid",
            "pid_kp": "30.0",
            "pid_ki": "2.0",
            "pid_kd": "100.0",
        }
    )
    heater = FakeProfileHeater(config)
    pmgr = ProfileManager(heater)
    with pytest.raises(FakeGcmd.error):
        pmgr.init_default_profile()


def test_save_profile_persists_dual_loop_inner_gains():
    # Regression: saving a dual_loop_pid profile used to silently drop
    # the inner_pid_* gains, losing the inner loop calibration.
    profile = {
        "control": "dual_loop_pid",
        "name": "default",
        "pid_target": 60.0,
        "pid_tolerance": 0.02,
        "smooth_time": None,
        "pid_kp": 30.0,
        "pid_ki": 2.0,
        "pid_kd": 100.0,
        "inner_pid_kp": 40.0,
        "inner_pid_ki": 3.0,
        "inner_pid_kd": 120.0,
    }
    heater = FakeProfileHeater(make_config(), control_profile=profile)
    pmgr = ProfileManager(heater)
    pmgr.save_profile(profile_name="tuned", verbose=False)
    saved = heater.configfile.values["pid_profile extruder tuned"]
    assert saved["pid_version"] == PID_PROFILE_VERSION
    assert saved["inner_pid_kp"] == "40.000"
    assert saved["inner_pid_ki"] == "3.000"
    assert saved["inner_pid_kd"] == "120.000"
    assert pmgr.profiles["tuned"]["name"] == "tuned"


def test_save_profile_rejects_unsupported_control():
    # Regression: PID_PROFILE SAVE= on an mpc/watermark heater used to
    # crash with KeyError('pid_kp').
    profile = {"control": "mpc", "name": "default"}
    heater = FakeProfileHeater(make_config(), control_profile=profile)
    pmgr = ProfileManager(heater)
    with pytest.raises(FakeGcmd.error):
        pmgr.save_profile(profile_name="tuned", verbose=False)


def test_saved_profile_options_by_control():
    assert saved_profile_options({"control": "watermark"}) is None
    assert saved_profile_options({"control": "mpc"}) is None
    assert "pid_kp" in saved_profile_options({"control": "pid"})
    dual = saved_profile_options({"control": "dual_loop_pid"})
    assert "inner_pid_kd" in dual


def test_load_profile_falls_back_to_default():
    default_profile = {
        "control": "pid",
        "name": "fallback",
        "pid_target": 60.0,
        "pid_tolerance": 0.02,
        "smooth_time": None,
        "pid_kp": 30.0,
        "pid_ki": 2.0,
        "pid_kd": 100.0,
    }
    heater = FakeProfileHeater(
        make_config(), control_profile={"name": "current"}
    )
    pmgr = ProfileManager(heater)
    pmgr.profiles["fallback"] = default_profile
    gcmd = FakeGcmd({"DEFAULT": "fallback"})
    pmgr.load_profile("missing", gcmd, True)
    assert heater.set_controls
    assert heater.get_control().get_profile() is default_profile


def test_load_profile_unknown_without_default_fails():
    heater = FakeProfileHeater(
        make_config(), control_profile={"name": "current"}
    )
    pmgr = ProfileManager(heater)
    gcmd = FakeGcmd({})
    with pytest.raises(FakeGcmd.error):
        pmgr.load_profile("missing", gcmd, True)


def test_remove_profile_clears_config_section():
    heater = FakeProfileHeater(make_config())
    pmgr = ProfileManager(heater)
    pmgr.profiles["tuned"] = {"name": "tuned"}
    pmgr.remove_profile("tuned", None, True)
    assert "tuned" not in pmgr.profiles
    assert heater.configfile.removed == ["pid_profile extruder tuned"]
