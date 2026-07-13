import pytest

from klippy.extras.pressure_advance_compat import PressureAdvanceCompat


class ConfigError(Exception):
    pass


class CommandError(Exception):
    pass


class StubSection:
    def __init__(self, name, options):
        self.name = name
        self.options = options

    def get_name(self):
        return self.name

    def get(self, key, default="!missing"):
        value = self.options.get(key, default)
        if value == "!missing":
            raise ConfigError("missing option '%s' in [%s]" % (key, self.name))
        return value

    def getlist(self, key, default="!missing"):
        value = self.get(key, default)
        if isinstance(value, str):
            return [v.strip() for v in value.split(",") if v.strip()]
        return list(value or [])

    def getfloat(self, key, default="!missing", **kwargs):
        value = self.get(key, default)
        return None if value is None else float(value)


class StubGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, callback, desc=None):
        self.commands[name] = callback


class StubEngine:
    def __init__(self, params=None):
        self.calls = []
        self.params = dict(params or {})

    def update_post_processor(self, name, key, value):
        self.calls.append((name, key, value))
        self.params[(name, key)] = value

    def post_processor_param(self, name, key):
        return self.params.get((name, key))


class RaisingEngine:
    def update_post_processor(self, name, key, value):
        raise ValueError("unknown post_processor '%s'" % name)


class StubActiveExtruder:
    def __init__(self, name):
        self.name = name

    def get_name(self):
        return self.name


class StubToolhead:
    def __init__(self, active_extruder_name):
        self.active_extruder_name = active_extruder_name

    def get_extruder(self):
        return StubActiveExtruder(self.active_extruder_name)


class StubPrinter:
    def __init__(self, objects):
        self.objects = objects

    def lookup_object(self, name, default="!missing"):
        if name in self.objects:
            return self.objects[name]
        if default == "!missing":
            raise KeyError(name)
        return default


class StubConfig:
    error = ConfigError

    def __init__(self, sections, printer, options=None):
        self.sections = sections
        self.printer = printer
        self.options = options or {}

    def get_printer(self):
        return self.printer

    def get(self, key, default="!missing"):
        value = self.options.get(key, default)
        if value == "!missing":
            raise ConfigError("missing '%s'" % key)
        return value

    def get_prefix_sections(self, prefix):
        return [s for s in self.sections if s.get_name().startswith(prefix)]


class StubGcmd:
    error = CommandError

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, key, default="!missing"):
        value = self.params.get(key, default)
        if value == "!missing":
            raise CommandError("missing '%s'" % key)
        return value

    def get_float(self, key, default="!missing", **kwargs):
        value = self.params.get(key, default)
        if value == "!missing":
            raise CommandError("missing '%s'" % key)
        return None if value is None else float(value)

    def respond_info(self, msg):
        self.responses.append(msg)


def axis(name, **opts):
    return StubSection("axis " + name, opts)


def post_processor(name, **opts):
    return StubSection("post_processor " + name, opts)


def extruder(name="extruder", **opts):
    return StubSection(name, opts)


def make(sections, engine=None, options=None, active_extruder=None):
    gcode = StubGcode()
    objects = {"gcode": gcode}
    if engine is not None:
        objects["motion_engine"] = engine
    if active_extruder is not None:
        objects["toolhead"] = StubToolhead(active_extruder)
    printer = StubPrinter(objects)
    cfg = StubConfig(sections, printer, options)
    obj = PressureAdvanceCompat(cfg)
    return obj, gcode, printer


DECLARED = [
    axis("e", follows="x,y,z", post_processors="pa"),
    post_processor("pa", type="linear_pressure_advance", k="0.04"),
    extruder(axis="e"),
]

DECLARED_WITH_SMOOTH = [
    axis("e", follows="x,y,z", post_processors="pa,st"),
    post_processor("pa", type="linear_pressure_advance", k="0.04"),
    post_processor("st", type="smooth_triangle", smooth_time="0.04"),
    extruder(axis="e"),
]


def run(gcode, params):
    gcmd = StubGcmd(params)
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    return gcmd


def test_binds_pa_from_extruder_axis():
    obj, gcode, _ = make(DECLARED)
    target = obj.extruders["extruder"].advance
    assert target.enabled
    assert target.post_processor == "pa"
    assert target.initial_value == pytest.approx(0.04)
    assert "SET_PRESSURE_ADVANCE" in gcode.commands


def test_binds_smooth_time_from_extruder_axis():
    obj, _, _ = make(DECLARED_WITH_SMOOTH)
    target = obj.extruders["extruder"].smooth_time
    assert target.enabled
    assert target.post_processor == "st"
    assert target.initial_value == pytest.approx(0.04)


def test_missing_pa_is_disabled_not_a_config_error():
    sections = [axis("e", follows="x,y,z"), extruder(axis="e")]
    engine = StubEngine()
    obj, gcode, _ = make(sections, engine=engine)
    assert not obj.extruders["extruder"].advance.enabled
    gcmd = run(gcode, {"EXTRUDER": "extruder", "ADVANCE": "0.05"})
    assert engine.calls == []
    assert any("cannot set ADVANCE" in r for r in gcmd.responses)


def test_smooth_time_disabled_reports_but_advance_applies():
    engine = StubEngine()
    _, gcode, _ = make(DECLARED, engine=engine)
    gcmd = run(
        gcode,
        {"EXTRUDER": "extruder", "ADVANCE": "0.05", "SMOOTH_TIME": "0.04"},
    )
    assert engine.calls == [("pa", "k", 0.05)]
    assert any("cannot set SMOOTH_TIME" in r for r in gcmd.responses)


def test_advance_updates_engine():
    engine = StubEngine()
    _, gcode, _ = make(DECLARED, engine=engine)
    run(gcode, {"EXTRUDER": "extruder", "ADVANCE": "0.1"})
    assert engine.calls == [("pa", "k", 0.1)]


def test_smooth_time_updates_engine():
    engine = StubEngine()
    _, gcode, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    run(gcode, {"EXTRUDER": "extruder", "SMOOTH_TIME": "0.06"})
    assert engine.calls == [("st", "smooth_time", 0.06)]


def test_smooth_time_zero_disables_smoothing():
    engine = StubEngine()
    _, gcode, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    run(gcode, {"EXTRUDER": "extruder", "SMOOTH_TIME": "0"})
    assert engine.calls == [("st", "smooth_time", 0.0)]


def test_multiple_pa_on_axis_disabled_with_hint():
    sections = [
        axis("e", post_processors="pa1,pa2"),
        post_processor("pa1", type="linear_pressure_advance", k="0.02"),
        post_processor("pa2", type="linear_pressure_advance", k="0.03"),
        extruder(axis="e"),
    ]
    obj, _, _ = make(sections)
    target = obj.extruders["extruder"].advance
    assert not target.enabled
    assert "multiple" in target.reason
    assert "post_processor" in target.reason


def test_override_resolves_multiple():
    sections = [
        axis("e", post_processors="pa1,pa2"),
        post_processor("pa1", type="linear_pressure_advance", k="0.02"),
        post_processor("pa2", type="linear_pressure_advance", k="0.03"),
        extruder(axis="e"),
    ]
    obj, _, _ = make(sections, options={"post_processor": "pa2"})
    target = obj.extruders["extruder"].advance
    assert target.enabled
    assert target.post_processor == "pa2"
    assert target.initial_value == pytest.approx(0.03)


def test_override_unknown_name_fails():
    with pytest.raises(ConfigError, match="ghost"):
        make(DECLARED, options={"post_processor": "ghost"})


def test_smooth_override_unknown_name_fails():
    with pytest.raises(ConfigError, match="ghost"):
        make(DECLARED_WITH_SMOOTH, options={"smooth_post_processor": "ghost"})


def test_unknown_extruder_errors():
    _, gcode, _ = make(DECLARED, engine=StubEngine())
    with pytest.raises(CommandError, match="extruder9"):
        run(gcode, {"EXTRUDER": "extruder9", "ADVANCE": "0.1"})


def test_default_extruder_resolved_via_toolhead():
    engine = StubEngine()
    _, gcode, _ = make(DECLARED, engine=engine, active_extruder="extruder")
    run(gcode, {"ADVANCE": "0.1"})
    assert engine.calls == [("pa", "k", 0.1)]


def test_multi_extruder_binds_per_axis():
    sections = [
        axis("e", post_processors="pa"),
        axis("d", post_processors="pa2"),
        post_processor("pa", type="linear_pressure_advance", k="0.04"),
        post_processor("pa2", type="linear_pressure_advance", k="0.02"),
        extruder("extruder", axis="e"),
        extruder("extruder1", axis="d"),
    ]
    engine = StubEngine()
    _, gcode, _ = make(sections, engine=engine)
    run(gcode, {"EXTRUDER": "extruder1", "ADVANCE": "0.1"})
    assert engine.calls == [("pa2", "k", 0.1)]


def test_no_args_reports_live_values():
    engine = StubEngine({("pa", "k"): 0.123})
    _, gcode, _ = make(DECLARED, engine=engine)
    gcmd = run(gcode, {"EXTRUDER": "extruder"})
    assert engine.calls == []
    assert any("0.123" in r for r in gcmd.responses)


def test_no_args_reports_disabled_smooth_time():
    _, gcode, _ = make(DECLARED, engine=StubEngine())
    gcmd = run(gcode, {"EXTRUDER": "extruder"})
    assert any("smooth_time: disabled" in r for r in gcmd.responses)


def test_status_fields_read_live_from_engine():
    engine = StubEngine({("pa", "k"): 0.07, ("st", "smooth_time"): 0.01})
    obj, _, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    fields = obj.get_status_fields("extruder")
    assert fields["pressure_advance"] == pytest.approx(0.07)
    assert fields["smooth_time"] == pytest.approx(0.01)


def test_status_fields_fall_back_to_config_without_engine():
    obj, _, _ = make(DECLARED_WITH_SMOOTH)
    fields = obj.get_status_fields("extruder")
    assert fields["pressure_advance"] == pytest.approx(0.04)
    assert fields["smooth_time"] == pytest.approx(0.04)


def test_status_fields_omit_disabled_targets():
    obj, _, _ = make(DECLARED)
    assert set(obj.get_status_fields("extruder")) == {"pressure_advance"}
    sections = [axis("e", follows="x,y,z"), extruder(axis="e")]
    obj, _, _ = make(sections)
    assert obj.get_status_fields("extruder") == {}


def test_status_fields_unknown_extruder_is_empty():
    obj, _, _ = make(DECLARED)
    assert obj.get_status_fields("extruder7") == {}


def test_engine_error_becomes_command_error():
    _, gcode, _ = make(DECLARED, engine=RaisingEngine())
    with pytest.raises(CommandError, match="pa"):
        run(gcode, {"EXTRUDER": "extruder", "ADVANCE": "0.1"})
