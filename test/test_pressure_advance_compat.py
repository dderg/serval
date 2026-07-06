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
    def __init__(self):
        self.calls = []

    def update_post_processor(self, name, key, value):
        self.calls.append((name, key, value))


class RaisingEngine:
    def update_post_processor(self, name, key, value):
        raise ValueError("unknown post_processor '%s'" % name)


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


def make(sections, engine=None, options=None):
    gcode = StubGcode()
    objects = {"gcode": gcode}
    if engine is not None:
        objects["motion_engine"] = engine
    printer = StubPrinter(objects)
    cfg = StubConfig(sections, printer, options)
    obj = PressureAdvanceCompat(cfg)
    return obj, gcode, printer


DECLARED = [
    axis("e", follows="x,y,z", post_processors="pa"),
    post_processor("pa", type="linear_pressure_advance", k="0.04"),
]

DECLARED_WITH_SMOOTH = [
    axis("e", follows="x,y,z", post_processors="pa,st"),
    post_processor("pa", type="linear_pressure_advance", k="0.04"),
    post_processor("st", type="smooth_triangle", smooth_time="0.04"),
]


def test_auto_discovers_single_pa_post_processor():
    obj, gcode, _ = make(DECLARED)
    assert obj.post_processor == "pa"
    assert obj.last_advance == pytest.approx(0.04)
    assert "SET_PRESSURE_ADVANCE" in gcode.commands


def test_no_pa_post_processor_fails_loudly():
    sections = [axis("x"), post_processor("is", type="smooth_mzv")]
    with pytest.raises(ConfigError, match="no"):
        make(sections)


def test_multiple_pa_post_processors_fail_without_override():
    sections = [
        axis("e", post_processors="pa1,pa2"),
        post_processor("pa1", type="linear_pressure_advance", k="0.02"),
        post_processor("pa2", type="linear_pressure_advance", k="0.03"),
    ]
    with pytest.raises(ConfigError, match="multiple"):
        make(sections)


def test_override_selects_named_post_processor():
    sections = [
        axis("e", post_processors="pa1,pa2"),
        post_processor("pa1", type="linear_pressure_advance", k="0.02"),
        post_processor("pa2", type="linear_pressure_advance", k="0.03"),
    ]
    obj, _, _ = make(sections, options={"post_processor": "pa2"})
    assert obj.post_processor == "pa2"
    assert obj.last_advance == pytest.approx(0.03)


def test_override_unknown_name_fails():
    with pytest.raises(ConfigError, match="ghost"):
        make(DECLARED, options={"post_processor": "ghost"})


def test_advance_updates_engine_and_tracks_value():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED, engine=engine)
    gcmd = StubGcmd({"ADVANCE": "0.1"})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert engine.calls == [("pa", "k", 0.1)]
    assert obj.last_advance == pytest.approx(0.1)


def test_no_advance_reports_last_value():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED, engine=engine)
    gcmd = StubGcmd({})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert engine.calls == []
    assert any("0.04" in r for r in gcmd.responses)


def test_smooth_time_ignored_without_smooth_post_processor():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED, engine=engine)
    assert obj.smooth_post_processor is None
    gcmd = StubGcmd({"ADVANCE": "0.05", "SMOOTH_TIME": "0.04"})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert engine.calls == [("pa", "k", 0.05)]
    assert any("SMOOTH_TIME" in r for r in gcmd.responses)


def test_auto_discovers_smooth_post_processor():
    obj, _, _ = make(DECLARED_WITH_SMOOTH)
    assert obj.smooth_post_processor == "st"
    assert obj.last_smooth_time == pytest.approx(0.04)


def test_multiple_smooth_post_processors_fail_without_override():
    sections = [
        axis("e", post_processors="pa,st1,st2"),
        post_processor("pa", type="linear_pressure_advance", k="0.04"),
        post_processor("st1", type="smooth_triangle", smooth_time="0.02"),
        post_processor("st2", type="smooth_triangle", smooth_time="0.06"),
    ]
    with pytest.raises(ConfigError, match="smooth_post_processor"):
        make(sections)


def test_smooth_override_selects_named_post_processor():
    sections = [
        axis("e", post_processors="pa,st1,st2"),
        post_processor("pa", type="linear_pressure_advance", k="0.04"),
        post_processor("st1", type="smooth_triangle", smooth_time="0.02"),
        post_processor("st2", type="smooth_triangle", smooth_time="0.06"),
    ]
    obj, _, _ = make(sections, options={"smooth_post_processor": "st2"})
    assert obj.smooth_post_processor == "st2"
    assert obj.last_smooth_time == pytest.approx(0.06)


def test_smooth_override_unknown_name_fails():
    with pytest.raises(ConfigError, match="ghost"):
        make(DECLARED_WITH_SMOOTH, options={"smooth_post_processor": "ghost"})


def test_smooth_time_updates_smooth_post_processor():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    gcmd = StubGcmd({"ADVANCE": "0.05", "SMOOTH_TIME": "0.06"})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert ("st", "smooth_time", 0.06) in engine.calls
    assert ("pa", "k", 0.05) in engine.calls
    assert obj.last_smooth_time == pytest.approx(0.06)
    assert obj.last_advance == pytest.approx(0.05)


def test_smooth_time_only_leaves_advance_untouched():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    gcmd = StubGcmd({"SMOOTH_TIME": "0.02"})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert engine.calls == [("st", "smooth_time", 0.02)]
    assert obj.last_advance == pytest.approx(0.04)


def test_smooth_time_zero_disables_smoothing():
    engine = StubEngine()
    obj, gcode, _ = make(DECLARED_WITH_SMOOTH, engine=engine)
    gcmd = StubGcmd({"SMOOTH_TIME": "0"})
    gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
    assert engine.calls == [("st", "smooth_time", 0.0)]
    assert obj.last_smooth_time == pytest.approx(0.0)


def test_engine_error_becomes_command_error():
    obj, gcode, _ = make(DECLARED, engine=RaisingEngine())
    gcmd = StubGcmd({"ADVANCE": "0.1"})
    with pytest.raises(CommandError, match="pa"):
        gcode.commands["SET_PRESSURE_ADVANCE"](gcmd)
