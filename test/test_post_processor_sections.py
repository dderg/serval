import pytest

from klippy.motion import Motion


class ConfigError(Exception):
    pass


class StubSection:
    def __init__(self, name, options):
        self.name = name
        self.options = options
        self.error = ConfigError

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

    def get_prefix_options(self, prefix):
        return [o for o in self.options if o.startswith(prefix)]


class StubConfig:
    error = ConfigError

    def __init__(self, sections):
        self.sections = {s.get_name(): s for s in sections}

    def has_section(self, name):
        return name in self.sections

    def get_prefix_sections(self, prefix):
        return [
            s for n, s in sorted(self.sections.items()) if n.startswith(prefix)
        ]


def axis(name, **opts):
    return StubSection("axis " + name, opts)


def post_processor(name, **opts):
    return StubSection("post_processor " + name, opts)


def make_toolhead():
    th = Motion.__new__(Motion)
    th.limit_sections = []
    return th


SPATIAL = [axis("x"), axis("y"), axis("z")]


def test_input_shaper_section_rejected_pointing_at_post_processor():
    th = make_toolhead()
    cfg = StubConfig(SPATIAL + [StubSection("input_shaper", {})])
    with pytest.raises(ConfigError, match=r"\[post_processor"):
        th._read_axes(cfg)


def test_post_processor_missing_type_fails():
    th = make_toolhead()
    cfg = StubConfig(SPATIAL + [post_processor("is", frequency_hz=50.0)])
    th._read_axes(cfg)
    with pytest.raises(ConfigError, match="type"):
        th._read_post_processors(cfg)


def test_axis_referencing_undeclared_post_processor_fails():
    th = make_toolhead()
    cfg = StubConfig(
        SPATIAL + [axis("e", follows="x,y,z", post_processors="pa")]
    )
    th._read_axes(cfg)
    with pytest.raises(ConfigError, match="pa"):
        th._read_post_processors(cfg)


def test_happy_path_parses_sections_for_init_planner():
    th = make_toolhead()
    cfg = StubConfig(
        SPATIAL
        + [
            axis("e", follows="x,y,z", post_processors="is,pa"),
            post_processor("is", type="smooth_bell", smooth_time="0.0182"),
            post_processor("pa", type="linear_pressure_advance", k="0.04"),
        ]
    )
    th._read_axes(cfg)
    th._read_post_processors(cfg)
    assert ("e", ["x", "y", "z"], [], ["is", "pa"]) in th.axis_sections
    assert ("is", "smooth_bell", [("smooth_time", 0.0182)]) in (
        th.post_processor_sections
    )
    assert ("pa", "linear_pressure_advance", [("k", 0.04)]) in (
        th.post_processor_sections
    )


def test_mode_inverse_section_parses_both_params():
    th = make_toolhead()
    cfg = StubConfig(
        SPATIAL
        + [
            axis("x", post_processors="slew,belt"),
            post_processor("slew", type="smooth_bell", smooth_time="0.0015"),
            post_processor(
                "belt",
                type="mode_inverse",
                frequency_hz="131.0",
                damping_ratio="0.05",
            ),
        ]
    )
    th._read_axes(cfg)
    th._read_post_processors(cfg)
    belt = next(s for s in th.post_processor_sections if s[0] == "belt")
    assert belt[1] == "mode_inverse"
    assert sorted(belt[2]) == [("damping_ratio", 0.05), ("frequency_hz", 131.0)]


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
