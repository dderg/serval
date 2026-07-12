import pytest


class _GCodeError(Exception):
    pass


class _ConfigError(Exception):
    pass


class _FakeGCmd:
    error = _GCodeError

    def __init__(self, params=None):
        self._params = params or {}

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None, above=None, minval=None):
        return float(self._params.get(name, default))

    def get_int(self, name, default=None, minval=None, maxval=None):
        v = self._params.get(name, default)
        return v if v is None else int(v)


class _FakeGCode:
    def create_gcode_command(self, command, commandline, params):
        return _FakeGCmd(params)

    def register_command(self, cmd, func, *args, **kwargs):
        pass


class _FakeToolhead:
    def __init__(self):
        self.moves = []
        self.position = [0.0, 0.0, 10.0, 0.0]

    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

    def get_last_move_time(self):
        return 0.0

    def wait_moves(self):
        pass

    def get_position(self):
        return list(self.position)


class _FakeProbe:
    def __init__(self, offsets=(0.0, 0.0, 1.5), measured_z=1.5):
        self.offsets = offsets
        self.measured_z = measured_z
        self.lifecycle = []
        self.probes = 0

    def get_lift_speed(self, gcmd=None):
        return 4.0

    def get_offsets(self):
        return self.offsets

    def multi_probe_begin(self):
        self.lifecycle.append("begin")

    def multi_probe_end(self):
        self.lifecycle.append("end")

    def run_probe(self, gcmd):
        self.probes += 1
        return [10.0 * self.probes, 20.0, self.measured_z]


class _FakePrinter:
    config_error = _ConfigError

    def __init__(self, objects):
        self.objects = objects

    def lookup_object(self, name, default="__raise__"):
        if name in self.objects:
            return self.objects[name]
        if default == "__raise__":
            raise self.config_error("unknown object %s" % (name,))
        return default


class _FakeConfig:
    def __init__(self, printer, options=None):
        self._printer = printer
        self._options = options or {}

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "fake_section"

    def get(self, name, default="__required__"):
        return self._options.get(
            name, None if default == "__required__" else default
        )

    def getfloat(self, name, default=None, above=None, minval=None):
        v = self._options.get(name, default)
        return v if v is None else float(v)

    def getboolean(self, name, default=None):
        return bool(self._options.get(name, default))

    def getlists(self, name, seps=(",", "\n"), parser=float, count=2):
        raw = self._options[name]
        return [
            tuple(parser(p) for p in line.split(","))
            for line in raw.strip().split("\n")
        ]


def _make_helper(probe_module, finalize, points="10,10\n50,50", options=None):
    toolhead = _FakeToolhead()
    fake_probe = _FakeProbe()
    printer = _FakePrinter(
        {"gcode": _FakeGCode(), "toolhead": toolhead, "probe": fake_probe}
    )
    opts = {"points": points}
    opts.update(options or {})
    config = _FakeConfig(printer, opts)
    helper = probe_module.ProbePointsHelper(config, finalize)
    return helper, toolhead, fake_probe


@pytest.fixture()
def probe_module(monkeypatch):
    from klippy.extras import manual_probe, probe

    monkeypatch.setattr(
        manual_probe, "verify_no_manual_probe", lambda printer: None
    )
    return probe


def test_automatic_probing_collects_all_points(probe_module):
    calls = []

    def finalize(offsets, results):
        calls.append((offsets, [list(r) for r in results]))

    helper, toolhead, fake_probe = _make_helper(probe_module, finalize)
    helper.start_probe(_FakeGCmd())
    assert fake_probe.probes == 2
    assert fake_probe.lifecycle == ["begin", "end"]
    assert len(calls) == 1
    offsets, results = calls[0]
    assert offsets == (0.0, 0.0, 1.5)
    assert results == [[10.0, 20.0, 1.5], [20.0, 20.0, 1.5]]


def test_finalize_retry_reprobes_batch(probe_module):
    outcomes = iter(["retry", None])
    calls = []

    def finalize(offsets, results):
        calls.append(len(results))
        return next(outcomes)

    helper, toolhead, fake_probe = _make_helper(probe_module, finalize)
    helper.start_probe(_FakeGCmd())
    assert calls == [2, 2]
    assert fake_probe.probes == 4


def test_use_offsets_shifts_target_positions(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    helper.use_xy_offsets(True)
    fake_probe.offsets = (24.0, 5.0, 1.5)
    helper.start_probe(_FakeGCmd())
    xy_moves = [m for m, speed in toolhead.moves if m[0] is not None]
    assert xy_moves[0][:2] == [10.0 - 24.0, 10.0 - 5.0]


def test_horizontal_move_z_below_z_offset_rejected(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    fake_probe.offsets = (0.0, 0.0, 6.0)
    with pytest.raises(_GCodeError, match="horizontal_move_z"):
        helper.start_probe(_FakeGCmd())


def test_rapid_scan_method_rejected(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    with pytest.raises(_GCodeError, match="METHOD"):
        helper.start_probe(_FakeGCmd({"METHOD": "rapid_scan"}))


def test_minimum_points_enforced(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    with pytest.raises(_ConfigError):
        helper.minimum_points(3)
    helper.minimum_points(2)


def test_update_probe_points_replaces_and_validates(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    helper.update_probe_points([(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)], 3)
    assert helper.get_probe_points() == [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
    with pytest.raises(_ConfigError):
        helper.update_probe_points([(1.0, 1.0)], 2)


def test_helper_get_lift_speed_uses_probe_value_after_start(probe_module):
    helper, toolhead, fake_probe = _make_helper(probe_module, lambda o, r: None)
    helper.start_probe(_FakeGCmd())
    assert helper.get_lift_speed() == 4.0
