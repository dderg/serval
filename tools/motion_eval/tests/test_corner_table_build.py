"""The corner table is built off the reactor thread, cached on disk, and
never used before it is complete."""

import os
import threading

import pytest

from klippy.extras import corner_deviation as cd
from klippy.extras import shaper_defs

OWNER_SPECS = (
    ("smoother",)
    + tuple(shaper_defs.get_mzv_smoother(200.0, normalize_coeffs=False)),
    ("smoother",)
    + tuple(shaper_defs.get_mzv_smoother(130.0, normalize_coeffs=False)),
)
BENCH_SPECS = tuple(
    ("pulses",) + tuple(shaper_defs.get_mzv_shaper(f, 0.1))
    for f in (50.0, 44.0)
)


class _Printer:
    def __init__(self):
        self.shutdowns = []

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


class _Move:
    def __init__(self, x, y):
        self.axes_r = (x, y, 0.0, 0.0)
        self.move_d = 10.0
        self.accel = 5000.0


def _limiter(cache_dir):
    limiter = object.__new__(cd.CornerDeviation)
    limiter.printer = _Printer()
    limiter.cache_dir = str(cache_dir)
    limiter.max_deviation = None
    limiter.limit = None
    limiter.building = False
    limiter.generation = 0
    limiter.reported_inactive = False
    limiter.ready = True
    limiter.holding_gcode = False
    limiter.gcode_mutex = _Mutex()
    limiter.counters = dict.fromkeys(
        ("limited", "stopped_by_cap", "step_limited", "xy_stops"), 0
    )
    return limiter


class _Mutex:
    def __init__(self):
        self.locked = False

    def lock(self):
        assert not self.locked
        self.locked = True

    def unlock(self):
        assert self.locked
        self.locked = False


def _equal(a, b):
    return a.keys() == b.keys() and all(
        a[k].tobytes() == b[k].tobytes() and a[k].shape == b[k].shape for k in a
    )


@pytest.mark.parametrize(
    "specs", [OWNER_SPECS, BENCH_SPECS], ids=["owner", "bench"]
)
def test_cached_table_is_the_built_table(tmp_path, specs):
    limiter = _limiter(tmp_path)
    kx, ky = (cd.kernel_from_spec(s) for s in specs)
    built = limiter._table_arrays(specs, kx, ky)
    (cached_file,) = os.listdir(tmp_path)
    loaded = limiter._table_arrays(specs, kx, ky)
    assert os.listdir(tmp_path) == [cached_file]
    assert _equal(loaded, built)
    assert _equal(built, cd.build_table_arrays(kx, ky))


def test_cache_key_covers_the_shaper(tmp_path):
    limiter = _limiter(tmp_path)
    for specs in (OWNER_SPECS, BENCH_SPECS):
        kx, ky = (cd.kernel_from_spec(s) for s in specs)
        limiter._table_arrays(specs, kx, ky)
    assert len(os.listdir(tmp_path)) == 2


def test_a_junction_planned_while_the_table_builds_shuts_down(tmp_path):
    limiter = _limiter(tmp_path)
    limiter.building = True
    assert limiter.junction_v2(_Move(1.0, 0.0), _Move(0.0, 1.0), 100.0) == 0.0
    assert limiter.printer.shutdowns == [
        "corner_deviation: junction planned while the corner table is building"
    ]
    assert not limiter.get_status(0.0)["table_ready"]


def _bench_limit():
    kx, ky = (cd.kernel_from_spec(s) for s in BENCH_SPECS)
    return cd.JunctionLimit(cd.build_table_arrays(kx, ky), kx, ky, None)


def test_only_the_latest_build_is_published(tmp_path):
    limiter = _limiter(tmp_path)
    limit = _bench_limit()
    limiter.generation = 2
    limiter.building = True
    limiter._publish(1, (limit, None))
    assert limiter.limit is None and limiter.building
    limiter._publish(2, (limit, None))
    assert limiter.limit is limit and not limiter.building


def test_counters_survive_a_shaper_change(tmp_path):
    limiter = _limiter(tmp_path)
    limiter.toolhead = type("Toolhead", (), {"max_accel": 5000.0})()
    limiter.generation = 1
    limiter._publish(1, (_bench_limit(), None))
    assert limiter.junction_v2(_Move(1.0, 0.0), _Move(0.0, 1.0), 1e6) < 1e6
    limiter._retire_limit()
    limiter.generation = 2
    limiter._publish(2, (_bench_limit(), None))
    assert limiter.junction_v2(_Move(1.0, 0.0), _Move(0.0, 1.0), 1e6) < 1e6
    assert limiter.get_status(0.0)["limited"] == 2


def test_a_failed_build_shuts_down(tmp_path):
    limiter = _limiter(tmp_path)
    limiter.generation = 1
    limiter.building = True
    limiter._publish(1, (None, ValueError("bad shaper")))
    assert limiter.limit is None and not limiter.building
    assert limiter.printer.shutdowns == [
        "corner_deviation: table build failed: bad shaper"
    ]


class _Reactor:
    def __init__(self):
        self.callbacks = []
        self.pauses = 0

    def monotonic(self):
        return 0.0

    def register_async_callback(self, callback):
        self.callbacks.append(callback)

    def pause(self, waketime):
        self.pauses += 1
        while self.callbacks:
            self.callbacks.pop(0)(waketime)
        return waketime


class _Shaper:
    def __init__(self, axis, spec):
        self.axis, self.spec = axis, spec

    def get_axis(self):
        return self.axis


class _InputShaper:
    def get_shapers(self):
        return [_Shaper(a, s) for a, s in zip("xy", BENCH_SPECS)]


def test_the_build_runs_off_the_reactor_thread(tmp_path, monkeypatch):
    threads = []
    real_build = cd.build_table_arrays

    def build(kx, ky):
        threads.append(threading.current_thread())
        return real_build(kx, ky)

    monkeypatch.setattr(cd, "build_table_arrays", build)
    monkeypatch.setattr(cd, "shaper_spec", lambda shaper: shaper.spec)
    limiter = _limiter(tmp_path)
    limiter.reactor = _Reactor()
    limiter.input_shaper = _InputShaper()
    limiter._refresh_kernels()
    assert threads and threading.main_thread() not in threads
    assert limiter.limit is not None and not limiter.building
    assert limiter.reactor.pauses >= 1


def test_gcode_waits_for_the_startup_table(tmp_path, monkeypatch):
    monkeypatch.setattr(cd, "shaper_spec", lambda shaper: shaper.spec)
    limiter = _limiter(tmp_path)
    limiter.ready = False
    limiter.reactor = _Reactor()
    limiter.input_shaper = _InputShaper()
    limiter._refresh_kernels()
    assert limiter.gcode_mutex.locked and limiter.reactor.pauses == 0
    limiter._refresh_kernels()
    assert limiter.gcode_mutex.locked
    while limiter.building:
        limiter.reactor.pause(0.0)
    assert limiter.limit is not None and not limiter.gcode_mutex.locked
