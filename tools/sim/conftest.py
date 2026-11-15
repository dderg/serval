from __future__ import annotations

import itertools
import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.sim.world import UNBOUNDED_VTIME_SPEED, SimWorld  # noqa: E402

LONG_TEST_S = 10.0


def _expected_seconds(item) -> float:
    mark = item.get_closest_marker("expected_seconds")
    return mark.args[0] if mark else 0.0


@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    """Longest tests first, each followed by a short one: an xdist worker
    holds the next test while it runs one (run.sh sends one at a time), so
    the second test a long one keeps waiting must not be long too."""
    long = sorted(
        (item for item in items if _expected_seconds(item)),
        key=_expected_seconds,
        reverse=True,
    )
    short = [item for item in items if not _expected_seconds(item)]
    pairs = itertools.zip_longest(long, short[: len(long)])
    paired = [item for pair in pairs for item in pair if item is not None]
    items[:] = paired + short[len(long) :]


def pytest_terminal_summary(terminalreporter):
    for reports in terminalreporter.stats.values():
        for rep in reports:
            if getattr(rep, "when", None) != "call":
                continue
            if (
                rep.duration <= LONG_TEST_S
                or "expected_seconds" in rep.keywords
            ):
                continue
            terminalreporter.write_line(
                f"hint: {rep.nodeid} ran {rep.duration:.0f} s without "
                "@pytest.mark.expected_seconds; mark it so xdist starts it first"
            )


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)


@pytest.fixture
def sim_world(request, tmp_path):
    """Factory: boot a simulated printer from generated config text.

    Usage:
        world = sim_world(lambda w: configs.minimal_config(
            w.h7_pty, str(w.gcode_dir)))

    The config callback receives the (not yet booted) SimWorld so it can
    reference the PTY paths and gcode dir. On test failure the fixture
    dumps klippy/MCU/event logs before shutting everything down.
    """
    worlds: list[SimWorld] = []

    def factory(
        config_fn,
        *,
        dual_mcu: bool = True,
        sc_mcu: bool = False,
        beacon: bool = False,
        cartographer: bool = False,
        expect_boot_error: str = None,
        spawn_mcus: bool = True,
        ready_timeout: float = 120.0,
        vtime_speed: float = UNBOUNDED_VTIME_SPEED,
        host_gated: bool = True,
        crystal_ppm: dict = None,
    ) -> SimWorld:
        world = SimWorld(
            tmp_path / f"world{len(worlds)}",
            dual_mcu=dual_mcu,
            sc_mcu=sc_mcu,
            beacon=beacon,
            cartographer=cartographer,
            vtime_speed=vtime_speed,
            host_gated=host_gated,
            crystal_ppm=crystal_ppm,
        )
        worlds.append(world)
        world.boot(
            config_fn(world),
            expect_boot_error=expect_boot_error,
            spawn_mcus=spawn_mcus,
            ready_timeout=ready_timeout,
        )
        return world

    yield factory

    rep = getattr(request.node, "rep_call", None)
    for world in worlds:
        if rep is not None and rep.failed:
            world.dump_diagnostics()
        world.shutdown()
