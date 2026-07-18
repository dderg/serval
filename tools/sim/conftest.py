from __future__ import annotations

import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.sim.world import SimWorld  # noqa: E402


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
        beacon: bool = False,
        cartographer: bool = False,
        expect_boot_error: str = None,
        spawn_mcus: bool = True,
        ready_timeout: float = 120.0,
    ) -> SimWorld:
        world = SimWorld(
            tmp_path / f"world{len(worlds)}",
            dual_mcu=dual_mcu,
            beacon=beacon,
            cartographer=cartographer,
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
