"""One scenario on a fresh SimWorld, judged: the result dict fuzz.py
prints and stores, and the smoke tests assert on."""

from __future__ import annotations

import os
import pathlib
import shutil
import time
import traceback
from typing import Optional

from tools.sim.fuzzing import checks, printer, scenario
from tools.sim.fuzzing.checks import Check, Violation
from tools.sim.fuzzing.scenario import Scenario

PASS, FAIL = "PASS", "FAIL"
WORLD_DIR = "world"
EVIDENCE_TAIL = 3000


def _boot(spec, world) -> str:
    config = printer.render_config(
        spec, world.h7_pty, world.f4_pty, str(world.gcode_dir)
    )
    try:
        world.boot(config)
    except Exception as error:
        raise Violation(Check.BOOT, str(error)[-EVIDENCE_TAIL:]) from error
    return config


def _run_session(chosen: Scenario, world, notes: list) -> tuple:
    from tools.sim.fuzzing.driver import Session
    from tools.sim.world import SimError

    session = None
    try:
        config = _boot(chosen.printer, world)
        session = Session(chosen, world, config)
        session.run()
        return session, None
    except Violation as violation:
        return session, violation
    except SimError as error:
        check = Check.WEDGE if "wedged" in str(error) else Check.HARNESS
        return session, Violation(check, str(error))
    except Exception:
        return session, Violation(
            Check.HARNESS, traceback.format_exc()[-EVIDENCE_TAIL:]
        )
    finally:
        if session is not None:
            notes += session.notes
            session.wall.stop()


def _save_world(world, directory: pathlib.Path) -> None:
    target = directory / WORLD_DIR
    if world.log_dir.exists():
        shutil.copytree(world.log_dir, target / "logs", dirs_exist_ok=True)
    config = world.workdir / "printer.cfg"
    if config.exists():
        shutil.copy2(config, target / config.name)
    for gcode in world.gcode_dir.glob("*.gcode"):
        shutil.copy2(gcode, target / gcode.name)


def run_scenario(
    chosen: Scenario, scratch: pathlib.Path, keep: Optional[pathlib.Path]
) -> dict:
    """Run `chosen` in `scratch`; on a failure copy the world's logs under
    `keep`."""
    from tools.sim.world import SimWorld

    os.environ["RUST_LOG"] = checks.PACER_LOG
    started = time.monotonic()
    notes: list[str] = []
    world = SimWorld(scratch, dual_mcu=chosen.printer.dual_mcu)
    try:
        session, step = _run_session(chosen, world, notes)
        virtual_s = world.vclock.now_ns() / 1e9 if world.vclock else 0.0
        alive = (
            world.klippy_proc is not None and world.klippy_proc.poll() is None
        )
        commanded = session.commanded_shutdowns if session else []
        logged = checks.scan(checks.Logs.read(world.log_dir), commanded, alive)
        violations = checks.ordered(step, logged)
        if violations and keep is not None:
            _save_world(world, keep)
    finally:
        world.shutdown()
    return _result(
        chosen, violations, notes, time.monotonic() - started, virtual_s
    )


def _result(
    chosen: Scenario, violations, notes, real_s: float, virtual_s: float
) -> dict:
    first = violations[0] if violations else None
    return {
        "seed": chosen.seed,
        "status": FAIL if first else PASS,
        "check": first.check.value if first else None,
        "signature": checks.signature(first) if first else None,
        "evidence": first.evidence if first else None,
        "violations": [
            {"check": v.check.value, "evidence": v.evidence} for v in violations
        ],
        "notes": notes,
        "real_s": round(real_s, 2),
        "virtual_s": round(virtual_s, 2),
        "summary": scenario.describe(chosen),
    }
