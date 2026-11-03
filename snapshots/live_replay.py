"""Replay G-code through the live host's own command handlers.

klippy's ``GCodeDispatch``, ``GCodeMove``, ``Motion`` (its ``Move``, velocity
and accel limit commands, dwell and drain) and kinematics ``check_move`` run
unmodified; only the motion engine underneath is replaced, by one that records
every ``submit_move`` / dwell / drain the host issues. The engine's runtime
limit state is the native ``PlannerLimits`` — the same ``RuntimeLimits``
setters and clamp the live ``MotionEngine`` delegates to — so each recorded
move carries the effective limits the live planner would plan it under.

This is the live side of the snapshot path's parity tests: the same G-code
fed to ``_motion_engine.parse_gcode`` must yield the same moves.
"""

from __future__ import annotations

import contextlib
import math
from dataclasses import dataclass
from pathlib import Path

import harness


@dataclass(frozen=True)
class LiveMove:
    delta: tuple[float, float, float, float]
    end: tuple[float, float, float, float]
    feedrate: float
    limits: tuple[float, float, float]


@dataclass(frozen=True)
class LiveRest:
    dwell_s: float


class _Reactor:
    NOW = 0.0
    NEVER = math.inf

    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def mutex(self, is_locked=False):
        return contextlib.nullcontext()

    def register_timer(self, callback, waketime=NEVER):
        return [callback, waketime]

    def update_timer(self, timer, waketime):
        timer[1] = waketime


class _Printer:
    def __init__(self, gcode_module, configfile_module):
        self.command_error = gcode_module.CommandError
        self.config_error = configfile_module.error
        self._reactor = _Reactor()
        self._objects = {}
        self._handlers = {}

    def add_object(self, name, obj):
        self._objects[name] = obj

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def get_reactor(self):
        return self._reactor

    def get_start_args(self):
        return {}

    def register_event_handler(self, event, callback):
        self._handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        return [cb(*params) for cb in self._handlers.get(event, [])]

    def is_shutdown(self):
        return False

    def invoke_shutdown(self, msg):
        raise RuntimeError(msg)


class _Mcu:
    def get_engine_handle(self):
        return 0

    def estimated_print_time(self, eventtime):
        return math.inf


class _Rail:
    def get_range(self):
        return (-math.inf, math.inf)


class RecordingEngine:
    """The motion engine as the live host drives it, minus the planner."""

    def __init__(self, config_text: str):
        engine = harness._import_engine()
        self._limits = engine.PlannerLimits(config_text)
        self.steps: list[LiveMove | LiveRest] = []
        self._position = [0.0, 0.0, 0.0, 0.0]

    def set_velocity_cap(self, velocity):
        self._limits.set_velocity_cap(velocity)

    def set_accel_cap(self, accel):
        self._limits.set_accel_cap(accel)

    def set_corner_deviation(self, corner_deviation):
        self._limits.set_corner_deviation(corner_deviation)

    def effective_limits(self):
        return self._limits.effective_limits()

    def submit_move(self, dx, dy, dz, de, feedrate):
        delta = (dx, dy, dz, de)
        self._position = [p + d for p, d in zip(self._position, delta)]
        self.steps.append(
            LiveMove(
                delta, tuple(self._position), feedrate, self.effective_limits()
            )
        )
        return True

    def submit_dwell(self, duration_s):
        self.steps.append(LiveRest(duration_s))

    def motion_drain_poll(self):
        self.steps.append(LiveRest(0.0))
        return True

    def motion_drain_finalize(self):
        pass

    def fence_start(self, force):
        if force:
            self.steps.append(LiveRest(0.0))
        return 1

    def fence_print_time_poll(self, fence_id, mcu_handle):
        return 0.0

    def frontier_print_time(self, mcu_handle):
        return 0.0

    def set_position(self, x, y, z):
        self._position[:3] = [x, y, z]


def _load_config(cfg_path: Path, printer):
    from klippy import configfile

    loader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
    loader.printer = printer
    return loader.read_config(str(cfg_path))


def _motion(printer, config, engine):
    from klippy import motion as motion_mod
    from klippy import motion_kinematics
    from klippy.kinematics import extruder

    motion = motion_mod.Motion.__new__(motion_mod.Motion)
    motion.printer = printer
    motion.reactor = printer.get_reactor()
    motion.engine = engine
    motion.mcu = _Mcu()
    motion.all_mcus = [motion.mcu]
    motion._cached_engine_mcus = [motion.mcu]
    motion.motion_lead = 0.0
    motion.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    motion._planner_ready = True
    motion._clock_sync_confirmed = True
    motion._engine_wakeup = None
    motion._last_reactor_yield = 0.0
    motion._lookahead_fences = []
    motion._load_motion_config(config)
    motion.extruder = extruder.DummyExtruder(printer)
    motion.extruder.check_move = lambda move: None
    motion._fire_active_callbacks = lambda axes_d: False

    kin = motion_kinematics._LinearKinematics.__new__(
        motion_kinematics._LinearKinematics
    )
    kin._motion = motion
    kin.rails = [_Rail(), _Rail(), _Rail()]
    kin.limits = [rail.get_range() for rail in kin.rails]
    kin._parked_dirty = [False, False, False]
    motion.kin = kin
    return motion


def _register_motion_commands(gcode, motion):
    for name, handler in (
        ("G4", motion.cmd_G4),
        ("M400", motion.cmd_M400),
        ("SET_VELOCITY_LIMIT", motion.cmd_SET_VELOCITY_LIMIT),
        ("RESET_VELOCITY_LIMIT", motion.cmd_RESET_VELOCITY_LIMIT),
        ("M204", motion.cmd_M204),
    ):
        gcode.register_command(name, handler)


def replay(gcode_text: str, cfg_path: Path) -> list[LiveMove | LiveRest]:
    """Every move and rest the live host submits for ``gcode_text``, in order."""
    from klippy import configfile
    from klippy import gcode as gcode_mod
    from klippy.extras import force_move, gcode_move

    printer = _Printer(gcode_mod, configfile)
    config = _load_config(cfg_path, printer)
    engine = RecordingEngine(config.fileconfig.write_string())
    gcode = gcode_mod.GCodeDispatch(printer)
    printer.add_object("gcode", gcode)
    motion = _motion(printer, config, engine)
    printer.add_object("toolhead", motion)
    printer.add_object("motion", motion)
    _register_motion_commands(gcode, motion)
    move_handler = gcode_move.GCodeMove(config)
    printer.add_object("gcode_move", move_handler)
    kinematic_position = force_move.ForceMove.__new__(force_move.ForceMove)
    kinematic_position.printer = printer
    gcode.register_command(
        "SET_KINEMATIC_POSITION",
        kinematic_position.cmd_SET_KINEMATIC_POSITION,
    )
    gcode._handle_ready()
    move_handler._handle_ready()
    gcode.run_script(gcode_text)
    return engine.steps
