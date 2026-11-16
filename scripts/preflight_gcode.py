"""The printer's own G-code front end, for scripts/preflight.py.

klippy reads the printer's config (its [include]s and SAVE_CONFIG block) and
dispatches every line of the print through its own G-code dispatch, with the
config's [gcode_macro]s and [bed_mesh] loaded. Commands that would reach the
motion planner are collected, each with the print line and macro chain it
came from, and sorted by how preflight treats them (`Treatment`).
"""

from __future__ import annotations

import contextlib
import enum
import logging
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from klippy import configfile
from klippy import gcode as gcode_module
from klippy.extras import bed_mesh, danger_options, gcode_macro, gcode_move

BED_MESH_SECTION = "bed_mesh"
DEFAULT_MESH_PROFILE = "default"
START_LINE = 0


class Treatment(enum.Enum):
    PLANNED = "planned"
    NO_MOTION = "not planned: no motion"
    HEATER_WAIT = "planned as M400: the toolhead stops while heating, for a time preflight cannot know"
    HOMING = "dropped: homing or probing motion preflight cannot know; the next move starts from the last commanded position"
    MOTOR_DRIVE = "dropped: drives motors outside the toolhead path"
    PLANNER_SETUP = (
        "dropped: changes the planner setup; the plan keeps the config's"
    )
    MESH = "dropped: bed mesh command without a [bed_mesh] section"
    PRINT_FLOW = "dropped: pause, resume, cancel or object exclusion"
    STOPS_PRINT = "the live host stops the print on it: the printer never runs this motion"
    CURVE = "the live host runs it, but preflight cannot plan curves"


PLANNED = (
    "G0",
    "G1",
    "G4",
    "G20",
    "G90",
    "G91",
    "G92",
    "M18",
    "M82",
    "M83",
    "M84",
    "M204",
    "M220",
    "M221",
    "M400",
    "SET_VELOCITY_LIMIT",
    "RESET_VELOCITY_LIMIT",
    "SET_KINEMATIC_POSITION",
    "SET_GCODE_OFFSET",
    "SAVE_GCODE_STATE",
    "RESTORE_GCODE_STATE",
)
HEATER_WAITS = ("M109", "M190", "TEMPERATURE_WAIT")
HOMING = (
    "G28",
    "PROBE",
    "PROBE_ACCURACY",
    "PROBE_CALIBRATE",
    "MANUAL_PROBE",
    "ACCEPT",
    "ABORT",
    "ADJUSTED",
    "NEXT",
    "TESTZ",
    "QUAD_GANTRY_LEVEL",
    "Z_TILT_ADJUST",
    "Z_TILT_AUTODETECT",
    "Z_TILT_CALIBRATE",
    "BED_MESH_CALIBRATE",
    "BED_TILT_CALIBRATE",
    "BED_SCREWS_ADJUST",
    "SCREWS_TILT_CALCULATE",
    "DELTA_CALIBRATE",
    "DELTA_ANALYZE",
    "Z_ENDSTOP_CALIBRATE",
    "AXIS_TWIST_COMPENSATION_CALIBRATE",
    "ATTACH_PROBE",
    "DETACH_PROBE",
    "MOVE_AVOIDING_DOCK",
    "MOVE_TO_APPROACH_PROBE",
    "MOVE_TO_DETACH_PROBE",
    "MOVE_TO_DOCK_PROBE",
    "MOVE_TO_EXTRACT_PROBE",
    "MOVE_TO_INSERT_PROBE",
    "NOZZLE_CLEANUP",
    "TOOL_LOCATE_SENSOR",
    "TOOL_CALIBRATE_TOOL_OFFSET",
)
MOTOR_DRIVES = (
    "FORCE_MOVE",
    "MANUAL_STEPPER",
    "SET_STEPPER_ENABLE",
    "STEPPER_BUZZ",
    "DIAG_STEPPER_BUZZ",
    "RESONANCE_BUZZ",
    "RESONANCE_BUZZ_SWEEP",
    "TEST_RESONANCES",
    "SHAPER_CALIBRATE",
)
PLANNER_SETUP = (
    "SET_PRESSURE_ADVANCE",
    "SET_SMOOTH_TIME",
    "SET_POST_PROCESSOR",
    "SET_INPUT_SHAPER",
    "SET_SKEW",
    "SKEW_PROFILE",
    "SET_Z_THERMAL_ADJUST",
    "ACTIVATE_EXTRUDER",
    "SYNC_EXTRUDER_MOTION",
    "SET_EXTRUDER_ROTATION_DISTANCE",
    "SET_MIXING_EXTRUDER",
    "SET_MIXING_EXTRUDER_GRADIENT",
    "TUNING_TOWER",
)
MESH_COMMANDS = ("BED_MESH_PROFILE", "BED_MESH_CLEAR", "BED_MESH_OFFSET")
PRINT_FLOW = ("PAUSE", "RESUME", "CANCEL_PRINT", "EXCLUDE_OBJECT")
STOPS_PRINT = tuple(gcode_move.UNSUPPORTED_MOTION)
CURVES = ("G5",)

TREATMENTS = {
    **dict.fromkeys(PLANNED, Treatment.PLANNED),
    **dict.fromkeys(HEATER_WAITS, Treatment.HEATER_WAIT),
    **dict.fromkeys(HOMING, Treatment.HOMING),
    **dict.fromkeys(MOTOR_DRIVES, Treatment.MOTOR_DRIVE),
    **dict.fromkeys(PLANNER_SETUP, Treatment.PLANNER_SETUP),
    **dict.fromkeys(MESH_COMMANDS, Treatment.MESH),
    **dict.fromkeys(PRINT_FLOW, Treatment.PRINT_FLOW),
    **dict.fromkeys(STOPS_PRINT, Treatment.STOPS_PRINT),
    **dict.fromkeys(CURVES, Treatment.CURVE),
}
HEATER_WAIT_REST = "M400"
KEPT_HANDLERS = frozenset({*MESH_COMMANDS, "BED_MESH_CHECK"})
KLIPPY_LOG = logging.NullHandler()


def treatment(command: str) -> Treatment:
    return TREATMENTS.get(command, Treatment.NO_MOTION)


@dataclass(frozen=True)
class Origin:
    """Line `line` of the print (0: before it), through `macros`, outermost
    first."""

    line: int
    macros: tuple[str, ...] = ()

    def describe(self) -> str:
        where = f"line {self.line}" if self.line else "before the print"
        if not self.macros:
            return where
        return f"{where} via {' > '.join(self.macros)}"


@dataclass(frozen=True)
class Command:
    name: str
    text: str
    origin: Origin


@dataclass(frozen=True)
class SkippedMacro:
    origin: Origin
    reason: str


@dataclass(frozen=True)
class MeshChange:
    """A bed mesh activation (`profile` and the request klippy's bed_mesh
    hands the engine's `set_bed_mesh`) or, with `profile` None, a clear."""

    origin: Origin
    profile: str | None
    request: dict | None


class MacroSkipped(Exception):
    def __init__(self, macros: tuple[str, ...], reason: str):
        super().__init__(reason)
        self.macros = macros
        self.reason = reason


class HostError(Exception):
    """klippy raises at `origin` and the print stops there."""

    def __init__(self, origin: Origin, message: str):
        super().__init__(f"{origin.describe()}: {message}")
        self.origin = origin
        self.message = message


class _Reactor:
    def monotonic(self):
        return 0.0

    def mutex(self, is_locked=False):
        return contextlib.nullcontext()


class _Printer:
    """The klippy printer as its G-code front end uses it."""

    command_error = gcode_module.CommandError
    config_error = configfile.error

    def __init__(self, config_path: Path):
        self._start_args = {"config_file": str(config_path)}
        self._objects: dict[str, object] = {}
        self._handlers: dict[str, list] = {}
        self._reactor = _Reactor()

    def add_object(self, name, obj):
        self._objects[name] = obj

    def lookup_object(self, name, default=configfile.sentinel):
        if name in self._objects:
            return self._objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default

    def lookup_objects(self, module=None):
        if module is None:
            return list(self._objects.items())
        prefix = module + " "
        return [
            (n, o) for n, o in self._objects.items() if n.startswith(prefix)
        ]

    def load_object(self, config, section, default=configfile.sentinel):
        if section in self._objects:
            return self._objects[section]
        raise self.config_error(f"preflight does not load [{section}]")

    def get_reactor(self):
        return self._reactor

    def get_start_args(self):
        return self._start_args

    def register_event_handler(self, event, callback):
        self._handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        return [callback(*params) for callback in self._handlers.get(event, [])]

    def get_state_message(self):
        return "Printer is ready", "ready"

    def is_shutdown(self):
        return False

    def invoke_shutdown(self, msg):
        pass

    def request_exit(self, result):
        pass


class _Toolhead:
    def rebase_gcode_z(self, z):
        pass


class _MeshRequests:
    """The engine's bed mesh entry points as klippy's bed_mesh calls them:
    the last activation's request, for the replay to activate itself."""

    NO_Z_BUDGET_EXCESS = (0.0, 0.0, math.inf, math.inf, 0.0)

    def __init__(self):
        self.last: dict | None = None

    def set_bed_mesh(self, request):
        self.last = dict(request)
        return 0.0, self.NO_Z_BUDGET_EXCESS

    def clear_bed_mesh(self):
        self.last = None
        return 0.0


class _ModelledStatus:
    """`printer` inside a macro: the status of the objects preflight runs
    (macros, bed_mesh); reading any other skips the macro instead of
    letting Jinja render it as undefined."""

    def __init__(self, front: FrontEnd):
        self._front = front

    def __getitem__(self, name):
        found = self._front.printer.lookup_object(str(name).strip(), None)
        if found is None or not hasattr(found, "get_status"):
            raise MacroSkipped(
                self._front.macros(),
                f"reads printer[{str(name)!r}], which preflight does not run",
            )
        return found.get_status(0.0)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return self[name]

    def __contains__(self, name):
        self[name]
        return True

    def __iter__(self):
        raise MacroSkipped(
            self._front.macros(),
            "iterates printer objects preflight does not run",
        )


class _ExpandedMacro(gcode_macro.GCodeMacro):
    """klippy's GCodeMacro whose evaluation failures skip the macro, and
    which keeps the front end's macro chain."""

    def __init__(self, config, front: FrontEnd):
        super().__init__(config)
        self._front = front
        function = self.template.function
        if isinstance(function, gcode_macro.TemplateWrapperJinja):
            function.render = self._guarded(function.render)

    def _guarded(self, render):
        def guarded(context=None):
            try:
                return render(context)
            except self.gcode.error as error:
                cause = error.__context__
                if isinstance(cause, (MacroSkipped, HostError)):
                    raise cause from None
                raise MacroSkipped(self._front.macros(), str(error)) from None

        return guarded

    def cmd(self, gcmd):
        with self._front.inside(self.alias):
            if not isinstance(
                self.template.function, gcode_macro.TemplateWrapperJinja
            ):
                raise MacroSkipped(
                    self._front.macros(),
                    "a Python macro: preflight evaluates Jinja macros only",
                )
            try:
                super().cmd(gcmd)
            except self.gcode.error as error:
                raise HostError(self._front.origin(), str(error)) from None


class FrontEnd:
    """klippy's G-code dispatch over the printer's macros and bed mesh,
    collecting what reaches the motion planner."""

    def __init__(self, config_path: Path):
        logging.getLogger().addHandler(KLIPPY_LOG)
        self.printer = _Printer(config_path)
        self.config = _read_config(self.printer)
        danger_options.load_config(self.config.getsection("danger_options"))
        self._gcode = gcode_module.GCodeDispatch(self.printer)
        self.printer.add_object("gcode", self._gcode)
        self.printer.add_object("toolhead", _Toolhead())
        self._mesh_requests = _MeshRequests()
        self.printer.add_object("motion_engine", self._mesh_requests)
        self._line = START_LINE
        self._macros: list[str] = []
        self.commands: list[Command] = []
        self.skipped: list[SkippedMacro] = []
        self.mesh_changes: list[MeshChange] = []
        self._load_macro_templates()
        self._bed_mesh = self._load_bed_mesh()
        self._record_builtins()
        self._load_macros()
        self.printer.send_event("klippy:connect")
        self._gcode._handle_ready()
        self._gcode.cmd_default = self._record

    def config_text(self) -> str:
        return self.config.fileconfig.write_string()

    def macros(self) -> tuple[str, ...]:
        return tuple(self._macros)

    def origin(self) -> Origin:
        return Origin(self._line, self.macros())

    @contextlib.contextmanager
    def inside(self, macro: str):
        self._macros.append(macro)
        try:
            yield
        finally:
            self._macros.pop()

    def expand(self, text: str) -> None:
        for number, line in enumerate(text.splitlines(), 1):
            self._line = number
            mark = len(self.commands)
            try:
                self._gcode.run_script_from_command(line)
            except MacroSkipped as skip:
                del self.commands[mark:]
                self.skipped.append(
                    SkippedMacro(Origin(number, skip.macros), skip.reason)
                )
            except self._gcode.error as error:
                raise HostError(Origin(number), str(error)) from None
        self._line = START_LINE

    def mesh_profiles(self) -> list[str]:
        if self._bed_mesh is None:
            return []
        return sorted(self._bed_mesh.pmgr.get_profiles())

    def load_mesh_profile(self, name: str) -> MeshChange:
        if self._bed_mesh is None:
            raise HostError(self.origin(), "the config has no [bed_mesh]")
        try:
            self._bed_mesh.pmgr.load_profile(name)
        except self._gcode.error as error:
            raise HostError(self.origin(), str(error)) from None
        return self.mesh_changes[-1]

    def _record(self, gcmd):
        name = gcmd.get_command()
        if name:
            self.commands.append(
                Command(name, gcmd.get_commandline(), self.origin())
            )

    def _recorder(self, name: str):
        def record(gcmd):
            text = f"{name} {gcmd.get_raw_command_parameters()}".strip()
            self.commands.append(Command(name, text, self.origin()))

        return record

    def _record_builtins(self):
        for name in [*self._gcode.ready_gcode_handlers]:
            if name in KEPT_HANDLERS:
                continue
            self._gcode.register_command(name, None)
            self._gcode.register_command(name, self._recorder(name))

    def _load_macro_templates(self):
        templates = gcode_macro.PrinterGCodeMacro(self.config)
        default_context = templates.create_template_context

        def context(eventtime=None):
            values = default_context(eventtime)
            values["printer"] = _ModelledStatus(self)
            values["action_raise_error"] = self._raise_host_error
            values["action_emergency_stop"] = self._raise_host_error
            values["action_call_remote_method"] = _ignore_remote_call
            return values

        templates.create_template_context = context
        self.printer.add_object("gcode_macro", templates)

    def _raise_host_error(self, msg="action_emergency_stop"):
        raise HostError(self.origin(), msg)

    def _load_bed_mesh(self):
        if not self.config.has_section(BED_MESH_SECTION):
            return None
        mesh = bed_mesh.BedMesh(self.config.getsection(BED_MESH_SECTION))
        apply_mesh = mesh.set_mesh

        def set_mesh(z_mesh):
            apply_mesh(z_mesh)
            profile = None if z_mesh is None else z_mesh.get_profile_name()
            request = None if z_mesh is None else self._mesh_requests.last
            self.mesh_changes.append(
                MeshChange(self.origin(), profile, request)
            )

        mesh.set_mesh = set_mesh
        self.printer.add_object(BED_MESH_SECTION, mesh)
        return mesh

    def _load_macros(self):
        sections = self.config.get_prefix_sections("gcode_macro ")
        for section in sections:
            renamed = section.get("rename_existing", None)
            alias = section.get_name().split()[1].upper()
            if renamed and alias not in self._gcode.ready_gcode_handlers:
                self._gcode.register_command(alias, self._recorder(alias))
        for section in sections:
            self.printer.add_object(
                section.get_name(), _ExpandedMacro(section, self)
            )


def _ignore_remote_call(method, **kwargs):
    return ""


def _read_config(printer: _Printer):
    """The config klippy runs: [include]s resolved, the SAVE_CONFIG block
    merged (`PrinterConfig.read_main_config`)."""
    reader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
    reader.printer = printer
    return reader.read_main_config()


@dataclass(frozen=True)
class Tally:
    count: int
    first: Origin


@dataclass
class Replay:
    """What the replay plans: the planned commands, each with its origin,
    and every command name preflight saw with its count and first origin."""

    lines: list[str]
    origins: list[Origin]
    tallies: dict[str, Tally]

    @classmethod
    def of(cls, commands: list[Command]) -> Replay:
        lines: list[str] = []
        origins: list[Origin] = []
        counts: Counter[str] = Counter()
        firsts: dict[str, Origin] = {}
        for command in commands:
            counts[command.name] += 1
            firsts.setdefault(command.name, command.origin)
            kind = treatment(command.name)
            if kind is Treatment.PLANNED:
                lines.append(command.text)
                origins.append(command.origin)
            elif kind is Treatment.HEATER_WAIT:
                lines.append(HEATER_WAIT_REST)
                origins.append(command.origin)
        tallies = {name: Tally(counts[name], firsts[name]) for name in counts}
        return cls(lines, origins, tallies)

    def text(self) -> str:
        return "\n".join(self.lines) + "\n"

    def origin_of(self, replay_line: int | None) -> Origin | None:
        if replay_line is None or not 1 <= replay_line <= len(self.origins):
            return None
        return self.origins[replay_line - 1]

    def by_treatment(self, kind: Treatment) -> dict[str, Tally]:
        return {n: t for n, t in self.tallies.items() if treatment(n) is kind}
