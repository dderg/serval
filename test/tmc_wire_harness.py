"""Wire-level test harness for the TMC driver stack.

Fakes only the hardware seam (register reads/writes, raw MCU commands,
reactor timers) and records every interaction in ONE ordered log, so tests
can assert the exact cross-object sequencing that the TMC code depends on:
register write order, error-check suppression windows, and MCU command
interleaving. Everything above the seam — FieldHelper, TMCErrorCheck,
TMCCommandHelper, TMCVirtualPinHelper, TMC5160 — runs real.

Log entry shapes:
    ("write", reg_name, value, print_time)   mcu_tmc.set_register
    ("read", reg_name)                       mcu_tmc.get_register
    ("cmd", command_name, args_tuple)        raw MCU command send
    ("query", command_name, args_tuple)      raw MCU query send
    ("timer+", callback_name)                reactor timer registered
    ("timer-", callback_name)                reactor timer unregistered
"""


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


_REQUIRED = object()


class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0

    def __init__(self, wire_log):
        self._log = wire_log
        self.time = 100.0
        self.timers = []
        self.callbacks = []

    def monotonic(self):
        return self.time

    def pause(self, waketime):
        self.time = max(self.time, waketime)

    def register_timer(self, callback, waketime=NEVER):
        timer = [callback, waketime]
        self.timers.append(timer)
        self._log.append(("timer+", callback.__name__))
        return timer

    def unregister_timer(self, timer):
        self.timers.remove(timer)
        self._log.append(("timer-", timer[0].__name__))

    def update_timer(self, timer, waketime):
        timer[1] = waketime

    def register_callback(self, callback, waketime=NOW):
        self.callbacks.append(callback)

    def run_callbacks(self):
        callbacks, self.callbacks = self.callbacks, []
        for cb in callbacks:
            cb(self.time)


class FakePrinter:
    command_error = CommandError
    config_error = ConfigError

    def __init__(self, wire_log):
        self.reactor = FakeReactor(wire_log)
        self.objects = {}
        self.event_handlers = {}
        self.shutdowns = []

    def get_reactor(self):
        return self.reactor

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=_REQUIRED):
        if name in self.objects:
            return self.objects[name]
        if default is not _REQUIRED:
            return default
        raise ConfigError("test harness has no printer object %r" % (name,))

    def load_object(self, config, name):
        return self.lookup_object(name)

    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)

    def fire_event(self, event):
        for handler in self.event_handlers.get(event, []):
            handler()

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

    def get_start_args(self):
        return {}


class FakeConfig:
    error = ConfigError

    def __init__(self, name, options, printer, sections):
        self._name = name
        self._options = dict(options)
        self._printer = printer
        self._sections = sections
        sections[name] = self

    def get_name(self):
        return self._name

    def get_printer(self):
        return self._printer

    def has_section(self, name):
        return name in self._sections

    def getsection(self, name):
        if name not in self._sections:
            raise ConfigError("no config section [%s]" % (name,))
        return self._sections[name]

    def _fetch(self, key, default):
        if key in self._options:
            return self._options[key]
        if default is _REQUIRED:
            raise ConfigError(
                "missing required option %r in [%s]" % (key, self._name)
            )
        return default

    def get(self, key, default=_REQUIRED, *_args, **_kwargs):
        return self._fetch(key, default)

    def getint(self, key, default=_REQUIRED, *_args, **_kwargs):
        value = self._fetch(key, default)
        return None if value is None else int(value)

    def getfloat(self, key, default=_REQUIRED, *_args, **_kwargs):
        value = self._fetch(key, default)
        return None if value is None else float(value)

    def getboolean(self, key, default=_REQUIRED, *_args, **_kwargs):
        value = self._fetch(key, default)
        return None if value is None else bool(value)

    def getchoice(self, key, choices, default=_REQUIRED):
        value = self._fetch(key, default)
        if value not in choices and all(isinstance(c, int) for c in choices):
            value = int(value)
        if value not in choices:
            raise ConfigError(
                "option %r in [%s]: %r is not a valid choice"
                % (key, self._name, value)
            )
        return choices[value]


class FakeCommand:
    def __init__(self, name, wire_log):
        self._name = name
        self._log = wire_log

    def send(self, args=()):
        self._log.append(("cmd", self._name, tuple(args)))


class FakeQueryCommand:
    def __init__(self, name, oid, mcu, wire_log):
        self._name = name
        self._oid = oid
        self._mcu = mcu
        self._log = wire_log

    def send(self, args=()):
        self._log.append(("query", self._name, tuple(args)))
        return self._mcu.next_query_response(self._name, self._oid)


class FakeMCU:
    """The raw-command seam: lookup_command / lookup_query_command."""

    def __init__(self, wire_log):
        self._log = wire_log
        self.non_critical_disconnected = False
        self._query_scripts = {}

    def get_name(self):
        return "mcu"

    def lookup_command(self, msgformat):
        return FakeCommand(msgformat.split()[0], self._log)

    def lookup_query_command(self, msgformat, respformat, oid=None):
        return FakeQueryCommand(msgformat.split()[0], oid, self, self._log)

    def script_query(self, name, responses, oid=None):
        """Queue responses for a query command; the last one repeats."""
        self._query_scripts[(name, oid)] = list(responses)

    def next_query_response(self, name, oid):
        for key in ((name, oid), (name, None)):
            if key in self._query_scripts:
                responses = self._query_scripts[key]
                if len(responses) > 1:
                    return responses.pop(0)
                return responses[0]
        raise AssertionError(
            "no scripted response for query %r (oid=%r)" % (name, oid)
        )


class _FakeSPIPin:
    def __init__(self, mcu):
        self._mcu = mcu
        self.oid = 5

    def get_mcu(self):
        return self._mcu


class _FakeTMCSPI:
    def __init__(self, mcu):
        self.spi = _FakeSPIPin(mcu)

    def get_bus_and_cs_ids(self):
        return (2, 7)


class FakeMcuTmc:
    """The register seam: set_register logs, get_register serves a script.

    Scripted reads: ``reads[reg]`` is a value, an exception instance, or a
    list of those (consumed in order, last entry repeats). Unscripted
    registers read as 0.
    """

    def __init__(self, fields, wire_log, mcu=None, frequency=12000000.0):
        self._fields = fields
        self._log = wire_log
        self._frequency = frequency
        self.mcu = mcu if mcu is not None else FakeMCU(wire_log)
        self.tmc_spi = _FakeTMCSPI(self.mcu)
        self.reads = {}

    def get_fields(self):
        return self._fields

    def get_tmc_frequency(self):
        return self._frequency

    def set_register(self, reg_name, value, print_time=None):
        self._log.append(("write", reg_name, value, print_time))

    def get_register(self, reg_name):
        self._log.append(("read", reg_name))
        scripted = self.reads.get(reg_name, 0)
        if isinstance(scripted, list):
            value = scripted.pop(0) if len(scripted) > 1 else scripted[0]
        else:
            value = scripted
        if isinstance(value, Exception):
            raise value
        return value


class FakeGCode:
    def __init__(self):
        self.mux_commands = []

    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_commands.append((cmd, value))


class FakeEnableLine:
    def __init__(self, dedicated=True):
        self._dedicated = dedicated
        self.state_callback = None

    def register_state_callback(self, callback):
        self.state_callback = callback

    def has_dedicated_enable(self):
        return self._dedicated


class FakeStepperEnable:
    def __init__(self, enable_line):
        self._enable_line = enable_line

    def lookup_enable(self, name):
        return self._enable_line


class FakeStepper:
    def __init__(self, pulse_duration=0.0000001, step_both_edge=True):
        self._pulse = (pulse_duration, step_both_edge)
        self.current_helper = None

    def set_tmc_current_helper(self, helper):
        self.current_helper = helper

    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        pass

    def get_pulse_duration(self):
        return self._pulse


class FakeForceMove:
    def __init__(self, stepper):
        self._stepper = stepper

    def lookup_stepper(self, name):
        return self._stepper


class FakeCurrentHelper:
    def get_current(self):
        return (0.8, 0.5, 0.5, 2.0, 0.8)


class FakePins:
    def __init__(self):
        self.chips = {}

    def register_chip(self, name, chip):
        self.chips[name] = chip


def writes(wire_log):
    return [entry for entry in wire_log if entry[0] == "write"]


def ops(wire_log):
    """Condense the log to (kind, name) pairs for sequence assertions."""
    return [(entry[0], entry[1]) for entry in wire_log]
