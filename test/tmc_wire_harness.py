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

from fakes import FakeConfig as _CanonicalFakeConfig
from fakes import FakeEnableLine, FakeGcode, FakePins, FakeStepper
from fakes import FakeMcu as _CanonicalFakeMcu
from fakes import FakePrinter as _CanonicalFakePrinter
from fakes import FakeReactor as _CanonicalFakeReactor
from fakes import FakeStepperEnable as _CanonicalFakeStepperEnable

__all__ = [
    "CommandError",
    "ConfigError",
    "FakeCommand",
    "FakeConfig",
    "FakeCurrentHelper",
    "FakeEnableLine",
    "FakeForceMove",
    "FakeGcode",
    "FakeMCU",
    "FakeMcuTmc",
    "FakePins",
    "FakePrinter",
    "FakeQueryCommand",
    "FakeReactor",
    "FakeStepper",
    "FakeStepperEnable",
    "ops",
    "writes",
]


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class FakeReactor(_CanonicalFakeReactor):
    """A static-clock reactor whose timer churn is recorded into the wire
    log, so tests can assert health-check timers start/stop in lockstep
    with the driver enable state.
    """

    def __init__(self, wire_log):
        super().__init__(now=100.0, tick=0.0)
        self._log = wire_log

    def register_timer(self, callback, waketime=_CanonicalFakeReactor.NEVER):
        timer_handler = super().register_timer(callback, waketime)
        self._log.append(("timer+", callback.__name__))
        return timer_handler

    def unregister_timer(self, timer_handler):
        super().unregister_timer(timer_handler)
        self._log.append(("timer-", timer_handler[0].__name__))

    def run_callbacks(self):
        callbacks, self.callbacks = self.callbacks, []
        for cb in callbacks:
            cb(self.now)


class FakePrinter(_CanonicalFakePrinter):
    """A printer that fires events directly (rather than queuing them) and
    keeps every handler registered per event, since several TMC driver
    instances share one printer in the phase-stepping group tests.
    """

    command_error = CommandError
    config_error = ConfigError

    def __init__(self, wire_log):
        super().__init__(reactor=FakeReactor(wire_log))
        self.shutdowns = []

    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)

    def fire_event(self, event):
        for handler in self.event_handlers.get(event, []):
            handler()

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


class FakeConfig(_CanonicalFakeConfig):
    error = ConfigError

    def __init__(self, name, options, printer, sections):
        super().__init__(
            printer=printer,
            name=name,
            values=options,
            sections=sections,
            error=ConfigError,
        )


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


class FakeMCU(_CanonicalFakeMcu):
    """The raw-command seam: lookup_command / lookup_query_command build
    wire-log-recording command objects, and script_query/next_query_response
    serve scripted responses for register-adjacent raw queries.
    """

    def __init__(self, wire_log):
        super().__init__()
        self._log = wire_log
        self._query_scripts = {}

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


class FakeStepperEnable(_CanonicalFakeStepperEnable):
    def __init__(self, enable_line):
        super().__init__(enable_line=enable_line)


class FakeForceMove:
    def __init__(self, stepper):
        self._stepper = stepper

    def lookup_stepper(self, name):
        return self._stepper


class FakeCurrentHelper:
    def get_current(self):
        return (0.8, 0.5, 0.5, 2.0, 0.8)


def writes(wire_log):
    return [entry for entry in wire_log if entry[0] == "write"]


def ops(wire_log):
    """Condense the log to (kind, name) pairs for sequence assertions."""
    return [(entry[0], entry[1]) for entry in wire_log]
