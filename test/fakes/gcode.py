import collections

from .printer import FakeCommandError

Coord = collections.namedtuple("Coord", ("x", "y", "z", "e"))


class _FakeMutex:
    def __init__(self, busy=False):
        self.busy = busy

    def test(self):
        return self.busy


class FakeGcode:
    Coord = Coord

    def __init__(self, mutex_busy=False):
        self.commands = {}
        self.mux_commands = []
        self.scripts = []
        self.responses = []
        self._mutex = _FakeMutex(mutex_busy)

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.commands[cmd] = func

    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_commands.append((cmd, value))

    def run_script(self, script):
        self.scripts.append(script)

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def get_mutex(self):
        return self._mutex

    def respond_info(self, msg, log=True):
        self.responses.append(msg)

    def respond_raw(self, msg):
        self.responses.append(msg)


class FakeGcmd:
    error = FakeCommandError

    class sentinel:
        pass

    def __init__(
        self, params=None, command="", commandline="", error=None, **kwparams
    ):
        merged = dict(params) if params else {}
        merged.update(kwparams)
        self.params = merged
        self._command = command
        self._commandline = commandline
        self.responses = []
        if error is not None:
            self.error = error

    def get_commandline(self):
        return self._commandline

    def get_command_parameters(self):
        return self.params

    def get_raw_command_parameters(self):
        return self._commandline[len(self._command) :].lstrip()

    def get(
        self,
        name,
        default=sentinel,
        parser=str,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        value = self.params.get(name)
        if value is None:
            if default is self.sentinel:
                raise self.error(
                    "Error on '%s': missing %s" % (self._commandline, name)
                )
            return default
        try:
            value = parser(value)
        except Exception:
            raise self.error(
                "Error on '%s': unable to parse %s" % (self._commandline, value)
            )
        if minval is not None and value < minval:
            raise self.error(
                "Error on '%s': %s must have minimum of %s"
                % (self._commandline, name, minval)
            )
        if maxval is not None and value > maxval:
            raise self.error(
                "Error on '%s': %s must have maximum of %s"
                % (self._commandline, name, maxval)
            )
        if above is not None and value <= above:
            raise self.error(
                "Error on '%s': %s must be above %s"
                % (self._commandline, name, above)
            )
        if below is not None and value >= below:
            raise self.error(
                "Error on '%s': %s must be below %s"
                % (self._commandline, name, below)
            )
        return value

    def get_int(self, name, default=sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)

    def get_float(
        self,
        name,
        default=sentinel,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        return self.get(
            name,
            default,
            parser=float,
            minval=minval,
            maxval=maxval,
            above=above,
            below=below,
        )

    def respond_info(self, msg, log=True):
        self.responses.append(msg)

    def respond_raw(self, msg):
        self.responses.append(msg)
