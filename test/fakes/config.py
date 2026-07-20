from .printer import FakeConfigError

_UNSET = object()


def _to_int(value):
    return None if value is None else int(value)


def _to_float(value):
    return None if value is None else float(value)


def _to_bool(value):
    if value is None or isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise ValueError("Not a boolean: %s" % value)


class FakeConfig:
    error = FakeConfigError

    def __init__(
        self, printer=None, name="", values=None, sections=None, error=None
    ):
        self._printer = printer
        self._name = name
        self._values = dict(values) if values else {}
        self._sections = {} if sections is None else sections
        if error is not None:
            self.error = error
        if name and name not in self._sections:
            self._sections[name] = self

    def get_name(self):
        return self._name

    def get_printer(self):
        return self._printer

    def has_section(self, name):
        return name in self._sections

    def getsection(self, name):
        if name not in self._sections:
            raise self.error("no section [%s]" % name)
        section = self._sections[name]
        if isinstance(section, FakeConfig):
            return section
        return FakeConfig(
            self._printer, name, section, self._sections, self.error
        )

    def get_prefix_sections(self, prefix):
        return [
            self.getsection(name)
            for name in self._sections
            if name.startswith(prefix)
        ]

    def _get_wrapper(
        self,
        coerce,
        option,
        default,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        if option not in self._values:
            if default is not _UNSET:
                return default
            raise self.error(
                "Option '%s' in section '%s' must be specified"
                % (option, self._name)
            )
        raw = self._values[option]
        try:
            val = coerce(raw)
        except self.error:
            raise
        except Exception:
            raise self.error(
                "Unable to parse option '%s' in section '%s'"
                % (option, self._name)
            )
        if val is None:
            return None
        if minval is not None and val < minval:
            raise self.error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self._name, minval)
            )
        if maxval is not None and val > maxval:
            raise self.error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self._name, maxval)
            )
        if above is not None and val <= above:
            raise self.error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self._name, above)
            )
        if below is not None and val >= below:
            raise self.error(
                "Option '%s' in section '%s' must be below %s"
                % (option, self._name, below)
            )
        return val

    def get(self, option, default=_UNSET, note_valid=True):
        return self._get_wrapper(
            lambda v: v, option, default, note_valid=note_valid
        )

    def getint(
        self, option, default=_UNSET, minval=None, maxval=None, note_valid=True
    ):
        return self._get_wrapper(
            _to_int, option, default, minval, maxval, note_valid=note_valid
        )

    def getfloat(
        self,
        option,
        default=_UNSET,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        return self._get_wrapper(
            _to_float,
            option,
            default,
            minval,
            maxval,
            above,
            below,
            note_valid=note_valid,
        )

    def getboolean(self, option, default=_UNSET, note_valid=True):
        return self._get_wrapper(
            _to_bool, option, default, note_valid=note_valid
        )

    def getchoice(self, option, choices, default=_UNSET, note_valid=True):
        if isinstance(choices, list):
            choices = {c: c for c in choices}
        if choices and isinstance(next(iter(choices)), int):
            picked = self.getint(option, default, note_valid=note_valid)
        else:
            picked = self.get(option, default, note_valid=note_valid)
        if picked not in choices:
            raise self.error(
                "Choice '%s' for option '%s' in section '%s'"
                " is not a valid choice" % (picked, option, self._name)
            )
        return choices[picked]

    def getlists(
        self,
        option,
        default=_UNSET,
        seps=(",",),
        count=None,
        parser=str,
        note_valid=True,
    ):
        def lparser(value, pos):
            if isinstance(value, (list, tuple)):
                parts = list(value)
            elif len(value.strip()) == 0:
                parts = []
            else:
                parts = [p.strip() for p in value.split(seps[pos])]
            if pos:
                return tuple(lparser(p, pos - 1) for p in parts if p)
            res = [parser(p) for p in parts]
            if count is not None and len(res) != count:
                raise self.error(
                    "Option '%s' in section '%s' must have %d elements"
                    % (option, self._name, count)
                )
            return res

        return self._get_wrapper(
            lambda v: lparser(v, len(seps) - 1),
            option,
            default,
            note_valid=note_valid,
        )

    def getlist(
        self, option, default=_UNSET, sep=",", count=None, note_valid=True
    ):
        return self.getlists(
            option,
            default,
            seps=(sep,),
            count=count,
            parser=str,
            note_valid=note_valid,
        )
