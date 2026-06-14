from klippy.extras import tmc


class FakeFields:
    TOFF_REG = "CHOPCONF"

    def __init__(self):
        self.registers = {}
        self.set_field_calls = []

    def set_field(self, name, value):
        self.set_field_calls.append((name, value))
        self.registers[self.TOFF_REG] = value
        return value

    def lookup_register(self, name):
        return self.TOFF_REG


class FakeMcuTmc:
    def __init__(self):
        self.writes = []

    def set_register(self, reg, val, print_time=None):
        self.writes.append((reg, val, print_time))


class FakeEcheck:
    def __init__(self, did_reset=False, supported=True):
        self._did_reset = did_reset
        self._supported = supported
        self.start_calls = 0

    def start_checks(self):
        self.start_calls += 1
        return self._did_reset

    def reset_detect_supported(self):
        return self._supported


class FakePrinter:
    command_error = RuntimeError

    def __init__(self):
        self.shutdowns = []

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


class FakeTMC:
    _do_enable_bridge = tmc.TMCCommandHelper._do_enable_bridge

    def __init__(
        self, toff=None, did_reset=False, supported=True, post_cb=None
    ):
        self.toff = toff
        self.fields = FakeFields()
        self.mcu_tmc = FakeMcuTmc()
        self.echeck_helper = FakeEcheck(did_reset, supported)
        self.printer = FakePrinter()
        self.stepper_name = "stepper_x"
        self._post_enable_cb = post_cb
        self.init_calls = 0

    def _init_registers(self, print_time=None):
        self.init_calls += 1


def test_dedicated_enable_no_reset_skips_reinit():
    t = FakeTMC(toff=None, did_reset=False)
    t._do_enable_bridge(0.0)
    assert t.init_calls == 0, "registers persist across a dedicated-pin toggle"
    assert t.mcu_tmc.writes == []
    assert t.echeck_helper.start_calls == 1, "status/reset check still runs"
    assert t.printer.shutdowns == []


def test_dedicated_enable_reset_reinits():
    t = FakeTMC(toff=None, did_reset=True)
    t._do_enable_bridge(0.0)
    assert t.init_calls == 1, "driver lost its registers — restore them"


def test_virtual_enable_no_reset_restores_toff_only():
    t = FakeTMC(toff=3, did_reset=False)
    t._do_enable_bridge(1.5)
    assert t.init_calls == 0, "no full re-init when the driver did not reset"
    assert t.fields.set_field_calls == [("toff", 3)]
    assert t.mcu_tmc.writes == [("CHOPCONF", 3, 1.5)], "only toff is rewritten"


def test_virtual_enable_reset_reinits_and_restores_toff():
    t = FakeTMC(toff=3, did_reset=True)
    t._do_enable_bridge(0.0)
    assert t.init_calls == 1
    assert ("toff", 3) in t.fields.set_field_calls
    assert t.mcu_tmc.writes == [], (
        "full init carries toff; no extra single write"
    )


def test_unsupported_reset_detection_always_reinits():
    t = FakeTMC(toff=None, did_reset=False, supported=False)
    t._do_enable_bridge(0.0)
    assert t.init_calls == 1, (
        "tmc2130-style drivers can't prove no-reset; be safe"
    )


def test_phase_stepping_path_inits_and_calls_post_enable():
    calls = []
    t = FakeTMC(toff=None, post_cb=lambda: calls.append("cb"))
    t._do_enable_bridge(0.0)
    assert t.init_calls == 1, "phase-mode entry needs the full register setup"
    assert calls == ["cb"]
    assert t.echeck_helper.start_calls == 0, "post-enable cb owns the checks"
