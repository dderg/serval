from fakes import FakePrinter

from klippy import mcu


class _EngineMcu:
    def __init__(self, claimed=True, death=None):
        self._claimed = claimed
        self._death = death
        self.takes = 0

    def is_claimed(self):
        return self._claimed

    def take_endpoint_death(self):
        self.takes += 1
        death, self._death = self._death, None
        return death


def _mcu(engine_mcu):
    printer = FakePrinter()
    m = object.__new__(mcu.MCU)
    m._printer = printer
    m._reactor = printer.get_reactor()
    m._name = "mcu"
    m.engine_mcu = engine_mcu
    return m, printer


def test_healthy_endpoint_keeps_polling_every_period():
    engine_mcu = _EngineMcu()
    m, printer = _mcu(engine_mcu)
    assert m._poll_endpoint_death(10.0) == 10.0 + mcu.ENDPOINT_DEATH_POLL_PERIOD
    assert engine_mcu.takes == 1
    assert printer.shutdown_reasons == []


def test_latched_death_shuts_down_with_the_cause_and_stops_polling():
    reason = "queue_step oid 9 is 2077 us behind the projected mcu clock"
    engine_mcu = _EngineMcu(death=reason)
    m, printer = _mcu(engine_mcu)
    assert m._poll_endpoint_death(10.0) == printer.reactor.NEVER
    assert printer.shutdown_reasons == [
        "MCU 'mcu' motion endpoint died: " + reason
    ]


def test_unclaimed_handle_never_polls():
    engine_mcu = _EngineMcu(claimed=False)
    m, printer = _mcu(engine_mcu)
    assert m._poll_endpoint_death(10.0) == printer.reactor.NEVER
    assert engine_mcu.takes == 0
