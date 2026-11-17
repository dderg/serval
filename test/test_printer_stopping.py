import pytest
from fakes import FakeReactor

import klippy.printer

STOP_EVENTS = (
    "klippy:stopping",
    "klippy:firmware_restart",
    "klippy:disconnect",
)


class _EndableReactor(FakeReactor):
    def end(self):
        pass


def _run_until_exit(result):
    printer = klippy.printer.Printer(_EndableReactor(), None, {})
    seen = []
    for event in STOP_EVENTS:
        printer.register_event_handler(
            event, lambda event=event: seen.append(event)
        )
    printer.request_exit(result)
    assert printer.run() == result
    return seen


@pytest.mark.parametrize("result", ["restart", "exit", "error_exit"])
def test_outputs_stop_before_the_host_disconnects(result):
    assert _run_until_exit(result) == ["klippy:stopping", "klippy:disconnect"]


def test_outputs_stop_before_the_firmware_restart_resets_the_mcus():
    assert _run_until_exit("firmware_restart") == list(STOP_EVENTS)
