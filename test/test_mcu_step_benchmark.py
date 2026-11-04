from types import SimpleNamespace

import pytest

from klippy import mcu


class FakeMcu:
    _query_step_benchmark = mcu.MCU._query_step_benchmark

    def __init__(self, commands, response=None):
        self._name = "mcu"
        self._mcu_freq = 84_000_000.0
        parser = SimpleNamespace(messages_by_name=dict.fromkeys(commands))
        self._serial = SimpleNamespace(get_msgparser=lambda: parser)
        self._response = response
        self.queries = []

    def lookup_query_command(self, msgformat, respformat):
        self.queries.append(msgformat)
        return SimpleNamespace(send=lambda: self._response)


def test_firmware_that_cannot_drive_steppers_has_no_benchmark():
    fake = FakeMcu(["beacon_contact"])
    assert fake._query_step_benchmark() is None
    assert fake.queries == []


def test_stepper_firmware_without_the_benchmark_is_refused():
    fake = FakeMcu(["config_stepper"])
    with pytest.raises(mcu.error, match="flash firmware built from this"):
        fake._query_step_benchmark()


def test_stepper_firmware_without_the_cpu_load_monitor_is_refused():
    fake = FakeMcu(["config_stepper", "get_step_benchmark"])
    with pytest.raises(mcu.error, match="lacks cpu_load"):
        fake._query_step_benchmark()


def test_stepper_firmware_reports_its_measured_cost():
    fake = FakeMcu(
        ["config_stepper", "get_step_benchmark", "cpu_load"],
        {
            "pulsed_ticks_per_step": 301,
            "both_edge_ticks_per_step": 160,
            "simulated": 0,
        },
    )
    assert fake._query_step_benchmark() == {
        "pulsed_ticks_per_step": 301,
        "both_edge_ticks_per_step": 160,
        "simulated": False,
    }


def test_an_empty_measurement_on_real_hardware_is_refused():
    fake = FakeMcu(
        ["config_stepper", "get_step_benchmark", "cpu_load"],
        {
            "pulsed_ticks_per_step": 0,
            "both_edge_ticks_per_step": 160,
            "simulated": 0,
        },
    )
    with pytest.raises(mcu.error, match="empty step benchmark"):
        fake._query_step_benchmark()


def test_a_simulated_mcu_reports_without_a_measurement():
    fake = FakeMcu(
        ["config_stepper", "get_step_benchmark", "cpu_load"],
        {
            "pulsed_ticks_per_step": 0,
            "both_edge_ticks_per_step": 0,
            "simulated": 1,
        },
    )
    assert fake._query_step_benchmark()["simulated"] is True
