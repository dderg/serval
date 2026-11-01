from __future__ import annotations

from types import SimpleNamespace

import capture_aux
import numpy as np
import pytest
import sim_report

MOTORS = [
    SimpleNamespace(role=role, name=name, channel=channel)
    for channel, (role, name) in enumerate(
        [("a", "a"), ("b", "b"), ("z", "z"), ("e", "extruder")]
    )
]
CONFIG = """
[tmc2209 a]
uart_address: 0
[tmc2209 b]
uart_address: 1
[tmc2209 extruder]
uart_address: 3
"""
RECORDS = 1_000
CONFIG_ID = 77
PPM = 30.0


def _records(dedge: int = 0b1011) -> np.ndarray:
    records = np.zeros(RECORDS, capture_aux.RECORD)
    ms = np.arange(RECORDS)
    records["time"] = ms * 40
    records["mco"] = np.round(ms * 6_250 * (1 + PPM * 1e-6)) + 0xFFFF_0000
    records["samples"] = ms * 10
    records["high"][:, 0] = ms * 5
    records["high"][:, 2] = ms * 10
    records["temp_centi"][:, 0] = 2_500 + ms
    records["temp_centi"][:, 1] = 2_500
    records["config_id"] = CONFIG_ID
    records["endstops"] = np.where(ms < 400, 0, 0b1000)
    records["flags"] = capture_aux.FLAG_MCO_OK
    records["dedge"] = dedge
    records["tmc_writes"] = (ms // 2) % 256
    return records


def _meta(mask: int = 0b1011, expect_mco: bool = True) -> dict:
    return {
        "mode": "counts",
        "sim_config_id": CONFIG_ID,
        "both_edges_mask": mask,
        "sim_config": {"expect_mco": expect_mco},
    }


def test_the_report_summarizes_the_simulated_peripherals():
    report = sim_report.report(_meta(), _records(), CONFIG, MOTORS)
    assert report["mco"]["f401_vs_f407_ppm"] == pytest.approx(PPM, abs=0.5)
    assert report["input_duty"] == pytest.approx(
        {"extruder_heater": 0.5, "bed_heater": 0.0, "part_fan": 1.0}
    )
    assert report["heaters"]["extruder"]["end_c"] == pytest.approx(25 + 9.99)
    assert report["endstop_transitions"] == 1
    [transition] = report["endstop_transition_examples"]
    assert transition["time_s"] == pytest.approx(400 * 40 / 40_000)
    assert transition["pins"]["probe"] == 1
    assert report["tmc"]["dedge_channels_mask"] == 0b1011
    assert report["tmc"]["writes"] == (RECORDS - 1) // 2


def test_dedge_disagreeing_with_the_capture_mask_is_fatal():
    with pytest.raises(sim_report.SimReportError, match="dedge"):
        sim_report.report(_meta(mask=0b0011), _records(), CONFIG, MOTORS)


def test_a_rejected_sim_config_is_fatal():
    records = _records()
    records["config_id"][5] = 0
    with pytest.raises(sim_report.SimReportError, match="config"):
        sim_report.report(_meta(), records, CONFIG, MOTORS)


def test_a_lost_mco_is_fatal_only_when_expected():
    records = _records()
    records["flags"][10] = 0
    with pytest.raises(sim_report.SimReportError, match="MCO"):
        sim_report.report(_meta(), records, CONFIG, MOTORS)
    report = sim_report.report(_meta(expect_mco=False), records, CONFIG, MOTORS)
    assert report["mco"]["records_ok"] == RECORDS - 1


def test_a_driver_without_a_captured_motor_is_refused():
    with pytest.raises(sim_report.SimReportError, match="captured motor"):
        sim_report.tmc_roles("[tmc2209 y_motor]\nuart_address: 2\n", MOTORS)
