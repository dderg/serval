from __future__ import annotations

from types import SimpleNamespace

import analyze
import capture_aux
import numpy as np
import pytest
import sim_report

STEP_MM = 0.0125
MOTORS = [
    SimpleNamespace(
        role=role, name=name, channel=channel, step_mm=STEP_MM, dir_sign=1
    )
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


PROBE_WALL = {
    "pin": 3,
    "active_high": True,
    "at_or_below": True,
    "coeffs": [0, 0, 1, 0],
    "threshold": 0,
}
ORIGIN_Z = 400


def _meta(mask: int = 0b1011, expect_mco: bool = True) -> dict:
    return {
        "mode": "counts",
        "sim_config_id": CONFIG_ID,
        "both_edges_mask": mask,
        "sim_config": {"expect_mco": expect_mco},
        "sim": {
            "config_id": CONFIG_ID,
            "origin": [0, 0, ORIGIN_Z, 0],
            "walls": [PROBE_WALL],
        },
    }


def _z_descends_one_step_per_record(records: np.ndarray) -> np.ndarray:
    positions = np.zeros((records.size, 4), np.int64)
    positions[:, 2] = -np.arange(records.size)
    return positions


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


def test_endstop_levels_agree_with_the_captured_positions():
    report = sim_report.report(
        _meta(), _records(), CONFIG, MOTORS, _z_descends_one_step_per_record
    )
    assert report["walls"]["mismatched_records"] == 0
    assert report["walls"]["final_toolhead_mm"] == pytest.approx(
        [0.0, 0.0, (ORIGIN_Z - (RECORDS - 1)) * STEP_MM]
    )
    [transition] = report["endstop_transition_examples"]
    assert transition["toolhead_mm"] == pytest.approx([0.0, 0.0, 0.0])


def test_a_one_record_disagreement_is_the_touch_hold():
    records = _records()
    records["endstops"][100] = 0b1000
    report = sim_report.report(
        _meta(), records, CONFIG, MOTORS, _z_descends_one_step_per_record
    )
    assert report["walls"]["mismatched_records"] == 1


def test_a_lasting_disagreement_with_the_walls_is_fatal():
    records = _records()
    records["endstops"][100:102] = 0b1000
    with pytest.raises(sim_report.SimReportError, match="2 records"):
        sim_report.report(
            _meta(), records, CONFIG, MOTORS, _z_descends_one_step_per_record
        )


def test_tmc_rejects_during_the_capture_are_fatal():
    records = _records()
    records["tmc_rejects"][500:] = 1
    with pytest.raises(sim_report.SimReportError, match="rejected 1 frames"):
        sim_report.report(_meta(), records, CONFIG, MOTORS)


def test_a_clamped_thermistor_is_fatal():
    records = _records()
    records["flags"][7] |= capture_aux.FLAG_DAC_CLAMPED[1]
    with pytest.raises(sim_report.SimReportError, match="clamped"):
        sim_report.report(_meta(), records, CONFIG, MOTORS)


def test_dedge_on_an_address_without_a_driver_section_is_fatal():
    with pytest.raises(sim_report.SimReportError, match="0b0100"):
        sim_report.report(_meta(), _records(dedge=0b1111), CONFIG, MOTORS)


def test_active_low_walls_idle_high():
    wall = dict(PROBE_WALL, active_high=False)
    positions = np.array([[0, 0, 1, 0], [0, 0, 0, 0]])
    assert sim_report.wall_levels([wall], positions).tolist() == [0b1000, 0]


def test_corexy_toolhead_from_motor_counts():
    positions = np.array([[240, -80, 400, 0]])
    assert sim_report.toolhead_mm(positions, MOTORS)[
        0
    ].tolist() == pytest.approx([1.0, 2.0, 5.0])


def test_count_records_read_positions_through_their_tick():
    deltas = np.array([[1, 0, 0, 0], [1, 0, -1, 0], [0, 0, -1, 0]], np.int8)
    capture = analyze.Capture(40_000.0, deltas)
    records = np.zeros(2, capture_aux.RECORD)
    records["time"] = [0, 2]
    positions = analyze.capture_positions(capture)(records)
    assert positions.tolist() == [[1, 0, 0, 0], [2, 0, -2, 0]]


def test_count_records_past_the_capture_are_refused():
    capture = analyze.Capture(40_000.0, np.zeros((3, 4), np.int8))
    records = np.zeros(1, capture_aux.RECORD)
    records["time"] = [3]
    with pytest.raises(analyze.AnalysisError, match="beyond"):
        analyze.capture_positions(capture)(records)


def test_event_records_count_steps_up_to_their_snapshot_time():
    def channel(ticks, signs):
        return SimpleNamespace(
            step_ticks=np.array(ticks, np.int64),
            step_signs=lambda: np.array(signs, np.int8),
        )

    events = SimpleNamespace(
        channels=[
            channel([10, 20, 30], [1, 1, -1]),
            channel([], []),
            channel([25], [-1]),
            channel([5], [1]),
        ]
    )
    capture = analyze.Capture(40_000.0, np.zeros((1, 4), np.int8), None, events)
    records = np.zeros(3, capture_aux.RECORD)
    records["time"] = [4, 20, 30]
    positions = analyze.capture_positions(capture)(records)
    assert positions.tolist() == [[0, 0, 0, 0], [2, 0, 0, 1], [1, 0, -1, 1]]


def test_a_missed_input_sample_is_fatal():
    records = _records()
    records["samples"][300:] -= 2
    with pytest.raises(sim_report.SimReportError, match="missed interrupts"):
        sim_report.report(_meta(), records, CONFIG, MOTORS)
