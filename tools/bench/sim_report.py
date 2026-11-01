"""The `simulator` section of analysis.json, from a capture's aux records.

Reports what the F407's simulated peripherals saw and did during a run
(tools/bench/capture-fw/PHASE2.md): the F401/F407 crystal ratio from the MCO
count, heater and fan duty, modelled temperatures, endstop transitions and
TMC2209 traffic. The TMC dedge state the F401 wrote must match the capture's
both-edges mask; a mismatch means the capture counted steps wrongly.
"""

from __future__ import annotations

import configparser

import capture_aux
import numpy as np

COUNT_TIME_HZ = 40_000
EVENT_TIME_HZ = 84_000_000
MCO_HZ = 6_250_000
REPORTED_TRANSITIONS = 50
COUNTER_WRAP = 256


class SimReportError(RuntimeError):
    pass


def record_seconds(records: np.ndarray, mode: str) -> np.ndarray:
    hz = EVENT_TIME_HZ if mode == "events" else COUNT_TIME_HZ
    return records["time"].astype(np.float64) / hz


def tmc_roles(config_text: str, motors) -> dict[int, str]:
    sections = configparser.RawConfigParser(strict=False)
    sections.read_string(config_text)
    role_of = {m.name: m.role for m in motors}
    roles = {}
    for name in sections.sections():
        kind, _, motor = name.partition(" ")
        if kind != "tmc2209":
            continue
        if motor not in role_of:
            raise SimReportError(f"[{name}] drives no captured motor")
        address = sections.getint(name, "uart_address", fallback=0)
        roles[address] = role_of[motor]
    return roles


def _counter_total(values: np.ndarray) -> int:
    steps = np.diff(values.astype(np.int64)) % COUNTER_WRAP
    return int(steps.sum())


def _mco_report(records: np.ndarray, seconds: np.ndarray, expect: bool) -> dict:
    ok = (records["flags"] & capture_aux.FLAG_MCO_OK) != 0
    if expect and not ok.all():
        raise SimReportError(
            f"MCO count out of range in {int((~ok).sum())} of {ok.size} records"
        )
    report = {"records_ok": int(ok.sum()), "records": int(ok.size)}
    if ok.sum() >= 2:
        mco = capture_aux.unwrapped_mco(records)[ok]
        slope = np.polyfit(seconds[ok], mco, 1)[0]
        report["f401_vs_f407_ppm"] = float((slope / MCO_HZ - 1.0) * 1e6)
    return report


def report(meta: dict, records: np.ndarray, config_text: str, motors) -> dict:
    if records.size == 0:
        raise SimReportError("aux.bin holds no records")
    sent = meta["sim_config_id"]
    if not (records["config_id"] == sent).all():
        raise SimReportError(f"aux records do not all carry sim config {sent}")
    seconds = record_seconds(records, meta["mode"])
    source = meta.get("sim_config") or {}
    samples = np.diff(records["samples"].astype(np.int64)).sum()
    high = np.diff(records["high"].astype(np.int64), axis=0).sum(axis=0)
    duty = {
        name: float(h / samples) if samples else None
        for name, h in zip(capture_aux.INPUTS, high)
    }
    temps = records["temp_centi"].astype(np.float64) / 100.0
    heaters = {
        name: {
            "start_c": float(temps[0, i]),
            "end_c": float(temps[-1, i]),
            "min_c": float(temps[:, i].min()),
            "max_c": float(temps[:, i].max()),
            "dac_clamped_records": int(
                (
                    (records["flags"] & capture_aux.FLAG_DAC_CLAMPED[i]) != 0
                ).sum()
            ),
        }
        for i, name in enumerate(("extruder", "bed"))
    }
    endstops = records["endstops"]
    changed = np.flatnonzero(np.diff(endstops.astype(np.int16)) != 0) + 1
    transitions = [
        {
            "time_s": float(seconds[i]),
            "pins": {
                pin: int((endstops[i] >> bit) & 1)
                for bit, pin in enumerate(capture_aux.PINS)
            },
        }
        for i in changed[:REPORTED_TRANSITIONS]
    ]
    roles = tmc_roles(config_text, motors)
    channel_of = {m.role: m.channel for m in motors}
    dedge = int(records["dedge"][-1])
    if not (records["dedge"] == dedge).all():
        raise SimReportError("TMC dedge changed during the capture")
    dedge_mask = 0
    for address, role in roles.items():
        if dedge & (1 << address):
            dedge_mask |= 1 << channel_of[role]
    if dedge_mask != meta["both_edges_mask"]:
        raise SimReportError(
            f"TMC dedge selects channels {dedge_mask:#x} but the capture counted "
            f"both edges on {meta['both_edges_mask']:#x}"
        )
    return {
        "config_id": sent,
        "records": int(records.size),
        "mco": _mco_report(records, seconds, bool(source.get("expect_mco"))),
        "input_duty": duty,
        "heaters": heaters,
        "endstop_transitions": len(changed),
        "endstop_transition_examples": transitions,
        "tmc": {
            "dedge_channels_mask": dedge_mask,
            "writes": _counter_total(records["tmc_writes"]),
            "reads": _counter_total(records["tmc_reads"]),
            "rejects": _counter_total(records["tmc_rejects"]),
        },
    }
