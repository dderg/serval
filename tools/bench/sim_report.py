"""The `simulator` section of analysis.json, from a capture's aux records.

Reports what the F407's simulated peripherals saw and did during a run
(tools/bench/capture-fw/PHASE2.md): the F401/F407 crystal ratio from the MCO
count, heater and fan duty, modelled temperatures, endstop transitions and
TMC2209 traffic. The TMC dedge state the F401 wrote must match the capture's
both-edges mask; a mismatch means the capture counted steps wrongly. Given
the captured positions, every record's endstop levels are recomputed from
the walls; a disagreement lasting MISMATCH_RECORDS records is fatal (one
record may differ: the firmware holds a pin for the snapshot in which the
position touched a wall). TMC rejects and thermistor DAC clamps are fatal:
both mean klippy saw something the model did not intend.
"""

from __future__ import annotations

import configparser
from collections.abc import Callable

import capture_aux
import numpy as np

COUNT_TIME_HZ = 40_000
EVENT_TIME_HZ = 84_000_000
MCO_HZ = 6_250_000
REPORTED_TRANSITIONS = 50
COUNTER_WRAP = 256
MISMATCH_RECORDS = 2
SAMPLES_PER_RECORD = 10
HEATER_NAMES = ("extruder", "bed")


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


def wall_levels(walls: list[dict], positions: np.ndarray) -> np.ndarray:
    triggered = np.zeros(len(positions), np.uint8)
    idle = 0
    for wall in walls:
        bit = 1 << wall["pin"]
        if not wall["active_high"]:
            idle |= bit
        form = positions.astype(np.int64) @ np.array(wall["coeffs"], np.int64)
        hit = (
            form <= wall["threshold"]
            if wall["at_or_below"]
            else form >= wall["threshold"]
        )
        triggered |= np.where(hit, bit, 0).astype(np.uint8)
    return idle ^ triggered


def toolhead_mm(positions: np.ndarray, motors) -> np.ndarray:
    motor = {
        m.role: positions[:, m.channel] * m.dir_sign * m.step_mm for m in motors
    }
    return np.stack(
        [
            (motor["a"] + motor["b"]) / 2,
            (motor["a"] - motor["b"]) / 2,
            motor["z"],
        ],
        axis=1,
    )


def _longest_run(flags: np.ndarray) -> int:
    longest = current = 0
    for flag in flags:
        current = current + 1 if flag else 0
        longest = max(longest, current)
    return longest


def _walls_report(
    meta: dict, records: np.ndarray, positions: np.ndarray, motors
) -> dict:
    sim = meta["sim"]
    absolute = positions + np.array(sim["origin"], np.int64)
    expected = wall_levels(sim["walls"], absolute)
    mismatch = expected != records["endstops"]
    longest = _longest_run(mismatch)
    if longest >= MISMATCH_RECORDS:
        first = int(np.flatnonzero(mismatch)[0])
        raise SimReportError(
            f"endstop levels disagree with the captured positions for {longest} "
            f"records (first at record {first}: fired {int(records['endstops'][first]):#06b}, "
            f"walls say {int(expected[first]):#06b} at {absolute[first].tolist()})"
        )
    return {
        "mismatched_records": int(mismatch.sum()),
        "final_toolhead_mm": toolhead_mm(absolute[-1:], motors)[0].tolist(),
    }


def report(
    meta: dict,
    records: np.ndarray,
    config_text: str,
    motors,
    positions_at: Callable[[np.ndarray], np.ndarray] | None = None,
) -> dict:
    if records.size == 0:
        raise SimReportError("aux.bin holds no records")
    sent = meta["sim_config_id"]
    if not (records["config_id"] == sent).all():
        raise SimReportError(f"aux records do not all carry sim config {sent}")
    seconds = record_seconds(records, meta["mode"])
    source = meta.get("sim_config") or {}
    per_record = np.diff(records["samples"].astype(np.int64))
    off = np.flatnonzero(np.abs(per_record - SAMPLES_PER_RECORD) > 1)
    if off.size:
        raise SimReportError(
            f"{off.size} aux records saw an input sample count other than "
            f"{SAMPLES_PER_RECORD}±1 (first: {int(per_record[off[0]])} at record "
            f"{int(off[0]) + 1}): the 10 kHz sampler missed interrupts"
        )
    samples = per_record.sum()
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
        for i, name in enumerate(HEATER_NAMES)
    }
    clamped = {n: h["dac_clamped_records"] for n, h in heaters.items()}
    if any(clamped.values()):
        raise SimReportError(
            f"thermistor DAC clamped (records per heater: {clamped}): klippy "
            "read a temperature the model did not have"
        )
    endstops = records["endstops"]
    changed = np.flatnonzero(np.diff(endstops.astype(np.int16)) != 0) + 1
    positions = None if positions_at is None else positions_at(records)
    transitions = []
    for i in changed[:REPORTED_TRANSITIONS]:
        transition = {
            "time_s": float(seconds[i]),
            "pins": {
                pin: int((endstops[i] >> bit) & 1)
                for bit, pin in enumerate(capture_aux.PINS)
            },
        }
        if positions is not None:
            absolute = positions[i : i + 1] + np.array(meta["sim"]["origin"])
            transition["toolhead_mm"] = toolhead_mm(absolute, motors)[
                0
            ].tolist()
        transitions.append(transition)
    roles = tmc_roles(config_text, motors)
    channel_of = {m.role: m.channel for m in motors}
    dedge = int(records["dedge"][-1])
    if not (records["dedge"] == dedge).all():
        raise SimReportError("TMC dedge changed during the capture")
    unmapped = dedge & ~sum(1 << address for address in roles)
    if unmapped:
        raise SimReportError(
            f"TMC dedge set on addresses {unmapped:#06b} that no [tmc2209] "
            "section names"
        )
    dedge_mask = 0
    for address, role in roles.items():
        if dedge & (1 << address):
            dedge_mask |= 1 << channel_of[role]
    if dedge_mask != meta["both_edges_mask"]:
        raise SimReportError(
            f"TMC dedge selects channels {dedge_mask:#x} but the capture counted "
            f"both edges on {meta['both_edges_mask']:#x}"
        )
    rejects = _counter_total(records["tmc_rejects"])
    if rejects:
        raise SimReportError(
            f"the TMC bus rejected {rejects} frames during the capture "
            "(bad CRC, stray bytes, overrun or a request during a reply)"
        )
    return {
        "config_id": sent,
        "records": int(records.size),
        "mco": _mco_report(records, seconds, bool(source.get("expect_mco"))),
        "input_duty": duty,
        "heaters": heaters,
        "endstop_transitions": len(changed),
        "endstop_transition_examples": transitions,
        "walls": None
        if positions is None
        else _walls_report(meta, records, positions, motors),
        "tmc": {
            "dedge_channels_mask": dedge_mask,
            "writes": _counter_total(records["tmc_writes"]),
            "reads": _counter_total(records["tmc_reads"]),
            "rejects": rejects,
        },
    }
