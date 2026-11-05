"""Decode aux.bin, the simulated-peripheral records of a capture.

Layout (little-endian, 44 bytes, tools/bench/capture-fw/PHASE2.md):
    u64 time | u32 mco | u32 samples | u32 high[3] | i16 temp_centi[2]
    | u16 dac[2] | u16 config_id | u8 endstops | u8 flags | u8 dedge
    | u8 tmc_writes | u8 tmc_reads | u8 tmc_rejects
time is the count-mode tick index or the event-mode 84 MHz master time of
the snapshot the record was taken in; mco is the F401 MCO1 count (6.25 MHz)
sampled in that same snapshot; samples/high are cumulative 10 kHz samples of
the extruder heater, bed heater and part fan inputs.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

RECORD = np.dtype(
    [
        ("time", "<u8"),
        ("mco", "<u4"),
        ("samples", "<u4"),
        ("high", "<u4", (3,)),
        ("temp_centi", "<i2", (2,)),
        ("dac", "<u2", (2,)),
        ("config_id", "<u2"),
        ("endstops", "u1"),
        ("flags", "u1"),
        ("dedge", "u1"),
        ("tmc_writes", "u1"),
        ("tmc_reads", "u1"),
        ("tmc_rejects", "u1"),
    ]
)
FLAG_MCO_OK = 1 << 0
FLAG_DAC_CLAMPED = (1 << 1, 1 << 2)
MCO_WRAP = 1 << 32
INPUTS = ("extruder_heater", "bed_heater", "part_fan")
PINS = ("x_endstop", "y_endstop", "z_endstop", "probe")


class AuxError(RuntimeError):
    pass


def decode(data: bytes) -> np.ndarray:
    if len(data) % RECORD.itemsize:
        raise AuxError(f"aux.bin of {len(data)} B is not whole records")
    return np.frombuffer(data, RECORD)


def load(run_dir: Path) -> np.ndarray:
    return decode((run_dir / "aux.bin").read_bytes())


def unwrapped_mco(records: np.ndarray) -> np.ndarray:
    mco = records["mco"].astype(np.int64)
    steps = np.diff(mco)
    steps[steps < 0] += MCO_WRAP
    return (
        np.concatenate(([mco[0]], mco[0] + np.cumsum(steps)))
        if mco.size
        else mco
    )
