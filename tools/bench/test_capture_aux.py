from __future__ import annotations

import capture_aux
import numpy as np
import pytest

GOLDEN_RECORD = (
    "08070605040302011413121124232221310000003200000033000000"
    "ffffc4098a0ea00f424109030f050607"
)


def test_the_golden_record_from_the_rust_encoder_decodes():
    [record] = capture_aux.decode(bytes.fromhex(GOLDEN_RECORD))
    assert record["time"] == 0x0102_0304_0506_0708
    assert record["mco"] == 0x1112_1314
    assert record["samples"] == 0x2122_2324
    assert record["high"].tolist() == [0x31, 0x32, 0x33]
    assert record["temp_centi"].tolist() == [-1, 2_500]
    assert record["dac"].tolist() == [3_722, 4_000]
    assert record["config_id"] == 0x4142
    assert (record["endstops"], record["flags"], record["dedge"]) == (
        0b1001,
        0b011,
        0xF,
    )
    assert (
        record["tmc_writes"],
        record["tmc_reads"],
        record["tmc_rejects"],
    ) == (5, 6, 7)


def test_partial_records_are_refused():
    with pytest.raises(capture_aux.AuxError):
        capture_aux.decode(bytes(capture_aux.RECORD.itemsize + 1))


def test_the_mco_count_unwraps_across_32_bits():
    records = np.zeros(3, capture_aux.RECORD)
    records["mco"] = [0xFFFF_F000, 0x0000_0800, 0x0000_2000]
    assert capture_aux.unwrapped_mco(records).tolist() == [
        0xFFFF_F000,
        0x1_0000_0800,
        0x1_0000_2000,
    ]
