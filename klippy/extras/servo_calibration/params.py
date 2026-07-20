from __future__ import annotations

GAIN_PARAMS = {
    "position": (
        "0x2001.0x01",
        1,
        20000,
        "C01.00 position loop gain",
        "0.1 rad/s",
        10.0,
    ),
    "speed": (
        "0x2001.0x02",
        1,
        20000,
        "C01.01 speed loop gain",
        "0.1 Hz",
        10.0,
    ),
    "integral": (
        "0x2001.0x03",
        15,
        51200,
        "C01.02 speed integral time",
        "0.01 ms",
        100.0,
    ),
    "torque_filter": (
        "0x2001.0x19",
        5,
        16000,
        "C01.18 torque feedforward filter cutoff",
        "Hz",
        1.0,
    ),
}

GAIN_LIST_PARAMS = {
    "POS_GAINS": "position",
    "SPEED_GAINS": "speed",
    "INTEGRALS": "integral",
    "TORQUE_FILTERS": "torque_filter",
}

INERTIA_RATIO_ADDR = "0x2000.0x07"
C00_06_INERTIA_RATIO_MAX = 12000

SYNC_LOSS_COUNT_ADDR = "0x2013.0x05"
SYNC_LOSS_THRESHOLD_ADDR = "0x2013.0x03"

NOTCH_MODE_ADDR = "0x2001.0x31"
NOTCH_READBACK: tuple[tuple[str, tuple[str, str, str]], ...] = (
    ("notch1", ("0x2001.0x41", "0x2001.0x42", "0x2001.0x43")),
    ("notch2", ("0x2001.0x44", "0x2001.0x45", "0x2001.0x46")),
    ("notch3", ("0x2001.0x47", "0x2001.0x48", "0x2001.0x49")),
    ("notch4", ("0x2001.0x4a", "0x2001.0x4b", "0x2001.0x4c")),
    ("notch5", ("0x2001.0x4d", "0x2001.0x4e", "0x2001.0x4f")),
)


def validate_gain_values(values: list[int], param: str) -> list[int]:
    if param not in GAIN_PARAMS:
        raise ValueError(
            "PARAM must be one of %s (got %r)" % (", ".join(GAIN_PARAMS), param)
        )
    _addr, lo, hi, _desc, _unit, _scale = GAIN_PARAMS[param]
    for v in values:
        if v <= 0:
            raise ValueError(
                "%s value %d is not a positive integer" % (param, v)
            )
        if not lo <= v <= hi:
            raise ValueError(
                "%s value %d outside drive range %d..%d" % (param, v, lo, hi)
            )
    return values
