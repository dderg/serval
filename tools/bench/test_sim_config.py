from __future__ import annotations

import ast
import json
import logging
import math
from pathlib import Path

import pytest
import sim_config

BENCH_DIR = Path(__file__).resolve().parent
KLIPPY_EXTRAS = BENCH_DIR.parents[1] / "klippy" / "extras"
GOLDEN_CONFIG = (
    "07000110e8030000f4ffffff0a00000000000000020a00030101000040060000020a03030000"
    "0100000000000321000000a0410000c0410000803f0000803e0000c8415043c74700803b4500"
    "401c46040101"
)
READ_IFCNT_ADDR0 = bytes([0xF5, 0x00, 0x02, 0xC8])
READ_CHOPCONF_ADDR2 = bytes([0xF5, 0x02, 0x6C, 0xD6])
WRITE_CHOPCONF_ADDR1 = bytes([0xF5, 0x01, 0xEC, 0x30, 0x01, 0x00, 0x53, 0xDA])
REPLY_IFCNT_1 = bytes([0x05, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x01, 0xC5])
REPLY_CHOPCONF = bytes([0x05, 0xFF, 0x6C, 0x30, 0x01, 0x00, 0x53, 0x02])
STEP_MM = {"a": 40 / 3200, "b": 40 / 3200, "z": 8 / 3200, "e": 7.73 / 3200}
SIGNS = {"a": 1, "b": 1, "z": 1, "e": 1}


def _klippy_class(module: str, name: str, methods: set[str] | None = None):
    tree = ast.parse((KLIPPY_EXTRAS / module).read_text())
    cls = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )
    if methods is not None:
        cls.body = [
            n
            for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in methods
        ]
    cls.bases = []
    namespace = {"math": math, "logging": logging, "KELVIN_TO_CELSIUS": -273.15}
    exec(compile(ast.Module([cls], []), module, "exec"), namespace)
    return namespace[name]


def _uart_encoder():
    cls = _klippy_class(
        "tmc_uart.py",
        "MCU_TMC_uart_bitbang",
        {"_calc_crc8", "_encode_read", "_encode_write", "_add_serial_bits"},
    )
    return object.__new__(cls)


def _bytes_of(encoder, msg: bytes) -> bytes:
    return bytes(msg[:-1]) + bytes([encoder._calc_crc8(bytearray(msg[:-1]))])


def test_the_firmware_golden_frames_are_what_klippy_sends_and_accepts():
    uart = _uart_encoder()
    assert _bytes_of(uart, READ_IFCNT_ADDR0) == READ_IFCNT_ADDR0
    assert _bytes_of(uart, READ_CHOPCONF_ADDR2) == READ_CHOPCONF_ADDR2
    assert _bytes_of(uart, WRITE_CHOPCONF_ADDR1) == WRITE_CHOPCONF_ADDR1
    assert uart._encode_write(0x05, 0xFF, 0x02, 1) == uart._add_serial_bits(
        REPLY_IFCNT_1
    )
    assert uart._encode_write(
        0x05, 0xFF, 0x6C, 0x30010053
    ) == uart._add_serial_bits(REPLY_CHOPCONF)


@pytest.mark.parametrize(
    ("temp_c", "ratio"), [(20, 0.922), (25, 0.909), (280, 0.088)]
)
def test_klippys_beta_thermistor_reads_the_documented_firmware_ratios(
    temp_c, ratio
):
    thermistor = _klippy_class("thermistor.py", "Thermistor")(10_000.0, 0.0)
    thermistor.setup_coefficients_beta(25.0, 100_000.0, 3000.0)
    assert thermistor.calc_adc(temp_c) == pytest.approx(ratio, abs=0.001)
    assert thermistor.calc_temp(thermistor.calc_adc(temp_c)) == pytest.approx(
        temp_c
    )


def test_the_golden_payload_matches_the_rust_decoder_test():
    config = sim_config.SimConfig(
        7,
        (1000, -12, 10, 0),
        (
            sim_config.Wall(0, True, True, (1, 1, 0, 0), 1600),
            sim_config.Wall(3, True, True, (0, 0, 1, 0), 0),
        ),
        (
            sim_config.Heater(
                0, (20.0, 24.0, 1.0, 0.25, 25.0, 102022.625, 3000.0, 10000.0)
            ),
        ),
        True,
    )
    assert sim_config.encode(config).hex() == GOLDEN_CONFIG
    command = sim_config.command(config)
    assert command[:1] == b"C"
    assert int.from_bytes(command[1:3], "little") == len(GOLDEN_CONFIG) // 2


def _build(spec: dict, signs=SIGNS, start=(0.0, 0.0, 0.0)):
    return sim_config.build(spec, 9, STEP_MM, signs, start)


def test_corexy_walls_are_toolhead_sums_of_channel_counts():
    config = _build(
        {
            "walls": [
                {
                    "pin": "x_endstop",
                    "axis": "x",
                    "at_mm": 5.0,
                    "trigger": "at_or_below",
                },
                {
                    "pin": "y_endstop",
                    "axis": "y",
                    "at_mm": 300.0,
                    "trigger": "at_or_above",
                },
                {
                    "pin": "probe",
                    "axis": "z",
                    "at_mm": -0.5,
                    "trigger": "at_or_below",
                },
            ]
        },
        start=(10.0, 20.0, 1.0),
    )
    x, y, probe = config.walls
    assert (x.pin, x.coeffs, x.threshold, x.at_or_below) == (
        0,
        (1, 1, 0, 0),
        800,
        True,
    )
    assert (y.pin, y.coeffs, y.threshold, y.at_or_below) == (
        1,
        (1, -1, 0, 0),
        48_000,
        False,
    )
    assert (probe.pin, probe.coeffs, probe.threshold) == (3, (0, 0, 1, 0), -200)
    assert config.origin == (2_400, -800, 400, 0)


def test_an_inverted_dir_pin_flips_its_channel():
    signs = dict(SIGNS, b=-1)
    config = _build(
        {
            "walls": [
                {
                    "pin": "x_endstop",
                    "axis": "x",
                    "at_mm": 5.0,
                    "trigger": "at_or_below",
                }
            ]
        },
        signs=signs,
        start=(10.0, 20.0, 0.0),
    )
    assert config.walls[0].coeffs == (1, -1, 0, 0)
    assert config.origin == (2_400, 800, 0, 0)


@pytest.mark.parametrize(
    ("wall", "error"),
    [
        (
            {
                "pin": "w_endstop",
                "axis": "x",
                "at_mm": 0,
                "trigger": "at_or_below",
            },
            "unknown",
        ),
        (
            {"pin": "probe", "axis": "q", "at_mm": 0, "trigger": "at_or_below"},
            "not x, y",
        ),
        (
            {
                "pin": "probe",
                "axis": "z",
                "at_mm": 0.001,
                "trigger": "at_or_below",
            },
            "whole number",
        ),
    ],
)
def test_bad_walls_are_refused(wall, error):
    with pytest.raises(sim_config.SimConfigError, match=error):
        _build({"walls": [wall]})


def test_the_full_run_config_builds():
    spec = json.loads((BENCH_DIR / "configs" / "full.sim.json").read_text())
    config = _build(spec)
    assert config.both_edges_mask == 0b1111
    assert config.expect_mco
    assert [h.index for h in config.heaters] == [0, 1]
    assert len(sim_config.encode(config)) <= sim_config.MAX_PAYLOAD


def test_unknown_keys_are_refused():
    with pytest.raises(sim_config.SimConfigError, match="unknown keys"):
        _build({"wals": []})
    wall = {
        "pin": "probe",
        "axis": "z",
        "at_mm": 0,
        "trigger": "at_or_below",
        "polarity": 1,
    }
    with pytest.raises(sim_config.SimConfigError, match="polarity"):
        _build({"walls": [wall]})
    spec = json.loads((BENCH_DIR / "configs" / "full.sim.json").read_text())
    spec["heaters"]["bed"]["mass_kg"] = 1.0
    with pytest.raises(sim_config.SimConfigError, match="mass_kg"):
        _build(spec)


def test_a_wall_needs_exactly_one_position():
    wall = {"pin": "probe", "axis": "z", "trigger": "at_or_below"}
    with pytest.raises(sim_config.SimConfigError, match="exactly one"):
        _build({"walls": [wall]})
    with pytest.raises(sim_config.SimConfigError, match="exactly one"):
        _build({"walls": [dict(wall, at_mm=0, at_steps=0)]})
    toolhead = {
        "pin": "x_endstop",
        "axis": "x",
        "at_steps": 0,
        "trigger": "at_or_below",
    }
    with pytest.raises(sim_config.SimConfigError, match="at_mm"):
        _build({"walls": [toolhead]})


def test_a_start_between_steps_is_refused():
    with pytest.raises(sim_config.SimConfigError, match="start z"):
        _build({}, start=(0.0, 0.0, 0.001))


def test_describe_records_what_the_firmware_evaluates():
    config = _build(
        {
            "walls": [
                {
                    "pin": "probe",
                    "axis": "z",
                    "at_mm": 0,
                    "trigger": "at_or_below",
                }
            ]
        },
        start=(0.0, 0.0, 1.0),
    )
    assert sim_config.describe(config) == {
        "config_id": 9,
        "origin": [0, 0, 400, 0],
        "walls": [
            {
                "pin": 3,
                "active_high": True,
                "at_or_below": True,
                "coeffs": (0, 0, 1, 0),
                "threshold": 0,
            }
        ],
    }
