import pathlib
import re

import pytest

from klippy import mcu as mcu_mod
from klippy import motion_setup
from klippy.mcu import (
    MCU,
    PHASE_TRANSPORT_PIECE,
    PHASE_TRANSPORT_SAMPLE,
    SAMPLE_COMMANDS,
    STEPPING_MODE_PIECE,
    STEPPING_MODE_STEPCOMPRESS,
)

REPO = pathlib.Path(__file__).resolve().parents[1]
WIRE_HEADER = REPO / "src" / "sample_wire.h"
WIRE_RUST = REPO / "rust" / "runtime" / "src" / "sample_wire.rs"


class FakeConfigError(Exception):
    pass


class FakeFileConfig:
    def __init__(self, options):
        self._options = options

    def has_option(self, _section, option):
        return option in self._options


class FakeConfig:
    error = FakeConfigError

    def __init__(self, options=None, name="mcu"):
        self._options = dict(options or {})
        self._name = name
        self.section = name
        self.fileconfig = FakeFileConfig(self._options)

    def get_name(self):
        return self._name

    def getchoice(self, option, choices, default):
        value = self._options.get(option, default)
        if value not in choices:
            raise FakeConfigError(
                "Choice '%s' for option '%s' is not valid" % (value, option)
            )
        return choices[value]

    def getfloat(self, option, default=None):
        return self._options.get(option, default)


def make_mcu(options=None):
    mcu = MCU.__new__(MCU)
    mcu._init_stepping_mode(FakeConfig(options))
    return mcu


def test_phase_transport_defaults_to_the_piece_path():
    assert make_mcu().get_phase_transport() == PHASE_TRANSPORT_PIECE


def test_phase_transport_sample_is_selectable():
    mcu = make_mcu({"phase_transport": "sample"})
    assert mcu.get_phase_transport() == PHASE_TRANSPORT_SAMPLE
    assert mcu.get_stepping_mode() == STEPPING_MODE_PIECE


def test_phase_transport_sample_is_rejected_on_a_stepcompress_mcu():
    with pytest.raises(FakeConfigError, match="needs stepping_mode: piece"):
        make_mcu(
            {
                "phase_transport": "sample",
                "stepping_mode": "stepcompress",
                "stepcompress_sample_rate": 20000.0,
            }
        )


def test_an_unknown_phase_transport_is_rejected():
    with pytest.raises(FakeConfigError):
        make_mcu({"phase_transport": "spline"})


def test_stepcompress_mcu_still_reports_its_encoder():
    mcu = make_mcu(
        {"stepping_mode": "stepcompress", "stepcompress_sample_rate": 20000.0}
    )
    assert mcu.get_stepping_mode() == STEPPING_MODE_STEPCOMPRESS
    assert mcu.get_phase_transport() == PHASE_TRANSPORT_PIECE


class FakePrinter:
    config_error = FakeConfigError


class FakeMotion:
    printer = FakePrinter()


class SampleCapableMcu:
    def __init__(self, missing=()):
        self._missing = set(missing)

    def try_lookup_command(self, msgformat):
        if msgformat in self._missing:
            return None
        return object()


def reject(mcu=None, axes=(0, 1), step_modes=None, endpoints=()):
    if mcu is None:
        mcu = SampleCapableMcu()
    if step_modes is None:
        step_modes = {0: motion_setup.STEP_MODE_MODULATED, 1: 1}
    motion_setup._reject_sample_transport_conflicts(
        FakeMotion(), "mcu", mcu, 11, list(axes), step_modes, set(endpoints)
    )


def test_a_phase_lane_on_capable_firmware_is_accepted():
    reject()


def test_an_ethercat_endpoint_cannot_use_the_sample_transport():
    with pytest.raises(FakeConfigError, match="ethercat"):
        reject(endpoints=(11,))


def test_an_mcu_with_no_phase_lane_is_rejected():
    with pytest.raises(FakeConfigError, match="phase_stepping: 1"):
        reject(step_modes={0: 1, 1: 1})


@pytest.mark.parametrize("missing", SAMPLE_COMMANDS)
def test_firmware_without_a_sample_command_is_rejected(missing):
    with pytest.raises(FakeConfigError, match="CONFIG_SAMPLE_STEPPING"):
        reject(mcu=SampleCapableMcu(missing=(missing,)))


def header_argstrings():
    text = WIRE_HEADER.read_text()
    found = {}
    for match in re.finditer(
        r"#define\s+(SAMPLE_\w+_ARGS)\s+((?:\\\s*\n|[^\n])*)", text
    ):
        name = match.group(1)
        pieces = re.findall(r'"([^"]*)"', match.group(2))
        found[name] = "".join(pieces)
    return found


def rust_argstrings():
    text = WIRE_RUST.read_text()
    found = {}
    for match in re.finditer(
        r"pub const (SAMPLE_\w+): &str =\s*((?:[^;])*);", text
    ):
        name = match.group(1)
        pieces = re.findall(r'"([^"]*)"', match.group(2))
        if pieces:
            found[name] = "".join(pieces)
    return found


PYTHON_ARGSTRINGS = {
    "SAMPLE_ANCHOR": mcu_mod.SAMPLE_ANCHOR_CMD,
    "SAMPLE_RUN": mcu_mod.SAMPLE_RUN_CMD,
    "SAMPLE_OVERLAY": mcu_mod.SAMPLE_OVERLAY_CMD,
    "SAMPLE_GET_POSITION": mcu_mod.SAMPLE_GET_POSITION_CMD,
    "SAMPLE_POSITION": mcu_mod.SAMPLE_POSITION_MSG,
}


@pytest.mark.parametrize("name,argstring", sorted(PYTHON_ARGSTRINGS.items()))
def test_klippy_argstrings_match_the_c_header(name, argstring):
    assert header_argstrings()["%s_ARGS" % (name,)] == argstring


@pytest.mark.parametrize("name,argstring", sorted(PYTHON_ARGSTRINGS.items()))
def test_klippy_argstrings_match_the_rust_contract(name, argstring):
    assert rust_argstrings()[name] == argstring


def test_the_wire_caps_match_the_c_header():
    text = WIRE_HEADER.read_text()
    for name, value in (
        ("SAMPLE_RUN_DATA_MAX", mcu_mod.SAMPLE_RUN_DATA_MAX),
        ("SAMPLE_RUN_COUNT_MAX", mcu_mod.SAMPLE_RUN_COUNT_MAX),
    ):
        match = re.search(r"#define\s+%s\s+(\d+)" % (name,), text)
        assert match is not None, "%s missing from sample_wire.h" % (name,)
        assert int(match.group(1)) == value
