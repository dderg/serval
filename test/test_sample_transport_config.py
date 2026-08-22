import pathlib
import re

import pytest

from klippy import mcu as mcu_mod
from klippy import motion_setup
from klippy.mcu import (
    MCU,
    SAMPLE_COMMANDS,
    STEPCOMPRESS_MAX_ERROR_DEFAULT,
    STEPCOMPRESS_SAMPLE_RATE_HZ,
)

REPO = pathlib.Path(__file__).resolve().parents[1]
WIRE_HEADER = REPO / "src" / "sample_wire.h"
WIRE_RUST = REPO / "rust" / "runtime" / "src" / "sample_wire.rs"


class FakeConfigError(Exception):
    pass


class FakeFileConfig:
    def __init__(self, options, accessed):
        self._options = options
        self._accessed = accessed

    def has_option(self, _section, option):
        self._accessed.add(option)
        return option in self._options


class FakeConfig:
    error = FakeConfigError

    def __init__(self, options=None, name="mcu"):
        self._options = dict(options or {})
        self._name = name
        self.section = name
        self.accessed = set()
        self.fileconfig = FakeFileConfig(self._options, self.accessed)

    def get_name(self):
        return self._name

    def getchoice(self, option, choices, default):
        self.accessed.add(option)
        value = self._options.get(option, default)
        if value not in choices:
            raise FakeConfigError(
                "Choice '%s' for option '%s' is not valid" % (value, option)
            )
        return choices[value]

    def getfloat(self, option, default=None):
        self.accessed.add(option)
        return self._options.get(option, default)


def make_mcu(options=None):
    mcu = MCU.__new__(MCU)
    mcu._init_stepcompress(FakeConfig(options))
    return mcu


def test_stepcompress_defaults_to_classic_error_budget():
    mcu = make_mcu()
    assert mcu.get_stepcompress_sample_rate() == STEPCOMPRESS_SAMPLE_RATE_HZ
    assert mcu.get_stepcompress_max_error() == STEPCOMPRESS_MAX_ERROR_DEFAULT


DELETED_MCU_KEYS = frozenset(
    [
        "stepping_mode",
        "stepcompress_sample_rate",
        "stepcompress_encoder",
        "phase_transport",
    ]
)


def test_the_deleted_mcu_keys_are_never_consumed():
    config = FakeConfig({"stepcompress_encoder": "hp"})
    mcu = MCU.__new__(MCU)
    mcu._init_stepcompress(config)
    assert config.accessed.isdisjoint(DELETED_MCU_KEYS)
    assert mcu.get_stepcompress_sample_rate() == STEPCOMPRESS_SAMPLE_RATE_HZ


class FakePrinter:
    config_error = FakeConfigError


class FakeMotion:
    printer = FakePrinter()


class SampleCapableMcu:
    def __init__(self, missing=(), constants=None):
        self._missing = set(missing)
        self._constants = (
            {"MOTION_SAMPLE_RATE_HZ": 10000.0, "SAMPLE_RUNS_PER_LANE": 12}
            if constants is None
            else constants
        )

    def try_lookup_command(self, msgformat):
        if msgformat in self._missing:
            return None
        return object()

    def get_constants(self):
        return self._constants


def reject(mcu=None, phase_axes=(0,)):
    if mcu is None:
        mcu = SampleCapableMcu()
    motion_setup._reject_phase_lane_conflicts(
        FakeMotion(), "mcu", mcu, list(phase_axes)
    )


def sample_rate(mcu=None, phase_axes=(0,)):
    if mcu is None:
        mcu = SampleCapableMcu()
    return motion_setup._phase_sample_rate(
        FakeMotion(), "mcu", mcu, list(phase_axes)
    )


def ring_depth(mcu=None, phase_axes=(0,)):
    if mcu is None:
        mcu = SampleCapableMcu()
    return motion_setup._phase_ring_depth(
        FakeMotion(), "mcu", mcu, list(phase_axes)
    )


def test_a_phase_lane_on_capable_firmware_is_accepted():
    reject()


@pytest.mark.parametrize("missing", SAMPLE_COMMANDS)
def test_firmware_without_a_sample_command_is_rejected(missing):
    with pytest.raises(FakeConfigError, match="CONFIG_SAMPLE_STEPPING"):
        reject(mcu=SampleCapableMcu(missing=(missing,)))


def test_the_phase_sample_rate_comes_from_the_advertised_constant():
    assert sample_rate() == 10000.0
    assert (
        sample_rate(SampleCapableMcu(constants={"MOTION_SAMPLE_RATE_HZ": 5000}))
        == 5000.0
    )


def test_a_missing_sample_rate_constant_is_rejected():
    with pytest.raises(FakeConfigError, match="MOTION_SAMPLE_RATE_HZ"):
        sample_rate(SampleCapableMcu(constants={}))


@pytest.mark.parametrize("bad", [0, -5000.0, float("inf")])
def test_a_nonpositive_sample_rate_constant_is_rejected(bad):
    with pytest.raises(FakeConfigError, match="finite positive"):
        sample_rate(SampleCapableMcu(constants={"MOTION_SAMPLE_RATE_HZ": bad}))


def test_the_phase_ring_depth_comes_from_the_advertised_constant():
    assert ring_depth() == 12
    assert (
        ring_depth(SampleCapableMcu(constants={"SAMPLE_RUNS_PER_LANE": 4})) == 4
    )


def test_a_missing_ring_depth_constant_is_rejected():
    with pytest.raises(FakeConfigError, match="SAMPLE_RUNS_PER_LANE"):
        ring_depth(SampleCapableMcu(constants={}))


@pytest.mark.parametrize("bad", [0, -4])
def test_a_nonpositive_ring_depth_constant_is_rejected(bad):
    with pytest.raises(FakeConfigError, match="positive run count"):
        ring_depth(SampleCapableMcu(constants={"SAMPLE_RUNS_PER_LANE": bad}))


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
    "SAMPLE_BARRIER": mcu_mod.SAMPLE_BARRIER_CMD,
    "SAMPLE_BARRIER_ACK": mcu_mod.SAMPLE_BARRIER_ACK_MSG,
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
