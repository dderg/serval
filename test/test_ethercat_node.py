import types

import pytest

from klippy.extras import ethercat_node


class FakeConfigError(Exception):
    pass


class FakeRail:
    def __init__(self, motor_name, chain_index, ff_config=(False, None, 30.0)):
        self._motor_name = motor_name
        self._chain_index = chain_index
        self._ff_config = ff_config

    def get_motor_name(self):
        return self._motor_name

    def get_chain_index(self):
        return self._chain_index

    def get_ff_config(self):
        return self._ff_config


def _node(rails):
    # rails: [(global_axis, FakeRail), ...] sorted by global axis (= slot order).
    printer = types.SimpleNamespace(config_error=FakeConfigError)
    node = types.SimpleNamespace(name="node_x", printer=printer)
    return node, sorted(rails, key=lambda pair: pair[0])


def test_validate_chain_accepts_distinct_indices():
    node, rails = _node([(0, FakeRail("x", 1)), (2, FakeRail("y", 2))])
    # Should not raise.
    ethercat_node.EtherCatNode._validate_chain(node, rails)


def test_validate_chain_rejects_duplicate_index():
    node, rails = _node([(0, FakeRail("x", 1)), (1, FakeRail("y", 1))])
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_chain(node, rails)
    assert "share ethercat_chain_index=1" in str(e.value)
    assert "x" in str(e.value) and "y" in str(e.value)


def test_validate_chain_rejects_out_of_range_index():
    bad = ethercat_node.MAX_CHAIN_INDEX + 1
    node, rails = _node([(0, FakeRail("x", bad))])
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_chain(node, rails)
    assert "exceeds" in str(e.value)


def test_validate_chain_rejects_ff_mismatch():
    node, rails = _node(
        [
            (0, FakeRail("x", 1, ff_config=(False, None, 30.0))),
            (1, FakeRail("y", 2, ff_config=(True, None, 30.0))),
        ]
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_chain(node, rails)
    assert "node-wide" in str(e.value)
