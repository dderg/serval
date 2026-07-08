import types

import pytest

from klippy.extras import ethercat_node


class FakeConfigError(Exception):
    pass


class FakeRail:
    def __init__(
        self,
        motor_name,
        chain_index,
        ff_config=(False, 30.0, 0),
        dynamics_profile=None,
    ):
        self._motor_name = motor_name
        self._chain_index = chain_index
        self._ff_config = ff_config
        self._dynamics_profile = dynamics_profile

    def get_motor_name(self):
        return self._motor_name

    def get_chain_index(self):
        return self._chain_index

    def get_ff_config(self):
        return self._ff_config

    def get_dynamics_profile(self):
        return self._dynamics_profile


def _node(rails):
    printer = types.SimpleNamespace(config_error=FakeConfigError)
    node = types.SimpleNamespace(name="node_x", printer=printer)
    return node, sorted(rails, key=lambda pair: pair[0])


def test_validate_chain_accepts_distinct_indices():
    node, rails = _node([(0, FakeRail("x", 0)), (2, FakeRail("y", 1))])
    ethercat_node.EtherCatNode._validate_chain(node, rails)


def test_validate_chain_rejects_duplicate_index():
    node, rails = _node([(0, FakeRail("x", 0)), (1, FakeRail("y", 0))])
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_chain(node, rails)
    assert "share ethercat_chain_index=0" in str(e.value)
    assert "x" in str(e.value) and "y" in str(e.value)


def test_validate_chain_rejects_out_of_range_index():
    bad = ethercat_node.EC_RT_MAX_SLAVES
    node, rails = _node([(0, FakeRail("x", bad))])
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_chain(node, rails)
    assert "exceeds" in str(e.value)


def test_validate_chain_accepts_per_motor_ff_differences():
    node, rails = _node(
        [
            (0, FakeRail("x", 0, ff_config=(False, 30.0, 0))),
            (1, FakeRail("y", 1, ff_config=(True, 60.0, 2))),
        ]
    )
    ethercat_node.EtherCatNode._validate_chain(node, rails)


def _dyn_node(rails, node_profile=None):
    printer = types.SimpleNamespace(config_error=FakeConfigError)
    node = types.SimpleNamespace(
        name="node_x", printer=printer, dynamics_profile=node_profile
    )
    return node, sorted(rails, key=lambda pair: pair[0])


def test_validate_dynamics_none_configured_is_ok():
    node, rails = _dyn_node([(0, FakeRail("x", 0)), (1, FakeRail("y", 1))])
    ethercat_node.EtherCatNode._validate_dynamics_profiles(node, rails)


def test_validate_dynamics_per_servo_all_set_is_ok():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, dynamics_profile="/cfg/x.toml")),
            (1, FakeRail("y", 1, dynamics_profile="/cfg/y.toml")),
        ]
    )
    ethercat_node.EtherCatNode._validate_dynamics_profiles(node, rails)


def test_validate_dynamics_rejects_node_and_per_servo_mix():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, dynamics_profile="/cfg/x.toml")),
            (1, FakeRail("y", 1)),
        ],
        node_profile="/cfg/node.toml",
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_dynamics_profiles(node, rails)
    assert "not both" in str(e.value)


def test_validate_dynamics_rejects_partial_per_servo():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, dynamics_profile="/cfg/x.toml")),
            (1, FakeRail("y", 1)),
        ]
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_dynamics_profiles(node, rails)
    assert "missing on: y" in str(e.value)


class FakeEngine:
    def __init__(self):
        self.calls = []

    def set_torque(self, handle, value, print_time):
        self.calls.append((handle, value, print_time))

    def set_torque_deferred(self, handle, value, print_time):
        self.calls.append((handle, value, print_time))
        return lambda: None


def _torque_node(engine):
    printer = types.SimpleNamespace(
        command_error=FakeConfigError,
        lookup_object=lambda name: engine,
    )
    return types.SimpleNamespace(
        name="node_x",
        printer=printer,
        engine_handle=7,
        _torque_motors=set(),
    )


def test_set_motor_torque_coalesces_enable_and_disable_across_motors():
    engine = FakeEngine()
    node = _torque_node(engine)
    ethercat_node.EtherCatNode.set_motor_torque(node, "x", True, 1.0)
    ethercat_node.EtherCatNode.set_motor_torque(node, "y", True, 1.0)
    # Only the 0->1 transition reaches the node-wide gate; the second motor's
    # enable must NOT issue a second set_torque (that double-enable is -312).
    assert engine.calls == [(7, True, 1.0)]
    ethercat_node.EtherCatNode.set_motor_torque(node, "x", False, 2.0)
    # y still wants torque -> no disable yet.
    assert engine.calls == [(7, True, 1.0)]
    ethercat_node.EtherCatNode.set_motor_torque(node, "y", False, 2.0)
    # Last motor off -> a single node-wide disable.
    assert engine.calls == [(7, True, 1.0), (7, False, 2.0)]


def test_set_motor_torque_without_engine_handle_raises():
    node = _torque_node(FakeEngine())
    node.engine_handle = None
    with pytest.raises(FakeConfigError):
        ethercat_node.EtherCatNode.set_motor_torque(node, "x", True, 1.0)


def test_coupled_uniformity_rejects_mismatched_ff_lead():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, ff_config=(True, 30.0, 2))),
            (1, FakeRail("y", 1, ff_config=(True, 30.0, 0))),
        ],
        node_profile="/cfg/node.toml",
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_coupled_uniformity(node, rails)
    assert "ff_lead_cycles must be identical" in str(e.value)
    assert "x=2" in str(e.value) and "y=0" in str(e.value)


def test_coupled_uniformity_rejects_mismatched_velocity_ff():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, ff_config=(True, 30.0, 0))),
            (1, FakeRail("y", 1, ff_config=(False, 30.0, 0))),
        ],
        node_profile="/cfg/node.toml",
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_coupled_uniformity(node, rails)
    assert "velocity_ff must be identical" in str(e.value)


def test_coupled_uniformity_rejects_mismatched_torque_clamp():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, ff_config=(True, 30.0, 0))),
            (1, FakeRail("y", 1, ff_config=(True, 60.0, 0))),
        ],
        node_profile="/cfg/node.toml",
    )
    with pytest.raises(FakeConfigError) as e:
        ethercat_node.EtherCatNode._validate_coupled_uniformity(node, rails)
    assert "ff_torque_clamp must be identical" in str(e.value)


def test_coupled_uniformity_allows_identical_ff_config():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, ff_config=(True, 30.0, 2))),
            (1, FakeRail("y", 1, ff_config=(True, 30.0, 2))),
        ],
        node_profile="/cfg/node.toml",
    )
    ethercat_node.EtherCatNode._validate_coupled_uniformity(node, rails)


def test_coupled_uniformity_allows_mismatch_on_independent_motors():
    node, rails = _dyn_node(
        [
            (0, FakeRail("x", 0, ff_config=(True, 30.0, 3))),
            (1, FakeRail("y", 1, ff_config=(False, 60.0, 0))),
        ]
    )
    ethercat_node.EtherCatNode._validate_coupled_uniformity(node, rails)


def _awd_kin_printer(node_name):
    from klippy.extras import servo_axis

    def motor(name, node):
        m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
        m.motor_name = name
        m.node_name = node
        return m

    rail_a = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail_a.motors = [motor("motor_a", node_name), motor("motor_a1", node_name)]
    rail_b = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail_b.motors = [motor("motor_b", node_name), motor("motor_b1", "other")]
    kin = types.SimpleNamespace(
        rails=[rail_a, rail_b],
        lanes=lambda: [(0, "x", []), (1, "y", [])],
    )
    toolhead = types.SimpleNamespace(get_kinematics=lambda: kin)
    printer = types.SimpleNamespace(
        lookup_object=lambda name: {"toolhead": toolhead}[name],
        config_error=FakeConfigError,
    )
    return printer


def test_find_motors_returns_every_drive_on_this_node_with_its_lane():
    node = types.SimpleNamespace(
        name="node_x", printer=_awd_kin_printer("node_x")
    )
    found = ethercat_node.EtherCatNode._find_motors(node)
    assert [(lane, m.get_motor_name()) for lane, m in found] == [
        (0, "motor_a"),
        (0, "motor_a1"),
        (1, "motor_b"),
    ]
