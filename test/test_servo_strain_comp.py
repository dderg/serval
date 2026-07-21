import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakePrinter,
    FakeReactor,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd

from klippy.extras import servo_axis, servo_strain_comp


class FakeEngine(_FakeEngine):
    """set_strain_comp records uploads; sdo_read simulates belt pairs whose
    differential torque responds linearly to the applied constant offset —
    directly on the offset pair (stiffness) and through the gantry on the
    other pair (cross, %/mm)."""

    def __init__(self, stiffness_pct_per_mm=200.0, cross_pct_per_mm=0.0):
        super().__init__()
        self.stiffness = stiffness_pct_per_mm
        self.cross = cross_pct_per_mm
        self.uploads = []
        self.applied_um = {}

    def set_strain_comp(self, handle, slot_a, slot_b, *args):
        values = args[-1]
        nx, ny = args[3], args[4]
        self.uploads.append((handle, slot_a, slot_b) + args)
        if nx == 0 or ny == 0:
            self.applied_um.pop((slot_a, slot_b), None)
        elif nx == 1 and ny == 1:
            self.applied_um[(slot_a, slot_b)] = values[0]

    def sdo_read(self, handle, slot, index, subindex):
        mine = (0, 1) if slot in (0, 1) else (2, 3)
        sign = 1.0 if slot == mine[0] else -1.0
        diff_pct = 0.0
        for pair, um in self.applied_um.items():
            gain = self.stiffness if pair == mine else self.cross
            diff_pct += gain * um / 1000.0
        raw = int(round(sign * diff_pct * 10.0)) & 0xFFFF
        return (2, raw)


class FakeGcmd(_FakeGcmd):
    error = RuntimeError

    def get_int(self, name, default=None, **kw):
        return int(self.params.get(name, default))

    def get_float(self, name, default=None, **kw):
        value = self.params.get(name, default)
        return None if value is None else float(value)


def make_motor(name, chain_index):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = "xy_drives"
    m.encoder_counts_per_rev = 131072
    m.rotation_distance = 40.0
    m.invert_direction = False
    m.chain_index = chain_index
    return m


def make_rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def make_comp(tmp_path, engine=None):
    engine = engine or FakeEngine()
    rails = [
        make_rail("x", [make_motor("m_a", 0), make_motor("m_a1", 1)]),
        make_rail("y", [make_motor("m_b", 2), make_motor("m_b1", 3)]),
    ]
    node = FakeNode(
        name="xy_drives",
        handle=7,
        slots={"m_a": 0, "m_a1": 1, "m_b": 2, "m_b1": 3},
    )
    kin = FakeKin(
        rails=rails,
        lanes=[(i, r.get_name(short=True), []) for i, r in enumerate(rails)],
        coupled_xy=True,
    )
    printer = FakePrinter(
        {
            "toolhead": FakeToolhead(kin=kin),
            "motion_engine": engine,
            "ethercat_node xy_drives": node,
            "gcode": FakeGcode(),
        },
        reactor=FakeReactor(tick=0.001),
    )
    map_file = str(tmp_path / "strain_comp.json")
    sc = servo_strain_comp.ServoStrainComp(
        FakeConfig(printer, values={"map_file": map_file})
    )
    return sc, engine


def test_disable_clears_both_pairs(tmp_path):
    sc, engine = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP(FakeGcmd(ENABLE="0"))
    assert [(u[1], u[2], u[6]) for u in engine.uploads] == [
        (0, 1, 0),
        (2, 3, 0),
    ]


def test_enable_without_map_file_errors_loudly(tmp_path):
    sc, _ = make_comp(tmp_path)
    with pytest.raises(RuntimeError, match="SERVO_MEASURE_STRAIN_MAP"):
        sc.cmd_SERVO_STRAIN_COMP(FakeGcmd(ENABLE="1"))
