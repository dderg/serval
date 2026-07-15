import json
import math
import struct

import pytest

from klippy.extras import servo_axis, servo_strain_comp


class FakeReactor:
    def __init__(self):
        self._t = 0.0
        self.pauses = []

    def monotonic(self):
        self._t += 0.001
        return self._t

    def pause(self, until):
        self.pauses.append(until - self._t)
        self._t = until


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeToolhead:
    def __init__(self, kin):
        self._kin = kin

    def get_kinematics(self):
        return self._kin

    def wait_moves(self):
        pass


class FakeKin:
    def __init__(self, rails):
        self.rails = rails

    def lanes(self):
        return [
            (i, r.get_name(short=True), []) for i, r in enumerate(self.rails)
        ]

    def coupled_xy(self):
        return True


class FakeNode:
    def __init__(self, name, handle, slots):
        self.name = name
        self._handle = handle
        self._slots = slots

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]


class FakeEngine:
    """set_strain_comp records uploads; sdo_read simulates belt pairs whose
    differential torque responds linearly to the applied constant offset —
    directly on the offset pair (stiffness) and through the gantry on the
    other pair (cross, %/mm)."""

    def __init__(self, stiffness_pct_per_mm=200.0, cross_pct_per_mm=0.0):
        self.stiffness = stiffness_pct_per_mm
        self.cross = cross_pct_per_mm
        self.uploads = []
        self.applied_um = {}

    def motion_drained(self):
        return True

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


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs
        self._reactor = FakeReactor()

    def lookup_object(self, name):
        return self._objs[name]

    def get_reactor(self):
        return self._reactor

    def is_shutdown(self):
        return False


class FakeConfig:
    def __init__(self, printer, map_file):
        self._printer = printer
        self._map_file = map_file

    def get_printer(self):
        return self._printer

    def get(self, name, default=None):
        if name == "map_file":
            return self._map_file
        return default


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None, **kw):
        value = self._params.get(name, default)
        return None if value is None else float(value)

    def get_int(self, name, default=None, **kw):
        return int(self._params.get(name, default))

    def respond_info(self, msg):
        self.responses.append(msg)


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
    node = FakeNode("xy_drives", 7, {"m_a": 0, "m_a1": 1, "m_b": 2, "m_b1": 3})
    printer = FakePrinter(
        {
            "toolhead": FakeToolhead(FakeKin(rails)),
            "motion_engine": engine,
            "ethercat_node xy_drives": node,
            "gcode": FakeGcode(),
        }
    )
    map_file = str(tmp_path / "strain_comp.json")
    sc = servo_strain_comp.ServoStrainComp(FakeConfig(printer, map_file))
    return sc, engine


# --- synthetic strain_map run ------------------------------------------------

CPM = 131072 / 40.0


def elastic_a(x, y):
    """Belt A's field: 0.1 %/mm slope in x around the center."""
    return 0.1 * (x - 50.0)


def elastic_b(x, y):
    """Belt B's field: 0.08 %/mm slope in y around the center."""
    return 0.08 * (y - 50.0)


def write_scap(path, xs, ys, field, field_b=None):
    channels = [
        {"name": "time", "offset": 0, "dtype": "u64"},
        {"name": "target_counts", "offset": 8, "dtype": "i32"},
        {"name": "torque_actual", "offset": 12, "dtype": "i16"},
    ]
    drives = [
        {"name": n, "counts_per_mm": CPM, "invert": False}
        for n in ("m_a", "m_a1", "m_b", "m_b1")
    ]
    record_size = 8 + 6 * len(drives)
    header = {
        "record_size": record_size,
        "drives": drives,
        "channels": channels,
    }
    rows = []
    for i, (x, y) in enumerate(zip(xs, ys)):
        pa, pb = x + y, x - y
        fa = field(x, y)
        fb = field_b(x, y) if field_b else 0.0
        row = struct.pack("<Q", i)
        for pos_mm, diff in ((pa, fa), (pa, -fa), (pb, fb), (pb, -fb)):
            row += struct.pack(
                "<ih", int(round(pos_mm * CPM)), int(round(diff * 10.0))
            )
        rows.append(row)
    with open(path, "wb") as fh:
        fh.write(json.dumps(header).encode() + b"\n" + b"".join(rows))


def write_run(run_dir, field=elastic_a, field_b=None):
    run_dir.mkdir()
    steps = []
    lines = [("xline_y%03d" % y, {"y": float(y)}) for y in (0, 50, 100)]
    lines += [("yline_x%03d" % x, {"x": float(x)}) for x in (0, 50, 100)]
    for name, swept in lines:
        n = 400
        fwd = [i * 100.0 / (n - 1) for i in range(n)]
        sweep = fwd + fwd[::-1]
        if "y" in swept:
            xs, ys = sweep, [swept["y"]] * len(sweep)
        else:
            xs, ys = [swept["x"]] * len(sweep), sweep
        write_scap(run_dir / ("step_%s.scap" % name), xs, ys, field, field_b)
        steps.append({"name": name, "swept": swept})
    manifest = {
        "experiment": "strain_map",
        "kinematics": "corexy",
        "stroke_plan": {
            "x_start": 0.0,
            "x_end": 100.0,
            "y_start": 0.0,
            "y_end": 100.0,
            "line_spacing": 50.0,
            "zero_xy": [50.0, 50.0],
        },
        "belts": "m_a:1+m_a1:1,m_b:1+m_b1:1",
        "steps": steps,
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest))


def test_stiffness_probe_reports_the_gantry_cross_coupling(tmp_path):
    sc, engine = make_comp(
        tmp_path, FakeEngine(stiffness_pct_per_mm=200.0, cross_pct_per_mm=24.0)
    )
    gcmd = FakeGcmd()
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(gcmd)
    cross_lines = [r for r in gcmd.responses if "cross-coupling" in r]
    assert len(cross_lines) == 2
    assert (
        "cross-coupling into belt y: +24.0 %/mm (12% of direct)"
        in (cross_lines[0])
    )


def test_stiffness_probe_stores_the_cross_terms_for_the_build(tmp_path):
    sc, _ = make_comp(
        tmp_path, FakeEngine(stiffness_pct_per_mm=200.0, cross_pct_per_mm=24.0)
    )
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(FakeGcmd())
    a, b = ("m_a", "m_a1"), ("m_b", "m_b1")
    assert sc.measured_cross[(b, a)] == pytest.approx(24.0, rel=0.05)
    assert sc.measured_cross[(a, b)] == pytest.approx(24.0, rel=0.05)


def test_stiffness_measurement_recovers_the_simulated_slope(tmp_path):
    sc, engine = make_comp(tmp_path, FakeEngine(stiffness_pct_per_mm=200.0))
    gcmd = FakeGcmd()
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(gcmd)
    for names in (("m_a", "m_a1"), ("m_b", "m_b1")):
        assert sc.measured_stiffness[names] == pytest.approx(200.0, rel=0.05)
    assert not engine.applied_um, "probe offsets must be cleared afterwards"
    cleared = [u for u in engine.uploads if u[6] == 0]
    assert len(cleared) == 2


def test_build_produces_offsets_that_cancel_the_field(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, engine = make_comp(tmp_path)
    sc.measured_stiffness = {
        ("m_a", "m_a1"): 200.0,
        ("m_b", "m_b1"): 200.0,
    }
    sc.measured_cross = {
        (("m_a", "m_a1"), ("m_b", "m_b1")): 0.0,
        (("m_b", "m_b1"), ("m_a", "m_a1")): 0.0,
    }
    gcmd = FakeGcmd(RUN=str(run_dir))
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a = payload["pairs"][0]
    assert belt_a["motors"] == ["m_a", "m_a1"]
    assert (belt_a["nx"], belt_a["ny"]) == (3, 3)
    grid = belt_a["offsets_um"]
    # elastic +5% at x=100 with 200 %/mm stiffness -> -25 um offset.
    for iy in range(3):
        assert grid[iy * 3 + 0] == pytest.approx(25, abs=3)
        assert grid[iy * 3 + 1] == pytest.approx(0, abs=3)
        assert grid[iy * 3 + 2] == pytest.approx(-25, abs=3)
    belt_b = payload["pairs"][1]
    assert all(abs(v) <= 3 for v in belt_b["offsets_um"])
    assert payload["zero_xy"] == [50.0, 50.0]


def test_build_zeroes_the_map_at_the_recorded_zero_point(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    manifest = json.loads((run_dir / "manifest.json").read_text())
    # Zero recorded off-center: the map must be zero THERE, not at the
    # region center, so applying it after a sync at that spot lines up.
    manifest["stroke_plan"]["zero_xy"] = [100.0, 50.0]
    (run_dir / "manifest.json").write_text(json.dumps(manifest))
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="0",
            CROSS_BA="0",
        )
    )
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a = payload["pairs"][0]
    # elastic at x=100 is +5%; zeroing there shifts the whole map by -5%
    # -> offsets 0 at x=100 and +50 um at x=0.
    grid = belt_a["offsets_um"]
    for iy in range(3):
        assert grid[iy * 3 + 2] == pytest.approx(0, abs=3)
        assert grid[iy * 3 + 0] == pytest.approx(50, abs=4)


def test_build_without_stiffness_errors_loudly(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    with pytest.raises(RuntimeError, match="SERVO_MEASURE_PAIR_STIFFNESS"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run_dir)))


def test_build_accepts_explicit_stiffness_overrides(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="100",
        STIFFNESS_B="100",
        CROSS_AB="0",
        CROSS_BA="0",
    )
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    assert payload["pairs"][0]["offsets_um"][0] == pytest.approx(50, abs=6)


def test_enable_uploads_grids_with_resolved_slots(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, engine = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="0",
            CROSS_BA="0",
        )
    )
    sc.cmd_SERVO_STRAIN_COMP(FakeGcmd(ENABLE="1"))
    grids = [u for u in engine.uploads if u[6] == 3]
    assert len(grids) == 2
    handle, slot_a, slot_b, lane_a, lane_b, kin, nx, ny = grids[0][:8]
    assert (handle, slot_a, slot_b) == (7, 0, 1)
    assert (lane_a, lane_b) == (0, 1)
    assert kin == servo_strain_comp.KIN_COREXY
    assert grids[1][1:3] == (2, 3)


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


def test_excessive_offsets_error_instead_of_clamping(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="5",
        STIFFNESS_B="5",
        CROSS_AB="0",
        CROSS_BA="0",
    )
    with pytest.raises(RuntimeError, match="implausibly low"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


def test_merge_adds_the_residual_on_top_of_the_existing_map(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="0",
            CROSS_BA="0",
        )
    )
    first = json.loads((tmp_path / "strain_comp.json").read_text())
    # The same field measured again as a "residual" and merged: offsets
    # must double (both passes correct the same thing), zero point kept.
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="0",
            CROSS_BA="0",
            MERGE="1",
        )
    )
    merged = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair_first, pair_merged in zip(first["pairs"], merged["pairs"]):
        assert pair_merged["zero_xy"] == pair_first["zero_xy"]
        for a, b in zip(pair_first["offsets_um"], pair_merged["offsets_um"]):
            assert abs(b - 2 * a) <= 2, (a, b)


def test_merge_reuses_the_matrix_recorded_in_the_existing_map(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="210",
            CROSS_AB="-50",
            CROSS_BA="-48",
        )
    )
    first = json.loads((tmp_path / "strain_comp.json").read_text())
    sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run_dir), MERGE="1"))
    merged = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair_first, pair_merged in zip(first["pairs"], merged["pairs"]):
        assert (
            pair_merged["stiffness_pct_per_mm"]
            == pair_first["stiffness_pct_per_mm"]
        )
        assert pair_merged["cross_pct_per_mm"] == pair_first["cross_pct_per_mm"]
        for a, b in zip(pair_first["offsets_um"], pair_merged["offsets_um"]):
            assert abs(b - 2 * a) <= 2, (a, b)


def test_merge_without_an_existing_map_errors_loudly(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    with pytest.raises(RuntimeError, match="MERGE=1 needs an existing map"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(
            FakeGcmd(
                RUN=str(run_dir),
                STIFFNESS_A="200",
                STIFFNESS_B="200",
                CROSS_AB="0",
                CROSS_BA="0",
                MERGE="1",
            )
        )


def test_build_solves_the_cross_coupled_system_jointly(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="-50",
            CROSS_BA="-50",
        )
    )
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a, belt_b = payload["pairs"]
    assert belt_a["stiffness_pct_per_mm"] == 200.0
    assert belt_a["cross_pct_per_mm"] == -50.0
    # The field lives on belt A only (+5% at x=100). Solving
    # [[200, -50], [-50, 200]] @ [o_a, o_b] = -[5, 0] gives
    # o_a = -200*5/37500 mm = -26.7 um and o_b = -50*5/37500 mm = -6.7 um:
    # the OTHER belt gets a same-sign quarter-strength offset, or the
    # correction would leak back through the gantry.
    for iy in range(3):
        assert belt_a["offsets_um"][iy * 3 + 2] == pytest.approx(-27, abs=3)
        assert belt_a["offsets_um"][iy * 3 + 0] == pytest.approx(27, abs=3)
        assert belt_b["offsets_um"][iy * 3 + 2] == pytest.approx(-7, abs=2)
        assert belt_b["offsets_um"][iy * 3 + 0] == pytest.approx(7, abs=2)


def test_probe_then_build_uses_the_measured_matrix(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(
        tmp_path, FakeEngine(stiffness_pct_per_mm=200.0, cross_pct_per_mm=24.0)
    )
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(FakeGcmd())
    sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run_dir)))
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a, belt_b = payload["pairs"]
    # inv([[200, 24], [24, 200]]) @ [5, 0]: o_a = -25.4 um, o_b = +3.0 um —
    # a POSITIVE cross term flips the helper offset's sign.
    for iy in range(3):
        assert belt_a["offsets_um"][iy * 3 + 2] == pytest.approx(-25, abs=3)
        assert belt_b["offsets_um"][iy * 3 + 2] == pytest.approx(3, abs=2)


def test_build_without_cross_terms_errors_loudly(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200")
    with pytest.raises(RuntimeError, match="CROSS_AB"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


def test_build_rejects_a_near_singular_stiffness_matrix(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="200",
        STIFFNESS_B="200",
        CROSS_AB="-200",
        CROSS_BA="-200",
    )
    with pytest.raises(RuntimeError, match="singular"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


def test_build_preserves_diagonal_features_between_lines(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(
        run_dir,
        field=lambda x, y: 4.0 * math.sin(2.0 * math.pi * (x + y) / 40.0),
    )
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir),
            STIFFNESS_A="200",
            STIFFNESS_B="200",
            CROSS_AB="0",
            CROSS_BA="0",
            SPACING="5",
        )
    )
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a = payload["pairs"][0]
    assert (belt_a["nx"], belt_a["ny"]) == (21, 21)

    def offset_at(x, y):
        ix = int(round((x - belt_a["x0"]) / belt_a["dx"]))
        iy = int(round((y - belt_a["y0"]) / belt_a["dy"]))
        return belt_a["offsets_um"][iy * belt_a["nx"] + ix]

    # A 4% belt-A-phase sine (40mm period, the pulley circumference) with
    # 200 %/mm stiffness needs -/+20 um at its extremes. The probed nodes
    # sit BETWEEN the raster lines (lines at 0/50/100): only a model that
    # knows diagonals stay diagonal can fill them in.
    assert offset_at(25.0, 25.0) == pytest.approx(-20, abs=4)  # u=50, sin +1
    assert offset_at(35.0, 35.0) == pytest.approx(20, abs=4)  # u=70, sin -1
    assert offset_at(45.0, 45.0) == pytest.approx(-20, abs=4)  # u=90, sin +1


def test_build_rejects_grids_beyond_the_endpoint_caps(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="200",
        STIFFNESS_B="200",
        CROSS_AB="0",
        CROSS_BA="0",
        SPACING="1",
    )
    with pytest.raises(RuntimeError, match="raise SPACING"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


def build_wrong_scalar_map(sc, baseline):
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(baseline),
            STIFFNESS_A="300",
            STIFFNESS_B="300",
            CROSS_AB="0",
            CROSS_BA="0",
        )
    )


def test_fit_recovers_the_in_use_matrix_from_a_run_pair(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    build_wrong_scalar_map(sc, baseline)

    # The map holds o = -f/300; the true in-use response is
    # [[200, -50], [-50, 200]], so the compensated field is f + K @ o.
    def after_a(x, y):
        return elastic_a(x, y) / 3.0 + elastic_b(x, y) / 6.0

    def after_b(x, y):
        return elastic_b(x, y) / 3.0 + elastic_a(x, y) / 6.0

    verification = tmp_path / "verification"
    write_run(verification, field=after_a, field_b=after_b)
    sc.cmd_SERVO_STRAIN_COMP_FIT(
        FakeGcmd(BASELINE=str(baseline), RUN=str(verification))
    )
    assert sc.measured_stiffness[("m_a", "m_a1")] == pytest.approx(200, abs=10)
    assert sc.measured_stiffness[("m_b", "m_b1")] == pytest.approx(200, abs=10)
    assert sc.measured_cross[
        (("m_a", "m_a1"), ("m_b", "m_b1"))
    ] == pytest.approx(-50, abs=5)
    assert sc.measured_cross[
        (("m_b", "m_b1"), ("m_a", "m_a1"))
    ] == pytest.approx(-50, abs=5)
    # A rebuild with no stiffness params picks the fitted matrix up.
    sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(baseline)))
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair in payload["pairs"]:
        assert pair["stiffness_pct_per_mm"] == pytest.approx(200, abs=10)
        assert pair["cross_pct_per_mm"] == pytest.approx(-50, abs=5)


def test_fit_without_a_map_errors_loudly(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline)
    verification = tmp_path / "verification"
    write_run(verification)
    sc, _ = make_comp(tmp_path)
    with pytest.raises(RuntimeError, match="current map"):
        sc.cmd_SERVO_STRAIN_COMP_FIT(
            FakeGcmd(BASELINE=str(baseline), RUN=str(verification))
        )


def test_fit_needs_offsets_on_both_belts(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline)
    sc, _ = make_comp(tmp_path)
    build_wrong_scalar_map(sc, baseline)
    verification = tmp_path / "verification"
    write_run(verification, field=lambda x, y: elastic_a(x, y) / 3.0)
    with pytest.raises(RuntimeError, match="too little excitation"):
        sc.cmd_SERVO_STRAIN_COMP_FIT(
            FakeGcmd(BASELINE=str(baseline), RUN=str(verification))
        )


def test_fit_rejects_collinear_offsets(tmp_path):
    baseline = tmp_path / "baseline"

    def half_a(x, y):
        return elastic_a(x, y) / 2.0

    write_run(baseline, field=elastic_a, field_b=half_a)
    sc, _ = make_comp(tmp_path)
    build_wrong_scalar_map(sc, baseline)
    verification = tmp_path / "verification"
    write_run(verification, field=elastic_a, field_b=half_a)
    with pytest.raises(RuntimeError, match="collinear"):
        sc.cmd_SERVO_STRAIN_COMP_FIT(
            FakeGcmd(BASELINE=str(baseline), RUN=str(verification))
        )


def test_fit_detects_a_run_captured_without_compensation(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    build_wrong_scalar_map(sc, baseline)

    # The map changed nothing; only an unrelated ripple drifted in.
    def drifted_a(x, y):
        return elastic_a(x, y) + 0.5 * math.sin(2.0 * math.pi * x / 7.0)

    verification = tmp_path / "verification"
    write_run(verification, field=drifted_a, field_b=elastic_b)
    with pytest.raises(RuntimeError, match="not credible"):
        sc.cmd_SERVO_STRAIN_COMP_FIT(
            FakeGcmd(BASELINE=str(baseline), RUN=str(verification))
        )


def write_response_run(run_dir, k_matrix):
    run_dir.mkdir()
    steps = []
    steps_um = [0.0, 50.0, -50.0, 100.0, -100.0]
    for belt_idx in (0, 1):
        for step_idx, value in enumerate(steps_um):
            offsets = [0.0, 0.0]
            offsets[belt_idx] = value / 1000.0

            def resp_a(x, y, o=offsets):
                return (
                    elastic_a(x, y)
                    + k_matrix[0][0] * o[0]
                    + k_matrix[0][1] * o[1]
                )

            def resp_b(x, y, o=offsets):
                return (
                    elastic_b(x, y)
                    + k_matrix[1][0] * o[0]
                    + k_matrix[1][1] * o[1]
                )

            name = "belt%s_step%d" % ("ab"[belt_idx], step_idx)
            n = 400
            fwd = [i * 100.0 / (n - 1) for i in range(n)]
            sweep = fwd + fwd[::-1]
            write_scap(
                run_dir / ("step_%s.scap" % name),
                sweep,
                [50.0] * len(sweep),
                resp_a,
                resp_b,
            )
            steps.append(
                {
                    "name": name,
                    "swept": {"belt": float(belt_idx), "offset_um": value},
                }
            )
    manifest = {
        "experiment": "strain_response",
        "stroke_plan": {
            "response_pairs": [["m_a", "m_a1"], ["m_b", "m_b1"]],
        },
        "steps": steps,
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest))


def test_rolling_response_fit_recovers_the_matrix(tmp_path):
    run_dir = tmp_path / "resp"
    write_response_run(run_dir, [[200.0, -50.0], [-50.0, 200.0]])
    sc, _ = make_comp(tmp_path)
    sc.fit_strain_response(FakeGcmd(), str(run_dir))
    assert sc.measured_stiffness[("m_a", "m_a1")] == pytest.approx(200, abs=3)
    assert sc.measured_stiffness[("m_b", "m_b1")] == pytest.approx(200, abs=3)
    assert sc.measured_cross[
        (("m_a", "m_a1"), ("m_b", "m_b1"))
    ] == pytest.approx(-50, abs=2)
    assert sc.measured_cross[
        (("m_b", "m_b1"), ("m_a", "m_a1"))
    ] == pytest.approx(-50, abs=2)
    # The stored matrix feeds a build with no stiffness params.
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_b)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(baseline)))
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair in payload["pairs"]:
        assert pair["stiffness_pct_per_mm"] == pytest.approx(200, abs=3)
        assert pair["cross_pct_per_mm"] == pytest.approx(-50, abs=2)


def test_rolling_response_fit_rejects_a_flat_response(tmp_path):
    run_dir = tmp_path / "resp"
    write_response_run(run_dir, [[0.0, 0.0], [0.0, 0.0]])
    sc, _ = make_comp(tmp_path)
    with pytest.raises(RuntimeError, match="not credible"):
        sc.fit_strain_response(FakeGcmd(), str(run_dir))


def test_constant_offset_session_applies_slew_aware_and_clears(tmp_path):
    sc, engine = make_comp(tmp_path)
    session = sc.begin_constant_offsets(FakeGcmd())
    assert session.pair_motor_names() == [
        ["m_a", "m_a1"],
        ["m_b", "m_b1"],
    ]
    assert session.apply(0, 100.0) == pytest.approx(0.1)
    assert engine.applied_um[(0, 1)] == 100
    assert session.apply(0, -100.0) == pytest.approx(0.2)
    session.clear()
    assert engine.applied_um == {}


def elastic_bx(x, y):
    """A belt B field that varies along x, so a fixed-y verification line
    exercises both belts' corrections."""
    return 0.06 * (x - 50.0)


def read_map_offset_grids(map_path):
    payload = json.loads(map_path.read_text())
    return [
        {
            "nx": p["nx"],
            "ny": p["ny"],
            "x0": p["x0"],
            "y0": p["y0"],
            "dx": p["dx"],
            "dy": p["dy"],
            "values_pct": [o / 1000.0 for o in p["offsets_um"]],
        }
        for p in payload["pairs"]
    ]


def write_line_scap(path, field, field_b, y):
    n = 400
    fwd = [i * 100.0 / (n - 1) for i in range(n)]
    sweep = fwd + fwd[::-1]
    write_scap(path, sweep, [y] * len(sweep), field, field_b)


def test_tune_converges_the_matrix_against_the_true_response(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_bx)
    sc, _ = make_comp(tmp_path)
    k_true = [[200.0, -50.0], [-50.0, 200.0]]
    # The probe got the SHAPE right but the scale 1.5x off — the loop's
    # job is to converge the scale without another raster.
    tuner = sc.begin_tune(
        FakeGcmd(
            STIFFNESS_A="300",
            STIFFNESS_B="300",
            CROSS_AB="-75",
            CROSS_BA="-75",
        ),
        str(baseline),
        None,
    )
    tune_dir = tmp_path / "tunerun"
    tune_dir.mkdir()
    last = None
    for iteration in range(3):
        tuner.rebuild_and_enable(FakeGcmd())
        grids = read_map_offset_grids(tmp_path / "strain_comp.json")

        def offsets_at(x, y):
            return (
                servo_strain_comp._grid_value_at(grids[0], x, y),
                servo_strain_comp._grid_value_at(grids[1], x, y),
            )

        def measured_a(x, y):
            o_a, o_b = offsets_at(x, y)
            return elastic_a(x, y) + k_true[0][0] * o_a + k_true[0][1] * o_b

        def measured_b(x, y):
            o_a, o_b = offsets_at(x, y)
            return elastic_bx(x, y) + k_true[1][0] * o_a + k_true[1][1] * o_b

        name = "iter%d" % iteration
        write_line_scap(
            tune_dir / ("step_%s.scap" % name), measured_a, measured_b, 50.0
        )
        last = tuner.score_line(FakeGcmd(), str(tune_dir), name, 50.0)
        if all(abs(r["rho"] - 1.0) <= 0.03 for r in last):
            break
        tuner.apply(last)
    assert all(abs(r["rho"] - 1.0) <= 0.03 for r in last)
    (kaa, kab), (kba, kbb) = tuner.matrix_rows()
    assert kaa == pytest.approx(200, rel=0.06)
    assert kbb == pytest.approx(200, rel=0.06)
    assert kab == pytest.approx(-50, rel=0.12)
    assert kba == pytest.approx(-50, rel=0.12)
    # The converged loop rebuilt before scoring, so the map on disk is
    # already the one built with the converged matrix.
    tuner.store_matrix()
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    assert sc.measured_stiffness[("m_a", "m_a1")] == pytest.approx(kaa)
    assert payload["pairs"][0]["stiffness_pct_per_mm"] == pytest.approx(kaa)
    assert payload["pairs"][0]["cross_pct_per_mm"] == pytest.approx(kab)


def test_tune_flags_a_line_that_ignores_the_correction(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_bx)
    sc, _ = make_comp(tmp_path)
    tuner = sc.begin_tune(
        FakeGcmd(
            STIFFNESS_A="300",
            STIFFNESS_B="300",
            CROSS_AB="-75",
            CROSS_BA="-75",
        ),
        str(baseline),
        None,
    )
    tune_dir = tmp_path / "tunerun"
    tune_dir.mkdir()
    tuner.rebuild_and_enable(FakeGcmd())
    write_line_scap(tune_dir / "step_iter0.scap", elastic_a, elastic_bx, 50.0)
    with pytest.raises(RuntimeError, match="not credible"):
        tuner.score_line(FakeGcmd(), str(tune_dir), "iter0", 50.0)


def test_tune_rejects_a_flat_line(tmp_path):
    baseline = tmp_path / "baseline"
    write_run(baseline, field=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    tuner = sc.begin_tune(
        FakeGcmd(
            STIFFNESS_A="300",
            STIFFNESS_B="300",
            CROSS_AB="0",
            CROSS_BA="0",
        ),
        str(baseline),
        None,
    )
    tune_dir = tmp_path / "tunerun"
    tune_dir.mkdir()
    tuner.rebuild_and_enable(FakeGcmd())
    grids = read_map_offset_grids(tmp_path / "strain_comp.json")

    def measured_a(x, y):
        return elastic_a(x, y) + 300.0 * servo_strain_comp._grid_value_at(
            grids[0], x, y
        )

    # elastic_b only varies with y, so along a fixed-y line belt B's map
    # applies no varying correction — unverifiable, fail loudly.
    write_line_scap(tune_dir / "step_iter0.scap", measured_a, elastic_b, 50.0)
    with pytest.raises(RuntimeError, match="no correction"):
        tuner.score_line(FakeGcmd(), str(tune_dir), "iter0", 50.0)
