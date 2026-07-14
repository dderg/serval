import json
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


def no_field(x, y):
    return 0.0


def write_scap(path, xs, ys, field_a, field_b):
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
        fa, fb = field_a(x, y), field_b(x, y)
        row = struct.pack("<Q", i)
        for pos_mm, diff in ((pa, fa), (pa, -fa), (pb, fb), (pb, -fb)):
            row += struct.pack(
                "<ih", int(round(pos_mm * CPM)), int(round(diff * 10.0))
            )
        rows.append(row)
    with open(path, "wb") as fh:
        fh.write(json.dumps(header).encode() + b"\n" + b"".join(rows))


def write_run(run_dir, field_a=elastic_a, field_b=no_field):
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
        write_scap(run_dir / ("step_%s.scap" % name), xs, ys, field_a, field_b)
        steps.append({"name": name, "swept": swept})
    manifest = {
        "experiment": "strain_map",
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


def test_stiffness_probe_stores_the_cross_coupling_for_the_build(tmp_path):
    sc, engine = make_comp(
        tmp_path, FakeEngine(stiffness_pct_per_mm=200.0, cross_pct_per_mm=24.0)
    )
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(FakeGcmd())
    key_ba = (("m_b", "m_b1"), ("m_a", "m_a1"))
    key_ab = (("m_a", "m_a1"), ("m_b", "m_b1"))
    assert sc.measured_cross[key_ba] == pytest.approx(24.0, rel=0.05)
    assert sc.measured_cross[key_ab] == pytest.approx(24.0, rel=0.05)


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


def test_build_solves_the_coupled_pair_response(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="200",
        STIFFNESS_B="200",
        CROSS_AB="-60",
        CROSS_BA="-60",
    )
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    belt_a, belt_b = payload["pairs"]
    assert belt_a["cross_pct_per_mm"] == -60.0
    # Field is 0.1 %/mm in x on belt A only; K = [[200, -60], [-60, 200]],
    # det = 36400. At x=0 (elastic -5%): o_a = +200*5/36400 mm = +27.5 um,
    # and belt B must counter the racking it would otherwise pick up:
    # o_b = -60*5/36400 mm = +8.2 um.
    for iy in range(3):
        assert belt_a["offsets_um"][iy * 3 + 0] == pytest.approx(27, abs=3)
        assert belt_a["offsets_um"][iy * 3 + 2] == pytest.approx(-27, abs=3)
        assert belt_b["offsets_um"][iy * 3 + 0] == pytest.approx(8, abs=3)
        assert belt_b["offsets_um"][iy * 3 + 2] == pytest.approx(-8, abs=3)


def test_build_uses_the_measured_cross_coupling_by_default(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(
        tmp_path, FakeEngine(stiffness_pct_per_mm=200.0, cross_pct_per_mm=24.0)
    )
    sc.cmd_SERVO_MEASURE_PAIR_STIFFNESS(FakeGcmd())
    gcmd = FakeGcmd(RUN=str(run_dir))
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair in payload["pairs"]:
        assert pair["cross_pct_per_mm"] == pytest.approx(24.0, rel=0.05)
    assert not any("independently" in r for r in gcmd.responses)


def test_build_without_cross_coupling_solves_independently(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200")
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    assert any("independently" in r for r in gcmd.responses)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    assert payload["pairs"][0]["cross_pct_per_mm"] == 0.0


def test_build_rejects_a_nearly_singular_stiffness_matrix(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    gcmd = FakeGcmd(
        RUN=str(run_dir),
        STIFFNESS_A="200",
        STIFFNESS_B="200",
        CROSS_AB="190",
        CROSS_BA="190",
    )
    with pytest.raises(RuntimeError, match="nearly singular"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


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
        FakeGcmd(RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200")
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
    gcmd = FakeGcmd(RUN=str(run_dir), STIFFNESS_A="100", STIFFNESS_B="100")
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    assert payload["pairs"][0]["offsets_um"][0] == pytest.approx(50, abs=6)


def test_enable_uploads_grids_with_resolved_slots(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, engine = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200")
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
    gcmd = FakeGcmd(RUN=str(run_dir), STIFFNESS_A="5", STIFFNESS_B="5")
    with pytest.raises(RuntimeError, match="implausibly low"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)


def test_merge_adds_the_residual_on_top_of_the_existing_map(tmp_path):
    run_dir = tmp_path / "strainrun"
    write_run(run_dir)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200")
    )
    first = json.loads((tmp_path / "strain_comp.json").read_text())
    # The same field measured again as a "residual" and merged: offsets
    # must double (both passes correct the same thing), zero point kept.
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(
            RUN=str(run_dir), STIFFNESS_A="200", STIFFNESS_B="200", MERGE="1"
        )
    )
    merged = json.loads((tmp_path / "strain_comp.json").read_text())
    for pair_first, pair_merged in zip(first["pairs"], merged["pairs"]):
        assert pair_merged["zero_xy"] == pair_first["zero_xy"]
        for a, b in zip(pair_first["offsets_um"], pair_merged["offsets_um"]):
            assert abs(b - 2 * a) <= 2, (a, b)


K_TRUE = [[150.0, -40.0], [-30.0, 160.0]]


def make_residual_field(belt_idx, k_used=200.0):
    """The field a rerun would measure with the first map active, when the
    real pair response is K_TRUE rather than the diagonal k_used the map
    was built with."""

    def field(x, y):
        offsets = [-elastic_a(x, y) / k_used, -elastic_b(x, y) / k_used]
        base = (elastic_a, elastic_b)[belt_idx](x, y)
        return base + sum(K_TRUE[belt_idx][j] * offsets[j] for j in range(2))

    return field


def test_merge_identifies_the_effective_response(tmp_path):
    run1 = tmp_path / "buildrun"
    write_run(run1, field_a=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run1), STIFFNESS_A="200", STIFFNESS_B="200")
    )
    run2 = tmp_path / "residualrun"
    write_run(
        run2,
        field_a=make_residual_field(0),
        field_b=make_residual_field(1),
    )
    gcmd = FakeGcmd(RUN=str(run2), MERGE="1")
    sc.cmd_SERVO_STRAIN_COMP_BUILD(gcmd)
    assert any("identified effective response" in r for r in gcmd.responses)
    payload = json.loads((tmp_path / "strain_comp.json").read_text())
    for belt_idx, pair in enumerate(payload["pairs"]):
        assert pair["stiffness_pct_per_mm"] == pytest.approx(
            K_TRUE[belt_idx][belt_idx], abs=10
        )
        assert pair["cross_pct_per_mm"] == pytest.approx(
            K_TRUE[belt_idx][1 - belt_idx], abs=10
        )
    # The merged map must cancel the ORIGINAL field under K_TRUE:
    # offsets = -K_TRUE^-1 * e. At (0, 0), e = (-5, -4).
    det = K_TRUE[0][0] * K_TRUE[1][1] - K_TRUE[0][1] * K_TRUE[1][0]
    expect_a = -(K_TRUE[1][1] * -5.0 - K_TRUE[0][1] * -4.0) / det * 1000.0
    expect_b = -(K_TRUE[0][0] * -4.0 - K_TRUE[1][0] * -5.0) / det * 1000.0
    belt_a, belt_b = payload["pairs"]
    assert belt_a["offsets_um"][0] == pytest.approx(expect_a, abs=4)
    assert belt_b["offsets_um"][0] == pytest.approx(expect_b, abs=4)


def test_merge_identification_requires_the_base_run(tmp_path):
    import shutil

    run1 = tmp_path / "buildrun"
    write_run(run1, field_a=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run1), STIFFNESS_A="200", STIFFNESS_B="200")
    )
    shutil.rmtree(run1)
    run2 = tmp_path / "residualrun"
    write_run(run2, field_a=elastic_a, field_b=elastic_b)
    with pytest.raises(RuntimeError, match="is gone"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run2), MERGE="1"))


def test_merge_identification_needs_offset_excitation(tmp_path):
    run1 = tmp_path / "buildrun"
    write_run(run1, field_a=no_field, field_b=no_field)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run1), STIFFNESS_A="200", STIFFNESS_B="200")
    )
    run2 = tmp_path / "residualrun"
    write_run(run2, field_a=elastic_a, field_b=elastic_b)
    with pytest.raises(RuntimeError, match="too small"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run2), MERGE="1"))


def test_merge_identification_rejects_an_unresponsive_field(tmp_path):
    run1 = tmp_path / "buildrun"
    write_run(run1, field_a=elastic_a, field_b=elastic_b)
    sc, _ = make_comp(tmp_path)
    sc.cmd_SERVO_STRAIN_COMP_BUILD(
        FakeGcmd(RUN=str(run1), STIFFNESS_A="200", STIFFNESS_B="200")
    )
    # Rerun measures the identical field: the map demonstrably did
    # nothing (not enabled?) — merging on top would double-correct.
    run2 = tmp_path / "residualrun"
    write_run(run2, field_a=elastic_a, field_b=elastic_b)
    with pytest.raises(RuntimeError, match="R\\^2"):
        sc.cmd_SERVO_STRAIN_COMP_BUILD(FakeGcmd(RUN=str(run2), MERGE="1"))


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
                MERGE="1",
            )
        )
