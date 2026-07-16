"""Belt strain compensation: measure the pair stiffness matrix (direct and
cross-belt terms — the shared gantry couples the pairs), build per-belt 2D
offset maps from a strain_map run by solving the joint 2x2 system per grid
node, and feed them to the endpoint's compensation bank (antisymmetric
position offsets keyed on the commanded carriage position)."""

import json
import math
import os

from .. import engine_wait
from . import servo_axis, servo_strokes

MOTION_DRAIN_TIMEOUT = 60.0
TORQUE_ACTUAL_INDEX = 0x6077
KIN_COREXY = 0
KIN_CARTESIAN = 1
MAX_OFFSET_UM = 500
MAX_STRAIN_STEP_UM = 200.0
MIN_LINE_SPACING_MM = 2.0
COMP_SLEW_MM_S = 1.0
TORQUE_READS = 5
BIN_MM = 2.0
FIELD_KNOT_MM = 2.0
FIELD_SMOOTH = 0.5
FIELD_2D_PITCH_MM = 20.0
FIELD_2D_SMOOTH = 2.0
# Mirrors MAX_COMP_GRID_DIM/_VALUES in rust/ethercat-rt/src/strain_comp.rs:
# u16 wire dims; total capped by the endpoint's 8 MB-per-pair grid budget.
MAX_GRID_DIM = 65535
MAX_GRID_VALUES = 1 << 20
FIT_MIN_EXCITATION_MM = 0.005
FIT_MAX_OFFSET_CORR = 0.98
FIT_MIN_R2 = 0.8


def _signed16(raw):
    return raw - 0x10000 if raw >= 0x8000 else raw


class BeltPair:
    def __init__(self, rail, node, kin_tag, lane_a, lane_b):
        self.rail = rail
        self.node = node
        self.kin_tag = kin_tag
        self.lane_a = lane_a
        self.lane_b = lane_b
        self.motors = servo_strokes.rail_motors_in_slot_order(rail)

    def axis_name(self):
        return self.rail.get_name(short=True)

    def motor_names(self):
        return [m.get_motor_name() for m in self.motors]

    def slots(self):
        return [
            self.node.get_slot_for_motor(m.get_motor_name())
            for m in self.motors
        ]

    def mech_signs(self):
        return [-1.0 if m.get_invert_direction() else 1.0 for m in self.motors]


class ConstantOffsetSession:
    """Steps constant antisymmetric offsets through the compensation bank
    for a moving stiffness measurement; apply() reports the slew time the
    change needs before the offset is fully in."""

    def __init__(self, comp, gcmd):
        self._comp = comp
        self._engine = comp.printer.lookup_object("motion_engine")
        self._pairs = comp._belt_pairs(gcmd)
        self._handles = [comp._node_handle(gcmd, p.node) for p in self._pairs]
        self._applied_um = [0.0] * len(self._pairs)

    def pair_count(self):
        return len(self._pairs)

    def pair_motor_names(self):
        return [pair.motor_names() for pair in self._pairs]

    def apply(self, belt_idx, value_um):
        self._comp._upload_constant(
            self._engine,
            self._handles[belt_idx],
            self._pairs[belt_idx],
            value_um,
        )
        slew_s = (
            abs(value_um - self._applied_um[belt_idx]) / 1000.0 / COMP_SLEW_MM_S
        )
        self._applied_um[belt_idx] = value_um
        return slew_s

    def clear(self):
        for handle, pair in zip(self._handles, self._pairs):
            self._comp._clear_pair(self._engine, handle, pair)


TUNE_RHO_MIN = 0.2
TUNE_RHO_MAX = 5.0


class StrainCompTune:
    """One closed-loop stiffness tune: rebuild the FULL map from the
    baseline run at the trial matrix (never merging), verify it along a
    single line, and scale each belt's matrix row by the fraction of the
    intended correction the line actually shows. When the line reads
    flat the matrix on disk is right for the whole map — it is the same
    map that was being verified."""

    def __init__(self, comp, gcmd, run_dir, manifest, spacing, k_matrix):
        self.comp = comp
        self.run_dir = run_dir
        self.manifest = manifest
        self.plan = manifest["stroke_plan"]
        self.spacing = (
            spacing if spacing is not None else self.plan["line_spacing"]
        )
        self.k_matrix = [list(row) for row in k_matrix]
        self.belts = _belt_motor_names(manifest)
        kinematics = manifest.get("kinematics")
        if kinematics is None:
            raise gcmd.error("manifest has no kinematics field")
        corexy = kinematics == "corexy"
        samples = _collect_elastic_samples(run_dir, manifest)
        self.baseline_models = []
        for belt_idx in range(2):
            if not samples[belt_idx]:
                raise gcmd.error("run contains no usable elastic samples")
            bx, by, bv = _flatten_lines(samples[belt_idx])
            self.baseline_models.append(
                _field_model_fit(bx, by, bv, self.plan, corexy)
            )

    def matrix_rows(self):
        return [list(row) for row in self.k_matrix]

    def enable_ramp_s(self):
        return MAX_OFFSET_UM / 1000.0 / COMP_SLEW_MM_S

    def rebuild_and_enable(self, gcmd):
        self.comp._build_and_write(
            gcmd,
            self.run_dir,
            self.manifest,
            self.spacing,
            [tuple(row) for row in self.k_matrix],
            quiet=True,
        )
        self.comp.enable_from_map(gcmd, quiet=True)

    def score_line(self, gcmd, capture_run_dir, step_name, line_y):
        """Per belt: the verification line's residual rms and the achieved
        fraction rho of the intended correction — the row-scale update."""
        import numpy as np

        mini_manifest = {
            "stroke_plan": self.plan,
            "belts": self.manifest["belts"],
            "steps": [{"name": step_name, "swept": {"y": line_y}}],
        }
        samples = _collect_elastic_samples(capture_run_dir, mini_manifest)
        offset_grids = self.comp._load_map_offset_grids(
            gcmd, self.belts, "no map at %s" % self.comp.map_file
        )
        results = []
        for belt_idx in range(2):
            if not samples[belt_idx]:
                raise gcmd.error(
                    "verification line has no usable samples for belt %s"
                    % "AB"[belt_idx]
                )
            rx, ry, rv = _flatten_lines(samples[belt_idx])
            baseline = self.baseline_models[belt_idx](rx, ry)
            delta = rv - baseline
            own = np.array(
                [
                    _grid_value_at(offset_grids[belt_idx], px, py)
                    for px, py in zip(rx, ry)
                ]
            )
            other = np.array(
                [
                    _grid_value_at(offset_grids[1 - belt_idx], px, py)
                    for px, py in zip(rx, ry)
                ]
            )
            row = self.k_matrix[belt_idx]
            if belt_idx == 0:
                predicted = row[0] * own + row[1] * other
            else:
                predicted = row[0] * other + row[1] * own
            predicted = predicted - predicted.mean()
            delta = delta - delta.mean()
            denom = float((predicted**2).sum())
            if denom < 1e-9:
                raise gcmd.error(
                    "the map applies no correction to belt %s along the "
                    "verification line — pick a Y where the field varies"
                    % "AB"[belt_idx]
                )
            rho = float((delta * predicted).sum() / denom)
            if not (TUNE_RHO_MIN < rho < TUNE_RHO_MAX):
                raise gcmd.error(
                    "belt %s achieved %.0f%% of the intended correction — "
                    "not credible; is the compensation actually applied?"
                    % ("AB"[belt_idx], rho * 100.0)
                )
            residual = rv - rv.mean()
            baseline_c = baseline - baseline.mean()
            results.append(
                {
                    "rho": rho,
                    "rms": float(np.sqrt(np.mean(residual**2))),
                    "base_rms": float(np.sqrt(np.mean(baseline_c**2))),
                }
            )
        return results

    def apply(self, results):
        for belt_idx, result in enumerate(results):
            self.k_matrix[belt_idx] = [
                value * result["rho"] for value in self.k_matrix[belt_idx]
            ]

    def store_matrix(self):
        (kaa, kab), (kba, kbb) = self.k_matrix
        belt_a, belt_b = tuple(self.belts[0]), tuple(self.belts[1])
        self.comp.measured_stiffness[belt_a] = kaa
        self.comp.measured_stiffness[belt_b] = kbb
        self.comp.measured_cross[(belt_a, belt_b)] = kab
        self.comp.measured_cross[(belt_b, belt_a)] = kba


class ServoStrainComp:
    cmd_SERVO_MEASURE_PAIR_STIFFNESS_help = (
        "Measure the belt stiffness matrix PARKED: step a known "
        "antisymmetric offset through the compensation bank and read every "
        "pair's differential torque response — the direct slope (%/mm) "
        "plus the cross-belt slope through the shared gantry. A parked "
        "belt reads stiffer than one rolling over the pulleys (~20% on "
        "the bench); the map operates rolling, so prefer "
        "SERVO_MEASURE_STRAIN_RESPONSE and keep this as a sanity check. "
        "STEP_UM (50) sets the probe amplitude, SETTLE (0.8) the wait per "
        "step, AXIS=X|Y limits to one pair."
    )
    cmd_SERVO_STRAIN_COMP_BUILD_help = (
        "Build the compensation map from a SERVO_MEASURE_STRAIN_MAP run: "
        "grid the elastic differential field per belt, solve the 2x2 "
        "stiffness system per grid node (an offset on one belt also "
        "strains the other through the gantry), write the map file. "
        "RUN=<run dir> is required; the stiffness matrix comes from "
        "SERVO_MEASURE_PAIR_STIFFNESS or STIFFNESS_A/STIFFNESS_B with "
        "CROSS_AB/CROSS_BA (%/mm, CROSS_AB = belt A's response to a belt "
        "B offset, 0 disables the cross term). The field is fit through "
        "a structured model (belt-phase diagonals on CoreXY, axis-locked "
        "1D terms, smooth 2D remainder) so pulley-period diagonal "
        "features survive between raster lines; SPACING sets the output "
        "grid pitch (defaults to the run's line spacing; 5 recommended "
        "on CoreXY so 40mm pulley harmonics survive the bilinear "
        "lookup). MERGE=1 treats the run as a residual measured WITH the "
        "current map enabled and adds the correction on top — the "
        "second-order iteration; it reuses the stiffness matrix recorded "
        "in the existing map unless overridden."
    )
    cmd_SERVO_STRAIN_COMP_FIT_help = (
        "Fit the stiffness matrix in the regime the map operates in: "
        "BASELINE=<strain_map run without compensation>, RUN=<strain_map "
        "run captured WITH the current map enabled>. The field change "
        "between the runs, regressed against the offsets the map applied "
        "at each sample point, is the in-use response — direct and cross "
        "terms in one least-squares (the parked probe reads high). The "
        "fitted matrix is stored for the next SERVO_STRAIN_COMP_BUILD."
    )
    cmd_SERVO_STRAIN_COMP_help = (
        "ENABLE=1 uploads the map file to the endpoint (offsets ramp in at "
        "1 mm/s); ENABLE=0 clears the compensation."
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.map_file = os.path.expanduser(
            config.get("map_file", "~/printer_data/config/strain_comp.json")
        )
        self.measured_stiffness = {}
        self.measured_cross = {}
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_MEASURE_PAIR_STIFFNESS",
            self.cmd_SERVO_MEASURE_PAIR_STIFFNESS,
            desc=self.cmd_SERVO_MEASURE_PAIR_STIFFNESS_help,
        )
        gcode.register_command(
            "SERVO_STRAIN_COMP_BUILD",
            self.cmd_SERVO_STRAIN_COMP_BUILD,
            desc=self.cmd_SERVO_STRAIN_COMP_BUILD_help,
        )
        gcode.register_command(
            "SERVO_STRAIN_COMP_FIT",
            self.cmd_SERVO_STRAIN_COMP_FIT,
            desc=self.cmd_SERVO_STRAIN_COMP_FIT_help,
        )
        gcode.register_command(
            "SERVO_STRAIN_COMP",
            self.cmd_SERVO_STRAIN_COMP,
            desc=self.cmd_SERVO_STRAIN_COMP_help,
        )

    def _belt_pairs(self, gcmd, axis_filter=None):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        lanes = [
            (lane_idx, kin.rails[lane_idx])
            for lane_idx, _axis_name, _motors in kin.lanes()
            if isinstance(kin.rails[lane_idx], servo_axis.ServoRail)
            and len(kin.rails[lane_idx].get_motors()) == 2
            and kin.rails[lane_idx].get_name(short=True) != "z"
        ]
        if len(lanes) != 2:
            raise gcmd.error(
                "strain compensation needs exactly two dual-drive belt "
                "axes, found %d" % len(lanes)
            )
        kin_tag = KIN_COREXY if kin.coupled_xy() else KIN_CARTESIAN
        lane_a, lane_b = lanes[0][0], lanes[1][0]
        pairs = []
        for lane_idx, rail in lanes:
            if (
                axis_filter is not None
                and rail.get_name(short=True) != axis_filter
            ):
                continue
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            pairs.append(BeltPair(rail, node, kin_tag, lane_a, lane_b))
        if not pairs:
            raise gcmd.error(
                "no belt pair matching AXIS=%s" % axis_filter.upper()
            )
        return pairs

    def _node_handle(self, gcmd, node):
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % node.name
            )
        return handle

    def _drain_motion(self, gcmd, engine):
        try:
            engine_wait.wait_for(
                self.printer,
                lambda: engine.motion_drained() or None,
                "strain comp motion drain",
                MOTION_DRAIN_TIMEOUT,
            )
        except engine_wait.EngineWaitTimeout:
            raise gcmd.error(
                "motion did not drain within %.0fs" % MOTION_DRAIN_TIMEOUT
            )

    def _read_diff_pct(self, engine, handle, pair):
        reactor = self.printer.get_reactor()
        slots = pair.slots()
        signs = pair.mech_signs()
        total = 0.0
        for _ in range(TORQUE_READS):
            mech = []
            for slot, sign in zip(slots, signs):
                _size, raw = engine.sdo_read(
                    handle, slot, TORQUE_ACTUAL_INDEX, 0
                )
                mech.append(_signed16(raw) / 10.0 * sign)
            total += (mech[0] - mech[1]) / 2.0
            reactor.pause(reactor.monotonic() + 0.02)
        return total / TORQUE_READS

    def _upload_constant(self, engine, handle, pair, value_um):
        engine.set_strain_comp(
            handle,
            pair.slots()[0],
            pair.slots()[1],
            pair.lane_a,
            pair.lane_b,
            pair.kin_tag,
            1,
            1,
            0.0,
            0.0,
            1.0,
            1.0,
            [int(value_um)],
        )

    def _clear_pair(self, engine, handle, pair):
        engine.set_strain_comp(
            handle,
            pair.slots()[0],
            pair.slots()[1],
            pair.lane_a,
            pair.lane_b,
            pair.kin_tag,
            0,
            0,
            0.0,
            0.0,
            1.0,
            1.0,
            [],
        )

    def cmd_SERVO_MEASURE_PAIR_STIFFNESS(self, gcmd):
        axis_filter = gcmd.get("AXIS", None)
        if axis_filter is not None:
            axis_filter = axis_filter.lower()
        step_um = gcmd.get_float(
            "STEP_UM", 50.0, above=0.0, maxval=MAX_STRAIN_STEP_UM
        )
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0)
        pairs = self._belt_pairs(gcmd, axis_filter)
        all_pairs = self._belt_pairs(gcmd, None)
        toolhead = self.printer.lookup_object("toolhead")
        engine = self.printer.lookup_object("motion_engine")
        reactor = self.printer.get_reactor()
        toolhead.wait_moves()
        self._drain_motion(gcmd, engine)
        for pair in pairs:
            handle = self._node_handle(gcmd, pair.node)
            steps_um = [0.0, step_um, -step_um, 2.0 * step_um, -2.0 * step_um]
            # An offset on one pair racks the gantry a little, so the other
            # belt sees it too — read every pair at every step and report
            # the cross-coupling next to the direct stiffness.
            points = {obs.axis_name(): [] for obs in all_pairs}
            try:
                for value_um in steps_um:
                    self._upload_constant(engine, handle, pair, value_um)
                    slew_s = abs(value_um) / 1000.0 / COMP_SLEW_MM_S
                    reactor.pause(reactor.monotonic() + settle + slew_s)
                    for obs in all_pairs:
                        obs_handle = self._node_handle(gcmd, obs.node)
                        diff = self._read_diff_pct(engine, obs_handle, obs)
                        points[obs.axis_name()].append(
                            (value_um / 1000.0, diff)
                        )
            finally:
                self._clear_pair(engine, handle, pair)
            ramp_out_s = 2.0 * step_um / 1000.0 / COMP_SLEW_MM_S
            reactor.pause(reactor.monotonic() + settle + ramp_out_s)
            restored = self._read_diff_pct(engine, handle, pair)
            slope, r2 = _fit_slope(points[pair.axis_name()])
            names = pair.motor_names()
            self.measured_stiffness[tuple(names)] = slope
            gcmd.respond_info(
                "belt %s (%s/%s): stiffness %.1f %%/mm (R^2 %.3f, "
                "points %s; restored to %+.2f%%)"
                % (
                    pair.axis_name(),
                    names[0],
                    names[1],
                    slope,
                    r2,
                    " ".join(
                        "%+.0fum:%+.2f%%" % (x * 1000, y)
                        for x, y in points[pair.axis_name()]
                    ),
                    restored,
                )
            )
            for obs in all_pairs:
                if obs.axis_name() == pair.axis_name():
                    continue
                cross, _cross_r2 = _fit_slope(points[obs.axis_name()])
                obs_key = tuple(obs.motor_names())
                self.measured_cross[(obs_key, tuple(names))] = cross
                gcmd.respond_info(
                    "  cross-coupling into belt %s: %+.1f %%/mm "
                    "(%.0f%% of direct)"
                    % (
                        obs.axis_name(),
                        cross,
                        abs(cross / slope) * 100.0 if slope else 0.0,
                    )
                )
            if abs(slope) < 1e-6 or r2 < 0.9:
                raise gcmd.error(
                    "stiffness fit for belt %s is not credible "
                    "(slope %.3f %%/mm, R^2 %.3f) — is torque enabled and "
                    "the axis at standstill?" % (pair.axis_name(), slope, r2)
                )

    def _stiffness_for(self, gcmd, belt_idx, motor_names, override):
        if override is not None:
            return override
        key = tuple(motor_names)
        if key in self.measured_stiffness:
            return self.measured_stiffness[key]
        reversed_key = tuple(reversed(motor_names))
        if reversed_key in self.measured_stiffness:
            return -self.measured_stiffness[reversed_key]
        raise gcmd.error(
            "no stiffness for belt %s (%s) — run "
            "SERVO_MEASURE_PAIR_STIFFNESS first or pass STIFFNESS_%s=<%%/mm>"
            % ("AB"[belt_idx], "/".join(motor_names), "AB"[belt_idx])
        )

    def _cross_for(self, gcmd, belt_idx, belts, override):
        """k[belt_idx][other]: this belt's differential torque response per
        mm of the OTHER belt's antisymmetric offset. Reversing either
        pair's motor order flips the sign."""
        if override is not None:
            return override
        obs = tuple(belts[belt_idx])
        drv = tuple(belts[1 - belt_idx])
        for obs_key, obs_sign in ((obs, 1.0), (tuple(reversed(obs)), -1.0)):
            for drv_key, drv_sign in (
                (drv, 1.0),
                (tuple(reversed(drv)), -1.0),
            ):
                if (obs_key, drv_key) in self.measured_cross:
                    return (
                        obs_sign
                        * drv_sign
                        * self.measured_cross[(obs_key, drv_key)]
                    )
        param = "CROSS_%s%s" % ("AB"[belt_idx], "AB"[1 - belt_idx])
        raise gcmd.error(
            "no cross stiffness for belt %s (%s) — run "
            "SERVO_MEASURE_PAIR_STIFFNESS first or pass %s=<%%/mm> "
            "(0 disables the cross term)"
            % ("AB"[belt_idx], "/".join(obs), param)
        )

    def cmd_SERVO_STRAIN_COMP_BUILD(self, gcmd):
        run_dir, manifest = _strain_map_run(gcmd, gcmd.get("RUN"))
        merge = gcmd.get_int("MERGE", 0, minval=0, maxval=1) == 1
        base_pairs = {}
        if merge:
            if not os.path.exists(self.map_file):
                raise gcmd.error(
                    "MERGE=1 needs an existing map at %s" % self.map_file
                )
            with open(self.map_file) as fh:
                base_pairs = {
                    tuple(p["motors"]): p for p in json.load(fh)["pairs"]
                }
        plan = manifest["stroke_plan"]
        spacing = gcmd.get_float(
            "SPACING", plan["line_spacing"], minval=MIN_LINE_SPACING_MM
        )
        stiffness_overrides = [
            gcmd.get_float("STIFFNESS_A", None),
            gcmd.get_float("STIFFNESS_B", None),
        ]
        cross_overrides = [
            gcmd.get_float("CROSS_AB", None),
            gcmd.get_float("CROSS_BA", None),
        ]
        belts = _belt_motor_names(manifest)
        if len(belts) != 2:
            raise gcmd.error(
                "the joint stiffness solve needs exactly two belts, run "
                "has %d" % len(belts)
            )
        for belt_idx, motor_names in enumerate(belts):
            base = base_pairs.get(tuple(motor_names))
            if base is None:
                continue
            if stiffness_overrides[belt_idx] is None:
                stiffness_overrides[belt_idx] = base.get("stiffness_pct_per_mm")
            if cross_overrides[belt_idx] is None:
                cross_overrides[belt_idx] = base.get("cross_pct_per_mm")
        k_matrix = self._resolve_k_matrix(
            gcmd, belts, stiffness_overrides, cross_overrides
        )
        self._build_and_write(
            gcmd, run_dir, manifest, spacing, k_matrix, merge, base_pairs
        )
        gcmd.respond_info(
            "strain compensation map written to %s — apply it with "
            "SERVO_STRAIN_COMP ENABLE=1" % self.map_file
        )

    def _resolve_k_matrix(
        self, gcmd, belts, stiffness_overrides, cross_overrides
    ):
        direct = [
            self._stiffness_for(
                gcmd, belt_idx, motor_names, stiffness_overrides[belt_idx]
            )
            for belt_idx, motor_names in enumerate(belts)
        ]
        cross = [
            self._cross_for(gcmd, belt_idx, belts, cross_overrides[belt_idx])
            for belt_idx in range(2)
        ]
        return [(direct[0], cross[0]), (cross[1], direct[1])]

    def _build_and_write(
        self,
        gcmd,
        run_dir,
        manifest,
        spacing,
        k_matrix,
        merge=False,
        base_pairs=None,
        quiet=False,
    ):
        base_pairs = base_pairs or {}
        plan = manifest["stroke_plan"]
        belts = _belt_motor_names(manifest)
        direct = [k_matrix[0][0], k_matrix[1][1]]
        cross = [k_matrix[0][1], k_matrix[1][0]]
        kinematics = manifest.get("kinematics")
        if kinematics is None:
            raise gcmd.error("manifest has no kinematics field")
        corexy = kinematics == "corexy"
        samples = _collect_elastic_samples(run_dir, manifest)
        grids = []
        bases = []
        for belt_idx, motor_names in enumerate(belts):
            grid = _build_grid(gcmd, plan, spacing, samples[belt_idx], corexy)
            base = base_pairs.get(tuple(motor_names))
            if merge:
                if base is None:
                    raise gcmd.error(
                        "existing map has no entry for belt %s (%s)"
                        % ("AB"[belt_idx], "/".join(motor_names))
                    )
                _rezero_grid_at(grid, base["zero_xy"])
                grid["zero_xy"] = list(base["zero_xy"])
            grids.append(grid)
            bases.append(base)
        belt_offsets = _offsets_from_grids(gcmd, grids, k_matrix)
        pairs_out = []
        for belt_idx, motor_names in enumerate(belts):
            grid = grids[belt_idx]
            offsets = belt_offsets[belt_idx]
            if merge:
                offsets = _merge_offsets(gcmd, grid, offsets, bases[belt_idx])
            pairs_out.append(
                {
                    "motors": motor_names,
                    "stiffness_pct_per_mm": direct[belt_idx],
                    "cross_pct_per_mm": cross[belt_idx],
                    "nx": grid["nx"],
                    "ny": grid["ny"],
                    "x0": grid["x0"],
                    "y0": grid["y0"],
                    "dx": grid["dx"],
                    "dy": grid["dy"],
                    "offsets_um": offsets,
                    "zero_xy": grid["zero_xy"],
                }
            )
            if not quiet:
                gcmd.respond_info(
                    "belt %s (%s): %dx%d grid, offsets %+d..%+d um "
                    "(stiffness %.1f, cross %.1f %%/mm)"
                    % (
                        "AB"[belt_idx],
                        "/".join(motor_names),
                        grid["nx"],
                        grid["ny"],
                        min(offsets),
                        max(offsets),
                        direct[belt_idx],
                        cross[belt_idx],
                    )
                )
        payload = {
            "version": 1,
            "run": run_dir,
            "zero_xy": pairs_out[0]["zero_xy"],
            "pairs": pairs_out,
        }
        tmp_path = self.map_file + ".tmp"
        with open(tmp_path, "w") as fh:
            json.dump(payload, fh)
        os.replace(tmp_path, self.map_file)

    def cmd_SERVO_STRAIN_COMP_FIT(self, gcmd):
        baseline_dir, baseline_manifest = _strain_map_run(
            gcmd, gcmd.get("BASELINE")
        )
        run_dir, run_manifest = _strain_map_run(gcmd, gcmd.get("RUN"))
        if baseline_manifest.get("belts") != run_manifest.get("belts"):
            raise gcmd.error("BASELINE and RUN have different belt layouts")
        if baseline_manifest.get("kinematics") != run_manifest.get(
            "kinematics"
        ):
            raise gcmd.error("BASELINE and RUN have different kinematics")
        kinematics = run_manifest.get("kinematics")
        if kinematics is None:
            raise gcmd.error("manifest has no kinematics field")
        corexy = kinematics == "corexy"
        belts = _belt_motor_names(run_manifest)
        if len(belts) != 2:
            raise gcmd.error(
                "the matrix fit needs exactly two belts, run has %d"
                % len(belts)
            )
        offset_grids = self._load_map_offset_grids(
            gcmd,
            belts,
            "no map at %s — RUN must be captured with the current map "
            "enabled" % self.map_file,
        )
        base_samples = _collect_elastic_samples(baseline_dir, baseline_manifest)
        run_samples = _collect_elastic_samples(run_dir, run_manifest)
        plan = baseline_manifest["stroke_plan"]
        for belt_idx, motor_names in enumerate(belts):
            k_own, k_cross, r2, n = _fit_in_use_response(
                gcmd,
                belt_idx,
                base_samples[belt_idx],
                run_samples[belt_idx],
                plan,
                corexy,
                offset_grids,
            )
            self.measured_stiffness[tuple(motor_names)] = k_own
            self.measured_cross[
                (tuple(motor_names), tuple(belts[1 - belt_idx]))
            ] = k_cross
            gcmd.respond_info(
                "belt %s (%s): in-use stiffness %.1f %%/mm, cross %.1f "
                "%%/mm (R^2 %.3f, %d samples)"
                % (
                    "AB"[belt_idx],
                    "/".join(motor_names),
                    k_own,
                    k_cross,
                    r2,
                    n,
                )
            )
        gcmd.respond_info(
            "in-use matrix stored — rebuild with SERVO_STRAIN_COMP_BUILD "
            "RUN=%s to apply it" % baseline_dir
        )

    def _load_map_offset_grids(self, gcmd, belts, missing):
        if not os.path.exists(self.map_file):
            raise gcmd.error(missing)
        with open(self.map_file) as fh:
            map_pairs = {tuple(p["motors"]): p for p in json.load(fh)["pairs"]}
        offset_grids = []
        for belt_idx, motor_names in enumerate(belts):
            pair = map_pairs.get(tuple(motor_names))
            if pair is None:
                raise gcmd.error(
                    "map has no entry for belt %s (%s)"
                    % ("AB"[belt_idx], "/".join(motor_names))
                )
            offset_grids.append(
                {
                    "nx": pair["nx"],
                    "ny": pair["ny"],
                    "x0": pair["x0"],
                    "y0": pair["y0"],
                    "dx": pair["dx"],
                    "dy": pair["dy"],
                    "values_pct": [o / 1000.0 for o in pair["offsets_um"]],
                }
            )
        return offset_grids

    def begin_constant_offsets(self, gcmd):
        return ConstantOffsetSession(self, gcmd)

    def begin_tune(self, gcmd, run_dir_raw, spacing_param):
        run_dir, manifest = _strain_map_run(gcmd, run_dir_raw)
        belts = _belt_motor_names(manifest)
        if len(belts) != 2:
            raise gcmd.error(
                "the joint stiffness solve needs exactly two belts, run "
                "has %d" % len(belts)
            )
        k_matrix = self._resolve_k_matrix(
            gcmd,
            belts,
            [
                gcmd.get_float("STIFFNESS_A", None),
                gcmd.get_float("STIFFNESS_B", None),
            ],
            [
                gcmd.get_float("CROSS_AB", None),
                gcmd.get_float("CROSS_BA", None),
            ],
        )
        return StrainCompTune(
            self, gcmd, run_dir, manifest, spacing_param, k_matrix
        )

    def fit_strain_response(self, gcmd, run_dir):
        manifest_path = os.path.join(run_dir, "manifest.json")
        with open(manifest_path) as fh:
            manifest = json.load(fh)
        if manifest.get("experiment") != "strain_response":
            raise gcmd.error(
                "%s is a %r run, need a strain_response run"
                % (run_dir, manifest.get("experiment"))
            )
        pair_names = [
            tuple(names) for names in manifest["stroke_plan"]["response_pairs"]
        ]
        if len(pair_names) != 2:
            raise gcmd.error(
                "the response fit needs exactly two belts, run has %d"
                % len(pair_names)
            )
        points = {(obs, drv): [] for obs in range(2) for drv in range(2)}
        for step in manifest["steps"]:
            drv = int(step["swept"]["belt"])
            offset_mm = step["swept"]["offset_um"] / 1000.0
            path = os.path.join(run_dir, "step_%s.scap" % step["name"])
            try:
                means = _rolling_elastic_means(path, pair_names)
            except ValueError as e:
                raise gcmd.error(str(e))
            for obs in range(2):
                points[(obs, drv)].append((offset_mm, means[obs]))
        for drv in range(2):
            slope, r2 = _fit_slope(points[(drv, drv)])
            if abs(slope) < 1e-6 or r2 < 0.9:
                raise gcmd.error(
                    "rolling stiffness fit for belt %s is not credible "
                    "(slope %.3f %%/mm, R^2 %.3f)" % ("AB"[drv], slope, r2)
                )
            obs = 1 - drv
            cross, _cross_r2 = _fit_slope(points[(obs, drv)])
            self.measured_stiffness[pair_names[drv]] = slope
            self.measured_cross[(pair_names[obs], pair_names[drv])] = cross
            gcmd.respond_info(
                "belt %s (%s): rolling stiffness %.1f %%/mm, cross into "
                "belt %s %+.1f %%/mm (R^2 %.3f)"
                % (
                    "AB"[drv],
                    "/".join(pair_names[drv]),
                    slope,
                    "AB"[obs],
                    cross,
                    r2,
                )
            )
        gcmd.respond_info(
            "rolling matrix stored — it feeds the next SERVO_STRAIN_COMP_BUILD"
        )

    def cmd_SERVO_STRAIN_COMP(self, gcmd):
        enable = gcmd.get_int("ENABLE", 1, minval=0, maxval=1) == 1
        if not enable:
            pairs = self._belt_pairs(gcmd)
            engine = self.printer.lookup_object("motion_engine")
            for pair in pairs:
                handle = self._node_handle(gcmd, pair.node)
                self._clear_pair(engine, handle, pair)
            gcmd.respond_info("strain compensation cleared (ramping out)")
            return
        self.enable_from_map(gcmd)

    def enable_from_map(self, gcmd, quiet=False):
        pairs = self._belt_pairs(gcmd)
        engine = self.printer.lookup_object("motion_engine")
        if not os.path.exists(self.map_file):
            raise gcmd.error(
                "no map file at %s — record one with "
                "SERVO_MEASURE_STRAIN_MAP and build it with "
                "SERVO_STRAIN_COMP_BUILD" % self.map_file
            )
        with open(self.map_file) as fh:
            payload = json.load(fh)
        by_motors = {tuple(p["motors"]): p for p in payload["pairs"]}
        for pair in pairs:
            key = tuple(pair.motor_names())
            entry = by_motors.pop(key, None)
            if entry is None:
                raise gcmd.error(
                    "map file has no entry for belt %s (%s) — rebuild it "
                    "against this printer's motors"
                    % (pair.axis_name(), "/".join(key))
                )
            handle = self._node_handle(gcmd, pair.node)
            engine.set_strain_comp(
                handle,
                pair.slots()[0],
                pair.slots()[1],
                pair.lane_a,
                pair.lane_b,
                pair.kin_tag,
                entry["nx"],
                entry["ny"],
                entry["x0"],
                entry["y0"],
                entry["dx"],
                entry["dy"],
                [int(v) for v in entry["offsets_um"]],
            )
            if not quiet:
                gcmd.respond_info(
                    "belt %s compensation enabled: %dx%d grid, "
                    "offsets %+d..%+d um"
                    % (
                        pair.axis_name(),
                        entry["nx"],
                        entry["ny"],
                        min(entry["offsets_um"]),
                        max(entry["offsets_um"]),
                    )
                )
        zero_xy = payload.get("zero_xy")
        if zero_xy and not quiet:
            gcmd.respond_info(
                "map zero point is (%.1f, %.1f) — any torque release "
                "(SERVO_SYNC, M84, idle timeout) re-anchors the map where "
                "torque returns; SERVO_SYNC at the zero point restores the "
                "calibrated anchor and the best accuracy"
                % (zero_xy[0], zero_xy[1])
            )
        if by_motors:
            raise gcmd.error(
                "map file entries %s match no belt pair on this printer"
                % ", ".join("/".join(k) for k in by_motors)
            )


def _fit_slope(points):
    n = len(points)
    mean_x = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    var = sum((p[0] - mean_x) ** 2 for p in points)
    cov = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points)
    slope = cov / var
    resid = sum((p[1] - mean_y - slope * (p[0] - mean_x)) ** 2 for p in points)
    total = sum((p[1] - mean_y) ** 2 for p in points)
    r2 = 1.0 - resid / total if total > 0.0 else 0.0
    return slope, r2


def _strain_map_run(gcmd, raw_dir):
    run_dir = os.path.expanduser(raw_dir)
    manifest_path = os.path.join(run_dir, "manifest.json")
    if not os.path.exists(manifest_path):
        raise gcmd.error("no manifest.json in %s" % run_dir)
    with open(manifest_path) as fh:
        manifest = json.load(fh)
    if manifest.get("experiment") != "strain_map":
        raise gcmd.error(
            "%s is a %r run, need a strain_map run"
            % (run_dir, manifest.get("experiment"))
        )
    return run_dir, manifest


def _belt_motor_names(manifest):
    belts = manifest.get("belts")
    if not belts:
        raise ValueError("manifest has no belts field")
    return [
        [m.split(":")[0] for m in belt.split("+")] for belt in belts.split(",")
    ]


def _load_scap(path):
    import numpy as np

    with open(path, "rb") as fh:
        raw = fh.read()
    nl = raw.index(b"\n")
    header = json.loads(raw[:nl])
    body = raw[nl + 1 :]
    rec = header["record_size"]
    n = len(body) // rec
    body = body[: n * rec]
    drives = header["drives"]
    chans = {c["name"]: c for c in header["channels"]}
    prefix = chans["target_counts"]["offset"]
    block = (rec - prefix) // len(drives)
    table = np.frombuffer(body, dtype=np.uint8).reshape(n, rec)

    def col(name, drive_idx):
        c = chans[name]
        off = c["offset"]
        if off >= prefix:
            off += drive_idx * block
        dt = {
            "u64": "<u8",
            "u8": "u1",
            "i32": "<i4",
            "i16": "<i2",
            "u16": "<u2",
            "f32": "<f4",
        }[c["dtype"]]
        width = np.dtype(dt).itemsize
        return table[:, off : off + width].copy().view(dt).ravel()

    return header, col


def _rolling_elastic_means(path, pair_names):
    """Mean elastic differential per pair over one forward+back stroke.
    Every step strokes the identical line, so the line's own strain field
    contributes the same mean to every step and cancels out of the
    offset-response slope; only the applied offset moves it."""
    import numpy as np

    header, col = _load_scap(path)
    hdr_drives = {d["name"]: i for i, d in enumerate(header["drives"])}
    for names in pair_names:
        for name in names:
            if name not in hdr_drives:
                raise ValueError("capture %s has no drive %s" % (path, name))
    lead = pair_names[0][0]
    lead_idx = hdr_drives[lead]
    target = col("target_counts", lead_idx).astype(np.float64)
    sweep_mm = (target - target[0]) / header["drives"][lead_idx][
        "counts_per_mm"
    ]
    vel = np.gradient(sweep_mm)
    fwd = vel > 1e-4
    back = vel < -1e-4
    if not fwd.any() or not back.any():
        raise ValueError(
            "capture %s has no forward+back sweep — the response "
            "measurement needs both directions to cancel friction" % path
        )
    means = []
    for names in pair_names:
        torques = []
        for name in names:
            drive_idx = hdr_drives[name]
            sign = -1.0 if header["drives"][drive_idx]["invert"] else 1.0
            torques.append(
                col("torque_actual", drive_idx).astype(np.float64) / 10.0 * sign
            )
        diff = (torques[0] - torques[1]) / 2.0
        means.append(float((diff[fwd].mean() + diff[back].mean()) / 2.0))
    return means


def _collect_elastic_samples(run_dir, manifest):
    """Per belt, one dense elastic profile per raster line, placed in
    absolute bed coordinates via the stroke plan: a list of
    {fixed_axis, fixed_value, coords, elastic} dicts."""
    import numpy as np

    plan = manifest["stroke_plan"]
    belts = _belt_motor_names(manifest)
    lines = [[] for _ in belts]
    for step in manifest["steps"]:
        swept = step.get("swept", {})
        if "y" in swept:
            fixed_axis, fixed_value = "y", float(swept["y"])
            sweep_start = plan["x_start"]
        elif "x" in swept:
            fixed_axis, fixed_value = "x", float(swept["x"])
            sweep_start = plan["y_start"]
        else:
            raise ValueError("step %s has no swept coordinate" % step["name"])
        path = os.path.join(run_dir, "step_%s.scap" % step["name"])
        header, col = _load_scap(path)
        hdr_drives = {d["name"]: i for i, d in enumerate(header["drives"])}
        cpm = {d["name"]: d["counts_per_mm"] for d in header["drives"]}
        tc = {}
        tq = {}
        for mname, di in hdr_drives.items():
            t = col("target_counts", di).astype(np.float64)
            sign = -1.0 if header["drives"][di]["invert"] else 1.0
            tc[mname] = sign * (t - t[0]) / cpm[mname]
            tq[mname] = (
                col("torque_actual", di).astype(np.float64) / 10.0 * sign
            )
        pa = tc[belts[0][0]]
        pb = tc[belts[1][0]]
        x = (pa + pb) / 2.0
        y = (pa - pb) / 2.0
        sweep = x if np.ptp(x) > np.ptp(y) else y
        sweep = sweep - sweep.min()
        span = np.ptp(sweep)
        if span <= 0.0:
            raise ValueError("step %s never moved" % step["name"])
        nbins = max(2, int(round(span / BIN_MM)))
        centers = (np.arange(nbins) + 0.5) * span / nbins
        vel = np.gradient(sweep)
        moving = np.abs(vel) > 1e-4
        fwd = moving & (vel > 0)
        back = moving & (vel < 0)
        idx = np.clip((sweep / span * nbins).astype(int), 0, nbins - 1)
        for belt_idx, (m0, m1) in enumerate(belts):
            diff = (tq[m0] - tq[m1]) / 2.0
            prof = {}
            for key, sel in (("f", fwd), ("b", back)):
                profile = np.full(nbins, np.nan)
                if sel.sum() > 0:
                    sums = np.bincount(
                        idx[sel], weights=diff[sel], minlength=nbins
                    )
                    cnts = np.bincount(idx[sel], minlength=nbins)
                    ok = cnts > 0
                    profile[ok] = sums[ok] / cnts[ok]
                prof[key] = profile
            elastic = (prof["f"] + prof["b"]) / 2.0
            coords = []
            values = []
            for center, value in zip(centers, elastic):
                if math.isnan(value):
                    continue
                coords.append(sweep_start + float(center))
                values.append(float(value))
            if coords:
                lines[belt_idx].append(
                    {
                        "fixed_axis": fixed_axis,
                        "fixed_value": fixed_value,
                        "coords": coords,
                        "elastic": values,
                    }
                )
    return lines


def _flatten_lines(belt_lines):
    import numpy as np

    xs, ys, vs = [], [], []
    for line in belt_lines:
        coords = np.asarray(line["coords"], float)
        vals = np.asarray(line["elastic"], float)
        if line["fixed_axis"] == "y":
            xs.append(coords)
            ys.append(np.full(len(coords), line["fixed_value"]))
        else:
            xs.append(np.full(len(coords), line["fixed_value"]))
            ys.append(coords)
        vs.append(vals)
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(vs)


def _field_model_fit(x, y, v, plan, corexy):
    """Least-squares fit of the structured field model: 1D components at
    2mm knots along each belt phase (x+y and x-y, CoreXY only) and along
    each axis, plus a smooth coarse 2D remainder. Point-sampling the
    raster at grid nodes aliases everything shorter than twice the node
    pitch; the model carries belt-phase (diagonal) and axis-locked
    structure between the raster lines at full 2mm resolution, so it
    survives evaluation onto any output grid."""
    import numpy as np

    coords_1d = [x + y, x - y, x, y] if corexy else [x, y]
    blocks = []
    col_idx = []
    col_w = []
    off = 0
    for u in coords_1d:
        u0 = float(u.min())
        n = int(np.ceil((float(u.max()) - u0) / FIELD_KNOT_MM)) + 1
        f = np.clip((u - u0) / FIELD_KNOT_MM, 0.0, n - 1.0)
        i0 = np.minimum(f.astype(int), n - 2)
        t = f - i0
        col_idx += [off + i0, off + i0 + 1]
        col_w += [1.0 - t, t]
        blocks.append((off, u0, n))
        off += n
    x0, x1 = plan["x_start"], plan["x_end"]
    y0, y1 = plan["y_start"], plan["y_end"]
    g_nx = max(2, int(round((x1 - x0) / FIELD_2D_PITCH_MM)) + 1)
    g_ny = max(2, int(round((y1 - y0) / FIELD_2D_PITCH_MM)) + 1)
    g_dx = (x1 - x0) / (g_nx - 1)
    g_dy = (y1 - y0) / (g_ny - 1)
    goff = off
    fx = np.clip((x - x0) / g_dx, 0.0, g_nx - 1.0)
    fy = np.clip((y - y0) / g_dy, 0.0, g_ny - 1.0)
    gix = np.minimum(fx.astype(int), g_nx - 2)
    giy = np.minimum(fy.astype(int), g_ny - 2)
    gtx, gty = fx - gix, fy - giy
    for di, dj, w in (
        (0, 0, (1 - gtx) * (1 - gty)),
        (0, 1, gtx * (1 - gty)),
        (1, 0, (1 - gtx) * gty),
        (1, 1, gtx * gty),
    ):
        col_idx.append(goff + (giy + di) * g_nx + gix + dj)
        col_w.append(w)
    cols = goff + g_nx * g_ny
    ata = np.zeros((cols, cols))
    atb = np.zeros(cols)
    for a in range(len(col_idx)):
        np.add.at(atb, col_idx[a], col_w[a] * v)
        for b in range(len(col_idx)):
            np.add.at(ata, (col_idx[a], col_idx[b]), col_w[a] * col_w[b])

    def add_second_diff(idx0, step, weight):
        base = np.asarray(idx0).ravel()
        k = np.array([1.0, -2.0, 1.0]) * weight
        triples = [base, base + step, base + 2 * step]
        for a in range(3):
            for b in range(3):
                np.add.at(ata, (triples[a], triples[b]), k[a] * k[b])

    for boff, _u0, n in blocks:
        if n >= 3:
            add_second_diff(boff + np.arange(n - 2), 1, FIELD_SMOOTH)
    ii, jj = np.meshgrid(np.arange(g_ny), np.arange(g_nx - 2), indexing="ij")
    add_second_diff(goff + ii * g_nx + jj, 1, FIELD_2D_SMOOTH)
    ii, jj = np.meshgrid(np.arange(g_ny - 2), np.arange(g_nx), indexing="ij")
    add_second_diff(goff + ii * g_nx + jj, g_nx, FIELD_2D_SMOOTH)
    ata[np.diag_indices(cols)] += 1e-6
    sol = np.linalg.solve(ata, atb)

    def evaluate(px, py):
        out = np.zeros(np.shape(px), float)
        coords = [px + py, px - py, px, py] if corexy else [px, py]
        for (boff, u0, n), u in zip(blocks, coords):
            f = np.clip((u - u0) / FIELD_KNOT_MM, 0.0, n - 1.0)
            i0 = np.minimum(f.astype(int), n - 2)
            t = f - i0
            out += sol[boff + i0] * (1.0 - t) + sol[boff + i0 + 1] * t
        efx = np.clip((px - x0) / g_dx, 0.0, g_nx - 1.0)
        efy = np.clip((py - y0) / g_dy, 0.0, g_ny - 1.0)
        eix = np.minimum(efx.astype(int), g_nx - 2)
        eiy = np.minimum(efy.astype(int), g_ny - 2)
        etx, ety = efx - eix, efy - eiy
        g = sol[goff:].reshape(g_ny, g_nx)
        out += (
            g[eiy, eix] * (1 - etx) * (1 - ety)
            + g[eiy, eix + 1] * etx * (1 - ety)
            + g[eiy + 1, eix] * (1 - etx) * ety
            + g[eiy + 1, eix + 1] * etx * ety
        )
        return out

    return evaluate


def _check_coverage(gcmd, plan, x, y, gx, gy):
    """Every output node needs a sample within one raster pitch, or the
    model is extrapolating there."""
    import numpy as np

    radius = float(plan["line_spacing"]) + 1e-6
    uncovered = 0
    px, py = np.meshgrid(gx, gy)
    nodes_x = px.ravel()
    nodes_y = py.ravel()
    for start in range(0, len(nodes_x), 256):
        nx_chunk = nodes_x[start : start + 256, None]
        ny_chunk = nodes_y[start : start + 256, None]
        d2 = (nx_chunk - x[None, :]) ** 2 + (ny_chunk - y[None, :]) ** 2
        uncovered += int((d2.min(axis=1) > radius * radius).sum())
    if uncovered:
        raise gcmd.error(
            "%d grid nodes are farther than %.0fmm from any sample — the "
            "run does not cover the region" % (uncovered, radius)
        )


def _build_grid(gcmd, plan, spacing, belt_lines, corexy):
    """Fit the structured field model to the dense line samples and
    evaluate it at the output grid nodes. The zero_xy node is the zero
    reference — SERVO_SYNC's zero point."""
    import numpy as np

    if not belt_lines:
        raise gcmd.error("run contains no usable elastic samples")
    x, y, v = _flatten_lines(belt_lines)
    x0, x1 = plan["x_start"], plan["x_end"]
    y0, y1 = plan["y_start"], plan["y_end"]
    nx = max(2, int(round((x1 - x0) / spacing)) + 1)
    ny = max(2, int(round((y1 - y0) / spacing)) + 1)
    if nx > MAX_GRID_DIM or ny > MAX_GRID_DIM or nx * ny > MAX_GRID_VALUES:
        raise gcmd.error(
            "%dx%d grid exceeds the endpoint's limits (%d per axis, %d "
            "total) — raise SPACING" % (nx, ny, MAX_GRID_DIM, MAX_GRID_VALUES)
        )
    dx = (x1 - x0) / (nx - 1)
    dy = (y1 - y0) / (ny - 1)
    gx = x0 + np.arange(nx) * dx
    gy = y0 + np.arange(ny) * dy
    _check_coverage(gcmd, plan, x, y, gx, gy)
    evaluate = _field_model_fit(x, y, v, plan, corexy)
    px, py = np.meshgrid(gx, gy)
    values = evaluate(px, py).ravel()
    zero_xy = plan.get("zero_xy", [(x0 + x1) / 2.0, (y0 + y1) / 2.0])
    grid = {
        "nx": nx,
        "ny": ny,
        "x0": x0,
        "y0": y0,
        "dx": dx,
        "dy": dy,
        "values_pct": [float(val) for val in values],
        "zero_xy": list(zero_xy),
    }
    zero_value = _grid_value_at(grid, zero_xy[0], zero_xy[1])
    grid["values_pct"] = [val - zero_value for val in grid["values_pct"]]
    return grid


def _rezero_grid_at(grid, zero_xy):
    """A delta map measured WITH the base map active carries its own run's
    DC anchor; shift it so zero sits at the base map's zero point — the two
    anchors must coincide for the sum to stay sync-consistent."""
    zero_value = _grid_value_at(grid, zero_xy[0], zero_xy[1])
    grid["values_pct"] = [v - zero_value for v in grid["values_pct"]]


def _merge_offsets(gcmd, grid, delta_offsets, base):
    base_grid = {
        "nx": base["nx"],
        "ny": base["ny"],
        "x0": base["x0"],
        "y0": base["y0"],
        "dx": base["dx"],
        "dy": base["dy"],
        "values_pct": [float(v) for v in base["offsets_um"]],
    }
    merged = []
    for i, delta in enumerate(delta_offsets):
        x = grid["x0"] + (i % grid["nx"]) * grid["dx"]
        y = grid["y0"] + (i // grid["nx"]) * grid["dy"]
        merged.append(int(round(_grid_value_at(base_grid, x, y))) + delta)
    worst = max(abs(o) for o in merged)
    if worst > MAX_OFFSET_UM:
        raise gcmd.error(
            "merged compensation would need %d um (max %d) — the maps are "
            "fighting each other, rebuild from scratch" % (worst, MAX_OFFSET_UM)
        )
    _check_span(gcmd, merged)
    return merged


def _check_span(gcmd, offsets):
    """The endpoint re-anchors the map wherever torque returns after a
    release, so the applied offset can reach the map's full span — the span,
    not just each value, must fit the offset budget."""
    span = max(offsets) - min(offsets)
    if span > MAX_OFFSET_UM:
        raise gcmd.error(
            "compensation spans %d um (max %d) — re-anchoring after a "
            "torque release could exceed the offset budget"
            % (span, MAX_OFFSET_UM)
        )


def _grid_value_at(grid, x, y):
    nx, ny = grid["nx"], grid["ny"]
    fx = min(max((x - grid["x0"]) / grid["dx"], 0.0), nx - 1)
    fy = min(max((y - grid["y0"]) / grid["dy"], 0.0), ny - 1)
    ix = min(int(fx), nx - 2) if nx > 1 else 0
    iy = min(int(fy), ny - 2) if ny > 1 else 0
    tx = fx - ix if nx > 1 else 0.0
    ty = fy - iy if ny > 1 else 0.0
    values = grid["values_pct"]

    def at(gx, gy):
        return values[min(gy, ny - 1) * nx + min(gx, nx - 1)]

    top = at(ix, iy) * (1.0 - tx) + at(ix + 1, iy) * tx
    bottom = at(ix, iy + 1) * (1.0 - tx) + at(ix + 1, iy + 1) * tx
    return top * (1.0 - ty) + bottom * ty


def _fit_in_use_response(
    gcmd, belt_idx, base_lines, run_lines, plan, corexy, offset_grids
):
    """The exact in-use stiffness row for one belt: model the baseline
    field, then regress the compensated run's change from it against the
    offsets the map applied at each sample point — both belts' offsets as
    regressors, so the direct and cross responses separate in one
    least-squares. An intercept absorbs the runs' DC anchor difference."""
    import numpy as np

    if not base_lines or not run_lines:
        raise gcmd.error("run contains no usable elastic samples")
    bx, by, bv = _flatten_lines(base_lines)
    rx, ry, rv = _flatten_lines(run_lines)
    baseline_field = _field_model_fit(bx, by, bv, plan, corexy)
    delta = rv - baseline_field(rx, ry)
    own_grid = offset_grids[belt_idx]
    other_grid = offset_grids[1 - belt_idx]
    own = np.array([_grid_value_at(own_grid, px, py) for px, py in zip(rx, ry)])
    other = np.array(
        [_grid_value_at(other_grid, px, py) for px, py in zip(rx, ry)]
    )
    for label, regressor in (("own", own), ("other", other)):
        if float(np.ptp(regressor)) < FIT_MIN_EXCITATION_MM:
            raise gcmd.error(
                "belt %s: the map's %s-belt offsets vary by less than "
                "%.0f um over the run — too little excitation to fit the "
                "response"
                % ("AB"[belt_idx], label, FIT_MIN_EXCITATION_MM * 1000.0)
            )
    corr = float(np.corrcoef(own, other)[0, 1])
    if abs(corr) > FIT_MAX_OFFSET_CORR:
        raise gcmd.error(
            "the two belts' offsets are collinear (corr %.3f) — the "
            "direct and cross responses cannot be separated" % corr
        )
    design = np.column_stack([own, other, np.ones_like(own)])
    coef, *_ = np.linalg.lstsq(design, delta, rcond=None)
    pred = design @ coef
    ss_res = float(((delta - pred) ** 2).sum())
    ss_tot = float(((delta - delta.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    if r2 < FIT_MIN_R2:
        raise gcmd.error(
            "belt %s: response fit is not credible (R^2 %.3f) — was RUN "
            "captured with the current map enabled?" % ("AB"[belt_idx], r2)
        )
    return float(coef[0]), float(coef[1]), r2, len(delta)


def _offsets_from_grids(gcmd, grids, k_matrix):
    """Solve the 2x2 stiffness system per grid node: an antisymmetric
    offset on one belt also strains the other through the shared gantry,
    so the offsets that null both fields are -inv(K) @ strain, not each
    field divided by its own stiffness."""
    (kaa, kab), (kba, kbb) = k_matrix
    det = kaa * kbb - kab * kba
    if abs(det) < 1e-2 * abs(kaa * kbb):
        raise gcmd.error(
            "stiffness matrix [[%.1f, %.1f], [%.1f, %.1f]] is near "
            "singular — cross terms rivaling the direct terms cannot be "
            "solved" % (kaa, kab, kba, kbb)
        )
    inv_own_other = [(kbb / det, -kab / det), (kaa / det, -kba / det)]
    values = [grid["values_pct"] for grid in grids]
    assert len(values[0]) == len(values[1])
    belt_offsets = []
    for belt_idx, (inv_own, inv_other) in enumerate(inv_own_other):
        offsets = [
            int(round(-(inv_own * va + inv_other * vb) * 1000.0))
            for va, vb in zip(values[belt_idx], values[1 - belt_idx])
        ]
        worst = max(abs(o) for o in offsets)
        if worst > MAX_OFFSET_UM:
            raise gcmd.error(
                "compensation for belt %s would need %d um (max %d) — the "
                "stiffness is implausibly low or the strain field is out "
                "of range" % ("AB"[belt_idx], worst, MAX_OFFSET_UM)
            )
        _check_span(gcmd, offsets)
        belt_offsets.append(offsets)
    return belt_offsets


def load_config(config):
    return ServoStrainComp(config)
