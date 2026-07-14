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
COMP_SLEW_MM_S = 1.0
TORQUE_READS = 5
BIN_MM = 2.0


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


class ServoStrainComp:
    cmd_SERVO_MEASURE_PAIR_STIFFNESS_help = (
        "Measure the belt stiffness matrix: step a known antisymmetric "
        "offset through the compensation bank and read every pair's "
        "differential torque response — the direct slope (%/mm) plus the "
        "cross-belt slope through the shared gantry. Both feed the build's "
        "joint solve. STEP_UM (50) sets the probe amplitude, SETTLE (0.8) "
        "the wait per step, AXIS=X|Y limits to one pair."
    )
    cmd_SERVO_STRAIN_COMP_BUILD_help = (
        "Build the compensation map from a SERVO_MEASURE_STRAIN_MAP run: "
        "grid the elastic differential field per belt, solve the 2x2 "
        "stiffness system per grid node (an offset on one belt also "
        "strains the other through the gantry), write the map file. "
        "RUN=<run dir> is required; the stiffness matrix comes from "
        "SERVO_MEASURE_PAIR_STIFFNESS or STIFFNESS_A/STIFFNESS_B with "
        "CROSS_AB/CROSS_BA (%/mm, CROSS_AB = belt A's response to a belt "
        "B offset, 0 disables the cross term); SPACING overrides the grid "
        "pitch (defaults to the run's line spacing). MERGE=1 treats the "
        "run as a residual measured WITH the current map enabled and adds "
        "the correction on top — the second-order iteration."
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
        step_um = gcmd.get_float("STEP_UM", 50.0, above=0.0, maxval=200.0)
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0, maxval=5.0)
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
        run_dir = os.path.expanduser(gcmd.get("RUN"))
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
            "SPACING", plan["line_spacing"], above=2.0, maxval=100.0
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
        k_matrix = [(direct[0], cross[0]), (cross[1], direct[1])]
        samples = _collect_elastic_samples(run_dir, manifest)
        grids = []
        bases = []
        for belt_idx, motor_names in enumerate(belts):
            grid = _build_grid(gcmd, plan, spacing, samples[belt_idx])
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
        gcmd.respond_info(
            "strain compensation map written to %s — apply it with "
            "SERVO_STRAIN_COMP ENABLE=1" % self.map_file
        )

    def cmd_SERVO_STRAIN_COMP(self, gcmd):
        enable = gcmd.get_int("ENABLE", 1, minval=0, maxval=1) == 1
        pairs = self._belt_pairs(gcmd)
        engine = self.printer.lookup_object("motion_engine")
        if not enable:
            for pair in pairs:
                handle = self._node_handle(gcmd, pair.node)
                self._clear_pair(engine, handle, pair)
            gcmd.respond_info("strain compensation cleared (ramping out)")
            return
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
        if zero_xy:
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


def _interp(coords, values, at):
    """Linear interpolation clamped to the profile's ends; coords ascend."""
    if at <= coords[0]:
        return values[0]
    if at >= coords[-1]:
        return values[-1]
    lo, hi = 0, len(coords) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if coords[mid] <= at:
            lo = mid
        else:
            hi = mid
    t = (at - coords[lo]) / (coords[hi] - coords[lo])
    return values[lo] * (1.0 - t) + values[hi] * t


def _build_grid(gcmd, plan, spacing, belt_lines):
    """Evaluate each raster line's dense profile at the grid nodes it
    crosses, averaging where an X line and a Y line meet, and fill
    uncrossed nodes from their neighbors. The center node is the zero
    reference — SERVO_SYNC's zero point."""
    if not belt_lines:
        raise gcmd.error("run contains no usable elastic samples")
    x0, x1 = plan["x_start"], plan["x_end"]
    y0, y1 = plan["y_start"], plan["y_end"]
    nx = max(2, int(round((x1 - x0) / spacing)) + 1)
    ny = max(2, int(round((y1 - y0) / spacing)) + 1)
    dx = (x1 - x0) / (nx - 1)
    dy = (y1 - y0) / (ny - 1)
    sums = [0.0] * (nx * ny)
    counts = [0] * (nx * ny)
    for line in belt_lines:
        if line["fixed_axis"] == "y":
            iy = int(round((line["fixed_value"] - y0) / dy))
            if not 0 <= iy < ny:
                continue
            if abs(line["fixed_value"] - (y0 + iy * dy)) > dy / 2.0:
                continue
            for ix in range(nx):
                value = _interp(line["coords"], line["elastic"], x0 + ix * dx)
                sums[iy * nx + ix] += value
                counts[iy * nx + ix] += 1
        else:
            ix = int(round((line["fixed_value"] - x0) / dx))
            if not 0 <= ix < nx:
                continue
            if abs(line["fixed_value"] - (x0 + ix * dx)) > dx / 2.0:
                continue
            for iy in range(ny):
                value = _interp(line["coords"], line["elastic"], y0 + iy * dy)
                sums[iy * nx + ix] += value
                counts[iy * nx + ix] += 1
    values = [
        sums[i] / counts[i] if counts[i] else None for i in range(nx * ny)
    ]
    for _ in range(nx + ny):
        holes = [i for i, v in enumerate(values) if v is None]
        if not holes:
            break
        for i in holes:
            ix, iy = i % nx, i // nx
            neighbors = [
                values[jy * nx + jx]
                for jx, jy in (
                    (ix - 1, iy),
                    (ix + 1, iy),
                    (ix, iy - 1),
                    (ix, iy + 1),
                )
                if 0 <= jx < nx
                and 0 <= jy < ny
                and values[jy * nx + jx] is not None
            ]
            if neighbors:
                values[i] = sum(neighbors) / len(neighbors)
    if any(v is None for v in values):
        raise gcmd.error(
            "grid has unfillable holes — the run does not cover the region"
        )
    zero_xy = plan.get("zero_xy", [(x0 + x1) / 2.0, (y0 + y1) / 2.0])
    grid = {
        "nx": nx,
        "ny": ny,
        "x0": x0,
        "y0": y0,
        "dx": dx,
        "dy": dy,
        "values_pct": values,
        "zero_xy": list(zero_xy),
    }
    zero_value = _grid_value_at(grid, zero_xy[0], zero_xy[1])
    grid["values_pct"] = [v - zero_value for v in values]
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
