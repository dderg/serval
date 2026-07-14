"""Belt strain compensation: measure the pair stiffness, build a per-belt
2D offset map from a strain_map run, and feed it to the endpoint's
compensation bank (antisymmetric position offsets keyed on the commanded
carriage position)."""

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
        "Measure each belt pair's differential stiffness: step a known "
        "antisymmetric offset through the compensation bank and read the "
        "differential torque response. The slope (%/mm) converts a strain "
        "map into compensation offsets. STEP_UM (50) sets the probe "
        "amplitude, SETTLE (0.8) the wait per step, AXIS=X|Y limits to one "
        "pair."
    )
    cmd_SERVO_STRAIN_COMP_BUILD_help = (
        "Build the compensation map from a SERVO_MEASURE_STRAIN_MAP run: "
        "grid the elastic differential field per belt, invert the pair "
        "response matrix, write the map file. RUN=<run dir> is required. "
        "The response matrix comes from the run's own probe lines "
        "(SERVO_MEASURE_STRAIN_MAP PROBE=1, the default); "
        "STIFFNESS_A/STIFFNESS_B and CROSS_AB/CROSS_BA (%/mm) override "
        "it, and the standstill SERVO_MEASURE_PAIR_STIFFNESS values are "
        "the last resort (they over-read). SPACING overrides the grid "
        "pitch (defaults to the run's line spacing). "
        "MERGE=1 treats the run as a residual measured WITH the current "
        "map enabled, identifies the effective response matrix from the "
        "map's applied offsets and the field change since its base run "
        "(the standstill probe over-reads the slopes), and adds the "
        "correction on top; explicit STIFFNESS/CROSS values skip the "
        "identification."
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
        self.map_enabled = False
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

    def _reject_active_map(self, gcmd, would):
        if self.map_enabled:
            raise gcmd.error(
                "strain compensation is enabled — %s would replace the "
                "active map; SERVO_STRAIN_COMP ENABLE=0 first" % would
            )

    def belt_pairs_for_probe(self, gcmd):
        self._reject_active_map(gcmd, "probe offsets (pass PROBE=0)")
        return self._belt_pairs(gcmd)

    def set_probe_offset(self, gcmd, pair, value_um):
        engine = self.printer.lookup_object("motion_engine")
        handle = self._node_handle(gcmd, pair.node)
        self._upload_constant(engine, handle, pair, value_um)

    def clear_probe_offset(self, gcmd, pair):
        engine = self.printer.lookup_object("motion_engine")
        handle = self._node_handle(gcmd, pair.node)
        self._clear_pair(engine, handle, pair)

    def cmd_SERVO_MEASURE_PAIR_STIFFNESS(self, gcmd):
        self._reject_active_map(gcmd, "the stiffness probe")
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
                self.measured_cross[
                    (tuple(obs.motor_names()), tuple(names))
                ] = cross
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

    def _cross_for(self, obs_names, act_names, override):
        if override is not None:
            return override
        obs_keys = [
            (tuple(obs_names), 1.0),
            (tuple(reversed(obs_names)), -1.0),
        ]
        act_keys = [
            (tuple(act_names), 1.0),
            (tuple(reversed(act_names)), -1.0),
        ]
        for obs_key, obs_sign in obs_keys:
            for act_key, act_sign in act_keys:
                value = self.measured_cross.get((obs_key, act_key))
                if value is not None:
                    return value * obs_sign * act_sign
        return None

    def _standstill_kmat(self, gcmd, belts, matrix_params):
        stiffness_a, stiffness_b, cross_ab, cross_ba = matrix_params
        stiffness = [
            self._stiffness_for(gcmd, 0, belts[0], stiffness_a),
            self._stiffness_for(gcmd, 1, belts[1], stiffness_b),
        ]
        cross = [
            self._cross_for(belts[0], belts[1], cross_ab),
            self._cross_for(belts[1], belts[0], cross_ba),
        ]
        if None in cross:
            gcmd.respond_info(
                "no cross-coupling measured or given (CROSS_AB/CROSS_BA) — "
                "solving the belts independently"
            )
            cross = [c if c is not None else 0.0 for c in cross]
        return [[stiffness[0], cross[0]], [cross[1], stiffness[1]]]

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
        base_payload = None
        base_pairs = {}
        if merge:
            if not os.path.exists(self.map_file):
                raise gcmd.error(
                    "MERGE=1 needs an existing map at %s" % self.map_file
                )
            with open(self.map_file) as fh:
                base_payload = json.load(fh)
            base_pairs = {tuple(p["motors"]): p for p in base_payload["pairs"]}
        plan = manifest["stroke_plan"]
        spacing = gcmd.get_float(
            "SPACING", plan["line_spacing"], above=2.0, maxval=100.0
        )
        matrix_params = [
            gcmd.get_float("STIFFNESS_A", None),
            gcmd.get_float("STIFFNESS_B", None),
            gcmd.get_float("CROSS_AB", None),
            gcmd.get_float("CROSS_BA", None),
        ]
        belts = _belt_motor_names(manifest)
        samples = _collect_elastic_samples(run_dir, manifest)
        bases = [None, None]
        if merge:
            for belt_idx, motor_names in enumerate(belts):
                bases[belt_idx] = base_pairs.get(tuple(motor_names))
                if bases[belt_idx] is None:
                    raise gcmd.error(
                        "existing map has no entry for belt %s (%s)"
                        % ("AB"[belt_idx], "/".join(motor_names))
                    )
        grids = []
        for belt_idx in range(2):
            grid = _build_grid(gcmd, plan, spacing, samples[belt_idx])
            if merge:
                _rezero_grid_at(grid, bases[belt_idx]["zero_xy"])
                grid["zero_xy"] = list(bases[belt_idx]["zero_xy"])
            grids.append(grid)
        explicit = any(p is not None for p in matrix_params)
        if merge and not explicit:
            kmat = _identify_response(gcmd, grids, bases, base_payload["run"])
        elif explicit:
            kmat = self._standstill_kmat(gcmd, belts, matrix_params)
        else:
            kmat = _kmat_from_probe_steps(gcmd, run_dir, manifest, belts)
            if kmat is None:
                gcmd.respond_info(
                    "run has no probe lines (SERVO_MEASURE_STRAIN_MAP "
                    "PROBE=1) — falling back to the standstill stiffness, "
                    "which over-reads the response"
                )
                kmat = self._standstill_kmat(gcmd, belts, matrix_params)
        per_belt_offsets = _offsets_from_grids(gcmd, grids, kmat)
        pairs_out = []
        for belt_idx, motor_names in enumerate(belts):
            grid = grids[belt_idx]
            delta_offsets = per_belt_offsets[belt_idx]
            offsets = delta_offsets
            if merge:
                offsets = _merge_offsets(
                    gcmd, grid, delta_offsets, bases[belt_idx]
                )
            pairs_out.append(
                {
                    "motors": motor_names,
                    "stiffness_pct_per_mm": kmat[belt_idx][belt_idx],
                    "cross_pct_per_mm": kmat[belt_idx][1 - belt_idx],
                    "nx": grid["nx"],
                    "ny": grid["ny"],
                    "x0": grid["x0"],
                    "y0": grid["y0"],
                    "dx": grid["dx"],
                    "dy": grid["dy"],
                    "offsets_um": offsets,
                    "applied_delta_um": delta_offsets,
                    "zero_xy": grid["zero_xy"],
                }
            )
            gcmd.respond_info(
                "belt %s (%s): %dx%d grid, offsets %+d..%+d um "
                "(stiffness %.1f %%/mm, cross %.1f %%/mm)"
                % (
                    "AB"[belt_idx],
                    "/".join(motor_names),
                    grid["nx"],
                    grid["ny"],
                    min(offsets),
                    max(offsets),
                    kmat[belt_idx][belt_idx],
                    kmat[belt_idx][1 - belt_idx],
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
            self.map_enabled = False
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
        self.map_enabled = True
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


def _step_profiles(run_dir, manifest, step):
    """One step's dense elastic profile per belt, in absolute bed
    coordinates: (fixed_axis, fixed_value, [(coords, elastic), ...])."""
    import numpy as np

    plan = manifest["stroke_plan"]
    belts = _belt_motor_names(manifest)
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
        tq[mname] = col("torque_actual", di).astype(np.float64) / 10.0 * sign
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
    profiles = []
    for m0, m1 in belts:
        diff = (tq[m0] - tq[m1]) / 2.0
        prof = {}
        for key, sel in (("f", fwd), ("b", back)):
            profile = np.full(nbins, np.nan)
            if sel.sum() > 0:
                sums = np.bincount(idx[sel], weights=diff[sel], minlength=nbins)
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
        profiles.append((coords, values))
    return fixed_axis, fixed_value, profiles


def _collect_elastic_samples(run_dir, manifest):
    """Per belt, one dense elastic profile per raster line, placed in
    absolute bed coordinates via the stroke plan: a list of
    {fixed_axis, fixed_value, coords, elastic} dicts. Probe lines (steps
    with applied offsets) are excitation, not field — skipped here."""
    belts = _belt_motor_names(manifest)
    lines = [[] for _ in belts]
    for step in manifest["steps"]:
        if step.get("applied"):
            continue
        fixed_axis, fixed_value, profiles = _step_profiles(
            run_dir, manifest, step
        )
        for belt_idx, (coords, values) in enumerate(profiles):
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


def _kmat_from_probe_steps(gcmd, run_dir, manifest, belts):
    """Identify the pair response from the run's own probe lines: sweeps
    captured with a constant offset applied on one pair, measured by the
    same fwd/back-averaged instrument as the field itself. The standstill
    probe reads a stiction-locked transient instead (bench data: 477/-140
    at standstill vs 380/-100 once the mechanism has moved), so it must
    not size the offsets."""
    probe_steps = [s for s in manifest["steps"] if s.get("applied")]
    if not probe_steps:
        return None
    by_pair = {0: [], 1: []}
    for step in probe_steps:
        applied = step["applied"]
        if len(applied) != 1 or "offset_um" not in applied[0]:
            raise gcmd.error(
                "probe step %s applied %r is not a single pair offset"
                % (step["name"], applied)
            )
        act = None
        for belt_idx, names in enumerate(belts):
            if set(names) == set(applied[0]["motors"]):
                act = belt_idx
        if act is None:
            raise gcmd.error(
                "probe step %s motors %s match no belt pair"
                % (step["name"], "/".join(applied[0]["motors"]))
            )
        _axis, _value, profiles = _step_profiles(run_dir, manifest, step)
        by_pair[act].append((float(applied[0]["offset_um"]), profiles))
    kmat = [[0.0, 0.0], [0.0, 0.0]]
    stds = [[0.0, 0.0], [0.0, 0.0]]
    for act in (0, 1):
        plus = [e for e in by_pair[act] if e[0] > 0]
        minus = [e for e in by_pair[act] if e[0] < 0]
        if not plus or not minus:
            raise gcmd.error(
                "run has probe lines but belt %s lacks a +/- pair of them"
                % "AB"[act]
            )
        value_p, prof_p = plus[0]
        value_m, prof_m = minus[0]
        denom_mm = (value_p - value_m) / 1000.0
        for obs in (0, 1):
            coords_p, vals_p = prof_p[obs]
            coords_m, vals_m = prof_m[obs]
            if not coords_p or not coords_m:
                raise gcmd.error(
                    "probe line for belt %s has no elastic samples" % "AB"[act]
                )
            slopes = [
                (vp - _interp(coords_m, vals_m, c)) / denom_mm
                for c, vp in zip(coords_p, vals_p)
                if coords_m[0] <= c <= coords_m[-1]
            ]
            if len(slopes) < 8:
                raise gcmd.error(
                    "probe lines for belt %s barely overlap" % "AB"[act]
                )
            mean = sum(slopes) / len(slopes)
            std = math.sqrt(sum((s - mean) ** 2 for s in slopes) / len(slopes))
            if obs == act and (abs(mean) < 1e-6 or std > 0.5 * abs(mean)):
                raise gcmd.error(
                    "probe response for belt %s is not credible "
                    "(%.1f +/- %.1f %%/mm) — was torque enabled?"
                    % ("AB"[act], mean, std)
                )
            kmat[obs][act] = mean
            stds[obs][act] = std
    gcmd.respond_info(
        "effective response from the run's probe lines: "
        "belt A [%.1f±%.1f, %.1f±%.1f] %%/mm, "
        "belt B [%.1f±%.1f, %.1f±%.1f] %%/mm"
        % (
            kmat[0][0],
            stds[0][0],
            kmat[0][1],
            stds[0][1],
            kmat[1][0],
            stds[1][0],
            kmat[1][1],
            stds[1][1],
        )
    )
    return kmat


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


IDENTIFY_MIN_OFFSET_RMS_UM = 5.0
IDENTIFY_MIN_R2 = 0.8


def _identify_response(gcmd, grids, bases, base_run):
    """The base map records the offset delta it applied at every node and
    the run it was built from records the field those offsets acted on, so
    regressing the field change against the offset delta yields the pair
    response that actually holds during a mapping run. The standstill
    probe over-reads both slopes (bench data: probe 477/-140, effective
    ~380/-100), so the probe matrix must not be reused for the merge."""
    manifest_path = os.path.join(base_run, "manifest.json")
    if not os.path.exists(manifest_path):
        raise gcmd.error(
            "the base map's run %s is gone — cannot identify the effective "
            "response; pass STIFFNESS_A/STIFFNESS_B (and CROSS_AB/CROSS_BA) "
            "to merge with an explicit matrix" % base_run
        )
    with open(manifest_path) as fh:
        base_manifest = json.load(fh)
    base_plan = base_manifest["stroke_plan"]
    base_samples = _collect_elastic_samples(base_run, base_manifest)
    base_fields = []
    offset_grids = []
    for belt_idx in range(2):
        field = _build_grid(
            gcmd,
            base_plan,
            base_plan["line_spacing"],
            base_samples[belt_idx],
        )
        _rezero_grid_at(field, bases[belt_idx]["zero_xy"])
        base_fields.append(field)
        base = bases[belt_idx]
        offset_grids.append(
            {
                "nx": base["nx"],
                "ny": base["ny"],
                "x0": base["x0"],
                "y0": base["y0"],
                "dx": base["dx"],
                "dy": base["dy"],
                "values_pct": [
                    float(v)
                    for v in base.get("applied_delta_um", base["offsets_um"])
                ],
            }
        )
    o_a = []
    o_b = []
    deltas = ([], [])
    grid = grids[0]
    for i in range(grid["nx"] * grid["ny"]):
        x = grid["x0"] + (i % grid["nx"]) * grid["dx"]
        y = grid["y0"] + (i // grid["nx"]) * grid["dy"]
        o_a.append(_grid_value_at(offset_grids[0], x, y) / 1000.0)
        o_b.append(_grid_value_at(offset_grids[1], x, y) / 1000.0)
        for belt_idx in range(2):
            deltas[belt_idx].append(
                grids[belt_idx]["values_pct"][i]
                - _grid_value_at(base_fields[belt_idx], x, y)
            )
    n = len(o_a)
    sxx = sum(v * v for v in o_a)
    syy = sum(v * v for v in o_b)
    sxy = sum(a * b for a, b in zip(o_a, o_b))
    for name, s in (("A", sxx), ("B", syy)):
        rms_um = math.sqrt(s / n) * 1000.0
        if rms_um < IDENTIFY_MIN_OFFSET_RMS_UM:
            raise gcmd.error(
                "the base map's belt %s offsets are too small "
                "(rms %.1f um) to identify the response — pass "
                "STIFFNESS_A/STIFFNESS_B to merge with an explicit matrix"
                % (name, rms_um)
            )
    det = sxx * syy - sxy * sxy
    if det < 0.01 * sxx * syy:
        raise gcmd.error(
            "the base map's belt offsets are too correlated to separate "
            "the direct and cross responses — pass STIFFNESS_A/STIFFNESS_B "
            "and CROSS_AB/CROSS_BA instead"
        )
    kmat = []
    r2s = []
    for belt_idx in range(2):
        d = deltas[belt_idx]
        sxd = sum(a * v for a, v in zip(o_a, d))
        syd = sum(b * v for b, v in zip(o_b, d))
        row = [
            (syy * sxd - sxy * syd) / det,
            (sxx * syd - sxy * sxd) / det,
        ]
        ss_res = sum(
            (v - row[0] * a - row[1] * b) ** 2 for v, a, b in zip(d, o_a, o_b)
        )
        ss_tot = sum(v * v for v in d)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 0.0
        if r2 < IDENTIFY_MIN_R2:
            raise gcmd.error(
                "the base map's offsets explain only R^2=%.2f of belt %s's "
                "field change — the strain field drifted between the runs "
                "or the map does not match the base run" % (r2, "AB"[belt_idx])
            )
        kmat.append(row)
        r2s.append(r2)
    gcmd.respond_info(
        "identified effective response from %s + this run: "
        "belt A [%.1f, %.1f] %%/mm (R^2 %.3f), "
        "belt B [%.1f, %.1f] %%/mm (R^2 %.3f)"
        % (
            base_run,
            kmat[0][0],
            kmat[0][1],
            r2s[0],
            kmat[1][0],
            kmat[1][1],
            r2s[1],
        )
    )
    return kmat


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


def _offsets_from_grids(gcmd, grids, kmat):
    """Solve the coupled pair response per grid node: an offset on either
    belt moves both belts' differential torque (direct stiffness on the
    diagonal, gantry cross-coupling off it), so cancelling the field means
    inverting the 2x2 matrix, not dividing each belt by its own slope."""
    if (grids[0]["nx"], grids[0]["ny"]) != (grids[1]["nx"], grids[1]["ny"]):
        raise gcmd.error(
            "belt grids disagree: %dx%d vs %dx%d — the run gridded the "
            "belts differently"
            % (
                grids[0]["nx"],
                grids[0]["ny"],
                grids[1]["nx"],
                grids[1]["ny"],
            )
        )
    det = kmat[0][0] * kmat[1][1] - kmat[0][1] * kmat[1][0]
    if abs(det) < 0.25 * abs(kmat[0][0] * kmat[1][1]):
        raise gcmd.error(
            "stiffness matrix [[%.1f, %.1f], [%.1f, %.1f]] is nearly "
            "singular — cross-coupling that strong cannot be compensated "
            "this way" % (kmat[0][0], kmat[0][1], kmat[1][0], kmat[1][1])
        )
    per_belt = ([], [])
    for ea, eb in zip(grids[0]["values_pct"], grids[1]["values_pct"]):
        oa = -(kmat[1][1] * ea - kmat[0][1] * eb) / det
        ob = -(kmat[0][0] * eb - kmat[1][0] * ea) / det
        per_belt[0].append(int(round(oa * 1000.0)))
        per_belt[1].append(int(round(ob * 1000.0)))
    for belt_idx, offsets in enumerate(per_belt):
        worst = max(abs(o) for o in offsets)
        if worst > MAX_OFFSET_UM:
            raise gcmd.error(
                "belt %s compensation would need %d um (max %d) — the "
                "stiffness %.1f %%/mm is implausibly low or the strain "
                "field is out of range"
                % (
                    "AB"[belt_idx],
                    worst,
                    MAX_OFFSET_UM,
                    kmat[belt_idx][belt_idx],
                )
            )
        _check_span(gcmd, offsets)
    return per_belt


def load_config(config):
    return ServoStrainComp(config)
