from __future__ import annotations

import os

from .measure import MeasureCommands

try:
    import tomllib
except ImportError:
    tomllib = None

from typing import Any, Callable

from ... import structured_log
from .. import servo_strokes
from .dynamics import (
    DYNAMICS_METRIC_BY_TERM,
    _copy_dynamics,
    _equal_or_opposite_columns,
    add_dynamics_direction_split,
    direction_split_candidate_metrics,
    discover_dynamics_pairs,
    parse_dynamics_profile,
    render_dynamics_toml,
    scale_dynamics,
    scale_dynamics_mode,
)
from .search import golden_section_search
from .sweep import DynamicsModelAdapter


class RefineCommands(MeasureCommands):
    cmd_SERVO_REFINE_DYNAMICS_help = (
        "Empirically refine the torque-feedforward dynamics profile on the "
        "RUNNING endpoint: golden-section search over a scale factor on the "
        "baseline profile's mass matrix (TERM=MASS, scored on mean per-move "
        "ferr_peak - the error window runs from move start through settle, "
        "so it covers in-move tracking and endpoint overshoot alike), "
        "viscous vector (TERM=VISCOUS, scored on mean ferr_rms) or "
        "coulomb vector (TERM=COULOMB, "
        "scored on mean ferr_peak - friction error peaks at breakaway), or "
        "an additive signed coefficient for each AWD pair "
        "(TERM=DIRECTION_SPLIT, scored on the even directional differential "
        "of signed moving-error means). On "
        "coupled_xy every vector term refines the two modes sequentially - "
        "X strokes scaling only the x-mode entry, then Y strokes scaling "
        "the y mode on top of the X winner - since the modes are "
        "independent physical quantities (moved mass, rail friction) and "
        "an axis stroke leaves the other mode's velocity at exactly zero. "
        "Each candidate runs the full "
        "SERVO_MEASURE_INERTIA ACCELS x SPEEDS grid in one tracking "
        "capture, so the score averages over every operating point. The "
        "baseline is PROFILE= or the "
        "node-level [ethercat_node] dynamics_profile (per-motor profiles "
        "are not supported). The live model is ALWAYS restored to the "
        "baseline afterwards (also on failure; if klippy dies mid-run the "
        "endpoint keeps the last candidate until restart). A torque-rail "
        "flag on any step aborts (clipped strokes cannot score a "
        "candidate); the resonance flag is ignored here - scaling a "
        "feedforward term does not move the loop's resonances. When a "
        "candidate beats the baseline, the refined profile is written to a "
        "new TOML - pointing "
        "dynamics_profile at it (then RESTART) is the only way to keep it. "
        "PATTERN=1 replaces the per-axis stroke grids with the "
        "TEST_SPEED-style XY pattern over the configured XY bounds inset "
        "by BOUND (plus a SMALL_SIZE box at center); short segments run "
        "triangular profiles on purpose, and TERM=DIRECTION_SPLIT is not "
        "supported with PATTERN=1 (direction metrics need rest-to-rest "
        "strokes). Params TERM (MASS) AXIS (X) SERVOS PROFILE LO HI TOL "
        "(0.02) MAX_EVALS (10) START END X_START X_END Y_START Y_END "
        "ACCELS SPEEDS ITERATIONS DWELL_MS TAG (refdyn) NAME PATTERN BOUND "
        "SMALL_SIZE"
    )

    def _refine_dynamics_node(self, gcmd: Any, servos: list[str]) -> Any:
        nodes = {}
        for servo in servos:
            node, _slot = self._resolve_node_slot(servo)
            nodes[node.name] = node
        if len(nodes) != 1:
            raise gcmd.error(
                "servos %s span multiple ethercat nodes (%s) - the dynamics "
                "model is per-node" % (servos, sorted(nodes))
            )
        return nodes.popitem()[1]

    def _load_baseline_dynamics(
        self, gcmd: Any, node: Any
    ) -> tuple[str, dict[str, Any]]:
        profile_path = gcmd.get("PROFILE", None) or node.get_dynamics_profile()
        if profile_path is None:
            raise gcmd.error(
                "no baseline dynamics profile - set dynamics_profile on "
                "[ethercat_node %s] or pass PROFILE= (per-motor profiles "
                "are not supported by SERVO_REFINE_DYNAMICS)" % (node.name,)
            )
        profile_path = os.path.expanduser(profile_path)
        try:
            with open(profile_path) as f:
                baseline = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "failed to load dynamics profile %s: %s" % (profile_path, e)
            )
        if len(baseline["axes"]) != node.get_drive_count():
            raise gcmd.error(
                "profile %s describes %d axes but node %s has %d drives"
                % (
                    profile_path,
                    len(baseline["axes"]),
                    node.name,
                    node.get_drive_count(),
                )
            )
        for profile_slot, motor in enumerate(baseline["axes"]):
            node_slot = node.get_slot_for_motor(motor)
            if node_slot is None:
                raise gcmd.error(
                    "profile %s axis %r is not a motor on node %s"
                    % (profile_path, motor, node.name)
                )
            if node_slot != profile_slot:
                raise gcmd.error(
                    "profile %s axis %r is at slot %d, but node %s maps it "
                    "to slot %d"
                    % (profile_path, motor, profile_slot, node.name, node_slot)
                )
        return profile_path, baseline

    def _direction_split_baseline(
        self, gcmd: Any, kin: Any, baseline: dict[str, Any]
    ) -> dict[str, Any]:
        if baseline.get("pairs"):
            return baseline
        pair_slots = None
        if kin.coupled_xy():
            layout = servo_strokes.corexy_fit_layout(gcmd, kin)
            pair_slots = layout["pairs"]
        derived = _copy_dynamics(baseline)
        if pair_slots is not None:
            pairs = [part.split(",") for part in pair_slots.split(";") if part]
            axis_index = {name: i for i, name in enumerate(baseline["axes"])}
            columns = [list(col) for col in zip(*baseline["frame"])]
            claimed: set[str] = set()
            for slots in pairs:
                if len(slots) != 2:
                    raise gcmd.error(
                        "kinematic AWD pair must contain exactly two slots "
                        "(got %s)" % (slots,)
                    )
                if slots[0] == slots[1]:
                    raise gcmd.error(
                        "kinematic AWD pair slots must be distinct (got %s)"
                        % (slots,)
                    )
                overlap = claimed.intersection(slots)
                if overlap:
                    raise gcmd.error(
                        "kinematic AWD pairs overlap at slots %s"
                        % (sorted(overlap),)
                    )
                if any(s not in axis_index for s in slots):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not match profile axes %s"
                        % (slots, baseline["axes"])
                    )
                claimed.update(slots)
                first, second = (axis_index[s] for s in slots)
                if not _equal_or_opposite_columns(
                    columns[first], columns[second]
                ):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not have equal parallel "
                        "or antiparallel frame columns" % (slots,)
                    )
            derived["pairs"] = [
                {"slots": slots, "direction_split": 0.0} for slots in pairs
            ]
        else:
            try:
                derived["pairs"] = discover_dynamics_pairs(baseline)
            except ValueError as e:
                raise gcmd.error("cannot derive dynamics pairs: %s" % (e,))
        if not derived["pairs"]:
            raise gcmd.error(
                "TERM=DIRECTION_SPLIT found no explicit [[pair]] tables, "
                "kinematic AWD pairs, or groups of exactly two equal "
                "parallel/antiparallel frame columns"
            )
        return derived

    def cmd_SERVO_REFINE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_REFINE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        term = gcmd.get("TERM", "MASS").upper()
        if term not in DYNAMICS_METRIC_BY_TERM:
            raise gcmd.error(
                "TERM must be MASS, VISCOUS, COULOMB or DIRECTION_SPLIT "
                "(got %r)" % (gcmd.get("TERM", ""),)
            )
        pattern = gcmd.get_int("PATTERN", 0)
        if pattern:
            if term == "DIRECTION_SPLIT":
                raise gcmd.error(
                    "TERM=DIRECTION_SPLIT needs rest-to-rest single-axis "
                    "strokes for per-move direction metrics - not "
                    "supported with PATTERN=1"
                )
            self._reject_pattern_stroke_bounds(gcmd)
        kin = self._kin()
        servos, rails, axis = self._grid_servos(gcmd, kin)
        node = self._refine_dynamics_node(gcmd, servos)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        if term == "DIRECTION_SPLIT":
            baseline = self._direction_split_baseline(gcmd, kin, baseline)
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        pattern_plan: dict[str, Any] = {}
        if pattern:
            points, start_x, start_y, pattern_plan = (
                self._pattern_geometry_params(gcmd)
            )

            def pattern_grid() -> None:
                self._goto_xy(start_x, start_y, dwell)
                for accel in accels:
                    for speed in speeds:
                        servo_strokes.emit_pattern(
                            self.gcode,
                            points,
                            start_x,
                            start_y,
                            speed,
                            accel,
                            iterations,
                            dwell,
                        )

        def axis_grid(
            ax: str, a_start: float, a_end: float, goto: tuple[float, float]
        ) -> Callable[[], None]:
            def run_grid() -> None:
                self._goto_xy(goto[0], goto[1], dwell)
                for accel in accels:
                    for speed in speeds:
                        self._strokes(
                            ax, a_start, a_end, speed, accel, iterations, dwell
                        )

            return run_grid

        def term_scale_fn(
            profile: dict[str, Any], scale: float
        ) -> dict[str, Any]:
            return scale_dynamics(profile, term, scale)

        if kin.coupled_xy():
            x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
                gcmd, self.bounds
            )
            x_center = (x_start + x_end) / 2.0
            y_center = (y_start + y_end) / 2.0

            def prep_axes() -> None:
                self._prep("X", dwell)
                self._prep("Y", dwell)

            x_grid = axis_grid("X", x_start, x_end, (x_start, y_center))
            y_grid = axis_grid("Y", y_start, y_end, (x_center, y_start))

            def both_grids() -> None:
                x_grid()
                y_grid()

            if term == "DIRECTION_SPLIT":

                def pair_add_fn(
                    index: int,
                ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                    def add_fn(
                        profile: dict[str, Any], delta: float
                    ) -> dict[str, Any]:
                        return add_dynamics_direction_split(
                            profile, index, delta
                        )

                    return add_fn

                phases = [
                    (
                        "direction_split_%s" % (pair["slots"][0],),
                        pair["slots"][0],
                        pair_add_fn(i),
                        both_grids,
                        list(pair["slots"]),
                    )
                    for i, pair in enumerate(baseline["pairs"])
                ]
            else:
                modes = baseline["modes"]
                if len(modes) != 2 or not {"x", "y"} <= set(modes):
                    raise gcmd.error(
                        "coupled_xy TERM=%s refine needs a 2-mode profile "
                        "with x and y modes; profile %s has modes %s"
                        % (term, profile_path, modes)
                    )

                def mode_scale_fn(
                    index: int,
                ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                    def scale_fn(
                        profile: dict[str, Any], scale: float
                    ) -> dict[str, Any]:
                        return scale_dynamics_mode(profile, term, index, scale)

                    return scale_fn

                grid_x = pattern_grid if pattern else x_grid
                grid_y = pattern_grid if pattern else y_grid
                phases = [
                    (
                        "%s_x" % (term.lower(),),
                        "x",
                        mode_scale_fn(modes.index("x")),
                        grid_x,
                        None,
                    ),
                    (
                        "%s_y" % (term.lower(),),
                        "y",
                        mode_scale_fn(modes.index("y")),
                        grid_y,
                        None,
                    ),
                ]
        else:
            start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)

            def prep_axes() -> None:
                if pattern:
                    self._prep("X", dwell)
                    self._prep("Y", dwell)
                else:
                    self._prep(axis, dwell)

            def cart_grid() -> None:
                for accel in accels:
                    for speed in speeds:
                        self._strokes(
                            axis, start, end, speed, accel, iterations, dwell
                        )

            if term == "DIRECTION_SPLIT":

                def pair_add_fn(
                    index: int,
                ) -> Callable[[dict[str, Any], float], dict[str, Any]]:
                    return lambda profile, delta: add_dynamics_direction_split(
                        profile, index, delta
                    )

                phases = [
                    (
                        "direction_split_%s" % (pair["slots"][0],),
                        pair["slots"][0],
                        pair_add_fn(i),
                        cart_grid,
                        list(pair["slots"]),
                    )
                    for i, pair in enumerate(baseline["pairs"])
                ]
            else:
                phases = [
                    (
                        term.lower(),
                        "",
                        term_scale_fn,
                        pattern_grid if pattern else cart_grid,
                        None,
                    )
                ]

        tag = gcmd.get("TAG", "refdyn")
        name = gcmd.get("NAME", "refined_%s" % (term.lower(),))
        if term == "DIRECTION_SPLIT":
            span = min(
                0.25,
                min(
                    0.9 * (0.5 - abs(pair["direction_split"]))
                    for pair in baseline["pairs"]
                ),
            )
            lo = gcmd.get_float("LO", -span)
            hi = gcmd.get_float("HI", span)
            tol = gcmd.get_float("TOL", 0.01, above=0.0)
        else:
            lo = gcmd.get_float("LO", 0.7, above=0.0)
            hi = gcmd.get_float("HI", 1.3)
            tol = gcmd.get_float("TOL", 0.02, above=0.0)
        max_evals = gcmd.get_int("MAX_EVALS", 10, minval=3)
        baseline_candidate = 0.0 if term == "DIRECTION_SPLIT" else 1.0
        if not lo < baseline_candidate < hi:
            raise gcmd.error(
                "bracket [LO, HI] = [%g, %g] must contain %g strictly - "
                "the search is centered on the baseline"
                % (lo, hi, baseline_candidate)
            )
        if term == "DIRECTION_SPLIT":
            for pair in baseline["pairs"]:
                base = pair["direction_split"]
                if abs(base + lo) >= 0.5 or abs(base + hi) >= 0.5:
                    raise gcmd.error(
                        "direction split delta bracket [%g, %g] takes pair "
                        "%s from %g outside abs(w) < 0.5"
                        % (lo, hi, pair["slots"], base)
                    )
        metric = DYNAMICS_METRIC_BY_TERM[term]
        stroke_plan = {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if pattern:
            stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_refine",
            tag,
            axis,
            servos,
            stroke_plan,
            rails,
        )
        run.manifest["dynamics_refine"] = {
            "baseline_profile": profile_path,
            "term": term.lower(),
            "metric": metric,
            "phases": [label for label, _s, _fn, _g, _d in phases],
            "bracket": [lo, hi],
            "tol": tol,
            "max_evals": max_evals,
        }
        run.write()
        report_metrics = ("overshoot", "ferr_rms", "ferr_peak")

        def metrics_line(values: dict[str, float]) -> str:
            return ", ".join("%s %.1f" % kv for kv in values.items())

        def run_phase(
            adapter: DynamicsModelAdapter,
            run_grid: Callable[[], None],
            drives: list[str] | None,
        ) -> tuple[float, float, float, dict[float, dict[str, float]]]:
            scores: dict[float, float] = {}
            reports: dict[float, dict[str, float]] = {}

            def gate_torque_rail(
                step_name: str, results: dict[str, Any]
            ) -> None:
                if "torque_saturated" in self._step_flags(results, step_name):
                    raise gcmd.error(
                        "step %s hit the torque rail - clipped strokes "
                        "cannot score a candidate, aborting refinement"
                        % (step_name,)
                    )

            def evaluate(scale: float) -> float:
                key = round(scale, 4)
                if key in scores:
                    return scores[key]
                step = self._engine.run_one(
                    adapter,
                    len(scores),
                    key,
                    max_evals + 1,
                    servos,
                    lambda _s: run_grid(),
                    gcmd,
                )
                results = self._run_analyze(gcmd, run, incremental=True)
                gate_torque_rail(step.name, results)
                reports[key] = {
                    m: self._step_metric_mean(gcmd, results, step.name, m)
                    for m in report_metrics
                }
                if drives is not None:
                    result_step = next(
                        (
                            item
                            for item in results.get("steps") or []
                            if item.get("name") == step.name
                        ),
                        None,
                    )
                    if result_step is None:
                        raise gcmd.error(
                            "step %r missing from results.json" % (step.name,)
                        )
                    try:
                        reports[key].update(
                            direction_split_candidate_metrics(
                                adapter.baseline, result_step, drives
                            )
                        )
                    except ValueError as e:
                        raise gcmd.error(str(e))
                gcmd.respond_info(
                    "%s %s %.4f -> %s (counts, mean per move)"
                    % (
                        adapter.label,
                        adapter.value_name,
                        key,
                        metrics_line(reports[key]),
                    )
                )
                scores[key] = reports[key][metric]
                return scores[key]

            baseline_score = evaluate(baseline_candidate)
            best, best_score, _probes = golden_section_search(
                evaluate, lo, hi, tol, max_evals
            )
            if baseline_score <= best_score:
                best, best_score = baseline_candidate, baseline_score
            return best, best_score, baseline_score, reports

        phase_out: list[tuple[str, str, float, float, float, dict]] = []
        adapters: list[DynamicsModelAdapter] = []
        refined = baseline
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            prep_axes()
            if pattern:
                self._pattern_reach_report(
                    gcmd, points, start_x, start_y, accels, speeds
                )
            for label, suffix, scale_fn, run_grid, drives in phases:
                adapter = DynamicsModelAdapter(
                    engine,
                    handle,
                    refined,
                    scale_fn,
                    label,
                    tag,
                    "delta" if term == "DIRECTION_SPLIT" else "scale",
                )
                adapters.append(adapter)
                best, best_score, baseline_score, reports = run_phase(
                    adapter, run_grid, drives
                )
                phase_out.append(
                    (label, suffix, best, best_score, baseline_score, reports)
                )
                refined = adapter.scaled(best)
        finally:
            try:
                if any(a.applied for a in adapters):
                    adapters[0].revert()
                    gcmd.respond_info(
                        "live dynamics model restored to baseline %s"
                        % (profile_path,)
                    )
            finally:
                self._restore()
                self._active_run = None
        for (
            label,
            _suffix,
            best,
            best_score,
            baseline_score,
            reports,
        ) in phase_out:
            for scale in sorted(reports):
                marker = "  <- best" if scale == round(best, 4) else ""
                gcmd.respond_info(
                    "  %s %s %.4f: %s%s"
                    % (
                        label,
                        "delta" if term == "DIRECTION_SPLIT" else "scale",
                        scale,
                        metrics_line(reports[scale]),
                        marker,
                    )
                )
            structured_log.event(
                "calibration",
                "dynamics_refined",
                run_dir=run.run_dir,
                term=label,
                metric=metric,
                best_scale=best,
                best_score=best_score,
                baseline_score=baseline_score,
                evals=len(reports),
            )
        if all(
            best == baseline_candidate
            for _l, _s, best, _bs, _bl, _r in phase_out
        ):
            gcmd.respond_info(
                "baseline already optimal within the bracket - no profile "
                "written | run %s" % (run.run_dir,)
            )
            return
        scales = {suffix: best for _l, suffix, best, _bs, _bl, _r in phase_out}
        with open(out_path, "w") as f:
            f.write(
                render_dynamics_toml(
                    refined, profile_path, term, scales, run.run_dir
                )
            )
        gcmd.respond_info(
            "%s | refined profile: %s | point [ethercat_node %s] "
            "dynamics_profile at it and RESTART | run %s"
            % (
                "; ".join(
                    "%s %s %.4f (%s %.1f -> %.1f)"
                    % (
                        label,
                        "delta" if term == "DIRECTION_SPLIT" else "scale",
                        best,
                        metric,
                        baseline_score,
                        best_score,
                    )
                    for label, _s, best, best_score, baseline_score, _r in (
                        phase_out
                    )
                ),
                out_path,
                node.name,
                run.run_dir,
            )
        )
