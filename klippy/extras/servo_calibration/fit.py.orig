from __future__ import annotations

import json
import os

from .refine import RefineCommands

try:
    import tomllib
except ImportError:
    tomllib = None

from collections.abc import Mapping
from typing import Any

from ... import structured_log
from .. import servo_strokes
from .dynamics import (
    DYNAMICS_TERM_KEYS,
    TUNE_MASS_FLOOR_FRACTION,
    TUNE_ZERO_FLOOR_STEPS,
    _copy_dynamics,
    dynamics_torque_changes,
    parse_dynamics_profile,
    render_fit_dynamics_toml,
    send_dynamics_model,
)
from .search import RmsLineSearch
from .sweep import ExperimentRun, SweepStep


class DynamicsFitCommands(RefineCommands):
    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward. On coupled_xy this "
        "is an iterative closed-loop identification: it runs the "
        "TEST_SPEED-style XY pattern (always - there is no PATTERN option), "
        "fits mass/viscous/coulomb (all three always regressed - the "
        "friction columns keep the mass estimate unbiased), streams the "
        "APPLIED model into the running endpoint, and re-captures with the "
        "feedforward active - "
        "with FF in the loop the drives track the command, so regressing "
        "measured torque against commanded kinematics loses its bias - "
        "until the parameters move less than TOL (torque-weighted, at the "
        "excitation ceiling) between rounds. It then re-identifies once at "
        "MAX_ACCEL: a converged model that shifts more than DRIFT there is "
        "a fit artifact, not physics, and the command aborts with the "
        "numbers. No SPEEDS matrix: give the calibration envelope as "
        "MAX_ACCEL/MAX_SPEED limits (e.g. capped below ringing; defaults "
        "are the config grid maxima) - convergence rounds run at half "
        "MAX_ACCEL, speeds at half and full MAX_SPEED. ACCELS=<comma list> "
        "runs an identify-only sweep instead: one capture + fit per accel "
        "under whatever model is currently live (nothing is streamed or "
        "applied), reporting mass per accel and the torque-weighted change "
        "between neighbours - the m(accel) curve that says whether the "
        "model extrapolates. The live model is "
        "restored to the configured dynamics_profile afterwards (also on "
        "failure); without one the last fitted model stays live until "
        "RESTART. TERMS picks what the applied/written model keeps "
        "(default MASS: with velocity_ff on, the speed-loop integrator "
        "already supplies friction torque at all but reversal transients, "
        "and a wrong friction FF is worse than none; fitted-but-dropped "
        "values are reported and recorded as fitted_* keys so enabling "
        "TERMS=MASS,COULOMB later is a data-driven call). Writes a "
        "timestamped node-level profile from the "
        "MAX_ACCEL verification fit. On non-coupled kinematics the "
        "single-shot per-axis grid fit remains (a per-motor candidate "
        "cannot be streamed into a multi-drive node), with params as "
        "SERVO_MEASURE_INERTIA plus DRIVE. Optional TORQUE_NM + "
        "INERTIA_KGM2 add the C00.06 recommendation. Params TERMS (MASS) "
        "MAX_ACCEL "
        "MAX_SPEED TOL (0.05) DRIFT (0.15) MAX_ROUNDS (4) ITERATIONS "
        "DWELL_MS BOUND SMALL_SIZE NAME SERVOS TORQUE_NM INERTIA_KGM2"
    )

    def _corexy_frame(
        self, gcmd: Any, kin: Any
    ) -> tuple[list[str], list[str], list[list[float]]]:
        rails = servo_strokes.axis_rails(gcmd, kin, "X")
        slots: list[tuple[str, int, float, int]] = []
        for belt_index, rail in enumerate(rails):
            motors = servo_strokes.rail_motors_in_slot_order(rail)
            drives = len(motors)
            for m in motors:
                sign = -1.0 if m.get_invert_direction() else 1.0
                slots.append((m.get_motor_name(), belt_index, sign, drives))
        axes = [name for name, _b, _s, _d in slots]
        frame_x = [sign / (2.0 * drives) for _n, _b, sign, drives in slots]
        frame_y = [
            (sign if belt == 0 else -sign) / (2.0 * drives)
            for _n, belt, sign, drives in slots
        ]
        return axes, ["x", "y"], [frame_x, frame_y]

    def _fit_plan(self, gcmd: Any) -> dict[str, Any]:
        kin = self._kin()
        if kin.coupled_xy():
            layout = servo_strokes.corexy_fit_layout(gcmd, kin)
            servo_strokes.check_servos_override(gcmd, layout)
            axes, modes, frame = self._corexy_frame(gcmd, kin)
            return {
                "corexy": True,
                "servos": layout["servos"],
                "axes": axes,
                "modes": modes,
                "frame": frame,
                "axis": "X",
                "rails": servo_strokes.axis_rails(gcmd, kin, "X"),
            }
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        drive = servo_strokes.scalar_fit_drive(gcmd, kin)
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        axes = [drive if drive is not None else servos[0]]
        return {
            "corexy": False,
            "servos": servos,
            "axes": axes,
            "modes": list(axes),
            "frame": [[1.0]],
            "axis": axis,
            "rails": None,
        }

    def _rotation_distance(self, gcmd: Any, servos: list[str]) -> float:
        distances = {
            self._resolve_motor(s).get_rotation_distance() for s in servos
        }
        if len(distances) != 1:
            raise gcmd.error(
                "drives disagree on rotation_distance (%s); cannot fit"
                % (sorted(distances),)
            )
        return distances.pop()

    def _fit_argv_for(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        scap: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
        response: str | None = None,
    ) -> list[str]:
        argv = [
            self._servo_cal(gcmd),
            "fit",
            "--capture",
            scap,
            "--frame",
            ";".join(
                ",".join("%g" % (f,) for f in row) for row in plan["frame"]
            ),
            "--modes",
            ",".join(plan["modes"]),
            "--axes",
            ",".join(plan["axes"]),
            "--out",
            out_path,
            "--rotation-distance-mm",
            "%g" % (self._rotation_distance(gcmd, plan["servos"]),),
        ]
        if torque is not None:
            argv += [
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ]
        if response is not None:
            argv += ["--response", response]
        return argv

    def _run_fit(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if self._kin().coupled_xy():
            if gcmd.get("ACCELS", None) is not None:
                return self._run_fit_sweep(gcmd, name, torque, inertia)
            return self._run_fit_iterative(gcmd, name, torque, inertia)
        return self._run_fit_grid(gcmd, name, torque, inertia)

    def _run_fit_grid(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if gcmd.get_int("PATTERN", 0):
            self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        run = self._begin_run(
            gcmd,
            "inertia_grid",
            name,
            plan["axis"],
            plan["servos"],
            self._grid_stroke_plan(gcmd),
            plan["rails"],
        )
        try:
            self._measure_inertia(gcmd, name)
            run.record_step(SweepStep(name, {}, []))
            out_path = self._dynamics_out_path(gcmd, run, name)
            argv = self._fit_argv_for(
                gcmd, plan, run.step_scap(name), out_path, torque, inertia
            )
            text = self._run(gcmd, argv, 120.0)
            gcmd.respond_info(
                "dynamics profile: %s | run %s" % (out_path, run.run_dir)
            )
        finally:
            self._active_run = None
        return run, text, out_path

    def _reject_fit_grid_params(self, gcmd: Any) -> None:
        stale = [
            p for p in ("SPEEDS", "PATTERN") if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: the iterative fit has no excitation matrix and always "
                "runs the XY pattern - give the calibration envelope as "
                "MAX_ACCEL/MAX_SPEED limits, or ACCELS=<comma list> for an "
                "identify-only sweep" % (", ".join(stale),)
            )

    def _validate_fit_slots(
        self, gcmd: Any, node: Any, profile: dict[str, Any]
    ) -> None:
        for slot, motor in enumerate(profile["axes"]):
            if node.get_slot_for_motor(motor) != slot:
                raise gcmd.error(
                    "fitted profile axis %r is at slot %d but node %s maps "
                    "it to %s - cannot stream the candidate model"
                    % (motor, slot, node.name, node.get_slot_for_motor(motor))
                )

    def _fit_round(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        run: ExperimentRun,
        step: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[dict[str, Any], str]:
        argv = self._fit_argv_for(
            gcmd, plan, run.step_scap(step), out_path, torque, inertia
        )
        text = self._run(gcmd, argv, 120.0)
        try:
            with open(out_path) as f:
                fitted = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit for step %s produced an unusable profile "
                "%s: %s" % (step, out_path, e)
            )
        return fitted, text

    def _dynamics_params_line(self, profile: dict[str, Any]) -> str:
        return " | ".join(
            "%s mass %.5g viscous %.5g coulomb %.5g"
            % (
                mode,
                profile["mass"][k],
                profile["viscous"][k],
                profile["coulomb"][k],
            )
            for k, mode in enumerate(profile["modes"])
        )

    def _run_fit_iterative(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_fit_grid_params(gcmd)
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        node = self._refine_dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        restore = None
        if node.get_dynamics_profile() is not None:
            _path, restore = self._load_baseline_dynamics(gcmd, node)
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        tol = gcmd.get_float("TOL", 0.05, above=0.0)
        drift = gcmd.get_float("DRIFT", 0.15, above=0.0)
        max_rounds = gcmd.get_int("MAX_ROUNDS", 4, minval=2)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS").split(",")
            if t.strip()
        ]
        if (
            not terms
            or any(t not in DYNAMICS_TERM_KEYS for t in terms)
            or "MASS" not in terms
        ):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB and include MASS (got %r)" % (gcmd.get("TERMS", ""),)
            )
        dropped = [
            key for term, key in DYNAMICS_TERM_KEYS.items() if term not in terms
        ]

        def applied_model(full: dict[str, Any]) -> dict[str, Any]:
            trimmed = _copy_dynamics(full)
            for key in dropped:
                trimmed[key] = [0.0] * len(full[key])
            return trimmed

        def round_line(full: dict[str, Any], trimmed: dict[str, Any]) -> str:
            line = self._dynamics_params_line(trimmed)
            if dropped:
                line += " | fitted but not applied: " + ", ".join(
                    "%s [%s]"
                    % (key, ", ".join("%.5g" % (v,) for v in full[key]))
                    for key in dropped
                )
            return line

        converge_accel = max_accel / 2.0
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "converge_accel": converge_accel,
            "speeds": speeds,
            "tol": tol,
            "drift": drift,
            "max_rounds": max_rounds,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_fit",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str, accel: float) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
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
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": accel}, []))

        def torque_changes(
            prev: dict[str, Any],
            new: dict[str, Any],
            accel: float,
        ) -> list[float]:
            try:
                return dynamics_torque_changes(prev, new, accel, max_speed)
            except ValueError as e:
                raise gcmd.error(str(e))

        applied = False
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd,
                points,
                start_x,
                start_y,
                [converge_accel, max_accel],
                speeds,
            )
            prev = None
            fitted = None
            converged = False
            last_change = None
            rounds_run = 0
            for round_i in range(max_rounds):
                step = "fit_r%d" % (round_i,)
                capture_round(step, converge_accel)
                fitted_full, _text = self._fit_round(
                    gcmd,
                    plan,
                    run,
                    step,
                    os.path.join(run.run_dir, "dynamics_%s.toml" % (step,)),
                    None,
                    None,
                )
                fitted = applied_model(fitted_full)
                rounds_run = round_i + 1
                if round_i == 0:
                    self._validate_fit_slots(gcmd, node, fitted)
                send_dynamics_model(engine, handle, fitted)
                applied = True
                if prev is None:
                    gcmd.respond_info(
                        "round %d: %s (feedforward now live for the next "
                        "round)" % (round_i, round_line(fitted_full, fitted))
                    )
                else:
                    last_change = max(
                        torque_changes(prev, fitted, converge_accel)
                    )
                    gcmd.respond_info(
                        "round %d: %s | torque-weighted change %.1f%% "
                        "(TOL %.1f%%)"
                        % (
                            round_i,
                            round_line(fitted_full, fitted),
                            100.0 * last_change,
                            100.0 * tol,
                        )
                    )
                    if last_change <= tol:
                        converged = True
                        break
                prev = fitted
            if not converged:
                raise gcmd.error(
                    "dynamics fit did not converge in %d rounds at accel "
                    "%.0f (last torque-weighted change %.1f%% > TOL %.1f%%) "
                    "- the identification is not settling; inspect run %s"
                    % (
                        max_rounds,
                        converge_accel,
                        100.0
                        * (last_change if last_change is not None else 1.0),
                        100.0 * tol,
                        run.run_dir,
                    )
                )
            capture_round("fit_verify", max_accel)
            verified_full, text = self._fit_round(
                gcmd,
                plan,
                run,
                "fit_verify",
                os.path.join(run.run_dir, "dynamics_fit_verify.toml"),
                torque,
                inertia,
            )
            verified = applied_model(verified_full)
            shift = max(torque_changes(fitted, verified, max_accel))
            if shift > drift:
                raise gcmd.error(
                    "converged model does not hold at MAX_ACCEL %.0f: "
                    "re-identification shifted the parameters %.1f%% "
                    "(DRIFT %.1f%%) - converged %s vs verify %s | the fit "
                    "at accel %.0f was an artifact of that operating "
                    "point, not physics; lower MAX_ACCEL below the "
                    "regime change or investigate | run %s"
                    % (
                        max_accel,
                        100.0 * shift,
                        100.0 * drift,
                        self._dynamics_params_line(fitted),
                        self._dynamics_params_line(verified),
                        converge_accel,
                        run.run_dir,
                    )
                )
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        verified, verified_full, terms, run.run_dir
                    )
                )
            run.manifest["dynamics_fit"] = {
                "rounds": rounds_run,
                "converged_change": last_change,
                "verify_shift": shift,
                "terms": [t.lower() for t in terms],
                "fitted_not_applied": {
                    key: verified_full[key] for key in dropped
                },
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_fit",
                run_dir=run.run_dir,
                rounds=rounds_run,
                converged_change=last_change,
                verify_shift=shift,
                profile=out_path,
            )
            gcmd.respond_info(
                "converged in %d rounds (change %.1f%%), holds at MAX_ACCEL "
                "%.0f (shift %.1f%% <= DRIFT %.1f%%) | dynamics profile: %s "
                "| run %s"
                % (
                    rounds_run,
                    100.0 * (last_change or 0.0),
                    max_accel,
                    100.0 * shift,
                    100.0 * drift,
                    out_path,
                    run.run_dir,
                )
            )
        finally:
            try:
                if applied:
                    if restore is not None:
                        send_dynamics_model(engine, handle, restore)
                        gcmd.respond_info(
                            "live dynamics model restored to configured "
                            "baseline"
                        )
                    else:
                        gcmd.respond_info(
                            "WARNING: no dynamics_profile configured - the "
                            "last fitted model stays live until RESTART"
                        )
            finally:
                self._restore()
                self._active_run = None
        return run, text, out_path

    def _run_fit_sweep(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        """Identify-only m(accel) curve: one pattern capture + fit per
        ACCELS entry, run under whatever dynamics model is currently live
        (nothing is streamed), so the points differ only in accel."""
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        stale = [
            p
            for p in (
                "SPEEDS",
                "PATTERN",
                "TOL",
                "DRIFT",
                "MAX_ROUNDS",
                "MAX_ACCEL",
                "TERMS",
            )
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: ACCELS runs an identify-only sweep - it takes only "
                "MAX_SPEED, ITERATIONS, DWELL_MS, NAME, SERVOS and the "
                "pattern geometry" % (", ".join(stale),)
            )
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        raw = gcmd.get("ACCELS")
        try:
            accels = [float(v) for v in raw.split(",") if v.strip()]
        except ValueError:
            raise gcmd.error(
                "ACCELS must be a comma list of accelerations (got %r)" % (raw,)
            )
        if (
            len(accels) < 2
            or any(a <= 0.0 for a in accels)
            or sorted(accels) != accels
            or len(set(accels)) != len(accels)
        ):
            raise gcmd.error(
                "ACCELS wants at least two distinct ascending positive "
                "accelerations (got %r)" % (raw,)
            )
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "accels": accels,
            "max_speed": max_speed,
            "speeds": speeds,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_sweep",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )
        text = ""
        out_path = ""
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, accels, speeds
            )
            fits: list[tuple[float, dict[str, Any]]] = []
            for accel in accels:
                step = "fit_a%d" % (round(accel),)
                self._start_capture(step, plan["servos"])
                self._goto_xy(start_x, start_y, dwell)
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
                self._stop_capture()
                run.record_step(SweepStep(step, {"accel": accel}, []))
                out_path = os.path.join(
                    run.run_dir, "dynamics_%s.toml" % (step,)
                )
                fitted, text = self._fit_round(
                    gcmd, plan, run, step, out_path, torque, inertia
                )
                fits.append((accel, fitted))
                gcmd.respond_info(
                    "accel %.0f: %s"
                    % (accel, self._dynamics_params_line(fitted))
                )
            for (a0, f0), (a1, f1) in zip(fits, fits[1:]):
                try:
                    change = max(dynamics_torque_changes(f0, f1, a1, max_speed))
                except ValueError as e:
                    raise gcmd.error(str(e))
                gcmd.respond_info(
                    "accel %.0f -> %.0f: torque-weighted change %.1f%%"
                    % (a0, a1, 100.0 * change)
                )
            modes = fits[0][1]["modes"]
            curve = {
                mode: [f[1]["mass"][k] for f in fits]
                for k, mode in enumerate(modes)
            }
            for mode in modes:
                masses = curve[mode]
                lo, hi = min(masses), max(masses)
                gcmd.respond_info(
                    "mode %s mass(accel): %s | spread %.1f%%"
                    % (
                        mode,
                        ", ".join(
                            "%.0f: %.5g" % (a, m)
                            for (a, _f), m in zip(fits, masses)
                        ),
                        200.0 * (hi - lo) / (hi + lo),
                    )
                )
            run.manifest["dynamics_sweep"] = {
                "accels": accels,
                "mass": curve,
                "max_speed": max_speed,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_sweep",
                run_dir=run.run_dir,
                accels=accels,
            )
            gcmd.respond_info(
                "identify-only sweep done - nothing was applied | run %s"
                % (run.run_dir,)
            )
        finally:
            self._restore()
            self._active_run = None
        return run, text, out_path

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=False)
        self._run_fit(gcmd, gcmd.get("NAME", "ident"), torque, inertia)

    def _reject_tune_dynamics_params(self, gcmd: Any) -> None:
        stale = [
            p
            for p in ("ACCELS", "SPEEDS", "PATTERN")
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: SERVO_TUNE_DYNAMICS always drives the XY pattern "
                "excitation at MAX_ACCEL/MAX_SPEED - it has no excitation "
                "matrix and no PATTERN toggle to override" % (", ".join(stale),)
            )

    def _load_ferr_fit(self, gcmd: Any, path: str) -> dict[str, Any]:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit --response ferr produced an unusable result "
                "%s: %s" % (path, e)
            )
        if data.get("version") != 1:
            raise gcmd.error(
                "ferr fit %s: unsupported version %r (expected 1)"
                % (path, data.get("version"))
            )
        n_modes = len(data.get("modes", []))
        for key in ("ferr_rms_raw", "onset_bias"):
            vec = data.get(key)
            if not isinstance(vec, list) or len(vec) != n_modes:
                raise gcmd.error(
                    "ferr fit %s has no per-mode %s - rebuild servo-cal "
                    "(make servo-cal), the binary predates the "
                    "rms-objective tuner" % (path, key)
                )
        return data

    cmd_SERVO_TUNE_DYNAMICS_help = (
        "Empirical closed-loop dynamics tuner on coupled_xy: coordinate "
        "descent that MEASURES tracking error instead of trusting a "
        "fitted correlation. Each round streams the trial model to the "
        "running endpoint (no restart), captures one XY pattern run at "
        "MAX_ACCEL/MAX_SPEED, and scores each mode by the raw wide-band "
        "rms of its following error over the whole capture - the number "
        "the operator actually experiences. (The ferr/accel regression "
        "is still fitted and reported per round, but only as a direction "
        "hint and diagnostic: on the bench its zero landed at 2.2x the "
        "rms-optimal mass while tracking got worse every round.) Terms "
        "tune one at a time in TERMS order, both modes per capture, each "
        "as a 1-D line search: the mass probe's first direction follows "
        "the ONSET BIAS (mean sign(accel)*ferr right after each accel "
        "step - the manual heuristic: only the first excursion when "
        "torque lands carries clean command-path sign, before the "
        "drive's own compensation reacts; positive = under-fed), other "
        "terms follow their regression coefficient's sign; a failed "
        "first probe flips once, the step grows while rms "
        "improves by more than TOL_UM (relative change capped at 40%% "
        "per probe), and the first non-improving probe triggers one "
        "parabolic refine through the bracket; ties go to the best "
        "measured value. Viscous/coulomb are floored at zero (a "
        "zero-valued term probes up by a fixed floor step), mass at 10%% "
        "of its baseline. Passes over the terms repeat until a full "
        "pass improves nothing, then the best model is written as a "
        "dynamics TOML and left LIVE (point [ethercat_node] "
        "dynamics_profile at it and RESTART to keep it). There is no "
        "round budget: the search runs until it converges (kill it if "
        "it overstays). torque_saturated aborts, restores the baseline "
        "and writes nothing; resonance_detected only warns. The "
        "baseline is PROFILE= or the node-level [ethercat_node] "
        "dynamics_profile (per-motor profiles are not supported). Params "
        "MAX_ACCEL MAX_SPEED STEP (0.15) TOL_UM (0.05) "
        "TERMS (mass,viscous,coulomb) NAME (tune) PROFILE SERVOS BOUND "
        "SMALL_SIZE"
    )

    def cmd_SERVO_TUNE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_tune_dynamics_params(gcmd)
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires coupled_xy kinematics - the "
                "ferr regression needs the mode-space frame"
            )
        plan = self._fit_plan(gcmd)
        node = self._refine_dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        baseline_modes = baseline["modes"]
        if len(baseline_modes) != 2 or not {"x", "y"} <= set(baseline_modes):
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS needs a 2-mode profile with x and y "
                "modes; profile %s has modes %s"
                % (profile_path, baseline_modes)
            )
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS,VISCOUS,COULOMB").split(",")
            if t.strip()
        ]
        if not terms or any(t not in DYNAMICS_TERM_KEYS for t in terms):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB (got %r)" % (gcmd.get("TERMS", ""),)
            )
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        step_frac = gcmd.get_float("STEP", 0.15, minval=0.02, maxval=0.5)
        tol_mm = 1e-3 * gcmd.get_float("TOL_UM", 0.05, above=0.0)
        name = gcmd.get("NAME", "tune")
        dwell = self.dwell_ms
        iterations = self.iterations
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "speeds": speeds,
            "step": step_frac,
            "tol_um": tol_mm * 1e3,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_tune",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    max_accel,
                    iterations,
                    dwell,
                )
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": max_accel}, []))

        current = _copy_dynamics(baseline)
        rounds_history: list[dict[str, Any]] = []
        search_summaries: list[dict[str, Any]] = []
        measured: dict[tuple[float, ...], dict[str, Any]] = {}
        applied = False
        success = False

        def model_key(values: Mapping[str, Any]) -> tuple[float, ...]:
            return tuple(
                float(v)
                for term in ("MASS", "VISCOUS", "COULOMB")
                for v in values[DYNAMICS_TERM_KEYS[term]]
            )

        def measure(round_i: int, trial: dict[str, Any]) -> dict[str, Any]:
            send_dynamics_model(engine, handle, trial)
            step = "tune_r%d" % (round_i,)
            capture_round(step)
            results = self._run_analyze(gcmd, run, incremental=True)
            flags = set(self._step_flags(results, step))
            if "torque_saturated" in flags:
                raise gcmd.error(
                    "step %s hit the torque rail - clipped strokes "
                    "cannot score tracking error, aborting "
                    "SERVO_TUNE_DYNAMICS" % (step,)
                )
            if "resonance_detected" in flags:
                gcmd.respond_info(
                    "WARNING step %s flagged resonance_detected - "
                    "continuing (feedforward tuning does not move the "
                    "loop's resonances)" % (step,)
                )
            ferr_out = os.path.join(run.run_dir, "ferr_r%d.json" % (round_i,))
            argv = self._fit_argv_for(
                gcmd,
                plan,
                run.step_scap(step),
                ferr_out,
                None,
                None,
                response="ferr",
            )
            self._run(gcmd, argv, 120.0)
            ferr = self._load_ferr_fit(gcmd, ferr_out)
            if ferr.get("modes") != plan["modes"]:
                raise gcmd.error(
                    "servo-cal fit --response ferr modes %s do not "
                    "match the requested modes %s"
                    % (ferr.get("modes"), plan["modes"])
                )
            return {
                "round": round_i,
                "rms": [float(v) for v in ferr["ferr_rms_raw"]],
                "coef": ferr["coef"],
                "stderr": ferr["stderr"],
                "onset": [float(v) for v in ferr["onset_bias"]],
                "samples": ferr.get("samples"),
            }

        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, [max_accel], speeds
            )
            phase_idx = 0
            searches: dict[str, RmsLineSearch] | None = None
            pass_improved = False
            round_i = 0
            while True:
                term = terms[phase_idx]
                key = DYNAMICS_TERM_KEYS[term]
                trial = _copy_dynamics(current)
                if searches is not None:
                    for mode, search in searches.items():
                        idx = baseline_modes.index(mode)
                        trial[key][idx] = (
                            search.best if search.done else search.trial
                        )
                cache_key = model_key(trial)
                cached = measured.get(cache_key)
                if cached is None:
                    applied = True
                    cached = measure(round_i, trial)
                    measured[cache_key] = cached
                    rms = cached["rms"]
                    label = "baseline" if searches is None else term.lower()
                    line = " | ".join(
                        "%s %s=%.6g rms=%.2fum (onset %+.2fum, g=%+.3g)"
                        % (
                            mode,
                            key,
                            trial[key][baseline_modes.index(mode)],
                            rms[fit_idx] * 1e3,
                            cached["onset"][fit_idx] * 1e3,
                            cached["coef"][key][fit_idx],
                        )
                        for fit_idx, mode in enumerate(plan["modes"])
                    )
                    gcmd.respond_info("r%d [%s] %s" % (round_i, label, line))
                    rounds_history.append(
                        {
                            "round": round_i,
                            "term": label,
                            "values": {
                                DYNAMICS_TERM_KEYS[t]: list(
                                    trial[DYNAMICS_TERM_KEYS[t]]
                                )
                                for t in terms
                            },
                            "ferr_rms_raw": list(rms),
                            "coef": dict(cached["coef"]),
                            "stderr": dict(cached["stderr"]),
                            "onset_bias": list(cached["onset"]),
                            "samples": cached["samples"],
                        }
                    )
                    round_i += 1
                rms = cached["rms"]
                if searches is None:
                    searches = {}
                    for fit_idx, mode in enumerate(plan["modes"]):
                        idx = baseline_modes.index(mode)
                        value = current[key][idx]
                        if term == "MASS":
                            lo = TUNE_MASS_FLOOR_FRACTION * baseline[key][idx]
                            step_size = step_frac * abs(value)
                        else:
                            lo = 0.0
                            step_size = (
                                step_frac * abs(value)
                                if value != 0.0
                                else TUNE_ZERO_FLOOR_STEPS[term]
                            )
                        hint = float(cached["coef"][key][fit_idx])
                        if term == "MASS" and cached["onset"][fit_idx] != 0.0:
                            hint = cached["onset"][fit_idx]
                        searches[mode] = RmsLineSearch(
                            value,
                            rms[fit_idx],
                            step_size,
                            tol_mm,
                            lo=lo,
                            hint=hint if hint != 0.0 else 1.0,
                        )
                else:
                    for fit_idx, mode in enumerate(plan["modes"]):
                        search = searches[mode]
                        if not search.done:
                            search.feed(rms[fit_idx])
                if all(search.done for search in searches.values()):
                    lines = []
                    for mode, search in searches.items():
                        idx = baseline_modes.index(mode)
                        current[key][idx] = search.best
                        pass_improved = pass_improved or search.improved
                        lines.append(
                            "%s %.6g @ %.2fum (%s)"
                            % (
                                mode,
                                search.best,
                                search.best_rms * 1e3,
                                search.note,
                            )
                        )
                        search_summaries.append(
                            {
                                "term": term.lower(),
                                "mode": mode,
                                "best": search.best,
                                "best_rms": search.best_rms,
                                "improved": search.improved,
                                "note": search.note,
                                "probes": len(search.history) - 1,
                            }
                        )
                    gcmd.respond_info(
                        "%s settled: %s" % (term.lower(), " | ".join(lines))
                    )
                    searches = None
                    phase_idx += 1
                    if phase_idx == len(terms):
                        if not pass_improved:
                            success = True
                            break
                        phase_idx = 0
                        pass_improved = False
            send_dynamics_model(engine, handle, current)
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        current,
                        current,
                        [t.lower() for t in terms],
                        run.run_dir,
                    )
                )
            run.manifest["dynamics_tune"] = {
                "terms": [t.lower() for t in terms],
                "max_accel": max_accel,
                "max_speed": max_speed,
                "step": step_frac,
                "tol_um": tol_mm * 1e3,
                "rounds": rounds_history,
                "search": search_summaries,
                "converged": True,
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_tune",
                run_dir=run.run_dir,
                rounds=len(rounds_history),
                profile=out_path,
            )
            gcmd.respond_info(
                "SERVO_TUNE_DYNAMICS converged in %d captures | tuned "
                "dynamics profile: %s | tuned model stays live until "
                "RESTART - point [ethercat_node %s] dynamics_profile at "
                "it to keep it | run %s"
                % (len(rounds_history), out_path, node.name, run.run_dir)
            )
        finally:
            try:
                if applied and not success:
                    send_dynamics_model(engine, handle, baseline)
                    gcmd.respond_info(
                        "live dynamics model restored to baseline %s"
                        % (profile_path,)
                    )
            finally:
                self._restore()
                self._active_run = None
