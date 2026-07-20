from __future__ import annotations

from typing import Any

from ... import structured_log
from .. import servo_strokes
from .autotune import AUTOTUNE_STAGES, AutotuneContext
from .gains import GainCommands
from .params import C00_06_INERTIA_RATIO_MAX, INERTIA_RATIO_ADDR
from .sweep import InertiaRatioAdapter, MotionAccelAdapter, SweepStep


class SweepCommands(GainCommands):
    cmd_SERVO_SWEEP_INERTIA_help = (
        "Empirical inertia sweep, gain-sweep style. Resolves every servo "
        "driving AXIS (both drives on CoreXY), writes each C00.06 ratio in "
        "RATIOS (percent, comma list) identically to all of them, one capture "
        "per step of all drives into a run directory, then servo-cal analyzes "
        "it into results.json. Restores the "
        "original ratio afterwards (also on failure). No automated pick "
        "(read the overshoot trend across steps), so APPLY=1 always errors "
        "here - use SERVO_SET_INERTIA_RATIO once you have chosen a value. "
        "Params RATIOS AXIS "
        "START END SPEED ACCEL ITERATIONS DWELL_MS TAG APPLY SERVO (comma "
        "list override)"
    )

    def cmd_SERVO_SWEEP_INERTIA(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "inertia")
        apply = gcmd.get_int("APPLY", 0)
        ratios: list[int] = []
        for r in self._floats(gcmd.get("RATIOS", "40,70,100,130")):
            rv = int(r)
            if not 0 <= rv <= C00_06_INERTIA_RATIO_MAX:
                raise gcmd.error(
                    "ratio %d outside C00.06 range 0..%d (%%)"
                    % (rv, C00_06_INERTIA_RATIO_MAX)
                )
            if rv not in ratios:
                ratios.append(rv)
        ratios.sort()
        original = self._read_param(servos[0], INERTIA_RATIO_ADDR)
        stroke_plan = {
            "start": start,
            "end": end,
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "inertia_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = InertiaRatioAdapter(self, servos, tag, original)

        def on_revert() -> None:
            gcmd.respond_info(
                "restoring C00.06 ratio %d%% on %s"
                % (original, ", ".join(servos))
            )

        try:
            self._prep(axis, dwell)
            steps = self._run_sweep_with_revert(
                adapter,
                ratios,
                servos,
                lambda rv: self._strokes(
                    axis, start, end, speed, accel, iterations, dwell
                ),
                gcmd,
                on_revert,
            )
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            self._active_run = None
        return steps

    cmd_SERVO_SWEEP_ACCEL_help = (
        "Accel sweep to find the max non-saturating acceleration. Runs one "
        "capture of strokes per ACCELS entry (mm/s^2, comma list, toolhead "
        "frame) named step_<TAG>_a<ACCEL>, then servo-cal analyzes the run "
        "into results.json (verdict: the highest non-railing accel). "
        "AXIS=X/Y strokes a single axis; AXIS=A/B strokes a CoreXY diagonal so "
        "one motor carries the whole load (belt accel is sqrt(2)x on a "
        "diagonal). Restores the velocity limit afterwards (also on failure). "
        "servo-cal flags samples at/above its 1400 per-mille torque ceiling "
        "as railed. APPLY=1 has no register to write (ACCEL is a stroke-plan "
        "parameter, not an SDO), so it runs the verification stroke at the "
        "recommended accel and reports before/after tracking metrics "
        "(default APPLY=0, report-only). "
        "Params ACCELS AXIS SPEED START END ITERATIONS DWELL_MS TAG APPLY"
    )

    def cmd_SERVO_SWEEP_ACCEL(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "accel")
        apply = gcmd.get_int("APPLY", 0)
        raw = self._floats(gcmd.get("ACCELS", None))
        if not raw:
            raise gcmd.error("ACCELS= required (comma list of mm/s^2)")
        accels: list[int] = []
        for a in raw:
            av = int(a)
            if av <= 0:
                raise gcmd.error("accel %d must be positive (mm/s^2)" % (av,))
            if av not in accels:
                accels.append(av)
        accels.sort()
        servos = plan.servos
        stroke_plan = {
            "start": plan.start,
            "end": plan.end,
            "speed": speed,
            "accel": None,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd,
            "accel_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = MotionAccelAdapter(tag)

        def run_step(av: int) -> None:
            servo_strokes.emit_strokes(
                self.gcode,
                plan.coord,
                plan.start,
                plan.end,
                plan.th_per_unit,
                speed,
                float(av),
                iterations,
                dwell,
            )

        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            try:
                steps = self._engine.run(
                    adapter, accels, servos, run_step, gcmd
                )
            finally:
                self._restore()
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                self._apply_verdict(gcmd, run, results, axis)
        finally:
            self._active_run = None
        return steps

    cmd_SERVO_AUTOTUNE_help = (
        "Packaged tuning sequence: baseline tracking -> inertia ratio "
        "identify -> apply C00.06 -> coarse gains (SERVO_APPLY_GAINS "
        "defaults) -> gain sweep (apply winner) "
        "-> fit dynamics -> verify vs baseline. APPLY=0 "
        "(default) is a dry run: it still measures the baseline and "
        "identifies the inertia ratio, then walks every remaining stage "
        "reporting what it WOULD write instead of touching the drive. "
        "APPLY=1 performs every stage for real and aborts loudly, naming "
        "the stage and run directory, on a torque/resonance flag on the "
        "chosen step, a null recommendation, or a final following-error "
        "regression over 20%% vs baseline. Never persists the result - "
        "run SERVO_SAVE_TUNING SERVO=... NAME=... afterwards. Params AXIS "
        "APPLY TORQUE_NM INERTIA_KGM2 SPEED_GAINS DWELL_MS"
    )

    def cmd_SERVO_AUTOTUNE(self, gcmd: Any) -> list[dict[str, Any]]:
        axis = gcmd.get("AXIS", "X").upper()
        apply = bool(gcmd.get_int("APPLY", 0))
        torque, inertia = self._motor(gcmd, required=False)
        if apply and (torque is None or inertia is None):
            raise gcmd.error(
                "SERVO_AUTOTUNE APPLY=1 requires rated_torque_nm/"
                "rotor_inertia_kgm2 (config or TORQUE_NM=/INERTIA_KGM2=) "
                "before the inertia_ratio stage runs"
            )
        ctx = AutotuneContext(
            gcmd=gcmd,
            axis=axis,
            apply=apply,
            torque_nm=torque,
            inertia_kgm2=inertia,
            speed_gains=gcmd.get("SPEED_GAINS", None),
            dwell_ms=gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0),
        )
        outcomes: list[dict[str, Any]] = []
        for stage in AUTOTUNE_STAGES:
            outcome = stage.run(self, ctx)
            structured_log.event(
                "calibration",
                "autotune_stage",
                stage=stage.name,
                run_dir=outcome.get("run_dir"),
                outcome=outcome.get("outcome"),
            )
            gcmd.respond_info(
                "autotune stage %s: %s" % (stage.name, outcome.get("outcome"))
            )
            outcomes.append({"stage": stage.name, **outcome})
        gcmd.respond_info(
            "\n".join(
                ["SERVO_AUTOTUNE summary:"]
                + ["  %-20s %s" % (o["stage"], o["outcome"]) for o in outcomes]
            )
        )
        if apply:
            gcmd.respond_info(
                "nothing persisted - run SERVO_SAVE_TUNING SERVO=... "
                "NAME=... to keep this result"
            )
        return outcomes
