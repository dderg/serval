from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .sweeps import SweepCommands

from dataclasses import dataclass
from typing import Any

from .common import _applied, _parse_c0006_recommendation
from .params import INERTIA_RATIO_ADDR
from .sweep import ExperimentRun, _OverrideGcmd

COARSE_GAINS = {"position": 400, "speed": 250, "integral": 3184}


@dataclass
class AutotuneContext:
    """State threaded through SERVO_AUTOTUNE's stage list - one instance per
    invocation, mutated in place as each stage records what it found."""

    gcmd: Any
    axis: str
    apply: bool
    torque_nm: float | None
    inertia_kgm2: float | None
    speed_gains: str | None
    dwell_ms: int
    baseline_run: ExperimentRun | None = None
    baseline_results: dict[str, Any] | None = None
    recommended_ratio: int | None = None

    def overrides(self, **extra: Any) -> dict[str, Any]:
        merged: dict[str, Any] = {"AXIS": self.axis, "DWELL_MS": self.dwell_ms}
        merged.update(extra)
        return merged

    def gcmd_for(self, **extra: Any) -> Any:
        return _OverrideGcmd(self.gcmd, self.overrides(**extra))


class AutotuneStage:
    name = "unnamed"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        raise NotImplementedError


class BaselineTrackingStage(AutotuneStage):
    name = "baseline"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        run, results = cal._measure_tracking(
            ctx.gcmd_for(), ctx.axis, "autotune_baseline"
        )
        ctx.baseline_run = run
        ctx.baseline_results = results
        return {"outcome": "ran", "run_dir": run.run_dir}


class InertiaRatioIdentifyStage(AutotuneStage):
    name = "inertia_ratio"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        run, text, _out_path = cal._run_fit(
            ctx.gcmd_for(), "autotune_inertia", ctx.torque_nm, ctx.inertia_kgm2
        )
        ratio = _parse_c0006_recommendation(text)
        if ratio is None:
            raise ctx.gcmd.error(
                "SERVO_AUTOTUNE: aborting at stage %r (run %s): could not "
                "parse a C00.06 recommendation from servo-cal fit output"
                % (self.name, run.run_dir)
            )
        ctx.recommended_ratio = ratio
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "recommended_ratio": ratio,
        }


class ApplyInertiaRatioStage(AutotuneStage):
    name = "apply_inertia_ratio"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        servos = cal._servos(ctx.gcmd, ctx.axis)
        if not ctx.apply:
            return {
                "outcome": "would_run",
                "ratio": ctx.recommended_ratio,
                "servos": servos,
            }
        assert ctx.recommended_ratio is not None
        applied = [
            _applied(s, INERTIA_RATIO_ADDR, ctx.recommended_ratio)
            for s in servos
        ]
        cal._issue_apply_writes(ctx.gcmd, applied)
        return {
            "outcome": "ran",
            "ratio": ctx.recommended_ratio,
            "servos": servos,
        }


class CoarseGainsStage(AutotuneStage):
    name = "coarse_gains"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        if not ctx.apply:
            return {"outcome": "would_run", "gains": COARSE_GAINS}
        cal.cmd_SERVO_APPLY_GAINS(ctx.gcmd_for())
        return {"outcome": "ran", "gains": COARSE_GAINS}


class GainSweepStage(AutotuneStage):
    name = "gain_sweep"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        extra: dict[str, Any] = {
            "TAG": "autotune_gain",
            "APPLY": 1 if ctx.apply else 0,
        }
        if ctx.speed_gains is not None:
            extra["SPEED_GAINS"] = ctx.speed_gains
        cal.cmd_SERVO_CALIBRATE_GAINS(ctx.gcmd_for(**extra))
        run, results = cal._last_sweep_run, cal._last_sweep_results
        assert run is not None and results is not None
        verdict = cal._check_clean_verdict(
            ctx.gcmd, self.name, run, results, require_step=ctx.apply
        )
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "recommended_step": verdict.get("recommended_step"),
        }


class FitDynamicsStage(AutotuneStage):
    name = "fit_dynamics"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        if not ctx.apply:
            return {
                "outcome": "would_run",
                "detail": "fit dynamics at the final tuned gains",
            }
        run, _text, out_path = cal._run_fit(
            ctx.gcmd_for(), "autotune_dynamics", ctx.torque_nm, ctx.inertia_kgm2
        )
        return {"outcome": "ran", "run_dir": run.run_dir, "profile": out_path}


class VerifyStage(AutotuneStage):
    name = "verify"

    def run(self, cal: "SweepCommands", ctx: AutotuneContext) -> dict[str, Any]:
        if not ctx.apply:
            return {
                "outcome": "skipped",
                "reason": "dry run - nothing was applied",
            }
        assert ctx.baseline_run is not None and ctx.baseline_results is not None
        run, results = cal._measure_tracking(
            ctx.gcmd_for(), ctx.axis, "autotune_verify"
        )
        base_name = ctx.baseline_results["steps"][0]["name"]
        final_name = results["steps"][0]["name"]
        base_ferr, _base_overshoot = cal._step_headline(
            ctx.baseline_results, base_name
        )
        final_ferr, _final_overshoot = cal._step_headline(results, final_name)
        if base_ferr > 0.0:
            pct = 100.0 * (final_ferr - base_ferr) / base_ferr
            if pct > 20.0:
                raise ctx.gcmd.error(
                    "SERVO_AUTOTUNE: aborting at stage 'verify' (run %s): "
                    "ferr peak regressed %.0f%% vs baseline (run %s): "
                    "%.0f -> %.0f counts"
                    % (
                        run.run_dir,
                        pct,
                        ctx.baseline_run.run_dir,
                        base_ferr,
                        final_ferr,
                    )
                )
        return {
            "outcome": "ran",
            "run_dir": run.run_dir,
            "baseline_ferr_peak": base_ferr,
            "final_ferr_peak": final_ferr,
        }


AUTOTUNE_STAGES: tuple[AutotuneStage, ...] = (
    BaselineTrackingStage(),
    InertiaRatioIdentifyStage(),
    ApplyInertiaRatioStage(),
    CoarseGainsStage(),
    GainSweepStage(),
    FitDynamicsStage(),
    VerifyStage(),
)
