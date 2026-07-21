"""Servo calibration toolkit (A6-EC over EtherCAT).

Loaded only when a printer.cfg contains a [servo_calibration] section
(typically on the EtherCAT bench, so no config in this repo references it);
run-invariant values (motor datasheet, stroke window, drive names,
excitation grid) live in the config section and every command reads them as
overridable defaults. Command and option reference:
docs/rewrite/servo-calibration.md.
"""

from __future__ import annotations

try:
    import tomllib
except ImportError:
    tomllib = None

from typing import Any

from ... import structured_log
from .autotune import (
    AUTOTUNE_STAGES,
    COARSE_GAINS,
    ApplyInertiaRatioStage,
    AutotuneContext,
    AutotuneStage,
    BaselineTrackingStage,
    CoarseGainsStage,
    FitDynamicsStage,
    GainSweepStage,
    InertiaRatioIdentifyStage,
    VerifyStage,
)
from .common import (
    _C0006_RE,
    DEFAULT_CAPTURES_ROOT,
    DEFAULT_DYNAMICS_DIR,
    REPO_ROOT,
    VERDICT_ABORT_FLAGS,
    ApplyResult,
    _applied,
    _git_rev,
    _git_rev_cache,
    _parse_c0006_recommendation,
    _utc_now,
)
from .dynamics import (
    DYNAMICS_TERM_KEYS,
    TUNE_MASS_FLOOR_FRACTION,
    TUNE_RELATIVE_CLAMP,
    TUNE_ZERO_FLOOR_STEPS,
    _copy_dynamics,
    _equal_or_opposite_columns,
    _frame_column_lambda,
    _parse_dynamics_pairs,
    add_dynamics_direction_split,
    discover_dynamics_pairs,
    dynamics_torque_changes,
    parse_dynamics_profile,
    render_fit_dynamics_toml,
    send_dynamics_model,
    send_ff_lead,
)
from .fit import DynamicsFitCommands
from .gains import GainCommands
from .host import CalibrationHost
from .measure import MeasureCommands
from .params import (
    C00_06_INERTIA_RATIO_MAX,
    GAIN_LIST_PARAMS,
    GAIN_PARAMS,
    INERTIA_RATIO_ADDR,
    NOTCH_MODE_ADDR,
    NOTCH_READBACK,
    SYNC_LOSS_COUNT_ADDR,
    SYNC_LOSS_THRESHOLD_ADDR,
    validate_gain_values,
)
from .search import RmsLineSearch
from .sweep import (
    ExperimentRun,
    InertiaRatioAdapter,
    MotionAccelAdapter,
    SingleGainAdapter,
    SweepEngine,
    SweepStep,
    _OverrideGcmd,
)
from .sweeps import SweepCommands


class ServoCalibration(SweepCommands):
    pass


def load_config(config: Any) -> ServoCalibration:
    config.get_printer().load_object(config, "servo_tuning")
    return ServoCalibration(config)


__all__ = [
    "AUTOTUNE_STAGES",
    "ApplyInertiaRatioStage",
    "ApplyResult",
    "AutotuneContext",
    "AutotuneStage",
    "BaselineTrackingStage",
    "C00_06_INERTIA_RATIO_MAX",
    "COARSE_GAINS",
    "CalibrationHost",
    "CoarseGainsStage",
    "DEFAULT_CAPTURES_ROOT",
    "DEFAULT_DYNAMICS_DIR",
    "DYNAMICS_TERM_KEYS",
    "DynamicsFitCommands",
    "ExperimentRun",
    "FitDynamicsStage",
    "GAIN_LIST_PARAMS",
    "GAIN_PARAMS",
    "GainCommands",
    "GainSweepStage",
    "INERTIA_RATIO_ADDR",
    "InertiaRatioAdapter",
    "InertiaRatioIdentifyStage",
    "MeasureCommands",
    "MotionAccelAdapter",
    "NOTCH_MODE_ADDR",
    "NOTCH_READBACK",
    "REPO_ROOT",
    "RmsLineSearch",
    "SYNC_LOSS_COUNT_ADDR",
    "SYNC_LOSS_THRESHOLD_ADDR",
    "ServoCalibration",
    "SingleGainAdapter",
    "SweepCommands",
    "SweepEngine",
    "SweepStep",
    "TUNE_MASS_FLOOR_FRACTION",
    "TUNE_RELATIVE_CLAMP",
    "TUNE_ZERO_FLOOR_STEPS",
    "VERDICT_ABORT_FLAGS",
    "VerifyStage",
    "_C0006_RE",
    "_OverrideGcmd",
    "_applied",
    "_copy_dynamics",
    "_equal_or_opposite_columns",
    "_frame_column_lambda",
    "_git_rev",
    "_git_rev_cache",
    "_parse_c0006_recommendation",
    "_parse_dynamics_pairs",
    "_utc_now",
    "add_dynamics_direction_split",
    "discover_dynamics_pairs",
    "dynamics_torque_changes",
    "load_config",
    "parse_dynamics_profile",
    "render_fit_dynamics_toml",
    "send_dynamics_model",
    "send_ff_lead",
    "structured_log",
    "tomllib",
    "validate_gain_values",
]
