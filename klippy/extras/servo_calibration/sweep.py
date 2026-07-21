from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gains import GainCommands
    from .host import CalibrationHost

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from .. import servo_param
from .common import ApplyResult, _applied
from .params import GAIN_PARAMS, INERTIA_RATIO_ADDR


class _OverrideGcmd:
    """Wraps a gcmd, forcing specific parameter values so a stage can drive
    another SERVO_* command's implementation directly - SERVO_AUTOTUNE's
    stages are the real command bodies, not a reimplementation of them."""

    def __init__(self, base: Any, overrides: dict[str, Any]):
        self._base = base
        self._overrides = overrides
        self.error = base.error
        self.respond_info = base.respond_info
        self.get_commandline = base.get_commandline

    def get(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get(name, default, **kw)

    def get_int(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get_int(name, default, **kw)

    def get_float(self, name: str, default: Any = None, **kw: Any) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        return self._base.get_float(name, default, **kw)


@dataclass
class SweepStep:
    name: str
    swept: dict[str, float]
    applied: list[dict[str, Any]]
    accel: str | None = None
    stops: list[float] | None = None


@dataclass
class ExperimentRun:
    """One experiment's run directory and its manifest, rewritten as steps
    complete so a crashed run keeps partial truth on disk."""

    run_dir: str
    stamp: str
    manifest: dict[str, Any]
    started_s: float = field(default_factory=time.time)

    @property
    def manifest_path(self) -> str:
        return os.path.join(self.run_dir, "manifest.json")

    def step_scap(self, name: str) -> str:
        return os.path.join(self.run_dir, "step_%s.scap" % (name,))

    def step_accel_csv(self, name: str) -> str:
        return os.path.join(self.run_dir, "step_%s_accel.csv" % (name,))

    def write(self) -> None:
        tmp = self.manifest_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.manifest, f, indent=2)
        os.replace(tmp, self.manifest_path)

    def record_step(self, step: SweepStep) -> None:
        entry = {
            "name": step.name,
            "swept": step.swept,
            "applied": step.applied,
            "capture": os.path.basename(self.step_scap(step.name)),
            "accel": step.accel,
        }
        if step.stops is not None:
            entry["stops"] = step.stops
        self.manifest["steps"].append(entry)
        self.write()


class SingleGainAdapter:
    """SERVO_CALIBRATE_GAINS: sweeps one gain, holding the others fixed."""

    def __init__(
        self,
        calibration: "GainCommands",
        servos: list[str],
        param: str,
        tag: str,
        original: dict[str, int],
        current: int,
    ):
        self._cal = calibration
        self.servos = servos
        self.param = param
        self.tag = tag
        self._original = original
        self.current = current

    def step_name(self, value: int) -> str:
        return "%s_%s_v%d" % (self.tag, self.param, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        _addr, _lo, _hi, desc, unit, scale = GAIN_PARAMS[self.param]
        marker = "  <- current" if value == self.current else ""
        return "sweep %s step %d/%d: %s = %d (%.4g %s)%s on %s" % (
            self.param,
            i + 1,
            total,
            desc,
            value,
            value / scale,
            unit,
            marker,
            ", ".join(servos),
        )

    def apply(self, value: int) -> ApplyResult:
        values = dict(self._original)
        values[self.param] = value
        self._cal._write_gains(self.servos, values)
        swept = {self.param: value}
        applied = self._cal._gain_write_records(self.servos, values)
        return swept, applied

    def revert(self) -> None:
        self._cal._write_gains(self.servos, self._original)


class InertiaRatioAdapter:
    """SERVO_SWEEP_INERTIA: sweeps C00.06 load inertia ratio."""

    ADDR = INERTIA_RATIO_ADDR

    def __init__(
        self,
        calibration: "CalibrationHost",
        servos: list[str],
        tag: str,
        original: int,
    ):
        self._cal = calibration
        self.servos = servos
        self.tag = tag
        self.original = original

    def step_name(self, value: int) -> str:
        return "%s_r%d" % (self.tag, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        return "inertia step %d/%d: C00.06 ratio %d%% on %s" % (
            i + 1,
            total,
            value,
            ", ".join(servos),
        )

    def _write(self, value: int) -> None:
        with servo_param.suppress_write_log():
            self._cal.gcode.run_script_from_command(
                "\n".join(
                    "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
                    % (servo, self.ADDR, value)
                    for servo in self.servos
                )
            )

    def apply(self, value: int) -> ApplyResult:
        self._write(value)
        swept = {"inertia_ratio": value}
        applied = [_applied(servo, self.ADDR, value) for servo in self.servos]
        return swept, applied

    def revert(self) -> None:
        self._write(self.original)


class MotionAccelAdapter:
    """SERVO_SWEEP_ACCEL: no SDO write, varies the stroke plan's accel."""

    def __init__(self, tag: str):
        self.tag = tag

    def step_name(self, value: int) -> str:
        return "%s_a%d" % (self.tag, value)

    def describe(
        self, i: int, value: int, total: int, servos: list[str]
    ) -> str:
        return "accel step %d/%d: %d mm/s^2 on %s" % (
            i + 1,
            total,
            value,
            ", ".join(servos),
        )

    def apply(self, value: int) -> ApplyResult:
        return {"accel": value}, []

    def revert(self) -> None:
        pass


class SweepEngine:
    """for each value: adapter.apply -> capture -> run strokes -> capture."""

    def __init__(self, calibration: "CalibrationHost"):
        self._cal = calibration

    def run_one(
        self,
        adapter: Any,
        i: int,
        value: Any,
        total: int,
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        accel_chip: Any = None,
        accel_chip_name: str | None = None,
    ) -> SweepStep:
        name = adapter.step_name(value)
        swept, applied = adapter.apply(value)
        gcmd.respond_info(adapter.describe(i, value, total, servos))
        self._cal._start_capture(name, servos)
        aclient = (
            None if accel_chip is None else accel_chip.start_internal_client()
        )
        try:
            run_step(value)
            self._cal._stop_capture()
        finally:
            if aclient is not None:
                aclient.finish_measurements()
        step = SweepStep(name, swept, applied)
        if aclient is not None:
            assert accel_chip_name is not None, (
                "accel client exists without a chip name"
            )
            accel_path = self._cal._write_accel_csv(
                gcmd, aclient, accel_chip_name, name
            )
            step.accel = os.path.basename(accel_path)
        self._cal._on_step_complete(step)
        return step

    def run(
        self,
        adapter: Any,
        values: list[Any],
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        accel_chip: Any = None,
        accel_chip_name: str | None = None,
    ) -> list[SweepStep]:
        return [
            self.run_one(
                adapter,
                i,
                value,
                len(values),
                servos,
                run_step,
                gcmd,
                accel_chip,
                accel_chip_name,
            )
            for i, value in enumerate(values)
        ]
