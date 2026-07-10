"""Servo calibration toolkit (A6-EC over EtherCAT).

Loaded only when a printer.cfg contains a [servo_calibration] section
(typically on the EtherCAT bench, so no config in this repo references it);
run-invariant values (motor datasheet, stroke window, drive names,
excitation grid) live in the config section and every command reads them as
overridable defaults. Command and option reference:
docs/rewrite/servo-calibration.md.
"""

from __future__ import annotations

import logging
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable

from . import servo_strokes

ApplyResult = tuple[dict[str, float], list[dict[str, Any]]]

SCRIPTS_DIR = os.path.join(
    os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ),
    "scripts",
)

GAIN_PARAMS = {
    "position": (
        "0x2001.0x01",
        1,
        30000,
        "C01.00 position loop gain",
        "0.1 rad/s",
        10.0,
    ),
    "speed": (
        "0x2001.0x02",
        1,
        20000,
        "C01.01 speed loop gain",
        "0.1 Hz",
        10.0,
    ),
    "integral": (
        "0x2001.0x03",
        15,
        51200,
        "C01.02 speed integral time",
        "0.01 ms",
        100.0,
    ),
}

INERTIA_RATIO_ADDR = "0x2000.0x07"


def refine_values(
    current: float,
    values_text: str | None,
    span: float | None,
    steps: int,
) -> list[int]:
    if values_text is not None:
        vals = [
            int(round(float(v))) for v in values_text.split(",") if v.strip()
        ]
        if not vals:
            raise ValueError("VALUES lists no usable numbers")
    else:
        if steps < 2:
            raise ValueError("STEPS must be at least 2")
        if span is None or not 0.0 < span < 1.0:
            raise ValueError(
                "SPAN must be between 0 and 1 (fraction of current)"
            )
        lo, hi = 1.0 - span, 1.0 + span
        vals = [
            int(round(current * (lo + (hi - lo) * i / (steps - 1))))
            for i in range(steps)
        ]
        vals.append(int(round(current)))
    return sorted(set(vals))


def validate_gain_values(values: list[int], param: str) -> list[int]:
    if param not in GAIN_PARAMS:
        raise ValueError(
            "PARAM must be position, speed or integral (got %r)" % (param,)
        )
    _addr, lo, hi, _desc, _unit, _scale = GAIN_PARAMS[param]
    for v in values:
        if v <= 0:
            raise ValueError(
                "%s value %d is not a positive integer" % (param, v)
            )
        if not lo <= v <= hi:
            raise ValueError(
                "%s value %d outside drive range %d..%d" % (param, v, lo, hi)
            )
    return values


def _applied(servo: str, addr: str, value: int) -> dict[str, Any]:
    return {"servo": servo, "addr": addr, "type": "u16", "value": value}


@dataclass
class SweepStep:
    name: str
    swept: dict[str, float]
    applied: list[dict[str, Any]]


class GainSetAdapter:
    """Speed-gain sweep: derives position/integral, writes gain set 1."""

    def __init__(
        self, calibration: "ServoCalibration", servos: list[str], tag: str
    ):
        self._cal = calibration
        self.servos = servos
        self.tag = tag

    @staticmethod
    def derive(speed_gain: int) -> tuple[int, int]:
        return round(speed_gain * 1.6), round(1250000 / speed_gain)

    def step_name(self, speed_gain: int) -> str:
        pos_gain, integral = self.derive(speed_gain)
        return "%s_p%d_s%d_i%d" % (self.tag, pos_gain, speed_gain, integral)

    def describe(
        self, i: int, speed_gain: int, total: int, servos: list[str]
    ) -> str:
        pos_gain, integral = self.derive(speed_gain)
        return (
            "gain step %d/%d: pos %.1f rad/s, speed %.1f Hz, Ti %.2f ms on %s"
            % (
                i + 1,
                total,
                pos_gain / 10.0,
                speed_gain / 10.0,
                integral / 100.0,
                ", ".join(servos),
            )
        )

    def apply(self, speed_gain: int) -> ApplyResult:
        pos_gain, integral = self.derive(speed_gain)
        self._cal._write_gains(self.servos, pos_gain, speed_gain, integral)
        swept = {
            "position": pos_gain,
            "speed": speed_gain,
            "integral": integral,
        }
        applied = self._cal._gain_write_records(
            self.servos, pos_gain, speed_gain, integral
        )
        return swept, applied

    def revert(self, values: list[int]) -> None:
        sg0 = values[0]
        pg0, ig0 = self.derive(sg0)
        self._cal._write_gains(self.servos, pg0, sg0, ig0)


class SingleGainAdapter:
    """SERVO_REFINE_GAIN: sweeps one gain, holding the other two fixed."""

    def __init__(
        self,
        calibration: "ServoCalibration",
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
        return "refine %s step %d/%d: %s = %d (%.4g %s)%s on %s" % (
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
        triple = dict(self._original)
        triple[self.param] = value
        self._cal._write_gains(
            self.servos, triple["position"], triple["speed"], triple["integral"]
        )
        swept = {self.param: value}
        applied = self._cal._gain_write_records(
            self.servos, triple["position"], triple["speed"], triple["integral"]
        )
        return swept, applied

    def revert(self) -> None:
        self._cal._write_gains(
            self.servos,
            self._original["position"],
            self._original["speed"],
            self._original["integral"],
        )


class InertiaRatioAdapter:
    """SERVO_SWEEP_INERTIA: sweeps C00.06 load inertia ratio."""

    ADDR = INERTIA_RATIO_ADDR

    def __init__(
        self,
        calibration: "ServoCalibration",
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

    def __init__(self, calibration: "ServoCalibration"):
        self._cal = calibration

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
        steps = []
        for i, value in enumerate(values):
            name = adapter.step_name(value)
            swept, applied = adapter.apply(value)
            gcmd.respond_info(adapter.describe(i, value, len(values), servos))
            self._cal._start_capture(name, servos)
            aclient = (
                None
                if accel_chip is None
                else accel_chip.start_internal_client()
            )
            try:
                run_step(value)
                self._cal._stop_capture()
            finally:
                if aclient is not None:
                    aclient.finish_measurements()
            if aclient is not None:
                self._cal._write_accel_csv(gcmd, aclient, accel_chip_name, name)
            steps.append(SweepStep(name, swept, applied))
        return steps


class ServoCalibration:
    def __init__(self, config: Any):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.servos = config.getlist("servos", ["stepper_x", "stepper_y"])
        self.rated_torque_nm = config.getfloat(
            "rated_torque_nm", None, above=0.0
        )
        self.rotor_inertia_kgm2 = config.getfloat(
            "rotor_inertia_kgm2", None, above=0.0
        )
        self.bounds: servo_strokes.Bounds = {
            "X": (
                config.getfloat("x_start", 20.0),
                config.getfloat("x_end", 200.0),
            ),
            "Y": (
                config.getfloat("y_start", 20.0),
                config.getfloat("y_end", 200.0),
            ),
        }
        self.accels = config.getfloatlist("accels", [5000.0, 10000.0, 20000.0])
        self.speeds = config.getfloatlist("speeds", [100.0, 400.0])
        self.iterations = config.getint("iterations", 3, minval=1)
        self.accel_chip_name = config.get("accel_chip", None)
        self.dwell_ms = config.getint("dwell_ms", 700, minval=0)
        self.travel_speed = config.getfloat("travel_speed", 100.0, above=0.0)
        self._engine = SweepEngine(self)
        for name in (
            "SERVO_MEASURE_TRACKING",
            "SERVO_MEASURE_INERTIA",
            "SERVO_FIT_DYNAMICS",
            "SERVO_CALIBRATE_INERTIA_RATIO",
            "SERVO_SHOW_TUNING",
            "SERVO_SET_INERTIA_RATIO",
            "SERVO_APPLY_GAINS",
            "SERVO_CALIBRATE_GAINS",
            "SERVO_REFINE_GAIN",
            "SERVO_SWEEP_INERTIA",
            "SERVO_SWEEP_ACCEL",
            "SERVO_SET_STIFFNESS",
        ):
            self.gcode.register_command(
                name,
                getattr(self, "cmd_" + name),
                desc=getattr(self, "cmd_" + name + "_help"),
            )

    def _kin(self) -> Any:
        return self.printer.lookup_object("toolhead").get_kinematics()

    def _floats(self, text: str | None) -> list[float] | None:
        return servo_strokes.parse_floats(text)

    def _motor(
        self, gcmd: Any, required: bool
    ) -> tuple[float | None, float | None]:
        torque = gcmd.get_float("TORQUE_NM", self.rated_torque_nm)
        inertia = gcmd.get_float("INERTIA_KGM2", self.rotor_inertia_kgm2)
        if required:
            if torque is None:
                raise gcmd.error(
                    "TORQUE_NM required - set rated_torque_nm in "
                    "[servo_calibration] or pass TORQUE_NM= (motor rated torque, N*m)"
                )
            if inertia is None:
                raise gcmd.error(
                    "INERTIA_KGM2 required - set rotor_inertia_kgm2 in "
                    "[servo_calibration] or pass INERTIA_KGM2= (rotor inertia, kg*m^2)"
                )
        elif (torque is None) != (inertia is None):
            raise gcmd.error(
                "TORQUE_NM and INERTIA_KGM2 must be given together"
            )
        return torque, inertia

    def _servo(self, gcmd: Any) -> str:
        default = self.servos[0] if len(self.servos) == 1 else None
        servo = gcmd.get("SERVO", default)
        if servo is None:
            raise gcmd.error(
                "SERVO= is required - name the drive explicitly (e.g. SERVO=motor_a)"
            )
        return servo

    def _servos(self, gcmd: Any, axis: str | None = None) -> list[str]:
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            return [s.strip() for s in servo.split(",") if s.strip()]
        if axis is None:
            axis = gcmd.get("AXIS", None)
        if axis is not None:
            return servo_strokes.axis_servos(gcmd, self._kin(), axis.upper())
        if len(self.servos) == 1:
            return [self.servos[0]]
        raise gcmd.error(
            "AXIS= or SERVO= is required (SERVO= accepts a comma list)"
        )

    def _reject_corexy_only_params(self, gcmd: Any) -> None:
        bad = [
            p
            for p in ("SERVOS", "X_START", "X_END", "Y_START", "Y_END")
            if gcmd.get(p, None) is not None
        ]
        if bad:
            raise gcmd.error(
                "%s require coupled_xy kinematics - the active kinematics "
                "is not CoreXY" % (", ".join(bad),)
            )

    def _strokes(
        self,
        axis: str,
        start: float,
        end: float,
        speed: float,
        accel: float,
        iterations: int,
        dwell: int,
    ) -> None:
        servo_strokes.emit_strokes(
            self.gcode,
            lambda u: "%s%.3f" % (axis, u),
            start,
            end,
            1.0,
            speed,
            accel,
            iterations,
            dwell,
        )

    def _goto_xy(self, x: float, y: float, dwell: int) -> None:
        servo_strokes.goto_xy(self.gcode, self.travel_speed, x, y, dwell)

    def _prep(self, axis: str, dwell: int) -> None:
        servo_strokes.prep(self.printer, self.gcode, axis, dwell)

    def _restore(self) -> None:
        self.gcode.run_script_from_command("RESET_VELOCITY_LIMIT")

    def _start_capture(self, name: str, servos: list[str]) -> None:
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s" % (",".join(servos), name)
        )

    def _stop_capture(self) -> None:
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")

    def _accel_chip(self, gcmd: Any) -> tuple[Any, str | None]:
        chip_name = gcmd.get("ACCEL_CHIP", self.accel_chip_name)
        if chip_name is None:
            return None, None
        return self.printer.lookup_object(chip_name.strip()), chip_name

    def _write_accel_csv(
        self, gcmd: Any, aclient: Any, chip_name: str, step_name: str
    ) -> None:
        if not aclient.has_valid_samples():
            raise gcmd.error(
                "accelerometer %r measured no data for step %s"
                % (chip_name, step_name)
            )
        from . import servo_capture

        capture_dir = os.path.expanduser(servo_capture.CAPTURE_DIR)
        os.makedirs(capture_dir, exist_ok=True)
        path = os.path.join(
            capture_dir,
            "%s_accel_%s.csv" % (step_name, time.strftime("%Y%m%d_%H%M%S")),
        )
        with open(path, "w") as f:
            f.write("#time,accel_x,accel_y,accel_z\n")
            for t, accel_x, accel_y, accel_z in aclient.get_samples():
                f.write(
                    "%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z)
                )
        gcmd.respond_info("Accelerometer data written to %s" % (path,))

    def _run(
        self, gcmd: Any, script: str, args: list[str], timeout: float
    ) -> None:
        reactor = self.printer.get_reactor()
        argv = [sys.executable, os.path.join(SCRIPTS_DIR, script)] + args
        try:
            proc = subprocess.Popen(
                argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
            )
        except Exception:
            logging.exception("servo_calibration: failed to launch %s", script)
            raise gcmd.error("Error launching %s" % (script,))
        fd = proc.stdout.fileno()
        buf = [""]

        def emit(data: str) -> None:
            buf[0] += data
            if "\n" in buf[0]:
                head, _, buf[0] = buf[0].rpartition("\n")
                gcmd.respond_info(head)

        def on_readable(eventtime: float) -> None:
            try:
                emit(os.read(fd, 4096).decode())
            except Exception:
                pass

        hdl = reactor.register_fd(fd, on_readable)
        gcmd.respond_info("Running %s ..." % (script,))
        eventtime = reactor.monotonic()
        endtime = eventtime + timeout
        complete = False
        while eventtime < endtime:
            eventtime = reactor.pause(eventtime + 0.05)
            if proc.poll() is not None:
                complete = True
                break
        reactor.unregister_fd(hdl)
        if not complete:
            proc.terminate()
            raise gcmd.error("%s timed out after %.0fs" % (script, timeout))
        while True:
            data = os.read(fd, 4096).decode()
            if not data:
                break
            emit(data)
        if buf[0]:
            gcmd.respond_info(buf[0])
        if proc.returncode:
            raise gcmd.error(
                "%s exited with code %d" % (script, proc.returncode)
            )

    cmd_SERVO_MEASURE_TRACKING_help = (
        "Single accel/speed stroke run with capture - the before/after check "
        "for any tuning change. AXIS=X/Y records every motor driving the axis "
        "(both lanes on CoreXY) and saves a per-motor + combined tracking PNG. "
        "AXIS=A/B run a CoreXY 45-degree diagonal that exercises one motor "
        "alone (A=+45 x&y up, motor A; B=-45 x up y down, motor B); SPEED is "
        "the toolhead feedrate, so belt speed is sqrt(2)x SPEED on a diagonal. "
        "Params AXIS START END SPEED ACCEL ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_TRACKING(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "track")
        servos = plan.servos
        for prep_axis in plan.prep:
            self._prep(prep_axis, dwell)
        self._start_capture(name, servos)
        servo_strokes.emit_strokes(
            self.gcode,
            plan.coord,
            plan.start,
            plan.end,
            plan.th_per_unit,
            speed,
            accel,
            iterations,
            dwell,
        )
        self._stop_capture()
        self._restore()
        report_args = ["--name", name, "--png"]
        rails = plan.rails
        if not plan.diagonal and len(rails) == 2 and axis in ("X", "Y"):
            belts = ",".join(
                "+".join(
                    "%s:%d"
                    % (
                        m.get_motor_name(),
                        -1 if m.get_invert_direction() else 1,
                    )
                    for m in servo_strokes.rail_motors_in_slot_order(r)
                )
                for r in rails
            )
            report_args += ["--axis", axis, "--combine-corexy", belts]
        self._run(gcmd, "servo_capture.py", report_args, 120.0)

    cmd_SERVO_MEASURE_INERTIA_help = (
        "Excitation grid for the inertia/friction fit (servo-ident). "
        "coupled_xy kinematics run the X+Y belt grid (SERVOS=/X_START etc "
        "override; travel_speed centers the idle axis between strokes); "
        "cartesian kinematics run a single AXIS grid and reject SERVOS/"
        "X_START/X_END/Y_START/Y_END. Params AXIS START END X_START X_END "
        "Y_START Y_END ACCELS SPEEDS ITERATIONS DWELL_MS NAME SERVOS"
    )

    def cmd_SERVO_MEASURE_INERTIA(self, gcmd: Any) -> None:
        self._measure_inertia(gcmd, gcmd.get("NAME", "ident"))

    def _measure_inertia(self, gcmd: Any, name: str) -> None:
        kin = self._kin()
        if kin.coupled_xy():
            self._measure_inertia_corexy(gcmd, name)
            return
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        self._prep(axis, dwell)
        self._start_capture(name, servos)
        for accel in accels:
            for speed in speeds:
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self._stop_capture()
        self._restore()

    def _measure_inertia_corexy(
        self, gcmd: Any, name: str, servos: str | list[str] | None = None
    ) -> None:
        kin = self._kin()
        if servos is None:
            servos = gcmd.get("SERVOS", None)
        if servos is None:
            servo_list = servo_strokes.axis_servos(gcmd, kin, "X")
        elif isinstance(servos, str):
            servo_list = [s.strip() for s in servos.split(",") if s.strip()]
        else:
            servo_list = list(servos)
        x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        x_center = (x_start + x_end) / 2.0
        y_center = (y_start + y_end) / 2.0
        self._prep("X", dwell)
        self._prep("Y", dwell)
        self._start_capture(name, servo_list)
        for accel in accels:
            for speed in speeds:
                self._goto_xy(x_start, y_center, dwell)
                self._strokes(
                    "X", x_start, x_end, speed, accel, iterations, dwell
                )
                self._goto_xy(x_center, y_start, dwell)
                self._strokes(
                    "Y", y_start, y_end, speed, accel, iterations, dwell
                )
        self._stop_capture()
        self._restore()

    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward - runs the "
        "SERVO_MEASURE_INERTIA grid, fits mass/viscous/coulomb, and writes "
        "a timestamped profile (node-level on coupled_xy, the coupled mass "
        "matrix with AWD drives paired from the kinematics; per-axis "
        "otherwise, DRIVE= picking the scalar fit on a multi-drive axis). "
        "Optional TORQUE_NM + INERTIA_KGM2 add the C00.06 recommendation. "
        "Params as SERVO_MEASURE_INERTIA plus TORQUE_NM INERTIA_KGM2 DRIVE"
    )

    def _fit_dynamics_args(self, gcmd: Any, name: str) -> list[str]:
        kin = self._kin()
        if kin.coupled_xy():
            layout = servo_strokes.corexy_fit_layout(gcmd, kin)
            servo_strokes.check_servos_override(gcmd, layout)
            self._measure_inertia_corexy(gcmd, name, servos=layout["servos"])
            args = ["--name", name, "--structure", "corexy"]
            if layout["pairs"] is not None:
                args += ["--pairs", layout["pairs"]]
            return args
        self._reject_corexy_only_params(gcmd)
        drive = servo_strokes.scalar_fit_drive(gcmd, kin)
        self._measure_inertia(gcmd, name)
        args = ["--name", name]
        if drive is not None:
            args += ["--drive", drive]
        return args

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd: Any) -> None:
        name = gcmd.get("NAME", "ident")
        torque, inertia = self._motor(gcmd, required=False)
        args = self._fit_dynamics_args(gcmd, name)
        if torque is not None:
            args += [
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ]
        self._run(gcmd, "servo_fit_dynamics.py", args, 120.0)

    cmd_SERVO_CALIBRATE_INERTIA_RATIO_help = (
        "Step 2 of servo tuning - identify the load inertia and print the "
        "recommended C00.06 (on coupled_xy kinematics: for both belt "
        "directions, via the coupled X+Y grid and mass-matrix fit; the "
        "drive takes one scalar, so start from the light-direction number "
        "and confirm with SERVO_SWEEP_INERTIA). TORQUE_NM and INERTIA_KGM2 "
        "required (config or param). Params as SERVO_MEASURE_INERTIA plus "
        "TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO(self, gcmd: Any) -> None:
        name = gcmd.get("NAME", "inertia")
        torque, inertia = self._motor(gcmd, required=True)
        args = self._fit_dynamics_args(gcmd, name)
        args += [
            "--rated-torque-nm",
            "%g" % (torque,),
            "--rotor-inertia-kgm2",
            "%g" % (inertia,),
        ]
        self._run(gcmd, "servo_fit_dynamics.py", args, 120.0)

    def _write_gains(
        self, servos: list[str], pos_gain: int, speed_gain: int, integral: int
    ) -> None:
        lines = []
        for servo in servos:
            lines += [
                "SERVO_PARAM SERVO=%s SET=0x2001.0x01 VALUE=%d TYPE=u16"
                % (servo, pos_gain),
                "SERVO_PARAM SERVO=%s SET=0x2001.0x02 VALUE=%d TYPE=u16"
                % (servo, speed_gain),
                "SERVO_PARAM SERVO=%s SET=0x2001.0x03 VALUE=%d TYPE=u16"
                % (servo, integral),
            ]
        self.gcode.run_script_from_command("\n".join(lines))

    def _gain_write_records(
        self, servos: list[str], pos_gain: int, speed_gain: int, integral: int
    ) -> list[dict[str, Any]]:
        values = {
            "position": pos_gain,
            "speed": speed_gain,
            "integral": integral,
        }
        return [
            _applied(servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in ("position", "speed", "integral")
        ]

    def _resolve_node_slot(self, servo: str) -> tuple[Any, int]:
        from . import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _read_param(self, servo: str, addr: str) -> int:
        from . import servo_param

        node, slot = self._resolve_node_slot(servo)
        handle = node.get_engine_handle()
        if handle is None:
            raise self.printer.command_error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        index, subindex = servo_param.parse_address(addr)
        _size, raw = engine.sdo_read(handle, slot, index, subindex)
        return raw

    def _read_gains(self, servo: str) -> dict[str, int]:
        return {
            name: self._read_param(servo, GAIN_PARAMS[name][0])
            for name in ("position", "speed", "integral")
        }

    def _set_manual_tuning(self, servos: list[str]) -> None:
        self.gcode.run_script_from_command(
            "\n".join(
                "SERVO_PARAM SERVO=%s SET=0x2000.0x05 VALUE=0 TYPE=u16"
                % (servo,)
                for servo in servos
            )
        )

    cmd_SERVO_SHOW_TUNING_help = (
        "Read back tuning mode, inertia ratio, gain set 1 and feedforward "
        "params from the drive(s). Params SERVO (comma list) or AXIS"
    )

    def cmd_SERVO_SHOW_TUNING(self, gcmd: Any) -> None:
        for servo in self._servos(gcmd):
            self._show_tuning(servo)

    def _show_tuning(self, servo: str) -> None:
        reads = [
            (
                "C00.04 auto-tuning mode (0=manual 1=stiffness 2=positioning):",
                ["0x2000.0x05"],
            ),
            (
                "C00.05 stiffness level (1..31, used in mode 1):",
                ["0x2000.0x06"],
            ),
            ("C00.06 load inertia ratio (%):", ["0x2000.0x07"]),
            ("C01.00 position loop gain (0.1 rad/s):", ["0x2001.0x01"]),
            ("C01.01 speed loop gain (0.1 Hz):", ["0x2001.0x02"]),
            ("C01.02 speed integral time (0.01 ms):", ["0x2001.0x03"]),
            (
                "C01.13 velocity FF source / C01.14 pct / C01.15 filter:",
                ["0x2001.0x14", "0x2001.0x15", "0x2001.0x16"],
            ),
            (
                "C01.16 torque FF source / C01.17 pct / C01.18 filter:",
                ["0x2001.0x17", "0x2001.0x18", "0x2001.0x19"],
            ),
        ]
        script = ['RESPOND MSG="=== %s ==="' % (servo,)]
        for msg, addrs in reads:
            script.append('RESPOND MSG="%s"' % (msg,))
            for addr in addrs:
                script.append("SERVO_PARAM SERVO=%s GET=%s" % (servo, addr))
        self.gcode.run_script_from_command("\n".join(script))

    cmd_SERVO_SET_INERTIA_RATIO_help = (
        "Write C00.06 load inertia ratio in percent. Params RATIO SERVO"
    )

    def cmd_SERVO_SET_INERTIA_RATIO(self, gcmd: Any) -> None:
        servo = self._servo(gcmd)
        ratio = gcmd.get_int("RATIO", minval=0, maxval=12000)
        self.gcode.run_script_from_command(
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
            % (servo, INERTIA_RATIO_ADDR, ratio)
        )

    cmd_SERVO_APPLY_GAINS_help = (
        "Switch the drive(s) to manual tuning (C00.04=0) and write gain set "
        "1 to every servo driving the axis. POS_GAIN 0.1 rad/s, SPEED_GAIN "
        "0.1 Hz, INTEGRAL 0.01 ms. Params AXIS or SERVO (comma list)"
    )

    def cmd_SERVO_APPLY_GAINS(self, gcmd: Any) -> None:
        servos = self._servos(gcmd)
        pos_gain = gcmd.get_int("POS_GAIN", 400)
        speed_gain = gcmd.get_int("SPEED_GAIN", 250)
        integral = gcmd.get_int("INTEGRAL", 3184)
        self._set_manual_tuning(servos)
        self._write_gains(servos, pos_gain, speed_gain, integral)
        for servo in servos:
            self._show_tuning(servo)

    def _run_sweep_with_revert(
        self,
        adapter: Any,
        values: list[Any],
        servos: list[str],
        run_step: Callable[[Any], None],
        gcmd: Any,
        on_revert: Callable[[], None],
    ) -> list[SweepStep]:
        try:
            steps = self._engine.run(adapter, values, servos, run_step, gcmd)
        finally:
            on_revert()
            adapter.revert()
            self._restore()
        return steps

    cmd_SERVO_CALIBRATE_GAINS_help = (
        "Gain sweep, shaper-calibrate style. Resolves every servo driving "
        "AXIS (both drives on CoreXY), writes each SPEED_GAINS entry (0.1 Hz "
        "units, comma list) to all of them, one capture per step of all "
        "drives, renders a comparison PNG and prints a recommendation. "
        "With an accelerometer (accel_chip config option or ACCEL_CHIP=) "
        "each step also records vibration data and the report gains a "
        "frequency-response + spectrogram row. Reverts to the lowest gains "
        "afterwards. Params SPEED_GAINS AXIS START END SPEED ACCEL "
        "ITERATIONS DWELL_MS TAG ACCEL_CHIP SERVO (comma list override)"
    )

    def cmd_SERVO_CALIBRATE_GAINS(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "cal")
        sgains = self._floats(gcmd.get("SPEED_GAINS", "500,650,800,1000"))
        for sg in sgains:
            if not 100 <= sg <= 3000:
                raise gcmd.error(
                    "SPEED_GAIN %d outside 100..3000 (0.1 Hz units)" % (sg,)
                )
        sgains = [int(sg) for sg in sgains]
        chip, chip_name = self._accel_chip(gcmd)
        self._prep(axis, dwell)
        self._set_manual_tuning(servos)
        adapter = GainSetAdapter(self, servos, tag)
        steps = self._engine.run(
            adapter,
            sgains,
            servos,
            lambda sg: self._strokes(
                axis, start, end, speed, accel, iterations, dwell
            ),
            gcmd,
            accel_chip=chip,
            accel_chip_name=chip_name,
        )
        sg0 = sgains[0]
        gcmd.respond_info(
            "sweep done - reverting to first step (%.1f Hz) until you apply "
            "the recommendation" % (sg0 / 10.0,)
        )
        adapter.revert(sgains)
        self._restore()
        report_args = [
            "--tag",
            tag,
            "--steps",
            ",".join(s.name for s in steps),
            "--axis",
            axis,
        ]
        if chip is not None:
            report_args.append("--require-accel")
        self._run(gcmd, "servo_gain_report.py", report_args, 120.0)
        return steps

    cmd_SERVO_REFINE_GAIN_help = (
        "1-D sensitivity sweep of a single drive gain around the current "
        "operating point, holding the other two fixed. PARAM=position|speed|"
        "integral. Reads the current gains from the drive; sweeps either an "
        "explicit VALUES= list or the current value +-SPAN over STEPS points "
        "(default +-30%% in 5 steps, always including the current value). "
        "Writes each step to EVERY drive on AXIS (both CoreXY lanes), one "
        "capture per step, restores the original gains afterwards (also on "
        "failure), then renders the comparison. Params PARAM AXIS VALUES SPAN "
        "STEPS CURRENT START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO "
        "(comma list override)"
    )

    def cmd_SERVO_REFINE_GAIN(self, gcmd: Any) -> list[SweepStep]:
        param = gcmd.get("PARAM", "").lower()
        if param not in GAIN_PARAMS:
            raise gcmd.error(
                "PARAM must be position, speed or integral (got %r)"
                % (gcmd.get("PARAM", ""),)
            )
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "refine")
        gains = self._read_gains(servos[0])
        current = gcmd.get_int("CURRENT", gains[param], minval=1)
        span = gcmd.get_float("SPAN", 0.3, above=0.0, below=1.0)
        stepcount = gcmd.get_int("STEPS", 5, minval=2)
        values_text = gcmd.get("VALUES", None)
        try:
            values = refine_values(current, values_text, span, stepcount)
            validate_gain_values(values, param)
        except ValueError as e:
            raise gcmd.error("SERVO_REFINE_GAIN: %s" % (e,))
        self._prep(axis, dwell)
        self._set_manual_tuning(servos)
        adapter = SingleGainAdapter(self, servos, param, tag, gains, current)

        def on_revert() -> None:
            gcmd.respond_info(
                "restoring original gains pos %d / speed %d / integral %d on %s"
                % (
                    gains["position"],
                    gains["speed"],
                    gains["integral"],
                    ", ".join(servos),
                )
            )

        steps = self._run_sweep_with_revert(
            adapter,
            values,
            servos,
            lambda v: self._strokes(
                axis, start, end, speed, accel, iterations, dwell
            ),
            gcmd,
            on_revert,
        )
        self._run(
            gcmd,
            "servo_refine_report.py",
            [
                "--param",
                param,
                "--tag",
                tag,
                "--steps",
                ",".join(s.name for s in steps),
                "--reference",
                "%d" % (current,),
            ],
            120.0,
        )
        return steps

    cmd_SERVO_SWEEP_INERTIA_help = (
        "Empirical inertia sweep, gain-sweep style. Resolves every servo "
        "driving AXIS (both drives on CoreXY), writes each C00.06 ratio in "
        "RATIOS (percent, comma list) identically to all of them, one capture "
        "per step of all drives, renders a comparison PNG. Restores the "
        "original ratio afterwards (also on failure). Params RATIOS AXIS "
        "START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO (comma list "
        "override)"
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
        ratios: list[int] = []
        for r in self._floats(gcmd.get("RATIOS", "40,70,100,130")):
            rv = int(r)
            if not 0 <= rv <= 12000:
                raise gcmd.error(
                    "ratio %d outside C00.06 range 0..12000 (%%)" % (rv,)
                )
            if rv not in ratios:
                ratios.append(rv)
        ratios.sort()
        original = self._read_param(servos[0], INERTIA_RATIO_ADDR)
        self._prep(axis, dwell)
        adapter = InertiaRatioAdapter(self, servos, tag, original)

        def on_revert() -> None:
            gcmd.respond_info(
                "restoring C00.06 ratio %d%% on %s"
                % (original, ", ".join(servos))
            )

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
        self._run(
            gcmd,
            "servo_inertia_report.py",
            ["--tag", tag, "--steps", ",".join(s.name for s in steps)],
            120.0,
        )
        return steps

    cmd_SERVO_SWEEP_ACCEL_help = (
        "Accel sweep to find the max non-saturating acceleration. Runs one "
        "capture of strokes per ACCELS entry (mm/s^2, comma list, toolhead "
        "frame) named <TAG>_a<ACCEL>, then renders a torque-saturation report. "
        "AXIS=X/Y strokes a single axis; AXIS=A/B strokes a CoreXY diagonal so "
        "one motor carries the whole load (belt accel is sqrt(2)x on a "
        "diagonal). Restores the velocity limit afterwards (also on failure). "
        "TORQUE_LIMIT is the drive's available-torque ceiling in per-mille "
        "of rated (default 1400); samples at/above it count as railed. "
        "Params ACCELS AXIS SPEED START END ITERATIONS DWELL_MS TAG "
        "TORQUE_LIMIT"
    )

    def cmd_SERVO_SWEEP_ACCEL(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "accel")
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
        for prep_axis in plan.prep:
            self._prep(prep_axis, dwell)
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
            steps = self._engine.run(adapter, accels, servos, run_step, gcmd)
        finally:
            self._restore()
        torque_limit = gcmd.get_int("TORQUE_LIMIT", 1400, minval=1)
        self._run(
            gcmd,
            "servo_accel_report.py",
            [
                "--tag",
                tag,
                "--steps",
                ",".join(s.name for s in steps),
                "--torque-limit",
                "%d" % (torque_limit,),
            ],
            120.0,
        )
        return steps

    cmd_SERVO_SET_STIFFNESS_help = (
        "Vendor-table tuning - switch to standard mode (C00.04=1) and set "
        "C00.05 stiffness level 1..31. Params LEVEL SERVO"
    )

    def cmd_SERVO_SET_STIFFNESS(self, gcmd: Any) -> None:
        servo = self._servo(gcmd)
        level = gcmd.get_int("LEVEL", minval=1, maxval=31)
        self.gcode.run_script_from_command(
            "\n".join(
                [
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x05 VALUE=1 TYPE=u16"
                    % servo,
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x06 VALUE=%d TYPE=u16"
                    % (servo, level),
                ]
            )
        )
        self.cmd_SERVO_SHOW_TUNING(gcmd)


def load_config(config: Any) -> ServoCalibration:
    return ServoCalibration(config)
