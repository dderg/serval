from __future__ import annotations

from typing import Any, Callable

from .. import servo_param, servo_strokes
from .common import _applied
from .fit import DynamicsFitCommands
from .params import (
    C00_06_INERTIA_RATIO_MAX,
    GAIN_LIST_PARAMS,
    GAIN_PARAMS,
    INERTIA_RATIO_ADDR,
    SYNC_LOSS_COUNT_ADDR,
    SYNC_LOSS_THRESHOLD_ADDR,
    validate_gain_values,
)
from .sweep import SingleGainAdapter, SweepStep


class GainCommands(DynamicsFitCommands):
    cmd_SERVO_CALIBRATE_INERTIA_RATIO_help = (
        "Step 2 of servo tuning - identify the load inertia and print the "
        "recommended C00.06 (on coupled_xy kinematics: for both belt "
        "directions, via the coupled X+Y grid and mode-space fit; the "
        "drive takes one scalar, so start from the light-direction number "
        "and confirm with SERVO_SWEEP_INERTIA). TORQUE_NM and INERTIA_KGM2 "
        "required (config or param). Params as SERVO_MEASURE_INERTIA plus "
        "TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=True)
        self._run_fit(gcmd, gcmd.get("NAME", "inertia"), torque, inertia)

    def _write_gains(self, servos: list[str], values: dict[str, int]) -> None:
        lines = [
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
            % (servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in values
        ]
        with servo_param.suppress_write_log():
            self.gcode.run_script_from_command("\n".join(lines))

    def _gain_write_records(
        self, servos: list[str], values: dict[str, int]
    ) -> list[dict[str, Any]]:
        return [
            _applied(servo, GAIN_PARAMS[name][0], values[name])
            for servo in servos
            for name in values
        ]

    def _read_gains(self, servo: str) -> dict[str, int]:
        return {
            name: self._read_param(servo, GAIN_PARAMS[name][0])
            for name in GAIN_PARAMS
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
            (
                "C13.02 sync loss fault threshold / C13.04 sync loss count:",
                [SYNC_LOSS_THRESHOLD_ADDR, SYNC_LOSS_COUNT_ADDR],
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
        ratio = gcmd.get_int("RATIO", minval=0, maxval=C00_06_INERTIA_RATIO_MAX)
        self.gcode.run_script_from_command(
            "SERVO_PARAM SERVO=%s SET=%s VALUE=%d TYPE=u16"
            % (servo, INERTIA_RATIO_ADDR, ratio)
        )

    cmd_SERVO_APPLY_GAINS_help = (
        "Switch the drive(s) to manual tuning (C00.04=0) and write gain set "
        "1 to every servo driving the axis. POS_GAIN 0.1 rad/s, SPEED_GAIN "
        "0.1 Hz, INTEGRAL 0.01 ms, TORQUE_FILTER Hz (C01.18, only written "
        "when given). Params AXIS or SERVO (comma list)"
    )

    def cmd_SERVO_APPLY_GAINS(self, gcmd: Any) -> None:
        servos = self._servos(gcmd)
        values = {
            "position": gcmd.get_int("POS_GAIN", 400),
            "speed": gcmd.get_int("SPEED_GAIN", 250),
            "integral": gcmd.get_int("INTEGRAL", 3184),
        }
        torque_filter = gcmd.get("TORQUE_FILTER", None)
        if torque_filter is not None:
            values["torque_filter"] = int(torque_filter)
            validate_gain_values([values["torque_filter"]], "torque_filter")
        self._set_manual_tuning(servos)
        self._write_gains(servos, values)
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
        "Sweep of exactly one drive gain, shaper-calibrate style. Give one "
        "of POS_GAINS (0.1 rad/s units), SPEED_GAINS (0.1 Hz units, default "
        "500,650,800,1000), INTEGRALS (0.01 ms units) or TORQUE_FILTERS "
        "(C01.18 torque feedforward filter cutoff, Hz) as a comma list; "
        "the other params stay at their current drive values, so tune "
        "each one individually. Resolves every servo driving AXIS (both "
        "drives on CoreXY; they must agree on the current gains), writes "
        "each entry to all of them, one capture per step of all "
        "drives into a run directory, then servo-cal analyzes it into "
        "results.json with a typed verdict (the recommended step). "
        "With an accelerometer (accel_chip config option or ACCEL_CHIP=) "
        "each step also records vibration data next to its capture. Always "
        "restores the gains that were active before the sweep (also on "
        "failure). APPLY=1 writes the verdict's "
        "recommended gains after the restore, reads them back, and runs one "
        "SERVO_MEASURE_TRACKING to report before/after tracking metrics "
        "(default APPLY=0, report-only). SERVO= (comma list) restricts the "
        "sweep to a subset of the axis servos; BASE_GAIN then pins the "
        "swept gain on every non-swept axis servo at that value "
        "for an asymmetric-gain experiment; those servos are restored too. "
        "PATTERN=1 replaces the single-axis strokes with a TEST_SPEED-style "
        "XY pattern (diagonals + box over the configured XY bounds inset by "
        "BOUND, then over a SMALL_SIZE box at center) exciting every XY "
        "servo; segments too short to reach SPEED run triangular profiles "
        "on purpose and are reported with their achieved peak velocity, and "
        "the per-step settle/overshoot metrics are not meaningful "
        "(continuous motion, no rest windows) - the verdict gates on "
        "resonance and torque saturation only. "
        "Params POS_GAINS SPEED_GAINS INTEGRALS TORQUE_FILTERS AXIS START "
        "END SPEED ACCEL ITERATIONS DWELL_MS TAG ACCEL_CHIP APPLY SERVO "
        "BASE_GAIN PATTERN BOUND SMALL_SIZE"
    )

    def _pattern_setup(
        self, gcmd: Any
    ) -> tuple[list[str], list[tuple[float, float]], float, float, dict]:
        if gcmd.get("BASE_GAIN", None) is not None:
            raise gcmd.error(
                "BASE_GAIN pins gains on the non-swept servos of one axis - "
                "not supported with PATTERN=1, which sweeps every XY servo"
            )
        if (
            gcmd.get("START", None) is not None
            or gcmd.get("END", None) is not None
        ):
            raise gcmd.error(
                "START/END are single-axis stroke bounds - PATTERN=1 uses "
                "the configured XY bounds with BOUND= inset"
            )
        points, start_x, start_y, plan = self._pattern_geometry_params(gcmd)
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            servos = [s.strip() for s in servo.split(",") if s.strip()]
        else:
            kin = self._kin()
            servos = list(
                dict.fromkeys(
                    servo_strokes.axis_servos(gcmd, kin, "X")
                    + servo_strokes.axis_servos(gcmd, kin, "Y")
                )
            )
        return servos, points, start_x, start_y, plan

    def _swept_gain_values(self, gcmd: Any) -> tuple[str, list[int]]:
        given = {p: gcmd.get(p, None) for p in GAIN_LIST_PARAMS}
        named = [p for p, text in given.items() if text is not None]
        if len(named) > 1:
            raise gcmd.error(
                "give exactly one of %s (got %s)"
                % (", ".join(GAIN_LIST_PARAMS), ", ".join(named))
            )
        chosen = named[0] if named else "SPEED_GAINS"
        param = GAIN_LIST_PARAMS[chosen]
        text = given.get(chosen) or "500,650,800,1000"
        values = [int(round(v)) for v in self._floats(text)]
        try:
            validate_gain_values(values, param)
        except ValueError as e:
            raise gcmd.error("SERVO_CALIBRATE_GAINS: %s" % (e,))
        return param, values

    def cmd_SERVO_CALIBRATE_GAINS(self, gcmd: Any) -> list[SweepStep]:
        axis = gcmd.get("AXIS", "X").upper()
        pattern = gcmd.get_int("PATTERN", 0)
        if pattern:
            servos, points, start_x, start_y, pattern_plan = (
                self._pattern_setup(gcmd)
            )
            axis = "XY"
            start = end = None
        else:
            servos = self._servos(gcmd, axis)
            start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "cal")
        param, values = self._swept_gain_values(gcmd)
        if gcmd.get("REVERT_GAIN", None) is not None:
            raise gcmd.error(
                "REVERT_GAIN was removed - the sweep always restores the "
                "gains that were active before it ran; keep a result with "
                "APPLY=1 or SERVO_APPLY_GAINS"
            )
        if gcmd.get("BASE_SPEED_GAIN", None) is not None:
            raise gcmd.error(
                "BASE_SPEED_GAIN was removed - the sweep no longer derives "
                "position/integral from the speed gain; use BASE_GAIN= to "
                "pin the swept gain on the non-swept axis servos"
            )
        base_gain = gcmd.get("BASE_GAIN", None)
        base_servos: list[str] = []
        if base_gain is not None:
            base_gain = int(base_gain)
            try:
                validate_gain_values([base_gain], param)
            except ValueError as e:
                raise gcmd.error("BASE_GAIN: %s" % (e,))
            axis_servos = servo_strokes.axis_servos(gcmd, self._kin(), axis)
            base_servos = [s for s in axis_servos if s not in servos]
            if not base_servos:
                raise gcmd.error(
                    "BASE_GAIN needs SERVO= to name a subset of the "
                    "axis servos - every servo on axis %s is already in the "
                    "sweep" % (axis,)
                )
        apply = gcmd.get_int("APPLY", 0)
        chip, chip_name = self._accel_chip(gcmd)
        affected = list(dict.fromkeys(servos + base_servos))
        prior = {s: self._read_gains(s) for s in affected}
        first = prior[servos[0]]
        for s in servos:
            if prior[s] != first:
                raise gcmd.error(
                    "servos disagree on the current gains (%s=%s vs %s=%s) "
                    "- the sweep holds the non-swept gains at one shared "
                    "value; align the drives first (SERVO_APPLY_GAINS)"
                    % (servos[0], first, s, prior[s])
                )

        def restore_prior() -> None:
            for s, g in prior.items():
                self._write_gains([s], g)

        stroke_plan = {
            "speed": speed,
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if pattern:
            stroke_plan.update(pattern_plan)
        else:
            stroke_plan.update({"start": start, "end": end})
        run = self._begin_run(
            gcmd,
            "gain_sweep",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        adapter = SingleGainAdapter(
            self, servos, param, tag, dict(first), first[param]
        )
        restored = False
        try:
            if pattern:
                self._prep("X", dwell)
                self._prep("Y", dwell)
                moves = servo_strokes.pattern_moves(
                    self.gcode, points, start_x, start_y, speed, accel
                )
                gcmd.respond_info(
                    servo_strokes.pattern_reach_summary(moves, speed)
                )
                self._goto_xy(start_x, start_y, dwell)
            else:
                self._prep(axis, dwell)
                servo_strokes.goto(
                    self.gcode,
                    self.travel_speed,
                    "%s%.3f" % (axis, start),
                    dwell,
                )
            self._set_manual_tuning(servos)
            if base_servos:
                self._set_manual_tuning(base_servos)
                for s in base_servos:
                    pinned = dict(prior[s])
                    pinned[param] = base_gain
                    self._write_gains([s], pinned)
                run.manifest["base_gains"] = {
                    "servos": base_servos,
                    "param": param,
                    "value": base_gain,
                }
                run.write()
                _addr, _lo, _hi, desc, unit, scale = GAIN_PARAMS[param]
                gcmd.respond_info(
                    "base %s pinned at %d (%.4g %s) on %s (held for the "
                    "whole sweep)"
                    % (
                        desc,
                        base_gain,
                        base_gain / scale,
                        unit,
                        ", ".join(base_servos),
                    )
                )

            def run_step(sg: Any) -> None:
                if pattern:
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
                else:
                    self._strokes(
                        axis, start, end, speed, accel, iterations, dwell
                    )

            steps = self._engine.run(
                adapter,
                values,
                servos,
                run_step,
                gcmd,
                accel_chip=chip,
                accel_chip_name=chip_name,
            )
            gcmd.respond_info(
                "sweep done - restoring the pre-sweep gains until you "
                "apply the recommendation"
            )
            restore_prior()
            restored = True
            self._restore()
            results = self._analyze_and_report(gcmd, run)
            self._last_sweep_run, self._last_sweep_results = run, results
            if apply:
                if pattern:
                    gcmd.respond_info(
                        "PATTERN=1: APPLY verification runs single-axis X "
                        "strokes (the tracking measurement is per-axis)"
                    )
                self._apply_verdict(
                    gcmd, run, results, "X" if pattern else axis
                )
        finally:
            if not restored:
                restore_prior()
            self._active_run = None
        return steps

    def _stroke_motion(self, gcmd: Any) -> tuple[float, float, int, int]:
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        return speed, accel, iterations, dwell
