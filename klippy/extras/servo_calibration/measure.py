from __future__ import annotations

import os
from typing import Any

from .. import servo_strain_comp, servo_strokes
from .host import CalibrationHost
from .sweep import SweepStep


class MeasureCommands(CalibrationHost):
    cmd_SERVO_MEASURE_TRACKING_help = (
        "Single accel/speed stroke run with capture - the before/after check "
        "for any tuning change. AXIS=X/Y records every motor driving the axis "
        "(both lanes on CoreXY) into a run directory that servo-cal analyzes "
        "into results.json (per-motor + combined tracking metrics). "
        "AXIS=A/B run a CoreXY 45-degree diagonal that exercises one motor "
        "alone (A=+45 x&y up, motor A; B=-45 x up y down, motor B); SPEED is "
        "the toolhead feedrate, so belt speed is sqrt(2)x SPEED on a diagonal. "
        "Params AXIS START END SPEED ACCEL ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_TRACKING(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        name = gcmd.get("NAME", "track")
        self._measure_tracking(gcmd, axis, name)

    MAX_DIFFERENTIAL_AMPLITUDE_MM = 0.5
    MAX_BUZZ_FREQ_HZ = 2000.0
    MAX_BUZZ_DURATION_S = 300.0

    cmd_SERVO_MEASURE_DIFFERENTIAL_help = (
        "Anti-phase chirp on one AWD belt pair via the engine buzz "
        "generator - the carriage holds still while the two drives strain "
        "the belt against each other, so the capture isolates the "
        "rotor-vs-rotor (differential) modes. servo-cal analyzes the run "
        "into a differential FRF with mode frequency, damping and "
        "coherence. Belt strain is twice AMPLITUDE. Params BELT=A|B "
        "FREQ_START FREQ_END HZ_PER_SEC DURATION AMPLITUDE RAMP DWELL_MS "
        "NAME"
    )

    def _belt_pair(self, gcmd, belt, cmd_name):
        return servo_strokes.belt_pair(
            self.printer, gcmd, self._kin(), belt, cmd_name
        )

    def cmd_SERVO_MEASURE_DIFFERENTIAL(self, gcmd):
        belt = gcmd.get("BELT", "A").upper()
        if belt not in ("A", "B"):
            raise gcmd.error("BELT must be A or B (got %r)" % (belt,))
        pair_names, motors, handle, slots = self._belt_pair(
            gcmd, belt, "SERVO_MEASURE_DIFFERENTIAL"
        )
        freq_start = gcmd.get_float("FREQ_START", 20.0, above=0.0)
        freq_end = gcmd.get_float("FREQ_END", 250.0, above=0.0)
        if max(freq_start, freq_end) > self.MAX_BUZZ_FREQ_HZ:
            raise gcmd.error(
                "buzz frequencies must stay at or below %.0f Hz"
                % (self.MAX_BUZZ_FREQ_HZ,)
            )
        amplitude = gcmd.get_float("AMPLITUDE", 0.05, above=0.0)
        if amplitude > self.MAX_DIFFERENTIAL_AMPLITUDE_MM:
            raise gcmd.error(
                "AMPLITUDE %.3f mm exceeds the %.1f mm differential ceiling "
                "(belt strain between the pair is twice the amplitude)"
                % (amplitude, self.MAX_DIFFERENTIAL_AMPLITUDE_MM)
            )
        hz_per_sec = gcmd.get_float("HZ_PER_SEC", 5.0, above=0.0)
        duration = gcmd.get_float("DURATION", 0.0, minval=0.0)
        if duration <= 0.0:
            duration = max(abs(freq_end - freq_start) / hz_per_sec, 0.5)
        if duration > self.MAX_BUZZ_DURATION_S:
            raise gcmd.error(
                "sweep duration %.0f s exceeds the %.0f s buzz ceiling; "
                "raise HZ_PER_SEC or narrow the frequency band"
                % (duration, self.MAX_BUZZ_DURATION_S)
            )
        ramp = gcmd.get_float(
            "RAMP",
            min(0.1 * duration, 3.0 / min(freq_start, freq_end)),
            above=0.0,
        )
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "diff")
        engine = self.printer.lookup_object("motion_engine")
        stroke_plan = {
            "belt": belt,
            "freq_start": freq_start,
            "freq_end": freq_end,
            "hz_per_sec": hz_per_sec,
            "duration": duration,
            "ramp": ramp,
            "amplitude": amplitude,
            "dwell_ms": dwell,
        }
        run = self._begin_run(
            gcmd, "differential", name, belt, pair_names, stroke_plan
        )
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            gcmd.respond_info(
                "differential sweep on belt %s (%s anti-phase %s): "
                "%.1f->%.1f Hz over %.1f s, amplitude %.3f mm"
                % (
                    belt,
                    pair_names[0],
                    pair_names[1],
                    freq_start,
                    freq_end,
                    duration,
                    amplitude,
                )
            )
            self._start_capture(name, pair_names)
            try:
                engine.resonance_buzz(
                    handle,
                    (1 << slots[0]) | (1 << slots[1]),
                    1 << slots[1],
                    int(round(freq_start * 1000.0)),
                    int(round(freq_end * 1000.0)),
                    int(round(amplitude * 1e6)),
                    int(round(duration * 1000.0)),
                    int(round(ramp * 1000.0)),
                )
                reactor = self.printer.get_reactor()
                reactor.pause(reactor.monotonic() + duration + 0.2)
            finally:
                self._stop_capture()
            run.record_step(SweepStep(name, {}, []))
            self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None

    RINGDOWN_MIN_DWELL_MS = 500
    RINGDOWN_DEFAULT_DWELL_MS = 1500
    RINGDOWN_DEFAULT_CRUISE_MS = 200

    cmd_SERVO_MEASURE_RINGDOWN_help = (
        "Ring-down resonance measurement: short strokes centered on the "
        "axis - accelerate to speed, cruise CRUISE_MS so the accel "
        "transient settles, then a full stop - with post-processors "
        "bypassed and jerk limiting lifted so the stop excites the raw "
        "closed-loop plant. One step per SPEEDS entry; each stroke's "
        "commanded-stop time is recorded. servo-cal fits the post-stop "
        "residual vibration (servo encoders + optional accelerometer) for "
        "per-mode frequency and damping ratio - the free decay a drive "
        "cannot compensate the way it fights a steady sweep. Params "
        "AXIS=X|Y|A|B SPEEDS ACCEL ITERATIONS DWELL_MS CRUISE_MS "
        "ACCEL_CHIP TAG"
    )

    def _ringdown_dynamics(self, gcmd: Any, engine: Any) -> tuple[float, float]:
        """(accel, max_velocity). ACCEL defaults to the printer's effective
        max accel — the sharpest stop excites the widest band (the decel
        pulse's spectral null sits at a/v). Asking for more than the
        machine allows fails loudly: SET_VELOCITY_LIMIT is a cap that
        silently min()s with [printer] max_accel, which would shallow the
        decel AND break the stroke-length math."""
        max_velocity, max_accel, _deviation = engine.effective_limits()
        accel = gcmd.get_float("ACCEL", max_accel, above=0.0)
        if accel > max_accel:
            raise gcmd.error(
                "ACCEL %.0f exceeds the printer's max accel %.0f - the "
                "runtime cap can only lower it, so the strokes would "
                "silently run shallower; raise [printer] max_accel instead"
                % (accel, max_accel)
            )
        return accel, max_velocity

    def _ringdown_strokes(
        self,
        gcmd: Any,
        plan: Any,
        accel: float,
        max_velocity: float,
        cruise_ms: int,
    ) -> list[tuple[int, float, float, float]]:
        """(speed, start_u, end_u, length_mm) per step: the shortest
        centered stroke that reaches cruise speed and holds it for
        `cruise_ms` before the stop."""
        speeds_raw = self._floats(gcmd.get("SPEEDS", None)) or list(self.speeds)
        speeds: list[int] = []
        for s in speeds_raw:
            sv = int(round(s))
            if sv <= 0:
                raise gcmd.error("speed %d must be positive (mm/s)" % (sv,))
            if sv > max_velocity:
                raise gcmd.error(
                    "speed %d exceeds the printer's max velocity %.0f - "
                    "the stroke would silently cruise slower than the "
                    "step claims" % (sv, max_velocity)
                )
            if sv not in speeds:
                speeds.append(sv)
        speeds.sort()
        center_u = (plan.start + plan.end) / 2.0
        avail_half_u = (plan.end - plan.start) / 2.0
        strokes = []
        for speed in speeds:
            length_mm = speed * speed / accel + speed * cruise_ms / 1000.0
            half_u = length_mm / (2.0 * plan.th_per_unit)
            if half_u > avail_half_u:
                raise gcmd.error(
                    "%d mm/s needs a %.1f mm stroke (%.1f mm accel+decel + "
                    "%.1f mm cruise) but only %.1f mm fit around the center "
                    "- lower SPEEDS or CRUISE_MS, or widen START/END"
                    % (
                        speed,
                        length_mm,
                        speed * speed / accel,
                        speed * cruise_ms / 1000.0,
                        2.0 * avail_half_u * plan.th_per_unit,
                    )
                )
            strokes.append(
                (speed, center_u - half_u, center_u + half_u, length_mm)
            )
        return strokes

    def cmd_SERVO_MEASURE_RINGDOWN(self, gcmd: Any) -> None:
        axis = gcmd.get("AXIS", "X").upper()
        plan = servo_strokes.build_plan(gcmd, self._kin(), self.bounds, axis)
        engine = self.printer.lookup_object("motion_engine")
        accel, max_velocity = self._ringdown_dynamics(gcmd, engine)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int(
            "DWELL_MS",
            max(self.dwell_ms, self.RINGDOWN_DEFAULT_DWELL_MS),
            minval=self.RINGDOWN_MIN_DWELL_MS,
        )
        cruise_ms = gcmd.get_int(
            "CRUISE_MS", self.RINGDOWN_DEFAULT_CRUISE_MS, minval=0
        )
        strokes = self._ringdown_strokes(
            gcmd, plan, accel, max_velocity, cruise_ms
        )
        tag = gcmd.get("TAG", "ringdown")
        chip, chip_name = self._accel_chip(gcmd)
        servos = plan.servos
        stroke_plan = {
            "center": (plan.start + plan.end) / 2.0,
            "speed": None,
            "speeds": [s for s, _, _, _ in strokes],
            "accel": accel,
            "iterations": iterations,
            "dwell_ms": dwell,
            "cruise_ms": cruise_ms,
            "accel_chip": chip_name,
        }
        run = self._begin_run(
            gcmd,
            "ringdown",
            tag,
            axis,
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, axis),
        )
        try:
            for prep_axis in plan.prep:
                self._prep(prep_axis, dwell)
            engine.set_post_processor_bypass(True)
            try:
                engine.set_jerk_override(float("inf"))
                try:
                    for i, (speed, start_u, end_u, length_mm) in enumerate(
                        strokes
                    ):
                        name = "%s_v%d" % (tag, speed)
                        gcmd.respond_info(
                            "ringdown %d/%d: %s at %d mm/s, accel %.0f "
                            "mm/s^2, %.1f mm stroke, %d stops"
                            % (
                                i + 1,
                                len(strokes),
                                axis,
                                speed,
                                accel,
                                length_mm,
                                iterations * 2,
                            )
                        )
                        servo_strokes.goto(
                            self.gcode,
                            self.travel_speed,
                            plan.coord(start_u),
                            dwell,
                        )
                        self._start_capture(name, servos)
                        aclient = (
                            None
                            if chip is None
                            else chip.start_internal_client()
                        )
                        try:
                            stops = servo_strokes.emit_strokes_with_stop_times(
                                self.printer,
                                self.gcode,
                                plan.coord,
                                start_u,
                                end_u,
                                plan.th_per_unit,
                                float(speed),
                                accel,
                                iterations,
                                dwell,
                            )
                            self._stop_capture()
                        finally:
                            if aclient is not None:
                                aclient.finish_measurements()
                        step = SweepStep(
                            name,
                            {"speed": float(speed), "stroke_mm": length_mm},
                            [],
                            stops=stops,
                        )
                        if aclient is not None:
                            assert chip_name is not None, (
                                "accel client exists without a chip name"
                            )
                            step.accel = os.path.basename(
                                self._write_accel_csv(
                                    gcmd, aclient, chip_name, name
                                )
                            )
                        run.record_step(step)
                finally:
                    engine.set_jerk_override(None)
            finally:
                engine.set_post_processor_bypass(False)
                self._restore()
            self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None

    MAX_DAMPER_CLAMP_TENTHS = 300.0
    MAX_DAMPER_LEAD_US = 5000.0

    cmd_SERVO_DIFF_DAMPER_help = (
        "Arm or disarm the differential belt-pair damper: the engine adds "
        "an antisymmetric torque offset (60B2h) proportional to the "
        "low-passed velocity difference between the two drives of a belt "
        "- a virtual dashpot between the rotors that damps the inter-motor "
        "belt mode at whatever frequency it sits. Zero on synchronized "
        "motion, so it costs no torque during printing. GAIN is in 0.1% "
        "rated torque per mm/s of differential velocity; GAIN=0 disarms. "
        "LEAD_US adds first-order phase lead to compensate the loop's "
        "transport and torque-path lag. Params BELT=A|B|AB GAIN CLAMP "
        "LPF_HZ LEAD_US"
    )

    def cmd_SERVO_DIFF_DAMPER(self, gcmd):
        belts = gcmd.get("BELT", "AB").upper()
        if belts not in ("A", "B", "AB"):
            raise gcmd.error("BELT must be A, B or AB (got %r)" % (belts,))
        gain = gcmd.get_float("GAIN", minval=0.0)
        clamp = gcmd.get_float("CLAMP", 50.0, above=0.0)
        if clamp > self.MAX_DAMPER_CLAMP_TENTHS:
            raise gcmd.error(
                "CLAMP %.0f exceeds the %.0f x0.1%%-rated-torque ceiling"
                % (clamp, self.MAX_DAMPER_CLAMP_TENTHS)
            )
        lpf_hz = gcmd.get_float("LPF_HZ", 300.0, above=0.0)
        lead_us = gcmd.get_float(
            "LEAD_US", 0.0, minval=0.0, maxval=self.MAX_DAMPER_LEAD_US
        )
        engine = self.printer.lookup_object("motion_engine")
        for belt in belts:
            pair_names, _motors, handle, slots = self._belt_pair(
                gcmd, belt, "SERVO_DIFF_DAMPER"
            )
            engine.set_diff_damper(
                handle,
                slots[0],
                slots[1],
                int(round(gain * 1000.0)),
                int(round(clamp)),
                int(round(lpf_hz * 1000.0)),
                int(round(lead_us)),
            )
            if gain > 0.0:
                gcmd.respond_info(
                    "belt %s damper armed (%s vs %s): gain %.3f "
                    "x0.1%%/(mm/s), clamp %.0f x0.1%%, lpf %.0f Hz, "
                    "lead %.0f us"
                    % (
                        belt,
                        pair_names[0],
                        pair_names[1],
                        gain,
                        clamp,
                        lpf_hz,
                        lead_us,
                    )
                )
            else:
                gcmd.respond_info("belt %s damper disarmed" % (belt,))

    STRAIN_MAP_MIN_LINE_SPACING_MM = servo_strain_comp.MIN_LINE_SPACING_MM

    cmd_SERVO_MEASURE_STRAIN_MAP_help = (
        "Raster the bed with slow constant-speed strokes, one capture per "
        "line - the measurement half of the belt strain map. Differential "
        "pair torque vs (x, y) separates trapped preload, pulley/idler "
        "runout (periodic in travel) and geometry (smooth) when the run is "
        "analyzed. Serpentine X sweeps stepped along Y by LINE_SPACING, "
        "then Y sweeps stepped along X; every line strokes forward and "
        "back so friction asymmetry averages out. Before rastering the "
        "carriage parks at the region center and SERVO_SYNC releases the "
        "trapped preload, so every map shares the same zero (SYNC=0 "
        "skips). CoreXY only. Params SPEED (50) ACCEL (1000) LINE_SPACING "
        "(10) X_START X_END Y_START Y_END DWELL_MS TAG SYNC"
    )

    @staticmethod
    def _raster_levels(start: float, end: float, spacing: float) -> list[float]:
        n = max(1, int(round((end - start) / spacing)))
        return [start + (end - start) * i / n for i in range(n + 1)]

    def cmd_SERVO_MEASURE_STRAIN_MAP(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_MAP requires coupled XY (CoreXY) "
                "kinematics - the strain map is a belt-pair measurement"
            )
        x_start, x_end, y_start, y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        spacing = gcmd.get_float(
            "LINE_SPACING", 10.0, minval=self.STRAIN_MAP_MIN_LINE_SPACING_MM
        )
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "strain")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        # The zero point must be reproducible when the map is APPLIED, not
        # just when it is measured: always the center of the configured
        # calibration area, never the (run-specific) raster region.
        zero_x = (self.bounds["X"][0] + self.bounds["X"][1]) / 2.0
        zero_y = (self.bounds["Y"][0] + self.bounds["Y"][1]) / 2.0
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y_start": y_start,
            "y_end": y_end,
            "speed": speed,
            "accel": accel,
            "line_spacing": spacing,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": [zero_x, zero_y],
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_MEASURE_STRAIN_MAP: [servo_sync] is not "
                    "configured - add it so every map shares a preload "
                    "zero, or pass SYNC=0 to raster without one"
                )
            self._goto_xy(zero_x, zero_y, dwell)
            gcmd.respond_info(
                "strain map zero point: SERVO_SYNC at (%.1f, %.1f) — the "
                "calibration area center; repeat there when applying the "
                "map" % (zero_x, zero_y)
            )
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_map",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        lines = [
            ("X", x_start, x_end, "y", level)
            for level in self._raster_levels(y_start, y_end, spacing)
        ] + [
            ("Y", y_start, y_end, "x", level)
            for level in self._raster_levels(x_start, x_end, spacing)
        ]
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            for i, (axis, start, end, fixed_axis, level) in enumerate(lines):
                if axis == "X":
                    self._goto_xy(start, level, dwell)
                else:
                    self._goto_xy(level, start, dwell)
                name = "%sline_%s%03d" % (
                    axis.lower(),
                    fixed_axis,
                    int(round(level)),
                )
                gcmd.respond_info(
                    "strain map line %d/%d: %s sweep at %s=%.1f"
                    % (i + 1, len(lines), axis, fixed_axis.upper(), level)
                )
                self._start_capture(name, servos)
                self._strokes(axis, start, end, speed, accel, 1, dwell)
                self._stop_capture()
                run.record_step(SweepStep(name, {fixed_axis: level}, []))
            self._restore()
            gcmd.respond_info(
                "strain map raster complete: %d lines in %s"
                % (len(lines), run.run_dir)
            )
        finally:
            self._active_run = None

    STRAIN_RESPONSE_STEPS = (0.0, 1.0, -1.0, 2.0, -2.0)
    MAX_STRAIN_STEP_UM = servo_strain_comp.MAX_STRAIN_STEP_UM

    cmd_SERVO_MEASURE_STRAIN_RESPONSE_help = (
        "Measure the belt stiffness matrix in the rolling regime — the one "
        "the strain map and its compensation operate in (a parked belt "
        "reads ~20% stiffer). Strokes ONE X line forward and back while "
        "stepping a constant antisymmetric offset through each pair's "
        "compensation bank; the line's own strain field is identical on "
        "every pass and cancels out of the offset-response slope, so no "
        "baseline raster is needed. Both pairs' responses are captured per "
        "step, so the direct and cross terms come from the same strokes. "
        "The fitted matrix is stored for SERVO_STRAIN_COMP_BUILD. CoreXY "
        "only, needs [servo_strain_comp]. Params SPEED (50) ACCEL (1000) "
        "STEP_UM (50) SETTLE (0.8) Y (area center) X_START X_END DWELL_MS "
        "TAG SYNC"
    )

    def cmd_SERVO_MEASURE_STRAIN_RESPONSE(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_RESPONSE requires coupled XY "
                "(CoreXY) kinematics - the response is a belt-pair "
                "measurement"
            )
        comp = self.printer.lookup_object("servo_strain_comp", None)
        if comp is None:
            raise gcmd.error(
                "SERVO_MEASURE_STRAIN_RESPONSE needs [servo_strain_comp] "
                "configured - it drives the compensation bank"
            )
        x_start, x_end, _y_start, _y_end = servo_strokes.xy_bounds(
            gcmd, self.bounds
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        step_um = gcmd.get_float(
            "STEP_UM", 50.0, above=0.0, maxval=self.MAX_STRAIN_STEP_UM
        )
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "strainresp")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        zero_x = (self.bounds["X"][0] + self.bounds["X"][1]) / 2.0
        zero_y = (self.bounds["Y"][0] + self.bounds["Y"][1]) / 2.0
        line_y = gcmd.get_float("Y", zero_y)
        session = comp.begin_constant_offsets(gcmd)
        steps_um = [k * step_um for k in self.STRAIN_RESPONSE_STEPS]
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y": line_y,
            "speed": speed,
            "accel": accel,
            "step_um": step_um,
            "offset_steps_um": steps_um,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": [zero_x, zero_y],
            "response_pairs": session.pair_motor_names(),
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_MEASURE_STRAIN_RESPONSE: [servo_sync] is not "
                    "configured - add it so the line shares the maps' "
                    "preload zero, or pass SYNC=0"
                )
            self._goto_xy(zero_x, zero_y, dwell)
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_response",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        reactor = self.printer.get_reactor()
        total = session.pair_count() * len(steps_um)
        try:
            self._prep("X", dwell)
            self._goto_xy(x_start, line_y, dwell)
            for belt_idx in range(session.pair_count()):
                for step_idx, value_um in enumerate(steps_um):
                    slew_s = session.apply(belt_idx, value_um)
                    reactor.pause(reactor.monotonic() + settle + slew_s)
                    name = "belt%s_step%d" % ("ab"[belt_idx], step_idx)
                    gcmd.respond_info(
                        "strain response %d/%d: belt %s at %+.0f um"
                        % (
                            belt_idx * len(steps_um) + step_idx + 1,
                            total,
                            "AB"[belt_idx],
                            value_um,
                        )
                    )
                    self._start_capture(name, servos)
                    self._strokes("X", x_start, x_end, speed, accel, 1, dwell)
                    self._stop_capture()
                    run.record_step(
                        SweepStep(
                            name,
                            {"belt": float(belt_idx), "offset_um": value_um},
                            [],
                        )
                    )
                slew_s = session.apply(belt_idx, 0.0)
                reactor.pause(reactor.monotonic() + slew_s)
            self._restore()
        finally:
            session.clear()
            self._active_run = None
        comp.fit_strain_response(gcmd, run.run_dir)

    TUNE_MAX_ITERS = 5

    cmd_SERVO_STRAIN_COMP_TUNE_help = (
        "Converge the strain map's stiffness matrix against reality: "
        "rebuild the FULL map from RUN=<baseline raster> at the trial "
        "matrix (starting from the probe's values or "
        "STIFFNESS_A/B+CROSS_AB/BA), enable it, sweep an X and a Y "
        "verification line, and refit every belt's direct AND cross "
        "stiffness from the measured response to the applied offsets - "
        "the two sweeps swap the belts' roles, so all four matrix "
        "elements are measured independently. Repeat until the measured "
        "matrix reproduces the applied one (per element, within TOL of "
        "the row's direct stiffness). Each iteration costs two line "
        "sweeps, not a raster, and the converged map is already on disk "
        "and enabled - it is the same full-bed map that was being "
        "verified. The residuals cover only the smooth elastic field: "
        "direction-dependent friction asymmetry and short-wavelength "
        "ripple are invisible to a position-keyed map. Fails loudly if "
        "MAX_ITERS passes don't converge. CoreXY only, needs "
        "[servo_strain_comp] and [servo_sync]. Params RUN (required) "
        "SPACING TOL (0.05) MAX_ITERS (5) Y (map zero) X (map zero) "
        "SPEED (50) ACCEL (1000) SETTLE (0.8) DWELL_MS TAG SYNC"
    )

    def cmd_SERVO_STRAIN_COMP_TUNE(self, gcmd: Any) -> None:
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_STRAIN_COMP_TUNE requires coupled XY (CoreXY) "
                "kinematics - the strain map is a belt-pair measurement"
            )
        comp = self.printer.lookup_object("servo_strain_comp", None)
        if comp is None:
            raise gcmd.error(
                "SERVO_STRAIN_COMP_TUNE needs [servo_strain_comp] "
                "configured - it builds and applies the map"
            )
        spacing = gcmd.get_float(
            "SPACING", None, minval=self.STRAIN_MAP_MIN_LINE_SPACING_MM
        )
        speed = gcmd.get_float("SPEED", 50.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.0)
        settle = gcmd.get_float("SETTLE", 0.8, above=0.0)
        tol = gcmd.get_float("TOL", 0.05, above=0.0, below=0.5)
        max_iters = gcmd.get_int("MAX_ITERS", self.TUNE_MAX_ITERS, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "straintune")
        zero_sync = gcmd.get_int("SYNC", 1, minval=0, maxval=1) == 1
        servos = servo_strokes.axis_servos(gcmd, kin, "X")
        tuner = comp.begin_tune(gcmd, gcmd.get("RUN"), spacing)
        x_start = tuner.plan["x_start"]
        x_end = tuner.plan["x_end"]
        y_start = tuner.plan["y_start"]
        y_end = tuner.plan["y_end"]
        zero_xy = tuner.plan["zero_xy"]
        line_y = gcmd.get_float("Y", zero_xy[1])
        line_x = gcmd.get_float("X", zero_xy[0])
        stroke_plan = {
            "x_start": x_start,
            "x_end": x_end,
            "y_start": y_start,
            "y_end": y_end,
            "y": line_y,
            "x": line_x,
            "speed": speed,
            "accel": accel,
            "tol": tol,
            "dwell_ms": dwell,
            "zero_sync": zero_sync,
            "zero_xy": list(zero_xy),
        }
        if zero_sync:
            sync = self.printer.lookup_object("servo_sync", None)
            if sync is None:
                raise gcmd.error(
                    "SERVO_STRAIN_COMP_TUNE: [servo_sync] is not "
                    "configured - add it so the verification line shares "
                    "the map's preload zero, or pass SYNC=0"
                )
            self._goto_xy(zero_xy[0], zero_xy[1], dwell)
            sync.run(gcmd)
        run = self._begin_run(
            gcmd,
            "strain_tune",
            tag,
            "XY",
            servos,
            stroke_plan,
            self._corexy_rails(gcmd, "X"),
        )
        reactor = self.printer.get_reactor()
        converged = False
        results = None
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            for iteration in range(max_iters):
                tuner.rebuild_and_enable(gcmd)
                reactor.pause(
                    reactor.monotonic() + settle + tuner.enable_ramp_s()
                )
                name_x = "iter%d_x" % iteration
                self._goto_xy(x_start, line_y, dwell)
                self._start_capture(name_x, servos)
                self._strokes("X", x_start, x_end, speed, accel, 1, dwell)
                self._stop_capture()
                name_y = "iter%d_y" % iteration
                self._goto_xy(line_x, y_start, dwell)
                self._start_capture(name_y, servos)
                self._strokes("Y", y_start, y_end, speed, accel, 1, dwell)
                self._stop_capture()
                results = tuner.score_lines(
                    gcmd,
                    run.run_dir,
                    [(name_x, "y", line_y), (name_y, "x", line_x)],
                )
                (kaa, kab), (kba, kbb) = tuner.matrix_rows()
                swept = {
                    "y": line_y,
                    "x": line_x,
                    "kaa": kaa,
                    "kab": kab,
                    "kba": kba,
                    "kbb": kbb,
                }
                for belt_idx, result in enumerate(results):
                    belt = "ab"[belt_idx]
                    swept["s_own_%s" % belt] = result["s_own"]
                    swept["s_cross_%s" % belt] = result["s_cross"]
                    for axis, (rms, base) in result["lines"].items():
                        swept["rms_%s_%s" % (belt, axis)] = rms
                        swept["base_rms_%s_%s" % (belt, axis)] = base
                run.record_step(SweepStep("iter%d" % iteration, swept, []))
                for belt_idx, result in enumerate(results):
                    k_own = tuner.k_matrix[belt_idx][belt_idx]
                    k_cross = tuner.k_matrix[belt_idx][1 - belt_idx]
                    lines = ", ".join(
                        "%s-line residual %.2f%% rms (smooth field was "
                        "%.2f%%)" % (axis.upper(), rms, base)
                        for axis, (rms, base) in sorted(result["lines"].items())
                    )
                    gcmd.respond_info(
                        "tune %d/%d belt %s: measured direct %.1f (map "
                        "used %.1f), cross %.1f (map used %.1f) %%/mm; %s"
                        % (
                            iteration + 1,
                            max_iters,
                            "AB"[belt_idx],
                            result["s_own"],
                            k_own,
                            result["s_cross"],
                            k_cross,
                            lines,
                        )
                    )
                if tuner.converged(results, tol):
                    converged = True
                    break
                tuner.apply(results)
            self._restore()
        finally:
            self._active_run = None
        if not converged:
            raise gcmd.error(
                "did not converge within %d iterations — last measured %s; "
                "the map from the final pass is still enabled"
                % (
                    max_iters,
                    ", ".join(
                        "belt %s direct %.1f cross %.1f"
                        % ("AB"[i], r["s_own"], r["s_cross"])
                        for i, r in enumerate(results)
                    ),
                )
            )
        tuner.store_matrix()
        (kaa, kab), (kba, kbb) = tuner.matrix_rows()
        gcmd.respond_info(
            "converged: stiffness A %.1f B %.1f, cross AB %.1f BA %.1f "
            "%%/mm - all four measured independently on the X and Y "
            "verification lines. Full-bed map rebuilt, written and "
            "ENABLED; the matrix is stored for future builds. The "
            "residuals above only cover the smooth elastic field: "
            "direction-dependent friction asymmetry and sub-%.0fmm ripple "
            "are invisible to a position-keyed map and remain in raw "
            "measurements."
            % (kaa, kbb, kab, kba, servo_strain_comp.FIELD_2D_PITCH_MM)
        )

    cmd_SERVO_MEASURE_INERTIA_help = (
        "Excitation grid for the inertia/friction fit (servo-ident). "
        "coupled_xy kinematics run the X+Y belt grid (SERVOS=/X_START etc "
        "override; travel_speed centers the idle axis between strokes); "
        "cartesian kinematics run a single AXIS grid and reject SERVOS/"
        "X_START/X_END/Y_START/Y_END. PATTERN=1 runs each ACCELS x SPEEDS "
        "cell as a TEST_SPEED-style XY pattern over the configured XY "
        "bounds inset by BOUND (plus a SMALL_SIZE box at center) exciting "
        "every XY servo; segments too short to reach SPEED run triangular "
        "profiles on purpose and are reported by achieved peak velocity, "
        "and it rejects START/END/X_START/X_END/Y_START/Y_END. Params AXIS "
        "START END X_START X_END Y_START Y_END ACCELS SPEEDS ITERATIONS "
        "DWELL_MS NAME SERVOS PATTERN BOUND SMALL_SIZE"
    )

    def _grid_stroke_plan(self, gcmd: Any) -> dict[str, Any]:
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        plan = {
            "speeds": speeds,
            "accels": accels,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        if gcmd.get_int("PATTERN", 0):
            _points, _sx, _sy, pattern_plan = self._pattern_geometry_params(
                gcmd
            )
            plan.update(pattern_plan)
        return plan

    def _grid_servos(
        self, gcmd: Any, kin: Any
    ) -> tuple[list[str], list[Any] | None, str]:
        if kin.coupled_xy():
            override = gcmd.get("SERVOS", None)
            if override is None:
                servos = servo_strokes.axis_servos(gcmd, kin, "X")
            else:
                servos = [s.strip() for s in override.split(",") if s.strip()]
            return servos, servo_strokes.axis_rails(gcmd, kin, "X"), "X"
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        return servo_strokes.axis_servos(gcmd, kin, axis), None, axis

    def cmd_SERVO_MEASURE_INERTIA(self, gcmd: Any) -> None:
        name = gcmd.get("NAME", "ident")
        if gcmd.get_int("PATTERN", 0):
            self._reject_pattern_stroke_bounds(gcmd)
        kin = self._kin()
        servos, belts_rails, axis = self._grid_servos(gcmd, kin)
        self._begin_run(
            gcmd,
            "inertia_grid",
            name,
            axis,
            servos,
            self._grid_stroke_plan(gcmd),
            belts_rails,
        )
        try:
            self._measure_inertia(gcmd, name)
            run = self._active_run
            assert run is not None, "inertia grid ran outside its run"
            run.record_step(SweepStep(name, {}, []))
        finally:
            self._active_run = None

    def _measure_inertia(self, gcmd: Any, name: str) -> None:
        kin = self._kin()
        if kin.coupled_xy():
            self._measure_inertia_corexy(gcmd, name)
            return
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        if gcmd.get_int("PATTERN", 0):
            points, start_x, start_y, _plan = self._pattern_geometry_params(
                gcmd
            )
            self._pattern_grid(gcmd, name, servos, points, start_x, start_y)
            return
        start, end = servo_strokes.axis_bounds(gcmd, self.bounds, axis)
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
        if gcmd.get_int("PATTERN", 0):
            points, start_x, start_y, _plan = self._pattern_geometry_params(
                gcmd
            )
            self._pattern_grid(gcmd, name, servo_list, points, start_x, start_y)
            return
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

    def _reject_pattern_stroke_bounds(self, gcmd: Any) -> None:
        bad = [
            p
            for p in ("START", "END", "X_START", "X_END", "Y_START", "Y_END")
            if gcmd.get(p, None) is not None
        ]
        if bad:
            raise gcmd.error(
                "%s are single-axis stroke bounds - PATTERN=1 uses the "
                "configured XY bounds with BOUND= inset" % (", ".join(bad),)
            )

    def _pattern_reach_report(
        self,
        gcmd: Any,
        points: list[tuple[float, float]],
        start_x: float,
        start_y: float,
        accels: list[float],
        speeds: list[float],
    ) -> None:
        for accel in accels:
            for speed in speeds:
                moves = servo_strokes.pattern_moves(
                    self.gcode, points, start_x, start_y, speed, accel
                )
                gcmd.respond_info(
                    "accel %.0f speed %.0f: %s"
                    % (
                        accel,
                        speed,
                        servo_strokes.pattern_reach_summary(moves, speed),
                    )
                )

    def _pattern_grid(
        self,
        gcmd: Any,
        name: str,
        servos: list[str],
        points: list[tuple[float, float]],
        start_x: float,
        start_y: float,
    ) -> None:
        accels, speeds, iterations, dwell = servo_strokes.grid(
            gcmd, self.accels, self.speeds, self.iterations, self.dwell_ms
        )
        self._prep("X", dwell)
        self._prep("Y", dwell)
        self._pattern_reach_report(
            gcmd, points, start_x, start_y, accels, speeds
        )
        self._goto_xy(start_x, start_y, dwell)
        self._start_capture(name, servos)
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
        self._stop_capture()
        self._restore()
