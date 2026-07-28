"""In-air pressure-advance identification via TMC load telemetry.

Runs a scripted extrude-into-air velocity schedule while sampling the
extruder TMC driver's StallGuard load register, and writes the samples
plus the commanded schedule to a CSV for offline fitting with
scripts/fit_pa_from_load.py.

SGT= accepts either one value for the whole capture or a comma list
with one value per velocity. The list form re-centers every velocity
step in the middle of the SG scale (the raw reading compresses near
its ends, which distorts the fitted advance curve) and runs the
staircase as blocks of constant sgt: motion stops at each sgt change
(register writes are immediate, never mid-motion), and adjacent blocks
re-measure each other's boundary velocities so the fitter can solve an
affine reading map per sgt from the overlaps instead of assuming one.

Copyright (C) 2026  Kalico contributors

This file may be distributed under the terms of the GNU GPLv3 license.
"""

DEFAULT_VELOCITIES = "1,2,3,4,6,8,10,12,15"
MAX_SEGMENT_E = 45.0
TEMP_TOLERANCE = 5.0
POLL_TAIL_TIME = 3.0
MIN_POLL_PAUSE = 0.005


class PAIdent:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.tmc_name = config.get("tmc")
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "PA_LOAD_IDENT",
            self.cmd_PA_LOAD_IDENT,
            desc=self.cmd_PA_LOAD_IDENT_help,
        )
        self.gcode.register_command(
            "PA_SGT_SCAN",
            self.cmd_PA_SGT_SCAN,
            desc=self.cmd_PA_SGT_SCAN_help,
        )
        self.samples = []
        self.poll_error = None
        self.poll_deadline = 0.0

    def _lookup_load_register(self, tmc_object):
        registers = tmc_object.mcu_tmc.name_to_reg
        if "SG_RESULT" in registers:
            return "SG_RESULT"
        return "DRV_STATUS"

    def _check_load_readable(self, gcmd, tmc_object, reg_name):
        fields = tmc_object.fields
        if reg_name != "SG_RESULT":
            return
        if fields.get_field("en_spreadcycle"):
            raise gcmd.error(
                "%s is in spreadcycle; its SG_RESULT only reports load "
                "in stealthchop. Use an SG2 driver (e.g. tmc5160) or "
                "enable stealthchop for the capture" % (self.tmc_name,)
            )

    def _apply_sgt(self, gcmd, tmc_object, sgt):
        fields = tmc_object.fields
        reg_name = fields.lookup_register("sgt", None)
        if reg_name is None:
            raise gcmd.error(
                "%s has no sgt field; SGT= only applies to SG2 drivers"
                % (self.tmc_name,)
            )
        tmc_object.mcu_tmc.set_register(
            reg_name, fields.override_register(reg_name, {"sgt": sgt})
        )
        return reg_name

    def _restore_sgt(self, tmc_object, reg_name):
        tmc_object.mcu_tmc.set_register(
            reg_name, tmc_object.fields.registers.get(reg_name, 0)
        )

    cmd_PA_SGT_SCAN_help = (
        "Sweep the SG2 sgt sensitivity while extruding at a fixed "
        "velocity and report the load reading per sgt value"
    )

    def cmd_PA_SGT_SCAN(self, gcmd):
        tmc_object = self.printer.lookup_object(self.tmc_name)
        mcu_tmc = tmc_object.mcu_tmc
        reg_name = self._lookup_load_register(tmc_object)
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        self._check_extruder_temp(gcmd, toolhead)
        velocity = gcmd.get_float("VELOCITY", 2.0, above=0.0)
        seg_time = gcmd.get_float("TIME", 2.0, above=0.5)
        sgt_lo = gcmd.get_int("FROM", 0, minval=-64, maxval=63)
        sgt_hi = gcmd.get_int("TO", 40, minval=sgt_lo, maxval=63)
        sgt_step = gcmd.get_int("STEP", 4, minval=1)
        interval = gcmd.get_float("INTERVAL", MIN_POLL_PAUSE, minval=0.0)

        reactor = self.printer.get_reactor()
        report = []
        sgt_reg = None
        try:
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=PA_SGT_SCAN\nM83"
            )
            for sgt in range(sgt_lo, sgt_hi + 1, sgt_step):
                sgt_reg = self._apply_sgt(gcmd, tmc_object, sgt)
                self.samples = []
                self.poll_error = None
                poll_done = reactor.completion()
                t_start = toolhead.get_last_move_time()
                self.poll_deadline = t_start + seg_time
                reactor.register_callback(
                    lambda e, done=poll_done: self._poll(
                        mcu_tmc, mcu, reg_name, interval, done
                    )
                )
                self.gcode.run_script_from_command(
                    "G1 E%.4f F%.1f\nM400"
                    % (velocity * seg_time, velocity * 60.0)
                )
                poll_done.wait()
                if self.poll_error is not None:
                    raise gcmd.error(
                        "TMC load polling failed: %s" % (self.poll_error,)
                    )
                settled = [
                    value & 0x3FF
                    for sample_time, value in self.samples
                    if sample_time >= t_start + 0.5 * seg_time
                ]
                if not settled:
                    raise gcmd.error("no samples for sgt=%d" % (sgt,))
                settled.sort()
                report.append(
                    "sgt=%3d: med=%4d min=%4d max=%4d n=%d"
                    % (
                        sgt,
                        settled[len(settled) // 2],
                        settled[0],
                        settled[-1],
                        len(settled),
                    )
                )
        finally:
            if sgt_reg is not None:
                self._restore_sgt(tmc_object, sgt_reg)
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=PA_SGT_SCAN"
            )
        gcmd.respond_info(
            "PA_SGT_SCAN at %.1f mm/s:\n%s" % (velocity, "\n".join(report))
        )

    def _check_extruder_temp(self, gcmd, toolhead):
        heater = toolhead.get_extruder().get_heater()
        systime = self.printer.get_reactor().monotonic()
        status = heater.get_status(systime)
        if status["target"] <= 0.0:
            raise gcmd.error(
                "Extruder has no target temperature set; heat it before "
                "PA_LOAD_IDENT"
            )
        if abs(status["temperature"] - status["target"]) > TEMP_TOLERANCE:
            raise gcmd.error(
                "Extruder temperature %.1f has not reached target %.1f"
                % (status["temperature"], status["target"])
            )

    def _smoothing_scripts(self, gcmd, toolhead, smooth_time):
        if smooth_time == 0.0:
            return None, None
        compat = self.printer.lookup_object("pressure_advance_compat", None)
        if compat is None:
            raise gcmd.error(
                "SMOOTH_TIME shaping needs [pressure_advance_compat]; "
                "pass SMOOTH_TIME=0 to run raw velocity steps"
            )
        extruder_name = toolhead.get_extruder().get_name()
        fields = compat.get_status_fields(extruder_name)
        if "smooth_time" not in fields:
            raise gcmd.error(
                "extruder '%s' has no smooth_triangle post_processor; "
                "add one to its axis or pass SMOOTH_TIME=0" % (extruder_name,)
            )
        pre = "SET_PRESSURE_ADVANCE SMOOTH_TIME=%.6f" % (smooth_time,)
        post = "SET_PRESSURE_ADVANCE SMOOTH_TIME=%.6f" % (
            fields["smooth_time"],
        )
        if "pressure_advance" in fields:
            pre += " ADVANCE=0"
            post += " ADVANCE=%.6f" % (fields["pressure_advance"],)
        if "nonlinear_offset" in fields:
            pre += " OFFSET=0"
            post += " OFFSET=%.6f" % (fields["nonlinear_offset"],)
        return pre, post

    def _dwell_block(self, velocity_seq, dwell, anchor_time):
        lines = []
        schedule = []
        current_time = anchor_time
        for velocity in velocity_seq:
            total_e = velocity * dwell
            segment_start = current_time
            while total_e > 0.0:
                chunk = min(total_e, MAX_SEGMENT_E)
                lines.append("G1 E%.4f F%.1f" % (chunk, velocity * 60.0))
                total_e -= chunk
            current_time += dwell
            schedule.append((segment_start, current_time, velocity))
        return lines, schedule

    def _pulse_sequence(self, velocities, pulse_reps):
        v_min = min(velocities)
        pulses = []
        for velocity in velocities[len(velocities) // 2 :]:
            pulses += [velocity, v_min]
        return pulses * pulse_reps

    def _sgt_blocks(self, velocities, sgt_list):
        groups = []
        for velocity, sgt in zip(velocities, sgt_list):
            if groups and groups[-1][0] == sgt:
                groups[-1][1].append(velocity)
            else:
                groups.append((sgt, [velocity]))
        blocks = []
        for i, (sgt, vels) in enumerate(groups):
            seq = list(vels)
            if i > 0:
                seq.insert(0, groups[i - 1][1][-1])
            if i + 1 < len(groups):
                seq.append(groups[i + 1][1][0])
            blocks.append((sgt, seq))
        return blocks

    def _parse_sgt(self, gcmd, velocities):
        sgt_str = gcmd.get("SGT", None)
        if sgt_str is None:
            return None, None
        try:
            sgt_values = [int(s) for s in sgt_str.split(",")]
        except ValueError:
            raise gcmd.error("Malformed SGT list %r" % (sgt_str,))
        if any(s < -64 or s > 63 for s in sgt_values):
            raise gcmd.error("SGT values must be in -64..63")
        if len(sgt_values) == 1:
            return sgt_values[0], None
        if len(sgt_values) != len(velocities):
            raise gcmd.error(
                "SGT list has %d values for %d velocities; give one per "
                "velocity (or a single value)"
                % (len(sgt_values), len(velocities))
            )
        return None, sgt_values

    def _poll(self, mcu_tmc, mcu, reg_name, interval, done):
        reactor = self.printer.get_reactor()
        try:
            while True:
                t_before = reactor.monotonic()
                if mcu.estimated_print_time(t_before) > self.poll_deadline:
                    break
                value = mcu_tmc.get_register(reg_name)
                t_after = reactor.monotonic()
                sample_time = mcu.estimated_print_time(
                    0.5 * (t_before + t_after)
                )
                self.samples.append((sample_time, value))
                reactor.pause(t_after + max(interval, MIN_POLL_PAUSE))
        except Exception as e:
            self.poll_error = str(e)
        done.complete(None)

    def _write_csv(
        self, path, tmc_name, reg_name, schedule, smooth_time, sgt=None
    ):
        with open(path, "w") as f:
            f.write("# pa_ident v2\n")
            f.write("# tmc=%s reg=%s\n" % (tmc_name, reg_name))
            f.write("# smooth_time=%.6f\n" % (smooth_time,))
            if sgt is not None:
                f.write("# sgt=%d\n" % (sgt,))
            for row in schedule:
                if len(row) == 4:
                    f.write("S,%.6f,%.6f,%.4f,%d\n" % row)
                else:
                    f.write("S,%.6f,%.6f,%.4f\n" % row)
            for sample_time, value in self.samples:
                f.write("D,%.6f,%d\n" % (sample_time, value))

    cmd_PA_LOAD_IDENT_help = (
        "Capture extruder TMC load over a scripted in-air extrusion "
        "velocity schedule and write a CSV for offline pressure-advance "
        "fitting"
    )

    def cmd_PA_LOAD_IDENT(self, gcmd):
        tmc_object = self.printer.lookup_object(self.tmc_name)
        mcu_tmc = tmc_object.mcu_tmc
        reg_name = self._lookup_load_register(tmc_object)
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        self._check_extruder_temp(gcmd, toolhead)
        self._check_load_readable(gcmd, tmc_object, reg_name)

        velocities_str = gcmd.get("VELOCITIES", DEFAULT_VELOCITIES)
        try:
            velocities = [float(v) for v in velocities_str.split(",")]
        except ValueError:
            raise gcmd.error("Malformed VELOCITIES list %r" % (velocities_str,))
        if not velocities or any(v <= 0.0 for v in velocities):
            raise gcmd.error("VELOCITIES must be positive")
        dwell = gcmd.get_float("DWELL", 3.0, above=0.0)
        pulse_reps = gcmd.get_int("PULSES", 1, minval=1)
        smooth_time = gcmd.get_float("SMOOTH_TIME", 0.03, minval=0.0)
        interval = gcmd.get_float("INTERVAL", MIN_POLL_PAUSE, minval=0.0)
        out_path = gcmd.get("OUT", "/tmp/pa_ident.csv")
        sgt, sgt_list = self._parse_sgt(gcmd, velocities)
        pulse_sgt = gcmd.get_int("PULSE_SGT", None, minval=-64, maxval=63)
        if pulse_sgt is not None and sgt_list is None:
            raise gcmd.error("PULSE_SGT= requires a per-velocity SGT list")

        pre_script, post_script = self._smoothing_scripts(
            gcmd, toolhead, smooth_time
        )
        if sgt_list is not None:
            schedule = self._run_stepped(
                gcmd,
                tmc_object,
                mcu_tmc,
                mcu,
                toolhead,
                reg_name,
                velocities,
                sgt_list,
                pulse_sgt if pulse_sgt is not None else sgt_list[0],
                dwell,
                pulse_reps,
                interval,
                pre_script,
                post_script,
            )
            csv_sgt = None
        else:
            schedule = self._run_continuous(
                gcmd,
                tmc_object,
                mcu_tmc,
                mcu,
                toolhead,
                reg_name,
                velocities,
                sgt,
                dwell,
                pulse_reps,
                interval,
                pre_script,
                post_script,
            )
            csv_sgt = sgt
        if self.poll_error is not None:
            raise gcmd.error("TMC load polling failed: %s" % (self.poll_error,))
        if not self.samples:
            raise gcmd.error("No load samples collected")

        self._write_csv(
            out_path, self.tmc_name, reg_name, schedule, smooth_time, csv_sgt
        )
        durations = [
            self.samples[i + 1][0] - self.samples[i][0]
            for i in range(len(self.samples) - 1)
        ]
        durations.sort()
        median_period = durations[len(durations) // 2] if durations else 0.0
        gcmd.respond_info(
            "pa_ident: %d samples (median period %.1f ms) over %d segments"
            " -> %s"
            % (
                len(self.samples),
                median_period * 1000.0,
                len(schedule),
                out_path,
            )
        )

    def _start_poll(self, mcu_tmc, mcu, reg_name, interval):
        reactor = self.printer.get_reactor()
        self.samples = []
        self.poll_error = None
        poll_done = reactor.completion()
        reactor.register_callback(
            lambda e: self._poll(mcu_tmc, mcu, reg_name, interval, poll_done)
        )
        return poll_done

    def _run_continuous(
        self,
        gcmd,
        tmc_object,
        mcu_tmc,
        mcu,
        toolhead,
        reg_name,
        velocities,
        sgt,
        dwell,
        pulse_reps,
        interval,
        pre_script,
        post_script,
    ):
        staircase = velocities + velocities[-2::-1]
        sequence = staircase + self._pulse_sequence(velocities, pulse_reps)
        anchor_time = toolhead.get_last_move_time()
        lines, schedule = self._dwell_block(sequence, dwell, anchor_time)
        script = ["SAVE_GCODE_STATE NAME=PA_LOAD_IDENT", "M83"]
        if pre_script is not None:
            script.insert(0, pre_script)
        script += lines
        script.append("M400")
        script.append("RESTORE_GCODE_STATE NAME=PA_LOAD_IDENT")

        self.poll_deadline = schedule[-1][1] + POLL_TAIL_TIME
        poll_done = self._start_poll(mcu_tmc, mcu, reg_name, interval)
        sgt_reg = None
        if sgt is not None:
            sgt_reg = self._apply_sgt(gcmd, tmc_object, sgt)
        try:
            self.gcode.run_script_from_command("\n".join(script))
            poll_done.wait()
        finally:
            if sgt_reg is not None:
                self._restore_sgt(tmc_object, sgt_reg)
            if post_script is not None:
                self.gcode.run_script_from_command(post_script)
        return schedule

    def _run_stepped(
        self,
        gcmd,
        tmc_object,
        mcu_tmc,
        mcu,
        toolhead,
        reg_name,
        velocities,
        sgt_list,
        pulse_sgt,
        dwell,
        pulse_reps,
        interval,
        pre_script,
        post_script,
    ):
        blocks = self._sgt_blocks(velocities, sgt_list)
        blocks.append((pulse_sgt, self._pulse_sequence(velocities, pulse_reps)))
        self.poll_deadline = float("inf")
        poll_done = self._start_poll(mcu_tmc, mcu, reg_name, interval)
        schedule = []
        sgt_reg = None
        try:
            preamble = ["SAVE_GCODE_STATE NAME=PA_LOAD_IDENT", "M83"]
            if pre_script is not None:
                preamble.insert(0, pre_script)
            self.gcode.run_script_from_command("\n".join(preamble))
            for block_sgt, sequence in blocks:
                sgt_reg = self._apply_sgt(gcmd, tmc_object, block_sgt)
                anchor_time = toolhead.get_last_move_time()
                lines, rows = self._dwell_block(sequence, dwell, anchor_time)
                self.gcode.run_script_from_command("\n".join(lines + ["M400"]))
                schedule += [
                    (t0, t1, velocity, block_sgt) for t0, t1, velocity in rows
                ]
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=PA_LOAD_IDENT"
            )
            self.poll_deadline = toolhead.get_last_move_time() + POLL_TAIL_TIME
            poll_done.wait()
        except BaseException:
            self.poll_deadline = 0.0
            raise
        finally:
            poll_done.wait()
            if sgt_reg is not None:
                self._restore_sgt(tmc_object, sgt_reg)
            if post_script is not None:
                self.gcode.run_script_from_command(post_script)
        return schedule


def load_config(config):
    return PAIdent(config)
