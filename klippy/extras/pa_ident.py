"""In-air pressure-advance identification via TMC load telemetry.

Runs a scripted extrude-into-air velocity schedule while sampling the
extruder TMC driver's StallGuard load register, and writes the samples
plus the commanded schedule to a CSV for offline fitting with
scripts/fit_pa_from_load.py.

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
        return pre, post

    def _build_schedule(self, velocities, dwell, anchor_time):
        script = ["SAVE_GCODE_STATE NAME=PA_LOAD_IDENT", "M83"]
        schedule = []
        current_time = anchor_time
        staircase = velocities + velocities[-2::-1]
        v_min = min(velocities)
        pulses = []
        for velocity in velocities[len(velocities) // 2 :]:
            pulses += [velocity, v_min]
        for velocity in staircase + pulses:
            total_e = velocity * dwell
            segment_start = current_time
            while total_e > 0.0:
                chunk = min(total_e, MAX_SEGMENT_E)
                script.append("G1 E%.4f F%.1f" % (chunk, velocity * 60.0))
                total_e -= chunk
            current_time += dwell
            schedule.append((segment_start, current_time, velocity))
        script.append("M400")
        script.append("RESTORE_GCODE_STATE NAME=PA_LOAD_IDENT")
        return script, schedule, current_time

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

    def _write_csv(self, path, tmc_name, reg_name, schedule, smooth_time):
        with open(path, "w") as f:
            f.write("# pa_ident v1\n")
            f.write("# tmc=%s reg=%s\n" % (tmc_name, reg_name))
            f.write("# smooth_time=%.6f\n" % (smooth_time,))
            for start, end, velocity in schedule:
                f.write("S,%.6f,%.6f,%.4f\n" % (start, end, velocity))
            for sample_time, value in self.samples:
                f.write("D,%.6f,%d\n" % (sample_time, value))

    cmd_PA_LOAD_IDENT_help = (
        "Run an in-air extrusion schedule while sampling extruder TMC "
        "load telemetry; writes a CSV for fit_pa_from_load.py"
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
        smooth_time = gcmd.get_float("SMOOTH_TIME", 0.03, minval=0.0)
        interval = gcmd.get_float("INTERVAL", MIN_POLL_PAUSE, minval=0.0)
        out_path = gcmd.get("OUT", "/tmp/pa_ident.csv")
        sgt = gcmd.get_int("SGT", None, minval=-64, maxval=63)

        pre_script, post_script = self._smoothing_scripts(
            gcmd, toolhead, smooth_time
        )
        anchor_time = toolhead.get_last_move_time()
        script, schedule, end_time = self._build_schedule(
            velocities, dwell, anchor_time
        )
        if pre_script is not None:
            script.insert(0, pre_script)

        self.samples = []
        self.poll_error = None
        reactor = self.printer.get_reactor()
        poll_done = reactor.completion()
        self.poll_deadline = end_time + POLL_TAIL_TIME
        reactor.register_callback(
            lambda e: self._poll(mcu_tmc, mcu, reg_name, interval, poll_done)
        )

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
        if self.poll_error is not None:
            raise gcmd.error("TMC load polling failed: %s" % (self.poll_error,))
        if not self.samples:
            raise gcmd.error("No load samples collected")

        self._write_csv(
            out_path, self.tmc_name, reg_name, schedule, smooth_time
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


def load_config(config):
    return PAIdent(config)
