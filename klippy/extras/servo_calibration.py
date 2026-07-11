# Servo calibration toolkit (A6-EC over EtherCAT). Loaded only when a
# printer.cfg contains a [servo_calibration] section (typically on the
# EtherCAT bench, so no config in this repo references it); run-invariant
# values (motor datasheet, stroke window, drive names, excitation grid) live
# in the config section and every command reads them as overridable defaults.
# Command and option reference: docs/rewrite/servo-calibration.md.
import logging
import math
import os
import subprocess
import sys
import time

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


def refine_values(current, values_text, span, steps):
    if values_text is not None:
        vals = [
            int(round(float(v))) for v in values_text.split(",") if v.strip()
        ]
        if not vals:
            raise ValueError("VALUES lists no usable numbers")
    else:
        if steps < 2:
            raise ValueError("STEPS must be at least 2")
        if not 0.0 < span < 1.0:
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


def validate_gain_values(values, param):
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


class ServoCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.servos = config.getlist("servos", ["stepper_x", "stepper_y"])
        self.rated_torque_nm = config.getfloat(
            "rated_torque_nm", None, above=0.0
        )
        self.rotor_inertia_kgm2 = config.getfloat(
            "rotor_inertia_kgm2", None, above=0.0
        )
        self.bounds = {
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
        for name in (
            "SERVO_MEASURE_TRACKING",
            "SERVO_MEASURE_DIFFERENTIAL",
            "SERVO_MEASURE_INERTIA",
            "SERVO_MEASURE_INERTIA_COREXY",
            "SERVO_MEASURE_FRICTION",
            "SERVO_FIT_DYNAMICS",
            "SERVO_FIT_DYNAMICS_COREXY",
            "SERVO_CALIBRATE_INERTIA_RATIO",
            "SERVO_CALIBRATE_INERTIA_RATIO_COREXY",
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

    def _grid(self, gcmd):
        accels = self._floats(gcmd.get("ACCELS", None)) or self.accels
        speeds = self._floats(gcmd.get("SPEEDS", None)) or self.speeds
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        return accels, speeds, iterations, dwell

    def _floats(self, text):
        if text is None:
            return None
        return [float(p.strip()) for p in text.split(",") if p.strip()]

    def _axis_bounds(self, gcmd, axis):
        lo, hi = self.bounds.get(axis, (None, None))
        start = gcmd.get_float("START", lo)
        end = gcmd.get_float("END", hi)
        if start is None or end is None:
            raise gcmd.error(
                "START/END required for axis %s - no bounds configured"
                % (axis,)
            )
        return start, end

    def _xy_bounds(self, gcmd):
        return (
            gcmd.get_float("X_START", self.bounds["X"][0]),
            gcmd.get_float("X_END", self.bounds["X"][1]),
            gcmd.get_float("Y_START", self.bounds["Y"][0]),
            gcmd.get_float("Y_END", self.bounds["Y"][1]),
        )

    def _motor(self, gcmd, required):
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

    def _servo(self, gcmd):
        default = self.servos[0] if len(self.servos) == 1 else None
        servo = gcmd.get("SERVO", default)
        if servo is None:
            raise gcmd.error(
                "SERVO= is required - name the drive explicitly (e.g. SERVO=motor_a)"
            )
        return servo

    def _servos(self, gcmd, axis=None):
        servo = gcmd.get("SERVO", None)
        if servo is not None:
            return [s.strip() for s in servo.split(",") if s.strip()]
        if axis is None:
            axis = gcmd.get("AXIS", None)
        if axis is not None:
            return self._axis_servos(gcmd, axis.upper())
        if len(self.servos) == 1:
            return [self.servos[0]]
        raise gcmd.error(
            "AXIS= or SERVO= is required (SERVO= accepts a comma list)"
        )

    def _axis_rails(self, gcmd, axis):
        from . import servo_axis

        if axis not in ("X", "Y", "Z"):
            raise gcmd.error("AXIS must be X, Y or Z (got %r)" % (axis,))
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        lane = "XYZ".index(axis)
        lanes = [0, 1] if kin.coupled_xy() and lane in (0, 1) else [lane]
        rails = []
        for i in lanes:
            rail = kin.rails[i]
            if not isinstance(rail, servo_axis.ServoRail):
                raise gcmd.error(
                    "axis %s is driven by non-servo rail %r"
                    % (axis, rail.get_name())
                )
            rails.append(rail)
        return rails

    def _axis_servos(self, gcmd, axis):
        return [
            m.get_motor_name()
            for r in self._axis_rails(gcmd, axis)
            for m in r.get_motors()
        ]

    def _rail_motors_in_slot_order(self, rail):
        return sorted(rail.get_motors(), key=lambda m: m.get_chain_index())

    def _corexy_fit_layout(self, gcmd):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        if not kin.coupled_xy():
            raise gcmd.error(
                "corexy fit commands need coupled_xy kinematics; use the "
                "non-COREXY variant for cartesian axes"
            )
        rails = self._axis_rails(gcmd, "X")
        pairs = [
            [m.get_motor_name() for m in self._rail_motors_in_slot_order(r)]
            for r in rails
        ]
        sizes = {len(p) for p in pairs}
        servos = [name for pair in pairs for name in pair]
        if sizes == {1}:
            return {"servos": servos, "pairs": None}
        if sizes == {2}:
            nodes = {m.get_node_name() for r in rails for m in r.get_motors()}
            if len(nodes) != 1:
                raise gcmd.error(
                    "AWD corexy fit needs all four drives on one ethercat "
                    "node (a coupled dynamics profile is per node); got "
                    "nodes: %s" % (", ".join(sorted(nodes)),)
                )
            return {
                "servos": servos,
                "pairs": ";".join(",".join(pair) for pair in pairs),
            }
        raise gcmd.error(
            "corexy fit needs one or two drives per belt on both belts, "
            "got %s"
            % (
                "; ".join(
                    "%s: %s" % (r.get_name(short=True), ", ".join(p))
                    for r, p in zip(rails, pairs)
                ),
            )
        )

    def _check_servos_override(self, gcmd, layout):
        override = gcmd.get("SERVOS", None)
        if override is None:
            return
        given = sorted(s.strip() for s in override.split(",") if s.strip())
        if given != sorted(layout["servos"]):
            raise gcmd.error(
                "SERVOS=%s does not match the drives the kinematics says "
                "power the belts (%s); the fit pairing is derived from the "
                "kinematics, so drop SERVOS= or fix the config"
                % (override, ", ".join(layout["servos"]))
            )

    def _scalar_fit_drive(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._axis_servos(gcmd, axis)
        drive = gcmd.get("DRIVE", None)
        if drive is None:
            if len(servos) > 1:
                raise gcmd.error(
                    "AXIS=%s records %d drives (%s); pass DRIVE= to pick "
                    "which one the scalar fit describes"
                    % (axis, len(servos), ", ".join(servos))
                )
            return None
        if drive not in servos:
            raise gcmd.error(
                "DRIVE=%s is not among the drives of AXIS=%s (%s)"
                % (drive, axis, ", ".join(servos))
            )
        return drive

    def _strokes(self, axis, start, end, speed, accel, iterations, dwell):
        self._emit_strokes(
            lambda u: "%s%.3f" % (axis, u),
            start,
            end,
            1.0,
            speed,
            accel,
            iterations,
            dwell,
        )

    def _emit_strokes(
        self,
        coord,
        start,
        end,
        th_per_unit,
        speed,
        accel,
        iterations,
        dwell,
    ):
        if end <= start:
            raise self.gcode.error(
                "END=%.1f must exceed START=%.1f" % (end, start)
            )
        length = (end - start) * th_per_unit
        reach = speed * speed / accel
        if reach > length:
            raise self.gcode.error(
                "stroke %.1fmm (toolhead frame) too short to reach %.0fmm/s "
                "at %.0fmm/s^2 (needs %.1fmm)" % (length, speed, accel, reach)
            )
        feed = int(speed * 60)
        lines = ["SET_VELOCITY_LIMIT ACCEL=%.0f" % (accel,), "G90"]
        for _ in range(iterations):
            lines += [
                "G1 %s F%d" % (coord(end), feed),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
                "G1 %s F%d" % (coord(start), feed),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
            ]
        self.gcode.run_script_from_command("\n".join(lines))

    def _diagonal_rail(self, gcmd, axis):
        from . import servo_axis

        kin = self.printer.lookup_object("toolhead").get_kinematics()
        if not kin.coupled_xy():
            raise gcmd.error(
                "AXIS=%s runs a CoreXY diagonal - the active kinematics is "
                "not coupled_xy" % (axis,)
            )
        lane = 0 if axis == "A" else 1
        rail = kin.rails[lane]
        if not isinstance(rail, servo_axis.ServoRail):
            raise gcmd.error(
                "CoreXY lane %d is driven by non-servo rail %r"
                % (lane, rail.get_name())
            )
        return rail

    def _stroke_plan(self, gcmd, axis):
        if axis in ("A", "B"):
            rail = self._diagonal_rail(gcmd, axis)
            x_start, x_end, y_start, y_end = self._xy_bounds(gcmd)
            xc = (x_start + x_end) / 2.0
            yc = (y_start + y_end) / 2.0
            half = min(abs(x_end - x_start), abs(y_end - y_start)) / 2.0
            start = gcmd.get_float("START", -half)
            end = gcmd.get_float("END", half)
            sign = 1.0 if axis == "A" else -1.0

            def coord(u):
                return "X%.3f Y%.3f" % (xc + u, yc + sign * u)

            return {
                "coord": coord,
                "start": start,
                "end": end,
                "th_per_unit": math.sqrt(2.0),
                "servos": [m.get_motor_name() for m in rail.get_motors()],
                "motors": list(rail.get_motors()),
                "prep": ("X", "Y"),
                "diagonal": True,
            }
        start, end = self._axis_bounds(gcmd, axis)
        rails = self._axis_rails(gcmd, axis)
        motors = [m for r in rails for m in r.get_motors()]

        def coord(u):
            return "%s%.3f" % (axis, u)

        return {
            "coord": coord,
            "start": start,
            "end": end,
            "th_per_unit": 1.0,
            "servos": [m.get_motor_name() for m in motors],
            "motors": motors,
            "rails": rails,
            "prep": (axis,),
            "diagonal": False,
        }

    def _goto_xy(self, x, y, dwell):
        self.gcode.run_script_from_command(
            "\n".join(
                [
                    "G90",
                    "G1 X%.3f Y%.3f F%d" % (x, y, int(self.travel_speed * 60)),
                    "M400",
                    "G4 P%d" % (dwell,),
                    "M400",
                ]
            )
        )

    def _prep(self, axis, dwell):
        curtime = self.printer.get_reactor().monotonic()
        toolhead = self.printer.lookup_object("toolhead")
        homed = toolhead.get_kinematics().get_status(curtime)["homed_axes"]
        lines = []
        if axis.lower() not in homed:
            lines.append("G28 %s" % (axis,))
        lines += ["M400", "G4 P%d" % (dwell,), "M400"]
        self.gcode.run_script_from_command("\n".join(lines))

    def _restore(self):
        self.gcode.run_script_from_command("RESET_VELOCITY_LIMIT")

    def _accel_chip(self, gcmd):
        chip_name = gcmd.get("ACCEL_CHIP", self.accel_chip_name)
        if chip_name is None:
            return None, None
        return self.printer.lookup_object(chip_name.strip()), chip_name

    def _write_accel_csv(self, gcmd, aclient, chip_name, step_name):
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

    def _run(self, gcmd, script, args, timeout):
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

        def emit(data):
            buf[0] += data
            if "\n" in buf[0]:
                head, _, buf[0] = buf[0].rpartition("\n")
                gcmd.respond_info(head)

        def on_readable(eventtime):
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

    def cmd_SERVO_MEASURE_TRACKING(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        plan = self._stroke_plan(gcmd, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "track")
        servos = plan["servos"]
        for prep_axis in plan["prep"]:
            self._prep(prep_axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s" % (",".join(servos), name)
        )
        self._emit_strokes(
            plan["coord"],
            plan["start"],
            plan["end"],
            plan["th_per_unit"],
            speed,
            accel,
            iterations,
            dwell,
        )
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()
        report_args = ["--name", name, "--png"]
        rails = plan.get("rails", [])
        if not plan["diagonal"] and len(rails) == 2 and axis in ("X", "Y"):
            belts = ",".join(
                "+".join(
                    "%s:%d"
                    % (
                        m.get_motor_name(),
                        -1 if m.get_invert_direction() else 1,
                    )
                    for m in self._rail_motors_in_slot_order(r)
                )
                for r in rails
            )
            report_args += ["--axis", axis, "--combine-corexy", belts]
        self._run(gcmd, "servo_capture.py", report_args, 120.0)

    MAX_DIFFERENTIAL_AMPLITUDE_MM = 0.5
    MAX_BUZZ_FREQ_HZ = 2000.0
    MAX_BUZZ_DURATION_S = 300.0

    cmd_SERVO_MEASURE_DIFFERENTIAL_help = (
        "Anti-phase chirp on one AWD belt pair via the engine buzz "
        "generator - the carriage holds still while the two drives strain "
        "the belt against each other, so the capture isolates the "
        "rotor-vs-rotor (differential) modes. Renders a differential FRF "
        "PNG with mode frequency, damping and coherence. Belt strain is "
        "twice AMPLITUDE. Params BELT=A|B FREQ_START FREQ_END HZ_PER_SEC "
        "DURATION AMPLITUDE RAMP DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_DIFFERENTIAL(self, gcmd):
        from . import servo_axis

        belt = gcmd.get("BELT", "A").upper()
        if belt not in ("A", "B"):
            raise gcmd.error("BELT must be A or B (got %r)" % (belt,))
        layout = self._corexy_fit_layout(gcmd)
        if layout["pairs"] is None:
            raise gcmd.error(
                "SERVO_MEASURE_DIFFERENTIAL needs two drives per belt "
                "(AWD); this printer has one drive per belt"
            )
        pair_names = layout["pairs"].split(";")["AB".index(belt)].split(",")
        motors = [
            servo_axis.resolve_servo_motor(
                self.printer, name, "SERVO_MEASURE_DIFFERENTIAL"
            )[1]
            for name in pair_names
        ]
        node = self.printer.lookup_object(
            "ethercat_node " + motors[0].get_node_name()
        )
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "belt %s drives have no live EtherCAT engine handle "
                "(node not claimed)" % (belt,)
            )
        slots = [node.get_slot_for_motor(m.get_motor_name()) for m in motors]
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
        self._prep("X", dwell)
        self._prep("Y", dwell)
        engine = self.printer.lookup_object("motion_engine")
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
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s"
            % (",".join(pair_names), name)
        )
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
            self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        pair_spec = "+".join(
            "%s:%d"
            % (m.get_motor_name(), -1 if m.get_invert_direction() else 1)
            for m in motors
        )
        self._run(
            gcmd,
            "servo_diff_report.py",
            [
                "--name",
                name,
                "--pair",
                pair_spec,
                "--freq-start",
                "%g" % (freq_start,),
                "--freq-end",
                "%g" % (freq_end,),
                "--png",
            ],
            120.0,
        )

    cmd_SERVO_MEASURE_INERTIA_help = (
        "Excitation grid for the inertia/friction fit (servo-ident). Params "
        "AXIS START END ACCELS SPEEDS ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_INERTIA(self, gcmd):
        self._measure_inertia(gcmd, gcmd.get("NAME", "ident"))

    def _measure_inertia(self, gcmd, name):
        axis = gcmd.get("AXIS", "X").upper()
        start, end = self._axis_bounds(gcmd, axis)
        servos = self._axis_servos(gcmd, axis)
        accels, speeds, iterations, dwell = self._grid(gcmd)
        self._prep(axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s" % (",".join(servos), name)
        )
        for accel in accels:
            for speed in speeds:
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()

    cmd_SERVO_MEASURE_INERTIA_COREXY_help = (
        "CoreXY excitation grid - one capture of BOTH drives with X and Y "
        "strokes at every accel/speed point. Params SERVOS X_START X_END "
        "Y_START Y_END ACCELS SPEEDS ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_INERTIA_COREXY(self, gcmd):
        self._measure_inertia_corexy(gcmd, gcmd.get("NAME", "ident"))

    def _measure_inertia_corexy(self, gcmd, name, servos=None):
        if servos is None:
            servos = gcmd.get("SERVOS", None)
        if servos is None:
            servos = ",".join(self._axis_servos(gcmd, "X"))
        x_start, x_end, y_start, y_end = self._xy_bounds(gcmd)
        accels, speeds, iterations, dwell = self._grid(gcmd)
        x_center = (x_start + x_end) / 2.0
        y_center = (y_start + y_end) / 2.0
        self._prep("X", dwell)
        self._prep("Y", dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s" % (servos, name)
        )
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
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()

    cmd_SERVO_MEASURE_FRICTION_help = (
        "Slow constant-speed sweeps for the torque-vs-position friction map. "
        "Params AXIS START END SPEED ACCEL ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_FRICTION(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        start, end = self._axis_bounds(gcmd, axis)
        speed = gcmd.get_float("SPEED", 20.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 300.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "friction")
        servos = self._axis_servos(gcmd, axis)
        self._prep(axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START SERVO=%s NAME=%s" % (",".join(servos), name)
        )
        self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()

    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward - runs the inertia "
        "excitation grid, fits mass/viscous/coulomb, and writes a timestamped "
        "profile. Optional TORQUE_NM + INERTIA_KGM2 add the C00.06 "
        "recommendation. On a multi-drive axis DRIVE= picks which drive "
        "the scalar fit describes. Params as SERVO_MEASURE_INERTIA plus "
        "TORQUE_NM INERTIA_KGM2 DRIVE"
    )

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd):
        name = gcmd.get("NAME", "ident")
        torque, inertia = self._motor(gcmd, required=False)
        drive = self._scalar_fit_drive(gcmd)
        self._measure_inertia(gcmd, name)
        args = ["--name", name]
        if drive is not None:
            args += ["--drive", drive]
        if torque is not None:
            args += [
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ]
        self._run(gcmd, "servo_fit_dynamics.py", args, 120.0)

    cmd_SERVO_FIT_DYNAMICS_COREXY_help = (
        "Identify the coupled CoreXY dynamics for torque feedforward - runs "
        "the X+Y excitation grid over every belt drive (two, or four on "
        "AWD), fits the coupled mass matrix, and writes a timestamped "
        "node-level profile. Optional TORQUE_NM + INERTIA_KGM2 "
        "add the C00.06 recommendation. Params as "
        "SERVO_MEASURE_INERTIA_COREXY plus TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_FIT_DYNAMICS_COREXY(self, gcmd):
        name = gcmd.get("NAME", "ident")
        torque, inertia = self._motor(gcmd, required=False)
        layout = self._corexy_fit_layout(gcmd)
        self._check_servos_override(gcmd, layout)
        self._measure_inertia_corexy(
            gcmd, name, servos=",".join(layout["servos"])
        )
        args = ["--name", name, "--structure", "corexy"]
        if layout["pairs"] is not None:
            args += ["--pairs", layout["pairs"]]
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
        "recommended C00.06. TORQUE_NM and INERTIA_KGM2 required (config or "
        "param). Params AXIS TORQUE_NM INERTIA_KGM2 START END ACCELS SPEEDS "
        "ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO(self, gcmd):
        name = gcmd.get("NAME", "inertia")
        torque, inertia = self._motor(gcmd, required=True)
        drive = self._scalar_fit_drive(gcmd)
        self._measure_inertia(gcmd, name)
        drive_args = [] if drive is None else ["--drive", drive]
        self._run(
            gcmd,
            "servo_fit_dynamics.py",
            drive_args
            + [
                "--name",
                name,
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ],
            120.0,
        )

    cmd_SERVO_CALIBRATE_INERTIA_RATIO_COREXY_help = (
        "Step 2 of CoreXY servo tuning - runs the X+Y excitation "
        "grid, fits the coupled mass matrix, and prints C00.06 for both "
        "directions. TORQUE_NM and INERTIA_KGM2 required (config or param). "
        "Params as SERVO_MEASURE_INERTIA_COREXY plus TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO_COREXY(self, gcmd):
        name = gcmd.get("NAME", "inertia")
        torque, inertia = self._motor(gcmd, required=True)
        layout = self._corexy_fit_layout(gcmd)
        self._check_servos_override(gcmd, layout)
        self._measure_inertia_corexy(
            gcmd, name, servos=",".join(layout["servos"])
        )
        pair_args = (
            [] if layout["pairs"] is None else ["--pairs", layout["pairs"]]
        )
        self._run(
            gcmd,
            "servo_fit_dynamics.py",
            pair_args
            + [
                "--name",
                name,
                "--structure",
                "corexy",
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ],
            120.0,
        )

    def _write_gains(self, servos, pos_gain, speed_gain, integral):
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

    def _resolve_node_slot(self, servo):
        from . import servo_axis

        _rail, motor = servo_axis.resolve_servo_motor(
            self.printer, servo, "SERVO_CALIBRATION"
        )
        node = self.printer.lookup_object(
            "ethercat_node " + motor.get_node_name()
        )
        return node, node.get_slot_for_motor(motor.get_motor_name())

    def _read_param(self, servo, addr):
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

    def _read_gains(self, servo):
        return {
            name: self._read_param(servo, GAIN_PARAMS[name][0])
            for name in ("position", "speed", "integral")
        }

    def _set_manual_tuning(self, servos):
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

    def cmd_SERVO_SHOW_TUNING(self, gcmd):
        for servo in self._servos(gcmd):
            self._show_tuning(servo)

    def _show_tuning(self, servo):
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

    def cmd_SERVO_SET_INERTIA_RATIO(self, gcmd):
        servo = self._servo(gcmd)
        ratio = gcmd.get_int("RATIO", minval=0, maxval=12000)
        self.gcode.run_script_from_command(
            "SERVO_PARAM SERVO=%s SET=0x2000.0x07 VALUE=%d TYPE=u16"
            % (servo, ratio)
        )

    cmd_SERVO_APPLY_GAINS_help = (
        "Switch the drive(s) to manual tuning (C00.04=0) and write gain set "
        "1 to every servo driving the axis. POS_GAIN 0.1 rad/s, SPEED_GAIN "
        "0.1 Hz, INTEGRAL 0.01 ms. Params AXIS or SERVO (comma list)"
    )

    def cmd_SERVO_APPLY_GAINS(self, gcmd):
        servos = self._servos(gcmd)
        pos_gain = gcmd.get_int("POS_GAIN", 400)
        speed_gain = gcmd.get_int("SPEED_GAIN", 250)
        integral = gcmd.get_int("INTEGRAL", 3184)
        self._set_manual_tuning(servos)
        self._write_gains(servos, pos_gain, speed_gain, integral)
        for servo in servos:
            self._show_tuning(servo)

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

    def cmd_SERVO_CALIBRATE_GAINS(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = self._axis_bounds(gcmd, axis)
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
        step_names = []
        for i, sg in enumerate(sgains):
            pg = round(sg * 1.6)
            ig = round(1250000 / sg)
            step_names.append("%s_p%d_s%d_i%d" % (tag, pg, sg, ig))
            gcmd.respond_info(
                "gain step %d/%d: pos %.1f rad/s, speed %.1f Hz, Ti %.2f ms"
                " on %s"
                % (
                    i + 1,
                    len(sgains),
                    pg / 10.0,
                    sg / 10.0,
                    ig / 100.0,
                    ", ".join(servos),
                )
            )
            self._write_gains(servos, pg, sg, ig)
            self.gcode.run_script_from_command(
                "SERVO_CAPTURE_START SERVO=%s NAME=%s"
                % (",".join(servos), step_names[-1])
            )
            aclient = None if chip is None else chip.start_internal_client()
            try:
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
                self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
            finally:
                if aclient is not None:
                    aclient.finish_measurements()
            if aclient is not None:
                self._write_accel_csv(gcmd, aclient, chip_name, step_names[-1])
        sg0 = sgains[0]
        gcmd.respond_info(
            "sweep done - reverting to first step (%.1f Hz) until you apply "
            "the recommendation" % (sg0 / 10.0,)
        )
        self._write_gains(servos, round(sg0 * 1.6), sg0, round(1250000 / sg0))
        self._restore()
        report_args = [
            "--tag",
            tag,
            "--steps",
            ",".join(step_names),
            "--axis",
            axis,
        ]
        if chip is not None:
            report_args.append("--require-accel")
        self._run(gcmd, "servo_gain_report.py", report_args, 120.0)

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

    def cmd_SERVO_REFINE_GAIN(self, gcmd):
        param = gcmd.get("PARAM", "").lower()
        if param not in GAIN_PARAMS:
            raise gcmd.error(
                "PARAM must be position, speed or integral (got %r)"
                % (gcmd.get("PARAM", ""),)
            )
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = self._axis_bounds(gcmd, axis)
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
        _addr, _lo, _hi, desc, unit, scale = GAIN_PARAMS[param]
        self._prep(axis, dwell)
        self._set_manual_tuning(servos)
        original = (gains["position"], gains["speed"], gains["integral"])
        step_names = []
        try:
            for i, v in enumerate(values):
                step = dict(gains)
                step[param] = v
                step_names.append("%s_%s_v%d" % (tag, param, v))
                gcmd.respond_info(
                    "refine %s step %d/%d: %s = %d (%.4g %s)%s on %s"
                    % (
                        param,
                        i + 1,
                        len(values),
                        desc,
                        v,
                        v / scale,
                        unit,
                        "  <- current" if v == current else "",
                        ", ".join(servos),
                    )
                )
                self._write_gains(
                    servos, step["position"], step["speed"], step["integral"]
                )
                self.gcode.run_script_from_command(
                    "SERVO_CAPTURE_START SERVO=%s NAME=%s"
                    % (",".join(servos), step_names[-1])
                )
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
                self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        finally:
            gcmd.respond_info(
                "restoring original gains pos %d / speed %d / integral %d on %s"
                % (original[0], original[1], original[2], ", ".join(servos))
            )
            self._write_gains(servos, *original)
            self._restore()
        self._run(
            gcmd,
            "servo_refine_report.py",
            [
                "--param",
                param,
                "--tag",
                tag,
                "--steps",
                ",".join(step_names),
                "--reference",
                "%d" % (current,),
            ],
            120.0,
        )

    cmd_SERVO_SWEEP_INERTIA_help = (
        "Empirical inertia sweep, gain-sweep style. Resolves every servo "
        "driving AXIS (both drives on CoreXY), writes each C00.06 ratio in "
        "RATIOS (percent, comma list) identically to all of them, one capture "
        "per step of all drives, renders a comparison PNG. Restores the "
        "original ratio afterwards (also on failure). Params RATIOS AXIS "
        "START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO (comma list "
        "override)"
    )

    def cmd_SERVO_SWEEP_INERTIA(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        servos = self._servos(gcmd, axis)
        start, end = self._axis_bounds(gcmd, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "inertia")
        ratios = []
        for r in self._floats(gcmd.get("RATIOS", "40,70,100,130")):
            rv = int(r)
            if not 0 <= rv <= 12000:
                raise gcmd.error(
                    "ratio %d outside C00.06 range 0..12000 (%%)" % (rv,)
                )
            if rv not in ratios:
                ratios.append(rv)
        ratios.sort()
        original = self._read_param(servos[0], "0x2000.0x07")
        self._prep(axis, dwell)
        step_names = []
        try:
            for i, rv in enumerate(ratios):
                step_names.append("%s_r%d" % (tag, rv))
                gcmd.respond_info(
                    "inertia step %d/%d: C00.06 ratio %d%% on %s"
                    % (i + 1, len(ratios), rv, ", ".join(servos))
                )
                lines = [
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x07 VALUE=%d TYPE=u16"
                    % (servo, rv)
                    for servo in servos
                ]
                lines.append(
                    "SERVO_CAPTURE_START SERVO=%s NAME=%s"
                    % (",".join(servos), step_names[-1])
                )
                self.gcode.run_script_from_command("\n".join(lines))
                self._strokes(axis, start, end, speed, accel, iterations, dwell)
                self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        finally:
            gcmd.respond_info(
                "restoring C00.06 ratio %d%% on %s"
                % (original, ", ".join(servos))
            )
            self.gcode.run_script_from_command(
                "\n".join(
                    "SERVO_PARAM SERVO=%s SET=0x2000.0x07 VALUE=%d TYPE=u16"
                    % (servo, original)
                    for servo in servos
                )
            )
            self._restore()
        self._run(
            gcmd,
            "servo_inertia_report.py",
            ["--tag", tag, "--steps", ",".join(step_names)],
            120.0,
        )

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

    def cmd_SERVO_SWEEP_ACCEL(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        plan = self._stroke_plan(gcmd, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 2, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        tag = gcmd.get("TAG", "accel")
        raw = self._floats(gcmd.get("ACCELS", None))
        if not raw:
            raise gcmd.error("ACCELS= required (comma list of mm/s^2)")
        accels = []
        for a in raw:
            av = int(a)
            if av <= 0:
                raise gcmd.error("accel %d must be positive (mm/s^2)" % (av,))
            if av not in accels:
                accels.append(av)
        accels.sort()
        servos = plan["servos"]
        for prep_axis in plan["prep"]:
            self._prep(prep_axis, dwell)
        step_names = []
        try:
            for i, av in enumerate(accels):
                step_names.append("%s_a%d" % (tag, av))
                gcmd.respond_info(
                    "accel step %d/%d: %d mm/s^2 on %s"
                    % (i + 1, len(accels), av, ", ".join(servos))
                )
                self.gcode.run_script_from_command(
                    "SERVO_CAPTURE_START SERVO=%s NAME=%s"
                    % (",".join(servos), step_names[-1])
                )
                self._emit_strokes(
                    plan["coord"],
                    plan["start"],
                    plan["end"],
                    plan["th_per_unit"],
                    speed,
                    float(av),
                    iterations,
                    dwell,
                )
                self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
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
                ",".join(step_names),
                "--torque-limit",
                "%d" % (torque_limit,),
            ],
            120.0,
        )

    cmd_SERVO_SET_STIFFNESS_help = (
        "Vendor-table tuning - switch to standard mode (C00.04=1) and set "
        "C00.05 stiffness level 1..31. Params LEVEL SERVO"
    )

    def cmd_SERVO_SET_STIFFNESS(self, gcmd):
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


def load_config(config):
    return ServoCalibration(config)
