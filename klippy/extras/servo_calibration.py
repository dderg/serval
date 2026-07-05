# Servo calibration toolkit (A6-EC over EtherCAT). Enable with a bare
# [servo_calibration]; run-invariant values (motor datasheet, stroke window,
# drive names, excitation grid) live in the config section and every command
# reads them as overridable defaults.
import logging
import os
import subprocess
import sys

SCRIPTS_DIR = os.path.join(
    os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ),
    "scripts",
)


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
        self.dwell_ms = config.getint("dwell_ms", 700, minval=0)
        self.travel_speed = config.getfloat("travel_speed", 100.0, above=0.0)
        for name in (
            "SERVO_MEASURE_TRACKING",
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
            "SERVO_SWEEP_INERTIA",
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

    def _axis_servos(self, gcmd, axis):
        from . import servo_axis

        if axis not in ("X", "Y", "Z"):
            raise gcmd.error("AXIS must be X, Y or Z (got %r)" % (axis,))
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        lane = "XYZ".index(axis)
        lanes = [0, 1] if kin.coupled_xy() and lane in (0, 1) else [lane]
        names = []
        for i in lanes:
            rail = kin.rails[i]
            if not isinstance(rail, servo_axis.ServoRail):
                raise gcmd.error(
                    "axis %s is driven by non-servo rail %r"
                    % (axis, rail.get_name())
                )
            names.append(rail.get_motor_name())
        return names

    def _strokes(self, axis, start, end, speed, accel, iterations, dwell):
        if end <= start:
            raise self.gcode.error(
                "END=%.1f must exceed START=%.1f" % (end, start)
            )
        reach = speed * speed / accel
        if reach > (end - start):
            raise self.gcode.error(
                "stroke %.0fmm too short to reach %.0fmm/s at %.0fmm/s^2 "
                "(needs %.1fmm)" % (end - start, speed, accel, reach)
            )
        feed = int(speed * 60)
        lines = ["SET_VELOCITY_LIMIT ACCEL=%.0f" % (accel,), "G90"]
        for _ in range(iterations):
            lines += [
                "G1 %s%.3f F%d" % (axis, end, feed),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
                "G1 %s%.3f F%d" % (axis, start, feed),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
            ]
        self.gcode.run_script_from_command("\n".join(lines))

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
        "for any tuning change. Params AXIS START END SPEED ACCEL ITERATIONS "
        "DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_TRACKING(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        start, end = self._axis_bounds(gcmd, axis)
        speed = gcmd.get_float("SPEED", 100.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 3000.0, above=0.0)
        iterations = gcmd.get_int("ITERATIONS", 3, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "track")
        self._prep(axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START AXIS=%s NAME=%s" % (axis, name)
        )
        self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()
        self._run(gcmd, "servo_capture.py", ["--name", name], 60.0)

    cmd_SERVO_MEASURE_INERTIA_help = (
        "Excitation grid for the inertia/friction fit (servo-ident). Params "
        "AXIS START END ACCELS SPEEDS ITERATIONS DWELL_MS NAME"
    )

    def cmd_SERVO_MEASURE_INERTIA(self, gcmd):
        self._measure_inertia(gcmd, gcmd.get("NAME", "ident"))

    def _measure_inertia(self, gcmd, name):
        axis = gcmd.get("AXIS", "X").upper()
        start, end = self._axis_bounds(gcmd, axis)
        accels, speeds, iterations, dwell = self._grid(gcmd)
        self._prep(axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START AXIS=%s NAME=%s" % (axis, name)
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

    def _measure_inertia_corexy(self, gcmd, name):
        servos = gcmd.get("SERVOS", ",".join(self.servos))
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
        self._prep(axis, dwell)
        self.gcode.run_script_from_command(
            "SERVO_CAPTURE_START AXIS=%s NAME=%s" % (axis, name)
        )
        self._strokes(axis, start, end, speed, accel, iterations, dwell)
        self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        self._restore()

    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward - runs the inertia "
        "excitation grid, fits mass/viscous/coulomb, and writes a timestamped "
        "profile. Optional TORQUE_NM + INERTIA_KGM2 add the C00.06 "
        "recommendation. Params as SERVO_MEASURE_INERTIA plus TORQUE_NM "
        "INERTIA_KGM2"
    )

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd):
        name = gcmd.get("NAME", "ident")
        torque, inertia = self._motor(gcmd, required=False)
        self._measure_inertia(gcmd, name)
        args = ["--name", name]
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
        "the two-drive X+Y excitation grid, fits the coupled mass matrix, "
        "and writes a timestamped profile. Optional TORQUE_NM + INERTIA_KGM2 "
        "add the C00.06 recommendation. Params as "
        "SERVO_MEASURE_INERTIA_COREXY plus TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_FIT_DYNAMICS_COREXY(self, gcmd):
        name = gcmd.get("NAME", "ident")
        torque, inertia = self._motor(gcmd, required=False)
        self._measure_inertia_corexy(gcmd, name)
        args = ["--name", name, "--structure", "corexy"]
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
        self._measure_inertia(gcmd, name)
        self._run(
            gcmd,
            "servo_fit_dynamics.py",
            [
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
        "Step 2 of CoreXY servo tuning - runs the two-drive X+Y excitation "
        "grid, fits the coupled mass matrix, and prints C00.06 for both "
        "directions. TORQUE_NM and INERTIA_KGM2 required (config or param). "
        "Params as SERVO_MEASURE_INERTIA_COREXY plus TORQUE_NM INERTIA_KGM2"
    )

    def cmd_SERVO_CALIBRATE_INERTIA_RATIO_COREXY(self, gcmd):
        name = gcmd.get("NAME", "inertia")
        torque, inertia = self._motor(gcmd, required=True)
        self._measure_inertia_corexy(gcmd, name)
        self._run(
            gcmd,
            "servo_fit_dynamics.py",
            [
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
        "Reverts to the lowest gains afterwards. Params SPEED_GAINS AXIS "
        "START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO (comma list "
        "override)"
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
            self._strokes(axis, start, end, speed, accel, iterations, dwell)
            self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        sg0 = sgains[0]
        gcmd.respond_info(
            "sweep done - reverting to first step (%.1f Hz) until you apply "
            "the recommendation" % (sg0 / 10.0,)
        )
        self._write_gains(servos, round(sg0 * 1.6), sg0, round(1250000 / sg0))
        self._restore()
        self._run(
            gcmd,
            "servo_gain_report.py",
            [
                "--tag",
                tag,
                "--steps",
                ",".join(step_names),
                "--axis",
                axis,
            ],
            120.0,
        )

    cmd_SERVO_SWEEP_INERTIA_help = (
        "Empirical inertia sweep, gain-sweep style. Writes each C00.06 ratio "
        "in RATIOS (percent, comma list), one capture per step, renders a "
        "comparison PNG. Reverts to the lowest ratio afterwards. Params "
        "RATIOS AXIS START END SPEED ACCEL ITERATIONS DWELL_MS TAG SERVO"
    )

    def cmd_SERVO_SWEEP_INERTIA(self, gcmd):
        axis = gcmd.get("AXIS", "X").upper()
        servo = self._servo(gcmd)
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
        self._prep(axis, dwell)
        step_names = []
        for i, rv in enumerate(ratios):
            step_names.append("%s_r%d" % (tag, rv))
            gcmd.respond_info(
                "inertia step %d/%d: C00.06 ratio %d%%"
                % (i + 1, len(ratios), rv)
            )
            self.gcode.run_script_from_command(
                "\n".join(
                    [
                        "SERVO_PARAM SERVO=%s SET=0x2000.0x07 VALUE=%d TYPE=u16"
                        % (servo, rv),
                        "SERVO_CAPTURE_START SERVO=%s NAME=%s"
                        % (servo, step_names[-1]),
                    ]
                )
            )
            self._strokes(axis, start, end, speed, accel, iterations, dwell)
            self.gcode.run_script_from_command("SERVO_CAPTURE_STOP")
        rv0 = ratios[0]
        gcmd.respond_info(
            "sweep done - reverting to first ratio (%d%%) until you apply your "
            "choice with SERVO_SET_INERTIA_RATIO" % (rv0,)
        )
        self.gcode.run_script_from_command(
            "SERVO_PARAM SERVO=%s SET=0x2000.0x07 VALUE=%d TYPE=u16"
            % (servo, rv0)
        )
        self._restore()
        self._run(
            gcmd,
            "servo_inertia_report.py",
            ["--tag", tag, "--steps", ",".join(step_names)],
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
