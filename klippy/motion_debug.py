import os


def _open_sim_control():
    sock_dir = os.environ.get("MCU_SIM_SOCK_DIR")
    if not sock_dir:
        return None
    sock_path = os.path.join(sock_dir, "sim_control")
    if not os.path.exists(sock_path):
        return None
    try:
        from tools.sim.emulators.sim_control_client import (
            SimControlClient,
        )
    except ImportError:
        return None
    return SimControlClient(sock_path)


class MotionDebugCommands:
    def __init__(self, motion, gcode):
        self.motion = motion
        self.printer = motion.printer
        gcode.register_command(
            "MCU_SIM_STEP_COUNT",
            self.cmd_MCU_SIM_STEP_COUNT,
            desc="[sim] Query cumulative step count for a stepper OID",
        )
        gcode.register_command(
            "MCU_SIM_AXIS_STEPS",
            self.cmd_MCU_SIM_AXIS_STEPS,
            desc="[sim] Query configured steps_per_mm for an axis OID",
        )
        gcode.register_command(
            "MCU_SIM_AXIS_ACCUM",
            self.cmd_MCU_SIM_AXIS_ACCUM,
            desc="[sim] Query step accumulator for an axis OID",
        )
        gcode.register_command(
            "MCU_SIM_ENDSTOP_SET_PIN",
            self.cmd_MCU_SIM_ENDSTOP_SET_PIN,
            desc="[sim] Drive a Linux-MCU GPIO level (test fixture)",
        )
        gcode.register_command(
            "MCU_SIM_MOTION_STATE",
            self.cmd_MCU_SIM_MOTION_STATE,
            desc="[sim] Query commanded motion state at a past print_time",
        )
        gcode.register_command(
            "MCU_SIM_CONSTANT_MOVE",
            self.cmd_MCU_SIM_CONSTANT_MOVE,
            desc="[sim] Submit a constant-velocity single-motor move",
        )
        gcode.register_command(
            "MCU_SIM_ARMED_WINDOW",
            self.cmd_MCU_SIM_ARMED_WINDOW,
            desc="[sim] Report the armed piece MCU-clock window for an axis",
        )
        gcode.register_command(
            "DIAG_DUMP",
            self.cmd_DIAG_DUMP,
            desc="Emit the live MCU diag snapshot (cause discriminators + "
            "event ring) to the structured-log store; no reset required",
        )

    def cmd_DIAG_DUMP(self, gcmd):
        sent = []
        for name, mcu_obj in self.printer.lookup_objects(module="mcu"):
            try:
                cmd = mcu_obj.lookup_command("runtime_diag_dump")
            except Exception:
                continue
            cmd.send([])
            sent.append(name)
        if sent:
            gcmd.respond_info(
                "DIAG_DUMP: requested live diag from %s "
                "(see printer_data/logs/events/<mcu>.jsonl)"
                % (", ".join(sent),)
            )
        else:
            gcmd.respond_info("DIAG_DUMP: no MCU exposes runtime_diag_dump")

    def cmd_MCU_SIM_ARMED_WINDOW(self, gcmd):
        motion = self.motion
        if motion.engine is None:
            raise gcmd.error("motion_engine not available")
        mcu_name = gcmd.get("MCU")
        axis = gcmd.get_int("AXIS", minval=0)
        mcu_obj = self.printer.lookup_object("mcu " + mcu_name, None)
        if mcu_obj is None and mcu_name in ("mcu", ""):
            mcu_obj = self.printer.lookup_object("mcu")
        if mcu_obj is None:
            raise gcmd.error("unknown MCU '%s'" % mcu_name)
        handle = mcu_obj.get_engine_handle()
        if handle is None:
            raise gcmd.error("MCU '%s' has no engine handle" % mcu_name)
        resp = motion.engine.engine_call(
            handle,
            "runtime_sim_axis_window axis=%d" % axis,
            "runtime_sim_axis_window_response",
            timeout_s=5.0,
        )
        start = resp["start_lo"] | (resp["start_hi"] << 32)
        end = resp["end_lo"] | (resp["end_hi"] << 32)
        gcmd.respond_info(
            "MCU_SIM_ARMED_WINDOW mcu=%s axis=%d armed=%d occupancy=%d "
            "start=%d end=%d"
            % (mcu_name, axis, resp["armed"], resp["occupancy"], start, end)
        )

    def cmd_MCU_SIM_CONSTANT_MOVE(self, gcmd):
        name = gcmd.get("STEPPER")
        distance = gcmd.get_float("DISTANCE")
        velocity = gcmd.get_float("VELOCITY", above=0.0)
        toolhead = self.printer.lookup_object("toolhead")
        mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(name)
        self.motion.submit_nudge(
            mcu_id, axis_idx, motor_idx, distance, velocity, 0.0
        )
        gcmd.respond_info(
            "MCU_SIM_CONSTANT_MOVE stepper=%s distance=%.9f velocity=%.9f"
            % (name, distance, velocity)
        )

    def cmd_MCU_SIM_MOTION_STATE(self, gcmd):
        motion = self.motion
        print_time = gcmd.get_float("PRINT_TIME", None)
        t_ago = gcmd.get_float("T_AGO", None)
        if (print_time is None) == (t_ago is None):
            raise gcmd.error("specify exactly one of PRINT_TIME or T_AGO")
        if t_ago is not None:
            print_time = motion.get_last_move_time() - t_ago
        if motion.engine is None:
            raise gcmd.error("motion_engine not available")
        state = motion.engine.motion_state_at(motion.mcu, print_time=print_time)
        parts = [
            "%s: pos=%.6f vel=%.6f accel=%.6f" % (name, p, v, a)
            for name, (p, v, a) in sorted(state.items())
        ]
        gcmd.respond_info(
            "motion_state @%.6f: %s" % (print_time, " | ".join(parts))
        )

    def _engine_handle(self, gcmd):
        motion = self.motion
        if motion.mcu is None:
            raise gcmd.error("mcu not available")
        handle = motion.mcu.get_engine_handle()
        if handle is None:
            raise gcmd.error("engine handle not set")
        return handle

    def cmd_MCU_SIM_STEP_COUNT(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0)
        handle = self._engine_handle(gcmd)
        try:
            resp = self.motion.engine.engine_call(
                handle,
                "runtime_sim_stepper_count_query oid=%d" % oid,
                "runtime_sim_stepper_count_response",
                timeout_s=5.0,
            )
            count = resp.get("count", 0)
            gcmd.respond_info(
                "[engine-async] MCU_SIM_STEP_COUNT oid=%d count=%d"
                % (oid, count)
            )
        except Exception as e:
            raise gcmd.error("step count query failed: %s" % e)

    def cmd_MCU_SIM_AXIS_STEPS(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        handle = self._engine_handle(gcmd)
        try:
            resp = self.motion.engine.engine_call(
                handle,
                "runtime_sim_axis_steps_query oid=%d" % oid,
                "runtime_sim_axis_steps_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli_spm", 0)
            gcmd.respond_info(
                "[engine-async] MCU_SIM_AXIS_STEPS oid=%d "
                "steps_per_mm=%.3f" % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis steps query failed: %s" % e)

    def cmd_MCU_SIM_AXIS_ACCUM(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        handle = self._engine_handle(gcmd)
        try:
            resp = self.motion.engine.engine_call(
                handle,
                "runtime_sim_axis_accum_query oid=%d" % oid,
                "runtime_sim_axis_accum_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli", 0)
            gcmd.respond_info(
                "[engine-async] MCU_SIM_AXIS_ACCUM oid=%d accum=%.3f"
                % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis accum query failed: %s" % e)

    def cmd_MCU_SIM_ENDSTOP_SET_PIN(self, gcmd):
        motion = self.motion
        gpio = gcmd.get_int("GPIO", minval=0, maxval=0xFFFF)
        level = gcmd.get_int("LEVEL", minval=0, maxval=1)
        client = _open_sim_control()
        if client is not None:
            MAX_GPIO_LINES = 288
            chip_id = gpio // MAX_GPIO_LINES
            line = gpio % MAX_GPIO_LINES
            try:
                with client:
                    client.set_gpio_input(
                        chip=chip_id,
                        line=line,
                        value=level,
                    )
                gcmd.respond_info(
                    "MCU_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (shim)"
                    % (gpio, level)
                )
                return
            except Exception as e:
                raise gcmd.error("set_gpio_input failed: %s" % e)
        if motion.mcu is None:
            raise gcmd.error("no MCU available for sim endstop set_pin")
        handle = motion.mcu.get_engine_handle()
        try:
            motion.engine.engine_send(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
            )
            gcmd.respond_info(
                "MCU_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (fw)"
                % (gpio, level)
            )
        except Exception as e:
            raise gcmd.error("runtime_sim_endstop_set_pin failed: %s" % e)
