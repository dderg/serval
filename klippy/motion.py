import logging
import math
import os
import signal
import time

from . import (
    configfile,
    engine_wait,
    motion_kinematics,
    motion_setup,
    structured_log,
)
from .extras import servo_axis
from .kinematics import extruder

REACTOR_YIELD_INTERVAL = 0.020


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


class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.end_pos = tuple(end_pos)
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        if move_d < 0.000000001:
            self.end_pos = (
                start_pos[0],
                start_pos[1],
                start_pos[2],
                end_pos[3],
            )
            axes_d[0] = axes_d[1] = axes_d[2] = 0.0
            self.move_d = move_d = abs(axes_d[3])
            velocity = speed
            self.is_kinematic_move = False
        self.min_move_t = move_d / velocity
        self.max_cruise_v2 = velocity**2

    def limit_speed(self, speed):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed

    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return self.toolhead.printer.command_error(m)


class Motion:
    def __init__(self, config):
        printer = config.get_printer()
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.engine = printer.lookup_object("motion_engine", None)
        if self.engine is None:
            from . import motion_engine

            self.engine = motion_engine._StubEngine()
        self.motion_lead = self.engine.motion_lead_secs()
        if self.motion_lead is None:
            self.motion_lead = 0.25
        self._motor_bindings = {}
        self.all_mcus = [m for n, m in printer.lookup_objects(module="mcu")]
        self.mcu = self.all_mcus[0]
        self.need_flush_time = 0.0
        self.do_kick_flush_timer = True
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self._lookahead_fences = []
        self._lookahead_fence_timer = self.reactor.register_timer(
            self._lookahead_fence_handler
        )
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]
        self._planner_ready = False
        self._load_motion_config(config)
        self.print_stall = 0
        _deprecated_buffer_time_high = config.getfloat(
            "buffer_time_high", 2.0, above=0.0
        )
        _deprecated_buffer_time_low = config.getfloat(
            "buffer_time_low", 1.0, minval=0.0
        )
        self._drip_active = False
        self._last_reactor_yield = 0.0
        gcode = printer.lookup_object("gcode")
        self.Coord = gcode.Coord
        self.extruder = extruder.DummyExtruder(printer)
        self._build_follower_steppers(config)
        self.kin = self._load_kinematics(config)
        if (
            config.has_section("dual_carriage")
            and not self.kin.supports_dual_carriage
        ):
            raise config.error(
                "dual_carriage not compatible with '%s' kinematics system"
                % (self.kin.kind,)
            )

        gcode.register_command("G4", self.cmd_G4)
        gcode.register_command("M400", self.cmd_M400)
        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            self.cmd_SET_VELOCITY_LIMIT,
            desc=self.cmd_SET_VELOCITY_LIMIT_help,
        )
        gcode.register_command(
            "RESET_VELOCITY_LIMIT",
            self.cmd_RESET_VELOCITY_LIMIT,
            desc=self.cmd_RESET_VELOCITY_LIMIT_help,
        )
        gcode.register_command(
            "SET_POST_PROCESSOR",
            self.cmd_SET_POST_PROCESSOR,
            desc=self.cmd_SET_POST_PROCESSOR_help,
        )
        gcode.register_command("M204", self.cmd_M204)
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
            "DIAG_DUMP",
            self.cmd_DIAG_DUMP,
            desc="Emit the live MCU diag snapshot (cause discriminators + "
            "event ring) to the structured-log store; no reset required",
        )

        for module_name in (
            "gcode_move",
            "homing",
            "idle_timeout",
            "statistics",
            "manual_probe",
            "tuning_tower",
            "garbage_collection",
            "motion_report",
        ):
            printer.load_object(config, module_name)

        printer.register_event_handler("klippy:connect", self._init_planner)
        printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

        def _sigterm_handler(signum, frame):
            self.printer.request_exit("exit")

        signal.signal(signal.SIGTERM, _sigterm_handler)

        logging.info("Motion: config phase complete")

    def _handle_disconnect(self):
        logging.info("Motion: _handle_disconnect called")
        if self.engine is not None:
            logging.info("Motion: calling engine.shutdown()")
            self.engine.shutdown()
            logging.info("Motion: engine.shutdown() returned")

    def _load_kinematics(self, config):
        return motion_kinematics.load_kinematics(config, self)

    def get_position(self):
        return list(self.commanded_pos)

    def set_position(self, newpos, homing_axes=()):
        self.flush_step_generation()
        self.commanded_pos[:] = newpos
        self.kin.set_position(newpos, homing_axes)
        self.printer.send_event("toolhead:set_position")

    def rebase_gcode_z(self, z):
        """Adopt the gcode Z the engine rebased to across a bed-mesh swap.
        The physical position is unchanged (the engine kept machine Z
        invariant); only the gcode-space name for it moved."""
        self.commanded_pos[2] = z
        self.printer.send_event("toolhead:set_position")

    def manual_move(self, coord, speed):
        curpos = list(self.commanded_pos)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)
        self.printer.send_event("toolhead:manual_move")

    def submit_nudge(self, mcu_id, axis_idx, motor_idx, delta_mm, speed, accel):
        motor_mask = 1 << motor_idx
        return self.engine.submit_nudge(
            mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
        )

    def submit_resonance_buzz(
        self,
        axis_mask,
        sign_mask,
        freq_start_millihz,
        freq_end_millihz,
        amplitude_nm,
        duration_ms,
        ramp_ms,
    ):
        from .extras.resonance_buzz import buzz_axis_to_motor_mask

        stepper_mask = axis_mask
        sent = False
        servo_targets = {}
        if self.kin is not None:
            for lane_idx, _axis_name, _motors in self.kin.lanes():
                rail = self.kin.rails[lane_idx]
                if not isinstance(rail, servo_axis.ServoRail):
                    continue
                rail_mask, _ = buzz_axis_to_motor_mask(rail.axis, False)
                if not (axis_mask & rail_mask):
                    continue
                stepper_mask &= ~rail_mask
                node = self.printer.lookup_object(
                    "ethercat_node " + rail.get_node_name(), None
                )
                handle = node.get_engine_handle() if node is not None else None
                if handle is None:
                    raise self.printer.command_error(
                        "RESONANCE_BUZZ: servo axis %s has no live EtherCAT "
                        "engine handle" % rail.axis
                    )
                slot_masks = servo_targets.setdefault(handle, [0, 0])
                for motor in rail.get_motors():
                    slot = node.get_slot_for_motor(motor.get_motor_name())
                    if slot is None:
                        raise self.printer.command_error(
                            "RESONANCE_BUZZ: servo motor %s has no claim "
                            "slot on ethercat node %s"
                            % (motor.get_motor_name(), rail.get_node_name())
                        )
                    slot_bit = 1 << slot
                    if slot_masks[0] & slot_bit:
                        raise self.printer.command_error(
                            "RESONANCE_BUZZ: servo motor %s maps to EtherCAT "
                            "slot %d which is already claimed by another "
                            "buzzed axis on the same node"
                            % (motor.get_motor_name(), slot)
                        )
                    slot_masks[0] |= slot_bit
                    if sign_mask & rail_mask:
                        slot_masks[1] |= slot_bit
        for handle, (slot_mask, slot_sign_mask) in servo_targets.items():
            self.engine.resonance_buzz(
                handle,
                slot_mask,
                slot_sign_mask,
                freq_start_millihz,
                freq_end_millihz,
                amplitude_nm,
                duration_ms,
                ramp_ms,
            )
            sent = True
        if stepper_mask:
            stepper_sent = False
            for mcu_obj in self._engine_mcus():
                try:
                    cmd = mcu_obj.lookup_command(
                        "kalico_resonance_buzz axis_mask=%c sign_mask=%c"
                        " freq_start_millihz=%u freq_end_millihz=%u amplitude_nm=%u"
                        " duration_ms=%u ramp_ms=%u"
                    )
                except Exception:
                    continue
                cmd.send(
                    [
                        stepper_mask,
                        sign_mask,
                        freq_start_millihz,
                        freq_end_millihz,
                        amplitude_nm,
                        duration_ms,
                        ramp_ms,
                    ]
                )
                stepper_sent = True
            if not stepper_sent:
                raise self.printer.command_error(
                    "No engine MCU advertises kalico_resonance_buzz; rebuild and "
                    "reflash MCU firmware with CONFIG_RUNTIME=y"
                )
            sent = True
        if not sent:
            raise self.printer.command_error(
                "RESONANCE_BUZZ: no target engine for axis_mask=0x%02x"
                % axis_mask
            )

    def set_extruder(self, extruder, extrude_pos):
        self.extruder = extruder
        self.commanded_pos[3] = extrude_pos

    def get_extruder(self):
        return self.extruder

    def get_kinematics(self):
        return self.kin

    def get_engine(self):
        return self.engine

    def get_motor_binding(self, stepper_name):
        binding = self._motor_bindings.get(stepper_name)
        if binding is None:
            raise self.printer.config_error(
                "Unknown motor '%s'; bound motors: %s"
                % (stepper_name, ", ".join(sorted(self._motor_bindings)))
            )
        return binding

    def get_max_axis_accel(self, axis_idx):
        axis_name = self._declared_axis_order()[axis_idx]
        if axis_name == "z":
            return min(self.max_accel, self.max_z_accel)
        return self.max_accel

    def _effective_limits(self):
        if self._planner_ready:
            return self.engine.effective_limits()
        return (
            self._max_velocity,
            self._max_accel,
            self._corner_deviation,
        )

    @property
    def max_velocity(self):
        return self._effective_limits()[0]

    @property
    def max_accel(self):
        return self._effective_limits()[1]

    @property
    def corner_deviation(self):
        return self._effective_limits()[2]

    @property
    def square_corner_velocity(self):
        _velocity, accel, corner_deviation = self._effective_limits()
        return motion_setup.scv_from_corner_deviation(corner_deviation, accel)

    def get_max_velocity(self):
        velocity, accel, _corner_deviation = self._effective_limits()
        return velocity, accel

    def get_status(self, eventtime):
        print_time = (
            self.engine.frontier_print_time(self.mcu.get_engine_handle())
            if self._planner_ready
            else 0.0
        )
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        velocity, accel, corner_deviation = self._effective_limits()
        res = dict(self.kin.get_status(eventtime))
        res.update(
            {
                "print_time": print_time,
                "stalls": self.print_stall,
                "estimated_print_time": estimated_print_time,
                "extruder": self.extruder.get_name(),
                "position": self.Coord(*self.commanded_pos),
                "max_velocity": velocity,
                "max_accel": accel,
                "minimum_cruise_ratio": self.min_cruise_ratio,
                "square_corner_velocity": motion_setup.scv_from_corner_deviation(
                    corner_deviation, accel
                ),
                "corner_deviation": corner_deviation,
            }
        )
        return res

    def cmd_G4(self, gcmd):
        delay = gcmd.get_float("P", 0.0, minval=0.0) / 1000.0
        self.dwell(delay)

    def cmd_M204(self, gcmd):
        accel = gcmd.get_float("S", None, above=0.0)
        if accel is None:
            p = gcmd.get_float("P", None, above=0.0)
            t = gcmd.get_float("T", None, above=0.0)
            if p is None or t is None:
                gcmd.respond_info(
                    'Invalid M204 command "%s"' % (gcmd.get_commandline(),)
                )
                return
            accel = min(p, t)
        self.set_accel(accel)

    def resync_parked_servos(self):
        dirty = self.kin.parked_dirty_axes()
        if not dirty:
            return
        measured = self.engine.query_motor_positions()
        missing = [a for a in dirty if "xyz"[a] not in measured]
        if missing:
            raise self.printer.command_error(
                "Cannot resync parked servo axis %s: the live motor query"
                " returned no position for it (EtherCAT endpoint down or"
                " drive faulted?) — home the axis again"
                % ", ".join("XYZ"[a] for a in missing)
            )
        newpos = list(self.commanded_pos)
        for axis in dirty:
            newpos[axis] = measured["xyz"[axis]][0]
        self.set_position(newpos)
        self.kin.clear_parked_dirty(dirty)

    def move(self, newpos, speed):
        self.resync_parked_servos()
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        dx, dy, dz, de = move.axes_d
        feedrate = move.move_d / move.min_move_t
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self.max_z_velocity)
        self._fire_active_callbacks(move.axes_d)
        self._submit_paced(self.engine.submit_move, dx, dy, dz, de, feedrate)
        self.commanded_pos[:] = move.end_pos
        self._sync_print_time()

    def move_curve(self, newpos, interior_control_points, submit, speed):
        self.resync_parked_servos()
        move = Move(self, self.commanded_pos, newpos, speed)
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        for cp in interior_control_points:
            cp_target = [cp[0], cp[1], cp[2], self.commanded_pos[3]]
            cp_move = Move(self, self.commanded_pos, cp_target, speed)
            if cp_move.move_d and cp_move.is_kinematic_move:
                self.kin.check_move(cp_move)
        endpoint_delta = [
            newpos[i] - self.commanded_pos[i] for i in (0, 1, 2, 3)
        ]
        dx, dy, dz, de = endpoint_delta
        path_speed_cap = min(speed, self.max_velocity)
        feedrate = path_speed_cap
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self.max_z_velocity)
        self._fire_active_callbacks([dx, dy, dz, de])
        self._submit_paced(submit, dx, dy, dz, de, feedrate)
        self.commanded_pos[:] = list(newpos)
        self._sync_print_time()

    def _fire_active_callbacks(self, axes_d):
        if self.kin is None:
            return False
        dx, dy, dz, de = axes_d
        owners = []
        for rail in self.kin.active_rails(dx, dy, dz):
            if isinstance(rail, servo_axis.ServoRail):
                owners.append(rail)
            else:
                owners.extend(rail.get_steppers())
        if abs(de) > 1e-9:
            owners.extend(self.follower_steppers)
        owners.extend(
            rail
            for rail in getattr(self.kin, "rails", ())
            if isinstance(rail, servo_axis.ServoRail)
        )
        fired = False
        move_time = None
        deferred = []
        try:
            for owner in owners:
                if not owner._active_callbacks:
                    continue
                cbs = owner._active_callbacks
                owner._active_callbacks = []
                if move_time is None:
                    move_time = self.get_last_move_time()
                for i, cb in enumerate(cbs):
                    cb_t0 = time.monotonic()
                    try:
                        followup = cb(move_time)
                    except Exception:
                        owner._active_callbacks = (
                            cbs[i:] + owner._active_callbacks
                        )
                        raise
                    cb_dt = time.monotonic() - cb_t0
                    if cb_dt > 0.020:
                        logging.warning(
                            "active callback for %s blocked %.1fms",
                            owner.get_name(),
                            cb_dt * 1000.0,
                        )
                    if followup is not None:
                        deferred.append(followup)
                fired = True
        finally:
            for followup in deferred:
                followup()
        return fired

    def drip_move(self, newpos, speed, drip_completion):
        if drip_completion is not None and drip_completion.test():
            return
        self._drip_active = True
        try:
            self.move(newpos, speed)
        finally:
            self._drip_active = False

    def dwell(self, delay):
        self.engine.submit_dwell(delay)
        if delay > 0.0:
            self._sync_print_time()

    def wait_moves(self):
        self._wait_mcu_drained()

    def wait_moves_and_mcu(self):
        self._wait_mcu_drained()

    def wait_until_print_time(self, print_time):
        """Block (reactor-yielding) until the MCU clock has really passed
        print_time. This is the sequencing primitive for anything scheduled
        on the MCU clock (scheduled torque changes, pin pulses): a wall
        clock pause can finish before the schedule fires and race it."""
        if self.mcu is None:
            return

        def _caught_up():
            est = self.mcu.estimated_print_time(self.reactor.monotonic())
            if est >= print_time:
                return True
            return None

        engine_wait.wait_for(
            self.printer,
            _caught_up,
            "wait_until_print_time",
            engine_wait.UNBOUNDED,
            interval_s=0.010,
        )

    def _wait_mcu_drained(self):
        engine_wait.wait_for(
            self.printer,
            lambda: self.engine.motion_drain_poll() or None,
            "wait_moves motion drain",
            engine_wait.UNBOUNDED,
            interval_s=0.010,
        )
        self.engine.motion_drain_finalize()
        # M400-after-G4 must not return before the dwell has really elapsed
        # on the MCU clock; the frontier includes queued dwells. The
        # motion_lead slice is the standing scheduling margin, not queued
        # time — waiting for it would tax every wait_moves.
        if self.mcu is not None:
            frontier = self.engine.frontier_print_time(
                self.mcu.get_engine_handle()
            )
            self.wait_until_print_time(frontier - self.motion_lead)

    def cmd_M400(self, gcmd):
        self.wait_moves_and_mcu()

    def _engine_mcus(self):
        if not hasattr(self, "_cached_engine_mcus"):
            mcus = set()
            if self.kin is not None:
                for s in self.kin.get_steppers():
                    mcus.add(s.get_mcu())
            self._cached_engine_mcus = list(mcus) if mcus else [self.mcu]
        return self._cached_engine_mcus

    def advance_flush_time(self, mq_time):
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)

    def _flush_handler(self, eventtime):
        try:
            self.do_kick_flush_timer = False
            while True:
                flush_time = self.need_flush_time
                for mcu in self.all_mcus:
                    mcu.flush_moves(flush_time, flush_time)
                if self.need_flush_time <= flush_time:
                    break
            self.do_kick_flush_timer = True
            return self.reactor.NEVER
        except Exception:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
            return self.reactor.NEVER

    def flush_step_generation(self):
        self._drain_to_mcu_execution()

    def _drain_to_mcu_execution(self):
        self.engine.wait_moves()
        frontier = self.engine.frontier_print_time(self.mcu.get_engine_handle())
        for mcu in self._engine_mcus():

            def _mcu_caught_up(mcu=mcu):
                est = mcu.estimated_print_time(self.reactor.monotonic())
                if frontier - est <= 0.0:
                    return True
                return None

            engine_wait.wait_for(
                self.printer,
                _mcu_caught_up,
                "mcu execution drain",
                engine_wait.UNBOUNDED,
                interval_s=0.010,
            )

    def _schedule_floor(self):
        now = self.reactor.monotonic()
        return (
            max(m.estimated_print_time(now) for m in self._engine_mcus())
            + self.motion_lead
        )

    def get_last_move_time(self):
        fence_print_time = self._fence_wait_blocking()
        return max(fence_print_time, self._schedule_floor())

    def _fence_wait_blocking(self):
        if self.mcu is None:
            return 0.0
        fence_id = [None]

        def _fence_print_time():
            if fence_id[0] is None:
                fence_id[0] = self.engine.fence_start(True)
            if fence_id[0] is None:
                return None
            return self.engine.fence_print_time_poll(
                fence_id[0], self.mcu.get_engine_handle()
            )

        return engine_wait.wait_for(
            self.printer,
            _fence_print_time,
            "get_last_move_time motion fence",
            engine_wait.UNBOUNDED,
            interval_s=0.002,
        )

    def register_lookahead_callback(self, callback):
        if self.mcu is None:
            callback(self.motion_lead)
            return
        fence_id = self.engine.fence_start(False)
        self._lookahead_fences.append([fence_id, callback])
        if len(self._lookahead_fences) == 1:
            self.reactor.update_timer(
                self._lookahead_fence_timer, self.reactor.NOW
            )

    def _lookahead_fence_handler(self, eventtime):
        while self._lookahead_fences:
            entry = self._lookahead_fences[0]
            if entry[0] is None:
                entry[0] = self.engine.fence_start(False)
                if entry[0] is None:
                    return eventtime + 0.020
            fence_print_time = self.engine.fence_print_time_poll(
                entry[0], self.mcu.get_engine_handle()
            )
            if fence_print_time is None:
                return eventtime + 0.020
            self._lookahead_fences.pop(0)
            entry[1](max(fence_print_time, self._schedule_floor()))
        return self.reactor.NEVER

    def _yield_to_reactor_if_due(self, now):
        if now - self._last_reactor_yield > REACTOR_YIELD_INTERVAL:
            self.reactor.pause(self.reactor.NOW)
            now = self.reactor.monotonic()
            self._last_reactor_yield = now
        return now

    def _submit_paced(self, submit, *args):
        if self.mcu is None or self._drip_active:
            if not submit(*args):
                raise self.printer.command_error(
                    "motion pipe reported full on an unpaced submit "
                    "(drip move or no mcu)"
                )
            return
        now = self._yield_to_reactor_if_due(self.reactor.monotonic())
        if submit(*args):
            return
        wait_start = now
        structured_log.event(
            "motion",
            "feed_throttle_enter",
            queued_motion=round(self.engine.queued_motion_secs(), 4),
            dispatched_lead=round(self.engine.dispatched_lead_secs(), 4),
            engine_frontier=round(self.engine.get_last_move_time(), 4),
        )
        engine_wait.wait_for(
            self.printer,
            lambda: submit(*args) or None,
            "motion pipe space",
            engine_wait.UNBOUNDED,
        )
        self._last_reactor_yield = self.reactor.monotonic()
        structured_log.event(
            "motion",
            "feed_throttle_exit",
            waited_s=round(self.reactor.monotonic() - wait_start, 4),
            queued_motion=round(self.engine.queued_motion_secs(), 4),
            engine_frontier=round(self.engine.get_last_move_time(), 4),
        )

    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        if self._planner_ready:
            print_time = self.engine.frontier_print_time(
                self.mcu.get_engine_handle()
            )
        else:
            print_time = est_print_time
        lookahead_empty = print_time <= est_print_time
        return print_time, est_print_time, lookahead_empty

    def _load_motion_config(self, config):
        """Parse and validate the motion-owned sections ([printer] limits,
        [kinematics] + [motor] topology, [axis], [post_processor],
        [extruder] caps) with the native reader — the same one the engine
        re-runs at init_planner — and record every option it consumed for
        the unused-option accounting."""
        self._motion_config_text = config.fileconfig.write_string()
        (
            (
                self._max_velocity,
                self._max_accel,
                self.max_jerk,
                self.max_z_velocity,
                self.max_z_accel,
                self._corner_deviation,
            ),
            self.axis_sections,
            self.kinematics_decl,
            consumed,
        ) = configfile._config_doc.read_motion_settings(
            self._motion_config_text
        )
        for section, option, value in consumed:
            config.access_tracking[(section.lower(), option.lower())] = value
        self.min_cruise_ratio = 0.0
        self.orig_cfg = {}

    def _build_follower_steppers(self, config):
        return motion_setup.build_follower_steppers(self, config)

    def _sync_print_time(self):
        if self.mcu is None:
            return
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        frontier = (
            self.engine.frontier_print_time(self.mcu.get_engine_handle())
            if self._planner_ready
            else 0.0
        )
        self.printer.send_event(
            "toolhead:sync_print_time",
            curtime,
            est_print_time,
            frontier,
        )

    def set_accel(self, accel):
        if accel is not None and accel > 0.0:
            self.engine.set_accel_cap(accel)

    def reset_accel(self):
        self.engine.set_accel_cap(None)

    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        accepted_legacy_noops = (
            "MINIMUM_CRUISE_RATIO",
            "ACCEL_TO_DECEL",
        )
        for legacy in accepted_legacy_noops:
            gcmd.get_float(legacy, None)
        v = gcmd.get_float("VELOCITY", None, above=0.0)
        a = gcmd.get_float("ACCEL", None, above=0.0)
        scv = gcmd.get_float("SQUARE_CORNER_VELOCITY", None, minval=0.0)
        corner_deviation = gcmd.get_float("CORNER_DEVIATION", None, minval=0.0)
        if scv is not None and corner_deviation is not None:
            raise gcmd.error(
                "SET_VELOCITY_LIMIT: SQUARE_CORNER_VELOCITY and "
                "CORNER_DEVIATION are aliases for the same corner budget; "
                "set exactly one"
            )
        if v is None and a is None and scv is None and corner_deviation is None:
            velocity, accel, deviation = self._effective_limits()
            gcmd.respond_info(
                "velocity=%s accel=%s corner_deviation=%s"
                " square_corner_velocity=%s"
                % (
                    velocity,
                    accel,
                    deviation,
                    motion_setup.scv_from_corner_deviation(deviation, accel),
                )
            )
            return
        if v is not None:
            self.engine.set_velocity_cap(v)
        if a is not None:
            self.engine.set_accel_cap(a)
        if scv is not None:
            corner_deviation = motion_setup.corner_deviation_from_scv(
                scv, self._max_accel
            )
        if corner_deviation is not None:
            try:
                self.engine.set_corner_deviation(corner_deviation)
            except ValueError as e:
                gcmd.respond_info(
                    "Warning: SET_VELOCITY_LIMIT left corner_deviation "
                    "unchanged: %s" % (e,)
                )

    cmd_SET_POST_PROCESSOR_help = (
        "Update a [post_processor] parameter; applies from the next replan"
    )

    def cmd_SET_POST_PROCESSOR(self, gcmd):
        name = gcmd.get("NAME")
        params = {
            key: value
            for key, value in gcmd.get_command_parameters().items()
            if key != "NAME"
        }
        if not params:
            raise gcmd.error(
                "SET_POST_PROCESSOR NAME=%s: provide at least one "
                "<PARAM>=<VALUE>" % name
            )
        for key, value in params.items():
            try:
                self.engine.update_post_processor(
                    name, key.lower(), float(value)
                )
            except (ValueError, RuntimeError) as e:
                raise gcmd.error(str(e))

    cmd_RESET_VELOCITY_LIMIT_help = "Reset printer velocity limits"

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        self.engine.set_velocity_cap(None)
        self.engine.set_accel_cap(None)
        self.engine.set_corner_deviation(None)

    def stats(self, eventtime):
        frontier = (
            self.engine.frontier_print_time(self.mcu.get_engine_handle())
            if self._planner_ready
            else 0.0
        )
        for m in self.all_mcus:
            if getattr(m, "non_critical_disconnected", False):
                continue
            m.check_active(frontier, eventtime)
        buffer_time = 0.0
        pump_backlog = 0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(eventtime)
            buffer_time = frontier - est
            pump_backlog = self.engine.pump_backlog()
        return (
            False,
            "print_time=%.3f buffer_time=%.3f pump_backlog=%d print_stall=%d"
            % (
                frontier,
                buffer_time,
                pump_backlog,
                self.print_stall,
            ),
        )

    def _declared_axis_order(self):
        return motion_setup.declared_axis_order(self)

    def _build_axis_to_handle(self):
        return motion_setup.build_axis_to_handle(self)

    def _derive_mcu_topology(self, axis_to_handle):
        return motion_setup.derive_mcu_topology(self, axis_to_handle)

    def _init_planner(self):
        return motion_setup.init_planner(self)

    def _follower_slots(self):
        return motion_setup.follower_slots(self)

    def _build_slot_steppers(self):
        return motion_setup.build_slot_steppers(self)

    def _configure_axes_per_mcu(self, engine_mcus):
        return motion_setup.configure_axes_per_mcu(self, engine_mcus)

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

    def cmd_MCU_SIM_MOTION_STATE(self, gcmd):
        print_time = gcmd.get_float("PRINT_TIME", None)
        t_ago = gcmd.get_float("T_AGO", None)
        if (print_time is None) == (t_ago is None):
            raise gcmd.error("specify exactly one of PRINT_TIME or T_AGO")
        if t_ago is not None:
            print_time = self.get_last_move_time() - t_ago
        if self.engine is None:
            raise gcmd.error("motion_engine not available")
        state = self.engine.motion_state_at(self.mcu, print_time=print_time)
        parts = [
            "%s: pos=%.6f vel=%.6f accel=%.6f" % (name, p, v, a)
            for name, (p, v, a) in sorted(state.items())
        ]
        gcmd.respond_info(
            "motion_state @%.6f: %s" % (print_time, " | ".join(parts))
        )

    def cmd_MCU_SIM_STEP_COUNT(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0)
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = self.mcu.get_engine_handle()
        if handle is None:
            raise gcmd.error("engine handle not set")
        try:
            resp = self.engine.engine_call(
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
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = self.mcu.get_engine_handle()
        if handle is None:
            raise gcmd.error("engine handle not set")
        try:
            resp = self.engine.engine_call(
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
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = self.mcu.get_engine_handle()
        if handle is None:
            raise gcmd.error("engine handle not set")
        try:
            resp = self.engine.engine_call(
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
        if self.mcu is None:
            raise gcmd.error("no MCU available for sim endstop set_pin")
        handle = self.mcu.get_engine_handle()
        try:
            self.engine.engine_send(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
            )
            gcmd.respond_info(
                "MCU_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (fw)"
                % (gpio, level)
            )
        except Exception as e:
            raise gcmd.error("runtime_sim_endstop_set_pin failed: %s" % e)


class ToolheadShim:
    def __init__(self, motion):
        self.motion = motion

    def register_lookahead_callback(self, callback):
        self.motion.register_lookahead_callback(callback)

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        self.motion.advance_flush_time(mq_time)

    def __getattr__(self, name):
        return getattr(self.motion, name)


def add_printer_objects(config):
    motion = Motion(config)
    printer = config.get_printer()
    printer.add_object("motion", motion)
    printer.add_object("toolhead", ToolheadShim(motion))
    extruder.add_printer_objects(config)
