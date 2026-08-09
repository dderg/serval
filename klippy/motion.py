import logging
import math
import os
import signal
import time

from . import (
    configfile,
    engine_wait,
    motion_debug,
    motion_kinematics,
    motion_setup,
)
from .extras import servo_axis
from .kinematics import extruder

REACTOR_YIELD_INTERVAL = 0.020

_AXIS_UNIT_DELTAS = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


class EngineWakeup:
    """Parks waiters on the engine's readiness fd. The fd becomes readable
    when input-channel space frees after a refused submit and on every fence
    resolution, so parked submits and fence pollers resume the moment the
    engine has something for them instead of on a poll interval."""

    def __init__(self, reactor, fd, on_wake):
        self.reactor = reactor
        self.fd = fd
        self.on_wake = on_wake
        self.waiters = []
        reactor.register_fd(fd, self._on_readable)

    def _on_readable(self, eventtime):
        try:
            os.read(self.fd, 4096)
        except OSError:
            pass
        waiters = self.waiters
        self.waiters = []
        for completion in waiters:
            completion.complete(None)
        self.on_wake()

    def park(self, max_wait_s):
        completion = self.reactor.completion()
        self.waiters.append(completion)
        completion.wait(self.reactor.monotonic() + max_wait_s)
        if completion in self.waiters:
            self.waiters.remove(completion)


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
        self._engine_wakeup = None
        self._load_motion_config(config)
        self.print_stall = 0
        _deprecated_buffer_time_high = config.getfloat(
            "buffer_time_high", 2.0, above=0.0
        )
        _deprecated_buffer_time_low = config.getfloat(
            "buffer_time_low", 1.0, minval=0.0
        )
        self._drip_active = False
        self._clock_sync_confirmed = False
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
        motion_debug.MotionDebugCommands(self, gcode)

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
        from .extras import resonance_buzz

        return resonance_buzz.submit_buzz(
            self,
            axis_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        )

    def set_extruder(self, extruder, extrude_pos):
        self.extruder = extruder
        self.commanded_pos[3] = extrude_pos

    def get_extruder(self):
        return self.extruder

    def get_kinematics(self):
        return self.kin

    def get_active_rails_for_axis(self, axis):
        if axis not in _AXIS_UNIT_DELTAS:
            raise ValueError("Invalid axis %s" % (axis,))
        dx, dy, dz = _AXIS_UNIT_DELTAS[axis]
        return self.kin.active_rails(dx, dy, dz)

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

    CLOCK_SYNC_TIMEOUT = 60.0

    def _await_clock_sync(self):
        if self._clock_sync_confirmed:
            return
        if self.mcu is None or self.mcu.is_fileoutput():
            self._clock_sync_confirmed = True
            return

        def pending_mcus():
            return [
                m
                for m in self.all_mcus
                if not m.non_critical_disconnected
                and not m.get_clocksync().is_synced()
            ]

        pending = pending_mcus()
        if pending:
            self.printer.lookup_object("gcode").respond_info(
                "Waiting for MCU clock synchronization..."
            )
            deadline = self.reactor.monotonic() + self.CLOCK_SYNC_TIMEOUT
            while pending:
                if self.reactor.monotonic() > deadline:
                    raise self.printer.command_error(
                        "MCU clock synchronization did not converge"
                        " within %.0fs (mcu: %s)"
                        % (
                            self.CLOCK_SYNC_TIMEOUT,
                            ", ".join(m.get_name() for m in pending),
                        )
                    )
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                pending = pending_mcus()
        self._clock_sync_confirmed = True

    def move(self, newpos, speed):
        self._await_clock_sync()
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
        self._await_clock_sync()
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
            park=self._engine_wakeup.park if self._engine_wakeup else None,
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
                    return eventtime + engine_wait.PARK_FALLBACK_S
            fence_print_time = self.engine.fence_print_time_poll(
                entry[0], self.mcu.get_engine_handle()
            )
            if fence_print_time is None:
                return eventtime + engine_wait.PARK_FALLBACK_S
            self._lookahead_fences.pop(0)
            entry[1](max(fence_print_time, self._schedule_floor()))
        return self.reactor.NEVER

    def _register_engine_wakeup(self):
        self._engine_wakeup = EngineWakeup(
            self.reactor,
            self.engine.feed_wakeup_fd(),
            self._kick_lookahead_fences,
        )

    def _kick_lookahead_fences(self):
        if self._lookahead_fences:
            self.reactor.update_timer(
                self._lookahead_fence_timer, self.reactor.NOW
            )

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
        self._yield_to_reactor_if_due(self.reactor.monotonic())
        if submit(*args):
            return
        engine_wait.wait_for(
            self.printer,
            lambda: submit(*args) or None,
            "motion pipe space",
            engine_wait.UNBOUNDED,
            park=self._engine_wakeup.park if self._engine_wakeup else None,
        )
        self._last_reactor_yield = self.reactor.monotonic()

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
