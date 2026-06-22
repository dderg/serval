import logging
import math
import os
import signal
import struct
from collections import defaultdict

from . import motion_kinematics, stepper
from .arc_fit_config import arc_fit_from_config
from .extras import servo_axis
from .kinematics import extruder

DRAIN_TIMEOUT = 60.0
_LEGACY_STEPPER_AXES = frozenset("xyzab")
_LEGACY_SERVO_SECTIONS = ("servo_x", "servo_y", "servo_z")


def _is_legacy_stepper_role_section(name):
    if not name.startswith("stepper_"):
        return False
    suffix = name[len("stepper_") :]
    if not suffix or suffix[0] not in _LEGACY_STEPPER_AXES:
        return False
    return suffix[1:] == "" or suffix[1:].isdigit()


def reject_legacy_role_sections(config):
    for sc in config.get_prefix_sections("stepper_"):
        if _is_legacy_stepper_role_section(sc.get_name()):
            raise config.error(
                "role-encoding motor sections are not supported: name the "
                "motor freely (e.g. [motor a]) and assign it in [kinematics] "
                "role lists / [axis <name>] motors:"
            )
    for name in _LEGACY_SERVO_SECTIONS:
        if config.has_section(name):
            raise config.error(
                "role-encoding servo sections are not supported: declare a "
                "[<motor>] section with 'drive: servo' and assign it in "
                "[kinematics]"
            )


def _open_sim_control():
    sock_dir = os.environ.get("MCU_SIM_SOCK_DIR")
    if not sock_dir:
        return None
    sock_path = os.path.join(sock_dir, "sim_control")
    if not os.path.exists(sock_path):
        return None
    try:
        from tools.sim_klippy.orchestrator.sim_control_client import (
            SimControlClient,
        )
    except ImportError:
        return None
    return SimControlClient(sock_path)


class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
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
            inv_move_d = 0.0
            if move_d:
                inv_move_d = 1.0 / move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity
        self.max_cruise_v2 = velocity**2

    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)

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
        self._mcu_pending_end_time = 0.0
        self.motion_lead = self.engine.motion_lead_secs()
        if self.motion_lead is None:
            self.motion_lead = 0.25
        self._motor_bindings = {}
        self.all_mcus = [m for n, m in printer.lookup_objects(module="mcu")]
        self.mcu = self.all_mcus[0]
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]
        self._planner_ready = False
        self._read_limits(config)
        self._read_axes(config)
        self._read_post_processors(config)
        self._read_arc_fit(config)
        self.print_time = 0.0
        self.print_stall = 0
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
        from .extras.resonance_tester import buzz_axis_to_motor_mask

        stepper_mask = axis_mask
        sent = False
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
                self.engine.resonance_buzz(
                    handle,
                    1,
                    1 if (sign_mask & rail_mask) else 0,
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
        accels = [
            a
            for _name, axes, _v, a, _j in self.limit_sections
            if a is not None and axis_name in axes
        ]
        return min(accels) if accels else self.max_accel

    def _effective_limits(self):
        if self._planner_ready:
            return self.engine.effective_limits()
        return (
            self._max_velocity,
            self._max_accel,
            self._square_corner_velocity,
        )

    @property
    def max_velocity(self):
        return self._effective_limits()[0]

    @property
    def max_accel(self):
        return self._effective_limits()[1]

    @property
    def square_corner_velocity(self):
        return self._effective_limits()[2]

    def get_max_velocity(self):
        velocity, accel, _scv = self._effective_limits()
        return velocity, accel

    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        velocity, accel, scv = self._effective_limits()
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
                "square_corner_velocity": scv,
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
        logging.info(
            "[engine-trace] move: newpos=%s speed=%s dx=%.4f dy=%.4f "
            "dz=%.4f de=%.4f feedrate=%.4f",
            list(newpos),
            speed,
            dx,
            dy,
            dz,
            de,
            feedrate,
        )
        self._fire_active_callbacks(move.axes_d)
        engine_lmt_before = self.engine.get_last_move_time()
        self.engine.submit_move(dx, dy, dz, de, feedrate)
        self._bump_pending_end_time(
            self.engine.get_last_move_time() - engine_lmt_before
        )
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
        engine_lmt_before = self.engine.get_last_move_time()
        submit(dx, dy, dz, de, feedrate)
        self._bump_pending_end_time(
            self.engine.get_last_move_time() - engine_lmt_before
        )
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
        for owner in owners:
            if not owner._active_callbacks:
                continue
            cbs = owner._active_callbacks
            owner._active_callbacks = []
            if move_time is None:
                move_time = self.get_last_move_time()
            for cb in cbs:
                cb(move_time)
            fired = True
        return fired

    def drip_move(self, newpos, speed, drip_completion):
        if drip_completion is not None and drip_completion.test():
            return
        self.move(newpos, speed)

    def dwell(self, delay):
        self.engine.submit_dwell(delay)
        if delay > 0.0:
            self._bump_pending_end_time(delay)
            self._sync_print_time()

    def wait_moves(self):
        self._drain_to_mcu_execution()

    def wait_moves_and_mcu(self):
        deadline = self.reactor.monotonic() + DRAIN_TIMEOUT
        while not self.engine.motion_drain_poll():
            now = self.reactor.monotonic()
            if now >= deadline:
                raise self.printer.command_error(
                    "wait_moves_and_mcu: motion drain timed out after %.0fs"
                    % (DRAIN_TIMEOUT,)
                )
            self.reactor.pause(now + 0.010)
        self.engine.motion_drain_finalize()
        self._ground_pending_end_time_after_engine_drain()

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

    def flush_step_generation(self):
        self._drain_to_mcu_execution()

    def _drain_to_mcu_execution(self):
        self.engine.wait_moves()
        if self._mcu_pending_end_time > 0.0:
            for mcu in self._engine_mcus():
                while True:
                    est = mcu.estimated_print_time(self.reactor.monotonic())
                    remaining = self._mcu_pending_end_time - est
                    if remaining <= 0.0:
                        break
                    self.reactor.pause(
                        self.reactor.monotonic() + remaining + 0.010
                    )
        self._ground_pending_end_time_after_engine_drain()

    def get_last_move_time(self):
        est = 0.0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(self.reactor.monotonic())
        floor = est + self.motion_lead
        if self._mcu_pending_end_time > est:
            return max(self._mcu_pending_end_time, floor)
        return floor

    def _ground_pending_end_time_after_engine_drain(self):
        if self.mcu is None:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        command_time = est + self.motion_lead
        if self._mcu_pending_end_time > command_time:
            self._mcu_pending_end_time = command_time

    def _bump_pending_end_time(self, duration_added):
        if self.mcu is None or duration_added <= 0.0:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        base = max(self._mcu_pending_end_time, est)
        self._mcu_pending_end_time = base + duration_added

    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        print_time = self._mcu_pending_end_time
        lookahead_empty = print_time <= est_print_time
        return print_time, est_print_time, lookahead_empty

    UNSUPPORTED_LIMIT_KEYS = (
        "max_accel_to_decel",
        "minimum_cruise_ratio",
    )

    def _read_axes(self, config):
        reject_legacy_role_sections(config)
        if config.has_section("firmware_retraction"):
            raise config.error(
                "[firmware_retraction] is not supported: it presupposes an "
                "extruder concept the motion system does not have"
            )
        if config.has_section("input_shaper"):
            raise config.error(
                "[input_shaper] is not supported: declare [post_processor "
                "<name>] sections and reference them from [axis] "
                "post_processors"
            )
        self.axis_sections = []
        for sc in config.get_prefix_sections("axis "):
            name = sc.get_name().split(None, 1)[1]
            follows = [a.strip().lower() for a in sc.getlist("follows", [])]
            motors = [m.strip() for m in sc.getlist("motors", [])]
            post_processors = [
                p.strip() for p in sc.getlist("post_processors", [])
            ]
            self.axis_sections.append((name, follows, motors, post_processors))
        declared = {name for name, _, _, _ in self.axis_sections}
        for _, axes, _, _, _ in self.limit_sections:
            for a in axes:
                if a not in declared:
                    raise config.error(
                        "[limit] references undeclared axis '%s' "
                        "(declare [axis %s])" % (a, a)
                    )

    def _build_follower_steppers(self, config):
        self.follower_steppers = []
        claimed = set(motion_kinematics.read_claimed_axes(config))
        for name, _follows, motors, _pp in self.axis_sections:
            if name in claimed or not motors:
                continue
            for motor_name in motors:
                motor_section, drive = motion_kinematics.resolve_motor_section(
                    config, motor_name, "[axis %s] motors" % name
                )
                if drive != "stepper":
                    raise config.error(
                        "[axis %s] motors references '%s' with drive: %s — "
                        "follower axes support stepper motors only"
                        % (name, motor_name, drive)
                    )
                self.follower_steppers.append(
                    stepper.PrinterStepper(
                        motor_section,
                        name=motion_kinematics.motor_short_name(motor_section),
                    )
                )

    def _read_post_processors(self, config):
        self.post_processor_sections = []
        for sc in config.get_prefix_sections("post_processor "):
            name = sc.get_name().split(None, 1)[1]
            ty = sc.get("type")
            params = [
                (opt, sc.getfloat(opt))
                for opt in sc.get_prefix_options("")
                if opt != "type"
            ]
            self.post_processor_sections.append((name, ty, params))
        declared = {name for name, _, _ in self.post_processor_sections}
        for axis_name, _, _, post_processors in self.axis_sections:
            for ref in post_processors:
                if ref not in declared:
                    raise config.error(
                        "[axis %s] references undeclared post_processor "
                        "'%s' (declare [post_processor %s])"
                        % (axis_name, ref, ref)
                    )

    def _read_arc_fit(self, config):
        self.arc_fit = arc_fit_from_config(config)

    def _read_limits(self, config):
        for key in self.UNSUPPORTED_LIMIT_KEYS:
            if config.get(key, None) is not None:
                raise config.error("[printer] %s is not supported" % key)
        self._max_velocity = config.getfloat("max_velocity", above=0.0)
        self._max_accel = config.getfloat("max_accel", above=0.0)
        self._square_corner_velocity = config.getfloat(
            "square_corner_velocity", 5.0, minval=0.0
        )
        self.max_jerk = config.getfloat(
            "max_jerk", self._max_accel * 2.0, above=0.0
        )
        self.max_z_velocity = config.getfloat(
            "max_z_velocity",
            self._max_velocity,
            above=0.0,
            maxval=self._max_velocity,
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self._max_accel, above=0.0, maxval=self._max_accel
        )
        self.limit_sections = []
        for sc in config.get_prefix_sections("limit "):
            name = sc.get_name().split(None, 1)[1]
            axes = [a.strip().lower() for a in sc.getlist("axes")]
            v = sc.getfloat("max_velocity", None, above=0.0)
            a = sc.getfloat("max_accel", None, above=0.0)
            j = sc.getfloat("max_jerk", None, above=0.0)
            self.limit_sections.append((name, axes, v, a, j))
        self.min_cruise_ratio = 0.0
        self.orig_cfg = {}

    def _sync_print_time(self):
        if self.mcu is None:
            return
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        self.printer.send_event(
            "toolhead:sync_print_time",
            curtime,
            est_print_time,
            self._mcu_pending_end_time,
        )

    def set_accel(self, accel):
        if accel is not None and accel > 0.0:
            self.engine.set_accel_cap(accel)

    def reset_accel(self):
        self.engine.set_accel_cap(None)

    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        for unsupported in (
            "MINIMUM_CRUISE_RATIO",
            "ACCEL_TO_DECEL",
        ):
            if gcmd.get_float(unsupported, None) is not None:
                raise gcmd.error(
                    "%s is not supported: declare limits in [limit] config "
                    "sections" % unsupported
                )
        v = gcmd.get_float("VELOCITY", None, above=0.0)
        a = gcmd.get_float("ACCEL", None, above=0.0)
        scv = gcmd.get_float("SQUARE_CORNER_VELOCITY", None, minval=0.0)
        if v is None and a is None and scv is None:
            velocity, accel, corner = self._effective_limits()
            gcmd.respond_info(
                "velocity=%s accel=%s square_corner_velocity=%s"
                % (velocity, accel, corner)
            )
            return
        if v is not None:
            self.engine.set_velocity_cap(v)
        if a is not None:
            self.engine.set_accel_cap(a)
        if scv is not None:
            self.engine.set_square_corner_velocity(scv)

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
        self.engine.set_square_corner_velocity(None)

    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self._mcu_pending_end_time)
        for m in self.all_mcus:
            if getattr(m, "non_critical_disconnected", False):
                continue
            m.check_active(max_queue_time, eventtime)
        return False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % (
            self.print_time,
            self.print_stall,
        )

    def _declared_axis_order(self):
        return [name for name, _, _, _ in self.axis_sections]

    def _build_axis_to_handle(self):
        axis_to_handle = {}
        for lane_idx, _axis_name, _motor_names in self.kin.lanes():
            rail = self.kin.rails[lane_idx]
            if isinstance(rail, servo_axis.ServoRail):
                node = self.printer.lookup_object(
                    "ethercat_node " + rail.get_node_name(), None
                )
                if node is None:
                    continue
                handle = node.get_engine_handle()
            else:
                steppers = rail.get_steppers()
                if not steppers:
                    continue
                handle = getattr(steppers[0].get_mcu(), "_engine_handle", None)
            if handle is None:
                continue
            axis_to_handle[lane_idx] = handle

        fm = self.printer.lookup_object("force_move", None)
        for _name, motors, slot_idx in self._follower_slots():
            if fm is None:
                continue
            primary = fm.steppers.get(motors[0])
            if primary is None:
                continue
            handle = getattr(primary.get_mcu(), "_engine_handle", None)
            if handle is None:
                continue
            axis_to_handle[slot_idx] = handle
        return axis_to_handle

    def _derive_mcu_topology(self, axis_to_handle):
        by_handle = {}
        for axis_idx, handle in axis_to_handle.items():
            by_handle.setdefault(handle, []).append(axis_idx)
        topo = []
        for handle in sorted(by_handle):
            axes = sorted(by_handle[handle])
            topo.append((handle, axes, self.kin.mcu_tag(axes)))
        return topo

    def _init_planner(self):
        engine_mcus = []
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_engine_handle", None)
            if handle is None:
                continue
            engine_mcus.append((name, mcu, handle))
        if not engine_mcus:
            logging.warning(
                "Motion: no MCU engine handles available; skipping init_planner"
            )
            return

        axis_to_handle = self._build_axis_to_handle()
        topology = self._derive_mcu_topology(axis_to_handle)
        if not topology:
            logging.warning(
                "Motion: no axis->MCU assignment resolved; "
                "skipping init_planner"
            )
            return

        try:
            self.engine.init_planner(
                list(self.axis_sections),
                list(self.limit_sections),
                list(self.post_processor_sections),
                topology,
                self.kin.claimed_axes(),
                (
                    self._max_velocity,
                    self._max_accel,
                    self.max_jerk,
                    self.max_z_velocity,
                    self.max_z_accel,
                    self._square_corner_velocity,
                ),
                arc_fit=self.arc_fit,
            )
            self._configure_axes_per_mcu(engine_mcus)
            self._planner_ready = True

        except Exception:
            logging.exception("Motion: init_planner failed")
            raise

    def _follower_slots(self):
        claimed = set(self.kin.claimed_axes())
        lane_slots = {
            lane_idx for lane_idx, _axis_name, _motor_names in self.kin.lanes()
        }
        free_slots = [i for i in range(4) if i not in lane_slots]
        followers = [
            (name, motors)
            for name, _follows, motors, _pp in self.axis_sections
            if name not in claimed and motors
        ]
        if len(followers) > len(free_slots):
            raise self.printer.command_error(
                "%d follower axes declared but only %d motion slot(s) free of "
                "kinematics lanes" % (len(followers), len(free_slots))
            )
        return [
            (name, motors, slot)
            for (name, motors), slot in zip(followers, free_slots)
        ]

    def _build_slot_steppers(self):
        slot_steppers = [[], [], [], []]
        for lane_idx, _axis_name, _motor_names in self.kin.lanes():
            slot_steppers[lane_idx] = [
                (s.get_name(), s)
                for s in self.kin.rails[lane_idx].get_steppers()
            ]
        fm = self.printer.lookup_object("force_move", None)
        for _name, motors, slot_idx in self._follower_slots():
            entries = []
            for motor_name in motors:
                s = None if fm is None else fm.steppers.get(motor_name)
                if s is not None:
                    entries.append((motor_name, s))
            slot_steppers[slot_idx] = entries
        return slot_steppers

    def _configure_axes_per_mcu(self, engine_mcus):
        coupled = self.kin.coupled_xy()
        awd_default = 0b0011 if coupled else 0b0000

        slot_steppers = self._build_slot_steppers()

        PHASE_STEPPING_CAPABILITY_BIT = 0x1
        STEP_MODE_MODULATED = 0
        STEP_MODE_STEP_TIME = 1

        for name, mcu_obj, mcu_handle in engine_mcus:
            present_mask = 0
            invert_mask = 0
            steps_per_mm = [0.0, 0.0, 0.0, 0.0]
            step_modes = [STEP_MODE_STEP_TIME] * 4
            bind_list = []
            for i in range(4):
                on_this_mcu = []
                for sname, s in slot_steppers[i]:
                    if len(engine_mcus) > 1:
                        try:
                            s_mcu = s.get_mcu()
                        except AttributeError:
                            s_mcu = None
                        if s_mcu is not None and s_mcu is not mcu_obj:
                            continue
                    on_this_mcu.append((sname, s))
                if not on_this_mcu:
                    continue
                primary_name, primary = on_this_mcu[0]
                step_dist = primary.get_step_dist()
                if step_dist <= 0.0:
                    continue
                steps_per_mm[i] = 1.0 / step_dist
                present_mask |= 1 << i
                if getattr(primary, "_invert_dir", False):
                    invert_mask |= 1 << i
                if getattr(primary, "phase_stepping", False):
                    step_modes[i] = STEP_MODE_MODULATED
                for sname, s in on_this_mcu:
                    inv = 1 if getattr(s, "_invert_dir", False) else 0
                    bind_list.append((i, sname, s.get_oid(), inv))
            phase_configs = []
            any_phase_stepping = False
            xy_coupled = coupled
            phase_groups = {}
            for i, slot in enumerate(slot_steppers):
                if step_modes[i] != STEP_MODE_MODULATED or not slot:
                    continue
                group_key = "xy" if (xy_coupled and i in (0, 1)) else i
                slot_tmcs = phase_groups.setdefault(group_key, [])
                for stepper_name, stepper_obj in slot:
                    tmc_name = "tmc5160 " + stepper_name
                    try:
                        tmc = self.printer.lookup_object(tmc_name)
                    except Exception:
                        raise self.printer.config_error(
                            "phase_stepping=True on stepper '%s' requires "
                            "a [tmc5160 %s] section (current driver type "
                            "or absence of TMC5160 section is "
                            "incompatible with phase stepping)"
                            % (stepper_name, stepper_name)
                        )
                    if not hasattr(tmc, "get_phase_config"):
                        raise self.printer.config_error(
                            "phase_stepping=True on stepper '%s' requires "
                            "a TMC5160 driver; found driver type with no "
                            "phase-stepping support" % stepper_name
                        )
                    bus_id, cs_pin_id = tmc.get_phase_config()
                    tmc.set_phase_stepper_oid(stepper_obj.get_oid())
                    slot_tmcs.append(tmc)
                    phase_configs.append((bus_id, cs_pin_id, i))
                    any_phase_stepping = True
            for group in phase_groups.values():
                for tmc in group:
                    tmc.set_phase_group(group)
            FIRMWARE_MAX_PHASE_STEPPED_MOTORS = 16
            if len(phase_configs) > FIRMWARE_MAX_PHASE_STEPPED_MOTORS:
                raise self.printer.config_error(
                    "phase_stepping enabled on %d motors but the firmware "
                    "supports up to %d phase-stepped motors total per MCU."
                    % (len(phase_configs), FIRMWARE_MAX_PHASE_STEPPED_MOTORS)
                )
            awd_mask = awd_default & present_mask
            if present_mask == 0:
                logging.info(
                    "Motion: no steppers matched MCU %s; "
                    "skipping configure_axes",
                    name,
                )
                continue
            mcu_caps = self.engine.get_mcu_capabilities(mcu_handle)
            for i in range(4):
                if step_modes[i] == STEP_MODE_MODULATED and not (
                    mcu_caps & PHASE_STEPPING_CAPABILITY_BIT
                ):
                    slot_name = (
                        slot_steppers[i][0][0]
                        if slot_steppers[i]
                        else "motor_%d" % i
                    )
                    raise self.printer.config_error(
                        "Stepper '%s' requests phase_stepping: 1, but MCU "
                        "'%s' did not advertise the PHASE_STEPPING capability "
                        "in its IdentifyResponse (caps=0x%x). This usually "
                        "means kalico-native identify timed out, which in "
                        "turn usually means the MCU's firmware was built "
                        "without CONFIG_RUNTIME=y. Rebuild that MCU "
                        "with CONFIG_RUNTIME=y (and the small or "
                        "large runtime profile for the chip family) and "
                        "reflash." % (slot_name, name, mcu_caps)
                    )
            try:
                configure_axis_cmd = mcu_obj.lookup_command(
                    "kalico_configure_axis axis_idx=%c mode=%c"
                    " microstep_distance=%u extrusion_per_xy_mm=%u"
                    " stepper_count=%c ring_depth=%hu steppers=%*s"
                )
            except Exception:
                logging.info(
                    "Motion: mcu=%s lacks kalico_configure_axis "
                    "(no new stepping redesign command); skipping runtime "
                    "binding",
                    name,
                )
                continue

            try:
                reset_cmd = mcu_obj.lookup_command("runtime_reset")
            except Exception:
                reset_cmd = None
            if reset_cmd is not None:
                reset_cmd.send([])
                logging.info(
                    "Motion: sent runtime_reset to mcu=%s",
                    name,
                )

            if any_phase_stepping:
                seen_buses = set()
                for bus_id, _cs_pin_id, _slot_idx in phase_configs:
                    if bus_id == 0xFF:
                        continue
                    if bus_id in seen_buses:
                        continue
                    seen_buses.add(bus_id)
                    logging.info(
                        "register_phase_bus mcu=%s bus_id=%d", name, bus_id
                    )
                    self.engine.register_phase_bus(
                        mcu_handle,
                        bus_id,
                        rate=2_000_000,
                    )
                for motor_idx, (bus_id, cs_pin_id, slot_idx) in enumerate(
                    phase_configs,
                ):
                    if bus_id == 0xFF:
                        continue
                    logging.info(
                        "register_phase_motor mcu=%s motor=%d bus=%d cs=%d "
                        "slot=%d",
                        name,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                        slot_idx,
                    )
                    self.engine.register_phase_motor(
                        mcu_handle,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                        slot_idx,
                    )
            axis_bindings = defaultdict(list)
            for slot_idx, sname, oid, inv in bind_list:
                axis_bindings[slot_idx].append((sname, oid, inv))

            MODE_PULSE = 0
            MODE_PHASE = 1
            TMC_CS_OID_NONE = 0xFF
            FLAGS_DEFAULT = 0

            for axis_idx, bindings in axis_bindings.items():
                spm = (
                    steps_per_mm[axis_idx]
                    if axis_idx < len(steps_per_mm)
                    else 0.0
                )
                if spm <= 0:
                    continue
                microstep_distance = 1.0 / spm
                microstep_bits = struct.unpack(
                    "<I", struct.pack("<f", microstep_distance)
                )[0]
                UNUSED_EXTRUSION_PER_XY_BITS = 0
                extrusion_bits = UNUSED_EXTRUSION_PER_XY_BITS
                blob = bytearray()
                for motor_idx, (sname, oid, inv) in enumerate(bindings):
                    self._motor_bindings[sname] = (
                        mcu_handle,
                        axis_idx,
                        motor_idx,
                    )
                    blob.append(oid)
                    blob.append(inv & 0x01)
                    tmc_oid = TMC_CS_OID_NONE
                    if step_modes[axis_idx] == STEP_MODE_MODULATED:
                        tmc_name = "tmc5160 " + sname
                        try:
                            tmc = self.printer.lookup_object(tmc_name)
                            tmc_oid = tmc.get_spi_oid()
                        except Exception:
                            pass
                    blob.append(tmc_oid)
                    blob.append(FLAGS_DEFAULT)
                ring_depth = self.engine.ring_depth_for_axis(
                    mcu_handle, axis_idx
                )
                axis_mode = (
                    MODE_PHASE
                    if step_modes[axis_idx] == STEP_MODE_MODULATED
                    else MODE_PULSE
                )
                configure_axis_cmd.send(
                    [
                        axis_idx,
                        axis_mode,
                        microstep_bits,
                        extrusion_bits,
                        len(bindings),
                        ring_depth,
                        bytes(blob),
                    ]
                )
            logging.info(
                "Motion: configure_axes mcu=%s kin=%s "
                "present=0x%x awd=0x%x invert=0x%x steps_per_mm=%s "
                "step_modes=%s mcu_caps=0x%x runtime_bindings=%s "
                "phase_configs=%s any_phase_stepping=%s "
                "phase_motor_count=%d",
                name,
                self.kin.kind,
                present_mask,
                awd_mask,
                invert_mask,
                steps_per_mm,
                step_modes,
                mcu_caps,
                [(m, n, o, i) for (m, n, o, i) in bind_list],
                phase_configs,
                any_phase_stepping,
                len(phase_configs),
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
        handle = getattr(self.mcu, "_engine_handle", None)
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
        handle = getattr(self.mcu, "_engine_handle", None)
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
        handle = getattr(self.mcu, "_engine_handle", None)
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
        handle = self.mcu._engine_handle
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
        callback(self.motion.get_last_move_time())

    def note_step_generation_scan_time(self, delay, old_delay=0.0):
        self.motion.flush_step_generation()

    def get_trapq(self):
        return None

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        pass

    def limit_next_junction_speed(self, speed):
        pass

    def __getattr__(self, name):
        return getattr(self.motion, name)


def add_printer_objects(config):
    motion = Motion(config)
    printer = config.get_printer()
    printer.add_object("motion", motion)
    printer.add_object("toolhead", ToolheadShim(motion))
    extruder.add_printer_objects(config)
