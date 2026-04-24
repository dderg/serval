# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import importlib
import logging
import math

from . import chelper
from . import jerk_math
from .chelper import jerk_profile as jp_mod
from .extras.danger_options import get_danger_options
from .kinematics import extruder

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)


# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.j_max = toolhead.max_jerk
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        if move_d < 0.000000001:
            # Extrude only move
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
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9

    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed**2)

    def reachable_v_from_v_end(self, v_end):
        """Jerk-aware reachable-velocity: return the largest v_start such
        that an accel-side group from v_start down to v_end spans
        self.move_d under (self.accel, self.j_max).

        By symmetry of the jerk profile the accel-side kinematics are the
        same whether we call the known endpoint v_start or v_end; A2b's
        reachable_v_end is therefore reused for the reverse pass by
        passing v_end in its v_start slot.
        """
        return jerk_math.reachable_v_end(
            v_start=v_end, a_max=self.accel, j_max=self.j_max, L=self.move_d,
        )

    def build_quintic_payload(self):
        """Build a jerk-limited XY + PA-baked E quintic_trapq_payload.

        Plan 9 A2d. Mirrors QuinticBlendMove.finalize_shape's payload
        format but skips shape baking (A3 scope). Must be called AFTER
        set_junction (requires self.jerk_profile). Returns the 9-tuple
        consumed by `_process_moves`:

            (phase_t_ends_tuple, total_t_baked,
             arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
             legacy_t_accel_end, legacy_t_decel_start, legacy_total_t)

        The XY polynomial is the A2a emitter's output; the .e slot is
        filled by linear_pa_compose / nonlinear_pa_compose selected via
        blendplanner._resolve_pa_dispatch(self.toolhead). No shape
        baking.
        """
        from .chelper.linear_quintic import build_jerk_profile_as_quintic_coeffs
        from .chelper import linear_pa_compose as _linear_pa_compose
        from .chelper import nonlinear_pa_compose as _nonlinear_pa_compose
        from . import blendplanner
        n_phases, phase_t_ends_list, coeff_buf_full = \
            build_jerk_profile_as_quintic_coeffs(
                self.jerk_profile,
                (self.axes_r[0], self.axes_r[1], self.axes_r[2]),
                (self.start_pos[0], self.start_pos[1], self.start_pos[2]),
            )
        active_len = n_phases * 15 * 4
        coeff_list = list(coeff_buf_full[:active_len])
        phase_t_ends_tuple = tuple(phase_t_ends_list)
        total_t_baked = phase_t_ends_tuple[-1] if phase_t_ends_tuple else 0.0
        arc_length = self.move_d
        if arc_length > 0.0:
            extr_r = self.axes_d[3] / arc_length
        else:
            extr_r = 0.0
        axis_n = (self.axes_r[0], self.axes_r[1], self.axes_r[2])
        pa_dispatch = blendplanner._resolve_pa_dispatch(self.toolhead)
        if pa_dispatch[0] == "linear":
            k_pa = pa_dispatch[1]
            coeff_list = _linear_pa_compose.linear_pa_compose(
                n_phases, coeff_list,
                axis_n=axis_n, extr_r=extr_r, k_pa=k_pa,
            )
        elif pa_dispatch[0] == "nonlinear":
            _, model, la, no, v_lin = pa_dispatch
            coeff_list, _residual = _nonlinear_pa_compose.nonlinear_pa_compose(
                n_phases, list(phase_t_ends_tuple),
                coeff_list,
                axis_n=axis_n, extr_r=extr_r,
                linear_advance=la,
                nonlinear_offset=no,
                linearization_velocity=v_lin,
                model=model,
            )
        else:
            coeff_list = _linear_pa_compose.linear_pa_compose(
                n_phases, coeff_list,
                axis_n=axis_n, extr_r=extr_r, k_pa=0.0,
            )
        coeff_tuple = tuple(coeff_list)
        v_cap_min = min(self.start_v, self.cruise_v, self.end_v)
        if v_cap_min < 0.0:
            v_cap_min = 0.0
        start_pos_xyz = (
            self.start_pos[0], self.start_pos[1], self.start_pos[2]
        )
        legacy_t_accel_end = self.accel_t
        legacy_t_decel_start = self.accel_t + self.cruise_t
        legacy_total_t = self.accel_t + self.cruise_t + self.decel_t
        return (
            phase_t_ends_tuple, total_t_baked,
            arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
            legacy_t_accel_end, legacy_t_decel_start, legacy_total_t,
        )

    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return self.toolhead.printer.command_error(m)

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extruder to calculate its maximum junction
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        max_start_v2 = min(
            extruder_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_move.max_start_v2 + prev_move.delta_v2,
        )
        # Find max velocity using "approximated centripetal velocity"
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(
            axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]
        )
        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))
        if cos_theta_d2 > 0.0:
            # Centripetal cap: the approximating circle must contact
            # each adjacent move no further than its midpoint, giving
            #   v_max² = 0.5 * move_d * accel * tan(theta/2).
            # Plan 9 A2c: written from the physical form directly
            # rather than as delta_v2 * 0.25 * sin/cos, so it is no
            # longer coupled to the constant-accel delta_v2
            # approximation. accel here is the per-move accel limit,
            # which already carries any kin.check_move / limit_speed
            # tightening.
            tan_theta_d2 = sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = 0.5 * self.move_d * self.accel * tan_theta_d2
            pmove_centripetal_v2 = (
                0.5 * prev_move.move_d * prev_move.accel * tan_theta_d2
            )
            max_start_v2 = min(
                max_start_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            )
        # Apply limits
        self.max_start_v2 = max_start_v2
        self.max_smoothed_v2 = min(
            max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2
        )

    def set_junction(self, start_v2, cruise_v2, end_v2):
        # Plan 9 A2c: jerk-aware phase timings via jerk_profile.compute_profile.
        # The 7-segment profile (J+/A+/J-/C/J-d/A-/J+d) is stored on
        # self.jerk_profile for the A2d emit path. Legacy trapezoidal
        # fields (accel_t/cruise_t/decel_t/start_v/cruise_v/end_v/accel)
        # are populated so existing consumers (extruder.move) emit a
        # trapezoidal approximation whose integral equals move_d and
        # whose endpoint velocities / total duration match the
        # jerk-limited profile exactly.
        start_v = math.sqrt(start_v2) if start_v2 > 0.0 else 0.0
        cruise_v = math.sqrt(cruise_v2) if cruise_v2 > 0.0 else 0.0
        end_v = math.sqrt(end_v2) if end_v2 > 0.0 else 0.0
        self.jerk_profile = jp_mod.compute_profile(
            v0=start_v, v1=end_v, v_peak=cruise_v,
            a_max=self.accel, j_max=self.j_max, L=self.move_d,
        )
        if self.jerk_profile.status != jp_mod.JP_OK:
            raise self.toolhead.printer.command_error(
                "Jerk profile infeasible for move "
                "(start_v=%.6f cruise_v=%.6f end_v=%.6f move_d=%.6f "
                "accel=%.6f j_max=%.6f status=%d)" % (
                    start_v, cruise_v, end_v, self.move_d,
                    self.accel, self.j_max, self.jerk_profile.status,
                )
            )
        # Collapse the 7-segment profile into accel / cruise / decel
        # totals. Segment type tags: J+ / A+ / J- = accel side;
        # C = cruise; J-d / A- / J+d = decel side.
        accel_types = {"J+", "A+", "J-"}
        decel_types = {"J-d", "A-", "J+d"}
        accel_t = 0.0
        cruise_t = 0.0
        decel_t = 0.0
        for seg in self.jerk_profile.segments:
            if seg.type in accel_types:
                accel_t += seg.T
            elif seg.type == "C":
                cruise_t += seg.T
            elif seg.type in decel_types:
                decel_t += seg.T
            else:
                raise self.toolhead.printer.command_error(
                    "Unknown jerk_profile segment type: %r" % (seg.type,)
                )
        self.start_v = start_v
        self.cruise_v = cruise_v
        self.end_v = end_v
        self.accel_t = accel_t
        self.cruise_t = cruise_t
        self.decel_t = decel_t
        # Back-compat emit path relies on the trapezoidal integral
        # (start_v + cruise_v) * 0.5 * accel_t to match the true
        # accel-side distance. Under the jerk-limited profile's
        # (start_v, cruise_v, accel_t) triple this identity holds
        # regardless of what `accel` we carry on self, so leaving
        # self.accel at its pre-set_junction value (the config
        # max_accel that limit_speed / kin.check_move may have
        # lowered) is correct. calc_junction's centripetal formula
        # on the NEXT move continues to reference prev_move.accel
        # for the same purpose.
        # A2d: populate quintic_trapq_payload so _process_moves routes
        # this move through trapq_append_quintic with a jerk-limited XY
        # polynomial and a PA-baked E polynomial. Kinematic moves only —
        # extrude-only (is_kinematic_move == False) fall back to the
        # legacy trapezoid path.
        if self.is_kinematic_move:
            self.quintic_trapq_payload = self.build_quintic_payload()


LOOKAHEAD_FLUSH_TIME = 0.250


# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time

    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None

    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.0
        for i in range(flush_count - 1, -1, -1):
            move = queue[i]
            # Jerk-aware reverse pass (Plan 9 A2c). next_end_v2 is the
            # velocity² the move must land at (next move's start, or 0
            # at end of queue). reachable_v_from_v_end returns the
            # largest v_start achievable on an accel-side group across
            # move.move_d under (move.accel, move.j_max). Symmetry lets
            # us reuse the same primitive for both accel and decel
            # reverse passes.
            reachable_start_v = move.reachable_v_from_v_end(
                math.sqrt(next_end_v2) if next_end_v2 > 0.0 else 0.0
            )
            reachable_start_v2 = reachable_start_v * reachable_start_v
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            # Smoothed pass uses max_accel_to_decel as its accel budget;
            # formula is otherwise identical.
            next_smoothed_v = (
                math.sqrt(next_smoothed_v2) if next_smoothed_v2 > 0.0 else 0.0
            )
            reachable_smoothed_v = jerk_math.reachable_v_end(
                v_start=next_smoothed_v,
                a_max=move.toolhead.max_accel_to_decel,
                j_max=move.j_max,
                L=move.move_d,
            )
            reachable_smoothed_v2 = reachable_smoothed_v * reachable_smoothed_v
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (
                    smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed
                ):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(
                        move.max_cruise_v2,
                        (smoothed_v2 + reachable_smoothed_v2) * 0.5,
                    )
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                m.set_junction(
                                    min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2)
                                )
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min(
                        (start_v2 + reachable_start_v2) * 0.5,
                        move.max_cruise_v2,
                        peak_cruise_v2,
                    )
                    move.set_junction(
                        min(start_v2, cruise_v2),
                        cruise_v2,
                        min(next_end_v2, cruise_v2),
                    )
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return
        # Generate step times for all moves ready to be flushed
        self.toolhead._process_moves(queue[:flush_count])
        # Remove processed moves from the queue
        del queue[:flush_count]

    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.0:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)


BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
BGFLUSH_LOW_TIME = 0.200
BGFLUSH_BATCH_TIME = 0.200
MIN_KIN_TIME = 0.100
MOVE_BATCH_TIME = 0.500
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001  # step+dir+step filter in stepcompress.c
MOVE_HISTORY_EXPIRE = 30.0

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100


class DripModeEndSignal(Exception):
    pass


# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module="mcu")
        ]
        self.mcu = self.all_mcus[0]
        from . import blendprepass, blendplanner
        inner_queue = LookAheadQueue(self)
        self.prepass = blendprepass.CollinearCollapser(self, move_cls=Move)
        self.blender = blendplanner.CornerBlender(self, move_cls=Move)
        self.lookahead = blendprepass.BlendPipelineLookAheadQueue(
            [self.prepass, self.blender], inner_queue
        )
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]
        # Velocity and acceleration control
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
        self.max_jerk = config.getfloat("max_jerk", 100000.0, above=0.0)
        self.corner_deviation = config.getfloat("corner_deviation", above=0.0)
        min_cruise_ratio = 0.5
        if config.getfloat("minimum_cruise_ratio", None) is None:
            req_accel_to_decel = config.getfloat(
                "max_accel_to_decel", None, above=0.0
            )
            if req_accel_to_decel is not None:
                config.deprecate("max_accel_to_decel")
                min_cruise_ratio = 1.0 - min(
                    1.0, (req_accel_to_decel / self.max_accel)
                )
        self.min_cruise_ratio = config.getfloat(
            "minimum_cruise_ratio", min_cruise_ratio, below=1.0, minval=0.0
        )
        scv_legacy = config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            config.deprecate("square_corner_velocity")
            logging.warning(
                "config option [printer] square_corner_velocity is obsolete; "
                "the new arc-blending planner ignores it. Remove it from your "
                "config to silence this warning."
            )
        self.orig_cfg = {}
        self.orig_cfg["max_velocity"] = self.max_velocity
        self.orig_cfg["max_accel"] = self.max_accel
        self.orig_cfg["max_jerk"] = self.max_jerk
        self.orig_cfg["corner_deviation"] = self.corner_deviation
        self.orig_cfg["min_cruise_ratio"] = self.min_cruise_ratio
        # Input stall detection
        self.check_stall_time = 0.0
        self.print_stall = 0
        # Input pause tracking
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        self.need_check_pause = -1.0
        # Print time tracking
        self.print_time = 0.0
        self.special_queuing_state = "NeedPrime"
        self.priming_timer = None
        self.drip_completion = None
        # Flush tracking
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.min_restart_time = 0.0
        self.need_flush_time = self.step_gen_time = self.clear_history_time = (
            0.0
        )
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        self.kin_flush_times = []
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []
        # Create kinematics class
        gcode = self.printer.lookup_object("gcode")
        self.Coord = gcode.Coord
        self.extruder = extruder.DummyExtruder(self.printer)
        # Plan 3: cached extruder-cap snapshot.
        # Refreshed by SET_PRESSURE_ADVANCE / SET_EXTRUDER_LIMITS handlers
        # and on Print Start. None when cap is disabled.
        self.extruder_cap_snapshot = None
        kin_name = config.get("kinematics")
        try:
            mod = importlib.import_module("klippy.kinematics." + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object("pins").error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)
        if (
            config.has_section("dual_carriage")
            and not self.kin.supports_dual_carriage
        ):
            raise config.error(
                "dual_carriage not compatible with '%s' kinematics system"
                % (kin_name,)
            )
        if hasattr(self.kin, "max_x_velocity"):
            self.orig_cfg["max_x_velocity"] = self.kin.max_x_velocity
        if hasattr(self.kin, "max_x_accel"):
            self.orig_cfg["max_x_accel"] = self.kin.max_x_accel
        if hasattr(self.kin, "max_y_velocity"):
            self.orig_cfg["max_y_velocity"] = self.kin.max_y_velocity
        if hasattr(self.kin, "max_y_accel"):
            self.orig_cfg["max_y_accel"] = self.kin.max_y_accel
        if hasattr(self.kin, "max_z_velocity"):
            self.orig_cfg["max_z_velocity"] = self.kin.max_z_velocity
        if hasattr(self.kin, "max_z_accel"):
            self.orig_cfg["max_z_accel"] = self.kin.max_z_accel

        # Register commands
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
        gcode.register_command("M204", self.cmd_M204)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        # Load some default modules
        modules = [
            "gcode_move",
            "homing",
            "idle_timeout",
            "statistics",
            "manual_probe",
            "tuning_tower",
            "garbage_collection",
        ]
        for module_name in modules:
            self.printer.load_object(config, module_name)

    def get_active_rails_for_axis(self, axis):
        # axis is 'x,y,z'
        active_rails = []
        rails = self.kin.rails
        for rail in rails:
            for stepper in rail.get_steppers():
                if stepper.is_active_axis(axis):
                    active_rails.append(rail)
                    break
        return active_rails

    # Print time and flush tracking
    def _advance_flush_time(self, flush_time):
        flush_time = max(flush_time, self.last_flush_time)
        # Generate steps via itersolve
        sg_flush_want = min(
            flush_time + STEPCOMPRESS_FLUSH_TIME,
            self.print_time - self.kin_flush_delay,
        )
        sg_flush_time = max(sg_flush_want, flush_time)
        for sg in self.step_generators:
            sg(sg_flush_time)
        self.min_restart_time = max(self.min_restart_time, sg_flush_time)
        # Free trapq entries that are no longer needed
        clear_history_time = self.clear_history_time
        if not self.can_pause:
            clear_history_time = flush_time - MOVE_HISTORY_EXPIRE
        free_time = sg_flush_time - self.kin_flush_delay
        self.trapq_finalize_moves(self.trapq, free_time, clear_history_time)
        self.extruder.update_move_time(free_time, clear_history_time)
        # Flush stepcompress and mcu steppersync
        for m in self.all_mcus:
            m.flush_moves(flush_time, clear_history_time)
        self.last_flush_time = flush_time

    def _advance_move_time(self, next_print_time):
        pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME
        flush_time = max(self.last_flush_time, self.print_time - pt_delay)
        self.print_time = max(self.print_time, next_print_time)
        want_flush_time = max(flush_time, self.print_time - pt_delay)
        while 1:
            flush_time = min(flush_time + MOVE_BATCH_TIME, want_flush_time)
            self._advance_flush_time(flush_time)
            if flush_time >= want_flush_time:
                break

    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        kin_time = max(est_print_time + MIN_KIN_TIME, self.min_restart_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + BUFFER_TIME_START, kin_time)
        if min_print_time > self.print_time:
            self.print_time = min_print_time
            self.printer.send_event(
                "toolhead:sync_print_time",
                curtime,
                est_print_time,
                self.print_time,
            )

    def _process_moves(self, moves):
        # Resync print_time if necessary
        if self.special_queuing_state:
            if self.special_queuing_state != "Drip":
                # Transition from "NeedPrime"/"Priming" state to main state
                self.special_queuing_state = ""
                self.need_check_pause = -1.0
            self._calc_print_time()
        # Queue moves into trapezoid motion queue (trapq)
        next_move_time = self.print_time
        ffi_main, ffi_lib = chelper.get_ffi()
        for move in moves:
            # Plan 5 D2c — quintic blend moves carry a pre-composed per-phase
            # position-in-t polynomial payload. Route through
            # trapq_append_quintic instead of the linear trapq_append path.
            qpayload = getattr(move, "quintic_trapq_payload", None)
            if qpayload is not None:
                # Plan 8 Chunk 3 payload layout:
                #   (phase_t_ends_tuple, total_t_baked,
                #    arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
                #    legacy_t_accel_end, legacy_t_decel_start, legacy_total_t)
                # phase_t_ends_tuple is a tuple of absolute move-local end
                # times per phase, length n_phases up to MOVE_MAX_PIECES.
                # coeff_tuple is n_phases * 15 * 4 doubles (x, y, z, e).
                # The .e slot carries the linear-PA-baked extruder
                # polynomial. total_t_baked is phase_t_ends_tuple[-1].
                (phase_t_ends_tuple, total_t_baked,
                 arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
                 *_legacy) = qpayload
                n_phases = len(phase_t_ends_tuple)
                coeff_buf = ffi_main.new(
                    f"double[{n_phases * 15 * 4}]", list(coeff_tuple)
                )
                phase_t_ends = ffi_main.new(
                    f"double[{n_phases}]", list(phase_t_ends_tuple)
                )
                # Planner-emitted blend moves are always shaped (the whole
                # point of baking) — shape_disabled=0. Future drip / homing
                # paths that route through this branch will need their
                # own flag plumbing (see Phase 0 §6.5 audit).
                ffi_lib.trapq_append_quintic(
                    self.trapq, next_move_time,
                    n_phases, phase_t_ends,
                    total_t_baked, arc_length, v_cap_min,
                    0,
                    start_pos_xyz[0], start_pos_xyz[1], start_pos_xyz[2],
                    coeff_buf,
                )
                if move.axes_d[3]:
                    self.extruder.move(next_move_time, move)
                next_move_time = next_move_time + total_t_baked
                for cb in move.timing_callbacks:
                    cb(next_move_time)
                continue
            # Plan 9 A2d: kinematic moves all carry quintic_trapq_payload
            # and took the `continue` path above. Only pure-E moves
            # (is_kinematic_move == False) reach here; they emit through
            # extruder.move's legacy trapezoid path below.
            if move.axes_d[3]:
                self.extruder.move(next_move_time, move)
            next_move_time = (
                next_move_time + move.accel_t + move.cruise_t + move.decel_t
            )
            for cb in move.timing_callbacks:
                cb(next_move_time)
        # Generate steps for moves
        if self.special_queuing_state:
            self._update_drip_move_time(next_move_time)
        self.note_mcu_movequeue_activity(
            next_move_time + self.kin_flush_delay, set_step_gen_time=True
        )
        self._advance_move_time(next_move_time)

    def _flush_lookahead(self):
        # Transit from "NeedPrime"/"Priming"/"Drip"/main state to "NeedPrime"
        self.lookahead.flush()
        self.special_queuing_state = "NeedPrime"
        self.need_check_pause = -1.0
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.0

    def flush_step_generation(self):
        self._flush_lookahead()
        self._advance_flush_time(self.step_gen_time)
        self.min_restart_time = max(self.min_restart_time, self.print_time)

    def get_last_move_time(self):
        if self.special_queuing_state:
            self._flush_lookahead()
            self._calc_print_time()
        else:
            self.lookahead.flush()
        return self.print_time

    def _check_pause(self):
        eventtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(eventtime)
        buffer_time = self.print_time - est_print_time
        if self.special_queuing_state:
            if self.check_stall_time:
                # Was in "NeedPrime" state and got there from idle input
                if est_print_time < self.check_stall_time:
                    self.print_stall += 1
                self.check_stall_time = 0.0
            # Transition from "NeedPrime"/"Priming" state to "Priming" state
            self.special_queuing_state = "Priming"
            self.need_check_pause = -1.0
            if self.priming_timer is None:
                self.priming_timer = self.reactor.register_timer(
                    self._priming_handler
                )
            wtime = eventtime + max(0.100, buffer_time - BUFFER_TIME_LOW)
            self.reactor.update_timer(self.priming_timer, wtime)
        # Check if there are lots of queued moves and pause if so
        while True:
            pause_time = buffer_time - BUFFER_TIME_HIGH
            if pause_time <= 0.0:
                break
            if not self.can_pause:
                self.need_check_pause = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1.0, pause_time))
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
        if not self.special_queuing_state:
            # In main state - defer pause checking until needed
            self.need_check_pause = est_print_time + BUFFER_TIME_HIGH + 0.100

    def _priming_handler(self, eventtime):
        self.reactor.unregister_timer(self.priming_timer)
        self.priming_timer = None
        try:
            if self.special_queuing_state == "Priming":
                self._flush_lookahead()
                self.check_stall_time = self.print_time
        except:
            logging.exception("Exception in priming_handler")
            self.printer.invoke_shutdown("Exception in priming_handler")
        return self.reactor.NEVER

    def _flush_handler(self, eventtime):
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            if not self.special_queuing_state:
                # In "main" state - flush lookahead if buffer runs low
                print_time = self.print_time
                buffer_time = print_time - est_print_time
                if buffer_time > BUFFER_TIME_LOW:
                    # Running normally - reschedule check
                    return eventtime + buffer_time - BUFFER_TIME_LOW
                # Under ran low buffer mark - flush lookahead queue
                self._flush_lookahead()
                if print_time != self.print_time:
                    self.check_stall_time = self.print_time
            # In "NeedPrime"/"Priming" state - flush queues if needed
            while 1:
                end_flush = (
                    self.need_flush_time
                    + get_danger_options().bgflush_extra_time
                )
                if self.last_flush_time >= end_flush:
                    self.do_kick_flush_timer = True
                    return self.reactor.NEVER
                buffer_time = self.last_flush_time - est_print_time
                if buffer_time > BGFLUSH_LOW_TIME:
                    return eventtime + buffer_time - BGFLUSH_LOW_TIME
                ftime = est_print_time + BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
                self._advance_flush_time(min(end_flush, ftime))
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER

    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)

    def set_position(self, newpos, homing_axes=()):
        self.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trapq_set_position(
            self.trapq, self.print_time, newpos[0], newpos[1], newpos[2]
        )
        self.commanded_pos[:] = newpos
        self.kin.set_position(newpos, homing_axes)
        self.printer.send_event("toolhead:set_position")

    def limit_next_junction_speed(self, speed):
        last_move = self.lookahead.get_last()
        if last_move is not None:
            last_move.limit_next_junction_speed(speed)

    def move(self, newpos, speed):
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        # Plan 3 extruder-cap for straight MOVE_LINEAR edges. Plan 5 D7
        # absorbs the cap-per-s contribution into the quintic corner
        # primitive via v_cap_fn(s)::v_extr(s); the CornerBlender's
        # emitted QuinticBlendMove is not routed through the linear
        # cap_move path here. On straight edges (pre/post-blend
        # truncated Moves, pure travel, pure extrusion) the
        # cap_move call applies unchanged.
        snap = self.extruder_cap_snapshot
        if snap is not None:
            from klippy import blendextruder
            import math as _m
            pa_snap, limits = snap
            v_cap, a_cap = blendextruder.cap_move(move, pa_snap, limits)
            # Safety: never pass zero or negative to limit_speed (ZeroDivisionError).
            v_cap_finite = _m.isfinite(v_cap) and v_cap > 0.0
            a_cap_safe = a_cap if (_m.isfinite(a_cap) and a_cap > 0.0) else move.accel
            if v_cap_finite:
                move.limit_speed(v_cap, a_cap_safe)
            elif _m.isfinite(a_cap) and a_cap > 0.0:
                # Only accel cap applies; leave cruise velocity unchanged.
                move.limit_speed(_m.sqrt(move.max_cruise_v2), a_cap)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.lookahead.add_move(move)
        if self.print_time > self.need_check_pause:
            self._check_pause()

    def manual_move(self, coord, speed):
        curpos = list(self.commanded_pos)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)
        self.printer.send_event("toolhead:manual_move")

    def dwell(self, delay):
        next_print_time = self.get_last_move_time() + max(0.0, delay)
        self._advance_move_time(next_print_time)
        self._check_pause()

    def wait_moves(self):
        self._flush_lookahead()
        eventtime = self.reactor.monotonic()
        while (
            not self.special_queuing_state
            or self.print_time >= self.mcu.estimated_print_time(eventtime)
        ):
            if not self.can_pause:
                break
            eventtime = self.reactor.pause(eventtime + 0.100)

    def set_extruder(self, extruder, extrude_pos):
        self.extruder = extruder
        self.commanded_pos[3] = extrude_pos
        self._refresh_extruder_cap_snapshot()

    def get_extruder(self):
        return self.extruder

    def _refresh_extruder_cap_snapshot(self):
        """Refresh cached (PAModelSnapshot, ExtruderLimits). Called when
        PA model or extruder limits change. Sets None if cap disabled."""
        extruder = getattr(self, "extruder", None)
        if extruder is None:
            self.extruder_cap_snapshot = None
            return
        # PrinterExtruder delegates to its primary ExtruderStepper.
        snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
        if snap_fn is None:
            steppers = getattr(extruder, "extruder_steppers", None)
            if steppers:
                snap_fn = getattr(steppers[0], "extruder_limits_snapshot", None)
        if snap_fn is None:
            self.extruder_cap_snapshot = None
            return
        self.extruder_cap_snapshot = snap_fn()

    # Homing "drip move" handling
    def _update_drip_move_time(self, next_print_time):
        flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + self.kin_flush_delay
        while self.print_time < next_print_time:
            if self.drip_completion.test():
                raise DripModeEndSignal()
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = self.print_time - est_print_time - flush_delay
            if wait_time > 0.0 and self.can_pause:
                # Pause before sending more steps
                self.drip_completion.wait(curtime + wait_time)
                continue
            npt = min(self.print_time + DRIP_SEGMENT_TIME, next_print_time)
            self.note_mcu_movequeue_activity(
                npt + self.kin_flush_delay, set_step_gen_time=True
            )
            self._advance_move_time(npt)

    def drip_move(self, newpos, speed, drip_completion):
        self.dwell(self.kin_flush_delay)
        # Transition from "NeedPrime"/"Priming"/main state to "Drip" state
        self.lookahead.flush()
        self.special_queuing_state = "Drip"
        self.need_check_pause = self.reactor.NEVER
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.0
        self.drip_completion = drip_completion
        # Submit move
        try:
            self.move(newpos, speed)
        except self.printer.command_error as e:
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
            self.flush_step_generation()
            raise
        # Transmit move in "drip" mode
        try:
            self.lookahead.flush()
        except DripModeEndSignal as e:
            self.lookahead.reset()
            self.trapq_finalize_moves(self.trapq, self.reactor.NEVER, 0)
        # Exit "Drip" state
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self.flush_step_generation()

    # Misc commands
    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self.last_flush_time)
        for m in self.all_mcus:
            m.check_active(max_queue_time, eventtime)
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
        buffer_time = self.print_time - est_print_time
        is_active = buffer_time > -60.0 or not self.special_queuing_state
        if self.special_queuing_state == "Drip":
            buffer_time = 0.0
        return is_active, (
            "print_time=%.3f buffer_time=%.3f print_stall=%d "
            "blend_moves=%d blend_corners=%d"
            % (
                self.print_time,
                max(buffer_time, 0.0),
                self.print_stall,
                self.blender.polyline_moves_emitted,
                self.blender.blends_emitted,
            )
        )

    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.lookahead.queue
        return self.print_time, est_print_time, lookahead_empty

    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        res = dict(self.kin.get_status(eventtime))
        res.update(
            {
                "print_time": print_time,
                "stalls": self.print_stall,
                "estimated_print_time": estimated_print_time,
                "extruder": self.extruder.get_name(),
                "position": self.Coord(*self.commanded_pos),
                "max_velocity": self.max_velocity,
                "max_accel": self.max_accel,
                "max_jerk": self.max_jerk,
                "minimum_cruise_ratio": self.min_cruise_ratio,
                "corner_deviation": self.corner_deviation,
            }
        )
        return res

    def _handle_shutdown(self):
        self.can_pause = False
        self.lookahead.reset()

    def get_kinematics(self):
        return self.kin

    def get_trapq(self):
        return self.trapq

    def register_step_generator(self, handler):
        self.step_generators.append(handler)

    def note_step_generation_scan_time(self, delay, old_delay=0.0):
        self.flush_step_generation()
        if old_delay:
            self.kin_flush_times.pop(self.kin_flush_times.index(old_delay))
        if delay:
            self.kin_flush_times.append(delay)
        new_delay = max(self.kin_flush_times + [SDS_CHECK_TIME])
        self.kin_flush_delay = new_delay

    def register_lookahead_callback(self, callback):
        last_move = self.lookahead.get_last()
        if last_move is None:
            callback(self.get_last_move_time())
            return
        last_move.timing_callbacks.append(callback)

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if set_step_gen_time:
            self.step_gen_time = max(self.step_gen_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)

    def get_max_velocity(self):
        return self.max_velocity, self.max_accel

    @property
    def max_accel_to_decel(self):
        # Derived live from min_cruise_ratio rather than cached, so M204 /
        # SET_VELOCITY_LIMIT mutations are visible without an explicit recompute.
        return self.max_accel * (1.0 - self.min_cruise_ratio)

    def cmd_G4(self, gcmd):
        # Dwell
        delay = gcmd.get_float("P", 0.0, minval=0.0) / 1000.0
        self.dwell(delay)

    def cmd_M400(self, gcmd):
        # Wait for current moves to finish
        self.wait_moves()

    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float("VELOCITY", None, above=0.0)
        max_accel = gcmd.get_float("ACCEL", None, above=0.0)
        max_jerk = gcmd.get_float("JERK", None, above=0.0)
        # Parsed but discarded: the new arc-blending planner ignores SCV.
        # Kept as a local for the all-None guard below so SET_VELOCITY_LIMIT
        # SQUARE_CORNER_VELOCITY=N does not spam the current-status dump.
        square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY", None, minval=0.0
        )
        min_cruise_ratio = gcmd.get_float(
            "MINIMUM_CRUISE_RATIO", None, minval=0.0, below=1.0
        )
        if min_cruise_ratio is None:
            req_accel_to_decel = gcmd.get_float(
                "ACCEL_TO_DECEL", None, above=0.0
            )
            if req_accel_to_decel is not None and max_accel is not None:
                min_cruise_ratio = 1.0 - min(
                    1.0, req_accel_to_decel / max_accel
                )
            elif req_accel_to_decel is not None and max_accel is None:
                min_cruise_ratio = 1.0 - min(
                    1.0, (req_accel_to_decel / self.max_accel)
                )
        corner_deviation = gcmd.get_float(
            "CORNER_DEVIATION", None, above=0.0
        )
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if max_jerk is not None:
            self.max_jerk = max_jerk
        if min_cruise_ratio is not None:
            self.min_cruise_ratio = min_cruise_ratio
        if corner_deviation is not None:
            self.corner_deviation = corner_deviation
        msg = [
            "max_velocity: %.6f" % self.max_velocity,
            "max_accel: %.6f" % self.max_accel,
            "max_jerk: %.6f" % self.max_jerk,
        ]
        if hasattr(self.kin, "max_x_velocity"):
            max_x_velocity = gcmd.get_float("X_VELOCITY", None)
            if max_x_velocity is not None:
                self.kin.max_x_velocity = max_x_velocity
            msg.append("max_x_velocity: %.6f" % self.kin.max_x_velocity)

        if hasattr(self.kin, "max_x_accel"):
            max_x_accel = gcmd.get_float("X_ACCEL", None)
            if max_x_accel is not None:
                self.kin.max_x_accel = max_x_accel
            msg.append("max_x_accel: %.6f" % self.kin.max_x_accel)

        if hasattr(self.kin, "max_y_velocity"):
            max_y_velocity = gcmd.get_float("Y_VELOCITY", None)
            if max_y_velocity is not None:
                self.kin.max_y_velocity = max_y_velocity
            msg.append("max_y_velocity: %.6f" % self.kin.max_y_velocity)

        if hasattr(self.kin, "max_y_accel"):
            max_y_accel = gcmd.get_float("Y_ACCEL", None)
            if max_y_accel is not None:
                self.kin.max_y_accel = max_y_accel
            msg.append(
                "max_y_accel: %.6f" % self.kin.max_y_accel,
            )

        if hasattr(self.kin, "max_z_velocity"):
            max_z_velocity = gcmd.get_float("Z_VELOCITY", None, above=0.0)
            if max_z_velocity is not None:
                self.kin.max_z_velocity = max_z_velocity
            msg.append("max_z_velocity: %.6f" % self.kin.max_z_velocity)

        if hasattr(self.kin, "max_z_accel"):
            max_z_accel = gcmd.get_float("Z_ACCEL", None, above=0.0)
            if max_z_accel is not None:
                self.kin.max_z_accel = max_z_accel
            msg.append("max_z_accel: %.6f" % self.kin.max_z_accel)

        msg.append("minimum_cruise_ratio: %.6f" % self.min_cruise_ratio)
        msg.append("corner_deviation: %.6f" % self.corner_deviation)

        if get_danger_options().log_velocity_limit_changes:
            self.printer.set_rollover_info(
                "toolhead", "toolhead: %s" % (" ".join(msg),)
            )
            if (
                max_velocity is None
                and max_accel is None
                and max_jerk is None
                and square_corner_velocity is None
                and min_cruise_ratio is None
                and corner_deviation is None
            ):
                gcmd.respond_info("\n".join(msg), log=False)

    cmd_RESET_VELOCITY_LIMIT_help = "Reset printer velocity limits"

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        self.max_velocity = self.orig_cfg["max_velocity"]
        self.max_accel = self.orig_cfg["max_accel"]
        self.max_jerk = self.orig_cfg["max_jerk"]
        msg = [
            "max_velocity: %.6f" % self.max_velocity,
            "max_accel: %.6f" % self.max_accel,
            "max_jerk: %.6f" % self.max_jerk,
        ]

        if hasattr(self.kin, "max_x_velocity"):
            self.kin.max_x_velocity = self.orig_cfg["max_x_velocity"]
            msg.append("max_x_velocity: %.6f" % self.kin.max_x_velocity)

        if hasattr(self.kin, "max_x_accel"):
            self.kin.max_x_accel = self.orig_cfg["max_x_accel"]
            msg.append("max_x_accel: %.6f" % self.kin.max_x_accel)

        if hasattr(self.kin, "max_y_velocity"):
            self.kin.max_y_velocity = self.orig_cfg["max_y_velocity"]
            msg.append("max_y_velocity: %.6f" % self.kin.max_y_velocity)

        if hasattr(self.kin, "max_y_accel"):
            self.kin.max_y_accel = self.orig_cfg["max_y_accel"]
            msg.append(
                "max_y_accel: %.6f" % self.kin.max_y_accel,
            )

        if hasattr(self.kin, "max_z_velocity"):
            self.kin.max_z_velocity = self.orig_cfg["max_z_velocity"]
            msg.append("max_z_velocity: %.6f" % self.kin.max_z_velocity)

        if hasattr(self.kin, "max_z_accel"):
            self.kin.max_z_accel = self.orig_cfg["max_z_accel"]
            msg.append("max_z_accel: %.6f" % self.kin.max_z_accel)

        self.min_cruise_ratio = self.orig_cfg["min_cruise_ratio"]
        self.corner_deviation = self.orig_cfg["corner_deviation"]
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "corner_deviation: %.6f" % self.corner_deviation,
            )
        )
        if get_danger_options().log_velocity_limit_changes:
            gcmd.respond_info("\n".join(msg), log=False)

    def cmd_M204(self, gcmd):
        # Use S for accel
        accel = gcmd.get_float("S", None, above=0.0)
        if accel is None:
            # Use minimum of P and T for accel
            p = gcmd.get_float("P", None, above=0.0)
            t = gcmd.get_float("T", None, above=0.0)
            if p is None or t is None:
                gcmd.respond_info(
                    'Invalid M204 command "%s"' % (gcmd.get_commandline(),)
                )
                return
            accel = min(p, t)
        self.max_accel = accel

    def set_accel(self, accel):
        self.max_accel = accel

    def reset_accel(self):
        self.max_accel = self.orig_cfg["max_accel"]


def add_printer_objects(config):
    config.get_printer().add_object("toolhead", ToolHead(config))
    extruder.add_printer_objects(config)
