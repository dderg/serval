# Mechanical bed tilt calibration with multiple Z steppers
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

import numpy as np

from klippy import mathutil

from . import probe


def params_to_normal_form(params, offsets):
    v = np.array([offsets[0], offsets[1], params["z_adjust"]])
    r = np.array([1, 0, params["x_adjust"]])
    s = np.array([0, 1, params["y_adjust"]])
    cp = np.cross(r, s)
    return np.append(cp, np.dot(cp, v))


def intersect_3_planes(p1, p2, p3):
    a = np.array([p1[0:3], p2[0:3], p3[0:3]])
    b = np.array([p1[3], p2[3], p3[3]])
    sol = np.linalg.solve(a, b)
    return sol


def _estimate_pivot_point(ad_params, offsets):
    planes = [params_to_normal_form(pr, offsets) for pr in ad_params]
    tilt_pos = (
        intersect_3_planes(planes[0], planes[2], planes[3])[:2],
        intersect_3_planes(planes[0], planes[1], planes[3])[:2],
        intersect_3_planes(planes[0], planes[1], planes[2])[:2],
    )
    tilt_neg = (
        intersect_3_planes(planes[0], planes[5], planes[6])[:2],
        intersect_3_planes(planes[0], planes[4], planes[6])[:2],
        intersect_3_planes(planes[0], planes[4], planes[5])[:2],
    )
    z_pos = []
    for _pos, _neg in zip(tilt_pos, tilt_neg):
        z_pos.append([(_p + _n) / 2 for _p, _n in zip(_pos, _neg)])
    return z_pos


def _check_convergence(runs, avlen):
    errors = np.std(runs[-avlen:], axis=0)
    return np.std(errors)


def _average_runs(runs, avlen):
    return np.mean(runs[-avlen:], axis=0).tolist()


def _format_offsets(offsets):
    s = ""
    for off in offsets:
        s += "%.6f, " % off
    return s[:-2]


def _format_positions(positions):
    s = ""
    for pos in positions:
        s += "%.6f, %.6f\n" % tuple(pos)
    return s


class ZAdjustHelper:
    def __init__(self, config, z_count):
        self.printer = config.get_printer()
        self.config = config
        self.name = config.get_name()
        self.z_count = z_count
        self.z_steppers = []
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )

    def handle_connect(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis("z")]
        if self.z_count is None:
            if len(z_steppers) != 3:
                raise self.printer.config_error(
                    "%s z_positions needs exactly 3 items for calibration"
                    % (self.name)
                )
        elif len(z_steppers) != self.z_count:
            raise self.printer.config_error(
                "%s z_positions needs exactly %d items"
                % (self.name, len(z_steppers))
            )
        if len(z_steppers) < 2:
            raise self.printer.config_error(
                "%s requires multiple z steppers" % (self.name,)
            )
        self.z_steppers = z_steppers

    def adjust_steppers(self, adjustments, speed):
        gcode = self.printer.lookup_object("gcode")
        reference = min(adjustments)
        deltas = [a - reference for a in adjustments]
        stepstrs = [
            "%s = %.6f" % (s.get_name(), d)
            for s, d in zip(self.z_steppers, deltas)
        ]
        gcode.respond_info(
            "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        )
        force_move = self.printer.load_object(self.config, "force_move")
        toolhead = self.printer.lookup_object("toolhead")
        accel = toolhead.get_max_axis_accel(2)
        for stepper, delta in zip(self.z_steppers, deltas):
            if delta < 1e-6:
                continue
            force_move.manual_move(stepper, delta, speed, accel)
        toolhead.flush_step_generation()
        curpos = toolhead.get_position()
        curpos[2] -= reference
        toolhead.set_position(curpos)


class ZAdjustStatus:
    def __init__(self, printer):
        self.applied = False
        printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

    def check_retry_result(self, retry_result):
        if (isinstance(retry_result, str) and retry_result == "done") or (
            isinstance(retry_result, float) and retry_result == 0.0
        ):
            self.applied = True
        return retry_result

    def reset(self):
        self.applied = False

    def get_status(self, eventtime):
        return {"applied": self.applied}

    def _motor_off(self, print_time):
        self.reset()


class RetryHelper:
    def __init__(self, config, error_msg_extra=""):
        self.gcode = config.get_printer().lookup_object("gcode")
        self.default_max_retries = config.getint("retries", 0, minval=0)
        self.default_retry_tolerance = config.getfloat(
            "retry_tolerance", 0.0, above=0.0
        )
        self.default_increasing_threshold = config.getfloat(
            "increasing_threshold", 0.0000001, above=0.0
        )
        self.value_label = "Probed points range"
        self.error_msg_extra = error_msg_extra

    def start(self, gcmd):
        self.max_retries = gcmd.get_int(
            "RETRIES", self.default_max_retries, minval=0, maxval=30
        )
        self.retry_tolerance = gcmd.get_float(
            "RETRY_TOLERANCE",
            self.default_retry_tolerance,
            minval=0.0,
            maxval=1.0,
        )
        self.increasing_threshold = gcmd.get_float(
            "INCREASING_THRESHOLD", self.default_increasing_threshold, above=0.0
        )
        self.current_retry = 0
        self.previous = None
        self.increasing = 0

    def check_increase(self, error):
        if self.previous and error > self.previous + self.increasing_threshold:
            self.increasing += 1
        elif self.increasing > 0:
            self.increasing -= 1
        self.previous = error
        return self.increasing > 1

    def check_retry(self, z_positions):
        if self.max_retries == 0:
            return "done"
        error = round(max(z_positions) - min(z_positions), 6)
        self.gcode.respond_info(
            "Retries: %d/%d %s: %0.6f tolerance: %0.6f"
            % (
                self.current_retry,
                self.max_retries,
                self.value_label,
                error,
                self.retry_tolerance,
            )
        )
        if self.check_increase(error):
            raise self.gcode.error(
                "Retries aborting: %s is increasing. %s"
                % (self.value_label, self.error_msg_extra)
            )
        if error <= self.retry_tolerance:
            return 0.0
        self.current_retry += 1
        if self.current_retry > self.max_retries:
            raise self.gcode.error("Too many retries")
        return error


class ZTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.section = config.get_name()

        self.z_positions = config.getlists(
            "z_positions", seps=(",", "\n"), parser=float, count=2
        )
        z_count = len(self.z_positions)

        self.retry_helper = RetryHelper(config)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        self.probe_helper.minimum_points(2)

        self.z_offsets = config.getlists(
            "z_offsets", parser=float, count=z_count, default=None
        )

        self.z_status = ZAdjustStatus(self.printer)
        self.z_helper = ZAdjustHelper(config, z_count)
        # probe points for calibrate/autodetect
        cal_probe_points = list(self.probe_helper.get_probe_points())
        self.num_probe_points = len(cal_probe_points)
        self.cal_helper = None
        if config.get("extra_points", None) is not None:
            self.cal_helper = probe.ProbePointsHelper(
                config, self.cal_finalize, option_name="extra_points"
            )
            cal_probe_points.extend(self.cal_helper.get_probe_points())
            self.cal_helper.update_probe_points(cal_probe_points, 3)
        self.ad_helper = probe.ProbePointsHelper(config, self.ad_finalize)
        self.ad_helper.update_probe_points(cal_probe_points, 3)
        self.cal_conf_avg_len = config.getint("averaging_len", 3, minval=1)
        self.ad_conf_delta = config.getfloat(
            "autodetect_delta", 1.0, minval=0.1
        )

        self.use_adjustments = config.getboolean("use_adjustments", False)

        # Register Z_TILT_ADJUST command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "Z_TILT_ADJUST",
            self.cmd_Z_TILT_ADJUST,
            desc=self.cmd_Z_TILT_ADJUST_help,
        )
        if self.cal_helper is not None:
            gcode.register_command(
                "Z_TILT_CALIBRATE",
                self.cmd_Z_TILT_CALIBRATE,
                desc=self.cmd_Z_TILT_CALIBRATE_help,
            )
        gcode.register_command(
            "Z_TILT_AUTODETECT",
            self.cmd_Z_TILT_AUTODETECT,
            desc=self.cmd_Z_TILT_AUTODETECT_help,
        )

    cmd_Z_TILT_ADJUST_help = "Adjust the Z tilt"
    cmd_Z_TILT_CALIBRATE_help = (
        "Calibrate Z tilt with additional probing points"
    )
    cmd_Z_TILT_AUTODETECT_help = "Autodetect pivot point of Z motors"

    def cmd_Z_TILT_ADJUST(self, gcmd):
        if self.z_positions is None:
            gcmd.respond_info(
                "No z_positions configured. Run Z_TILT_AUTODETECT first"
            )
            return
        self.z_status.reset()
        self.retry_helper.start(gcmd)
        self.probe_helper.start_probe(gcmd)

    def perform_coordinate_descent(self, offsets, positions):
        # Setup for coordinate descent analysis
        z_offset = offsets[2]
        logging.info("Calculating bed tilt with: %s", positions)
        params = {"x_adjust": 0.0, "y_adjust": 0.0, "z_adjust": z_offset}

        # Perform coordinate descent
        def adjusted_height(pos, params):
            x, y, z = pos
            return (
                z
                - x * params["x_adjust"]
                - y * params["y_adjust"]
                - params["z_adjust"]
            )

        def errorfunc(params):
            total_error = 0.0
            for pos in positions:
                total_error += adjusted_height(pos, params) ** 2
            return total_error

        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc
        )

        # Apply results
        logging.info("Calculated bed tilt parameters: %s", new_params)
        return new_params

    def apply_adjustments(self, offsets, new_params):
        z_offset = offsets[2]
        speed = self.probe_helper.get_lift_speed()
        x_adjust = float(new_params["x_adjust"])
        y_adjust = float(new_params["y_adjust"])
        z_adjust = float(
            new_params["z_adjust"]
            - z_offset
            - x_adjust * offsets[0]
            - y_adjust * offsets[1]
        )
        adjustments = [
            x * x_adjust + y * y_adjust + z_adjust for x, y in self.z_positions
        ]
        self.z_helper.adjust_steppers(adjustments, speed)
        return adjustments

    def probe_finalize(self, offsets, positions):
        if self.z_offsets is not None:
            positions = [
                [p[0], p[1], p[2] - o]
                for (p, o) in zip(positions, self.z_offsets)
            ]
        new_params = self.perform_coordinate_descent(offsets, positions)
        adjustments = self.apply_adjustments(offsets, new_params)
        return self.z_status.check_retry_result(
            self.retry_helper.check_retry(
                adjustments
                if self.use_adjustments
                else [p[2] for p in positions]
            )
        )

    def cmd_Z_TILT_CALIBRATE(self, gcmd):
        self.cal_avg_len = gcmd.get_int("AVGLEN", self.cal_conf_avg_len)
        self.cal_gcmd = gcmd
        self.cal_runs = []
        self.cal_helper.start_probe(gcmd)

    def cal_finalize(self, offsets, positions):
        avlen = self.cal_avg_len
        new_params = self.perform_coordinate_descent(offsets, positions)
        self.apply_adjustments(offsets, new_params)
        self.cal_runs.append([p[2] for p in positions])
        if len(self.cal_runs) < avlen + 1:
            return "retry"
        prev_error = _check_convergence(self.cal_runs[:-1], avlen)
        this_error = _check_convergence(self.cal_runs, avlen)
        self.cal_gcmd.respond_info(
            "previous error: %.6f current error: %.6f"
            % (prev_error, this_error)
        )
        if this_error < prev_error:
            return "retry"
        z_offsets = _average_runs(self.cal_runs, avlen)
        z_offsets = [z - offsets[2] for z in z_offsets]
        self.z_offsets = z_offsets
        s_zoff = _format_offsets(z_offsets[: self.num_probe_points])
        self.cal_gcmd.respond_info("final z_offsets are: %s" % (s_zoff))
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section, "z_offsets", s_zoff)
        self.cal_gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config\n"
            "file with these parameters and restart the printer."
        )

    def ad_init(self):
        self.ad_phase = 0
        self.ad_params = []

    def cmd_Z_TILT_AUTODETECT(self, gcmd):
        self.cal_avg_len = gcmd.get_int("AVGLEN", self.cal_conf_avg_len)
        self.ad_delta = gcmd.get_float("DELTA", self.ad_conf_delta, minval=0.1)
        self.ad_init()
        self.ad_gcmd = gcmd
        self.ad_runs = []
        self.ad_points = []
        self.ad_error = None
        self.ad_helper.start_probe(gcmd)

    ad_adjustments = [
        [0.5, -0.5, -0.5],  # p1 up
        [-1, 1, 0],  # p2 up
        [0, -1, 1],  # p3 up
        [0, 1, 0],  # p3 + p2 up
        [1, -1, 0],  # p3 + p1 up
        [0, 1, -1],  # p2 + p1 up
        [-0.5, -0.5, 0.5],  # back to level
    ]

    def ad_finalize(self, offsets, positions):
        avlen = self.cal_avg_len
        delta = self.ad_delta
        speed = self.probe_helper.get_lift_speed()
        new_params = self.perform_coordinate_descent(offsets, positions)
        if self.ad_phase in range(1, 4):
            new_params["z_adjust"] -= delta / 2
        if self.ad_phase in range(4, 7):
            new_params["z_adjust"] += delta / 2
        if self.ad_phase == 0:
            self.ad_points.append(
                [z for _, _, z in positions[: self.num_probe_points]]
            )
        self.ad_params.append(new_params)
        adjustments = [_a * delta for _a in self.ad_adjustments[self.ad_phase]]
        self.z_helper.adjust_steppers(adjustments, speed)
        if self.ad_phase < 6:
            self.ad_phase += 1
            return "retry"
        z_pos = _estimate_pivot_point(self.ad_params, offsets)
        self.ad_gcmd.respond_info(
            "current estimated z_positions %s" % (_format_positions(z_pos))
        )
        self.ad_runs.append(z_pos)
        self.z_positions = _average_runs(self.ad_runs, avlen)
        self.apply_adjustments(offsets, self.ad_params[0])
        if len(self.ad_runs) >= avlen:
            error = _check_convergence(self.ad_runs, avlen).item()
            if self.ad_error is None:
                self.ad_gcmd.respond_info("current error: %.6f" % (error))
            else:
                self.ad_gcmd.respond_info(
                    "previous error: %.6f current error: %.6f"
                    % (self.ad_error, error)
                )
            if self.ad_error is not None:
                if error >= self.ad_error:
                    self.ad_finalize_done(offsets)
                    return
            self.ad_error = error
        self.ad_init()
        return "retry"

    def ad_finalize_done(self, offsets):
        avlen = self.cal_avg_len
        z_offsets = _average_runs(self.ad_points, avlen)
        z_offsets = [z - offsets[2] for z in z_offsets]
        self.z_offsets = z_offsets
        logging.info("final z_offsets %s", (z_offsets))
        configfile = self.printer.lookup_object("configfile")
        section = self.section
        s_zoff = _format_offsets(z_offsets)
        configfile.set(section, "z_offsets", s_zoff)
        s_zpos = _format_positions(self.z_positions)
        configfile.set(section, "z_positions", s_zpos)
        self.ad_gcmd.respond_info("final z_positions are %s" % (s_zpos))
        self.ad_gcmd.respond_info("final z_offsets are: %s" % (s_zoff))
        self.ad_gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config\n"
            "file with these parameters and restart the printer."
        )

    def get_status(self, eventtime):
        return self.z_status.get_status(eventtime)


def load_config(config):
    return ZTilt(config)
