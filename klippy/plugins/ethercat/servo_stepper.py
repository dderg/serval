# A servo axis in place of a stepper: same kinematics, sampled positions
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math

from klippy import chelper, stepper

from .stream import EthercatError, round_half_away

UNIT_COORDS = (
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
LINEARITY_PROBES = ((37.0, -11.0, 5.0), (-123.0, 71.0, -29.0))
LINEARITY_REL_TOLERANCE = 1e-9

SAMPLE_ERRORS = {
    -1: "no stepper kinematics or trapq",
    -2: "sample time precedes the retained trapq history",
    -3: "kinematic position disagrees with the shaped linear model",
    -4: "non-finite sample",
}


class NonlinearKinematics(Exception):
    pass


def probe_linear_map(sk):
    """Return (offset, row) with position = offset + row . (x, y, z)."""
    ffi_main, ffi_lib = chelper.get_ffi()

    def pos(coord):
        return ffi_lib.itersolve_calc_position_from_coord(sk, *coord)

    origin, ex, ey, ez = (pos(c) for c in UNIT_COORDS)
    row = (ex - origin, ey - origin, ez - origin)
    for probe in LINEARITY_PROBES:
        predicted = origin + sum(r * c for r, c in zip(row, probe))
        actual = pos(probe)
        if not (
            math.isfinite(actual)
            and abs(predicted - actual)
            <= LINEARITY_REL_TOLERANCE * max(1.0, abs(actual))
        ):
            raise NonlinearKinematics(
                "position at %r is %r, a linear map predicts %r"
                % (probe, actual, predicted)
            )
    return origin, row


class ServoStepper:
    """MCU_stepper look-alike for an EtherCAT servo.

    It owns a regular itersolve stepper_kinematics on the toolhead trapq,
    so kinematics, input_shaper and motion_queuing treat it as a stepper.
    Instead of steps it emits the commanded motion sampled on the DC grid.
    """

    def __init__(self, config, units_in_radians):
        printer = config.get_printer()
        self._name = config.get_name()
        self._units_in_radians = units_in_radians
        self.slave_position = config.getint("ethercat_slave", minval=0)
        self.axis_index = config.getint("ethercat_axis", minval=0, maxval=255)
        counts_per_rev = config.getint("encoder_counts_per_rev", minval=1)
        if units_in_radians:
            self._rotation_dist = 2.0 * math.pi
            config.get("gear_ratio")
        else:
            self._rotation_dist = config.getfloat(
                "rotation_distance", above=0.0
            )
        gearing = stepper.parse_gear_ratio(config, True)
        self._counts_per_rotation = counts_per_rev * gearing
        self._step_dist = self._rotation_dist / self._counts_per_rotation
        self._invert_dir = config.getboolean("invert_dir", False)
        self._dir_sign = -1 if self._invert_dir else 1
        self._mcu = printer.lookup_object("mcu")
        motion_queuing = printer.load_object(config, "motion_queuing")
        self._syncemitter = motion_queuing.allocate_syncemitter(
            self._mcu, self._name.split()[-1], alloc_stepcompress=False
        )
        ffi_main, ffi_lib = chelper.get_ffi()
        self._sampler = ffi_main.gc(
            ffi_lib.servo_sampler_alloc(), ffi_lib.servo_sampler_free
        )
        self._sampler_sk = None
        self._stepper_kinematics = None
        self._trapq = ffi_main.NULL
        self._mcu_position_offset = 0.0
        self._active_callbacks = []
        self._itersolve_check_active = ffi_lib.itersolve_check_active

    def get_mcu(self):
        return self._mcu

    def get_name(self, short=False):
        if short and self._name.startswith("stepper_"):
            return self._name[8:]
        return self._name

    def units_in_radians(self):
        return self._units_in_radians

    def get_tmc_current_helper(self):
        return None

    def get_step_dist(self):
        return self._step_dist

    def get_rotation_distance(self):
        return self._rotation_dist, self._counts_per_rotation

    def get_dir_inverted(self):
        return self._invert_dir, self._invert_dir

    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        try:
            probe_linear_map(sk)
        except NonlinearKinematics as e:
            raise self._mcu.get_printer().config_error(
                "%s: EtherCAT servos need kinematics linear in x, y, z"
                " (%s uses %s: %s)" % (self._name, self._name, alloc_func, e)
            )
        self.set_stepper_kinematics(sk)

    def calc_position_from_coord(self, coord):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2]
        )

    def set_position(self, coord):
        mcu_pos = self.get_mcu_position()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_position(
            self._stepper_kinematics, coord[0], coord[1], coord[2]
        )
        self._set_mcu_position(mcu_pos)

    def get_commanded_position(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_get_commanded_pos(self._stepper_kinematics)

    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is None:
            cmd_pos = self.get_commanded_position()
        return round_half_away(
            (cmd_pos + self._mcu_position_offset) / self._step_dist
        )

    def _set_mcu_position(self, mcu_pos):
        mcu_pos_dist = mcu_pos * self._step_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist - self._mcu_position_offset

    def dump_steps(self, count, start_clock, end_clock):
        return [], 0

    def get_stepper_kinematics(self):
        return self._stepper_kinematics

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.syncemitter_set_stepper_kinematics(self._syncemitter, sk)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk

    def get_trapq(self):
        return self._trapq

    def set_trapq(self, tq):
        ffi_main, ffi_lib = chelper.get_ffi()
        if tq is None:
            tq = ffi_main.NULL
        ffi_lib.itersolve_set_trapq(
            self._stepper_kinematics, tq, self._step_dist
        )
        old_tq = self._trapq
        self._trapq = tq
        return old_tq

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
        if len(self._active_callbacks) == 1:
            motion_queuing = self._mcu.get_printer().lookup_object(
                "motion_queuing"
            )
            motion_queuing.register_flush_callback(self._check_active)

    def _check_active(self, must_flush_time, max_step_gen_time):
        ret = self._itersolve_check_active(
            self._stepper_kinematics, max_step_gen_time
        )
        if not ret:
            return
        motion_queuing = self._mcu.get_printer().lookup_object("motion_queuing")
        motion_queuing.unregister_flush_callback(self._check_active)
        cbs = self._active_callbacks
        self._active_callbacks = []
        for cb in cbs:
            cb(ret)

    def is_active_axis(self, axis):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_is_active_axis(
            self._stepper_kinematics, axis.encode()
        )

    # Drive units
    def drive_counts(self, pos):
        return self._dir_sign * self.get_mcu_position(pos)

    def drive_velocity(self, vel):
        return round_half_away(self._dir_sign * vel / self._step_dist)

    # Sampling
    def _sync_sampler_kinematics(self):
        sk = self._stepper_kinematics
        if self._sampler_sk == sk:
            return
        try:
            offset, row = probe_linear_map(sk)
        except NonlinearKinematics as e:
            raise EthercatError("%s: %s" % (self._name, e))
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.servo_sampler_set_kinematics(self._sampler, sk, offset, *row)
        self._sampler_sk = sk

    def sample(self, start_time, interval, count):
        """Commanded position, velocity, acceleration (stepper units) at
        start_time + i * interval for i in range(count)."""
        self._sync_sampler_kinematics()
        ffi_main, ffi_lib = chelper.get_ffi()
        pos = ffi_main.new("double[]", count)
        vel = ffi_main.new("double[]", count)
        acc = ffi_main.new("double[]", count)
        err = ffi_main.new("struct servo_sample_error *")
        ret = ffi_lib.servo_sampler_generate(
            self._sampler, start_time, interval, count, pos, vel, acc, err
        )
        if ret:
            raise EthercatError(
                "%s: %s at print_time %.9f (position %r, linear model %r)"
                % (
                    self._name,
                    SAMPLE_ERRORS[ret],
                    err.time,
                    err.position,
                    err.linear_position,
                )
            )
        return pos, vel, acc
