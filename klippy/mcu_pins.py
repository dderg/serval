# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from . import pins

# The maximum number of clock cycles an MCU is expected
# to schedule into the future, due to the protocol and firmware.
MAX_SCHEDULE_TICKS = (1 << 31) - 1
MIN_SCHEDULE_LEAD = 0.050


######################################################################
# Wrapper classes for MCU pins
######################################################################


class MCU_digital_out:
    def __init__(self, mcu, pin_params):
        self._printer = mcu.get_printer()
        self._mcu = mcu
        self._oid = None
        self._mcu.register_config_callback(self._build_config)
        self._pin = pin_params["pin"]
        self._invert = pin_params["invert"]
        self._start_value = self._shutdown_value = self._invert
        self._max_duration = 2.0
        self._last_clock = 0
        self._set_cmd = None

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration

    def setup_start_value(self, start_value, shutdown_value):
        self._start_value = (not not start_value) ^ self._invert
        self._shutdown_value = (not not shutdown_value) ^ self._invert

    def _build_config(self):
        if self._max_duration and self._start_value != self._shutdown_value:
            raise pins.error(
                "Pin with max duration must have start"
                " value equal to shutdown value"
            )
        mdur_ticks = self._mcu.seconds_to_clock(self._max_duration)
        if mdur_ticks > MAX_SCHEDULE_TICKS:
            raise pins.error("Digital pin max duration too large")
        self._mcu.request_move_queue_slot()
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_digital_out oid=%d pin=%s value=%d default_value=%d"
            " max_duration=%d"
            % (
                self._oid,
                self._pin,
                self._start_value,
                self._shutdown_value,
                mdur_ticks,
            )
        )
        self._mcu.add_config_cmd(
            "update_digital_out oid=%d value=%d"
            % (self._oid, self._start_value),
            on_restart=True,
        )
        self._set_cmd = self._mcu.lookup_command(
            "queue_digital_out oid=%c clock=%u on_ticks=%u"
        )

    def set_digital(self, print_time, value):
        if self._mcu.non_critical_disconnected:
            raise self._printer.command_error(
                f"Cannot set pin on disconnected MCU '{self._mcu.get_name()}'"
            )
        est = self._mcu.estimated_print_time(
            self._printer.get_reactor().monotonic()
        )
        if print_time < est + MIN_SCHEDULE_LEAD:
            raise self._printer.command_error(
                "digital_out %s on mcu '%s' scheduled with stale print_time:"
                " print_time=%.6f estimated_now=%.6f lead=%.1fms (< %.0fms)"
                % (
                    self._pin,
                    self._mcu.get_name(),
                    print_time,
                    est,
                    (print_time - est) * 1000.0,
                    MIN_SCHEDULE_LEAD * 1000.0,
                )
            )
        clock = self._mcu.print_time_to_clock(print_time)
        self._set_cmd.send(
            [self._oid, clock, (not not value) ^ self._invert],
            minclock=self._last_clock,
            reqclock=clock,
        )
        self._last_clock = clock


class MCU_pwm:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._hardware_pwm = False
        self._cycle_time = 0.100
        self._max_duration = 2.0
        self._oid = None
        self._mcu.register_config_callback(self._build_config)
        self._pin = pin_params["pin"]
        self._invert = pin_params["invert"]
        self._start_value = self._shutdown_value = float(self._invert)
        self._last_clock = 0
        self._pwm_max = 0.0
        self._set_cmd = None

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self._cycle_time = cycle_time
        self._hardware_pwm = hardware_pwm

    def setup_start_value(self, start_value, shutdown_value):
        if self._invert:
            start_value = 1.0 - start_value
            shutdown_value = 1.0 - shutdown_value
        self._start_value = max(0.0, min(1.0, start_value))
        self._shutdown_value = max(0.0, min(1.0, shutdown_value))

    def _build_config(self):
        if self._max_duration and self._start_value != self._shutdown_value:
            raise pins.error(
                "Pin with max duration must have start"
                " value equal to shutdown value"
            )
        curtime = self._mcu.get_printer().get_reactor().monotonic()
        printtime = self._mcu.estimated_print_time(curtime)
        self._last_clock = self._mcu.print_time_to_clock(printtime + 0.200)
        cycle_ticks = self._mcu.seconds_to_clock(self._cycle_time)
        mdur_ticks = self._mcu.seconds_to_clock(self._max_duration)
        if mdur_ticks > MAX_SCHEDULE_TICKS:
            raise pins.error("PWM pin max duration too large")
        if self._hardware_pwm:
            self._pwm_max = self._mcu.get_constant_float("PWM_MAX")
            self._mcu.request_move_queue_slot()
            self._oid = self._mcu.create_oid()
            self._mcu.add_config_cmd(
                "config_pwm_out oid=%d pin=%s cycle_ticks=%d value=%d"
                " default_value=%d max_duration=%d"
                % (
                    self._oid,
                    self._pin,
                    cycle_ticks,
                    self._start_value * self._pwm_max,
                    self._shutdown_value * self._pwm_max,
                    mdur_ticks,
                )
            )
            svalue = int(self._start_value * self._pwm_max + 0.5)
            self._mcu.add_config_cmd(
                "queue_pwm_out oid=%d clock=%d value=%d"
                % (self._oid, self._last_clock, svalue),
                on_restart=True,
            )
            self._set_cmd = self._mcu.lookup_command(
                "queue_pwm_out oid=%c clock=%u value=%hu"
            )
            return
        # Software PWM
        if self._shutdown_value not in [0.0, 1.0]:
            raise pins.error("shutdown value must be 0.0 or 1.0 on soft pwm")
        if cycle_ticks > MAX_SCHEDULE_TICKS:
            raise pins.error("PWM pin cycle time too large")
        self._mcu.request_move_queue_slot()
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_digital_out oid=%d pin=%s value=%d"
            " default_value=%d max_duration=%d"
            % (
                self._oid,
                self._pin,
                self._start_value >= 1.0,
                self._shutdown_value >= 0.5,
                mdur_ticks,
            )
        )
        self._mcu.add_config_cmd(
            "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d"
            % (self._oid, cycle_ticks)
        )
        self._pwm_max = float(cycle_ticks)
        svalue = int(self._start_value * cycle_ticks + 0.5)
        self._mcu.add_config_cmd(
            "queue_digital_out oid=%d clock=%d on_ticks=%d"
            % (self._oid, self._last_clock, svalue),
            is_init=True,
        )
        self._set_cmd = self._mcu.lookup_command(
            "queue_digital_out oid=%c clock=%u on_ticks=%u"
        )

    def set_pwm(self, print_time, value):
        if self._invert:
            value = 1.0 - value
        v = int(max(0.0, min(1.0, value)) * self._pwm_max + 0.5)
        clock = self._mcu.print_time_to_clock(print_time)
        self._set_cmd.send(
            [self._oid, clock, v], minclock=self._last_clock, reqclock=clock
        )
        self._last_clock = clock


class MCU_adc:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._pin = pin_params["pin"]
        self._min_sample = self._max_sample = 0.0
        self._sample_time = self._report_time = 0.0
        self._sample_count = self._range_check_count = 0
        self._report_clock = 0
        self._last_state = (0.0, 0.0)
        self._oid = self._callback = None
        self._mcu.register_config_callback(self._build_config)
        self._inv_max_adc = 0.0

    def get_mcu(self):
        return self._mcu

    def setup_minmax(
        self,
        sample_time,
        sample_count,
        minval=0.0,
        maxval=1.0,
        range_check_count=0,
    ):
        self._sample_time = sample_time
        self._sample_count = sample_count
        self._min_sample = minval
        self._max_sample = maxval
        self._range_check_count = range_check_count

    def setup_adc_callback(self, report_time, callback):
        self._report_time = report_time
        self._callback = callback

    def get_last_value(self):
        return self._last_state

    def _build_config(self):
        if not self._sample_count:
            return
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_analog_in oid=%d pin=%s" % (self._oid, self._pin)
        )
        clock = self._mcu.get_query_slot(self._oid)
        sample_ticks = self._mcu.seconds_to_clock(self._sample_time)
        mcu_adc_max = self._mcu.get_constant_float("ADC_MAX")
        max_adc = self._sample_count * mcu_adc_max
        self._inv_max_adc = 1.0 / max_adc
        self._report_clock = self._mcu.seconds_to_clock(self._report_time)
        min_sample = max(0, min(0xFFFF, int(self._min_sample * max_adc)))
        max_sample = max(
            0, min(0xFFFF, int(math.ceil(self._max_sample * max_adc)))
        )
        self._mcu.add_config_cmd(
            "query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d"
            " rest_ticks=%d min_value=%d max_value=%d range_check_count=%d"
            % (
                self._oid,
                clock,
                sample_ticks,
                self._sample_count,
                self._report_clock,
                min_sample,
                max_sample,
                self._range_check_count,
            ),
            is_init=True,
        )
        self._mcu.register_response(
            self._handle_analog_in_state, "analog_in_state", self._oid
        )

    def _handle_analog_in_state(self, params):
        last_value = params["value"] * self._inv_max_adc
        next_clock = self._mcu.clock32_to_clock64(params["next_clock"])
        last_read_clock = next_clock - self._report_clock
        last_read_time = self._mcu.clock_to_print_time(last_read_clock)
        self._last_state = (last_value, last_read_time)
        if self._callback is not None:
            self._callback(last_read_time, last_value)
