# PWM heater control algorithms
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from ..control_mpc import ControlMPC

AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class HeaterControl:
    control_type = None

    def __init__(self, profile, heater):
        self.profile = profile
        self.heater = heater
        self.heater_max_power = heater.get_max_power()

    def get_profile(self):
        return self.profile

    def get_type(self):
        return self.control_type

    def update_smooth_time(self):
        self.smooth_time = self.heater.get_smooth_time()


class ControlBangBang(HeaterControl):
    control_type = "watermark"

    def __init__(self, profile, heater, load_clean=False):
        super().__init__(profile, heater)
        self.max_delta = profile["max_delta"]
        self.heating = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.heating and temp >= target_temp + self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp - self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.0)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp - self.max_delta


class ControlPID(HeaterControl):
    control_type = "pid"

    def __init__(self, profile, heater, load_clean=False):
        super().__init__(profile, heater)
        self.Kp = profile["pid_kp"] / PID_PARAM_BASE
        self.Ki = profile["pid_ki"] / PID_PARAM_BASE
        self.Kd = profile["pid_kd"] / PID_PARAM_BASE
        self.min_deriv_time = (
            self.heater.get_smooth_time()
            if profile["smooth_time"] is None
            else profile["smooth_time"]
        )
        self.heater.set_inv_smooth_time(1.0 / self.min_deriv_time)
        self.temp_integ_max = 0.0
        if self.Ki:
            self.temp_integ_max = self.heater_max_power / self.Ki
        self.prev_temp = (
            AMBIENT_TEMP
            if load_clean
            else self.heater.get_temp(self.heater.reactor.monotonic())[0]
        )
        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0

    def calculate_output(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff)
                + temp_diff
            ) / self.min_deriv_time
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
        return co, bounded_co

    def temperature_update(self, read_time, temp, target_temp):
        _, bounded_co = self.calculate_output(read_time, temp, target_temp)
        self.heater.set_pwm(read_time, bounded_co)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA
            or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE
        )


class ControlVelocityPID(HeaterControl):
    control_type = "pid_v"

    def __init__(self, profile, heater, load_clean=False):
        super().__init__(profile, heater)
        self.Kp = profile["pid_kp"] / PID_PARAM_BASE
        self.Ki = profile["pid_ki"] / PID_PARAM_BASE
        self.Kd = profile["pid_kd"] / PID_PARAM_BASE
        smooth_time = (
            self.heater.get_smooth_time()
            if profile["smooth_time"] is None
            else profile["smooth_time"]
        )
        self.heater.set_inv_smooth_time(1.0 / smooth_time)
        self.smooth_time = smooth_time
        self.temps = (
            ([AMBIENT_TEMP] * 3)
            if load_clean
            else (
                [self.heater.get_temp(self.heater.reactor.monotonic())[0]] * 3
            )
        )
        self.times = [0.0] * 3
        self.d1 = 0.0
        self.d2 = 0.0
        self.pwm = 0.0 if load_clean else self.heater.last_pwm_value

    def temperature_update(self, read_time, temp, target_temp):
        self.temps.pop(0)
        self.temps.append(temp)
        self.times.pop(0)
        self.times.append(read_time)

        # Derivatives are of the temp, not the error, to prevent
        # derivative kick
        d1 = self.temps[-1] - self.temps[-2]

        error = self.times[-1] - self.times[-2]
        error = error * (target_temp - self.temps[-1])

        d2 = self.temps[-1] - 2.0 * self.temps[-2] + self.temps[-3]
        d2 = d2 / (self.times[-1] - self.times[-2])

        # Modified moving average that handles unevenly spaced points
        n = max(1.0, self.smooth_time / (self.times[-1] - self.times[-2]))
        self.d1 = ((n - 1.0) * self.d1 + d1) / n
        self.d2 = ((n - 1.0) * self.d2 + d2) / n

        p = self.Kp * -self.d1
        i = self.Ki * error
        d = self.Kd * -self.d2

        self.pwm = max(0.0, min(self.heater_max_power, self.pwm + p + i + d))
        if target_temp == 0.0:
            self.pwm = 0.0

        self.heater.set_pwm(read_time, self.pwm)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA or abs(self.d1) > PID_SETTLE_SLOPE
        )


class ControlInnerPID(ControlPID):
    def __init__(self, profile, heater, load_clean=False):
        super().__init__(profile, heater, load_clean)
        self.Kp = profile["inner_pid_kp"] / PID_PARAM_BASE
        self.Ki = profile["inner_pid_ki"] / PID_PARAM_BASE
        self.Kd = profile["inner_pid_kd"] / PID_PARAM_BASE
        if self.Ki:
            self.temp_integ_max = self.heater_max_power / self.Ki


class ControlDualLoopPID(HeaterControl):
    control_type = "dual_loop_pid"

    def __init__(self, profile, heater, load_clean=False):
        super().__init__(profile, heater)
        # Outer (primary) loop, e.g. bed surface; inner (secondary)
        # loop, e.g. heater element
        self.primary_pid = ControlPID(profile, heater, load_clean)
        self.secondary_pid = ControlInnerPID(profile, heater, load_clean)
        self.secondary_max_temp = self.heater.config.getfloat("inner_max_temp")

    def temperature_update(
        self, read_time, primary_temp, target_temp, secondary_temp
    ):
        if secondary_temp is None:
            raise ValueError("Secondary temperature must be provided!")
        primary_co, _ = self.primary_pid.calculate_output(
            read_time, primary_temp, target_temp
        )
        secondary_co, _ = self.secondary_pid.calculate_output(
            read_time, secondary_temp, self.secondary_max_temp
        )
        co = min(primary_co, secondary_co)
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.primary_pid.check_busy(
            eventtime, smoothed_temp, target_temp
        )


CONTROL_ALGOS = {
    "watermark": ControlBangBang,
    "pid": ControlPID,
    "pid_v": ControlVelocityPID,
    "mpc": ControlMPC,
    "dual_loop_pid": ControlDualLoopPID,
}
