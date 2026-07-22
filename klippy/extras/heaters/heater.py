# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import threading

from .control import (
    CONTROL_ALGOS,
    PID_PARAM_BASE,
    ControlDualLoopPID,
    ControlPID,
    ControlVelocityPID,
)
from .profiles import ProfileManager

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 3.0
MAX_MAINTHREAD_TIME = 5.0
QUELL_STALE_TIME = 7.0


class Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = short_name = self.name.split()[-1]
        self.reactor = self.printer.get_reactor()
        self.config = config
        self.configfile = self.printer.lookup_object("configfile")
        # Setup sensor
        self.sensor = sensor
        self.mpc_sensors = []
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            "min_extrude_temp",
            170.0,
            minval=self.min_temp,
            maxval=self.max_temp,
        )
        is_fileoutput = (
            self.printer.get_start_args().get("debugoutput") is not None
        )
        self.can_extrude = self.min_extrude_temp <= 0.0 or is_fileoutput
        self.cold_extrude = False
        self.max_power = config.getfloat(
            "max_power", 1.0, above=0.0, maxval=1.0
        )
        self.config_smooth_time = config.getfloat("smooth_time", 1.0, above=0.0)
        self.smooth_time = self.config_smooth_time
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.verify_mainthread_time = -999.0
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.0
        self.last_temp_time = 0.0
        # pwm caching
        self.next_pwm_time = 0.0
        self.last_pwm_value = 0.0
        # Those are necessary so the klipper config check does not complain
        config.get("control", None)
        config.getfloat("pid_kp", None)
        config.getfloat("pid_ki", None)
        config.getfloat("pid_kd", None)
        config.getfloat("max_delta", None)
        # Setup output heater pin
        heater_pin = config.get("heater_pin")
        ppins = self.printer.lookup_object("pins")
        self.mcu_pwm = ppins.setup_pin("pwm", heater_pin)
        pwm_cycle_time = config.getfloat(
            "pwm_cycle_time", 0.100, above=0.0, maxval=self.pwm_delay
        )
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (short_name,))
        self.printer.load_object(config, "pid_calibrate")
        self.gcode = self.printer.lookup_object("gcode")
        self.pmgr = ProfileManager(self)
        self.control = self.lookup_control(
            self.pmgr.init_default_profile(), True
        )
        self.gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE",
            "HEATER",
            short_name,
            self.cmd_SET_HEATER_TEMPERATURE,
            desc=self.cmd_SET_HEATER_TEMPERATURE_help,
        )
        self.gcode.register_mux_command(
            "COLD_EXTRUDE",
            "HEATER",
            self.name,
            self.cmd_COLD_EXTRUDE,
            desc=self.cmd_COLD_EXTRUDE_help,
        )
        self.gcode.register_mux_command(
            "SET_SMOOTH_TIME",
            "HEATER",
            short_name,
            self.cmd_SET_SMOOTH_TIME,
            desc=self.cmd_SET_SMOOTH_TIME_help,
        )
        self.gcode.register_mux_command(
            "PID_PROFILE",
            "HEATER",
            short_name,
            self.pmgr.cmd_PID_PROFILE,
            desc=self.pmgr.cmd_PID_PROFILE_help,
        )
        self.gcode.register_mux_command(
            "SET_HEATER_PID",
            "HEATER",
            short_name,
            self.cmd_SET_HEATER_PID,
            desc=self.cmd_SET_HEATER_PID_help,
        )

        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

    def lookup_control(self, profile, load_clean=False):
        return CONTROL_ALGOS[profile["control"]](profile, self, load_clean)

    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.0 or read_time > self.verify_mainthread_time:
            value = 0.0
        if (read_time < self.next_pwm_time or not self.last_pwm_value) and abs(
            value - self.last_pwm_value
        ) < 0.05:
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = (
            pwm_time + MAX_HEAT_TIME - (3.0 * self.pwm_delay + 0.001)
        )
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)

    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (
                self.smoothed_temp >= self.min_extrude_temp or self.cold_extrude
            )
        for mpc_sensor in self.mpc_sensors:
            mpc_sensor.process_temp_update(self.get_control(), read_time)

    def _handle_shutdown(self):
        self.verify_mainthread_time = -999.0

    # External commands
    def get_name(self):
        return self.name

    def add_mpc_sensor(self, mpc_sensor):
        self.mpc_sensors.append(mpc_sensor)

    def get_pwm_delay(self):
        return self.pwm_delay

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def set_inv_smooth_time(self, inv_smooth_time):
        self.inv_smooth_time = inv_smooth_time

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        with self.lock:
            if degrees != 0.0:
                check_valid = getattr(self.control, "check_valid", None)
                if check_valid is not None:
                    check_valid()
            self.target_temp = degrees

    def get_temp(self, eventtime):
        est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        quell_time = est_print_time - QUELL_STALE_TIME
        with self.lock:
            if self.last_temp_time < quell_time:
                return 0.0, self.target_temp
            return self.smoothed_temp, self.target_temp

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp
            )

    def set_control(self, control, keep_target=True):
        with self.lock:
            old_control = self.control
            self.control = control
            if not keep_target:
                self.target_temp = 0.0
        return old_control

    def get_control(self):
        return self.control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp

    def stats(self, eventtime):
        est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        if not self.printer.is_shutdown():
            self.verify_mainthread_time = est_print_time + MAX_MAINTHREAD_TIME
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.0
        return is_active, "%s: target=%.0f temp=%.1f pwm=%.3f" % (
            self.short_name,
            target_temp,
            last_temp,
            last_pwm_value,
        )

    def get_status(self, eventtime):
        control_stats = None
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
            get_control_status = getattr(self.control, "get_status", None)
            if get_control_status is not None:
                control_stats = get_control_status(eventtime)
        ret = {
            "temperature": round(smoothed_temp, 2),
            "target": target_temp,
            "power": last_pwm_value,
            "pid_profile": self.get_control().get_profile()["name"],
        }
        if control_stats is not None:
            ret["control_stats"] = control_stats
        return ret

    def is_adc_faulty(self):
        if self.last_temp > self.max_temp or self.last_temp < self.min_temp:
            return True
        return False

    def set_cold_extrude(self, cold_extrude, min_extrude_temp):
        if cold_extrude is None and min_extrude_temp is None:
            self.gcode.respond_info(
                "Cold extrudes are %s (min temp %.2fC)"
                % (
                    "enabled" if self.cold_extrude else "disabled",
                    self.min_extrude_temp,
                )
            )
            return
        self.cold_extrude = True if cold_extrude else False
        if min_extrude_temp is not None:
            self.min_extrude_temp = min_extrude_temp
            self.configfile.set(
                self.name, "min_extrude_temp", self.min_extrude_temp
            )
            self.gcode.respond_info(
                "min_extrude_temp has been set to %.2fC "
                "for [%s] for the current session.\n"
                "The SAVE_CONFIG command will update the "
                "printer config file and restart the "
                "printer." % (self.min_extrude_temp, self.name)
            )
        self.can_extrude = (
            self.smoothed_temp >= self.min_extrude_temp or self.cold_extrude
        )

    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float("TARGET", 0.0)
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self, temp)

    cmd_COLD_EXTRUDE_help = "Control cold extrusions"

    def cmd_COLD_EXTRUDE(self, gcmd):
        cold_extrude = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        min_extrude_temp = gcmd.get_float(
            "MIN_EXTRUDE_TEMP", None, minval=self.min_temp, maxval=self.max_temp
        )
        self.set_cold_extrude(cold_extrude, min_extrude_temp)

    cmd_SET_SMOOTH_TIME_help = "Set the smooth time for the given heater"

    def cmd_SET_SMOOTH_TIME(self, gcmd):
        save_to_profile = gcmd.get_int("SAVE_TO_PROFILE", 0, minval=0, maxval=1)
        self.smooth_time = gcmd.get_float(
            "SMOOTH_TIME", self.config_smooth_time, above=0.0
        )
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.get_control().update_smooth_time()
        if save_to_profile:
            self.get_control().get_profile()["smooth_time"] = self.smooth_time
            self.pmgr.save_profile()

    cmd_SET_HEATER_PID_help = "Sets a heater PID parameter"

    def cmd_SET_HEATER_PID(self, gcmd):
        if not isinstance(self.control, (ControlPID, ControlVelocityPID)):
            raise gcmd.error("Not a PID/PID_V controlled heater")
        kp = gcmd.get_float("KP", None)
        if kp is not None:
            self.control.Kp = kp / PID_PARAM_BASE
        ki = gcmd.get_float("KI", None)
        if ki is not None:
            self.control.Ki = ki / PID_PARAM_BASE
        kd = gcmd.get_float("KD", None)
        if kd is not None:
            self.control.Kd = kd / PID_PARAM_BASE


class DualSensorHeater(Heater):
    def __init__(self, config, primary_sensor, secondary_sensor):
        super().__init__(config=config, sensor=primary_sensor)
        self.secondary_sensor = secondary_sensor

        if (
            isinstance(self.control, ControlDualLoopPID)
            and self.secondary_sensor is None
        ):
            raise config.error("dual_loop_pid requires a secondary sensor")

    def temperature_callback(self, read_time, primary_temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = primary_temp
            self.last_temp_time = read_time

            secondary_status = self.secondary_sensor.get_status(read_time)
            secondary_temp = secondary_status["temperature"]

            self.control.temperature_update(
                read_time, primary_temp, self.target_temp, secondary_temp
            )

            temp_diff = primary_temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (
                self.smoothed_temp >= self.min_extrude_temp or self.cold_extrude
            )
