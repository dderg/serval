# Persistent storage of heater control profiles
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from ..control_mpc import (
    FILAMENT_TEMP_SRC_AMBIENT,
    FILAMENT_TEMP_SRC_FIXED,
    FILAMENT_TEMP_SRC_SENSOR,
)

PID_PROFILE_VERSION = 1
PID_PROFILE_OPTIONS = {
    "pid_target": (float, "%.2f"),
    "pid_tolerance": (float, "%.4f"),
    "control": (str, "%s"),
    "smooth_time": (float, "%.3f"),
    "pid_kp": (float, "%.3f"),
    "pid_ki": (float, "%.3f"),
    "pid_kd": (float, "%.3f"),
}
INNER_PID_OPTIONS = {
    "inner_pid_kp": (float, "%.3f"),
    "inner_pid_ki": (float, "%.3f"),
    "inner_pid_kd": (float, "%.3f"),
}


def _load_watermark_profile(pmgr, config_section, name):
    return {"max_delta": config_section.getfloat("max_delta", 2.0, above=0.0)}


def _load_pid_profile(pmgr, config_section, name):
    profile = {}
    for key, (option_type, _) in PID_PROFILE_OPTIONS.items():
        can_be_none = key not in ("pid_kp", "pid_ki", "pid_kd")
        profile[key] = pmgr._check_value_config(
            key, config_section, option_type, can_be_none
        )
    if name == "default":
        profile["smooth_time"] = None
    return profile


def _load_dual_loop_profile(pmgr, config_section, name):
    profile = _load_pid_profile(pmgr, config_section, name)
    for key, (option_type, _) in INNER_PID_OPTIONS.items():
        profile[key] = pmgr._check_value_config(
            key, config_section, option_type, False
        )
    return profile


def _load_mpc_filament_temp_src(config_section):
    raw = config_section.get("filament_temperature_source", "ambient")
    value = raw.lower().strip()
    if value == "sensor":
        return (FILAMENT_TEMP_SRC_SENSOR,)
    if value == "ambient":
        return (FILAMENT_TEMP_SRC_AMBIENT,)
    try:
        fixed = float(value)
    except ValueError:
        raise config_section.error(
            "Unable to parse option 'filament_temperature_source' "
            "in section '%s'" % (config_section.get_name(),)
        )
    return (FILAMENT_TEMP_SRC_FIXED, fixed)


def _lookup_printer_object(config_section, object_name):
    printer = config_section.get_printer()
    obj = printer.load_object(config_section, object_name, None)
    if obj is None:
        obj = printer.lookup_object(object_name, None)
    return obj


def _load_mpc_ambient_sensor(config_section):
    sensor_name = config_section.get("ambient_temp_sensor", None)
    if sensor_name is None:
        return None
    sensor = _lookup_printer_object(config_section, sensor_name)
    if sensor is None:
        raise config_section.error(
            "Unknown ambient_temp_sensor '%s' specified" % (sensor_name,)
        )
    return sensor


def _load_mpc_cooling_fan(config_section):
    fan_name = config_section.get("cooling_fan", None)
    if fan_name is None:
        return None
    fan_obj = _lookup_printer_object(config_section, fan_name)
    if fan_obj is None:
        raise config_section.error(
            "Unknown part_cooling_fan '%s' specified" % (fan_name,)
        )
    if not hasattr(fan_obj, "fan") or not hasattr(fan_obj.fan, "set_speed"):
        raise config_section.error(
            "part_cooling_fan '%s' is not a valid fan object" % (fan_name,)
        )
    return fan_obj.fan


def _load_mpc_profile(pmgr, config_section, name):
    getfloat = config_section.getfloat
    return {
        "block_heat_capacity": getfloat(
            "block_heat_capacity", above=0.0, default=None
        ),
        "ambient_transfer": getfloat(
            "ambient_transfer", minval=0.0, default=None
        ),
        "target_reach_time": getfloat(
            "target_reach_time", above=0.0, default=2.0
        ),
        "smoothing": getfloat("smoothing", above=0.0, maxval=1.0, default=0.83),
        "heater_power": getfloat("heater_power", above=0.0),
        "sensor_responsiveness": getfloat(
            "sensor_responsiveness", above=0.0, default=None
        ),
        "min_ambient_change": getfloat(
            "min_ambient_change", above=0.0, default=1.0
        ),
        "steady_state_rate": getfloat(
            "steady_state_rate", above=0.0, default=0.5
        ),
        "filament_diameter": getfloat(
            "filament_diameter", above=0.0, default=1.75
        ),
        "filament_density": getfloat(
            "filament_density", above=0.0, default=1.2
        ),
        "filament_heat_capacity": getfloat(
            "filament_heat_capacity", above=0.0, default=1.8
        ),
        "maximum_retract": getfloat("maximum_retract", above=0.0, default=2.0),
        "filament_temp_src": _load_mpc_filament_temp_src(config_section),
        "ambient_temp_sensor": _load_mpc_ambient_sensor(config_section),
        "cooling_fan": _load_mpc_cooling_fan(config_section),
        "fan_ambient_transfer": config_section.getfloatlist(
            "fan_ambient_transfer", []
        ),
    }


PROFILE_LOADERS = {
    "watermark": _load_watermark_profile,
    "mpc": _load_mpc_profile,
    "pid": _load_pid_profile,
    "pid_v": _load_pid_profile,
    "dual_loop_pid": _load_dual_loop_profile,
}


def saved_profile_options(profile):
    control = profile["control"]
    options = dict(PID_PROFILE_OPTIONS)
    if control == "dual_loop_pid":
        options.update(INNER_PID_OPTIONS)
    elif control not in ("pid", "pid_v"):
        return None
    return options


class ProfileManager:
    def __init__(self, heater):
        self.heater = heater
        self.profiles = {}
        self.incompatible_profiles = []
        stored_profs = heater.config.get_prefix_sections(
            "pid_profile %s" % heater.short_name
        )
        for profile in stored_profs:
            self._init_profile(profile, profile.get_name().split(" ", 2)[-1])

    def init_default_profile(self):
        return self._init_profile(self.heater.config, "default")

    def _init_profile(self, config_section, name):
        version = config_section.getint("pid_version", 1)
        if version != PID_PROFILE_VERSION:
            logging.info(
                "Profile [%s] not compatible with this version "
                "of pid_profile.\n"
                "Profile Version: %d Current Version: %d"
                % (name, version, PID_PROFILE_VERSION)
            )
            self.incompatible_profiles.append(name)
            return None
        control = self._check_value_config(
            "control", config_section, str, False
        )
        loader = PROFILE_LOADERS.get(control)
        if loader is None:
            raise self.heater.printer.config_error(
                "Unknown control type '%s' in [pid_profile %s %s]."
                % (control, self.heater.short_name, name)
            )
        profile = loader(self, config_section, name)
        profile["control"] = control
        profile["name"] = name
        self.profiles[name] = profile
        return profile

    def _check_value_config(
        self, key, config_section, option_type, can_be_none
    ):
        if option_type is int:
            value = config_section.getint(key, None)
        elif option_type is float:
            value = config_section.getfloat(key, None)
        else:
            value = config_section.get(key, None)
        if not can_be_none and value is None:
            raise self.heater.gcode.error(
                "pid_profile: '%s' has to be "
                "specified in [pid_profile %s %s]."
                % (key, self.heater.short_name, config_section.get_name())
            )
        return value

    def _compute_section_name(self, profile_name):
        if profile_name == "default":
            return self.heater.short_name
        return "pid_profile %s %s" % (self.heater.short_name, profile_name)

    def _check_value_gcmd(
        self,
        name,
        default,
        gcmd,
        option_type,
        can_be_none,
        minval=None,
        maxval=None,
    ):
        if option_type is int:
            value = gcmd.get_int(name, default, minval=minval, maxval=maxval)
        elif option_type is float:
            value = gcmd.get_float(name, default, minval=minval, maxval=maxval)
        else:
            value = gcmd.get(name, default)
        if not can_be_none and value is None:
            raise self.heater.gcode.error(
                "pid_profile: '%s' has to be specified." % name
            )
        return value.lower() if option_type == "lower" else value

    def set_values(self, profile_name, gcmd, verbose):
        current_profile = self.heater.get_control().get_profile()
        target = self._check_value_gcmd("TARGET", None, gcmd, float, False)
        tolerance = self._check_value_gcmd(
            "TOLERANCE", current_profile["pid_tolerance"], gcmd, float, False
        )
        control = self._check_value_gcmd(
            "CONTROL", current_profile["control"], gcmd, "lower", False
        )
        kp = self._check_value_gcmd("KP", None, gcmd, float, False)
        ki = self._check_value_gcmd("KI", None, gcmd, float, False)
        kd = self._check_value_gcmd("KD", None, gcmd, float, False)
        smooth_time = self._check_value_gcmd(
            "SMOOTH_TIME", None, gcmd, float, True
        )
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        temp_profile = {
            "pid_target": target,
            "pid_tolerance": tolerance,
            "control": control,
            "smooth_time": smooth_time,
            "pid_kp": kp,
            "pid_ki": ki,
            "pid_kd": kd,
            "name": profile_name,
        }
        temp_control = self.heater.lookup_control(temp_profile, load_clean)
        self.heater.set_control(temp_control, keep_target)
        msg = (
            "PID Parameters:\n"
            "Target: %.2f,\n"
            "Tolerance: %.4f\n"
            "Control: %s\n" % (target, tolerance, control)
        )
        if smooth_time is not None:
            msg += "Smooth Time: %.3f\n" % smooth_time
        msg += (
            "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "have been set as current profile." % (kp, ki, kd)
        )
        self.heater.gcode.respond_info(msg)
        self.save_profile(profile_name=profile_name, verbose=True)

    def get_values(self, profile_name, gcmd, verbose):
        temp_profile = self.heater.get_control().get_profile()
        target = temp_profile["pid_target"]
        tolerance = temp_profile["pid_tolerance"]
        control = temp_profile["control"]
        kp = temp_profile["pid_kp"]
        ki = temp_profile["pid_ki"]
        kd = temp_profile["pid_kd"]
        smooth_time = (
            self.heater.get_smooth_time()
            if temp_profile["smooth_time"] is None
            else temp_profile["smooth_time"]
        )
        name = temp_profile["name"]
        self.heater.gcode.respond_info(
            "PID Parameters:\n"
            "Target: %.2f,\n"
            "Tolerance: %.4f\n"
            "Control: %s\n"
            "Smooth Time: %.3f\n"
            "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "name: %s"
            % (target, tolerance, control, smooth_time, kp, ki, kd, name)
        )

    def save_profile(self, profile_name=None, gcmd=None, verbose=True):
        temp_profile = self.heater.get_control().get_profile()
        options = saved_profile_options(temp_profile)
        if options is None:
            raise self.heater.gcode.error(
                "pid_profile: saving is not supported for control '%s'"
                % (temp_profile["control"],)
            )
        if profile_name is None:
            profile_name = temp_profile["name"]
        section_name = self._compute_section_name(profile_name)
        self.heater.configfile.set(
            section_name, "pid_version", PID_PROFILE_VERSION
        )
        for key, (_, placeholder) in options.items():
            value = temp_profile[key]
            if value is not None:
                self.heater.configfile.set(
                    section_name, key, placeholder % value
                )
        temp_profile["name"] = profile_name
        self.profiles[profile_name] = temp_profile
        if verbose:
            self.heater.gcode.respond_info(
                "Current PID profile for heater [%s] "
                "has been saved to profile [%s] "
                "for the current session.  The SAVE_CONFIG command will\n"
                "update the printer config file and restart the printer."
                % (self.heater.short_name, profile_name)
            )

    def load_profile(self, profile_name, gcmd, verbose):
        verbose = self._check_value_gcmd("VERBOSE", "low", gcmd, "lower", True)
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        current_name = self.heater.get_control().get_profile()["name"]
        if profile_name == current_name and not load_clean:
            if verbose == "high" or verbose == "low":
                self.heater.gcode.respond_info(
                    "PID Profile [%s] already loaded for heater [%s]."
                    % (profile_name, self.heater.short_name)
                )
            return
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        profile = self.profiles.get(profile_name, None)
        defaulted = False
        default = gcmd.get("DEFAULT", None)
        if profile is None:
            if default is None:
                raise self.heater.gcode.error(
                    "pid_profile: Unknown profile [%s] for heater [%s]."
                    % (profile_name, self.heater.short_name)
                )
            profile = self.profiles.get(default, None)
            defaulted = True
            if profile is None:
                raise self.heater.gcode.error(
                    "pid_profile: Unknown default "
                    "profile [%s] for heater [%s]."
                    % (default, self.heater.short_name)
                )
        control = self.heater.lookup_control(profile, load_clean)
        self.heater.set_control(control, keep_target)

        if verbose != "high" and verbose != "low":
            return
        if defaulted:
            self.heater.gcode.respond_info(
                "Couldn't find profile "
                "[%s] for heater [%s]"
                ", defaulted to [%s]."
                % (profile_name, self.heater.short_name, default)
            )
        self.heater.gcode.respond_info(
            "PID Profile [%s] loaded for heater [%s].\n"
            % (profile["name"], self.heater.short_name)
        )
        if verbose == "high":
            smooth_time = (
                self.heater.get_smooth_time()
                if profile["smooth_time"] is None
                else profile["smooth_time"]
            )
            msg = "Target: %.2f\nTolerance: %.4f\nControl: %s\n" % (
                profile["pid_target"],
                profile["pid_tolerance"],
                profile["control"],
            )
            if smooth_time is not None:
                msg += "Smooth Time: %.3f\n" % smooth_time
            msg += "PID Parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n" % (
                profile["pid_kp"],
                profile["pid_ki"],
                profile["pid_kd"],
            )
            self.heater.gcode.respond_info(msg)

    def remove_profile(self, profile_name, gcmd, verbose):
        if profile_name in self.profiles:
            section_name = self._compute_section_name(profile_name)
            self.heater.configfile.remove_section(section_name)
            profiles = dict(self.profiles)
            del profiles[profile_name]
            self.profiles = profiles
            self.heater.gcode.respond_info(
                "Profile [%s] for heater [%s] "
                "removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer"
                % (profile_name, self.heater.short_name)
            )
        else:
            self.heater.gcode.respond_info(
                "No profile named [%s] to remove" % profile_name
            )

    cmd_PID_PROFILE_help = "PID Profile Persistent Storage management"

    def cmd_PID_PROFILE(self, gcmd):
        options = {
            "LOAD": self.load_profile,
            "SAVE": self.save_profile,
            "GET_VALUES": self.get_values,
            "SET_VALUES": self.set_values,
            "REMOVE": self.remove_profile,
        }
        for key, handler in options.items():
            profile_name = gcmd.get(key, None)
            if profile_name is not None:
                if not profile_name.strip():
                    raise self.heater.gcode.error(
                        "pid_profile: Profile must be specified"
                    )
                handler(profile_name, gcmd, True)
                return
        raise self.heater.gcode.error(
            "pid_profile: Invalid syntax '%s'" % (gcmd.get_commandline(),)
        )
