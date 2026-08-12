# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

_OPTIONS_WITHOUT_A_PLANNER_CONCEPT = (
    "max_extrude_cross_section",
    "max_extrude_only_distance",
    "instantaneous_corner_velocity",
)


# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.0
        self._reject_unsupported_options(config)
        # Setup hotend heater
        pheaters = self.printer.load_object(config, "heaters")
        gcode_id = "T%d" % (extruder_num,)
        self.heater = pheaters.setup_heater(config, gcode_id)
        self.nozzle_diameter = config.getfloat("nozzle_diameter", above=0.0)
        self.filament_diameter = config.getfloat(
            "filament_diameter", minval=self.nozzle_diameter
        )
        self.max_extrude_only_velocity = config.getfloat(
            "max_extrude_only_velocity", None, above=0.0
        )
        self.max_extrude_only_accel = config.getfloat(
            "max_extrude_only_accel", None, above=0.0
        )
        self.trapq = None
        self.trapq_finalize_moves = lambda *a: None

        for stepper_key in (
            "step_pin",
            "dir_pin",
            "rotation_distance",
            "microsteps",
        ):
            if config.get(stepper_key, None) is not None:
                raise config.error(
                    "[%s]: stepper config is not supported here — move "
                    "step_pin/dir_pin/rotation_distance/microsteps to a "
                    "[<motor>] section and reference it from "
                    "[axis e] motors:" % self.name
                )
        axis_name = config.get("axis")
        self.motion = self.printer.lookup_object("motion", None)
        if self.motion is None:
            raise config.error(
                "[%s] axis: requires the [motion] object, which was not "
                "available at extruder load time" % self.name
            )
        declared = {
            n: follows for n, follows, motors, _pp in self.motion.axis_sections
        }
        if axis_name not in declared:
            raise config.error(
                "[%s] axis: '%s' is not a declared [axis %s] section"
                % (self.name, axis_name, axis_name)
            )
        if not declared[axis_name]:
            raise config.error(
                "[%s] axis: '%s' must be a follower axis "
                "(declare 'follows:' on [axis %s])"
                % (self.name, axis_name, axis_name)
            )
        self.axis_name = axis_name
        self.pa_compat = self.printer.lookup_object(
            "pressure_advance_compat", None
        )
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        if self.name == "extruder":
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.set_extruder(self, 0.0)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
            gcode.register_command("M302", self.cmd_M302)
        gcode.register_mux_command(
            "ACTIVATE_EXTRUDER",
            "EXTRUDER",
            self.name,
            self.cmd_ACTIVATE_EXTRUDER,
            desc=self.cmd_ACTIVATE_EXTRUDER_help,
        )

    def update_move_time(self, flush_time, clear_history_time):
        self.trapq_finalize_moves(self.trapq, flush_time, clear_history_time)

    def get_status(self, eventtime):
        sts = self.heater.get_status(eventtime)
        sts["can_extrude"] = self.heater.can_extrude
        if self.pa_compat is not None:
            sts.update(self.pa_compat.get_status_fields(self.name))
        return sts

    def get_name(self):
        return self.name

    def get_heater(self):
        return self.heater

    def get_trapq(self):
        return self.trapq

    def stats(self, eventtime):
        return self.heater.stats(eventtime)

    def _reject_unsupported_options(self, config):
        for key in _OPTIONS_WITHOUT_A_PLANNER_CONCEPT:
            if config.get(key, None) is not None:
                raise config.error(
                    "[%s] option '%s' is no longer supported: the planner "
                    "has no such concept" % (self.name, key)
                )

    def check_move(self, move):
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details"
            )

    def find_past_position(self, print_time):
        state = self.motion.engine.motion_state_at(
            self.motion.mcu, print_time=print_time, axis="e"
        )
        return state["e"][0]

    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float("S", 0.0)
        index = gcmd.get_int("T", None, minval=0)
        if index is not None:
            section = "extruder"
            if index:
                section = "extruder%d" % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.0:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object("toolhead").get_extruder()
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(extruder.get_heater(), temp, wait)

    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)

    def cmd_M302(self, gcmd):
        index = gcmd.get_int("T", None, minval=0)
        if index is not None:
            section = "extruder"
            if index:
                section = "extruder%d" % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                raise gcmd.error("Extruder%d not configured", (index,))
        else:
            extruder = self.printer.lookup_object("toolhead").get_extruder()
        heater = extruder.get_heater()
        cold_extrude = gcmd.get_int("P", None, minval=0, maxval=1)
        min_extrude_temp = gcmd.get_float(
            "S", None, minval=heater.min_temp, maxval=heater.max_temp
        )
        heater.set_cold_extrude(cold_extrude, min_extrude_temp)

    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"

    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")


# Dummy extruder class used when a printer has no extruder at all
class DummyExtruder:
    def __init__(self, printer):
        self.printer = printer

    def update_move_time(self, flush_time, clear_history_time):
        pass

    def check_move(self, move):
        raise move.move_error("Extrude when no extruder present")

    def find_past_position(self, print_time):
        return 0.0

    def get_name(self):
        return ""

    def get_heater(self):
        raise self.printer.command_error("Extruder not configured")

    def get_trapq(self):
        raise self.printer.command_error("Extruder not configured")


def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = "extruder"
        if i:
            section = "extruder%d" % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
