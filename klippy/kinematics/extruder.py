# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math


# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.0
        # Setup hotend heater
        pheaters = self.printer.load_object(config, "heaters")
        gcode_id = "T%d" % (extruder_num,)
        self.heater = pheaters.setup_heater(config, gcode_id)
        # Setup kinematic checks
        self.nozzle_diameter = config.getfloat("nozzle_diameter", above=0.0)
        filament_diameter = config.getfloat(
            "filament_diameter", minval=self.nozzle_diameter
        )
        self.filament_area = math.pi * (filament_diameter * 0.5) ** 2
        def_max_cross_section = 4.0 * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            "max_extrude_cross_section", def_max_cross_section, above=0.0
        )
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object("toolhead")
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            "max_extrude_only_velocity",
            max_velocity * def_max_extrude_ratio,
            above=0.0,
        )
        self.max_e_accel = config.getfloat(
            "max_extrude_only_accel",
            max_accel * def_max_extrude_ratio,
            above=0.0,
        )
        self.max_e_dist = config.getfloat(
            "max_extrude_only_distance", 50.0, minval=0.0
        )
        self.instant_corner_v = config.getfloat(
            "instantaneous_corner_velocity", 1.0, minval=0.0
        )
        # Setup extruder trapq (trapezoidal motion queue). Bridge mode:
        # planner / kinematic state lives in Rust, the host stub is
        # callable but never queues real moves.
        self.trapq = None
        self.trapq_append = lambda *a: None
        self.trapq_finalize_moves = lambda *a: None

        # The E motor is an ordinary [<motor>] section referenced from
        # [axis e] motors:; [extruder] is heater-only.
        self.extruder_stepper = None
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
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        if self.name == "extruder":
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
        if self.extruder_stepper is not None:
            sts.update(self.extruder_stepper.get_status(eventtime))
        return sts

    def get_name(self):
        return self.name

    def get_heater(self):
        return self.heater

    def get_trapq(self):
        return self.trapq

    def stats(self, eventtime):
        return self.heater.stats(eventtime)

    def check_move(self, move):
        axis_r = move.axes_r[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details"
            )
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.0:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist)
                )
            inv_extrude_r = 1.0 / abs(axis_r)
            move.limit_speed(
                self.max_e_velocity * inv_extrude_r,
                self.max_e_accel * inv_extrude_r,
            )
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug(
                "Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                axis_r,
                self.max_extrude_ratio,
                area,
                move.move_d,
            )
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area)
            )

    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r)) ** 2
        return move.max_cruise_v2

    def move(self, print_time, move):
        axis_r = move.axes_r[3]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        pressure_advance = 0.0
        use_pa_from_trapq = 0.0
        if self.extruder_stepper:
            if self.extruder_stepper.per_move_pressure_advance:
                use_pa_from_trapq = 1.0
            if axis_r > 0.0 and (move.axes_d[0] or move.axes_d[1]):
                pressure_advance = self.extruder_stepper.pressure_advance
        # Queue movement (x is extruder movement, y is pressure advance flag)
        self.trapq_append(
            self.trapq,
            print_time,
            move.accel_t,
            move.cruise_t,
            move.decel_t,
            move.start_pos[3],
            0.0,
            0.0,
            1.0,
            pressure_advance,
            use_pa_from_trapq,
            start_v,
            cruise_v,
            accel,
        )
        self.last_position = move.end_pos[3]

    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.0
        return self.extruder_stepper.find_past_position(print_time)

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

    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2

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
