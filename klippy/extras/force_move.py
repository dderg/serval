# Stepper registry and low-level kinematic position commands
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

PHASE5_GATE = "%s is not yet supported under the new motion path until Phase 5"


class ForceMove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "STEPPER_BUZZ",
            self.cmd_STEPPER_BUZZ,
            desc=self.cmd_STEPPER_BUZZ_help,
        )
        if not config.getboolean("enable_force_move", True):
            return

        gcode.register_command(
            "FORCE_MOVE", self.cmd_FORCE_MOVE, desc=self.cmd_FORCE_MOVE_help
        )
        gcode.register_command(
            "SET_KINEMATIC_POSITION",
            self.cmd_SET_KINEMATIC_POSITION,
            desc=self.cmd_SET_KINEMATIC_POSITION_help,
        )

    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper

    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    def manual_move(self, stepper, dist, speed, accel=0.0):
        raise self.printer.command_error(PHASE5_GATE % ("manual_move",))

    cmd_STEPPER_BUZZ_help = "Oscillate a given stepper to help id it"

    def cmd_STEPPER_BUZZ(self, gcmd):
        raise gcmd.error(PHASE5_GATE % ("STEPPER_BUZZ",))

    cmd_FORCE_MOVE_help = "Manually move a stepper; invalidates kinematics"

    def cmd_FORCE_MOVE(self, gcmd):
        raise gcmd.error(PHASE5_GATE % ("FORCE_MOVE",))

    cmd_SET_KINEMATIC_POSITION_help = "Force a low-level kinematic position"

    def cmd_SET_KINEMATIC_POSITION(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.get_last_move_time()
        curpos = toolhead.get_position()
        x = gcmd.get_float("X", curpos[0])
        y = gcmd.get_float("Y", curpos[1])
        z = gcmd.get_float("Z", curpos[2])
        clear = gcmd.get("CLEAR", "").upper()
        axes = ["X", "Y", "Z"]
        clear_axes = [axes.index(a) for a in axes if a in clear]
        logging.info(
            "SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f clear=%s",
            x,
            y,
            z,
            ",".join((axes[i] for i in clear_axes)),
        )
        toolhead.set_position([x, y, z, curpos[3]], homing_axes=(0, 1, 2))
        toolhead.get_kinematics().clear_homing_state(clear_axes)


def load_config(config):
    return ForceMove(config)
