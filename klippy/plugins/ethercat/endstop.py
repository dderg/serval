# Endstop for rails driven by EtherCAT servos
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

HOMING_UNAVAILABLE = (
    "EtherCAT servo endstop '%s' cannot home or report a state yet: servo"
    " homing needs the live endpoint's torque trip. Position servo axes with"
    " SET_KINEMATIC_POSITION."
)


class ServoEndstop:
    """The endstop of a servo rail (pin 'ethercat:<name>').

    An MCU endstop stops steppers by oid on its own MCU and cannot stop a
    servo, so servo rails take their endstop from the servo bus. Homing
    through it arrives with the live endpoint; until then every use fails.
    """

    def __init__(self, printer, name):
        self._printer = printer
        self._name = name
        self._steppers = []

    def add_stepper(self, stepper):
        if stepper not in self._steppers:
            self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def _unavailable(self):
        return self._printer.command_error(HOMING_UNAVAILABLE % (self._name,))

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        raise self._unavailable()

    def home_wait(self, home_end_time):
        raise self._unavailable()

    def query_endstop(self, print_time):
        raise self._unavailable()
