# Endstop for rails driven by EtherCAT servos: the endpoint's torque trip
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

from .servo_stepper import ServoStepper

# How long after the homing move's last sample played a trip may still be
# reported: the drive can reach the threshold a few cycles after the
# commanded motion stops against the hard stop.
TRIP_REPORT_MARGIN_S = 0.050


class ServoEndstop:
    """The endstop of a servo rail (pin 'ethercat:<name>').

    Homing arms the endpoint's sensorless endstop on every servo of the
    rail. The endpoint cuts the stream on the trip itself (its trsync), and
    reports the clock of the sample whose commanded position the rotor
    stalled at; that sample's print time is the trigger time, and the sample
    the endpoint last played is where the servos hold.
    """

    def __init__(self, bus, name, endstop_id):
        self._bus = bus
        self._printer = bus.printer
        self._name = name
        self.endstop_id = endstop_id
        self._steppers = []
        self._completion = None
        self._trip = None
        self._lanes = []

    def add_stepper(self, stepper):
        if stepper not in self._steppers:
            self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def _homing_lanes(self):
        error = self._printer.command_error
        lanes = []
        for stepper in self._steppers:
            if not isinstance(stepper, ServoStepper):
                raise error(
                    "EtherCAT endstop '%s' cannot stop %s, which is not an"
                    " EtherCAT servo" % (self._name, stepper.get_name())
                )
            if stepper.homing_torque_tenth_pct is None:
                raise error(
                    "%s: set homing_torque to home on EtherCAT endstop '%s'"
                    % (stepper.get_name(), self._name)
                )
            lanes.append(self._bus.lane_of(stepper))
        return lanes

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        if not triggered:
            raise self._printer.command_error(
                "EtherCAT endstop '%s' trips on contact; it cannot home away"
                " from it" % (self._name,)
            )
        self._lanes = self._homing_lanes()
        self._completion = self._printer.get_reactor().completion()
        self._trip = None
        self._bus.endpoint.arm_endstop(
            self.endstop_id,
            [
                (lane.slot_idx, lane.servo.homing_torque_tenth_pct)
                for lane in self._lanes
            ],
            self._on_trip,
        )
        return self._completion

    def _on_trip(self, contact_clock, playback_clocks):
        """Socket thread: the endpoint has already cut the stream."""
        if self._trip is not None:
            return
        self._trip = (contact_clock, playback_clocks)
        self._printer.get_reactor().async_complete(self._completion, False)

    def home_wait(self, home_end_time):
        """In batch mode there is no endpoint: like an MCU endstop writing
        to a file, the endstop triggers at the end of the move. Live, a trip
        the endpoint sent before it disarmed arrives ahead of the disarm
        reply, so it has been seen once disarming returns."""
        endpoint = self._bus.endpoint
        slots = [lane.slot_idx for lane in self._lanes]
        if not self._bus.live:
            endpoint.disarm_endstop(self.endstop_id, slots)
            return home_end_time
        grid = endpoint.grid
        end_clock = grid.clock(grid.first_index_at_or_after(home_end_time))
        self._completion.wait(end_clock / 1e9 + TRIP_REPORT_MARGIN_S)
        endpoint.disarm_endstop(self.endstop_id, slots)
        if self._trip is None:
            return 0.0
        contact_clock, playback_clocks = self._trip
        return self._bus.hold_after_trip(
            self._name, self._lanes, contact_clock, playback_clocks
        )

    def query_endstop(self, print_time):
        """A torque trip is an event, not a level: between homing moves
        nothing presses on it."""
        return 0
