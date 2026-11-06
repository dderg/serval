# EtherCAT servo axes driven from the toolhead trapq
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

from . import wire
from .endpoint import FileEndpoint
from .endstop import ServoEndstop
from .servo_stepper import ServoStepper
from .stream import EthercatError, ServoLane, split_run
from .timemap import SyntheticGrid

CYCLE_US_QUANTUM = 250


class EthercatBus:
    """[ethercat]: owns the servo steppers and streams their samples.

    Each flush samples every servo on the DC grid over exactly the window
    the steppers generate steps for, and pushes the runs to the endpoint.
    """

    def __init__(self, config):
        self.printer = printer = config.get_printer()
        cycle_us = config.getint("cycle_us", 250, minval=CYCLE_US_QUANTUM)
        if cycle_us % CYCLE_US_QUANTUM:
            raise config.error(
                "[ethercat] cycle_us must be a multiple of %d"
                % (CYCLE_US_QUANTUM,)
            )
        self.cycle_ns = cycle_us * 1000
        if not printer.lookup_object("mcu").is_fileoutput():
            raise config.error(
                "[ethercat] the live EtherCAT endpoint is not available yet;"
                " servos run only in batch mode (klippy -o)"
            )
        self.output_path = config.get(
            "file_output",
            printer.get_start_args()["debugoutput"] + ".ethercat",
        )
        self.grid = SyntheticGrid(self.cycle_ns)
        self.next_index = self.grid.first_index_at_or_after(0.0)
        self.servos = []
        self.lanes = []
        self.endpoint = None
        printer.lookup_object("pins").register_chip("ethercat", self)
        motion_queuing = printer.load_object(config, "motion_queuing")
        motion_queuing.register_flush_callback(self._flush)
        printer.register_event_handler("klippy:connect", self._connect)
        printer.register_event_handler("klippy:disconnect", self._disconnect)

    # PrinterStepper hook ("servo: ethercat" in a stepper section)
    def setup_stepper(self, config, units_in_radians):
        servo = ServoStepper(config, units_in_radians)
        self.servos.append(servo)
        return servo

    # Pin chip "ethercat"
    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop":
            raise self.printer.config_error(
                "ethercat:%s: EtherCAT pins are servo endstops, not %s"
                % (pin_params["pin"], pin_type)
            )
        return ServoEndstop(self.printer, pin_params["pin"])

    def _connect(self):
        self._check_endstops()
        servos = sorted(self.servos, key=lambda s: s.slave_position)
        for first, second in zip(servos, servos[1:]):
            if first.slave_position == second.slave_position:
                raise self.printer.config_error(
                    "%s and %s share ethercat_slave %d"
                    % (
                        first.get_name(),
                        second.get_name(),
                        first.slave_position,
                    )
                )
        self.lanes = [
            ServoLane(servo, slot, self.cycle_ns)
            for slot, servo in enumerate(servos)
        ]
        self.endpoint = FileEndpoint(self.output_path, self.cycle_ns)

    def _check_endstops(self):
        query_endstops = self.printer.lookup_object("query_endstops", None)
        if query_endstops is None:
            return
        for endstop, name in query_endstops.endstops:
            if isinstance(endstop, ServoEndstop):
                continue
            servos = [
                s.get_name()
                for s in endstop.get_steppers()
                if isinstance(s, ServoStepper)
            ]
            if servos:
                raise self.printer.config_error(
                    "endstop '%s' cannot stop EtherCAT servo %s; servo rails"
                    " need an 'ethercat:' endstop_pin"
                    % (name, ", ".join(servos))
                )

    def _disconnect(self):
        if self.endpoint is not None:
            self.endpoint.close()

    def _flush(self, flush_time, step_gen_time):
        try:
            self._emit_until(step_gen_time)
        except (EthercatError, wire.WireError) as e:
            self.printer.invoke_shutdown("ethercat: %s" % (e,))
            raise

    def _emit_until(self, step_gen_time):
        start = self.next_index
        end = self.grid.first_index_at_or_after(step_gen_time)
        count = end - start
        if count <= 0 or not self.servos:
            return
        if self.endpoint is None:
            raise EthercatError(
                "motion to sample before klippy:connect set up the endpoint"
            )
        start_time = self.grid.print_time(start)
        runs = []
        for lane in self.lanes:
            pos, vel, acc = lane.servo.sample(
                start_time, self.grid.interval, count
            )
            run = lane.build_run(start, pos, vel, acc, count)
            runs.append(split_run(run, wire.MAX_FILL_CYCLES))
        for chunk in zip(*runs):
            self.endpoint.push(list(chunk))
        self.next_index = end


def load_config(config):
    return EthercatBus(config)
