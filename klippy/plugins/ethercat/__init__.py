# EtherCAT servo axes driven from the toolhead trapq
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
import shlex

from klippy.extras import motion_queuing

from . import wire
from .endpoint import FileEndpoint, LiveEndpoint
from .endstop import ServoEndstop
from .servo_stepper import ServoStepper
from .shaping import ServoShapingGuard
from .stream import EthercatError, ServoLane, split_run

CYCLE_US_QUANTUM = 250
DEFAULT_SOCKET = "/tmp/kalico-ethercat.sock"
ANCHOR_LEADS = 2
MAX_ENDSTOP_ID = 255


class EthercatBus:
    """[ethercat]: owns the servo steppers and streams their samples.

    Each flush samples every servo on the DC grid over exactly the window
    the steppers generate steps for, and pushes the runs to the endpoint: a
    live ethercat-rt, or a frame file when klippy runs in batch mode.
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
        self.live = not printer.lookup_object("mcu").is_fileoutput()
        if self.live:
            binary = config.get("endpoint_binary", None)
            command = None
            if binary is not None:
                args = shlex.split(config.get("endpoint_args", ""))
                command = [binary] + args
            min_lead = config.getfloat("min_lead", 0.050, above=0.0)
            self.endpoint = LiveEndpoint(
                printer,
                config.get("socket", DEFAULT_SOCKET),
                command,
                self.cycle_ns,
                min_lead,
                motion_queuing.BGFLUSH_SG_HIGH_TIME + min_lead,
            )
        else:
            self.endpoint = FileEndpoint(
                config.get(
                    "file_output",
                    printer.get_start_args()["debugoutput"] + ".ethercat",
                ),
                self.cycle_ns,
            )
        self.next_index = None
        self.last_step_gen_time = 0.0
        self.servos = []
        self.lanes = []
        self.endstops = []
        self._resume_on_set_position = False
        printer.lookup_object("pins").register_chip("ethercat", self)
        mq = printer.load_object(config, "motion_queuing")
        mq.register_flush_callback(self._flush)
        printer.register_event_handler("klippy:connect", self._connect)
        printer.register_event_handler("klippy:shutdown", self._shutdown)
        printer.register_event_handler("klippy:disconnect", self._disconnect)
        printer.register_event_handler(
            "toolhead:set_position", self._handle_set_position
        )

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
        if len(self.endstops) == MAX_ENDSTOP_ID:
            raise self.printer.config_error(
                "ethercat:%s: more than %d EtherCAT endstops"
                % (pin_params["pin"], MAX_ENDSTOP_ID)
            )
        endstop = ServoEndstop(self, pin_params["pin"], len(self.endstops) + 1)
        self.endstops.append(endstop)
        return endstop

    # Lifecycle
    def _connect(self):
        self._check_endstops()
        if self.servos:
            ServoShapingGuard(self.printer, self.servos).check_config()
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
        for lane in self.lanes:
            lane.servo.attach_lane(lane, self.endpoint)
        self.endpoint.connect(self.lanes)
        if self.live:
            self.endpoint.enable_drives()
        else:
            self.next_index = self.endpoint.grid.first_index_at_or_after(0.0)

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

    def _shutdown(self):
        if self.live:
            self.endpoint.halt()

    def _disconnect(self):
        self.endpoint.close()

    def stats(self, eventtime):
        self.endpoint.recalibrate(self.last_step_gen_time, eventtime)
        return False, self.endpoint.stats()

    def get_status(self, eventtime):
        return self.endpoint.get_status()

    # Homing
    def lane_of(self, servo):
        for lane in self.lanes:
            if lane.servo is servo:
                return lane
        raise EthercatError("%s has no lane" % (servo.get_name(),))

    def hold_after_trip(self, endstop_name, homing_lanes, contact, cursors):
        """The endpoint cut the stream on a homing trip and holds every slot
        at the last sample it played. Returns the trigger print time."""
        grid = self.endpoint.grid
        if cursors is None or not all(
            cursors[lane.slot_idx] for lane in homing_lanes
        ):
            self._fail(
                "endstop '%s' tripped before the endpoint reported its"
                " playback position (%r)" % (endstop_name, cursors)
            )
        halt = grid.index_of_clock(cursors[homing_lanes[0].slot_idx]) - 1
        for lane in self.lanes:
            if lane in homing_lanes:
                lane.servo.hold_at_mcu_position(lane.mcu_position_at(halt))
            elif lane.last_motion_index is not None and (
                lane.last_motion_index >= halt
            ):
                self._fail(
                    "endstop '%s' cut the stream at index %d while %s was"
                    " moving (motion queued up to index %d)"
                    % (
                        endstop_name,
                        halt,
                        lane.servo.get_name(),
                        lane.last_motion_index,
                    )
                )
        self._resume_on_set_position = True
        return grid.print_time(grid.index_of_clock(contact))

    def _handle_set_position(self):
        """Homing ends by setting the toolhead position; the trapq then no
        longer holds the motion the trip cut off and streaming resumes."""
        if not self._resume_on_set_position:
            return
        self._resume_on_set_position = False
        self.endpoint.resume_stream()
        for lane in self.lanes:
            lane.reanchor()
        self.next_index = None

    def _fail(self, msg):
        self.printer.invoke_shutdown("ethercat: " + msg)
        raise self.printer.command_error(msg)

    # Streaming
    def _flush(self, flush_time, step_gen_time):
        try:
            self._emit_until(step_gen_time)
        except (EthercatError, wire.WireError) as e:
            self.printer.invoke_shutdown("ethercat: %s" % (e,))
            raise

    def _emit_until(self, step_gen_time):
        if not self.lanes or self.endpoint.stream_halted:
            return
        grid = self.endpoint.grid
        end = grid.first_index_at_or_after(step_gen_time)
        start = self.next_index
        now = self.endpoint.now_index()
        if now is not None:
            lead = self.endpoint.min_lead_cycles
            if start is None or start < now + lead:
                # (Re)anchor with a second lead of headroom: sampling and
                # packing this batch takes time before the push is checked
                anchor = now + ANCHOR_LEADS * lead
                if start is None:
                    start = max(anchor, self._first_samplable_index())
                else:
                    start = self._skip_behind_real_time(
                        start, min(anchor, end), now
                    )
        if end <= start:
            return
        self.last_step_gen_time = step_gen_time
        start_time = grid.print_time(start)
        count = end - start
        runs = []
        for lane in self.lanes:
            pos, vel, acc = lane.servo.sample(start_time, grid.interval, count)
            run = lane.build_run(start, pos, vel, acc, count)
            runs.append(split_run(run, wire.MAX_FILL_CYCLES))
        for chunk in zip(*runs):
            self.endpoint.push(list(chunk))
        self.next_index = end

    def _first_samplable_index(self):
        """A homing trip leaves the cut-off move in the trapq history up to
        the print time homing set the position at; the servo held still
        through it, so a new epoch starts once no sample reaches into it."""
        earliest = max(lane.servo.earliest_sample_time() for lane in self.lanes)
        if earliest == -math.inf:
            return 0
        return self.endpoint.grid.first_index_at_or_after(earliest)

    def _skip_behind_real_time(self, start, stop, now):
        """The stream fell behind the endpoint's play index. That is only
        allowed while every servo holds still: the motion already played,
        and the samples real time passed are stationary. The servos then
        re-anchor at stop."""
        grid = self.endpoint.grid
        for lane in self.lanes:
            if (
                lane.last_motion_index is not None
                and lane.last_motion_index >= now
            ):
                raise EthercatError(
                    "%s: stream behind real time while moving (motion queued"
                    " up to index %d, endpoint playing %d)"
                    % (lane.servo.get_name(), lane.last_motion_index, now)
                )
            pos, vel, acc = lane.servo.sample(
                grid.print_time(start), grid.interval, stop - start
            )
            moving = [i for i in range(stop - start) if vel[i] or acc[i]]
            if moving:
                index = start + moving[0]
                short_ms = (
                    (now + self.endpoint.min_lead_cycles - index)
                    * self.cycle_ns
                    / 1e6
                )
                raise EthercatError(
                    "%s: motion at index %d arrives %.1f ms inside the"
                    " endpoint's lead (play index %d)"
                    % (lane.servo.get_name(), index, short_ms, now)
                )
            lane.reanchor()
        self.next_index = stop
        return stop


def load_config(config):
    return EthercatBus(config)
