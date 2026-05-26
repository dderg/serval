# Diagnostic tool for reporting stepper and kinematic positions
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import bulk_sensor


# Extract stepper queue_step messages
class DumpStepper:
    def __init__(self, printer, mcu_stepper):
        self.printer = printer
        self.mcu_stepper = mcu_stepper
        self.last_batch_clock = 0
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            printer, self._process_batch
        )
        api_resp = {"header": ("interval", "count", "add")}
        self.batch_bulk.add_mux_endpoint(
            "motion_report/dump_stepper",
            "name",
            mcu_stepper.get_name(),
            api_resp,
        )

    def get_step_queue(self, start_clock, end_clock):
        mcu_stepper = self.mcu_stepper
        res = []
        while True:
            data, count = mcu_stepper.dump_steps(128, start_clock, end_clock)
            if not count:
                break
            res.append((data, count))
            if count < len(data):
                break
            end_clock = data[count - 1].first_clock
        res.reverse()
        return ([d[i] for d, cnt in res for i in range(cnt - 1, -1, -1)], res)

    def log_steps(self, data):
        if not data:
            return
        out = []
        out.append(
            "Dumping stepper '%s' (%s) %d queue_step:"
            % (
                self.mcu_stepper.get_name(),
                self.mcu_stepper.get_mcu().get_name(),
                len(data),
            )
        )
        for i, s in enumerate(data):
            out.append(
                "queue_step %d: t=%d p=%d i=%d c=%d a=%d"
                % (
                    i,
                    s.first_clock,
                    s.start_position,
                    s.interval,
                    s.step_count,
                    s.add,
                )
            )
        logging.info("\n".join(out))

    def _process_batch(self, eventtime):
        data, cdata = self.get_step_queue(self.last_batch_clock, 1 << 63)
        if not data:
            return {}
        clock_to_print_time = self.mcu_stepper.get_mcu().clock_to_print_time
        first = data[0]
        first_clock = first.first_clock
        first_time = clock_to_print_time(first_clock)
        self.last_batch_clock = last_clock = data[-1].last_clock
        last_time = clock_to_print_time(last_clock)
        mcu_pos = first.start_position
        start_position = self.mcu_stepper.mcu_to_commanded_position(mcu_pos)
        step_dist = self.mcu_stepper.get_step_dist()
        d = [(s.interval, s.step_count, s.add) for s in data]
        return {
            "data": d,
            "start_position": start_position,
            "start_mcu_position": mcu_pos,
            "step_distance": step_dist,
            "first_clock": first_clock,
            "first_step_time": first_time,
            "last_clock": last_clock,
            "last_step_time": last_time,
        }


STATUS_REFRESH_TIME = 0.250


class PrinterMotionReport:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        # get_status information
        self.next_status_time = 0.0
        gcode = self.printer.lookup_object("gcode")
        self.last_status = {
            "live_position": gcode.Coord(0.0, 0.0, 0.0, 0.0),
            "live_velocity": 0.0,
            "live_extruder_velocity": 0.0,
            "steppers": [],
            "trapq": [],
        }
        # Register handlers
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("klippy:shutdown", self._shutdown)

    def register_stepper(self, config, mcu_stepper):
        ds = DumpStepper(self.printer, mcu_stepper)
        self.steppers[mcu_stepper.get_name()] = ds

    def _connect(self):
        self.last_status["steppers"] = list(sorted(self.steppers.keys()))

    # Shutdown handling
    def _dump_shutdown(self, eventtime):
        # Step pulses are emitted from the Rust runtime; there is no
        # host-side step queue to spill at shutdown. Log the shutdown
        # print-time derived from each MCU's shutdown clock for diagnostics.
        for dstepper in self.steppers.values():
            mcu = dstepper.mcu_stepper.get_mcu()
            sc = mcu.get_shutdown_clock()
            if not sc:
                continue
            shutdown_time = mcu.clock_to_print_time(sc)
            logging.info(
                "MCU '%s' shutdown at print_time %.6f",
                mcu.get_name(),
                shutdown_time,
            )

    def _shutdown(self):
        self.printer.get_reactor().register_callback(self._dump_shutdown)

    # Status reporting
    def get_status(self, eventtime):
        if eventtime < self.next_status_time:
            return self.last_status
        self.next_status_time = eventtime + STATUS_REFRESH_TIME
        # Live position and velocity are owned by the bridge; return zeros here.
        toolhead = self.printer.lookup_object("toolhead")
        self.last_status = dict(self.last_status)
        self.last_status["live_position"] = toolhead.Coord(
            0.0, 0.0, 0.0, 0.0
        )
        self.last_status["live_velocity"] = 0.0
        self.last_status["live_extruder_velocity"] = 0.0
        return self.last_status


def load_config(config):
    return PrinterMotionReport(config)
