# Diagnostic tool for reporting stepper positions
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bulk_sensor


class DumpStepper:
    def __init__(self, printer, mcu_stepper):
        self.printer = printer
        self.mcu_stepper = mcu_stepper
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

    def _process_batch(self, eventtime):
        return {}


class PrinterMotionReport:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        gcode = self.printer.lookup_object("gcode")
        self.last_status = {
            "live_position": gcode.Coord(0.0, 0.0, 0.0, 0.0),
            "live_velocity": 0.0,
            "live_extruder_velocity": 0.0,
            "steppers": [],
            "trapq": [],
        }
        self.printer.register_event_handler("klippy:connect", self._connect)

    def register_stepper(self, config, mcu_stepper):
        ds = DumpStepper(self.printer, mcu_stepper)
        self.steppers[mcu_stepper.get_name()] = ds

    def _connect(self):
        self.last_status["steppers"] = list(sorted(self.steppers.keys()))
        self.engine = self.printer.lookup_object("motion_engine", None)
        self.motion = self.printer.lookup_object("motion", None)

    def get_status(self, eventtime):
        # Live position is the *commanded* trajectory evaluated at the current
        # MCU time (estimated_print_time), like mainline's trapq lookup — same
        # clock domain as gcode_position, so "actual" tracks "requested" instead
        # of lagging a 200ms hardware poll on a separate clock.
        engine = getattr(self, "engine", None)
        motion = getattr(self, "motion", None)
        if engine is None or motion is None or motion.mcu is None:
            return self.last_status
        est = motion.mcu.estimated_print_time(eventtime)
        try:
            state = engine.motion_state_at(motion.mcu, print_time=est)
        except Exception:
            return self.last_status
        if not state:
            return self.last_status
        gcode = self.printer.lookup_object("gcode")
        prev = self.last_status["live_position"]

        def axis(name, idx):
            pos, vel, _accel = state.get(name, (prev[idx], 0.0, 0.0))
            return pos, vel

        x, xv = axis("x", 0)
        y, yv = axis("y", 1)
        z, zv = axis("z", 2)
        e, ev = axis("e", 3)
        live_velocity = (xv * xv + yv * yv + zv * zv) ** 0.5
        self.last_status = {
            "live_position": gcode.Coord(x, y, z, e),
            "live_velocity": live_velocity,
            "live_extruder_velocity": ev,
            "steppers": self.last_status["steppers"],
            "trapq": self.last_status["trapq"],
        }
        return self.last_status


def load_config(config):
    return PrinterMotionReport(config)
