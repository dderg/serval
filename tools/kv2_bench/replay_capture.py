"""Batch-mode klippy that regenerates steps from a recorded live trapq.

    python -m tools.kv2_bench.replay_capture CAPTURE_JSON PHASES_JSON <klippy args...>

The input gcode runs REPLAY_LIVE_TRAPQ once. It skips klippy's planner:
- sets the toolhead position to the first recorded phase's start (the step
  lattice is anchored there, as SET_KINEMATIC_POSITION anchored it live);
- appends every recorded phase to its trapq ("toolhead", or an extruder's)
  at its live print_time, so the MCU clocks come out on the live timeline;
- generates steps in the live background flusher's batch size;
- records every stepper's queue_step history, like batch_capture.py.

Step generation runs through the unmodified chelper itersolve, input-shaper
and stepcompress code of the klippy tree under test.
"""

from __future__ import annotations

import json
import math
import sys

from klippy import printer, toolhead
from klippy.extras import motion_queuing
from tools.motion_eval import capture as motion_capture


class ReplayCapture:
    def __init__(self, phases: dict):
        self.phases = phases
        self.history = None
        self.undrained_serial = False
        self.printer = None
        self.steppers = []

    def attach(self, toolhead_obj):
        self.printer = toolhead_obj.printer
        self.toolhead = toolhead_obj
        self.printer.register_event_handler("klippy:connect", self._connect)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("REPLAY_LIVE_TRAPQ", self.cmd_replay)

    def _extruders(self):
        return dict(self.printer.lookup_objects("extruder"))

    def _connect(self):
        steppers = list(self.toolhead.get_kinematics().get_steppers())
        for extruder in self._extruders().values():
            steppers += [s.stepper for s in extruder.get_extruder_steppers()]
        self.steppers = steppers
        self.history = motion_capture.StepHistory(steppers)
        self.printer.lookup_object("motion_queuing").register_flush_callback(
            self.history.pull
        )

    def _trapq(self, name):
        if name == "toolhead":
            return self.toolhead.get_trapq()
        extruder = self._extruders().get(name)
        if extruder is None:
            raise motion_capture.UnsupportedRun(f"no trapq named {name!r}")
        return extruder.get_trapq()

    def cmd_replay(self, gcmd):
        toolhead_phases = self.phases["toolhead"]
        x, y, z = toolhead_phases[0]["start"]
        self.toolhead.set_position([x, y, z, 0.0])
        for name, phases in self.phases.items():
            if name != "toolhead":
                extruder = self._extruders()[name]
                e0 = phases[0]["start"][0]
                for es in extruder.get_extruder_steppers():
                    es.stepper.set_position((e0, 0.0, 0.0))
        mq = self.printer.lookup_object("motion_queuing")
        append = mq.lookup_trapq_append()
        start = min(p[0]["t"] for p in self.phases.values())
        end = 0.0
        for name, phases in self.phases.items():
            trapq = self._trapq(name)
            for p in phases:
                accel = p["a"]
                accel_t = p["dur"] if accel > 0.0 else 0.0
                decel_t = p["dur"] if accel < 0.0 else 0.0
                cruise_t = p["dur"] if accel == 0.0 else 0.0
                cruise_v = p["v0"] + accel * accel_t
                append(
                    trapq, p["t"], accel_t, cruise_t, decel_t,
                    *p["start"], *p["axes"],
                    p["v0"], cruise_v, abs(accel),
                )  # fmt: skip
                end = max(end, p["t"] + p["dur"])
        mq._advance_flush_time(start - 1.0)
        batch = (
            motion_queuing.BGFLUSH_SG_HIGH_TIME
            - motion_queuing.BGFLUSH_SG_LOW_TIME
        )
        target = end + mq.get_kin_flush_delay() + batch
        while mq.last_step_gen_time < target:
            mq._advance_flush_time(0.0, mq.last_step_gen_time + batch)
        mq._advance_flush_time(target)

    def finish(self):
        if not self.history.pulled_at_disconnect:
            raise motion_capture.UnsupportedRun(
                "history not read at disconnect"
            )
        return {
            "steppers": {
                s.get_name(): {
                    "oid": s.get_oid(),
                    "mcu": s.get_mcu().get_name(),
                    "clock_freq": s.get_mcu().get_constant_float("CLOCK_FREQ"),
                    "step_dist": s.get_step_dist(),
                    "invert_dir": bool(s.get_dir_inverted()[0]),
                    "step_history": self.history.entries[s.get_name()],
                }
                for s in self.steppers
            },
            "input_shaper": motion_capture._shaper_state(self.printer),
        }


def main():
    capture_path, phases_path = sys.argv[1], sys.argv[2]
    with open(phases_path) as f:
        capture = ReplayCapture(json.load(f))
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        capture.attach(self)

    toolhead.ToolHead.__init__ = init
    motion_capture.drain_before_disconnect(capture)
    motion_queuing.MOVE_HISTORY_EXPIRE = math.inf
    sys.argv = ["klippy"] + sys.argv[3:]
    printer.main()
    with open(capture_path, "w") as f:
        json.dump(capture.finish(), f)


if __name__ == "__main__":
    main()
