"""Batch-mode klippy run that records every stepper's queue_step history.

    python -m tools.kv2_bench.batch_capture CAPTURE_JSON <klippy args...>

Run with cwd and PYTHONPATH[0] at the klippy tree under test (kv2 or
upstream) and the kv2 tree after it, so `klippy` comes from the tree under
test and `tools` from kv2. Extends motion_eval's capture to the extruder
steppers, which it leaves out, since the bench captures the extruder too.
"""

from __future__ import annotations

import json
import math
import sys

from klippy import printer, toolhead
from klippy.extras import motion_queuing
from tools.motion_eval import capture as motion_capture


class BenchCapture(motion_capture.Capture):
    def attach(self, toolhead_obj):
        super().attach(toolhead_obj)
        self.printer.register_event_handler(
            "klippy:connect", self._add_extruder_steppers
        )

    def _extruder_steppers(self):
        return [
            extruder_stepper.stepper
            for _, extruder in self.printer.lookup_objects("extruder")
            for extruder_stepper in extruder.get_extruder_steppers()
        ]

    def _add_extruder_steppers(self):
        for stepper in self._extruder_steppers():
            name = stepper.get_name()
            self.history.steppers.append(stepper)
            self.history.entries[name] = []
            self.history.newest_first_clock[name] = -1

    def finish(self):
        result = super().finish()
        for stepper in self.history.steppers:
            name = stepper.get_name()
            result["steppers"].setdefault(
                name,
                {
                    "oid": stepper.get_oid(),
                    "mcu": stepper.get_mcu().get_name(),
                    "clock_freq": stepper.get_mcu().get_constant_float(
                        "CLOCK_FREQ"
                    ),
                    "step_dist": stepper.get_step_dist(),
                    "invert_dir": bool(stepper.get_dir_inverted()[0]),
                    "final_mcu_position": stepper.get_mcu_position(),
                },
            )["step_history"] = self.history.entries[name]
        return result


def main():
    capture_path = sys.argv[1]
    capture = BenchCapture()
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        capture.attach(self)

    toolhead.ToolHead.__init__ = init
    motion_capture.drain_before_disconnect(capture)
    motion_queuing.MOVE_HISTORY_EXPIRE = math.inf
    sys.argv = ["klippy"] + sys.argv[2:]
    printer.main()
    with open(capture_path, "w") as f:
        json.dump(capture.finish(), f)


if __name__ == "__main__":
    main()
