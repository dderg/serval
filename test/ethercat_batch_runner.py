"""Run klippy in batch mode and record the toolhead trapq moves, the input
shaper, and the geometry and stepcompress history of every kinematic stepper
and servo.

    python test/ethercat_batch_runner.py CAPTURE_JSON <klippy args...>
"""

import json
import sys

from klippy import printer, toolhead
from klippy.plugins.ethercat.servo_stepper import ServoStepper
from tools.motion_eval.capture import TRAPQ_APPEND_FIELDS, _shaper_state

HISTORY_END_CLOCK = (1 << 63) - 1


def step_history(stepper):
    """queue_step history, oldest first, as stepcompress recorded it."""
    batch = 4096
    while True:
        data, count = stepper.dump_steps(batch, 0, HISTORY_END_CLOCK)
        if count < batch:
            break
        batch *= 2
    entries = []
    for i in reversed(range(count)):
        h = data[i]
        entries.append(
            {
                "first_clock": h.first_clock,
                "start_position": h.start_position,
                "step_count": h.step_count,
                "interval": h.interval,
                "add": h.add,
                "add2": h.add2,
                "shift": h.shift,
            }
        )
    return entries


def describe(stepper):
    info = {
        "step_dist": stepper.get_step_dist(),
        "invert_dir": bool(stepper.get_dir_inverted()[0]),
        "commanded_pos_at_mcu_zero": stepper.mcu_to_commanded_position(0),
    }
    if isinstance(stepper, ServoStepper):
        info["servo"] = True
        info["ethercat_slave"] = stepper.slave_position
    else:
        info["servo"] = False
        info["clock_freq"] = stepper.get_mcu().get_constant_float("CLOCK_FREQ")
        info["history"] = step_history(stepper)
    return info


def main():
    capture_path = sys.argv[1]
    capture = {}
    moves = []
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        real_append = self.trapq_append

        def trapq_append(tq, *args):
            if tq == self.trapq:
                moves.append(dict(zip(TRAPQ_APPEND_FIELDS, args)))
            return real_append(tq, *args)

        def on_disconnect():
            steppers = self.get_kinematics().get_steppers()
            capture["steppers"] = {s.get_name(): describe(s) for s in steppers}
            capture["shaper"] = _shaper_state(self.printer)

        self.trapq_append = trapq_append
        self.printer.register_event_handler("klippy:disconnect", on_disconnect)

    toolhead.ToolHead.__init__ = init
    sys.argv = ["klippy"] + sys.argv[2:]
    try:
        printer.main()
    finally:
        if capture:
            capture["moves"] = moves
            with open(capture_path, "w") as f:
                json.dump(capture, f)


if __name__ == "__main__":
    main()
