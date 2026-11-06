"""Run klippy in batch mode; when it exits, record the toolhead trapq moves,
the input shaper and the geometry of every kinematic stepper and servo.

    python test/ethercat_batch_runner.py CAPTURE_JSON <klippy args...>
"""

import json
import sys

from klippy import printer, toolhead
from klippy.plugins.ethercat.servo_stepper import ServoStepper
from tools.motion_eval.capture import TRAPQ_APPEND_FIELDS, _shaper_state


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
        info["oid"] = stepper.get_oid()
        info["clock_freq"] = stepper.get_mcu().get_constant_float("CLOCK_FREQ")
    return info


def main():
    capture_path = sys.argv[1]
    toolheads = []
    moves = []
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        toolheads.append(self)
        real_append = self.trapq_append

        def trapq_append(tq, *args):
            if tq == self.trapq:
                moves.append(dict(zip(TRAPQ_APPEND_FIELDS, args)))
            return real_append(tq, *args)

        self.trapq_append = trapq_append

    toolhead.ToolHead.__init__ = init
    sys.argv = ["klippy"] + sys.argv[2:]
    try:
        printer.main()
    finally:
        if toolheads:
            th = toolheads[-1]
            steppers = th.get_kinematics().get_steppers()
            capture = {
                "steppers": {s.get_name(): describe(s) for s in steppers},
                "moves": moves,
                "shaper": _shaper_state(th.printer),
            }
            with open(capture_path, "w") as f:
                json.dump(capture, f)


if __name__ == "__main__":
    main()
