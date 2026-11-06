"""Run klippy in batch mode and record what the planner produced.

Invoked as a subprocess by run.py:
    python -m tools.motion_eval.capture CAPTURE_JSON <klippy args...>

ToolHead is wrapped (not modified) so that every move appended to the
toolhead trapq, the stepper geometry and the input shaper parameters are
written to CAPTURE_JSON once klippy exits.
"""

from __future__ import annotations

import json
import sys

from klippy import chelper, printer, toolhead
from klippy.extras import input_shaper

TRAPQ_APPEND_FIELDS = (
    "print_time",
    "accel_t",
    "cruise_t",
    "decel_t",
    "start_x",
    "start_y",
    "start_z",
    "axes_r_x",
    "axes_r_y",
    "axes_r_z",
    "start_v",
    "cruise_v",
    "accel",
)

UNIT_COORDS = (
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
LINEARITY_PROBE = (37.0, -11.0, 5.0)


class UnsupportedRun(Exception):
    pass


def _stepper_linear_map(stepper):
    _, ffi_lib = chelper.get_ffi()
    sk = stepper.get_stepper_kinematics()

    def pos(coord):
        return ffi_lib.itersolve_calc_position_from_coord(sk, *coord)

    origin, ex, ey, ez = (pos(c) for c in UNIT_COORDS)
    row = [ex - origin, ey - origin, ez - origin]
    predicted = origin + sum(r * c for r, c in zip(row, LINEARITY_PROBE))
    actual = pos(LINEARITY_PROBE)
    if abs(predicted - actual) > 1e-9 * max(1.0, abs(actual)):
        raise UnsupportedRun(
            "stepper %s kinematics are not linear in xyz (predicted %r, got %r)"
            % (stepper.get_name(), predicted, actual)
        )
    return {"offset": origin, "row": row}


def _shaper_state(printer_obj):
    shaper_obj = printer_obj.lookup_object("input_shaper", None)
    if shaper_obj is None:
        return {}
    state = {}
    for shaper in shaper_obj.get_shapers():
        axis = shaper.get_axis()
        if isinstance(shaper, input_shaper.AxisInputShaper):
            state[axis] = {
                "kind": "pulses",
                "type": shaper.get_type(),
                "A": list(shaper.A),
                "T": list(shaper.T),
            }
        elif isinstance(shaper, input_shaper.AxisInputSmoother):
            state[axis] = {
                "kind": "smoother",
                "type": shaper.get_type(),
                "coeffs": list(shaper.coeffs),
                "smooth_time": shaper.smooth_time,
            }
        else:
            raise UnsupportedRun("unknown shaper class %r" % (type(shaper),))
    return state


class Capture:
    def __init__(self):
        self.toolhead = None
        self.printer = None
        self.moves = []
        self.linear_maps = {}
        self.steppers = None
        self.shaper = None
        self.settings = None

    def attach(self, toolhead_obj):
        self.toolhead = toolhead_obj
        self.printer = toolhead_obj.printer
        for stepper in toolhead_obj.get_kinematics().get_steppers():
            self.linear_maps[stepper.get_name()] = _stepper_linear_map(stepper)
        real_append = toolhead_obj.trapq_append
        real_set_position = toolhead_obj.set_position

        def trapq_append(tq, *args):
            if tq == toolhead_obj.trapq:
                if not self.moves:
                    self._snapshot_start()
                self.moves.append(dict(zip(TRAPQ_APPEND_FIELDS, args)))
            return real_append(tq, *args)

        def set_position(newpos, homing_axes=""):
            if self.moves:
                raise UnsupportedRun(
                    "toolhead.set_position after motion started (homing or"
                    " SET_KINEMATIC_POSITION inside the gcode) is not supported"
                )
            return real_set_position(newpos, homing_axes)

        toolhead_obj.trapq_append = trapq_append
        toolhead_obj.set_position = set_position

    def _snapshot_start(self):
        th = self.toolhead
        self.settings = {
            "max_velocity": th.max_velocity,
            "max_accel": th.max_accel,
            "square_corner_velocity": th.square_corner_velocity,
            "minimum_cruise_ratio": th.min_cruise_ratio,
        }
        self.shaper = _shaper_state(self.printer)
        self.steppers = {}
        for stepper in th.get_kinematics().get_steppers():
            mcu = stepper.get_mcu()
            name = stepper.get_name()
            self.steppers[name] = {
                "oid": stepper.get_oid(),
                "mcu": mcu.get_name(),
                "clock_freq": mcu.get_constant_float("CLOCK_FREQ"),
                "step_dist": stepper.get_step_dist(),
                "invert_dir": bool(stepper.get_dir_inverted()[0]),
                "commanded_pos_at_mcu_zero": stepper.mcu_to_commanded_position(
                    0
                ),
                "linear_map": self.linear_maps[name],
            }

    def finish(self):
        if not self.moves:
            raise UnsupportedRun("gcode produced no toolhead motion")
        if _shaper_state(self.printer) != self.shaper:
            raise UnsupportedRun("input shaper changed after motion started")
        for stepper in self.toolhead.get_kinematics().get_steppers():
            self.steppers[stepper.get_name()]["final_mcu_position"] = (
                stepper.get_mcu_position()
            )
        return {
            "moves": self.moves,
            "steppers": self.steppers,
            "shaper": self.shaper,
            "settings": self.settings,
        }


def main():
    capture_path = sys.argv[1]
    capture = Capture()
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        capture.attach(self)

    toolhead.ToolHead.__init__ = init
    sys.argv = ["klippy"] + sys.argv[2:]
    printer.main()
    with open(capture_path, "w") as f:
        json.dump(capture.finish(), f)


if __name__ == "__main__":
    main()
