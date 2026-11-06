# EtherCAT servo axes must be shaped by an input smoother
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import copy

from klippy.extras.input_shaper import AxisInputSmoother

REASON = (
    "EtherCAT servos need an enabled input smoother (smooth_*) on every"
    " axis they move: a pulse shaper, or no shaping, leaves the velocity"
    " steps of cornering junctions in the commanded motion, and their"
    " acceleration impulses are missing from the servo feedforward"
)


def _axis_problem(input_shaper, shapers, axis):
    if input_shaper is None:
        return "with input shaping off (no [input_shaper])"
    shaper = shapers.get(axis)
    if shaper is None:
        return "which [input_shaper] cannot shape"
    if not isinstance(shaper, AxisInputSmoother):
        return "shaped by the pulse shaper '%s'" % (shaper.get_type(),)
    if not shaper.is_enabled():
        return "with its smoother '%s' disabled" % (shaper.get_type(),)
    return None


def shaping_errors(servos, input_shaper, shapers):
    """One message per servo axis that is not smoothed by shapers (the
    [input_shaper] axis shapers, or a prediction of them)."""
    by_axis = {s.get_axis(): s for s in shapers}
    errors = []
    for servo in servos:
        for axis in servo.moved_axes():
            problem = _axis_problem(input_shaper, by_axis, axis)
            if problem is not None:
                errors.append(
                    "%s moves %s %s" % (servo.get_name(), axis.upper(), problem)
                )
    return errors


def describe(errors):
    return "%s; %s" % ("; ".join(errors), REASON)


class ServoShapingGuard:
    """Keeps [input_shaper] from leaving a servo axis unsmoothed.

    The check runs at connect and before SET_INPUT_SHAPER and
    DISABLE_INPUT_SHAPER take effect, on a copy of the axis shapers the
    command would leave. Any of the three commands that still ends with a
    servo axis unsmoothed (input_shaper disables an axis whose parameters
    the kinematics refused) shuts the printer down.
    """

    def __init__(self, printer, servos):
        self.printer = printer
        self.servos = servos
        self.input_shaper = printer.lookup_object("input_shaper", None)

    def check_config(self):
        shapers = self._current()
        errors = shaping_errors(self.servos, self.input_shaper, shapers)
        if errors:
            raise self.printer.config_error(describe(errors))
        if self.input_shaper is None:
            return
        gcode = self.printer.lookup_object("gcode")
        shaper = self.input_shaper
        for name, handler, predict in (
            ("SET_INPUT_SHAPER", shaper.cmd_SET_INPUT_SHAPER, self._after_set),
            (
                "DISABLE_INPUT_SHAPER",
                shaper.cmd_DISABLE_INPUT_SHAPER,
                self._after_disable,
            ),
            ("ENABLE_INPUT_SHAPER", shaper.cmd_ENABLE_INPUT_SHAPER, None),
        ):
            gcode.register_command(name, None)
            gcode.register_command(
                name,
                self._guarded(handler, predict),
                desc=getattr(shaper, "cmd_%s_help" % (name,)),
            )

    def _current(self):
        if self.input_shaper is None:
            return []
        return self.input_shaper.get_shapers()

    def _after_set(self, gcmd):
        if not gcmd.get_command_parameters():
            return self._current()
        factory = self.input_shaper.shaper_factory
        return [
            factory.update_shaper(copy.deepcopy(s), gcmd)
            for s in self._current()
        ]

    def _after_disable(self, gcmd):
        axes = {a.strip().lower() for a in gcmd.get("AXIS", "").split(",")}
        shapers = [copy.deepcopy(s) for s in self._current()]
        for s in shapers:
            if s.get_axis() in axes:
                s.disable_shaping()
        return shapers

    def _guarded(self, handler, predict):
        def cmd(gcmd):
            if predict is not None:
                errors = shaping_errors(
                    self.servos, self.input_shaper, predict(gcmd)
                )
                if errors:
                    raise gcmd.error(describe(errors))
            try:
                handler(gcmd)
            finally:
                errors = shaping_errors(
                    self.servos, self.input_shaper, self._current()
                )
                if errors:
                    self.printer.invoke_shutdown(
                        "ethercat: %s" % (describe(errors),)
                    )

        return cmd
