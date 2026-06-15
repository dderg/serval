# Move a single motor of a multi-stepper axis via host-planned correction moves
#
# This file may be distributed under the terms of the GNU GPLv3 license.

ADJUST_SETTLE_PAD = 0.05


class MotorAdjust:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "MOTOR_ADJUST",
            self.cmd_MOTOR_ADJUST,
            desc=self.cmd_MOTOR_ADJUST_help,
        )

    cmd_MOTOR_ADJUST_help = (
        "Move a single motor of a multi-motor axis by DELTA mm without"
        " changing the commanded axis position"
    )

    def _ensure_motor_enabled(self, toolhead, stepper_name):
        stepper_enable = self.printer.lookup_object("stepper_enable", None)
        if stepper_enable is None:
            return
        try:
            enable_line = stepper_enable.lookup_enable(stepper_name)
        except Exception:
            return
        if not enable_line.is_motor_enabled():
            enable_line.motor_enable(toolhead.get_last_move_time())
            toolhead.wait_moves()

    def adjust(self, stepper_name, delta_mm, speed, accel):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.wait_moves_and_mcu()
        mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(stepper_name)
        self._ensure_motor_enabled(toolhead, stepper_name)
        reactor = self.printer.get_reactor()
        duration = toolhead.submit_motor_adjust(
            mcu_id, axis_idx, motor_idx, delta_mm, speed, accel
        )
        deadline = reactor.monotonic() + duration + ADJUST_SETTLE_PAD
        while reactor.monotonic() < deadline:
            reactor.pause(reactor.monotonic() + 0.01)

    def cmd_MOTOR_ADJUST(self, gcmd):
        stepper_name = gcmd.get("MOTOR")
        delta_mm = gcmd.get_float("DELTA")
        speed = gcmd.get_float("SPEED", 5.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 100.0, above=0.0)
        self.adjust(stepper_name, delta_mm, speed, accel)
        gcmd.respond_info(
            "motor %s adjusted by %.6f mm" % (stepper_name, delta_mm)
        )


def load_config(config):
    return MotorAdjust(config)
