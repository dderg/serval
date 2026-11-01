# Support for enable pins on stepper motor drivers
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy.gcode import CommandError

DISABLE_STALL_TIME = 0.100


class StepperEnablePin:
    def __init__(self, mcu_enable, enable_count):
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
        self.is_dedicated = True
        self.last_switch_time = float("-inf")

    def _switch(self, print_time, value):
        if print_time < self.last_switch_time:
            raise CommandError(
                "stepper enable pin switched to %d at print_time %.6f,"
                " before its pending %s at %.6f"
                % (
                    value,
                    print_time,
                    "disable" if value else "enable",
                    self.last_switch_time,
                )
            )
        self.last_switch_time = print_time
        return self.mcu_enable.set_digital(print_time, value)

    def set_enable(self, print_time):
        wait = None
        if not self.enable_count:
            wait = self._switch(print_time, 1)
        self.enable_count += 1
        return wait

    def set_disable(self, print_time):
        if self.enable_count == 1:
            self._switch(print_time, 0)
        self.enable_count -= 1


def setup_enable_pin(printer, pin):
    if pin is None:
        enable = StepperEnablePin(None, 9999)
        enable.is_dedicated = False
        return enable
    ppins = printer.lookup_object("pins")
    pin_params = ppins.lookup_pin(
        pin, can_invert=True, share_type="stepper_enable"
    )
    enable = pin_params.get("class")
    if enable is not None:
        enable.is_dedicated = False
        return enable
    mcu_enable = pin_params["chip"].setup_pin("digital_out", pin_params)
    mcu_enable.setup_max_duration(0.0)
    enable = pin_params["class"] = StepperEnablePin(mcu_enable, 0)
    return enable


class EnableTracking:
    def __init__(self, stepper, enable):
        self.stepper = stepper
        self.enable = enable
        self.callbacks = []
        self.is_enabled = False
        self.stepper.add_active_callback(self.energize)

    def register_state_callback(self, callback):
        self.callbacks.append(callback)

    def energize(self, earliest, latest):
        if self.is_enabled:
            return None
        print_time = max(earliest, self.enable.last_switch_time)
        if print_time > latest:
            raise CommandError(
                "%s cannot be enabled before its move: the enable pin's last"
                " switch is at print_time %.6f, the move may start at %.6f"
                % (self.stepper.get_name(), print_time, latest)
            )
        wait = self.enable.set_enable(print_time)
        self.is_enabled = True

        def notify_state_callbacks():
            if wait is not None:
                wait()
            for cb in self.callbacks:
                cb(print_time, True)

        return notify_state_callbacks

    def motor_enable(self, print_time):
        notify = self.energize(print_time, print_time)
        if notify is not None:
            notify()

    def motor_disable(self, print_time):
        if self.is_enabled:
            for cb in self.callbacks:
                cb(print_time, False)
            self.enable.set_disable(print_time)
            self.is_enabled = False
            self.stepper.add_active_callback(self.energize)

    def is_motor_enabled(self):
        return self.is_enabled

    def has_dedicated_enable(self):
        return self.enable.is_dedicated


class PrinterStepperEnable:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enable_lines = {}
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("M18", self.cmd_M18)
        gcode.register_command("M84", self.cmd_M18)
        gcode.register_command(
            "SET_STEPPER_ENABLE",
            self.cmd_SET_STEPPER_ENABLE,
            desc=self.cmd_SET_STEPPER_ENABLE_help,
        )

    def register_stepper(self, config, mcu_stepper):
        enable = setup_enable_pin(self.printer, config.get("enable_pin", None))
        self.register_motor(mcu_stepper.get_name(), mcu_stepper, enable)

    def register_motor(self, name, motor, enable):
        self.enable_lines[name] = EnableTracking(motor, enable)

    def motor_off(self):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        for el in self.enable_lines.values():
            el.motor_disable(print_time)
        self.printer.send_event("stepper_enable:motor_off", print_time)
        toolhead.dwell_until(print_time + DISABLE_STALL_TIME)

    def motor_debug_enable(self, stepper, enable):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        el = self.enable_lines[stepper]
        if enable:
            toolhead.resync_parked_servos()
            print_time = toolhead.get_last_move_time()
            el.motor_enable(print_time)
            logging.info("%s has been manually enabled", stepper)
        else:
            print_time = toolhead.get_last_move_time()
            el.motor_disable(print_time)
            logging.info("%s has been manually disabled", stepper)
        toolhead.dwell_until(print_time + DISABLE_STALL_TIME)

    def motor_enable_group(self, stepper_names):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        toolhead.resync_parked_servos()
        shared_print_time = toolhead.get_last_move_time()
        notifiers = []
        for name in stepper_names:
            notify = self.enable_lines[name].energize(
                shared_print_time, shared_print_time
            )
            if notify is not None:
                notifiers.append(notify)
            logging.info("%s enabled", name)
        for notify in notifiers:
            notify()
        toolhead.dwell_until(shared_print_time + DISABLE_STALL_TIME)

    def get_status(self, eventtime):
        steppers = {
            name: et.is_motor_enabled()
            for (name, et) in self.enable_lines.items()
        }
        return {"steppers": steppers}

    def _handle_request_restart(self, print_time):
        self.motor_off()

    def cmd_M18(self, gcmd):
        self.motor_off()

    cmd_SET_STEPPER_ENABLE_help = "Enable/disable individual stepper by name"

    def cmd_SET_STEPPER_ENABLE(self, gcmd):
        stepper_name = gcmd.get("STEPPER", None)
        if stepper_name not in self.enable_lines:
            gcmd.respond_info(
                'SET_STEPPER_ENABLE: Invalid stepper "%s"' % (stepper_name,)
            )
            return
        stepper_enable = gcmd.get_int("ENABLE", 1)
        self.motor_debug_enable(stepper_name, stepper_enable)

    def lookup_enable(self, name):
        if name not in self.enable_lines:
            raise self.printer.config_error("Unknown stepper '%s'" % (name,))
        return self.enable_lines[name]

    def get_steppers(self):
        return list(self.enable_lines.keys())


def load_config(config):
    return PrinterStepperEnable(config)
