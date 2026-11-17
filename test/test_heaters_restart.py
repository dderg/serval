import pytest
from fakes import FakeConfig, FakePrinter

from klippy import gcode as gcode_mod
from klippy.extras import heaters

REPORT_S = 0.3
RESTART_WAIT_AFTER_M400_S = 0.21
WARM_C = 30.0
TARGET_C = 150.0
LOW_DUTY = 0.03
FIRST_REPORT = 10.0
SECOND_REPORT = FIRST_REPORT + REPORT_S
SENSOR_TYPE = "restart_test_sensor"
FULL_POWER_CONTROL = {"control": "watermark"}
PROPORTIONAL_CONTROL = {
    "control": "pid",
    "pid_kp": heaters.PID_PARAM_BASE,
    "pid_ki": 0.0,
    "pid_kd": 0.0,
}


class _Sensor:
    report = None
    temp = WARM_C

    def setup_minmax(self, min_temp, max_temp):
        pass

    def setup_callback(self, callback):
        self.report = callback

    def get_report_time_delta(self):
        return REPORT_S


class _Mcu:
    def __init__(self, sensor):
        self.sensor = sensor
        self.print_time = FIRST_REPORT
        self.next_report = FIRST_REPORT

    def estimated_print_time(self, eventtime):
        return self.print_time

    def run_until(self, print_time):
        while self.next_report <= print_time:
            self.print_time = self.next_report
            self.sensor.report(self.next_report, self.sensor.temp)
            self.next_report += REPORT_S
        self.print_time = max(self.print_time, print_time)


class _HeaterPin:
    def __init__(self, mcu):
        self.mcu = mcu
        self.commands = []

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        pass

    def setup_max_duration(self, max_duration):
        pass

    def get_mcu(self):
        return self.mcu

    def set_pwm(self, print_time, value):
        self.commands.append((print_time, value))


class _Pins:
    def __init__(self, pin):
        self.pin = pin

    def setup_pin(self, pin_type, pin_desc):
        return self.pin


class _ConfigfileWithoutSensorPresets:
    def __init__(self, printer):
        self.printer = printer

    def read_config(self, filename):
        return FakeConfig(self.printer)


class _ToolheadAfterM400:
    def __init__(self, mcu):
        self.mcu = mcu

    def get_last_move_time(self):
        return self.mcu.print_time

    def dwell(self, delay):
        pass

    def wait_moves(self):
        self.mcu.run_until(self.mcu.print_time + RESTART_WAIT_AFTER_M400_S)

    def wait_until_print_time(self, print_time):
        self.mcu.run_until(print_time)


class _Printer(FakePrinter):
    def __init__(self, pin, mcu):
        FakePrinter.__init__(self)
        self.handlers = {}
        self.pin = pin
        self.mcu = mcu

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

    def request_exit(self, result):
        self.commands_at_exit = list(self.pin.commands)
        self.print_time_at_exit = self.mcu.print_time


def _heated_until_the_second_report(control, second_report_temp):
    sensor = _Sensor()
    mcu = _Mcu(sensor)
    pin = _HeaterPin(mcu)
    printer = _Printer(pin, mcu)
    printer.add_object("configfile", _ConfigfileWithoutSensorPresets(printer))
    printer.add_object("pins", _Pins(pin))
    printer.add_object("verify_heater extruder", object())
    printer.add_object("pid_calibrate", object())
    printer.add_object("toolhead", _ToolheadAfterM400(mcu))
    dispatch = gcode_mod.GCodeDispatch(printer)
    printer.add_object("gcode", dispatch)
    values = {
        "heater_pin": "PA1",
        "sensor_type": SENSOR_TYPE,
        "min_temp": 0.0,
        "max_temp": 300.0,
    }
    values.update(control)
    config = FakeConfig(printer, "extruder", values)
    printer_heaters = heaters.PrinterHeaters(config)
    printer_heaters.add_sensor_factory(SENSOR_TYPE, lambda config: sensor)
    heater = printer_heaters.setup_heater(config)
    printer.send_event("klippy:ready")
    heater.stats(FIRST_REPORT)
    heater.set_temp(TARGET_C)
    mcu.run_until(FIRST_REPORT)
    sensor.temp = second_report_temp
    mcu.run_until(SECOND_REPORT)
    return dispatch, mcu, pin, printer


def _assert_off_on_the_mcu_at_exit(printer):
    off_time, value = printer.commands_at_exit[-1]
    assert value == 0.0, printer.commands_at_exit
    assert printer.print_time_at_exit >= off_time, (
        printer.print_time_at_exit,
        off_time,
    )


def test_restart_with_no_report_in_its_wait_switches_the_heater_off():
    dispatch, mcu, pin, printer = _heated_until_the_second_report(
        FULL_POWER_CONTROL, WARM_C
    )
    assert pin.commands[-1][1] == 1.0

    dispatch.request_restart("restart")

    _assert_off_on_the_mcu_at_exit(printer)


def test_restart_switches_off_a_heater_below_the_change_threshold():
    dispatch, mcu, pin, printer = _heated_until_the_second_report(
        PROPORTIONAL_CONTROL, TARGET_C - LOW_DUTY
    )
    assert pin.commands[-1] == pytest.approx(
        (SECOND_REPORT + REPORT_S, LOW_DUTY)
    )
    mcu.run_until(SECOND_REPORT + REPORT_S - RESTART_WAIT_AFTER_M400_S)

    dispatch.request_restart("restart")

    _assert_off_on_the_mcu_at_exit(printer)
