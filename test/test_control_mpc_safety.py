from klippy.extras.control_mpc import (
    FILAMENT_TEMP_SRC_FIXED,
    ControlMPC,
)


class FakePrinter:
    def __init__(self):
        self.shutdowns = []

    def lookup_object(self, name):
        return None

    def invoke_async_shutdown(self, message):
        self.shutdowns.append(message)


class FakeHeater:
    def __init__(self):
        self.printer = FakePrinter()
        self.pwm = []

    def get_max_power(self):
        return 1.0

    def get_name(self):
        return "extruder"

    def set_pwm(self, read_time, duty):
        self.pwm.append((read_time, duty))


def mpc_profile():
    return {
        "block_heat_capacity": 24.5422,
        "ambient_transfer": 0.137173,
        "target_reach_time": 2.0,
        "heater_power": 62.0,
        "smoothing": 0.01,
        "sensor_responsiveness": 0.0593174,
        "min_ambient_change": 1.0,
        "steady_state_rate": 0.5,
        "filament_diameter": 1.75,
        "filament_density": 1.04,
        "filament_heat_capacity": 1.8,
        "maximum_retract": 2.0,
        "filament_temp_src": (FILAMENT_TEMP_SRC_FIXED, 25.0),
        "ambient_temp_sensor": None,
        "cooling_fan": None,
        "fan_ambient_transfer": [0.137173],
    }


def test_mpc_shuts_down_instead_of_powering_far_above_target():
    heater = FakeHeater()
    control = ControlMPC(mpc_profile(), heater, load_clean=True, register=False)
    control.state_block_temp = 200.0
    control.state_sensor_temp = 200.0
    control.last_temp_time = 9.9

    control.temperature_update(10.0, 225.0, 205.0)

    assert heater.pwm == [(10.0, 0.0)]
    assert control.last_power == 0.0
    assert len(heater.printer.shutdowns) == 1
    assert "20.0C above the 205.0C target" in heater.printer.shutdowns[0]
