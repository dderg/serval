from klippy.extras.control_mpc import (
    FILAMENT_TEMP_SRC_FIXED,
    ControlMPC,
)


class FakePrinter:
    def __init__(self):
        self.shutdowns = []
        self.toolhead = None

    def lookup_object(self, name):
        return self.toolhead if name == "toolhead" else None

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


class FakeExtruder:
    def __init__(self, heater, speed):
        self.heater = heater
        self.speed = speed

    def find_past_position(self, print_time):
        return self.speed * print_time

    def get_heater(self):
        return self.heater


class FakeToolhead:
    def __init__(self, extruder):
        self.extruder = extruder

    def get_extruder(self):
        return self.extruder


def mpc_profile(**overrides):
    profile = {
        "block_heat_capacity": 24.5422,
        "ambient_transfer": 0.137173,
        "target_reach_time": 2.0,
        "heater_power": 62.0,
        "smoothing": 0.83,
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
    profile.update(overrides)
    return profile


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


def test_mpc_corrects_ambient_estimate_at_partial_heater_power():
    heater = FakeHeater()
    control = ControlMPC(mpc_profile(), heater, load_clean=True, register=False)
    control.state_block_temp = 200.0
    control.state_sensor_temp = 200.0
    control.last_power = 50.0
    control.last_temp_time = 9.9

    control.temperature_update(10.0, 200.0, 205.0)

    assert abs(control.state_ambient_temp - 24.9) < 1.0e-9


def test_fixed_filament_temperature_applies_to_model_and_output():
    profile = mpc_profile(filament_temp_src=(FILAMENT_TEMP_SRC_FIXED, 205.0))
    stationary_heater = FakeHeater()
    stationary_control = ControlMPC(
        profile, stationary_heater, load_clean=True, register=False
    )
    moving_heater = FakeHeater()
    moving_heater.printer.toolhead = FakeToolhead(
        FakeExtruder(moving_heater, 20.0)
    )
    moving_control = ControlMPC(
        profile, moving_heater, load_clean=True, register=False
    )
    equilibrium_power = (205.0 - 25.0) * profile["ambient_transfer"]
    for control in (stationary_control, moving_control):
        control.state_block_temp = 205.0
        control.state_sensor_temp = 205.0
        control.last_power = equilibrium_power
        control.last_temp_time = 9.7

    stationary_control.temperature_update(10.0, 205.0, 205.0)
    moving_control.temperature_update(10.0, 205.0, 205.0)

    assert (
        abs(
            moving_control.state_block_temp
            - stationary_control.state_block_temp
        )
        < 1.0e-9
    )
    assert (
        abs(moving_control.last_power - stationary_control.last_power) < 1.0e-9
    )
    assert moving_control.last_loss_filament == 0.0
