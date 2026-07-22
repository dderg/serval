from fakes import FakeConfig, FakeReactor

from klippy.extras.heaters.control import (
    PID_PARAM_BASE,
    ControlBangBang,
    ControlDualLoopPID,
    ControlPID,
    ControlVelocityPID,
)


class FakeControlHeater:
    def __init__(self, max_power=1.0, smooth_time=1.0, temp=25.0):
        self.max_power = max_power
        self.smooth_time = smooth_time
        self.inv_smooth_time = 1.0 / smooth_time
        self.last_pwm_value = 0.0
        self.reactor = FakeReactor()
        self.config = FakeConfig(values={"inner_max_temp": 120.0})
        self.pwm_calls = []
        self._temp = temp

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def set_inv_smooth_time(self, inv_smooth_time):
        self.inv_smooth_time = inv_smooth_time

    def get_temp(self, eventtime):
        return self._temp, 0.0

    def set_pwm(self, read_time, value):
        self.pwm_calls.append((read_time, value))
        self.last_pwm_value = value


def pid_profile(kp=100.0, ki=1.0, kd=10.0, **extra):
    profile = {
        "control": "pid",
        "name": "default",
        "pid_kp": kp,
        "pid_ki": ki,
        "pid_kd": kd,
        "smooth_time": None,
        "pid_target": 200.0,
        "pid_tolerance": 0.02,
    }
    profile.update(extra)
    return profile


def test_bangbang_hysteresis_switches_around_target():
    heater = FakeControlHeater()
    control = ControlBangBang(
        {"max_delta": 2.0, "control": "watermark"}, heater
    )
    control.temperature_update(1.0, 190.0, 200.0)
    assert heater.pwm_calls[-1] == (1.0, 1.0)
    control.temperature_update(2.0, 201.0, 200.0)
    assert heater.pwm_calls[-1][1] == 1.0
    control.temperature_update(3.0, 202.5, 200.0)
    assert heater.pwm_calls[-1][1] == 0.0
    control.temperature_update(4.0, 199.0, 200.0)
    assert heater.pwm_calls[-1][1] == 0.0
    control.temperature_update(5.0, 197.5, 200.0)
    assert heater.pwm_calls[-1][1] == 1.0


def test_pid_output_matches_reference_computation():
    heater = FakeControlHeater()
    control = ControlPID(pid_profile(), heater, load_clean=True)
    read_time, temp, target = 2.0, 100.0, 200.0
    co, bounded_co = control.calculate_output(read_time, temp, target)
    kp = 100.0 / PID_PARAM_BASE
    ki = 1.0 / PID_PARAM_BASE
    kd = 10.0 / PID_PARAM_BASE
    temp_deriv = (temp - 25.0) / 2.0
    temp_integ = min(100.0 * 2.0, heater.max_power / ki)
    expected = kp * 100.0 + ki * temp_integ - kd * temp_deriv
    assert co == expected
    assert bounded_co == max(0.0, min(1.0, expected))


def test_pid_integral_windup_is_clamped():
    heater = FakeControlHeater()
    control = ControlPID(pid_profile(), heater, load_clean=True)
    for i in range(1, 200):
        control.calculate_output(float(i * 10), 100.0, 200.0)
    ki = 1.0 / PID_PARAM_BASE
    assert control.prev_temp_integ <= heater.max_power / ki


def test_velocity_pid_forces_zero_pwm_when_target_cleared():
    heater = FakeControlHeater()
    heater.last_pwm_value = 0.7
    control = ControlVelocityPID(pid_profile(), heater)
    control.temperature_update(1.0, 150.0, 0.0)
    assert heater.pwm_calls[-1][1] == 0.0


def test_dual_loop_takes_minimum_of_both_loops():
    heater = FakeControlHeater()
    profile = pid_profile(
        control="dual_loop_pid",
        inner_pid_kp=100.0,
        inner_pid_ki=1.0,
        inner_pid_kd=10.0,
    )
    control = ControlDualLoopPID(profile, heater, load_clean=True)
    primary_co, _ = control.primary_pid.calculate_output(1.0, 100.0, 200.0)
    heater2 = FakeControlHeater()
    control2 = ControlDualLoopPID(profile, heater2, load_clean=True)
    control2.temperature_update(1.0, 100.0, 200.0, 119.0)
    secondary_co, _ = control.secondary_pid.calculate_output(1.0, 119.0, 120.0)
    expected = max(0.0, min(1.0, min(primary_co, secondary_co)))
    assert heater2.pwm_calls[-1][1] == expected


def test_dual_loop_requires_secondary_temperature():
    heater = FakeControlHeater()
    profile = pid_profile(
        control="dual_loop_pid",
        inner_pid_kp=100.0,
        inner_pid_ki=1.0,
        inner_pid_kd=10.0,
    )
    control = ControlDualLoopPID(profile, heater, load_clean=True)
    try:
        control.temperature_update(1.0, 100.0, 200.0, None)
        assert False, "expected ValueError"
    except ValueError:
        pass
