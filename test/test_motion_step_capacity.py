import logging
from types import SimpleNamespace

import pytest
from fakes import FakeMcu, FakePrinter, FakeStepper
from fakes.printer import FakeConfigError

from klippy.kinematics.extruder import PrinterExtruder
from klippy.motion_step_capacity import CapacityMotor, mcu_step_capacity

F401_CLOCK = 84_000_000
F401_BENCHMARK = {
    "pulsed_ticks_per_step": 301,
    "both_edge_ticks_per_step": 160,
    "simulated": False,
}
AB_STEP = 40.0 / 3200
Z_STEP = 8.0 / 3200
E_STEP = 7.73 / 3200
COREXY_A = (1.0, 1.0, 0.0)
COREXY_B = (1.0, -1.0, 0.0)
Z_LANE = (0.0, 0.0, 1.0)


def bench_mcu(benchmark=F401_BENCHMARK, clock=F401_CLOCK):
    return FakeMcu(
        name="mcu", constants={"CLOCK_FREQ": clock}, step_benchmark=benchmark
    )


def extruder(max_velocity):
    obj = PrinterExtruder.__new__(PrinterExtruder)
    obj.axis_name = "e"
    obj.max_velocity = max_velocity
    obj.max_extrude_only_velocity = max_velocity
    return obj


def bench_motion(max_velocity=1000.0, max_z_velocity=25.0):
    return SimpleNamespace(
        printer=FakePrinter(objects={"extruder": extruder(266.0)}),
        _max_velocity=max_velocity,
        max_z_velocity=max_z_velocity,
        kinematics_decl=("corexy", [], [("e", ["extruder"], 3)]),
    )


def bench_motors(both_edge=False, ab_step=AB_STEP):
    def stepper(name, step_dist, weights, edge=False):
        return FakeStepper(
            name=name,
            step_dist=step_dist,
            step_both_edge=edge,
            lane_weights=weights,
        )

    return [
        CapacityMotor(
            0, stepper("stepper_a", ab_step, COREXY_A, both_edge), True
        ),
        CapacityMotor(
            1, stepper("stepper_b", ab_step, COREXY_B, both_edge), True
        ),
        CapacityMotor(2, stepper("stepper_z", Z_STEP, Z_LANE), True),
        CapacityMotor(3, stepper("extruder", E_STEP, None), True),
    ]


def stepped(name, step_dist, mode="pulsed"):
    return {"name": name, "step_distance": step_dist, "mode": mode}


def test_the_bench_printer_hands_the_planner_its_benchmark(caplog):
    mcu = bench_mcu()
    with caplog.at_level(logging.INFO):
        step_benchmark = mcu_step_capacity(
            bench_motion(), "mcu", mcu, bench_motors()
        )
    assert step_benchmark == {
        "mcu": "mcu",
        "clock_freq": F401_CLOCK,
        "capacity_fraction": 0.5,
        "pulsed_ticks_per_step": 301,
        "both_edge_ticks_per_step": 160,
        "simulated": False,
        "motors": [
            stepped("stepper_a", AB_STEP),
            stepped("stepper_b", AB_STEP),
            stepped("stepper_z", Z_STEP),
            stepped("extruder", E_STEP),
        ],
    }
    assert mcu.step_capacity == step_benchmark
    summary = caplog.text
    assert "mcu 'mcu' step capacity" in summary
    assert "301 ticks/step pulsed and 160 ticks/step both-edge" in summary
    assert "139535 steps/s pulsed (50.0 % of the 84000000 Hz clock)" in summary
    assert "stepper_a 113137 steps/s pulsed" in summary
    assert "stepper_z 10000 steps/s pulsed" in summary
    assert "extruder 110116 steps/s pulsed" in summary
    assert "caps can bind" in summary


def test_both_edge_motors_cost_what_that_mode_measured(caplog):
    with caplog.at_level(logging.INFO):
        step_benchmark = mcu_step_capacity(
            bench_motion(), "mcu", bench_mcu(), bench_motors(both_edge=True)
        )
    assert [m["mode"] for m in step_benchmark["motors"]] == [
        "both-edge",
        "both-edge",
        "pulsed",
        "pulsed",
    ]
    assert "262500 steps/s both-edge" in caplog.text


def test_slow_moves_never_bind(caplog):
    with caplog.at_level(logging.INFO):
        mcu_step_capacity(
            bench_motion(max_velocity=100.0, max_z_velocity=5.0),
            "mcu",
            bench_mcu(),
            bench_motors()[:3],
        )
    assert "caps can never bind" in caplog.text


def test_phase_stepped_motors_dispatch_no_steps():
    motors = bench_motors()
    motors[0] = motors[0]._replace(stepping=False)
    step_benchmark = mcu_step_capacity(
        bench_motion(), "mcu", bench_mcu(), motors
    )
    assert [m["name"] for m in step_benchmark["motors"]] == [
        "stepper_b",
        "stepper_z",
        "extruder",
    ]


def test_a_simulated_mcu_is_not_capped_and_says_so(caplog):
    simulated = {
        "pulsed_ticks_per_step": 0,
        "both_edge_ticks_per_step": 0,
        "simulated": True,
    }
    mcu = bench_mcu(simulated)
    with caplog.at_level(logging.INFO):
        assert (
            mcu_step_capacity(bench_motion(), "mcu", mcu, bench_motors())
            is None
        )
    assert mcu.step_capacity is None
    assert "mcu 'mcu' is simulated; its step capacity is not modeled" in (
        caplog.text
    )


def test_an_mcu_without_steppers_reports_no_benchmark_and_is_not_capped():
    assert (
        mcu_step_capacity(
            bench_motion(), "beacon", bench_mcu(benchmark=None), []
        )
        is None
    )


def test_a_stepper_on_an_mcu_without_a_benchmark_is_a_config_error():
    with pytest.raises(FakeConfigError, match="mcu 'beacon' drives steppers"):
        mcu_step_capacity(
            bench_motion(), "beacon", bench_mcu(benchmark=None), bench_motors()
        )


def test_an_mcu_that_cannot_step_a_sane_axis_speed_is_refused():
    slow_mcu = bench_mcu(
        benchmark={
            "pulsed_ticks_per_step": 600,
            "both_edge_ticks_per_step": 300,
            "simulated": False,
        },
        clock=16_000_000,
    )
    with pytest.raises(FakeConfigError) as refused:
        mcu_step_capacity(
            bench_motion(),
            "mcu",
            slow_mcu,
            bench_motors(ab_step=40.0 / 12800),
        )
    message = str(refused.value)
    assert "mcu 'mcu' cannot step a pure X move at 50.0 mm/s" in message
    assert "600 ticks/step pulsed and 300 ticks/step both-edge" in message
    assert "13333 steps/s pulsed" in message
    assert "stepper_a 16000 steps/s (pulsed, 9600000 ticks/s)" in message
    assert "stepper_b 16000 steps/s" in message
    assert "reaches only 20.8 mm/s" in message
    assert "fewer microsteps" in message
    assert "dedge" in message
    assert "another MCU" in message


def test_z_only_has_to_reach_its_own_lower_limit():
    slow_z = bench_mcu(
        benchmark={
            "pulsed_ticks_per_step": 600,
            "both_edge_ticks_per_step": 300,
            "simulated": False,
        },
        clock=12_000_000,
    )
    motors = bench_motors()[2:3]
    assert mcu_step_capacity(
        bench_motion(max_z_velocity=20.0), "mcu", slow_z, motors
    )
    with pytest.raises(FakeConfigError, match="pure Z move at 50.0 mm/s"):
        mcu_step_capacity(
            bench_motion(max_z_velocity=100.0), "mcu", slow_z, motors
        )
