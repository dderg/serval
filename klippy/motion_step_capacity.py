"""How many steps per second each MCU can dispatch, from the cost it measured
for one step at connect (src/stepper_bench.c), and what the configured
motors ask of it.

STEP_CAPACITY_FRACTION: on the F401 (tools/bench/README.md, "MCU step-rate
capacity") p99 step lateness holds at 4.1-5.1 us up to 225 k steps/s, rises
to 7-8 us between 225 and 275 k and ~10 us beyond, and the MCU shuts down
("Stepper too far in past") at ~300 k: the knee sits at 75 % of the ceiling.
The self-benchmark reads 0.93 of that ceiling, so 0.75 of the benchmarked
capacity keeps every move below the knee with margin.

STEP_CAPACITY_MIN_AXIS_VELOCITY: below 50 mm/s an axis cannot run the
first-layer and perimeter speeds slicers default to (20-60 mm/s), so a
machine whose MCU cannot step a pure move along an axis that fast is
misconfigured rather than slow; z asks only for its own max_z_velocity when
that is lower.
"""

import logging
import math
from collections import namedtuple

from .kinematics.extruder import PrinterExtruder

STEP_CAPACITY_FRACTION = 0.75
STEP_CAPACITY_MIN_AXIS_VELOCITY = 50.0
SPATIAL_AXIS_NAMES = ("x", "y", "z")
UNIT_COORDS = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
MODE_PULSED = "pulsed"
MODE_BOTH_EDGE = "both-edge"
BENCHMARK_KEYS = {
    MODE_PULSED: "pulsed_ticks_per_step",
    MODE_BOTH_EDGE: "both_edge_ticks_per_step",
}
REMEDIES = (
    "use fewer microsteps or a larger rotation_distance on those motors, "
    "step them on both edges (a TMC driver's dedge), or move motors to "
    "another MCU"
)

CapacityMotor = namedtuple("CapacityMotor", ["slot", "stepper", "stepping"])
SteppedMotor = namedtuple(
    "SteppedMotor", ["name", "slot", "step_dist", "mode", "ticks_per_step"]
)


def stepper_mode(stepper):
    _pulse_duration, both_edge = stepper.get_pulse_duration()
    return MODE_BOTH_EDGE if both_edge else MODE_PULSED


def mcu_step_capacity(motion, name, mcu_obj, motors):
    """The capacity in clock ticks per second the planner holds `mcu_obj` to
    and each motor's ticks per step, in `motors` order; `(None, [])` where
    nothing is capped."""
    if mcu_obj is None:
        return None, []
    stepped = [m for m in motors if m.stepping]
    benchmark = mcu_obj.get_step_benchmark()
    if benchmark is None:
        if stepped:
            raise motion.printer.config_error(
                "mcu '%s' drives steppers %s, but its firmware has no "
                "steppers to report a step benchmark for"
                % (name, sorted(m.stepper.get_name() for m in stepped))
            )
        return None, []
    if benchmark["simulated"]:
        logging.info(
            "Motion: mcu '%s' is simulated; its step capacity is not "
            "modeled and no move is capped by it",
            name,
        )
        return None, []
    if not stepped:
        return None, []
    clock_freq = mcu_obj.get_constant_float("CLOCK_FREQ")
    ticks_per_s = STEP_CAPACITY_FRACTION * clock_freq
    ticks_per_step = [
        benchmark[BENCHMARK_KEYS[stepper_mode(m.stepper)]] if m.stepping else 0
        for m in motors
    ]
    stepped_motors = [
        SteppedMotor(
            m.stepper.get_name(),
            m.slot,
            m.stepper.get_step_dist(),
            stepper_mode(m.stepper),
            ticks,
        )
        for m, ticks in zip(motors, ticks_per_step)
        if m.stepping
    ]
    capacity = _McuCapacity(name, benchmark, clock_freq, ticks_per_s)
    weights = {
        m.stepper.get_name(): _lane_weights(m.stepper)
        for m in stepped
        if m.slot < len(SPATIAL_AXIS_NAMES)
    }
    capacity.refuse_unreachable_axes(motion, stepped_motors, weights)
    capacity.log_summary(motion, stepped_motors, weights)
    return ticks_per_s, ticks_per_step


def _lane_weights(stepper):
    return [stepper.calc_position_from_coord(list(u)) for u in UNIT_COORDS]


def _axis_limits(motion):
    return (motion._max_velocity, motion._max_velocity, motion.max_z_velocity)


def _follower_velocity(motion, slot):
    _kind, _lanes, followers = motion.kinematics_decl
    axis_name = next(name for name, _m, s in followers if s == slot)
    for _name, obj in motion.printer.lookup_objects():
        if isinstance(obj, PrinterExtruder) and obj.axis_name == axis_name:
            return max(obj.max_velocity, obj.max_extrude_only_velocity)
    return None


class _McuCapacity:
    def __init__(self, name, benchmark, clock_freq, ticks_per_s):
        self.name = name
        self.benchmark = benchmark
        self.clock_freq = clock_freq
        self.ticks_per_s = ticks_per_s

    def steps_per_s(self, mode):
        return self.ticks_per_s / self.benchmark[BENCHMARK_KEYS[mode]]

    def describe(self, modes):
        return (
            "%d ticks/step pulsed and %d ticks/step both-edge measured, "
            "capacity %s (%.0f %% of the %.0f Hz clock)"
            % (
                self.benchmark["pulsed_ticks_per_step"],
                self.benchmark["both_edge_ticks_per_step"],
                ", ".join(
                    "%.0f steps/s %s" % (self.steps_per_s(mode), mode)
                    for mode in modes
                ),
                100.0 * STEP_CAPACITY_FRACTION,
                self.clock_freq,
            )
        )

    def refuse_unreachable_axes(self, motion, motors, weights):
        spatial = [m for m in motors if m.name in weights]
        for axis, (axis_name, axis_limit) in enumerate(
            zip(SPATIAL_AXIS_NAMES, _axis_limits(motion))
        ):
            required = min(axis_limit, STEP_CAPACITY_MIN_AXIS_VELOCITY)
            demand = [
                (m, abs(weights[m.name][axis]) * required / m.step_dist)
                for m in spatial
                if weights[m.name][axis]
            ]
            used = sum(steps * m.ticks_per_step for m, steps in demand)
            if used <= self.ticks_per_s:
                continue
            reachable = required * self.ticks_per_s / used
            raise motion.printer.config_error(
                "mcu '%s' cannot step a pure %s move at %.1f mm/s (the lesser "
                "of the axis's max velocity and %.0f mm/s): %s; that move "
                "asks %s, %.0f %% of its capacity, so it reaches only "
                "%.1f mm/s. To fix it, %s."
                % (
                    self.name,
                    axis_name.upper(),
                    required,
                    STEP_CAPACITY_MIN_AXIS_VELOCITY,
                    self.describe(sorted({m.mode for m, _ in demand})),
                    ", ".join(
                        "%s %.0f steps/s (%s, %.0f ticks/s)"
                        % (m.name, steps, m.mode, steps * m.ticks_per_step)
                        for m, steps in demand
                    ),
                    100.0 * used / self.ticks_per_s,
                    reachable,
                    REMEDIES,
                )
            )

    def log_summary(self, motion, motors, weights):
        max_velocity, _, max_z_velocity = _axis_limits(motion)
        worst = []
        for m in motors:
            if m.name in weights:
                w = weights[m.name]
                motor_velocity = (
                    math.hypot(w[0], w[1]) * max_velocity
                    + abs(w[2]) * max_z_velocity
                )
            else:
                motor_velocity = _follower_velocity(motion, m.slot)
            worst.append(
                (
                    m,
                    None
                    if motor_velocity is None
                    else motor_velocity / m.step_dist,
                )
            )
        unbounded = any(steps is None for _, steps in worst)
        used = sum(
            steps * m.ticks_per_step for m, steps in worst if steps is not None
        )
        logging.info(
            "Motion: mcu '%s' step capacity: %s; worst-case demand at the "
            "configured max speeds: %s; %.0f %% of capacity summed, an "
            "upper bound on any one move; step capacity caps %s",
            self.name,
            self.describe(sorted({m.mode for m in motors})),
            ", ".join(
                "%s %s"
                % (
                    m.name,
                    "unbounded (no extruder limits)"
                    if steps is None
                    else "%.0f steps/s %s" % (steps, m.mode),
                )
                for m, steps in worst
            ),
            100.0 * used / self.ticks_per_s,
            "can bind"
            if unbounded or used > self.ticks_per_s
            else "can never bind",
        )
