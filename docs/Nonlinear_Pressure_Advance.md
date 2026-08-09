# Nonlinear pressure advance

Serval supports nonlinear pressure advance as a motion post-processor. It is not the retired Kalico `bleeding-edge-v2` `[pa_test]`/`RUN_PA_TEST` workflow, and it does not use that workflow's `SET_PRESSURE_ADVANCE OFFSET`, `VELOCITY`, or `TIME_OFFSET` parameters.

Use one nonlinear post-processor on the follower/extruder axis:

```ini
[post_processor extruder_pa]
type: tanh_pressure_advance
linear_advance: 0.04
nonlinear_offset: 0.0
linearization_velocity: 10.0

[axis e]
follows: x, y, z
motors: extruder_motor
post_processors: extruder_pa
```

`recipr_pressure_advance` accepts the same three parameters and may be used instead of `tanh_pressure_advance`. Do not attach both nonlinear models, or a nonlinear and linear pressure-advance processor, to the same axis.

## Model and parameters

The commanded advance is:

```
advance(v) = linear_advance * v + nonlinear_offset * s(v / linearization_velocity)
```

where `s` is either a bounded odd `tanh` curve or the reciprocal curve. Both nonlinear terms have the same small-signal slope, `nonlinear_offset / linearization_velocity`; `tanh` approaches its bound more quickly, while `recipr` approaches it more gradually.

- `linear_advance` is non-negative.
- `nonlinear_offset` is non-negative. At zero, the processor reduces to linear pressure advance.
- `linearization_velocity` must be positive.

These parameters affect motor demand. Start conservatively, validate the entire motion configuration and extrusion limits, and tune on a controlled test rather than copying values from a different hotend, filament path, or machine. `SET_POST_PROCESSOR` can update a named processor for future replanning.

For the complete schema, axis setup, and validation constraints, see [Motion configuration reference](Config_Reference_Motion.md#tanh_pressure_advance-and-recipr_pressure_advance).
