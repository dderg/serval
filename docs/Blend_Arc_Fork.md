# Blend-Arc Fork Additions

This fork replaces Kalico's junction-deviation / square-corner-velocity
corner handling with a geometric tangent-arc corner blender, and exposes
a shaper-smoothing quality knob that controls the per-axis acceleration
budget at corners. All additions listed here are changes relative to
upstream Kalico.

## New config parameters

### `[printer] corner_deviation`

**Required.** Maximum perpendicular distance (mm) from the original
corner vertex to the blended curve. Replaces `square_corner_velocity`
as the quality-vs-speed knob for corners.

```
[printer]
corner_deviation: 0.05
```

Typical values: `0.03`–`0.2`. Smaller → corners stay closer to the
designed vertex, slower through corners. Larger → more rounding,
faster through corners.

### `[input_shaper] target_smoothing`

**Optional.** Default `0.12`. Position cusp (mm) at a 180° reversal
that the input shaper is allowed to smooth. Defines the per-axis
acceleration budget the corner blender uses when sizing arc velocity:

```
A_axis = 2 · target_smoothing / σ²_T
```

where `σ²_T` is the second moment of the shaper impulse response.
Used both by `find_shaper_max_accel` during shaper tuning and at
runtime by `blendmath._extract_shapers` when sizing corner arcs.

```
[input_shaper]
shaper_freq_x: 62
shaper_type_x: mzv
shaper_freq_y: 40
shaper_type_y: mzv
target_smoothing: 0.12
```

Raising it allows higher corner speeds at the cost of more shaper
residual ringing. Above ~`max_accel · σ²_T / 2`, the corner cap is
dominated by `max_accel` and further increases do not buy corner
speed.

## Deprecated / ignored config

### `[printer] square_corner_velocity`

Parsed for backwards compat but has no effect on motion planning.
Corner behavior is entirely governed by `corner_deviation` and
`target_smoothing` now. Safe to remove from your config.

## New / modified runtime commands

### `SET_VELOCITY_LIMIT CORNER_DEVIATION=<mm>`

Set `corner_deviation` at runtime. Propagates to the next corner
(buffered blends retain the value they were sized with, same semantics
as `VELOCITY=` / `ACCEL=`).

```
SET_VELOCITY_LIMIT CORNER_DEVIATION=0.1
```

`RESET_VELOCITY_LIMIT` restores the config value.

### `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=<mm/s>`

Parsed but ignored. Kept for macro compatibility with stock slicer
start-gcode. Recommend removing from macros.

### `SET_INPUT_SHAPER TARGET_SMOOTHING=<mm>`

Set `target_smoothing` at runtime. Live-read by blendmath on each
blend, so propagates immediately to the next corner. Does not trigger
the C-level shaper rebuild (no step-generation flush) when used alone.

```
SET_INPUT_SHAPER TARGET_SMOOTHING=0.25
```

To change shaper frequency AND smoothing in one command, both take
effect together and the rebuild fires once:

```
SET_INPUT_SHAPER SHAPER_FREQ_X=55 TARGET_SMOOTHING=0.18
```

## Behavior changes

### Corner blending

Upstream Klipper/Kalico uses a velocity cap at corners
(junction-deviation / SCV) but moves along the original polyline
geometry. This fork blends the geometry itself: a tangent arc is
inserted between the two segments, with chord error ≤ `corner_deviation`.
The extruder follows the blended path (E interpolated proportional to
arc length), so pressure advance sees the actual curved motion.

Collinear / near-collinear chains from naive-CAM slicers are collapsed
by a prepass filter before blending.

### Per-segment centripetal cap

Arc polyline segments are each capped at `v² ≤ a_max · R` locally,
not at a single worst-case v across the whole polyline. Allows the
planner to accelerate through the lower-curvature portions of long
blend sequences.

### Shaper-derived corner velocity cap

In addition to centripetal and feedrate caps, corner v is bounded by
a per-axis shaper-smoothing ceiling:

```
v ≤ sqrt(A_axis · R / projection_on_axis)
```

This prevents corner speeds from driving the shaper into overshoot
that exceeds `target_smoothing` at the entry step.

### Pythagorean relaxation

Upstream LinuxCNC-style junction caps use `a_max · √3/2 ≈ 0.866·a_max`
for centripetal to reserve budget for tangential accel. This fork
drops that factor; the truncated-linear segment around the arc handles
tangential separately, so the arc itself can use the full `a_max`.

## Tuning recipe

1. Set `max_accel` to the highest value that doesn't cause ringing on
   long straight moves (this is the ringing-bound limit; use
   resonance tests or visual ghosting to find it).
2. Set `corner_deviation` based on dimensional tolerance you want —
   `0.05mm` is a reasonable starting point, matches a sharp-eyed
   visual inspection.
3. Leave `target_smoothing` at `0.12` (default) initially. If corner
   speed feels slow, raise it in `0.05` increments and retest on a
   representative print. Ghosting scales with `target_smoothing`.
4. Remove `square_corner_velocity` from your config (or leave it; it's
   ignored either way).

## Diagnostics

- `SET_VELOCITY_LIMIT` with no args echoes current values including
  `corner_deviation`.
- `SET_INPUT_SHAPER` with no args echoes per-axis shaper params and
  `target_smoothing`.
- `printer.print_stats.print_duration` (already in Kalico) gives pure
  active-print time, excluding heating and pauses. Use this for
  reliable A/B print-time comparisons.
