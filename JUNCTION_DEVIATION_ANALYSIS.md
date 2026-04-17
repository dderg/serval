# Junction Deviation & Input Shaper Conservatism — Analysis

Working notes for a proposed fork addressing Klipper/Kalico's cornering model
and the conservatism of its input shaper `max_accel` recommendations on
high-acceleration, high-speed printers.

These are research findings, not a spec. Conclusions here inform later design
and planning work.

---

## Problem statement

On a rigid, high-speed printer:

1. The user's config is sized around a rule of thumb of roughly
   `scv ≈ max_accel / 1000` (e.g. `scv=70` with `max_accel=70000`). Prints
   work. Stepper load is not elevated at corners relative to straight
   acceleration phases.
2. Stock Kalico's `SHAPER_CALIBRATE` recommends `max_accel` in the 40–56k
   range for this hardware.
3. Shake&Tune on the *same* data recommends ~9k. A 4–6× gap.
4. Forcing Shake&Tune to run at a low SCV (e.g. 5 mm/s) brings its number
   back in line with stock Klipper.

The goal of the analysis was to pin down *why*, and whether the gap
reflects a real problem with the cornering model or the calibration
formula (or both).

---

## What the code actually does at a corner

A 90° junction between two straight-line moves looks like this in
`kalico/klippy/chelper/trapq.c` and `kalico/klippy/toolhead.py`:

- Moves are split into up to three trapq segments each (accel / cruise /
  decel), each with its own `axes_r` (direction unit vector) and constant
  acceleration. See `trapq.c:119-164` (`trapq_append`).
- Segments are **sequential in `print_time`** — `trapq.c:102` explicitly
  inserts a null move if there's a gap. There is no time overlap between
  moves.
- At a junction, the previous move's decel phase ends at velocity
  `end_v = scv` (not zero). The next move's accel phase starts at
  `start_v = scv`. See `toolhead.py:124-138` (`set_junction`) and
  `toolhead.py:211-215` in the lookahead flush.

Per axis, that means the commanded velocity profile at a 90° X→Y corner is:

| time | X velocity | Y velocity |
|---|---|---|
| just before junction | `scv` | `0` |
| just after junction | `0` | `scv` |

An **instantaneous velocity step of magnitude `scv`** on each axis.
The motion planner does not do arc blending; it computes a velocity cap
for the junction and treats the direction change as zero-duration.

### Why steppers don't skip at this discontinuity

`kin_shaper.c:91-103` convolves the commanded position across move
boundaries:

```c
static inline double
calc_position(struct move *m, int axis, double move_time,
              struct shaper_pulses *sp)
{
    double res = 0.;
    int num_pulses = sp->num_pulses, i;
    for (i = 0; i < num_pulses; ++i) {
        double t = sp->pulses[i].t, a = sp->pulses[i].a;
        res += a * get_axis_position_across_moves(m, axis, move_time + t);
    }
    return res;
}
```

The stepper never sees the raw trapq trajectory — it sees the
**shaped** trajectory. The junction's velocity step is smeared over the
shaper's impulse window (roughly `1/(2·f_shaper)` seconds, ~4–8 ms for
typical printer frequencies). Effective junction acceleration felt by the
stepper is `scv / shaper_window`. At `scv=70, f=120Hz, window≈4ms`:

```
effective_junction_accel ≈ 70 / 0.004 ≈ 17500 mm/s²
```

That's *lower* than a configured `max_accel` of 70k. Corners are less
stressful on the stepper than normal acceleration phases, which matches
the empirical observation that raising SCV does not load the motors.

---

## `junction_deviation` is a mathematical fiction in Klipper

`toolhead.py:787-789`:

```python
def _calc_junction_deviation(self):
    scv2 = self.square_corner_velocity**2
    self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
```

And `toolhead.py:79-122` (`calc_junction`) uses it only to compute a
velocity cap, derived from the Grbl 2011 centripetal-velocity formula:

```python
R_jd = sin_theta_d2 / one_minus_sin_theta_d2
move_jd_v2 = R_jd * self.junction_deviation * self.accel
# ...
max_start_v2 = min(max_start_v2, move_jd_v2, ...)
```

The formula's *derivation* assumes a virtual circular arc of radius `r`
such that `r·(1 − sin(θ/2)) = jd`, and caps speed by centripetal
acceleration. **Klipper does not execute that arc.** It uses the
formula's velocity output as the junction velocity of a zero-duration
direction change.

This is confirmed by the Klipper maintainers:

> "square_corner_velocity model is not really particularly physical...
> it calculates and uses the cornering radius that's just a model and
> does not exist in practice."
> — Dmitry Butyugin, https://klipper.discourse.group/t/square-corner-velocity-what-is-the-reasonable-range-of-values/7298

> SCV is "an extruder quality setting and not a kinematic setting."
> — Kevin O'Connor, https://klipper.discourse.group/t/proportional-acceleration-control/3970

### The scaling trap

Because `jd = scv² · (√2 − 1) / a`, the virtual arc radius **shrinks
with increasing `max_accel`** when `scv` is held constant:

| config | effective `jd` |
|---|---|
| `scv=5, a=5000` | 0.002 mm |
| `scv=5, a=50000` | **0.0002 mm** (200 nm) |
| `scv=50, a=50000` | 0.021 mm |
| `scv=70, a=70000` | 0.029 mm |

At `scv=5, a=50k`, the virtual arc radius is 200 nanometers — below any
physical resolution of a 3D printer. A nonsense number used to cap real
motion.

### The circle traversal pathology

Slicers segment arcs into polygons. Each segment is a shallow-angle
corner. The junction velocity formula:

```
v² = a · jd · sin(θ/2) / (1 − sin(θ/2))
```

For a 36-sided polygon approximating a circle (10° turn per segment),
`sin(5°)/(1 − sin(5°)) ≈ 0.095`:

| config | max speed through each 10° segment |
|---|---|
| `scv=5, a=50000` | **~1 mm/s** |
| `scv=50, a=50000` | ~10 mm/s |

At the stock recommendation of `scv=5` and high `max_accel`, slicer-
approximated circles are capped near 1 mm/s. The finer the slicer's
polygon approximation, the worse it gets. Documented in Klipper Issue
\#4228 as "sharp corners and smooth circles are mutually exclusive."

This means the upstream guidance ("keep `scv=5`, raise `max_accel`") is
mathematically incompatible with printing curved geometry at the target
acceleration.

---

## Input shaper calibration uses the same SCV value

`klippy/extras/shaper_calibrate.py` `_get_shaper_smoothing` (~line 240)
includes the `offset_90` term:

```python
for i in range(n):
    if T[i] >= ts:
        offset_90 += A[i] * (scv + half_accel * (T[i] - ts)) * (T[i] - ts)
    offset_180 += A[i] * half_accel * (T[i] - ts) ** 2
offset_90 *= inv_D * math.sqrt(2.0)
offset_180 *= inv_D
return max(offset_90, offset_180)
```

And `find_shaper_max_accel` bisects for the largest acceleration where
`max(offset_90, offset_180) ≤ TARGET_SMOOTHING` (hardcoded 0.12 mm).

The `offset_90` term models the commanded-vs-shaped position deviation
at a 90° corner assuming the perpendicular axis holds velocity `scv` for
the full shaper settling window. Linear in `scv`. At high SCV this
term dominates the smoothing budget and collapses `max_accel`.

`TARGET_SMOOTHING = 0.12` mm is the **print quality** budget — how much
corner rounding from shaper convolution is acceptable. It is not a
stepper load or step-skip constraint. Its 0.12 value is documented only
as "empirically-derived... produces good projections for max_accel
without much smoothing." No public derivation, no empirical validation
dataset, no discussion of how it should scale with hardware.

---

## What actually explains the 9k-vs-40k gap

Both tools call the same Klipper algorithm. Inputs:

| input | stock Kalico `SHAPER_CALIBRATE` | Shake&Tune |
|---|---|---|
| `scv` | toolhead status | toolhead status (overridable) |
| `max_smoothing` | config | config |
| `max_freq` | computed | computed |
| `test_damping_ratios` | `[0.075, 0.1, 0.15]` | same |
| **`damping_ratio`** | `None` → `0.1` default | **measured ζ from PSD** |

The damping ratio is the only inputlevel difference. It shifts shaper
coefficients (A, T) slightly, which moves `max_accel` by ~5–10% for ZV
at matched SCV — not 4–6×.

The dominant factor, as analyzed above, is the `scv` input value
combined with the formula's linear penalty. When SCV is high, both
tools converge to low max_accel. When SCV is low, both converge high.

Hypothesis for the user's specific data:
- Klipper's published graph was taken at `scv≈5` (toolhead default at
  that calibration moment), giving 40–56k.
- Shake&Tune was run with `scv=70` (the user's configured value),
  crushing the budget via the linear SCV term and yielding 9k.
- Forcing Shake&Tune to SCV=5 matches Klipper. Confirmed empirically.

The "measured ζ vs default 0.1" difference is real but minor. The
primary variable is SCV.

---

## Prior art — community discussion

Proposals to address the architectural problem have been raised
multiple times and rejected or left stale.

### Issue #468 — `junction_deviation` → `square_corner_velocity`
https://github.com/Klipper3d/klipper/issues/468
Pure reparameterization. No algorithmic change. Motivated by
`junction_deviation` being "a magic number" in Kevin O'Connor's words.
The new parameter is equally a magic number, just with velocity units.

### Issue #5227 — SCV's hidden coupling to `max_accel`
https://github.com/Klipper3d/klipper/issues/5227
Closed. Reporter points out exactly the issue documented here: SCV
behavior depends invisibly on `max_accel`. No fix adopted.

### Issue #4228 — sharp corners vs smooth circles
https://github.com/Klipper3d/klipper/issues/4228
richfelker: the current formula makes segmented curves impossible at
any SCV useful for real corners. Proposed a softer power-law alternative.
Stale.

### Discourse #3970 — proportional acceleration control
https://klipper.discourse.group/t/proportional-acceleration-control/3970
Piezo proposed scaling SCV with `√a` to preserve corner geometry across
acceleration changes. Explicitly rejected by Butyugin ("actual intention
is to improve extrusion, not kinematics") and O'Connor ("less knobs"
philosophy).

### Shake&Tune Issue #10
https://github.com/Frix-x/klippain-shaketune/issues/10
User reports Shake&Tune `ei@49.6Hz max_accel≤4600` vs Klipper
`mzv@77.6Hz max_accel≤17700` on same data, measured ζ=0.049. The exact
gap pattern observed here. Open, unresolved.

### Voron forum #921
https://forum.vorondesign.com/threads/accelerations-higher-than-input-shaper-suggested.921/
Community consensus: shaper recommendations are conservative; users
routinely run 2–3× the recommendation successfully. No agreement on why.

### Unexplored area
Neither `offset_90`, nor `_get_shaper_smoothing`, nor the hardcoded
`TARGET_SMOOTHING = 0.12` appears in any public architectural critique.
The formula's lingering-SCV model is undocumented in GitHub issues,
Discourse threads, or community writeups.

---

## Proposed direction for the fork

The core change is to make **corner geometry** — not `square_corner_velocity` —
the user-facing configuration parameter. Specifics (parameter name, scope,
input shaper coupling) are open questions for the design phase; this document
only records the conclusions motivating the direction.

Rough sketch, to be refined:

1. Expose `junction_deviation` (or equivalently an arc-radius parameter) as a
   first-class config field, interpreted as the desired geometric rounding at
   corners.
2. Derive `square_corner_velocity` per-move from the geometric parameter and
   the current move's acceleration, rather than the other way around.
3. Keep SCV available as an alternate config surface for backwards
   compatibility, but treat it as a derived quantity.

Implications for `shaper_calibrate.py` (to be designed later, not now):

- The calibration bisection becomes self-consistent — at each test
  acceleration `a`, the effective SCV is `sqrt(a · jd / (√2 − 1))`, so the
  `offset_90` term scales with `sqrt(a)` rather than being linear in a
  user-set constant.
- The `TARGET_SMOOTHING = 0.12` budget remains a separate, empirical knob.
  Worth exposing as a parameter independent of the geometry change.

What this does **not** fix:
- The commanded motion is still a zero-duration direction change at
  junctions. Actual arc blending would require a motion-planner rewrite.
- `TARGET_SMOOTHING` is unchanged; its conservatism remains.
- The measured-ζ vs default-ζ gap in Shake&Tune's ~5–10% effect remains.

---

## Reality check on what the input shaper formula actually protects

`offset_90` measures commanded-vs-shaped position deviation at a 90°
corner. This shows up as **visible corner rounding**, not stepper stress.
It is a print-quality budget. The `max_accel` recommendation says: "go
faster than this and your corner rounding exceeds 0.12 mm."

Users empirically tolerate more than 0.12 mm of corner rounding. That
is why running 2–3× above the recommendation works. The formula is
accurate at predicting what it claims to predict; the claim itself
(0.12 mm is the quality threshold) is the empirical assumption.

---

## Key file references

| file | notable lines | purpose |
|---|---|---|
| `klippy/toolhead.py` | 79-122 | `calc_junction` — applies junction formula |
| `klippy/toolhead.py` | 124-138 | `set_junction` — trapezoid phase timing |
| `klippy/toolhead.py` | 164-226 | `flush` — backward sweep lookahead |
| `klippy/toolhead.py` | 787-789 | `_calc_junction_deviation` |
| `klippy/chelper/trapq.c` | 97-164 | `trapq_add_move`, `trapq_append` |
| `klippy/chelper/kin_shaper.c` | 91-103 | shaper convolution across moves |
| `klippy/extras/shaper_calibrate.py` | 240-260 | `_get_shaper_smoothing` |
| `klippy/extras/shaper_calibrate.py` | 361-371 | `find_shaper_max_accel` |
| `klippy/extras/resonance_tester.py` | 570-580 | SCV read from toolhead |

---

## Open questions

- What is the right name for the geometric parameter? `junction_deviation`
  carries legacy baggage from Grbl/Marlin. `corner_rounding_radius`,
  `max_corner_deviation`, or similar may be clearer.
- Should the input shaper calibration accept the geometric parameter
  directly, or should it continue to read SCV from the toolhead (which
  would then be derived)?
- How is compatibility with existing SCV-based configs preserved during
  a transition?
- Is the right project scope a Kalico PR or a standalone fork?
