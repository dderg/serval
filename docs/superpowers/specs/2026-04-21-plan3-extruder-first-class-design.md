# Plan 3 — Extruder as first-class planning constraint (pillar 3)

**Status:** draft, 2026-04-21
**Branch target:** `magnum-opus`
**Predecessor:** Plan 2 Phase A (`496365b2`) — smooth-shapers merged, non-linear PA in tree
**Successor (planned):** Plan 5 — pillar 2 unified `v(s)` along the curve (will subsume this cap's per-move application into per-s evaluation)

## Goal

Treat the extruder stepper as a first-class kinematic constraint in the motion planner. Today the planner picks XY acceleration without knowing whether the resulting **post-Pressure-Advance stepper output** is within the extruder motor's torque/RPM budget. On acceleration-limited extruders (direct-drive, high-flow builds) this causes step skipping at corners and during sharp accel phases, forcing users to either globally derate `max_accel` or detune PA — both of which penalize the 95% of moves that are not extruder-bound.

Plan 3 ships a per-move cap `cap_move(move, pa_model, extruder_limits) → (v_cap, a_cap)` that the planner consults on every move. The cap reads the **live PA model** (linear / tanh / recipr) and computes the tightest `(v_xy, a_xy)` such that the post-PA stepper motion stays within the configured `max_extruder_accel` and `max_extruder_rpm`. Moves below the cap are untouched; moves that would exceed it have their individual `max_cruise_v2` and `accel` clamped. No global derate, no fixed-point iteration, no multi-pass planner rewrite.

## Non-goals

- Not a continuous `v(s)` along the curve — that lands in Plan 5 (pillar 2).
- Not a PA model change — uses the three PA models already in tree (`PALinearModel`, `PATanhModel`, `PAReciprModel`).
- Not an auto-derive from stepper config — user sets `max_extruder_accel` and `max_extruder_rpm` explicitly.
- Not a velocity cap on cruise motion as a separate slowdown guillotine — the cap is a joint `(v, a)` bound that the existing planner uses to pick the fastest feasible trapezoidal profile.

## Architecture

### File layout

- **New**: `klippy/blendextruder.py` — pure Python module at the planner layer (root, not `extras/` — mirrors `blendmath.py`, `blendquintic.py`, etc.).
  - Public API: `cap_move(move, pa_model, extruder_limits) → (v_cap, a_cap)`
  - Internal: per-PA-model helpers, fixed-point-free formula evaluation
- **Modified**: `klippy/kinematics/extruder.py` — each `PA*Model` class gains `f_prime(v)` and `f_double_prime(v)` methods (pure math, reads the model's live params).
- **Modified**: `klippy/blendshape.py` — `ExtruderLimits` extended to `(a_E_max, v_E_max, smooth_time)` (we need `K_h = (15/8)/smooth_time`; current dataclass has `(accel_max, rpm_max)` — rename/expand).
- **Modified**: `klippy/kinematics/extruder.py` — each `PA*Model` class gains `f_prime(v)` and `f_double_prime(v)` methods. The `ExtruderStepper` (or its config-parsing wrapper in `klippy/extras/extruder.py`) exposes a single `extruder_limits_snapshot()` method that returns an `ExtruderLimits` instance built from current config + live PA smooth_time.
- **Modified**: `klippy/extras/extruder.py` — config parsing for `max_extruder_accel` and `max_extruder_rpm` keys on the `[extruder]` section; new `SET_EXTRUDER_LIMITS` gcode command for runtime tuning.
- **Modified**: `klippy/blendplanner.py` — populates `KinematicLimits.extruder_caps` from the toolhead snapshot (forward-compatible with Plan 5; not consumed by Plan 3 directly, since the cap lives at Move-level not shape-level).
- **Modified**: `klippy/toolhead.py` (or `klippy/move.py`) — `Move.limit_speed` (or the equivalent hook immediately after `kin.check_move(move)` in Move construction) calls `blendextruder.cap_move()` and applies the returned `(v_cap, a_cap)` via the existing `limit_speed` machinery. This ensures the cap applies to **all** moves — user gcode moves AND blend-polyline moves emitted by `blendplanner._emit_blend`.

### Data flow

```
user gcode
  ↓
toolhead.add_move(move)
  ↓
Move.__init__ computes max_cruise_v2, accel from kinematics
  ↓
Move.calc_junction(prev_move) — existing lookahead
  ↓
[NEW] blendextruder.cap_move(move, pa_model_snapshot, extruder_limits_snapshot)
  → returns (v_cap, a_cap)
  ↓
move.limit_speed(v_cap, a_cap) — existing ceiling-setting
  ↓
lookahead / flush — existing planner pipeline
  ↓
trapq, stepcompress, MCU
```

`cap_move` is idempotent and side-effect-free: given the same move + pa_model + extruder_limits, it always returns the same `(v_cap, a_cap)`.

### Snapshots, not live references

`cap_move` takes a **snapshot** of the PA model and extruder limits, not a live reference. Rationale:
1. PA model can be changed at runtime (`SET_PRESSURE_ADVANCE` / `SET_EXTRUDER_LIMITS`). Planner must use the values that were active when the move was queued, not when the move was flushed.
2. Snapshot pattern matches `blendmath._extract_shapers` — proven idiom.

Snapshot shape:
```python
@dataclass
class PAModelSnapshot:
    kind: str              # "linear" | "tanh" | "recipr"
    # linear: pressure_advance
    # tanh/recipr: linear_advance, nonlinear_offset, linearization_velocity
    params: tuple

@dataclass
class ExtruderLimits:
    a_E_max: float         # mm/s² on filament (config: max_extruder_accel)
    v_E_max: float         # mm/s on filament (config: max_extruder_rpm → linear via rotation_distance)
    smooth_time: float     # pressure_advance_smooth_time (for K_h computation)
```

### Core math

From the subagent's derivation (verified numerically to 1e-9 accuracy; see appendix).

**Master formula** — post-PA stepper output in terms of base kinematics:
```
stepper_v(t) = v_E(t) + f'(V_s(t)) · V_s'(t)
stepper_a(t) = a_E(t) + f''(V_s(t)) · V_s'(t)² + f'(V_s(t)) · V_s''(t)
```
where `v_E = k · v_xy` (filament-demand velocity, `k = dE/dL` flow ratio), `V_s` is the smoothed velocity convolved with the PA smoothing kernel, and `f` is the PA model's advance function.

**Kernel peak factor**:
```
K_h = (15/8) / smooth_time        [s⁻¹]
```
At `smooth_time = 40 ms`, `K_h = 46.875 s⁻¹`. Smaller `smooth_time` → larger `K_h` → tighter cap during phase transitions.

**PA derivatives**:

| Model | `f'(v)` | `f''(v)` |
|---|---|---|
| `PALinearModel` | `PA` (constant) | `0` |
| `PATanhModel` | `LA + (NO/LV) · sech²(v/LV)` | `−(2·NO/LV²) · sech²(v/LV) · tanh(v/LV)` |
| `PAReciprModel` | `LA + (NO/LV) / (1 + v/LV)²` | `−(2·NO/LV²) / (1 + v/LV)³` |

where `PA`, `LA`, `NO`, `LV` are the respective model parameters.

**Peak `stepper_a` binds at phase transitions** (jerk impulse from `a_E` step change). At the start-of-accel moment, `V_s = v_prev` (entering velocity), and `V_s''` spikes to `±a_E · K_h`. At end-of-decel, `V_s = v_next`. Since `f'` is monotonically decreasing in V for NL models, peak `stepper_a` is at whichever of `{v_prev, v_next}` is **lower**.

**Peak `stepper_v` binds at mid-accel-plateau** where `V_s ≈ v_cruise` while `V_s'` is at its `a_E` plateau. So `v_eval` for the velocity cap = `v_cruise`.

### Cap inversion

**Accel cap**:
```
v_eval_a = k · min(v_prev, v_next)      # peak stepper_a moment
a_E_cap = a_E_max / (1 + f'(v_eval_a) · K_h)
a_cap = a_E_cap / k                       # return as XY tangential accel
```

**Velocity cap**: solve for the max `v_xy_cruise` such that `stepper_v_peak ≤ v_E_max`. `stepper_v_peak = k · v_xy + f'(k · v_xy) · a_E_at_peak`. For the `v_E_max` test, use the already-computed `a_E_cap` as the accel term:
```
solve for v_xy:   k · v_xy + f'(k · v_xy) · a_E_cap ≤ v_E_max
```
- Linear PA: closed form → `v_xy_cap = (v_E_max − PA · a_E_cap) / k`.
- NL PA: 1-D bisection on a monotone function. Starts bracketed by `[0, v_E_max / k]`. Tolerance: 1e-6 mm/s. Max ~25 iterations.

**Cruise-only bound** (fallback when cruise dominates):
```
v_cap = min(v_xy_accel_cap, v_E_max / k)
```

**Edge cases**:
- `k == 0` (travel move): cap is infinite. Return `(+inf, +inf)`.
- PA model disabled (e.g. linear with `PA = 0`): `f' = 0`, `f'' = 0`. Cap collapses to `a_E_max / k`, `v_E_max / k`. Correct.
- Direction reversal (`V_s ≤ 0`): C code bypasses PA term at `kin_extruder.c:223`. The cap formula at `v_eval_a = 0` still gives a valid conservative bound for the transition itself. Rare corner case; no special handling needed.
- Non-kinematic moves (retract, G10): `cap_move` returns `(+inf, +inf)`; caller's `limit_speed` no-ops.

### Integration point

The cleanest integration is **after kinematics' `check_move()` and before the lookahead finalizes the move**. The snapshot is cached on the toolhead (refreshed when `SET_EXTRUDER_LIMITS` or `SET_PRESSURE_ADVANCE` fires; reads from the live extruder state). Sketch:

```python
# on toolhead init + after SET_EXTRUDER_LIMITS/SET_PRESSURE_ADVANCE:
self.extruder_cap_snapshot = self.extruder.extruder_limits_snapshot()  # or None if disabled

# in Move.__init__ (or equivalent hook) after kin.check_move(move):
snap = self.toolhead.extruder_cap_snapshot
if snap is not None and snap.pa_model is not None:
    v_cap, a_cap = blendextruder.cap_move(move, snap.pa_model, snap.limits)
    if math.isfinite(v_cap) or math.isfinite(a_cap):
        move.limit_speed(v_cap, a_cap)    # existing min-taking handles the rest
```

`limit_speed` already handles min-taking against existing ceilings, so passing the cap is safe even when it's looser than a competing constraint. For moves with `k=0` (pure travel), `cap_move` returns `(+inf, +inf)` and `limit_speed` is a no-op.

The snapshot is a pair `(pa_model: PAModelSnapshot, limits: ExtruderLimits)` — both immutable. Cached once per PA/limits change; read-only during planning. Caching avoids re-pickling the PA state on every move.

## Config surface

### Config keys (`[extruder]` section)

| Key | Type | Default | Required | Effect |
|---|---|---|---|---|
| `max_extruder_accel` | float (mm/s² on filament) | `0` | no | `0` disables the cap (status quo). Positive value activates it. |
| `max_extruder_rpm` | float (RPM on drive pulley) | `0` | no | `0` disables velocity cap. Positive value activates it. |

Internally converted:
- `a_E_max = max_extruder_accel` (direct).
- `v_E_max = (max_extruder_rpm / 60) · rotation_distance` — derived from the existing `rotation_distance` config.

### GCODE command

```
SET_EXTRUDER_LIMITS [EXTRUDER=<name>] [ACCEL=<mm/s²>] [RPM=<RPM>]
```

Runtime-settable; applies to moves queued after the command. Matches the pattern of `SET_PRESSURE_ADVANCE`.

Omitting both ACCEL and RPM reports current values:
```
EXTRUDER 'extruder': max_extruder_accel=5000.0, max_extruder_rpm=200.0
```

Setting either to `0` disables that cap (matches config behavior). No `SAVE_CONFIG` integration in Plan 3 — leave manual config edits for persistence.

## Testing

### Unit tests (`test/test_blendextruder.py`)

1. **Mathematical correctness** — for each PA model:
   - `f_prime(v)` and `f_double_prime(v)` return the correct derivatives at canonical points (v=0, v=LV, v=large).
   - Symbolic vs numeric derivative agreement (central finite-difference at `h=1e-5 · v`, rel tolerance 1e-6).

2. **Cap invariants**:
   - `k=0` → `(inf, inf)`.
   - `a_E_max=0` → `a_cap=0`, `v_cap=v_E_max/k`.
   - `v_E_max=0` → `v_cap=0`, `a_cap=a_E_max/k` (at k=0 limit).
   - Linear PA with `PA=0` → `a_cap = a_E_max/k`, `v_cap = v_E_max/k`.
   - Linear PA closed-form match.

3. **Phase-transition `v_eval` correctness** — given `v_prev < v_cruise`, cap uses `v_prev` for `f'` evaluation; given `v_next < v_prev < v_cruise`, cap uses `v_next`.

4. **Bisection convergence** — NL PA cap converges to within 1e-6 mm/s in ≤25 iterations for a range of `(NO, LV, a_E_max)` tuples.

5. **Snapshot isolation** — two `cap_move` calls with the same inputs but different live pa_model state return identical caps. Mutating the model after snapshot creation doesn't affect the cap.

### End-to-end sim (`klipper-sim`)

Targeted at `~/Developer/klipper-sim/`:

1. **No-cap baseline**: reference print time + stepper peak_accel trace on a corner-dense gcode.
2. **Cap at realistic limit**: `max_extruder_accel=5000`, `max_extruder_rpm=200`. Check:
   - Stepper peak_accel never exceeds 5000.
   - Total print time degrades by no more than ~2% on a typical print (expecting 5% of moves to bind).
3. **Sweep**: vary `max_extruder_accel` from 2000 to 20000. Plot (print_time, fraction_of_moves_capped) vs `max_extruder_accel`.

### Regression coverage for existing blend* tests

Re-run `test/test_blendplanner.py` + `test/test_blendquintic.py` + `test/test_blendmath.py` — confirm no regressions from the `extruder_caps` wire-up.

### Hardware validation (deferred to user)

On user's Trident:
1. Print a corner-dense ringing tower before/after enabling the cap with aggressive `max_extruder_accel`. Look for skipped steps (klippy.log `stepcompress` errors) — should be eliminated.
2. Set `max_extruder_accel` to a deliberately-too-low value; confirm the print slows only on the binding subset of moves (inspect `print_time` delta vs baseline).

## Interaction with other pillars

- **Pillar 1 (inverse shaper, Plan 6)**: inverse-shaper pre-distortion produces a *commanded* XY trajectory whose accel may briefly exceed the planned `a_max`. Extruder cap reads the *planned* XY velocity/accel, not the commanded one (PA is computed from the planned-path filament demand anyway). No conflict — cap operates one layer above pillar 1's pre-distortion.
- **Pillar 2 (unified `v(s)`, Plan 5)**: will evaluate the extruder cap at every point along the curve, not just at move boundaries. Plan 3's per-move cap is a coarse approximation of this; Plan 5 lifts it to continuous. The cap formula itself stays identical — only the evaluation grid changes.
- **Pillar 4 (global optimizer, deferred)**: would treat the extruder cap as just another constraint in the feasible set. No change required.

## Open questions (resolve during implementation)

1. **Pickling the PA model snapshot cheaply**. Options: dataclass with immutable tuple; frozen dict; lightweight `namedtuple`. Pick whichever benchmarks best when called 1000×/s during planning. Default to `dataclass(frozen=True)` unless profiling shows hot path.

2. **Smooth-time 0** (no smoothing at all). Edge case: `K_h → ∞`, cap → 0. Is this ever a valid config? If yes, the cap formula needs a guard. Leaning toward "no, reject at config parse with a helpful error (`pressure_advance_smooth_time must be > 0 when max_extruder_accel is active`)." Verify during implementation.

3. **Does `cap_move` need to see the Move's `axes_d`/`axes_r` directly, or is `move.axes_r[3]` (k) and `move.max_cruise_v` enough?** Probably just k and max_cruise_v. Plan 3 doesn't need the geometric path — it's all velocity-space reasoning. If v_prev/v_next are needed, they're from lookahead state (`move.junction_max_v2` or similar), NOT from re-deriving geometry.

## Success criteria

1. Tests green: all existing + new `test_blendextruder.py`.
2. Klipper-sim validation: stepper peak_accel stays below configured `max_extruder_accel` on a diverse corner-dense gcode, with <5% print time penalty at realistic limits.
3. Code review: `blendextruder.py` is ≤400 lines, self-contained, no dependencies beyond `blendshape` + stdlib math.
4. Docs: `docs/Config_Reference.md` gains the two new config keys with a paragraph explaining when to use them.
5. Hardware test (deferred): user runs a corner-dense print with aggressive `max_extruder_accel` and no skipped-step errors.

---

## Appendix A — derivation verification

The full derivation is in the subagent research output (saved-in-conversation). Key verification:

- Script `/tmp/plan3_pa_verify.py` executed; compared closed-form `stepper_a(t) = a_E + f''·V_s'² + f'·V_s''` against finite-difference of the numerical PA output.
- Sweep: `v_xy ∈ {80, 150, 300, 500}` mm/s × `a_xy ∈ {2000, 10000, 40000, 80000}` mm/s² × 3 models.
- Max relative error across all 48 combinations: **7.41e-10** (tanh), **5.71e-10** (linear), **3.92e-10** (recipr). Well within numerical precision.
- Cap-tightness check (`/tmp/plan3_cap_check.py`): applying the cap with `v_eval = v_cruise` and running the move back through the simulator, peak `stepper_a` was 99.5% of `a_E_max` for tanh/recipr and 80% for linear (conservative by ~20% for linear because of the time-separated peaks; the rigorous `v_eval = min(v_prev, v_next)` version recovers this margin).

## Appendix B — why `v_eval = min(v_prev, v_next)` is rigorous

Peak `stepper_a` over the move happens at either the start-of-accel phase transition (V_s = v_prev, V_s'' = +a_E · K_h) or the end-of-decel transition (V_s = v_next, V_s'' = −a_E · K_h). The formula uses `|stepper_a_peak|`, so both transitions contribute with the same magnitude. For NL PA, `f'(V)` is monotonically decreasing in V, so the **tighter** cap comes from the lower of `{v_prev, v_next}`. Using the min guarantees the cap is satisfied at both transition moments.

For linear PA, `f'` is constant, so the choice of `v_eval` doesn't affect the cap at all — the min reduces to just `PA · K_h` regardless.
