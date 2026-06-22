# Code map — Input Shaping on the new path

Load-bearing implementation reference for SPEC-input-shaper-port. File:line anchors verified against the live tree on 2026-06-22; re-confirm before editing. Read alongside [[spec-pressure-advance-port]]'s `code-map.md` — this work rides the same rail PA does.

## Live path facts (verified)

- Active solver is `temporal::multi` (Consolini-Locatelli SOCP). `temporal::topp` and the dead `emit_shaped.rs` emit path are NOT on the live path.
- The live emit path is `StreamPlanner → StreamState::commit → lowering::lower_move → Sampler::axis_state`. It consumes **only** `CompiledChain.gain` (`rust/motion-engine/src/lowering.rs:217-218`, `follower_gains`). It **never reads `CompiledChain.kernel`**. Spatial axes (`axis < 3`) return raw geometry with zero post-processing (`lowering.rs:125-129`). So input shaping is entirely absent from the live path — the `kernel` field is dead weight there today.
- The shaper TYPES, kernel math, config parse, and live retune rail already exist and are tested. The gap is purely the live convolution wiring.

## The defining difference from PA — locality

PA's correction is **local**: at sample time `t`, `pos' = pos + k·ė(t)`, `vel' = vel + k·ë(t)` — same `t`, no neighbors (`lowering.rs:130-143`). That is why PA dropped straight into the pointwise `Sampler`.

Input shaping is **non-local convolution**: `shaped(t) = ∫ kernel(τ)·base(t−τ) dτ` over a window of half-support `h` around `t`. `ShapedSignal::new` samples `[t_start + k_lo, t_end + k_hi]`, so live shaping needs both past history and future lookahead. The current `Sampler::axis_state` is a pure pointwise evaluator with no window and no neighbor/history access. Bringing shaping onto the live path therefore is not a "confirm the wiring" job like PA was — it is a real algorithmic addition: supply each shaped sample a padded `[t−h, t+h]` window of the base signal, carry history from committed output, and hold back live-edge samples until enough future base is planned.

## Work-item hooks

| CAP | What | Where |
|-----|------|-------|
| CAP-1 | Live emit site — pointwise, gain-only today; shaping must be added here or just after | `rust/motion-engine/src/lowering.rs:121-147` `Sampler::axis_state`; fit loop `:243-255` |
| CAP-1 | Kernel already compiled into the chain (unused on live path) | `rust/trajectory/src/post_processor.rs:35-39` `CompiledChain{ kernel, gain }`; `:77-87` `action()` → `PlanAction::Kernel` |
| CAP-1 | Validated convolution model to reuse (dead path, reference oracle) | `rust/trajectory/src/shaper.rs` `ShapedSignal::new/eval`; applied at `emit_shaped.rs:227-238` (spatial), `:336-368` (follower) |
| CAP-1 | Kernel builders + Hz→smooth-time constants (reuse verbatim) | `rust/trajectory/src/post_processor.rs:1-6` consts; `crate::kernel::build_smooth_zv_kernel` / `build_smooth_mzv_kernel` |
| CAP-2 | Config parse / instance build for `smooth_zv`/`smooth_mzv` | `rust/motion-engine/src/config.rs` (`PostProcessorSet::try_new`/`compile`, ~`:148-191`) |
| CAP-2 | Config sections (Python); legacy `[input_shaper]` rejected | `klippy/extras/post_processor.py`; `[post_processor NAME]` + `[axis x] post_processors:`; rejection at `klippy/motion.py:623-628` |
| CAP-2 | Runtime tune command → live swap | `klippy/motion.py` `cmd_SET_POST_PROCESSOR` → `bridge.rs` `update_post_processor` (~`:3713-3737`) → `stream_planner.rs` `SetAxisChains` (~`:20-31`, `:442-444`) → `stream.rs` `set_axis_chains` (`:83-85`) |
| CAP-2 | **Fail-loud gap**: shaper `set_param` does NOT validate `frequency_hz` finite/positive (PA's `k` does) | `rust/trajectory/src/post_processor.rs:94-103` (add the check PA has at `:104-118`) |
| CAP-3 | Cross-batch history and forward-lookahead model to mirror onto the live streaming state | dead path `pad_segment_axis_with_history`, `PerAxisHistory`, and `ShapedSignal::new`'s `[t_start + k_lo, t_end + k_hi]` sampling interval in `emit_shaped.rs`/`shaper.rs`; live streaming state `rust/motion-engine/src/stream.rs` |
| CAP-3 | Per-axis composition / one-kernel-per-axis already enforced | `rust/trajectory/src/post_processor.rs:123-157` `CompiledChain::compile` |
| CAP-4 | Model seam — add smooth shaper = new `PostProcessorType` arm + `type` string + kernel builder | `post_processor.rs:8-13` enum, `:77-87` `action()` |

## Why CAP-2 is mostly confirmation

The chain compiled from a `smooth_zv` config already carries `kernel: Some(...)` (`post_processor.rs:140`, via `action()` → `PlanAction::Kernel`). The runtime swap path (`update_post_processor` → `SetAxisChains` → `set_axis_chains`) is the exact path PA proved with `update_post_processor_applies_to_new_plans_only`. So once the live emit consumes `kernel`, retune is inherited. The one real CAP-2 task beyond confirmation is the `frequency_hz` validation gap.

## Live vs dead stacks (verified 2026-06-22)

The bridge drives exactly ONE emit path. Everything in the dead column is compiled but never instantiated in production.

| Live (bridge-driven) | Evidence |
|----|----|
| `bridge.rs init_planner` → `StreamPlannerHandle::spawn` | `bridge.rs:3285` |
| `StreamState::commit` → `lower_move` → `Sampler` | `stream.rs:157`, `lowering.rs:194` |
| `CompiledChain.gain` (PA coefficient) | read at `lowering.rs:218` |

| Dead (superseded) | Only consumer |
|----|----|
| `motion-engine/src/planner.rs` (`PlannerHandle`, `PlannerMsg`) | nothing but `examples/plan_gcode.rs` |
| `trajectory::streaming` (`mod`/`emit`/`state`) | dead `planner.rs` |
| `trajectory::emit_shaped` (whole module) | dead `streaming` + `beta` |
| `trajectory::beta` | experimental, off live path |
| `post_processor::apply_derivative_gain` (NURBS PA) | dead `emit_shaped` (`:205`) |
| `CompiledChain.kernel`, `PlanAction::Kernel` | dead `emit_shaped` only; never read live |

Nuance: `PlanAction` is NOT fully dead — `CompiledChain::compile` runs live and the `DerivativeGain` arm sets the live `gain` (`post_processor.rs:142-152`). Only the `Kernel` arm is dead. The unified interface (CAP-5) replaces the whole enum.

## CAP-5 — unified interface & PA re-homing hooks

Target shape (replaces the `with_pa` flag + `{kernel,gain}` flattening):

```
trait BaseSignal { value(t); deriv(t); deriv2(t); }      // exact unshaped axis evaluator
trait AxisPostProcessor { half_support()->(f64,f64); eval(&self, base:&dyn BaseSignal, t)->(f64,f64); }
```

- `BaseSignal`: the first-stage base is analytic, derived from `Phase` + spatial segment or follower demand. Do not make the unshaped base a fitted NURBS just to satisfy `ShapedSignal::new`; that would put PA's byte-identity at risk.
- `LinearPa{k}`: `eval = (value + k·deriv, deriv + k·deriv2)`, support `(0,0)`. `deriv2` == today's `phase.accel` used at `lowering.rs:141`. Exact ⇒ `apply_derivative_gain` (`post_processor.rs:214`) no longer needed. Zero-support stages reuse the incoming fit grid and do not run a fresh adaptive residual pass.
- Shaper: convolution over `value(·)` via the rescued `ShapedSignal` math (`shaper.rs:36-77`); `half_support = kernel.support()`. Add a closure/evaluator-taking constructor or equivalent sampled-analytic adapter so the convolution consumes `BaseSignal` directly instead of forcing the whole fold through a NURBS input.

| What | Where to change |
|----|----|
| Drop `with_pa`/`follower_gains`; make `Sampler` the `BaseSignal` | `lowering.rs:113-147`, `:217-219`, `:251-252` |
| `lower_move` emits unshaped base only | `lowering.rs:194-269` |
| New post-lowering stage over the batch (holds history and future lookahead, with live-edge hold-back) | wrap at `StreamState::commit`, `stream.rs:157-184` |
| Per-axis ordered chain replaces `CompiledChain{kernel,gain}` | `post_processor.rs:35-39`, `:123-157`; build at `config.rs:89-105` |
| Live swap re-points to the new chain repr | `stream.rs:83-85`, `stream_planner.rs:442-444`, `bridge.rs:3713-3737` |

## Reference syntax

### Config (already exists — unchanged; input shaping needs no new surface)

Param names per `config.rs:126-174`: `type` ∈ `smooth_zv | smooth_mzv | linear_pressure_advance`; required param is `frequency_hz` for shapers, `k` for PA. Attachment to an axis is by name in `post_processors:`; the list order is the application order. There is no extruder concept — a "PA on the extruder" is just a processor attached to a follower axis.

```ini
[post_processor my_pa]
type: linear_pressure_advance
k: 0.04

[post_processor xy_shaper]
type: smooth_zv
frequency_hz: 52.0

[axis x]
motors: stepper_x
post_processors: xy_shaper

[axis y]
motors: stepper_y
post_processors: xy_shaper

[axis e]
follows: x, y
motors: extruder_stepper
post_processors: my_pa
```

Composing on one axis — declaration order is application order (shape first, then PA on the shaped motion):

```ini
[axis e]
follows: x, y
motors: extruder_stepper
post_processors: xy_shaper, my_pa
```

Live retune (identical to PA; the runtime `set_param` at `post_processor.rs:96-102` must gain the `frequency_hz > 0` check that config-build already has at `config.rs:150`):

```gcode
SET_POST_PROCESSOR NAME=xy_shaper FREQUENCY_HZ=48.5
SET_POST_PROCESSOR NAME=my_pa K=0.035
```

### Proposed Rust interface (CAP-5 — the new code)

```rust
/// The unshaped axis, evaluable exactly anywhere — including into the padding
/// window a shaper reaches past the segment edge.
trait BaseSignal {
    fn value(&self, t: f64) -> f64;
    fn deriv(&self, t: f64) -> f64;   // 1st (velocity)
    fn deriv2(&self, t: f64) -> f64;  // 2nd (acceleration)
}

/// Every post-processor is one of these — PA, shaping, anything later.
trait AxisPostProcessor {
    fn half_support(&self) -> (f64, f64);                          // PA: (0.0, 0.0)
    fn eval(&self, base: &dyn BaseSignal, t: f64) -> (f64, f64);   // (value, velocity)
}

// PA — local, exact, no NURBS pass (obsoletes apply_derivative_gain):
struct LinearPa { k: f64 }
impl AxisPostProcessor for LinearPa {
    fn half_support(&self) -> (f64, f64) { (0.0, 0.0) }
    fn eval(&self, b: &dyn BaseSignal, t: f64) -> (f64, f64) {
        ( b.value(t) + self.k * b.deriv(t),     // p + k·p'
          b.deriv(t) + self.k * b.deriv2(t) )   // p' + k·p''  (p'' == today's phase.accel, lowering.rs:141)
    }
}

// Shaper — non-local, reuses the rescued ShapedSignal convolution:
struct Shaper { kernel: PiecewisePolynomialKernel<f64> }
impl AxisPostProcessor for Shaper {
    fn half_support(&self) -> (f64, f64) { self.kernel.support() }
    fn eval(&self, b: &dyn BaseSignal, t: f64) -> (f64, f64) {
        // ShapedSignal's discrete convolution of b.value(·) over [t − k_hi, t − k_lo]
        // (+ its derivative for the velocity) — shaper.rs math, but reading b.value(·)
        // instead of sampling a baked NURBS curve.
    }
}
```

The stage (runs at `StreamState::commit`, where the batch + cross-batch history and forward lookahead live):

```rust
let mut signal: Box<dyn BaseSignal> = analytic_base_for_axis(axis);
let mut grid = unshaped_fit_grid;
for pp in &chains[axis] {
    if pp.half_support() == (0.0, 0.0) {
        signal = evaluate_on_existing_grid(pp, &*signal, grid);
    } else {
        require_history_and_lookahead(pp.half_support());
        signal = refit_with_padding(pp, &*signal);
        grid = signal.fit_grid();
    }
}
emit(signal);
```

Composition note: nonzero-support stages are re-fit to a curve that becomes the next stage's `BaseSignal`, so a later PA sees the *shaped* signal's derivative. Zero-support stages preserve the incoming grid; this is what keeps a PA-only plan byte-identical while still allowing `xy_shaper, my_pa` to mean "PA on already-shaped motion."

## Dead-code marking (do NOT delete here)

Decision: the old pipeline is left in place and retired wholesale by the maintainer in a separate pass. This work only (a) keeps the new live stage free of any dependency on it, and (b) marks it unmistakably dead by expression — relocate under a `legacy`/`dead` module and/or `#[deprecated]`, so the compiler flags any live-path use.

Mark as dead: `motion-engine/src/planner.rs` (`PlannerHandle`, `PlannerMsg`), `trajectory/src/streaming/`, `trajectory/src/emit_shaped.rs`, `trajectory/src/beta.rs`, `post_processor::apply_derivative_gain`, `CompiledChain.kernel`, `PlanAction::Kernel`. `examples/plan_gcode.rs` (its only live user) goes with the old pipeline at retirement, not now.

Stays put (shared primitives, used by BOTH dead emit and the new live stage): `trajectory/src/shaper.rs` (`ShapedSignal`, `eval_kernel`), `crate::kernel::build_smooth_*`, the `SMOOTH_*_T_SM_PER_HZ` consts.

Tests: add a live PA `eval` value-test (don't touch the existing `apply_derivative_gain` tests at `post_processor/tests.rs:92,100` — they ride with the dead pipeline until it's retired).

## Test anchors

- `rust/trajectory/src/post_processor/tests.rs` — `SmoothZv`/`zv()` instances already exist; extend for live-path equivalence.
- The dead `ShapedSignal` output is the natural **oracle**: a live-path shaped axis must equal `ShapedSignal::eval` for the same base signal and kernel (to fit tolerance). This is the CAP-1 cross-check and an argument for keeping the dead code until the test exists.
- `rust/motion-engine/src/lowering/tests.rs` — holds PA live-path tests; live shaping tests belong here.
- Streaming seam: mirror PA's `streaming_replan` / `update_post_processor_commits_held_output_before_swap` for the freq-retune-mid-stream case; add two-batch-vs-one-batch shaped-trajectory equality tests for both past-history and future-lookahead sides of CAP-3. Add a live-edge test proving shaped samples inside the shaper's future half-support are held back until future base exists, and clamp-at-edge is used only at absolute print start/end.
