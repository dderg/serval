# Plan: generalize `ChainStage::LinearPressureAdvance` to second-order `DerivativeGains`

## Why

We want a mode-inversion post-processor for servo-driven axes: the dominant ringing
mode is belt compliance between motor and toolhead — a 2nd-order plant with
identified natural frequency ω and damping ζ. Commanding the motor along

```
x_motor(t) = x_head(t) + (2ζ/ω)·ẋ_head(t) + (1/ω²)·ẍ_head(t)
```

makes the toolhead track the intended path exactly (to model accuracy). This is
the same trick as linear pressure advance (`x + k·ẋ`), one order higher.

Rather than adding a new `ChainStage` variant and a fifth copy of special-case
logic, this task **generalizes the existing PA stage** to carry two gains:

```rust
ChainStage::DerivativeGains { k1: f64, k2: f64 }
```

`LinearPressureAdvance` compiles to `{ k1: k, k2: 0.0 }`. A separate task (see
`docs/plans/mode-inverse-post-processor.md`) adds the `mode_inverse`
post-processor algo that compiles to `{ k1: 2ζ/ω, k2: 1/ω² }`. **This task's
deliverable is the stage machinery only** — after it lands, all existing
behavior must be bit-identical (PA has k2 = 0), and the new stage must be
correct for k2 ≠ 0 at every application site.

Both operators are LTI, so a `DerivativeGains` stage commutes with a
`SmoothKernel` stage mathematically; the existing pre-kernel/post-kernel
application split stays valid unchanged.

## Current state (all paths relative to repo root)

The stage enum is `rust/trajectory/src/chain.rs:11-15`:

```rust
pub enum ChainStage {
    SmoothKernel(PiecewisePolynomialKernel),
    LinearPressureAdvance { k: f64 },
}
```

There is no generic operator evaluation — PA is special-cased by `match` at
**four** application sites:

1. **Closed-form straight lowering** — `rust/motion-pipeline/src/lowering/straight.rs:12-23`,
   applied per Bézier piece around line 113. Current transform:
   `c′_i = c_i + k·(i+1)·c_{i+1}` (monomial coefficients, derivative folded in).
   Stops (`break`) at the first `SmoothKernel` — the kernel is applied downstream.
2. **Sampled-path state** — `rust/motion-pipeline/src/lowering/sampled.rs:188-197`
   (`axis_state_side`): `pos += k·vel; vel += k·accel; accel` handling — read
   carefully, see "sampled site" below. Also `break`s on `SmoothKernel`.
3. **Post-kernel track transform** — `rust/motion-pipeline/src/shaper.rs:640-680`
   (`apply_trailing_zero_support` → `apply_pressure_advance_to_track`): applies a
   PA that sits *after* a kernel to the convolved output track by
   differentiating each Bézier piece (`piece.differentiate()`, line ~665) and adding.
4. **Follower projection** — `rust/motion-pipeline/src/follower_projection.rs:171-181`
   (`apply_leading_stages`): pre-kernel PA baked into the projected track.

Composition rules: `chain.rs:26-31` gives each stage a `composition_slot()` —
kernel is slot 0, PA is slot 1 ("derivative-gain"); `CompiledChain::compile`
(`chain.rs:117-138`) allows at most one stage per slot. `half_support` for PA is
`(0.0, 0.0)` (`chain.rs:22`).

The algo that produces the stage: `rust/trajectory/src/algos/linear_pressure_advance.rs`.

## Design

### The stage

Replace the variant:

```rust
pub enum ChainStage {
    SmoothKernel(PiecewisePolynomialKernel),
    DerivativeGains { k1: f64, k2: f64 },
}
```

- `composition_slot()` unchanged: slot 1, name "derivative-gain". One per axis —
  PA and mode-inverse on the *same* axis are mutually exclusive in v1, which is
  fine (PA is for the extruder, inversion for x/y).
- `half_support()` stays `(0.0, 0.0)` — a differential operator adds no window.
- `linear_pressure_advance.rs` compile becomes
  `Some(ChainStage::DerivativeGains { k1: *k, k2: 0.0 })`. Its params/bounds/
  type name are untouched — config-visible behavior identical.

Update every `match` on the old variant. The compiler will find them all; the
list above is the expected complete set plus test files.

### Per-site math for k2

The operator is `y = x + k1·ẋ + k2·ẍ` applied to the scalar axis track.

1. **Straight lowering** (`straight.rs`): monomial coefficients in local time.
   Extend the closed form:
   `c′_i = c_i + k1·(i+1)·c_{i+1} + k2·(i+1)·(i+2)·c_{i+2}`
   (terms beyond the coefficient vector are zero). Preserve the existing
   `mul_add` style. Note the output polynomial degree does not grow — only the
   coefficients change.
2. **Sampled state** (`sampled.rs:188-197`): the state carried is
   (pos, vel, accel). Second-order transform of the state:
   `pos′ = pos + k1·vel + k2·accel`, `vel′ = vel + k1·accel + k2·jerk`,
   `accel′ = accel + k1·jerk + k2·snap`. Jerk/snap are not carried in this
   state. **Investigate what this state is used for downstream** before deciding:
   if only position continuity matters at these evaluation points (as the
   current `accel = 0` write suggests — read the surrounding code and its
   consumers), transform pos with the full formula and handle vel/accel the same
   way the current code handles them for PA (it currently sets `accel = 0` —
   understand why, mirror the reasoning, don't guess). If the velocity value is
   load-bearing, the underlying curve is polynomial, so jerk is obtainable —
   check whether the caller can supply it. Fail loudly (assert/panic with a
   clear message) rather than silently dropping a needed term.
3. **Post-kernel track** (`shaper.rs:657-680`): rename
   `apply_pressure_advance_to_track` → `apply_derivative_gains_to_track`;
   differentiate each Bézier piece twice and accumulate
   `c + k1·d + k2·dd`. The convolved track is a quintic-or-higher fit, so the
   second derivative is well-defined; the kernel guarantees C² output (bell and
   the be2 kernels vanish at their support edges with continuous derivative), so
   no new discontinuities appear.
4. **Follower projection** (`follower_projection.rs:171-181`): same coefficient
   transform as site 1/3 — inspect which representation it operates on and
   extend identically.

### What is explicitly out of scope

- The planner does **not** currently fold chain output into velocity/accel
  limits (verified: `motion-pipeline/src/planner.rs` never consults the chain).
  The k2 term demands extra motor acceleration proportional to jerk; with the
  repo's jerk limits this is bounded but unbudgeted. Do not build limit-folding
  in this task. Leave a `TODO:` marker where the planner builds `VelocityLimits`.
- The `mode_inverse` algo/config plumbing — separate task.

## Tests

- All existing tests must pass unchanged (`cargo nextest run` from `rust/`) —
  PA behavior with k2 = 0 must be bit-identical; the snapshot suite
  (`./snapshots/snapshot-tests.sh --ci`) must stay green with **zero changed
  cases** (baselines: 33 ok expected on this branch except pre-existing 2
  changed + 2 pending in post_processor group from the smooth_zv/mzv work —
  those are awaited baseline reviews, not yours; do not regenerate baselines).
- New unit tests (separate test files per repo rules — e.g. extend
  `rust/motion-pipeline/src/tests.rs` and `rust/trajectory/src/chain/tests.rs`):
  - `DerivativeGains { k1: 0, k2: c }` applied to a known cubic axis track
    equals `x + c·ẍ` evaluated analytically, at each application site you can
    reach from a test (straight lowering and post-kernel at minimum).
  - Composition: kernel + derivative-gains in both orders produce the same
    trajectory to numerical tolerance on a smooth test signal (LTI commutation) —
    this is a strong whole-machinery check. If an existing test fixture makes
    this easy (see `rust/motion-pipeline/src/tests.rs` helpers like
    `xy_shaper_follower_chains`), use it.
  - Two derivative-gain post-processors on one axis still rejected
    (`UnsupportedComposition`) — likely already covered; verify the test names
    still make sense after the rename.

## Verification gates (all must be green before PR)

```
cd rust && cargo nextest run
./scripts/ci.sh quick
./scripts/ci.sh py          # klippy untouched, but cheap — run it
./snapshots/snapshot-tests.sh --ci
```

PR bases on `sota-motion`. Never amend or force-push. No Claude/Co-Authored-By
trailers. Comments are a failure of expression — express intent through naming
(`apply_derivative_gains_to_track`, `k1`, `k2` with doc on the stage enum only
if the units/semantics can't be named).
