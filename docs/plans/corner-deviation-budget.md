# Plan: `corner_deviation` as the primary corner budget, with kernel σ² deduction

## Why

Two problems, one root cause:

1. **scv is the wrong primary unit.** The clothoid fitter already converts scv
   to a junction-deviation distance the moment it uses it
   (`rust/geometry/src/fitter.rs:620-623`:
   `delta = scv² · (√2−1) / accel`, in mm). The physical budget the user cares
   about is that distance — how far the printed corner may deviate from the
   commanded geometry. Corner *speed* should be an output (planner rides the
   accel limit through whatever blend fits the budget), not an input.
2. **Smoothing kernels spend the same budget invisibly.** A convolution kernel
   pulls the path inward by ≈ (σ²/2)·a_normal wherever it curves (σ² = the
   kernel's second moment in s²). Today that deviation stacks *on top of* the
   blend tolerance — the corner budget is spent twice and the user controls
   neither the sum nor the split. The fix: the configured deviation is the
   **total** budget; the fitter deducts the kernel's known share and spends only
   the remainder on blend geometry.

## Part 1 — deviation-primary config with scv compatibility

### Config surface

New `[printer]` option `corner_deviation` (mm, ≥ 0; 0 = sharp corners /
zero-deviation behavior, matching what scv = 0 means today).

- **Do not name it `max_deviation` or `max_path_deviation`** — a
  `max_path_deviation` option already exists as the NURBS grid-fit tolerance
  (`klippy/motion_setup.py:175-179`, `fit_tolerance_mm`), a completely
  different knob. Collision here would be a support nightmare.
- Compatibility: `square_corner_velocity` remains accepted. If **both** are set
  in config, fail loudly at startup (config error naming both keys). If only
  scv is set, convert once at parse/validate time:
  `corner_deviation = scv² · (√2−1) / max_accel` — using the printer's
  configured `max_accel`. If neither is set, default = the conversion of the
  current default scv (5.0 mm/s) at max_accel, preserving today's out-of-box
  behavior.

### Semantics change (intentional, document it)

Today `delta` is recomputed per junction from the *move's* accel limit
(`junction_deviation(m_in).min(junction_deviation(m_out))`, `fitter.rs:281,553`),
so low-accel moves get proportionally smaller deviation (constant corner
speed). With deviation primary, `delta` is constant and corner *speed* scales
with √accel instead. That is the point of the change: constant geometric error,
speed as the free variable. Mention this in the README/option docs.

### Plumbing (follow the existing option template end to end)

1. `klippy/motion_setup.py` `read_limits` (:152-188): parse `corner_deviation`,
   keep parsing `square_corner_velocity`, enforce mutual exclusion, convert.
   `CartesianLimits` namedtuple (:20-30) — **field order is load-bearing**
   (see comment :9-12); extend it and the matching Rust extraction together.
2. PyO3 boundary: `rust/motion-engine/src/bridge/planner_api.rs`
   `CartesianLimitsArg` (:67-88).
3. `rust/planner-config/src/lib.rs`: `CartesianLimits` (:381-389), defaults
   (:391-402), `validate()` (:404-421). Store `corner_deviation_mm` as the
   canonical field; scv does not need to exist in the Rust config at all if
   klippy converts before the boundary — prefer that (one canonical
   representation past the boundary; keep the conversion in one place).
4. `rust/geometry/src/frontend.rs` `VelocityLimits` (:8-11): replace
   `square_corner_velocity_mm_s` with `corner_deviation_mm`; update
   construction at `planner_api.rs:413-429` and `pipeline_setup.rs:53`.
5. `rust/geometry/src/fitter.rs:620-623`: `junction_deviation()` returns the
   configured deviation directly (the min-of-in/out at :281 and :553 keeps
   working if per-`[limit]` overrides ever differ).

### Runtime tuning and status readback

- `set_square_corner_velocity` (`planner_api.rs:569-592`,
  `runtime_square_corner_velocity` in planner-config :357) must keep working:
  convert to deviation at the same single conversion point. Add a sibling
  runtime setter for deviation, and accept `CORNER_DEVIATION=` in
  `SET_VELOCITY_LIMIT` (`klippy/motion.py:782-788`) alongside
  `SQUARE_CORNER_VELOCITY=` (setting both in one command: error).
- Status readback (`klippy/motion.py:380-400` `get_status`, `_effective_limits`
  :355-362, `effective_limits()` planner-config :447-454): keep reporting
  `square_corner_velocity`, now **derived**:
  `scv = sqrt(corner_deviation · max_accel / (√2−1))` from effective limits, so
  existing frontends/macros reading it keep seeing a sensible value. Also
  report `corner_deviation` as a new status field. Telemetry
  (`klippy/extras/telemetry.py:172`) follows whatever `get_status` exposes —
  verify it doesn't break.

## Part 2 — kernel σ² deduction

### Second moment of a kernel

Add to `PiecewisePolynomialKernel` (`rust/nurbs/src/algebra.rs:100-141`) a
`second_moment(&self) -> f64`: analytic ∫ t²·k(t) dt summed over pieces
(each piece is a polynomial in absolute t on [u_start, u_end] — integrate the
degree+2 polynomial exactly; no sampling). All shipped kernels are
mean-centered (bell/triangle by symmetry, smooth_zv/smooth_mzv by construction
in `rust/trajectory/src/kernel.rs`), so the moment about t = 0 is the variance.
Add a unit test against the closed forms: bell σ² = T²/28, triangle σ² = T²/24,
and the zv/mzv kernels against a numerically integrated reference.

### Exposing it and deducting it

- `CompiledChain` / `AxisChainSet` (`rust/trajectory/src/chain.rs`): expose the
  kernel variance per axis (0.0 when no kernel; there is at most one kernel per
  axis by the composition rule). `DerivativeGains` stages contribute 0.
- The fitter needs, per junction: `deducted = (max σ² over the axes being
  blended) / 2 × the accel limit used for that junction`, and then
  `delta_effective = corner_deviation − deducted`.
  Plumb the variance into `VelocityLimits` (a `kernel_variance_s2` field set
  where limits are built, `planner_api.rs:413-429` — it has access to the
  compiled chains) or into `CornerFitConfig`
  (`rust/geometry/src/fitter/config.rs:6-24`) — pick whichever the fit-stage
  data flow makes least awkward; `VelocityLimits` is per-move, which matches
  `junction_deviation(limits)`'s existing signature and handles per-`[limit]`
  accel differences for free. First cut: worst axis σ² (conservative,
  isotropic); leave the direction-dependent refinement as a TODO marker.
- **Fail loudly**: if `deducted >= corner_deviation` (and corner_deviation > 0),
  the config is unsatisfiable — raise a clear error at planner init (and at
  runtime chain/param updates) saying which kernel spends how many mm of the
  budget at the configured accel, e.g. "smooth kernel on axis x already
  deviates 0.081 mm at accel 100000; corner_deviation is 0.050 — increase
  corner_deviation or shorten the kernel". Do not clamp silently.
- **Runtime coherence**: `SET_POST_PROCESSOR` recompiles chains and swaps them
  at a pipeline fence (`rust/motion-core/src/worker/ingress.rs:250-254`,
  `Control::SetAxisChains`). A kernel duration change alters σ², which alters
  the fitter's effective delta — the deduction must refresh through the same
  fence-synchronized path that delivers the new chains, so fit stage and shaper
  never disagree. Trace how `SetAxisChains` reaches (or fails to reach) the fit
  stage and wire the new value along it; if the fit stage currently receives
  nothing on that control path, that's the integration point to build. Same for
  runtime accel-cap changes (`set_accel_cap`) if the deduction caches an accel
  value — prefer computing the deduction per junction from the live limits so
  there is no cache to invalidate.

## Interaction with the other two plans

Independent of `derivative-gains-chain-stage.md` and
`mode-inverse-post-processor.md` — `mode_inverse` has zero σ² and no support,
so it deducts nothing (correct: inversion adds no path deviation). No file
conflicts expected except possibly `chain.rs` (both touch it — coordinate merge
order; the σ² exposure is additive and small).

## Tests

- Rust: `junction_deviation` returns the configured deviation; σ² closed-form
  unit tests (above); fitter-level test that a kernel-bearing chain shrinks the
  blend (compare trim/deviation of the fitted corner with and without a kernel
  at fixed corner_deviation); the unsatisfiable-budget error fires.
- Python (`./scripts/ci.sh py` — required, this touches `klippy/`): config
  parsing (deviation only / scv only / both → error / neither → default),
  status readback of derived scv, `SET_VELOCITY_LIMIT` with each key.
- Snapshots: **expect changes** — corner geometry shifts for every case with a
  post-processor kernel (the deduction shrinks blends), and possibly cases
  whose per-move accel differs from max_accel (semantics change in Part 1).
  Run `./snapshots/snapshot-tests.sh --ci`, report exactly which cases changed
  and why in the PR description. **Do not regenerate baselines** — the user
  reviews and accepts (repo rule). If cases change that the analysis says
  should NOT change, that's a bug — stop and investigate.
- Consider splitting Part 1 and Part 2 into separate PRs (Part 1 changes
  semantics user-visibly; Part 2 is the budget fix) — both base on
  `main`.

## Verification gates

```
cd rust && cargo nextest run
./scripts/ci.sh quick
./scripts/ci.sh py
./snapshots/snapshot-tests.sh --ci
```

Never amend or force-push. No Claude/Co-Authored-By trailers. Comments are a
failure of expression — encode the budget arithmetic in well-named functions
(`kernel_corner_deviation_mm`, `effective_junction_deviation`) rather than
comments.
