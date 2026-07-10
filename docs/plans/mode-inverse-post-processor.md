# Plan: `mode_inverse` post-processor

## Depends on

`docs/plans/derivative-gains-chain-stage.md` — the generalized
`ChainStage::DerivativeGains { k1, k2 }` stage. This task can be developed in
parallel against that interface contract, but cannot merge (or have its
integration tests pass) until that stage lands. If developing in parallel,
stub against the agreed variant shape and rebase.

## Why

Servo axes (EtherCAT, torque/velocity feedforward) can cancel the dominant
belt-compliance resonance by plant inversion instead of avoiding excitation:
for a 2nd-order mode with natural frequency ω = 2π·f and damping ratio ζ,
commanding the motor along

```
x_motor = x_head + (2ζ/ω)·ẋ_head + (1/ω²)·ẍ_head
```

makes the toolhead follow the nominal path. Unlike smoothing kernels this adds
**zero path deviation and zero delay**; residual ringing is proportional to
model error in (f, ζ). It amplifies high frequencies (the ẍ term), so the
intended axis chain pairs it with a short smoothing kernel that bandlimits the
input — e.g.

```
[post_processor slew]
type: smooth_bell
smooth_time: 0.0015

[post_processor belt_x]
type: mode_inverse
frequency_hz: 131.0
damping_ratio: 0.05

[axis x]
post_processors: slew, belt_x
```

Kernel and inversion are both LTI, so config order does not change the math;
the chain machinery already supports one kernel + one derivative-gain stage per
axis in either order.

## Deliverable

A new registered post-processor type, config-reachable end to end, runtime
tunable, with tests and snapshot cases.

### The algo

New file `rust/trajectory/src/algos/mode_inverse.rs`, mirroring the structure
of `rust/trajectory/src/algos/smooth_zv.rs` (read it first — it is the freshest
template):

- `type_name()` → `"mode_inverse"`.
- `params()` → two `ParamSpec`s, **in this order** (param order is the
  positional contract with `compile(values)`):
  - `frequency_hz`, `Bound::Positive`
  - `damping_ratio`, `Bound::NonNegative`
- `compile(&[frequency_hz, damping_ratio])`:
  - `omega = 2.0 * PI * frequency_hz`
  - `Some(ChainStage::DerivativeGains { k1: 2.0 * damping_ratio / omega, k2: 1.0 / (omega * omega) })`
  - A damping_ratio ≥ 1.0 is not a resonance; per the repo's fail-loudly rule,
    reject it. The `ParamSpec::check` bound machinery only knows
    Positive/NonNegative (`rust/trajectory/src/algos/mod.rs:33-61`), so either
    (a) extend `Bound` with a variant that carries an upper limit (preferred if
    it stays small — e.g. `Bound::UnitInterval` meaning `0.0 <= v < 1.0`), or
    (b) panic in `compile` with a clear message. Option (a) gives the user a
    proper `PostProcessorError::BadParam` at config parse and at
    `SET_POST_PROCESSOR` runtime tuning — do that.

Register in `rust/trajectory/src/algos/mod.rs`: `mod` + `pub use` + `REGISTRY`
entry. The registry list is what `supported_type_names()` reports in config
error messages.

### Config plumbing

None needed beyond registration — `[post_processor NAME]` sections are parsed
generically (`klippy/motion_setup.py`, `read_post_processors`) and validated
against the Rust registry (`rust/planner-config/src/lib.rs`,
`PostProcessorSet::compile`). Verify by adding a case to the Python test
`test/test_post_processor_sections.py` (see how `smooth_bell` is covered).

Runtime tuning comes for free via `SET_POST_PROCESSOR NAME=belt_x
FREQUENCY_HZ=128.5` (klippy/motion.py:800-818 → `set_param` → recompile →
`SetAxisChains` at a pipeline fence). Add a test in
`rust/trajectory/src/chain/tests.rs` that `set_param` on both keys works and
rejects out-of-bound values (mirror
`set_param_rejects_negative_and_non_finite_smooth_time`).

### Tests

- `rust/trajectory/src/chain/tests.rs`: compile produces the expected gains
  for a known (f, ζ) — assert `k1`, `k2` against hand-computed values;
  damping_ratio ≥ 1.0 and non-finite/negative params rejected; missing param
  error names the missing key.
- `rust/planner-config/src/tests.rs`: a `post_processor_decls` entry with
  `mode_inverse` compiles into an axis chain (mirror the existing smooth_bell
  entries); wrong/missing params produce errors naming `frequency_hz` /
  `damping_ratio`.
- `rust/pipeline-snapshot/src/tests.rs`,
  `all_post_processor_types_are_reachable`: add
  `("mode_inverse", [("frequency_hz", 40.0), ("damping_ratio", 0.1)])`.
- **The physics test** (most important — put it with the motion-pipeline tests,
  e.g. `rust/motion-pipeline/src/tests.rs`, where full-pipeline fixtures live):
  simulate the mode itself. Drive a damped oscillator
  `z̈ + 2ζω ż + ω² z = ω² x_cmd(t)` (i.e. z = toolhead, x_cmd = shaped motor
  command) by numerically integrating over the pipeline's output trajectory for
  a test move with a sharp velocity change; assert that with the
  `mode_inverse` stage the oscillator tracks the *nominal* (pre-inversion)
  trajectory to tight tolerance, and without it the residual oscillation
  amplitude is large. Use `libm` for any transcendental math (workspace clippy
  disallows `f64::sin` etc. for cross-platform determinism) and a simple RK4 or
  semi-implicit Euler at fine dt written inline in the test file.
- Snapshot case: new `snapshots/cases/post_processor/mode_inverse.cfg`
  mirroring `smooth_zv.cfg` (same [printer] block; `type: mode_inverse`,
  `frequency_hz: 40`, `damping_ratio: 0.1`). It will show as PENDING —
  **do not generate baselines**; the user reviews and accepts them
  (repo rule: new baselines are always generated by the user).

### Docs

README.md "Post-processors" section: one short paragraph. Position it as the
third kind of linear operator: smooths (kernels), sharpens (pressure advance),
and inverts (mode_inverse — cancels an identified resonance by commanding the
motor through the inverse of the belt-compliance model; pair with a short
smoothing kernel to bound high-frequency effort). Match the existing section's
voice; keep it tight.

## Out of scope

- Planner limit-folding of the extra acceleration the ẍ term demands
  (the planner does not consult chains for limits yet — pre-existing gap).
- Load-side identification tooling (fitting f and ζ from accelerometer data).
- Cross-axis (2×2) inversion for corexy modes not aligned with x/y.

## Verification gates

```
cd rust && cargo nextest run
./scripts/ci.sh quick
./scripts/ci.sh py
./snapshots/snapshot-tests.sh --ci   # new case PENDING is expected; nothing else may change
```

PR bases on `sota-motion`. Never amend or force-push. No Claude/Co-Authored-By
trailers. Comments are a failure of expression — the inversion formula's
provenance belongs in the README paragraph and in test assertions, not inline
comments.
