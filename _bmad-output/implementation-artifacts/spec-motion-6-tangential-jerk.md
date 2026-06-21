---
title: 'Build step 6 — tangential 1-D jerk limit in the velocity lookahead: closed-form double-S kinematics (reach + apex) replacing constant-accel ramps, trim peak velocity on short straights'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: '0cadc816b08612a9ae2c622969aeb37b8036ee93'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-5-velocity-planning.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The step-5 sweep assumes infinite tangential jerk: every ramp is constant-accel (`v_out² = v_in² + 2aL`, trapezoid apex `√((v_s²+v_e²)/2 + aL)`). A real `+a_max → −a_max` reversal landing on a short straight between two features is physically unrealizable — instantaneous accel sign-flip is infinite jerk, which the input shaper cannot absorb and the extruder cannot follow. Build-sequence step 6.

**Approach:** Add a single tangential **jerk limit** and replace the two constant-accel formulas with their closed-form **double-S (seven-segment) jerk-limited** analogs, wired into the *same* node-based forward-backward sweep — no new pass, no fitter touch, no σ read (that is step 7), no geometry change. Tangential jerk lives entirely in `s(t)`. Two primitives, both closed-form: (1) `reach(v0,L,a,J)` — max end-speed reachable over `L` under accel `a` and jerk `J` — drives both sweep passes; (2) `apex(v_s,v_e,L,a,J,ceiling)` — the peak the move actually attains, **trimmed below `ceiling` on short straights so the bounded-jerk up-then-down reversal fits** `L`. The distance of a symmetric velocity change is `T·(v_in+v_out)/2` (the double-S velocity profile is point-symmetric), giving a Cardano-cubic (accel-triangular regime) / quadratic (accel-trapezoidal regime) inverse for `reach` and a monotone-bisection root for `apex`. Both reduce **exactly** to the step-5 formulas as `J→∞`.

## Boundaries & Constraints

**Always:**
- Strictly additive within `geometry::velocity`: new child module `velocity::scurve` (closed-form 1-D kinematics) + jerk wiring in `velocity.rs`. No edits to `fitter`, `frontend`, `path`, `segment`, `gcode`, `trajectory`.
- Jerk knob is `VelocityConfig::max_jerk_mm_s3` (additive field). Validate at `plan_velocity` entry: `max_jerk_mm_s3 > 0.0` and not `NaN`; `+∞` is **allowed** and recovers step-5 constant-accel behavior bit-for-bit. Reject `≤0`/`NaN` with `VelocityError::InvalidConfig`.
- `reach(v0,L,a,J)` is the *only* node-speed feasibility law in both sweep passes (replaces `√(v0²+2aL)`); the result is `≤` the constant-accel reach, so the chain stays accel-feasible (`|v_out²−v_in²| ≤ 2aL`) **and** jerk-feasible.
- `apex` ∈ `[max(v_s,v_e), ceiling]`, monotone in the peak; returns `ceiling` when the full accel-up→decel-down fits with room, else the trimmed peak that exactly fills `L`. `cruise_v = apex(...)`.
- Symmetric-change distance: `dist(v0,v1) = T·(v0+v1)/2`, `T = 2√(Δ/J)` when `Δ ≤ a²/J` (accel never reaches `a`), else `Δ/a + a/J`, `Δ=|v1−v0|`. This identity (mean speed = endpoint mean for a point-symmetric double-S) is the spine of both primitives.
- `MoveVelocity` carries `jerk` (the per-move limit, for the downstream `s(t)` builder — progressive IR enrichment); `VelocityReport.jerk_bound` counts moves whose cruise the jerk limit trimmed below the accel-only apex.
- Determinism preserved: identical input ⇒ identical `VelocityProfile`. Bisection iteration count is fixed (no data-dependent loop bound).
- All step-5 fail-loud seam guards (`Inconsistent`/`NonAlphabet`/`NonFinite`) unchanged.

**Ask First:**
- Moving the jerk knob out of `VelocityConfig` into per-move `VelocityLimits` (a step-3 type) — would let M-code set jerk per move but widens blast radius.
- Reading `σ` to ride the lateral limit inside a clothoid (that is step 7), or coupling jerk with curvature.
- Emitting the seven-segment time breakpoints / `s(t)` here (that is execution lowering, the EX stage).
- The numeric **default** `max_jerk_mm_s3` value — see Design Notes; it is a tuning placeholder pending the SPEC "jerk-limit floor" open question.

**Never:**
- No fully-coupled jerk TOPP / SOCP / SLP; no per-axis or lateral jerk constraint (lateral jerk is free on a clothoid — geometry owns it). No new sweep, no grid, no Fresnel/`PositionProfile`.
- No extruder-transient / PA-augmented (`τ·s⃛`) bound and no volumetric-flow cap (needs `τ` + flow config not in the IR — stays deferred, see `deferred-work.md`).
- No change to step-5 caps, junction-stop derivation, or the `min(feedrate,max_velocity,√(a/κ))` ceiling.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Long straight, jerk slack | `Line`, `L` ≫ ramp length | cruise reaches `ceiling`; `jerk_bound` not incremented | N/A |
| Short straight between features | low `start_v`/`end_v`, small `L` | `cruise_v < ceiling` **and** `< √((v_s²+v_e²)/2 + aL)` (jerk trims below the accel apex); reversal fits | N/A |
| Accel-triangular reach | `Δv ≤ a²/J` | `reach` via Cardano cubic; `dist == L` within tol | N/A |
| Accel-trapezoidal reach | `Δv > a²/J` | `reach` via quadratic; `dist == L` within tol | N/A |
| `J = +∞` | any chain | `VelocityProfile` byte-identical to the step-5 constant-accel plan | N/A |
| Arc / blended corner cap | curvature-bound ceiling | cruise still capped at `√(a/κ_peak)`; jerk only ever lowers, never raises | N/A |
| Sharp corner / virtual move | step-5 stop conditions | node pinned `v=0`; neighbors ramp to/from 0 under jerk | N/A |
| Bad jerk config | `max_jerk_mm_s3 ≤ 0` or `NaN` | — | `Err(VelocityError::InvalidConfig)` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/velocity/scurve.rs` — **new**: `vel_change_distance`, `jerk_limited_reach` (Cardano/quadratic by accel regime), `jerk_limited_apex` (monotone bisection in `[max(v_s,v_e), ceiling]`). Pure free functions, no I/O; ends with `#[cfg(test)] mod tests;`.
- `rust/geometry/src/velocity/scurve/tests.rs` — **new**: regime-boundary, `dist`-vs-numeric-integration cross-check, monotonicity, and the `J→∞` reduction to `√(v0²+2aL)` / `√((v_s²+v_e²)/2+aL)`.
- `rust/geometry/src/velocity.rs` — wire `mod scurve;`; swap both sweep passes to `scurve::jerk_limited_reach`, cruise to `scurve::jerk_limited_apex`; add `VelocityConfig::max_jerk_mm_s3` (+ entry validation → `InvalidConfig`), `MoveVelocity.jerk`, `VelocityReport.jerk_bound`, `VelocityError::InvalidConfig`.
- `rust/geometry/src/velocity/tests.rs` — re-express `short_move_apex_below_ceiling` jerk-aware (it pins the constant-accel apex `√(a·L)`); add the matrix rows above.
- `rust/geometry/src/lib.rs` — re-exports unchanged (additive fields ride existing `pub use`).
- `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — "Continuity and jerk" + "Velocity planning" §: tangential jerk in `s(t)`, Ruckig closed-form S-curve, lookahead trims peak on short straights.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/velocity/scurve.rs` — implemented `velocity_change_distance`, `max_reachable_velocity` (Cardano triangular / quadratic trapezoidal), `peak_velocity` (fixed-step monotone bisection); `+∞` jerk drives `a²/J → 0` into the trapezoidal branch and recovers the constant-accel forms exactly.
- [x] `rust/geometry/src/velocity.rs` — added `max_jerk_mm_s3` to `VelocityConfig` (default `100_000.0`, flagged TODO) + entry validation (`>0`, not `NaN`, `+∞` ok) → `VelocityError::InvalidConfig`; both sweep passes and the cruise apex now call the `scurve` primitives; added `MoveVelocity.jerk` and `VelocityReport.jerk_bound` (incremented when the jerk apex falls below the accel-only apex).
- [x] `rust/geometry/src/velocity/scurve/tests.rs` — paired unit tests for both accel regimes, the symmetric-distance identity vs numeric `∫v dt`, monotonicity of reach, and the `J→∞` reduction.
- [x] `rust/geometry/src/velocity/tests.rs` — `short_move_apex_below_ceiling` re-expressed jerk-aware (`short_move_apex_trimmed_by_jerk_below_accel_apex`); added `infinite_jerk_recovers_constant_accel_apex` and `invalid_jerk_config_is_rejected`.

**Acceptance Criteria:**
- Given a short straight with low end speeds and small `L`, when planned, then `cruise_v < ceiling` and `cruise_v < √((start_v²+end_v²)/2 + accel·length)` (jerk strictly trims below the step-5 accel apex) and `jerk_bound ≥ 1`.
- Given any planned chain, when checked, then forward-backward feasibility still holds (`|end_v²−start_v²| ≤ 2·accel·length + tol`, `cruise_v ≤ ceiling`, rest at both ends) — jerk reach is `≤` accel reach so step-5 invariants are a superset.
- Given `VelocityConfig { max_jerk_mm_s3: f64::INFINITY, .. }`, when planned, then the `VelocityProfile` equals the step-5 constant-accel plan within `1e-9` (ignoring the additive `jerk`/`jerk_bound` fields).
- Given `reach(v0,L,a,J)`, when its result `v1` is fed to `vel_change_distance(v0,v1,a,J)`, then the distance equals `L` within `1e-6` (the inverse is exact), and `reach` is monotone non-decreasing in `L` and in `J`.
- Given `max_jerk_mm_s3 ≤ 0` or `NaN`, when planned, then `Err(VelocityError::InvalidConfig)`; given `+∞`, then `Ok`.
- (Additive) workspace builds; `cargo nextest run -p trajectory` unchanged; `frontend`, `fitter`, `path`, `segment`, `gcode` byte-for-byte untouched; the `velocity` module contains no `point_at`/`heading_at`/`PositionProfile` and no `σ`/`dkappa_ds` read in the speed law (only the existing validation guard).

## Spec Change Log

- [Review][Patch] **Bisection precondition unguarded (Blind Hunter).** `peak_velocity` bisects `[max(v_in,v_out), ceiling]`; if a node speed ever exceeded its ceiling, `lo > hi` would converge to garbage. Structurally impossible — the forward-backward sweep inits each node to `min(adjacent ceilings)` and only lowers it, so `start_v,end_v ≤ caps[j].ceiling` always — but the precondition is now asserted fail-loud (`debug_assert!(lo <= ceiling)`). KEEP: the sweep's node ≤ ceiling invariant. [`scurve.rs`]
- [Review][Patch] **Dimensional mislabel in the `jerk_bound` threshold (Blind Hunter).** The observability counter reused the length tolerance `LENGTH_EPS_MM` (mm) as a velocity tolerance (mm/s). Numerically harmless but unclear; replaced with a dedicated `VELOCITY_EPS_MM_S`. [`velocity.rs`]
- [Review][Patch] **`J=∞`-equals-step-5 AC tested only on a single move (Acceptance Auditor).** Added `infinite_jerk_recovers_constant_accel_plan_chainwide` — a 4-move Line/Arc/Clothoid/Line chain at `J=∞` asserting every `cruise_v` equals the step-5 `ceiling.min(√(½(v_s²+v_e²)+aL))` apex within `1e-9` and `jerk_bound == 0`. [`velocity/tests.rs`]
- [Review][Reject] **`jerk=+∞` collapses distance to 0 (Blind Hunter, HIGH).** Reviewer arithmetic error: `delta ≤ accel²/∞` is `delta ≤ 0`, false for `delta>0`, so the *trapezoidal* branch runs (`a/J → 0`) and recovers constant-accel exactly. Contradicted by passing `infinite_jerk_recovers_*` tests and the Edge-Case Hunter's independent check (`vcd(20,120,∞)=7.0`).
- [Review][Reject] **No `length>0` guard in the helpers / `∞`-ceiling public-API fragility (Blind + Edge-Case Hunters, LOW).** Guaranteed upstream: `plan_velocity` validates `length > LENGTH_EPS_MM` before any `scurve` call, and `VelocityLimits::check` forces a finite `max_velocity` that bounds `ceiling` finite. Not reachable from the validated frontend; the loud failure already lives at ingest.

## Design Notes

**Why distance `= T·(v0+v1)/2`.** A symmetric double-S accel profile (`0→peak→0`, triangular or trapezoidal) makes `v(t)` point-symmetric about its midpoint: `v(t)+v(T−t)=v0+v1`. So `∫₀ᵀ v dt = T·(v0+v1)/2` — no separate integral. This collapses the seven-segment kinematics to one clean identity and is what makes `reach`/`apex` closed-form.

**The `J→∞` reduction (and why it is the safety check).** As `J→∞` the triangular regime vanishes (`a²/J→0`), `T→Δ/a`, and `dist→(v1²−v0²)/(2a)` — i.e. `reach→√(v0²+2aL)` and `apex→√((v_s²+v_e²)/2+aL)`, the exact step-5 formulas. Hence the `J=∞`-equals-step-5 AC: the new code is provably a strict generalization, and step-5's Dong-Stori optimality argument carries (jerk only tightens reach, never loosens it).

Sketch (reach, triangular regime — `L ≤ L* = (2a/J)(v0 + a²/2J)`):
```
// L = u·(2v0 + J·u²),  u = √(Δ/J)   ⇒  J·u³ + 2v0·u − L = 0  (one real root, Cardano)
let p = 2.0 * v0 / jerk;            // ≥ 0
let q = -length / jerk;             // < 0
let disc = (q*q/4.0 + p*p*p/27.0).sqrt();
let u = (-q/2.0 + disc).cbrt() + (-q/2.0 - disc).cbrt();
v0 + jerk * u * u                   // = v0 + Δ
```

**Default jerk is a placeholder.** The SPEC lists the "jerk-limit floor" (largest jerk that still lets `max_accel` reach the no-ring ceiling) as open; pick a documented finite default that barely binds on long moves and binds on sub-mm straights, and flag it for tuning. It lives in `VelocityConfig` (global) because, unlike per-move `accel`, jerk is a single machine constant in V1.

## Verification

**Commands:**
- `cargo nextest run -p geometry` — expected: new `scurve` + `velocity` tests pass, other geometry tests unchanged.
- `cargo nextest run -p trajectory` — expected: unchanged (additivity).
- `./scripts/ci.sh rust-clippy` && `./scripts/ci.sh rust-fmt` — expected: green (`-D warnings`).
- `! grep -rnE 'point_at|heading_at|PositionProfile' rust/geometry/src/velocity.rs rust/geometry/src/velocity/` — expected: no matches.
- `git diff --stat` on `rust/geometry/src/{frontend.rs,fitter.rs,path,segment.rs}` and `rust/gcode` — expected: empty.

## Suggested Review Order

**Design intent (start here)**

- The whole jerk wiring: validate the knob, then both sweep passes and the cruise apex call the closed-form `scurve` primitives — no new pass.
  [`velocity.rs:72`](../../rust/geometry/src/velocity.rs#L72)

**The speed law — closed-form double-S kinematics (highest-leverage math)**

- The spine identity: symmetric velocity change ⇒ `dist = T·(v_in+v_out)/2`, with the triangular/trapezoidal accel-regime split.
  [`scurve.rs:3`](../../rust/geometry/src/velocity/scurve.rs#L3)
- `max_reachable_velocity` — the Cardano cubic (triangular) / quadratic (trapezoidal) inverse; reduces to `√(v0²+2aL)` as `J→∞`.
  [`scurve.rs:13`](../../rust/geometry/src/velocity/scurve.rs#L13)
- `peak_velocity` — monotone bisection trims the peak below the ceiling so the bounded-jerk reversal fits; precondition asserted fail-loud.
  [`scurve.rs:30`](../../rust/geometry/src/velocity/scurve.rs#L30)

**Sweep + cruise wiring (where the law replaces step-5's constant-accel)**

- Both forward/backward passes now use jerk-limited reach instead of `√(v²+2aL)`.
  [`velocity.rs:150`](../../rust/geometry/src/velocity.rs#L150)
- Cruise = jerk apex; `jerk_bound` counts where jerk trims below the accel-only apex.
  [`velocity.rs:166`](../../rust/geometry/src/velocity.rs#L166)

**Knob, config & fail-loud (boundary)**

- Global jerk knob with documented default; entry validation rejects `≤0`/`NaN`, allows `+∞`.
  [`velocity.rs:15`](../../rust/geometry/src/velocity.rs#L15)

**Tests (peripherals)**

- Headline AC: short straight trims below the accel apex, `jerk_bound≥1`, distance round-trips.
  [`tests.rs:202`](../../rust/geometry/src/velocity/tests.rs#L202)
- Chain-level `J=∞` equivalence to the step-5 constant-accel plan.
  [`tests.rs:332`](../../rust/geometry/src/velocity/tests.rs#L332)
- Primitive round-trip (`reach` inverts `dist`) and numeric-integration cross-check of the identity.
  [`scurve/tests.rs:55`](../../rust/geometry/src/velocity/scurve/tests.rs#L55)
