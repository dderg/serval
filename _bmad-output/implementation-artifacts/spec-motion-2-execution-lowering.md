---
title: 'Build step 2 — execution lowering for the geometry::path IR (position-space seam + constant-speed sampler)'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'b30737d17c45fb4b37f3c690beb392fa2a945197'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-1-typed-segment-ir.md'
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-velocity-planner-segment-ir-requirements-research-2026-06-18.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Step 1 shipped the κ-space planner contract (`CurvatureProfile`) and deliberately **barred position** from it (no `point_at`, no Fresnel, no heading) — that is the other half of the seam and lives in execution lowering. Nothing can yet turn a `geometry::path` segment into position-vs-time, so the IR cannot be observed, plotted, or checked against its own curvature. The architecture's build-step 2 is exactly this: "execution lowering from IR at constant speed — observability first."

**Approach:** Add an additive `path::lowering` module owning the **position-space** mirror of the seam: a `PositionProfile` trait (`point_at`, `heading_at`) with closed-form impls for Line (lerp) and Arc (planar circle), and a **Fresnel-integral** impl for Clothoid. Compose it with a trivial constant-speed time-law `s(t)=v·t` into a fixed-rate sampler that lowers a single `PathSegment` (spatial channel + follower channel) to position-vs-time. The headline gate is **AC-SEAM-1** (research §4 R1, "the only catastrophic one"): sample the lowered Fresnel curve, numerically estimate κ, assert it equals the analytic `CurvatureProfile::kappa`. Position stays off the κ-trait; the two halves are two modules, never one object.

## Boundaries & Constraints

**Always:**
- `PositionProfile` lives in `path::lowering`, **separate from** `CurvatureProfile`; position is never added to the κ-trait (seam by module separation). Surface: `point_at(s) -> [f64;3]`, `heading_at(s) -> [f64;3]` (unit tangent), closed enum `match` dispatch on `Segment`, no `dyn`.
- **Heading anchor convention (decided here, binding on the future fitter):** at `s=0` the unit tangent is `u`; the curve bends toward `+v` for positive κ. Arc point `= origin + r·(cos θ·u + sin θ·v)`, `θ(s)=start_angle + sign(sweep)·s/r`. Clothoid heading `φ(s)=κ₀·s + ½σ·s²`, point `= start_pose + (∫₀ˢcos φ)·u + (∫₀ˢsin φ)·v`. Both must satisfy `|d²r/ds²| = |κ(s)|` by construction.
- Clothoid position via a **Fresnel approximation, not a per-segment arc-length table** (CAP-4): a vetted rational approximation of the Fresnel integrals C(x)/S(x) with documented max abs error ≤ 1e-9, affine-mapped per (κ₀,σ,s). The **σ=0 limit is closed-form** (κ₀=0 → straight `∫=s`; κ₀≠0 → circular `sin/cos`); evaluate it directly, never divide by σ.
- Constant-speed sampler `lower_constant_speed(&PathSegment, speed_mm_s, rate_hz)`: emits samples at `t_k=k/rate`, arc length `s_k = min(speed·t_k, s_len)`, follower position `ratio·s_k` per `FollowerDemand`; the last sample lands exactly at `s_len`. Spatial samples carry `point_at(s_k)`; a **virtual (spatial-less) move** emits `position = None` and only advances followers over `[0, virtual_path_mm]`.
- Fail loudly via a typed `GeometryError` on non-finite / non-positive `speed_mm_s` or `rate_hz` — never clamp or pad.
- Unit tests in a separate file (`path/lowering/tests.rs`), per project convention.

**Ask First:**
- Any change to a step-1 type's stored fields or to `CurvatureProfile`. The lowering impls read the existing **public** anchor fields; if a field is genuinely missing (e.g. an explicit start-heading the convention above cannot supply), HALT.
- Emitting lowered samples through the structured-logging / `telemetry` pipeline rather than returning them as data. Default: return data only.
- Multi-segment concatenation, a running cross-segment pose, or any non-constant `s(t)`.

**Never:**
- Velocity planning, S-curve / jerk timing, fitter, corner caps, junctions, or any `s(t)` other than constant speed (steps 4–8).
- Adding position/heading to `CurvatureProfile`, or letting the planner read position. The κ side stays position-free.
- Replacing, renaming, or removing the step-1 `path` types or the legacy NURBS `Segment`/`CubicSegment`. Strictly additive.
- A numerical arc-length **table** or position→arc-length inversion (the deleted 1024-point table). Per-point Fresnel evaluation is not a table.
- Wiring lowering into `shape_batch`, the temporal/trajectory planner, or the MCU path. Definition + tests only.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Line lower | start≠end | `point_at` lerps start→end; `heading_at`=unit(end−start); κ-from-position ≡ 0 | N/A |
| Arc lower | r>0, sweep≠0 | planar circle; `point_at(0)`=start pt, `point_at(L)`=end pt; κ-from-position ≡ 1/r | N/A |
| Clothoid lower (σ≠0) | κ₀,σ,L | Fresnel position; κ-from-position = κ₀+σs within tol; `heading_at`=(cos φ,sin φ) in basis | N/A |
| Clothoid lower (σ=0, κ₀≠0) | constant κ | matches the Arc circular limit to f64 tol (continuity) | N/A |
| Clothoid lower (σ=0, κ₀=0) | straight | matches the Line limit | N/A |
| Constant-speed sample | spatial seg, v>0, f>0 | times `k/f`, arc `min(v·t,L)`, last sample at exactly L; followers `ratio·s` | N/A |
| Virtual move lower | spatial=None, virtual_path_mm>0 | `position=None`; followers advance over `[0,virtual_path_mm]` | N/A |
| Bad sampler params | speed or rate ≤0 / non-finite | reject before sampling | typed error |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/path/profile.rs` -- `CurvatureProfile` (the κ side); reference only — **do not** add position here.
- `rust/geometry/src/path/line.rs`, `arc.rs`, `clothoid.rs` -- variants store public anchors (`start/end`; `origin/u/v/radius/start_angle/sweep`; `start_pose/u/v/kappa_0/sigma/length`) — lowering reads these.
- `rust/geometry/src/path/mod.rs` -- `Segment` enum + `PathSegment { spatial, followers, virtual_path_mm }`; `s_len`, `try_new_virtual` — sampler inputs. Register `pub mod lowering;`.
- `rust/geometry/src/error.rs:75` -- `GeometryError`; add the lowering-param variant.
- research §1.3 (position-via-Fresnel + L-consistency), §4 R1 / AC-SEAM-1 (the killer test), §1.1 (the seam); SPEC CAP-4; architecture "Representation (IR)" (Fresnel split) + build-step 2.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/path/lowering.rs` -- define `trait PositionProfile { point_at, heading_at }`; `impl` for `Line` (lerp), `Arc` (planar circle), `Clothoid` (via `fresnel`), and `Segment` (`match` dispatch); `struct LoweredSample { t_s, position: Option<[f64;3]>, followers: Vec<f64> }`; `fn lower_constant_speed(&PathSegment, speed_mm_s, rate_hz) -> Result<Vec<LoweredSample>, GeometryError>` with fail-loud param validation.
- [x] `rust/geometry/src/path/lowering/fresnel.rs` -- Fresnel C(x)/S(x) via the Cephes vetted rational approximation (uniform over all x, ~2e-17, no domain wall) + `clothoid_offset(kappa_0, sigma, s) -> (f64, f64)` returning `(∫cos φ, ∫sin φ)`, with the closed-form σ=0 / κ₀=0 limits.
- [x] `rust/geometry/src/path/mod.rs` -- `pub mod lowering;` (additive registration).
- [x] `rust/geometry/src/error.rs` -- new `GeometryError::InvalidLowering` variant for invalid lowering params (non-finite / non-positive speed or rate).
- [x] `rust/geometry/src/path/lowering/tests.rs` -- AC set below + the I/O matrix, with an independent composite Gauss-Legendre quadrature reference for the Fresnel positions.

**Acceptance Criteria:**
- (AC-SEAM-1, the R1 killer) For each variant, sampling `point_at` across `[0,s_len]` and numerically estimating curvature (`κ = |r′×r″|/|r′|³`) matches analytic `CurvatureProfile::kappa(s)` within tol — analytic-κ and lowered-position never silently disagree.
- (AC-POS-1) `point_at(0)` equals the stored start anchor; for a Line `point_at(s_len)==end`; for an Arc `point_at(s_len)` equals the closed-form end point.
- (AC-POS-2, property) `heading_at(s)` is unit and equals the normalized numerical derivative of `point_at(s)` within tol, for every variant.
- (AC-FRES-1) The Fresnel `clothoid_offset` matches a high-order Gauss-Legendre quadrature of `(∫cos φ, ∫sin φ)` within the documented bound, including the σ=0 / κ₀=0 limits (continuity with Arc and Line).
- (AC-SAMP-1) `lower_constant_speed` emits samples at `t_k=k/rate` with `s_k=min(speed·t_k,s_len)`, the final sample exactly at `s_len`, and follower positions `ratio·s_k`.
- (AC-SAMP-2) A virtual `PathSegment` lowers with `position == None` and followers advancing over `[0,virtual_path_mm]`.
- Given non-finite or non-positive `speed_mm_s`/`rate_hz`, `lower_constant_speed` returns the typed error and emits no samples.
- Given the new module, the workspace builds, `CurvatureProfile` is unchanged, and `cargo nextest run -p trajectory` is unchanged (additive-only proof).

## Spec Change Log

- **Review patch (signed-κ seam test).** Acceptance auditor noted AC-SEAM-1 compared `|κ|`, leaving the decided "+v bend for positive κ" convention unverified. Added `ac_seam1_clothoid_signed_curvature_matches_bend_direction` (signed κ about the u×v normal vs signed analytic `kappa(s)`). Avoids a wrong-way Fresnel passing the seam test. KEEP: Arc `kappa()` is magnitude-only (turn sign lives in `sweep`), so the abs-based check stays correct for Arc.
- **Resolved (frozen wording — Fresnel method).** Reviewers flagged a first cut that used a convergent power series (valid only `|x| ≤ 3`, fenced by a loud fail-out) + composite-Simpson reference, which met the frozen ≤1e-9 / independent-reference intent but not its literal "rational approximation" / "Gauss-Legendre" wording. Per human decision, swapped to the **Cephes vetted rational approximation** (uniform over all x, ~2e-17, no domain wall — so the earlier fail-loud guard is gone, moot) and a **composite Gauss-Legendre** test reference. Code now honors the frozen wording directly. Reason for rational over series: uniform accuracy and predictable (constant-time, branch-light) cost — the right properties for the fixed-rate execution path.

## Design Notes

**The seam, position side (research §1.1, §1.3).** Step 1's κ-trait is barred from position by design; step 2 adds the *position-space* trait in a sibling module and never back-references the κ-trait. The one thing binding them is AC-SEAM-1: κ analytically (planner path) vs κ numerically off the Fresnel position (lowering path) — "two code paths for the same curve," the textbook silent-disagreement setup, so the equivalence is a must-write test, not a nicety.

**Why the anchor convention is sufficient.** Clothoid stores `start_pose + (u,v)` but no explicit start-heading. The convention *tangent(0)=u, bends toward +v* fully determines `φ(s)=κ₀s+½σs²` and makes `|r″|=|κ|` hold by construction — no new field. The future fitter (step 4) must emit clothoids honoring this; recorded here so it isn't silently violated.

**Fresnel, σ=0 limit.** `φ=κ₀s+½σs²`; the affine map to standard C/S divides by `√|σ|`, so σ=0 is a removable singularity — branch to the circular limit (`∫cos κ₀t dt = sin(κ₀s)/κ₀`) and, for κ₀=0, the straight limit (`∫=s`). This keeps the Clothoid→Arc→Line degeneracy continuous (matrix rows 4–5) and avoids a 0/0.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` -- expected: all geometry tests pass, including new `path::lowering::tests`.
- `cd rust && cargo clippy -p geometry --all-targets -- -D warnings` -- expected: clean.
- `cd rust && cargo nextest run -p trajectory` -- expected: unchanged/green (additive-slice proof).
- `cd rust && cargo fmt --check` -- expected: clean.

## Suggested Review Order

**The seam (position barred from κ-space)**

- Entry point: the position-space trait, owned by lowering, separate from `CurvatureProfile`.
  [`lowering.rs:7`](../../rust/geometry/src/path/lowering.rs#L7)

- Closed-enum `match` dispatch — no `dyn`; mirrors the step-1 κ dispatch.
  [`lowering.rs:134`](../../rust/geometry/src/path/lowering.rs#L134)

**Per-variant position (the Fresnel half)**

- The clothoid's only Fresnel call — position from the closed-form offset.
  [`lowering.rs:123`](../../rust/geometry/src/path/lowering.rs#L123)

- Completion-of-the-square → standard C/S, with the σ=0 / κ₀=0 closed-form limits.
  [`fresnel.rs:126`](../../rust/geometry/src/path/lowering/fresnel.rs#L126)

- C(x)/S(x) via the Cephes rational approximation — uniform over all x, no domain wall.
  [`fresnel.rs:98`](../../rust/geometry/src/path/lowering/fresnel.rs#L98)

- Arc as a planar circle; tangent sign follows `sweep`.
  [`lowering.rs:97`](../../rust/geometry/src/path/lowering.rs#L97)

- Line lerp — the trivial base case.
  [`lowering.rs:71`](../../rust/geometry/src/path/lowering.rs#L71)

**Constant-speed lowering + fail-loud**

- The observability deliverable: `s(t)=v·t`, fixed-rate, last sample exactly at `s_len`, virtual moves emit `None`.
  [`lowering.rs:19`](../../rust/geometry/src/path/lowering.rs#L19)

- New typed error for non-finite / non-positive speed or rate.
  [`error.rs:83`](../../rust/geometry/src/error.rs#L83)

**Tests (the R1 killer first)**

- AC-SEAM-1 signed: κ-from-position vs signed analytic κ — verifies the +v bend convention (review patch).
  [`tests.rs:133`](../../rust/geometry/src/path/lowering/tests.rs#L133)

- Fresnel vs independent Gauss-Legendre quadrature (both rational branches), incl. the σ=0 / κ₀=0 limits.
  [`tests.rs:207`](../../rust/geometry/src/path/lowering/tests.rs#L207)

- Additive registration.
  [`mod.rs:5`](../../rust/geometry/src/path/mod.rs#L5)
