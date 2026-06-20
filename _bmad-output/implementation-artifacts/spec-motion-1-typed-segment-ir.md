---
title: 'Build step 1 — typed-segment IR (Line | Arc | Clothoid) + follower channel'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'ddde61701ecb9c9e2eb02cb2512102104e043c2d'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-velocity-planner-segment-ir-requirements-research-2026-06-18.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The rewrite's foundation is a closed-form typed-segment IR — `enum Segment { Line | Arc | Clothoid }` + a follower channel — exposing a small κ-space read-contract the velocity planner consumes without any numerical arc-length table. The architecture annotates this "(largely exists)", but only the **follower half** genuinely does (`FollowerDemand`, `try_new_virtual`, the post-shaper odometer). The **spatial alphabet does not exist** (today's `Segment` is `{ Cubic | CornerBlend | Junction }` on NURBS — see Code Map); this slice is mostly build, not wire-up.

**Approach:** Build the `{ Line | Arc | Clothoid }` alphabet as a new, additive `path` module in `rust/geometry/`, alongside the running NURBS planner (no rewiring, near-zero blast radius). Each variant implements the committed **`CurvatureProfile`** read-contract (research §1.2) — arc length and curvature in closed form, no numerical integration, no `dyn`. Position/Fresnel/heading are barred from this side of the seam by module privacy and belong to execution lowering (step 2). Reuse the existing follower/virtual-path channel verbatim. Full test coverage per variant, per fail-loud edge case, and the v1 acceptance set (research §5).

## Boundaries & Constraints

**Always:**
- The planner-facing surface is exactly `CurvatureProfile`, closed-form, κ-space only:
  `s_len()` (>0, asserted), `kappa(s)`, `dkappa_ds(s)`, `kappa_peak() -> (s*, κ_max)`, `kappa_endpoints() -> (κ(0), κ(L))`.
- `dkappa_ds` (σ) is implemented, finite, and tested **from day one but left uncalled** until the step-6 jerk lookahead — a new consumer, not a trait migration. The σ-discipline is the type discipline: Line σ≡0/κ≡0; Arc σ≡0/κ≡const; Clothoid σ≡const/κ linear.
- `kappa_peak` is closed-form, never a root-find: Line `(·,0)`, Arc `(·,1/r)`, Clothoid `max(|κ(0)|,|κ(L)|)` (|linear| maxes at an endpoint).
- Closed enum + `match` dispatch (or per-variant `impl CurvatureProfile`); **no `dyn`**, no per-segment arc-length table, no numerical κ.
- Arc/Clothoid are **planar** primitives embedded in 3D via an explicit plane (origin + orthonormal basis); Line is a straight 3D segment. No torsion / helical curves.
- Reuse `geometry::FollowerDemand { axis_index, ratio }` unchanged; mirror `try_new_virtual`'s invariants for the zero-length-spatial follower-only move.
- Fail loudly via a typed error (extend/reuse `geometry::GeometryError`) on every degenerate construction — never silently clamp.
- Unit tests in a separate file (`path/tests.rs`), per project convention.

**Ask First:**
- Replacing, renaming, or removing any existing `geometry::Segment` / `CubicSegment` / NURBS code — this slice is strictly additive.
- Adding any position/Fresnel/heading **evaluation** to a step-1 type (that is step 2). If a task seems to need it, HALT.
- Introducing a junction record (`κ⁻,κ⁺,σ⁻,σ⁺,G1-flag, KappaStep`) — deferred to the fitter (step 4); do not seed it here without a decision.

**Never:**
- Position-space evaluation of any kind in step 1 — no `point_at`, no heading sampling, no Fresnel. Variants **store** the design-space anchor (start pose + plane basis) as data; nothing evaluates it until lowering (step 2).
- The fitter, corner blends, δ/SCV logic, velocity caps, junctions, or any G-code front-end (steps 3–8).
- G5 Bézier-spline G-code support — dropped for now. The alphabet is not required to represent G5; this narrows the sufficiency surface (research §2.1).
- Wiring the new IR into `shape_batch` / the temporal planner / the MCU path. Definition + tests only.
- `dyn Segment`, trait objects, or a numerical arc-length table for the new types.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected `CurvatureProfile` behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Line | start≠end | κ(s)≡0, σ≡0, `kappa_peak`=(·,0), `s_len`=‖end−start‖ | N/A |
| Arc | R>0, sweep≠0, plane | κ(s)≡1/R, σ≡0, `kappa_peak`=(·,1/R), `s_len`=R·\|sweep\| | N/A |
| Clothoid | κ₀, σ, L>0 | κ(s)=κ₀+σs, `kappa_peak`=max(\|κ(0)\|,\|κ(L)\|), `kappa_endpoints`=(κ₀,κ₀+σL) | N/A |
| Clothoid degenerate | σ=0 | κ≡κ₀ const (valid); κ₀=0 ⇒ Line case | N/A |
| Virtual follower-only move | zero spatial length, ≥1 follower, virtual_path_mm>0 | valid; arclength = follower displacement; no κ cap | N/A |
| Degenerate Line | start==end, no follower | reject | `ZeroDisplacement` |
| Bad Arc | R≤0 / sweep==0 / non-orthonormal basis | reject | typed error |
| Bad Clothoid | L≤0 / non-finite κ₀,σ,L | reject | typed error |
| Virtual move, no follower | zero spatial length, empty followers | reject | `ZeroDisplacement` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/segment.rs:33` -- existing `FollowerDemand { axis_index, ratio }` — reuse verbatim.
- `rust/geometry/src/segment.rs:120` -- existing `CubicSegment::try_new_virtual` — pattern to mirror for the container's virtual-path invariants.
- `rust/geometry/src/lib.rs` -- crate root + `GeometryError`; register the new `path` module and new error variants here.
- `rust/geometry/src/segment.rs:3` -- legacy `enum Segment { Cubic | CornerBlend | Junction }` — leave untouched; new enum lives at `geometry::path::Segment` (no name collision).
- `rust/nurbs/src/eval.rs:508` -- existing numerical `curvature_from_derivs` — reference for what closed-form κ replaces; do not modify.
- research doc §1.2 (`CurvatureProfile`), §1.4 (σ day-one hook), §5 (v1 acceptance set) -- the committed read-contract this implements.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/path/profile.rs` -- define `trait CurvatureProfile` (signature in Boundaries → Always); position is *not* on it — the seam, enforced by module privacy.
- [x] `rust/geometry/src/path/line.rs` -- `Line { start, end }` (Vec3) storing the anchor; `impl CurvatureProfile` with κ≡0, σ≡0; constructor rejects start==end -- trivial base case.
- [x] `rust/geometry/src/path/arc.rs` -- planar `Arc` (plane origin+orthonormal basis, radius, start_angle, sweep) storing the anchor; `impl CurvatureProfile` with κ≡1/R; constructor rejects R≤0 / sweep==0 / non-orthonormal basis -- constant-κ primitive.
- [x] `rust/geometry/src/path/clothoid.rs` -- `Clothoid` (start pose + plane basis, κ₀, σ, length); `impl CurvatureProfile` with κ(s)=κ₀+σs, closed-form endpoint `kappa_peak`; constructor rejects L≤0 / non-finite -- the linear-κ primitive the planner rides.
- [x] `rust/geometry/src/path/mod.rs` -- `enum Segment { Line | Arc | Clothoid }` delegating `CurvatureProfile` by `match`; path container = one spatial `Segment` + `Vec<FollowerDemand>` + virtual-path support mirroring `try_new_virtual`; re-export `geometry::path`.
- [x] `rust/geometry/src/lib.rs` -- `pub mod path;` + new `GeometryError` variants (e.g. `NonPlanarBasis`, `DegenerateArc`, `DegenerateClothoid`).
- [x] `rust/geometry/src/path/tests.rs` -- the v1 acceptance set + matrix (see AC below).

**Acceptance Criteria:**
- (AC-CP-1) Every constructed segment reports `s_len() > 0`; a would-be `s_len() ≤ 0` fails loudly at construction.
- (AC-CP-2, property) Given any implementor, `dkappa_ds(s)` matches a central difference of `kappa(s)` within tol across `[0,s_len]` — gated from v1 though `dkappa_ds` is otherwise uncalled, so the hook can't rot.
- (AC-CP-3) Given any variant, `kappa_endpoints() == (kappa(0), kappa(s_len))`.
- (AC-CP-4) Given any variant, `kappa_peak().1 ≥ |kappa(s)|` at sampled interior points (extremum dominance), and for the Clothoid equals `max(|κ(0)|,|κ(L)|)` in closed form.
- Given a Clothoid with σ=0, κ₀≠0, when κ(s) is sampled across `[0,L]`, then it equals κ₀ to f64 tolerance (continuity with the Arc case).
- Given any degenerate constructor input in the matrix, when construction is attempted, then it returns the typed error and never produces a segment.
- Given the new module, when the workspace builds, then `geometry::path` has no consumers yet and `cargo nextest run -p trajectory` is unchanged (additive-only proof).

*(Deferred, not step 1: AC-SEAM-1 cross-representation κ vs Fresnel — needs lowering, step 2. AC-NODE-1 node-coverage + junction records — step 4/5.)*

## Spec Change Log

## Design Notes

**Seam + the σ hook (rationale in research §1.1, §1.4).** `v ≤ √(a/κ)` never references position, so step 1 ships only the κ-space surface (`CurvatureProfile`) plus stored anchor data; position/Fresnel is step 2, kept off the trait by module privacy. `dkappa_ds` (σ) ships now — finite and tested (AC-CP-2) but **uncalled** until the step-6 jerk lookahead — so that upgrade is a new *consumer*, not a trait migration. Planar embedding: store `origin: Vec3` + orthonormal `(u,v): Vec3`; design point `(x,y) ↦ origin + x·u + y·v` (no torsion). G5 is out of scope (Never), which is *causally* why `kappa_peak` is a closed-form endpoint, not a numerical κ-search (§1.2).

**Bed-mesh Z (CAP-5) is nonlinear** in XY — unlike the constant-ratio `FollowerDemand` (linear in arc length). Whether mesh-Z is a generalized (nonlinear) follower is **open, not decided here** (probably not). The related helical-G2/G3 → planar-Arc + linear-Z-follower decomposition (SPEC, research §6.3) is the one place the planar-κ alphabet can genuinely leak — needs an explicit yes, out of step-1 scope. Recorded so nobody reuses `FollowerDemand` as-is for mesh-Z without a decision.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` -- expected: all geometry tests pass, including the new `path::tests`.
- `cd rust && cargo clippy -p geometry -- -D warnings` -- expected: clean.
- `cd rust && cargo nextest run -p trajectory` -- expected: unchanged/green (additive-slice proof — nothing downstream perturbed).
- `cd rust && cargo fmt --check` -- expected: clean.

## Suggested Review Order

**The seam (κ-space read-contract)**

- Entry point: the exact planner-facing surface — five closed-form methods, position barred.
  [`profile.rs:1`](../../rust/geometry/src/path/profile.rs#L1)

**Per-variant curvature (the σ-discipline)**

- Linear-κ primitive: closed-form endpoint `kappa_peak`, never a root-find.
  [`clothoid.rs:69`](../../rust/geometry/src/path/clothoid.rs#L69)

- Constant-κ primitive; constructor now guards the computed `radius·|sweep|` against underflow (AC-CP-1 fix).
  [`arc.rs:37`](../../rust/geometry/src/path/arc.rs#L37)

- Trivial base case: κ≡0, σ≡0; rejects start==end with `ZeroMotion`.
  [`line.rs:15`](../../rust/geometry/src/path/line.rs#L15)

**Dispatch + follower channel**

- Closed enum, `match` dispatch, no `dyn`; the planner reads `CurvatureProfile` off this.
  [`mod.rs:15`](../../rust/geometry/src/path/mod.rs#L15)

- Container mirrors `try_new_virtual`; empty-follower virtual move rejected with `ZeroMotion`.
  [`mod.rs:83`](../../rust/geometry/src/path/mod.rs#L83)

**Supporting changes**

- New typed error variants for degenerate construction (fail-loud).
  [`error.rs:80`](../../rust/geometry/src/error.rs#L80)

- Shared orthonormality helper extracted from arc/clothoid (de-duplication).
  [`basis.rs:11`](../../rust/geometry/src/path/basis.rs#L11)

- v1 acceptance set + I/O matrix, incl. the Arc-underflow regression test.
  [`tests.rs:253`](../../rust/geometry/src/path/tests.rs#L253)
