---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments:
  - _bmad-output/specs/spec-motion-pipeline-rewrite/SPEC.md
  - _bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md
  - _bmad-output/specs/spec-motion-pipeline-rewrite/.decision-log.md
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: "Velocity planner's requirements on the segment IR (geometry-first motion-pipeline rewrite)"
research_goals: "Derive the velocity planner's contract on the segment IR — κ(s), σ=dκ/ds, position-via-Fresnel, junction-discontinuity bounds — from architecture.md + SPEC.md, and verify the {Line|Arc|Clothoid} alphabet is sufficient now that G5 cubic-Bézier input is dropped."
user_name: 'dderg'
date: '2026-06-18'
web_research_enabled: true
source_verification: true
method: 'spec-internal derivation + BMAD party-mode adversarial roundtable (Winston/Architect, Amelia/Engineer, Murat/Test-Architect, Dr. Quinn/Problem-Solver)'
---

# Research Report: technical

**Date:** 2026-06-18
**Author:** dderg
**Research Type:** technical

---

## Research Overview

**Question.** Two coupled deliverables: (1) derive the velocity planner's *read-contract* on the segment IR — what each `{Line | Arc | Clothoid}` variant and each junction must guarantee about `κ(s)`, `σ = dκ/ds`, the position-via-Fresnel boundary, and the junction-discontinuity bounds; and (2) verify the three-letter alphabet is *sufficient* now that native G5 cubic-Bézier input is dropped.

**Method.** Ground truth is `SPEC.md` + `architecture.md` + the decision log. The derivation was pressure-tested through a BMAD party-mode roundtable of four independent subagents — Winston (architecture/seam), Amelia (implementation/read-contract), Murat (risk/test), Dr. Quinn (systems/sufficiency proof) — across three rounds with the project owner steering. The literature grounding (Dong-Stori, Moreton-Séquin, Bertolazzi-Frego, Ruckig, Pham) was already established in the spec's 2026-06-18 research pass and is cited here for the claims it underwrites, not re-derived.

**Verdict in one line.** The alphabet is **sufficient**, and dropping G5 is *causally why*. The velocity planner's contract is closed-form in κ-space; the one previously-muddy area (G2-vs-G3, extrusion's geometric demand, "the grid") resolved cleanly and made the system *simpler*, not richer.

---

## 1. The velocity planner's read-contract on the segment IR

### 1.1 The seam: κ-space vs position-space

The planner lives entirely in **κ-space**; execution lowering lives entirely in **position-space**. This is not a stylistic cut — it falls out of the physics: the speed constraint `v ≤ √(a/κ)` is written in curvature and never references position. Therefore:

- The planner has a **right** to demand `κ(s)` and arc length `L`, closed-form.
- The planner has **no right** to position. Fresnel integrals are barred from its contract; they appear only at execution lowering.
- Discipline test for the whole IR: *anything the planner needs must be closed-form in κ; anything that needs Fresnel is on the wrong side of the seam.*

### 1.2 The per-segment read-contract (the `CurvatureProfile` trait)

Committed trait surface (Amelia, settled round 2). Position/Fresnel is deliberately **not** on it — enforced by module privacy, not convention:

```rust
trait CurvatureProfile {
    fn s_len(&self) -> f64;                    // arc length, design space, > 0 (asserted)
    fn kappa(&self, s: f64) -> f64;            // s ∈ [0, s_len], closed-form
    fn dkappa_ds(&self, s: f64) -> f64;        // closed-form; v1: implemented but UNCALLED (see §1.4)
    fn kappa_peak(&self) -> (f64, f64);        // (s*, κ_max) extremum, closed-form
    fn kappa_endpoints(&self) -> (f64, f64);   // (κ(0), κ(s_len)) for the join check
}
```

Per-variant guarantees — the σ = dκ/ds discipline *is* the type discipline:

| Variant | σ = dκ/ds | κ | κ_peak | Binding cap |
|---|---|---|---|---|
| **Line** | 0 | ≡ 0 | (·, 0) | none — planner must **skip** `√(a/κ)` (else divide-by-zero); only `F`, `max_velocity`, `v_flow` bind |
| **Arc** | 0 | ≡ const ≠ 0 | (·, 1/r) | single cap `√(a/|κ|)`, no apex search |
| **Clothoid** | const ≠ 0 | linear | **endpoint** extremum `max(|κ(0)|, |κ(L)|)` — closed-form, no search | pointwise `√(a/κ(s))`, apex (high-κ endpoint) binds |
| **Follower** | — | n/a | n/a | no κ cap; carries `ratio` for `v_flow=Q_max/(w·h)` and the `(v+τ·a)` PA guard |

Key consequence of dropping G5: because clothoid κ is **linear**, `kappa_peak` is a closed-form endpoint expression, not a root-find. A cubic-Bézier's κ(s) had no such form and would have forced a numerical κ-extremum search — the very thing the alphabet eliminates.

### 1.3 The position-via-Fresnel contract

The planner is handed `(κ(0), κ(L), σ, L)`; lowering is handed *the same record* plus the design-space anchor (start pose/heading) and computes position by Fresnel. The single cross-seam invariant:

- **L-consistency.** The Fresnel evaluation must reproduce exactly the `L` the planner timed against. A path of length `L ± ε` makes the timing a lie. Validate `σ == (κ(L) − κ(0)) / L` at the **fitter→planner seam** and **fail loud** on violation — do not let the planner defend against a degenerate IR.

### 1.4 Junction-discontinuity bounds & the phase-qualified σ contract

The continuity ladder the IR must carry, per junction (not per segment):

- `κ⁻` (exit curvature in) and `κ⁺` (entry curvature out); the **G1 tangent-continuity flag**, guaranteed everywhere except move-starts.
- If `κ⁻ ≠ κ⁺`: residual-step cap `√(a / max(|κ⁻|, |κ⁺|))` — bounded, closed-form, **graceful** (not fail-loud).
- Carry `σ⁻, σ⁺` (dκ/ds either side) on the junction record **from day one**, even though the v1 sweep ignores them.

**The "reads only κ" resolution.** The SPEC line *"velocity planning needs only κ"* is not stale-wrong — it **lost its phase qualifier**. It is true for the jerk-unaware sweep (SPEC build step 5) and false for the jerk-aware lookahead (step 6+, which reads σ for the S-curve). The owner's "build jerk-unaware first, jerk-aware second" instinct **is** the existing step-5→step-6 ladder. Therefore:

- `dκ/ds` lands on `CurvatureProfile` at **v1**, implemented and populated-and-finite, but **uncalled** until step 6. This makes the v1→v2 upgrade **purely additive** — a new *consumer*, zero trait churn, zero re-touch of existing IR→IR seams (proven by AC-TJ-3 below). Adding the method at step 6 instead would couple the jerk algorithm to a trait-surface migration across every implementor. Rejected.

---

## 2. Alphabet sufficiency — verdict and the G2-vs-G3 finding

### 2.1 Sufficient, and G5-drop is causally why

With G5 gone, inputs are G0/G1/G2/G3 — lines and circular arcs only. Every input primitive is *already* a Line or Arc; the fitter only *adds* Clothoids to blend corners. Piecewise-linear κ(s) ≡ the alphabet (slope-0-value-0 = Line, slope-0 = Arc, slope≠0 = Clothoid), and the set is genuinely closed: any continuous PWL κ is a finite concatenation of the three. **No fourth letter is needed for any geometry a slicer emits.**

The sufficiency is *coupled* to the G5 drop, not coincidental: cubic Bézier was the only input demanding κ *cubic in s*, which Arc+Clothoid cannot represent and which would have forced a fourth variant or a sampled fallback.

### 2.2 The real finding: PWL κ is the alphabet-*projected* MVC, not MVC itself (Dr. Quinn)

The architecture's phrasing "PWL κ *is* the Minimum-Variation-Curve" is imprecise. The free Euler-Lagrange minimiser of `∫(dκ/ds)²` is κ piecewise-**cubic** (G3, dκ/ds continuous). PWL κ is the minimiser **constrained to the {Line|Arc|Clothoid} alphabet** — i.e. the *projection* of the optimum onto a closed primitive set. The projection residual is exactly the **dκ/ds steps** at blend seams (line→clothoid, clothoid→arc). Honest restatement: *the alphabet-constrained minimiser of `∫(dκ/ds)²`.*

### 2.3 Two independent jerk channels — and which one each requirement lives in

A trajectory `r(s(t))` splits in the Frenet frame:

- **Tangential jerk** ∝ `s⃛` — owned **100%** by the time-law `s(t)`. Made bounded/continuous by building `s(t)` as a C² seven-segment S-curve. *Geometry cannot touch it.*
- **Lateral jerk** ∝ `(dκ/ds)·ṡ³ + 3κṡs̈` — owned by `dκ/ds`. A dκ/ds **step** is a bounded lateral-jerk **step** (a discontinuity, not an impulse).

The substitution to avoid: **C1 tangential velocity does *not* buy continuous lateral jerk** — the discontinuity lives in `dκ/ds`, which `s(t)`'s smoothness multiplies but cannot launder.

### 2.4 Extrusion makes **zero** demand on the geometry

The owner's reframe, confirmed and sharpened: extrusion-consistency does **not** demand G2 or G3. The real failure ("post-PA acceleration too violent on the extruder") is purely tangential. Pressure advance commands `E_pa = E_nominal + τ·v_extrude`; the extruder *acceleration* therefore carries `τ·(da/dt) = τ·(tangential jerk) = τ·s⃛`. Bound `s⃛` in the S-curve and the PA-augmented extruder transient is bounded directly. **No κ, no dκ/ds, no geometric continuity anywhere in that expression.**

⇒ Extrusion-consistency collapses entirely into the tangential `s(t)` channel and **drops out of the alphabet-sufficiency debate**. The extruder bound is *derived*, not a new IR field: `|τ · ratio · j_tangential| ≤ a_extruder_transient_max`, enforced in the step-6 jerk stage; the binding limit is `j_t ≤ min(j_max, a_extruder_transient_max / (τ · max_ratio))`. `FollowerDemand { axis_index, ratio }` is unchanged.

### 2.5 So is the alphabet a leaky abstraction? Only for surface finish — and that's measurable, not structural

With extrusion severed, the alphabet-sufficiency question reduces to a **pure surface-finish question**: do the bounded lateral-jerk *steps* at blend seams excite visible ringing? Decisive facts:

- The goal is **bounded** lateral jerk, **not zero / not G3** — the input shaper owns resonance. We do not pay trajectory time for G3 on speculation (the project's "never ship a measurably slower trajectory" line cuts *both* ways).
- A shaper is a **notch** filter; a dκ/ds step is **broadband** (~1/f). The one thing a notch cannot null is broadband energy in its **inter-notch** bands. So the *only* residual case where the alphabet could leak in practice: out-of-notch jerk-step energy above the surface-finish floor.
- **This is an instrument question, not a design decision:** measure post-shaper lateral-accel spectral energy at blend seams, scored in the inter-notch bands, on representative slicer output. Below the floor → ship the alphabet with a clear conscience. Above it → and *only* then — the project's non-negotiable obliges upgrading to a piecewise-cubic-κ primitive (G3 clothoid-spline / Pythagorean-hodograph quintic). It does **not** block v1.

---

## 3. "The grid" — clarification (no planning grid exists)

A terminology collision surfaced and was resolved. Three distinct things were being called "grid":

1. **The OLD per-segment 1024-point arc-length TABLE** — a numerical inversion of a Bézier's position→arc-length map. **DELETED**, structurally: clothoid κ is closed-form, so κ comes from an equation, not a lookup.
2. **The classical-TOPP arc-length discretization grid** — uniform sampling of `s` to *find* curvature peaks. **NOT OURS.** Because clothoid κ is linear, the peak is a closed-form endpoint; junction caps are closed-form. The sweep is therefore **node-based** (one node per junction + per clothoid apex, analytic constant-accel kinematics between), not a sampled grid. Round-1 "grid" language was a reflexive textbook-TOPP import and was retracted by both Amelia and Murat.
3. **The execution fixed-rate evaluator** (MCU step rate) — a **time** grid, the step generator. Always present, intended, unrelated to planning. *(The orchestrator's "step compression" reference was a mainline-Klipper import with no basis in this codebase — retracted. The execution-side sampling question is independent of the IR contract and out of scope for this research.)*

**Does jerk need a grid? No.** The S-curve is the closed-form seven-segment Ruckig solution — analytic breakpoints, not samples. The *one* residual numerical integration in the whole planner is the step-7 limit-riding "1D ODE per clothoid" — local, per-segment, adaptive; categorically not the deleted global table.

**The round-1 "sub-grid √(a/κ) collapse" guard dissolves** into: a **node-coverage invariant** (every junction/apex *is* a node; no κ-step lives mid-segment — fail-loud) + an **analytic-vs-numeric κ_peak equivalence** test (the canary if a non-linear-κ segment ever sneaks in). Correctness comes from "every extremum is a node," not from "cells are fine enough."

---

## 4. Risk ranking — what corrupts a committed (already-streamed) move (Murat)

1. **R1 — κ understates true peak curvature.** The only catastrophic one. `v ≤ √(a/κ)` with κ too low commits an unattainable velocity; no downstream check (already on the wire). P(silent) is elevated because κ (analytic, planner) and position (Fresnel, lowering) are **two code paths for the same curve** — the textbook silent-disagreement setup. **The must-write test:** cross-representation equivalence — sample the lowered Fresnel curve, numerically estimate κ, assert it matches the analytic κ the planner read.
2. **R2 — L disagrees** between planner and lowering (see §1.3, the L-consistency fail-loud).
3. **R3 — junction κ-continuity assumed but false** (node-coverage invariant, §3).
4. **R4 — follower/extrusion desync** from the tangential parameterization (a flow defect, no kinematic check catches it).
5. **R5 — accel-limit semantics** (tangential vs centripetal) confused at the seam — newtypes mitigate.

**Graceful-degradation trap:** a residual κ-step → speed cap looks identical to a legitimately sharp corner, so a *buggy* κ prints as a *plausible* cap. Mitigation: tag the step provenance — `KappaStep { declared: bool }`; graceful for **declared** discontinuities (move-start, declared G1 corner), **fail-loud** for undeclared ones.

**Offline-testability:** G2 (κ continuity) and C1 (velocity continuity) are properties of data structures + a deterministic sweep — fully provable on a laptop to machine precision, no printer. The *only* thing the bench is irreplaceable for is calibrating the constants (`a`, `τ` up/down ≈ 37/10 ms) — a calibration question, not a contract question.

---

## 5. Acceptance-test deltas produced by this pass

**v1 (build step 5):**
- AC-CP-1: `s_len() > 0` asserted at construction; `≤ 0` fails loudly.
- AC-CP-2 (property): `dkappa_ds(s)` matches central-difference of `kappa(s)` for every implementor — **gated from v1 though uncalled**, so the hook can't rot.
- AC-CP-3: `kappa_endpoints() == (kappa(0), kappa(s_len))` (closed-form consistency).
- AC-CP-4: `kappa_peak()` ≥ `kappa` at sampled interior points (extremum dominance).
- AC-NODE-1: node-coverage invariant — node count == junctions + apexes; no mid-segment κ-step (runtime assert, fail-loud).
- AC-SEAM-1: cross-representation κ equivalence — analytic κ (planner) vs κ estimated from Fresnel position (lowering), within tol. *(The R1 killer.)*
- AC-SEAM-2: L-consistency — `σ == (κ(L)−κ(0))/L` at fitter→planner; fail-loud otherwise.

**v2 (build step 6):**
- AC-TJ-1: `|j_t(t)| ≤ j_max` everywhere.
- AC-TJ-2 (property): with PA on and `ratio>0`, peak post-PA extruder transient `≤ a_extruder_transient_max`, sweeping `τ`, `ratio`, corner κ.
- AC-TJ-3 (additive-proof): v1 IR→IR seam types byte-identical and `CurvatureProfile` surface unchanged pre/post step-6 merge — *proves the "leave the σ hook from day one" call.*

---

## 6. Conditional policies & open items (the only things between "slogan" and "enforceable contract")

Sufficient **provided** the following are written as explicit policy (Winston):

1. **Zero-length spatial segments collapsed at the fitter**; the planner assumes `L > 0`. Zero-length lives only in the follower channel (pure retraction).
2. **Near-zero-κ Arcs snap to Line** by a fitter quantization rule (a numerical κ_floor decision, not a type decision) — avoids a denormal-hazard `√(a/κ)`.
3. **Helical G2/G3 decomposes into a planar Arc + a linear Z-follower** (Z as a follower channel like the extruder), keeping the planner's κ a *plane* curvature. **← Still OPEN; needs an explicit yes.** This is the one place the planar-κ assumption could genuinely leak (a true space curve has torsion). If anyone models a 3D space-curve segment, the alphabet does *not* cover it.

Open, non-blocking:
- **Surface-finish G3 measurement** (§2.5) — the inter-notch lateral-spectral-energy reading. Decides whether the alphabet ever needs a piecewise-cubic-κ upgrade. Instrument question; does not block v1.
- **Clothoid inflection split** — does the fitter guarantee no sign-change of κ inside one clothoid (so κ_peak stays an endpoint)? Assume fitter; not yet written.
- **G2-tolerance numeric value** — "κ continuous within tolerance" has no number yet; it is exactly the node-coverage assertion threshold.

---

## 7. Citations (underwriting the load-bearing claims)

- **Forward-backward sweep optimality / node-based caps:** Dong & Stori, "A Generalized Time-Optimal Bidirectional Scan Algorithm," ASME JDSMC 2006 (single-pass global optimality for accel+curvature). Pham, T-RO 2014 (switching-point taxonomy — tangent/discontinuity/zero-inertia; zero-inertia structurally absent on constant-mass Cartesian/CoreXY).
- **Fitter objective / G3 minimiser:** Moreton & Séquin, SIGGRAPH 1992 (`∫(dκ/ds)²` Minimum Variation Curve; free minimiser is piecewise-cubic). Bertolazzi & Frego, G1/G2 clothoid fitting (2015/2018), `github.com/ebertolazzi/Clothoids` (O(n) G2 clothoid splines).
- **Closed-form S-curve / no jerk grid:** Berscheid & Kröger, "Ruckig," RSS 2021 (closed-form 1D jerk-limited S-curve).
- **Decoupling gap (unmeasured for FFF):** Anand et al. 2024 (jerk limits ~3–5% over jerk-free).
- **Pressure advance (extrusion = tangential):** Habib et al., RPJ 2019 (first-order PA, τ≈45–90 ms); Klipper PA-smoothing #4442 + dmbutyugin post-shaper-follower thread.
- **Curvature speed heuristic the fitter removes:** Klipper junction-deviation (Sonny Jeon 2011) and faceted-arc throttling, Klipper #4228.

---

## Synthesis

The velocity planner's contract on the IR is small, closed-form, and lives entirely in κ-space: `(L, κ(0), κ(L), σ)` per segment, `(κ⁻, κ⁺, σ⁻, σ⁺, G1-flag)` per junction, with L-consistency as the one fail-loud cross-seam invariant and position/Fresnel barred to the execution side. The `{Line | Arc | Clothoid}` alphabet is **sufficient** for every geometry a slicer emits, and dropping G5 is the reason `kappa_peak` is closed-form rather than a root-find. The historically-fuzzy parts all resolved in the direction of *simplification*: extrusion-consistency asks nothing of the geometry (it is a tangential-jerk bound in `s(t)`), the "reads only κ" contradiction was a missing phase-qualifier (σ joins the contract as a day-one unwired hook), and there is **no planning grid** — the sweep is node-based on closed-form caps. The single residual that could ever force a richer primitive is surface-finish ringing from bounded lateral-jerk steps leaking into the shaper's inter-notch bands — and that is a measurement on real output, not a design commitment, and it does not block v1. The only contract gap that is genuinely open is the **helical G2/G3 → planar-Arc + linear-Z-follower** decomposition, which must be stated explicitly to keep the planner's κ a plane curvature.
