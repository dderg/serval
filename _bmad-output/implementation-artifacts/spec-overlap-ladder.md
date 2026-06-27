---
title: 'Overlap ladder — G2 resolution of arc↔arc boundaries and clustered/short-runway corners'
type: 'feature'
created: '2026-06-26'
status: 'in-development'
baseline_commit: 'f0a4b534357e57b2515eda89949148d5f7c959d5'
context:
  - '{project-root}/_bmad-output/project-context.md'
  - '{project-root}/_bmad-output/brainstorming/brainstorming-session-2026-06-26-004207.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-causal-unified-fitter.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The fitter leaves every arc↔arc junction as a **raw curvature step**. The driver eases an arc into a neighbour only when that neighbour is not itself an arc (`causal.rs:179-184`, the `!occupied[a±1]` gate), never blends arc↔arc corners (`fit` skips the blend at any `run_boundary`), and abandons them via `arc_boundary_unblend` → `UnblendReason::ArcIncident` (`causal.rs:277`, the `Segment::Line else ArcIncident` arm). On `fillet.gcode` two opposite-turning arcs (segments 15→16) meet at a bare cusp — the visible sharp apex — violating the no-curvature-step-under-motion mandate. The restriction is inherited verbatim from the old two-stage fitter, so the cutover preserved it. Noise also fragments one curve into adjacent same-sign arcs that should be a single arc or a smooth κ-ramp.

**Approach:** A new **overlap** pass over the detected runs with one binary policy — **blend everything that is not collinear and not tangent.** Classify each junction by continuity and emit the transition that makes it G2:
- **collinear**, or **tangent with continuous κ** → already G2, left untouched.
- **tangent but κ-discontinuous** (line→arc, or arc→arc same direction / different radius) → an **arc→clothoid→arc κ-ramp** (one clothoid ramping κ across the junction).
- **not tangent** (a real corner — the apex, line corners, arc corners) → an **arc→biclothoid-corner→arc** blend, with the biclothoid generalized to nonzero entry/exit curvature, clamped to the runway (asymmetric `L_in≠L_out` so each side uses its own runway).

When adjacent corners are so close their blends cannot physically coexist (overlapping runways), the **fallback** is to merge them into ONE biclothoid across the cluster (sharing one δ budget, arc-aware decimation), never crossing an inflection. The terminal rung is always a tiny high-κ biclothoid — **never** a drop to G1. This is the deferred Spec 2; per the brainstorming it is "where the actual novel work is."

## Boundaries & Constraints

**Always:**
- The blend policy is binary: a junction that is **neither collinear nor tangent** is ALWAYS blended (corner biclothoid); a **tangent junction with a κ-step** always gets a κ-ramp; only **collinear** and **already-G2 tangent** junctions are left untouched. No junction that is not collinear-or-tangent is ever left raw.
- Every emitted element handoff is G2 by construction: signed κ is continuous (|Δκ| ≤ 1e-9) at every boundary the pass resolves. **The ONE and only permitted κ-step is a 180° line-line reversal** (collinear, opposite direction) — it comes to a full stop at planner rest (zero velocity ⇒ zero centripetal-accel jump) and is reported, not silent. Every other junction — every corner and every sharp tip, however small the runway — is blended G2; there is no other at-rest fallback.
- Stay in the position band δ at every point, measured against the FINAL fitted curve — ONE shared budget across decimation + transition + corner (no decimate-then-fillet 2δ double-count).
- Reuse proven kernels (`Clothoid::try_new`, `build_spiral`, `joint_refit`, `biclothoid::canonical`, Fresnel via `path`). Generalize the biclothoid to nonzero boundary κ rather than re-deriving clothoid math.
- Curvature is inherited end-to-end: a transition's entry κ == predecessor arc's exit κ, exit κ == successor arc's entry κ (signed). Settle the Arc(unsigned)/Clothoid(signed) convention first so G2 checks compare SIGNED κ.
- Clamp to the available runway (asymmetric per side); δ shrinks as L shrinks so an isolated corner is always representable — never refused, never G1.
- Drop-in: `fit_chain`, `fit_chain_with_head_restore`, `fit_corners`, `FitOutcome`/`FitReport`/`FitError` keep their shapes; `motion-engine` callers compile unchanged. The ladder is internal to the geometry crate.
- Fail loudly: non-finite transition geometry → `FitError` with the source `line_no`.

**Ask First:**
- Accepting regenerated baselines — `fillet` (and any case with adjacent arcs) legitimately changes; new baselines are human-accepted via the snapshot web UI, never auto-overwritten.
- Any new `[arc_fit]` config knob (e.g. a merge aggressiveness); default behavior must need none.

**Never:**
- Never leave any junction raw except the 180° line-line reversal; never emit a curvature step under motion. A sharp corner is clamped to a runway biclothoid, never dropped to a stop.
- Never merge across an inflection (curvature sign change) — split the cluster at the inflection, merge only within a monotone-turning run.
- Never exceed δ to force a merge — robustness comes from the shared-budget fit, not a looser tolerance.
- No comments; unit tests in `overlap/tests.rs`.

## I/O & Edge-Case Matrix

| Scenario | Input | Expected output | Error |
|----------|-------|-----------------|-------|
| Not tangent — opposite-sign cusp (the apex) | `fillet.gcode` seg 15↔16 | blended: arc→clothoid→biclothoid→clothoid→arc; κ continuous, peak finite, ≤δ | N/A |
| Tangent, κ-step (diff radius) | two abutting arcs, κ1≠κ2, θ≈0 | κ-ramp: single arc→clothoid→arc, ≤δ | N/A |
| Tangent, κ continuous (cocircular) | adjacent cocircular arcs | left as-is — already G2, NOT merged | N/A |
| Collinear | line→line, θ≈0, κ=0 | left as-is (one line) | N/A |
| Tight corner cluster, overlapping runways | ≥3 corners within √(24Rδ) | fallback: ONE biclothoid across the cluster, ≤δ | N/A |
| Cluster spanning an inflection | +κ run then −κ run | split at the inflection; merge only within each monotone run | N/A |
| Isolated sharp tip, tiny runway | short flanking legs | blended: runway-clamped biclothoid, κ_peak=θ/L high but continuous (NOT a stop) | N/A |
| 180° line-line reversal | collinear, opposite direction (hairpin retrace) | full stop at rest — the ONLY at-rest κ-step; reported | N/A |
| Non-finite transition geometry | NaN/inf | — | `FitError` with `line_no` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/fitter/overlap.rs` — NEW. `pub(super) fn resolve(runs: Vec<Run>, moves, config, tol) -> Result<Vec<RunOrTransition>, FitError>`: classify each adjacent-run boundary (and short-line-bridged cluster), choose a ladder rung, build the transition. Houses the ladder, the inflection split, and the cluster grouping.
- `rust/geometry/src/fitter/overlap/tests.rs` — NEW. Unit-test each rung + the inflection guard + runway clamp.
- `rust/geometry/src/fitter/biclothoid.rs` — generalize `solve`/`canonical` to nonzero entry/exit curvature (`κ_in`,`κ_out`): `half1` from κ_in→κ_peak, `half2` from κ_peak→κ_out, asymmetric lengths; keep the line↔line case (κ_in=κ_out=0) bit-identical.
- `rust/geometry/src/fitter/kernels.rs` — add an arc→arc κ-ramp transition (`bridge_arcs`/extend `build_spiral` to ramp κ between two nonzero curvatures); let `neighbor`/`joint_refit` accept an arc-backed neighbour (curvature, not just a line tangent); add the shared-budget arc-aware decimation helper.
- `rust/geometry/src/fitter/causal.rs` — replace the `!occupied` independent-easing block (`chain_runs:179-185`) and the emit loop's run-boundary handling (`fit:91-109`) to call `overlap::resolve` and emit transitions between runs; delete the `ArcIncident`-for-arc-neighbour path; keep `RunRole` for line↔run boundaries.
- `rust/geometry/src/path/` — settle the signed-κ convention so `Arc::kappa_endpoints` and `Clothoid` agree in sign; update `worst_kappa_jump`/`max_kappa_jump` (tests) to compare signed κ.
- `snapshots/{cases,baselines}/arc_fit/` — `fillet` re-baselines; add `arc_to_arc.gcode` (tangent diff-radius) + `corner_cluster.gcode` (overlapping runways) cases. Human-gated accept.

## Tasks & Acceptance

**Execution (dependency order):**
- [ ] Settle signed-κ convention in `path` (Arc vs Clothoid) and make G2 checks compare signed κ; fix fallout in existing tests/snapshots.
- [ ] `biclothoid.rs` — generalize `solve`/`canonical` to (κ_in, κ_out, θ, δ, budget_in, budget_out); line↔line case unchanged; unit-test symmetric/asymmetric/nonzero-boundary.
- [ ] `kernels.rs` — arc→clothoid→arc κ-ramp bridge (tangent, differing radius) within δ; arc-backed `Neighbor`; arc-aware decimation sharing ONE δ budget (used by the cluster fallback).
- [ ] `overlap.rs` — junction classification (collinear / tangent-κ-continuous / tangent-κ-step / not-tangent) and the binary blend policy: leave collinear & already-G2; κ-ramp tangent-κ-step; corner-blend everything not tangent (asymmetric runway clamp). Cluster fallback (one biclothoid across overlapping-runway corners) + inflection split. Returns the resolved run/transition stream.
- [ ] `causal.rs` — replace `!occupied` easing + run-boundary emit with `overlap::resolve`; replace `ArcIncident` reporting with a single 180°-reversal full-stop report (every other junction blends); preserve line↔run easing and head-restore.
- [ ] `overlap/tests.rs` — every I/O-matrix row; assert κ continuity (signed, ≤1e-9), in-band ≤δ, never-G1, no-merge-across-inflection.
- [ ] snapshots — re-baseline `fillet`; add `arc_to_arc`/`corner_cluster`; human-accept.
- [ ] proptest — extend `fit_proptest.rs`: random multi-arc polylines assert SIGNED κ continuity ≤1e-9 at every non-rest handoff AND in-band ≤1.5δ (both hearts).

**Acceptance Criteria:**
- Given `fillet.gcode`, when fitted, then segments 15↔16 are joined by a continuous-κ transition (no bare cusp); peak κ is finite; max deviation ≤ δ; no `UnblendReason::ArcIncident` remains.
- Given any two adjacent arc runs, when resolved, then signed κ at the handoff is continuous (≤1e-9) — always (arc↔arc is never the 180° reversal exception).
- Given any junction in the fitted stream, when it is not a 180° collinear-opposite line-line reversal, then it is G2 (signed κ continuous); only that reversal is a reported full-stop κ-step.
- Given a corner cluster whose runways overlap, when fitted, then one biclothoid spans it and the path stays ≤ δ; given a cluster crossing an inflection, then it is split at the sign change.
- Given `motion-engine`, when built, then callers compile unchanged against `fit_chain*`/`FitOutcome`.
- Given `./scripts/ci.sh quick` (+ `py` for harness/klippy), when run, then fully green (snapshots re-baselined and human-accepted).

## Design Notes

The decision is by continuity, not a cleverness cascade: classify the junction, then act. Collinear and tangent-with-continuous-κ are the only leave-alone cases — a cocircular arc pair is already G2, so it stays two arcs (no merge; merging would be cosmetic and risk pushing the refit out of band). The corner primitive is one generalized biclothoid κ_in→κ_peak→κ_out: each half is a `Clothoid` with its own σ and length, apex at κ_peak; κ_peak set by the shared trim/δ relation (`canonical`) and clamped per-side to the runway, so `L_in≠L_out` falls out naturally and the symmetric κ_in=κ_out=0 case reduces to today's `solve`. A tangent κ-step needs no corner — just one κ-ramp clothoid (σ=(κ2−κ1)/L). The cluster fallback fires ONLY when independent blends overlap; it merges into one biclothoid across the cluster and shares one δ budget (decimation + blend draw from the same δ, never δ each — no decimate-then-fillet stacking). The terminal rung never degrades below a tiny biclothoid.

Signed-κ first: today `Arc::kappa` is unsigned (+1/R) while `Clothoid::kappa` is signed, so `max_kappa_jump` compares |κ| and a +κ→−κ cusp reads as zero jump — which would mask the very apex this spec fixes. The convention must be settled before the G2 assertions mean anything.

## Verification

- `cd rust && cargo nextest run -p geometry` — all green incl. `overlap/tests.rs`.
- `cd rust && cargo nextest run -p geometry -E 'binary(fit_proptest)'` — signed-κ continuity + in-band hold over random multi-arc inputs.
- `cd rust && cargo build -p motion-engine` — callers compile unchanged.
- `./scripts/ci.sh snapshot` then `snapshots/snapshot-tests.sh` — review `fillet` (rounded apex) + new cases; human-accept.
- `./scripts/ci.sh quick` + `./scripts/ci.sh py` — fully green.

**Manual check:** render `fillet.gcode` (snapshot web UI) and confirm the apex is rounded and every arc flows into its neighbour with no visible cusp.

## Suggested Review Order

1. Signed-κ convention + G2 metric — the measurement must be honest first. `path/`, metric tests.
2. Generalized biclothoid (nonzero boundary κ). `biclothoid.rs`
3. The ladder + classification + inflection guard. `overlap.rs`
4. Driver wiring (replaces `!occupied` + `ArcIncident`). `causal.rs`
5. Snapshots (`fillet` apex rounded) + proptest. `snapshots/`, `fit_proptest.rs`
