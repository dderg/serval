---
title: 'Motion-14: jerk-limit the cruise on/off-ramp — a_t steps a_max→0 at max_velocity'
type: 'feature'
created: '2026-06-21'
status: 'in-progress'
baseline_commit: '24dd8679d780f15095a0529666ac25716c6baca4'
context:
  - '{project-root}/CLAUDE.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-12-tangential-jerk-c2-continuity.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-7-limit-riding.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/cruise-onset-jerk-limit-handoff.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/cruise-onset-repro/'
---

> **Canonical contract.** This SPEC is the complete, preservation-validated contract for what to build, test, and validate. The handoff in `context:` (`investigations/cruise-onset-jerk-limit-handoff.md`) is traceability only — consult it for the full evidence narrative and runnable repro, not for the contract.

> **Lineage (2026-06-21):** Motion-12 (T3, PR #88, merged) made tangential acceleration C2 *within a run* by carrying `(v,a)` across junctions and bridging forward/backward **sign-flip** crossovers (apex `+→−`, valley `−→+`). It did **not** govern the transition onto the flat velocity ceiling. This spec is the next, distinct piece: the cruise on/off-ramp `+a → 0` / `0 → −a` step. It is **tangential** and fixable; it is **not** the lateral/G3 clothoid seam (that stays a deliberate Non-Goal — see Motion-12).

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The velocity planner jerk-limits the **start** of acceleration (jerk-up from rest) but **not the end** of it. When `v` reaches `max_velocity` and the move goes to cruise, tangential acceleration `a_t` **steps discontinuously from `a_max` (or whatever accel it had) to `0`** — an unbounded tangential jerk at every cruise entry. The mirror happens leaving cruise into a decel (`0 → −a_max`). This is a **C1 violation on the flat velocity ceiling**, distinct from the mid-run sign-flip crossovers Motion-12 T3 dissolved. It hits **every move that reaches `max_velocity`**, not just corners — which is why the `demo4` fixture (peaks `v≈134 < 150`, sub-cruise) never exercised it and T3 shipped without catching it. The Neptune `test2.gcode` fixture exposes it (reaches cruise coming out of each corner). Verified on the live planner (`pipeline_snapshot`, `v100 a1000 j4000`): a single trapezoidal straight steps `a_t: 894 → 0` at the cruise onset (`s≈7.45`); a triangular straight peaking *below* cruise does **not** step (`max|Δa_t|≈15`, discretization noise) — so the failure is the **ceiling touch**, not "reaching cruise needs new machinery."

**Root cause — the bridge already does this, the detector just never invokes it at the ceiling.** The forward/backward crossover jerk-bridge already rolls `a_t` smoothly through a target accel: that is exactly how "acceleration ends mid-segment" is handled today — at a sub-cruise **apex** `build_run_bridge` shoots a constant-jerk arc that carries `a_t` from `+a` through `0` to `−a`, and the result is C2 (the triangular-straight evidence above). But `reconstruct_flat` (`disk.rs:616-622`) only fires that bridge on a strict accel **sign flip** (`aa>0 && ab<0` apex, `aa<0 && ab>0` valley). When the forward accel ends by **touching the flat ceiling** instead of crossing the backward envelope, the base-sample transition is `+a → 0` (cruise entry) / `0 → −a` (cruise exit) — a touch of the `a=0` cruise rail, *not* a sign flip — so the detector hits `else { continue }`, no bridge is built, and the `a_max → 0` step survives. The bridge machinery itself already handles a zero-accel endpoint: `shoot`'s `right_a` already returns `0` at the ceiling, so the same arc that rolls through an apex would roll `+a → 0` onto cruise — it is simply never called there.

**Approach:** Extend the crossover detector in `reconstruct_flat` to also fire on a **ceiling touch** — `+a → 0` (cruise entry) and `0 → −a` (cruise exit) — and generalize `build_run_bridge`/`shoot` to accept a bridge whose far endpoint sits on the `a=0` cruise rail (one-sided-zero target). This **reuses the existing closed-form constant-jerk arc + 1-D splice root**; it is **not** a new scurve primitive (the roll-off is the *same shape* the apex bridge already produces — Motion-14 just invokes it for one more transition class). The arc dips a hair under the flat ceiling as `a_t` eases to 0 (cruise reached slightly later); the existing `v ≤ min(fwd,bwd)` envelope check inside `build_run_bridge` already bounds this — it is the one legitimate, tiny C2 cost, not a regression (CLAUDE.md throughput-SOTA: C1 was reporting an unrealizable infinite-jerk step; the rolled-off profile is the best *feasible* one).

This supersedes the handoff's "first option" (extend the scurve primitive to add a jerk-down phase). The scurve `SevenSeg` genuinely has no jerk-down phase (`accel_at` returns `accel_max` flat after jerk-up), but the cruise roll-off is **not** produced by scurve — it is produced by the bridge — so the fix lives in `reconstruct_flat`/`build_run_bridge`, not `scurve.rs`.

## Boundaries & Constraints

**Always:** Reuse the existing `build_run_bridge`/`shoot` constant-jerk arc — extend its detector and endpoint handling, do **not** add a new scurve jerk-down phase or a parallel bridge. Keep the node-based forward-backward sweep **O(N) two-pass with closed-form per-edge work** — no SOCP/QP/grid/iterative inner solver. Keep XY jerk a single global scalar (`[printer] max_jerk`). Per-sample `a_t` stays **analytic** (binding-rail closed form / `accel_at` / bridge arc) — **never finite-differenced** (the prior T3 attempt exploded on `v·Δv/Δs` over sub-µm seam `Δs`; the repro/viz must read the planner's analytic `kin_a_t` from `pipeline_snapshot`, not `np.gradient` of `kin_v`). Keep `(v,a)=(0,0)` pinned at true rest anchors with the existing fail-loud `RestAnchorAccel`. Preserve the Motion-12 T3 sign-flip apex/valley bridge behavior unchanged (this is an *additional* transition class, not a rewrite). Reuse the existing emit backend unchanged.

**Ask First:** Any fallback to a *genuinely iterative / multivariable* inner solver (an SOCP/QP/grid solve) in the new roll-off or ceiling bridge — escalate, do not silently relax. (A bounded 1-D splice root on monotone envelopes is within the no-inner-solver boundary, as in Motion-12 R1.) Loosening the cruise-onset continuity acceptance to absorb a real residual step.

**Never:** Re-introduce the sampled Consolini-Locatelli coupled-jerk SOCP. Add a planner-level **lateral**-jerk constraint or cap (the fitter's clothoid shape owns lateral jerk — Motion-12 Non-Goal). Add per-axis / per-`[limit]`-section jerk. Pad or advance to hide the step instead of jerk-limiting it (fail loud on out-of-contract state, never saturate-and-continue).

</frozen-after-approval>

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Cruise entry | `v → max_velocity` on a forward ramp | `a_t` **eases to 0** as `v → v_max` (jerk-down), no `a_max → 0` step | N/A |
| Cruise exit | cruise (`a=0`) → decel into the next limit | `a_t` **eases from 0** (jerk-up off the ceiling), no `0 → −a_max` step | N/A |
| Collinear junction at the cruise onset | two collinear straights, cruise onset placed just before / at / just after the junction (`L1 ∈ {d_accel±1.5, d_accel}`) | roll-off glued to the **cruise onset** (`s = d_accel`), not the junction; junction stays transparent (`a_t` continuous across it) | N/A |
| Low jerk, ceiling before plateau | `max_jerk` low enough that jerk-up hasn't reached `a_max` before the ceiling (`v_max < a_max²/(2j)`) | `a_t` rolls off from its actual value `√(2·j·v_max)`, not from `a_max`; still no step | N/A |
| Sub-cruise velocity peak | move peaks below `max_velocity` (e.g. `demo4`, `134 < 150`) | **unchanged** — never touches the flat ceiling; Motion-12 T3 sign-flip bridge owns this | N/A |
| Mid-run sign-flip crossover | apex `+→−` / valley `−→+` inside a run | **unchanged** — Motion-12 T3 bridge still owns this | N/A |
| Rest anchor with non-zero entry accel | run arrives at a `v=0` anchor with `|a|>ε_a` | **raise** `RestAnchorAccel` (unchanged from T3) | `Err`, no pad |
| Infinite jerk | `max_jerk = ∞` | no roll-off (instantaneous accel is C1-by-definition); skip the new path | N/A |

## Non-Goals

- **Lateral / G3 clothoid-seam jerk.** `j_n = κ'·v³ + 2κ·v·a_t` steps at clothoid↔{line,arc} seams because the clothoid is G2-not-G3; that is the **fitter shape's** responsibility and an explicit Motion-12 Non-Goal. The user has accepted leaving it; a G3 corner-shape fitter is a separate spec (`investigations/clothoid-straight-seam-discontinuity-investigation.md`). Do **not** add a planner-side lateral cap here.
- **Motion-12 T4** (the durable CI feasibility gate + throughput non-regression job). Still owed under Motion-12. This spec's acceptance reuses the same time-domain feasibility check philosophy (recover `a_t` from the emitted trajectory, evaluate `accel_at` at adjacent `SevenSeg` endpoints) but does not deliver the standing CI job.
- **The corner-apex near-stop valley** (curvature-ceiling touch at the biclothoid apex). Clothoids are working fine (user, 2026-06-21); Motion-14 stays on the **flat** velocity ceiling only and does not widen the bridge to the curvature ceiling. See R2.
- **Reworking `scripts/viz_pipeline.py`** to plot the analytic `kin_a_t/kin_j_t` instead of `np.gradient(kin_v)`. Nice-to-have de-aliasing, separate.
- Per-axis / per-section jerk; input shaping; reviving any SOCP/NLP oracle.

## Code Map

- `rust/geometry/src/velocity/disk.rs` — **primary edit site.**
  - `reconstruct_flat` (`:607-637`) — the crossover detector. The `apex` predicate (`:616-622`) requires a strict sign flip (`aa>0 && ab<0` / `aa<0 && ab>0`) and `continue`s on a ceiling touch (`aa>0 && ab≈0`, `aa≈0 && ab<0`). **Extend it to classify and bridge ceiling touches.**
  - `build_run_bridge` (`:532-605`) / `shoot` (`:461-497`) — the constant-jerk arc + 1-D splice root. `shoot`'s `right_a` already returns `0` at the ceiling; generalize the left/right wiring and the `apex` argument so the far endpoint may sit on the `a=0` cruise rail. The `v ≤ min(fwd,bwd)` env check (`:594-601`) already bounds the slight cruise-onset dip.
  - `forward_branch`/`backward_branch` (`:210-257`) — return `a=ceiling_accel(=0 on a straight)` once `v ≈ flat_ceiling`; this is what produces the `a_max → 0` base-sample step. `ceiling_accel`/`curvature_ceiling_accel` (`:191-208`).
- `rust/geometry/src/velocity.rs` — `plan_velocity_warm_start` (run sweep driver); rest-anchor pin / `RestAnchorAccel` (unchanged).
- `rust/geometry/src/velocity/scurve.rs` — `breakpoints`/`accel_at` (the `SevenSeg` is jerk-up-then-hold, `accel_at` flat at `accel_max` after jerk-up). **Read-only context** — confirms the roll-off is *not* a scurve responsibility; do not add a jerk-down phase here.
- `rust/geometry/tests/c2_continuity.rs` — add the cruise-onset cases (single trapezoidal straight + the collinear-straights sweep below).
- `rust/motion-engine/src/viz.rs` — `pipeline_snapshot` exposes the analytic `kin_s/kin_v/kin_a_t/kin_kappa/kin_dkappa_ds` the repro/test consume.

## Tasks & Acceptance

**One PR on the `curvature-profile` feature branch.**

**T1 — red-first cruise-onset test** ✅
- [x] Add the cruise-onset cases to `c2_continuity.rs` (or a new `cruise_onset.rs`): the minimal repro is a **single trapezoidal straight** (`L=40, v100 a1000 j4000`) — assert **no `a_t` step at the cruise onset** (entry *and* exit). Get it **RED** on the committed planner first (it must bite — proves the step is real and the test sees it). → `rust/geometry/tests/cruise_onset.rs::cruise_onset_no_tangential_accel_step`, RED on `24dd8679d`: reports `894.43 → 0.00` at `s=7.4536` (entry) and `0.00 → -894.43` at `s=32.5464` (exit).
- AC-T1a: the new test is RED on `24dd8679d` — reports the `a_t: 894 → 0` step at the cruise onset (`s≈7.45`) on the single trapezoidal straight, and the `0 → −894` step at cruise exit.
- AC-T1b: the test reads the planner's **analytic** `kin_a_t` (`pipeline_snapshot` / `reconstruct_run`), never a finite difference of `kin_v`.

**T2 — bridge the ceiling touch** ✅
- [x] Extend `reconstruct_flat`'s detector to classify ceiling touches (`+a → 0` cruise entry, `0 → −a` cruise exit) as bridgeable transitions alongside the existing apex/valley sign flips, and generalize `build_run_bridge`/`shoot` to a far endpoint on the `a=0` cruise rail. Per-sample `a_t` stays analytic (the bridge arc). Preserve the T3 apex/valley bridge and the `(0,0)` rest-anchor pin unchanged. Implemented in `rust/geometry/src/velocity/disk.rs`:
  - `reconstruct_flat` detector now classifies `CeilingEntry` (`+a→0`, gated on `v ≈ flat_ceiling` so a curvature-apex zero-crossing is never misread) and `CeilingExit` (`0→−a`); both reuse the existing `apex=+1` constant-jerk arc.
  - `build_run_bridge` pins `s_star` to the transition (the `gap` root is degenerate on a flat plateau), and constrains `s_left` to the correct departure side (`CeilingEntry` leaves the rising accel ramp, `CeilingExit` leaves the rail) so the single physical root is selected.
  - New `scan_cross` finds the jerk-direction-consistent crossing, so `shoot` lands on the *smooth* decel past the backward branch's own cruise-exit step instead of latching onto that discontinuity.
  - `interp_flat` makes member-boundary samples read the bridged profile (not base `run_eval`), so a roll-off straddling a collinear junction stays continuous (AC-T2b).
  - Overlap guard: a move too short to sustain cruise (entry+exit roll-offs would overlap — only reachable at low jerk; the base sweep over-reports its peak) falls back to the base profile rather than emitting interleaved arcs. See R4.
  - Tests: `rust/geometry/tests/cruise_onset.rs` (entry/exit, collinear sweep, low-jerk, sub-cruise no-fire, plateau-interior, envelope/rest-anchor, overlap fallback) — all green.
- AC-T2a (**cruise entry/exit continuity** — headline): `|Δa_t|` between adjacent samples across the cruise on-ramp and off-ramp ≤ `ε_a` (no step); `|j_t| ≤ max_jerk·(1+ε)` at cruise entry and exit. `ε` derived from discretization + float epsilon, not arbitrary.
- AC-T2b (**onset tracks the ceiling, not the boundary**): on the collinear-straights sweep (`L1 ∈ {d_accel+1.5, d_accel, d_accel−1.5}`), the roll-off stays glued to `s = d_accel` in all three cases and the junction is transparent (`a_t` continuous across it).
- AC-T2c (**low-jerk roll-off**): with `max_jerk` such that the jerk-up never reaches `a_max` before the ceiling (`v_max < a_max²/(2j)`), `a_t` still rolls off continuously from `√(2·j·v_max)`, no step.
- AC-T2d (**no collateral regression**): triangular/sub-cruise fixtures (`demo4`) and mid-run sign-flip apex/valley crossovers are unchanged — the existing `c2_continuity.rs` cases stay green; `|a_t| ≤ a_max` everywhere; `(v,a)=(0,0)` at both run ends. The bridge must not fire on the *interior* of a long cruise plateau (only its entry/exit edges).
- AC-T2e (**SOTA cost is bounded & legitimate**): cruise is reached no earlier than C1 and the only time cost is the roll-off dip (`Σ` over cruise on/off-ramps); no spurious slowdown away from the ceiling. No O(N) regression — still closed-form, two-pass, no inner solver.

**Acceptance Criteria (spec-level):**
- Given a move that reaches `max_velocity`, when planned, then `a_t` eases to/from `0` at the cruise on/off-ramp (`|Δa_t| ≤ ε_a`, `|j_t| ≤ max_jerk`) — the `a_max → 0` step is gone.
- Given the collinear-straights sweep, when the cruise onset is placed before/at/after a junction, then the roll-off tracks the cruise onset (`s = d_accel`) and the junction stays transparent.
- Given a sub-cruise fixture or a mid-run sign-flip crossover, when planned, then behavior is unchanged (no collateral regression to Motion-12 T3).
- Given `./scripts/ci.sh quick` + `cargo nextest run -p geometry`, when run, then green.

## Design Notes

**Why this is C2-completion, not a new mechanism.** Motion-12 T3 already carries `(v,a)` across junctions and already has the closed-form jerk-bridge machinery. The cruise on/off-ramp is the one transition class T3 left out, because `+a → 0` is not a sign flip. The fix is the smallest possible: teach `reconstruct_flat`'s detector that a ceiling touch is also a bridgeable transition and let `build_run_bridge` land its arc on the `a=0` cruise rail. No new solver, no new state, no new primitive — the same global-scalar-jerk, bang-bang-on-rails architecture, invoked for one more transition class.

**The apex case proves the shape already works (empirical, host-only, `pipeline_snapshot`).** "Acceleration ends mid-segment" and "acceleration ends at cruise" want the *same* jerk-limited roll of `a_t`; the planner already does the first correctly and only fails the second:

| Fixture (`v100 a1000 j4000`) | `v_max` | `max\|Δa_t\|` | Verdict |
|---|---|---|---|
| Triangular straight `L≤14` (apex below cruise) | 33–58 | ~15–20 (discretization) | smooth ✓ |
| Trapezoidal straight `L=16` / `L=40` (reaches cruise) | 100 | **894** (`a_t: 894→0` at `s=7.45`) | steps ✗ |
| Two collinear straights, cruise onset before/at/after junction | 100 | **894**, glued to `s=d_accel`; junction transparent | steps ✗ |

The apex (triangular) result is the existence proof: `build_run_bridge` already produces the correct `+a → 0` roll — it is just never called at the ceiling. The trapezoidal result kills the "single straight works" hypothesis (a single straight that *reaches cruise* steps too); the discriminator is **ceiling-touch vs apex-crossover**, not segment count.

**Evidence the step is tangential, not lateral (from the handoff, reproduced host-only).** Decomposing the planner jerk on `test2`: every large spike is `j_t` (along-path, `~1e10` at the worst); lateral `j_n = κ'·v³` is `~1e5`, negligible. At the worst spike `a_t` literally steps `+1000 → 0` over `~10 nm` of arclength (`dt ≈ 23 ns`) at `v = 100 = max_velocity`. Reproduced at both `max_jerk = 1e6` (Neptune) and `max_jerk = 4000` (steps `894 → 0`), so it is **structural, not the jerk setting**. The collinear-straights sweep proves the step tracks the **cruise onset** (`s = d_accel`), not any move boundary — so the earlier "couldn't apply the jerk limit at the boundary" framing is wrong; it is the ceiling.

**Plain-English recast.** The planner already "feathers the throttle" *up* to speed smoothly (limited jerk on the way up), but at top speed it "slams the throttle to zero" in a single instant instead of feathering back. The machine feels that slam as a jolt on every move that hits its speed limit — most moves on a real print. The fix teaches the planner to feather *down* onto cruising speed too, the same way it feathers up. Cost: you hit top speed a hair later (the feather takes a little room), which is the honest, physically-realizable behavior — the old "instant" was a number the machine could never actually execute.

## Risks & Open Questions

- **R1 — Roll-off shape. [RESOLVED 2026-06-21, empirical + code-grounded]** Decided: **extend `reconstruct_flat`'s detector + `build_run_bridge` to bridge the ceiling touch** (the handoff's "second option"), **not** the scurve primitive. Grounds: (1) the roll-off is produced by the bridge, not scurve — `scurve::accel_at` is read-only context here; (2) the apex (triangular-straight) case is an existence proof that the bridge arc already produces the exact `+a → 0` roll we need, smoothly; (3) `shoot`'s `right_a` already returns `0` at the ceiling, so the delta is the detector predicate (`disk.rs:616-622`) plus the bridge's left/right endpoint wiring for a one-sided-zero target — minimal, and it keeps all "how to cruise" logic in one place (the bridge). Within the no-inner-solver boundary (constant-jerk arc + 1-D splice root). HALT and surface per **Ask First** if the one-sided-zero endpoint tempts a multivariable solver.
- **R2 — Corner-apex near-stop valley (curvature-ceiling touch at the biclothoid apex). [RESOLVED 2026-06-21 — out of scope, do not touch]** This is the curvature-ceiling analogue of the cruise flat-ceiling touch (tangential `a_t` valley crossing 0 at the corner velocity minimum; would mean generalizing the bridge endpoint from `a=0` to the non-zero `curvature_ceiling_accel` rail). **User: clothoids are working fine; no change needed.** Motion-14 stays strictly the **flat** velocity ceiling (`a=0`) cruise on/off-ramp. The bridge endpoint generalization is *not* widened to the curvature ceiling. The separate lateral G3 `dκ/ds` step at the same apex remains the fitter's Non-Goal (unchanged).
- **R4 — Short-move overlap: base sweep over-reports the peak. [DISCOVERED + scoped out 2026-06-21]** A straight the base velocity sweep marks as reaching `flat_ceiling` but shorter than `~2×` the jerk-limited accel distance is **not jerk-feasible at the ceiling**: a jerk-limited up-*and*-down ramp to `v_max` needs more room than the move has, so its true peak is below `v_max`. The entry and exit roll-offs then overlap. T2 detects the overlap (entry arc end past exit arc start) and **leaves the move on the base profile** (keeping its base cruise-touch step) rather than splicing interleaved arcs — no garbage, no regression vs pre-T2. This is **only reachable at low jerk** (the test fixtures use `j4000`); realistic configs (Neptune `j=1e6`) have sub-millimetre roll-offs and never overlap. The real fix is **jerk-aware peak estimation in the forward/backward sweep** (so the reported peak is itself feasible), which is a `velocity.rs` change beyond Motion-14's reconstruction scope. Tracked in `deferred-work.md`. Guarded by `cruise_onset.rs::short_overlap_move_falls_back_to_base_without_garbage`.
- **R3 — Acceptance without the durable gate. [RESOLVED 2026-06-21 — confirmed split]** This spec's continuity assertion lives in `c2_continuity.rs`, recovering `a_t` from the planner's analytic track. The durable CI feasibility/throughput gate — including the anti-circularity check (recover `a_t` from the *emitted time-domain* `ShapedSegment` stream; evaluate `accel_at` at adjacent `SevenSeg` endpoints to defeat aliasing) — stays owed by **Motion-12 T4** and is **not** rebuilt here (user-confirmed). Until T4 lands, the cruise-onset case shares T3's validation posture.

## Verification

**Commands:**
- `cargo nextest run -p geometry` — `c2_continuity` (incl. the new cruise-onset case) green.
- `cargo nextest run -p motion-engine` — viz/probe green.
- `./scripts/ci.sh quick` — ruff/clippy `-D warnings`/fmt/rust tests green before PR.

**Reproduction (host-only, no bench).** Build the PyO3 module: `make -f Makefile.rust motion-engine`. Runnable viz scripts live in `investigations/cruise-onset-repro/` (run from repo root; PNGs to `/tmp/viz_out/`):
- `decompose_jerk.py [gcode]` — splits planner jerk into tangential `j_t` vs lateral `j_n`; proves the spikes are tangential. Defaults to `/tmp/test2.gcode`. → `*_decomposed.png`.
- `cruise_boundary_sweep.py` — the two-collinear-straights sweep; overlays the three cases and prints the step-location table proving the `a_t` step tracks the cruise onset, not the junction. → `cruise_onset_overlay.png`.

Both consume the planner's **analytic** `kin_a_t` from `pipeline_snapshot` — never finite differences of `kin_v`.

**Key numbers / facts:**
- `d_accel` (rest→cruise, `v100 a1000 j4000`) ≈ **7.45 mm**; cruise reached at accel `√(2·j·v_max) ≈ 894 mm/s²` (jerk-up doesn't reach `a_max=1000` because `v_max=100 < a_max²/(2j)=125`).
- Neptune `test2.gcode` config: `max_velocity=100, max_accel=1000, max_jerk=1000000, square_corner_velocity=30`; relative waypoints `(0,0)→(-20,0)→(-30,-40)→(20,-30)→(20,0)`; corners 76°/115°/79°. Fetch: `scp dderg@ethercatpi5.local:~/printer_data/gcodes/test2.gcode /tmp/test2.gcode` (neptune-bench skill has the host).
- `demo4` (`v150 a200 j4000 scv5`) peaks `v≈134 < 150` → sub-cruise → never hits this path (why T3 missed it).
- Decomposition rule: path-frame `j_t = d(a_t)/dt` (the spikes here); `j_n = d(κv²)/dt = κ'v³ + 2κv·a_t` (lateral, the G3 seam — Non-Goal).

**Manual check:** run `test2.gcode` through the viz; confirm `a_t` eases to 0 at each cruise onset (no vertical drop) and `j_t ≤ max_jerk` there.
