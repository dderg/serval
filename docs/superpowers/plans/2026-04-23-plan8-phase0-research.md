# Plan 8 — Phase 0 Research Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve the five research gaps in the Plan 8 spec so the implementation plans for Chunks 1–3 can be written with concrete technical detail.

**Architecture:** Five opus-model research subagents, one per gap, dispatched in parallel where possible. Each produces a markdown artifact under `docs/superpowers/plans/plan8-research/`. A final collation task merges findings into a concise research summary referenced by downstream implementation plans.

**Tech Stack:** git, Agent tool (opus subagents), markdown artifacts.

**Spec reference:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §6.

---

## Prerequisites

- `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` exists and is approved.
- Working directory is the Kalico repo root (`/Users/daniladergachev/Developer/kalico`).
- Current branch: `magnum-opus`.
- `mkdir -p docs/superpowers/plans/plan8-research` has been run (first step below).

## Convention for all research tasks

1. Subagent runs at opus (architecture/math review rigor, per project memory).
2. Each subagent writes its own artifact file directly (efficient; no copy step).
3. Parent agent verifies the subagent's load-bearing claims against the code before committing.
4. Artifacts live under `docs/superpowers/plans/plan8-research/`.
5. One commit per research task.
6. Commits backdate to today outside work hours per project memory.

---

### Task 0: Create research directory and scaffolding

**Files:**
- Create: `docs/superpowers/plans/plan8-research/.gitkeep` (empty — ensures directory tracked)

- [ ] **Step 1: Create the directory**

```bash
mkdir -p docs/superpowers/plans/plan8-research
touch docs/superpowers/plans/plan8-research/.gitkeep
```

- [ ] **Step 2: Verify it exists**

```bash
ls -la docs/superpowers/plans/plan8-research/
```
Expected: directory with `.gitkeep` inside.

- [ ] **Step 3: Commit (backdate to outside work hours)**

```bash
GIT_AUTHOR_DATE="2026-04-23T07:45:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:45:00+02:00" \
  git add docs/superpowers/plans/plan8-research/.gitkeep && \
  GIT_AUTHOR_DATE="2026-04-23T07:45:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:45:00+02:00" \
  git commit -m "docs(magnum-opus): Plan 8 Phase 0 research directory"
```

---

### Task 1: Research — FIR piecewise evaluator performance

**Artifact:** `docs/superpowers/plans/plan8-research/fir_piecewise_performance.md`

**Spec gap:** §6.1. Does the per-step select-piece-then-evaluate cost stay within itersolve's secant-solver budget when FIR shaping is baked in, especially for sharp-corner moves that produce brief polynomial reversals?

- [ ] **Step 1: Dispatch opus subagent**

Use the Agent tool:
- `subagent_type`: `general-purpose`
- `model`: `opus`
- `description`: `FIR piecewise evaluator perf research`
- `prompt`:

```
You're researching a performance question for a Kalico motion-planner rewrite.

Repo: /Users/daniladergachev/Developer/kalico, branch magnum-opus.

Context: Plan 8 bakes input shaping into the planner. For FIR shaper families (MZV / ZV / EI / EI3 / ZVD — impulse trains with N=2..4 impulses), the planner emits a piecewise polynomial per move with breakpoints at impulse delay offsets. Step-gen selects the right piece at each evaluation time and evaluates. See spec section 3.3 at docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md.

Question: does step-gen's secant-solver (klippy/chelper/itersolve.c:46-123) stay performant when the move polynomial has N breakpoints AND may have brief reversals from aggressive FIR weighting at sharp corners?

Specific investigations:

1. Read klippy/chelper/itersolve.c:46-123 and characterize the secant solver's worst-case cost when the polynomial has interior breakpoints. Does the bisection fallback work correctly across breakpoints?

2. Read klippy/chelper/kin_shaper.c:80-101 (shaper_calc_position + get_axis_position_across_moves) and estimate the per-evaluation cost of today's approach vs the proposed piecewise-polynomial evaluator. Count flops per call for MZV (3 impulses) and EI3 (4 impulses).

3. Analyze when MZV-weighted moves produce polynomial reversal: for a sharp-V corner with pre-corner velocity v1 and post-corner velocity v2 (opposite signs), do MZV's (0.25, 0.5, 0.25) weights produce a sign change inside the piecewise polynomial for the axis whose velocity reverses? Derive the condition.

4. Estimate check_oscillate firing frequency: look at itersolve.c:86-94 for the reversal bracket; what fraction of FIR-baked sharp-corner moves would land in check_oscillate=1 loops that require bisection? Ballpark is fine.

5. Propose a safe fallback: if FIR-baked moves at sharp corners exceed a step-gen performance threshold, what's the mitigation? Options: restrict FIR baking to non-declined corners, fall back to post-hoc FIR for specific moves, reject config with warning.

Write your findings directly to:
/Users/daniladergachev/Developer/kalico/docs/superpowers/plans/plan8-research/fir_piecewise_performance.md

Target length ~1500 words. Include:
- Polynomial reversal derivation with the actual MZV weights
- Cost estimate with concrete numbers (flops, microseconds on a rough CPU budget)
- Reversal/check_oscillate frequency estimate
- Recommendation on mitigation if needed
- Cite file:line references

Then summarize back to me in under 300 words: the verdict (safe / performance concern / blocker with mitigation) and the top-3 load-bearing numeric claims.
```

- [ ] **Step 2: Verify the subagent's load-bearing claims**

Spot-check two claims from the summary by reading the cited file:line references directly. If any claim doesn't match the code, note the discrepancy. Do not mechanically verify every claim — just spot-check the top-3 load-bearing ones.

- [ ] **Step 3: Verify the artifact was written**

```bash
wc -l docs/superpowers/plans/plan8-research/fir_piecewise_performance.md
```
Expected: >100 lines.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T07:50:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:50:00+02:00" \
  git add docs/superpowers/plans/plan8-research/fir_piecewise_performance.md && \
  GIT_AUTHOR_DATE="2026-04-23T07:50:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:50:00+02:00" \
  git commit -m "docs(plan8): research — FIR piecewise evaluator performance"
```

---

### Task 2: Research — Non-linear PA Chebyshev piecewise fit

**Artifact:** `docs/superpowers/plans/plan8-research/pa_piecewise_fit.md`

**Spec gap:** §6.2. What's the worst-case error for Chebyshev-fit tanh / recipr PA across the full supported flow range, not just mid-range?

- [ ] **Step 1: Dispatch opus subagent**

Use the Agent tool:
- `subagent_type`: `general-purpose`
- `model`: `opus`
- `description`: `Non-linear PA piecewise fit research`
- `prompt`:

```
You're researching a numerical approximation question for a Kalico motion-planner rewrite.

Repo: /Users/daniladergachev/Developer/kalico, branch magnum-opus.

Context: Plan 8 bakes pressure advance into the planner. Linear PA is exact (polynomial composition). Non-linear PA models (tanh, recipr) are represented as piecewise Chebyshev polynomials fit per-move against the velocity polynomial. See spec section 3.5.

Question: how many Chebyshev pieces, at what degree, are needed to keep the PA filament-position error under the target (~1 µm) across the full supported velocity range, especially at edges like retracts, hops, ramp-ups?

Specific investigations:

1. Read klippy/kinematics/extruder.py:176-240 for the tanh and recipr PA model math. Specifically `pressure_advance_tanh_model_func` and `pressure_advance_recipr_model_func` in klippy/chelper/kin_extruder.c:182-203.

2. Derive the supported velocity range: read the max v_xy from config parameters (max_velocity), typical retract velocities (20-80 mm/s), small-movement velocities during hops.

3. Derive the numerical error bound for a 2-piece, 3-piece, 5-piece Chebyshev fit of tanh(v / v_lin) across the velocity range. Normalize `v_lin` per the model's scaling. Show the error as a function of v, not just peak error.

4. Same analysis for recipr (reciprocal model — nonlinear saturation).

5. Edge cases that break the fit: retraction (E velocity negative while XY may be zero or nonzero), z-hop (tiny XY motion plus E flow ~0), rapid deceleration (v hits 0 sharply). Does the piecewise fit behave correctly at v=0 and across v-sign changes?

6. Recommend: default number of pieces and degree per piece. Acceptance criterion: reject the fit if error exceeds what?

7. Translate polynomial error to filament-position error: for a 1 mm move at average v = 100 mm/s with PA coefficient 0.05, a 1e-4 relative error in the PA term is how many µm of filament? Sanity-check the "~1 µm" target.

Write your findings directly to:
/Users/daniladergachev/Developer/kalico/docs/superpowers/plans/plan8-research/pa_piecewise_fit.md

Target length ~1500 words. Include:
- Error-vs-pieces curve with concrete numerical values
- Edge-case analysis with at least 3 scenarios
- Recommended default (piece count + degree)
- Acceptance threshold
- Filament-position-error translation
- Cite file:line references

Then summarize back to me in under 300 words: the verdict (recommended defaults) and the top-3 numeric claims.
```

- [ ] **Step 2: Verify top-3 claims**

Spot-check numeric claims against the cited code and model definitions. Do not re-derive the Chebyshev math — trust the subagent's derivation but verify the inputs (max velocity from config, model formulas from the code).

- [ ] **Step 3: Verify artifact**

```bash
wc -l docs/superpowers/plans/plan8-research/pa_piecewise_fit.md
```
Expected: >100 lines.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T07:55:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:55:00+02:00" \
  git add docs/superpowers/plans/plan8-research/pa_piecewise_fit.md && \
  GIT_AUTHOR_DATE="2026-04-23T07:55:00+02:00" GIT_COMMITTER_DATE="2026-04-23T07:55:00+02:00" \
  git commit -m "docs(plan8): research — non-linear PA Chebyshev piecewise fit"
```

---

### Task 3: Research — Per-axis frequency polynomial layout

**Artifact:** `docs/superpowers/plans/plan8-research/per_axis_frequency.md`

**Spec gap:** §6.3. How do we represent a move's polynomial when X and Y have different `f_sh` → different kernel widths → different natural phase boundaries?

- [ ] **Step 1: Dispatch opus subagent**

Use the Agent tool:
- `subagent_type`: `general-purpose`
- `model`: `opus`
- `description`: `Per-axis frequency polynomial layout research`
- `prompt`:

```
You're researching a data-structure design question for a Kalico motion-planner rewrite.

Repo: /Users/daniladergachev/Developer/kalico, branch magnum-opus.

Context: Plan 8 bakes input shaping into the planner polynomial. Each move stores a quintic-in-t polynomial per axis with phases (accel/cruise/decel/blend). See klippy/chelper/trapq.h:33-60 for the current move_quintic_phase struct and klippy/chelper/trapq.c for the quintic machinery.

Problem: Klipper permits different shaper frequencies per axis (shaper_freq_x = 50 Hz, shaper_freq_y = 120 Hz is legal). Different frequencies produce different kernel widths and different natural phase boundaries per axis. But the current move_quintic_phase struct has ONE t_end per phase, shared across axes.

Question: how should the polynomial struct represent axes with different kernel widths?

Candidate approaches:

(A) Pick the finer (narrower kernel) time partition for all axes. Pad the coarser-kernel axis by splitting its natural phase into the finer partition. Polynomial coefficients on the wider-kernel axis repeat/project across the sub-phases.

(B) Per-axis move struct. Each axis owns its own phases array. Breaks the current move_quintic_phase shared-t_end invariant. Invasive.

(C) Shared partition plus per-axis phase mask. Axes flag which sub-phases they're non-trivial on.

Specific investigations:

1. Read klippy/chelper/trapq.h:33-60 to understand the current struct move / move_quintic_phase layout. Note what's shared across axes vs per-axis.

2. Read klippy/chelper/trapq.c:45-100 for quintic_pick_phase and the polynomial evaluation. Where does the shared t_end matter? Could it be split safely?

3. Analyze the polynomial-complexity inflation under approach (A): at worst-case axis mismatch (50 Hz vs 150 Hz — 3× ratio in kernel width), how many extra phase segments per move? Does the step-gen evaluation cost go up proportionally?

4. Analyze approach (B) invasiveness: list every file that currently reads move_quintic_phase expecting shared t_end. Rough LOC impact of the split.

5. Recommend one approach. Tiebreaker priority: evaluator simplicity > emit-side simplicity > memory footprint.

6. If recommending (A), derive the exact padding/projection math for a narrower-kernel axis projected onto a finer partition.

Write your findings directly to:
/Users/daniladergachev/Developer/kalico/docs/superpowers/plans/plan8-research/per_axis_frequency.md

Target length ~1500 words. Include:
- Struct layout comparison (current vs each candidate)
- Worst-case inflation analysis with concrete numbers
- Affected-files list for invasive approach
- Recommendation with reasoning
- If padding/projection chosen: full derivation
- Cite file:line references

Then summarize back to me in under 300 words: the verdict and the rationale.
```

- [ ] **Step 2: Verify**

Spot-check the affected-files claim by grepping for `move_quintic_phase` and `t_end` usage. Verify the recommendation is defensible.

- [ ] **Step 3: Verify artifact**

```bash
wc -l docs/superpowers/plans/plan8-research/per_axis_frequency.md
```
Expected: >100 lines.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T08:00:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:00:00+02:00" \
  git add docs/superpowers/plans/plan8-research/per_axis_frequency.md && \
  GIT_AUTHOR_DATE="2026-04-23T08:00:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:00:00+02:00" \
  git commit -m "docs(plan8): research — per-axis frequency polynomial layout"
```

---

### Task 4: Research — Lookahead commit window

**Artifact:** `docs/superpowers/plans/plan8-research/lookahead_window.md`

**Spec gap:** §6.4. What's the exact minimum extension to `LOOKAHEAD_FLUSH_TIME` required to guarantee move N's polynomial has all kernel-support-worth of neighbors committed before emit?

- [ ] **Step 1: Dispatch opus subagent**

Use the Agent tool:
- `subagent_type`: `general-purpose`
- `model`: `opus`
- `description`: `Lookahead commit window extension research`
- `prompt`:

```
You're researching a timing constraint for a Kalico motion-planner rewrite.

Repo: /Users/daniladergachev/Developer/kalico, branch magnum-opus.

Context: Plan 8 bakes the input shaper into the planner polynomial. For move N's polynomial to encode kernel-shaped motion, the planner needs to know all neighboring moves within the kernel's temporal support BEFORE emitting N. Today the lookahead sets junction velocity during flush (klippy/toolhead.py:194 set_junction in _process_lookahead). If N's polynomial must already encode shape-convolved-with-N+1, N can't emit until N+1..N+k are finalized (k = ceil(kernel_support / min_move_t)).

Question: what's the minimum safe extension to LOOKAHEAD_FLUSH_TIME such that the kernel-support-worth of future moves is always committed before a move emits its polynomial?

Specific investigations:

1. Read klippy/toolhead.py:134, :147, :149-158, :315-323 for the flush timer and commit-window machinery. Understand what "flush" commits (trapq) vs "step_gen" advances (stepcompress).

2. Enumerate worst-case kernel support across all supported shapers:
   - MZV at lowest supported freq (e.g., 20 Hz): how wide?
   - bs5 at lowest supported freq: how wide?
   - smooth_ei at lowest supported freq: how wide?
   Give σ_T or full-support numbers in ms.

3. Characterize min_move_t on the regression corpus (Voron Cube, Cowling, speedbench). What's the p95, p99 shortest move duration? Ref: klippy/toolhead.py MIN_KIN_TIME or equivalent.

4. Late-arrival gcode handling: does Klipper tolerate arbitrary gaps in gcode streaming? What happens if a print pauses mid-flight (M0, M400, idle timeout)? Trace the code path to confirm the flush timer handles quiescent periods correctly.

5. Derive: given worst-case kernel support S and worst-case moves-per-second M, minimum extra flush window = S plus a safety margin of max(S, 10ms). Compute the number.

6. Verify current 250ms (LOOKAHEAD_FLUSH_TIME) is adequate, too tight, or overbuilt. If inadequate, what should it become?

7. Edge case: homing / probing moves bypass shape baking via shape_disabled flag. Do they also bypass the extended commit window? Likely yes, but verify by tracing drip_move through toolhead.py.

Write your findings directly to:
/Users/daniladergachev/Developer/kalico/docs/superpowers/plans/plan8-research/lookahead_window.md

Target length ~1500 words. Include:
- Kernel support table across shapers & frequencies
- Corpus move-duration distribution
- Derived flush-window bound with margin
- Current 250ms adequacy verdict
- Late-arrival and homing edge cases
- Cite file:line references

Then summarize back to me in under 300 words: the verdict and the numeric bound.
```

- [ ] **Step 2: Verify**

Spot-check the kernel-support numbers against the shaper defs in `klippy/extras/shaper_defs.py`. Verify the flush-window conclusion is consistent with `toolhead.py:134`.

- [ ] **Step 3: Verify artifact**

```bash
wc -l docs/superpowers/plans/plan8-research/lookahead_window.md
```
Expected: >100 lines.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T08:05:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:05:00+02:00" \
  git add docs/superpowers/plans/plan8-research/lookahead_window.md && \
  GIT_AUTHOR_DATE="2026-04-23T08:05:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:05:00+02:00" \
  git commit -m "docs(plan8): research — lookahead commit window extension"
```

---

### Task 5: Research — `shape_disabled` flag threading audit

**Artifact:** `docs/superpowers/plans/plan8-research/shape_disabled_audit.md`

**Spec gap:** §6.5. Audit every code path that emits to the trapq. Which should set `shape_disabled = true`?

- [ ] **Step 1: Dispatch opus subagent**

Use the Agent tool:
- `subagent_type`: `general-purpose`
- `model`: `opus`
- `description`: `shape_disabled flag audit research`
- `prompt`:

```
You're auditing code paths for a Kalico motion-planner rewrite.

Repo: /Users/daniladergachev/Developer/kalico, branch magnum-opus.

Context: Plan 8 bakes shaping into the planner polynomial. Some code paths require UNSHAPED motion (homing — position accuracy; probing — exact touchpoint; manual stepper — diagnostics). Solution: a shape_disabled flag on struct move. The planner's polynomial composer skips baking when the flag is set.

Question: audit every code path that emits to the trapq (direct or via lookahead) and determine which must set shape_disabled = true.

Specific investigations:

1. Enumerate all call sites that result in trapq_append or trapq_append_quintic. Start by grepping the Python and C layers:
   - klippy/toolhead.py (lookahead + _process_lookahead)
   - klippy/extras/force_move.py
   - klippy/extras/manual_stepper.py
   - klippy/kinematics/extruder.py (extruder-only moves)
   - klippy/chelper/trapq.c (direct C emitters)
   - klippy/homing.py
   - klippy/extras/probe*.py
   - klippy/extras/tmc*.py (probe-related)
   - Any IDEX / dual_carriage / multi_pin code

2. For each call site, classify:
   - Must-be-unshaped (homing, probing, manual stepper diagnostics): shape_disabled = true
   - Must-be-shaped (normal print moves): shape_disabled = false
   - Conditional (extruder-only — does shaping make sense if no XY motion?): explain

3. Identify edge cases:
   - set_position boundary: can kernel support from before a set_position leak into after? Read klippy/chelper/trapq.c:336-362 for trapq_set_position behavior. Propose handling.
   - drip_move path: does it flow through lookahead.flush? Verify in toolhead.py:749-775.
   - Extruder-only move (pure E with zero XY velocity): should E still see shape from XY's baked kernel? Discuss.

4. For each identified site, state the exact code edit needed (file:line, approximately what line to add the flag setter).

5. Test plan: for each must-be-unshaped site, propose one test that exercises it and verifies the emitted polynomial is truly unshaped (coefficients match the degenerate-linear case).

Write your findings directly to:
/Users/daniladergachev/Developer/kalico/docs/superpowers/plans/plan8-research/shape_disabled_audit.md

Target length ~1500 words. Include:
- Call-site table (path, classifier, reason)
- Edge-case discussion
- Required edits with file:line
- Test plan per must-be-unshaped site
- Cite file:line references

Then summarize back to me in under 300 words: the verdict and the call-site count split (must-be-unshaped / must-be-shaped / conditional).
```

- [ ] **Step 2: Verify**

Spot-check the call-site list by grepping `trapq_append` across the repo and confirming the subagent didn't miss sites. Also confirm the conditional cases have clear reasoning.

- [ ] **Step 3: Verify artifact**

```bash
wc -l docs/superpowers/plans/plan8-research/shape_disabled_audit.md
```
Expected: >100 lines.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T08:10:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:10:00+02:00" \
  git add docs/superpowers/plans/plan8-research/shape_disabled_audit.md && \
  GIT_AUTHOR_DATE="2026-04-23T08:10:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:10:00+02:00" \
  git commit -m "docs(plan8): research — shape_disabled flag threading audit"
```

---

### Task 6: Collate findings into a research summary

**Files:**
- Create: `docs/superpowers/plans/plan8-research/00-summary.md`

**Purpose:** Single-page summary of all five research outcomes. Referenced by the Chunks 1–3 implementation plans so they don't have to re-read every artifact.

- [ ] **Step 1: Read all five artifacts**

```bash
ls docs/superpowers/plans/plan8-research/*.md
```

Expected: `fir_piecewise_performance.md`, `pa_piecewise_fit.md`, `per_axis_frequency.md`, `lookahead_window.md`, `shape_disabled_audit.md`.

Read each one. Extract the verdict and the load-bearing numeric/structural decisions.

- [ ] **Step 2: Write the summary**

Create `docs/superpowers/plans/plan8-research/00-summary.md` with this structure:

```markdown
# Plan 8 — Phase 0 Research Summary

**Date:** 2026-04-23
**Spec:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md`

## 6.1 FIR piecewise evaluator performance

**Verdict:** [safe / performance concern / blocker]

**Key decisions:**
- [bullet from research artifact]

**Blocks implementation detail:** [which Chunks / tasks]

## 6.2 Non-linear PA Chebyshev piecewise fit

**Verdict:** [recommended defaults]

**Key decisions:**
- [bullet]

**Blocks implementation detail:** [which Chunks]

## 6.3 Per-axis frequency polynomial layout

**Verdict:** [approach A / B / C]

**Key decisions:**
- [bullet]

**Blocks implementation detail:** [which Chunks]

## 6.4 Lookahead commit window

**Verdict:** [numeric bound]

**Key decisions:**
- [bullet]

**Blocks implementation detail:** [which Chunks]

## 6.5 `shape_disabled` flag threading

**Verdict:** [site count summary]

**Key decisions:**
- [bullet]

**Blocks implementation detail:** [which Chunks]

---

## Cross-cutting findings

Any surprises discovered during research that affect scope, risk, or implementation ordering.

## Ready-to-implement status

- Chunk 1 (Plan 6 fold): ready / blocked by [gap]
- Chunk 2 (Bake XY shaper): ready / blocked by [gap]
- Chunk 3 (Bake E + PA): ready / blocked by [gap]
```

Fill in each section from the corresponding artifact. Target <500 words total. One page.

- [ ] **Step 3: Update the Plan 8 spec to reference the summary**

Modify `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §6 header to add this line at the top:

```markdown
**Status:** resolved. See `docs/superpowers/plans/plan8-research/00-summary.md` for outcomes.
```

(Use Edit tool. Find the line "## 6. Research gaps (resolve before writing the implementation plan)" and add the status line right after it.)

- [ ] **Step 4: Commit both files together**

```bash
GIT_AUTHOR_DATE="2026-04-23T08:25:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:25:00+02:00" \
  git add docs/superpowers/plans/plan8-research/00-summary.md docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md && \
  GIT_AUTHOR_DATE="2026-04-23T08:25:00+02:00" GIT_COMMITTER_DATE="2026-04-23T08:25:00+02:00" \
  git commit -m "docs(plan8): Phase 0 research summary + spec cross-ref"
```

---

## Post-Phase-0 deliverable

Once Tasks 0–6 complete, this plan is done. The next step is to write the Chunk 1 (Plan 6 fold) implementation plan informed by the research findings. That's a separate writing-plans invocation against a new plan document: `docs/superpowers/plans/2026-04-YY-plan8-chunk1-plan6-fold.md` (date TBD based on when Phase 0 lands).

## Not in this plan

- Chunk 1, 2, 3 implementation tasks — separate plans each.
- Final cleanup tasks — separate plan after Chunk 3.
- Hardware validation — user-driven, never plan-gated.
