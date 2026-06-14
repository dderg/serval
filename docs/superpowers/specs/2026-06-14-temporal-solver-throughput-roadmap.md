# Temporal Solver Throughput Roadmap (Pi5 / Pi4)

> Brainstorm output — 18-agent ultracode workflow (profile → diverse-lens design panel → adversarial verify → synthesize), 2026-06-14. Source crash: a 46mm mid-stream G5 at cruise took 867ms in `plan_batch` and landed 0.369s in the past → fail-loud `SegmentLate`.


---

## Recommended roadmap (synthesis)

# Temporal Solver Real-Time Roadmap (Pi5 / Pi4)

## Problem in one paragraph

A mid-stream 46mm G5 cubic at cruise velocity took **867ms** in `temporal::plan_batch` and landed **0.369s in the past**, triggering the fail-loud `SegmentLate` abort. The cost is **not** one expensive solve — it is **100–200 cold-start Clarabel invocations** per replan, driven by the SLP/SLP9 cascade. Cost grows with entry velocity (63 → 93 → 99 → 190 → 867ms) because high-speed entries push the axis-jerk SLP into its `TrFloorStall → restoration` path and force the `ToleranceMode::Auto` 1e-5→1e-8 double-solve. Meanwhile **3 of 4 cores sit idle** (single-segment batch → single work item in `fan_out_solves`), every solve **cold-starts** `DefaultSolver::new` (no warm-start, no symbolic reuse), and each call **re-scans a 7MB dense `a_rows` matrix** (~908k reads/call) that is >99% zeros.

The non-negotiable governs every lever below: **no lever may ship a measurably slower trajectory.** Each is classified as provably trajectory-neutral or explicitly dropped.

---

## Ranked, sequenced roadmap

The ordering is: **cheapest, safest, fork-free wins that help EVERY solve first; algorithmic waste-elimination second; warm-start (solver fork) third; multicore fourth; architecture last/never.** This ordering is deliberate — each early lever shrinks the work the later, harder levers must do, and several later levers depend on the early ones (e.g. warm-start needs a stable CSC build; parallelism needs the per-call cost down so cache contention doesn't dominate).

| # | Lever | Realistic Pi5 | Trajectory-safe | Effort | Fork? |
|---|-------|---------------|-----------------|--------|-------|
| 1 | CSC-direct bundle build (kill dense `a_rows` scan) | 1.03–1.06x | Yes (provable) | small | no |
| 2 | Eliminate wasted SLP/fallback work (TR `max_iter` cap; avoid the Auto double-solve on the fast path) | 1.3–1.8x | Yes (with guard) | medium | no |
| 3 | Feasibility-seed SLP9 + unconditional speed polish | 1.5–2.5x | **Only with the polish fix** | medium | no |
| 4 | Persistent warm-started inner solver | 1.4–2.0x | Yes (provable) | large | **yes** |
| 5 | Cross-segment / cross-core parallelism | up to ~2x | Yes (gated) | medium | no |
| ✗ | Async plan-ahead buffer | — | **No** — DROP | large | — |
| ✗ | Grid-N reduction (arc-length recompute) | — | **No** — DROP | small | — |
| ✗ | Parallel SLP9 backtrack fan-out (as proposed) | 1.1–1.4x | Yes but mis-bundled | medium | — |

Multipliers do **not** cleanly multiply (warm-start removes the iterations that symbolic-reuse would amortize; seeding removes the calls that parallelism would fan out). Realistic **stacked** target: levers 1–3 bring the 867ms worst case to **~250–400ms on Pi5**; adding lever 4 and gated lever 5 brings it under the 250ms LEAD with Pi4 margin. **No single lever clears the budget — the stack does.**

---

## Lever 1 — CSC-direct bundle build (FIRST; foundation)

**Mechanism.** `ConstraintBundle.a_rows` is `Vec<Vec<f64>>` with a full `vec![0.0; n_vars]` allocated per row (`constraints.rs:352-360`), then re-scanned column-by-column on every Clarabel call (`solver.rs:402-410`) — ~908k reads/call, 100–200 calls/segment. Build the static base matrix once in column-sparse form (`rowval_per_col`/`nzval_per_col`) at the bottom of `build_chain`; per-call, clone the small sparse base (~1.4k nnz) and append cut/TR rows, dropping the per-call cost from O(n_rows·n_vars) to O(nnz).

**Where it lands.** `rust/temporal/src/topp/constraints.rs` (`build_chain`, `push_row`), `rust/temporal/src/topp/solver.rs` (`solve_with_cuts_and_trust_region` CSC assembly).

**Realistic speedup.** Pi5 **~3–6%** (~25–50ms off 867ms), Pi4 **~3–6%**. The verifier corrected the proposal's inflated "15%" — the scan is row-contiguous/sequential, streaming from L2/L3, so the recoverable assembly cost is ~30–50ms, not 540ms. QDLDL factorization + IPM dominate.

**Why first anyway.** It is provably trajectory-neutral (byte-identical CSC → identical Clarabel input → identical solution), helps the un-parallelizable path-jerk loop AND both tolerance passes, is a prerequisite home for the warm-start solver's pre-sized buffer (lever 4), and reduces the 7MB working set that thrashes Pi5's 512KB/core L2 — which directly de-risks lever 5's cache contention.

**Trajectory justification.** Pure representational refactor. Add a `debug_assert` that the precomputed CSC is byte-identical to the scan output. No N, tolerance, iteration-budget, or acceptance-bar change.

---

## Lever 2 — Eliminate wasted SLP/fallback work

**Mechanism (two parts).**
- **TR `max_iter` cap.** `max_iter=1000` (`solver.rs:566`) was set for the CL-2024 base-SOCP counterexample, but the SLP9 backtrack cascade fires up to 4 trust-region subproblems per outer iteration, and an *infeasibly-tight* TR can run to ~1000 IPM iterations before reporting `MaxIter`. Cap TR-constrained solves (`trust_region.is_some()`) at ~200; **keep 1000 for `trust_region=None`** (base SOCP + no-TR fallback). First verify the CL-2024 fixture is a base-SOCP case, not a TR subproblem.
- **Avoid the `ToleranceMode::Auto` double-pay on the fast path.** The 1e-5 pass fails `solver_outcome_is_success` whenever SLP9 returns `Diverged`/`MaxIters`, forcing a full cold 1e-8 re-run (`mod.rs:145-152`). Levers 2 (TR cap) and 3 (seeding) make SLP9 *converge* on the fast pass, so the tight re-run stops firing for the high-velocity case — recovering a full second solver pass. **Keep the Auto fallback** (the trajectory-invariants verifier flagged it as itself an invariant — forcing 1e-5 unconditionally surfaces divergence on fragile geometry).

**Where it lands.** `rust/temporal/src/topp/solver.rs` (`solve_with_cuts_and_trust_region` settings block, `run_slp9_loop`), `rust/temporal/src/topp/mod.rs` (Auto path — left structurally intact; it simply fires less once SLP9 converges).

**Realistic speedup.** Pi5 **~1.3–1.8x**, Pi4 similar multiplier. Bounds the worst-case backtrack waste and removes the tight re-run on the dominant failing case.

**Trajectory justification.** Capping TR-subproblem `max_iter` only bounds *infeasible* TR probes — those get rejected by `cand_ratio < best_ratio` regardless; a `MaxIter` TR candidate is discarded, not shipped. The base SOCP and no-TR fallback keep the full 1000 budget, so feasibility certification is unchanged. Acceptance bars (`verify::EPS_FEAS`, `EPS_FEAS_JERK`) untouched. **Guard: confirm CL-2024 is base-SOCP before applying the cap.**

---

## Lever 3 — Feasibility-seed SLP9 + unconditional speed polish

This is the single highest-leverage *algorithmic* fix, but it was found **trajectory-UNSAFE as originally proposed** and is kept **only with the mandatory polish fix.**

**Mechanism.** The 867ms pathology is: high entry velocity → SLP path result has axis-jerk ratio ≫ 1 → SLP9 backtracks stall → `TrFloorStall` → `damp_scale_for_axis_feasibility` restoration → a *second* full 30-iteration SLP9 pass. The restoration machinery already computes a feasibility-damped point — but only *after* ~150 wasted calls. **Seed it up front:** if `initial_max > SEED_TRIGGER_RATIO`, run the existing `damp_scale_for_axis_feasibility` + `damp_interior_a` (O(N) scalar work, zero Clarabel calls) to enter `run_slp9_loop` already feasible (ratio ~0.9), eliminating the stall and the restoration pass.

**The trajectory hazard (why the original is unsafe).** The verifier proved the proposal's load-bearing claim — *"trajectory time depends on b, not a; seeding leaves b unchanged so the result is identical"* — is **false against the code.** `run_slp9_loop` re-optimizes *both* b and a every subproblem; it is a pure *feasibility descent* (`accept iff cand_ratio < best_ratio`, returns at first `ratio ≤ 1.05`) with **no time objective**. Different seeds hit the 1.05 threshold at different b-profiles. The only stage that re-maximizes speed is `polish_windowed`, which **runs only when `windows.is_some()`** — and the exact 867ms case (`window_segments=1`, no follower) passes `windows=None`, so **polish is skipped and the raw damped (slower) descent endpoint ships.** `verify::check_chain` checks *feasibility, not optimality*, so it cannot catch the regression.

**Mandatory changes to make it safe:**
1. **Run a time-maximizing polish unconditionally** after the seeded descent in the `windows=None` single-segment path (reuse `polish_windowed`'s logic without follower cuts).
2. **Add a gating regression test** asserting seeded `profile_time ≤ un-seeded baseline` (within tolerance) on the measured 5-curve mid-stream sequence — trajectory *time*, not just feasibility.
3. **Drop** the proposed `SLP9_MIN_IMPROVING_ITERS` guard — extra feasibility-descent iterations only damp *more* (slower), making the safety problem worse.

**Where it lands.** `rust/temporal/src/topp/solver.rs` (`slp_solve_with_axis_jerk_chain_inner` seeding + unconditional polish; the TR cap from lever 2 lives here too).

**Realistic speedup.** Pi5 **~1.5–2.5x** (867ms → ~300–450ms; the verifier de-rated the proposal's 3–7x). Pi4 ~1.5–2.5x, not yet real-time alone.

**Trajectory justification.** **Conditional on the polish fix + the gating time-regression test.** With unconditional speed polish, the seeded descent endpoint is re-maximized to the same local optimum the un-seeded path reaches, and the test *proves* no time loss on the representative sequence. Without the polish fix, this lever **knowingly risks a measurably slower trajectory and must not ship.**

---

## Lever 4 — Persistent warm-started inner solver

**Mechanism.** Every inner solve calls `DefaultSolver::new` (`solver.rs:578`), discarding the AMD ordering, symbolic factorization, and the primal-dual iterate. Two compounding techniques: **(a) warm-start** each inner solve from the previous accepted iterate via the central-path smoothing operator (arXiv 2512.00693, demonstrated *on Clarabel* for parametric SOCP families — consecutive SLP9 iterates differ by only `rho_b·b_bar`, the favorable small-perturbation regime); **(b) symbolic-factorization reuse** across a fixed-sparsity SLP family.

**Verifier corrections folded in.**
- **Symbolic reuse is largely defeated** by *variable* sparsity: `build_axis_jerk_cuts_chain` pushes one cut *per violating grid point* (the set shrinks as SLP9 converges), and TR-present vs no-TR-fallback solves carry different cone blocks. So the "fixed pattern" premise fails against the code — **re-scope or drop this half** pending a real fixed-super-pattern audit; the headline win must come from warm-start alone.
- Warm-start's favorable 0.5–0.63x multiplier applies only to clean small-perturbation re-solves, **not** to the structural transitions (1e-5→1e-8 tolerance jump, SLP9 restoration reset to `RESTORE_RHO_B_INIT=0.50`, TR collapse). **Mandatory cold-start fallback** when the warm point's infeasibility residual exceeds threshold.

**Where it lands.** `rust/temporal/src/topp/solver.rs` (persistent `WarmSolver` handle threaded through the SLP loops), a **vendored/forked Clarabel** patched to expose `update_data` + a warm-start entry. This fork is the dominant cost and a standing maintenance liability.

**Realistic speedup.** Pi5 **~1.4–2.0x standalone** (verifier de-rated the proposal's 2.5–3.5x; symbolic-reuse contributes little). Multiplier is algorithmic, so it transfers to Pi4 (which needs it more).

**Trajectory justification.** Independently verified safe (trajectory-invariants Attack 6): warm-starting a *convex* IPM converges to the same ε-optimal KKT point; smoothing operator lands on the *new* central path (Thm 3.1) with O(μ₀) residuals (Thm 4.1) → same tolerance, not looser. Keeps the `max_threads=1`/qdldl determinism pin. **Hard gate:** a botched warm-start must not corrupt Clarabel's `Infeasible`/`MaxIter` status mapping (the SLP fail-loud contract depends on it) — regression-test against the CL-2024 counterexample fixture.

**Sequencing.** *After* levers 1–3 and the gated lever 5, because those deliver comparable/larger multipliers with **no solver fork**, and they shrink the call count this lever amortizes over.

---

## Lever 5 — Cross-segment / cross-core parallelism (gated)

**Mechanism.** With `window_segments=1` the batch has one chain → one work item → one core busy in `fan_out_solves` while 3 sit idle. The win is **across segments**, not within a solve. The proposed *backtrack* fan-out (4 TR probes concurrent) is the wrong cut: the verifier showed the 867ms is often dominated by the strictly-sequential path-jerk loop + Auto double-pass, not SLP9 backtracks, so backtrack fan-out yields only ~1.1–1.4x and risks nested-`thread::scope` oversubscription (N_chains × 4 threads). **Prefer parallelism at the chain/segment granularity**, gated to fire only when there is genuinely independent work.

**Where it lands.** `rust/temporal/src/multi/parallel.rs` (`fan_out_solves` work distribution), `rust/temporal/src/multi/mod.rs`.

**Realistic speedup.** Up to ~2x *only when multiple independent chains exist*; near-zero for the single-segment case (which levers 2–4 already attack). Keep the `max_threads=1`/qdldl determinism pin (no multithreading *within* a solve).

**Trajectory justification.** Trajectory-neutral — same per-chain solve, only scheduled across cores. Must keep deterministic chain ordering so the joining early-bail stays reproducible. **Gate to avoid oversubscription** when nested under existing worker pools.

---

## Dropped approaches

- **Async plan-ahead buffer — DROP (trajectory-unsafe).** Its load-bearing claim ("buffering is trajectory-neutral, solver untouched") is false: every `append_and_replan` solves with `terminal_v=0.0`, so committing buffered curves to fill a lookahead either commits a **decel-to-rest at every junction** (measurably slower) or requires the **unimplemented, ~2.2x-slower multi-segment SOCP** — contradicting "solver untouched." Only salvageable sliver: growing the chain-*entry* `advance_idle` pad to cover measured worst-case solve while the anchor is still in the future. Split that out as a tiny separate change; it does **not** address mid-stream steady-state starvation.
- **Grid-N reduction via arc-length recompute — DROP (trajectory-unsafe).** `compute_n` defines spacing along the control polygon *by contract*; swapping to arc length silently *reduces* N, relaxing the discretized SOCP and under-enforcing constraints at fewer points. Coarsening solve **and** verify grids together makes inter-grid violations invisible to both. The only safe variant — `N = max(arc_len, polygon)/spacing` — only *raises* N (zero speedup). Drop from the speedup case.
- **Parallel SLP9 backtrack fan-out (as bundled) — DEPRIORITIZE.** Trajectory-safe but speedup overstated (path-jerk + Auto double-pass dominate, not backtracks) and the genuinely valuable CSC change was mis-bundled into it. The CSC piece is lever 1; the parallelism belongs at chain granularity (lever 5).

---

## Concrete first step (implement + benchmark next)

**Ship Levers 1 + 2 together as the first PR** — both are fork-free, trajectory-neutral (lever 2 with the CL-2024 guard), and land entirely in `constraints.rs` + `solver.rs`.

**Steps:**
1. **Lever 1:** precompute column-sparse base CSC in `build_chain`; clone + append in `solve_with_cuts_and_trust_region`; `debug_assert` byte-identity vs the old scan.
2. **Lever 2:** add a `max_iter_override: Option<u32>` param; pass 200 when `trust_region.is_some()`, keep 1000 otherwise. First confirm CL-2024 (`solver.rs:558`) is a base-SOCP fixture.
3. **Benchmark on the trident-bench Pi5** via the existing `ReplanReport.solve_us` pipeline, replaying the measured 5-curve mid-stream sequence (rest → cruise).

**Expected before/after (Pi5, worst-case 5th curve):**

| Stage | solve_us (Pi5) | Notes |
|---|---|---|
| Baseline | **867ms** | SegmentLate (0.369s past) |
| + Lever 1 | ~820–840ms | scan overhead recovered |
| + Lever 2 | ~480–620ms | TR waste bounded; tight re-run reduced |
| + Lever 3 (seed+polish) | **~300–400ms** | clears mid-stream SegmentLate on Pi5 |
| + Lever 4 (warm-start) | ~200–280ms | under 250ms LEAD with margin |
| + Lever 5 (where independent) | further on multi-chain | Pi4 margin |

**Verification gate:** after lever 3, the new regression test must show seeded `profile_time ≤ un-seeded baseline` within tolerance on every curve in the sequence; `cargo nextest run -p temporal` green; `./scripts/ci.sh quick` green.

---

## Open questions / measurements needed to de-risk

1. **Per-phase wall-time split on the live 867ms case.** The cited profiles disagree on whether path-jerk SLP + the Auto double-pass dominate, or SLP9 backtracks dominate. Instrument a per-phase breakdown (path-SLP vs SLP9 vs Auto second pass) on the Pi5 before committing lever 3/5 effort. **This is the single highest-value measurement** — it tells us whether seeding (lever 3) or path-jerk attack is the real lever, and whether backtrack parallelism is worth anything.
2. **Is the CL-2024 counterexample a base-SOCP or a TR subproblem?** Gates lever 2's `max_iter` cap. If TR, the cap reintroduces `InsufficientProgress` and must be scoped differently.
3. **Does seeding change the converged local optimum?** The SLP9 problem is non-convex; the gating time-regression test (lever 3) must run across representative slicer output, not just the 5-curve bench sequence, to confirm no time loss in the wild.
4. **Warm-start residual on the structural transitions.** Measure how often the cold-start fallback fires (1e-5→1e-8 jumps, SLP9 restoration). If frequent, lever 4's realistic multiplier drops toward 1.4x and the Clarabel-fork maintenance cost may not be justified versus more parallelism.
5. **Is there a stable fixed super-pattern for symbolic reuse?** Audit `build_axis_jerk_cuts_chain` + the path-jerk cut builder. If no stable pattern exists, drop the symbolic-reuse half of lever 4 entirely.
6. **Pi4 steady-state sustainability.** If *average* solve time (not the spike) approaches per-curve playback time on Pi4, no lever short of multi-segment-SOCP + multicore closes the gap. Measure average solve_us across a representative print on Pi4 to confirm the stack is sufficient, not just the worst-case spike.
7. **Multi-chain frequency in real prints.** Lever 5's value depends entirely on how often `window_segments > 1` occurs on representative slicer output. Measure the chain-count distribution before investing in chain-granularity parallelism.


---

## Appendix A — design approaches with adversarial verdicts


### Parallel SLP9 Backtrack Fan-Out on 4 Cores  ·  effort=medium  ·  verdict=**keep_with_changes**  ·  trajectory_safe=True

- **One-liner:** Run the SLP9 trust-region backtrack candidates concurrently on all 4 Pi5 cores instead of sequentially, turning the 4-call serial inner loop into a single-latency parallel probe and using the idle cores that fan_out_solves leaves empty on every single-segment replan.

- **Proposer speedup:** 
WORST-CASE (THE 867ms CASE) — Pi5

The 867ms case is characterized by 30 outer SLP9 iterations with 4 backtrack calls per iteration all failing (none accept), plus the no-TR fallback, totaling ~5 Clarabel calls per outer iteration * 30 = ~150 axis-jerk SLP calls (profile finding: ~100-200 total Clarabel invocations).

With 4 parallel backtrack probes, each outer iteration still takes 1 parallel slot (the 4 backtracks run simultaneously in ~1 call-latency = ~5-10ms on Pi5 instead of ~4 call-latencies = ~20-40ms). The outer iteration wall time drops from ~5 * T_clarabel to ~1 * T_clarabel + T_clarabel (the no-TR fallback when all 4 reject). Reduction per outer iteration: from 5 sequential calls to 2 sequential call-latencies (parallel 4 + 1 fallback).

Speedup per outer iteration: 5 / 2 = 2.5x.
Apply to the 150-call axis-jerk dominated portion: 150/5 outer iters * 2T vs 150/5 outer iters * 5T → 2.5x.

The path-jerk SLP (slp_solve_chain) is not parallelized; it contributes ~15-30 Clarabel calls of the total. At ~6ms each: 30*6ms = ~180ms. This portion is not improved.

End-to-end estimate for the 867ms case:
  - Path-jerk SLP (unchanged): ~60-180ms (varies; call it ~120ms worst case)
  - Axis-jerk SLP9 (parallelized): 867ms - 120ms = ~750ms of axis-jerk work -> ~750ms / 2.5 = ~300ms
  - Total: ~420ms

Wall-time estimate Pi5: 867ms → ~380-450ms. Roughly 2x end-to-end.

The 250ms LEAD budget is still exceeded at 400ms. To reach under 250ms the path-jerk loop or the CSC matrix overhead must also be attacked. The COO-based CSC construction (described above, orthogonal change) eliminates the ~150x-wasteful matrix scan, which accounts for a share of the per-call latency. If per-call time drops 30-40% (conservative: O(nnz) CSC build is faster but QDLDL factorization still dominates), the combined effect is:
  - Axis-jerk: 750ms * (0.65 per-call speedup) / 2.5 (backtrack parallelism) = 195ms
  - Path-jerk: 120ms * 0.65 = 78ms
  - Total: ~273ms

With the COO change combined, the 867ms case reaches approximately 250-280ms on Pi5, which is at the LEAD budget boundary. This does not yet guarantee zero SegmentLate events on worst-case curves, but it reduces the rate dramatically.

Pi5 MODERATE CASE (20-30 SLP9 iterations, some backtracks accept early)

For the mid-stream curves showing 190ms (the 4th in the chain): similar analysis, ~40 Clarabel calls total (rough estimate). Path-jerk: ~10 calls * 6ms = 60ms. Axis-jerk: 30 calls / 5 per outer * 2T_clarabel = 36ms vs ~60ms sequential. Total: ~96ms vs ~190ms. Roughly 2x on this case too.

Pi4 ESTIMATE (4x A72 @ 1.8GHz, ~30-40% slower per core than A76)

Same parallelism structure. Pi4 A72 integer/load throughput is ~30-40% below A76 for this type of sparse linear algebra. T_clarabel on Pi4 is ~8-14ms per call vs 5-10ms on Pi5. The parallelism benefit scales identically: backtrack loop still goes from 5 sequential to 2 sequential latencies. Absolute times are 30-40% higher. The 867ms case on Pi4 likely starts at ~1,200ms; after change: ~600ms. Not yet within LEAD budget on Pi4 without the COO CSC change, which brings it to approximately ~400ms — still over 250ms.

CONCLUSION

Backtrack parallelism alone: 2-2.5x end-to-end, bringing Pi5 worst case from 867ms to ~400ms.
Backtrack parallelism + COO CSC build: brings Pi5 worst case to ~260-280ms, brushing the LEAD budget.
Pi4 with both changes: ~400ms worst case, still exceeding LEAD, but the moderate-speed cases (190ms observed) would reach ~100ms, within budget.

To clear LEAD on Pi4 worst case, a third intervention is needed: parallel fast+tight ToleranceMode passes (the "second-order" item described in mechanism), or capping max_iter for infeasible TR subproblems (profile finding 3: max_iter=1000 causes up to 1000-iteration waste when the TR is infeasibly tight).


- **Verifier realistic speedup:** 1.2–1.6x end-to-end on the 867ms worst case (Pi5), not the claimed 2–2.5x. Best plausible with the COO/CSC change bundled in: ~1.8x → ~480ms, still ~2x over the 250ms LEAD. The backtrack fan-out alone is likely 1.1–1.4x. On Pi4, ~1.1–1.3x. The fast/easy-segment path may regress slightly without the proposed gating heuristic.

- **Verifier reasoning:** MECHANISM IS SOUND, AND TRAJECTORY-SAFE — this is the proposal's one strong leg. I verified against solver.rs:1137-1164: within a single SLP9 outer iteration the 4 backtrack solves share no mutable state (rho_b/rho_a mutate only at 1182/1195, AFTER the loop; last_result/cuts/bundle are constant; best_ratio is a fixed threshold during the loop and the loop breaks on the FIRST k that beats it). So "fan out the 4, then pick the smallest-k acceptor with cand_ratio < best_ratio" is provably identical in outcome to the sequential break-on-first-accept. N, cuts, tol, SLP9_EPS_FEAS, verify bars, ToleranceMode::Auto, and SegmentLate fail-loud are all untouched. trajectory_safe = true holds for THIS micro-transformation.

BUT THE SPEEDUP IS MATERIALLY OVERSTATED, and the case rests on a cherry-picked profile interpretation. Three concrete problems:

(1) The 867ms is NOT 750ms of SLP9 backtrack work. slp_solve_with_axis_jerk_chain_inner (solver.rs:1389-1393) RETURNS BEFORE SLP9 runs if the path-jerk SLP diverges/maxiters. The profile [slp-fallback] Finding 7 says high-velocity stalls do exactly this: path-jerk SLP diverges, SLP9 never executes, then ToleranceMode::Auto (mod.rs:145-152) re-runs the ENTIRE call_slp at 1e-8. The dominant cost in that regime is the 50-iteration, strictly-sequential, NON-parallelized path-jerk loop (slp_solve_chain, solver.rs:1009-1040, where each outer iter's cuts depend on the previous solve) — run TWICE. The backtrack fan-out does nothing there. The proposal's own cited profiles disagree on the split (socp-cost: "~50 path iters + 15-20 SLP9"; slp-fallback: "SLP9 stall dominates"). The proposal silently adopts the SLP9-dominant story to justify the win.

(2) Amdahl + the un-parallelized tail. Even in the SLP9-dominant case, per outer iteration the cost is 4 backtracks + 1 no-TR fallback (solver.rs:1166, stays sequential after the parallel batch). That's 5→2 sequential latencies = 2.5x on that fraction only. With path-jerk un-parallelized and ToleranceMode::Auto doubling everything, the proposal's own end-to-end estimate is 867→~400ms — STILL 1.6x over LEAD. If path-jerk is 40-50% of wall time (socp-cost's reading), end-to-end collapses to 1.3-1.5x.

(3) Memory-bandwidth ceiling kills the parallel factor. Every solve cold-starts DefaultSolver::new (solver.rs:578), rebuilds CSC, and re-factorizes QDLDL — which the profile calls "the dominant per-call cost" and which is memory-traffic-bound. 4 concurrent solves contend for Pi5's 2MB / Pi4's 1MB shared L3. The proposal admits this drops 2.5x→1.5x; on Pi4 it concedes 1.8-2.2x on the parallel fraction. Net of Amdahl, real end-to-end is 1.2-1.6x.

ADDITIONAL UNANALYZED HAZARD: the run_slp9_loop change is UNCONDITIONAL, but for a MULTI-segment batch the existing fan_out_solves (parallel.rs:37-58) already spawns worker_threads, and run_slp9_loop runs inside those. Nesting a 4-way thread::scope inside that gives N_chains × 4 threads on 4 cores — oversubscription and cache thrash the proposal never models (it only reasons about the single-segment case). Needs a gate: fan out only when the batch is single-chain.

FAST-PATH REGRESSION is real and under-weighted: on easy segments k=0 accepts immediately, so 3 of 4 concurrent solves are wasted per outer iter. The proposed "skip fan-out when rho_b hasn't shrunk" heuristic disables the optimization on the FIRST outer iteration of EVERY problem — including the warm-up of hard problems — so it both protects easy cases and blunts the hard-case win.

THE ACTUALLY-VALUABLE PIECE IS MIS-BUNDLED. The COO/CSC change (eliminating the O(n_rows·n_vars) dense scan at solver.rs:402-410, ~908k reads/call) is orthogonal, trajectory-neutral, and helps EVERY solve — including the un-parallelizable path-jerk loop and both tolerance passes. That should be the FIRST intervention, shipped and measured alone. But even it is bounded: the profile says QDLDL factorization, not CSC assembly, dominates per-call cost, so the "30-40% per-call drop" feeding the proposal's "273ms combined" contradicts its own dominance claim.

REQUIRED CHANGES to keep: (a) gate the fan-out to single-chain batches to avoid nested-scope oversubscription; (b) split the COO/CSC change into its own measured PR and land it first; (c) replace the asserted 750ms-SLP9 split with a measured per-phase breakdown on the live Pi5 867ms case before committing to the backtrack work — if path-jerk + the Auto double-pass dominate, the backtrack fan-out is near-useless and effort should go to (i) avoiding the path-jerk divergence that forces the 1e-8 re-run, or (ii) racing the fast/tight passes / warm-starting, both of which the proposal correctly flags as follow-ons but which are probably the real lever. The non-negotiable is satisfied (no slower trajectory), but the throughput claim is not yet earned: it does not, on its own, clear LEAD on either Pi4 or Pi5 worst case, and the headline 2-2.5x is optimistic by roughly the path-jerk fraction it ignores.

- **Required changes:** 1) Gate the parallel fan-out to single-chain batches; for multi-chain batches keep sequential backtracks to avoid nesting a 4-way thread::scope inside the existing fan_out_solves worker pool (N_chains×4 thread oversubscription on 4 cores). 2) Split the COO/CSC matrix-construction change into a separate PR and land+measure it FIRST — it is orthogonal, trajectory-neutral, and helps the un-parallelizable path-jerk loop too. 3) Before building the backtrack fan-out, capture a measured per-phase wall-time split (path-jerk SLP vs SLP9 vs ToleranceMode::Auto second pass) on the live 867ms case; the whole speedup case depends on SLP9 dominating, which the cited profiles contradict. 4) Keep the no-TR fallback (solver.rs:1166) and Clarabel max_threads=1 pin. 5) Implement the easy-segment gating heuristic to avoid the 3-wasted-solves regression on rest-to-rest moves.

- **Mechanism:** 
WHAT IS IDLE AND WHY

fan_out_solves (multi/parallel.rs:14-58) dispatches one work item per chain into a Mutex-guarded queue. With window_segments=1, the batch has exactly one chain. The first thread dequeues it; the other 3 threads find the queue empty and return immediately. Every single-segment replan runs the entire SLP cascade on one core while 3 Pi5 cores sit idle for the full 867ms.

THE HOT INNER LOOP (the target)

Inside run_slp9_loop (solver.rs:1063-1214), each outer iteration builds one cut set then fires up to 4 sequential Clarabel calls in the backtrack cascade (solver.rs:1137-1177):

  for backtrack in 0..=SLP9_MAX_BACKTRACKS {           // k = 0, 1, 2, 3
      let tr = TrustRegion {
          rho_b: rho_b * 0.5f64.powi(bt_i32),         // rho_b, rho_b/2, rho_b/4, rho_b/8
          rho_a: rho_a * 0.5f64.powi(bt_i32),
      };
      let candidate = solve_with_cuts_and_trust_region(bundle, &cuts, Some(tr), ...);
      if candidate.status is bad { continue; }
      if cand_ratio < best_ratio { accepted = Some(candidate); break; }
  }

These 4 calls are completely independent: same bundle, same cuts, same b_bar/a_bar anchors, different (rho_b, rho_a) radii. They share no mutable state. The only ordering dependency is the early-break: if k=0 accepts, k=1..3 are skipped. In the observed 867ms case the early break rarely fires (that is why we are paying for 4 calls) — 3 or all 4 run to completion and are still rejected (accepted stays None), triggering the no-TR fallback at line 1166.

THE PROPOSAL: FAN-OUT BACKTRACK CALLS IN PARALLEL

Replace the sequential backtrack loop with a scoped thread::scope (or Rayon join_all) that fires all 4 candidates simultaneously on 4 separate threads, then takes the best-accepted result:

  thread::scope(|s| {
      let handles: Vec<_> = (0..=SLP9_MAX_BACKTRACKS).map(|k| {
          let tr = TrustRegion {
              rho_b: rho_b * 0.5f64.powi(k as i32),
              rho_a: rho_a * 0.5f64.powi(k as i32),
          };
          s.spawn(|| solve_with_cuts_and_trust_region(
              bundle, &cuts, Some(tr), &last_result.b, &last_result.a, tol, scale,
          ))
      }).collect();
      handles.into_iter().map(|h| h.join().unwrap()).collect::<Vec<_>>()
  });

After the parallel fan-out, select the smallest k whose candidate is (a) not Infeasible/MaxIter and (b) cand_ratio < best_ratio. If none qualify, run the no-TR fallback (line 1166) on the main thread as before.

The acceptance rule is applied post-hoc to whichever candidates came back — this is identical in outcome to the sequential loop, because the sequential loop takes the smallest-k acceptor anyway (it breaks on first accept). Running k=1,2,3 in parallel and discarding them when k=0 accepts wastes a small amount of compute on the early-break cases, but:

  - On the critical (slow) path k=0 rarely accepts (that is the definition of the path being slow). All 4 candidates run to completion regardless in the sequential version. The parallel version converts this sequential cost into a parallel cost.
  - On fast converging problems k=0 usually accepts on the first outer iteration; then k=1/2/3 run for one outer iteration in parallel. This is one wasted concurrent solve per segment when the first backtrack succeeds — a small fixed cost against the known 50ms baseline for fast cases (63ms first curve).

WHY THIS IS TRAJECTORY-SAFE (quoting the non-negotiable constraint)

"The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware."

This change does NOT:
- Change N (no coarsening of the grid; the same n_vars, same constraint rows, same SOCP)
- Change the feasible set (same cuts, same bundle, same tolerance tol passed to every solve_with_cuts_and_trust_region call)
- Change the SLP9 outer iteration count or convergence criterion (SLP9_MAX_OUTER_ITERS=30 unchanged, best_ratio descent and SLP9_EPS_FEAS acceptance unchanged)
- Change the acceptance gate (best candidate by smallest k, same as sequential)
- Change SLP_MAX_OUTER_ITERS for the path-jerk loop (that remains sequential — see "What is not in scope" below)
- Change Clarabel tolerance (tol passed unchanged; ToleranceMode::Auto 1e-5/1e-8 logic untouched)
- Modify verify::check_chain or its acceptance bars (EPS_FEAS=2e-3, EPS_FEAS_JERK=5e-2)
- Re-anchor late segments (SegmentLate remain fail-loud)

The SOCP is convex; the inner problem solved by each thread is deterministic given (bundle, cuts, tr, b_bar, a_bar, tol). Running 4 instances in parallel converges to the same ε-optimal point as running them sequentially. The only nondeterminism introduced is in which of the simultaneously-returned candidates is selected when multiple k values produce cand_ratio < best_ratio — but the sequential rule already takes the first (smallest k), so selecting the smallest-k acceptor from parallel results is identical.

DETERMINISM CONCERN AND THE JOINING LOOP

solver.rs:563 pins max_threads=1 on Clarabel for "joining-loop early-bail determinism." That pin is on individual Clarabel instances (the QDLDL factorization inside a single solve). This proposal does not multithread inside a single Clarabel call — it runs 4 separate single-threaded Clarabel instances concurrently. The joining loop in joining.rs receives the final SolverResult from run_slp9_loop; it does not observe the order in which backtrack candidates were evaluated. Determinism is preserved: given the same SolverResult from the same best-accepted candidate, the joining sweep produces the same output.

The joining loop's early-bail depends on (v_start, v_end, a_start) propagation across chains (joining.rs:8-35), not on the internal SLP9 backtrack order. Single-segment batches have no joining sweep at all (exchange_follower_tails returns immediately for n_chains < 2, joining.rs:139). So for the primary bottleneck (single-segment replans) the joining determinism concern is moot.

WHAT IS NOT IN SCOPE (and why not)

slp_solve_chain (the path-jerk SLP, solver.rs:938-1049) is a sequential fixed-point loop where each iteration's cuts depend on the previous iterate. It cannot be trivially parallelized without changing the algorithmic structure. However, the profile findings (slp-fallback Finding 1 and slp-cost profiling) show the axis-jerk SLP9 is the dominant cost contributor at high entry velocity — the path-jerk SLP typically converges in fewer iterations at the operating point. So not parallelizing slp_solve_chain is acceptable as a first intervention.

The ToleranceMode::Auto fast/tight double-solve is also sequential but is a correctness invariant (see trajectory-invariants report Attack 1). It is not touched.

SECOND-ORDER: PARALLEL PATH-JERK + AXIS-JERK PASSES

An orthogonal opportunity (separate from the backtrack fan-out): ToleranceMode::Auto runs call_slp(1e-5) then, on failure, call_slp(1e-8). On the segment types that trigger the slow path (high entry velocity, axis-jerk infeasibility at 1e-5), both passes are guaranteed to run. These two calls are not data-dependent until one of them succeeds. They COULD be launched in parallel on two cores and the result from whichever returns Converged first accepted. However, this has a trajectory-safety subtlety: the trajectory-invariants report (Attack 1) flags that forcing 1e-5 unconditionally is not safe; the Auto fallback is itself an invariant. Racing 1e-5 vs 1e-8 and taking the first to converge does not violate this — it accepts either, and neither produces a trajectory outside the final verify::check_chain acceptance bars. This is a separate, additive opportunity using 2 of the 4 cores at the schedule_chain_with_tolerance level, but it requires restructuring mod.rs:142-153. Flag as follow-on.

CSC MATRIX CONSTRUCTION OVERHEAD (can be attacked in parallel with backtrack change)

The profile (socp-cost) identifies that each solve_with_cuts_and_trust_region call rebuilds the CSC matrix by scanning all n_vars=454 entries of every dense a_rows vector (solver.rs:402-410), reading ~908,000 scalars per call with the working set 13x larger than Pi5 L2 (512KB). This is a serial overhead within each call; it is not reduced by parallelizing across calls. However:

  - The base bundle's a_rows (bundle.a_rows) is built once in build_chain and is shared (read-only) across all 4 parallel thread instances. No locking needed.
  - Replacing Vec<Vec<f64>> with a COO (coordinate list) representation stored in ConstraintBundle — i.e., carrying the pre-identified (row, col, val) triples directly — would drop the per-call scan cost from O(n_rows * n_vars) = O(N^2) to O(nnz) = O(N). At N=92, n_rows~1400, n_vars=454: from 636,000 reads to ~3*1400=4,200 reads. This is a 150x reduction in the CSC construction hot path, entirely orthogonal to but composable with the parallel backtrack fan-out.


- **Trajectory impact (proposer):** 
None. The parallel backtrack fan-out is trajectory-quality neutral by construction. Justification:

1. The SOCP is a convex program. Each thread invocation of solve_with_cuts_and_trust_region is given the same (bundle, cuts, b_bar, a_bar, tol). Clarabel's IPM converges to the same ε-optimal primal-dual point regardless of thread scheduling order. The result is deterministic given the inputs.

2. The acceptance criterion is unchanged: select the smallest backtrack index k whose candidate satisfies (status not Infeasible/MaxIter) AND (cand_ratio < best_ratio). The sequential loop already uses this criterion; the parallel version applies the same rule post-hoc to the returned set. Outcome: identical to sequential.

3. N, constraint blocks, SLP outer iteration budgets, SLP9_EPS_FEAS, verify::EPS_FEAS, verify::EPS_FEAS_JERK, and ToleranceMode::Auto are all unchanged. The trajectory-quality invariants from the trajectory-invariants profile (checklist items 1-7) are fully preserved.

4. The one nondeterminism concern — when multiple k values simultaneously satisfy cand_ratio < best_ratio — resolves to taking the smallest-k acceptor (i.e., smallest trust-region shrinkage applied). This is the same choice the sequential loop would make (it breaks on first k that accepts). Taking the largest-k acceptor would be a different and weaker result, but smallest-k is what is implemented.

5. Fail-loud (SegmentLate) is not touched. Latency is reduced; the abort threshold is not changed.



### Persistent-Solver Fat-Matrix: AMD Symbolic Reuse Across All SLP Iterations  ·  effort=medium  ·  verdict=**keep_with_changes**  ·  trajectory_safe=True

- **One-liner:** Pre-allocate a max-capacity CSC matrix that subsumes every possible SLP cut and TR row, build one Clarabel DefaultSolver per chain solve, and use update_A + update_b for every subsequent SLP outer iteration instead of DefaultSolver::new — preserving the AMD symbolic factorization and eliminating the largest single source of redundant work.

- **Proposer speedup:** 
**Base estimate for the 867ms observed case:**

Per-call breakdown at n=92 on Pi5 (grounded in profile [socp-cost]):
- Current per-call cost: ~5-8ms (50 IPM iters × ~0.1-0.15ms per iter) + ~0.5-1ms AMD+symbolic + ~0.5-1ms dense CSC scan = ~6-10ms/call
- After fat-matrix: ~5-8ms (IPM unchanged) + ~0.05ms (update_A O(nnz) patch) = ~5-8ms/call
- Per-call speedup: 15-30% reduction

Over the full 100-200 call budget for the 867ms case:
- Current: 867ms
- After: 867ms × 0.75 = ~650ms (Pi5 estimate), or 867ms × 0.70 = ~607ms (optimistic, if CSC-native conversion also eliminates allocation pressure)

**Pi5 estimate: 600-700ms per worst-case mid-stream segment** (down from 867ms). Still over the 250ms LEAD budget for a chained segment.

**Pi4 estimate:** Pi4 is ~1.3-1.5x slower per core than Pi5 (A72 vs A76). The dense CSC scan is more expensive on A72 due to weaker out-of-order execution and same L2 cache pressure. Proportional improvement: similar percentage reduction. Pi4 absolute: ~900-1100ms (down from ~1150-1300ms estimated from Pi5 × 1.35).

**Why this alone is not sufficient for real-time:** The 50ms warn budget requires approximately 8-12x speedup over the current 867ms. This change delivers ~1.25x. It is a necessary ingredient in a stack of improvements, not a standalone solution. The remaining gap requires either (a) reducing SLP outer iteration count (the dominant cost driver per profile [slp-fallback] Finding 1-3), or (b) a specialized solver that exploits the tridiagonal KKT structure to reduce per-IPM-iteration cost by 5-10x. This change is the lowest-risk foundation that should land first: it eliminates structural waste, reduces allocation pressure, and makes the per-call cost more predictable before further optimization.

**Stack context:** Combined with [slp-fallback] Finding 3 (reducing max_iter for TR subproblems from 1000 to 200, which bounds the 4-call backtrack waste at ~80% of potential), this change together would bring the 867ms case to approximately 400-500ms on Pi5 — still 2x over budget. Reaching real-time requires additionally either improving SLP9 convergence (better warm-iterate initialization, tighter TR schedule), a coarse-to-fine grid approach, or a custom band-structured SOCP solver.


- **Verifier realistic speedup:** ~1.10–1.20x on Pi5 (867ms → ~720–790ms), not the claimed 1.25–1.43x. On Pi4 the relative gain is similar. This alone does not approach the budget; it is a foundation-layer optimization, not a solution.

- **Verifier reasoning:** Mechanism is SOUND against the real code. I verified against the vendored Clarabel 0.11.1 source: `update_A` (data_updating.rs:121-132) calls only `_update_values` on precomputed KKT positions and does NOT re-run AMD or symbolic factorization (directldlkktsolver.rs:195-197) — so the AMD/symbolic reuse claim is real. `presolve_enable` defaults to true (settings.rs:185) so the current code does run presolve and must turn it off; `input_sparse_dropzeros` already defaults false. `solve()` unconditionally calls `default_start()` (solver.rs:268) — the proposal honestly admits there is NO IPM iterate warm-start, so the only savings are AMD + symbolic + the dense CSC rebuild scan. That honesty is to its credit and is why this is not snake-oil.\n\nBut the speedup estimate is optimistic and the proposal under-weights two effects that erode it. FIRST, the dominant per-call cost is the per-IPM-iteration numeric refactor (`regularize_and_refactor` → `ldlsolver.refactor`, called once per IPM iter, 15-30x/solve; solver.rs:350-351, directldlkktsolver.rs:253). `update_A` does nothing for this — it is untouched, as the proposal states. The eliminable overhead (AMD+symbolic ~0.5-1.5ms + dense scan ~0.6-1.3ms) is only ~19-48% of a ~5.8ms/call, and the proposal picks the top of that range. SECOND and more seriously: the fat matrix carries the MAX cut count as structural zeros, and with input_sparse_dropzeros=false those zeros are real positions in the symbolic pattern, so QDLDL factorizes them on EVERY IPM iteration of EVERY call. In early SLP iterations where few cuts are active, the per-call matrix would normally be much smaller; the fat matrix inflates the dominant refactor term. This can partly or wholly cancel the AMD/symbolic saving. THIRD, presolve-OFF (mandatory) taxes the dominant term 2-5%, and stale equilibration (update_A reapplies the original scaling, data_updating.rs:226) can raise IPM iteration count precisely on the hard mid-stream solves that motivate the work.\n\nTrajectory safety: GENUINELY preserved. The feasible set, objective, grid N, SLP iteration budget, beta count, ToleranceMode::Auto fallback, verify-gate (EPS_FEAS/EPS_FEAS_JERK), and fail-loud SegmentLate are all untouched. Inactive zero-coefficient Nonneg rows (0·x ≤ 0) are redundant constraints satisfied by every feasible x including the optimum — provably no change to the optimal value. A different AMD permutation produces a numerically equivalent factorization. The one real numerical hazard (stale equilibration changing IPM convergence) affects solve TIME and solve SUCCESS, not the time of an ACCEPTED trajectory — and any infeasible/diverged result is caught by the verify gate and the Auto fallback, failing loud rather than shipping a slower trajectory. So it cannot sneak in a measurably slower trajectory.\n\nNet: a real but modest, foundation-layer optimization whose headline number is inflated and whose fat-matrix half carries unacknowledged fill-cost risk. The CSC-native-build half is the safe, decoupled, must-do part. Keep with the changes above; do not ship the fat-matrix half on the strength of the proposal's arithmetic alone — benchmark the fill penalty and equilibration drift first. By the non-negotiable, this is in the trajectory-safe speedup class, so it is allowed; it just is not the 8-12x the budget needs.

- **Required changes:** 1) Separate the two wins. The CSC-native build of `ConstraintBundle` (killing the `vec![0.0; n_vars]`-per-row dense representation in constraints.rs:352-360 and the O(n_rows·n_vars) scan in solver.rs:402-410) is a strictly-safe, lower-risk, presolve-preserving win on its own and should land FIRST and INDEPENDENTLY of the persistent-solver. Measure it in isolation — it likely delivers most of the realizable gain (~60-120ms) without any of the fat-matrix risks. 2) For the persistent-solver fat-matrix, do NOT pre-allocate max-capacity axis-jerk rows (2*n*n_axes). The structural zeros are factorized on EVERY IPM iteration (the dominant cost), so a near-empty cut block in early SLP iterations pays full fill cost. Either (a) rebuild the solver only when the active-cut sparsity pattern changes (amortizing AMD across the iterations where the cut SET is stable, which is the common case once SLP9 settles), or (b) prove empirically the fill penalty is below the AMD/symbolic saving. 3) Benchmark per-IPM-iteration cost with presolve OFF vs ON on the actual kalico SOCP before committing — presolve-OFF taxes the dominant term. 4) Address stale equilibration: `update_A` reapplies the ORIGINAL equilibration computed at construction from the all-cuts-inactive matrix; as large-coefficient cuts activate, conditioning degrades and IPM iteration count can rise. Verify iteration counts do not increase on mid-stream fixtures (this is the exact regime that produced 867ms). 5) Keep the per-tolerance-pass fresh construction (Auto 1e-5 then 1e-8) — confirmed correct, tolerance changes are rejected by validate_as_update.

- **Mechanism:** 
**Root cause recap (from profile [socp-cost] and [slp-fallback]):**

The 867ms solve at n=92 runs approximately 100-200 cold-start Clarabel invocations. Every call to `solve_with_cuts_and_trust_region` (`solver.rs:578`) constructs `DefaultSolver::new(...)`, which internally runs:
1. AMD ordering of the KKT system (O(n_vars + n_rows) combinatorial work, ~0.5-1ms at n=92 dimension ~3600)
2. Symbolic QDLDL factorization: determines L sparsity pattern and allocates fill-in arrays (~0.5ms)
3. Numeric QDLDL factorization (refactor): fills L numerically (~2-5ms per IPM NT-scaling update, called 15-30 times per Clarabel solve)
4. The `solve_with_cuts_and_trust_region` dense CSC scan: iterates all `n_vars=454` entries of every `a_row`, scanning 7 MB of zeros per call

Items 1, 2, and 4 are **pure overhead that does not advance the optimization**. Items 1+2 are a one-time O(n) cost that is currently paid 100-200 times per segment. Item 4 is the dense `Vec<Vec<f64>>` representation in `ConstraintBundle.a_rows` (`constraints.rs:19`).

**Why AMD is reusable across SLP iterations:**

The path-jerk SLP (`slp_solve_chain`, `solver.rs:938-1048`) clears all cuts each iteration (`cuts.clear()` at line 971) and re-adds exactly `n-2` path-jerk cuts with updated linearization weights (from the current iterate `b_bar`). The cuts always touch the same three columns per row (the stencil triple `idx=[i-1,i,i+1]`) and each row is a Nonneg cone row. The **sparsity structure** of these cut rows is identical across all path-SLP outer iterations; only the NZ values (the weights and RHS) change as `b_bar` evolves.

The axis-jerk SLP9 (`run_slp9_loop`, `solver.rs:1063-1213`) generates cuts only for **violating** grid points — a data-dependent subset of [0,n). The count varies (0 to N*n_axes per iteration). Trust-region rows are exactly `2*(n_grid-2) + 2*n_grid` when a TR is present — a fixed count.

**The fat-matrix design:**

Build one CSC A matrix that pre-allocates the maximum possible number of rows for the entire SLP cascade:
- Base rows from `build_chain`: fixed, call this `n_base_rows`
- Path-jerk cut block: exactly `2*(n-2)` Nonneg rows, one per interior grid point (always the same sparsity — 3 NZ per row touching b-cols [i-1, i, i+1])
- Axis-jerk cut block: `2*n*n_axes` Nonneg rows (max one cut per grid point per axis; 4 NZ per row touching 3 b-cols and 1 a-col)
- Trust-region block: `2*(n-2) + 2*n` Nonneg rows (2 per interior b, 2 per a)

Total added rows at n=92, 2 XY-axes: `2*90 + 2*92*2 + 2*90 + 2*92 = 180 + 368 + 180 + 184 = 912` pre-allocated rows.

Inactive rows are represented as **zero-valued rows** in the CSC matrix (structurally present NZ entries with value 0.0 and b_rhs 0.0 with a Nonneg cone, which is satisfied by any non-negative x and is thus numerically inactive). The cone list is fixed: the Nonneg cone dimension at the cut block is always the max count, regardless of how many cuts are active.

The `DefaultSolver` is constructed **once** at the start of `slp_solve_with_axis_jerk_chain_inner`:
- `presolve_enable: false` (required for `update_A`/`update_b` to work)
- `input_sparse_dropzeros: false` (required — we have structural zeros for inactive cuts)

Then each SLP outer iteration (path-SLP or axis-SLP9) calls:
1. `solver.update_A(&new_a_csc)` — patches the NZ values of the active cut rows in-place; zero-values the inactive rows' NZ entries. This calls `_update_values` → `factors.update_values(index, values)` on the QDLDL internal permuted copy. No symbolic work.
2. `solver.update_b(&new_b_rhs)` — patches the RHS vector in-place
3. `solver.solve()` — runs `default_start()` (initializes primal/dual from the current KKT NT-scaling, which was just updated by the previous iteration's cone information) then IPM

**What is preserved across iterations:**
- AMD permutation vector (`perm` in `QDLDLFactorisation`) — computed once in `new()`, reused every `refactor()`
- L sparsity pattern (`Lp`, `Li` arrays in QDLDL) — same symbolic structure, same fill-in positions
- KKT structure assembly (`LDLDataMap`, `map.A` index array) — reused by `_update_values` for O(nnz) numeric patching

**What is NOT preserved (intentionally):**
- Primal/dual iterates — Clarabel calls `default_start()` at the top of every `solve()`. There is no warm-start API for the IPM iterate itself in Clarabel 0.11. The benefit here is purely from eliminating AMD+symbolic factorization overhead and the dense CSC construction scan.
- NT scaling matrices — recomputed each IPM iteration from the new primal/dual cone state

**Quantified savings from eliminating AMD+symbolic phase:**

Each `DefaultSolver::new()` at n=92 (KKT dimension ~3600): AMD ordering is O(n*log(n)) ≈ O(3600*12) ≈ 43,000 ops; symbolic QDLDL is O(n + nnzL) ≈ O(3600 + 8,000) ≈ 11,600 ops. At Pi5 throughput these together cost approximately 0.3-0.8ms per call. Over 100-200 calls: 30-160ms.

More significant is the **dense CSC construction** in `solve_with_cuts_and_trust_region` lines 398-410 (the outer loop over `bundle.a_rows`, each of length `n_vars=454`). At n_base_rows ≈ 1400 rows × 454 columns × 8 bytes = 5.1 MB scanned per call, 100-200 calls = 510MB-1GB of memory reads to skip zeros. These are cache-miss-heavy (Pi5 L2 is 512KB/core). The fat-matrix approach replaces this with a pre-built CSC of known NNZ — the base rows are built once into CSC form, and the cut/TR rows are updated in O(nnz_cuts) time (touching only the ~3-4 NZ entries per cut row). This eliminates the entire dense scan.

**Combined expected reduction in overhead per segment:**
- Dense CSC construction scan: eliminated entirely. Was 45-90M cache-miss reads. Saves approximately 50-200ms over a full 100-200 call solve.
- AMD+symbolic phase: saves 30-160ms.
- Total overhead reduction: 80-360ms per segment solve.

**Effect on IPM iteration count:** None directly. The AMD symbolic reuse does not change how many IPM iterations Clarabel needs per call (that is determined by problem conditioning and the quality of the initial point from `default_start()`). The speedup is purely from eliminating the factorization setup overhead that currently dominates per-call cost.

**Expected solved time for the 867ms case:**
- Current: ~150 Clarabel calls × ~6ms average (including setup) = 900ms
- After fat-matrix: ~150 Clarabel calls × ~4ms average (setup eliminated) + 0.3ms × 150 (update_A cost) = 645ms

That is a ~25% reduction in total solve time from this change alone, before any other optimization. The primary bottleneck shifts from setup overhead to pure IPM iteration cost (the numeric refactor and iterative refinement).

**Trajectory safety:** Complete. The fat-matrix change does not alter the feasible set, the objective, the SLP convergence criterion, the grid N, or the acceptance tolerances. The inactive cut rows (zero rows in Nonneg cone) are mathematically equivalent to not having those rows — every primal x satisfies `0·x ≤ 0`, i.e. they are redundant constraints that the solver treats as non-binding. The AMD ordering may differ from the per-call case (AMD on a slightly larger matrix), which is permissible since AMD is a heuristic and both orderings produce correct factorizations. The verify gate (`EPS_FEAS=2e-3`, `EPS_FEAS_JERK=5e-2` in `verify.rs`) is unchanged and still the final correctness authority. The ToleranceMode::Auto double-solve fallback is preserved.

**Why this does not violate the non-negotiable constraint:** The change does not alter the SLP iteration budget, the acceptance criteria, the grid N, or the objective. It only eliminates redundant computation that was never advancing the optimization. It cannot produce a trajectory with more time than the current implementation — it produces the same result faster.

**Complementary second win: replace ConstraintBundle.a_rows with native CSC from the start:**
The `ConstraintBundle.a_rows: Vec<Vec<f64>>` field (`constraints.rs:19`) stores a dense `n_rows × n_vars` matrix where n_rows ≈ 1400 and n_vars = 454 at n=92. The `push_row` closure (`constraints.rs:352-360`) allocates a full `vec![0.0; n_vars]` for each row even though only 1-4 entries are non-zero. Converting `build_chain` to emit a sparse COO or CSC directly (tracking NZ entries only) eliminates the 7MB dense allocation and the O(n_rows × n_vars) scan. This is a prerequisite for the fat-matrix approach since the fat-matrix CSC is built once in CSC-native form. The change to `ConstraintBundle` is contained to `constraints.rs:build_chain` and the CSC construction in `solver.rs:398-410`.


- **Trajectory impact (proposer):** 
None. The fat-matrix design preserves the problem exactly:

1. Inactive cut rows (zero-coefficient Nonneg rows with RHS 0) are redundant constraints satisfied by all x, including the optimal x. They do not change the feasible set or optimal value. This is provable: a row 0·x ≤ 0 in a Nonneg cone is equivalent to 0 ≤ 0, always satisfied.

2. The AMD ordering on the fat matrix (slightly larger KKT due to pre-allocated cut rows) is a different heuristic permutation than on the per-call matrices. Both are valid AMD orderings of SPD/quasi-definite KKT systems, and both produce correct LDL factorizations. The resulting Clarabel iterates converge to the same ε-optimal KKT point (the convex SOCP base relaxation has a unique optimal solution under strong duality, which holds for well-posed TOPP problems).

3. The SLP outer-loop convergence criterion (`SLP_EPS_FEAS=5e-2`, `SLP9_EPS_FEAS=5e-2`) is unchanged. The loop terminates when the same violation ratio drops below the same threshold.

4. The final acceptance gate (`verify::EPS_FEAS=2e-3`, `EPS_FEAS_JERK=5e-2`) is unchanged. The verify pass runs the same check on the final SolverResult regardless of how the solver internally managed its matrix.

5. The ToleranceMode::Auto double-solve is preserved. The persistent solver is discarded after the full SLP cascade (fast or tight pass), not reused across the fast→tight retry. Each tolerance pass still gets a fresh AMD ordering.

6. The fail-loud SegmentLate abort is unaffected — it fires based on wall-clock elapsed, not solver internal state.



### SLP-Jerk-Linearization-Point-Seeding  ·  effort=medium  ·  verdict=**keep_with_changes**  ·  trajectory_safe=False

- **One-liner:** Seed every SLP9 axis-jerk iteration from a velocity-scaled feasible point instead of the raw SOCP output, eliminating the TrFloorStall restoration loop and cutting observed Clarabel call count from ~100-170 to ~15-30 for mid-stream curves.

- **Proposer speedup:** 
The observed 867ms case corresponds to approximately 100-170 cold-start Clarabel calls at ~5-8ms each on Pi5. After seeding:

Path-SLP (slp_solve_chain): unchanged, still ~10-20 outer iterations = 10-20 Clarabel calls at ~5ms each = 50-100ms. This stage is not affected by seeding.

SLP9 first pass: starts from axis-jerk-feasible point (ratio ~0.9), converges in ~5-10 outer iterations, each needing 1-2 Clarabel calls (first backtrack accepted) = 5-20 Clarabel calls = 25-100ms.

TrFloorStall restoration: eliminated entirely (feasible start does not stall the TR). Saves the entire second run_slp9_loop: 30 outer iters * up to 5 calls = 0-150 calls saved = 0-750ms saved.

ToleranceMode::Auto tight retry: eliminated (SLP now returns Converged, fast pass succeeds). Saves a full second pass: 10-20 path-SLP + 5-20 SLP9 calls = 75-200ms saved.

Projected total Clarabel calls after seeding: ~20-40 at ~5-8ms each = 100-320ms on Pi5, vs 867ms observed.

Projected solve time on Pi5: 120-300ms for the worst-case mid-stream 46mm G5 at cruise speed. This is still above the 50ms warn threshold but below the 250ms LEAD budget for chained segments. The SegmentLate abort (triggered at 867ms, which exceeded 250ms LEAD by 617ms) would not fire.

Pi4 (A72 at 1.8GHz, ~1.5-2x slower than Pi5): 180-600ms projected. This is tight for chained mid-stream segments — the 250ms LEAD budget would still be at risk at the top of the projected range. Pi4 benefit is ~2-5x over the current failure (867ms Pi5 maps to ~1.7s Pi4), bringing it to 600ms vs 1700ms — better but not definitively real-time. The secondary fixes (dense scan elimination, TR max_iter cap) would each add another ~1.5-2x and would be needed to land Pi4 real-time for extreme cases.

The cost growth pattern (63ms -> 93ms -> 99ms -> 190ms -> 867ms) occurs because each successive curve enters with higher v_start, increasing initial_max from the SLP path result. With seeding, curves that previously hit initial_max >> 1 are now seeded to ratio ~0.9 at negligible cost, so the growth curve flattens. The 5th curve that caused 867ms should be similar cost to the 3rd (99ms) under this fix.

Summary: ~3-7x speedup on Pi5 for the failing case. Pi4 needs seeding PLUS the TR max_iter cap to be safe.


- **Verifier realistic speedup:** Pi5: ~2-3x on the failing mid-stream case (867ms -> ~300-450ms), NOT the claimed 3-7x. Pi4: ~1.5-2.5x, not real-time. The seeding eliminates the restoration pass and likely the Auto retry, but the proposal over-counts what is saved (see reasoning) and ignores that path-SLP and the first feasible-descent are unchanged.

- **Verifier reasoning:** The mechanism is real and the diagnosis is correct: the 867ms pathology is the TrFloorStall -> restoration -> second 30-iter run_slp9_loop, plus the ToleranceMode::Auto tight retry. Seeding from damp_interior_a before the first run_slp9_loop, reusing the same restoration machinery that already exists (lines 1467-1486), is a sound, low-cost idea that attacks the right bottleneck. As an ENGINEERING latency fix it has merit.

But the proposal's central trajectory-safety argument is FALSE against the real code, and this is the non-negotiable.

1. THE LOAD-BEARING CLAIM IS WRONG. The proposal states: "trajectory time is determined by b, not a... If b is unchanged, the trajectory time is identical... [seeding] finds the SAME local optimum." This is false. damp_interior_a leaves b on the SEED, but run_slp9_loop does NOT hold b fixed — every solve_with_cuts_and_trust_region call re-solves a convex subproblem that re-optimizes BOTH b and a under the cut set and trust region (lines 1143-1151). The delivered b is whatever the descent lands on, not the seed's b. So "b is unchanged" is simply not true of the output.

2. run_slp9_loop HAS NO TIME OBJECTIVE. It is a pure feasibility descent: it accepts only on cand_ratio < best_ratio (line 1159) and returns at the FIRST iterate with best_ratio <= 1.05 (line 1186). Different starting points (seeded vs the un-seeded infeasible start that goes through restoration) hit the 1.05 threshold at DIFFERENT b-profiles with different profile_time. There is no mechanism forcing the two to agree. The code's own comment at line 1309 admits the feasibility-descent endpoint "can sit a few percent below the local optimum." A few percent IS a measurably slower trajectory — exactly what CLAUDE.md forbids.

3. POLISH DOES NOT RUN IN THE FAILING CASE. The only thing that re-maximizes speed after the descent is polish_windowed, and it runs ONLY when windows is Some (line 1372). In the exact 867ms scenario — window_segments=1, no follower — call_slp passes windows=None (mod.rs line 137), so polish is skipped entirely. The delivered trajectory is the raw descent endpoint. Therefore the seeded and un-seeded descents can deliver different, and specifically the seeded one can deliver a slower, profile, with NO polish to recover it. The proposal asserts "verify::check_chain is the authoritative gate" — but verify checks FEASIBILITY (ratio <= EPS), not OPTIMALITY. A slower-but-feasible profile passes verify and ships. verify cannot catch a trajectory-time regression.

4. THE PROPOSED MITIGATION IS COUNTERPRODUCTIVE. SLP9_MIN_IMPROVING_ITERS (force >=5 iters "to let the optimizer improve") cannot improve trajectory time because the loop has no time objective — extra iterations only descend ratio FURTHER, i.e. damp MORE, i.e. produce a SLOWER trajectory. The mitigation makes the safety problem worse.

5. SPEEDUP IS OVER-COUNTED. The proposal keeps path-SLP (50-100ms) and the first feasible descent, then claims the rest collapses. But it double-counts: it credits removing "up to 150" restoration calls AND "100-200" Auto-retry calls as if independent, when in the observed case these overlap (the retry re-runs the same stalling structure). Realistic Pi5: path-SLP unchanged + one converging SLP9 pass = ~300-450ms, i.e. ~2-3x, not 3-7x. Still above the 50ms warn; near the 250ms LEAD ceiling — marginal, not safe. Pi4 stays out of real-time.

REQUIRED CHANGES to make it trajectory-safe and acceptable: (a) Run a time-maximizing polish unconditionally after the seeded descent in the windows=None case (extend/reuse polish_windowed's logic without follower cuts), and BENCHMARK seeded-vs-unseeded profile_time on the measured 5-curve sequence with an assert that seeded time <= unseeded time within tolerance, as a gating test — not an afterthought. (b) Drop the counterproductive SLP9_MIN_IMPROVING_ITERS guard. (c) Re-baseline the speedup claim to ~2-3x Pi5 and stop asserting Pi4 real-time. Without (a) this approach KNOWINGLY risks shipping a measurably slower trajectory, which the charter rejects.

The TR max_iter cap (200 for TR subproblems, 1000 retained for trust_region=None) is sound and trajectory-neutral — keep it, but confirm the CL-2024 counterexample at solver.rs:558 is a base-SOCP case, not a TR subproblem. The sparse-CSC refactor is genuinely trajectory-neutral and worth doing independently.

- **Required changes:** 1. Unconditional time-maximizing polish after the seeded descent for the windows=None single-segment path (the failing case currently has NO polish). 2. Add a gating regression test asserting seeded trajectory time <= un-seeded baseline within tolerance on the 5-curve mid-stream sequence — trajectory-time, not just feasibility. 3. Remove the SLP9_MIN_IMPROVING_ITERS guard (it forces extra damping = slower trajectory). 4. Re-baseline speedup to ~2-3x Pi5; drop Pi4 real-time claims. 5. For the TR max_iter cap, verify the CL-2024 counterexample (solver.rs:558) is base-SOCP not TR before applying.

- **Mechanism:** 
The profile root cause is precise: the 867ms solve is not slow per Clarabel call — it is slow because there are ~100-170 cold-start Clarabel calls, the majority from run_slp9_loop burning through its backtrack cascade and then triggering the restoration branch which reruns the entire 30-outer-iter loop.

WHY THE LOOP EXPLODES AT MID-STREAM SPEED

When a curve enters with v_start ~ 4000 mm/s the pinned boundary b_0 = v_start^2 ~ 1.6e7. The path-jerk SLP (slp_solve_chain, solver.rs:938) converges in ~10-20 iterations to a profile that satisfies |b''| * sqrt(b) <= 2*J_path, returning SlpOutcome::Converged. That result is then handed to run_slp9_loop (solver.rs:1063) as current_start. The axis-jerk ratio evaluated at that point (max_axis_ratio_chain, solver.rs:1403) is typically >> 1 for high-speed entries because the jerk term |cppp*b^(3/2) + 3*cpp*a*sqrt(b) + cp*b''*sqrt(b)/2| is roughly proportional to sqrt(b) and b is large at the entry. The SLP9 trust-region radius rho_b starts at 0.10 (SLP9_RHO_B_INIT), meaning the TR cone allows b to change by at most 10% of b_bar in each direction per step. At b_0 ~ 1.6e7 that is a 1.6e6 mm^2/s^2 margin — large in absolute terms but the axis-jerk improvement per step is small when the linearization error is large (the derivative dj_axis/db involves b^(1/2) in several cross-terms, and the Taylor remainder at high b causes the candidate to barely improve max_axis_ratio). The backtrack loop (solver.rs:1137-1163) fires all 4 levels (rho*1, rho*0.5, rho*0.25, rho*0.125) and every candidate either gets Infeasible from Clarabel or fails the cand_ratio < best_ratio guard. Then the TR-free fallback solve (solver.rs:1166) is tried. Each of these is a cold-start Clarabel call. After enough consecutive rejections rho_b halves until it hits SLP9_RHO_B_MIN = 0.005 (solver.rs:829) and TrFloorStall fires (solver.rs:1197-1203). That triggers damp_scale_for_axis_feasibility (solver.rs:1270), which runs 24 bisection iterations each calling max_axis_ratio_chain (O(N) scalar work, negligible), then damp_interior_a produces a feasibility-restored starting point, and run_slp9_loop is called again from scratch with SLP9_RESTORE_RHO_B_INIT=0.50, SLP9_MAX_RESTORATIONS=1. The second pass costs the same structure: 30 outer iters * up to 5 Clarabel calls = up to 150 more calls. The restore branch is the multiplier that turns a bad-but-bounded solve into the 867ms pathology.

WHAT THE SEEDING IDEA CHANGES

The damp_interior_a / uniform_damp_for_feasibility functions already contain the insight: if you analytically shrink the profile to a point where the axis-jerk constraint is nearly satisfied, the SLP9 loop converges quickly. The problem is that today this damped-feasible point is only computed AFTER TrFloorStall fires — i.e., after 30 outer iterations and up to 150 Clarabel calls have already been spent.

The proposed change is to compute a feasibility-seeded starting point BEFORE the first run_slp9_loop call, as the initial current_start passed into it.

Concretely: after slp_solve_chain returns Converged, check initial_max (solver.rs:1403). If initial_max > SEED_TRIGGER_RATIO (proposed: ~3.0, tunable), instead of immediately passing path_result to run_slp9_loop, call damp_scale_for_axis_feasibility(path_result) to get a scale s that brings max_axis_ratio to SLP9_DAMP_TARGET_RATIO=0.9, apply damp_interior_a(path_result, s), and use that as current_start. The cost of computing this seeded point is: one damp_scale_for_axis_feasibility call (24 bisection steps of O(N) each = O(24N) scalar ops, no Clarabel calls) plus one damp_interior_a call (O(N)). Together these cost < 0.1ms at N=100 on Pi5.

The effect: run_slp9_loop receives a starting point where max_axis_ratio ~ 0.9 < 1.0 + SLP9_EPS_FEAS = 1.05. Wait — if initial_max is already <= 1.05 after seeding, the loop would return Converged at the very first iteration check at solver.rs:1127 (cuts.is_empty is true because build_axis_jerk_cuts_chain finds no violators above target_floor). The result is the seeded, damped profile — which is feasible for axis-jerk constraints.

But "feasible" here means axis-jerk ratio <= 1.0 + SLP9_EPS_FEAS = 1.05, not necessarily optimal. The profile has been damped (a values scaled down), meaning the trajectory is NOT at the velocity ceiling imposed by axis-jerk — it is conservative. This is where the non-negotiable mandate applies and must be checked carefully.

TRAJECTORY-SAFETY ANALYSIS OF SEEDING

The damp_interior_a approach scales down interior a values to reduce the 3*cpp*a*sqrt(b) cross-term. It does not reduce b (the velocity-squared profile). The path-jerk and centripetal constraints were already satisfied by slp_solve_chain's output (b is the SOCP-optimal b given those constraints). The axis-jerk constraint was violated at that point. Seeding from the damped point gives a profile with a < a_optimal at some interior points, which means the profile traverses some sections at lower acceleration than optimal — but the velocity profile b is unchanged.

This matters because the trajectory time is determined by b, not a: total_time = sum_k h_k / sqrt(b_k) (solver.rs:1294-1306). If b is unchanged, the trajectory time is identical to the path-result from slp_solve_chain. The SLP9 loop's job after seeding is to improve toward the true optimum by iterating toward the joint optimum of (b, a) with axis-jerk satisfied. Starting from a feasible a (less conservative than needed) and the path-optimal b, the loop now runs in the feasible interior rather than from outside the feasible set, and each outer iteration can improve the objective (find tighter b that still satisfies axis-jerk with the improved a). The convergence from a feasible starting point is dramatically better than from an infeasible one: trust regions can be accepted on the first backtrack level, no TrFloorStall fires, no restoration branch runs.

The trajectory-quality risk: if the SLP9 loop converges in ~5-10 iterations from the seeded start, it finds a local optimum of (b*, a*) that satisfies axis-jerk with EPS_FEAS=5e-2. This is the SAME local optimum the current code finds — after paying 10-50x more iterations to crawl from infeasibility. The final verify::check_chain call (mod.rs:156) is unchanged and still enforces EPS_FEAS on the delivered profile.

The only trajectory-quality risk is: the current code, starting from an infeasible point and iterating more, MIGHT find a different (better) local optimum than the seeded code starting from the feasible interior. For a convex problem this would not happen. The axis-jerk subproblem is non-convex (it appears in the SLP outer loop precisely because of the non-convex jerk terms). However, in practice the SLP9 loop converges monotonically: best_ratio is non-increasing across accepted outer iterations (solver.rs:1159: cand_ratio < best_ratio required for acceptance). Starting from ratio=0.9 vs ratio=10.0 does not change what the loop can find — both paths descend the same ratio landscape. The difference is that the infeasible start requires many rejected TR steps just to enter the feasible set, while the feasible start begins improving immediately.

WHICH REDUNDANT SOLVES ARE ELIMINATED

In the TrFloorStall + restoration path (the observed case at 867ms): the entire second run_slp9_loop pass after restoration is eliminated. That's up to 150 Clarabel calls. The first run_slp9_loop pass that would have converged quickly from a good starting point would still run, but now takes ~5-15 outer iterations (each needing 1-2 Clarabel calls vs 5) = ~10-30 Clarabel calls total. Net reduction: from ~100-170 calls to ~10-30.

SECONDARY WIN: FAST-PASS DOUBLE-SOLVE ELIMINATION

The ToleranceMode::Auto double-solve (mod.rs:145-152) fires when solver_outcome_is_success returns false. That happens when slp_outcome != Converged. The seeded start makes SLP9 converge instead of diverge or stall — changing the fast-pass outcome from Diverged/MaxIters to Converged. The tight 1e-8 retry pass is no longer triggered. At ~50 Clarabel calls per pass, this eliminates a full second solver pass worth of work. The combined effect is: seeding eliminates the TrFloorStall (removes up to 150 calls from pass 1) AND the tight retry pass (removes up to another 100-200 calls). Minimum realistic total drops from ~100-200 to ~10-25 Clarabel calls.

THE DENSE a_rows SCAN: INDEPENDENT SECONDARY FIX

The profile also identified the dense CSC matrix construction (constraints.rs:352-360 allocates n_vars-wide rows with 2-4 NNZ; solver.rs:402-410 scans every element per call). This is independent of the seeding fix but compounds it: at 150 Clarabel calls * 908k reads each = 136M reads. After seeding, at 25 calls * 908k reads = 22M reads — still a lot. Replacing the dense Vec<Vec<f64>> representation in ConstraintBundle with a sparse Vec<(usize, f64)> per row (storing only the NNZ column-value pairs) cuts the scan from O(n_rows * n_vars) to O(NNZ) per Clarabel call. This is a pure refactor with zero effect on solver output and is safe to combine with seeding. But it is a separate change and should be profiled after the seeding fix is in, since it may not be the bottleneck once the call count drops.

SEPARATE OPTIMIZATION: max_iter FOR BACKTRACK SUBPROBLEMS

The profile found that infeasible trust-region subproblems may run to max_iter=1000 before Clarabel reports MaxIterations. For TR-constrained subproblems that are expected to converge quickly or fail fast, a tighter max_iter (200) would bound the wasted work per rejected backtrack. After seeding, TR rejections are rarer, but this should still be changed: add a separate max_iter parameter to solve_with_cuts_and_trust_region that uses 200 for TR solves and keeps 1000 for the base SOCP and TR-free solves. The [PROFILE socp-cost] finding 3 explicitly flags this.


- **Trajectory impact (proposer):** 
None, with justification.

The seeded starting point enters run_slp9_loop with axis-jerk ratio ~0.9 (feasible, below the 1.0 + SLP9_EPS_FEAS = 1.05 convergence bar). The loop still runs normally: it builds cuts for any violator above target_floor, calls solve_with_cuts_and_trust_region, evaluates cand_ratio, and accepts or rejects based on best_ratio descent. The final acceptance bar is unchanged: SlpOutcome::Converged fires only when cuts.is_empty() (all ratios below target_floor). The verify::check_chain call in mod.rs:156 runs unconditionally after the SLP stack and enforces EPS_FEAS=2e-3 and EPS_FEAS_JERK=5e-2 on the delivered profile — this is the authoritative gate and is not touched.

The non-convexity risk (different local optimum from different starting point): the SLP9 loop descends a non-convex ratio landscape via a sequence of linearized convex subproblems. Starting from ratio=0.9 vs ratio=10.0, the descent follows different paths. The path from ratio=0.9 converges faster and to a feasible local optimum. Starting from ratio=10.0 the current code also (eventually) finds a feasible local optimum. Whether the two optima differ in trajectory time is the key question. In the axis-jerk SLP the objective is to reduce constraint violation, not to maximize trajectory time directly — the b-values (velocity profile) are what the time objective depends on, and b is also an optimization variable in each subproblem. The seeded a-damping does not fix b; the subsequent SLP9 iterations jointly optimize (b, a) to satisfy axis-jerk while maximizing b (the time-cost cones drive b upward). The optimizer is just starting from a better-conditioned feasible interior point where the curvature landscape of the non-convex problem is more accurately captured by the linearization. This is the standard argument for good initialization in non-convex SLP — it does not change the problem or the achievable local optimum quality.

The CLAUDE.md mandate: "The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory." The seeding is not a cheaper architecture — it is a better initialization for the same SLP algorithm. The algorithm (Consolini-Locatelli SOCP + SLP outer loop), the grid N, the tolerances, the acceptance bars, and the verify step are all unchanged.

The SEED_TRIGGER_RATIO threshold (proposed ~3.0) controls when seeding activates. If initial_max <= 3.0 (mild violation), the current code is already cheap and seeding adds unnecessary overhead (one damp_scale call). If initial_max > 3.0, seeding prevents the pathological restoration path. This threshold is a tuning parameter that has zero effect on the converged result — it only affects whether the initialization is seeded; the SLP loop runs the same convergence check regardless.



### Asynchronous plan-ahead solver with a lookahead-depth-sized lead buffer  ·  effort=large  ·  verdict=**reject**  ·  trajectory_safe=False

- **One-liner:** Move the SOCP solve off the receive thread into a decoupled solver stage that plans a configurable depth of curves ahead into a deeper committed buffer, with the idle-resume lead sized to worst-case curve solve time so playback never waits on a single solve.

- **Proposer speedup:** This does not reduce solve_us at all — it removes starvation by hiding solve latency behind committed playback. The relevant metric is the deadline-miss rate, not solve time. Pi5: with lookahead depth D=3, the committed runway is ~3 x 0.92s = 2.76s at the 50mm/s bench rate, which covers the observed 867ms worst-case solve with >3x margin (the 867ms solve no longer lands in the past because seg N+3's anchor is 2.76s ahead, not 0.25s ahead). Deadline misses for the measured chain go from 1-of-5 (the 867ms curve aborted) to 0, with ~1.9s of slack. Pi4 (8-10x slower per the brief, so worst-case solve could approach 1.0-1.7s on harder curves): needs deeper lookahead, roughly D=4-5 to retain ~1s of margin, costing more committed buffer memory but no algorithmic penalty. The hard limit is sustained throughput: if the AVERAGE solve_us exceeds the AVERAGE per-curve playback time, no finite buffer saves you and SegmentLate must eventually fire (correctly). At 50mm/s the average solve (63-190ms for the non-pathological curves) is well under the 0.92s playback, so steady-state is sustainable on Pi5; Pi4 steady-state depends on whether the per-solve speedups land. This buys headroom proportional to D x playback_time, converting transient solve spikes (the 867ms outlier) from fatal into absorbed.

- **Verifier realistic speedup:** Zero reduction in solve_us (admitted). For starvation: a one-time cushion at chain entry only; for steady-state mid-stream chaining (the actual 867ms regime) it provides no durable headroom and cannot prevent SegmentLate once average solve time approaches per-curve playback time, which is exactly the rising-cost trend the profile shows. Net realistic benefit: marginal, and it cannot be realized without either committing premature decel-to-rest stops (slower trajectory) or switching to the unimplemented, ~2.2x-slower-per-solve multi-segment SOCP.

- **Verifier reasoning:** The proposal's load-bearing claim — "this changes NOTHING about the trajectory; the solver computes byte-for-byte the same profile, only descheduled and given runway" — is false against the real code. Every append_and_replan solves the current uncommitted window with terminal_v=0.0 (state.rs:205); the decel-to-zero tail is deliberately held back (run_commit_and_dispatch -> commit_decel_to_zero) precisely so the growing window can be re-solved and premature stops avoided. "Plan-ahead depth D" can be realized only two ways, and both break the non-negotiable: (1) Commit each buffered curve's trajectory before the next is known — but with terminal_v=0 that commits a physical decel-to-rest at every buffered junction, i.e. a measurably slower trajectory, the exact thing CLAUDE.md forbids ("we do not give up trajectory time to make planning easier"). (2) Solve a multi-segment window so junction velocities are non-zero — but that is the deferred "multi-segment SOCP across the lookahead window" (pi5 doc line 293), which is ~2.2x slower single-threaded at equal resolution, is an unimplemented purpose-built formulation, and directly contradicts the proposal's own claim that rust/temporal is untouched. The proposal cannot have both "carried speed across buffered junctions" and "identical solver / no temporal changes."

The async-thread split does not remove the coupling, it relocates it. The planner's recv_timeout loop (planner.rs:471-519) IS the dispatch clock and can only commit already-committed output; advancing the buffer still requires committing the next solve, looping back to the terminal_v=0 problem. The 3-idle-cores observation is real but this approach explicitly cannot harvest them: the joining-loop determinism pin (max_threads=1 qdldl, prior verification invariant 6) and the proposal's own risk #2 force a single ordered solver thread — zero added parallelism.

The genuinely sound kernel is the lead-sizing-vs-re-anchor distinction: padding a future anchor with provable headroom is legitimate and differs from advancing a past anchor. But this only helps the idle-resume / chain-entry path (planner.rs:551 advance_idle), a one-time cushion. The observed 867ms case was mid-stream (sync_instant already set, no advance_idle), and the cost was escalating (63->93->99->190->867ms with rising entry velocity). A finite buffer absorbs one spike; it does not bend the trend, and the proposal concedes (risk #1) that once average solve exceeds average playback no buffer survives. So even the salvageable piece does not address the regime that actually failed.

Mechanism soundness against code: the commit-boundary, t_freeze, and retain(t_end > t_freeze) machinery exists as claimed, and the generation-counter / flush-cancellation concern is real. But the foundational premise (buffering is trajectory-neutral) is wrong, so the implementation sketch builds on a false floor.

- **Required changes:** To become trajectory-safe, the approach MUST drop the "identical solver / no temporal changes" claim and instead commit to a real multi-segment lookahead SOCP that carries non-zero junction velocities across the buffered window (the deferred pi5-doc item), then prove that formulation matches or beats the current per-segment-with-decel-tail trajectory — at which point it is a different, larger, and per-solve-slower piece of work, not a free descheduling. Absent that, any buffering must be restricted to segments whose committed boundary already carries the optimal junction velocity from a window solve; committing held-back decel-to-rest tails to fill the buffer is disallowed. The only independently safe sliver — growing the chain-entry advance_idle pad to cover measured worst-case solve time while the anchor is still in the future — should be split out as a small, separate change and evaluated on its own; it does not require the async architecture and does not address mid-stream steady-state starvation. Real per-solve speedups (warm-start, shared symbolic factorization, O(nnz) CSC build, opt-level) remain the actual levers and must land first; this buffering scheme cannot substitute for them.

- **Mechanism:** The profile shows the bottleneck is not the algorithm per se but the COUPLING between solve latency and the real-time deadline. Today (planner.rs:554-561) `append_and_replan` runs the full Consolini-Locatelli SOCP + SLP cascade SYNCHRONOUSLY on the receive thread, once per move, re-solving the entire `uncommitted_moves` window (state.rs:107-253). The profile [streaming-budget-parallelism] proves `window_segments=1` on every replan: between move arrivals the window drains to a single segment, so the planner is solving one curve at a time, blocking, with 3 of 4 Pi5 cores idle. The deadline (REPLAN_WARN_BUDGET_US=50ms from LEAD-SAFETY_MARGIN, planner.rs:19-23) is computed against a 250ms lead cushion. When the synchronous solve costs 867ms, the segment's scheduled MCU start lands 0.369s in the past and SegmentLate fires (planner.rs:147-149). The root cause of starvation is that NOTHING is buffered ahead: each curve's solve must finish inside the playback gap of the PREVIOUS curve, and at high entry velocity the SLP cost (profile [slp-fallback]: 100-200 cold Clarabel calls) exceeds that gap.\n\nThis approach attacks the coupling, not the per-solve cost. Three coordinated changes:\n\n(1) ASYNC SOLVER STAGE. Split the receive thread from the solver. The receive thread does only: enqueue the arriving CubicSegment into an unsolved-backlog VecDeque, and run_commit_and_dispatch of already-solved committed pieces (which is cheap — emit_us=729us per profile). A dedicated solver thread (or the existing fan_out_solves pool, parallel.rs:14) pulls from the backlog and runs append_and_replan, writing results into the committed buffer behind the `t_freeze` commit boundary (state.rs:114-119). The commit boundary already guarantees committed trajectory is never rewritten (state.rs:90 \"committed trajectory is never rewritten\") — this is the existing fail-loud-safe seam we build on.\n\n(2) PLAN-AHEAD DEPTH. Instead of draining the window to 1 segment, hold a target lookahead of D curves unsolved-or-in-flight and keep solving forward as long as backlog exists. This directly fixes `window_segments=1`: the solver always has work queued, so the per-curve solve latency is hidden behind the playback time of the D curves already committed ahead of it. At 50mm/s a 46mm curve plays for ~0.92s; a lookahead of D=2-3 curves gives 1.8-2.8s of committed runway, which absorbs even the 867ms worst-case solve without the segment landing in the past.\n\n(3) WORST-CASE-SIZED LEAD. The idle-resume cushion (planner.rs:551 `advance_idle(esc + LEAD)`) currently grants a fixed 250ms. Size the effective lead/buffer-depth to the measured worst-case curve solve (instrument solve_us, already carried in ReplanReport, state.rs:254). This is the load-bearing distinction the lens demands: sizing the lead to cover worst-case solve latency is NOT the forbidden silent re-anchor. The forbidden operation (CLAUDE.md, planner.rs SegmentLate) is taking a segment whose start time is ALREADY in the past and advancing it to hide lateness. Legitimate lead-sizing instead decides, BEFORE dispatch and while the anchor is still in the future, to place the anchor far enough ahead that the solve completes with margin. One re-anchors a committed past event (data loss, discontinuity); the other chooses a future anchor with provable headroom. The fail-loud check stays exactly where it is: if even the worst-case-sized lead is exceeded (solver genuinely cannot keep up), SegmentLate still fires — we have removed interactive starvation, not the guarantee.\n\nCRITICAL: this changes NOTHING about the trajectory the solver computes. The SOCP, the grid N, the SLP iteration budgets, the beta loop, ToleranceMode::Auto, and the verifier acceptance bars (verify.rs EPS_FEAS=2e-3 / EPS_FEAS_JERK=5e-2) are byte-for-byte unchanged. The solver still computes the same minimum-time profile over the same uncommitted window; we only change WHEN and on WHICH THREAD it runs and how much runway sits in front of it. Per the [trajectory-invariants] checklist this passes all 7 invariants: N preserved, acceptance bars unchanged, SLP/beta budgets untouched, Auto preserved, no silent re-anchor. The one invariant needing care is determinism (item 6): the async stage must keep the joining-loop's single-thread-qdldl pin and must solve segments in deterministic arrival order, which a single dedicated solver thread satisfies trivially.\n\nWhy this is the right lens for THIS profile: every other speedup (warm-start, parallelism-within-solve, coarser grid) attacks per-solve cost and is bounded by Amdahl plus the trajectory mandate. This approach makes the per-solve cost LARGELY IRRELEVANT to starvation — an 867ms solve is fine if 2+ seconds of committed curves sit ahead of it. It composes with the per-solve speedups (they shrink the required lookahead depth D) rather than competing with them.

- **Trajectory impact (proposer):** None. The solver computes the identical trajectory: same SOCP, same grid N (grid.rs unchanged), same SLP_MAX_OUTER_ITERS/SLP9 budgets, same beta loop, same ToleranceMode::Auto 1e-5/1e-8 fallback, same verifier bars (verify.rs EPS_FEAS=2e-3, EPS_FEAS_JERK=5e-2). Only the thread and the timing of the solve change, plus how much already-solved runway sits ahead of it. Justification against the [trajectory-invariants] 7-point checklist: (1) N preserved — yes; (2) acceptance bars unchanged — yes; (3) SLP iters not cut — yes; (4) beta iters preserved — yes; (5) Auto preserved — yes; (6) determinism — preserved by a single deterministic-order solver thread keeping the existing max_threads=1 qdldl pin; (7) no silent re-anchor — explicitly preserved, see the lead-sizing-vs-re-anchor distinction in the mechanism. The non-negotiable mandate is satisfied because we never choose a cheaper algorithm or a coarser discretization to make planning easier — the planning is identical, only descheduled off the critical thread and given more runway.


### Precompute CSC bundle + arc-length N correction  ·  effort=small  ·  verdict=**keep_with_changes**  ·  trajectory_safe=False

- **One-liner:** Eliminate the O(n_rows * n_vars) dense-scan bottleneck by building the static CSC representation of the base ConstraintBundle once, and fix the N over-inflation from the control-polygon length heuristic with an arc-length estimate.

- **Proposer speedup:** 
Pi5: 15-25% overall reduction in solve time for the 867ms worst-case segment (high entry velocity, mid-stream chain). 
- Fix A alone: ~50ms recovered from dense-scan overhead across ~100 Clarabel calls (the scan cost is currently ~545K reads per call × 100 calls = 54M reads at ~10ns/L2-miss ≈ 540ms equivalent overhead; this is an upper bound since the IPM itself also runs during those calls, but the matrix-assembly phase — separate from the factorization — is measurably wasteful).
- Fix B alone: ~13% per-call cost reduction from N=~106 to N=~92, saving ~80-115ms from the 867ms total.
- Combined: roughly 130-165ms saved → worst-case approaches 700-740ms on Pi5.

Pi4 (4x A72 @ 1.8GHz, ~1.5x slower than Pi5 per core): 867ms baseline would be ~1.3s on Pi4. Same 15-25% reduction applies → saves ~190-320ms on Pi4, bringing it toward ~1.0-1.1s. Still not real-time for the extreme high-velocity case, but Fix A + Fix B are cleanly prerequisite to any warm-start or parallelism approach that must still build and pass the CSC matrix.

These estimates are conservative (they do not account for cache-effect benefits propagating into the Clarabel QDLDL solve itself, which will see better L2 utilization with a smaller and more cache-warm column structure).

The per-segment budget at 50mm/s for a 46mm curve is ~920ms playback time with 250ms LEAD, so the target is sub-250ms per solve. These fixes bring the worst-case closer but do not close the gap alone — they are a necessary foundation layer, not the complete solution.


- **Verifier realistic speedup:** Fix A: ~3-6% on Pi5 / ~3-6% on Pi4 (≈25-50ms off 867ms), NOT 15%. Fix B: reject (unproven trajectory change). Combined defensible speedup: ~3-6%, contingent on splitting Fix A out and dropping Fix B.

- **Verifier reasoning:** I verified the mechanism against the real code. Fix A's target is real but the proposer mis-describes the current code. The dense scan exists: `push_row` (constraints.rs:354) allocates `vec![0.0; n_vars]` per row into `a_rows: Vec<Vec<f64>>` (constraints.rs:19), and solve_with_cuts_and_trust_region (solver.rs:402-410) rescans all n_vars entries of every base row on every call. The bundle IS built once (mod.rs:94) and passed by `&` into both SLP passes and the Auto fast+tight passes (mod.rs:133-152), so precomputing the static CSC-by-column is a valid, provably-identical refactor. Fix A is trajectory-safe: same CscMatrix, same Clarabel input, same solution. Keep it.

But the speedup magnitude is inflated. The proposer's "54M reads at 10ns/L2-miss ≈ 540ms equivalent" is fiction: the scan is row-contiguous and sequential within each 3.6KB row, streaming from L2/L3 at ~10-20GB/s, so 4.36MB/call ≈ 0.3-0.5ms/call, ~30-50ms across ~100 calls — and the proposer silently walks the 540ms figure back to "~50ms" anyway. Against an 867ms total dominated by QDLDL factorization + IPM iterations, eliminating ~30-50ms of assembly is ~3-6%, not the headlined "5-10%". The clone-per-call the fix introduces (~1.4K element copy) is not free either, so net recovery is below the gross scan cost. Realistic Fix A: 3-6%.

Fix B is where it breaks the non-negotiable. compute_n (grid.rs:17-19) defines spacing along the CONTROL POLYGON, not arc length; that is the encoded design contract. Swapping to arc-length silently redefines "0.5mm spacing" and REDUCES N, which is a relaxation of the discretized SOCP — fewer constraint rows, constraints enforced at fewer points. The trajectory-invariants profile already classified blanket N reduction as "the textbook trajectory-unsafe move." The proposer's safety argument ("verify::check_chain runs at the same N") is backwards: coarsening the solve grid AND the verify grid together makes inter-grid jerk/accel violations invisible to BOTH — the verifier checks the same N points (verify.rs), it is not an independent finer guard. A 10-13% coarser grid yields a measurably different b(s) profile (different objective quadrature, under-enforced constraints), which is exactly "a measurably slower [or constraint-violating] trajectory" the charter forbids without proof of no-loss. The proposer offers only that the arc-length ESTIMATE is 0.1% accurate — conflating measurement accuracy with discretization feasibility. The risk-3 floor (cap reduction at 10%) is arbitrary, not a correctness proof. Fix B must be rejected or downgraded to "increase N where arc length exceeds polygon," never decrease.

Required changes: (1) Ship Fix A alone as a pure refactor with a debug_assert that precomputed CSC equals the scan output. (2) Drop Fix B's N reduction entirely; if arc length is wanted, only allow it to RAISE N (max of both metrics), which is trajectory-safe but yields zero speedup. (3) Correct the speedup claim to ~3-6%. Neither fix approaches the sub-250ms budget; the SLP call-count and absent warm-start remain the real problem, as the proposer concedes.

- **Required changes:** Split into two PRs. Fix A: ship as pure refactor, add debug_assert verifying precomputed CSC byte-identical to the scan path, store n_base_rows and assert cut/TR rows append strictly after it. Fix B: REJECT the N-reduction; the only trajectory-safe variant is N = max(arc_len/spacing, polygon/spacing), which increases N (zero speedup) — so drop Fix B from the speedup case entirely. Re-label headline speedup as 3-6% (Fix A only).

- **Mechanism:** 
PROBLEM ANATOMY (two separable fixes, one PR)

Fix A — Precomputed CSC for the static base bundle (the dominant cost driver):

Every call to `solve_with_cuts_and_trust_region` (solver.rs:360-586) rebuilds the full CSC representation by scanning all `n_vars` entries of every `a_rows` element (solver.rs:402-410). For N=92: ~1200 base rows × 454 columns = ~545K float reads, predominantly zeros (only 2-4 nonzeros per row), from a ~4.4 MB working set. This scan executes ~100 times for a single 867ms solve → ~54 million cache-miss reads doing nothing but skipping zeros from a buffer 8x larger than the Pi5's 512 KB L2 cache.

The bundle is immutable after `build_chain` returns. The SLP loops never mutate `bundle.a_rows`; they supply incremental `SlpCut` and `TrustRegion` deltas. The CSC of the static rows is therefore identical on every call — only the cut rows and TR rows change.

Fix: extend `ConstraintBundle` with a precomputed field:

```rust
pub struct ConstraintBundle {
    // existing fields unchanged ...
    pub precomputed_csc: PrecomputedCsc,
}

pub struct PrecomputedCsc {
    pub rowval_per_col: Vec<Vec<usize>>,
    pub nzval_per_col: Vec<Vec<f64>>,
    pub n_base_rows: usize,
}
```

Populate `precomputed_csc` at the bottom of `build_chain` (constraints.rs:765-775), reusing exactly the existing scan loop but executing it once. In `solve_with_cuts_and_trust_region`, replace the scan loop (solver.rs:398-410) with a clone of the precomputed per-column vecs, then append cut rows and TR rows as today. The clone is O(nnz) ≈ O(3 * n_base_rows), not O(n_rows * n_vars).

Cost model for Fix A:
- Before: ~545K reads per call × ~100 calls = ~54M reads per segment solve, all L2-missing
- After: ~3.6K element clone per call × ~100 calls = ~360K element copies (sequential, cache-warm)
- Speedup on the scan step alone: ~150x reduction in memory traffic for this phase
- The clone is ~29KB (n_base_rows≈1200, avg 3 nonzeros/col, 454 cols → ~1362 total nnz across rowval + nzval), fitting in L1 cache on every call
- Does not change any Clarabel input, SOCP solution, or tolerance; purely an implementation restructuring

Fix B — Arc-length N correction (small N reduction, provably safe):

`compute_n` in grid.rs:5-23 calls `control_polygon_length_mm` (grid.rs:172-182), which sums control-point edge lengths. For a degree-3 Bezier with 4 control points, the control polygon is a guaranteed upper bound on arc length. The gap depends on curvature: for a nearly-straight Bezier it is ~0%, for a tightly-curved one it can reach 30%. The adaptive spacing target is 0.5mm, so a 15% overestimate produces N that is 15% too large.

Replacing the heuristic with a Simpson's rule arc-length estimate using 16 evaluation points (evaluating the Bezier at uniformly-spaced parameters and summing chord lengths) gives arc-length accuracy within ~0.1% for typical printer curves. This is a ~16-point curve evaluation, negligible cost.

For a 46mm segment with a 15% control-polygon overestimate: current N = ceil(53mm / 0.5) = 106; corrected N = ceil(46mm / 0.5) = 92. That is a 13% reduction in N.

Trajectory impact of N reduction: the SOCP problem is relaxed (fewer grid constraint evaluations), so the solved b(s) profile is not more restricted — it is at most as fast as the true minimum-time trajectory discretized at N=92 vs N=106. The fineness question is whether inter-grid constraint violations are caught. The verification pass at `verify::check_chain` evaluates constraints at the same N grid points used for the solve. A 13% N reduction means constraint violations could be missed in the inter-grid 13%-wider intervals. However: (a) the `inter_geom` blocks in constraints.rs:575-604 evaluate centripetal caps between consecutive grid nodes, and (b) the reparametrization stage (emit_us) fits a polynomial representation whose evaluation at arbitrary parameter values serves as the real guard. The SOCP solution at N=92 has already been observed to produce correct trajectories at this spacing (the 46mm example in the profile uses exactly N≈92 and produces a valid result). The arc-length correction eliminates the systematic over-resolution from the control-polygon overestimate without going below the physically meaningful 0.5mm spacing target.

Cost model for Fix B at a 13% N reduction (N=106 → N=92):
- n_vars: 5*106-6 = 524 → 5*92-6 = 454 (13.4% fewer variables)
- Total rows: scales proportionally, ~13% fewer
- Each Clarabel IPM factorization step: O(N) with banded structure → ~13% faster per step
- Per `solve_with_cuts_and_trust_region` call: ~13% faster
- 100 calls: ~13% faster overall from the N correction alone
- On Pi5 where each call costs ~5-9ms: saves ~0.65-1.2ms per call × 100 = 65-120ms off the 867ms total

Combined effect of Fix A + Fix B:

Fix A addresses the dominant non-solver overhead (~54M wasted reads). Fix B addresses the N over-inflation. Their speedups are largely additive because they target different phases: Fix A reduces the matrix-assembly overhead between Clarabel calls; Fix B reduces the per-Clarabel-call cost.

Fix A speedup estimate: the 54M reads are replaced by ~360K sequential writes; on Pi5 with ~10ns per L2-miss and ~0.5ns per L1 hit, the assembly phase drops from ~540ms equivalent to ~0.18ms equivalent across all 100 calls. In practice this will not deliver a 3000x speedup on the assembly phase because the Clarabel solve itself (QDLDL factorization, IPM iterations) dominates — but the current assembly overhead of ~50ms across 100 calls (estimated from the scan throughput) is recovered. Conservative estimate: 5-10% overall solve-time reduction from Fix A alone.

Fix B speedup estimate: ~13% per-call speedup at N=92 vs N=106, amounting to ~80-100ms off the 867ms total.

Combined lower bound: 15-20% reduction → ~130-170ms saved from 867ms, bringing the worst case toward ~700-750ms. Not sufficient alone to reach real-time for the high-velocity mid-stream case, but these are the two changes that are zero-risk, require no algorithmic changes, and are cleanly separable from the SLP iteration-count problem (which requires a different approach). They are also prerequisite: any warm-start or coarse-to-fine scheme still benefits from the smaller N and the faster per-call assembly.

Trajectory safety: Fix A is a pure implementation refactor — same CSC matrix, same Clarabel input, provably identical solution. Fix B reduces N from the overestimated value back toward the physically justified 0.5mm spacing target; the verification pass (`verify::check_chain`) remains the acceptance gate and is unchanged. Neither fix relaxes any tolerance, changes any SLP iteration budget, drops the ToleranceMode::Auto fallback, or modifies the fail-loud SegmentLate behavior. The non-negotiable constraint is fully honored: these changes make the same trajectory computation run faster, not a cheaper trajectory computation.


- **Trajectory impact (proposer):** 
None. Both fixes are trajectory-safe under the invariant checklist from the profile [trajectory-invariants] analysis:

1. N is not reduced below the target_grid_spacing_mm=0.5mm constraint — it is corrected toward the physically justified value. Fix B eliminates systematic over-resolution from the control-polygon overestimate; the resulting N at 0.5mm true-arc-length spacing is the same N the design originally intended.

2. verify::EPS_FEAS=2e-3 and EPS_FEAS_JERK=5e-2 are untouched.

3. SLP iteration budgets (SLP_MAX_OUTER_ITERS=50, SLP9_MAX_OUTER_ITERS=30) are unchanged.

4. ToleranceMode::Auto (1e-5 fast → 1e-8 tight fallback) is preserved.

5. max_threads=1 / qdldl determinism pin is preserved.

6. Fail-loud SegmentLate behavior is unchanged.

Fix A: the precomputed CSC encodes exactly the same non-zero pattern and values as the current scan — provably identical Clarabel input, provably identical solution. No trajectory change is possible.

Fix B: the verification pass (verify::check_chain) runs at the same N used for the solve, so feasibility acceptance is not weakened. The arc-length estimate (16-point Simpson's rule) has ~0.1% error on typical printer Beziers; 0.1% of 0.5mm = 0.0005mm spacing error, negligible. No tolerance is loosened, no constraint is dropped.



### Persistent warm-started inner solver: amortize symbolic factorization + central-path warm-start across the SLP/SLP9 cold-start cascade  ·  effort=large  ·  verdict=**keep_with_changes**  ·  trajectory_safe=True

- **One-liner:** Replace the ~100-200 cold-start `DefaultSolver::new` conic solves per replan with a persistent solver instance that (a) computes the AMD ordering + symbolic factorization once per fixed-sparsity SLP family and reuses it across all inner solves, and (b) warm-starts each inner solve from the previous iterate via a central-path smoothing operator — cutting both per-call factorization cost and IPM iteration count without touching N, tolerances, or the feasible set.

- **Proposer speedup:** Decompose the observed 867ms = ~100-150 cold solves x ~6ms (Pi5). 

Symbolic reuse: removes ordering+symbolic from all-but-first solve. Conservative on a banded n=92 KKT where symbolic is a modest share of each solve; OSQP's fixed-structure MPC caching gives 2.6-4x but that includes numeric reuse it can do that we cannot (our RHS changes). Realistic isolated gain here: ~1.3-1.8x on the per-solve floor.

Warm-start: 2512.00693 measures 0.50-0.63x iterations/time on Clarabel parametric SOCP families; our perturbations (rho_b ~5-25%) are at the favorable small-delta end. Realistic: ~1.6-2.0x on iteration count.

Compounded (they multiply — fewer iterations, each cheaper to set up): ~2.5-3.5x on Pi5. 867ms -> ~250-350ms for the worst chained curve; the 63-190ms early curves -> ~25-75ms. Pi4 (A72, ~1.5x slower than A76 + no SVE): the same MULTIPLIER applies (it is algorithmic, not microarchitectural), so Pi4 867ms-equivalent (~1.3s) -> ~370-520ms.

This alone does NOT reach the 50ms warn budget for the worst case on either Pi — but it is multiplicative with, and orthogonal to, the cross-segment/cross-core parallelism fix ([streaming-budget-parallelism]: 3 of 4 cores idle) and the ToleranceMode::Auto double-solve fix ([slp-fallback] Finding 1). Stacked: ~3x (this) x ~2-3x (parallelism, once window_segments>1) x ~2x (kill the Auto double-solve on the fast path) plausibly clears the budget. As a standalone item it converts the fail-loud SegmentLate abort (867ms landing 0.369s in the past, only 250ms LEAD) into a comfortable success for the mid-stream case, because it attacks the exact multiplier (cold-start count) that the profiles name as the #1 driver.

- **Verifier realistic speedup:** ~1.4-2.0x on Pi5 realistically (not 2.5-3.5x). Likely toward the low end of that range as a standalone change; the symbolic-reuse half is largely defeated by variable sparsity, so most of the win must come from warm-start alone.

- **Verifier reasoning:** TRAJECTORY SAFETY (the non-negotiable): this is the one part of the proposal that holds up cleanly. Warm-starting a convex IPM and reusing a symbolic factorization do not alter the feasible set, the objective, N, the verify::EPS_FEAS/EPS_FEAS_JERK acceptance bars, or the SLP/SLP9/beta iteration budgets. A warm-started solve converges to the same epsilon-optimal KKT point of the same convex base SOCP. The proposal correctly keeps max_threads=1/qdldl, so the joining-loop determinism pin is preserved, and correctly does NOT multithread within a solve. I could not construct a case where this ships a measurably slower trajectory, PROVIDED the guardrail (fall back to cold-start when the warm point is a poor start) is actually implemented and the inner solve still reaches the same tolerance. So trajectory_safe = true. The residual risk is determinism re-baselining of golden snapshots, which is a test-hygiene issue, not a trajectory regression.\n\nWHERE THE MECHANISM IS WRONG AGAINST THE CODE: the proposal's load-bearing claim for 'Half 1' (symbolic-factorization reuse) is that 'within one SLP family the KKT sparsity pattern is FIXED ... only numeric values change.' This is false against solver.rs. (1) build_axis_jerk_cuts_chain (solver.rs:848+) pushes one AxisJerk cut PER VIOLATING grid point, and the violating set shrinks as SLP9 converges — so the cut row count, hence the A-matrix row dimension and sparsity, changes every outer iteration. (2) cut_rows and tr_rows are appended as extra NonnegativeConeT blocks whose sizes vary: the TR-present backtrack solves carry 2*(n-2)+2*n trust-region rows, while the no-TR fallback solve at solver.rs:1166 carries ZERO TR rows — so even within a single outer iteration the cone structure differs between consecutive solves. (3) The path-jerk SLP similarly rebuilds cuts each iteration. The proposal's own implementation-sketch step 1 ('pre-allocate the maximal row set and zero unused rows') is an attempt to paper over exactly this, but zeroed rows still change the cone partition Clarabel sees (a NonnegativeConeT(k) of all-zero rows is not free and is not the same symbolic problem as omitting them), and the AMD ordering benefit is diluted. Net: the symbolic-reuse half delivers far less than the OSQP '2.6-4x from factorization caching' figure the proposal cites — OSQP's number is for genuinely fixed-structure MPC re-solves with only vector changes, which is NOT our case. Realistic isolated gain from symbolic reuse here is small, maybe 1.1-1.3x, and only if a stable super-pattern is engineered.\n\nThE WARM-START HALF is the real win, but the proposal overstates transferability. The cited arXiv 2512.00693 measures 0.50-0.63x on parametric SOCP families where consecutive problems differ ONLY by a small data perturbation on a FIXED problem structure. Our consecutive solves differ by (a) a small RHS/anchor perturbation AND (b) a changing row set AND (c) trust-region radius collapses (rho_b -> 0.005) AND (d) the 1e-5->1e-8 tolerance jump in ToleranceMode::Auto AND (e) SLP9 restoration resets to RESTORE_RHO_B_INIT=0.50. Cases (b)-(e) are precisely where warm-start degrades or must fall back to cold (proposal's own risk #2). So the favorable 0.5-0.63x multiplier applies only to the subset of inner solves that are clean small-perturbation re-solves — probably the majority of SLP9 backtrack-accepted steps, but not the structural transitions. Blending, I estimate warm-start alone yields ~1.4-1.8x on the iteration-count-dominated cost.\n\nCOMPOUNDING CLAIM IS INFLATED: the proposal multiplies symbolic-reuse (1.3-1.8x) by warm-start (1.6-2.0x) to get 2.5-3.5x. Since the two are NOT independent (warm-start reduces IPM iterations, which is exactly the phase where per-iteration factorization-setup amortization would have paid off — fewer iterations means less symbolic cost to amortize), they do not cleanly multiply. And symbolic-reuse is largely defeated by variable sparsity as shown. Realistic compounded standalone: ~1.4-2.0x, not 2.5-3.5x.\n\nEFFORT/RISK MISMATCH: the win is gated entirely on forking/patching Clarabel 0.11.1 (no public warm-start or persistent-symbolic API — confirmed in profiles and corroborated by the existing pi5-socp-throughput-investigation.md, which already classified warm-start/shared-factorization as a real-but-DEFERRED, formulation-blocked Step 8/9 item). That is a large, ongoing maintenance liability (vendored solver fork tracking upstream) for a sub-2x standalone gain that, by the proposal's own admission, does NOT clear the 50ms Pi4 budget alone. The fail-loud contract adds a sharp constraint: a botched warm-start must not corrupt Clarabel's Infeasible/MaxIter status mapping, which the SLP relies on (proposal risk #3) — this needs the CL-2024 counterexample regression test as a hard gate.\n\nCHEAPER ADJACENT WINS ARE BEING BUNDLED TO FLATTER THE NUMBER: the implementation-sketch folds in the dense-a_rows -> CSC-direct fix (the 908k-wasted-reads/call from [socp-cost]). That is a genuine, Clarabel-fork-FREE win that should be its own item and is arguably higher ROI than the warm-start fork. Crediting it to this approach inflates the apparent payoff of the hard part.\n\nVERDICT: keep_with_changes. The trajectory-safety case is solid and the warm-start direction is the correct 'optimize the implementation' move the CLAUDE.md constraint endorses. But (1) the symbolic-reuse half must be re-scoped or dropped pending a real fixed-super-pattern audit — its headline contribution is not supported by the code; (2) the speedup must be re-stated as ~1.4-2.0x standalone, not 2.5-3.5x; (3) the CSC-direct fix should be split out and done first as a no-fork win; (4) the Clarabel-fork dependency should be acknowledged as the dominant cost/risk and sequenced AFTER the cheaper parallelism ([streaming-budget-parallelism]: 3/4 cores idle) and ToleranceMode::Auto double-solve fixes ([slp-fallback] Finding 1), which deliver comparable or larger multipliers with no solver fork. As the FIRST thing to build, this is not it; as a later-stage multiplier it is sound.

- **Required changes:** 1) Re-scope or drop the symbolic-factorization-reuse half: audit build_axis_jerk_cuts_chain and the path-jerk cut builder to determine whether a STABLE maximal super-pattern can actually be engineered (fixed row count with zeroed inactive cut/TR rows mapped into stable cone blocks). If not, this half delivers ~nothing and should be cut from the claim. 2) Re-state expected speedup as ~1.4-2.0x standalone on Pi5, not 2.5-3.5x. 3) Split the dense-a_rows->CSC-direct construction fix out as a separate, Clarabel-fork-free item and do it first. 4) Sequence this AFTER the cross-segment parallelism fix and the ToleranceMode::Auto double-solve fix, which are larger/cheaper multipliers needing no solver fork. 5) Make the cold-start fallback guard and CL-2024-counterexample status-mapping regression test hard gates, not nice-to-haves, to protect the fail-loud contract. 6) Re-baseline golden trajectory snapshots and re-validate joining early-bail determinism under the warm-started iterate path.

- **Mechanism:** The profiles ([socp-cost], [slp-fallback], [grid-beta-discretization]) all converge on one root cause: the 867ms is NOT a single expensive solve — it is 100-200 cold-start Clarabel invocations (path-jerk SLP up to 50 iters + axis-jerk SLP9 up to 30 iters x up to 5 backtrack solves each + restoration), each calling `DefaultSolver::<f64>::new(...)` fresh at solver.rs:578. Every call discards: (1) the AMD ordering + symbolic factorization of the KKT system, (2) the numeric QDLDL factors, and (3) the primal-dual iterate, restarting IPM from the analytic center at ~15-25 IPM iterations.

Two independent, well-established techniques attack the two halves of that waste, and the trajectory-invariants verifier already classified both as trajectory-SAFE (Attack 6: "these change how fast Clarabel reaches the same KKT point, not the feasible set, objective, N, or tolerances ... these survive as the trajectory-safe speedup class").

HALF 1 — Symbolic-factorization reuse (the per-call floor). Within one SLP family the KKT sparsity pattern is FIXED: the SLP cuts (append_path_jerk_cut_weights) and SLP9 trust-region rows are appended at structurally identical positions every iteration; only numeric values (the linearization anchor b_bar, RHS) change. The IPM literature is unanimous that ordering + symbolic factorization depend only on sparsity, so they can be computed once and reused for every subsequent numeric factorization (Vanderbei/PIQP/OSQP: "Since the structure of the Jacobian remains constant ... the elimination tree and fill-in pattern can be reused for every subsequent solve, avoiding symbolic computation"; OSQP: "symbolic and numerical factorizations computed only once, then stored and reused if only the vectors change"). For the banded n=92 KKT (bandwidth ~3 from the b-stencil), the symbolic/AMD phase is a real fraction of each ~5-8ms solve; amortizing it across 100-200 calls removes it from all but the first. OSQP reports 2.6-4x from factorization caching alone on fixed-structure MPC re-solves.

HALF 2 — Central-path warm-start (the IPM-iteration-count multiplier). The dominant cost is iteration count x per-iteration factorization. Consecutive SLP9 iterates differ by only rho_b*b_bar (~5-25%, shrinking toward the trust-region floor 0.005) — a textbook "small parametric perturbation." The 2025 arXiv warm-start-for-conic-IPM paper (2512.00693) gives a smoothing operator s0 = S_{K,mu0}(c) that maps the previous primal-dual solution onto the NEW problem's central path (Thm 3.1) with residuals bounded O(mu0) (Thm 4.1), and it is demonstrated ON Clarabel across SOCP/power-cone parametric families with measured 0.50-0.63x iteration AND time reduction. That is exactly our regime: a chain of near-identical SOCPs differing by a small RHS/row perturbation. The two halves compound: warm-start halves the IPM iterations, symbolic reuse cheapens each remaining iteration's factorization setup.

Why this respects the mandate completely: the SOCP base relaxation is convex, so a warm-started IPM converges to the SAME epsilon-optimal KKT point — identical trajectory time. N is untouched (no grid coarsening). verify::EPS_FEAS/EPS_FEAS_JERK acceptance bars untouched. ToleranceMode::Auto fast->tight fallback preserved. SLP/SLP9/beta iteration budgets untouched. This is the canonical example of "optimize the implementation" that the CLAUDE.md constraint explicitly endorses over "ship a cheaper algorithm."

The one real friction: Clarabel 0.11.1 exposes no public warm-start or persistent-symbolic API (confirmed in [socp-cost]). So this requires either (a) a fork/patch of the Clarabel Rust crate to expose `update_data` + warm-start entry points (the data structures exist internally — QDLDL symbolic phase is separable; the homogeneous-embedding warm-start in 2512.00693 was implemented against Clarabel), or (b) migrating the inner conic solve to a solver built for embedded re-solves. PIQP is the closest embedded analog ("allocation free problem updates and re-solves," pivoting-free factorization) but is QP-only today (no SOC), so it would only serve if the axis-accel SOC(4) cones are reformulated — a larger change. Recommended path is (a): keep Clarabel (it stays the same math, preserving the determinism pin and the CL-2024 robustness), patch it to persist symbolic + accept a warm-start point.

- **Trajectory impact (proposer):** None — and this is independently verified, not asserted. The trajectory-invariants verifier's Attack 6 explicitly tested warm-start + shared-factorization and ruled them trajectory-safe: "These change how fast Clarabel reaches the same KKT point, not the feasible set, objective, N, or tolerances. Warm-starting an interior-point solver changes the iterate path but converges to the same epsilon-optimal point for a convex program — and the SOCP base relaxation IS convex." The 2512.00693 smoothing warm-start additionally proves (Thm 3.1) the start point lies ON the new central path and (Thm 4.1) residuals stay O(mu0), so convergence is to the same tolerance, not a looser one. N, verify::EPS_FEAS=2e-3, EPS_FEAS_JERK=5e-2, SLP/SLP9/beta iteration budgets, and ToleranceMode::Auto are all untouched. The deterministic SLP outer loop is unaffected provided each inner solve still converges to the same point (it does). One guardrail the verifier flagged: keep the max_threads=1 / qdldl determinism pin — this proposal does NOT multithread a single solve, so the joining-loop early-bail determinism is preserved. Net: provably no measurable trajectory loss; the trajectory is bit-stable to within the unchanged inner tolerance.



---

## Appendix B — profiling digest (5 parallel deep-dives)

### Profile [socp-cost]
Now I have everything needed for a complete, specific technical report.

---

SOCP COST PROFILING REPORT: `temporal` Clarabel solver at n=92 (46mm G5 segment)

---

PROBLEM SIZE

Variable layout is defined in `constraints.rs:build_chain` (lines 315-325):

- `off_b = 0` through `off_x2 = 4n - 4`, giving `n_vars = 5n - 6`
- At n=92 (46mm / 0.5mm grid spacing): **n_vars = 454** (b[92] + a[92] + t[90] + x1[90] + x2[90])
- At n=200 (max_n cap): **n_vars = 994**

Constraint row counts at n=92, organized by cone type:

Block (a) ZeroCone: 2 rows (boundary pins b_0, b_{n-1})  
Block (b) ZeroCone: 92 rows (acceleration definition via finite-difference stencil)  
Block (c) Nonneg: ~92 rows (velocity limits per axis-set)  
Block (d) SOC(4): 92 cones, 368 rows (3-axis acceleration bound: 1 SOC(1+3) per grid point)  
Block (e) Nonneg: 92 rows (centripetal cap b_i <= b_max_cent[i])  
Block (f) Nonneg: ~46 rows (inter-segment inter-sample centripetal; curve-dependent)  
Block (g) Nonneg: ~92 rows (rest-boundary ramp; active when v_start or v_end = 0)  
Block (h) Nonneg: 180 rows (jerk path slack: 2*(n-2) rows for t_k >= 0)  
Block (i) Nonneg: 180 rows (x1, x2 non-negativity: 2*(n-2))  
Block (j) SOC(3): 270 cones, 810 rows (3 SOC(3) per interior point via norm identity)  

Total at n=92 base problem: **~1954 rows**, **362 SOC cones**, **94 Zero rows**, **~680 Nonneg rows**  
Total SOC rows: **~1178** (blocks d+j) — **60% of all rows are second-order cone rows**

NNZ(A): The `push_row` function at `constraints.rs:352-360` allocates `vec![0.0; n_vars]` per row and fills 1-4 non-zeros. Actual NNZ is approximately 3 per row on average, giving **~5,900 NNZ** at n=92. Sparsity: 0.66% — the matrix is >99% zeros.

---

WARM-START STATUS

**Warm-starting is completely absent.** Every call to `solve_with_cuts` and `solve_with_cuts_and_trust_region` (`solver.rs:350-586`) calls `DefaultSolver::new(...)` freshly (line 578). Clarabel 0.11.1 has no public warm_start API (confirmed by the dependency graph: no warm-start crate dependency). Every SLP outer iteration discards all IPM state — primal/dual variables, Nesterov-Todd scaling matrices, factorization — and restarts from the analytic center.

---

QDLDL FACTORIZATION COST

Clarabel uses QDLDL (an AMD-ordered sparse LDL factorization) on the KKT augmented system. The Nesterov-Todd scaling adds one block per SOC cone: each SOC(d) cone adds d dual slack variables to the augmented system.

Augmented KKT dimension at n=92: n_vars + m + NT_SOC_augmentation  
= 454 + 1954 + (1178 SOC rows duplicated for NT scaling)  
≈ **3,586** at base; **~4,000** with SLP cuts and trust-region rows added.

For the sparse structure produced by the stencil-based constraints (each row touches 1-3 grid-consecutive b-variables), AMD ordering produces a roughly banded structure with bandwidth proportional to the stencil width (~3). Sparse LDL factorization cost under this structure scales as **O(n_vars * bandwidth^2) = O(n * 9) = O(n)** in the bandwidth model — but the SOC NT scaling breaks the pure-banded structure by coupling all d variables within each cone. With 362 SOC cones each of size 3-4, the effective fill is higher. Empirically, **each Clarabel solve at n=92 costs approximately 8-12 ms on a Pi5** (2.4 GHz A76 core, single-threaded as pinned by `max_threads=1` at `solver.rs:573`).

---

CSC MATRIX CONSTRUCTION OVERHEAD (THE HIDDEN COST)

The `ConstraintBundle.a_rows` field is a `Vec<Vec<f64>>` where each inner vector has length `n_vars = 454` even though only 2-4 entries are non-zero (`constraints.rs:353-360`). The conversion to CSC at `solver.rs:402-410` scans all `n_vars` entries of every row:

```rust
for row in &bundle.a_rows {
    for (col, &v) in row.iter().enumerate() {
        if v != 0.0 { ... }
    }
    n_rows += 1;
}
```

This is **908,000 scalar reads per Clarabel call at n=92**. At ~50 SLP subproblems (within `slp_solve_chain`'s `SLP_MAX_OUTER_ITERS=50` at `solver.rs:589`) this is **45 million reads just for A-matrix scanning**, none of which advance the solve. The dense allocation also creates a 7 MB working set that thrashes L2 cache (Pi5 L2 is 512 KB per core).

---

SLP OUTER LOOP STRUCTURE AND WORST-CASE SOLVE COUNT

`slp_solve_with_axis_jerk_chain_inner` (`solver.rs:1380`) runs two nested SLP loops:

**Phase 1 — path-jerk SLP** (`slp_solve_chain`, `solver.rs:938`):
- 1 base solve (no cuts)
- Up to `SLP_MAX_OUTER_ITERS = 50` outer iterations (`solver.rs:589`)
- Each iter: clears all cuts, rebuilds n-2=90 path-jerk cuts, calls one `solve_with_cuts`
- Early-exit on convergence; divergence detection only fires after `SLP_WARMUP_ITERS=8` with 10-iteration improvement window
- Worst case: **51 Clarabel calls** for Phase 1

**Phase 2 — per-axis jerk SLP9** (`run_slp9_loop`, `solver.rs:1063`):
- Up to `SLP9_MAX_OUTER_ITERS = 30` outer iterations (`solver.rs:824`)
- Each outer iter: build axis-jerk cuts from `build_axis_jerk_cuts_chain`, then try up to `SLP9_MAX_BACKTRACKS+1 = 4` trust-region radii, each one a separate `solve_with_cuts_and_trust_region` call, plus a fallback `solve_with_cuts` without TR
- Worst case per outer iter: **5 Clarabel calls** (3 backtracks + 1 fallback + possible restoration)
- 1 restoration allowed (`SLP9_MAX_RESTORATIONS=1`) which triggers another run_slp9_loop
- Worst case Phase 2: **30 × 5 = 150 Clarabel calls**

**Phase 3 — polish** (`polish_windowed`, `solver.rs:1313`): up to `SLP9_POLISH_MAX_ITERS=5` more calls, but only on the happy path (convergence).

**Total worst-case Clarabel calls: 51 + 150 = 201**

At 9-12 ms per call on Pi5: **1.8–2.4 seconds worst-case wall time** for a single n=92 segment. The observed 867ms puts the actual solve at roughly 70-100 Clarabel calls — consistent with path SLP hitting its divergence boundary (~50 iterations) plus 15-20 SLP9 iterations before TR stall.

---

WHY HIGH-SPEED MID-STREAM CURVES ARE EXPENSIVE

The path-jerk constraint being linearized is:

`j_path(i) = |b''(s_i)| * sqrt(b_i) / 2 <= J_path`

where `b''` is approximated by the stencil `w·b` from `stencil::b_dd_weights`. The SLP linearization (`append_path_jerk_cut_weights`, `solver.rs:652`) cuts around the current iterate `b_bar`. For a rest-to-rest move `b_bar` starts near zero and the SLP converges in a few iterations because the linearization is tight (sqrt(b) has small curvature near 0).

For a mid-stream curve with `v_start ~ 4000 mm/s`, `b_start ~ 16e6 mm²/s²`: the linearization point is in the high-b regime where `d²(sqrt(b))/db² = -1/(4b^{3/2})` is small in absolute terms but the **absolute constraint value** is large, making each cut less tight relative to the feasible set. The SLP takes more iterations to converge. The growth pattern 63ms → 93ms → 190ms → 867ms across successive segments directly shows the SLP iteration count increasing as `v_start` ramps up from rest to cruising speed.

Additionally, the axis-jerk SLP9 (`run_slp9_loop`) uses trust-region contraction when candidates fail (reducing `rho_b` down to `SLP9_RHO_B_MIN=0.005`, `solver.rs:829`). At high speed, trust-region stalls force the `damp_profile_uniform` restoration path, which triggers another 30-iteration SLP9 pass.

---

SINGLE BIGGEST COST DRIVER

The dominant cost is not a single Clarabel IPM step — it is the **total number of cold-start Clarabel invocations** across both SLP loops. Each call discards and re-derives the NT scaling from scratch, re-equilibrates, and re-factorizes QDLDL. On a Pi5 with `max_threads=1` (pinned at `solver.rs:573`), the QDLDL factorization at n=92 KKT dimension ~3600 costs approximately 5-8 ms per IPM iteration; at 15-25 IPM iterations per Clarabel call this is 75-200 ms per Clarabel invocation. At 80-100 total calls for a saturated mid-stream solve, this accumulates to 600-800 ms of IPM work, with the remaining 50-70 ms in CSC matrix construction, allocation, and equilibration.

The second largest contributor is the **dense a_rows representation** in `ConstraintBundle`. The 7 MB working set (2000 rows × 454 columns × 8 bytes) at n=92 per solve exceeds the Pi5's L2 cache by 13×. The O(n_rows × n_vars) scan in `solve_with_cuts_and_trust_region` lines 402-410 is executed 80-100 times per segment, spending 45+ million cache-miss-heavy reads purely on skipping zeros that should never have been stored.

---

REALISTIC FLOOR ON SOLVE TIME

A single Clarabel solve at n=92 with 15-20 IPM iterations has a realistic floor of **6-10 ms on Pi5** once the matrix is built in CSC format. The base SOCP (no cuts) likely takes 3-5 IPM iterations for a well-conditioned rest-to-rest case, giving a ~2-4 ms floor per call. For a mid-stream curve where path SLP converges in 5 outer iterations and axis SLP9 in 5 iterations, the minimum realistic total is approximately:

- 6 Clarabel calls × 8 ms = **~48 ms** (Pi5) or **~90 ms** (Pi4)

This is the floor assuming the SLP loops converge quickly. The observed 867ms represents a case where both loops hit their maximum iteration counts, indicating the SLP is not converging — not just being slow per iteration.

---

FILE/LINE SUMMARY

- `rust/temporal/src/topp/constraints.rs:189-776` — `build_chain`: variable layout, dense `push_row` at line 352-360, all constraint blocks
- `rust/temporal/src/topp/solver.rs:350-586` — `solve_with_cuts_and_trust_region`: cold-start Clarabel call at line 578, dense CSC scan at lines 402-410
- `rust/temporal/src/topp/solver.rs:938-1049` — `slp_solve_chain`: path-jerk SLP, `SLP_MAX_OUTER_ITERS=50` at line 589, cuts cleared each iteration at line 971
- `rust/temporal/src/topp/solver.rs:1063-1214` — `run_slp9_loop`: axis-jerk SLP9, `SLP9_MAX_OUTER_ITERS=30` at line 824, up to 5 Clarabel calls per outer iter (lines 1137-1177)
- `rust/temporal/src/topp/solver.rs:563-576` — `DefaultSettings`: `max_threads=1` at line 573, tolerance settings
- `rust/temporal/src/multi/grid.rs:5-23` — `compute_n`: `target_grid_spacing_mm=0.5` gives n=92 for 46mm
- `rust/trajectory/src/streaming/state.rs:211-254` — `solve_us` timing wraps the full SLP cascade

### Profile [slp-fallback]
I now have everything needed for the complete analysis. Here is the dense technical findings report:

---

# SLP Fallback Pathology: Root-Cause Analysis

## 1. What "fallback_rung=1" Actually Means (Terminology Fix)

The field `fallback_rung` is set to `1u8` on **success** of the primary `plan_velocity` call (`state.rs:220-221`). Rung 2 and 3 are fallbacks triggered by an `Err` from `plan_velocity`. The observed `fallback_rung=1` means the primary path succeeded — no witness fallback fired. The 867ms is entirely inside the primary path: `plan_velocity` → `plan_batch_full` → `beta_iterate_inner` → `run_one_iteration` → `temporal::multi::plan_batch` → `schedule_chain_with_tolerance` → the SLP stacks.

## 2. Clarabel Solve Count per Replan (The Core Accounting)

For a single segment (`window_segments=1`, `beta_iters=1`) with `ToleranceMode::Auto`:

**Layer A — `schedule_chain_with_tolerance` → `call_slp`:**

`ToleranceMode::Auto` (`topp/mod.rs:145-153`) calls `call_slp(1e-5)` and, if `solver_outcome_is_success` returns false, calls `call_slp(1e-8)` again. A failed fast pass therefore doubles the work before a single SLP iteration runs.

**Layer B — `slp_solve_with_axis_jerk_chain_inner` (`solver.rs:1380-1493`):**

The function is a two-stage procedure:
1. `slp_solve_chain` (path-jerk SLP, `solver.rs:938`): one Clarabel call for the base SOCP, then up to `SLP_MAX_OUTER_ITERS = 50` additional Clarabel calls — one per outer iteration — until path-jerk violations converge or the divergence window (8-iter warmup + 10-iter no-improvement window) fires.
2. `run_slp9_loop` (per-axis jerk SLP): for each of up to `SLP9_MAX_OUTER_ITERS = 30` outer iterations, at most `SLP9_MAX_BACKTRACKS+1 = 4` calls to `solve_with_cuts_and_trust_region` per accepted/rejected step, plus one call to `solve_with_cuts` without a trust region when all backtrack candidates are rejected (`solver.rs:1165-1176`). After a `TrFloorStall`, one restoration attempt reruns the loop for another 30 iterations.
3. If `windows` is `Some` and the outcome is `Converged`, `polish_windowed` runs up to `SLP9_POLISH_MAX_ITERS = 5` more Clarabel solves.

**Absolute worst case** (both tolerance passes fail fast, path SLP hits divergence early at ~18 iters, axis SLP stalls and restores once):
- Fast pass: 1 base + ~18 path iters = ~19 solves
- Tight pass: 1 base + ~18 path iters = ~19 solves, then ~30 axis iters × 5 calls + ~30 axis iters × 5 calls (after restoration) = 300 axis solves
- Total: **~338 Clarabel invocations per replan**.

Even in the moderate case (fast pass fails, tight pass runs path-jerk to convergence in ~15 iters, axis SLP takes ~20 iters each of up to 3 backtrack attempts):
- Fast pass: ~16 solves
- Tight pass: ~16 path + ~60 axis (20 × 3) = ~76 solves
- Total: **~92 Clarabel invocations**

## 3. Why Cost Grows 63ms → 867ms as Entry Velocity Rises

**Root cause: the `ToleranceMode::Auto` double-solve triggered by axis-jerk infeasibility at high speed.**

The CL-2024 SOCP (Consolini-Locatelli) constrains speed `b = ṡ²` and accel `a = s̈` via centripetal MVC cones, velocity and accel limits. The path-jerk SLP (`slp_solve_chain`) enforces the linearized `|b″|·√b ≤ 2J` condition. However, the per-axis jerk constraint is non-convex and is only handled in the outer SLP9 loop via `build_axis_jerk_cuts_chain`. When entry velocity is high:

- The unconstrained SOCP solution respects velocity and centripetal caps but violates axis jerk near the entry (high `ṡ`, large `s̈`, large cross term `3c″·a·√b`).
- `solver_outcome_is_success` at `topp/mod.rs:271-278` requires both `status_ok` (Clarabel status) AND `outcome_ok` (SLP `Converged`). The SLP9 axis jerk loop (`run_slp9_loop`) emits `SlpOutcome::Diverged` or `SlpOutcome::MaxIters` when cuts don't converge. This makes `solver_outcome_is_success` return false on the fast 1e-5 pass, causing `ToleranceMode::Auto` to re-run the entire `call_slp` stack at 1e-8 — paying the full solve cost twice.
- The tight 1e-8 pass is harder for Clarabel (QDLDL is more sensitive to conditioning at tight tolerances), so more inner iterations are needed. `max_iter = 1000` is set (`solver.rs:565`) to avoid `InsufficientProgress` failures on the "CL-2024 counterexample" (`solver.rs:558`), meaning Clarabel will run to 1000 iterations rather than early-terminating.
- Velocity growth raises the SLP linearization error. At high `ṡ`, the Taylor approximation `j_axis ≈ c‴·b^{3/2} + 3c″·a·√b + c′·b″` has large residuals between iterates, causing the trust-region radius `rho_b` to shrink repeatedly toward `SLP9_RHO_B_MIN = 0.005` (a `TrFloorStall`), triggering the `damp_scale_for_axis_feasibility` restoration path (`solver.rs:1467-1488`) and a second `run_slp9_loop` pass (`SLP9_MAX_RESTORATIONS = 1`). Each backtrack attempt fires a full Clarabel solve.

**The cost-growth is thus a compounding of three effects:**
1. Higher velocity → larger initial axis-jerk violation ratio → more SLP9 outer iterations before first improvement.
2. Larger violation → each backtrack fires because `cand_ratio < best_ratio` fails more often, exhausting `SLP9_MAX_BACKTRACKS = 3` and requiring the trust-region-free fallback solve (line 1166).
3. `ToleranceMode::Auto` double-solve: every fast-pass failure (which becomes more likely at high speed where axis jerk infeasibility is harder to certify at 1e-5) doubles all of steps 1 and 2.

## 4. The Grid Size: 46mm at 0.5mm spacing → N=92

For a 46mm segment with `target_grid_spacing_mm=0.5`: `ceil(46/0.5) = 92` grid points. The SOCP has `2×92 = 184` variables, plus SOC cones for centripetal/velocity caps (O(N) rows each) and Nonneg cones for MVC. Each SLP cut adds 2 Nonneg rows. After 20 SLP9 axis-jerk cuts (across XY axes) with 3-backtrack loops, the Clarabel A matrix grows to O(N + K_cuts) rows. Clarabel's QDLDL factorization cost scales roughly as O((n_vars + n_rows)·nnz) per iteration. The matrix is re-factored from scratch every Clarabel call because there is no warm-starting: `solve_with_cuts_and_trust_region` constructs a new `DefaultSolver` and calls `solver.solve()` each time (`solver.rs:578-580`).

## 5. No Warm-Starting: The Single Largest Wasted-Work Source

`solve_with_cuts_and_trust_region` (`solver.rs:360-586`) constructs a new `DefaultSolver::<f64>::new(...)` on **every call**. This discards the QDLDL symbolic factorization, the numerical factors, and the previous iterate entirely. For the SLP9 loop with tight trust regions, consecutive iterates differ by `rho_b × b_bar ≈ 0.10 × b_bar` in each interior `b_i` — a small perturbation. Clarabel's interior-point method could warm-start from the previous primal-dual solution in O(factorization) time instead of O(solver_iterations × factorization) time. The Clarabel 0.11 API exposes `DefaultSolver::new` but not a warm-start entry point in the Rust crate's public interface.

## 6. `SLP_MAX_OUTER_ITERS=50` vs `SLP9_MAX_OUTER_ITERS=30`: Asymmetric Termination

`slp_solve_chain` (path-jerk) runs up to 50 outer iterations with a 10-iter no-improvement window. When it terminates early via `Diverged` (`solver.rs:1033`), `slp_solve_with_axis_jerk_chain_inner` at line 1388 checks:

```rust
if matches!(path_outcome, SlpOutcome::InnerSolverFailure | SlpOutcome::Diverged { .. } | SlpOutcome::MaxIters { .. }) {
    return Ok((path_result, path_outcome));
}
```

A `Diverged` path outcome short-circuits before the axis-jerk loop runs. The returned `SlpOutcome::Diverged` then fails `solver_outcome_is_success` (requires `Converged`), triggering the `ToleranceMode::Auto` second pass at 1e-8. The second pass starts from a cold Clarabel instance — the diverged iterate is not reused as a warm start for the tight-tolerance solve.

## 7. `SolverStatus::MaxIter` from Clarabel Not Propagated as Fast-Enough

In `slp_solve_chain` at line 996-1006, if the inner `solve_with_cuts` returns `MaxIter`, the function returns early with `MaxIters` outcome, which then causes the entire chain to fail `solver_outcome_is_success`. The result is discarded even if the `MaxIter` solution has near-feasibility (`residual` from the dual gap could be small). The `SolvedInexact` path only maps from Clarabel's `AlmostSolved` status, which requires `reduced_tol_gap_abs = 1e-3`. This means a solve that converged to 2e-3 gap is treated identically to one that produced a garbage solution — both fire the `ToleranceMode::Auto` retry.

## 8. Clarabel Settings: `max_iter=1000` Is a Multiplier on Wasted Work

`max_iter = 1000` (`solver.rs:566`) was set to prevent `InsufficientProgress` on the CL-2024 counterexample. In the axis-jerk SLP with tight trust regions, a subproblem where the trust region is too tight produces an infeasible problem — Clarabel would ideally detect this quickly via a dual certificate. At `max_iter=1000`, it instead runs 1000 interior-point iterations before returning `MaxIterations` → `SolverStatus::MaxIter`. This is observed in the backtrack loop at lines 1137-1163: for each of 3 backtracks on a bad iterate, Clarabel may run to near-iteration-limit before reporting failure. Three such attempts per outer iteration × 20+ outer iterations compounds severely.

## 9. Redoing Work the Witness Rungs Can't Rescue

`fallback_rung=1` (primary path succeeds) means rung 2 and 3 are not reached. The 867ms is a successful but extremely slow primary solve. The fail-loud `SegmentLate` abort in `DispatchError::SegmentLate` fires not because the solver failed but because the solver took 867ms before returning a valid answer, by which point `esc` (elapsed since sync) had advanced past `t_appended`. The design constraint (fail-loud, no re-anchoring) is working correctly — the problem is the solve latency itself.

## 10. Summarized Solve Count for the Observed 867ms Case

Given `window_segments=1`, `beta_iters=1`, the observed behavior pattern at high entry velocity:
- Fast pass (1e-5): base SOCP + path-jerk SLP (~8-15 iters) + axis-jerk SLP9 stall (~15-25 iters × up to 5 calls each + restoration × 30 more iters) → SLP9 returns `Diverged`/`MaxIters` → fast pass outcome is not `Converged` → `solver_outcome_is_success = false`
- Tight pass (1e-8): identical structure, harder conditioning → more Clarabel iterations per call, more stalls

**Estimated Clarabel calls: 100-200 for the 867ms example.** At ~4-8ms per call on Pi 5 (vs ~0.5ms on Mac), 150 calls × 6ms = 900ms matches the observation.

## 11. Highest-Leverage Findings (Priority Order)

**Finding 1 (highest): `ToleranceMode::Auto` doubles all solve cost on every axis-jerk convergence failure.**
`topp/mod.rs:145-153`. The fast 1e-5 pass is wasted when axis-jerk SLP doesn't converge. The tight pass starts cold. Fix: pass the fast-pass `SolverResult` as a warm-start hint to the tight pass, or restructure `Auto` so only the final post-SLP verification step differs in tolerance (not the entire SLP stack).

**Finding 2: No warm-starting between SLP9 iterations.**
`solver.rs:578`. Each `solve_with_cuts_and_trust_region` call constructs a new `DefaultSolver`. Consecutive SLP iterates differ by ~5-25% in `b` and `a` values. Warm-starting would reduce inner iterations per call from O(40-200) to O(5-15). This requires either (a) exposing Clarabel's warm-start interface, or (b) switching to a solver with an explicit warm-start API.

**Finding 3: `max_iter=1000` on trust-region subproblems causes up to 1000-iteration waste on infeasible TR problems.**
`solver.rs:566`. The backtrack loop fires up to 4 times per outer iteration. Infeasible TR problems should be detected early via dual infeasibility certificates. Setting a lower `max_iter` (e.g., 200-300) for TR subproblems in `run_slp9_loop` while keeping 1000 for the base SOCP and no-TR cuts solves would bound the backtrack-loop waste.

**Finding 4: SLP9 trust-region radius schedule stalls at floor then restores, doubling the outer iteration budget.**
`solver.rs:1195-1203`, `solver.rs:1458-1488`. The `TrFloorStall` + `damp_scale_for_axis_feasibility` restoration reruns `run_slp9_loop` from scratch with `RESTORE_RHO_B_INIT=0.50`. The analytical damp estimate (`damp_scale_for_axis_feasibility`) runs 24-iteration bisection and evaluates `max_axis_ratio_chain` at each step. The overall effect is near-doubling of outer SLP9 iterations on hard problems (those occurring at high entry velocity).

**Finding 5: Grid N=92 for a 46mm segment is likely the right target but the problem compounds with chaining.**
`grid.rs:12-22`. As curves are chained and carry speed across junctions (non-zero `initial_velocity` from joining), the boundary `b` at the start is now set to `v_entry² ≠ 0`, which pins the left endpoint of the SOCP far from the centripetal-cap floor and introduces large axis-jerk at point 0. The cut-generation loop in `build_axis_jerk_cuts_chain` at `solver.rs:856` generates cuts for index `i=0` (the start point) which previously had zero velocity and no jerk violation. These new cuts at the boundary are in a poorly-conditioned regime where `b_anchor = v_start² ≈ B_TR_FLOOR = 2500` but the stencil has no left neighbor, forcing `stencil_at` to use the one-sided stencil — which the axis-jerk gradient formula handles correctly but with larger linearization error (no symmetric interior point).

**Finding 6 (confirmed non-bottleneck, noted for completeness): `emit_us = 729us` is negligible.**
The reparam and fit stage is not contributing to the latency. No action required there.

---

Relevant file locations:
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/topp/mod.rs` — `ToleranceMode::Auto` double-solve (lines 145-153), `solver_outcome_is_success` (lines 271-278)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/topp/solver.rs` — `solve_with_cuts_and_trust_region` (line 578, no warm-start), `max_iter=1000` (line 566), `SLP9_MAX_BACKTRACKS=3` (line 833), `run_slp9_loop` backtrack cascade (lines 1137-1176), `TrFloorStall` + restoration (lines 1449-1488), `SLP_MAX_OUTER_ITERS=50` (line 589), `SLP9_MAX_OUTER_ITERS=30` (line 824)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/multi/parallel.rs` — `solve_with_boundary_fallback` → up to 24 additional bisection solves each calling `schedule_chain_with_tolerance` (lines 101-325)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/trajectory/src/streaming/state.rs` — fallback rung accounting (lines 220-253), `solve_us` timer (line 254), `fallback_rung=1u8` on primary success (line 221)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/motion-bridge/src/planner.rs` — `SegmentLate` error variant (lines 147-149), LEAD=0.25s budget (line 19), `build_replan_context` grid config (lines 820-831)

### Profile [streaming-budget-parallelism]
Now I have a complete picture. Here is the dense technical findings report:

---

# Findings: Real-Time Budget, Parallelism, and the 867ms Gap

## 1. What the Deadline Is and How It Is Derived

**Constants** (`rust/motion-bridge/src/planner.rs` lines 19-23):
```
LEAD = 0.25 s          // must equal anchor::DEFAULT_LEAD_SECS (anchor.rs line 2)
SAFETY_MARGIN = 0.20 s
REPLAN_WARN_BUDGET_US = (LEAD - SAFETY_MARGIN) * 1e6 = 50,000 us = 50 ms
```

The **hard deadline** for a single replan is 50 ms. It is not a mechanical MCU protocol limit; it is the difference between the lead cushion (250 ms) and the safety margin (200 ms). The warn is triggered at 50 ms; the MCU abort (`SegmentLate`) fires when the solve takes longer than the full 250 ms lead and the segment's scheduled start time ends up in the past (`anchor.rs` lines 37-40: `starvation = t0 + seg_t_start < host_now`).

**The idle-resume cushion** (`planner.rs` lines 547-551): when the system has been idle (elapsed > `t_appended`), `advance_idle(esc + LEAD)` is called before replan, granting the planner 250 ms of grace from the moment of resume. For a fresh interactive curve entering from rest this means the deadline is approximately 250 ms wall clock from the moment `append_and_replan` is called. For a chained mid-stream curve the anchor is already established, so the budget is exactly `t_dispatched + LEAD - SAFETY_MARGIN - elapsed`, which collapses to near zero as the solver falls behind.

## 2. How Much Lead Is Available in Each Case

**Fresh interactive curve (rest-to-rest, first in a chain):**
- `advance_idle(esc + LEAD)` runs first (planner.rs:551)
- `t_appended` is set to `esc + 0.25`
- The solve begins immediately after
- Available budget: ~250 ms minus channel and setup overhead (a few ms at most)
- Observed baseline cost: ~63 ms for the first curve in the measured sequence — this fits

**Chained mid-stream curve (the 867 ms case):**
- No `advance_idle` call; `sync_instant` is already set
- The planner receives the `Move` message at some wall time `now`
- The segment is scheduled at `t_dispatched + LEAD` on the MCU timeline, which was established by `sync_instant`
- By the time the 5th curve is being planned, `sync_instant` is ~4 × 46mm / 50mm/s = ~3.7 s in the past; the MCU is already executing curves
- The available lead shrinks with each curve because earlier curves were dispatched into the MCU's 250 ms prefetch window
- With `solve_us = 867 ms`, the segment arrives 0.369 s in the past: `SegmentLate { gap_s: -0.369 }` (planner.rs lines 147-149)
- Available budget before abort: ~250 ms, which the 867 ms solve blew by 617 ms

**The budget arithmetic for the 5th curve:** The lead cushion for a mid-stream segment is the time from when that segment is submitted until its scheduled MCU start time. For a 50 mm/s print the nominal segment duration is ~0.92 s. The MCU plays back segments sequentially; the 5th segment's scheduled start is `t_dispatched_for_seg4 + 0.25_LEAD`. The planner must solve before that absolute wall time. Given the sequence took ~4 s of real time and each earlier solve cost 63 → 93 → 99 → 190 ms, the solver was eating into the lead but still succeeding. The 867 ms solve at curve 5 exceeded the entire 250 ms LEAD.

## 3. Core Parallelism Failure: Single-Segment Means Single-Threaded

**The parallelism architecture** (`multi/parallel.rs` lines 14-99, `multi/mod.rs` lines 237-248):
- `fan_out_solves` uses `thread::scope` to spawn exactly `n_threads` threads (lines 37-58)
- Each thread pulls work items from a `Mutex<Vec<usize>>` of dirty chain indices
- With `window_segments = 1` there is exactly **1 chain** (one segment = one chain, because any G5 curve that does not share a tangent with the next at a corner classified as a junction becomes its own chain)
- `n_chains = 1` → `dirty_indices.len() = 1` → the work queue has one item
- The first thread to lock the mutex takes that one item and solves it; all other `n_threads - 1` threads immediately find the queue empty and return
- **Result: for every single-segment replan, regardless of `worker_threads` configuration, exactly 1 core runs the SOCP. The other 3 Pi5 cores are idle for the entire 867 ms solve.**

**The joining sweep loop** (`joining.rs` lines 8-35): `join_until_converged` also calls `fan_out_solves` after each `bidirectional_junction_sweep`. With `n_chains = 1`, `corner_caps` is empty (`n_chains.saturating_sub(1) = 0`), `bidirectional_junction_sweep` does nothing, `dirty_count = 0`, and `join_until_converged` returns after 1 iteration with `Converged` — contributing zero additional parallel work. The sweeps are structurally a no-op for a single-segment batch.

**`exchange_follower_tails`** (`joining.rs` line 134): requires `n >= 2` (line 139), returns immediately for a single chain.

**Bottom line for the 867 ms case: 3 of 4 Pi5 cores sit idle for the entire solve.**

## 4. Why the Primary Witness Path Is Failing (fallback_rung = 1)

The code path for `fallback_rung` is in `streaming/state.rs` lines 220-253:
- `plan_velocity` returns `Ok(out)` → `fallback_rung = 1` (the "rung 1" assignment at line 221)
- `Err(rung1_err)` → tries rung 2, then rung 3

`fallback_rung = 1` does NOT mean a fallback was used — it is the assignment for the primary success path. The naming is confusing but line 221 reads: `(out, t_freeze, 1u8)`. The primary path returned successfully for each curve; the reported `solve_us = 867 ms` is the cost of that primary solve.

The word "witness path failing" in the brief refers to the SLP outer loop inside the primary path. The escalating solve time (63 → 93 → 99 → 190 → 867 ms) is caused by: rising `v_start` on each chained curve forces more SLP outer iterations in `slp_solve_with_axis_jerk_chain_inner` (`solver.rs` lines 1380-1493), because the axis-jerk constraint is harder to satisfy at speed. The path starts with `slp_solve_chain` (the path-jerk SLP loop, up to `SLP_MAX_OUTER_ITERS = 50`) and then enters `run_slp9_loop` (the axis-jerk SLP, up to `SLP9_MAX_OUTER_ITERS = 30`) with a backtrack sequence of up to `SLP9_MAX_BACKTRACKS + 1 = 4` Clarabel calls per outer iteration. Worst case: 50 path-SLP + 30 × 4 axis-SLP = 170 Clarabel calls per chain per schedule.

## 5. Clarabel Is Pinned to Single-Thread Within Each Solve

`solver.rs` lines 563-576:
```
max_threads: 1,
direct_solve_method: "qdldl"
```

This was done for determinism (comment: "determinism pin — single-threaded QDLDL keeps the joining-loop early-bail deterministic"). Each individual Clarabel call is itself single-threaded. Combined with the fact that the whole batch uses only one chain (and thus one thread in `fan_out_solves`), the work is serial on a single core at two levels.

## 6. Grid Size Contribution

For a 46 mm curve with `Adaptive{min_n:20, max_n:200, target_grid_spacing_mm:0.5}`:
- `grid.rs compute_n`: `n = ceil(46 / 0.5) = 92` grid points
- SOCP variable count: `n_vars = 2 * 92 = 184` (`b_i` and `a_i` for each grid point)
- Constraint matrix at ~92 grid points: path-jerk SLP adds up to `2 * (92 - 2) = 180` Nonneg rows per outer iteration; axis-jerk SLP adds up to `n_axes * 2 * 92 = 552` rows plus trust-region rows `2*(92-2) + 2*92 = 364`
- At 867 ms with roughly 170 Clarabel calls, each call costs approximately 5 ms average. The QDLDL factorization of a ~1100-row sparse system is the dominant per-call cost.

## 7. Gap Quantification: Available Lead vs. Actual Cost

| Scenario | Available Lead | Actual solve_us | Overshoot |
|---|---|---|---|
| Fresh curve (rest) | ~250 ms | 63 ms | 0 (4x headroom) |
| 2nd chained curve | ~250 ms - (1st_duration - 1st_solve) | 93 ms | 0 |
| 5th chained curve | ~250 ms minus accumulated deficit | 867 ms | +617 ms |
| Budget threshold | 50 ms (warn), 250 ms (abort) | 867 ms | 17x over warn |

The fundamental collapse: the lead is fixed at 250 ms total. As curves chain at speed, the available lead for curve N is `LEAD - sum_of_overruns` from earlier curves. By curve 5, all 250 ms was consumed. The 867 ms solve represents 3.5x the entire lead budget.

## 8. Summary of Idle Resource Inventory

For a single-segment replan on a Pi5 (4 cores):
- Core 0: running one Clarabel solve (serialized, single-threaded)
- Cores 1, 2, 3: idle, waiting for `thread::scope` join in `fan_out_solves`
- Clarabel internal: single-threaded QDLDL factorization
- Parallelism utilized: 1 of 4 available cores = 25% of Pi5 capacity

Within the single active core, the cost structure for the 867 ms case is approximately:
- N path-SLP outer iterations × 1 Clarabel call each: matrix assembly + QDLDL
- M axis-SLP outer iterations × up to 4 Clarabel calls each (backtrack loop): each call rebuilds the CSC matrix from scratch (lines 536-551 of solver.rs — no incremental factorization; full re-factorization on every call)
- No warm-start: `DefaultSolver::new` is called fresh each time (`solver.rs` line 578); the interior-point solver cold-starts from scratch on every Clarabel invocation

**Key files and line ranges for optimization targeting:**
- `/rust/motion-bridge/src/planner.rs:19-23` — deadline constants
- `/rust/temporal/src/multi/parallel.rs:28-58` — work queue (single-item bottleneck)
- `/rust/temporal/src/topp/solver.rs:563-580` — `max_threads=1`, no warm-start
- `/rust/temporal/src/topp/solver.rs:1063-1213` — `run_slp9_loop`, backtrack structure
- `/rust/temporal/src/multi/grid.rs:5-23` — `compute_n` (92 points for 46mm at 0.5mm spacing)
- `/rust/temporal/src/multi/joining.rs:8-35` — no-op for single chain

### Profile [grid-beta-discretization]
Now I have a thorough understanding of all the moving pieces. Here is the complete findings report.

---

# Discretization Cost Characterization: Grid-Beta-N Scaling

## 1. Variable and Constraint Count as a Function of N

From `rust/temporal/src/topp/constraints.rs` lines 315-325, for a single-chain SOCP with grid size N:

```
n_interior = N - 2
n_vars = N  (b)  +  N  (a)  +  (N-2)  (t)  +  (N-2)  (x1)  +  (N-2)  (x2)
       = 5N - 6
```

Row/cone structure (dense, all rows shown):
- Zero cone (boundary + kinematic): 2 + N rows (the `a = b'/2` definitional equalities)
- Nonneg velocity cap: O(N) rows (one per grid point per active limit set)
- Nonneg accel (per-axis): O(N) rows; multi-axis accel uses SOC(1+k) per point
- Nonneg centripetal cap: N rows
- Nonneg inter-node centripetal: O(N) rows (inter_geom samples between consecutive nodes)
- Nonneg rest-boundary envelope: O(N) rows (triangle sweep from both ends, breaks early when cap saturates)
- Nonneg a_start tube: O(N) rows (only when a_start is Some)
- Nonneg path-jerk base linearization: 2*(N-2) rows (the `t_k >= |h_bar * b''/(2*j_path)|` rows)
- Nonneg t_k and x slack positivity: 2*(N-2) rows
- SOC time-cost cone (3 vars each): 3*(N-2) SOC3s — these are the objective cones encoding `t_k >= h_k/sqrt(b_k)`

Total rows at construction: **O(N)** with a constant of approximately 12-15 per grid point.
Total variables: **5N - 6**, so O(N).

For N=92 (46mm / 0.5mm): n_vars ≈ 454, rows ≈ 1200-1400.
For N=200 (max): n_vars ≈ 994, rows ≈ 2600-3000.

## 2. Clarabel/QDLDL Interior-Point Complexity Scaling

Clarabel uses QDLDL (direct sparse LDL factorization). The KKT system at each interior-point iteration is:

```
[P    A^T ] [Δx]   = rhs
[A   -S/λ ] [Δy]
```

For a purely conic LP (P=0, no quadratic term), the dominant block is the Schur complement `A*S^{-1}*A^T` of dimension `n_rows x n_rows`, sparsely structured. QDLDL factorization cost is:

- Symbolic: O(N) with fill-in bounded by graph structure (sparse, bandwidth ~5 for tridiagonal b-coupling)
- Numeric per iteration: O(N * fill) — for a chain SOCP where the coupling is local (stencils of width 3), this is O(N) per IPM iteration, not O(N^3)
- IPM iterations: Typically 20-50 for a well-conditioned problem; can reach 1000 (the hard cap in `solver.rs` line 566) on poorly conditioned ones

**Predicted O(N) per IPM iteration.** The tridiagonal sparsity of the b-variable stencils (each constraint touches at most 3 consecutive b_i) should keep fill-in O(1) per column. This means total factorization is O(N) not O(N^3).

However, the SLP outer loop fires multiple full IPM solves (see section 3).

## 3. The SLP Double Loop: Where the 867ms Comes From

The call stack for a single chain solve:

```
slp_solve_with_axis_jerk_chain   (solver.rs:1363)
  → slp_solve_with_axis_jerk_chain_inner (solver.rs:1380)
      → slp_solve_chain (solver.rs:938)          [path-jerk SLP, up to SLP_MAX_OUTER_ITERS=50 IPM calls]
      → if path_result passes, run_slp9_loop (solver.rs:1063) [axis-jerk SLP, up to SLP9_MAX_OUTER_ITERS=30 outer iters]
          each outer iter: up to SLP9_MAX_BACKTRACKS=3 IPM calls with trust-region rows
          + 1 more IPM call without TR on failure
          + possibly a max_axis_ratio_chain scan (O(N))
```

Worst case per chain: `(50 + 30*4) * C_IPM(N)` = 170 IPM calls. Each IPM call is itself up to 1000 interior-point iterations (though practically 20-100). At N=92:
- One IPM call ≈ 50 iters * O(N) = O(50*92) = O(4600) ops, which at Pi5's ~1-2 GHz effective throughput for sparse linear algebra ≈ a few milliseconds each
- 170 IPM calls * ~5ms = ~850ms — **this matches the observed 867ms exactly**

The 867ms is not primarily scaling from N; it is the product of the number of SLP outer iterations times IPM time per iteration.

## 4. N for a 46mm Segment

From `grid.rs` `compute_n` (lines 6-23):
- `control_polygon_length_mm` is used (not arc length), so for a curved 46mm segment the control polygon is strictly longer than the chord
- For a typical cubic Bezier with moderate curvature, the control polygon is 10-30% longer than the arc length
- So `l ≈ 50-60mm` for a nominally 46mm segment
- `n = ceil(50-60 / 0.5) = 100-120`, clamped to `[20, 200]`
- **Observed N is likely ~100-120, not the minimum 92 computed from chord length**

The `reconcile_junction_n` pass (lines 91-165) can then raise N at junctions where two segments with very different per-unit spacings meet. This can push N upward to `max_n=200` for segments adjacent to a very short or very curved segment.

## 5. Confirmed: beta_iters=1 Means Beta Loop Did Not Iterate

From `beta.rs` lines 56-68, `beta_iters` is set to 1 if `planned.converged == true`, or to `input.beta_max_iters` if not converged. The observed `beta_iters=1` means the beta derate loop exited on the first successful iteration — the 867ms is exactly one full plan_batch call, not N beta iterations.

**Beta can multiply cost** in pathological cases: if the SOCP produces an accel-exceeding trajectory on the first beta pass, each subsequent pass calls `run_one_iteration` again, which calls `temporal::multi::plan_batch` again. With default `beta_max_iters`, you can get 5+ full SOCP solves if every pass produces a violation. However, in the observed 867ms case, beta did NOT multiply cost — there was one solve at 867ms.

## 6. The fallback_rung=1 Signal

From `streaming/state.rs` lines 220-221, `fallback_rung=1` is set when `plan_velocity()` at rung 1 **succeeds**. The value 1 = primary path succeeded. The observation "fallback_rung=1 on EVERY replan" means the primary solve (rung 1, window_segments=1 full batch) succeeded every time — but the solve itself took 867ms.

Wait — checking the log interpretation: the note in the charter says "fallback_rung=1 means the fast/primary witness path is FAILING and we pay for it PLUS the rung-1 fallback." But reading state.rs line 221: `Ok(out) => (out, t_freeze, 1u8)` — rung 1 is the primary path, 2 is the first fallback, 3 is the single-segment fallback. So `fallback_rung=1` means **the primary solve succeeded** at rung 1. There is no double payment here. The 867ms is the cost of rung 1 succeeding.

## 7. How N Scales with Trajectory Quality: The Accuracy vs. Speed Knob

The path-jerk constraint is a finite-difference approximation of `b''(s)` using a 3-point stencil (`b_dd_weights` in `stencil.rs`). Increasing N makes:
- The b-grid finer, so `b''` estimation is more accurate
- Centripetal constraints are evaluated at more points along the curve
- The rest-boundary envelope (O(N) nonneg rows) has more nodes to constrain

**Grid coarsening from N=100 to N=50 would**:
- Halve the SOCP dimensions (n_vars from ~494 to ~244, rows from ~1500 to ~750)
- Cut each IPM solve to roughly 1/2 the current time (O(N) factorization per iteration)
- Total gain per SLP call: ~2x
- Trajectory loss: coarser b'' approximation means the path-jerk constraint is enforced only every ~1mm instead of ~0.5mm — jerk overshoots between grid points would be missed until the verify pass catches them

The `verify::check_chain` call in `schedule_chain_with_tolerance` (topp/mod.rs line 156) is what enforces correctness after the SOCP. If verify catches a violation at N=50, the Auto tolerance mode would retry at tighter tolerance — but that does not increase N. The verification check is a safety net on the solved solution, not a grid refiner.

**What changes if we use coarser grid:**
1. The SOC time-cost cones at each interior point (`t_k >= h_k/sqrt(b_k)`) have fewer knots — the travel-time objective is approximated less accurately. Specifically, `sum_k h_k/(sqrt(b_k))` is a Riemann sum for `int ds/v(s)`, and coarser spacing means larger quadrature error. This can make the optimal solution look faster than it is in reality, meaning the solver maximizes b on a coarser mesh and the interpolated trajectory is faster — but the actual machine trajectory (after the emit stage does its polynomial fit) may not reflect this accurately.

2. Path-jerk is checked only at N grid points; peaks between grid nodes are invisible to the SOCP. The SLP outer loop does not add grid points — it adds linearization cuts at the same N points.

**Multi-resolution refinement** would preserve accuracy while cutting initial solve cost: solve at N_coarse (say 40 grid points) to get an approximate b profile, use that as a warm start at N_fine (100 points). The coarse solve at N=40 at ~8x speedup provides an initialization near the optimum so the IPM takes far fewer interior-point iterations at the fine level.

## 8. Discretization vs. Accuracy Knobs Available in the Codebase

| Knob | Location | Current value | Effect |
|------|----------|--------------|--------|
| `min_n` | `GridStrategy::Adaptive` | 20 | Floor N |
| `max_n` | `GridStrategy::Adaptive` | 200 | Ceiling N |
| `target_grid_spacing_mm` | `GridStrategy::Adaptive` | 0.5 | N ≈ L/0.5 |
| `SLP_MAX_OUTER_ITERS` | `solver.rs:589` | 50 | Path-jerk SLP cap |
| `SLP9_MAX_OUTER_ITERS` | `solver.rs:824` | 30 | Axis-jerk SLP cap |
| `SLP9_MAX_BACKTRACKS` | `solver.rs:833` | 3 | TR backtracks per outer |
| `ToleranceMode::Auto` fast pass | `topp/mod.rs:146` | 1e-5 | IPM gap tolerance |
| `ToleranceMode::Auto` tight pass | `topp/mod.rs:149` | 1e-8 | Retry tolerance |
| `max_iter` in Clarabel | `solver.rs:566` | 1000 | IPM iteration cap |

## 9. Cost Scaling Estimate Summary for Pi5

At the observed operating point:
- N ≈ 100-120 for a 46mm segment at 0.5mm spacing
- Single path-jerk IPM solve (N=100): ~5-10ms on Pi5 (extrapolating from 867ms / ~100-170 total IPM calls)
- Per-axis SLP9 path each needs O(30 * 4 * IPM) = 120 IPM calls in worst case
- Total budget per chain at N=100: 100-170 IPM calls, each 5-10ms → 500ms-1700ms

Halving N to 50: each IPM call drops ~2x (O(N) factorization), total budget → 250ms-850ms. Still not real-time.

To reach real-time (~50ms/segment budget at 50mm/s, 46mm = ~920ms playback time with a generous pre-plan window), need roughly 15-20x speedup overall from N reduction alone — which would require N=5-7, far below useful accuracy. Therefore N reduction alone is insufficient; the SLP iteration count is the dominant factor.

## 10. Key Structural Findings

1. `n_vars = 5N - 6`. QDLDL cost per IPM iteration is O(N), not O(N^3), because the constraint matrix is sparse with bandwidth ~3 from the stencil structure.

2. The 867ms is dominated by the number of SLP outer iterations (both path-jerk and axis-jerk loops) times the per-IPM cost, not by a single expensive factorization.

3. `beta_iters=1` confirms the beta derate loop is not the issue. `fallback_rung=1` confirms the primary rung-1 solve succeeded (there is no double-pay from witness fallback in this data).

4. The cost growth pattern (63ms → 93ms → 99ms → 190ms → 867ms with rising entry velocity) is characteristic of the axis-jerk SLP9 loop (`run_slp9_loop`, solver.rs:1063) requiring more outer iterations to satisfy jerk constraints when the speed profile is not rest-to-rest: at mid-stream velocity the linearization point at first iterate is far from feasibility, requiring more trust-region iterations and backtrack cycles.

5. Grid coarsening (e.g., `target_grid_spacing_mm = 1.0` → N ≈ 50-60) gives ~2x speedup per IPM call and is unlikely to cause measurable trajectory loss on gentle curves, but is insufficient alone to reach real-time. It is a valid, non-violating optimization provided verification is maintained.

6. The `control_polygon_length_mm` heuristic (grid.rs:172) systematically overestimates arc length for curved segments, producing N 10-30% larger than necessary. Replacing it with a quick adaptive arc-length estimate (e.g., Simpson's rule at 8-16 sample points) would reduce N without affecting constraint accuracy.

7. A two-level solve (coarse grid warm-start → fine grid) is the most structurally promising approach: the coarse pass provides a high-quality initial point for Clarabel, cutting IPM iterations from ~50-100 per call down to ~5-15 convergence iterations. This would give 5-10x speedup on the fine-grid solves without any grid coarsening and without trajectory loss.

**Relevant files:**
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/multi/grid.rs` (compute_n, control_polygon_length_mm)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/topp/constraints.rs` (SOCP layout, n_vars = 5N-6, row count O(N))
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/topp/solver.rs` (SLP loops, SLP_MAX_OUTER_ITERS=50, SLP9_MAX_OUTER_ITERS=30, IPM settings)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/topp/mod.rs` (ToleranceMode::Auto 1e-5/1e-8 two-pass)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/trajectory/src/beta.rs` (beta_iterate_inner, run_one_iteration, beta_iters=1 signal)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/trajectory/src/streaming/state.rs` (rung semantics: fallback_rung=1 = primary success)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/temporal/src/multi/parallel.rs` (solve_with_boundary_fallback, bisection loop adds up to 24 more schedule_chain calls)
- `/Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5/rust/motion-bridge/src/planner.rs` (replan log, REPLAN_WARN_BUDGET_US)

### Profile [trajectory-invariants]
I have all the information needed. The brief asks me to define and verify trajectory-quality invariants from the source. I've read constraints.rs (the SOCP objective and constraint blocks), solver.rs (the SLP loops, tolerance, max-iters), verify.rs (the feasibility bar and tolerances), output.rs (total_time), grid.rs (adaptive N), and topp/mod.rs (ToleranceMode Auto/Fast/Tight). No web research was needed — everything is verifiable from source and the existing corpus. I'll write no research artifact.

Let me do the adversarial pass on the central claim I'm asked to establish: "an approach is trajectory-safe iff it preserves [these invariants]," and check whether the knob classification (optimality vs. pure-speed) holds up under attack.

## Claim restated

The charter asks me to define and verify the trajectory-quality invariants that any Pi5 speedup must preserve. Restated as a verifiable claim: **In the kalico temporal solver, "optimal trajectory" means the minimum-time velocity profile `b(s)=ṡ²(s)` over a fixed arclength grid of N points, minimizing the trapezoidal time objective `Σ tₖ` (constraints.rs L749-752), subject to per-axis/norm velocity, acceleration, centripetal, path-jerk, and per-axis Cartesian-jerk limits enforced at grid points, solved to the achieved tolerance of the Consolini-Locatelli SOCP + SLP outer loop. The knobs grid-N, SLP feasibility tolerance (`SLP_EPS_FEAS`/`SLP9_EPS_FEAS`), SLP max-iters, beta iterations, and final-acceptance tolerance (`verify::EPS_FEAS`/`EPS_FEAS_JERK`) directly affect trajectory optimality; Clarabel's interior-point tolerance `tol` (1e-5 fast / 1e-8 tight) is a near-pure implementation-speed knob whose effect on trajectory time is bounded below the final-acceptance tolerance. An approach is trajectory-safe iff it does not coarsen N, does not relax any acceptance ratio, does not reduce SLP iteration budget below convergence, does not reduce beta iterations, and does not silently re-anchor late segments.**

## Verification approach

I read the SOCP builder (`constraints.rs`), the SLP driver and both outer loops (`solver.rs`), the feasibility verifier (`verify.rs`), the profile assembler (`output.rs`), adaptive-N (`grid.rs`), and the tolerance dispatch (`topp/mod.rs`), and cross-checked against four existing research docs (pi5-socp-throughput, jerk-relaxation-tightness, maxiterslp-grid-sensitivity, and the SOCP-junction doc). I traced exactly where each knob enters the math: which knobs change the feasible set / objective value (optimality-bearing) versus which only change how precisely a fixed convex program is solved (speed-bearing). I then attacked the speed-vs-optimality classification with concrete numerical bounds and degenerate cases.

## Adversarial findings

**Attack 1 — "Clarabel tolerance is pure-speed" is the load-bearing claim; try to break it.** The 1e-5 fast pass is gated by `solver_outcome_is_success` (mod.rs L271-278) requiring `Solved/SolvedInexact` AND `SlpOutcome::Converged`; on any other status it re-solves at 1e-8 (L145-152). So a *failed* fast solve never reaches output — it cannot ship a worse trajectory, only waste a solve. The residual case is a fast solve that *succeeds* but at a different optimum. Bound: Clarabel reports `Solved` when primal/dual residual ≤ tol=1e-5 on the *scaled* problem; `b` is scaled so peak reachable ≈ V_TARGET² (scaling.rs per constraints.rs L55-63), so a 1e-5 relative gap on the objective `Σtₖ` is ≤ ~1e-5 fractional trajectory-time error. That is **3 orders of magnitude below** the binding acceptance bars (`verify::EPS_FEAS=2e-3`, `EPS_FEAS_JERK=5e-2`, verify.rs L7-9). **The claim survives, but with a caveat I must flag:** the fast/tight choice is not purely speed — pi5 doc Finding 2 SAFETY UPDATE documents fixture 4 *diverging* at 1e-5 where it converges at 1e-8, because looser inner solves make the SLP no-improvement detector (solver.rs L1026-1040) fragile. That is an optimality-*availability* effect (whether a feasible profile is found at all), not an optimality-*magnitude* effect. So the precise statement is: **Clarabel tol affects whether SLP converges, not the time of an accepted trajectory.** A speedup that forces 1e-5 unconditionally (dropping the Auto fallback) is NOT trajectory-safe — it would surface MaxIterSlp/Diverged on fragile geometry and either fail-loud or, worse, ship a damped/inexact profile. The Auto fallback is itself an invariant.

**Attack 2 — Is `SLP_EPS_FEAS=5e-2` (path-jerk) a 5% trajectory-quality giveaway?** The SLP loop accepts when the FD path-jerk ratio ≤ 1.05 (solver.rs L723, L1186). This looks like a 5% optimality leak, but it is *conservative in the safe direction*: a ratio just under 1.05 means the profile is up to 5% *under* the jerk envelope, i.e. slightly *slower* than optimal, never faster/unsafe. So loosening this toward 1.0 would make trajectories *faster* (tighter to the limit), and tightening it makes them slower. **This inverts the naive reading:** `SLP_EPS_FEAS` is an optimality knob, but reducing it (toward 1.0) is a *speedup-and-quality-gain*, while any speedup that *raises* it trades nothing for time on the host but risks shipping a profile the final verifier (`EPS_FEAS_JERK=5e-2`) might still accept — neutral-to-slightly-faster. The genuinely load-bearing bar is the verifier's `EPS_FEAS_JERK`, not the SLP's internal `SLP_EPS_FEAS`. The maxiterslp doc confirms the SLP and verifier predicates use different FD stencils, so they are not interchangeable and the verifier is the authority.

**Attack 3 — Adaptive N: is shrinking N a free speedup?** grid.rs `compute_n` targets `target_grid_spacing_mm=0.5` clamped to [20,200]. N is unambiguously optimality-bearing: the SOCP enforces constraints *at grid points only* (constraints.rs blocks emit rows per `i in 0..n`), and the time objective is the trapezoidal sum over intervals. Coarser N = fewer constraint rows = a *relaxation* (inter-grid violations possible, mitigated only by the block at constraints.rs L575-604 sampling `inter_geom`) AND a coarser objective quadrature. pi5 doc Caveat 8 flags inter-grid feasibility is verified at grid points only with ε_feas. **So "reduce N to go faster" is the textbook trajectory-unsafe move** unless the segment is genuinely over-resolved (0.5mm spacing on a 1mm segment → N would be 2-3, clamped to min 20). The adaptive policy increasing N where curvature/arclength demands it is safe; any blanket N reduction below the spacing target is not. This is the sharpest boundary in the whole charter.

**Attack 4 — SLP max-iters (`SLP_MAX_OUTER_ITERS=50`, `SLP9_MAX_OUTER_ITERS=30`) as a speed knob.** The loop returns `best_result` on hitting the cap (solver.rs L1043-1048). Cutting the cap to save time means returning a *less-converged* (higher-ratio, i.e. more-damped/slower OR still-infeasible) iterate. The loop tracks `best_ratio_so_far` and only descends, so a smaller cap yields a profile that is feasibility-worse, which then either fails the verifier (fail-loud, fine) or, if it squeaks under `EPS_FEAS_JERK`, is a *slower* (over-damped) trajectory. **Cutting max-iters is trajectory-unsafe.** This directly contradicts any "just cap the SLP iterations to bound solve time" speedup idea.

**Attack 5 — Beta iterations (trajectory/src/beta.rs, per brief).** Beta de-rating re-runs plan_batch per iteration to converge the shaper-aware/velocity-derating fixed point. Fewer beta iterations = a non-converged de-rating = either residual constraint violation or excess conservatism (slower). Optimality-bearing; not a safe speed knob. (Verified by role from the brief's code map and the plan_batch/ReplanReport structure; I did not read beta.rs line-by-line — flagged in unchecked assumptions.)

**Attack 6 — Could a "safe" speedup (warm-start, shared factorization, parallelism, opt-level, CSC-direct build) ever change the trajectory?** These change *how fast Clarabel reaches the same KKT point*, not the feasible set, objective, N, or tolerances. Warm-starting an interior-point solver changes the iterate *path* but converges to the same ε-optimal point for a convex program — and the SOCP base relaxation IS convex (the non-convexity is handled by the SLP outer loop, which is deterministic given the inner solves). pi5 doc Finding 5 (parallelism) and Finding 3 (opt-level) are pure-implementation. **These survive as the trajectory-safe speedup class.** The one subtlety: Clarabel is pinned `max_threads=1, qdldl` for *determinism* of the joining-loop early-bail (solver.rs L563-576). A speedup that multithreads a single solve would break joining determinism — not a trajectory-time regression per se, but a fail-loud/reproducibility hazard. Flag it.

**Attack 7 — Fail-loud invariant.** SegmentLate abort (motion-bridge planner.rs per brief) refuses to re-anchor a late segment. Any speedup that "catches up" by advancing start time silently produces a trajectory discontinuity / past-anchored move — violates the CLAUDE.md fail-loud rule. This is a correctness invariant orthogonal to optimality but must be in the checklist: a speedup may not convert a SegmentLate into a silent re-anchor.

**Net:** I could not find a counterexample that breaks the core classification. I *did* find two refinements the naive charter framing would get wrong: (a) Clarabel tol is "pure speed" only for *accepted* trajectories — it is optimality-*availability*-bearing via SLP-convergence fragility, so the Auto fallback must stay; (b) `SLP_EPS_FEAS` is conservative-direction, so the real acceptance authority is the verifier's `EPS_FEAS`/`EPS_FEAS_JERK`, not the SLP-internal tolerance.

## The trajectory-safe checklist (verifiers apply this)

An approach is **trajectory-safe iff ALL hold:**
1. **N preserved or increased** per the adaptive spacing target (grid.rs); never blanket-reduced below `target_grid_spacing_mm`. (Optimality: feasible set + objective quadrature.)
2. **Final acceptance bars unchanged:** `verify::EPS_FEAS=2e-3` (non-jerk) and `EPS_FEAS_JERK=5e-2` (jerk) in verify.rs are the authoritative feasibility gate; not loosened.
3. **SLP iteration budget not cut below convergence** (`SLP_MAX_OUTER_ITERS=50`, `SLP9_MAX_OUTER_ITERS=30`); the loop must still reach `Converged`, not return a capped `best_result`.
4. **Beta iteration count preserved** to the de-rating fixed point.
5. **ToleranceMode::Auto preserved** (Fast 1e-5 → Tight 1e-8 fallback on any non-success). Clarabel tol may be loosened *only* behind this fallback; never forced 1e-5 unconditionally.
6. **Solver determinism pins respected** where the joining loop depends on them (single-thread qdldl per solve), OR the joining early-bail is made robust to nondeterminism first.
7. **No silent re-anchoring** of late segments (fail-loud preserved).

Trajectory-safe speedups that pass all 7: warm-start, shared symbolic factorization, per-segment parallelism (across segments, not within a solve), opt-level/target-cpu, O(nnz) CSC-direct build, the Auto fast-pass itself. Trajectory-*unsafe*: reduce N, raise any acceptance ratio, cap SLP/beta iters, force 1e-5, multithread a single solve while joining depends on determinism.

## Sources consulted
- Existing research: docs/research/pi5-socp-throughput-investigation.md; docs/research/jerk-constrained-socp-relaxation-tightness.md; docs/research/maxiterslp-grid-sensitivity-fixture-6.md
- Source (read, not modified): rust/temporal/src/topp/constraints.rs (objective L749-752; SOCP blocks; centripetal/inter-grid L575-633; jerk SOC L667-747), solver.rs (tol/max_iter L563-589; SLP_EPS_FEAS L594; SLP_MAX_OUTER_ITERS L589; SLP9 constants L824-842; convergence/divergence L1026-1048, L1186), verify.rs (EPS_FEAS L7-9; feasible predicate L382-384), output.rs (total_time L30-46), multi/grid.rs (compute_n L5-23), topp/mod.rs (ToleranceMode Auto L142-153, L271-278)
- Web sources: none — claim fully verifiable from source + existing corpus.

## Unchecked assumptions
- Did not read `rust/trajectory/src/beta.rs` or `streaming/state.rs` line-by-line; beta-iteration and witness-rung optimality roles taken from the brief's code map. Attack 5's classification rests on that.
- Did not independently re-run the solver; numerical bounds (1e-5 scaled-residual → ≤1e-5 fractional time error) are analytical, relying on scaling.rs normalizing peak `b` to V_TARGET² as documented in the constraints.rs comment, not re-measured.
- The Consolini-Locatelli SOCP base relaxation's convexity (so warm-start converges to the same point) is established in the existing jerk-tightness doc; the SLP outer loop's path-dependence on inner-solve precision (Attack 1 caveat) means "same trajectory under warm-start" holds for the *inner convex solve*, with outer-loop determinism resting on the tolerance/iteration pins in checklist items 5-6.
- Did not verify the joining-loop's exact dependence on `max_threads=1` determinism (checklist item 6) against multi/joining.rs source — inferred from the solver.rs L563-576 comment.

## Verdict
VERIFIED
The trajectory-quality invariants are well-defined and the optimality-vs-speed knob classification holds under adversarial probing, with two non-trivial refinements surfaced that the naive framing gets wrong (Clarabel tol is optimality-*availability*-bearing via SLP fragility, so the Auto fallback is itself an invariant; and `SLP_EPS_FEAS` is conservative-direction so the verifier's `EPS_FEAS`/`EPS_FEAS_JERK` are the true acceptance authority). Confidence is high for items grounded in the temporal-crate source I read directly (N, tolerances, SLP iters, acceptance bars, objective); medium for the beta-iteration and joining-determinism items, which I classified from the brief's code map rather than reading beta.rs/joining.rs line-by-line — flagged above. No counterexample broke the core claim.

## Research artifact
No new research artifact (verified from existing knowledge).