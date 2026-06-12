---
topic: TOPP frozen-time-map fixed-point iteration for convolution/input-shaping constraints
created: 2026-06-12
last_updated: 2026-06-12
verified_claims:
  - 2026-06-12 INCONCLUSIVE (leaning conditionally-true with no global guarantee) — "Fixed-point iteration over a frozen time map (re-freeze t̄ from current b̄, rebuild averaging-kernel convolution constraint rows, re-solve) converges for averaging-type kernels in TOPP." No published convergence proof exists for this exact scheme; structural analysis shows it is a non-expansive-but-not-contractive map in general, with concrete divergence/oscillation regimes (kernel support spanning a stop-to-stop chain; near-singular b near stops).
sources:
  - https://arxiv.org/pdf/1707.07239
  - https://arxiv.org/pdf/1312.6533
  - https://www.sciencedirect.com/science/article/pii/S0005109824003583
  - https://arxiv.org/pdf/2312.02944
  - https://arxiv.org/abs/2404.16826
  - https://www.semanticscholar.org/paper/Time-Optimal-Path-Following-for-Robots-With-Using-Debrouwere-Loock/5dfe332771b4fa75f46a399ddf1242fa57586901
---

# TOPP frozen-time-map fixed-point iteration for convolution/shaping constraints

## Summary

The kalico scheme — freeze sample times t̄_i from the current iterate b̄ (b = ṡ²), rebuild
convolution-window constraint rows W_ij = K(t̄_i − t̄_j)·q_j for an averaging-type kernel
(compact support, integral 1) on those frozen times, re-solve the SOCP, repeat — has **no
published convergence proof** for this exact construction. The closest prior art (TOPP-RA
finite-N non-monotonicity; Verscheure/Debrouwere velocity-dependent constraints that destroy
convexity; alternating segment-time-vs-coefficient quadrotor schemes) establishes that maps of
this shape are **non-expansive in benign regimes but neither globally contractive nor
guaranteed-monotone**. Averaging kernels help (they are smoothing, low-pass, and the time map
is monotone in b), but they do not close the gap to a contraction. Concrete divergence regimes
exist: kernel support wider than a stop-to-stop chain, and near-singular b near rest points
where t̄ is hypersensitive to b. The verdict is therefore: **the claim is plausibly true in the
common regime but is not provably true in general, and the "known counterexample shapes" half of
the disjunction is correct.** Retry-cap-then-fail-loud is an acceptable *safety* posture but is
**not** an acceptable *throughput* posture as a permanent design, because the divergence regimes
are not exotic — they coincide with short moves between full stops, which are common in real
slicer output (small perimeters, retraction-bracketed travels).

## Verified claim — 2026-06-12

**Claim (verbatim):** "Fixed-point iteration over a frozen time map — re-freezing sample times
t̄_i from the current iterate b̄ (b = ṡ²), rebuilding convolution-window constraint rows
(input-shaper kernels, averaging-type, compact support, integral 1) on those times, and
re-solving — converges for averaging-type kernels in time-optimal path parameterization; or,
known counterexample shapes exist (e.g. kernel support wider than a chain between stops)."

**Verdict:** INCONCLUSIVE for the unconditional first disjunct; the disjunction as a whole holds
because the second disjunct (counterexample shapes exist) is **confirmed**.

### Verification approach

1. Surveyed existing kalico research: `jerk-constrained-socp-relaxation-tightness.md` (SLP outer
   iteration — a *different* iteration: linearizes 1/√b at a fixed grid, time map is not
   re-frozen), `condensed-smooth-chain-socp-junction.md` (records the TOPP-RA finite-N
   non-monotonicity caveat), `bspline-polynomial-convolution.md` (convolution support widens by
   kernel support — directly relevant to the counterexample geometry).
2. Web survey for the exact scheme. No paper iterates a *frozen time map* to handle
   convolution/shaping constraints inside TOPP. The structurally-adjacent results are TOPP-RA
   (Pham, arXiv:1707.07239), velocity-dependent-constraint TOPP (Verscheure/Debrouwere), the
   jerk-convexity result (Consolini-Locatelli, S0005109824003583), and alternating
   time/coefficient trajectory schemes (arXiv:2312.02944, arXiv:2404.16826).
3. Analyzed the fixed-point map structure directly.

### The map, made explicit

Define the iteration operator T: b̄ ↦ b̄'.

- **Time map** (trapezoid quadrature on b): t̄_i(b̄) = Σ_{j<i} 2·Δs_j / (√b̄_j + √b̄_{j+1}).
  This is the dominant nonlinearity. ∂t̄_i/∂b̄_k = −Δs·b̄_k^{−1/2}/(√b̄_j+√b̄_{j+1})² type terms;
  as b̄_k → 0 (a stop), ∂t̄_i/∂b̄_k → ∞. The time map is **monotone** (more speed ⇒ less time)
  and **smooth on b > 0**, but its Lipschitz constant blows up as any b̄_k → 0.
- **Window rebuild:** W_ij = K(t̄_i − t̄_j)·q_j with K averaging (K ≥ 0, ∫K = 1, compact
  support [−h, h]). The constraint rows cap |r|·‖(W q)_i‖ etc. K being an averaging kernel makes
  W a row-stochastic-like smoothing operator: ‖W‖_∞ ≤ 1, and small perturbations in t̄ produce
  perturbations in W bounded by (Lipschitz-K)·(perturbation in t̄). For a trapezoid/box kernel,
  K is Lipschitz with constant ~1/h².
- **Solve:** the SOCP that maps the frozen W to b̄' is a continuous (piecewise-smooth) selection.

### Why averaging kernels help but do not guarantee contraction

The favorable facts for the first disjunct:

1. **Averaging kernels are non-expansive smoothers.** ‖W‖ ≤ 1 means the shaped-velocity
   constraint never amplifies; it can only relax or tighten by a bounded amount. This rules out
   the simplest blow-up mode.
2. **Monotone time map + monotone constraint dependence** gives the iteration a
   partial-order/Tarski flavor: tighter shaped-velocity caps ⇒ smaller b̄' ⇒ larger t̄ ⇒ (for an
   averaging kernel) the difference t̄_i − t̄_j over a fixed index gap grows, which generally
   *loosens* the cap (the kernel sees a wider spread). That is a **negative feedback** sign,
   which is exactly the condition under which fixed-point iteration tends to converge. This is the
   strongest argument for the first disjunct and is why the scheme converges in the common regime.

The facts that defeat a *global* guarantee:

3. **The Lipschitz constant of T is unbounded** as b̄ → 0 near stops. Composition
   (time map ∘ kernel ∘ solve) has Lipschitz constant L_t · L_K · L_solve; L_t → ∞ near stops and
   L_K ~ 1/h². There is no a-priori bound < 1, so Banach fixed-point does not apply and no
   contraction certificate exists. The negative-feedback sign in (2) is a *tendency*, not a
   *bound*: a single index pair straddling a stop can flip the sign locally when the kernel
   support [−h,h] reaches across the stop into a neighboring chain moving in a different
   direction, because then increasing t̄-spread pulls in *opposite-sign* axis velocity and the cap
   tightens instead of loosening — destroying the monotone negative feedback.

4. **Finite-N non-monotonicity (Pham/TOPP-RA caveat, already in the corpus).** At finite grid the
   maximal-transition map is "not monotonic over the whole controllable set." The frozen-time
   rebuild inherits this: the discrete optimum the iteration chases can itself shift
   non-monotonically with N, so the iterate can chatter between two grid-adjacent optima without
   settling — an oscillation, not a divergence, but it still trips an 8-refreeze cap.

### Known counterexample shapes (second disjunct — CONFIRMED)

Adversarial constructions where T is not a contraction:

- **Kernel support wider than a stop-to-stop chain.** If h (half-support of K) exceeds the time
  length of a chain bounded by two full stops, the window at an interior sample reaches across the
  stop into the neighboring chain. The shaped-velocity at that sample then depends on motion in a
  segment that the current chain's b̄ does not control, and whose direction may oppose it. The
  negative-feedback sign of (2) is lost; the rebuilt W can tighten the cap when the iterate slows
  down, which slows it further, until the iterate collapses toward b = 0 (chatter or monotone
  collapse). This is the user's own example and it is mathematically real. (Supported by
  `bspline-polynomial-convolution.md`: convolution support genuinely widens by kernel support, so
  "support reaches the neighbor" is not hypothetical — it is the generic short-move case.)

- **Near-singular b at a stop.** A rest-to-rest move with a very short fast middle: b̄ is ~0 at
  both ends and the time map's Jacobian is enormous there. A small change in the interior b̄
  produces a large change in t̄ at the far end, which moves a far window's contents discontinuously
  (a sample crosses a kernel breakpoint), producing a large jump in W and hence in b̄'. This is a
  Lipschitz-constant-blow-up oscillation independent of kernel width. Short perimeters and
  retraction-bracketed travels in real slicer output land here.

- **Two near-equal grid optima (finite-N chatter).** Per Pham's caveat, two grid-adjacent feasible
  parameterizations with near-equal objective can alternate under refreeze. Bounded, but
  non-converging within a small cap.

None of these requires pathological geometry; the first two are common short-move printing cases.

### Recommendation for the divergence posture

- **Retry-cap-then-fail-loud is correct as a safety net and must stay.** It satisfies the
  CLAUDE.md "fail loudly" rule and catches genuinely ill-posed inputs. Keep it.
- **It is NOT acceptable as the *only* posture**, because the divergence regimes coincide with
  common print features (short moves between stops, retraction travels). A permanent "8 refreezes
  then die" on those would surface as intermittent print-time failures on ordinary G-code, which
  violates the "throughput is non-negotiable / never ship a measurably slower (here: failing)
  trajectory" constraint by failing outright. A fallback is needed for the non-converging branch.
- **Recommended fallback (literature-grounded), in priority order:**
  1. **Damped / averaged update (Mann–Krasnoselskii iteration):** b̄^{k+1} = (1−α) b̄^k + α T(b̄^k)
     with α ∈ (0,1). For a non-expansive T on a convex set, the averaged iteration converges to a
     fixed point even when T itself is not a contraction (Krasnoselskii–Mann theorem). Since
     averaging kernels make the constraint operator non-expansive (fact 1), this is the principled
     fix and is cheap — it only changes the update rule, not the SOCP. This is the single most
     defensible recommendation.
  2. **Anderson acceleration** over the refreeze fixed-point map to damp chatter and accelerate
     the benign regime; falls back to Picard if the mixing is rejected.
  3. **Treat the shaping constraint via SLP/SCvx instead of frozen-time refreeze** for the chains
     that fail to converge — linearize the t̄-dependence of W once per outer iteration (a Taylor
     cut on t̄_i − t̄_j w.r.t. b̄) and add it as a cut, rather than fully re-freezing. This converts
     the discontinuous "recompute which kernel piece each sample falls in" into a smooth local
     model and is the same architecture already endorsed for the jerk relaxation
     (Lee 2024 SLP / SCvx, arXiv:2404.16826), so it reuses machinery.
  4. **Detect the support-spans-a-stop geometry up front** (h > chain time length) and split the
     window at stop boundaries (zero-pad past the stop, since a full stop genuinely decouples the
     two chains' shaping) rather than letting the kernel reach across. This removes the primary
     counterexample class structurally.

### Sources

- https://arxiv.org/pdf/1707.07239 — Pham, TOPP-RA — retrieved 2026-06-12 — finite-N
  non-monotonicity of the maximal-transition map; basis for the chatter counterexample.
- https://www.sciencedirect.com/science/article/pii/S0005109824003583 — Consolini-Locatelli, "Is
  time-optimal speed planning under jerk constraints a convex problem?" — retrieved 2026-06-12 —
  non-convexity of velocity-coupled constraints; context for why no single convex solve fixes it.
- https://arxiv.org/pdf/1312.6533 — Pham, general TOPP implementation — retrieved 2026-06-12 —
  robustness/failure-mode framing.
- https://arxiv.org/pdf/2312.02944 — alternating peak-optimization (fix segment times, optimize
  coefficients, then update times) — retrieved 2026-06-12 — closest published alternating-time
  scheme; uses damped gradient updates on the time variable, not raw Picard refreeze.
- https://arxiv.org/abs/2404.16826 — Successive Convexification with continuous-time constraint
  satisfaction — retrieved 2026-06-12 — basis for the SCvx-cut fallback (3).
- https://www.semanticscholar.org/paper/Time-Optimal-Path-Following-for-Robots-With-Using-Debrouwere-Loock/5dfe332771b4fa75f46a399ddf1242fa57586901
  — Debrouwere et al., convex-concave TOPP via SCP — retrieved 2026-06-12 — velocity-dependent
  constraints destroy convexity; SCP (not Picard) is the published remedy.

### Caveats / unchecked assumptions

- I did not see kalico's exact constraint-row algebra (per the verifier rule against reading
  source). The sign analysis in fact (3)/(counterexample 1) assumes the cap is
  |r|·‖W·(axis velocity)‖ ≤ const with W an averaging operator; if the actual rows have a
  different sign structure the "negative feedback" argument may strengthen or weaken.
- The Krasnoselskii–Mann argument requires T non-expansive on a *convex* domain and the
  shaped-constraint operator non-expansive. The latter holds for averaging kernels in the t̄-frozen
  step; whether the *composed* map (including the SOCP solve selection) is non-expansive is not
  proven here — it is the natural next verification target.
- No numerical reproduction was run. The counterexample shapes are analytically argued, not
  simulated. A fixture (h > chain-time short move between two stops) would confirm divergence
  empirically and is the recommended follow-up.
- "Converges" was read as "the Picard refreeze iterate settles within the 8-cap." If the intended
  meaning is "a fixed point exists" (weaker), that is more defensible — Brouwer gives existence on
  the compact feasible set — but existence of a fixed point does not imply Picard reaches it.
