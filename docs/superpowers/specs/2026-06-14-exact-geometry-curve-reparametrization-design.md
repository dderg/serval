# Exact-Geometry Curve Reparametrization — Design

**Status:** Design (approved approach; spike-validated)
**Date:** 2026-06-14
**Scope:** Fix the planner so it never produces an unfittable curved segment, by keeping the curve geometry exact through the reduce/fit pipeline. Bug-fix + small re-architecture of the arc-length reparametrization stage. Not a new feature.

---

## 1. Problem

Typing a `G5` (cubic Bézier) at the console crashes the planner. On the Trident bench a single rest-to-rest `G5` produced:

```
WitnessFallbackFailed { rung1/rung3: FitFailure {
  index: 30, detail: ToleranceNotReached { achieved_mm: 0.00093, at_degree: 5 } } }
→ "single-segment rest-to-rest plan unsolvable" → planner_fatal → abort
```

The abort itself is **correct** — fail-loud is the design, and an unfittable segment is a real defect, not something to tolerate or skip. The defect is that we *produced* an unfittable segment at all. This is a **core curved-motion bug**, not a G5-specific one: every non-straight segment hits it; straight lines are immune. G5 is simply the first thing that feeds the planner a curve.

### Root cause (reproduced byte-for-byte, offline)

The failing stage is purely **geometric** — no dynamics, no time, no input-shaping involvement. In `trajectory/src/reparam.rs::compose_segment`, each path piece is reparametrized by:

```
fit_x_to_arc_length_piece(curve, table, s_lo, s_hi, target_degree=3, max_degree=5, tol)
```

This fits **position-as-a-function-of-arc-length `x(s)`** as a polynomial (degree capped at 5, *no subdivision fallback*) against a hard-coded `arc_fit_tolerance = 1e-4` mm (0.1 µm, set in `beta.rs`). For a curved segment, `x(s) = x(u(s))` (the exact cubic geometry composed with the nonlinear arc-length inverse `u(s)`) cannot be represented by a single degree-≤5 polynomial to 0.1 µm, and there is no recourse → hard error → abort.

Evidence (all via `cargo nextest`, scratch test `trajectory/tests/experiment_smooth_curve_fit.rs`):

- **Exact reproduction.** A smooth 50 mm arch reproduces the bench's identical `segment 30, achieved 0.0009304…`.
- **Feedrate-independent, shaped == unshaped, byte-identical** → not dynamics, not the shaper.
- **`index ≠ 0`** in the error → the failure is in `compose_segment` (Stage B), not the downstream time-domain Hermite fit (`fit_and_split`, which works).
- **Raising the degree cap (5→15): no help** (Lagrange/Runge ill-conditioning).
- **Subdivision alone: plateaus ~2 µm, does not converge.**
- **Straight lines are immune** — their `x(s)` is exactly linear (degree 1). This is precisely why `G1` works and every curve crashes.

**Plain English:** before sending a move to the motors, the planner re-describes the curve as "position versus distance-travelled," is allowed only a low-order formula with one try, and is held to an absurd 0.1 µm standard. A straight line re-describes trivially. A real curve can't, there's no plan B, and giving up means the whole engine stops.

---

## 2. Goal & constraints

- A smooth (cusp-free) curve must **always** plan successfully.
- **Trajectory optimality is preserved.** This change is representation *accuracy* of the geometry, not the speed schedule — it costs no trajectory time. (Per CLAUDE.md: we never trade trajectory quality for easier planning; this trades nothing.)
- **No throughput regression.** The bench was already emitting `replan_overrun`; the fix must be at least as fast, so the arc-length inversion is computed efficiently (per-segment, not per-piece).
- **Fail-loud is retained for genuinely degenerate geometry.** A true cusp (zero tangent, `|x'(u)| → 0`) is physically a mandatory stop and is legitimately unplannable as a smooth segment; it must still error — ideally with a clearer "zero-tangent / cusp" message — rather than silently degrade.

---

## 3. Approach (B): keep the geometry exact; reparametrize only the 1-D bridge

The curve `x(u)` is already stored as an **exact cubic Bézier** (`u ∈ [0,1]`). The only reason arc length `s` enters the pipeline is that the temporal solver expresses speed/accel limits in mm. So we **never re-describe the shape**. Per path piece:

1. Compute the **1-D arc-length inverse** `u(s)` accurately ("what curve-parameter corresponds to this many mm of travel?"). This is a single monotone, smooth, well-conditioned number line — not a 3-D shape.
2. Fit that 1-D `u(s)` to a low-degree polynomial (it is near-linear over a ~0.5 mm grid piece; subdivide if ever needed).
3. Compose with the velocity profile and the **exact** cubic geometry to produce position-over-time `x(t) = x(u(s(t)))`.
4. Hand the composed `x(t)` pieces to the existing time-domain fitter `fit_and_split` — **unchanged**.

The geometry is never approximated; only the 1-D reparametrization is, to a tolerance far tighter than the user budget.

**Plain English:** keep the road shape exact; only translate the speed schedule onto it via a single odometer↔percent number line, then read positions straight off the exact road.

### Spike validation (2026-06-14)

A throwaway in-place spike (reverted) ran all four crashing arches through the **real** `shape_batch` pipeline with B:

- All four arches × {F1500, F3000, F6000} × {shaped, unshaped} → **OK** (previously all `ERR`).
- **Start/end joints exact to machine precision** (0.000000 mm) — the rest-to-rest joints behave.
- Max interior deviation 0.009 mm (gentle) / 0.018 mm (tight_loop), **entirely from the existing 5 µm time-domain fit**; the `u(s)` bridge contributed **≤ 0.1 µm**.
- No regressions in the `trajectory` suite.

This is the evidence base; the design below productionizes it (the spike was deliberately crude on mechanics and performance).

---

## 4. Detailed design

### 4.1 Data-flow change in `compose_segment` (`trajectory/src/reparam.rs`)

Only the **non-near-zero** branch changes; the `near_zero` (constant-position dwell) branch and `build_s_of_t_pieces` are untouched.

Replace:

```
x_of_s = fit_x_to_arc_length_piece(curve, table, s_lo, s_hi, 3, 5, arc_fit_tol)   // 3-D, the bug
composed = compose_vector_piece(&x_of_s, &s_of_t)
```

with (per piece, over `[s_lo, s_hi]` → time `[t_lo, t_hi]`):

```
u_of_s   = fit_u_of_s(curve, table, s_lo, s_hi)        // 1-D, low degree, exact-inversion samples
u_of_t   = compose_vector_piece::<1>(&[&u_of_s], &s_of_t)[0]
x_of_u   = exact_cubic_about(curve, u_lo, u_hi)        // exact cubic in power basis about u_lo
composed = compose_vector_piece::<3>(&[&x_of_u…], &u_of_t)
```

where `u_lo = u_of_s.evaluate(s_lo)`, `u_hi = u_of_s.evaluate(s_hi)` (so `compose_vector_piece`'s endpoint-matching precondition is satisfied by construction).

New private helpers in `reparam.rs`:

- **`exact_cubic_about(curve, u_lo, u_hi) -> [BezierPiece;3]`** — the exact cubic geometry in the power basis about `u_lo`, `u_start=u_lo, u_end=u_hi`. Built from 4 samples of `vector_eval` in `[u_lo,u_hi]` via a 4×4 Vandermonde; exact because the geometry is degree 3.
- **`fit_u_of_s(curve, table, s_lo, s_hi) -> BezierPiece` (with subdivision)** — sample `u` at Chebyshev nodes in `s` via accurate inversion (§4.2), fit a low-degree polynomial in `(s - s_lo)`, and **subdivide on miss** (mirroring the Hermite fitter's pattern) until the position-equivalent residual `|x(u_poly(s)) − x(u_exact(s))|` is `≤ U_FIT_TOL`. Keep `u_of_s` low degree (start at 2–3) to bound the composition degree.

The composed degree is `deg(x_of_u) · deg(u_of_t) = 3 · (deg(u_of_s)·deg(s_of_t))`; with `u_of_s` degree 2–3 and `s_of_t` degree 2, that is ≤ ~12–18 per piece — well within `fit_and_split`'s adaptive subdivision.

### 4.2 Accurate, efficient `s → u` inversion

Accuracy requirement: `u` accurate to `≤ ~1e-9` so the geometry error `|x'(u)|·Δu ≲ 0.1 µm` (`|x'(u)| ≈ 75–150 mm` for these arches).

Efficiency requirement: **build the arc-length table once per segment** (it already is, in `beta.rs:494`), never per piece. The spike's per-call 16 k-entry rebuild is rejected outright.

Mechanism: **Newton refinement on the analytic curve speed.** Seed `u₀ = param_from_arc_length(table, s)` (existing linear-interp lookup), then refine

```
u ← u − (S(u) − s) / |x'(u)|
```

where `|x'(u)|` is evaluated analytically from the curve derivative, and `S(u)` (arc length to `u`) is computed accurately enough to let Newton beat the table's own accuracy (local high-order quadrature of `|x'|` between the bracketing table nodes, rather than the table's linear `arc_length_from_param`). 1–2 steps suffice. Exact constants (table tolerance/sample count, Newton step count, `U_FIT_TOL`) are calibrated in implementation against the §6 accuracy test; the linear-interpolation `param_from_arc_length` is used only as a Newton seed, never as the final answer.

### 4.3 `compose_vector_piece` endpoint robustness (`nurbs/src/algebra.rs`)

`compose_vector_piece` rejects with `SupportMismatch` when `outer.u_start/u_end` differ from `inner.evaluate(endpoints)` by more than `1e-9`. The spike hit this only when chaining *high-degree* polynomials (Horner endpoint drift). Keeping `u_of_s` low degree (§4.1) keeps the drift sub-`1e-9`. As defence-in-depth, the composition sites set the outer domain *from* `inner.evaluate(endpoints)` so the precondition holds by construction. No change to `compose_vector_piece`'s contract is required; if drift still bites, snap `inner`'s endpoint coefficient so `inner.evaluate(u_start) == outer.u_start` exactly (a localized, documented robustness fix).

### 4.4 Tolerance reconciliation

- The hard-coded `arc_fit_tolerance = 1e-4` in `beta.rs` becomes **obsolete** — there is no `x(s)` fit. Remove it.
- The new `u(s)` fit has its own accuracy target (`U_FIT_TOL`, position-equivalent, `≤ ~0.1 µm` — far below the user budget, so the geometry is effectively exact).
- The user-facing `fit_tolerance_mm` (default 5 µm) remains the **only** trajectory-accuracy knob and continues to govern the downstream time-domain fit.

### 4.5 Cusp / zero-tangent handling (preserve fail-loud)

A genuine cusp (`|x'(u)| → 0`, e.g. `P1 == P0`) makes the Newton step and `u(s)` ill-defined and is physically a mandatory full stop — legitimately unplannable as one smooth segment. Such input must **still fail loudly**. The design detects a near-zero tangent (`|x'(u)|` below a floor) during inversion and returns a typed error (e.g. `ShapeError::ZeroTangent`/cusp) with a clear message, instead of the opaque `ToleranceNotReached`. This keeps the fail-loud contract for degenerate geometry while making *smooth* curves succeed. (The existing cusp experiment cases remain rejected — by design.)

### 4.6 Dead code

`fit_x_to_arc_length_piece` (`nurbs/src/algebra.rs`) becomes unused by the live pipeline. Remove it and its tests (or, if any other caller exists, confirm and leave). Verify via `grep`/clippy that nothing else depends on it.

---

## 5. Components / files touched

| File | Change |
|---|---|
| `rust/trajectory/src/reparam.rs` | Rewrite `compose_segment` non-near-zero branch (approach B); add `exact_cubic_about`, `fit_u_of_s` (with subdivision), accurate `s→u` inversion helpers. |
| `rust/trajectory/src/beta.rs` | Remove `arc_fit_tolerance = 1e-4`; set the per-segment arc-length table tolerance/samples for the accuracy target (calibrated). |
| `rust/nurbs/src/algebra.rs` | Remove dead `fit_x_to_arc_length_piece` (+ tests) once confirmed unused; optional endpoint-snap robustness at the composition call sites. |
| `rust/trajectory/src/lib.rs` (or `ShapeError`) | Add a typed cusp/zero-tangent error variant with a clear message. |
| `rust/trajectory/tests/…` | New regression test (the four arches: assert OK + geometric accuracy + exact joints); a cusp-still-rejected test. Remove/convert the scratch `experiment_smooth_curve_fit.rs`. |

---

## 6. Testing

1. **Regression (the four arches).** `gentle_arch`, `wide_arch`, `tight_loop`, `s_curve` through `shape_batch` at live config (`fit_tolerance_mm=0.005`, Adaptive{20,200,0.5}), each at multiple feedrates, shaped and unshaped → all must return `Ok`.
2. **Geometric accuracy.** For at least two arches: sample the planned `x(t)` densely; every sample within `fit_tolerance_mm`-class distance of the exact curve; start point `== curve(0)` and end point `== curve(1)` to machine precision (joints).
3. **Cusp still rejected.** The `P1==P0` cusp and near-cusp return the typed cusp/zero-tangent error (not a silent success, not the opaque fit error).
4. **No regression.** Full `cargo nextest run` green; `./scripts/ci.sh quick` green (clippy `-D warnings`, fmt).
5. **Performance guard.** Confirm the arc-length table is built once per segment, not per piece; sanity-check planning time on a representative multi-segment curve batch (no new `replan_overrun`-class slowdown).
6. **Live bench.** Re-run the original `G5`/`G5.1` examples on the Trident; confirm motion completes with no `planner_fatal`.

---

## 7. Non-goals

- **Junction deviation / corner velocity.** Future feature; it edits geometry only at G-code *boundaries* (junctions), not within a segment. This design leaves that door open and untouched.
- **Bed mesh / surface-following transform.** Out of scope (separately gated).
- **Changing the temporal solver, shaper, or output primitive** (uniform cubic Bézier).

---

## 8. Risks & mitigations

- **Composition degree / endpoint drift.** Mitigated by keeping `u_of_s` low degree and constructing outer domains from evaluated inner endpoints (§4.3). Fallback 1: localized endpoint snap. Fallback 2 (already spike-validated end-to-end): skip the polynomial double-composition and instead sample the exact `x(u(s(t)))` at Chebyshev nodes per piece and fit a single polynomial in `t` — geometry contribution stays ≤ 0.1 µm either way, since the samples come from the exact curve at the accurately-inverted `u`. Choose the double-composition (§4.1) for cleanliness; drop to Fallback 2 if endpoint robustness proves fiddly. Both satisfy the accuracy test (§6.2), which is the acceptance gate regardless of mechanism.
- **Inversion accuracy vs cost.** Mitigated by per-segment table + analytic-derivative Newton (§4.2); calibrated against the accuracy test, with a hard performance guard (§6.5).
- **Cusp boundary.** The zero-tangent floor must reject true cusps without rejecting merely high-curvature smooth curves (the four arches include a near-closing `tight_loop`, which must pass). Calibrated and tested (§6.1, §6.3).
