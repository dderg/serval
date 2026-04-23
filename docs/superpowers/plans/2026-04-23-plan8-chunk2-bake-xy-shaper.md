# Plan 8 Chunk 2 — Bake XY shaper into planner

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Planner emits XY motion with the chosen input-shaping kernel already baked into the polynomial. Post-hoc convolution (`kin_shaper.c`) retires.

**Architecture:** Three paths based on shaper family:
- **None / empty `shaper_type`:** planner emits the degenerate quintic from Chunk 1 (unshaped) — no change.
- **bs (`bs1`..`bs5`):** planner analytically convolves each phase's degree-10 polynomial with the bs kernel per `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md`. Output: piecewise polynomial with up to 28 sub-intervals (bs5 worst case), each of degree `m+9` (bs1 = 10 → fits current; bs5 = 14 → needs coefficient expansion).
- **FIR (`zv`, `mzv`):** planner emits a piecewise polynomial with N = 2 or 3 sub-intervals per phase (per Phase 0 §6.1). Each sub-interval keeps degree 10 (no smoothing bump, impulse-train just shifts + scales).

Both shaped paths require **variable-length piece storage** on `struct move`. Chunk 1's fixed `{accel, cruise, decel}` trio is retired; moves carry a `phases[N_MAX]` array of piece polynomials.

**Spec:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §7 Chunk 2.

**Research:**
- `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md` — composer math
- `docs/superpowers/plans/plan8-research/per_axis_frequency.md` — N_MAX=32 + Pascal shift
- `docs/superpowers/plans/plan8-research/fir_piecewise_performance.md` — FIR evaluator perf
- `docs/superpowers/plans/plan8-research/shape_disabled_audit.md` — homing/probing threading

**Branch:** `magnum-opus`. Commit with actual system time. NO Co-Authored-By trailers.

---

## Task structure

Chunk 2 has four stages:

- **Stage A** (Tasks 1-3): struct upgrade — variable-length pieces + coefficient-count bump for bs5. Unshaped path still produces a valid "one-piece degree-10" polynomial. All Chunk 1 tests pass throughout.
- **Stage B** (Tasks 4-7): implement the bs polynomial composer + expose it to Python.
- **Stage C** (Tasks 8-10): FIR (zv/mzv) piecewise baking.
- **Stage D** (Tasks 11-14): shape_disabled flag, input_shaper.py rewiring, retire post-hoc kin_shaper.c + feedforward inverse + target_smoothing.
- **Stage E** (Task 15): final regression + sim corpus.

---

## Stage A — Variable-length pieces, degree-14 coefficients

### Task 1: Bump `MOVE_QUINTIC_POLY_COEFFS` to 15, add `MOVE_MAX_PIECES`

**Files:**
- `klippy/chelper/trapq.h` — change `MOVE_QUINTIC_POLY_COEFFS` 11 → 15, add `MOVE_MAX_PIECES` = 32
- `klippy/chelper/integrate.h` — SMOOTHER_NUM_MOMENTS stays 11 (smoother moments are a separate concept from polynomial coefficient count)
- Phase 0 research §6.3 said N_MAX=32 worst case. bs5 composer gives ~28. Use 32 with margin.

- [ ] **Step 1** — Audit every site that assumes `MOVE_QUINTIC_POLY_COEFFS == 11`:

```bash
grep -rn 'MOVE_QUINTIC_POLY_COEFFS\|c\[11\]\|c\[10\]\|for.*k < 11\|out_c\[SMOOTHER_NUM_MOMENTS\]' \
  klippy/chelper/ test/ klippy/ 2>/dev/null
```

Understand each site before editing.

- [ ] **Step 2** — Change `#define MOVE_QUINTIC_POLY_COEFFS 11` → `15` in `trapq.h:33`. Add `#define MOVE_MAX_PIECES 32`.

- [ ] **Step 3** — The integrator's `out_c` array in `integrate.c:move_axis_phase_polynomial` is sized to `SMOOTHER_NUM_MOMENTS` (11). It must now read up to 15 coefficients from `ph->c`. Either bump `SMOOTHER_NUM_MOMENTS` to 15 (expands binomial tables — larger), or keep it at 11 and read-and-drop coefficients beyond that (integration via 11-moment kernel is approximate for degree-14 polynomials; error bounded by kernel's degree-4 truncation term).

**Decision: keep `SMOOTHER_NUM_MOMENTS` at 11 in Chunk 2.** Reason: after Chunk 3 retires `kin_extruder.c`'s convolution, the 11-moment integrator is dead code entirely. Patching it now to degree-14 is wasted effort. Instead, in this task, document the truncation and add an assertion at composer-emit time that any degree >10 coefficient on a move destined for the post-hoc extruder path is within tolerance (1e-6 of the polynomial's range). The bs5 composer will fire a warning if asked to feed kin_extruder.c with degree>10 content; user can fall back to bs3 if needed.

- [ ] **Step 4** — Rebuild C extension, run full target suite. Expected: all Chunk 1 tests pass (the degenerate quintic from `linear_as_quintic_coeffs` populates only c[0..2], leaving c[3..14] = 0, so nothing changes observably).

```
rm -rf klippy/chelper/c_helper.so* && python3 -m pytest test/ --ignore=test/klippy -q 2>&1 | tail -5
```

- [ ] **Step 5** — Commit:

```
chunk2: bump MOVE_QUINTIC_POLY_COEFFS to 15, add MOVE_MAX_PIECES=32
```

---

### Task 2: Replace fixed `{accel, cruise, decel}` with `phases[MOVE_MAX_PIECES]`

**Files:**
- `klippy/chelper/trapq.h` — rewrite `struct move` to use `phases[]` array + `n_phases`
- `klippy/chelper/trapq.c` — rewrite `quintic_pick_phase`, `trapq_append_quintic`, `move_get_coord`, `move_is_nonnull`
- `klippy/chelper/integrate.c` — rewrite `move_axis_phase_polynomial` to iterate `phases[]`
- `klippy/chelper/kin_extruder.c` — rewrite `pa_move_integrate`
- `klippy/chelper/itersolve.c` — rewrite `check_active`
- `klippy/chelper/__init__.py` — update FFI `struct move` mirror if exposed; check pull_move fields

- [ ] **Step 1** — New struct layout:

```c
struct move {
    double print_time, move_t;
    struct coord start_pos;
    double arc_length;
    double v_cap_min;
    int n_phases;                                 // 1..MOVE_MAX_PIECES
    struct move_quintic_phase phases[MOVE_MAX_PIECES];
    struct list_node node;
};
```

- [ ] **Step 2** — `quintic_pick_phase` now does a linear scan over `phases[0..n_phases-1]`. Per Phase 0 research: linear scan is optimal up to N≤4, binary search above. Use linear scan for simplicity in Chunk 2; optimize in a follow-up task if benchmarks warrant.

- [ ] **Step 3** — `trapq_append_quintic` signature changes. New signature:

```c
void trapq_append_quintic(
    struct trapq *tq, double print_time,
    int n_phases, const double *phase_t_ends,    // length n_phases
    double move_t, double arc_length, double v_cap_min,
    double start_pos_x, double start_pos_y, double start_pos_z,
    const double *coeff_buf                      // length n_phases * MOVE_QUINTIC_POLY_COEFFS * 3
);
```

Old 3-phase callers pass `n_phases=3, phase_t_ends={t_accel_end, t_decel_start, move_t}, coeff_buf=99-doubles`.

- [ ] **Step 4** — Update `append_trapezoid_as_quintic` Python wrapper to use the new signature. `linear_as_quintic_coeffs` now pads c[3..14] with zeros (currently pads c[3..10], so extend).

- [ ] **Step 5** — Update all `m->accel`, `m->cruise`, `m->decel` references across the C codebase to `m->phases[0]`, `m->phases[1]`, `m->phases[2]` respectively (for the existing 3-phase case).

- [ ] **Step 6** — `move_get_coord` null-move fallback: update check to `if (m->n_phases == 0)` — that's the new "null move" sentinel. `move_alloc`'s memset produces `n_phases = 0`.

- [ ] **Step 7** — Rebuild + full regression. All Chunk 1 tests must still pass. The struct change is invisible at the API level; internal layout differs.

- [ ] **Step 8** — Commit:

```
chunk2: struct move uses variable-length phases[MOVE_MAX_PIECES]
```

---

### Task 3: Update Python CornerBlender emit to new signature

**Files:**
- `klippy/blendquintic.py` — compose_phase_polynomials packs 99-double buffer; extend to include degree-14 zeros for unshaped output (no bs baking yet)
- `klippy/blendplanner.py` — CornerBlender._emit_blend call site

- [ ] **Step 1** — Update `compose_phase_polynomials` to output `n_phases * 15 * 3 = n_phases * 45` doubles instead of `n_phases * 33`. For unshaped (Chunk 1 output), c[11..14] = 0 on every phase.

- [ ] **Step 2** — Update the FFI call in `_emit_blend` to pass `n_phases` and `phase_t_ends` array.

- [ ] **Step 3** — Run full test suite. All Chunk 1 + Plan 5 tests pass.

- [ ] **Step 4** — Commit:

```
chunk2: CornerBlender emit uses variable-length phases
```

---

## Stage B — bs polynomial composer

### Task 4: Research-verify bs composer formulas (subagent math-review)

**Dispatch opus subagent** to re-derive the key formulas from `bs_polynomial_composer.md` and verify they match `shaper_defs.py:_cardinal_bspline_pieces` numerically on a single-phase test case (e.g. cruise-only motion at v=50 mm/s, bs3 at 60 Hz). Output: verification memo at `docs/superpowers/plans/plan8-research/bs_composer_verification.md`. Required prior to Task 5 implementation.

- [ ] **Step 1** — Dispatch.
- [ ] **Step 2** — Review the verification result; if discrepancy surfaces, escalate (revise composer before coding).
- [ ] **Step 3** — Commit verification doc.

---

### Task 5: Implement `bs_compose` C function

**Files:**
- Create: `klippy/chelper/bs_compose.h`, `.c`
- Register in `__init__.py` sources

Signature:

```c
// Given input phase polynomials and a bs kernel, fill out_coeff_buf with
// the composed piecewise polynomial. Returns number of output sub-intervals.
int bs_compose(
    int n_input_phases, const double *input_phase_t_ends,  // length n_input_phases
    const double *input_coeffs,  // n_input_phases * 15 * 3 doubles, per-phase quintic
    int bs_order,  // m ∈ {1..5}
    double shaper_freq, double damping_ratio,
    int n_max_output,
    double *out_phase_t_ends,  // [n_max_output]
    double *out_coeff_buf,     // [n_max_output * 15 * 3]
    double *out_support_left,  // kernel support extends by T_sm/2 left of move
    double *out_support_right  // ... and T_sm/2 right
);
```

- [ ] **Step 1** — Implement per `bs_polynomial_composer.md`:
  - Enumerate breakpoint set (Minkowski-sum of phase boundaries × kernel piece edges)
  - For each sub-interval, integrate the convolution analytically
  - Degree bound: `min(input_degree + kernel_degree, 14)` = degree m+9
  - Cost estimate: ~35k flops per move worst case — trivial

- [ ] **Step 2** — Unit tests in `test/test_bs_compose.py`:
  - Single-phase cruise at constant v: output is an m-degree polynomial (smoothed step response)
  - Zero-accel, zero-v input: output is zero
  - bs1 composer with input degree-10 polynomial matches a hand-computed reference

- [ ] **Step 3** — Expose to Python via FFI + wrapper `bs_compose.py`.

- [ ] **Step 4** — Commit:

```
chunk2: bs_compose — bs kernel ⊛ quintic phase polynomial composer
```

---

### Task 6: Handle kernel support extending past move boundaries

**Problem:** bs kernel convolution "smears" motion across move boundaries. If move N ends at t=T and kernel support is [-T_sm/2, T_sm/2], then move N's shaped output at time T extends to t = T + T_sm/2 — into move N+1's time.

**Two viable approaches:**

(A) **Boundary padding.** Each move's composer takes neighboring moves' polynomials and integrates across the boundary. Planner must commit neighboring moves before the current one can be emitted. Per Phase 0 §6.4, `LOOKAHEAD_FLUSH_TIME = 250 ms` is adequate — kernel support max 109 ms.

(B) **Continuation pieces.** Each move's composer outputs an "extra" set of sub-intervals covering the kernel-support-out region (t ∈ [T, T + T_sm/2]). These pieces attach to move N's struct but are evaluated at move N+1's times by step-gen.

**Decision: (A).** Simpler; Phase 0 research already verified flush window is adequate. Adds some plumbing in the CornerBlender emit path — the composer needs to receive 1-2 neighbor moves' polynomials.

- [ ] **Step 1** — Extend `bs_compose` signature to take previous/next phase polynomials as inputs. Composer integrates across boundaries.

- [ ] **Step 2** — CornerBlender keeps a small ring buffer of previous/next move polynomials; emits to trapq only after ring fills.

- [ ] **Step 3** — Unit test: two adjacent moves (accel → decel), verify continuity of shaped output across the boundary.

- [ ] **Step 4** — Commit:

```
chunk2: bs_compose handles neighbor-move boundary continuity
```

---

### Task 7: Wire bs_compose into the planner emit path

**Files:**
- `klippy/blendquintic.py` / `klippy/blendplanner.py` — when shaper config specifies `bs1`..`bs5`, call `bs_compose` before `trapq_append_quintic`
- `klippy/extras/input_shaper.py` — no longer dispatches to kin_shaper; instead hands the kernel parameters to the planner

- [ ] **Step 1** — Add `shaper_kernel` parameter threading: config → toolhead → CornerBlender → compose call.

- [ ] **Step 2** — For bs kernels, call `bs_compose` and pass the result to `trapq_append_quintic`. For empty shaper, skip (pass through unshaped).

- [ ] **Step 3** — Integration test: a single accel move with bs3 at 60 Hz; compare sim output to the post-hoc shaper output. Target: <1 µm difference.

- [ ] **Step 4** — Commit:

```
chunk2: planner emits bs-shaped motion via bs_compose
```

---

## Stage C — FIR (zv, mzv) piecewise baking

### Task 8: Implement `fir_compose` C function

**Files:**
- Create: `klippy/chelper/fir_compose.h`, `.c`

FIR baking is simpler than bs: given N impulses `(a_i, tau_i)`, the output is `sum_i a_i * x(t - tau_i)`. Each input phase polynomial becomes N polynomials (one per impulse) shifted in time by tau_i. The output piecewise polynomial has boundaries at `{phase_boundary + tau_i}` for all phases and all impulses.

For mzv (3 impulses): 3 input phases × 3 impulses = 9 breakpoints per move worst case, each sub-interval polynomial is a sum of up to 3 shifted phase polynomials.

- [ ] **Step 1** — Implement. Degree stays at 10 (no smoothing bump). Output pieces: up to 9 for mzv, 6 for zv.

- [ ] **Step 2** — Unit tests: single-phase pure-cruise input, mzv shaping, verify output amplitudes sum to 1.

- [ ] **Step 3** — Commit:

```
chunk2: fir_compose — FIR impulse-train bakery for zv / mzv
```

---

### Task 9: Wire fir_compose into planner emit

**Files:**
- `klippy/blendplanner.py` — dispatch to `fir_compose` for FIR shapers

- [ ] **Step 1** — Add dispatch: `shaper_type in ('zv', 'mzv')` → `fir_compose`; `shaper_type in ('bs1'..'bs5')` → `bs_compose`; else pass-through.

- [ ] **Step 2** — Integration test: mzv-shaped accel, compare to post-hoc mzv. <1 µm tolerance.

- [ ] **Step 3** — Commit:

```
chunk2: planner emits FIR-shaped motion via fir_compose
```

---

### Task 10: FIR step-gen secant-solver performance verification

**Rationale:** Phase 0 §6.1 predicted ~1% of steps on corner-heavy prints hit the bisection branch. Verify on sim corpus.

- [ ] **Step 1** — Run klipper-sim Cowling/speedbench gcode with mzv baked in; count check_oscillate firings vs today's post-hoc mzv (run both for comparison).

- [ ] **Step 2** — If >5% (5× the prediction), investigate. If <2%, document and move on.

- [ ] **Step 3** — Commit the performance report to `plan8-research/`.

---

## Stage D — shape_disabled flag + input_shaper retirement

### Task 11: Add `shape_disabled` flag to struct move + FFI + composer dispatch

**Files:**
- `klippy/chelper/trapq.h` — add `int shape_disabled;` field to struct move
- `klippy/chelper/__init__.py` — expose via FFI if needed
- Planner composer dispatch — skip `bs_compose` / `fir_compose` when flag set

- [ ] **Step 1** — Add flag field. Default false.

- [ ] **Step 2** — Composer dispatch: check flag; if true, emit raw degenerate quintic (Chunk 1 behavior) regardless of `shaper_type`.

- [ ] **Step 3** — Thread flag through emit sites per `shape_disabled_audit.md`:
  - `force_move.py`, `manual_stepper.py`: pass `shape_disabled=True`
  - `extruder.py:772` pure-E moves: `shape_disabled=True`
  - Drip-mode (homing): covered implicitly because drip_move routes through lookahead.flush immediately — verify in test

- [ ] **Step 4** — Unit test: exercise each bypass path; verify emitted polynomial is the raw degenerate quintic.

- [ ] **Step 5** — Commit:

```
chunk2: shape_disabled flag bypasses shaper baking for homing / force / manual
```

---

### Task 12: Rewire input_shaper.py to hand kernel to planner

**Files:**
- `klippy/extras/input_shaper.py`

- [ ] **Step 1** — Read current state of input_shaper.py around `:463-607` and `:857`. Identify what sends kernel config to `kin_shaper.c` via FFI today.

- [ ] **Step 2** — Replace FFI calls to `input_shaper_set_smoother_params` / `input_shaper_set_shaper_params` with calls that hand kernel config to the planner's CornerBlender directly. The post-hoc path no longer runs.

- [ ] **Step 3** — `SET_INPUT_SHAPER` gcode command triggers flush + reconfig. Verify via integration test.

- [ ] **Step 4** — Commit:

```
chunk2: input_shaper.py hands kernel config to planner directly
```

---

### Task 13: Retire kin_shaper.c + feedforward inverse + target_smoothing

**Files:**
- Delete: `klippy/chelper/kin_shaper.c`, `kin_shaper.h`
- Delete: `klippy/chelper/bspline_inverse.py` + `bspline_inverse.c` (if exists)
- Delete: `klippy/extras/extruder_smoother.py` (Pillar 1 feedforward inverse for extruder, retires)
- `klippy/chelper/__init__.py` — drop from source list, drop FFI
- `klippy/extras/shaper_calibrate.py` — delete target_smoothing machinery
- `klippy/extras/input_shaper.py:463-607` — delete fused-kernel wire-up

- [ ] **Step 1** — Audit every call site of `kin_shaper.c` public functions (init_shaper, init_smoother, etc). All must be gone.

- [ ] **Step 2** — Delete the files. Remove from build.

- [ ] **Step 3** — Delete target_smoothing config parsing + runtime cap.

- [ ] **Step 4** — Rebuild; run full test suite. Tests referencing retired infrastructure need rewriting.

- [ ] **Step 5** — Commit:

```
chunk2: retire kin_shaper.c, feedforward inverse, target_smoothing
```

---

### Task 14: Test suite cleanup — rewrite shaper tests against baked path

**Files:**
- `test/test_shaper_*.py`, `test/test_input_shaper.py`, `test/test_plan5_integration.py` (Pillar 1 tests), `test/test_blendextruder.py`

- [ ] **Step 1** — Enumerate tests that exercise post-hoc shaper or feedforward inverse.

- [ ] **Step 2** — Retire tests that no longer apply (feedforward inverse, fused kernel).

- [ ] **Step 3** — Replace "post-hoc shaper + commanded motion → shaped position" tests with "planner-emit with shaper config → shaped position".

- [ ] **Step 4** — Commit:

```
chunk2: rewrite shaper tests against baked-in planner
```

---

## Stage E — Final regression

### Task 15: Sim corpus regression + close Chunk 2

- [ ] **Step 1** — Run klipper-sim test suite.

- [ ] **Step 2** — Run full kalico pytest (excluding klippy-cfg).

- [ ] **Step 3** — Record any new regressions in a summary note.

- [ ] **Step 4** — (user-driven, not gated) HW test on Trident.

---

## Exit criteria

- `kin_shaper.c` deleted. `bspline_inverse.py` / `.c` deleted. `extruder_smoother.py` deleted.
- Every XY move emits via `trapq_append_quintic` with either:
  - Unshaped degenerate quintic (no shaper configured), or
  - bs-composed piecewise polynomial (bs1..bs5), or
  - FIR-composed piecewise polynomial (zv, mzv)
- `shape_disabled` flag honored by composer; homing / force / manual bypass cleanly
- Step-gen has no shaper dispatch
- Input shaper config (`shaper_type`, `shaper_freq_*`) still works from user perspective
- `target_smoothing` config key produces error (retired)
- All target tests + sim corpus pass

## Not in Chunk 2

- Extruder-side shape/PA baking (Chunk 3)
- New kernel families
- Phase stepping
- `move_get_distance` arc-length correction (Chunk 3 or later)
