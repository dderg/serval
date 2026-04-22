# Plan 5 — Adversarial Architecture Review (Round 2)

**Date:** 2026-04-22
**Reviewer scope:** architecture / integration / engineering (math scope assigned to a second reviewer)
**Target:** `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md` (post-revision, ~990 lines)
**Mode:** file:line citations against the actual tree on branch `magnum-opus`.

---

## Verdict

**Major revision needed.** The spec has made real progress — the kinematics-fanout claim is now honest, the extruder is correctly in-scope, and literature anchors no longer include fabrications. But three load-bearing architectural claims do not hold against the actual code:

1. The six-moment extension in D2a is undersized for the D7 quintic-of-quadratic composition (the actual polynomial in `t` on accel/decel phases is degree 10, not 5).
2. The D7 "outer lookahead two-pass convergence" is not implementable against the current `BlendPipelineLookAheadQueue` — that queue is a single-pass feed-forward filter chain with no back-edge.
3. The D7 struct sizing claim (24 doubles / 192 B) silently omits the `s_tab`/`t_tab` that `unified_v_of_s.md §5.3` requires (40 more doubles). Either the struct is 3× larger than the spec says, or query-time cost is not what the spec says.

Fixable, but not a ship.

---

## Files / code actually inspected

- `klippy/chelper/trapq.h:1-53` — current `struct move` layout, 88 bytes.
- `klippy/chelper/trapq.c:23-39` (move_get_coord, move_get_distance inline), `:230-256` (`trapq_extract_old` — linear-only `struct pull_move`).
- `klippy/chelper/integrate.h:1-30`, `integrate.c:18-113` — smoother struct + calc_antiderivatives (flat polynomial up to n=12).
- `klippy/chelper/kin_shaper.c:63-98` (`get_axis_position` — direct `m->axes_r.axis[..]`, direct `move_get_distance`), `:105-160` (`range_integrate`), `:184-230` (proxy-move pattern `is->m`), `:267-330` (`shaper_note_generation_time`, `input_shaper_set_smoother_params`).
- `klippy/chelper/kin_extruder.c:38-49` (`pa_move_integrate` — direct `m->axes_r.x > 0.`), `:184-226` (`extruder_calc_position` — reads `move_get_distance` + `m->start_pos` + `m->axes_r` directly).
- `klippy/chelper/kin_cartesian.c:14-33`, `kin_corexy.c:14-30`, `kin_corexz.c:14-30`, `kin_delta.c:14-35`, `kin_deltesian.c:20-29`, `kin_polar.c:14-34`, `kin_rotary_delta.c:14-60`, `kin_winch.c:14-30`, `kin_idex.c:23-45` — all use `move_get_coord` exclusively (never `m->axes_r`, never `move_get_distance`).
- `klippy/chelper/itersolve.c:20-110` — secant/bisection over `calc_position_cb`.
- `klippy/chelper/__init__.py:185-198` — CFFI defs for `input_shaper_set_smoother_params(sk, axis, n, a[], t_sm)`.
- `klippy/extras/input_shaper.py:29-70` (TypedInputShaperParams), `:285-332` (TypedInputSmootherParams, generic "Unsupported shaper type" error), `:417-431` (FFI call with flat `self.coeffs`).
- `klippy/extras/shaper_defs.py:208-221` — INPUT_SHAPERS (zv, mzv only) + INPUT_SMOOTHERS (6 smooth variants).
- `klippy/extras/shaper_calibrate.py:422-449` (`_get_smoother_sigma2`, `_get_smoother_smoothing`, `find_smoother_max_accel`) — iterates flat coeff list.
- `klippy/extras/motion_report.py:110-200` — consumes `struct pull_move` via `trapq_extract_old`, serializes `(start_v, accel, x_r, y_r, z_r)` to websocket.
- `klippy/toolhead.py:139-210` (LookAheadQueue — single backward pass), `:261-265` (BlendPipelineLookAheadQueue wrap), `:619-625` (`limit_next_junction_speed` — no back-edge), `:803-824` (`note_step_generation_scan_time` — `max()` of all registered windows).
- `klippy/blendprepass.py:146-210` — `BlendPipelineLookAheadQueue.add_move` / `flush`: strictly forward filter chain, **no back-pressure or re-plan hook.**
- `klippy/blendplanner.py:86-122` (`CornerBlender.feed` — runs before lookahead; only `max_cruise_v2` is known, not converged `cruise_v`), `:166-230` (`_emit_blend` — has `prev.axes_r[3]`/`nxt.axes_r[3]` available).
- `klippy/blendquintic.py:39-100` (quintic stored as 6 Bezier control points `Q[0..5]`, not monomial), `:569-598` (`v_cap_fn`, `polyline`).
- `klippy/blendshaper.py:28-62` (AxisShaperSnapshot — no `inverse_G`; `_SMOOTH_SPAN_FACTOR` dict — must gain `bs*` keys or the fallback path breaks).
- `klippy/blendextruder.py:91-165` (`cap_move` — per-move scalar cap, reads `move.axes_r[3]`).
- `~/Developer/klipper-sim/klipsim/sampler.py:3-130` — consumes Python Move objects (`start_v/cruise_v/end_v/axes_r`), NOT the C `struct move`. No binary trapq deserializer exists here.
- `/Users/daniladergachev/Developer/klipper-sim/` — separate local git repo with no remote; same-user co-edit is cheap.
- `docs/Resonance_Compensation.md` — exists in tree (not imagined).

---

## Architectural issues found

### Critical

**C1. The D7 composition is degree-10, not degree-5. D2a's "6 moments" is wrong.**

Spec D7 (lines 694-718) encodes the trapezoid-in-s profile as piecewise-quadratic `s(t)` on accel and decel phases. Spec D2a (lines 321-340) says `integrate_move` needs "six moments `(m0..m5)` for a quintic `x(t) = Σ c_k t^k` (k=0..5)."

But the actual commanded axis position under Pillar 2b is `x_axis(t) = poly5_in_s( s(t) )` where `s(t) = v_in·t + 0.5·a_max·t²` on the accel phase. Composing a degree-5 polynomial in `s` with a degree-2 polynomial in `t` produces a **degree-10 polynomial in `t` on each of accel / decel**. Cruise phase `s(t) = s_0 + cruise_v·t` is degree 5 in t. So `range_integrate` must compute moments up to `m10` on accel/decel sub-ranges and `m5` on cruise sub-range, AND it must sub-divide the move into three phases to honor the v(s) profile.

This is not a numerical-conditioning footnote (spec D1-5) — it is a fundamental mismatch between D2a's integration kernel and D7's velocity profile. The spec says "for linear moves, `c_3 = c_4 = c_5 = 0` and the extra coefficients are zero — linear path falls out of the same formula," but that optimism does not extend to the quintic path because the composition degree is variable across phases, and `integrate_move` currently handles exactly ONE polynomial piece per trapq move.

Fix: either (a) pre-project the 3-phase composed polynomial into `m0..m10` at emit time and do a three-sub-range integration in `range_integrate`, or (b) subdivide a single quintic-trapezoid move into 3 smaller trapq entries internally at step-gen time (but then the kernel crosses piece boundaries with different polynomial degrees, complicating `pm_diff` caching). Either way, **D2a's "extend from 3 to 6 moments" is under-specced by roughly 2×**.

**C2. BlendPipelineLookAheadQueue has no back-edge; D7's "two-pass convergence" is not implementable as written.**

D7 (lines 742-747): *"outer lookahead reads back the quintic's feasible (`v_in`, `v_out`) pair. If the blend's cap forces a stricter cruise than the neighbor moves planned, feed back and re-run the neighbor's accel-profile. Two-pass convergence: worst case one re-plan of prev/nxt."*

Actual mechanics:
- `BlendPipelineLookAheadQueue.add_move` (`klippy/blendprepass.py:168-173`) pipes each incoming move through filters in forward order: CornerBlender emits `[trunc_prev, blend_moves..., trunc_next_head]` and pushes each into the inner `LookAheadQueue`.
- Once in `LookAheadQueue`, the move's `max_cruise_v2` is fixed. `LookAheadQueue.flush` (`toolhead.py:157-210`) does ONE backward pass (reachable velocity reduction).
- There is no mechanism to retract an already-pushed move from `LookAheadQueue` and re-push it after a downstream CornerBlender discovers a stricter cap. `limit_next_junction_speed` (`toolhead.py:619-625`) only affects the NEXT upcoming junction before it is pushed.

Achieving true "two-pass convergence" requires one of:
  (a) Buffering prev in CornerBlender until after the blend's v_cap is computed, so `limit_next_junction_speed` can apply before prev reaches `LookAheadQueue`. This adds a lookahead-depth dependency (prev must not have been flushed).
  (b) A new retract-and-replan API on `LookAheadQueue` — design change.
  (c) Accepting that D7's `v_in`/`v_out` are conservative over-estimates (use the lookahead's `max_smoothed_v2` floor) and living with time-optimality loss at the boundaries.

Pick one and write it into the spec. The current text implies (b) without saying so.

**C3. The D7 `struct move_quintic_trap` size is inconsistent with the companion memo.**

Spec §D7 (lines 705-723): "Total ~24 doubles (~192 bytes) per move — fits comfortably in the `union` slot."

Companion memo `unified_v_of_s.md §5.3` (lines 336-340): *"`s → u`: precomputed `s_tab` / `t_tab` (stored C-side inside the `struct move`, `n_subintervals = 40`)."*

40 subintervals of (s, t) pairs = 80 doubles = 640 bytes, added on top of the 24 doubles in the spec's struct sketch. Either:
  (a) The spec's struct sketch is missing `s_tab[40]` and `t_tab[40]` — total is closer to ~64 doubles / 512 bytes.
  (b) The s_tab must be rebuilt on every `calc_position_cb` call — killing the "40-50 flops per query" cost claim in `unified_v_of_s.md §5`, because each query then re-runs the s→t Newton/integration over the quintic.

This is load-bearing: current `struct move` is 88 bytes; a 512-byte variant (6× size) materially changes cache footprint for `range_integrate`'s inner loop (which traverses prev/next moves). Please reconcile before implementation — and update the risk section if the true size is 512 B.

---

### Important

**I1. Tagged-union layout preserves ABI — OK, but C2b oversells the kinematics-fanout risk.**

Re-check of the 10 kinematics files: **8 of 10 access `struct move` exclusively via `move_get_coord`**. Only two files access `axes_r`/`start_v`/`half_accel` directly:
  - `kin_shaper.c:66-69` (`get_axis_position` — direct `m->axes_r.axis[..]`, direct `move_get_distance`).
  - `kin_extruder.c:44` (direct `m->axes_r.x > 0.` gate), `:200-208` (`move_get_distance` + `m->start_pos.axis[i] + m->axes_r.axis[i] * move_dist` fallback when no shaper/smoother is set on an axis).
  - Plus `integrate.c:55-62` (`integrate_move` — direct `m->axes_r`, `m->start_v`, `m->half_accel`).
  - Plus `itersolve.c` is fine (no `axes_r`, no `start_v`).

If `move_get_coord` / `move_get_distance` dispatch internally on `m->kind`, the other 8 kin_*.c files need zero changes. The spec is currently more alarmist than the code requires (paying 10 reviews / test runs when 3 are load-bearing).

BUT — the shaper proxy-move pattern (`kin_shaper.c:192-230`) is more subtle than the spec lets on:
  - `shaper_{x,y,xy}_calc_position` computes a shaped-position scalar via `smoother_calc_position` or `shaper_calc_position` (both read the original quintic move's axes directly, NOT through `move_get_coord`), stuffs it into `is->m.start_pos.{x,y}`, and delegates to `orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T)`.
  - The downstream `cart_stepper_x_calc_position` etc. then call `move_get_coord(&is->m, DUMMY_T)` on the PROXY move, which is a `MOVE_LINEAR` zero-velocity null move by design (`is->m.axes_r = 0`, `is->m.start_v = 0`).

Implication: when **any** axis is shaped (the common case), downstream kinematics never see the quintic at all. So even the 2 true direct-quintic consumers reduce to `kin_shaper.c` (must read quintic for shaping), `kin_extruder.c` (must read quintic for E and PA), and `integrate.c` (must integrate quintic polynomials). All other kinematics are reached only via the proxy move — unaffected.

This is great news for scope. Update the spec to call this out: "8 of 10 `kin_*.c` files are untouched; fanout concentrates in `integrate.c`, `kin_shaper.c`, `kin_extruder.c`, plus the `move_get_coord`/`move_get_distance` dispatch in `trapq.c`." This honest scoping also compresses the effort estimate.

**I2. The piecewise `struct smoother` redesign ripples into `_get_smoother_sigma2` which iterates a flat list.**

`klippy/extras/shaper_calibrate.py:422-441` computes `sigma^2` as an enumerate-over-flat-polynomial raw moment integral:

```python
def raw_moment(k):
    s = 0.0
    for i, c in enumerate(C):
        if (i + k) % 2 == 0:
            s += c * 2.0 * hst ** (i + k + 1) / (i + k + 1)
    return s
```

This formula is specific to a single symmetric polynomial over `[-hst, +hst]`. Piecewise form requires summing `∫_{t_j}^{t_{j+1}} t^k · P_j(t) dt` over each piece. Not hard, but the spec's D1 text *"Rest of the pipeline (`ShaperCalibrate.find_smoother_max_accel`, ...) works against the new family via the same polynomial-moment code path as before, modulo the piecewise extension below"* glosses over a real rewrite here.

Also: `_SMOOTH_SPAN_FACTOR` in `klippy/blendshaper.py:55-62` is a hard-coded dict keyed by old shaper names. The spec D1 adds `bs1..bs5` but does not mention extending this dict — `compute_shaper_bounds`'s `shaper_span` path will KeyError on the new family until it is updated.

**I3. `trapq_extract_old` + motion_report wire format is broken by quintic.**

`klippy/chelper/trapq.c:230-256` fills `struct pull_move` with `start_v`, `accel`, `x_r`, `y_r`, `z_r`, `start_x/y/z` — every field assumes a linear move. `klippy/extras/motion_report.py:110-200` forwards these over websocket to Mainsail/Fluidd.

Spec §D2c line 450-455 acknowledges this and handwaves with "version: 2 field" + "announce the change." Risk section item 2 says "UI tools degrade gracefully when they see `kind = 1` — worst case they skip quintic moves in visualization, which is a regression but not a correctness issue."

In fact, a silent "visualization gap" **IS** a correctness issue for Shake&Tune / motion_report consumers that integrate `start_v + accel * t` to reconstruct position. The version-2 schema needs to either (a) expand `struct pull_move` with 15 extra coefficient doubles for the quintic, OR (b) emit a synthetic linear approximation with the correct endpoint velocities. Either way, more work than "degrade gracefully" — spec should pick one and commit.

**I4. "klipper-sim parity in same batch" misreads what klipper-sim is.**

Spec §D2c lines 442-448: *"The batch-sim harness at `~/Developer/klipper-sim/` reads `trapq` state. D2b tagged-union change breaks its deserializer."*

Actual klipper-sim (`~/Developer/klipper-sim/klipsim/sampler.py:3-130`) consumes Python-level `Move` objects — `start_v, cruise_v, end_v, accel_t, cruise_t, decel_t, axes_r` — NOT the C `struct move`. There is no trapq binary deserializer in klipper-sim. What breaks is the PYTHON-level Move sampling path when CornerBlender emits a quintic primitive instead of polyline sub-moves — klipper-sim will see a Move-like object with no meaningful `accel_t/cruise_t/decel_t` decomposition.

This is easier than the spec implies (local same-user edit, no cross-repo coordination), BUT the scope is different: klipper-sim needs a new sampler branch for quintic-shape moves that exposes `x(t), v(t), a(t)` analytically from the quintic + v(s) profile. That's new code, not a deserializer patch.

**I5. Inlining discipline decision is deferred, but the spec's D2b has already changed the call site.**

`klippy/chelper/trapq.c:24, 31` declare `move_get_coord` and `move_get_distance` as `inline`. Every `kin_*.c` file gets them inlined at each call site. Adding a `kind` branch compiles either as:
  (a) inlined branch per call site — 8+ call sites × a 2-way branch each, probably free on modern branch predictors but bloats code.
  (b) un-inlined with a function call.

Spec says "benchmark both before implementation." That's reasonable, but it's an implementation detail that blocks the D2b patch from even compiling cleanly — you must pick one for the first commit. If (b), there are 8+ consumers to switch to the non-inline signature. If (a), the quintic path adds a 24-double struct read from the union per call, which is a real I-cache / D-cache hit inside the itersolve secant loop. Not the free-lunch the spec's tone suggests.

**I6. `v_in`/`v_out` at emit time are provisional; TOPP based on them is provisional too.**

`CornerBlender.feed` (`klippy/blendplanner.py:86-122`) runs ONE FILTER UPSTREAM of `LookAheadQueue.flush`. At the point `_emit_blend` is called, only `prev.max_cruise_v2` and `nxt.max_cruise_v2` are known — not the converged cruise velocities. If a downstream move forces `prev` to decelerate below `max_cruise_v2`, the TOPP profile computed on `v_in = sqrt(prev.max_cruise_v2)` is no longer time-optimal and may not even satisfy the accel bound (TOPP's bang-bang at the shoulders assumes `v_in` is reached).

Possible resolutions:
  - Compute TOPP with `v_in = min(prev.max_start_v2, prev.max_cruise_v2)` (worst-case high bound) — fails too-fast; accel bound violated.
  - Re-run TOPP at LookAhead-flush time after cruise velocities converge — requires restructuring `_emit_blend` to run AFTER the inner queue's backward pass.
  - Accept suboptimality: emit with `v_in = v_out = min(prev.max_cruise_v2, nxt.max_cruise_v2, v_cap(L/2))` as a safe-but-slow default. Matches current polyline behavior. Loses most of the "~20% over unsafe baseline" gain.

Spec D7 line 740-742 says *"`CornerBlender._emit_blend`: call `compute_topp_profile` and emit `MOVE_QUINTIC_TRAPEZOID_S`"* as a one-liner — hides this structural question entirely.

**I7. Fused-kernel piece count is unbounded.**

Spec §D3 line 487-490: *"`k_fused` is piecewise polynomial (convolution of two piecewise polynomials is piecewise polynomial with more pieces — for bs_m with m+1 pieces convolved with FIR inverse of similar piece count, the fused has ≤ 2(m+1) pieces)."*

Then §D1 line 294: *"Replace flat coefficient arrays with `struct smoother_piece { double coeffs[6]; double t_start, t_end; }` and an array of up to 6 pieces (bs5 has 6 pieces)."*

Contradiction: bs5 is `m=5`, so forward has `m+1 = 6` pieces. Fused has ≤ `2(m+1) = 12` pieces by D3's own bound. The "6 pieces max" limit in D1 cannot hold the fused kernel for bs5. Also, FIR inverse support `T_h = 2·T_sm` with cosine taper — the inverse's own piece count before convolution is at minimum `2(m+1) = 12` pieces (Besset-Béarée §III depends on windowing approach), so the bound is likely higher than 12.

Either size the piece array to 16-20, or keep 6 and never ship bs4/bs5 with the inverse. The D1 sizing and D3 contradiction should be made consistent before implementation.

**I8. Extruder inverse on E is not the same as "same fused kernel as XY."**

Spec §D3 line 474-479: *"one fused kernel `k_fused = h ⊛ w` is precomputed per shaper-reset and applied by every consumer of `struct smoother`."*

`extruder_calc_position` (`kin_extruder.c:184-226`) runs:
  1. Shaper convolution on XY commanded positions via `shaper_calc_position(m, 'x', ...)` or `smoother_calc_position(m, 'x', ...)`.
  2. PA velocity integral via `pa_range_integrate` (a DIFFERENT integral — velocity, not position).
  3. Sum: `position = e_pos.x + e_pos.y + e_pos.z`, then `pa_func(position, pa_velocity, ...)`.

If we apply `k_fused` (not just `w`) to the XY-axis smoother used in step 1, the E stepper's XY tracking is now feedforward-corrected. But **step 2's `pa_range_integrate` integrates `m->start_v * axis_r + ...` directly** — it reads the planned trapezoid's velocity, not a shaped velocity. So PA feedforward and position feedforward DO NOT compose the way they would in a pure XY path.

Concretely: `pa_velocity_integral` is the planned (unshaped) velocity convolved with the smoother kernel. If we swap that kernel for `k_fused = h⊛w`, the PA term becomes the planned velocity convolved with `h⊛w` — which is NOT the "inverse-corrected shaped velocity." It is the planned velocity, re-convolved with a kernel that has unit integral but different shape. PA phase alignment **does** rotate here, and the claim *"If XY gets feedforward inverse and E does not, XY traces the plan faithfully while E lags by ~T_sm/2"* is true for the XY-POSITION portion of the E command only; the PA-VELOCITY portion has a separate story that the spec never spells out.

Need a dedicated derivation of (planned × [h⊛w]) composition on the PA path. Flagging to the math reviewer, but the architectural plumbing assumption that "one kernel rules all consumers" is not obviously correct.

---

### Minor

**M1. "All variants invertible" claim depends on `T_sm > (m+1)/f_sh`.**

D1 line 195-198 requires this condition for absence of spectral zeros. Table in D1 at `f_sh = 40 Hz`, `ζ = 0.1`, 5% residual lists `T_sm` values that need verification against this threshold:
  - bs1 m=1: threshold = 2/40 = 50 ms; T_sm = 38.88 ms — **violates** threshold.
  - bs5 m=5: threshold = 6/40 = 150 ms; T_sm = 68.13 ms — **violates** threshold.

Hand-off to the math reviewer, but the table sizing vs the "no spectral zeros" sufficient condition needs checking before shipping — if the condition fails on the default sizings, the "all variants invertible" architectural claim fails.

**M2. `inverse_G` default = 1.0 at `AxisShaperSnapshot` collides with new shaper family which always has `G > 1`.**

Spec §D4 line 554: *"Default value `inverse_G = 1.0` means no inverse — the cap reduces to the existing Plan 4 form."*

But all `bs*` variants have `G = 1.92..2.84` per D1's table. A user on `bs2` with `inverse_G = 1.0` silently bypasses the saturation cap and gets ringing at corners. The default must be "read from the shaper's published G" at `_extract_shapers` time, not 1.0. Spec implies this at the end of §D4 but then contradicts with "Default value 1.0." Clarify which object owns the default.

**M3. D6 migration error path exists but needs plumbing.**

`klippy/extras/input_shaper.py:293-296` currently raises `config.error("Unsupported shaper type: %s")`. The spec's friendly error-with-hint is a ~5-line change (add a retired-name dict keyed to `bs*` names, check before the generic error). Not hard — but worth flagging that a hint-map lives somewhere and needs to be synced with `Resonance_Compensation.md`. The doc file does exist (verified).

**M4. Minor: `move_t` semantics for quintic.**

D2b reuses `move_t` as move duration for the quintic. Current callers:
  - `trapq_check_sentinels` (`trapq.c:89-92`) — computes `m->print_time + m->move_t` (fine).
  - `trapq_finalize_moves` (`trapq.c:180`) — same.
  - `trapq_extract_old` (`trapq.c:244-245`) — uses `move_t` as a range gate (fine).
  - `itersolve_gen_steps_range` (`itersolve.c:34, 37`) — `end = min(end, m->move_t)` (fine).
  - `shaper_calc_position` (`kin_shaper.c:79-82`) — walks moves via `move_t` (fine).

`move_t` stays correctly "duration of this trapq entry." No semantic breakage. Minor issue only: for a MOVE_QUINTIC_TRAPEZOID_S, `move_t = t_accel_t + t_cruise_t + t_decel_t` must be kept consistent with the profile fields. Add an invariant assert.

**M5. Trapq GC memory estimate missing.**

Spec §D5 risk 7 acknowledges stacking but doesn't quantify. Current Trident: PA contributes ~40 ms, new shaper contributes up to ~136 ms (bs5 `T_fused`), flush delay stacks to ~136 ms worst case (max, not sum). Trapq entries at printer cruise velocity (say 200 mm/s, blend every 2 mm) = 100 blends/sec × 0.136 s = ~14 extra entries buffered. At ~512 B per quintic entry (per C3), that's 7 kB extra. Fine on an SoC with 1 GB RAM; measurable on a Pi3 with 512 MB. Worth a one-line "verified OK on Pi3" test in D5 validation.

---

## Integration gaps

**G1.** BlendPipelineLookAheadQueue cannot deliver D7's two-pass convergence without a design change (see C2).

**G2.** `_SMOOTH_SPAN_FACTOR` (`klippy/blendshaper.py:55-62`) must grow `bs1..bs5` entries; spec never mentions this. Without it, `compute_shaper_bounds` KeyErrors on new shapers.

**G3.** `_get_smoother_sigma2` (`klippy/extras/shaper_calibrate.py:422-441`) assumes flat-coefficient form. D1 piecewise redesign requires rewriting this to sum piece-wise raw moments. Spec claims "same code path" — wrong.

**G4.** `struct pull_move` + `trapq_extract_old` (wire format consumed by motion_report) has no extension path for quintic. D2 spec handwaves at "version: 2 field." Concrete schema change needs speccing.

**G5.** Shaper proxy-move pattern (I1) means downstream kinematics don't see quintic when any shaping is active — this is GOOD (reduces scope) but the spec claims 10 kin files need changes. Re-scope to 3 files (`integrate.c`, `kin_shaper.c`, `kin_extruder.c`) plus the 2 `trapq.c` dispatchers.

**G6.** `CornerBlender` runs before lookahead convergence, so `v_in`/`v_out` for TOPP are provisional (see I6). Architecturally unresolved.

**G7.** PA-velocity integral path (`pa_range_integrate`) and position-shaping path use the same `struct smoother`. If `k_fused` replaces `w` in that single struct, the PA-velocity path silently gets a different kernel than intended (see I8).

**G8.** klipper-sim update is a new sampler branch, not a deserializer patch (see I4). Easier than spec implies but different work.

**G9.** D2b "tagged union" (fake-zero-cost if the union is as small as spec claims) may actually add 500+ bytes per struct move if D7's s_tab is stored in-struct (see C3). Inner-loop cache behavior for `range_integrate` needs re-evaluation.

**G10.** `Resonance_Compensation.md` exists — good. But spec D6 doesn't specify which SECTION is rewritten. The file is ~600 lines; the SIS section is buried in the middle. Worth a line-range reference in D6.

---

## Effort realism — per deliverable

Overall: **8-11 weeks** more likely than the stated 6-8 weeks.

- **D1 (B-spline family): 7-10 days** (vs spec 5-7). The piecewise redesign ripples into `_get_smoother_sigma2`, `_SMOOTH_SPAN_FACTOR`, FFI signature (breaks every external caller of `input_shaper_set_smoother_params`), and the forward-kernel closed-form implementation. Multiple consumers of flat-coeff list need piecewise versions.

- **D2 (direct-quintic step gen): 5-7 days** (vs spec 7-10). Rescoped smaller once the kinematics-fanout is honest (see I1/G5). Main work is `integrate_move` + `trapq.c` dispatchers + `kin_shaper.c`/`kin_extruder.c`. But layered onto this is the "6 moments vs 11 moments" question (C1) — if that's 11 moments, add 2 days.

- **D3 (feedforward inverse): 6-9 days** (vs spec 3-4). Under-scoped. Includes (a) closed-form `h` for each variant, (b) fused kernel convolution (piece count 10-20 per I7), (c) FFI to pass both `w` and `h`, (d) E-axis PA-velocity composition (see I8 — may require a SECOND kernel for the PA path or a non-obvious mathematical reconciliation). Math reviewer should confirm the composition story before sizing.

- **D4 (saturation cap): 1-2 days** (vs spec 0.5). The one-liner in `v_cap_fn` is trivial. The non-trivial bits: `AxisShaperSnapshot` field default policy (see M2) + `_extract_shapers` changes to read published G values per variant + test matrix across all bs1..bs5.

- **D5 (lookahead extension): 1.5 days** (vs spec 1). The `max()` reduction is clean, but SET_INPUT_SHAPER live-tuning stacks with PA flush — needs a mid-print test to verify no `send-too-old`.

- **D6 (config migration): 1 day** (vs spec 1-1.5). Friendly-error dict plus doc edits; low risk.

- **D7 (unified v(s) / Pillar 2b): 8-12 days** (vs spec 4-6). **The highest-slip item.** TOPP algorithm itself is 1-2 days. The problem is:
  - C1 (degree-10 composition): 2-3 days to get the integration kernel right.
  - C2 (outer-lookahead convergence): 3-4 days to buffer-in-CornerBlender or redesign `BlendPipelineLookAheadQueue` for retracts.
  - C3 (struct sizing): 1 day but cascades into D2 cache testing.
  - I6 (provisional v_in/v_out): 1-2 days to settle the semantics.

- **Integration + HW smoke: 7-10 days** (vs spec 5-7). The number of moving parts (new shaper family, new primitive, new inverse, new v(s) profile) means HW calibration iteration is non-trivial. Expect 2-3 rounds of "corner looks wrong → tune G or inverse_G → retest."

**Most-likely-to-slip: D7.** The spec tries to scope it into a 4-6-day single-deliverable, but it couples to D2 (polynomial degree), D3 (fused kernel), D4 (saturation), and the lookahead (convergence). It is the Plan's "load-bearing integration" deliverable and its risks are not properly surfaced in the risk section.

---

## Open questions

1. **Composition degree.** The spec's D2a claims 6 moments. My read says 11 on accel/decel phases (degree-5 poly-in-s composed with degree-2 poly-in-t = degree 10). Have not algebraically verified the composed polynomial — flagging to the math reviewer, but the CLAIMED 6 is almost certainly wrong.

2. **s_tab storage.** Does the final plan carry `s_tab[40]` / `t_tab[40]` inside `struct move`, or rebuild per query? Cannot tell from the spec; `unified_v_of_s.md` implies "stored C-side inside `struct move`" but the spec's struct sketch omits it.

3. **PA-velocity composition with k_fused.** Whether applying `k_fused` (fused forward+inverse) to the SAME `struct smoother` used by `pa_range_integrate` produces correct extruder behavior requires a proper derivation. Did not verify — handing to the math reviewer as a blocker on D3's "one kernel for all consumers."

4. **`BlendPipelineLookAheadQueue` retract.** Whether the spec intends to add a retract mechanism (redesign) or accept conservative v_in/v_out is genuinely ambiguous. Product decision.

5. **`_SMOOTH_SPAN_FACTOR` for `bs*`.** Derived from kernel construction or measured? Closed form via `T_sm` table in D1 should be easy but needs a line of text in the spec.

6. **Classic FIR shaper path untouched claim.** Spec says `zv`/`mzv` unchanged. But they share the same `input_shaper_set_smoother_params` FFI for live-tuning? Actually no — they use `input_shaper_set_shaper_params`, a separate call. Fine. But verify that smoother-is-zero sentinel path (`sm->hst = 0`) still works after piecewise redesign. D1 must include a zv/mzv regression test.

7. **klipper-sim shapes.** Once CornerBlender emits a quintic primitive, `sampler.py` needs a new branch. How will klipper-sim represent the quintic without pulling in blendquintic (klipper-sim is minimal dependencies)? Architecture unclear.

---

## Recommendation

Send back for a revision pass addressing the three Critical items, the eight Important items, and the ten integration gaps. Particular priorities:
  - Fix C1 or descope D7 to "constant-v quintic, v(s) as Plan 6."
  - Fix C2 or accept conservative v_in/v_out and document the performance gap.
  - Fix C3 or re-do the struct-size / cache footprint / effort estimate.
  - Rescope the kinematics fanout honestly (I1/G5): 3 files + dispatchers, not 10.
  - Size the fused-kernel piece array honestly (I7): ~16-20, not 6.

Once the revision lands, re-review specifically D7's TOPP + struct + composition story as an independent unit; it is the long pole and currently the most under-specified.
