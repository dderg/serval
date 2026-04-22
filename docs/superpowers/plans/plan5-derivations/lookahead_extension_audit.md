# Plan 5 — Step-gen lookahead extension audit

**Context.** Pillar 1 adds a feedforward inverse kernel `h⁻¹(t)` in front of
the shaper `w(t)`. The composed operator `h⁻¹ ⊛ w` has support roughly
`T_sm + T_h`. Step-gen must see moves this far into the future of the
currently-emitted sample. This doc audits every plumbing point that touches
the scan window.

Notation: `T_sm` = shaper smooth_time (FIR pulse span or polynomial window).
`T_h` = inverse-companion kernel support (≈ `T_sm` for stable inversions).
Default `T_sm` on Trident today ≈ 0.040 s (MZV @ 50 Hz); bounded by shaper
choice, typically in `[0.020, 0.080] s`.

---

## 1. Current lookahead architecture

```
gcode.move() ──► toolhead.move ──► LookAheadQueue ──► blendplanner
                                                         │
                                                         ▼
                                                   trapq_append
                                                         │
                          (moves live on trapq for > kin_flush_delay)
                                                         │
                                                         ▼
   ┌────────── step_generators[*] = itersolve_generate_steps ───────┐
   │  per stepper_kinematics:                                       │
   │    calc_position_cb (convolution with shaper impulses/smoother)│
   │    gen_steps_pre_active  ← T_sm/2 (+ t_offs) for smooth-IS     │
   │    gen_steps_post_active ← T_sm/2 (− t_offs)                   │
   └────────────────────────────────────────────────────────────────┘
                                                         │
                                                         ▼
                                            stepcompress / MCU FIFO
```

Flush tick math (`klippy/toolhead.py:413-434`):
```
sg_flush_want = min(flush_time + STEPCOMPRESS_FLUSH_TIME,
                    print_time − kin_flush_delay)
sg_flush_time = max(sg_flush_want, flush_time)
for sg in step_generators: sg(sg_flush_time)
free_time     = sg_flush_time − kin_flush_delay   # ≥ trapq GC cutoff
```

So `kin_flush_delay` is the single number that (a) keeps the next-to-emit
sample at least that far behind `print_time`, and (b) keeps trapq moves
pinned that far past `sg_flush_time`.

`kin_flush_delay` is recomputed as `max(kin_flush_times ∪ {SDS_CHECK_TIME})`
where `SDS_CHECK_TIME = 1 ms` (`klippy/toolhead.py:240, 815`). Every
kinematics/shaper/extruder contributor registers its own required window via
`note_step_generation_scan_time(delay, old_delay)`.

Per-stepper scan window (`klippy/chelper/itersolve.c:157, 162-164, 191, 207`):
`gen_steps_pre_active` / `gen_steps_post_active` gate step generation on a
per-stepper basis so an idle stepper's queue doesn't bloat. These are set by
`shaper_note_generation_time` (`klippy/chelper/kin_shaper.c:267-293`) and
`extruder_note_generation_time` (`klippy/chelper/kin_extruder.c:229-247`).

---

## 2. Callers of `note_step_generation_scan_time`

Defined `klippy/toolhead.py:809-816`. Three production sites today.

1. **Input shaper live-reconfigure** — `klippy/extras/input_shaper.py:635`.
   - Registers `new_delay = input_shaper_get_step_gen_window(is_sk)`
     (`klippy/chelper/kin_shaper.c:333-338`: `max(pre_active, post_active)`).
   - For smooth-IS: `hst = T_sm/2`, `pre_active = hst + |t_offs|`,
     `post_active = hst − t_offs`; so delay ≈ `T_sm/2 + |t_offs|`.
     On Trident with MZV-equivalent smooth-IS at `T_sm ≈ 0.040 s`,
     `t_offs ≈ 0`, delay ≈ **0.020 s**.
   - For FIR: delay = `max(last_pulse.t, −first_pulse.t)` ≈ `T_sm/2` as well.

2. **Extruder PA reconfigure** — `klippy/kinematics/extruder.py:420`.
   - Registers `extruder_get_step_gen_window(sk_extruder)`
     (`klippy/chelper/kin_extruder.c:299-305`).
   - Includes PA `time_offset` + smoother `hst` + `t_offs` per axis
     (see `extruder_note_generation_time`).
   - Typical `time_offset ≤ 0.040 s`, `hst ≈ 0.020 s` → delay ≈ `0.040 s`
     from the extruder side, often the *dominant* contributor.

3. **Extruder shaper-update path** — `klippy/kinematics/extruder.py:443`.
   - Same window as above but on the "shaper changed" codepath (called via
     `input_shaper.py:641`, the fan-out through `update_input_shaping`).

No kinematics (CoreXY, Delta, etc.) register a window today — they rely on
`SDS_CHECK_TIME` = 1 ms as the floor.

On a typical Trident config with both active, `kin_flush_times` holds
one or two entries; the realized `kin_flush_delay` is dominated by the
extruder value (`~0.040 s`) rather than the pure XY shaper value
(`~0.020 s`). This matters for Plan 5's delta — see §3.

---

## 3. Plan 5 delta

Pillar 1 adds `h⁻¹(t)` inside the calc_position path (per Magnum Opus design
doc `docs/Magnum_Opus_Design.md:77-102, 276-282`: new
`blendshaper_inverse.py` module and C counterpart). The composite
`h⁻¹ ⊛ w` has support ≈ `T_sm + T_h`.

- **FIR case.** Cho/Sencer finite-window deconvolution truncates `h⁻¹` to
  a kernel of support ≈ `T_sm` to `2·T_sm` depending on zero locations.
  Assume worst-case `T_h = T_sm`. Composite support ≈ `2·T_sm` →
  pre/post each ≈ `T_sm`.
- **Smooth-IS case.** Polynomial inverse is local; support equals the
  shaper's (`T_h = T_sm`), same budget.

**New per-stepper window:**
```
gen_steps_pre_active  = hst + |t_offs| + T_h   = T_sm/2 + T_h   ≈ T_sm
gen_steps_post_active = hst − t_offs  + T_h   = T_sm/2 + T_h    ≈ T_sm
```

**New `kin_flush_delay`** on Trident (`T_sm = 0.040 s`):
- Pure XY: **0.040 s** (was 0.020 s) → 2× XY contribution.
- With PA already at 0.040 s, the effective `max()`-reduced delay moves
  from 0.040 s (PA-dominated) to **~0.080 s** (PA + shaper composite
  independently register, both ~0.040 s … wait: `note_step_generation_scan_time`
  takes `max`, not sum; see below).

**Important subtlety:** `kin_flush_delay = max(...)`. Each caller registers
*its own* required delay. PA's window (0.040 s) and shaper's window (0.040 s
post-Plan-5) do not stack at the toolhead level — the toolhead flushes to the
larger of the two. The per-stepper `gen_steps_pre/post_active` is what pins
each stepper's queue; the toolhead delay is the envelope.

**So post-Plan-5:**
- Shaper caller registers **~0.040 s** (doubled).
- Extruder caller: unchanged unless Plan 5 adds an inverse in the extruder
  path too — which it should, for pillar 1 to be lossless for extruder sync.
  If yes, extruder registers **~0.080 s** (PA 0.040 + inverse companion
  0.040).
- Realized `kin_flush_delay` ≈ **0.080 s** (extruder-dominated if extruder
  inverse is added; shaper-dominated at 0.040 s if XY-only).

Which caller registers it: **input_shaper.py:635** (XY) and, conditionally,
**extruder.py:420, 443** (E-axis). Both already exist — the *values they
read back* from the C layer change, not the plumbing.

---

## 4. Touchpoints requiring code changes

Estimates assume the existing `sk->gen_steps_pre_active/post_active` model
is kept (just widened), not a rearchitecture.

| # | File:line | Change | LOC |
|---|---|---|---|
| T1 | `klippy/chelper/kin_shaper.c:267-293` (`shaper_note_generation_time`) | Add inverse-kernel contribution to `pre_active`/`post_active`; likely via a new `struct inverse_kernel` pointer on `input_shaper` populated by a new `input_shaper_set_inverse_params`. | ~30 |
| T2 | `klippy/chelper/kin_shaper.c` (new) | `input_shaper_set_inverse_params(sk, axis, n, a[], t_h)` and (probably) inverse analog of `init_smoother`. | ~80 |
| T3 | `klippy/chelper/kin_extruder.c:229-247` (`extruder_note_generation_time`) | Same widening if extruder path gets an inverse. | ~20 |
| T4 | `klippy/chelper/__init__.py:186-196` | Add FFI declaration for new `input_shaper_set_inverse_params` (mirroring `:193` `input_shaper_set_smoother_params`). | ~5 |
| T5 | `klippy/extras/input_shaper.py` (new `InverseShaper` wrapper, ~`SmoothInputShaper` pattern at line 389) | Call `set_inverse_params` after `set_smoother_params`; `update_stepper_kinematics` existing-pattern at 417-431 extended. | ~40 |
| T6 | `klippy/extras/input_shaper.py:616-648` (`_update_input_shaping`) | No structural change — `input_shaper_get_step_gen_window` already returns the new widened value once C-side picks up inverse. Zero code change here, zero additional calls to `note_step_generation_scan_time`. | 0 |
| T7 | `klippy/chelper/itersolve.c:157, 162-207` | **No change required**: already reads `gen_steps_pre/post_active` per-sk. Doubling them is a configuration-level fix. | 0 |
| T8 | `klippy/toolhead.py:809-816` | **No change required**: `max()` handles the widened values transparently. | 0 |
| T9 | `klippy/toolhead.py:704, 722` (drip mode) | Verify `flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + kin_flush_delay` still works with larger delay. Currently 0.100 + 0.050 + 0.040 = 0.190 s; post-Plan-5 0.100 + 0.050 + 0.080 = **0.230 s**. The drip pre-fill `dwell(self.kin_flush_delay)` at `:722` silently grows from 40 ms to 80 ms (acceptable — pre-homing pause). | 0 |
| T10 | `klippy/blendshaper_inverse.py` (new, per design doc) | Plan 5 Pillar 1 deliverable; Python side that computes inverse kernel coefficients and pushes them via T2's FFI. | ~200 (outside this audit's scope) |

Plumbing-only delta: **~175 LOC** (T1–T5). T10 is the actual Pillar 1 work.

---

## 5. User-visible impact

- **M400 latency** (`klippy/toolhead.py:847-849` → `wait_moves` at `:665-674`).
  `wait_moves` polls `print_time vs estimated_print_time`. The
  `print_time - kin_flush_delay` boundary is already baked into planning;
  `print_time` itself isn't pushed forward by widening `kin_flush_delay`.
  **Net: M400 does not wait measurably longer.** A single extra `T_sm/2`
  of settling shows up once as the last-move tail but the planner already
  accounts for that.

- **`SET_INPUT_SHAPER` pause** (`klippy/extras/input_shaper.py:616-648`).
  Calls `flush_step_generation()` at `:617`. Current cost: drain lookahead +
  advance flush to `step_gen_time` — on the order of the current print
  buffer depth `BUFFER_TIME_HIGH = 2.0 s` worst case
  (`klippy/toolhead.py:233, 510`). **Plan 5 does not change this** — the
  flush drains to `step_gen_time`, independent of `kin_flush_delay`. The
  user sees the same 0.5-2 s hiccup they see today.

- **`SET_PRESSURE_ADVANCE` pause** (`klippy/kinematics/extruder.py:404`).
  Same story — flush is pre-existing. Unchanged by Plan 5.

- **Homing drip-fill dwell** (`klippy/toolhead.py:722`).
  `dwell(kin_flush_delay)` grows from 40 ms → 80 ms. **User-visible: no**
  — homing already has 100s of ms of built-in dwells (`HOMING_START_DELAY`
  in `homing.py`). An extra 40 ms is invisible.

- **Print-start buffer priming.** `_calc_print_time` (`klippy/toolhead.py:447-460`)
  bumps `kin_time += kin_flush_delay` before setting `min_print_time`.
  Doubling the delay means the planner stays 40 ms further ahead of MCU
  clock on first move — one-time, sub-perceptual.

- **trapq memory** (see §6, Risks). Negligible.

---

## 6. Risks

1. **Shaper-off or zero-kernel edge case.** With `shaper = none`, smooth_time = 0.
   `hst = 0`, `t_offs = 0` → `gen_steps_pre/post_active = 0`. Plan 5 must
   not unconditionally add `T_h` when the inverse is disabled. Need a
   `t_h == 0` guard in `shaper_note_generation_time`. **Mitigation:**
   treat `t_h == 0` as "no inverse" and skip the addition. Required check
   in T1.

2. **Shaper-change mid-print** (`klippy/extras/input_shaper.py:616`).
   Sequence: `flush_step_generation()` → reconfigure → new delay registered
   → `flush_step_generation()` again inside `note_step_generation_scan_time`
   (`toolhead.py:810`). The double-flush already exists; widening the delay
   only extends the *second* flush's target by `T_h`. No deadlock, just
   +`T_sm/2` ms on the hiccup.

3. **Emergency stop / shutdown during extended window**
   (`klippy/toolhead.py:796-798` `_handle_shutdown`). Just calls
   `lookahead.reset()`. Not affected by `kin_flush_delay`.

4. **Homing transition moment.** Homing calls `flush_step_generation` at
   `homing.py:111, 158, 395`. `drip_move` pre-dwells by `kin_flush_delay`
   (`toolhead.py:722`). If a user disables the shaper for homing and
   re-enables after, two `flush_step_generation` cycles fire — total added
   latency from Plan 5 = 2 × `T_sm/2` = `T_sm` ≈ 40 ms. Imperceptible.

5. **trapq lifetime.** `free_time = sg_flush_time − kin_flush_delay`
   (`toolhead.py:428`). Moves older than `free_time` are moved to history
   (`trapq.c:168-199`). Widening `kin_flush_delay` pins each move for an
   additional `T_sm/2` seconds. At 50 mm/s with 1 mm moves = 20 Hz move
   rate, that's +1 move pinned ≈ ~250 bytes. **Negligible.**

6. **Pillar 2 (quintic/smooth-accel) interaction.** Pillar 2 is upstream of
   trapq; its output feeds the same trapq→itersolve pipeline. No change to
   lookahead accounting from Pillar 2.

7. **Plan 3 extruder cap** (`klippy/toolhead.py:631-644`). Runs in
   `toolhead.move` *before* trapq. Independent of step-gen window. Confirmed
   unaffected.

8. **PA time_offset already consumed** (`klippy/chelper/kin_extruder.c:236-239`).
   `pre_active_axis = sm->hst + sm->t_offs + es->time_offset`. If Plan 5
   also adds an extruder inverse, that inverse's `T_h` adds on top. Double-
   check Pillar 1 design: does the extruder need its own inverse, or does
   it sync to the pre-distorted XY command directly? Per Magnum Opus doc
   `:71-73`: *"The planned path is the physical motion"* — extruder follows
   physical, not commanded. So **extruder does NOT need its own inverse**;
   extruder scan window stays at today's value. This halves T3's scope to
   "verify no change needed."

---

## 7. Engineering estimate

These are time-to-working-code estimates assuming the Pillar 1 research
(kernel derivation, stability analysis) is already done and the Python
inverse-computation module is separate scope.

| Task | Estimate |
|---|---|
| T1 (C-side `shaper_note_generation_time` widening + guard) | 1-2 h |
| T2 (new C FFI `input_shaper_set_inverse_params` + `init_inverse`) | 3-5 h (mostly data-structure pattern-match on `init_smoother`; no math) |
| T3 (verify extruder path unchanged; no code) | 30 min |
| T4 (FFI declarations) | 10 min |
| T5 (Python wrapper hook-up) | 1-2 h |
| **Plumbing total** | **6-10 h of focused work** |
| Integration test (printer config with Pillar 1 active, M400/SET_INPUT_SHAPER round-trip, drip-homing smoke) | 2-4 h |
| HW validation dwell | 1 session |

**I can't estimate T2 precisely without implementing it** — `init_smoother`
uses polynomial antiderivatives (`klippy/chelper/integrate.c:81`). If the
inverse needs the same precomputation structure, T2 may balloon to 1-2 days
*at the math boundary*, but the plumbing audit stops there; that belongs to
Pillar 1 proper.

Plumbing-only confidence: **~1 working day, plus testing.**

---

## Summary for cross-reference

- **Today:** `kin_flush_delay ≈ 0.040 s` (extruder-dominated; Trident/MZV-eq).
- **Post-Plan-5:** `kin_flush_delay ≈ 0.080 s` if extruder inverse is added,
  else `~0.040 s` (unchanged — shaper contribution doubles 0.020 → 0.040
  but stays below the extruder PA contribution).
- **Plumbing is additive, not invasive.** The scan-window accounting is
  already per-stepper (`gen_steps_pre/post_active`) and the toolhead just
  takes `max`. Plan 5 adjusts *values*, not wiring.
- **Highest-risk code path:** `shaper_note_generation_time`
  (`klippy/chelper/kin_shaper.c:267-293`) — needs a clean `t_h == 0` guard
  so shaper-disabled configs don't inherit an unexpected window.
