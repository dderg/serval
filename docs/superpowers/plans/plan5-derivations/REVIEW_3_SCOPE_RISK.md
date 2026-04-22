# REVIEW 3 — Scope / risk / implementation-time surprises

**Reviewer angle.** Adversarial scope review. Prior rounds covered math
(`REVIEW_2_MATH.md`) and architecture / C-code
(`REVIEW_2_ARCH.md`). This round asks: *what will bite an implementer
who sits down with this spec on day 1?* Target is gaps, under-specified
decisions, and unknown-unknowns — not more math.

## Verdict

**ship with fixes.** The core architectural choices (Option A tagged
union, compose-at-emit-time, fused kernel, Option Z upstream
junction-cap) are solid. The spec has absorbed two review rounds
well — the math is defensible, the C-side fanout is now bounded.

But the spec is still written as an architecture doc, not an
implementation doc. There are a half-dozen surface-area gaps (listed
below) that will either (a) silently break users, (b) block the
regression-diagnosis workflow, or (c) cost an extra week of rework
discovered mid-implementation. None are architecture-invalidating;
they are scope additions that belong in the plan before a PR opens.

Recommendation: address the 4 **Critical** items as spec addenda
before implementation starts; the 3 **Important** items can be logged
as implementation-task TODOs and handled en route; the **Minor**
items are cleanups.

## Gaps found

### Critical

**C1. `trapq_append` API fanout (manual_stepper, force_move,
trad_rack).** The spec's D2b lists "3 C files" that need changes
but silently excludes the **Python callers of `trapq_append`**.
Five files call it directly:
`klippy/extras/force_move.py:103`,
`klippy/extras/manual_stepper.py:78`,
`klippy/extras/trad_rack.py:2391-2437`,
`klippy/kinematics/extruder.py`,
`klippy/toolhead.py`.

These are all *linear-move* callers and the spec's tagged-union
design does preserve the linear path, but:

- Does `trapq_append` keep its current signature (appends a
  `MOVE_LINEAR` kind) while a new `trapq_append_quintic` is added?
  Or does the existing function gain a `kind` parameter?
- `trad_rack` maintains its own `trapq` and may not need quintic
  support at all — but will it still work if the C-side
  `struct move` layout changes underneath it?
- `force_move` is used for safe-Z-home and stepper_buzz — any
  ABI break here silently breaks homing on every printer.

**Fix.** Add an "FFI surface" subsection to D2b listing every
Python caller of `trapq_*` and `move_*`. Explicit statement:
"linear callers use the existing `trapq_append` unchanged;
`trapq_append_quintic` is additive." Then verify: no direct field
access through `pull_move` breaks because `pull_move` layout
stays the same (it already doesn't carry quintic info).

**C2. `motion_report` cannot represent quintic moves.**
`klippy/extras/motion_report.py:109-117` defines an API header:

```python
"header": ("time", "duration", "start_velocity", "acceleration",
           "start_position", "direction",)
```

This is the schema Mainsail / Fluidd / Moonraker parse. It has
no room for per-phase polynomial coefficients. The spec says
"Emit a `version: 2` field" — but the concrete schema is not
specified. Either:

- Add a `"polys": [[c0,...c10], ...]` per-axis per-phase —
  dramatic payload-size increase (840 B/move vs ~80 B today).
- Or serialize a sampled trapezoid approximation — lossy but
  small.

**Fix.** D2b needs a concrete `pull_move_quintic` C struct + API
header definition. The trade-off is user-visible (bandwidth vs
fidelity) and the spec should make the call, not defer to
implementation time. Related: `tap_analysis.py:453` iterates
trapq-extracted moves — will it silently skip or crash on a v2
move?

**C3. `AUTOTUNE_SHAPERS` + `SHAPER_CALIBRATE` workflow.**
`klippy/extras/shaper_calibrate.py:29-38` lists
`AUTOTUNE_SHAPERS` including all 6 `smooth_*` names. The
resonance_tester `SHAPER_CALIBRATE` command (`:369`) uses this
list to auto-recommend a shaper. D6 retires the `smooth_*`
names; D1 introduces `bs1..bs5`. **If `AUTOTUNE_SHAPERS` isn't
updated in the same commit as `INPUT_SMOOTHERS`, `SHAPER_CALIBRATE`
will either recommend shapers that now error at config-load, or
silently skip the smooth family entirely.**

Additionally: the recommendation logic itself
(`ShaperCalibrate.find_best_shaper`) scores shapers on
(accel, smoothing, vibration) — does that ranking remain
calibrated for the new family, or does it systematically prefer
the old-family minimum that no longer exists? The 5% residual
target + new `σ_T²` values may shift the optimum; user hitting
`SHAPER_CALIBRATE` on first run of new firmware may get
different recommendations than before, from the same data.

**Fix.** D6 or D1 spec addendum: enumerate the full set of
config/calibration surfaces that must update together —
`INPUT_SMOOTHERS`, `AUTOTUNE_SHAPERS`, and any score thresholds
baked into `find_best_shaper`. Gate a test that a known calibration
CSV produces the expected `bs*` recommendation.

**C4. First-move-after-homing is not specified.**
`blendplanner.CornerBlender.feed` (line 86-91): when `self._prev
is None` (first move), the new move is stashed and nothing is
emitted. No quintic is formed. Fine so far.

But what about the **next** move? That's the first real blend,
and its `prev` is the post-homing positioning move. If homing
dwell hasn't settled the lookahead window, the feedforward
inverse's `T_h` extension past the homing transition may try to
look at step-gen history that doesn't exist (or is zero).

Extension: any `_suppress_and_advance` boundary (e.g. pi-radian
reversal) effectively resets the blend state. Does the
feedforward inverse handle the discontinuity, or does it
introduce a transient on both sides of the suppressed corner?

**Fix.** Add a short subsection to D3 or a new "transient
boundaries" subsection covering: (a) first move after homing,
(b) suppressed-V boundaries, (c) print pause / resume
(`pause_resume.py` calls `flush_step_generation` — does the
inverse's lookahead window flush cleanly?), (d) kinematic mode
switches. All four should be integration-tested.

### Important

**I1. Short-move degeneracy (move_t < T_sm).** The spec does not
specify what happens when a planned move is shorter than the
smoothing window `T_sm` (~40-70 ms at 40 Hz). Typical fine-detail
slicer output has sub-mm moves at 50 mm/s (0.02 s/move). Under
`bs3` with `T_sm = 56 ms` and `T_h = 112 ms` (fused 168 ms), a
**single smoother query spans ~8 consecutive moves**.

The shaper convolution at query time already handles this
correctly (range_integrate iterates through the move list). But
two new failure modes appear:

- `v_cap_min` computed per-blend may be misleading if neighboring
  moves have different κ profiles that the inverse sees as one
  integrated signal.
- Junction-cap contract (Option Z) feeds `v_cap_min` to
  LookAheadQueue forward+backward pass. If the true binding
  constraint arises from *cross-move* interaction (multiple short
  moves + fused kernel), a per-blend v_cap_min is locally
  accurate but globally optimistic.

**Fix.** Integration test specifically for short-move sequences
(50-move polyline of 0.1 mm moves at various angles) — this is
the dense-slicer pathological case. Add a runtime assertion /
warning when a single fused-kernel window spans more than N
moves; document the interaction.

**I2. `klipper-sim` / `batch-sim` update timing.** Spec says
"Update klipper-sim deserializer in the same batch" (D2c) — but
the Magnum Opus branch's primary regression diagnosis workflow is
offline klipper-sim bisection (per `reference_klipper_sim.md`, 59
tests). If the deserializer update is part of the Plan 5 PR but
lives in a separate repo (`~/Developer/klipper-sim/`), the
engineer mid-implementation has no bisection tool between
"master works, new commit breaks, where did it go wrong." A
3-day bug hunt with no sim is harder than a 3-hour bug hunt
with sim.

**Fix.** Ship the klipper-sim deserializer update **before** D2
lands in-tree, in a prep commit, behind a feature flag. OR
explicitly call out that bisection during D2 uses direct HW
logging + manual trapq inspection, not sim. Call that choice
before the work starts, not after the sim breaks.

**I3. Tuning surface — user has no way to pick bs1 vs bs5.**
Spec §D1 presents 5 variants with A_axis in the 3635-3810 band
(nearly identical) and `T_sm` varying 39→68 ms (nearly 2×). A
user staring at `shaper_type = bs?` has no principled way to
choose. Currently the guide is "smooth_mzv is the default;
smooth_ei if the frequency identification is imprecise." What's
the analog for bs-family?

From the tables: bs1 has worse passband error (4.79%) but
better HF attenuation. bs5 inverts that (0.54% passband, similar
HF). The "default" is therefore application-dependent. Without
a rule, users will copy random config snippets, some of which
will tune wrong.

**Fix.** D6 needs a short "variant selection guide":
- Default recommendation (`bs2` or `bs3`?)
- "Use `bs1` if: frequency is well-identified, you want fastest
  corners."
- "Use `bs5` if: frequency is uncertain, you want robust to
  mistuning."
- `SHAPER_CALIBRATE` recommendation logic must map to this.

**I4. Comparative-print test absent.** Validation section has
"corner fidelity test" via calipers. That's qualitative. The
spec lacks a *paired* regression test: same gcode, same
toolhead, new vs old firmware, is the new one demonstrably
better or at least not worse? Visual-inspection is not a
regression gate — it's confirmation that something isn't
obviously broken.

**Fix.** Define at least one quantitative acceptance metric:
e.g., corner-region accelerometer RMS under `Voron_Cube` run
— must be within ±10% of pre-Plan-5 figures (not 2× worse).

### Minor

**m1. Plan vs emergency stop (E-stop) interaction.** D5 notes
"Emergency stop: step queue drains within the extended window;
no change to physical stop time." But the queue is larger now;
stop time from *command issue* to *motion ends* grows by
`T_fused/2 - T_sm/2 = T_sm` (40-70 ms). For users with physical
safety interlocks this is measurable.

**m2. `target_smoothing = 0` sentinel on `bs*` variants.** Risk
#8 notes it must survive D1. Strong +1; spec should elevate this
to an explicit test in D1 (currently it's only in the risks
list).

**m3. `SET_INPUT_SHAPER` live-switch.** Risk doesn't address
switching *family* during a print (e.g. `bs2` → `bs5`) where
`T_h` changes. The lookahead window must re-register. Is there
a hazard window where in-flight moves are mid-integration with
the old kernel but the new kernel is already registered?

**m4. `DISABLE_INVERSE_SHAPER` / A-B test knob absent.** For
debugging regressions — *is the inverse making things worse?* —
users and developers need a runtime toggle. Spec doesn't define
one. `target_smoothing = 0` disables the cap but not the
inverse convolution itself.

**m5. EtherCAT claim without numbers.** "Plan 5 is the foundation
for EtherCAT" is an architecture claim. At what PDO rate? What's
the jitter budget through `compute_topp_profile` at emit time?
Not a P1 issue, but the Why-now section leans on this and gets
rhetorical weight it hasn't earned.

**m6. Polar / rotary-delta are not in-scope but the risks list
says they'll "fail cleanly."** The spec should state the
fail-mode **test**: load a polar printer config with Magnum
Opus; verify the error message is informative (not a segfault,
not a silent wrong-answer).

## Unstated assumptions

1. **B-spline order `m` is a config-time constant per-run.**
   Confirmed in "Out of scope" (no adaptive per-move selection),
   but the spec could make this explicit in D1: `shaper_type =
   bs3` picks order `m=3` at config load and never changes
   except via `SET_INPUT_SHAPER`.

2. **All axes share the same `bs*` variant.** Extruder uses the
   same kernel as XY (D3). Multi-family per-axis combinations
   are not supported. If a user configures `shaper_type_x = bs2
   shaper_type_y = bs5`, what happens to the fused kernel?
   Separate per-axis `k_fused`, or error out?

3. **TOPP always converges.** Spec says "ill-posed cases surface
   as plan errors at emit time." But TOPP can produce a profile
   with `v_cruise < v_in` (deceleration needed through the whole
   blend), which is *correct* behavior, not an error. Fall-back
   is a 2-phase profile (decel+cruise, no accel). Does TOPP
   handle this case, or does it assume 3-phase and misbehave?

4. **`corner_deviation` default remains unchanged.** D1 shows
   A_axis drops ~35% for `smooth_zv` users. `corner_deviation`
   is the chord-tolerance knob feeding into blend radius. If
   users raise it to recover speed, blend radius grows and the
   quintic's min-radius changes → `v_sat(s)` changes. Is the
   feedback loop stable? Did anyone think about the user's
   iteration cycle?

5. **`kin_flush_delay` is computed once at config load.** If
   `SET_INPUT_SHAPER` can change the variant mid-print and
   `T_h` changes, then `kin_flush_delay` must recompute and
   `toolhead.note_step_generation_scan_time` must honor the
   new value. Tested?

6. **Linear moves are bit-identical post-Plan-5.** D2a says
   "Linear moves `c_3…c_10 = 0`; existing behavior falls out
   naturally." If the compiler doesn't elide the zero-FMAs,
   there's a float-rounding delta between degree-2 evaluation
   and degree-10-with-zeros evaluation. Bit-identical is the
   stated regression gate — this may fail on certain GCC
   versions. Test on both Trident's GCC and a desktop GCC.

## Suggested additions to the spec

1. **"FFI + ABI surface" subsection (D2b).** Explicit enumeration
   of Python callers of `trapq_*` and `move_*`. Confirm each
   caller's intended behavior post-tagged-union.

2. **"motion_report v2 schema" subsection (D2b).** Concrete on-wire
   format for quintic moves, including payload size numbers.

3. **"Calibration workflow update" subsection (D6).**
   `AUTOTUNE_SHAPERS`, `SHAPER_CALIBRATE`, auto-recommendation
   logic, thresholds, variant-selection guide.

4. **"Transient boundaries" subsection (D3 or D5).** First move
   after homing; suppressed-V; pause/resume; E-stop;
   `SET_INPUT_SHAPER` live-switch. Each gets an integration
   test.

5. **"Debug / diagnostic knobs" subsection (new).**
   `DISABLE_INVERSE_SHAPER` runtime toggle; `LOG_TRAPQ_V2` for
   inspection; metric reporting (v_cap_min histogram per-blend;
   TOPP convergence flag).

6. **"Quantitative acceptance test" subsection (Validation).**
   Specific numeric gates, not just "visually matches."

7. **"MVP slice" subsection (new).** See below.

## MVP slice recommendation

If the 5-7 week estimate slips badly, what's a coherent shippable
subset? Proposed stages, each internally shippable:

**MVP-0 (2 weeks): Plan 5 dry-run without motion-pipeline surgery.**
- D1 alone — ship the `bs*` family on the *existing* polyline
  emit path. Users see new shaper names, improved corner
  fidelity via better kernels, no direct-quintic.
- Skip D2, D3, D7.
- Cap rewards: invertibility proven in production, HW-tested
  on real users before committing to the deep C-side rewrite.

**MVP-1 (+1-2 weeks): Add feedforward inverse (D3) on top of
MVP-0.**
- Still polyline-emit.
- Inverse kernel applied on XY + E.
- Validates Pillar 1 independently of Pillar 2b.
- Main integration risk: lookahead-extension (D5) stacks on
  polyline; verify step-gen budget.

**MVP-2 (full Plan 5): Add direct-quintic (D2) + unified v(s) (D7).**
- The motion-pipeline rewrite.
- Ship only after MVP-0/MVP-1 show HW gains justify the C-side
  disruption.

**Deferrable entirely (if time runs short).**
- D7 Pillar-2b unified v(s). REVIEW_2_MATH #3 showed
  trapezoid-in-s is 1.4% slower than TOPP-optimal. Keep the
  existing midpoint cap + junction contract; accept the 1.4%.
  Revisit as Plan 6.
- Direct-quintic kin_extruder changes — keep extruder on
  polyline for one more release; ship XY direct-quintic first.
  Breaks the spec's "XY+E together" argument but matches
  reality that E-axis ringing has never been user-reported.

## Rollout / rollback plan

**Rollout gates** (in order):
1. D1 lands on a branch; CI passes full test suite + new `bs*`
   tests. User runs `SHAPER_CALIBRATE`; recommendation maps to
   `bs*` without error.
2. klipper-sim deserializer updated; 59-test suite passes
   against new tagged-union format.
3. D2 lands; linear-move bit-identical regression gate passes.
4. D3 lands; cascade identity test passes at ≤ 2% passband err.
5. HW smoke on reference Trident: one print, no sysload
   regression, no `Timer too close`.
6. Invitation to 2-3 community beta testers; one week feedback.
7. Merge to `main`.

**Rollback mechanism.** The branch-as-feature-flag model means
rollback = revert merge commit. But that's 5-7 weeks of work
reverted atomically. More realistic:

- Stage Plan 5 as 4 commits (D1 / D2 / D3 / D7) that can each be
  reverted independently **if** the tagged-union boundary is
  designed so D1 works without D2 and D3 works without D2.
- Today's spec bundles them. **Recommend:** re-order to
  D1 → D3 → D2 → D7 (smooth-path first, then motion-pipeline
  surgery) so that D1+D3 works standalone with polyline emit.

**Diagnostic cadence post-launch.**
- First week: monitor `fatal`, `Timer too close`, sysload
  statistics via user opt-in telemetry or community-reporting
  thread.
- First month: gather user print-quality deltas; track regression
  reports; A-B test via `DISABLE_INVERSE_SHAPER` on reported
  regressions.
- 3-month check: hardware validation against
  `project_hardware_validation.md` — has corner fidelity
  actually improved as predicted?
