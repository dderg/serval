# Shaper-Calibration SCV Removal — Design Spec

**Date:** 2026-04-18
**Branch:** `blend-arc` (continues sub-spec #5 work)
**Sub-spec:** #6a of the blend-arc roadmap
**Status:** DRAFT — pending user review

---

## Goal

Delete the `scv` (square corner velocity) parameter from the shaper-calibration math chain — `_get_shaper_smoothing`, `find_shaper_max_accel`, `fit_shaper`, `find_best_shaper` — and from every caller (`resonance_tester.py`, `blendmath.py`, `scripts/calibrate_shaper.py`, tests). Sub-spec #5 removed SCV from the planner but left a vestigial `scv` parameter threading through shaper calibration as a temporary bridge. This sub-spec finishes that removal.

The `scv` term in `_get_shaper_smoothing` modeled **shaper-induced corner-cutoff at a sharp 90° turn entered at velocity `scv`**. In the arc-blending pipeline shipped in sub-specs #1–#4, sharp 90° corners at any velocity do not occur — every kin↔kin corner is rounded into a tangent arc bounded by shaper-derived caps. The `offset_90` term is therefore modeling a non-event, and the `scv` parameter threaded through four functions serves no purpose.

Like sub-spec #5, this is a **pure code-deletion pass**: no new behavior, no new knobs, no migration shims. Fork-as-gate.

## Architecture

```
Before (post sub-spec #5):
  _get_shaper_smoothing(shaper, accel, scv) → max(offset_90, offset_180)
    offset_90  = √2 · Σ(i≥ts) A_i · (scv + a/2·(T_i-ts)) · (T_i-ts) / ΣA
    offset_180 = Σ(all i) A_i · a/2 · (T_i-ts)² / ΣA
  find_shaper_max_accel(shaper, scv) → bisect {smoothing ≤ 0.12mm}
  fit_shaper(..., scv, ...) → per-freq smoothing + per-shaper max_accel
  find_best_shaper(..., scv=None, ...) → public API entry
  resonance_tester.py:572 → scv = 5.0 (hardcoded bridge from #5)
  blendmath.py:291 → sc.find_shaper_max_accel(impulses, scv=0.0)
  scripts/calibrate_shaper.py → --scv CLI option, default 5.0

After:
  _get_shaper_smoothing(shaper, accel) → offset_180 only
    offset_180 = Σ(all i) A_i · a/2 · (T_i-ts)² / ΣA
  find_shaper_max_accel(shaper) → bisect {offset_180 ≤ 0.12mm}
  fit_shaper(...) → no scv
  find_best_shaper(...) → no scv
  resonance_tester.py:570-572 → no scv read, call drops kwarg
  blendmath.py:291 → sc.find_shaper_max_accel(impulses)
  scripts/calibrate_shaper.py → no --scv, no scv kwarg
```

## Verification (cross-checked by two independent research subagents)

**Shaper-physics agent** — derived `offset_90` step by step: it is the Euclidean corner-cutoff distance at a sharp 90° turn entered at speed `scv`, with `√2` from combining orthogonal x/y shaper-lag residuals. Derived `offset_180`: shaper residual displacement at the cusp of a 180° velocity reversal from zero, modeled as the shaped position when commanded position is `(a/2)(t−ts)²`. Verified Kalico's U-turn handling (`next_junction_v2 = 0` at blender-declined corners) matches `offset_180`'s kinematic assumption. Verdict: in an arc-blending pipeline where no sharp 90° corner is ever executed, `offset_90` models a fictitious event; `offset_180` remains physically correct for U-turns. Flagged that steady-state arc traversal at centripetal accel `a_c = v²/R` also carries a shaper residual of magnitude `(a_c/2)·σ²_T`, but since the blender caps `v² ≤ A_axis · R` by construction, `a_c ≤ A_axis`, and the arc residual at the bound equals the U-turn residual at the same accel. No additional `offset_arc` term required — adding one would not change `A_axis` materially. Cited Singer & Seering 1990, Singhose 1997, Cho et al. 2018.

**Calibration prior-art agent** — surveyed Fanuc AICC II, Siemens 840D G645, Haas HSM, LinuxCNC, Prunt, DangerKlipper, Klippain Shake&Tune, Orca Slicer. Finding: every industrial system and every peer-reviewed treatment calibrates input-shaping (or FIR-equivalent) filters from **measured system dynamics alone** — resonance frequency, damping ratio, system ID — decoupled from any trajectory knob. Corner-rounding tolerance is a separate parameter (Siemens `$SC_SMOOTH_CONTUR_TOL`, Fanuc G05.1 level, Prunt `max_corner_deviation`). Klipper's `find_shaper_max_accel` bisection against a 0.12 mm path-deviation target is a Klipper-original convenience, not an industry standard. The `scv` parameter specifically is Dmitry Butyugin's heuristic for sharp-corner deviation in Klipper's junction model; no published derivation uses this formulation. Verdict: drop `scv` entirely, keep the 0.12 mm bisection for now (it is a reasonable empirical heuristic validated by years of Klipper user data). Cited Cho 2018, Klippain Shake&Tune source, Fanuc AI Servo Tuning docs, Siemens 840D manual, Prunt implementing_a_shaper docs.

**Numerical verification** — with `scv=0`, `_get_shaper_smoothing` already returns `offset_180` in the dominant case (verified algebraically for symmetric shapers like ZV: at `scv=0`, `offset_90 = offset_180 / √2`, so `max()` always picks `offset_180`). The runtime `blendmath.py:291` call already passes `scv=0.0`, so the arc-blend pipeline is already effectively on Option A. Only the user-facing `SHAPER_CALIBRATE` path in `resonance_tester.py` (with `scv=5.0` hardcoded in #5) has a behavioral change; at realistic calibration accels (~20k mm/s² for ZV at 50 Hz), the `scv=5` contribution is under 10% of `offset_90` and the bisection result shifts by <5% — negligible compared to the ±20% spread in real-world shaper recommendations.

## In Scope

### `klippy/extras/shaper_calibrate.py`

**`_get_shaper_smoothing`** (line 240) — drop `scv` parameter; drop `offset_90` block; drop `offset_90` from the `max()` return. After:

```python
def _get_shaper_smoothing(self, shaper, accel=5000):
    half_accel = accel * 0.5
    A, T = shaper
    inv_D = 1.0 / sum(A)
    n = len(T)
    ts = sum([A[i] * T[i] for i in range(n)]) * inv_D
    offset_180 = 0.0
    for i in range(n):
        offset_180 += A[i] * half_accel * (T[i] - ts) ** 2
    return offset_180 * inv_D
```

**`find_shaper_max_accel`** (line 361) — drop `scv` parameter; update internal `_get_shaper_smoothing` call. After:

```python
def find_shaper_max_accel(self, shaper):
    TARGET_SMOOTHING = 0.12
    max_accel = self._bisect(
        lambda test_accel: (
            self._get_shaper_smoothing(shaper, test_accel)
            <= TARGET_SMOOTHING
        )
    )
    return max_accel
```

**`fit_shaper`** (line 262) — drop `scv` from parameter list; drop `scv` from the `_get_shaper_smoothing` call at line 302 and the `find_shaper_max_accel` call at line 314.

**`find_best_shaper`** (line 373) — drop `scv` from parameter list; drop `scv` from the `fit_shaper` invocation at line 398.

### `klippy/extras/resonance_tester.py`

**Lines 570–577** — delete the hardcoded `scv = 5.0` bridge added in sub-spec #5; drop `scv=scv` from the `helper.find_best_shaper(...)` call. After:

```python
toolhead = self.printer.lookup_object("toolhead")
# Note: A_axis for the blender comes from find_shaper_max_accel,
# called by _extract_shapers in blendmath.py. Nothing to pass here.
max_freq = self._get_max_calibration_freq()
best_shaper, all_shapers = helper.find_best_shaper(
    calibration_data[axis],
    max_smoothing=max_smoothing,
    max_freq=max_freq,
    logger=gcmd.respond_info,
)
```

The `toolhead_info = toolhead.get_status(systime)` line at (former) line 571 is also deleted — nothing reads from it anymore after this change.

### `klippy/blendmath.py`

**Line 291** — drop the `scv=0.0` kwarg from the `find_shaper_max_accel` call:

```python
# BEFORE
A_axis = float(sc.find_shaper_max_accel(impulses, scv=0.0))
# AFTER
A_axis = float(sc.find_shaper_max_accel(impulses))
```

No other changes in blendmath (the adapter has no other `scv` references).

### `scripts/calibrate_shaper.py`

**Lines 76, 94, 98, 246, 249, 352** — drop `scv` parameter threading and the `--scv` CLI option:

- Remove `scv` from the local function signature (line 76).
- Remove `scv=scv` from the `helper.find_best_shaper` call (lines 94–98).
- Remove the `--scv` argparse entry (lines 246–249).
- Remove `scv=options.scv` from the invocation (line 352).

Per the SCV-removal spec sub-spec #5 deferral note, this script was flagged for sub-spec #7 for docs/script updates. Because the `find_best_shaper` signature change is a hard break, it is folded into 6a rather than left dangling.

### `test/test_blendshaper.py`

**Line 339** — drop `scv=0.0` from the `sc.find_shaper_max_accel(shaper, scv=0.0)` test-helper call:

```python
# BEFORE
return sc.find_shaper_max_accel(shaper, scv=0.0)
# AFTER
return sc.find_shaper_max_accel(shaper)
```

### New tests (in `test/test_blendplanner.py` or a new `test/test_shaper_calibrate.py`)

Kalico does not currently have a dedicated `test_shaper_calibrate.py` file. New tests land in an existing test file nearest the concern. Given the math tests in `test_blendshaper.py` and to avoid a one-test file, we extend `test/test_blendplanner.py` with shaper-smoothing structural tests.

1. **`test_shaper_smoothing_returns_offset_180_only`** — construct a ZV shaper at 50 Hz (known `A=[0.5, 0.5]`, `T=[0, 0.01]`), call `_get_shaper_smoothing(shaper, accel=10000)`, assert return value matches the closed-form `(a/2) · σ²_T`:

   ```python
   def test_shaper_smoothing_returns_offset_180_only():
       from klippy.extras import shaper_defs, shaper_calibrate
       sc = shaper_calibrate.ShaperCalibrate(printer=None)
       shaper = shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)
       # A, T per shaper_defs; closed form:
       A = [0.5, 0.5]
       T_d = 1.0 / (50.0 * math.sqrt(1.0 - 0.01))
       T = [0.0, 0.5 * T_d]
       ts = 0.5 * T[1]
       sigma2 = sum(A_i * (T_i - ts) ** 2 for A_i, T_i in zip(A, T)) / sum(A)
       accel = 10000.0
       expected = 0.5 * accel * sigma2
       actual = sc._get_shaper_smoothing(shaper, accel)
       assert actual == pytest.approx(expected, rel=1e-9)
   ```

2. **`test_shaper_smoothing_signature_drops_scv`** — assert `_get_shaper_smoothing` raises `TypeError` when called with `scv=` kwarg:

   ```python
   def test_shaper_smoothing_signature_drops_scv():
       sc = shaper_calibrate.ShaperCalibrate(printer=None)
       shaper = shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)
       with pytest.raises(TypeError, match="scv"):
           sc._get_shaper_smoothing(shaper, 5000, scv=5.0)
   ```

3. **`test_find_shaper_max_accel_drops_scv_positional`** — same check for positional arg:

   ```python
   def test_find_shaper_max_accel_drops_scv_positional():
       sc = shaper_calibrate.ShaperCalibrate(printer=None)
       shaper = shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)
       with pytest.raises(TypeError):
           sc.find_shaper_max_accel(shaper, 0.0)  # old positional scv
   ```

4. **`test_find_shaper_max_accel_bisection_regression`** — regression pin: for a ZV shaper at 50 Hz with damping ratio 0.1, the closed-form answer is `A = 0.24 / σ²_T` with `σ²_T = (T_d / 4)² · 1` where `T_d = 1 / (f · √(1 − ζ²))`. At `f=50, ζ=0.1`: `T_d ≈ 0.02010`, `σ²_T ≈ 2.525e-5`, `A ≈ 9505 mm/s²`. Assert returned value in [9000, 10000]:

   ```python
   def test_find_shaper_max_accel_bisection_regression():
       sc = shaper_calibrate.ShaperCalibrate(printer=None)
       shaper = shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)
       max_accel = sc.find_shaper_max_accel(shaper)
       assert 9000.0 <= max_accel <= 10000.0
   ```

5. **`test_find_best_shaper_drops_scv_kwarg`** — assert the public API rejects `scv=`:

   ```python
   def test_find_best_shaper_drops_scv_kwarg():
       sc = shaper_calibrate.ShaperCalibrate(printer=None)
       with pytest.raises(TypeError, match="scv"):
           sc.find_best_shaper(calibration_data=None, scv=5.0)
   ```

## Out of Scope (deferred)

- **Z-axis blender audit** — Sub-spec **6b**. Separate concern: whether `CornerBlender` should blend corners that involve Z transitions (layer changes, Z-lift retracts). Orthogonal to shaper math.
- **Ghosting-aware analytical `A_axis` derivation** — **future sub-spec** (see "Future Work" below). Replaces the `0.12 mm path-deviation` bisection with a direct physics-based calculation from shaper `(freq, damping_ratio, type)` properties grounded in residual-vibration theory.
- **Docs** (`docs/Resonance_Compensation.md`, `docs/Measuring_Resonances.md`, `docs/Config_Reference.md`) — still sub-spec **#7**.
- **Klippain Shake&Tune fix** — External. File an upstream issue against `github.com/Frix-x/klippain-shaketune` documenting the `find_best_shaper` signature change. No in-fork shim (fork-as-gate). Shake&Tune already crashes post-#5 via `KeyError` on `square_corner_velocity`; this sub-spec adds a second break (TypeError on `scv=` kwarg) that the same upstream fix will resolve.
- **TARGET_SMOOTHING = 0.12 constant** — unchanged. Empirically validated Klipper default; reconsidering requires its own sub-spec and user-facing impact assessment.

## Future Work: Ghosting-Aware Analytical `A_axis`

Noted here for traceability; not part of 6a.

The long-term direction is to replace the `find_shaper_max_accel` bisection with a closed-form analytical formula that directly predicts **at what acceleration the input shaper's residual vibration amplitude exceeds a ghosting threshold**, rather than the current `path-deviation ≤ 0.12 mm` heuristic.

Motivation:
- The 0.12 mm target is a Klipper-era convenience, not tied to any print-quality metric.
- Industry (Fanuc/Siemens/Prunt/Cho 2018) derives filter parameters from system-identification only (resonance frequency + damping), decoupled from trajectory. The bisection against a deviation target is a Klipper-original shortcut.
- In the arc-blend world, path deviations during arcs are naturally bounded by the blender's own caps. The calibration-time deviation target is loosely related to runtime behavior.

Candidate rewrite:

```
A_axis = f(shaper_freq, damping_ratio, shaper_type)
```

grounded in Singer-family residual-vibration theory and a print-quality threshold (e.g., visible ghosting amplitude at typical print speeds). Inputs align with industry system-ID practice; output is an analytical scalar, no bisection.

Deferred because:
- Requires derivation + hardware validation on real printers (not just math).
- Changes the user-facing `SHAPER_CALIBRATE` recommendation numbers, needs migration story.
- Orthogonal to 6a's cleanup goal.

To be addressed in a later sub-spec (provisionally "sub-spec 6c: analytical ghosting-aware A_axis") after 6a + 6b ship and real-world arc-blend data is available.

## Known Downstream Impact

| Consumer | Impact | Symptom |
|---|---|---|
| Klippain Shake&Tune | HARD CRASH (second break) | Calls `helper.find_best_shaper(..., scv=scv, ...)` in `shaketune/graph_creators/computations/shaper_computation.py`. Already crashes post-#5 via `KeyError` on `toolhead_info["square_corner_velocity"]`. After 6a: additional `TypeError: find_best_shaper() got unexpected keyword argument 'scv'`. Upstream fix: drop the dict read and the `scv=` kwarg. |
| `scripts/calibrate_shaper.py` CLI users | HARD BREAK | `--scv 5.0` CLI argument no longer exists. Users invoking this offline tool with `--scv` will see argparse error. Acceptable per fork-as-gate; loud signal that the tool updated. |
| SHAPER_CALIBRATE gcode output | SOFT SHIFT | Recommended `shaper_type_X` and `max_accel_X` values may shift by ≤5% (dropping the 5 mm/s residual `scv` term in the bisection). No user action required. |
| Moonraker / Mainsail / Fluidd / KlipperScreen / OctoPrint | NO_IMPACT | None of these introspect `scv` — only the planner-side fields that #5 removed. |
| jschuh klipper-macros | NO_IMPACT | No shaper-calibration references. |

**Recommendation**: file an upstream issue against Klippain Shake&Tune consolidating both breaks (the `KeyError` from #5 and the `TypeError` from 6a). One fix-PR covers both.

## Risk Surface

Single residual risk: users calibrating at aggressive acceleration regimes (>30 000 mm/s²) with low-frequency shapers (<40 Hz) may see slightly different recommendations than pre-#5 Klipper. The direction is **looser** (dropping the 5 mm/s entry-velocity term raises A_axis), not tighter, so no mechanical overload risk. Difference bounded at ~5% in the worst case.

No risk in the arc-blend runtime path: `blendmath.py:291` already passed `scv=0.0` in sub-spec #3 (shaper-derived `j_eff`), so the runtime `A_axis` value is bit-identical before and after.

## Implementation Notes

- All changes land on the `blend-arc` branch on top of sub-spec #5's commits.
- No new files (unless a new `test/test_shaper_calibrate.py` is cleaner than extending `test_blendplanner.py` — decide at plan-writing time).
- Estimated diff: ~40 LOC removed, ~60 LOC added (mostly tests).
- Estimated commits: 4–6 small TDD commits (one per function signature change + tests).

## Open Items for Sub-Spec 6b (Z-axis audit)

- Audit `CornerBlender.feed` for corners where one or more involved moves has a non-zero Z component.
- Decide policy: (a) skip blending if Z changes, (b) force hard stop at Z transitions, (c) blend only if all three points share Z, (d) trust existing math. Recommended (c) by default, matching industrial-CNC convention.
- Layer-change pattern: `XY → Z-lift → XY` — verify current behavior, add regression test.
- Z-lift retract pattern: `XY+E- → Z+ → Z- → XY+E+` — verify current behavior.

## Open Items for Sub-Spec #7 (docs + examples)

Unchanged from sub-spec #5's deferral list. 6a does not add to this list.
