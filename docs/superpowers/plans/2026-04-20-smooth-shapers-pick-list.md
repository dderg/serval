# Smooth-Shapers Port Pick-List Archaeology

**Date:** 2026-04-20
**Branch:** `smooth-shapers` (worktree: `~/Developer/kalico-smooth-shapers`, base `4d7a5aa3`)
**Source:** `upstream/bleeding-edge-v2` (KalicoCrew/kalico)
**Commit range analyzed:** 23 PICK commits from `a241cc71` (2021, extruder X/Y/Z split) through `f7def4b7` (2024, `linear_offset` rename).

Verification methodology: `git show <hash> --stat`, `git show <hash> -- <file>` for each commit, cross-referenced against:

- Symbols introduced by the four SKIP commits (`dc2c4d98`, `9c49716e`, `f7a57f05`, `36255dec`).
- blend-arc's local edits in `klippy/extras/input_shaper.py` (28 lines, adds `target_smoothing` + `cmd_SET_INPUT_SHAPER` plumbing) and `klippy/extras/shaper_calibrate.py` (141 lines, rewrites `_get_shaper_smoothing`, drops `scv` from `fit_shaper`/`find_best_shaper`, rewrites `find_shaper_max_accel` with target_smoothing dispatch).

All commands below are rerunnable from the smooth-shapers worktree.

## Main table

| order | hash      | subject                                                                         | files touched                                                                                                        | dep-on-skip? | IS.py conflict | SC.py conflict | notes |
|-------|-----------|---------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|--------------|----------------|----------------|-------|
| 1     | a241cc71  | extruder: Split extruder motion into X/Y/Z components                           | `kin_extruder.c` (+34/-19), `kinematics/extruder.py` (+23/-21)                                                       | No           | None           | None           | Pre-PA refactor. Blend-arc doesn't touch either file. Clean pick. |
| 2     | d3c48f1e  | extruder: Added support for time offset of extruder vs kinematic moves          | `chelper/__init__.py` (+1/-1), `kin_extruder.c` (+27/-3), `kinematics/extruder.py` (+21/-8)                           | No           | None           | None           | Extends `extruder_set_smoothing`; clean. |
| 3     | 957db99d  | extruder: Explicit PA velocity term calculation                                 | `kin_extruder.c` (+49/-26)                                                                                           | No           | None           | None           | C-only. Clean. |
| 4     | 61d1626d  | extruder: Improve numerical stability of time-weighted averaging                | `kin_extruder.c` (+18/-26)                                                                                           | No           | None           | None           | C-only. Clean. |
| 5     | a32f2c62  | extruder: Added support for non-linear Pressure Advance                         | `chelper/__init__.py` (+11/-1), `kin_extruder.c` (+72/-11), `kinematics/extruder.py` (+181/-18)                       | No           | None           | None           | Large but isolated to extruder stack. Python API changes only inside `PrinterExtruder`. |
| 6     | cad447cc  | extruder: Sync extruder motion with input shaping                               | `chelper/__init__.py` (+6/-1), `kin_extruder.c` (+69/-6), `kin_shaper.c` (+13/-13), `kin_shaper.h` (+17/-0), `input_shaper.py` (+123/-8), `kinematics/extruder.py` (+84/-71) | No           | **Medium**     | None           | Adds `enabled_extruders` config, `ENABLE/DISABLE_INPUT_SHAPER` commands, `extruders = []`, `config_extruder_names` — inserted exactly at line 128 of blend-arc's `input_shaper.py`, one line above blend-arc's `self.target_smoothing = config.getfloat(...)`. Resolvable by merging hunks line-by-line; target_smoothing attribute and extruder registration can coexist. Verify: `git show cad447cc -- klippy/extras/input_shaper.py \| sed -n '/@@ -125,6/,/@@ -177,/p'`. |
| 7     | 5800e9e3  | input_shaper: Added custom input shapers support                                | `input_shaper.py` (+188/-45)                                                                                         | No           | **Medium**     | None           | Rewrites `InputShaperParams`, introduces `AxisInputShaper` factoring. Blend-arc's `target_smoothing = config.getfloat(...)` at line ~129 falls inside the refactored `InputShaper.__init__`; the `cmd_SET_INPUT_SHAPER` override at ~218 is also refactored here. Resolution: accept upstream structure, re-apply blend-arc's knob on top (plan Task 4 Step 1 already prescribes this). Verify: `git show 5800e9e3 -- klippy/extras/input_shaper.py \| grep -E '^@@ '`. |
| 8     | ee1181d4  | input_shaper: Added support of smooth input shapers                             | `chelper/__init__.py` (+6/-3), `integrate.c` (+97/-0), `integrate.h` (+15/-0), `kin_extruder.c` (+71/-75), `kin_shaper.c` (+129/-20), `kin_shaper.h` (+3/-1), `trapq.{c,h}` (+4/-4 each), `input_shaper.py` (+154/-0), `kinematics/extruder.py` (+74/-30) | No           | **Medium**     | None           | The big one (543-line commit). Adds `AxisInputSmoother` class below line 256 of upstream `input_shaper.py`. Because blend-arc's knob lives in `InputShaper.__init__` (line ~129) which the 5800e9e3 refactor already reshapes, ee1181d4 sees minor additional conflict in the same zone only if Task 4 Step 1 left stubs. Clean on `trapq.{c,h}` (only 4-line header tweaks). Verify: `git show ee1181d4 -- klippy/chelper/trapq.h \| head -20`. |
| 9     | 01e3767c  | input_shaper: Added some predefined input smoothers                             | `input_shaper.py` (+49/-6), `shaper_defs.py` (+176/-1)                                                               | No           | Trivial        | None           | Adds `INPUT_SMOOTHERS` table in `shaper_defs.py`; `input_shaper.py` touch is a few lines in `ShaperFactory` (line ~407 after earlier picks). Contextual diff mentions `get_2hump_ei_shaper` / `get_3hump_ei_shaper` names — these exist in pre-36255dec upstream already (36255dec is a later redefinition). No dependency on the SKIP. Verify: `git show 01e3767c -- klippy/extras/shaper_defs.py \| grep -E '^\+def ' \| head`. |
| 10    | dc649b48  | integrate: Slightly more optimized versions of smoother integration             | `integrate.c` (+99/-22), `integrate.h` (+12/-1), `kin_extruder.c` (+18/-27), `kin_shaper.c` (+11/-17)                | No           | None           | None           | C-only. Clean. |
| 11    | d7a22b66  | input_shaper: Updated and added some smoother definitions                       | `shaper_defs.py` (+30/-76)                                                                                           | No           | None           | None           | Pure `shaper_defs.py` edit. blend-arc doesn't touch this file. Clean. |
| 12    | 0469a0ed  | integrate: Faster integration via antiderivatives calculation                   | `integrate.c` (+44/-70), `integrate.h` (+17/-4), `kin_extruder.c` (+39/-18), `kin_shaper.c` (+31/-8)                  | No           | None           | None           | C-only. Clean. |
| 13    | b50d378a  | scripts: Support smoothers in shaper calibration and plotting scripts           | `shaper_calibrate.py` (+169/-18), `scripts/graph_shaper.py` (+192/-183)                                              | No           | None           | **Heavy**      | **Highest SC.py risk.** Adds `get_shaper_offset`, `get_smoother_offset`, `estimate_smoother` at top level; modifies `MAX_FREQ`/`MAX_SHAPER_FREQ` constants (blend-arc raised these to 275/215); rewrites `_get_shaper_smoothing` signature (blend-arc already dropped `scv`); rewrites `fit_shaper` loop (blend-arc already dropped `scv` param); rewrites `find_best_shaper` (blend-arc already dropped `scv` param). Every hunk overlaps blend-arc's SCV-removal edits. Resolution: accept upstream as base, re-apply blend-arc's MAX_FREQ/MAX_SHAPER_FREQ constants and `DEFAULT_TARGET_SMOOTHING`/`ShaperCalibrate.__init__(target_smoothing=None)` attribute. The `_get_shaper_smoothing` signature with `accel=5000` (no `scv`) is already blend-arc's intent — keep it. Verify: `git show b50d378a -- klippy/extras/shaper_calibrate.py \| grep -E '^@@ '`. |
| 14    | 9c2c129a  | input_shaper: Added smooth_zvd_ei smoother                                      | `shaper_calibrate.py` (+4/-3), `shaper_defs.py` (+18/-0)                                                             | No           | None           | Trivial        | 3-line change to `AUTOTUNE_SHAPERS` list near top of SC.py. Might conflict with blend-arc's `DEFAULT_TARGET_SMOOTHING` insertion at line ~80, resolvable trivially. |
| 15    | 2c8e98d1  | shaper_calibrate: Use system backwards velocity for shaper estimations          | `shaper_calibrate.py` (+98/-5), `scripts/graph_shaper.py` (+11/-8)                                                   | No           | None           | Medium         | Touches `get_smoother_offset`, `estimate_shaper`, `estimate_smoother` — all are upstream-side functions that don't exist on blend-arc (they arrive via b50d378a). So conflict is only "in the rolled-up post-b50d378a base state." Should merge cleanly on top of b50d378a. |
| 16    | 04b7f77e  | shaper_calibrate: A modified shaper calibration approach                        | `shaper_calibrate.py` (+56/-42), `scripts/calibrate_shaper.py` (+17/-2)                                              | No           | None           | **Medium**     | Rewrites `_estimate_remaining_vibrations` (blend-arc leaves untouched — clean) and `_get_shaper_smoothing` AND `find_shaper_max_accel`. Blend-arc rewrote `find_shaper_max_accel` to accept `target_smoothing=None` and drop `scv`. Upstream here passes `max_scoring_vals` and restructures. Resolution: keep blend-arc's target_smoothing dispatch, pick up upstream's internal restructure. Verify: `git show 04b7f77e -- klippy/extras/shaper_calibrate.py \| sed -n '/@@ -415,/,/@@ -441,/p'`. |
| 17    | 132732c5  | input_shaper: Updated minimum smoother frequencies                              | `shaper_defs.py` (+7/-6)                                                                                             | No           | None           | None           | Trivial. |
| 18    | aad71be9  | input_shaper: Added customized smoothers for extruder                           | `chelper/__init__.py` (+1/-1), `kin_extruder.c` (+2/-1), `input_shaper.py` (+83/-20), `shaper_defs.py` (+165/-0), `kinematics/extruder.py` (+7/-5) | No           | Medium         | None           | Touches `AxisInputShaper` (line ~172) and `AxisInputSmoother` (line ~361) — mostly in zones 5800e9e3/ee1181d4 already reshaped. Blend-arc's knob zone (line ~128 post-port) is not re-edited here, but the `InputShaper` class does gain a `self.extruders` hookup (line ~546). Clean if Task 4 Step 1 re-integration is deferred cleanly. |
| 19    | 124436fd  | input_shaper: Moved shaper/smoother offset calculation functions                | `input_shaper.py` (+9/-17), `shaper_calibrate.py` (+1/-18), `shaper_defs.py` (+24/-0), `scripts/graph_shaper.py` (+2/-2) | No           | Trivial        | Trivial        | Moves helper functions between files. `shaper_calibrate.py` change is a deletion inside the post-b50d378a zone, blend-arc doesn't touch either removed block. |
| 20    | 5cfa8168  | shaper_calibrate: Small fix for input smoother max velocity estimation          | `shaper_calibrate.py` (+8/-5)                                                                                        | No           | None           | Trivial        | Small edit to `estimate_smoother`; function is upstream-only (arrives via b50d378a/2c8e98d1). |
| 21    | 7ac2c445  | input_shaper: Explicit calculation of extruder smoothers                        | `integrate.h` (+1/-1), `extruder_smoother.py` (+199/-0 NEW), `input_shaper.py` (+15/-15), `shaper_defs.py` (-165), `scripts/get_extruder_smoother.py` (+105/-0 NEW) | No           | Medium         | None           | Creates `klippy/extras/extruder_smoother.py`. Deletes 165 lines from `shaper_defs.py` that aad71be9 added. Blend-arc never touched this file. `input_shaper.py` edits are around `AxisInputShaper` line ~231 and `AxisInputSmoother` line ~387 — clean if earlier picks resolved. |
| 22    | ccfb128a  | pressure advance: do not smooth base extruder position, only advance (#212)     | `integrate.c` (+12/-7), `integrate.h` (+3/-2), `kin_extruder.c` (+25/-33), `kin_shaper.c` (+4/-4)                    | No           | None           | None           | C-only. Clean. |
| 23    | f7def4b7  | extruder: rename linear_offset to nonlinear_offset (#622)                       | `kin_extruder.c` (+5/-5), `extruder_smoother.py` (+3/-2), `kinematics/extruder.py` (+12/-12), `test/klippy/extruders.cfg` (+1/-1) | No           | None           | None           | Mechanical rename. Depends on 7ac2c445 having landed (creates `extruder_smoother.py`). Clean. |

**Conflict severity key:** `None` = no overlap with blend-arc zones. `Trivial` = syntactic-only, 3-way merge likely succeeds. `Medium` = overlapping hunks requiring manual resolution but the intent is unambiguous. `Heavy` = rewrites the same function(s) blend-arc rewrote; resolution needs planning.

## Dependencies on SKIP commits

**None found.** Grep across all 23 PICK commits for symbols introduced by the SKIPs returns empty:

```bash
for hash in <PICK_LIST>; do
  git show "$hash" 2>/dev/null | \
    grep -E "high_precision|stepcompress_hp|queue_step_hp|motion_report|motan|stepcompress\.h"
done
# → empty (verified)
```

Detail per SKIP:

- **`9c49716e` high-precision stepper protocol** — introduces `stepcompress_hp_alloc`, `queue_flush_far`, `stepcompress_hp.c`, `_high_precision_steps` attribute on stepper, `queue_step_hp` CLI. None of these symbols appear in any PICK diff. Verify: `git show 9c49716e -- klippy/chelper/stepcompress.h klippy/stepper.py | grep -E '^\+[^+]' | grep -E 'hp_|high_|precision' | head`.
- **`dc2c4d98` motan extended queued-step format** — introduces `add2` and `shift` fields on `history_steps`, and an updated `scripts/motan/readlog.py` format parser. No PICK touches `stepcompress.c`, `stepcompress.h`, or `motion_report.py`. Verify: `for h in <PICK_LIST>; do git show "$h" --stat | grep -E 'stepcompress|motion_report'; done` → empty.
- **`f7a57f05` chelper O3/NEON/native optimization flags** — modifies only `klippy/chelper/__init__.py:_sources`. No PICK references `-O3`, `-march=native`, or `neon` in their diffs. Verify: `for h in <PICK_LIST>; do git show "$h" | grep -E 'O3|NEON|native|march'; done` → empty.
- **`36255dec` *EI shaper redefinition** — rewrites bodies of pre-existing `get_2hump_ei_shaper` and `get_3hump_ei_shaper`, adds a private helper `_get_shaper_from_expansion_coeffs`. Four PICKs mention these names in context (01e3767c, b50d378a, 132732c5, 124436fd), but every reference is to the **pre-36255dec** function (either listing `InputShaperCfg("3hump_ei", get_3hump_ei_shaper, ...)` in a table, or deleting a stale `get_ei_shaper` in `graph_shaper.py`). No PICK calls `_get_shaper_from_expansion_coeffs`. Verify: `for h in 01e3767c b50d378a 132732c5 124436fd; do git show "$h" | grep _get_shaper_from_expansion_coeffs; done` → empty.

**Implication for the plan:** the tentative SKIP list is safe. No minimal-subset pulls needed. No local re-implementation of skipped infrastructure needed.

## Recommended pick-order adjustments

**None needed for correctness** — the topological order already satisfies every inter-commit file dependency (e.g., 7ac2c445 comes after aad71be9 because it deletes 165 lines aad71be9 added).

Two tactical suggestions:

1. **Keep Phase A (task 3) exactly as planned.** Commits 1–6 (`a241cc71` … `cad447cc`) have zero `shaper_calibrate.py` conflict and only one `input_shaper.py` conflict (`cad447cc`). Phase A is the cheapest warm-up and de-risks the chelper rebuild path before the harder picks.

2. **Inside Phase C (task 5), reorder so the `shaper_calibrate.py`-only picks come first.** The plan already lists `01e3767c, d7a22b66, b50d378a, 9c2c129a, 132732c5` as step 1, then the three calibrate commits (`2c8e98d1, 04b7f77e, 5cfa8168`) as step 2. **But `b50d378a` is a calibrate commit** (it adds `get_shaper_offset`, `get_smoother_offset`, `estimate_smoother` to `shaper_calibrate.py` — the biggest SC.py change in the series). Move `b50d378a` into step 2 so all four SC.py-conflict commits are resolved in a cluster:

   ```
   Step 1 (clean picks):      01e3767c, d7a22b66, 9c2c129a, 132732c5
   Step 2 (SC.py conflicts):  b50d378a, 2c8e98d1, 04b7f77e, 5cfa8168
   ```

   Rationale: once the implementer has b50d378a merged cleanly with blend-arc's SCV-removal, the subsequent SC.py picks rest on a stable base and each one's diff becomes substantially smaller.

## Pre-pick sanity checks

Run before each phase. If any check fails, stop and surface.

### Before Phase A (`a241cc71` … `cad447cc`)

```bash
cd ~/Developer/kalico-smooth-shapers
# Confirm worktree is clean and on smooth-shapers
git status --porcelain && git rev-parse --abbrev-ref HEAD
# Confirm chelper builds from blend-arc base
python -c "from klippy import chelper; ffi, lib = chelper.get_ffi(); print('chelper OK')"
# Confirm blend-arc's input_shaper.py still has target_smoothing
grep -n 'target_smoothing' klippy/extras/input_shaper.py
# Expected: hit at line ~137, line ~221, line ~244 (three references)
```

### Before `cad447cc` (within Phase A)

```bash
# Dry-run the pick to preview the conflict
git show cad447cc -- klippy/extras/input_shaper.py | sed -n '/@@ -125,/,/@@ -177,/p'
# Compare to blend-arc's current file at the same line range
sed -n '125,180p' klippy/extras/input_shaper.py
# Rehearse: the target_smoothing attribute belongs at the end of __init__'s
# self.shapers = [...] block; cad447cc's self.extruders / config_extruder_names
# belong before it (they register printer-level state).
```

### Before Phase B (`5800e9e3` … `0469a0ed`)

```bash
# Confirm Phase A's extruder work compiles under the current blend-arc kinematics
python -m pytest test/test_extruder_overrides_simple.py -x -q
# Confirm blend-arc's target_smoothing attribute still exists after 5800e9e3's
# InputShaperParams refactor — sed-check the file before committing the pick
grep -n 'self.target_smoothing' klippy/extras/input_shaper.py
```

### Before Phase C (`01e3767c` … `5cfa8168`)

```bash
# Confirm the reordered pick list (see Recommendation 2). Verify blend-arc's
# shaper_calibrate.py signature changes are still in place before b50d378a.
grep -n 'def find_shaper_max_accel\|def _get_shaper_smoothing\|def fit_shaper\|def find_best_shaper' klippy/extras/shaper_calibrate.py
# Each signature should still be scv-free (blend-arc's state).
# If any line shows a 'scv' parameter, an earlier pick accidentally pulled
# upstream's older API — stop and audit.
```

### Before Phase D (`aad71be9` … `f7def4b7`)

```bash
# Confirm INPUT_SMOOTHERS table is present (arrived in ee1181d4/01e3767c)
python -c "from klippy.extras import shaper_defs; print(len(shaper_defs.INPUT_SMOOTHERS))"
# Expected: ≥ 5 before 7ac2c445 (smooth_si arrives later in the pick order).
# Confirm extruder_smoother.py does NOT exist yet (7ac2c445 creates it)
test -f klippy/extras/extruder_smoother.py && echo "ALREADY EXISTS" || echo "OK, absent"
# Expected: absent until 7ac2c445 lands.
```

### After each pick in any phase

```bash
# 1. chelper rebuild
python -c "from klippy import chelper; ffi, lib = chelper.get_ffi(); print('chelper OK')"
# 2. Python compile check
python -m compileall -q klippy/
# 3. Quick assertion: blend-arc's target_smoothing attribute is still alive
grep -c 'target_smoothing' klippy/extras/input_shaper.py
# Expected: ≥ 1 (even if re-located by the pick). If it's 0 after a pick,
# stop and re-apply the blend-arc knob before continuing.
```

---

**Summary of findings:** No hard dependencies on any SKIP commit. The two non-trivial conflict zones are `cad447cc` vs blend-arc's `target_smoothing` config parse (Medium, line-level merge), `5800e9e3`+`ee1181d4` vs blend-arc's `InputShaper.__init__`/`cmd_SET_INPUT_SHAPER` (Medium, refactor on top of which blend-arc's knob must be re-applied — plan Task 4 Step 1 already prescribes this), and `b50d378a`+`04b7f77e` vs blend-arc's SCV-removal in `shaper_calibrate.py` (Medium–Heavy, same-function rewrites; resolution is to keep blend-arc's scv-free signatures while pulling in upstream's smoother-handling additions). One pick-order nit: move `b50d378a` into Phase C step 2 alongside the other SC.py conflict commits.
