# Smooth Shapers Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port Dmitry Butyugin's smooth input shapers (and the PA/extruder refactor chain they rest on) from `upstream/bleeding-edge-v2` onto the `smooth-shapers` branch, prune impulse shapers down to `zv`/`mzv`, and adapt the blend-arc `target_smoothing` runtime cap so it uses the correct velocity-envelope formula for the configured shaper family.

**Architecture:** Cherry-pick ~23 commits from `upstream/bleeding-edge-v2` onto a branch cut from `blend-arc`. Skip ~4 unrelated commits (motan, high-precision stepper, chelper O3/NEON flags, impulse-EI shaper redefinition). Resolve merge conflicts against the two files blend-arc modifies (`klippy/extras/input_shaper.py`, `klippy/extras/shaper_calibrate.py`). After the port, prune the impulse-shaper set and re-derive the `target_smoothing` cap formula for the smooth-shaper family (math delegated to a research subagent per the project's math-via-subagent rule).

**Tech Stack:** Python 3 (Kalico/Klipper host-side planner), C (chelper kinematics), git cherry-pick for upstream import, pytest for regression tests, host-simulator (`test/configs/hostsimulator.config`) for end-to-end behavior.

**Worktree:** `~/Developer/kalico-smooth-shapers`
**Branch:** `smooth-shapers` (cut from `blend-arc` at commit `04943583`)
**Spec:** `docs/superpowers/specs/2026-04-20-smooth-shapers-port-design.md`

---

## File Structure

Files that will be **modified** by the cherry-pick chain (upstream authors land most of these):

- `klippy/chelper/kin_shaper.c` — smooth shaper convolution path
- `klippy/chelper/kin_shaper.h` — smoother API typedefs
- `klippy/chelper/kin_extruder.c` — extruder-side smoothing, PA refactor
- `klippy/chelper/integrate.c` — antiderivative-based smoother integration
- `klippy/chelper/integrate.h` — integration API
- `klippy/chelper/trapq.c` / `trapq.h` — touched by extruder/PA prereq commits
- `klippy/chelper/__init__.py` — C bindings for new functions
- `klippy/kinematics/extruder.py` — X/Y/Z split, time-offset, non-linear PA, smoother hookup
- `klippy/extras/input_shaper.py` — smooth shaper classes, family dispatch; **conflicts with blend-arc `target_smoothing` knob**
- `klippy/extras/shaper_defs.py` — `INPUT_SMOOTHERS` table; impulse table trimmed to `zv`/`mzv`
- `klippy/extras/shaper_calibrate.py` — smoother candidate path, revised scoring; **conflicts with blend-arc changes**
- `scripts/calibrate_shaper.py` — support smoother plotting
- `scripts/graph_shaper.py` — same

Files **created** by this plan (not by upstream):

- `test/test_smooth_shaper.py` — sanity tests for the new shaper classes (import, minimal attrs, end-to-end step-queue smoke with `smooth_mzv`)
- `test/test_target_smoothing_families.py` — regression pins for the family-dispatch `target_smoothing` cap

Files that will **remain untouched** by this plan:

- `klippy/extras/blendmath.py`, `klippy/extras/blendplanner.py`, `klippy/extras/blendshaper.py` — blend-arc core; none of the cherry-picked commits touch these.
- Other tests under `test/test_blend*.py` — must keep passing after the port.

---

## Standard cherry-pick procedure (reference)

Each cherry-pick task below uses this core sequence. Inlined verbatim in the first phase task; later phases reference it.

```bash
# From ~/Developer/kalico-smooth-shapers
git cherry-pick <hash>            # initiate pick
# If conflict: resolve in the listed files, then:
#   git add <files>
#   git cherry-pick --continue
# After commit lands:
make -C klippy/chelper             # rebuild c_helper.so
python -m compileall klippy/       # ensure Python compiles
```

Then, at phase end only, run the test subset listed for that phase.

---

## Task 1: Baseline smoke

**Files:**
- Modify: none (verification only)

- [ ] **Step 1: Confirm worktree is on `smooth-shapers`**

Run:
```bash
git -C ~/Developer/kalico-smooth-shapers rev-parse --abbrev-ref HEAD
```
Expected: `smooth-shapers`

- [ ] **Step 2: Confirm base commit**

Run:
```bash
git -C ~/Developer/kalico-smooth-shapers rev-parse HEAD
```
Expected: commit matching the base of `blend-arc` at plan creation (`04943583` or the spec-commit head `2a3ffbbe`). Either is fine — the spec commit is a doc-only addition on top of `blend-arc`.

- [ ] **Step 3: Build chelper from scratch**

Run:
```bash
cd ~/Developer/kalico-smooth-shapers && make -C klippy/chelper clean && make -C klippy/chelper
```
Expected: no errors, `klippy/chelper/c_helper.so` produced.

- [ ] **Step 4: Run full host-side test suite**

Run:
```bash
cd ~/Developer/kalico-smooth-shapers && python -m pytest test/ -x -q 2>&1 | tail -40
```
Expected: all pass. Record the pass count in a scratch note so we can compare after the port.

- [ ] **Step 5: Run host-simulator regression**

Run:
```bash
cd ~/Developer/kalico-smooth-shapers && python scripts/test_klippy.py -d dict/ test/klippy/*.test 2>&1 | tail -5
```
Expected: "All tests passed" or equivalent. Record.

- [ ] **Step 6: No commit — baseline only.**

---

## Task 2: Git-archaeology subagent — ordered pick list

**Files:**
- Create: `docs/superpowers/plans/2026-04-20-smooth-shapers-pick-list.md` (subagent output)

- [ ] **Step 1: Dispatch research subagent**

Dispatch a general-purpose agent with this prompt verbatim:

> I am porting smooth input shapers from `upstream/bleeding-edge-v2` to a branch cut from `blend-arc` (current commit `2a3ffbbe`, cwd `~/Developer/kalico-smooth-shapers`).
>
> The spec is at `docs/superpowers/specs/2026-04-20-smooth-shapers-port-design.md`. Read it first.
>
> Tentative pick list (topo-order, `git log --topo-order --reverse upstream/bleeding-edge-v2 ^upstream/main -- <shaper/extruder files>`):
>
> ```
> a241cc71 extruder: Split extruder motion into X/Y/Z components
> d3c48f1e extruder: Added support for time offset of extruder vs kinematic moves
> 957db99d extruder: Explicit PA velocity term calculation
> 61d1626d extruder: Improve numerical stability of time-weighted averaging
> a32f2c62 extruder: Added support for non-linear Pressure Advance
> cad447cc extruder: Sync extruder motion with input shaping
> dc2c4d98 motan: Report queued steps in extended format            -- SKIP (tentative)
> 9c49716e stepper: New optional high precision stepping protocol   -- SKIP (tentative)
> 5800e9e3 input_shaper: Added custom input shapers support
> ee1181d4 input_shaper: Added support of smooth input shapers
> 01e3767c input_shaper: Added some predefined input smoothers
> dc649b48 integrate: Slightly more optimized versions of smoother integration
> f7a57f05 chelper: Added O3, NEON (for ARM) and native CPU optimizations flags  -- SKIP (tentative)
> d7a22b66 input_shaper: Updated and added some smoother definitions
> 0469a0ed integrate: Faster integration via antiderivatives calculation
> b50d378a scripts: Support smoothers in shaper calibration and plotting scripts
> 9c2c129a input_shaper: Added smooth_zvd_ei smoother
> 2c8e98d1 shaper_calibrate: Use system backwards velocity for shaper estimations
> 04b7f77e shaper_calibrate: A modified shaper calibration approach
> 132732c5 input_shaper: Updated minimum smoother frequencies
> aad71be9 input_shaper: Added customized smoothers for extruder
> 124436fd input_shaper: Moved shaper/smoother offset calculation functions
> 5cfa8168 shaper_calibrate: Small fix for input smoother max velocity estimation
> 7ac2c445 input_shaper: Explicit calculation of extruder smoother
> 36255dec input_shaper: Updated definitions of *EI input shapers   -- SKIP (dropping impulse EI)
> ccfb128a pressure advance: do not smooth base extruder position, only advance (#212)
> f7def4b7 extruder: rename linear_offset to nonlinear_offset (#622)
> ```
>
> For each commit, produce: (a) files touched, (b) whether it depends on any SKIP commit (especially `9c49716e` stepper-protocol and `dc2c4d98` motan), and (c) expected conflict severity against `blend-arc` on `klippy/extras/input_shaper.py` and `klippy/extras/shaper_calibrate.py`.
>
> **Verification: for each dependency claim, show the specific file/symbol pair that creates the dependency.** Do not assert "depends on X" without a file-level diff reference.
>
> If you find a hard dependency on a SKIP commit, propose either (i) pulling in a minimal subset of that skipped commit, or (ii) a local patch over the picked commit that re-implements the dependency without the skipped infrastructure.
>
> Deliverable: write your findings to `docs/superpowers/plans/2026-04-20-smooth-shapers-pick-list.md` as a table with columns `order | hash | subject | files | dep-on-skip? | conflict-with-blend-arc | notes`, followed by a "pre-pick sanity checks" section if you found issues.
>
> Stay under 300 lines.

- [ ] **Step 2: Review subagent output**

Read the generated `pick-list.md`. If the subagent flagged hard dependencies on skipped commits, stop and surface them to the user before proceeding with the cherry-picks — the plan may need revision.

- [ ] **Step 3: Commit the archaeology output**

```bash
cd ~/Developer/kalico-smooth-shapers
git add docs/superpowers/plans/2026-04-20-smooth-shapers-pick-list.md
git commit -m "docs: smooth-shapers port pick-list archaeology"
```

---

## Task 3: Cherry-pick Phase A — extruder/PA prereqs (6 commits)

**Files (aggregate across phase):**
- Modify: `klippy/chelper/kin_extruder.c`, `klippy/chelper/trapq.c`, `klippy/chelper/trapq.h`, `klippy/chelper/__init__.py`, `klippy/kinematics/extruder.py`

Phase commits, in order:

| # | Hash | Subject |
|---|------|---------|
| 1 | `a241cc71` | extruder: Split extruder motion into X/Y/Z components |
| 2 | `d3c48f1e` | extruder: Added support for time offset of extruder vs kinematic moves |
| 3 | `957db99d` | extruder: Explicit PA velocity term calculation |
| 4 | `61d1626d` | extruder: Improve numerical stability of time-weighted averaging |
| 5 | `a32f2c62` | extruder: Added support for non-linear Pressure Advance |
| 6 | `cad447cc` | extruder: Sync extruder motion with input shaping |

- [ ] **Step 1: Pick commits 1–6 in sequence**

For each hash in the table above, run:

```bash
cd ~/Developer/kalico-smooth-shapers
git cherry-pick <hash>
```

Conflicts expected: unlikely — blend-arc does not touch `kin_extruder.c`, `trapq.c/h`, or `kinematics/extruder.py`. If a conflict does occur, resolve by keeping the upstream (`--theirs`) version for hunks that are pure refactor, and by merging manually for hunks that overlap blend-arc-specific code. After resolving:

```bash
git add <resolved files>
git cherry-pick --continue
```

- [ ] **Step 2: Rebuild chelper**

```bash
cd ~/Developer/kalico-smooth-shapers && make -C klippy/chelper
```
Expected: clean build.

- [ ] **Step 3: Compile check Python**

```bash
cd ~/Developer/kalico-smooth-shapers && python -m compileall klippy/
```
Expected: no syntax errors.

- [ ] **Step 4: Run extruder & blend-planner tests**

```bash
cd ~/Developer/kalico-smooth-shapers && python -m pytest test/test_extruder_overrides_simple.py test/test_blendmath.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendshaper.py -x -q
```
Expected: all pass. If a `test_extruder_overrides_simple` test fails because the non-linear PA change altered semantics, pause and surface to user — this is a scope question.

- [ ] **Step 5: No additional commit (cherry-picks already committed)**

Move to Task 4.

---

## Task 4: Cherry-pick Phase B — smooth-shaper chelper core (4 commits)

**Files:**
- Modify: `klippy/chelper/kin_shaper.c`, `klippy/chelper/kin_shaper.h`, `klippy/chelper/integrate.c`, `klippy/chelper/integrate.h`, `klippy/chelper/__init__.py`, `klippy/chelper/kin_extruder.c` (incidental)

Phase commits:

| # | Hash | Subject |
|---|------|---------|
| 1 | `5800e9e3` | input_shaper: Added custom input shapers support |
| 2 | `ee1181d4` | input_shaper: Added support of smooth input shapers |
| 3 | `dc649b48` | integrate: Slightly more optimized versions of smoother integration |
| 4 | `0469a0ed` | integrate: Faster integration via antiderivatives calculation |

**Known conflict risk:** `ee1181d4` and `5800e9e3` touch `klippy/extras/input_shaper.py`, where blend-arc has `target_smoothing` knob code (28 lines). Plan: accept upstream structure, then re-integrate blend-arc's knob in Task 9.

- [ ] **Step 1: Pick `5800e9e3`**

```bash
cd ~/Developer/kalico-smooth-shapers && git cherry-pick 5800e9e3
```
Expected conflict in `klippy/extras/input_shaper.py` on the blend-arc knob block. Resolution strategy:
- Keep upstream's refactored file as the base.
- Preserve blend-arc's `target_smoothing` parsing / `SET_INPUT_SHAPER` hook / `ts=0` sentinel / status exposure by porting them onto the new class structure.
- If unsure, keep the blend-arc hooks commented out with `# TARGET_SMOOTHING: re-integrated in Task 9`, and continue.

After resolving:
```bash
git add klippy/extras/input_shaper.py
git cherry-pick --continue
```

- [ ] **Step 2: Pick `ee1181d4` (the core smooth-shaper commit)**

```bash
git cherry-pick ee1181d4
```
This is the biggest commit of the series (~543 lines across 10 files). Expect conflict again in `input_shaper.py` if Task 4 Step 1's resolution left stub markers. Resolve analogously. Rebuild chelper after the pick:

```bash
make -C klippy/chelper
```
Expected: clean build. Smooth-shaper types (`SmoothShaperClass`) should now be defined.

- [ ] **Step 3: Pick `dc649b48` and `0469a0ed`**

```bash
git cherry-pick dc649b48
git cherry-pick 0469a0ed
```
Both touch only chelper (`integrate.c/h`, `kin_shaper.c`, `kin_extruder.c`). Expect clean picks.

- [ ] **Step 4: Rebuild and compile check**

```bash
make -C klippy/chelper
python -m compileall klippy/
```

- [ ] **Step 5: Smoke — import + instantiate a smooth shaper**

Run inline:
```bash
cd ~/Developer/kalico-smooth-shapers && python -c "
from klippy.extras import shaper_defs
smoothers = getattr(shaper_defs, 'INPUT_SMOOTHERS', None)
assert smoothers is not None, 'INPUT_SMOOTHERS missing — ee1181d4 not applied'
print('INPUT_SMOOTHERS count:', len(smoothers))
print('names:', [s.name for s in smoothers])
"
```
Expected: at least `smooth_zv`, `smooth_mzv` present (and likely more, which we'll prune later). If `INPUT_SMOOTHERS` is empty or missing, stop.

- [ ] **Step 6: No additional commit (cherry-picks already committed).**

---

## Task 5: Cherry-pick Phase C — shaper defs + calibrate (8 commits)

**Files:**
- Modify: `klippy/extras/shaper_defs.py`, `klippy/extras/shaper_calibrate.py`, `klippy/extras/input_shaper.py`, `scripts/calibrate_shaper.py`, `scripts/graph_shaper.py`

Phase commits:

| # | Hash | Subject |
|---|------|---------|
| 1 | `01e3767c` | input_shaper: Added some predefined input smoothers |
| 2 | `d7a22b66` | input_shaper: Updated and added some smoother definitions |
| 3 | `b50d378a` | scripts: Support smoothers in shaper calibration and plotting scripts |
| 4 | `9c2c129a` | input_shaper: Added smooth_zvd_ei smoother |
| 5 | `2c8e98d1` | shaper_calibrate: Use system backwards velocity for shaper estimations |
| 6 | `04b7f77e` | shaper_calibrate: A modified shaper calibration approach |
| 7 | `132732c5` | input_shaper: Updated minimum smoother frequencies |
| 8 | `5cfa8168` | shaper_calibrate: Small fix for input smoother max velocity estimation |

**Known conflict risk:** every commit that touches `shaper_calibrate.py` will conflict with blend-arc's 60-line changes. These are the ones to pay careful attention to: `2c8e98d1`, `04b7f77e`, `5cfa8168`.

- [ ] **Step 1: Pick non-calibrate commits first**

```bash
cd ~/Developer/kalico-smooth-shapers
git cherry-pick 01e3767c
git cherry-pick d7a22b66
git cherry-pick b50d378a
git cherry-pick 9c2c129a
git cherry-pick 132732c5
```
Most touch `shaper_defs.py` or scripts — expect clean picks.

- [ ] **Step 2: Pick the three calibrate commits, resolve carefully**

```bash
git cherry-pick 2c8e98d1
```
Resolve conflict in `shaper_calibrate.py`. Strategy:
- Accept upstream's restructured scoring as the new base.
- Blend-arc's `shaper_calibrate.py` changes were part of sub-spec 6a (SCV removal). Verify: read `docs/superpowers/specs/2026-04-18-subspec-6a-shaper-scv-removal-design.md` for the intent of the blend-arc edits, then re-apply those edits onto the upstream base. Commit the result.

```bash
git cherry-pick 04b7f77e
git cherry-pick 5cfa8168
```
Repeat resolution strategy for each.

- [ ] **Step 3: Rebuild + compile**

```bash
make -C klippy/chelper
python -m compileall klippy/
```

- [ ] **Step 4: Run shaper calibrate tests**

```bash
python -m pytest test/test_shaper_calibrate.py test/test_blendshaper.py -x -q
```
Expected: all pass. If `test_find_shaper_max_accel_matches_offset_180_closed_form` now fails, our sub-spec 6a re-application missed something; investigate before continuing.

- [ ] **Step 5: No additional commit.**

---

## Task 6: Cherry-pick Phase D — extruder smoother tail (4 commits)

**Files:**
- Modify: `klippy/extras/input_shaper.py`, `klippy/chelper/kin_extruder.c`, `klippy/kinematics/extruder.py`, `klippy/chelper/kin_shaper.c`, `klippy/chelper/__init__.py`

Phase commits:

| # | Hash | Subject |
|---|------|---------|
| 1 | `aad71be9` | input_shaper: Added customized smoothers for extruder |
| 2 | `124436fd` | input_shaper: Moved shaper/smoother offset calculation functions |
| 3 | `7ac2c445` | input_shaper: Explicit calculation of extruder smoother |
| 4 | `ccfb128a` | pressure advance: do not smooth base extruder position, only advance (#212) |

Also included at the end: `f7def4b7` (rename `linear_offset` → `nonlinear_offset`).

- [ ] **Step 1: Pick in order**

```bash
cd ~/Developer/kalico-smooth-shapers
git cherry-pick aad71be9
git cherry-pick 124436fd
git cherry-pick 7ac2c445
git cherry-pick ccfb128a
git cherry-pick f7def4b7
```

For each: resolve any `input_shaper.py` conflicts using the same strategy as Phase B. The blend-arc knob re-integration is still deferred to Task 9.

- [ ] **Step 2: Rebuild + compile**

```bash
make -C klippy/chelper
python -m compileall klippy/
```

- [ ] **Step 3: Full suite**

```bash
python -m pytest test/ -x -q 2>&1 | tail -20
```
Expected: everything passes **except** any `target_smoothing`-dependent test, which may be stubbed off due to Task 4 Step 1's deferred re-integration. Record failures; they should be resolved by Task 9.

- [ ] **Step 4: No additional commit.**

---

## Task 7: Prune impulse shapers to `zv` and `mzv`

**Files:**
- Modify: `klippy/extras/shaper_defs.py`
- Modify: `klippy/extras/shaper_calibrate.py`
- Modify: `scripts/calibrate_shaper.py`, `scripts/graph_shaper.py`
- Modify: `docs/Resonance_Compensation.md`, `docs/Measuring_Resonances.md`, other docs that list impulse shapers

- [ ] **Step 1: Identify every definition of a non-kept impulse shaper**

```bash
cd ~/Developer/kalico-smooth-shapers
grep -nE "get_(zvd|2hump_ei|3hump_ei|ei|si)_shaper|'(zvd|2hump_ei|3hump_ei|ei|si)'" klippy/extras/shaper_defs.py klippy/extras/shaper_calibrate.py
```

- [ ] **Step 2: Remove from `shaper_defs.INPUT_SHAPERS`**

Edit `klippy/extras/shaper_defs.py` so `INPUT_SHAPERS = [...]` contains only the `zv` and `mzv` entries. Delete the helper functions (`get_zvd_shaper`, `get_ei_shaper`, `get_2hump_ei_shaper`, `get_3hump_ei_shaper`, `get_si_shaper`) and any module-level constants that only serve those shapers.

- [ ] **Step 3: Remove from `shaper_calibrate.AUTOTUNE_SHAPERS`**

In `klippy/extras/shaper_calibrate.py`, the top-of-file `AUTOTUNE_SHAPERS` list should contain only `'zv'`, `'mzv'`, plus the six smooth variants:
```python
AUTOTUNE_SHAPERS = [
    "zv",
    "mzv",
    "smooth_zv",
    "smooth_mzv",
    "smooth_ei",
    "smooth_2hump_ei",
    "smooth_zvd_ei",
    "smooth_si",
]
```

- [ ] **Step 4: Scrub scripts and docs**

```bash
grep -nE "zvd|2hump_ei|3hump_ei|'ei'|\bei\b|'si'" scripts/calibrate_shaper.py scripts/graph_shaper.py docs/Resonance_Compensation.md docs/Measuring_Resonances.md
```
Remove mentions of dropped impulse shapers. Keep mentions of `smooth_ei`, `smooth_2hump_ei`, etc. (smooth variants remain).

- [ ] **Step 5: Compile check**

```bash
python -m compileall klippy/ scripts/
```

- [ ] **Step 6: Full suite**

```bash
python -m pytest test/ -x -q 2>&1 | tail -20
```
If a test pins a dropped impulse shaper by name (e.g., `test_shaper_calibrate.py` may reference `ei`), update the test to pin `mzv` or `smooth_mzv` instead. Record changes.

- [ ] **Step 7: Commit**

```bash
git add klippy/extras/shaper_defs.py klippy/extras/shaper_calibrate.py scripts/calibrate_shaper.py scripts/graph_shaper.py docs/
git add test/test_shaper_calibrate.py  # if modified
git commit -m "shaper: prune impulse set to zv and mzv"
```

---

## Task 8: Target_smoothing math — research subagent

**Files:**
- Create: `docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md` (subagent output)

- [ ] **Step 1: Dispatch research subagent**

Dispatch a general-purpose agent with this prompt verbatim:

> I need a derivation for a runtime accel cap on smooth-family input shapers, under the same design goal as the existing impulse-family cap on `blend-arc`.
>
> **Context:** Read `docs/superpowers/specs/2026-04-20-smooth-shapers-port-design.md` for goals, and these two prior specs for how the impulse cap was derived on blend-arc:
>
> - `docs/superpowers/specs/2026-04-18-subspec-6a-shaper-scv-removal-design.md`
> - The git log subject of commit `310a3ee9` ("blendmath: derive sigma_T from impulse pattern, not target_smoothing") and `5743ed91` ("blendmath: suppress arc when mainline-SCV equivalent would be no slower") — both on `blend-arc`.
>
> **Existing impulse formula:** the cap currently uses `sigma_T` derived from the impulse train, and the max accel satisfies `offset_180(A, sigma_T) ≤ target_smoothing`. Read `klippy/extras/shaper_calibrate.py` and the current `input_shaper.py` (on `blend-arc`) for the exact formulas in use now.
>
> **Deliverables:**
>
> 1. For smooth shapers (support function `S(t)` with finite width `2·t_sm`), derive the analogue of `sigma_T`. Use the support function's second moment: `sigma_T^2 = ∫ t^2 · S(t) dt - (∫ t · S(t) dt)^2`.
> 2. Close-form or bisection-ready formula for `find_max_accel(smoother, target_smoothing) = max A such that offset_180(A, smoother) ≤ target_smoothing`.
> 3. **Limiting-case check:** show that as `t_sm → 0` with the support function concentrating into a pair of deltas, the smooth formula reduces to the impulse formula. Numerically verify at three widths: `t_sm ∈ {1e-4, 1e-3, 1e-2}` seconds, with a ZV-like impulse pair at 50 Hz.
> 4. **Mainline-SCV floor** (per `5743ed91`): give the smooth-shaper analogue. The cap should not force a slower max_accel than the planner would choose under a pure mainline-SCV constraint with no smoother cap. Derive the comparison.
> 5. A recommended function signature for `input_shaper.py`: `def find_smoother_max_accel(smoother, target_smoothing: float, scv: float | None = None) -> float:`.
>
> **Numerical verification** must run end-to-end: produce a small script that plots or tables `offset_180(A, smoother)` as a function of `A` for `smooth_mzv` at 40 Hz, and confirms the bisected root matches the closed-form.
>
> Stay under 400 lines. Write findings to `docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md`.

- [ ] **Step 2: Review subagent output**

Read the generated spec. Confirm:
- Limiting case passes numerically.
- The recommended function signature and formula are precise enough to implement from.
- The mainline-SCV floor analogue is concrete (not hand-wavy).

If any of the above fails, push back on the subagent with specific concerns.

- [ ] **Step 3: Commit**

```bash
cd ~/Developer/kalico-smooth-shapers
git add docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md
git commit -m "docs: smooth-shaper target_smoothing derivation"
```

---

## Task 9: Re-integrate blend-arc `target_smoothing` with family dispatch

**Files:**
- Modify: `klippy/extras/shaper_calibrate.py` (primary — `ShaperCalibrate.find_shaper_max_accel` lives here as an instance method)
- Modify: `klippy/extras/input_shaper.py` (call site for the cap in the runtime path)
- Create: `test/test_target_smoothing_families.py`

**Context — where the cap actually lives on blend-arc:**

- `ShaperCalibrate.find_shaper_max_accel(self, shaper, target_smoothing=None)` at `klippy/extras/shaper_calibrate.py:365` — bisects accel using `_get_shaper_smoothing`.
- `klippy/extras/input_shaper.py` calls into that at line 138 (runtime cap path).
- After the BE-v2 port, `ShaperCalibrate._get_smoother_smoothing` also exists alongside `_get_shaper_smoothing`. The dispatch should pick one or the other based on input type.

- [ ] **Step 1: Record the impulse baseline on `blend-arc` BEFORE this task begins changing anything**

This step must run against the original `blend-arc` worktree, not the smooth-shapers worktree, because we need the pre-refactor value.

```bash
cd /Users/daniladergachev/Developer/kalico
python -c "
from klippy.extras import shaper_calibrate, shaper_defs
sc = shaper_calibrate.ShaperCalibrate(printer=None)
s = shaper_defs.get_mzv_shaper(shaper_freq=50.0, damping_ratio=0.1)
print(repr(sc.find_shaper_max_accel(s, target_smoothing=0.12)))
" > /tmp/mzv_baseline.txt
cat /tmp/mzv_baseline.txt
```

Note: if `ShaperCalibrate.__init__` requires a non-None `printer`, fall back to instantiating via the module-level constructor the unit tests use — read `test/test_shaper_calibrate.py` for the canonical construction pattern and mirror it.

Record the printed number — this becomes `EXPECTED_ACCEL_MZV_50HZ` in the test below.

- [ ] **Step 2: Write the failing tests (in the smooth-shapers worktree)**

Create `test/test_target_smoothing_families.py`:

```python
"""Family-dispatch regression for target_smoothing runtime cap.

After smooth-shapers port: ShaperCalibrate.find_shaper_max_accel must
work for both impulse and smooth families. Uses the derivation in
docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md.
"""
import math

import pytest

from klippy.extras import shaper_calibrate, shaper_defs


TARGET_SMOOTHING = 0.12  # mm

# Recorded from pre-port blend-arc in Task 9 Step 1. Fill in before
# running the tests. Value is the float printed by the baseline command.
EXPECTED_ACCEL_MZV_50HZ = None  # <<< replace with Task 9 Step 1 output


def _sc():
    """Mirror the ShaperCalibrate construction used in test_shaper_calibrate."""
    return shaper_calibrate.ShaperCalibrate(printer=None)


def test_impulse_family_unchanged_mzv_50hz():
    """After refactor, impulse branch must reproduce pre-refactor value."""
    assert EXPECTED_ACCEL_MZV_50HZ is not None, \
        "Baseline not filled — run Task 9 Step 1 first"
    sc = _sc()
    shaper = shaper_defs.get_mzv_shaper(shaper_freq=50.0, damping_ratio=0.1)
    accel = sc.find_shaper_max_accel(shaper, target_smoothing=TARGET_SMOOTHING)
    assert accel == pytest.approx(EXPECTED_ACCEL_MZV_50HZ, rel=1e-6)


def _smooth_mzv():
    smoother_cfg = [s for s in shaper_defs.INPUT_SMOOTHERS
                    if s.name == "smooth_mzv"][0]
    return smoother_cfg.init_func(shaper_freq=40.0, damping_ratio=0.1)


def test_smooth_family_returns_finite_accel():
    """Smooth branch must return a positive, finite accel."""
    sc = _sc()
    sm = _smooth_mzv()
    accel = sc.find_shaper_max_accel(sm, target_smoothing=TARGET_SMOOTHING)
    assert math.isfinite(accel) and accel > 0.0


def test_smooth_family_tighter_at_smaller_budget():
    """Halving the target_smoothing budget must not raise the cap."""
    sc = _sc()
    sm = _smooth_mzv()
    accel_loose = sc.find_shaper_max_accel(sm, target_smoothing=0.24)
    accel_tight = sc.find_shaper_max_accel(sm, target_smoothing=0.12)
    assert accel_tight <= accel_loose + 1e-9


def test_dispatch_accepts_both_families():
    """Single public entry point, two families, no exceptions."""
    sc = _sc()
    mzv = shaper_defs.get_mzv_shaper(shaper_freq=50.0, damping_ratio=0.1)
    sm = _smooth_mzv()
    assert sc.find_shaper_max_accel(mzv, target_smoothing=TARGET_SMOOTHING) > 0
    assert sc.find_shaper_max_accel(sm, target_smoothing=TARGET_SMOOTHING) > 0
```

If `init_func` is not the correct attribute on the `INPUT_SMOOTHERS` entry (the BE-v2 naming might differ), read `klippy/extras/shaper_defs.py` after the port and adjust — the field that returns a callable producing the smoother instance is what we want.

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd ~/Developer/kalico-smooth-shapers && python -m pytest test/test_target_smoothing_families.py -x -v
```
Expected: fails because (a) `EXPECTED_ACCEL_MZV_50HZ` is `None`, or (b) `find_shaper_max_accel` does not yet dispatch on smoother input.

Fill in `EXPECTED_ACCEL_MZV_50HZ` from Task 9 Step 1, rerun:

```bash
python -m pytest test/test_target_smoothing_families.py -x -v
```
Expected now: `test_impulse_family_unchanged_mzv_50hz` may pass (if the impulse branch is untouched) and the smooth tests fail because the method doesn't know what to do with a smoother.

- [ ] **Step 4: Implement the family dispatch in `shaper_calibrate.py`**

At `klippy/extras/shaper_calibrate.py`, modify `ShaperCalibrate.find_shaper_max_accel` so it dispatches to the appropriate smoothing getter:

```python
def find_shaper_max_accel(self, shaper, target_smoothing=None):
    """Bisect on the largest accel whose effective smoothing stays under
    ``target_smoothing``. Dispatches on family:
    - impulse shaper: uses ``_get_shaper_smoothing`` (existing).
    - smooth shaper: uses ``_get_smoother_smoothing`` (from BE-v2 port).
    Derivation of the smooth branch:
    docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md.
    """
    target = (
        self.target_smoothing if target_smoothing is None
        else target_smoothing
    )
    if _is_smoother(shaper):
        get_smoothing = self._get_smoother_smoothing
    else:
        get_smoothing = self._get_shaper_smoothing
    max_accel = self._bisect(
        lambda test_accel: get_smoothing(shaper, test_accel) <= target
    )
    return max_accel
```

Add the type test at module scope (above the class):

```python
def _is_smoother(obj):
    """Return True for smooth-family shapers (support function + t_sm).

    Impulse shapers are ``(amplitudes, times)`` pair lists; smooth
    shapers are instances of the BE-v2 smoother class with a callable
    support function. Exact attribute to sniff depends on the ported
    smoother class — update this after Task 4 lands the class, based on
    what ``INPUT_SMOOTHERS[0].init_func(...)`` returns.
    """
    return hasattr(obj, "t_sm") or hasattr(obj, "support")
```

If the smoother class exposes a cleaner discriminator (e.g., `isinstance(obj, SmoothShaperClass)`), prefer that — read what `ee1181d4` introduced and use the upstream type check.

If Task 4/5/6 left any `# TARGET_SMOOTHING: re-integrated in Task 9` stub markers in `input_shaper.py`, resolve them now by routing the call through the updated `ShaperCalibrate.find_shaper_max_accel`.

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd ~/Developer/kalico-smooth-shapers && python -m pytest test/test_target_smoothing_families.py -x -v
```
Expected: all 4 tests pass.

- [ ] **Step 6: Run the full suite**

```bash
python -m pytest test/ -x -q 2>&1 | tail -20
```
Expected: all pre-existing tests continue passing, plus the 4 new tests. Any blend-arc-specific test that referenced `target_smoothing` should now pass without stubs.

- [ ] **Step 7: Commit**

```bash
git add klippy/extras/shaper_calibrate.py klippy/extras/input_shaper.py test/test_target_smoothing_families.py
git commit -m "shaper_calibrate: family-aware target_smoothing cap (smooth + impulse)"
```

---

## Task 10: End-to-end verification

**Files:** verification only.

- [ ] **Step 1: Full Python test suite**

```bash
cd ~/Developer/kalico-smooth-shapers && python -m pytest test/ -x -q 2>&1 | tail -20
```
Expected: same pass count as Task 1 baseline + 4 new tests from Task 9.

- [ ] **Step 2: Host-simulator regression**

```bash
cd ~/Developer/kalico-smooth-shapers && python scripts/test_klippy.py -d dict/ test/klippy/*.test 2>&1 | tail -5
```
Expected: same as baseline.

- [ ] **Step 3: Smoke with smooth_mzv configured**

Copy `test/configs/hostsimulator.config` to a scratch name and switch its `[input_shaper]` section to:
```ini
[input_shaper]
shaper_type_x: smooth_mzv
shaper_freq_x: 40
shaper_type_y: smooth_mzv
shaper_freq_y: 40
target_smoothing: 0.12
```

Then run the host simulator on a short G-code (20 s cube perimeter or similar) and confirm:
- No exceptions.
- Deterministic step-queue output.
- The status field `target_smoothing` from the status-reference commit still reads 0.12.

Record the run as a baseline for hardware validation later.

- [ ] **Step 4: Verify no impulse shaper except zv/mzv is loadable**

```bash
cd ~/Developer/kalico-smooth-shapers && python -c "
from klippy.extras import shaper_defs
names = [s.name for s in shaper_defs.INPUT_SHAPERS]
print('impulse shapers:', names)
assert set(names) == {'zv', 'mzv'}, names
print('smoothers:', [s.name for s in shaper_defs.INPUT_SMOOTHERS])
"
```
Expected: `impulse shapers: ['zv', 'mzv']` plus the six smooth variants listed.

- [ ] **Step 5: Push the branch (optional, for backup)**

```bash
git push -u origin smooth-shapers
```
Only if user has approved the push (see `finishing-a-development-branch` flow).

---

## Non-goals / reminders (repeated from spec)

- No feature flags or runtime compat shims — branch replaces behavior cleanly.
- No `Co-Authored-By: Claude` trailers in any commit.
- Any new math beyond Task 8 goes through a research subagent, not inline reasoning.
- After the implementation closes, invoke the `finishing-a-development-branch` skill rather than ad-hoc merge decisions.
