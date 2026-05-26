# Remove Legacy C Chelper Motion Code

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the legacy C step-generation code (itersolve, trapq, kin_*) and their Python FFI bindings that are dead since the Rust bridge owns all motion.

**Architecture:** The Rust motion bridge (`motion_bridge_native.so`) replaced the C itersolve→trapq→stepcompress step-generation pipeline. Commit `a0f0b9209` ("klippy: drop non-bridge code paths") already removed the Python-side legacy branches, leaving the C source files and their FFI declarations as dead weight. `stepcompress.c` stays — it's still used by `pwm_tool.py` for queued PWM scheduling (heater/fan control, not motion). `trdispatch.c` stays — it's used by probe MCU endstop homing.

**Tech Stack:** Python (klippy), C (chelper), cffi

---

## File Map

### C files to delete
- `klippy/chelper/itersolve.c` — zero Python callers
- `klippy/chelper/itersolve.h` — zero Python callers
- `klippy/chelper/trapq.c` — allocated by `ToolHead.__init__` but never populated in bridge mode
- `klippy/chelper/trapq.h` — header for trapq.c
- `klippy/chelper/kin_cartesian.c` — `cartesian_stepper_alloc` called by no-op `setup_itersolve`
- `klippy/chelper/kin_extruder.c` — `extruder_stepper_alloc` / `extruder_set_pressure_advance` called by trad_rack but functionally dead
- `klippy/chelper/kin_shaper.c` — `input_shaper_*` called by `input_shaper.py` but all steppers have `get_trapq() == None` so the loop body never executes
- `klippy/chelper/kin_idex.c` — `dual_carriage_*` called by `idex_modes.py` which already unconditionally errors

### C files to keep
- `stepcompress.c` / `stepcompress.h` — live caller: `pwm_tool.py`
- `serialqueue.c` / `serialqueue.h` — live: serial communication
- `trdispatch.c` — live: probe endstop homing on non-bridge MCUs
- `pollreactor.c` / `pollreactor.h` — live: reactor
- `msgblock.c` / `msgblock.h` — live: message framing
- `pyhelper.c` / `pyhelper.h` — live: logging
- `compiler.h` — live: macros used by kept files
- `list.h` — live: list macros used by kept files (check: may only be used by trapq — verify)

### Python files to modify
- `klippy/chelper/__init__.py` — remove deleted C files from SOURCE_FILES/OTHER_FILES, remove `defs_itersolve`, `defs_trapq`, `defs_kin_*` from FFI declarations and `defs_all`
- `klippy/toolhead.py` — remove trapq alloc/append/finalize; keep everything else (move planning, lookahead, etc. are still inherited by MotionToolhead)
- `klippy/extras/input_shaper.py` — remove all C FFI calls; keep config parsing, status reporting, and `cmd_SET_INPUT_SHAPER` (which delegates to bridge)
- `klippy/extras/motion_report.py` — remove `DumpTrapQ` class and all `trapq_extract_old` calls; keep `DumpStepper` (API compat) and `PrinterMotionReport` status reporting
- `klippy/extras/force_move.py` — remove chelper import; dead code behind unconditional errors already uses lambda stubs
- `klippy/kinematics/extruder.py` — already stubs trapq as None/lambda; remove any remaining chelper references
- `klippy/kinematics/idex_modes.py` — remove chelper FFI calls; already unconditionally errors
- `klippy/extras/manual_stepper.py` — remove trapq alloc/append/generate_steps
- `klippy/extras/z_tilt.py` — remove set_trapq calls (no-ops on stepper stubs)
- `klippy/extras/z_tilt_ng.py` — remove set_trapq calls
- `klippy/extras/trad_rack.py` — remove trapq/cartesian/extruder FFI calls
- `klippy/extras/mixing_extruder.py` — remove set_trapq calls
- `klippy/mcu.py` — remove dead `_stepqueues`, `_steppersync`, `register_stepqueue`, `flush_moves`; keep `_bridge_drives_steppers` (real per-MCU distinction for probe MCUs)

### Python API stubs to keep on `stepper.py`
- `set_trapq()` / `get_trapq()` — called by z_tilt, extruder, trad_rack, manual_stepper, mixing_extruder; keeping as no-op stubs avoids touching all callers for zero benefit
- `generate_steps()` — called by extruder; no-op
- `set_stepper_kinematics()` / `get_stepper_kinematics()` — called by input_shaper, idex, trad_rack; no-op

---

### Task 1: Delete C source files and update chelper/__init__.py

**Files:**
- Delete: `klippy/chelper/itersolve.c`, `klippy/chelper/itersolve.h`, `klippy/chelper/trapq.c`, `klippy/chelper/trapq.h`, `klippy/chelper/kin_cartesian.c`, `klippy/chelper/kin_extruder.c`, `klippy/chelper/kin_shaper.c`, `klippy/chelper/kin_idex.c`
- Modify: `klippy/chelper/__init__.py`

- [ ] **Step 1: Delete the 8 C files**

```bash
cd klippy/chelper
rm itersolve.c itersolve.h trapq.c trapq.h kin_cartesian.c kin_extruder.c kin_shaper.c kin_idex.c
```

- [ ] **Step 2: Check if list.h is used by any kept file**

```bash
grep -l "list.h" stepcompress.c serialqueue.c trdispatch.c pollreactor.c msgblock.c pyhelper.c
```

If only deleted files included it, remove `list.h` from OTHER_FILES too.

- [ ] **Step 3: Update chelper/__init__.py**

Remove from `SOURCE_FILES`: `itersolve.c`, `trapq.c`, `kin_cartesian.c`, `kin_extruder.c`, `kin_shaper.c`, `kin_idex.c`
Remove from `OTHER_FILES`: `itersolve.h`, `trapq.h`
Delete the entire `defs_itersolve`, `defs_trapq`, `defs_kin_cartesian`, `defs_kin_extruder`, `defs_kin_shaper`, `defs_kin_idex` string blocks.
Remove them from `defs_all`.

Result: `defs_all` should contain only `defs_pyhelper`, `defs_serialqueue`, `defs_std`, `defs_stepcompress`, `defs_trdispatch`.

- [ ] **Step 4: Delete stale c_helper.so to force rebuild**

```bash
rm -f klippy/chelper/c_helper.so
```

- [ ] **Step 5: Verify c_helper.so builds**

```bash
cd <worktree-root> && python3 -c "from klippy.chelper import get_ffi; get_ffi(); print('OK')"
```

Expected: `OK` — the shared object rebuilds with only the kept source files.

- [ ] **Step 6: Commit**

```bash
git add -A klippy/chelper/
git commit -m "remove legacy C motion code (itersolve, trapq, kin_*) from chelper"
```

---

### Task 2: Remove trapq usage from toolhead.py

**Files:**
- Modify: `klippy/toolhead.py`

- [ ] **Step 1: Remove trapq allocation and references from ToolHead.__init__**

Remove:
- `self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)` → `self.trapq = None`
- `self.trapq_append = ffi_lib.trapq_append` → delete
- `self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves` → delete
- The `ffi_main, ffi_lib = chelper.get_ffi()` call (if only used for trapq — check if it has other uses)

- [ ] **Step 2: Remove trapq_append calls from _process_moves**

The `self.trapq_append(self.trapq, ...)` call in `_process_moves` is dead (MotionToolhead.move() bypasses it). Remove the call but keep the method structure (extruder.move still runs through it).

- [ ] **Step 3: Remove trapq_finalize_moves calls**

Remove from `_advance_flush_time` and `_handle_shutdown`.

- [ ] **Step 4: Remove trapq_set_position call from set_position**

The `ffi_lib.trapq_set_position(self.trapq, ...)` call in `set_position`.

- [ ] **Step 5: Remove chelper import if no longer needed**

Check if `toolhead.py` still uses chelper for anything else. If not, remove the import.

- [ ] **Step 6: Commit**

```bash
git add klippy/toolhead.py
git commit -m "toolhead: remove dead trapq alloc/append/finalize"
```

---

### Task 3: Remove C FFI calls from input_shaper.py

**Files:**
- Modify: `klippy/extras/input_shaper.py`

- [ ] **Step 1: Remove chelper import**

- [ ] **Step 2: Remove C FFI calls from AxisInputShaper.set_shaper_kinematics**

Replace the body with `return True` — the bridge handles IS via `bridge.update_shaper()`.

- [ ] **Step 3: Remove _get_input_shaper_stepper_kinematics**

Delete the entire method — it allocates C stepper_kinematics objects.

- [ ] **Step 4: Simplify _update_input_shaping**

Remove the stepper loop that calls C FFI. The method can become a no-op or just validate params.

- [ ] **Step 5: Remove input_shaper_stepper_kinematics and orig_stepper_kinematics lists**

No longer needed.

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/input_shaper.py
git commit -m "input_shaper: remove dead C FFI calls (IS lives in Rust bridge)"
```

---

### Task 4: Remove DumpTrapQ from motion_report.py

**Files:**
- Modify: `klippy/extras/motion_report.py`

- [ ] **Step 1: Delete the DumpTrapQ class entirely**

Lines 99-200.

- [ ] **Step 2: Remove trapq references from PrinterMotionReport**

- Remove `self.trapqs = {}` and all code that reads/writes it
- Remove `_dump_shutdown`'s trapq extraction loop and toolhead position lookup
- Remove `get_status`'s trapq position lookup — return zeros or the bridge-sourced position

- [ ] **Step 3: Remove chelper import**

- [ ] **Step 4: Commit**

```bash
git add klippy/extras/motion_report.py
git commit -m "motion_report: remove DumpTrapQ (trapq is dead in bridge mode)"
```

---

### Task 5: Clean up remaining Python files

**Files:**
- Modify: `klippy/extras/force_move.py`, `klippy/kinematics/extruder.py`, `klippy/kinematics/idex_modes.py`, `klippy/extras/manual_stepper.py`, `klippy/extras/z_tilt.py`, `klippy/extras/z_tilt_ng.py`, `klippy/extras/trad_rack.py`, `klippy/extras/mixing_extruder.py`

- [ ] **Step 1: force_move.py — remove chelper import and dead trapq stubs**

Remove `from klippy import chelper`. The lambda stubs for `trapq_append` / `trapq_finalize_moves` and `self.stepper_kinematics = None` can stay (dead code behind `raise` guards) or be cleaned up.

- [ ] **Step 2: extruder.py — confirm no chelper usage remains**

`PrinterExtruder` already stubs trapq as `None` / `lambda`. Verify no chelper import exists. `ExtruderStepper` may reference chelper via `extruder_set_pressure_advance` — check and remove.

- [ ] **Step 3: idex_modes.py — remove chelper FFI calls**

Remove `dual_carriage_alloc`, `dual_carriage_set_sk`, `dual_carriage_set_transform` calls. The code already unconditionally errors, so the FFI calls are unreachable.

- [ ] **Step 4: manual_stepper.py — remove trapq alloc/append**

Remove `ffi_lib.trapq_alloc()`, `ffi_lib.trapq_append`, `ffi_lib.trapq_finalize_moves`, and the chelper import. The `ManualStepper.do_move` and `do_homing_move` methods use these — they're dead in bridge mode. Replace with stubs or bridge calls as appropriate.

- [ ] **Step 5: z_tilt.py and z_tilt_ng.py — remove set_trapq calls**

The `s.set_trapq(None)` / `stepper.set_trapq(toolhead.get_trapq())` calls are no-ops (stepper.set_trapq just stores a Python reference). These can stay as harmless no-ops or be removed. Remove for cleanliness.

- [ ] **Step 6: trad_rack.py — remove trapq/FFI calls**

Remove `ffi_lib.trapq_alloc()`, `ffi_lib.trapq_append`, `ffi_lib.trapq_finalize_moves`, `ffi_lib.cartesian_stepper_alloc`, `ffi_lib.extruder_stepper_alloc` calls and the chelper import.

- [ ] **Step 7: mixing_extruder.py — remove set_trapq calls**

The `set_trapq` calls are no-ops. Remove for cleanliness.

- [ ] **Step 8: Commit**

```bash
git add klippy/extras/force_move.py klippy/kinematics/extruder.py \
       klippy/kinematics/idex_modes.py klippy/extras/manual_stepper.py \
       klippy/extras/z_tilt.py klippy/extras/z_tilt_ng.py \
       klippy/extras/trad_rack.py klippy/extras/mixing_extruder.py
git commit -m "extras: remove dead chelper/trapq references across helper modules"
```

---

### Task 6: Clean up mcu.py dead stepcompress infrastructure

**Files:**
- Modify: `klippy/mcu.py`

- [ ] **Step 1: Remove _stepqueues and _steppersync**

- Remove `self._stepqueues = []` from `MCU.__init__`
- Remove `self._steppersync = None` (all occurrences)
- Remove `register_stepqueue` method
- Remove steppersync comments in `_connect` and `_firmware_restart`

- [ ] **Step 2: Simplify flush_moves**

`flush_moves` is already a no-op. Remove the commented-out steppersync logic, keeping the method as a no-op (it's called by `ToolHead._advance_flush_time`).

- [ ] **Step 3: Commit**

```bash
git add klippy/mcu.py
git commit -m "mcu: remove dead _stepqueues/_steppersync infrastructure"
```

---

### Task 7: Verify build and imports

- [ ] **Step 1: Verify c_helper.so builds**

```bash
rm -f klippy/chelper/c_helper.so
python3 -c "from klippy.chelper import get_ffi; get_ffi(); print('OK')"
```

- [ ] **Step 2: Verify Python imports**

```bash
python3 -c "
from klippy import stepper, mcu, serialhdl, motion_toolhead, toolhead
from klippy.kinematics import extruder, idex_modes
from klippy.extras import input_shaper, motion_report, force_move, manual_stepper
from klippy.extras import z_tilt, z_tilt_ng, mixing_extruder
print('all imports ok')
"
```

- [ ] **Step 3: Run any existing tests**

```bash
python3 -m pytest test/ -x -q 2>&1 | tail -20
```

- [ ] **Step 4: Final commit if any fixups needed**
