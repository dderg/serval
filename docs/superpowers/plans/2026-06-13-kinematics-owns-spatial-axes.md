# Kinematics Owns Spatial Axes (delete hardcoded x/y/z in the host kinematics layer) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the `[kinematics]` bindings the single source of truth for which axes are spatial (and therefore which are followers), deleting `motion.py`'s hardcoded `_SPATIAL_AXIS_NAMES`, and make `motion_kinematics.py`'s corexy coupling lane-indexed instead of keyed on the literal strings `"x"/"y"/"z"`.

**Architecture:** Spec `docs/superpowers/specs/2026-06-13-kinematics-owns-spatial-axes-design.md`. The kinematics module already binds axis roles explicitly (`[kinematics] axis_x: <name>`) and the Rust engine is already name-agnostic (`VectorNurbs<f64,3>` + index-keyed followers). Only the klippy host duplicated the spatial names. This change removes that duplication; it is **behavior-preserving** (every shipped config binds x/y/z, so output is bit-identical). The G-code surface (`homing.py` `G28`, `gcode.py` `Coord`) deliberately keeps X/Y/Z — that is the G-code coordinate standard and is out of scope.

**Tech Stack:** klippy Python (`klippy/motion.py`, `klippy/motion_kinematics.py`); tests via the in-repo `test/test_motion_kinematics.py` `FakeConfig` harness, run in the `dangerklippers/klipper-build` docker image; full gate `./scripts/ci.sh py` + `./scripts/ci.sh ruff`; e2e via the kalico-sim skill.

**Branch:** continue on `e-follows-xy` (the branch where the `[motor]`/`[kinematics]`/`[axis]` schema shipped).

**Out of scope:** arbitrary spatial axis *names* at the G-code level (spatial stays X/Y/Z); `homing.py` and `gcode.py` x/y/z usage; named/multi-kinematics ("motion channels", parked in `docs/rewrite/future-motion-channels-multi-kinematics.md`); `[extruder] axis:` (already shipped, commit `6fbcf7cf4`).

**Repo rules for every task:** unit tests live in a separate file from the tested code; no explanatory comments — name/extract instead; fail loudly (no silent fallbacks); commit after every task; no Claude/Anthropic commit trailers; `cargo fmt`/ruff clean before any PR push; `./scripts/ci.sh quick` + `./scripts/ci.sh py` green before opening/updating the PR.

---

## File structure

- `klippy/motion_kinematics.py` — **modify.** Add a module-level `read_claimed_axes(config)` (the spatial-axis source of truth, readable before the rails are built). Rewrite `_LinearKinematics.active_rails` to be lane-indexed. Responsibility unchanged: parse `[kinematics]`, build lanes, expose the kinematics interface.
- `klippy/motion.py` — **modify.** Delete `_SPATIAL_AXIS_NAMES` and its two uses; derive the spatial/claimed set from `motion_kinematics.read_claimed_axes(config)`.
- `test/test_motion_kinematics.py` — **modify (add tests).** New tests for `read_claimed_axes`. The existing `active_rails`/`claimed_axes` tests here and in `test/test_active_rails.py` are the regression guard for behavior-preservation.

Why no `homing.py`/`gcode.py`: their `XYZ`/`Coord` usages are the G-code coordinate standard (`G28 X`, `G1 X`), not the kinematics layer's source of truth. Spatial axes remain addressed as X/Y/Z at the G-code boundary by design.

---

## Task 1: `read_claimed_axes(config)` — the spatial-axis source of truth

The follower classifier in `motion.py` runs before `_load_kinematics`, so it cannot use `self.kin.claimed_axes()`. Add a lightweight reader that returns the role-bound axis names directly from the `[kinematics]` section, without building rails.

**Files:**
- Modify: `klippy/motion_kinematics.py` (add function after `KINEMATICS_TYPES`, before `load_kinematics`)
- Test: `test/test_motion_kinematics.py`

- [ ] **Step 1: Write the failing tests**

Add to `test/test_motion_kinematics.py` (the file already imports `pytest`, `motion_kinematics`, and defines `FakePrinter`, `FakeConfig`, `FakeError`, `corexy_sections`, `cartesian_sections`):

```python
def test_read_claimed_axes_returns_role_bound_names():
    printer = FakePrinter()
    assert motion_kinematics.read_claimed_axes(
        FakeConfig(printer, corexy_sections())
    ) == ["x", "y", "z"]
    assert motion_kinematics.read_claimed_axes(
        FakeConfig(printer, cartesian_sections())
    ) == ["x", "y", "z"]


def test_read_claimed_axes_unknown_type_rejected():
    sections = corexy_sections()
    sections["kinematics"]["type"] = "bogus"
    with pytest.raises(FakeError):
        motion_kinematics.read_claimed_axes(
            FakeConfig(FakePrinter(), sections)
        )


def test_read_claimed_axes_missing_section_rejected():
    sections = corexy_sections()
    del sections["kinematics"]
    with pytest.raises(FakeError):
        motion_kinematics.read_claimed_axes(
            FakeConfig(FakePrinter(), sections)
        )
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_motion_kinematics.py -k read_claimed_axes -q
```
Expected: FAIL — `AttributeError: module 'klippy.motion_kinematics' has no attribute 'read_claimed_axes'`.

- [ ] **Step 3: Implement `read_claimed_axes`**

In `klippy/motion_kinematics.py`, insert immediately after the `KINEMATICS_TYPES = { ... }` block (after current line 36) and before `def load_kinematics`:

```python
def read_claimed_axes(config):
    if not config.has_section("kinematics"):
        raise config.error("[kinematics] section is required")
    section = config.getsection("kinematics")
    kind = section.get("type")
    if kind not in KINEMATICS_TYPES:
        raise config.error(
            "[kinematics] type '%s' is not supported (supported: %s)"
            % (kind, ", ".join(sorted(KINEMATICS_TYPES)))
        )
    return [
        section.get(axis_role_key)
        for _role_motors_key, axis_role_key, _lane_idx in KINEMATICS_TYPES[kind]
    ]
```

- [ ] **Step 4: Run the tests to verify they pass**

Run:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_motion_kinematics.py -k read_claimed_axes -q
```
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_kinematics.py test/test_motion_kinematics.py
git commit -m "feat(klippy): read_claimed_axes — spatial-axis source of truth from [kinematics] bindings"
```

---

## Task 2: Delete `_SPATIAL_AXIS_NAMES`; derive spatial/follower from kinematics

`motion.py` holds a second, independent source of truth for "what is spatial." Remove it: the required-axis check is already enforced by `_LinearKinematics._read_lanes` (it raises if a role-bound axis has no `[axis <name>]` section), and follower classification becomes "declared but not claimed by kinematics."

**Files:**
- Modify: `klippy/motion.py` — delete the constant (line 14), the `_read_axes` spatial loop, and switch `_build_follower_steppers` to the claimed set.
- Regression guard: `test/test_motion_kinematics.py::test_role_binding_to_undeclared_axis_rejected` (already covers "claimed axis must be declared", via `_read_lanes`), plus the kalico-sim follower boot in Task 4.

- [ ] **Step 1: Establish the green baseline**

Run the existing tests that guard this behavior:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_motion_kinematics.py test/test_motion_topology.py test/test_extruder_split.py -q
```
Expected: PASS (note the count; it must not drop after the edits).

- [ ] **Step 2: Delete the constant**

In `klippy/motion.py`, delete line 14:
```python
_SPATIAL_AXIS_NAMES = ("x", "y", "z")
```

- [ ] **Step 3: Delete the hardcoded spatial-requirement loop in `_read_axes`**

In `klippy/motion.py` `_read_axes`, replace:
```python
        declared = {name for name, _, _, _ in self.axis_sections}
        for required in _SPATIAL_AXIS_NAMES:
            if required not in declared:
                raise config.error(
                    "[axis %s] section is required (every axis must be "
                    "declared)" % required
                )
        for _, axes, _, _, _ in self.limit_sections:
```
with:
```python
        declared = {name for name, _, _, _ in self.axis_sections}
        for _, axes, _, _, _ in self.limit_sections:
```
(The "every claimed axis must be declared" rule now lives solely in `_LinearKinematics._read_lanes`, which raises `"[kinematics] axis_x binds to axis 'x' but no [axis x] section exists"`. `declared` is retained for the limit-section coverage check below it.)

- [ ] **Step 4: Switch `_build_follower_steppers` to the claimed set**

In `klippy/motion.py` `_build_follower_steppers`, replace:
```python
    def _build_follower_steppers(self, config):
        self.follower_steppers = []
        for name, _follows, motors, _pp in self.axis_sections:
            if name in _SPATIAL_AXIS_NAMES or not motors:
                continue
```
with:
```python
    def _build_follower_steppers(self, config):
        self.follower_steppers = []
        claimed = set(motion_kinematics.read_claimed_axes(config))
        for name, _follows, motors, _pp in self.axis_sections:
            if name in claimed or not motors:
                continue
```

- [ ] **Step 5: Confirm no other references to the deleted constant**

Run:
```bash
rg -n "_SPATIAL_AXIS_NAMES" klippy/ test/
```
Expected: no output.

- [ ] **Step 6: Run the regression tests — must still pass**

Run:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_motion_kinematics.py test/test_motion_topology.py test/test_extruder_split.py -q
```
Expected: PASS with the same count as Step 1. In particular `test_role_binding_to_undeclared_axis_rejected` must still pass (the rejection now comes from `_read_lanes`).

- [ ] **Step 7: Commit**

```bash
git add klippy/motion.py
git commit -m "refactor(klippy): spatial/follower set comes from [kinematics] bindings, not hardcoded x/y/z"
```

---

## Task 3: Lane-index `active_rails` (drop literal "x"/"y"/"z" keys)

`active_rails` is the last name-semantics use of `"x"/"y"/"z"` in the kinematics module (the corexy coupling). Rewrite it to key on lane index. `test/test_active_rails.py` and the `test_active_rails_*` tests in `test/test_motion_kinematics.py` pin the exact behavior and are the regression guard. (The remaining `"xyz"[lane_idx]` in `setup_itersolve` is the C itersolve's lane→char label, and the `enumerate("xyz")`/`Coord(...)` in `get_status` are the reported `homed_axes`/status surface — both are G-code-convention labels and stay.)

**Files:**
- Modify: `klippy/motion_kinematics.py` `_LinearKinematics.active_rails`
- Regression guard: `test/test_active_rails.py`, `test/test_motion_kinematics.py` (`test_active_rails_couples_xy_for_corexy`, `test_active_rails_independent_for_cartesian`)

- [ ] **Step 1: Establish the green baseline**

Run:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_active_rails.py test/test_motion_kinematics.py -k active_rails -q
```
Expected: PASS (note the count).

- [ ] **Step 2: Rewrite `active_rails` lane-indexed**

In `klippy/motion_kinematics.py`, replace:
```python
    def active_rails(self, dx, dy, dz):
        moved = {
            axis: abs(delta) > 1e-9 for axis, delta in zip("xyz", (dx, dy, dz))
        }
        coupled = dict(moved)
        if self.coupled_xy():
            coupled["x"] = coupled["y"] = moved["x"] or moved["y"]
        active = []
        for lane_idx, _, _ in self._lanes:
            if coupled["xyz"[lane_idx]]:
                active.append(self.rails[lane_idx])
        return active
```
with:
```python
    def active_rails(self, dx, dy, dz):
        moved = [abs(dx) > 1e-9, abs(dy) > 1e-9, abs(dz) > 1e-9]
        if self.coupled_xy():
            moved[0] = moved[1] = moved[0] or moved[1]
        return [
            self.rails[lane_idx]
            for lane_idx, _, _ in self._lanes
            if moved[lane_idx]
        ]
```

- [ ] **Step 3: Run the regression tests — must still pass**

Run:
```bash
docker run --rm -v "$PWD:/klipper" dangerklippers/klipper-build --python 3.13 \
  py.test test/test_active_rails.py test/test_motion_kinematics.py -k active_rails -q
```
Expected: PASS with the same count as Step 1.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_kinematics.py
git commit -m "refactor(klippy): active_rails keys on lane index, not literal x/y/z"
```

---

## Task 4: Full gates + kalico-sim regression

Behavior-preservation is proven by the whole Python suite plus live cartesian + corexy boots.

**Files:** none (verification only).

- [ ] **Step 1: ruff**

Run:
```bash
./scripts/ci.sh ruff
```
Expected: `All checks passed!` (and formatting clean).

- [ ] **Step 2: Full Python suite**

Run:
```bash
./scripts/ci.sh py
```
Expected: PASS — no failures; the passed count is ≥ the pre-change baseline (275+ passed, with the 3 new `read_claimed_axes` tests added).

- [ ] **Step 3: kalico-sim cartesian self-test (rebuild from HEAD)**

Run:
```bash
bash tools/kalico-sim/run.sh 2>&1 | tail -5
```
Expected: `SIMULATION RESULT ... Status: PASS` (cartesian boot — followers + spatial axes resolve).

- [ ] **Step 4: kalico-sim corexy phase-stepping test**

Run:
```bash
docker run --rm kalico-sim --phase-test --timeout 120 2>&1 \
  | grep -E "Status:|configure_axes|FAIL|Traceback" | tail -6
```
Expected: `Status: PASS`, and `configure_axes ... runtime_bindings=[(0,'x',...),(1,'y',...),(2,'z',...)]` — corexy coupling/phase-handover still forms (lanes 0/1 coupled).

- [ ] **Step 5: Commit (only if any verification fixups were needed)**

If Steps 1–4 required no code changes, there is nothing to commit. Otherwise:
```bash
git add -A
git commit -m "test(klippy): gate fixups for kinematics-owned spatial axes"
```

---

## Self-review (performed during plan authoring)

- **Spec coverage:** spec change (1) "delete `_SPATIAL_AXIS_NAMES`, claimed set is source of truth" → Task 2; change (2) "lane-index corexy coupling" → Task 3; helper to make (1) possible pre-`_load_kinematics` → Task 1; "G-code surface keeps X/Y/Z" → honored by explicitly leaving `homing.py`/`gcode.py` and the `setup_itersolve`/`get_status` label sites untouched (Task 3 note); behavior-preservation → Task 4. No spec requirement is unaddressed.
- **Placeholder scan:** none — every code/test step shows full code and exact commands with expected output.
- **Type/name consistency:** `read_claimed_axes(config)` is defined in Task 1 and used with that exact signature in Task 2; `active_rails(self, dx, dy, dz)` keeps its signature in Task 3 (callers: `homing.py` `kin.active_rails(*homing_deltas)`, unchanged); `_lanes` entries are `(lane_idx, axis_name, motor_names)` as used.
- **Ordering correctness:** `_build_follower_steppers` (init line 128) runs before `_load_kinematics` (129); `read_claimed_axes` reads `[kinematics]` directly, so it works at that point. A claimed axis missing its `[axis]` section still fails loudly — now in `_read_lanes` (line 129) instead of the deleted loop (line ~465), one step later, with a clearer message.
