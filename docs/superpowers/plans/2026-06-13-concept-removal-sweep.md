# Concept Removal Sweep & Declared Kinematic Map Implementation Plan (Plan 5)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-developme
nt (recommended) or superpowers:executing-plans to implement this plan task-by-task. Ste
ps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The toolhead concept dies: `MotionToolhead` becomes `Motion` registered as `"m
otion"` with a thin `ToolheadShim` keeping the published `"toolhead"` surface alive; kin
ematics becomes a declared `[kinematics]` config section binding axis roles to arbitrari
ly-named motor sections, implemented as constant-matrix host-side modules — role-encodin
g section names (`stepper_x`, `servo_x`, the stepper config inside `[extruder]`) are rej
ected at load with pointers to the replacement.

**Architecture:** Spec: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-desi
gn.md` §1 (axes/motors/kine phases. **Phase A** (behavior-p
reserving): rename the klippy seam (`motion_toolhead.py` → `motion.py`, class `Motion`,
registry key `"motion"`), i the `"toolhead"` key carrying t
he legacy method + status surface, and sweep the trivial toolhead/fossil names out of `r
ust/`. **Phase B**: `Kinemaaxis↔motor transforms) replaces
the hardcoded corexy branches in `motion-engine`'s `enqueue.rs`/`dispatch.rs`/`bridge.rs
`/`kinematics.rs`; klippy gmed motor sections + axis-domain
 homing/range keys on `[axis]`; `BridgeKinematics`, `_MOTOR_SLOT_PREFIXES`, and `Extrude
rStepper` die.

**Tech stack:** Rust (`motionly), pyo3 (`bridge.rs`), klipp
y Python (`printer.py`, `motion_toolhead.py` → `motion.py`, `stepper.py`, `kinematics/ex
truder.py`, new `motion_kinextest run` from `rust/` (never
bare `cargo test`); Python via `./scripts/ci.sh py`; e2e via the kalico-sim skill.

**PRECONDITION: Plan 4 (`docs/superpowers/plans/2026-06-12-per-axis-emission-chain.md`)
is fully landed.** Verify beline | head` shows plan 4's fin
al commit; `rg -n "post_processor" klippy/motion_toolhead.py` hits (plan 4 Task 7's sect
ion parsing); `rg -n "Extruy/` is empty; `cargo nextest run
` from `rust/` and `./scripts/ci.sh py` are green. Plan 4 was in flight while this plan
was written — **all code exl; anchor by symbol name and the
 given grep commands, never by line, and re-read every touched file before editing.** Wh
ere this plan shows `init_pwith plan 4 Task 6's final signa
ture (post-processor args replaced the shaper args).

**Branch/PR:** new worktree off `sota-motion` (after plan 4 merges); PR bases on `sota-m
otion`, never `main`.

**Out of scope (later plans/deferred per spec §6):** binding-constraint reporting (plan
6); motor-space `[limit]` rows (`motors:` key on limits — syntax reserved, nothing built
); velocity-dependent caps; delta/polar/IDEX kinematics modules (nonlinear path is a dec
lared enum variant that errors, not an implementation); `SET_PRESSURE_ADVANCE` / `[input
_shaper]` / `SYNC_EXTRUDER_MOTION` compat shims; migrating the ~116 in-repo `lookup_obje
ct("toolhead")` call sites off the shim (each retires opportunistically when its extra i
s next rewritten); dual_carriage support (stays rejected).

**Repo rules for every task:** unit tests in separate files from tested code; no explana
tory comments — name/extract instead; fail loudly (no silent fallbacks); commit after ev
ery task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR p
ush; `./scripts/ci.sh quick` green before opening/updating the PR, plus `./scripts/ci.sh
 py` because klippy changes throughout.

---

## Design decisions this plan makes (agreed with the user 2026-06-13)

1. **Bare motor sections, discovered by reference only.** A motor is a config section wh
ose name is arbitrary (`[motor_a]`, `[extruder_motor]`), found exclusively through refer
ences from `[kinematics]` role lists and `[axis].motors` keys — never by prefix scan. Th
is keeps every name-keyed convention working with zero churn: `[tmc5160 motor_a]`, `tmc5
160_motor_a:virtual_endstop`, `SET_STEPPER_ENABLE STEPPER=motor_a`, `force_move` registr
y keys. Orphaned motor sections fail loudly via klippy's existing unused-option validati
on. Each motor section declares its drive technology: `drive: stepper` (default) or `dri
ve: servo` (replaces the `[servo_x]` section family).

2. **Axis-domain keys live on `[axis]`.** `position_min`, `position_max`, `position_ends
top`, `endstop_pin`, `homing_speed`, `second_homing_speed`, `homing_retract_dist`, `homi
ng_retract_speed`, `homing_positive_dir` move from stepper sections to `[axis <name>]` (
spatial axes bound to kinematics roles only — a follower axis declaring them is a config
 error). Motor sections keep pure hardware: pins, microsteps, rotation_distance, gear_ra
tio, current/driver keys, `phase_stepping`, and optional per-motor `endstop_pin` for mul
ti-endstop leveling (z_tilt-style), which stays supported.

3. **Seam rename + thin shim.** `klippy/motion_toolhead.py` → `klippy/motion.py`, class
`Motion`, registered `"motion"`. A `ToolheadShim` class stays registered as `"toolhead"`
, delegating the legacy method surface and publishing today's exact status keys. Fossil
no-op methods (`register_lookahead_callback`, `note_step_generation_scan_time`, `get_tra
pq`, `note_mcu_movequeue_activity`, `limit_next_junction_speed`) live only on the shim.
The `toolhead:set_position` / `toolhead:manual_move` / `toolhead:sync_print_time` event
names are published compat surface like the status object — `Motion` keeps firing them u
nder those names; in-repo handlers (gcode_move, idle_timeout) stay untouched. The extrud
er bookkeeping (`get_extruder`/`set_extruder`/`self.extruder`, the `check_move` hook) st
ays on `Motion` for now — klippy-side only, retired with the shim, conscious carry-over.

4. **Kinematics types shipped: `cartesian` and `corexy`.** `hybrid_corexy` dies (it alre
ady skips `_configure_axes_per_mcu`, so it cannot drive motors in bridge mode). `[printe
r] kinematics:` joins the rejected-legacy list pointing at `[kinematics]`. Leftover `[st
epper_x]`/`[stepper_y]`/`[stepper_z]`/`[servo_x]`-family sections and stepper keys insid
e `[extruder]` are rejected with errors naming the replacement.

5. **Wire facts (verified 2026-06-13):** the `ConfigureAxes` protocol message (`rust/kal
ico-protocol/src/messages.rs`, `kinematics: u8` field) has **zero senders** — it is dead
 protocol surface; `runtime/src/engine.rs::configure_kinematics` is a vestigial stub. Th
e kinematics tag crosses only the Python↔Rust `init_planner` boundary (the `(handle, axe
s, tag)` topology tuples). The MCU receives pre-transformed motor-frame data (`runtime_s
eed_position`, per-lane cubic pieces); **zero MCU edits anywhere in this plan** — `src/`
, `rust/runtime` wire handlers, and `docs/rewrite/mcu-c-rust-boundary.md` are unt
ouched (the one `rust/runtime` edit is a doc-comment + enum-variant rename with discrimi
nants pinned by the existing const assert). `KinematicTag` discriminants (0=corexy, 1=ca
rtesian) stay frozen as the Python↔Rust contract.

6. **Kinematics module shape (spec §1's four items, linear-only build):** config schema
= klippy-side section parsing per type (role bindings `axis_x:`/`axis_y:`/`axis_z:` + pe
r-role motor lists); inverse transform (axes→motors) and forward transform (motors→axes)
 = constant 3×3 matrices on the Rust host consumed by emission, seeding, and homing reco
very; linearity declaration = `KinematicsKind` is matrix-only today, nonlinear is a futu
re variant that must bring the sample-and-refit path (spec §1 item 4), not built now. Id
entity lanes short-circuit in `enqueue.rs` so cartesian output stays bit-identical.

7. **Coverage rule lands in Rust config:** every declared axis is motor-mapped exactly o
nce — claimed by a kinematics role XOR carrying its own `[axis].motors` key. Validated i
n `motion-engine/src/config.rs` (the planner-side single source of truth, same home as p
lan 2's registry rules), fed through `init_planner`; klippy performs the section-level c
hecks it alone can see (sections exist, drive values valid, no homing keys on followers)
.

8. **`ExtruderStepper` dies.** `[extruder]` keeps heater + filament-geometry keys and `c
heck_move` validation; its stepper config moves to a referenced motor section. `SET_PRES
SURE_ADVANCE`, `SYNC_EXTRUDER_MOTION`, `SET_EXTRUDER_ROTATION_DISTANCE` disappear with i
t (deferred compat shims per spec §6 — plan 4's `SET_POST_PROCESSOR` is the live tuning
path). `[extruder_stepper]` extras are rejected at load.

---

# Phase A — seam rename and shim (behavior-preserving)

## Task 1: status-surface snapshot test (the Phase A gate, written first)

**Files:**
- Create: `test/test_toolhead_shim.py`

This test pins the published surface **before** any rename so Tasks 2–4 cannot drift it.
 Match the instantiation idiom of existing tests (`test/test_rail.py`, `test/test_active
_rails.py`, `test/conftest.py` fixtures) — they construct printer objects against fake c
onfigs; reuse their helpers.

- [ ] **Step 1: Write the test (passing against current code):**

```python
import pytest

EXPECTED_STATUS_KEYS = {
    "homed_axes",
    "axis_minimum",
    "axis_maximum",
    "print_time",
    "stalls",
    "estimated_print_time",
    "extruder",
    "position",
    "max_velocity",
    "max_accel",
    "minimum_cruise_ratio",
    "square_corner_velocity",
}

LEGACY_METHODS = [
    "move", "manual_move", "dwell", "wait_moves", "wait_moves_and_mcu",
    "get_last_move_time", "get_position", "set_position",
    "flush_step_generation", "get_status", "check_busy", "stats",
    "get_kinematics", "get_max_velocity", "get_extruder", "set_extruder",
    "register_step_generator", "register_lookahead_callback",
    "note_step_generation_scan_time", "note_mcu_movequeue_activity",
    "limit_next_junction_speed", "get_trapq",
]


def test_toolhead_status_keys_exact(toolhead_fixture):
    toolhead = toolhead_fixture.printer.lookup_object("toolhead")
    status = toolhead.get_status(toolhead_fixture.eventtime)
    assert set(status.keys()) == EXPECTED_STATUS_KEYS


def test_toolhead_method_surface_complete(toolhead_fixture):
    toolhead = toolhead_fixture.printer.lookup_object("toolhead")
    missing = [m for m in LEGACY_METHODS if not callable(getattr(toolhead, m, None))]
    assert missing == []
```

`toolhead_fixture` is whatever existing fixture boots a printer object far enough to loo
k up `"toolhead"` — find it with `rg -n "lookup_object" test/conftest.py test/test_activ
e_rails.py` and reuse; if none boots that far, build the minimal fake config the way `te
st_rail.py` does. The exact status keys above were read from `MotionToolhead.get_status`
 + `BridgeKinematics.get_status` — re-verify against the post-plan-4 file before committ
ing (plan 4 does not touch get_status, but verify).

- [ ] **Step 2: Run:** `./scripts/ci.sh py` → PASS (test is green against unrenamed code
 — that is the point).
- [ ] **Step 3: Commit** — `test(klippy): pin published toolhead status keys and method
surface`

---

## Task 2: Rust fossil renames

**Files:**
- Modify: `rust/runtime/src/segment.rs` (enum variant names + stale doc comment)
- Modify: every `KinematicTag::` reference — find: `rg -ln "CoreXyAndE|CartesianXyzAndE"
 rust/`
- Modify: the toolhead-named test items — find: `rg -ni "toolhead" rust/` (expect ~6 hit
s: `peak_toolhead_accel`, `corexy_inverse_maps_motor_to_toolhead`, `corexy_inverse_recov
ers_toolhead`, a comment in a `bezier_root.rs`, plus strays)

- [ ] **Step 1: Rename enum variants, fix the lying comment.** `KinematicTag::CoreXyAndE
` → `KinematicTag::CoreXy`, `CartesianXyzAndE` → `Cartesian`. Discriminants stay 0/1 — t
he existing `const _: () = assert!(...)` in `dispatch.rs` keeps them pinned. Replace the
 doc comment claiming the discriminant is "embedded in the MCU wire protocol" (false — `
ConfigureAxes` has no senders) with the truth:

```rust
/// Discriminants are the kinematics tag crossing the Python↔Rust
/// `init_planner` topology tuples; klippy mirrors them numerically.
/// Renumbering breaks that contract — dispatch.rs pins them with a
/// compile-time assert.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    CoreXy = 0,
    Cartesian = 1,
}
```

- [ ] **Step 2: Rename the toolhead-named tests/comments.** `peak_toolhead_accel` → `pea
k_carriage_accel` (or match the local naming of the file it lives in), `corexy_inverse_m
aps_motor_to_toolhead` → `corexy_inverse_maps_motor_to_axes`, `corexy_inverse_recovers_t
oolhead` → `corexy_inverse_recovers_axes`; reword comments to say "axes"/"carriage". Aft
er: `rg -ni "toolhead" rust/` → empty.
- [ ] **Step 3: Run** — `cargo nextest run` from `rust/` → PASS; `cargo fmt --all --chec
k` clean.
- [ ] **Step 4: Commit** — `refactor(rust): toolhead naming dies; KinematicTag variants
lose AndE; wire-comment corrected`

---

## Task 3: rename the seam — `motion.py`, class `Motion`, registry key `"motion"`

Pure rename; the `"toolhead"` registry key still points at the same (renamed) object in
this task — the shim split is Task 4.

**Files:**
- Rename: `klippy/motion_toolhead.py` → `klippy/motion.py` (`git mv`)
- Modify: `klippy/printer.py` (the import list ~line 25–35 and the `for m in [motion_too
lhead]` loop ~line 357)
- Modify: every other reference — find: `rg -ln "motion_toolhead|MotionToolhead" klippy/
 tools/ test/ scripts/ docs/` and fix each (expect: `printer.py`, possibly `tools/sim_kl
ippy/`, test files, log strings inside the module itself)

- [ ] **Step 1: `git mv klippy/motion_toolhead.py klippy/motion.py`**, rename class `Mot
ionToolhead` → `Motion`, update the module's own log prefixes (`"MotionToolhead:"` → `"M
otion:"`).
- [ ] **Step 2: Register under both keys** in `add_printer_objects` (temporary until Tas
k 4 splits them):

```python
def add_printer_objects(config):
    motion = Motion(config)
    printer = config.get_printer()
    printer.add_object("motion", motion)
    printer.add_object("toolhead", motion)
    extruder.add_printer_objects(config)
```

- [ ] **Step 3: Fix every reference** found by the grep in the Files list. In `printer.p
y`: `motion_toolhead` → `motion` in the import tuple and the `add_printer_objects` loop.
 Event names (`toolhead:*`) are NOT renamed (decision 3).
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS including Task 1's snapshot test; `r
g -n "motion_toolhead|MotionToolhead" .` (excluding `docs/superpowers/`, git history) →
empty.
- [ ] **Step 5: Commit** — `refactor(klippy): motion_toolhead becomes motion.Motion, reg
istered as "motion" (+"toolhead" alias)`

---

## Task 4: `ToolheadShim` — the `"toolhead"` key becomes a thin delegate

**Files:**
- Modify: `klippy/motion.py` (add `ToolheadShim`, strip fossil methods from `Motion`)
- Create: `test/test_toolhead_shim_delegation.py`

- [ ] **Step 1: Write failing tests:**

```python
def test_toolhead_is_shim_motion_is_real(toolhead_fixture):
    printer = toolhead_fixture.printer
    shim = printer.lookup_object("toolhead")
    motion = printer.lookup_object("motion")
    assert shim is not motion
    assert shim.motion is motion


def test_fossil_methods_only_on_shim(toolhead_fixture):
    printer = toolhead_fixture.printer
    motion = printer.lookup_object("motion")
    shim = printer.lookup_object("toolhead")
    for fossil in (
        "register_lookahead_callback",
        "note_step_generation_scan_time",
        "get_trapq",
        "note_mcu_movequeue_activity",
        "limit_next_junction_speed",
    ):
        assert not hasattr(motion, fossil)
        assert callable(getattr(shim, fossil))


def test_shim_delegates_state(toolhead_fixture):
    printer = toolhead_fixture.printer
    shim = printer.lookup_object("toolhead")
    motion = printer.lookup_object("motion")
    assert shim.get_position() == motion.get_position()
    assert shim.get_status(toolhead_fixture.eventtime) == motion.get_status(
        toolhead_fixture.eventtime
    )
```

- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL (`shim is motion`,
 fossils still on Motion).
- [ ] **Step 3: Implement.** In `motion.py`:

```python
class ToolheadShim:
    def __init__(self, motion):
        self.motion = motion

    def register_lookahead_callback(self, callback):
        callback(self.motion.get_last_move_time())

    def note_step_generation_scan_time(self, delay, old_delay=0.0):
        self.motion.flush_step_generation()

    def get_trapq(self):
        return None

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        pass

    def limit_next_junction_speed(self, speed):
        pass

    def __getattr__(self, name):
        return getattr(self.motion, name)
```

`__getattr__` delegation keeps the shim honest-by-construction: every real method (`move
`, `dwell`, `wait_moves`, `get_status`, `get_kinematics`, `get_extruder`, attribute read
s like `Coord`/`max_velocity`) resolves on `Motion`; only fossils are shim-local. Delete
 the five fossil methods from `Motion` (also delete `self.trapq = None` and `self.step_g
enerators` if `register_step_generator` turns out fossil — check first: `rg -n "register
_step_generator|step_generators" klippy/` — `kinematics/extruder.py` calls it, so it sta
ys on `Motion` until Task 12 revisits). `add_printer_objects` becomes:

```python
def add_printer_objects(config):
    motion = Motion(config)
    printer = config.get_printer()
    printer.add_object("motion", motion)
    printer.add_object("toolhead", ToolheadShim(motion))
    extruder.add_printer_objects(config)
```

- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS including Task 1's snapshot (the shi
m publishes the identical surface).
- [ ] **Step 5: Commit** — `feat(klippy): ToolheadShim carries the published toolhead su
rface; Motion sheds fossil methods`

---

## Task 5: Phase A gate

- [ ] **Step 1:** `./scripts/ci.sh quick` → green; `./scripts/ci.sh py` → green.
- [ ] **Step 2:** kalico-sim boot check (see the `kalico-sim` skill): bring the simulate
d printer up on this branch, confirm clean connect, `G28`-readiness state via status que
ry, and that Moonraker-style status (`printer.toolhead.*`) reads identically to a pre-br
anch capture. No motion commands beyond what the sim harness itself runs.
- [ ] **Step 3: Commit** any stragglers — `chore: phase A gate green`

---

# Phase B — the declared kinematic map

## Task 6: Rust `KinematicsModule` — constant-matrix transforms

**Files:**
- Modify: `rust/motion-engine/src/kinematics.rs` (rewrite)
- Modify: `rust/motion-engine/src/kinematics/tests.rs`
- Modify: `rust/motion-engine/src/dispatch.rs` (`SPATIAL_AXES`, loud tag validation, `mo
tor_frame_xy` via module)

- [ ] **Step 1: Write failing tests** in `kinematics/tests.rs` (keep the existing corexy
 value tests, add):

```rust
use super::*;

#[test]
fn from_tag_zero_is_corexy_one_is_cartesian() {
    assert_eq!(KinematicsModule::from_tag(0).unwrap().kind(), KinematicsKind::CoreXy);
    assert_eq!(KinematicsModule::from_tag(1).unwrap().kind(), KinematicsKind::Cartesian)
;
}

#[test]
fn from_tag_unknown_is_loud() {
    assert!(KinematicsModule::from_tag(7).is_err());
}

#[test]
fn corexy_forward_matches_legacy_values() {
    let m = KinematicsModule::from_tag(0).unwrap();
    assert_eq!(m.forward([150.0, 150.0, 50.0]), [300.0, 0.0, 50.0]);
    assert_eq!(m.forward([10.0, 4.0, 0.0]), [14.0, 6.0, 0.0]);
}

#[test]
fn corexy_roundtrip_is_identity() {
    let m = KinematicsModule::from_tag(0).unwrap();
    let axes = [12.5, -3.25, 7.0];
    let back = m.inverse(m.forward(axes));
    for (a, b) in axes.iter().zip(back.iter()) {
        assert!((a - b).abs() < 1e-12);
    }
}

#[test]
fn cartesian_is_identity_lanes() {
    let m = KinematicsModule::from_tag(1).unwrap();
    assert!(m.lane_is_identity(0) && m.lane_is_identity(1) && m.lane_is_identity(2));
    assert_eq!(m.forward([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}

#[test]
fn corexy_lane_weights_are_sum_and_difference() {
    let m = KinematicsModule::from_tag(0).unwrap();
    assert_eq!(m.lane_weights(0), [1.0, 1.0, 0.0]);
    assert_eq!(m.lane_weights(1), [1.0, -1.0, 0.0]);
    assert_eq!(m.lane_weights(2), [0.0, 0.0, 1.0]);
    assert!(!m.lane_is_identity(0));
    assert!(m.lane_is_identity(2));
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(k
inematics)'` → FAIL.
- [ ] **Step 3: Implement** `kinematics.rs`:

```rust
pub const SPATIAL_AXES: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsKind {
    CoreXy,
    Cartesian,
}

#[derive(Debug, Clone, Copy)]
pub struct KinematicsModule {
    kind: KinematicsKind,
    axis_to_motor: [[f64; SPATIAL_AXES]; SPATIAL_AXES],
    motor_to_axis: [[f64; SPATIAL_AXES]; SPATIAL_AXES],
}

#[derive(Debug, thiserror::Error)]
#[error("unknown kinematics tag {0}; known: 0=corexy, 1=cartesian")]
pub struct UnknownKinematicsTag(pub u8);

impl KinematicsModule {
    pub fn from_tag(tag: u8) -> Result<Self, UnknownKinematicsTag> { /* match 0 | 1 */ }
    pub fn kind(&self) -> KinematicsKind { /* ... */ }
    pub fn tag(&self) -> u8 { /* KinematicTag discriminant */ }
    pub fn lane_weights(&self, lane: usize) -> [f64; SPATIAL_AXES] { /* row of axis_to_m
otor */ }
    pub fn lane_is_identity(&self, lane: usize) -> bool { /* row == unit vector e_lane *
/ }
    pub fn forward(&self, axes: [f64; SPATIAL_AXES]) -> [f64; SPATIAL_AXES] { /* axis_to
_motor · axes */ }
    pub fn inverse(&self, motors: [f64; SPATIAL_AXES]) -> [f64; SPATIAL_AXES] { /* motor
_to_axis · motors */ }
}
```

CoreXy matrices: `axis_to_motor = [[1,1,0],[1,-1,0],[0,0,1]]`, `motor_to_axis = [[0.5,0.
5,0],[0.5,-0.5,0],[0,0,1]]`. Cartesian: identity both ways. Delete the free functions `f
orward_corexy`/`inverse_corexy`/`forward(tag,..)`/`inverse(tag,..)` once Task 7 retarget
s their callers (this task may keep them as thin wrappers over the module to stay green
mid-task; they must be gone by Task 7 Step 4's grep). In `dispatch.rs`: add to `build_mc
u_configs` a loud validation — `KinematicsModule::from_tag(*tag)` must succeed and a cor
exy-tagged MCU must list both `AXIS_X` and `AXIS_Y` (today's `cfg_is_corexy` silently tr
eats that mismatch as cartesian — that silent fallback dies):

```rust
pub fn build_mcu_configs<S: ::std::hash::BuildHasher>(
    mcus: &[(u32, Vec<u8>, u8)],
    caps_by_handle: &HashMap<u32, McuCaps, S>,
) -> Result<Vec<McuAxisConfig>, KinematicsConfigError> { /* validate per-MCU, then map a
s today */ }
```

with `KinematicsConfigError` naming the handle and the problem. Chase the (few) `build_m
cu_configs` callers — `rg -n "build_mcu_configs" rust/` — and propagate the error to a `
PyErr` in `init_planner`.

- [ ] **Step 4: Run** — `cargo nextest run -p motion-engine` → PASS.
- [ ] **Step 5: Commit** — `feat(motion-engine): KinematicsModule constant-matrix transf
orms; loud tag validation`

---

## Task 7: generalize emission, seeding, and homing recovery onto the module

**Files:**
- Modify: `rust/motion-engine/src/enqueue.rs` (motor-lane combine via `lane_weights`)
- Modify: `rust/motion-engine/src/enqueue/tests.rs`
- Modify: `rust/motion-engine/src/dispatch.rs` (`motor_frame_xy` → `motor_frame`, `cfg_i
s_corexy` dies)
- Modify: `rust/motion-engine/src/bridge.rs` (homing inverse + `trip_position_to_motor_f
rame` + `< AXIS_E` filters; grep anchors below)

- [ ] **Step 1: Write failing tests** in `enqueue/tests.rs`:

```rust
#[test]
fn cartesian_lanes_are_bitwise_passthrough() {
    // identity module: the enqueued lane curve's knots and control points are
    // bit-identical (==) to the ShapedSegment axis curve, not a refit copy.
}

#[test]
fn corexy_lane_combine_matches_legacy_sum_difference() {
    // lane 0 == add_with_knot_union(x, y); lane 1 == add_with_knot_union(x, neg y),
    // compared bitwise against curves built with the same nurbs calls the old
    // code used — pins that the generalized weighted fold takes the identical
    // numeric path for weights ±1.
}

#[test]
fn follower_lanes_never_pass_through_the_spatial_matrix() {
    // a 4-axis ShapedSegment on a corexy MCU: lane 3 (follower) is the raw
    // axis-3 curve untouched.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(e
nqueue)'` → FAIL (new assertions against the not-yet-generalized code where they don't a
lready hold; the legacy-equivalence test may pass trivially before the rewrite — that's
fine, it's the pin).
- [ ] **Step 3: Implement.**
  - `enqueue.rs` (anchor: `rg -n "cfg_is_corexy|AXIS_X < seg.axes.len()" rust/motion-bri
dge/src/enqueue.rs`): replace the corexy branch with a per-lane weighted fold for spatia
l lanes:

```rust
fn lane_curve(module: &KinematicsModule, seg_axes: &[ScalarNurbs<f64>], lane: usize)
    -> ScalarNurbs<f64>
{
    if lane >= SPATIAL_AXES || module.lane_is_identity(lane) {
        return seg_axes[lane].clone();
    }
    let w = module.lane_weights(lane);
    let mut acc: Option<ScalarNurbs<f64>> = None;
    for (axis, &weight) in w.iter().enumerate() {
        if weight == 0.0 {
            continue;
        }
        let term = scale_curve_exact(&seg_axes[axis], weight);
        acc = Some(match acc {
            None => term,
            Some(prev) => add_with_knot_union(&prev, &term)
                .unwrap_or_else(|e| panic!("lane combine knot-union failed (invariant vi
olation — all ShapedSegment axes share one time domain): {e:?}")),
        });
    }
    acc.expect("kinematics lane with all-zero weights is a module construction bug")
}
```

  `scale_curve_exact` returns a clone for `weight == 1.0` and the existing negate path f
or `-1.0` (match whatever the current code calls — `rg -n "neg|negate|scalar_multiply" r
ust/motion-engine/src/enqueue.rs rust/nurbs/src/algebra*` — so ±1 weights reproduce toda
y's corexy bit-exactly), general `scalar_multiply` otherwise.
  - `dispatch.rs`: `motor_frame_xy(cfg, x, y)` → `motor_frame(cfg, [x, y, z]) -> [f64; 3
]` calling `module.forward`; `cfg_is_corexy` deleted; seed builders pass z through the m
atrix (identity for both shipped modules — behavior unchanged, shape general). `AXIS_E`
filters in `bridge.rs` (`rg -n "AXIS_E" rust/motion-engine/src/`) become `SPATIAL_AXES`
comparisons where they mean "spatial only"; `AXIS_E` the constant dies — follower indice
s come from the registry (`rg -n "AXIS_E" rust/` → empty after, tests use literal `3` wi
th a name like `FOLLOWER_E` local to the test file if wanted).
  - `bridge.rs` homing (anchors: `rg -n "trip_position_to_motor_frame|kinematics::invers
e" rust/motion-engine/src/bridge.rs`): `trip_position_to_motor_frame` returns `[f64; SPA
TIAL_AXES]` and asserts `axis < SPATIAL_AXES` loudly (a follower axis in a homing trip i
s a bug, not a case); both recovery sites call `KinematicsModule::from_tag(tag)?.inverse
(frame)`.
- [ ] **Step 4: Run** — `cargo nextest run` (workspace) → PASS; `rg -n "forward_corexy|i
nverse_corexy|cfg_is_corexy|AXIS_E" rust/` → empty.
- [ ] **Step 5: Commit** — `refactor(motion-engine): emission/seeding/homing run on Kine
maticsModule; corexy special-case dies`

---

## Task 8: Rust config — motor-mapped-exactly-once coverage

**Files:**
- Modify: `rust/motion-engine/src/config.rs` (+ its tests file — anchor: `rg -n "mod tes
ts|#\[cfg\(test\)\]" rust/motion-engine/src/config.rs` and the existing `config/tests.rs
` if split)
- Modify: `rust/motion-engine/src/bridge.rs` (`init_planner` gains `kinematics_axes: Vec
<String>`)
- Modify: `klippy/motion_engine.py` (wrapper signature)

- [ ] **Step 1: Write failing tests** (next to plan 2's `AxisRegistry` tests):

```rust
#[test]
fn axis_claimed_by_kinematics_and_motors_key_is_rejected() {
    // [axis z] motors: m1  +  kinematics claims z  → MotorMappingDuplicate("z")
}

#[test]
fn axis_with_neither_claim_nor_motors_is_rejected() {
    // [axis e] with empty motors, not claimed → MotorMappingMissing("e")
}

#[test]
fn kinematics_claim_of_undeclared_axis_is_rejected() {
    // claim lists "w", no [axis w] → UnknownClaimedAxis("w")
}

#[test]
fn follower_with_own_motors_and_spatial_claims_pass() {
    // x,y,z claimed; e has motors: extruder_motor → Ok
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(c
onfig)'` → FAIL.
- [ ] **Step 3: Implement.** `AxisRegistry::try_new` (or a sibling `validate_motor_mappi
ng(decls, kinematics_axes)` called from the same construction path — match plan 2's stru
cture) enforces: for every declared axis, `claimed XOR !motors.is_empty()`; every claime
d name is a declared axis. New `AxisConfigError` variants with messages naming the axis
and both homes (`"axis 'z' is motor-mapped twice: by [kinematics] and by [axis z] motors
:"`). `init_planner` (anchor: `rg -n "fn init_planner" rust/motion-engine/src/bridge.rs`
; reconcile with plan 4 Task 6's final arg list) gains `kinematics_axes: Vec<String>` an
d threads it in; `klippy/motion_engine.py`'s wrapper mirrors. The Python caller side lan
ds in Task 10 — until then the wrapper passes `[]`... no: **fail loudly instead** — make
 the argument required and update the single klippy call site in the same commit (Task 1
0 reworks it again; green at every commit matters more than task isolation here, so this
 task's klippy edit is the minimal `kinematics_axes=["x","y","z"]` hardcode matching cur
rent behavior, replaced in Task 10).
- [ ] **Step 4: Run** — `cargo nextest run` → PASS; `./scripts/ci.sh py` → PASS.
- [ ] **Step 5: Commit** — `feat(motion-engine): every axis motor-mapped exactly once, v
alidated in the registry`

---

## Task 9: klippy — motor sections, `drive:` key, axis-domain rail keys

**Files:**
- Modify: `klippy/stepper.py` (`AxisRail` construction path; anchor: `class PrinterRail`
, `class BaseRail`)
- Modify: `klippy/extras/servo_axis.py` (`ServoRail` takes axis section + motor section)
- Create: `test/test_axis_rail.py`
- Modify: `test/test_rail.py`, `test/test_servo_param.py`, `test/test_servo_homing.py` f
ixtures as their config dicts change shape

- [ ] **Step 1: Write failing tests** in `test/test_axis_rail.py` (mirror `test_rail.py`
's fake-config idiom):

```python
def test_axis_rail_reads_range_from_axis_section(fake_config):
    # [axis x] position_min: 0 / position_max: 300 / position_endstop: 0 /
    # endstop_pin: ^PG6 / homing_speed: 50 ; [motor_a] step_pin/dir_pin/
    # rotation_distance/microsteps
    rail = stepper.AxisRail(
        fake_config.getsection("axis x"), [fake_config.getsection("motor_a")]
    )
    assert rail.get_range() == (0.0, 300.0)
    assert rail.position_endstop == 0.0
    assert len(rail.get_steppers()) == 1
    assert rail.get_steppers()[0].get_name() == "motor_a"


def test_axis_rail_multiple_motors_lockstep(fake_config):
    # z role with three motor sections → three steppers, one endstop from [axis z]
    ...
    assert len(rail.get_steppers()) == 3


def test_motor_section_per_motor_endstop_override(fake_config):
    # second z motor section carries its own endstop_pin → two endstops on the rail
    ...
    assert len(rail.get_endstops()) == 2


def test_homing_keys_on_motor_section_rejected(fake_config):
    # [motor_a] position_min: 0 → config error naming [axis] as the home
    with pytest.raises(...):
        ...
```

- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL (`AxisRail` missin
g).
- [ ] **Step 3: Implement.**
  - `stepper.py`: `AxisRail(axis_config, motor_configs)` — reads every range/homing key
(`position_min`, `position_max`, `position_endstop`, `endstop_pin`, `homing_speed`, `sec
ond_homing_speed`, `homing_retract_dist`, `homing_retract_speed`, `homing_positive_dir`)
 from `axis_config`; builds one `PrinterStepper` per motor section (first is primary); r
egisters the axis endstop from `axis_config` with all steppers attached; a motor section
 carrying its own `endstop_pin` adds a per-motor endstop exactly like today's `add_extra
_stepper` path. Reuse `BaseRail`'s existing machinery — this is a constructor variant, n
ot a rewrite; `PrinterRail` (single-section form) keeps existing only if a non-motion co
nsumer still constructs it (`rg -n "PrinterRail(" klippy/` — `manual_stepper` etc.); hom
ing keys appearing on a motor section raise a config error pointing at `[axis <name>]`.
  - `stepper.py` or `motion_kinematics.py` (Task 10's file): the motor resolver —

```python
DRIVE_CHOICES = {"stepper": "stepper", "servo": "servo"}

def resolve_motor_section(config, name, referenced_by):
    if not config.has_section(name):
        raise config.error(
            "%s references motor '%s' but no [%s] section exists"
            % (referenced_by, name, name)
        )
    section = config.getsection(name)
    drive = section.getchoice("drive", DRIVE_CHOICES, "stepper")
    return section, drive
```

  - `servo_axis.py`: `ServoRail(axis_config, motor_config)` — servo hardware keys (`prot
ocol`, params block, counts/torque keys) read from the motor section (`drive: servo`), r
ange/homing keys from the axis section; `get_name(short=True)` returns the axis name so
`active_rails`/`servo_param` consumers keep working; the `servo_%s` section-name error s
trings update to motor-section names.
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS.
- [ ] **Step 5: Commit** — `feat(klippy): AxisRail — axis-domain homing/range keys on [a
xis], bare motor sections with drive:`

---

## Task 10: klippy — `[kinematics]` section, module classes, `BridgeKinematics` dies

**Files:**
- Create: `klippy/motion_kinematics.py`
- Create: `test/test_motion_kinematics.py`
- Modify: `klippy/motion.py` (`_load_kinematics`, `_read_axes` motor consumption, `init_
planner` call gains real `kinematics_axes`)

- [ ] **Step 1: Write failing tests** in `test/test_motion_kinematics.py`:

```python
def test_corexy_section_parses_roles_and_motors(fake_config):
    # [kinematics] type: corexy / axis_x: x / axis_y: y / axis_z: z /
    # a_motors: motor_a / b_motors: motor_b / z_motors: motor_z0, motor_z1
    kin = motion_kinematics.load_kinematics(fake_config, fake_motion)
    assert kin.kind == "corexy"
    assert kin.claimed_axes() == ["x", "y", "z"]
    assert kin.lanes()[0] == (0, "x", ["motor_a"])
    assert kin.lanes()[1] == (1, "y", ["motor_b"])
    assert kin.lanes()[2] == (2, "z", ["motor_z0", "motor_z1"])


def test_cartesian_uses_xyz_motor_roles(fake_config):
    # type: cartesian / x_motors: / y_motors: / z_motors:
    ...


def test_unknown_type_rejected(fake_config):
    # type: hybrid_corexy → error naming supported types
    ...


def test_role_binding_to_undeclared_axis_rejected(fake_config):
    # axis_x: w with no [axis w] → error
    ...


def test_missing_kinematics_section_rejected(fake_config):
    ...


def test_printer_kinematics_key_rejected(fake_config):
    # [printer] kinematics: corexy → error pointing at [kinematics]
    ...
```

- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL.
- [ ] **Step 3: Implement** `motion_kinematics.py`:

```python
class _LinearKinematics:
    supports_dual_carriage = False

    def __init__(self, config, motion, kind, role_specs):
        # role_specs: [("a_motors", "axis_x", 0, kin_tag_for_lane), ...] per type
        # parse axis_<role> bindings against declared [axis] sections;
        # resolve motor names via resolve_motor_section; build AxisRail or
        # ServoRail per lane (axis section + that lane's motor sections);
        # self.rails, self.limits exactly as BridgeKinematics held them.

    def claimed_axes(self): ...
    def lanes(self): ...          # [(lane_idx, axis_name, [motor names])]
    def coupled_xy(self): ...     # True for corexy: phase handover + awd grouping
    def mcu_tag(self, lanes_on_mcu): ...  # 0 if corexy and lanes 0&1 present, else 1

    # consumer surface carried over verbatim from BridgeKinematics:
    # get_steppers, rails, get_status, clear_homing_state, set_position,
    # calc_position, check_move, note_z_not_homed, active_rails


KINEMATICS_TYPES = {
    "corexy": [("a_motors", "axis_x", 0), ("b_motors", "axis_y", 1),
               ("z_motors", "axis_z", 2)],
    "cartesian": [("x_motors", "axis_x", 0), ("y_motors", "axis_y", 1),
                  ("z_motors", "axis_z", 2)],
}


def load_kinematics(config, motion):
    if config.getsection("printer").get("kinematics", None) is not None:
        raise config.error(
            "[printer] kinematics is not supported: declare a [kinematics] "
            "section (type + axis role bindings + motor lists)"
        )
    if not config.has_section("kinematics"):
        raise config.error("[kinematics] section is required")
    section = config.getsection("kinematics")
    kind = section.get("type")
    if kind not in KINEMATICS_TYPES:
        raise config.error(
            "[kinematics] type '%s' is not supported (supported: %s)"
            % (kind, ", ".join(sorted(KINEMATICS_TYPES)))
        )
    return _LinearKinematics(config, motion, kind, KINEMATICS_TYPES[kind])
```

  Carry over from `BridgeKinematics` unchanged in meaning: `check_move`'s z-ratio speed
clamp, `set_position` → `bridge.set_position` + per-axis range install, `get_status` (`h
omed_axes`/`axis_minimum`/`axis_maximum` now read ranges from the axis sections via the
rails), `clear_homing_state`, the `stepper_enable:motor_off` hook, `homing.resolve_endst
ops` load, `active_rails` (coupling from `coupled_xy()` instead of string comparison). E
ach motor stepper gets `setup_itersolve("cartesian_stepper_alloc", <lane axis letter>)`
exactly as today (a_motors get `"x"`, b_motors `"y"` — preserves `is_active_axis` behavi
or for z_tilt-style consumers; conscious klippy-side carry-over). `Motion._load_kinemati
cs` calls `motion_kinematics.load_kinematics`; `BridgeKinematics` and the `Move`-class's
 reliance on it are updated in place; the `dual_carriage` check keys off `supports_dual_
carriage` as today. `_init_planner` passes `kinematics_axes=self.kin.claimed_axes()` (re
placing Task 8's hardcode) and keeps shipping the same `(handle, axes, tag)` topology —
now derived in Task 11.
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS; `rg -n "BridgeKinematics" klippy/ t
est/` → empty.
- [ ] **Step 5: Commit** — `feat(klippy): [kinematics] section with declared modules rep
laces BridgeKinematics`

---

## Task 11: klippy — slot assignment and MCU topology from the explicit map

**Files:**
- Modify: `klippy/motion.py` (`_init_planner`, `_configure_axes_per_mcu`, `_derive_mcu_t
opology`; delete `_MOTOR_SLOT_PREFIXES`, `_name_motor_slot`, `_stepper_motor_slot`, `_KI
N_COREXY`/`_KIN_CARTESIAN` mirrors)
- Create/Modify: `test/test_motion_topology.py` (new; move/adapt any existing `_derive_m
cu_topology` tests — find them: `rg -ln "_derive_mcu_topology|_name_motor_slot" test/`)

- [ ] **Step 1: Write failing tests:**

```python
def test_topology_from_lanes_two_mcu_corexy(fake_motion):
    # lanes: 0→motor_a(mcu A), 1→motor_b(mcu A), 2→motor_z(mcu B),
    # follower e → extruder_motor(mcu A)
    topo = fake_motion._derive_mcu_topology()
    assert topo == [(handle_a, [0, 1, 3], 0), (handle_b, [2], 1)]


def test_topology_cartesian_all_tags_one(fake_motion):
    ...


def test_lane_slot_steppers_ordered_primary_first(fake_motion):
    # multi-motor z role: slot 2 lists motors in declared order, first is primary
    ...
```

- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL.
- [ ] **Step 3: Implement.** Slot map = spatial lanes from `kin.lanes()` + one slot per
follower axis at its registry index, motors from the `[axis].motors` list (`self.axis_se
ctions` — parsed since plan 2, consumed here at last). For each slot: primary motor = fi
rst declared; MCU handle = primary's `get_mcu()._engine_handle` (servo lanes resolve thr
ough `ethercat_node` exactly as the current servo branch does); per-MCU tag from `kin.mc
u_tag(...)`. `_configure_axes_per_mcu` builds `slot_steppers` from this map instead of `
_name_motor_slot` prefix matching — the body downstream (steps_per_mm, invert, phase con
figs, `kalico_configure_axis` sends) is reused as-is; `awd_default`/phase-group `xy_coup
led` come from `kin.coupled_xy()`. Delete the prefix tables and the numeric `_KIN_*` mir
rors (the tag now arrives from `kin.mcu_tag`, whose values are pinned by Task 6's Rust-s
ide `from_tag` validation). Follower slots beyond 3 are not new here — the slot count fo
llows the registry as plan 4 left it; assert loudly if a follower's declared motor maps
to a slot the MCU build rejects.
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS; `rg -n "_MOTOR_SLOT_PREFIXES|_name_
motor_slot|stepper_x" klippy/motion.py` → empty.
- [ ] **Step 5: Commit** — `feat(klippy): slot/topology assignment from the declared kin
ematic map; name-prefix matching dies`

---

## Task 12: extruder split — `[extruder]` keeps the heater, the motor moves out

**Files:**
- Modify: `klippy/kinematics/extruder.py` (delete `ExtruderStepper`; `PrinterExtruder` s
heds stepper construction)
- Modify: `klippy/extras/extruder_stepper.py` if present (`rg -ln "ExtruderStepper" klip
py/`) — section rejected at load
- Modify: `klippy/motion.py` (drop `register_step_generator`/`step_generators` if the ex
truder was the last caller — verify: `rg -n "register_step_generator" klippy/`)
- Create: `test/test_extruder_split.py`

- [ ] **Step 1: Write failing tests:**

```python
def test_extruder_section_with_step_pin_rejected(fake_config):
    # [extruder] step_pin: PE2 → error: "move stepper config to a motor
    # section and reference it from [axis e] motors:"
    ...


def test_extruder_heater_only_section_loads(fake_config):
    # [extruder] with heater/filament keys only + [axis e] motors: extruder_motor
    # + [extruder_motor] drive defaults to stepper → printer objects load;
    # lookup_object("extruder").get_heater() works; check_move still validates
    # max_extrude_cross_section.
    ...


def test_extruder_stepper_extra_rejected(fake_config):
    # [extruder_stepper foo] → error naming the motor-section replacement
    ...
```

- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL.
- [ ] **Step 3: Implement.** `PrinterExtruder.__init__` raises on `step_pin`/`dir_pin`/`
rotation_distance`/`microsteps` present in its section (error text above); the `Extruder
Stepper` class, its commands (`SET_PRESSURE_ADVANCE`, `SET_EXTRUDER_ROTATION_DISTANCE`,
`SYNC_EXTRUDER_MOTION`, `SET_E_STEP_DISTANCE`...), and its `register_step_generator` use
 are deleted — `rg -n "ExtruderStepper" klippy/` → empty (the `[extruder_stepper]` extra
 module body becomes a config-error raise). `PrinterExtruder` keeps: heater wiring, `get
_heater`, `get_status`, `check_move` (reads E motion limits from its own keys exactly as
 today), `get_name`, M104/M109/ACTIVATE_EXTRUDER, `set_extruder` interplay with `Motion`
. `DummyExtruder` unchanged.
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS.
- [ ] **Step 5: Commit** — `feat(klippy): [extruder] is a heater; the E motor is an ordi
nary motor section`

---

## Task 13: legacy rejection sweep + fixture rewrite

**Files:**
- Modify: `klippy/motion.py` (legacy section/key rejections at `_read_axes`/`_read_limit
s`'s side — same pattern as `LEGACY_LIMIT_KEYS`)
- Modify: `tools/sim_klippy/printer.cfg` (the canonical bridge-mode fixture)
- Modify: every test config the suite boots — find: `rg -ln "stepper_x|kinematics:" test
/ tools/ | grep -v __pycache__`
- Modify: `test/test_active_rails.py`, `test/test_rail.py`, servo tests — fixture shapes

- [ ] **Step 1: Write failing tests** (in `test/test_motion_kinematics.py`):

```python
def test_stepper_x_section_rejected(fake_config):
    # [stepper_x] present → error: "role-encoding motor sections are not
    # supported: name the motor freely and assign it in [kinematics]"
    ...


def test_servo_x_section_rejected(fake_config):
    # [servo_x] present → error pointing at drive: servo motor sections
    ...
```

- [ ] **Step 2: Run to verify failure**, **implement the rejections** (check `has_sectio
n` for `stepper_x|stepper_y|stepper_z|stepper_a|stepper_b` + `servo_x|servo_y|servo_z` a
nd any `stepper_<axis><digit>` via prefix scan — these are *rejections* of known-legacy
names, not discovery; arbitrary motor names are untouched).
- [ ] **Step 3: Rewrite `tools/sim_klippy/printer.cfg`:**

```ini
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: motor_a
b_motors: motor_b
z_motors: motor_z

[axis x]
position_min: ...     ; moved verbatim from [stepper_x]
position_max: ...
position_endstop: ...
endstop_pin: ...
homing_speed: ...

[axis e]
follows: x, y, z
motors: extruder_motor

[motor_a]
; step_pin/dir_pin/enable_pin/microsteps/rotation_distance from old [stepper_x]

[tmc5160 motor_a]
; verbatim from [tmc5160 stepper_x]

[extruder_motor]
; sim-pin stepper for the follower lane (plan 4's lane-3 e2e needs it —
; reconcile with whatever extruder motor plan 4's fixture work added)
```

  …and the same for `motor_b`/`motor_z` and the axis sections; `[post_processor]` sectio
ns stay as plan 4 left them. Update every other booted config the grep found the same wa
y.
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS (whole suite, rewritten fixtures).
- [ ] **Step 5: Commit** — `feat(klippy): legacy stepper_x/servo_x/[printer]kinematics r
ejected; fixtures speak [kinematics]`

---

## Task 14: end-to-end + gates + PR

- [ ] **Step 1:** `cargo nextest run` from `rust/` → green; `cargo test --doc` if doc ex
amples touched; `./scripts/ci.sh quick` → green; `./scripts/ci.sh py` → green.
- [ ] **Step 2:** kalico-sim end-to-end (see the `kalico-sim` skill): boot the rewritten
 sim config; run the sim harness's homing flow (G28 path through endstop trip → `Kinemat
icsModule.inverse` recovery) and a short print stream; assert final positions and step c
ounts match a pre-branch capture of the same G-code on the plan-4 baseline (`KALICO_SIM_
STEP_COUNT` / `KALICO_SIM_MOTION_STATE` per the skill). Record numbers in the PR descrip
tion.
- [ ] **Step 3:** Snapshot test from Task 1 still green — the published `toolhead` surfa
ce survived both phases byte-for-byte.
- [ ] **Step 4:** `cargo fmt --all --check`; commit stragglers; open the PR (base: `sota
-motion`) with the Phase A/B summary, the sim numbers, and the bench-config migration sn
ippet (Trident corexy example) in the description. Bench configs on the Pis are rewritte
n at flash time, not in this repo.

---

## Self-review notes (already applied)

- **Spec §6 item 5 coverage:** toolhead out of the Rust side (Task 2), seam rename (Task
 3), thin status shim retired on its own schedule (Task 4 + out-of-scope note), declared
 kinematic map with explicit motor-to-role assignment replacing role-encoding names (Tas
ks 6–13). Spec §1 coverage rules: motor-mapped-exactly-once (Task 8), every-axis-in-a-li
mit and follows-references-declared landed in plans 1–2, letter rules landed in plan 2 (
`RESERVED_LETTERS`, `rust/motion-engine/src/config.rs:11`).
- **MCU untouched:** no task edits `src/`, `rust/runtime` beyond Task 2's comment/varian
t rename (discriminants pinned), or the boundary doc — checked against every Files list.
- **Pure-function planner requirement:** untouched — no task changes `temporal`/`traject
ory` interfaces; kinematics sits at emission (`enqueue.rs`), downstream of the planner,
exactly where spec §1 places it.
- **Fail-loud audit:** unknown tag (Task 6), corexy-tag-without-XY silent fallback dies
(Task 6), all-zero lane weights panic (Task 7), follower in homing trip asserts (Task 7)
, motor-mapping XOR (Task 8), homing keys on motor sections (Task 9), unknown kinematics
 type / missing section / `[printer]` key / legacy sections / `[extruder]` step_pin / `[
extruder_stepper]` (Tasks 10–13).
- **Type consistency:** `KinematicsModule::{from_tag, kind, tag, lane_weights, lane_is_i
dentity, forward, inverse}` used identically in Tasks 6–7; `kin.{claimed_axes, lanes, co
upled_xy, mcu_tag}` identically in Tasks 10–11; `resolve_motor_section` defined Task 9,
consumed Task 10; `AxisRail(axis_config, motor_configs)` consistent Tasks 9–10.
- **Known sequencing seam:** Task 8 hardcodes `kinematics_axes=["x","y","z"]` at the sin
gle klippy call site to stay green, Task 10 replaces it with `kin.claimed_axes()` — flag
ged in both tasks.
