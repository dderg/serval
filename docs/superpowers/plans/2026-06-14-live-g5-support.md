# Live G5 / G5.1 Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `G5` (cubic Bézier) and `G5.1` (quadratic Bézier) typed at the console or emitted from a macro actually move the toolhead along the curve.

**Architecture:** Python (`gcode_move.py`/`motion.py`) keeps all g-code coordinate semantics and resolves the endpoint + raw control-point offsets; new pyo3 bridge entries (`submit_bezier`/`submit_quadratic`) carry the control points into Rust, where `classify_bezier`/`classify_quadratic` assemble a `CubicSegment` (control-point math lives in a new `geometry::curve` module) and hand it to the existing optimizer unchanged. Smooth-G5 chaining state and the extruder arc-length ratio live in Rust; range-check, transform gate, and bed-mesh gate live in Python.

**Tech Stack:** Rust (crates `geometry`, `nurbs`, `motion-engine` with pyo3 0.29), Python (klippy). Rust tests via `cargo nextest run`; Python tests via `./scripts/ci.sh py`. Spec: `docs/superpowers/specs/2026-06-14-live-g5-support-design.md`.

---

## File Structure

**Rust:**
- Create `rust/geometry/src/curve.rs` — control-point math: `to_collinear_bezier` (relocated from `compat`), `g5_control_points`, `g51_control_points`.
- Create `rust/geometry/src/curve/tests.rs` — unit tests for the three functions.
- Modify `rust/geometry/src/lib.rs` — add `pub mod curve;`.
- Modify `rust/motion-engine/src/classify.rs` — swap the `compat` import for `geometry::curve`; add `classify_bezier`, `classify_quadratic`, shared `classify_curve`.
- Modify `rust/motion-engine/src/classify/tests.rs` — tests for the new classifiers.
- Modify `rust/motion-engine/src/bridge.rs` — add `last_g5_pq` field, `e_followers` helper, `submit_bezier`/`submit_quadratic` pymethods; clear chain in `submit_move`.
- Modify `rust/motion-engine/Cargo.toml` — remove the `compat` dependency.
- Modify `rust/compat/src/collinear.rs` + `rust/compat/src/collinear/tests.rs` — delete `to_collinear_bezier` and its test (moved to `geometry`).

**Python:**
- Modify `klippy/motion_engine.py` — `submit_bezier`/`submit_quadratic` passthroughs; add both to `_STUB_MOTION_METHODS`.
- Modify `klippy/motion.py` — `Motion.move_curve(...)`.
- Modify `klippy/extras/gcode_move.py` — register + implement `cmd_G5`, `cmd_G5_1`, transform-gate helper.
- Modify `klippy/extras/bed_mesh.py` — activation gate in `set_mesh`.

**Tests:**
- `test/test_g5_console.py` (pytest) — Python-side parse/validation/gate tests against a fake bridge.
- Rust tests inline per crate.

---

## Phase 1 — geometry foundation

### Task 1: Create the `geometry::curve` module with `to_collinear_bezier`

**Files:**
- Create: `rust/geometry/src/curve.rs`
- Create: `rust/geometry/src/curve/tests.rs`
- Modify: `rust/geometry/src/lib.rs` (module list, currently `pub mod error; pub mod params; pub mod pipeline; pub(crate) mod reduce; pub mod segment; pub mod splitter; pub mod telemetry;`)

- [ ] **Step 1: Write the failing test**

Create `rust/geometry/src/curve/tests.rs`:

```rust
use super::*;

#[test]
fn collinear_places_control_points_at_thirds() {
    let cps = to_collinear_bezier([0.0, 0.0, 0.0], [9.0, 0.0, 0.0]);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]);
    assert_eq!(cps[1], [3.0, 0.0, 0.0]);
    assert_eq!(cps[2], [6.0, 0.0, 0.0]);
    assert_eq!(cps[3], [9.0, 0.0, 0.0]);
}

#[test]
fn collinear_handles_3d_diagonal() {
    let cps = to_collinear_bezier([1.0, 2.0, 3.0], [4.0, 8.0, 6.0]);
    assert_eq!(cps[1], [2.0, 4.0, 4.0]);
    assert_eq!(cps[2], [3.0, 6.0, 5.0]);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p geometry -E 'test(collinear_places_control_points_at_thirds)'`
Expected: FAIL — `curve` module / `to_collinear_bezier` does not exist.

- [ ] **Step 3: Create the module and function**

Create `rust/geometry/src/curve.rs`:

```rust
//! Cubic-Bézier control-point construction for the live motion path.
//! Owns the geometry primitives the planner needs; the offline `compat`
//! preprocessor is a separate crate the engine must not depend on.

/// Straight-line move as a degenerate cubic Bézier — control points collinear
/// at the 1/3 and 2/3 marks. Relocated from `compat::collinear` so the live
/// engine no longer links the offline preprocessor.
#[must_use]
pub fn to_collinear_bezier(start: [f64; 3], end: [f64; 3]) -> [[f64; 3]; 4] {
    let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    let p1 = [
        start[0] + d[0] / 3.0,
        start[1] + d[1] / 3.0,
        start[2] + d[2] / 3.0,
    ];
    let p2 = [
        start[0] + 2.0 * d[0] / 3.0,
        start[1] + 2.0 * d[1] / 3.0,
        start[2] + 2.0 * d[2] / 3.0,
    ];
    [start, p1, p2, end]
}

#[cfg(test)]
mod tests;
```

Add to `rust/geometry/src/lib.rs`, in the module list (keep alphabetical-ish, after `pub mod, params;`):

```rust
pub mod curve;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p geometry -E 'test(collinear)'`
Expected: PASS (both tests).

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/curve.rs rust/geometry/src/curve/tests.rs rust/geometry/src/lib.rs
git commit -m "feat(geometry): add curve module with relocated to_collinear_bezier"
```

### Task 2: Add the G5 cubic control-point builder

**Files:**
- Modify: `rust/geometry/src/curve.rs`
- Modify: `rust/geometry/src/curve/tests.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/geometry/src/curve/tests.rs`:

```rust
#[test]
fn g5_assembles_control_points_with_linear_z() {
    // start (0,0,0), endpoint delta (10,0,6), I/J=(2,4), P/Q=(-3,4)
    let cps = g5_control_points([0.0, 0.0, 0.0], 2.0, 4.0, -3.0, 4.0, 10.0, 0.0, 6.0);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]); // P0 = start
    assert_eq!(cps[1], [2.0, 4.0, 2.0]); // P1 = start+(I,J), z = dz/3
    assert_eq!(cps[2], [7.0, 4.0, 4.0]); // P2 = end+(P,Q) = (10-3,0+4), z = 2dz/3
    assert_eq!(cps[3], [10.0, 0.0, 6.0]); // P3 = end
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p geometry -E 'test(g5_assembles_control_points_with_linear_z)'`
Expected: FAIL — `g5_control_points` not found.

- [ ] **Step 3: Implement**

Append to `rust/geometry/src/curve.rs` (before `#[cfg(test)] mod tests;`):

```rust
/// G5 cubic Bézier control points. `i,j` = XY offset from start to the first
/// control point; `p,q` = XY offset from the end to the second control point.
/// Z is interpolated linearly (thirds), so segment Z is linear in path — our
/// planar-G5 extension (standard G5 is XY-only).
#[must_use]
pub fn g5_control_points(
    start: [f64; 3],
    i: f64,
    j: f64,
    p: f64,
    q: f64,
    dx: f64,
    dy: f64,
    dz: f64,
) -> [[f64; 3]; 4] {
    let end = [start[0] + dx, start[1] + dy, start[2] + dz];
    let p1 = [start[0] + i, start[1] + j, start[2] + dz / 3.0];
    let p2 = [end[0] + p, end[1] + q, start[2] + 2.0 * dz / 3.0];
    [start, p1, p2, end]
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p geometry -E 'test(g5_assembles)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/curve.rs rust/geometry/src/curve/tests.rs
git commit -m "feat(geometry): add G5 cubic control-point builder"
```

### Task 3: Add the G5.1 quadratic→cubic exact elevation

**Files:**
- Modify: `rust/geometry/src/curve.rs`
- Modify: `rust/geometry/src/curve/tests.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/geometry/src/curve/tests.rs`:

```rust
use nurbs::eval::eval;

#[test]
fn g51_elevation_is_exact_against_the_quadratic() {
    // Quadratic: Q0=start, Q1=start+(I,J), Q2=end. Sample both, compare.
    let start = [0.0, 0.0, 0.0];
    let (i, j, dx, dy, dz) = (3.0, 5.0, 8.0, 0.0, 4.0);
    let cubic = g51_control_points(start, i, j, dx, dy, dz);

    let q0 = start;
    let q1 = [start[0] + i, start[1] + j, start[2] + dz / 2.0];
    let q2 = [start[0] + dx, start[1] + dy, start[2] + dz];
    let quad = |t: f64| {
        let mt = 1.0 - t;
        [0usize, 1, 2].map(|k| mt * mt * q0[k] + 2.0 * mt * t * q1[k] + t * t * q2[k])
    };

    let cubic_nurbs = nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cubic.to_vec(),
    )
    .unwrap();
    for n in 0..=10 {
        let t = f64::from(n) / 10.0;
        let got = eval(&cubic_nurbs, t);
        let want = quad(t);
        for k in 0..3 {
            assert!((got[k] - want[k]).abs() < 1e-12, "t={t} axis={k}");
        }
    }
}

#[test]
fn g51_z_is_linear_after_elevation() {
    let cps = g51_control_points([0.0, 0.0, 0.0], 1.0, 1.0, 0.0, 0.0, 6.0);
    assert!((cps[1][2] - 2.0).abs() < 1e-12); // dz/3
    assert!((cps[2][2] - 4.0).abs() < 1e-12); // 2dz/3
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p geometry -E 'test(g51_)'`
Expected: FAIL — `g51_control_points` not found.

- [ ] **Step 3: Implement**

Append to `rust/geometry/src/curve.rs` (before `#[cfg(test)] mod tests;`):

```rust
/// G5.1 quadratic Bézier (single control point `i,j` offset from start),
/// elevated *exactly* to cubic. Z interpolated linearly.
#[must_use]
pub fn g51_control_points(
    start: [f64; 3],
    i: f64,
    j: f64,
    dx: f64,
    dy: f64,
    dz: f64,
) -> [[f64; 3]; 4] {
    let q0 = start;
    let q1 = [start[0] + i, start[1] + j, start[2] + dz / 2.0];
    let q2 = [start[0] + dx, start[1] + dy, start[2] + dz];
    let elevate = |a: [f64; 3], mid: [f64; 3]| {
        [
            a[0] + 2.0 / 3.0 * (mid[0] - a[0]),
            a[1] + 2.0 / 3.0 * (mid[1] - a[1]),
            a[2] + 2.0 / 3.0 * (mid[2] - a[2]),
        ]
    };
    [q0, elevate(q0, q1), elevate(q2, q1), q2]
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p geometry -E 'test(g51_)'`
Expected: PASS.
If `nurbs::eval::eval` has a different path, run `grep -rn "pub fn eval" rust/nurbs/src/eval.rs` and adjust the import.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/curve.rs rust/geometry/src/curve/tests.rs
git commit -m "feat(geometry): add exact G5.1 quadratic->cubic elevation"
```

### Task 4: Sever the `compat` dependency from the live engine

**Files:**
- Modify: `rust/motion-engine/src/classify.rs:1` (import)
- Modify: `rust/motion-engine/Cargo.toml` (remove `compat`)
- Modify: `rust/compat/src/collinear.rs` (delete `to_collinear_bezier`)
- Modify: `rust/compat/src/collinear/tests.rs` (delete its test)

- [ ] **Step 1: Repoint the import**

In `rust/motion-engine/src/classify.rs`, change line 1:

```rust
use geometry::curve::to_collinear_bezier;
```

(was `use compat::collinear::to_collinear_bezier;`)

- [ ] **Step 2: Delete the moved function from compat**

In `rust/compat/src/collinear.rs`, delete the `to_collinear_bezier` function (lines 20-33) entirely. Keep `to_collinear_g5` (the text emitter) untouched.

In `rust/compat/src/collinear/tests.rs`, delete the two tests that reference `to_collinear_bezier` (`fn` bodies calling it). Keep tests for `to_collinear_g5`.

- [ ] **Step 3: Remove the dependency**

In `rust/motion-engine/Cargo.toml`, delete the line:

```toml
compat = { path = "../compat" }
```

- [ ] **Step 4: Verify the whole workspace builds and tests pass**

Run: `cd rust && cargo nextest run -p geometry -p compat -p motion-engine && cargo build -p motion-engine`
Expected: PASS, and no `unused dependency: compat` from clippy.
Run: `cd rust && cargo clippy -p motion-engine -- -D warnings`
Expected: clean.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-engine/src/classify.rs rust/motion-engine/Cargo.toml rust/compat/src/collinear.rs rust/compat/src/collinear/tests.rs
git commit -m "refactor: move to_collinear_bezier to geometry, drop motion-engine->compat dep"
```

---

## Phase 2 — motion-engine classifiers + pyo3 entries + chaining

### Task 5: Add `classify_curve` / `classify_bezier` / `classify_quadratic`

**Files:**
- Modify: `rust/motion-engine/src/classify.rs`
- Modify: `rust/motion-engine/src/classify/tests.rs`

These build a `CubicSegment` from explicit control points, using **true arc length** for both `distance_mm` and the follower ratio (the spec's one extrusion-correctness requirement). They mirror `classify_and_build` but skip `to_collinear_bezier`.

- [ ] **Step 1: Write the failing test**

Append to `rust/motion-engine/src/classify/tests.rs`:

```rust
#[test]
fn classify_bezier_uses_arc_length_for_distance_and_ratio() {
    // A curved G5 with an E delta. distance_mm must be the arc length (> chord),
    // and the follower ratio must be de / arc_length (not de / chord).
    let start = [0.0, 0.0, 0.0];
    let m = classify_bezier(start, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 0.0, &[(3usize, 2.0)], 30.0)
        .expect("curve classifies");
    let chord = 10.0_f64;
    assert!(m.distance_mm > chord, "arc length must exceed the chord");
    let ratio = m.segment.followers[0].ratio;
    assert!((ratio - 2.0 / m.distance_mm).abs() < 1e-9, "ratio is de/arc_length");
}

#[test]
fn classify_quadratic_builds_a_segment() {
    let m = classify_quadratic([0.0, 0.0, 0.0], 5.0, 5.0, 10.0, 0.0, 0.0, &[], 30.0)
        .expect("quadratic classifies");
    assert!(m.distance_mm > 10.0);
    assert_eq!(m.segment.feedrate_mm_s, 30.0);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p motion-engine -E 'test(classify_bezier_uses_arc_length)'`
Expected: FAIL — `classify_bezier` not found.

- [ ] **Step 3: Implement**

In `rust/motion-engine/src/classify.rs`, update imports at the top:

```rust
use geometry::curve::{g5_control_points, g51_control_points, to_collinear_bezier};
```

Append these functions after `classify_and_build`:

```rust
fn classify_curve(
    cps: [[f64; 3]; 4],
    followers: &[(usize, f64)],
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    let xyz = build_cubic(cps)?;
    let arc_len = nurbs::arc_length::path_arc_length(&xyz);
    if arc_len <= DISPLACEMENT_EPSILON {
        return Err(ClassifyError::ZeroDisplacement);
    }
    let demands = followers
        .iter()
        .copied()
        .filter(|&(_, d)| d.abs() > DISPLACEMENT_EPSILON)
        .map(|(axis_index, delta)| FollowerDemand {
            axis_index,
            ratio: delta / arc_len,
        })
        .collect();
    let source = SourceRange {
        start_line: 0,
        end_line: 0,
    };
    let segment = CubicSegment::try_new(xyz, demands, feedrate_mm_s, source, None)
        .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;
    Ok(ClassifiedMove {
        segment,
        distance_mm: arc_len,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn classify_bezier(
    start: [f64; 3],
    i: f64,
    j: f64,
    p: f64,
    q: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    followers: &[(usize, f64)],
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    classify_curve(
        g5_control_points(start, i, j, p, q, dx, dy, dz),
        followers,
        feedrate_mm_s,
    )
}

#[allow(clippy::too_many_arguments)]
pub fn classify_quadratic(
    start: [f64; 3],
    i: f64,
    j: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    followers: &[(usize, f64)],
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    classify_curve(
        g51_control_points(start, i, j, dx, dy, dz),
        followers,
        feedrate_mm_s,
    )
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p motion-engine -E 'test(classify_bezier) + test(classify_quadratic)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-engine/src/classify.rs rust/motion-engine/src/classify/tests.rs
git commit -m "feat(motion-engine): classify_bezier/quadratic with arc-length follower ratio"
```

### Task 6: Add the chaining-state field + `e_followers` helper to the bridge

**Files:**
- Modify: `rust/motion-engine/src/bridge.rs` (struct fields ~478-518; constructor where `commanded_pos` is initialized; `submit_move` ~2909)

- [ ] **Step 1: Add the field**

In the `PyMotionEngine` struct (`rust/motion-engine/src/bridge.rs`, near `commanded_pos: Mutex<[f64; 3]>,`), add:

```rust
    last_g5_pq: Mutex<Option<(f64, f64)>>,
```

- [ ] **Step 2: Initialize it in the constructor**

Find where `commanded_pos: Mutex::new([0.0, 0.0, 0.0])` is set in the struct literal (run `grep -n "commanded_pos: Mutex::new" rust/motion-engine/src/bridge.rs`). Add alongside it:

```rust
            last_g5_pq: Mutex::new(None),
```

- [ ] **Step 3: Add the `e_followers` helper and clear the chain in `submit_move`**

Add this private method inside the same `impl PyMotionEngine` block (near `submit_move`):

```rust
    fn e_followers(&self, de: f64) -> PyResult<Vec<(usize, f64)>> {
        if de.abs() > 0.0 {
            let cfg = self
                .planner_config
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let axis_index = cfg.axis_registry.axis_index("e").map_err(|_| {
                PyRuntimeError::new_err(
                    "E word on a move but no [axis e] is declared — declare the \
                     follower axis or stop sending E",
                )
            })?;
            Ok(vec![(axis_index, de)])
        } else {
            Ok(vec![])
        }
    }
```

In `submit_move`, after the position is advanced (`pos[2] += dz;`), clear the chain:

```rust
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
```

- [ ] **Step 4: Build to verify it compiles**

Run: `cd rust && cargo build -p motion-engine`
Expected: compiles (no test yet — exercised in Task 7 via the pymethods; field is otherwise `dead_code` until then, so this step may emit a dead-code warning, which Task 7 clears).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-engine/src/bridge.rs
git commit -m "feat(motion-engine): add G5 chaining state + e_followers helper; clear chain on linear move"
```

### Task 7: Add `submit_bezier` and `submit_quadratic` pymethods

**Files:**
- Modify: `rust/motion-engine/src/bridge.rs` (the `#[pymethods] impl PyMotionEngine`)
- Modify: `rust/motion-engine/src/bridge.rs` tests (or `rust/motion-engine/tests/`) — a Rust-level test is awkward through pyo3; this method is covered by the Python integration test in Phase 5. Add a focused unit test for the chaining math instead (Step 1).

- [ ] **Step 1: Write the failing test (chaining reflection math)**

Append to `rust/motion-engine/src/classify/tests.rs` a test of the reflection convention used by the pymethod, so the math is covered without pyo3:

```rust
#[test]
fn chain_reflection_negates_previous_pq() {
    // Chained G5: omitted I/J => (I,J) = (-P_prev, -Q_prev). Verify the cubic's
    // start tangent (P1-P0) opposes the previous exit tangent direction.
    let prev_pq = (3.0, -2.0);
    let (i, j) = (-prev_pq.0, -prev_pq.1);
    let cps = geometry::curve::g5_control_points([0.0, 0.0, 0.0], i, j, 1.0, 1.0, 10.0, 0.0, 0.0);
    assert_eq!(cps[1][0], -3.0);
    assert_eq!(cps[1][1], 2.0);
    assert_eq!(cps[1][2], 0.0); // linear Z preserved, NOT a 3D reflection
}
```

- [ ] **Step 2: Run test to verify it fails (or compiles)**

Run: `cd rust && cargo nextest run -p motion-engine -E 'test(chain_reflection_negates_previous_pq)'`
Expected: PASS already if `g5_control_points` is public (it is) — this test documents the convention. If it fails, fix the sign convention before proceeding.

- [ ] **Step 3: Implement the two pymethods**

In the `#[pymethods] impl PyMotionEngine` block (where `submit_move` lives), add:

```rust
    #[pyo3(signature = (i, j, p, q, dx, dy, dz, de, feedrate))]
    fn submit_bezier(
        &self,
        py: Python<'_>,
        i: Option<f64>,
        j: Option<f64>,
        p: f64,
        q: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        py.detach(|| -> PyResult<()> {
            let followers = self.e_followers(de)?;
            let (ii, jj) = match (i, j) {
                (Some(i), Some(j)) => (i, j),
                (None, None) => {
                    let prev = *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner());
                    let (pp, qq) = prev.ok_or_else(|| {
                        PyRuntimeError::new_err("G5 without I J must follow another G5")
                    })?;
                    (-pp, -qq)
                }
                _ => {
                    return Err(PyRuntimeError::new_err(
                        "G5 I and J must both be present or both omitted",
                    ));
                }
            };
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let classified =
                classify::classify_bezier(pos, ii, jj, p, q, dx, dy, dz, &followers, feedrate)
                    .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
            {
                let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                planner.submit_move(classified).map_err(planner_err)?;
            }
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = Some((p, q));
            Ok(())
        })
    }

    #[pyo3(signature = (i, j, dx, dy, dz, de, feedrate))]
    fn submit_quadratic(
        &self,
        py: Python<'_>,
        i: f64,
        j: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        py.detach(|| -> PyResult<()> {
            let followers = self.e_followers(de)?;
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let classified =
                classify::classify_quadratic(pos, i, j, dx, dy, dz, &followers, feedrate)
                    .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
            {
                let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                planner.submit_move(classified).map_err(planner_err)?;
            }
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
            Ok(())
        })
    }
```

- [ ] **Step 4: Build + clippy**

Run: `cd rust && cargo build -p motion-engine && cargo clippy -p motion-engine -- -D warnings`
Expected: clean (the `last_g5_pq` dead-code warning from Task 6 is now resolved).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-engine/src/bridge.rs rust/motion-engine/src/classify/tests.rs
git commit -m "feat(motion-engine): submit_bezier/submit_quadratic pymethods with G5 chaining"
```

---

## Phase 3 — Python bridge passthrough

### Task 8: Wire `submit_bezier`/`submit_quadratic` through the Python wrapper + stub

**Files:**
- Modify: `klippy/motion_engine.py` (`_STUB_MOTION_METHODS` ~28-65; `MotionEngineWrapper` methods after `submit_dwell` ~390)

- [ ] **Step 1: Add to the stub method set**

In `klippy/motion_engine.py`, add to the `_STUB_MOTION_METHODS` frozenset (alongside `"submit_move"`, `"submit_dwell"`):

```python
        "submit_bezier",
        "submit_quadratic",
```

- [ ] **Step 2: Add the passthrough methods**

In `MotionEngineWrapper`, after `submit_dwell` (line ~390), add:

```python
    def submit_bezier(self, i, j, p, q, dx, dy, dz, de, feedrate):
        return self._bridge.submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)

    def submit_quadratic(self, i, j, dx, dy, dz, de, feedrate):
        return self._bridge.submit_quadratic(i, j, dx, dy, dz, de, feedrate)
```

- [ ] **Step 3: Smoke-check import**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -c "import klippy.motion_engine as m; print('submit_bezier' in m._STUB_MOTION_METHODS)"`
Expected: prints `True`.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_engine.py
git commit -m "feat(klippy): submit_bezier/submit_quadratic passthroughs + stub entries"
```

---

## Phase 4 — Python motion.move_curve (validation + range check)

### Task 9: Add `Motion.move_curve`

**Files:**
- Modify: `klippy/motion.py` (after `Motion.move`, ~343)
- Test: `test/test_g5_console.py`

`move_curve` mirrors `Motion.move`'s validation + bookkeeping but (a) takes a `submit` callback for the curve-specific bridge call, (b) range-checks interior control points, (c) derives feedrate as the speed cap (the chord is meaningless for a curve).

- [ ] **Step 1: Write the failing test**

Create `test/test_g5_console.py`:

```python
import types


class FakeKin:
    def __init__(self):
        self.checked = []

    def check_move(self, move):
        self.checked.append(tuple(move.end_pos))
        # simulate a bed of +/-100 in X/Y, +/-50 in Z
        ep = move.end_pos
        if not (-100 <= ep[0] <= 100 and -100 <= ep[1] <= 100 and -50 <= ep[2] <= 50):
            raise RuntimeError("out of range")


def make_motion():
    import klippy.motion as motion

    m = motion.Motion.__new__(motion.Motion)
    m.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    m.max_velocity = 300.0
    m.max_accel = 3000.0
    m.kin = FakeKin()
    m.extruder = types.SimpleNamespace(check_move=lambda mv: None)
    m.bridge = types.SimpleNamespace(
        calls=[],
        get_last_move_time=lambda: 0.0,
        submit_bezier=lambda *a: m.bridge.calls.append(("bezier", a)),
    )
    m.mcu = None
    m._mcu_pending_end_time = 0.0
    m._fire_active_callbacks = lambda axes_d: None
    m._sync_print_time = lambda: None
    m._axis_limit = lambda axis, kind: 100.0
    return m


def test_move_curve_rejects_out_of_range_control_point():
    m = make_motion()
    # endpoints in range, but P1 control point at Y=500 bulges off the bed
    submit = lambda dx, dy, dz, de, fr: m.bridge.submit_bezier(dx, dy, dz, de, fr)
    interior = [[10.0, 500.0, 0.0], [10.0, 0.0, 0.0]]
    try:
        m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
        assert False, "expected out-of-range rejection"
    except RuntimeError as e:
        assert "out of range" in str(e)


def test_move_curve_submits_and_advances_when_in_range():
    m = make_motion()
    submit = lambda dx, dy, dz, de, fr: m.bridge.submit_bezier(dx, dy, dz, de, fr)
    interior = [[10.0, 5.0, 0.0], [10.0, -5.0, 0.0]]
    m.move_curve([20.0, 0.0, 0.0, 0.0], interior, submit, 100.0)
    assert m.bridge.calls and m.bridge.calls[0][0] == "bezier"
    assert m.commanded_pos[0] == 20.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k move_curve -v`
Expected: FAIL — `Motion` has no attribute `move_curve`.

- [ ] **Step 3: Implement**

In `klippy/motion.py`, after `Motion.move` (line ~343), add:

```python
    def move_curve(self, newpos, interior_control_points, submit, speed):
        # newpos: [x, y, z, e] absolute endpoint (already coordinate-resolved).
        # interior_control_points: list of [x, y, z] interior CPs to range-check
        #   (P0=start and the endpoint are covered by the endpoint check below).
        # submit(dx, dy, dz, de, feedrate): bridge call carrying the curve params.
        move = Move(self, self.commanded_pos, newpos, speed)
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        # Convex-hull range guard: a Bézier can bulge outside the endpoint box.
        for cp in interior_control_points:
            cp_target = [cp[0], cp[1], cp[2], self.commanded_pos[3]]
            cp_move = Move(self, self.commanded_pos, cp_target, speed)
            if cp_move.move_d and cp_move.is_kinematic_move:
                self.kin.check_move(cp_move)
        # Deltas come straight from the endpoint, NOT move.axes_d: for a
        # closed-loop curve (chord ~ 0) Move zeroes axes_d, which would drop the
        # curve's endpoint delta. The bridge rejects a genuinely zero curve
        # (arc length 0 -> ZeroDisplacement), so no early-return here.
        dx = newpos[0] - self.commanded_pos[0]
        dy = newpos[1] - self.commanded_pos[1]
        dz = newpos[2] - self.commanded_pos[2]
        de = newpos[3] - self.commanded_pos[3]
        # Feedrate is the path-speed cap; the chord length is meaningless for a
        # curve, so cap directly. The Rust optimizer re-derives per-axis limits.
        feedrate = min(speed, self.max_velocity)
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self._axis_limit("z", "max_velocity"))
        self._fire_active_callbacks([dx, dy, dz, de])
        bridge_lmt_before = self.bridge.get_last_move_time()
        submit(dx, dy, dz, de, feedrate)
        self._bump_pending_end_time(
            self.bridge.get_last_move_time() - bridge_lmt_before
        )
        self.commanded_pos[:] = list(newpos)
        self._sync_print_time()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k move_curve -v`
Expected: PASS (both).

- [ ] **Step 5: Commit**

```bash
git add klippy/motion.py test/test_g5_console.py
git commit -m "feat(klippy): Motion.move_curve with convex-hull range guard"
```

---

## Phase 5 — gcode_move cmd_G5 / cmd_G5.1 + transform gate

### Task 10: Add the active-transform gate helper

**Files:**
- Modify: `klippy/extras/gcode_move.py`
- Test: `test/test_g5_console.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_g5_console.py`:

```python
def make_gcode_move():
    import klippy.extras.gcode_move as gm

    g = gm.GCodeMove.__new__(gm.GCodeMove)
    g.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: g._toolhead
    )
    g._toolhead = types.SimpleNamespace(get_position=lambda: [0.0, 0.0, 0.0, 0.0])
    g.position_with_transform = lambda: [0.0, 0.0, 0.0, 0.0]
    return g


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


def test_transform_gate_passes_when_identity():
    g = make_gcode_move()
    # identity: transformed == raw -> no raise
    g._reject_curve_if_transform_active(FakeGcmd())


def test_transform_gate_rejects_when_active():
    g = make_gcode_move()
    g.position_with_transform = lambda: [0.0, 2.0, 0.0, 0.0]  # bent in Y
    try:
        g._reject_curve_if_transform_active(FakeGcmd())
        assert False, "expected rejection"
    except RuntimeError as e:
        assert "active move transform" in str(e)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k transform_gate -v`
Expected: FAIL — no `_reject_curve_if_transform_active`.

- [ ] **Step 3: Implement**

In `klippy/extras/gcode_move.py`, add a method to `GCodeMove` (after `cmd_G1`):

```python
    def _reject_curve_if_transform_active(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        raw = toolhead.get_position()
        transformed = self.position_with_transform()
        if any(abs(a - b) > 1e-9 for a, b in zip(raw[:3], transformed[:3])):
            raise gcmd.error(
                "G5/G5.1 not supported with an active move transform yet "
                "(skew_correction / bed_tilt / bed_mesh)"
            )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k transform_gate -v`
Expected: PASS (both).

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/gcode_move.py test/test_g5_console.py
git commit -m "feat(klippy): active-transform gate helper for curve commands"
```

### Task 11: Implement and register `cmd_G5`

**Files:**
- Modify: `klippy/extras/gcode_move.py` (handlers list ~32-46; new `cmd_G5`)
- Test: `test/test_g5_console.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_g5_console.py`:

```python
class ParamGcmd:
    def __init__(self, params):
        self._p = params

    def get_command_parameters(self):
        return self._p

    def get_commandline(self):
        return "G5 " + " ".join("%s%s" % kv for kv in self._p.items())

    def error(self, msg):
        return RuntimeError(msg)


def make_full_gcode_move():
    import klippy.extras.gcode_move as gm

    g = gm.GCodeMove.__new__(gm.GCodeMove)
    g.absolute_coord = True
    g.absolute_extrude = True
    g.base_position = [0.0, 0.0, 0.0, 0.0]
    g.last_position = [0.0, 0.0, 0.0, 0.0]
    g.extrude_factor = 1.0
    g.speed = 50.0
    g.speed_factor = 1.0 / 60.0
    g.curve_calls = []
    g._toolhead = types.SimpleNamespace(
        get_position=lambda: [0.0, 0.0, 0.0, 0.0],
        move_curve=lambda *a, **k: g.curve_calls.append((a, k)),
    )
    g.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: g._toolhead
    )
    g.position_with_transform = lambda: [0.0, 0.0, 0.0, 0.0]
    return g


def test_cmd_g5_requires_p_and_q():
    g = make_full_gcode_move()
    try:
        g.cmd_G5(ParamGcmd({"X": "10", "Y": "0", "I": "2", "J": "2"}))
        assert False
    except RuntimeError as e:
        assert "P and Q" in str(e)


def test_cmd_g5_calls_move_curve_with_interior_points():
    g = make_full_gcode_move()
    g.cmd_G5(ParamGcmd({"X": "10", "Y": "0", "I": "2", "J": "4", "P": "-3", "Q": "4"}))
    assert g.curve_calls, "move_curve should be invoked"
    (args, _kwargs) = g.curve_calls[0]
    newpos, interior, _submit, _speed = args
    assert newpos[0] == 10.0 and newpos[1] == 0.0
    # P1 = start+(I,J) = (2,4); P2 = end+(P,Q) = (7,4)
    assert interior[0][:2] == [2.0, 4.0]
    assert interior[1][:2] == [7.0, 4.0]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k cmd_g5 -v`
Expected: FAIL — no `cmd_G5`.

- [ ] **Step 3: Implement + register**

In `klippy/extras/gcode_move.py`, add `"G5"` and `"G5.1"` to the `handlers` list. Note `cmd_G5.1` is not a valid Python attr name, so register `G5.1` explicitly with `getattr(self, "cmd_G5_1")`. Replace the registration loop tail; after `gcode.register_command("G0", self.cmd_G1)` add:

```python
        gcode.register_command("G5", self.cmd_G5)
        gcode.register_command("G5.1", self.cmd_G5_1)
```

(Do **not** add `"G5"`/`"G5.1"` to the `handlers` list — the loop uses `cmd_` + name and `cmd_G5.1` is not a valid identifier; register them explicitly as above.)

Add the helper that resolves an endpoint axis exactly like `cmd_G1`, then `cmd_G5`:

```python
    def _resolve_curve_endpoint(self, gcmd, params):
        # Mirror cmd_G1's coordinate resolution for X/Y/Z/E/F; returns nothing,
        # mutates self.last_position / self.speed in place.
        try:
            for pos, axis in enumerate("XYZ"):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        self.last_position[pos] += v
                    else:
                        self.last_position[pos] = v + self.base_position[pos]
            if "E" in params:
                v = float(params["E"]) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    self.last_position[3] += v
                else:
                    self.last_position[3] = v + self.base_position[3]
            if "F" in params:
                gcode_speed = float(params["F"])
                if gcode_speed <= 0.0:
                    raise gcmd.error(
                        "Invalid speed in '%s'" % (gcmd.get_commandline(),)
                    )
                self.speed = gcode_speed * self.speed_factor
        except ValueError:
            raise gcmd.error(
                "Unable to parse curve '%s'" % (gcmd.get_commandline(),)
            )

    def cmd_G5(self, gcmd):
        self._reject_curve_if_transform_active(gcmd)
        params = gcmd.get_command_parameters()
        if "P" not in params or "Q" not in params:
            raise gcmd.error("G5 requires P and Q")
        has_i, has_j = "I" in params, "J" in params
        if has_i != has_j:
            raise gcmd.error("G5 I and J must both be present or both omitted")
        start = list(self.last_position)
        self._resolve_curve_endpoint(gcmd, params)
        try:
            p = float(params["P"])
            q = float(params["Q"])
            i = float(params["I"]) if has_i else None
            j = float(params["J"]) if has_j else None
        except ValueError:
            raise gcmd.error(
                "Unable to parse curve '%s'" % (gcmd.get_commandline(),)
            )
        end = self.last_position
        dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
        # Interior control points for the range guard (chained I/J: unknown
        # here, so reflect nothing — the bridge owns chaining; we guard P2 and,
        # when I/J are explicit, P1). P2 = end + (P,Q).
        interior = [[end[0] + p, end[1] + q, start[2] + 2.0 * dz / 3.0]]
        if i is not None:
            interior.append([start[0] + i, start[1] + j, start[2] + dz / 3.0])
        toolhead = self.printer.lookup_object("toolhead")
        submit = lambda sdx, sdy, sdz, sde, fr: self._submit_bezier_to_bridge(
            i, j, p, q, sdx, sdy, sdz, sde, fr
        )
        toolhead.move_curve(list(self.last_position), interior, submit, self.speed)
```

The `submit` closure needs the bridge's `submit_bezier`. Add a tiny accessor on `GCodeMove` that reaches the motion bridge. Add:

```python
    def _submit_bezier_to_bridge(self, i, j, p, q, dx, dy, dz, de, fr):
        motion = self.printer.lookup_object("motion")
        motion.bridge.submit_bezier(i, j, p, q, dx, dy, dz, de, fr)
```

> Note for the implementer: the test stubs `toolhead.move_curve`, so the `submit` closure is not invoked in the unit test. The closure is exercised end-to-end in Task 14. Keep `i, j` as `None` when omitted — the Rust `submit_bezier` signature takes `Option<f64>`, which maps from Python `None`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k cmd_g5 -v`
Expected: PASS (both).

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/gcode_move.py test/test_g5_console.py
git commit -m "feat(klippy): cmd_G5 — parse, validate, range-guard, dispatch to move_curve"
```

### Task 12: Implement and register `cmd_G5_1`

**Files:**
- Modify: `klippy/extras/gcode_move.py`
- Test: `test/test_g5_console.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_g5_console.py`:

```python
def test_cmd_g5_1_requires_i_or_j():
    g = make_full_gcode_move()
    try:
        g.cmd_G5_1(ParamGcmd({"X": "10", "Y": "0"}))
        assert False
    except RuntimeError as e:
        assert "I and/or J" in str(e)


def test_cmd_g5_1_dispatches_quadratic():
    g = make_full_gcode_move()
    g.cmd_G5_1(ParamGcmd({"X": "10", "Y": "0", "I": "5", "J": "5"}))
    assert g.curve_calls
    (args, _kwargs) = g.curve_calls[0]
    newpos, interior, _submit, _speed = args
    # single quadratic control point Q1 = start + (I,J) = (5,5)
    assert interior[0][:2] == [5.0, 5.0]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k g5_1 -v`
Expected: FAIL — no `cmd_G5_1`.

- [ ] **Step 3: Implement**

In `klippy/extras/gcode_move.py`, add:

```python
    def cmd_G5_1(self, gcmd):
        self._reject_curve_if_transform_active(gcmd)
        params = gcmd.get_command_parameters()
        if "I" not in params and "J" not in params:
            raise gcmd.error("G5.1 requires I and/or J")
        start = list(self.last_position)
        self._resolve_curve_endpoint(gcmd, params)
        try:
            i = float(params.get("I", 0.0))
            j = float(params.get("J", 0.0))
        except ValueError:
            raise gcmd.error(
                "Unable to parse curve '%s'" % (gcmd.get_commandline(),)
            )
        end = self.last_position
        dz = end[2] - start[2]
        interior = [[start[0] + i, start[1] + j, start[2] + dz / 2.0]]
        submit = lambda sdx, sdy, sdz, sde, fr: self._submit_quadratic_to_bridge(
            i, j, sdx, sdy, sdz, sde, fr
        )
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.move_curve(list(self.last_position), interior, submit, self.speed)

    def _submit_quadratic_to_bridge(self, i, j, dx, dy, dz, de, fr):
        motion = self.printer.lookup_object("motion")
        motion.bridge.submit_quadratic(i, j, dx, dy, dz, de, fr)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k g5_1 -v`
Expected: PASS (both).

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/gcode_move.py test/test_g5_console.py
git commit -m "feat(klippy): cmd_G5_1 — quadratic, exact elevation in the bridge"
```

---

## Phase 6 — bed mesh activation gate

### Task 13: Gate `set_mesh` against activation

**Files:**
- Modify: `klippy/extras/bed_mesh.py` (`set_mesh`, ~190)
- Test: `test/test_g5_console.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_g5_console.py`:

```python
def test_bed_mesh_activation_is_gated():
    import klippy.extras.bed_mesh as bm

    bedmesh = bm.BedMesh.__new__(bm.BedMesh)
    bedmesh.z_mesh = None
    raised = {}

    class G:
        def error(self, msg):
            raised["msg"] = msg
            return RuntimeError(msg)

    bedmesh.gcode = G()
    # Activating any non-None mesh must raise.
    try:
        bedmesh.set_mesh(object())
        assert False, "expected activation to be gated"
    except RuntimeError:
        assert "not supported under the new motion planner" in raised["msg"]
    # Clearing (None) must be allowed (no raise from the gate).
    raised.clear()
    try:
        bedmesh.set_mesh(None)
    except RuntimeError:
        pass  # downstream of the gate may still touch other state; the gate
        # itself must not have fired:
    assert "not supported" not in raised.get("msg", "")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k bed_mesh -v`
Expected: FAIL — `set_mesh` activates without raising.

- [ ] **Step 3: Implement the gate**

In `klippy/extras/bed_mesh.py`, at the very top of `set_mesh` (before the existing `if mesh is not None and self.fade_end ...`), add:

```python
    def set_mesh(self, mesh):
        if mesh is not None:
            raise self.gcode.error(
                "bed_mesh: activating a mesh is not supported under the new "
                "motion planner yet (the surface-following transform layer has "
                "not been ported). BED_MESH_CLEAR is allowed."
            )
```

(Keep the rest of the original `set_mesh` body below this guard — it now only runs for the `mesh is None` / clear path.)

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && python -m pytest test/test_g5_console.py -k bed_mesh -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/bed_mesh.py test/test_g5_console.py
git commit -m "feat(klippy): fail-loud gate on bed-mesh activation (unsupported under new planner)"
```

---

## Phase 7 — cusp experiment + end-to-end

### Task 14: Cusp / adversarial-curve experiment (drives the cusp decision)

**Files:**
- Test: `rust/motion-engine/src/classify/tests.rs` (or a dedicated test under `rust/trajectory/`)

This task is an **experiment**, not a feature: it feeds degenerate polygons through the live solver and records what happens. The outcome decides whether cusp-splitting is needed (spec §"Cusps").

- [ ] **Step 1: Write the experiment test**

Append to `rust/motion-engine/src/classify/tests.rs`:

```rust
#[test]
fn experiment_cusp_and_near_cusp_classification_is_finite() {
    // Exact cusp: P1 == P0 (zero start tangent). Near-cusp: tiny start leg.
    // High curvature: control points fold sharply. We only assert classify
    // itself stays finite; the solver behavior is recorded by running the
    // full pipeline in an integration test (Step 3).
    let exact = classify_bezier([0.0, 0.0, 0.0], 0.0, 0.0, -5.0, 0.0, 5.0, 0.0, 0.0, &[], 30.0);
    let near = classify_bezier([0.0, 0.0, 0.0], 1e-7, 0.0, -5.0, 0.0, 5.0, 0.0, 0.0, &[], 30.0);
    for m in [&exact, &near] {
        if let Ok(mv) = m {
            assert!(mv.distance_mm.is_finite(), "arc length must be finite");
            assert!(mv.distance_mm >= 0.0);
        }
    }
}
```

- [ ] **Step 2: Run it**

Run: `cd rust && cargo nextest run -p motion-engine -E 'test(experiment_cusp)'`
Expected: PASS (classify is finite). Record the result.

- [ ] **Step 3: Run the cusp through the real solver and record the outcome**

Run (find the live planning entry; the trajectory crate's batch planner is `trajectory::plan_velocity` per spec):
`cd rust && grep -rn "pub fn plan_velocity" trajectory/src/plan_velocity.rs`

Write a `#[test]` in `rust/trajectory/tests/` (or the nearest existing integration test module) that builds a single cusp `CubicSegment` (control points `[[0,0,0],[0,0,0],[-5,0,0],[5,0,0]]`, feedrate 30) and calls the live velocity planner. Record in the spec's "Cusps" section which outcome occurred:
- clean stop (v→0 at the cusp) → **no further work**; mark cusps supported.
- finite-but-suboptimal → file a follow-up for split-at-cusp.
- NaN / stall / SLP-restoration-cap → implement the **interim fail-loud** guard now (Step 4).

- [ ] **Step 4 (conditional): interim fail-loud guard**

ONLY if Step 3 showed NaN/stall: add a `min|x'(t)|` check in `classify_curve` (sample the cubic's derivative at, e.g., 17 points; if the min speed is below `1e-6 * arc_len`, return a new `ClassifyError::DegenerateCusp`). Wire `ClassifyError::DegenerateCusp` to a clear pyo3 message `"degenerate G5 control polygon (cusp / zero-velocity) not supported"`. Add a test asserting the exact-cusp `classify_bezier` returns that error.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "test: cusp/near-cusp experiment recording solver behavior (drives cusp decision)"
```

### Task 15: End-to-end gate green + docs

**Files:**
- None new — run the full gate.

- [ ] **Step 1: Run the Rust suite**

Run: `cd rust && cargo nextest run`
Expected: all green.

- [ ] **Step 2: Run clippy + fmt**

Run: `cd rust && cargo clippy --all-targets -- -D warnings && cargo fmt --all --check`
Expected: clean.

- [ ] **Step 3: Run the Python tests**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && ./scripts/ci.sh py`
Expected: green (includes `test/test_g5_console.py`).

- [ ] **Step 4: Run the full quick gate**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && ./scripts/ci.sh quick`
Expected: green.

- [ ] **Step 5: Commit any fmt/lint fixups**

```bash
git add -A
git commit -m "chore: gate green for live G5/G5.1 support"
```

---

## Notes for the implementer

- **The optimizer is unchanged.** G5/G5.1 produce a `CubicSegment` exactly like G1; everything downstream (SOCP/SLP, shaping, MCU) is untouched. If a curve fails to plan, the bug is in control-point assembly or the segment, not the optimizer.
- **Corners are full stops today** (`corner_caps = vec![0.0]`); junction deviation is unimplemented. A chained (smooth) G5 flows; a G5↔G1 corner stops — same as any corner. Do not "fix" this here.
- **Chaining is XY-only.** `P1.z` always comes from the linear assembly (`start.z + dz/3`), never a 3D reflection. The bridge stores `(P, Q)` and uses `(-P, -Q)` for omitted `I/J`.
- **Extruder ratio uses arc length** (`classify_curve` divides by `path_arc_length`), never the chord — otherwise curves over-extrude.
- **`G5.1` registration** uses `register_command("G5.1", self.cmd_G5_1)` — the Python dispatcher keeps `G5.1` intact as the command key (verified: `args_r` splits on letters, so `5.1` stays with the number token).

**Two known v1 limitations (acceptable, document in the spec's Future-work if not already):**
- **Chained-G5 reflected `P1` is not range-checked.** For a chained `G5` (omitted `I/J`), `P1` is reflected in the bridge from the previous segment's `(P,Q)` — Python doesn't hold that state, so it can't add `P1` to the convex-hull guard. The endpoint and `P2` are still checked. Chaining is for smooth continuations where `P1` sits near the path, so the exposure is small; revisit if it bites.
- **Closed-loop curve (start == endpoint) callbacks.** `_fire_active_callbacks([dx,dy,dz,de])` sees zero net delta for a loop, so `active_rails` may not power the XY servos even though the toolhead traverses the loop. Rare for console use; if it matters, fire callbacks from the control-point spread instead of the net delta.
