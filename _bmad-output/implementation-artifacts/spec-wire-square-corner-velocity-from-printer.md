---
title: 'Wire square_corner_velocity from [printer] config'
type: 'bugfix'
created: '2026-06-19'
status: 'done'
context: []
baseline_commit: '90ee2e1cce2f74ad60c80efeb320abed29858507'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Square corner velocity (SCV) is hardcoded to the engine default `DEFAULT_SQUARE_CORNER_VELOCITY_MM_S = 5.0` in the Rust planner and never read from config. Worse, `square_corner_velocity` sits in `UNSUPPORTED_LIMIT_KEYS`, so a user who sets it in `[printer]` gets a hard config error. Users cannot tune cornering the way mainline allows.

**Approach:** Read `square_corner_velocity` from the `[printer]` section in `klippy/motion.py` (mainline semantics: default `5.0`, `minval=0.0`), thread it through the existing `[printer]`-limits cutover channel — the `cartesian_limits` tuple passed to `init_planner` — into `CartesianLimits`, and consume that value in the two live SCV sites instead of the hardcoded constant.

## Boundaries & Constraints

**Always:** Mirror mainline's config contract exactly — key name `square_corner_velocity`, default `5.0`, `minval=0.0`. Follow the existing `[printer]`-limits cutover pattern (the `cartesian_limits` tuple → `CartesianLimits` struct) — do not invent a new channel. SCV must be finite and `>= 0.0` (matches `geometry::VelocityLimits::check`). Default behavior unchanged when the key is absent (still 5 mm/s).

**Ask First:** Any change to the junction-deviation math or to how SCV is consumed in the geometry pipeline (out of scope — this is pure config wiring).

**Never:** Do not touch the dead `path_velocity_limits()` path or the `[limit <name>]` sections. Do not remove `DEFAULT_SQUARE_CORNER_VELOCITY_MM_S` (still the default source + used by tests). Do not change the `submit_move` or stream-planner algorithms beyond swapping the SCV source.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Key absent | `[printer]` has no `square_corner_velocity` | Planner uses 5.0 mm/s (unchanged behavior) | N/A |
| Key set | `[printer] square_corner_velocity: 8` | Planner corners with SCV = 8 mm/s; reported in status | N/A |
| Zero | `square_corner_velocity: 0` | Accepted; junction deviation collapses to 0 (mainline parity) | N/A |
| Negative | `square_corner_velocity: -1` | Config rejected at load | Klippy `config.error` via `minval=0.0` |

</frozen-after-approval>

## Code Map

- `klippy/motion.py` -- `UNSUPPORTED_LIMIT_KEYS` (527) rejects the key; `_read_limits` (608) sets `self.square_corner_velocity = 0.0` hardcoded (637); `_init_planner` (821) builds the 5-tuple `cartesian_limits` (827-833); status dict already reports `self.square_corner_velocity` (295).
- `klippy/motion_engine.py` -- `init_planner` wrapper (360) passes `cartesian_limits` through opaquely; no signature change needed.
- `rust/motion-engine/src/bridge.rs` -- `init_planner` PyO3 signature `cartesian_limits: (f64, f64, f64, f64, f64)` (2530); destructure + build `CartesianLimits` (2586-2593); stream-planner SCV site (3288-3293); `submit_move` SCV site (3352-3357).
- `rust/motion-engine/src/config.rs` -- `CartesianLimits` struct (416), `Default` (424), `validate` (437); `DEFAULT_SQUARE_CORNER_VELOCITY_MM_S` (507).

## Tasks & Acceptance

**Execution:**
- [x] `klippy/motion.py` -- Remove `"square_corner_velocity"` from `UNSUPPORTED_LIMIT_KEYS`; in `_read_limits` replace `self.square_corner_velocity = 0.0` with `config.getfloat("square_corner_velocity", 5.0, minval=0.0)`; add it as the 6th element of the `cartesian_limits` tuple in `_init_planner`.
- [x] `rust/motion-engine/src/bridge.rs` -- Extend `init_planner` tuple type to `(f64, f64, f64, f64, f64, f64)`, destructure the 6th value, set `cartesian.square_corner_velocity`; replace both `config::DEFAULT_SQUARE_CORNER_VELOCITY_MM_S` consumers (stream init + `submit_move`) with `cart.square_corner_velocity`.
- [x] `rust/motion-engine/src/config.rs` -- Add `square_corner_velocity: f64` to `CartesianLimits`; default it to `DEFAULT_SQUARE_CORNER_VELOCITY_MM_S`; extend `validate()` to require finite and `>= 0.0` with a clear `[printer] square_corner_velocity` error message.
- [x] `rust/motion-engine/src/config/tests.rs` -- Unit-test `CartesianLimits::validate` accepts SCV `0.0` and a positive value, rejects negative/NaN; verify default equals `DEFAULT_SQUARE_CORNER_VELOCITY_MM_S`.
- [x] `test/test_motion_topology.py` -- Set `square_corner_velocity` on the mock and assert the configured value reaches the `cartesian_limits` tuple as its 6th element (toolhead-shim fixtures already tolerate the change).

**Acceptance Criteria:**
- Given a `[printer]` config with `square_corner_velocity: 8`, when the planner initializes, then the stream planner and `submit_move` build `VelocityLimits` with `square_corner_velocity_mm_s == 8.0` and status reports `8.0`.
- Given no `square_corner_velocity` key, when the planner initializes, then SCV is 5.0 (byte-for-byte unchanged trajectory vs. before this change).
- Given `square_corner_velocity: -1`, when Klippy loads the config, then it raises a config error and does not start the planner.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine` -- expected: green, including new `CartesianLimits` validate tests.
- `cd rust && cargo clippy --all-targets -- -D warnings && cargo fmt --all --check` -- expected: clean.
- `./scripts/ci.sh py` -- expected: host tests green (motion.py touched).
- `./scripts/ci.sh quick` -- expected: full quick gate green before PR.

## Suggested Review Order

**Config read (entry point)**

- Reads the [printer] key with mainline semantics — default 5.0, minval 0.0.
  [`motion.py:636`](../../klippy/motion.py#L636)

- The key is no longer rejected at load; this removal is what unblocks the read above.
  [`motion.py:527`](../../klippy/motion.py#L527)

**Threading SCV through the cutover channel**

- Appends SCV as the 6th element of the existing `cartesian_limits` tuple — no new channel.
  [`motion.py:834`](../../klippy/motion.py#L834)

- The PyO3 boundary widens to a 6-tuple; destructure + populate the new field.
  [`bridge.rs:2530`](../../rust/motion-engine/src/bridge.rs#L2530)

**Consuming the configured value**

- Stream-planner init now uses the configured SCV instead of the constant.
  [`bridge.rs:3299`](../../rust/motion-engine/src/bridge.rs#L3299)

- Per-move limits in `submit_move` pull SCV from the locked config alongside v/a.
  [`bridge.rs:3358`](../../rust/motion-engine/src/bridge.rs#L3358)

**Struct + validation**

- New `square_corner_velocity` field, defaulted to the const so absent-key behavior is unchanged.
  [`config.rs:422`](../../rust/motion-engine/src/config.rs#L422)

- Validation accepts `>= 0.0` (mainline parity), rejecting negative/non-finite.
  [`config.rs:450`](../../rust/motion-engine/src/config.rs#L450)

**Tests**

- Validate accepts 0.0 and positive, rejects negative/NaN; default equals const.
  [`config/tests.rs:11`](../../rust/motion-engine/src/config/tests.rs#L11)

- Host-side assertion that a configured SCV reaches the `cartesian_limits` tuple.
  [`test_motion_topology.py:211`](../../test/test_motion_topology.py#L211)
