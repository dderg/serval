# Live G5 / G5.1 (cubic & quadratic Bézier) support — design

Date: 2026-06-14
Status: approved for planning

## Goal

Make `G5` and `G5.1` typed at the console (or emitted from a macro) actually
move the toolhead along the curve they describe. Today both hit the default
handler and are rejected as `Unknown command`.

This is a **plumbing** task, not a motion-math task. The planner is already
Bézier-native and runs the full time-optimal optimizer on every live move; the
curve's control points only need to survive the trip from the g-code line into
the planner's segment.

## Scope

In scope:

- `cmd_G5` — cubic Bézier: endpoint `X Y`, first control-point offset `I J`,
  second control-point offset `P Q`, optional `Z E F`.
- `cmd_G5.1` — quadratic Bézier: endpoint `X Y`, single control-point offset
  `I J`, optional `Z E F`. Lifted to cubic by **exact** degree elevation.
- Smooth `G5` chaining: a follow-on `G5` may omit `I J`, reflecting the
  previous segment's exit control point for C1 continuity.
- A new geometric bridge entry point carrying control points.
- Extraction of the one geometry primitive the live engine borrows from
  `compat` into `geometry`, severing the planner's dependency on `compat`.
- Convex-hull range checking (the curve, not just the endpoint).
- The extruder ratio computed against true arc length (not the chord).
- Fail-loud move-transform gates: bed-mesh activation, and an active-transform
  curve-time gate.

Explicitly out of scope (deferred, not prerequisites):

- `G5.2` / `G5.3` (NURBS blocks). NURBS are rational and arbitrary-order —
  exactly what our core forbids (see "Geometry mapping"). They are flattened
  to a `G5` cubic stream by the slicer/preprocessor, like arcs.
- Live `G2` / `G3`. Arc-to-Bézier is the slicer's job; its math stays in
  `compat`, which is moving to the slicer.
- Any bed-mesh / skew / bed-tilt interaction with moves. The move-transform
  layer needs rework for G1 too; that is a separate project. Until then bed
  mesh activation is gated off and curves are gated against any active
  transform (see "Move-transform gates").

## The G5 standard we follow

`G5` is defined consistently across firmwares; we follow the **LinuxCNC**
family because it is the complete one (it also defines `G5.1` quadratic and
`G5.2/3` NURBS), and adhering to it gives **Marlin `G5` compatibility for
free** — the cubic semantics and chaining rule are identical.

Verbatim semantics (both dialects agree):

- `X Y` = destination (the last control point).
- `I J` = "offset from the start point to the first control point."
- `P Q` = "offset from the end point to the second control point."
- "`P` and `Q` are required (otherwise you just get a linear movement)."
- "`I` and `J` are required for the first `G5` command in a series. For
  subsequent `G5` commands, either both `I` and `J` must be specified, or
  neither." When omitted: "the starting direction of the cubic will
  automatically match the ending direction of the previous cubic (as if `I`
  and `J` are the negation of the previous `P` and `Q`)."

`G5.1` (LinuxCNC quadratic spline): `I J` = offset from start to the single
control point; "not specifying I or J gives zero offset … so one or both must
be given." No `P Q`; no chaining.

Sources: Marlin G5 (`MarlinDocumentation/_gcode/G005.md`,
`marlinfw.org/docs/gcode/G005.html`); LinuxCNC G-code reference
(`linuxcnc.org/docs/html/gcode/g-code.html`).

### Geometry mapping (why G5/G5.1 are native and G5.2/3 are not)

Our core primitive is a **uniform cubic (degree-3), non-rational
(polynomial) Bézier** (CLAUDE.md). Two words decide everything: *cubic* and
*non-rational*.

| Code | Curve | Maps to our cubic Bézier? |
|---|---|---|
| `G5` | cubic Bézier | Exact, direct — it *is* the primitive. |
| `G5.1` | quadratic Bézier | Exact, via closed-form degree elevation. |
| `G5.2/3` | NURBS (rational, arbitrary order) | Not exact in general — rational weights + order>3 force bounded-error flattening; a slicer/preprocessor job, like arcs. |

A quadratic Bézier is *identically* a cubic Bézier — degree elevation is exact,
no tolerance. A NURBS with non-unit weights represents conics (circles) that no
polynomial Bézier equals exactly; that approximation is the same reason arcs
aren't native, and it belongs in the slicer.

## Background: how a live move works today

`G1 X100 F3000` flows:

```
gcode_move.cmd_G1            # owns g-code coordinate semantics
  -> Motion.move(newpos,spd) # validates (kin/extruder check_move), computes deltas
  -> bridge.submit_move(dx,dy,dz,de,F)        # pyo3 seam; geometry only
  -> classify_and_build(start,dx,dy,dz,...)   # bridge knows `start`
  -> to_collinear_bezier(start,end)           # a degenerate (straight) cubic
  -> append_and_replan -> plan_velocity       # full SOCP + SLP optimizer
       -> temporal::multi::plan_batch              (smooth runs = one profile)
  -> emit_shaped -> Bernstein pieces -> MCU ring
  -> ISR ~40 kHz: eval_horner -> step times
```

There is no trapezoidal profiler. The planner's native primitive is a cubic
Bézier; a G1 is just the special case where the four control points are
collinear, fed to the same optimizer. **G5/G5.1 reach the identical optimizer —
the only difference is that their control points are not collinear.**

Junction handling (inherited, not G5-specific): segments are partitioned into
chains by tangent agreement. A *smooth* junction is solved inside one
continuous chain profile (flows at speed); a *corner* (tangent disagreement,
including a Z-slope change) is a **full stop** — `corner_caps` is currently
`vec![0.0]` (`temporal/src/multi/mod.rs:239`), so junction deviation /
cornering-at-speed is stubbed, not yet implemented. G5 inherits this verbatim
(a chained, tangent-continuous G5 flows; a G5↔G1 corner stops, exactly like a
G1↔G1 corner today) and gains cornering automatically when it lands.

## Architecture: where each responsibility lives

The pyo3 seam already separates the two jobs, and the split does not move:

- **Python (`gcode_move.py`, `motion.py`) owns g-code semantics and
  coordinate state** — absolute/relative (`G90/G91`), `base_position`
  (`G92`), gcode offset, extrude factor (`M221`), absolute extrude
  (`M82/M83`), speed factor (`M220`/`F`), `SAVE`/`RESTORE_GCODE_STATE`, and
  the per-move validation (`kin.check_move`, `extruder.check_move`). Shared by
  every motion command; reused by G5/G5.1 unchanged.
- **Rust (`motion-bridge`) owns curve geometry** — turning control points into
  a `CubicSegment`, the quadratic→cubic elevation, holding the chaining state,
  and feeding the optimizer. The bridge already tracks the running `start`.

Rejected alternative: moving motion-gcode parsing into Rust. The coordinate
state and transforms are Python objects shared by many commands; relocating
them across FFI is large, invasive, and buys nothing.

### Plain-English summary

Python stays the translator and bookkeeper: it knows where the nozzle is,
whether numbers are absolute or relative, what the speed override is. It does
that for the curve's endpoint exactly as it already does for a line, reads the
extra "steering" numbers off the command, and hands plain numbers to Rust.
Python never learns what a Bézier is; Rust never learns g-code state. The fence
stays where it is.

## Compat extraction

`compat` is a g-code -> g-code preprocessor (the offline `kalico-compat`
binary, bound for the slicer). The planner and g-code parser must not depend
on it.

Audit: the live engine's entire compat footprint is one function,
`to_collinear_bezier` (`compat/src/collinear.rs:20`), called only from
`motion-bridge/src/classify.rs`. Compat's own logic never calls it (it uses
`to_collinear_g5`, the text emitter). The control-point primitive is simply
mis-filed in a g-code-text crate.

Changes:

1. Move `to_collinear_bezier` (and its two unit tests) from `compat` into
   `geometry`; update `classify.rs` to import from `geometry`.
2. Remove `compat` from `motion-bridge/Cargo.toml`. The planner then has zero
   compat edges.
3. Add the G5 control-point builder and the exact quadratic→cubic elevation
   alongside it in `geometry`.
4. Everything else stays in `compat` (`converter`, `fitter`, `arc`,
   `hausdorff`, `degree_elev`, `g5_canon`, `to_collinear_g5`, `modal`,
   `corner`, `run`, `emit`).

Dependency direction stays one-way: a preprocessor may depend on `geometry`;
`geometry` never depends on `compat`.

## Components and data flow

```
cmd_G5 / cmd_G5.1 (gcode_move.py)
  parse X Y Z E F (+ I J P Q for G5; I J for G5.1)
  resolve ENDPOINT (X Y Z E) through existing coordinate state, like cmd_G1
  forward control-point offsets raw (they are deltas; no base_position)
  -> Motion.move_curve(newpos, <ctrl offsets>, speed)   # sibling of Motion.move
       validate endpoint via kin/extruder check_move; compute dx,dy,dz,de
  -> bridge.submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)   # G5
     bridge.submit_quadratic(i, j, dx, dy, dz, de, feedrate)      # G5.1
  -> Rust pyo3 -> classify_bezier(...) assembles control points -> CubicSegment
  -> (unchanged) append_and_replan -> optimizer -> MCU
```

Dispatch: both `G5` and `G5.1` register in `gcode_move.py`
(`register_command("G5", ...)`, `register_command("G5.1", ...)`); the Python
dispatcher keeps the `.1` minor intact in the command key.

`Motion.move_curve` must replicate `Motion.move`'s side effects, not just the
submit: `_fire_active_callbacks` (powers servo/follower axes — a G5 that skips
it faults a parked servo), the `commanded_pos` update, the pending-end-time
bump, and `_sync_print_time`.

### The handoff

`submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)` — `i, j` are `Option`
(`None` => chained; reflect previous exit control point). `submit_quadratic(i,
j, dx, dy, dz, de, feedrate)` for `G5.1`. Python resolves everything into
machine-frame numbers; it never constructs a control point as geometry.

### Control-point assembly (Rust, in `geometry` + `classify`)

XY-plane curves with Z interpolated linearly. With `start = P0` and
`end = start + (dx, dy, dz)`:

- **G5 (cubic):** `P0 = start`, `P1 = (start.x+I, start.y+J, start.z+dz/3)`,
  `P2 = (end.x+P, end.y+Q, start.z+2dz/3)`, `P3 = end`.
- **G5.1 (quadratic, elevated exactly to cubic):** quadratic control points
  `Q0 = start`, `Q1 = (start.x+I, start.y+J, start.z+dz/2)`, `Q2 = end`; then
  `C0 = Q0`, `C1 = Q0 + 2/3(Q1-Q0)`, `C2 = Q2 + 2/3(Q1-Q2)`, `C3 = Q2`.

`I J P Q` are XY offsets; the Z thirds/half keep Z linear across the segment.

### Distance: true arc length, not chord

`distance_mm` feeds `nominal_duration`, which `submit_move` uses to
**provisionally** advance the bridge's `last_move_time` before the optimizer
runs (`planner.rs:204`); the planner thread later **rectifies** it to the true
trajectory time (`planner.rs:524`, `rectify_last_move_time`). The host reads
`get_last_move_time()` synchronously right after submit.

For a straight line the chord *is* the path length, so it was exact for free.
For a curve the chord underestimates, handing the scheduler a known-low number
every time. Compute the **true arc length**: reuse the `geometry`/`nurbs`
arc-length facility (`nurbs::arc_length::path_arc_length`, already used by the
follower pipeline) — it exists; otherwise an 8-point Gauss-Legendre quadrature
on the cubic (effectively exact, a handful of flops).

### Extruder ratio must use arc length, not chord

The downstream extrusion machinery is already correct for curves: the follower
pipeline computes `ratio = delta / path_arc_length` (3D arc length,
`geometry/src/pipeline.rs:260`) and drives E by the integrated path speed
`E(t) = E_start + ratio·∫√(ẋ²+ẏ²+ż²)dτ` (`emit_shaped.rs:330`), tested on a G5
helix. **But the live `classify.rs` builds the follower ratio as
`de / spatial_distance` (the chord)** — correct only because G1 is collinear.
The new G5 path **must** build its followers via the arc-length
`classify_followers` (`geometry/src/pipeline.rs`) / `path_arc_length` — **not**
the chord-based ratio in `classify.rs` — or a curve over-extrudes by exactly
the arc/chord ratio. This is the one extrusion-correctness requirement of the
whole task; the rest is already done.

### Chaining (smooth G5)

`G5`-only (a `G5.1` requires ≥1 of `I/J`, never omits, and does not
participate). State lives in the bridge, where the running position lives:

- After building any `G5` segment, store its `(P, Q)` offset pair.
- On `submit_bezier` with `i,j = None`: set `(I, J) = (-P_prev, -Q_prev)`.
  This reflects in **XY only** (`I/J/P/Q` are XY offsets). `P1.z` always comes
  from the linear-Z assembly (`start.z + dz/3`) — **never** the 3D form
  `P1 = 2*P0 - prev_P2`, which would corrupt linear Z. Standard G5 is planar
  (XY-only), so the XY reflection is verbatim the standard; our linear Z is a
  planar-G5 extension (like helical arcs), outside the standard's scope.
- Any non-`G5` move (`submit_move`, `submit_quadratic`, dwell, …) **clears**
  the stored offsets; chaining only bridges consecutive `G5`s.

## Endpoint & parameter validation

Endpoint axes follow `cmd_G1`: `X`/`Y`/`Z` optional, omitted = keep current.
The curve is defined by its control points, so a move that progresses only in
one axis while bulging via control points is valid and must not be rejected.
The real constraints come from the standard:

- `G5`: `P` and `Q` required. Missing -> error (`G5 requires P and Q`).
- `G5`: `I` present without `J` or vice-versa -> error
  (`G5 I and J must both be present or both omitted`).
- `G5`: `I J` omitted with no previous `G5` in the chain -> error
  (`G5 without I J must follow another G5`). Validated in the **bridge**
  (only it holds the chain state); surfaced as a pyo3 error -> `gcmd.error`.
- `G5.1`: at least one of `I`/`J` required -> error
  (`G5.1 requires I and/or J`).
- Zero-displacement / zero-length curve -> existing
  `ClassifyError::ZeroDisplacement`.

Presence/pairing checks live in Python (cheap, has `gcmd.error`); the chain
availability check lives in the bridge. No silent recovery; every rejected
command raises with a clear message.

## Range checking — the curve, not just the endpoint

`kin.check_move` validates the **endpoint** against axis limits. For a straight
line that suffices (every point lies in the start/end bounding box). **A Bézier
does not** — control points can push the curve far outside the endpoints (a move
between two in-range points can bulge off the bed). Endpoint-only validation
would let the toolhead crash.

A Bézier is contained in the **convex hull of its four control points**, so
validating all four 3D control points is a conservative, sufficient bound.
`cmd_G5`/`move_curve` computes the control points and runs each through the
**kinematic** range check (`kin.check_move`-style, as if each were a move
endpoint) — not a per-axis min/max box, which would be wrong for delta/corexy
reachability. Fail loud on any unreachable control point.

This complements the endpoint check. `check_move` on the (chord-based) `Move`
still does the useful work on the endpoint — unhomed check, endpoint
reachability, and the feedrate cap (only a cap; the Rust optimizer re-derives
true limits). The chord `Move`'s direction and length are otherwise unused for
a curve; the convex-hull pass is what guards the bulge.

## Cusps / degenerate control polygons

A console user or buggy macro can submit a G5 whose control points fold back on
themselves, creating a **cusp** — a point where the geometric tangent `|x'(t)|`
passes through zero. Physically a cusp is a mandatory full stop (the toolhead
must decelerate to zero and reverse); that is a valid, in-spec motion. The
difficulty is **numerical**, not physical: the time-optimal solver measures
path speed by dividing by `|x'(t)|`, which is a divide-by-zero at the cusp. The
solver has only ever been fed collinear (regular) curves, so its behavior here
is **untested**.

Plan: drive the decision by evidence, not assumption.

1. **Experiment first** — sweep adversarial polygons through the live solver:
   an exact cusp (fold-back), **near-cusps** (`|x'|` tiny but nonzero —
   numerically worse: huge-but-finite curvature, ill-conditioned), and
   high-curvature curves. Record the outcome: clean stop, finite-but-suboptimal,
   or NaN/stall/SLP-restoration-cap.
2. If the solver produces a clean stop → cusps already work; no special code.
3. If not → **split the segment at the cusp** and impose a `v = 0` boundary
   there (the physically-correct, SOTA handling — it hands the solver the stop
   instead of a `0/0`). Detection: `min|x'(t)|` below threshold.
4. **Interim stopgap** only if split-at-cusp is deferred: detect and fail loud
   (`degenerate G5 control polygon (cusp / zero-velocity) not supported`). Safe
   because no slicer emits cusps.

## Move-transform gates

`set_move_transform` has seven callers (`bed_mesh`, `skew_correction`,
`bed_tilt`, `z_thermal_adjust`, `exclude_object`, `mixing_extruder`,
`tuning_tower`) that warp moves *above* the bridge. None is ported to the new
planner. Two distinct failure modes need two distinct gates.

### Bed mesh — broken for all moves (gate at activation)

Bed mesh follows the surface by **splitting** a move into short pieces
(`MoveSplitter`, `split_delta_z = 0.025`), unvalidated against the new planner —
so it is unsupported for lines *and* curves. Gate at `bed_mesh.py`
`set_mesh(self, mesh)`, the single activation chokepoint (both profile `LOAD`
and calibration's `probe_finalize` funnel through it). When `mesh is not None`,
raise before `self.z_mesh` is assigned:

```
bed_mesh: activating a mesh is not supported under the new motion planner
yet (the surface-following transform layer has not been ported).
BED_MESH_CLEAR is allowed.
```

By design: `BED_MESH_PROFILE LOAD` and `BED_MESH_CALIBRATE` (which auto-applies)
are rejected; `set_mesh(None)` (`BED_MESH_CLEAR`) is allowed.

### Affine transforms — fine for G1, wrong for curves (gate at curve time)

`skew_correction`/`bed_tilt` work for G1 (they transform the endpoint) but a
curve **bypasses** them → silently wrong geometry. Gate curves, not lines.

A blanket "`move_transform is not None` → reject" over-fires: `bed_mesh`
installs its transform at load even with no active mesh, and most printers carry
`[bed_mesh]` in config — that would reject every G5. So the curve-time gate must
detect an *actually-active* (non-identity) transform: in `cmd_G5`/`cmd_G5.1`,
probe `position_with_transform()` against the raw toolhead position; if they
differ beyond epsilon, a transform is bending coordinates → raise
`G5 not supported with an active move transform yet`. (The single-point probe's
blind spot — a mesh that is identity at the probe point — is covered by the
bed-mesh activation gate above, which keeps an inactive mesh a true no-op.)

Both gates are removed when the move-transform layer is reworked for the new
planner (separate project).

## Testing

Rust (`cargo nextest run`):

- `geometry`: G5 control-point assembly; G5.1 quadratic→cubic elevation is
  exact (sample both forms, compare); Z thirds/half give linear Z; arc length
  vs chord on a known curve; relocated `to_collinear_bezier` keeps its tests.
- `motion-bridge` `classify`: builds a non-degenerate `CubicSegment` from
  control points; chaining `(I,J) = -(P_prev,Q_prev)`; chain cleared by an
  intervening `submit_move`/`submit_quadratic`; chain-without-prior-`G5`
  returns an error.
- **Extruder ratio** uses arc length, not chord: a curved G5 with a given `E`
  delivers exactly that `E` over the true path (assert against
  `path_arc_length`, not the chord — the chord would over-extrude).
- **Cusp experiment**: a fold-back control polygon through the live solver;
  record outcome (clean stop / suboptimal / NaN-stall). Drives the cusp
  decision.

Python (`./scripts/ci.sh py`):

- `cmd_G5` / `cmd_G5.1` parsing and endpoint resolution under
  absolute/relative, `base_position`, extrude factor; optional endpoint axes;
  control-point offsets forwarded raw.
- Each error case raises (`gcmd.error`) with the specified message.
- **Range check**: a G5 whose endpoints are in range but a control point is
  out of range raises (convex-hull bound), while an in-hull curve passes.
- Bed mesh gate: activating a mesh raises; `BED_MESH_CLEAR` does not.
- Curve-time transform gate: with an active skew/tilt, `G5` raises; with no
  active transform (incl. `[bed_mesh]` configured but no mesh loaded), it does
  not.

End-to-end: a `G5` and a `G5.1` through the sim reach the planner as curved
segments (distinct from the collinear G1 path) and produce the expected step
stream.

## Future work (not prerequisites)

- `G5.2` / `G5.3` (NURBS) and `G2` / `G3` (arcs): slicer/preprocessor
  converters to a `G5` cubic stream with bounded error.
- Mesh-aware moves: port the move-transform layer to the new planner; teach
  `MoveSplitter` to subdivide a Bézier (de Casteljau is exact in XY) with
  per-piece Z; remove the activation gate.
- Affine transforms (skew, bed_tilt) applied exactly to control points
  without splitting (removes the curve-time transform gate).
- Cusp handling: split a segment at a cusp with a `v = 0` boundary (the SOTA
  stop-and-reverse), if the cusp experiment shows the solver needs it.
