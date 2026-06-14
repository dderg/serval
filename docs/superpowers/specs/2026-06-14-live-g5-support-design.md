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
- A fail-loud gate rejecting **activation** of a bed mesh.

Explicitly out of scope (deferred, not prerequisites):

- `G5.2` / `G5.3` (NURBS blocks). NURBS are rational and arbitrary-order —
  exactly what our core forbids (see "Geometry mapping"). They are flattened
  to a `G5` cubic stream by the slicer/preprocessor, like arcs.
- Live `G2` / `G3`. Arc-to-Bézier is the slicer's job; its math stays in
  `compat`, which is moving to the slicer.
- Any bed-mesh / skew / bed-tilt interaction with moves. The move-transform
  layer needs rework for G1 too; that is a separate project. Until then bed
  mesh activation is gated off.

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
  -> append_and_replan -> plan_velocity       # full SOCP + SLP optimizer,
       -> temporal::multi::plan_batch              with multi-move lookahead
  -> emit_shaped -> Bernstein pieces -> MCU ring
  -> ISR ~40 kHz: eval_horner -> step times
```

There is no trapezoidal profiler. The planner's native primitive is a cubic
Bézier; a G1 is just the special case where the four control points are
collinear, fed to the same optimizer. **G5/G5.1 reach the identical optimizer —
the only difference is that their control points are not collinear.**

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
arc-length facility if it exists (the optimizer needs path length, so it
likely does); otherwise add an 8-point Gauss-Legendre quadrature on the cubic
(effectively exact, a handful of flops).

### Chaining (smooth G5)

`G5`-only (a `G5.1` requires ≥1 of `I/J`, never omits, and does not
participate). State lives in the bridge, where the running position lives:

- After building any `G5` segment, store its `(P, Q)` offset pair.
- On `submit_bezier` with `i,j = None`: set `(I, J) = (-P_prev, -Q_prev)`
  (equivalently `P1 = 2*P0 - prev_P2` — the spec's reflection).
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

## Bed mesh activation gate

The move-transform layer (`bed_mesh`, `skew_correction`, `bed_tilt`) warps
moves above the bridge and has not been ported to the new motion planner —
for lines or curves. Bed mesh follows the surface by **splitting** a move into
short pieces (`MoveSplitter`, `split_delta_z = 0.025`), which has not been
validated against the new planner. Rather than silently print a
surface-ignoring path, fail loud at activation.

Gate point: `bed_mesh.py` `set_mesh(self, mesh)` — the single chokepoint;
both profile `LOAD` and calibration's `probe_finalize` funnel through it.
When `mesh is not None`, raise before `self.z_mesh` is assigned:

```
bed_mesh: activating a mesh is not supported under the new motion planner
yet (the surface-following transform layer has not been ported).
BED_MESH_CLEAR is allowed.
```

Consequences, by design: `BED_MESH_PROFILE LOAD` and `BED_MESH_CALIBRATE`
(which auto-applies via `set_mesh`) are rejected; `set_mesh(None)`
(`BED_MESH_CLEAR`) is allowed. The gate is removed when the move-transform
layer is reworked for the new planner (separate project).

## Testing

Rust (`cargo nextest run`):

- `geometry`: G5 control-point assembly; G5.1 quadratic→cubic elevation is
  exact (sample both forms, compare); Z thirds/half give linear Z; arc length
  vs chord on a known curve; relocated `to_collinear_bezier` keeps its tests.
- `motion-bridge` `classify`: builds a non-degenerate `CubicSegment` from
  control points; chaining `(I,J) = -(P_prev,Q_prev)`; chain cleared by an
  intervening `submit_move`/`submit_quadratic`; chain-without-prior-`G5`
  returns an error.

Python (`./scripts/ci.sh py`):

- `cmd_G5` / `cmd_G5.1` parsing and endpoint resolution under
  absolute/relative, `base_position`, extrude factor; optional endpoint axes;
  control-point offsets forwarded raw.
- Each error case raises (`gcmd.error`) with the specified message.
- Bed mesh gate: activating a mesh raises; `BED_MESH_CLEAR` does not.

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
  without splitting.
