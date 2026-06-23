# Finality barriers — why a prefix is provably locked

This companion is the correctness argument behind SPEC-incremental-planning. It replaces an earlier "compute a cache-invalidation distance and verify by comparison" framing, which was unnecessary: finality is structural and can be detected, not estimated.

## The only future→past coupling is the backward sweep

The planner has three append-relevant stages:

- **Fit** (`fit_chain_with_head_restore`, `rust/geometry/src/fitter.rs:188-291`) — each corner's biclothoid blend depends on its two adjacent moves' geometry, all at-or-behind the corner. Appending a move at the far end cannot change a blend behind it. **Append-invariant.**
- **Forward velocity sweep** — accelerate-from-history; the feasible velocity at seam `i` depends only on geometry ≤ `i`. **Append-invariant.**
- **Backward velocity sweep** (`plan_velocity_warm_start`, `rust/motion-engine/src/stream.rs:245-257`) — "be slow enough now to satisfy a constraint ahead." This is the *only* path by which a later move can lower an earlier seam's velocity.

So the entire question of "what can a future move change" collapses to the backward sweep.

## Two structural facts make the backward sweep blockable

1. **Append-only streaming.** New moves are added *only at the far end*; nothing is ever inserted between two existing moves. Therefore the nearest downstream constraint to any seam can only recede or stay put as the stream grows — never move closer.
2. **Monotone backward pass.** The backward sweep only ever *lowers* a seam's velocity (it intersects the forward-feasible profile with a decelerate-into-the-future envelope). It never raises it.

Combine them: to lower seam `i` below its current value, a future move would have to impose a constraint *closer* to `i` than the one currently binding it. Append-only forbids that. Hence **any seam whose velocity is already at its ceiling cannot be lowered by any future append.**

## What is final — and it is almost everything

A seam is **final** (append-invariant) if its velocity is dictated by anything other than the buffer's tentative terminal:

- **Acceleration, pinned by the past** — `v = v_forward`, below its ceiling. Set by the forward sweep from history; appends are downstream and cannot touch it.
- **Full cruise** — `v = max_velocity`. The global ceiling; nothing raises it, append-only means nothing lowers it.
- **Curvature-limited corner peak** — `v = v_cap(κ)`. The cap is fixed by geometry already in the buffer.
- **Brake into an already-buffered corner** — `v = v_backward`, but bound by a real corner already in the queue, not by the terminal. The corner is seen, so the brake into it is fixed.
- **Genuine interior reversal stop** — `v = 0` forced by a near-180° corner or a commanded dwell (the degenerate ceiling-touch, cap = 0).

The **only non-final region** is the trailing stretch whose velocity is pulled down specifically by the buffer's fictional terminal rest — the brake-to-rest from the last real ceiling/corner to "the queue ends here." Equivalently: the last barrier is the last seam where `v` meets `min(v_forward, ceiling)` rather than being dragged below it by the terminal. Everything behind that point is locked.

## The barrier places itself — reconvergence terminates the sweep

You do not estimate the barrier's position; the backward sweep finds it. Started from the tentative terminal rest, the backward sweep walks toward the frontier's past, raising the feasible velocity as it goes (you can be faster the further you are from the stop). It stops the instant its decel ramp **reconverges** — meets a seam already at `min(v_forward, ceiling)`, where the existing profile binds and the terminal's influence ends. That reconvergence point *is* the last barrier.

Two consequences:

1. **The sweep runs only over the open tail, not the buffer.** Reconvergence happens within one braking distance — a handful of moves — no matter how deep the buffer is. This is the mechanism behind CAP-1's flat-in-depth cost; the 217 ms spikes came from re-sweeping the whole buffer instead of stopping at reconvergence.
2. **Arcs and clothoids are exact for free.** The sweep walks the real geometry and respects each segment's curvature cap. Curvature can only force a *lower* speed into the stop, so it can only *shorten* the open tail — it never extends reconvergence further back than the straight-line case.

## The braking closed form is for sizing, not for locating

The jerk-limited time to decelerate `v → 0` under `a_max`, `j_max`:

- `t_brake = v/a + a/j` when `v > a²/j` (reaches `a_max`), else `t_brake = 2·√(v/j)` (triangular, never reaches `a_max`).

This is **not** used to locate the barrier (the sweep does that, exactly). It is used only to:

- **Size the flush-trigger watermark** — trigger the brake-to-rest solve when the locked lead drops to `t_brake(v_barrier) ` plus the planner solve-time plus a margin. `v_barrier` is free (the locked solve already ends there) and tight; the section max feedrate is the conservative alternative.
- **Bound how many moves stay open** — the section max feedrate gives a velocity-independent upper bound on the open-tail length (memory/work cap).

For this estimate, do *not* integrate the curve: the straight-line `t_brake` from `v` is a safe over-estimate (curvature only slows the stop), so it makes the trigger fire slightly early, which is safe.

## The locked solve terminates at the barrier, not at rest

This is what makes the prefix cheap to commit: the locked solve ends at the last barrier **pinned to that barrier's own ceiling velocity** (cruise speed, corner cap — not zero). It never has to look past the barrier, so committing the prefix never requires building the brake-to-rest. The deceleration tail is a separate, flush-only artifact.

## Deferred brake-to-rest (flush only)

Because the brake-to-rest exists only to honor the terminal fiction, it is built only when that fiction becomes real — a **flush**:

- **True end-of-stream.** No time pressure: the MCU is draining a full buffer, so there is ~a buffer's worth of slack to solve one short ramp.
- **Producer-stall low-watermark.** When the locked lead ahead of real-time falls to `braking-time + solve-time + margin`, trigger the brake-to-rest *then* — early enough that it always finishes before its first piece must dispatch. If a move arrives inside that window, discard the provisional brake and resume locked commits; you paid for one short solve only because you were genuinely near-stall. If nothing arrives, decelerating to a stop is the correct response.

So over a healthy print the brake-to-rest solve runs exactly once, at the end — never the per-batch recompute-and-discard that produced the 217 ms spikes.

## The trap: the buffer-terminal `v=0` is not a barrier

The planner pins the last buffered move to rest because it cannot see past the queue. That `v=0` is an **artifact**: it sits at the *bottom of the trailing braking ramp*, below its ceiling, and it rises the instant the next move arrives. It must never be selected as a barrier — doing so commits a velocity that is about to change. The discriminator is "at its ceiling" vs "on the way down to the tentative end." A real reversal stop is at its ceiling (cap = 0); the terminal artifact is not.

This is why the design needs no differential runtime guard: a correctly-detected barrier is final by construction, so there is nothing to compare against. The only guard is fail-loud (a `debug_assert` that a committed seam is never later revised) — unreachable under the proof, so a tripwire for an implementation bug.

## Note on the fit leading edge

The fit's one non-determinism is at the *front* of the buffer: trimming the committed head changes the leading move's head-reserve branch and the next corner's blend budget (`docs/rewrite/windowed-fit-ceiling-jitter.md`). This is already handled by `committed_head_len` / `fit_chain_with_head_restore` (`rust/motion-engine/src/stream.rs:105`), which makes the leading-corner fit window-invariant. Incremental reuse must keep carrying that state; it is orthogonal to the velocity barrier (front edge vs trailing ramp).

## Note on input shaping

Input shaping and pressure advance are **post-solver per-axis post-processors** (sota-motion; not yet in this branch). They are applied to already-planned motion after the per-axis split and do not affect the limits/velocity calculation. So they introduce **no backward coupling** — the velocity barrier is exact, no shaper-window setback. This exactness depends on shaping staying out of the solver; if it were ever moved into the limits path, a one-shaper-window setback behind the barrier would be required.

## Anchors for the implementer

- Commit sequence and the held-back tail today: `rust/motion-engine/src/stream.rs:218-362`; the `keep_secs` heuristic this replaces at `:14-23`.
- Batch driver (commit once per coalesced burst, `COALESCE_BATCH_MOVES = 64`): `rust/motion-engine/src/stream_planner.rs:599-735`.
- Persistent seam state today (`entry_v`, `odometer`, `t_committed`, `committed_head_len`): `rust/motion-engine/src/stream.rs:92-120` — the only things carried across commits now; the fitted/planned/lowered intermediates are recomputed and discarded each call.
- Regression guards to keep green: `rust/motion-engine/src/stream/tests.rs`.
