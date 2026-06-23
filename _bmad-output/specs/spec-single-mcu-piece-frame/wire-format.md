# Wire format — single-MCU PushPieces

Companion to `SPEC.md`. Holds the byte-level contract, the frame budget, and the
file-by-file change surface. Layer-0 invariant: this is a **framing** change. The
per-axis ring, `RingDescriptor`, `PieceEntry` (32 B), and the
`runtime_write_piece` / `runtime_commit_head` FFI seam are untouched.

## PushPieces — request

**Before (single-axis):**

```
axis_idx:u8  piece_count:u8  start_slot:u16  new_head:u32  pieces[piece_count×32]
```

**After (single-MCU):** the old body becomes a per-axis block; the frame is a
length-prefixed list of those blocks.

```
axis_count:u8
repeat axis_count×:
    axis_idx:u8  piece_count:u8  start_slot:u16  new_head:u32  pieces[piece_count×32]
```

Per-axis block layout is byte-identical to the old flat message. `axis_count = 1`
is byte-for-byte the prior single-axis frame plus one leading count byte.

## PushPiecesResponse — response

**Before:** `result:i32  arrival_clock:u64  front_start_time:u64`

**After:** `result` is **frame-level** (one verdict for the whole MCU transaction);
`arrival_clock` is global; `front_start_time` stays per-axis but is **diagnostic
only**, never control.

```
result:i32            // frame-level: OK, or a fatal error (ring overflow / logic)
arrival_clock:u64     // global, sampled at frame-receive-complete (before parse/FFI)
axis_count:u8
repeat axis_count×:
    axis_idx:u8  front_start_time:u64   // echo for transit-diag arrival_lead only
```

Rationale: the only per-axis "not OK" the runtime produces in normal operation is
**ring overflow** (`Overcommit` → `RUNTIME_ERR_RING_FULL`), which means the host
over-pushed — a pacing/accounting bug or host↔MCU desync, not a per-axis recover.
Every other "not OK" (bad axis, misconfig, gated) is frame-global or a bug. On a
hard-real-time motion path there is no legitimate "axis 0 OK, axis 1 not, carry on"
— that is desync (wrong motion). So a single frame verdict is both simpler and
safer. The host still computes per-axis `arrival_lead = front_start_time −
arrival_clock` from the diagnostic echo.

## Failure taxonomy (host handling)

| Class | When | Host action |
| --- | --- | --- |
| Transport (CRC fail, truncation, timeout) | before any commit — CRC gates the parse | **retry** the whole frame (idempotent: zero committed) |
| Frame OK | all axes committed | advance ring bookkeeping (all-or-nothing) |
| Fatal (ring overflow / logic) | a commit failed mid-frame | **halt the print loud — do NOT retry** |

Today the host maps *any* MCU "not OK" to a transient retry; the new design splits
it: overflow/logic → fatal halt (retry can't help), transport → retry. This
**designs out** the partial-commit-then-retry hazard rather than relying on
slot-overwrite idempotency to survive it — when a commit fails mid-frame the MCU
fail-loud halts the whole engine, so the half-committed rings never execute
(`runtime_tick` is stopped) and nothing re-sends them.

## Frame budget (the binding constraint)

`MCU_TX_BUF_SIZE = 256` → ~250 B usable payload. It is a **shared** `#define`
compiled into every chip's firmware (F401/F446/H7), sized for the most
RAM-constrained MCU in the fleet (today the F401, 64 KB SRAM). The host caps pieces
to this budget for **every** MCU — chip-agnostic, never hardcoded per target.
Per-axis block overhead is 8 B (`axis_idx + piece_count + start_slot + new_head`).

- N axes × 1 piece: `1 + N×(8 + 32)` — e.g. 3 axes = 121 B, fits with wide margin
  (fits up to ~6 axes at 1 piece each).
- Ceiling at 250 B with 3 axes ≈ **2 pieces/axis**.

So this collapses **round-trips** (N→1 per MCU); it does **not** also unlock deep
per-axis batching. Deeper batching needs a larger shared buffer, which costs SRAM on
every chip and must stay safe on the most-constrained one — out of scope here.

## Change surface (host → MCU)

```
WireSink.send_mcu_frames  → encode one PushPieces{axes:[…]} per MCU
   src/mcu_demux.c             byte-agnostic feed + CRC — UNCHANGED
   src/mcu_transport_dispatch.c  piece_sink: parse axis_count blocks,
                                  write + commit EACH axis, build per-axis response
   rust/c-api runtime_write_piece / runtime_commit_head   ← per-axis, UNCHANGED
   WireSink response decode    → loop resp.axes, per-axis transit-diag
```

- `rust/mcu-protocol/src/messages.rs` — `PushPieces`→`{axes: Vec<AxisPieces>}`,
  `PushPiecesResponse`→`{arrival_clock, axes: Vec<AxisResult>}`, encode/decode.
- `rust/motion-engine/src/pump.rs` — `WireSink::send_mcu_frames` override (build
  + send one frame, decode multi-axis response, per-axis transit-diag); the
  default fan-out scaffold (commit `8d3e5adf9`) is replaced by this real override.
- `src/mcu_transport_dispatch.c` — `piece_sink` multi-block parse, per-axis
  write/commit loop, multi-axis response build; bound `axis_count` and per-block
  `piece_count` against the receive buffer.
- `rust/ethercat-rt/` — the **third endpoint** (EtherCAT X servo, single-axis):
  `src/wire.rs` decode + response build, `src/bin/ethercat-rt.rs` handler, and the
  stub/test files. Always `axis_count = 1`; reads `axes[0]`, replies via the
  single-axis response helper. Mechanical but real — it is a live decoder, not just
  tests.

`mcu-protocol` compiles independently of its consumers, so the types + round-trip
tests land first (green in that crate); the three endpoints migrate next.

## Safety / fail-loud

- **CRC is verified before the parse loop runs** (whole-frame, not per-block). A
  bad `axis_count`/`piece_count`/CRC is rejected before any `runtime_write_piece`
  or `runtime_commit_head`, so transport failures are always pre-commit and a
  retry re-sends a zero-committed frame.
- Decode rejects any `axis_count` / `axis_idx` / `piece_count` whose declared bytes
  exceed the remaining frame or `MCU_TX_BUF_SIZE`, and rejects duplicate
  `axis_idx`; a static assert pins the worst-case block count so the C `piece_sink`
  cannot walk off its buffer.
- A commit failure mid-frame (ring overflow / logic) **fail-loud halts the whole
  engine**; the host does not retry a fatal result. Half-committed rings never
  execute because `runtime_tick` is stopped.
- `arrival_clock` is sampled at **frame-receive-complete**, before parse/FFI, so
  the host's clock-offset estimate is invariant to frame payload size (1 axis vs N).
- Protocol **version byte bumped**. A host/firmware flash mismatch is **refused**
  (distinct error, no connect) rather than mis-parsing a re-laid-out frame.
- `PieceStartInPast` behavior preserved per axis — late start times still fault,
  never padded or advanced.
- Host pump is all-or-nothing: ring bookkeeping (`pushed`,
  `physical_write_cursor`) commits only on whole-frame `result == OK`; a single
  `all_ok` predicate gates the advance. `runtime_write_piece` is a full-slot
  overwrite (not accumulate) — a precondition to verify in `runtime_ffi.rs`.
