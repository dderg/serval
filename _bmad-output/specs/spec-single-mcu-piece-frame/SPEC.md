---
id: SPEC-single-mcu-piece-frame
companions: [wire-format.md]
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Single-MCU PushPieces frame

## Why

Print throughput is non-negotiable, and on dense G-code the planner can't keep the
MCU fed. Bench evidence (Neptune, `print-1782209653`): a layer-3 dense section
crashes mid-print with `PieceStartInPast` (fault `65228`). Transit-diag added this
session localized it precisely — `piece_count=1` on every frame with ring `room`
wide open (61–261 free slots) and the lead horizon deep, so it is **not** ring- or
horizon-bound. The cause is general and chip-independent: the wire frame is
**single-axis**, so each of an MCU's axes costs its own serial round-trip (~5 ms).
On the reproduction (the Neptune serial MCU — an F401 — with three stepper axes
Y/Z/E on one link) that saturates the port at ~15 ms per send pass delivering one
piece each, ~0.47× real-time; any MCU with several axes on one transport shares the
floor. The delivered lead drains to zero and the MCU fail-loud halts.
Raising the lead horizon (2 s) and ring depth only deferred the crash from layer 1
to layer 3; the throughput floor is unchanged until the frame addresses an **MCU**
(all its axes) instead of a single axis. See `wire-format.md` for the byte contract.

## Capabilities

- id: CAP-1
  intent: The pump delivers every axis frame destined for one MCU in a single PushPieces transaction, so a send pass costs one serial round-trip regardless of axis count.
  success: On a dense-section bench run, transit-diag shows one round-trip per MCU carrying all its axes (mcu0 `send_gap` ≈ one round-trip, not three), and mcu0 delivery rises from ~0.47× to ≥1× real-time.

- id: CAP-2
  intent: The wire protocol carries multiple axis-blocks per request frame and returns one frame-level result, a global arrival clock, and a per-axis diagnostic echo.
  success: Round-trip encode/decode unit tests pass for the multi-axis `PushPieces` and the frame-level `PushPiecesResponse` (per `wire-format.md`); `axis_count = 1` reproduces today's behavior beyond the count byte; the host advances ring bookkeeping only on `result == OK` via a single `all_ok` gate.

- id: CAP-3
  intent: Every non-OK outcome fails loud with the correct disposition — transport errors retry a zero-committed frame, ring-overflow/logic errors halt the print, and corrupt/oversized/version-mismatched frames are rejected before the MCU touches a ring.
  success: CRC is verified before any `runtime_write_piece`/`runtime_commit_head`; decode rejects out-of-range `axis_count`/`axis_idx`/`piece_count` and duplicate `axis_idx` with a static assert pinning the worst-case block count; a mid-frame commit failure halts the engine (no retry); a bumped version byte refuses a mismatched flash; an ASan fuzz of the C parser over all frame lengths/counts reads nothing past the buffer.

## Constraints

- The frame must fit `MCU_TX_BUF_SIZE` — a **shared** `#define` (256 B, ~250 B usable) compiled into every chip's firmware, sized for the most RAM-constrained MCU in the fleet (today the F401 at 64 KB SRAM). The host caps pieces per that budget for **every** MCU regardless of chip (parameterized, never chip-hardcoded; could become per-MCU via capability negotiation later). At 256 B, the worst-case multi-axis frame (the Neptune serial MCU's 3 axes) is ~121 B at 1 piece/axis, ceiling ~2 pieces/axis. The buffer is **left unchanged** — one change at a time, and a raise would cost SRAM on every chip and must stay safe on the tightest one. This collapses round-trips only; deeper batching is a later change if a trace shows 1.4× is insufficient.
- Clean break: host and both MCU firmwares are flashed in lockstep; the protocol version byte is bumped so a mismatch fails loudly (project fail-loud rule). No on-wire back-compat with the single-axis frame.
- Framing change only, within the C/Rust MCU boundary (`docs/rewrite/mcu-c-rust-boundary.md`): no new shared state. Per-axis ring, `RingDescriptor`, `PieceEntry` (32 B), and the `runtime_write_piece`/`runtime_commit_head` FFI seam are unchanged — C parses N blocks and calls the existing per-axis FFI in a loop.
- **Three** endpoints speak this protocol, all migrate to the one format (clean break): the host pump (`WireSink`), the shared C `piece_sink` (compiled for every chip — F401/F446/H7; carries however many axes that MCU configures), and `ethercat-rt` — the host-side runtime for the EtherCAT servo node, a second decoder/responder. The EtherCAT node is permanently single-axis, so it always sends `axis_count = 1`; it gains nothing from batching but must understand the format. The C change is chip-agnostic (one `mcu_transport_dispatch.c` built for all targets), so it lands on every MCU at once.
- `result` is one frame-level verdict (no per-axis success on a hard-real-time path — a partial frame is desync, not partial success); `arrival_clock` is one value per frame sampled at receive-complete; `front_start_time` stays per-axis but is diagnostic-only (the transit-diag echo), not control.
- CRC is verified before the parse loop, so transport failures are always pre-commit and retryable on a zero-committed frame; a mid-frame commit failure fail-loud halts the whole engine and is never retried.
- `PieceStartInPast` stays fail-loud per axis — late start times are never padded or advanced.

## Non-goals

- Deep per-axis batching (more than ~1–2 pieces/axis/frame); bounded by the 256 B buffer and deferred to a separate effort that grows the MCU buffer.
- Transport pipelining of single-axis frames — the alternative fix considered and superseded by this one.
- Skipping constant/empty pieces for non-moving axes (e.g. stationary Z) — a separate, host-only optimization that follows this one.
- Further raising `MAX_LEAD_SECS` or ring capacity — already landed (2 s horizon, 1024 EtherCAT ring) and orthogonal.
- The EtherCAT velocity-readback cosmetic bug — unrelated reporting issue.
- Any change to the per-axis ring layout, the FFI seam, or the `PieceEntry` format.

## Success signal

The same G-code that crashed mid-layer with `PieceStartInPast` on the Neptune bench prints through that section and to completion, with delivered lead holding near the 2 s horizon instead of draining; transit-diag confirms one round-trip per MCU carrying all its axes, and mcu0 serial delivery moves from ~0.47× to ≥1× real-time.

## Assumptions

- Clean break (no on-wire back-compat with the single-axis frame) is acceptable because the host and both MCUs are flashed together; surfaced for explicit sign-off.
- Request layout (`axis_count` + per-axis blocks) and the frame-level response are ratified — the request shape passed a four-expert roundtable greenlight-with-conditions, and the response was reduced from per-axis `result` to one frame-level `result` by user decision (a partial frame is desync, not partial success). The party-mode conditions (all_ok gate, full-slot-overwrite, CRC-before-parse, ASan fuzz, version-refuse-on-mismatch, green bench trace) are folded into CAP-2/CAP-3 and `wire-format.md`.
- `front_start_time` per-axis echo is kept as diagnostic (it is the signal that localized the original crash); droppable later for a minimal response since the host already knows what it sent.
- Decided (2026-06-23): round-trip-collapse **only**; `MCU_TX_BUF_SIZE` untouched. Build smaller, one change at a time; the buffer raise is a cheap later follow-up if a trace ever proves 1.4× insufficient.
