# Investigation: MCU telemetry memory reclamation (F401 ring growth)

## Hand-off Brief

1. **What happened.** The premise — "a chunk of MCU memory was dedicated to telemetry that was never built" — is only *partially* confirmed: exactly **40 bytes** of genuinely-dead telemetry counters exist in `SharedState`, and the *entire* telemetry/diagnostic surface on the MCU totals well under 1 KB.
2. **Where the case stands.** Confirmed: 4 unwired counters (40 B) are reclaimable with zero functional impact. Deduced: reclaiming telemetry memory cannot meaningfully grow the F401 piece ring — the ring is 32 KB and the entire diagnostic surface is <1 KB. The real lever for ring growth is the `rt_storage` ceiling vs. the C-side dynamic pool split.
3. **What's needed next.** Decide the actual goal: (a) tidy the 40 B of dead counters (cheap, correct, but immaterial to ring size), or (b) pursue ring growth via the `RUNTIME_STORAGE_SIZE_SMALL` ceiling and SRAM budget — a different change with no telemetry connection.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-24                                                                  |
| Status           | Active                                                                      |
| System           | Kalico fork, MCU firmware; targets STM32H7 / STM32F4(F401) / STM32G0       |
| Evidence sources | Source code (rust/runtime, src/), Kconfig, linker script, git log          |

## Problem Statement

User recollection: "we dedicated a chunk of MCU memory to telemetry, and we never actually built this telemetry." Goal: free that memory — **especially on F401** (Neptune3 Pro, 64 KB SRAM) — so the piece ring (`CONFIG_RUNTIME_PIECE_RING_SIZE`) can be increased. The recollection is treated as a hypothesis to verify, not a fact.

## Evidence Inventory

| Source                                   | Status    | Notes                                                                 |
| ---------------------------------------- | --------- | --------------------------------------------------------------------- |
| `rust/runtime/src/state.rs` SharedState  | Available | The MCU's telemetry/diagnostic counter surface lives here             |
| `src/Kconfig`                            | Available | Per-target ring + rt_storage byte budgets                             |
| `src/generic/armcm_link.lds.S`           | Available | Linker sections; no telemetry-named region exists                     |
| `src/runtime_storage.c`                  | Available | `rt_storage[]` backing buffer + static-assert headroom guard          |
| `docs/rewrite/mcu-c-rust-boundary.md`    | Available | B2 names "future telemetry rings" — aspirational, not allocated       |
| git history (`-i --grep=telemetr`)       | Available | All telemetry commits are EtherCAT/servo host-side, not MCU SRAM      |

## Confirmed Findings

### Finding 1: Exactly 40 bytes of dead telemetry counters in SharedState

**Evidence:** `rust/runtime/src/state.rs:227-231`

```
pub queue_high_water: [AtomicU32; 4],            // 16 B
pub queue_overflow_count: [AtomicU32; 4],        // LIVE — 2 write/read sites
pub spi_saturated_samples: AtomicU32,            //  4 B
pub sample_isr_peak_cycles: AtomicU32,           //  4 B
pub per_axis_consumer_peak_cycles: [AtomicU32; 4], // 16 B
```

**Detail:** A grep for non-declaration, non-test uses returns **0 sites** for `queue_high_water`, `spi_saturated_samples`, `sample_isr_peak_cycles`, and `per_axis_consumer_peak_cycles` — they are only declared (lines 227, 229-231) and zero-initialized (`new()`), never written or read by live code. `queue_overflow_count` is live (2 sites). Total dead = 16 + 4 + 4 + 16 = **40 bytes**. These were added alongside `queue_overflow_count` as a batch of "five new telemetry counters" during the stepping-redesign bring-up; only `queue_overflow_count` was ever wired.

### Finding 2: No telemetry-named linker section or buffer exists

**Evidence:** `src/generic/armcm_link.lds.S:101-135`; `docs/rewrite/mcu-c-rust-boundary.md` (B2)

**Detail:** The MCU linker sections are `.sched_protected`, `.persistent_diag`, `.axi_bss` (H7-only), `.bkp_bss` (H7-only) — all live. There is **no** telemetry/trace/capture section. The boundary doc lists "future telemetry rings" only as an *example* of state that *would* live C-side if it were ever built — it was never allocated. No large telemetry/trace/scope buffer exists in `src/*.c` either.

### Finding 3: The entire MCU telemetry/diagnostic surface is < 1 KB

**Evidence:** `rust/runtime/src/state.rs:85-258` field-type tally — 69×`AtomicU32`, 13×`AtomicU64`, 6×`AtomicU8`, 10×`AtomicBool`, 4×`AtomicI32`, 1×`AtomicU16`, plus a handful of `[_;4]` arrays.

**Detail:** Scalar counters ≈ 414 B; the per-axis arrays add ~112 B; stepper-OID arrays are functional, not telemetry. The whole forensic/diagnostic block is on the order of **0.5–0.7 KB**. Most of it (the `isr_last_*`, `producer_*_total`, `last_push_*`, `last_resolved_*`, `tick_blocker_*` fields) is stepping-redesign bring-up forensics that *is* written by live code — so it is "diagnostic cruft that could be retired," not "telemetry that was never built." Retiring it requires also removing its write sites and any `DIAG_DUMP`/`status_drain` readers.

## Deduced Conclusions

### Deduction 1: Reclaiming telemetry memory cannot meaningfully grow the F401 ring

**Based on:** Findings 1 & 3, plus the F401 budget below.

**Reasoning:** F401 `CONFIG_RUNTIME_PIECE_RING_SIZE = 32768` (32 KB) — `Kconfig:389`. The total telemetry/diagnostic surface is <1 KB, of which only 40 B is genuinely dead. Even retiring the *entire* diagnostic block frees <1 KB — ~3% of the existing ring, ~1.5% of the 64 KB SRAM. It does not change the ring-growth math.

**Conclusion:** Telemetry reclamation is the wrong lever for ring growth. It is a worthwhile *tidy* (the 40 B, and optionally the bring-up forensics), but it is not what unlocks a bigger ring.

### Deduction 2: The real F401 ring-growth lever is the rt_storage ceiling / SRAM split

**Based on:** `src/Kconfig:386-401, 476-480`; `src/runtime_storage.c:15-32`

**Reasoning:** On F401 the ring lives inside `rt_storage[RT_STORAGE_SIZE]`, ceiling `RUNTIME_STORAGE_SIZE_SMALL = 36864` (36 KB). Fixed (non-ring) RuntimeContext ≈ 2520 B, so current occupancy ≈ 2520 + 32768 = 35288 B → **~1.5 KB of slack already exists under the ceiling** (the ring could grow ~1.5 KB today with no other change). Beyond that, the ceiling itself must rise, which eats into the leftover C-side dynamic pool and stack on the 64 KB part. That budget split — not telemetry — is the constraint.

### Deduction 3: The "C-side dynamic pool" is leftover RAM, not a fixed reservation — the Kconfig "~17 KB" note is doubly misleading

**Based on:** `src/generic/armcm_boot.c:182-192`; `src/stm32/Makefile:59`; `src/linux/Makefile:8`, `src/simulator/Makefile:6`; `src/generic/alloc.c:9`; `src/stm32/Kconfig:275` (STACK_SIZE=4096)

**Reasoning:** On STM32/ARM, `dynmem_start() = &_persistent_diag_end` and `dynmem_end() = &_stack_start` (`armcm_boot.c:182-192`) — the dynamic pool is **all RAM between the end of the .bss/.persistent_diag region and the top-of-RAM stack**, i.e. a *computed leftover*, not an allocation. The fixed `static char dynmem_pool[20 * 1024]` in `generic/alloc.c:9` is compiled **only for `linux` and `simulator`** (`src/*/Makefile`), never for F401 — it is a red herring. Therefore: (1) the Kconfig "~17 KB" figure is not a budget line item, it is whatever happens to be left over; (2) **raising `RUNTIME_STORAGE_SIZE_SMALL` shrinks the dynamic pool 1:1** — there is no separate pool to "free", only a gap to re-divide. The hard floor is whatever the printer's config-time allocations (oid tables + `alloc_chunk` command objects, `src/basecmd.c:31-51`) actually consume at runtime; everything above that high-water is ring headroom.

**Conclusion:** F401 ring growth is bounded by the runtime **dynmem high-water**, not by any reservation. The number cannot be read off the source — it needs a live measurement (or a conservative static bound from a build `.map`).

## Hypothesized Paths

### Hypothesis 1: User's "chunk dedicated to telemetry" referred to a larger reservation

**Status:** Refuted

**Theory:** A multi-KB telemetry ring/buffer was reserved in SRAM or the linker map.

**Would refute:** No telemetry-named section, Kconfig budget, or large static buffer anywhere; the only telemetry-tagged memory is the <1 KB SharedState counter block, 40 B of it dead.

**Resolution:** Refuted by Findings 1–3. The recollection most likely conflates (a) the 40 B of unwired counters with (b) the broader bring-up forensic block, neither of which is a reclaimable "chunk" at ring scale.

## Missing Evidence

| Gap                                          | Impact                                              | How to Obtain                                              |
| -------------------------------------------- | --------------------------------------------------- | --------------------------------------------------------- |
| Live F401 SRAM map (.map file / size output) | Confirms exact dynamic-pool + stack headroom        | Build F401 image, inspect `.map` / `arm-none-eabi-size`   |
| C-side dynamic pool actual high-water        | How much of the ~17 KB pool is really used on F401  | Runtime `dynmem` stats / DIAG dump on the bench           |

## Source Code Trace

| Element       | Detail                                                                    |
| ------------- | ------------------------------------------------------------------------- |
| Dead counters | `rust/runtime/src/state.rs:227,229-231` (decl) + `new()` init (~line 387) |
| Ring budget   | `src/Kconfig:386-401` (`RUNTIME_PIECE_RING_SIZE`, F401=32768)             |
| Storage ceiling | `src/Kconfig:471-480` (`RUNTIME_STORAGE_SIZE_SMALL`, F401=36864)         |
| Backing buffer | `src/runtime_storage.c:15` (`rt_storage[]`) + static-asserts             |
| Caps report   | `src/mcu_transport_dispatch.c:212` (ring size surfaced to host)           |

## Conclusion

**Confidence:** High (for the telemetry accounting); Medium (for ring-growth headroom, pending a live `.map`).

Confirmed: only **40 bytes** of MCU memory is telemetry-that-was-never-built (4 unwired `SharedState` counters); they are safe to delete. The broader diagnostic surface is still <1 KB and mostly live-written bring-up forensics. **The premise that reclaiming telemetry frees a meaningful chunk for ring growth is not supported.** The actual F401 ring lever is the `rt_storage` ceiling (36864) and the SRAM split with the C-side dynamic pool — ~1.5 KB of ring growth is already available under today's ceiling without touching telemetry at all.

## Recommended Next Steps

### Fix direction

- **Tidy (cheap, immaterial to ring):** delete the 4 dead counters at `state.rs:227,229-231` and their `new()` initializers. ~40 B, zero behavior change. Candidate for `bmad-quick-dev`.
- **Ring growth (the actual goal):** treat as a separate change — raise F401 `RUNTIME_STORAGE_SIZE_SMALL` (and/or `RUNTIME_PIECE_RING_SIZE`) after measuring real C-side dynamic-pool + stack headroom from a live `.map`. Not a telemetry change.

### Diagnostic

Build the F401 image and capture `arm-none-eabi-size` + `.map` to confirm how much of the 64 KB is actually free; pull `dynmem` high-water from the bench to size the safe ceiling increase.

## Side Findings

- Host-side telemetry (EtherCAT `EcTelemetry`, servo `SERVO_CAPTURE`, geometry `TelemetryEvent`) is fully built and live — unrelated to MCU SRAM. (`rust/ethercat-rt/src/ffi.rs:10`, `rust/geometry/src/telemetry.rs`)
- `event_log` ring (`src/event_log.c:26`, 64 entries) is the live structured-log ring — built and in use, not reclaimable.
- The large `isr_last_*` / `producer_*` forensic counter block is a legitimate future cleanup target if bring-up is considered closed, but it is live-written and tied to `DIAG_DUMP` readers — removal is a deliberate de-instrumentation, not free memory.
