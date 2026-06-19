# Moving the phase-mode enter/exit handshake onto the MCU

## Why

Entering or leaving TMC5160 phase stepping today is orchestrated entirely from
the host (`klippy/extras/tmc5160.py:561-702`). Each handover is a chain of
**blocking, synchronous SPI register transactions** issued over USB — disable
ISR writes, write CHOPCONF, RMW GCONF (`direct_mode`), read MSCNT, preload
XDIRECT, query, align, enable ISR writes, flip axis mode — repeated serially
for every motor of the handover group (the corexy A/B pair is one group). On
the Trident's shared `spi1` bus, with the USB link saturated by the bulk motion
stream, this measures ~50 ms per motor, ~160 ms for the four-motor group.

For ~160 ms the host reactor cannot feed the motion ring while MCU time keeps
advancing. The first motion piece queued after the handover carries a
`start_time` computed before the stall; by the time the MCU's TIM5 ISR adopts
it the piece is already past the `MAX_START_IN_PAST_SECS = 200 µs` drift budget
(`rust/runtime/src/motion_core.rs:114-128`), and the runtime latches
`PieceStartInPast` (-308) — correctly, per the fail-loud contract. **The bug is
the stall, not the check.** This was observed on the Trident (2026-06-18,
session `k-1781781133-9650`): four `phase_stepping enter` events at
`11:12:45.77–45.93`, then `-308` at `11:12:46.24`. A clean phase-stepping run on
2026-06-11 predates the corexy merged-handover-group rework (`b53a917e7`,
`74342047a`, 2026-06-12) that turned the handshake into one ~160 ms burst.

The MCU sits next to the SPI peripheral and already has all the hard parts:

- **Bus arbitration is solved.** On H7, the general `spi_transfer`
  (`src/stm32/stm32h7_spi.c:197-208`) already spin-acquires the `phase_spi_busy`
  cooperative flag and releases it, mediating the TIM5-ISR XDIRECT writer
  against lower-priority register access (`src/stm32/phase_stepping_spi.h:35-42`).
- **The phase state machine is on the MCU.** `set_axis_mode`,
  `phase_align_to`, `phase_jog_to`, `phase_state`, `enable_writes` /
  `disable_writes` already live in `rust/runtime/src/engine.rs` and
  `rust/runtime/src/phase_handover.rs`.

What's still host-side is only the **one-time register config** of a handover.
Moving it onto the MCU collapses ~160 ms of USB round-trips into one
sub-millisecond command, off the host timeline, dissolving the regression
without relaxing the 200 µs check or padding any start time (both forbidden by
`CLAUDE.md`).

## Architecture boundary

Respects `docs/rewrite/mcu-c-rust-boundary.md`:

- **C owns the SPI peripheral.** The blocking register transactions (datagram
  framing, CS toggling, busy-flag arbitration, RXDR capture, timeout) are new
  C functions in `src/stm32/phase_stepping_spi.c`. Rust never touches SPI
  registers.
- **Rust owns the orchestration.** Sequence ordering, MSCNT caching, coil
  preload selection, offset alignment, mode flip — all in
  `rust/runtime/src/phase_handover.rs`, calling the C primitives via
  `extern "C"`.
- **The seam stays `extern "C"` + scalar/ptr-len.** One new logical method on
  the existing `Runtime` opaque handle (`runtime_set_axis_mode_group`); the
  oid array crosses as `ptr + len` (boundary rule B4). No new C-visible structs.
  `phase_enter_mscnt` is Rust-internal atomic state that never crosses the ABI.

## Enter is synchronous; exit is not

This asymmetry drives the command shape and is the single most important design
fact:

- **Enter** (`Pulse → Phase`) is immediate: preload XDIRECT to the freshly-read
  MSCNT, set `direct_mode`, align the offset *directly* (no walk —
  `phase_handover::align_to` sets `phase_offset == target` in one store), flip
  the mode. No ramp. This is the path the reported jog-crash hits, so a single
  synchronous MCU command fixes `PieceStartInPast`.

- **Exit** (`Phase → Pulse`) has an inherent multi-tick settle: the rotor must
  walk back to the chip's frozen enter-MSCNT before `direct_mode` is cleared, or
  the coils snap from the live driven angle to the frozen MSCNT angle — a
  physical lurch. The walk runs at ≤ `PHASE_JOG_MAX_PER_SAMPLE` microsteps per
  sample (`tmc5160.py:652-683`), spanning many ISR ticks. A foreground call
  cannot wait on the ISR. So **exit keeps the host's existing cheap
  `kalico_get_phase_state` poll** for the settle, but moves the heavy register
  dance (CHOPCONF/GCONF/MSCNT/XDIRECT) to the MCU. Exit is a two-step host flow
  (begin-walk → poll settled → finalize), not one synchronous call.

`direct_mode` freezes the chip's internal microstep sequencer, so MSCNT holds
the enter-time value for the whole phase episode. Walking XDIRECT back to that
cached value before clearing `direct_mode` is exactly today's proven host
procedure — this design **relocates** it, it does not redesign the physics.

## Command surface

Extend the existing `kalico_set_axis_mode` path; do not invent a verb.

| Phase | Command | Args |
|---|---|---|
| Enter | `kalico_set_axis_mode_group` | `axis_idx=%c mode=%c stepper_count=%c stepper_oids=%*s` |
| Exit begin | `kalico_set_axis_mode_group` with `mode=2` (phase→pulse-walk) | same |
| Exit poll | `kalico_get_phase_state oid=%c` (unchanged) | — |
| Exit finalize | `kalico_set_axis_mode_group` with `mode=0` | same |

`mode`: `0 = Pulse (finalize)`, `1 = Phase (enter)`, `2 = begin-exit-walk`.
The group oid list lets one command cover the whole corexy A/B group so it
enters/exits atomically (all-validated before any SPI write).

Removed from the host-visible set (now internal to the MCU sequence):
`kalico_phase_stepping_enable_spi`, `kalico_phase_stepping_disable_spi`,
`kalico_phase_align_to`. Kept: `kalico_get_phase_state` (poll + diagnostics),
`kalico_phase_jog_to` (still used by tests/diagnostics).

This is a breaking protocol change — host and firmware version together (gated
by the kalico-native MCU identify).

## Blocker resolutions (from the adversarial design review)

The first design pass returned **needs-rework**. Each blocker is resolved here;
do not implement around them.

1. **Read returns garbage on timeout (fail-loud gap).** The H7 inline XDIRECT
   block (`phase_stepping_spi.c:143-185`) discards RXDR and `goto bail`s
   silently on timeout — fine for a fire-and-forget write, fatal for a read
   feeding `coil_for_phase` (a zero MSCNT → rotor snaps to electrical angle 0).
   **Resolution:** the new `phase_spi_read_register` / `_rmw_register` return a
   success flag; on a per-byte timeout they signal failure, and the Rust caller
   raises a distinct fail-loud fault (`MscntReadTimeout` / `GconfVerifyFailed`)
   instead of trusting the value. No garbage ever reaches the coil math.

2. **F4 blocking-read self-deadlock.** A helper that acquires `phase_spi_busy`
   then calls the spin-acquiring `spi_transfer` deadlocks. **Resolution:** the
   register helpers acquire `phase_spi_busy` **once** and call only
   `spi_transfer_locked` (the "caller already holds the flag" variant,
   `phase_stepping_spi.h:48-64`); never the spin-acquiring `spi_transfer`. Add a
   debug assert that the flag is held on entry to `spi_transfer_locked`. Phase
   motors are on the H7 here, but the F4 path is made correct too.

3. **oid → motor_idx must be one shared helper.** `motor_idx` is derived today
   only by an inline scan in the ISR (`dispatch_stepper.rs:422-451`: count the
   j-th `tmc_cs` stepper on the axis, walk `phase_slot_idx[m]` for the m-th slot
   matching `axis_idx`). **Resolution:** extract it to
   `phase_handover::motor_idx_for(shared, axis_idx, axis, stepper) -> Option<u8>`
   and call it from **both** `dispatch_phase` and the new `enter_group`/exit so
   the preload targets the exact physical driver the ISR drives. Fail loud
   (`PhaseMotorUnmapped`, -313, already exists) if any payload oid resolves to no
   slot.

4. **Bit-exact preload.** No second formula: `coil_for_phase(mscnt)` returns
   `PHASE_LUT[mscnt & 0x3FF]` — the identical table the ISR indexes
   (`phase_lut.rs:6`). Bit-exact by construction; a unit test asserts the table
   is what the host formula produced.

5. **Motion-active TOCTOU, widened by the long dance.** `set_axis_mode` checks
   `armed.is_some()` but there's no lock between the gate and the mode flip; the
   new SPI dance lives inside that window. **Resolution:** set a
   `handover_in_progress` flag (per-axis, in `AxisState`) that the piece-arm
   path checks and refuses to arm against; the group enter/exit sets it before
   the gate and clears it after the mode store. The arm path already rejects on
   a faulted/parked axis — extend it to also reject mid-handover.

6. **Global `phase_spi_writes_enabled` freezes all phase axes during the
   dance.** It is one flag (`phase_stepping_spi.c:15`). **Resolution / accepted
   scope:** today's host handshake *already* disables ISR writes globally for
   the entire ~160 ms; the MCU dance shrinks that window to sub-millisecond, so
   this is strictly better, not a regression. On this hardware the only
   phase-mode axes are the X/Y group being handed over anyway. Documented; a
   per-axis flag is a future refinement, not a prerequisite.

7. **en_pwm_mode (bit 2) cache divergence.** The host clears `en_pwm_mode` in
   the GCONF value it writes at enter (`tmc5160.py:580`) and never restores it;
   the field cache currently tracks this in the foreground. **Resolution:** the
   MCU's enter RMW clears bit 2 and sets bit 16; the exit RMW restores bit 2 to
   the host-cached value and clears bit 16, so chip GCONF reconverges with the
   host cache on exit (not only after the next homing `arm()`). The host passes
   its cached GCONF base (without `direct_mode`) once at `init_planner` time so
   the MCU knows the non-handover bits; the MCU never invents register payloads.

8. **Exit rotor-jump verification.** Before clearing `direct_mode`, the MCU
   reads chip MSCNT and asserts it equals `phase_enter_mscnt` and the
   step-generator phase within tolerance; mismatch → `PhaseExitDesync` (fail
   loud). Replaces the host-side mode-desync check (`tmc5160.py:636-648`).

## Changes

### C (`src/stm32/phase_stepping_spi.c` / `.h`, `src/stepper.c`, `src/stm32/spi.c`)

- Factor the H7 inline SPI block out of `phase_stepping_write_xdirect` into a
  static `phase_spi_xfer(bus, cs, buf, len, capture_rx)` that, when
  `capture_rx`, writes received bytes back into `buf` and returns 0 on success /
  non-zero on per-byte timeout.
- `phase_spi_read_register(motor_idx, addr, *out) -> int`,
  `phase_spi_write_register(motor_idx, addr, val) -> int`,
  `phase_spi_rmw_register(motor_idx, addr, mask, set_bits, *verified) -> int`.
  Each acquires `phase_spi_busy` once, uses `spi_transfer_locked`, releases.
  TMC address constants (`GCONF 0x00`, `CHOPCONF 0x6C`, `MSCNT 0x6A`,
  `XDIRECT 0x2D`, `GCONF_DIRECT_MODE 1<<16`, `GCONF_EN_PWM 1<<2`) replace the
  inline `0xAD` magic.
- Guard the F4 `spi_transfer` (`src/stm32/spi.c`) with the same
  acquire/release so MCU-initiated reads on F4 can't be corrupted by a TIM5
  XDIRECT write (closes the documented latent bug).
- `command_kalico_set_axis_mode_group` decodes `(axis_idx, mode, count, oids)`
  and calls `runtime_set_axis_mode_group`; maps each non-zero rc to a distinct
  `shutdown()` string per FaultCode. Delete the now-unused enable/disable/align
  DECL_COMMANDs.

### Rust runtime

- `error.rs`: add `MscntReadTimeout`, `GconfVerifyFailed`, `PhaseExitDesync`,
  `PhaseEnterPreconditionFailed` after -314; update `from_u16`, `code_name`,
  doctests, and `error/tests.rs`.
- `fault_helpers.rs`: `raise_*` for each, matching `raise_piece_start_in_past`.
- `phase_lut.rs`: `coil_for_phase(mscnt: u16) -> (i16, i16)` returning
  `PHASE_LUT[mscnt & 0x3FF]`.
- `phase_handover.rs`: extract `motor_idx_for(...)`; add `enter_group(...)` and
  `exit_begin_group(...)` / `exit_finalize_group(...)` implementing the ordered
  sequences below; reuse `find_stepper`/`align_to`/`jog_to`/`phase_of`.
- `stepping_state.rs`: `StepperRef.phase_enter_mscnt: AtomicI32`;
  `AxisState.handover_in_progress: AtomicBool`.
- `engine.rs`: `set_axis_mode_group(...)` delegating to `phase_handover`; declare
  the new `extern "C"` SPI symbols alongside `phase_stepping_write_xdirect`.
- `dispatch_stepper.rs`: replace the inline motor_idx scan with
  `motor_idx_for(...)`; teach the arm path to reject when
  `handover_in_progress`.

### Rust c-api

- `runtime_ffi.rs`: `runtime_set_axis_mode_group(rt, axis_idx, mode, oids_ptr,
  oid_len)` following the `runtime_set_axis_mode` pattern (null/init guards,
  ptr+len slice per B4).
- `include/runtime.h`: declare it; regenerate via `regen_headers.sh` so
  cbindgen-drift CI stays green.

### Host (`klippy/extras/tmc5160.py`, `klippy/motion.py`)

- `_enter_phase_mode_single` collapses to: send one
  `kalico_set_axis_mode_group mode=1` with the group oid list. Drop the
  foreground disable_spi/CHOPCONF/GCONF/MSCNT/XTARGET/enable_spi/align traffic.
- `exit_phase_mode` collapses to: send `mode=2` (begin walk), poll
  `kalico_get_phase_state` until `settled && phase == cached` (the MCU caches
  MSCNT now, so the host no longer needs `_cached_mscnt`), send `mode=0`
  (finalize). Keep `_echeck_helper.stop_checks()/start_checks()`.
- Send **one command per group** covering all `_phase_group_members()`, not one
  per motor.
- `motion.py init_planner`: pass each phase motor's cached GCONF base to the MCU
  (for the en_pwm_mode restore on exit).
- Homing (`tmc.py arm`/`disarm`): unchanged in shape — they call
  `exit_phase_mode()` / `enter_phase_mode()`, now resolving to the cheap
  command path. Host still owns the homing toggle (the MCU can't know a move is
  a StallGuard trip).

## MCU enter sequence (`mode=1`, synchronous)

1. Decode oids → `runtime_set_axis_mode_group(rt, axis_idx, 1, oids, len)`.
2. **Pre-validate the whole group, no SPI yet:** reject if any axis in the
   group has `armed.is_some()` (-31 MotionInProgress) or any oid is unmapped
   (-313) or lacks `tmc_cs` (PhaseEnterPreconditionFailed). Transactional gate —
   a partial group is never left half-entered. Set `handover_in_progress`.
3. `phase_stepping_disable_writes()` (suppress ISR XDIRECT for the dance).
4. Per stepper: `phase_spi_write_register(CHOPCONF, cached_chopconf)` to ensure
   `toff > 0` **before** `direct_mode` (charge-pump bootstrap; `direct_mode`
   with `toff=0` drains the caps → uv_cp).
5. Per stepper: `phase_spi_rmw_register(GCONF, mask=DIRECT_MODE|EN_PWM,
   set=DIRECT_MODE)`; read-verify the bits; mismatch → `GconfVerifyFailed`.
6. Per stepper: read MSCNT (fail-loud on timeout); cache in
   `phase_enter_mscnt`.
7. Per stepper: write XDIRECT to `coil_for_phase(mscnt)` so the driven angle
   already matches the chip — no torque step when `direct_mode` latches.
8. Per stepper: `align_to(mscnt)` (offset set directly, no walk);
   `last_phase_target = last_step_count + offset`.
9. Reset each group axis's step-queue head/tail (idle-proven).
10. `phase_stepping_enable_writes()`.
11. Store `axis.mode = Phase` (Release) for each group axis — last, so a
    concurrent dispatch sees fully-prepared state. Clear `handover_in_progress`.
12. Return 0 (no ack message; `set_axis_mode` is response-less).

## MCU exit sequence (`mode=2` walk, then poll, then `mode=0` finalize)

`mode=2`:
1. Pre-validate: every named stepper's axis is `mode == Phase` (else
   `PhaseExitDesync`); set `handover_in_progress`.
2. Per stepper: `jog_to(phase_enter_mscnt, max=PHASE_JOG_MAX_PER_SAMPLE)` while
   still in Phase so the ISR keeps driving XDIRECT through the walk. Return 0.

Host polls `kalico_get_phase_state` until `settled && phase == enter_mscnt`
(existing 0.5 s timeout, `tmc5160.py:662-683`).

`mode=0`:
3. Pre-validate settled; read chip MSCNT and assert `== phase_enter_mscnt` and
   step-generator phase within tolerance (else `PhaseExitDesync`).
4. `phase_stepping_disable_writes()`.
5. Per stepper: `phase_spi_rmw_register(GCONF, mask=DIRECT_MODE|EN_PWM,
   set=cached_en_pwm_bit)` — clear `direct_mode`, restore `en_pwm_mode` to the
   host-cached value; read-verify `direct_mode` cleared (else
   `GconfVerifyFailed`).
6. Reset each group axis's step-queue head/tail.
7. Store `axis.mode = Pulse` (Release). Clear `handover_in_progress`. Return 0.

## Tests

- `rust/runtime/src/phase_lut/tests.rs`: `coil_for_phase` equals
  `PHASE_LUT[i]` for all 1024 i.
- `rust/runtime/src/phase_handover/tests.rs`: `motor_idx_for` matches the old
  inline scan on representative bindings; enter_group rejects motion-active /
  unknown-oid / missing-cs; enter sets `mode=Phase` for all group axes; exit
  rejects desync; exit returns to cached MSCNT with zero first-tick delta.
- `rust/runtime/src/error/tests.rs`, `fault_helpers/tests.rs`: round-trip and
  raise for the new codes.
- `rust/runtime/tests/phase_handover_group.rs` (new): drive enter→exit through
  the engine with stubbed `extern "C"` SPI symbols (mirroring
  `test_xdirect_capture`); assert the C-call order (CHOPCONF before GCONF-set,
  GCONF read-verify, MSCNT read, XDIRECT preload, GCONF-clear read-verify).
- `test/test_tmc_enable.py`: expect the single group command, not the old
  per-motor SPI sequence.

## Implementation staging (bench-flashable increments)

The MCU C SPI register path and the real-time enter/exit cannot be compiled or
validated locally (MCU firmware builds on the Pi; per bench rule, never
cross-compile locally). Stage so each increment is independently flashable and
testable on the Trident:

- **Stage 0 (local, no hardware):** `coil_for_phase` + test;
  `motor_idx_for(...)` extraction reused by `dispatch_phase` + test; the new
  FaultCodes + tests. Compiles and `cargo nextest` locally; zero behavior change
  on hardware.
- **Stage 1 (enter, fixes the reported crash):** C register primitives +
  `command_kalico_set_axis_mode_group`; `enter_group` + FFI; host
  `_enter_phase_mode_single` collapse. Flash, jog with phase stepping, confirm
  no `-308`.
- **Stage 2 (exit):** `exit_begin/finalize_group` + host `exit_phase_mode`
  collapse; verify homing toggles cleanly with no rotor lurch and chip/cache
  GCONF reconvergence.
- **Stage 3 (cleanup):** delete the dead enable/disable/align commands and the
  host `_cached_mscnt` / angle math.

## Open questions

- Whether the homing `arm()` exit path can ever present a non-converged offset
  ramp (if so, `mode=2`→poll is mandatory; if always idle+converged, a future
  optimization could fold exit into one call).
- Reconnect-mid-phase: after an MCU restart while in phase mode, `_init_registers`
  re-asserts GCONF without `direct_mode` (chip drops to pulse) but the MCU mode
  word may disagree — define a force-`mode=0`-on-connect resync.
