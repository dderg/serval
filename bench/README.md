# A6-EC / AS715N EtherCAT bench

Minimal SOEM-based bring-up for the STEPPERONLINE **A6-EC** (OEM: **ANCTL AS715N**)
EtherCAT AC servo. `ec_spin.c` brings the drive up in **CSP** (Cyclic Synchronous
Position) over Distributed Clock and streams a hardcoded position trajectory each
cycle, in one of two profiles:

- **`sine`** (default) — gentle `A·(1−cos ωt)` oscillation: 1 rev peak-to-peak,
  4 s period, 3 cycles. Starts/ends at zero velocity, so enable and disable are
  bump-free and there are no velocity discontinuities.
- **`ramp`** — the original constant-velocity there-and-back: 30 rpm forward 3 s,
  reverse 3 s, hold, de-energize. The instant FWD→REV reversal is a velocity step
  that spikes following error — useful as the "violent" counterexample to `sine`.

Both share the identical DC bring-up; only `traj_offset()` differs.

Deliberately simple, known-good reference code (CLAUDE.md "no-throwaway"
exception) — the foundation the real Rust implementation builds on, not the
final architecture.

## Hardware / host
- Drive: **A6-200EC** (ANCTL AS715N), 100 W motor, 17-bit absolute encoder, 1:1 gear
  → **131072 counts/rev** (30 rpm = 65536 counts/s).
  Vendor `0x00400000`, Product `0x00000715`, Rev `0x00002EF8`, CiA402.
- Host: Raspberry Pi 3B, Raspberry Pi OS (kernel 6.12 `PREEMPT`), **SOEM v1.4.0**.
- `eth0` = dedicated bare EtherCAT port (NetworkManager-unmanaged, admin-up, no IP).
  Pi `eth0` → drive **CN3 (IN)**.

## Build (on the Pi)
```
S=~/ethercat/SOEM
gcc -O2 -Wall -o ec_spin ec_spin.c \
  -I$S/soem -I$S/osal -I$S/osal/linux -I$S/oshw/linux -I$S/oshw \
  $S/build/libsoem.a -lpthread -lrt -lm
```
SOEM v1.4.0 needs `-Werror` removed from its `CMakeLists.txt` to build on GCC 14.

## Run
```
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
sudo ./ec_spin eth0 [cycle_us] [sine|ramp]   # defaults: 2000 us = 500 Hz, sine
```

## Hard-won gotchas — the A6-EC DC-sync handshake
1. **Drive supports ONLY DC SYNC0** (`1C32:04 = 0x04`) but powers up declaring
   SM-sync (`1C32:01 = 1`, a mode it doesn't even support). Write `1C32:01 = 2`
   and `1C33:01 = 2` in PRE-OP, else **Er74.1 "no sync signal"** (the ESC pulses
   SYNC0 but the firmware never arms its handler).
2. **SYNC0 must be ACTIVE before the SAFE-OP transition.** The drive validates DC
   config on entering SAFE-OP; DC declared + SYNC0 cycle register still 0 →
   **AL 0x0030 "invalid DC sync configuration"** and it never reaches OP. So:
   `configdc()` + `dcsync0(TRUE)` in PRE-OP, *then* `config_map()` (→ SAFE-OP).
3. Phase-align the DC loop (PI on `ec_DCtime`) for ~1 s before requesting OP.
4. CiA402 fault reset = **rising edge** of controlword bit 7 — pulse it, don't hold.
5. **Real-time is required** for stable DC on the Pi 3B's USB-attached NIC:
   `SCHED_FIFO` + `mlockall` + performance governor + pinned core. PREEMPT_RT not
   needed at 500 Hz.

## Verified
2026-05-30, Pi 3B @ 500 Hz / 2 ms, `wkc=3`, `AL=0x0000`, `toff` within ~±150 ns:
- `ramp`: clean CSP move; following error peaked ~3258 counts in a single cycle at
  the FWD→REV velocity step.
- `sine`: smooth tracking, `ferr` ~±2900 gliding through zero at each turnaround
  (no reversal spike). Grabbing the shaft by hand mid-swing drove `ferr` to ~−4200
  and `trq` from ~−20 to −69 (the loop fighting back), then it recovered the instant
  it was released — confirms the position loop is genuinely closed on the streamed
  setpoints, with fault-threshold headroom above a manual load.

## `kalico-ethercat-rt` — the Rust motion-node endpoint (Plan 1)

`bench/ec_spin.c` is the throwaway reference; the real implementation is the
`kalico-ethercat-rt` crate (`rust/kalico-ethercat-rt`). It reuses this bench's
DC/CSP bring-up — extracted into `libecrt.c`/`libecrt.h` as a C shim — but instead
of a hardcoded `traj_offset()` it decodes the **kalico-native piece stream** off a
Unix socket and streams the evaluated trajectory as encoder counts. This is the
standalone first half of the EtherCAT motion-node design (other axes faked, no
Klipper yet — Klipper integration is Plan 2).

### Build (on the Pi)
```
cd ~/kalico/bench && make            # builds libecrt.a against ~/ethercat/SOEM
cd ~/kalico/rust && \
  ECRT_LIB_DIR=~/kalico/bench SOEM_LIB_DIR=~/ethercat/SOEM/build \
  cargo build --release -p kalico-ethercat-rt --features hw -j2
```
The `hw` feature gates the EtherCAT FFI + native link; without it the crate builds
and its 12 unit tests run on any dev machine / CI with no C libraries.

### Run
```
# endpoint (root: raw EtherCAT socket). Relaxes the UDS to 0o666 so a non-root
# client can connect.
sudo ./target/release/kalico-ethercat-rt eth0 --socket /tmp/kalico-ethercat.sock
# client: one gentle there-and-back move (20 mm * 3276.8 counts/mm = 65536 counts
# = 0.5 rev, 2 s each way), stamped on the shared CLOCK_MONOTONIC timeline.
./target/release/ec-test-client --socket /tmp/kalico-ethercat.sock --mm 20 --secs 2
```

### Verified
2026-05-30, Pi 3B @ 1 kHz / 1 ms, `wkc=3`, `err=0x0000` throughout. The endpoint
armed the pushed segment (`active=true`), captured the origin at the rotor's
resting position, swept the full ~65536-count excursion (≈0.5 rev) out and back,
and returned to the start point (`ferr` settling to ±2 at rest). Transient `ferr`
peaked ~1200–1670 counts at peak velocity — position-loop lag on an untuned
half-rev-in-2s move (≈4.6° on the 131072-count/rev encoder), no fault.

### Cross-process clock — the bug that cost a test cycle
First hardware run didn't move: the motor held position, no trajectory played.
Root cause: the endpoint and client each used `std::time::Instant`, whose epoch is
**per-process**. The client's `PushSegment.t_start`/`t_end` were therefore
meaningless on the endpoint's timeline — they landed ~16 s in the endpoint's past,
so the segment was `is_done` on arrival and never sampled. Fix: both binaries read
the **host-wide `CLOCK_MONOTONIC`** epoch directly (`src/clock.rs` via
`clock_gettime`), which every process on the machine shares; the client stamps
`t_start = monotonic_ns() + 150 ms` lead. (Plan 2's `motion-bridge` will negotiate
the host↔endpoint reference on this same primitive.)

### `ErC1.1` "Synchronization loss" after killing the endpoint
Killing the endpoint mid-OP stops SYNC0 + cyclic PDO delivery abruptly, so the
drive latches **`ErC1.1`** (CoE `0xC11` / emergency `0x8700`, *resettable* —
manual Ch.10). It is benign: the bring-up's CiA402 loop detects the Fault bit
(`sw & 0x0008`), pulses fault-reset (controlword bit 7), and — because we bring up
DC/SYNC0 *before* the enable walk (the A6-EC SYNC0-before-SAFE-OP quirk) — SYNC0 is
already flowing when the reset fires, so the next launch self-clears it and reaches
operation-enabled (`sw=0x1637`). No manual intervention needed. (A clean SIGTERM
shutdown that disables the drive before dropping SYNC0 would avoid the latch
entirely — a future nicety, not required.)
