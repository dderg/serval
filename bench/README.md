# A6-EC / AS715N EtherCAT bench

Minimal SOEM-based bring-up for the STEPPERONLINE **A6-EC** (OEM: **ANCTL AS715N**)
EtherCAT AC servo. `ec_spin.c` brings the drive up in **CSP** (Cyclic Synchronous
Position) over Distributed Clock and runs a hardcoded velocity ramp by integrating
position: **30 rpm forward 3 s, reverse 3 s, hold, de-energize**.

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
sudo ./ec_spin eth0 [cycle_us]     # default 2000 us = 500 Hz; floor 250 us
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
2026-05-30, Pi 3B @ 500 Hz / 2 ms: clean CSP move, `wkc=3`, `AL=0x0000`, no faults,
following error peaked ~3200 counts (window 429483), `toff` within ~±150 ns,
returned to start position and de-energized.
