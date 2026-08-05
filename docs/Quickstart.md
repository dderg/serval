# Quickstart: try Serval on your printer

This walks an existing Klipper/Kalico installation onto the Serval branch:
add a git remote, switch the branch, build the Rust host parts, reflash the
firmware, and migrate the configuration. Switching back is a branch
checkout plus a reflash.

Serval replaces the motion stack, so two things differ from a normal
Klipper update:

- the host needs native Rust artifacts built once per checkout, and
- the MCU protocol is different — **host and MCU firmware must be updated
  together**, in both directions.

## 1. Prerequisites

### Check your board first

The firmware builds for four STM32 families, plus a Linux-process MCU and
the host simulator. `make menuconfig` offers exactly:

- **STM32F4** — F401, F411, F405, F407, F427, F429, F446
- **STM32G0** — G070, G071, G0B0, G0B1
- **STM32H7** — H723, H743, H750
- **STM32F1** — F103, high-density parts only (xC/xD/xE). Builds and
  boots, but is **not supported** for printing — no FPU. See below.

Everything else mainline supports is absent from `src/Kconfig` on this
branch: AVR, LPC176x, RP2040, SAMD, HC32, and the STM32 F0/F2/F7/L4/G4
families. Each family needs its own motion-ISR tick path
(`src/stm32/runtime_tick_*.c`) and a Rust runtime built for its rustc
target; those four are the ones that exist.

This is a hard gate, not a rough edge. The MCU executes the trajectory, so
an unsupported chip cannot be worked around from the host side. Look up
your board's chip before spending time on the rest of this page.

#### If your board is an F103: not supported

The Cortex-M3 has no floating-point hardware, and this architecture
evaluates the trajectory on the MCU in float math. On the bench (SKR Mini
E3 v2.0, CoreXY Voron 0, 2026-07-30/31) the motion tick measured ~11,500
cycles per axis-sample in soft-float library calls — three streamed lanes
at 2 kHz consume 96% of the core, the MCU latches `-311
TickIntervalExceeded`, USB stops being serviced, and homing aborts. For
scale, an F446 (hard FPU, 180 MHz) runs four axes at 10 kHz. This is a
silicon limit: lowering the sample rate or the TMC UART baud made it
worse, not better. Get an F4/G0/H7 board.

What the port run established, kept for reference:

- The firmware boots, enumerates, connects, binds three lanes, executes
  streamed moves, and completed one sensorless `G28 X` — the port is
  *correct*, the chip is just too slow to hold the real-time contract.
- **`!PA14` is mandatory on the SKR Mini E3 v2.0.** Select "Enable extra
  low-level configuration options" and set "GPIO pins to set at
  micro-controller startup" to `!PA14`, exactly as the stock Klipper
  config for that board instructs. That pin gates USB: without it the
  board never enumerates and gives the host no logging whatsoever. Start
  from your board's stock config rather than writing one from the pinout.
- The part must be high-density (F103xC/xD/xE); the motion tick needs
  TIM5, which medium-density F103s lack.
- RAM is the binding constraint, not flash: ~11 KB left for klipper's C
  dynamic pool on a 48 KB RCT6.
- Every F1 timer is 16 bit, so the step-output compare chases far-future
  deadlines in <=455 us hops.
- F1 cannot do USB and CAN at once — they share one 512-byte packet
  buffer — so a bridge-mode F103 is not available.

### On the host

- A working Klipper or Kalico installation (`~/klipper`, klippy virtualenv,
  your web stack of choice).
- Native build dependencies. On a fresh image the package index is stale,
  so update first or the install 404s:

  ```bash
  sudo apt update
  sudo apt install pkg-config libudev-dev
  ```
- The Rust toolchain manager, [rustup](https://rustup.rs/):

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
  source "$HOME/.cargo/env"
  ```

  The build uses the toolchain pinned in `rust/rust-toolchain.toml`
  (1.85.0); rustup fetches it automatically on first build. The `source`
  line matters in the shell you install from — the build calls `cargo`
  straight off `PATH`, and a fresh rustup install only lands there in
  later shells.

## 2. Add the remote and switch branches

```bash
cd ~/klipper
git remote add serval https://github.com/dderg/kalico.git
git fetch serval
git checkout -b sota-motion serval/sota-motion
```

Your previous branch stays untouched; `git checkout <old-branch>` brings it
back at any time.

## 3. Build the host-side Rust artifacts

```bash
sudo service klipper stop
./scripts/build-native.sh
```

This produces `klippy/_config_doc.so`, `klippy/_motion_engine.so`, and
`klippy/_shaper_ident.so`. Klippy refuses to start without them — a plain
`git checkout` is never enough after Rust sources change; rerun the script
when you pull updates.

### On a 1 GB host, cap the build parallelism

Cargo builds one job per core by default. Four concurrent `rustc`
processes do not fit in 1 GB, and the kernel kills one partway through:

```
error: could not compile `shaper-ident` (lib)
Caused by:
  process didn't exit successfully: `.../rustc ...` (signal: 9, SIGKILL)
```

That signal 9 is the OOM killer, not a compiler bug — `dmesg` shows
`Out of memory: Killed process ... (rustc)`. Serialise the build instead:

```bash
CARGO_BUILD_JOBS=1 ./scripts/build-native.sh
```

Verified on a 1 GB Raspberry Pi 4 running MainsailOS: the default build
was OOM-killed after 12 minutes, and `CARGO_BUILD_JOBS=1` completed. Use
`2` if the host has 2 GB. The three artifacts total about 130 MB on disk,
most of it debug info that is never mapped into klippy's memory.

## 4. Rebuild and flash the MCU firmware

The firmware speaks a different protocol than mainline (it executes
trajectory pieces, not a step queue), so every MCU must be reflashed from
this branch:

```bash
make clean
make menuconfig   # select your board as usual
make
```

Flashing mechanics are unchanged — follow
[Installation](Installation.md#building-and-flashing-the-micro-controller)
for your board's flash method. Repeat for every MCU in the printer.

The motion sample rate is a per-target build option; the default is right
for typical boards. It lives under "Enable extra low-level configuration
options" in `make menuconfig`, alongside the piece-ring and `rt_storage`
size ceilings. All three are derived from the processor model, so you
should not need to touch them.

## 5. Migrate the configuration

The config model is intentionally different: `[kinematics]`,
`[motor <name>]`, `[axis <name>]`, and `[post_processor <name>]` replace
the role-encoded `[stepper_*]` sections, `[input_shaper]`, and the
`[extruder]` motion options. The full walkthrough, with a worked
before/after example and an option mapping table:

**[Config migration guide](Config_Migration.md)**

## 6. Start it

```bash
sudo service klipper start
```

If the config parser rejects an option, it fails loudly with the option
name — work through the messages against the
[migration guide](Config_Migration.md) and the
[motion config reference](Config_Reference_Motion.md).

## Switching back

```bash
sudo service klipper stop
cd ~/klipper
git checkout <old-branch>
make clean && make menuconfig && make   # reflash mainline firmware too
sudo service klipper start
```

The protocol difference cuts both ways: after switching branches, always
reflash the MCUs to match the host.
