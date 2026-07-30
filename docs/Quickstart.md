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

The firmware only builds for three STM32 families, plus a Linux-process
MCU and the host simulator. `make menuconfig` offers exactly:

- **STM32F4** — F401, F411, F405, F407, F427, F429, F446
- **STM32G0** — G070, G071, G0B0, G0B1
- **STM32H7** — H723, H743, H750

Everything else mainline supports is absent from `src/Kconfig` on this
branch: AVR, LPC176x, RP2040, SAMD, HC32, and the STM32 F0/F1/F7/L4/G4
families. Each family needs its own motion-ISR tick path in the firmware
(`src/stm32/runtime_tick_*.c`), and those three are the ones that exist.

This is a hard gate, not a rough edge. The MCU executes the trajectory,
so an unsupported chip cannot be worked around from the host side. A
popular example: the SKR Mini E3 v2 is an STM32F103, so a printer built
around one cannot run this branch until the board is replaced. Look up
your board's chip before spending time on the rest of this page.

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

The motion sample rate is a per-target build option in `make menuconfig`;
the default is right for typical boards.

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
