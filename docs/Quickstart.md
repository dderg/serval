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

- A working Klipper or Kalico installation (`~/klipper`, klippy virtualenv,
  your web stack of choice).
- The Rust toolchain manager, [rustup](https://rustup.rs/):

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```

  The build uses the toolchain pinned in `rust/rust-toolchain.toml`
  (1.85.0); rustup fetches it automatically on first build.
- Native build dependencies:

  ```bash
  sudo apt install pkg-config libudev-dev
  ```

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
