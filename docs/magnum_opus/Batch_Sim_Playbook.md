# Batch-sim playbook (trusted offline predictor)

Step-by-step for running Klippy's batch mode end-to-end on a Linux
host against a fork + mainline worktree, to get print-time
predictions that are *relatively* reliable for comparing motion-
pipeline changes before burning filament.

**What this is**: a reproducible offline simulator. Feed it a gcode
file, it produces a predicted print time using the real Klippy
motion planner. Calibrated against hardware measurements to ~2% on
this repo as of 2026-04-20.

**What this is not**: a bit-exact model of hardware. Absolute
predictions have a systematic ~95–150 s under-prediction gap vs real
hardware (common to both mainline and fork; see
`~/.claude/projects/-Users-daniladergachev-Developer-kalico/memory/reference_klipper_sim.md`).
Use for **relative** comparisons (change X vs change Y), not
absolute-number forecasting.

## Prerequisites

- Linux host with:
  - `gcc`, `make`, `avr-gcc` (package `gcc-avr` on Debian)
  - `python3` + the Kalico klippy-env virtualenv (or any klippy-env
    with its `requirements.txt` deps installed)
- Two Klipper checkouts:
  - **Mainline**: Kalico `main` branch, at e.g. `~/klipper-main`
  - **Fork**: your working branch (e.g. `blend-arc` or
    `magnum-opus`), at e.g. `~/klipper`

The fork repo has `test/configs/hostsimulator.config` which is the
KConfig for the simulator *target* — we don't use this path (it
produces a dict without ADC_MAX, which blows up for configs that
declare heaters). We use **atmega2560** as the dict target instead,
matching Klipper's own test suite. See "Why not the simulator target"
below.

## One-time setup per worktree

Switch each Klipper tree to the atmega2560 build target, then
rebuild.

```sh
cd ~/klipper-main
cp .config .config.bak.$(date +%s)   # preserve whatever was there
cat > .config << 'EOF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_AVR=y
CONFIG_MACH_atmega2560=y
CONFIG_AVRDUDE_PROTOCOL="wiring"
CONFIG_CLOCK_FREQ=8000000
CONFIG_SERIAL=y
CONFIG_SERIAL_BAUD=250000
EOF
make olddefconfig
make clean
make -j4

# Repeat the same inside ~/klipper (the fork tree)
```

The build produces `out/klipper.dict` in each tree. That's the only
artifact batch mode needs (the `out/klipper.elf` is unused).

Sanity check:

```sh
~/klippy-env/bin/python3 -c 'import json; \
  d=json.load(open("$HOME/klipper-main/out/klipper.dict")); \
  print("ADC_MAX:", d["config"].get("ADC_MAX"), \
        "CLOCK_FREQ:", d["config"].get("CLOCK_FREQ"))'
# expect: ADC_MAX: 1023  CLOCK_FREQ: 16000000
```

If `ADC_MAX` is missing, you built the wrong target. Repeat.

## Sim config files

Two minimal printer configs, one per branch. These are in-repo at
`docs/magnum_opus/sim_main.cfg` and `docs/magnum_opus/sim_blendarc.cfg`.
Copy them onto your sim host (anywhere, e.g. `~/sim_main.cfg` and
`~/sim_blendarc.cfg`) before running.

Key invariants — keep these synced with your real printer's
`printer.cfg` for apples-to-apples comparisons:

- `max_velocity`, `max_accel`, `minimum_cruise_ratio`
- shaper `shaper_freq_*`, `shaper_type_*`, `damping_ratio_*`
- `corner_deviation`, `target_smoothing` (fork only)

The stepper pin definitions are irrelevant for timing — atmega2560
understands PA0 / PB0 / etc. and the sim MCU is never actually
connected. Pins just need to parse.

The configs include `[gcode_macro PRINT_START]` and `PRINT_END` stubs
that do `G28` / `M84`. This matters for timing accounting:
**sim's reported `print time` includes G28** (~269 s with the stub
config in a simulator without real endstops), so subtract it when
comparing to Mainsail's post-purge clock.

## Running batch mode

Single command, one per branch. Given a gcode file:

```sh
GCODE=~/Downloads/Voron_Design_Cube_v7_ABS_22m13s.gcode

# Mainline
~/klippy-env/bin/python3 ~/klipper-main/klippy/klippy.py \
    ~/sim_main.cfg \
    -i "$GCODE" \
    -o /tmp/bm_main.serial \
    -d ~/klipper-main/out/klipper.dict \
    -l /tmp/bm_main.log

# Fork
~/klippy-env/bin/python3 ~/klipper/klippy/klippy.py \
    ~/sim_blendarc.cfg \
    -i "$GCODE" \
    -o /tmp/bm_blend.serial \
    -d ~/klipper/out/klipper.dict \
    -l /tmp/bm_blend.log
```

Each run takes ~2-3 minutes for a 24-minute print. Results:

```sh
grep "Exiting.*print time" /tmp/bm_{main,blend}.log
# /tmp/bm_main.log:Exiting (print time 1557.150s)
# /tmp/bm_blend.log:Exiting (print time 1426.911s)
```

## Extracting the comparable number

Klippy's reported `print time` includes everything from first G1 in
the gcode onward, including PRINT_START (our stub = just G28). To
compare against Mainsail's post-purge clock, subtract G28 time:

```sh
# G28-only run — measure once per sim config
echo "G28" > /tmp/gcode_homeonly.gcode
~/klippy-env/bin/python3 ~/klipper-main/klippy/klippy.py \
    ~/sim_main.cfg \
    -i /tmp/gcode_homeonly.gcode \
    -o /tmp/bm_home.serial \
    -d ~/klipper-main/out/klipper.dict \
    -l /tmp/bm_home.log
grep "Exiting" /tmp/bm_home.log
# Exiting (print time 268.697s)
```

Post-G28 time: `total_print_time - 268.697s`. That's the apples-to-
apples vs Mainsail's post-purge measurement.

## Calibration offsets (reference, 2026-04-20)

Measured gap between batch-sim-post-G28 and real-hardware post-purge,
on the Voron cube + benchy reference gcode:

| Config | Sim post-G28 | Real | Offset (real − sim) |
|---|---|---|---|
| Mainline SCV=45 | 1288.5 s | 1441 s | **+152.5 s** |
| blend-arc cd=0.14, ts=0 (baseline) | 1365.7 s | 1472 s | **+106.3 s** |
| blend-arc cd=0.14, ts=0 + vsup rule | 1158.2 s | TBD | ~+12 s fork-specific |

The **delta** between configs in sim (e.g. fork − main = −130.3 s)
tracks the real delta within ~12 s — that's the useful signal. The
absolute offset is not (hidden shaper/stepcompress effects).

## Why not the simulator target

Klipper's native `src/simulator/` target (`CONFIG_MACH_SIMU=y`):

- Doesn't declare `ADC_MAX` as a firmware constant, so any config
  with a heater sensor (i.e., any real printer) fails at startup
  with `klippy.msgproto.error: Firmware constant 'ADC_MAX' not found`.
- Doesn't understand `PA0`-style pin names (no pin alias map for
  board-generic). Configs with GPIO-named pins fail with
  `ValueError: invalid literal for int() with base 0: 'PA0'`.

Both can be patched (one-line `DECL_CONSTANT("ADC_MAX", 4095);` in
`src/simulator/timer.c` fixes the first), but the atmega2560 target
has both for free and is what Klipper's own regression tests use.

The simulator target also doesn't build on macOS (Mach-O linker
rejects ELF section attrs). atmega2560 build-with-avr-gcc works on
any host with the toolchain.

## Running with different gcodes / settings

Keep `sim_main.cfg` and `sim_blendarc.cfg` synced with the real
printer's `printer.cfg` for max_velocity / max_accel /
minimum_cruise_ratio / shaper. Any mid-print `SET_VELOCITY_LIMIT
ACCEL=...` in the gcode overrides the config, so if your gcode
drops accel on layer 1 for bed adhesion, the sim honours it.

For testing new motion-pipeline changes:

1. Edit fork source tree.
2. No rebuild needed for klippy Python changes (only dict is
   from firmware build, and we rarely touch that).
3. Rerun batch-mode sim.
4. Compare predicted time against baseline.
5. Expect ~2% precision on relative deltas.

Hardware validation is still needed at the end, but sim lets you
iterate dozens of design choices per hardware-test.
