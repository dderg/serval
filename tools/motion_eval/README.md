# motion_eval

Offline evaluation of what the planner and input shaper do with a gcode file.
Runs unmodified klippy in batch mode (`klippy.py -i gcode -o out -d dict`), so no
printer or MCU is needed.

```
uv run python -m tools.motion_eval CONFIG GCODE --start X,Y,Z [-o result.json] \
    [--png plot.png --sample-dt 0.0001]          # --png needs: uv run --group graphs
uv run pytest tools/motion_eval
python3 ~/Developer/serval/scripts/slots.py run -- \
    uv run python -m tools.motion_eval.sweep --jobs 12   # corpus sweep, FINDINGS.md
    uv run python -m tools.motion_eval.owner_point --jobs 12   # 50k / 200-130 Hz point
python3 ~/Developer/serval/scripts/slots.py run -- \
    uv run python -m tools.motion_eval.fuzz --seeds 0:10000 --jobs 12  # random gcode
KLIPPER_LINUX_MCU=.../klipper.elf \
    uv run python -m tools.motion_eval.live --case arcs   # live klippy CPU, one core
uv run python -m tools.motion_eval.corpus.build_corpus  # regenerate corpus/
```

The gcode must not home: the harness puts `SET_KINEMATIC_POSITION` at `--start`
in front of it and `M400` after it. One MCU, linear kinematics (cartesian,
corexy, ...) are required; a shaper change mid-print starts a new epoch, each
evaluated with its own kernels. Anything else fails with an error.

`oscillator.py` drives a damped oscillator per axis with the executed motion
and reports the free-vibration amplitude after corners, a ringing proxy for
tuned and untuned modes (`owner_point.py` uses it).

`fuzz.py` checks [corner_deviation] on random gcode (floor, executed accel
within what the planner admits or max_corner_accel, unchanged accels
and final positions, flush invariance); failing seeds go to
`tests/fuzz_seeds.txt`. `live.py` runs klippy in real time against the
linux-process MCU (KLIPPER_LINUX_MCU, built by kv2-notes/gate.sh) with
`step_recorder.c` preloaded and compares the live step stream with batch.
`object_probe.py` calls, in batch mode, the hooks only a live klippy calls
(statistics `stats`, every get_status) and measures reactor stalls.

`printer_test.py` writes the hardware test gcodes (`printer_test/`).
`accel_residual.py` is oscillator.py's hardware twin: it aligns a Klipper
ACCELEROMETER_MEASURE record with the replanned gcode and reports the measured
ringing and corner acceleration. Procedure: PRINTER_TEST.md.

Batch mode can exit with output still queued in the serial queue, dropping the
tail of the stream under load. `capture.py` waits for the queue to drain before
klippy disconnects.

## MCU dictionary

Batch mode needs the MCU data dictionary. `mcu_dict.build()` compiles the
firmware out of tree from a stock `test/configs/<name>.config` (`--mcu-config`,
default `stm32h723`, which needs `arm-none-eabi-gcc`; the printer.cfg pins must
exist on that MCU) into `tools/motion_eval/.build/<name>/out/klipper.dict`.
Equivalent by hand:

```
make OUT=tools/motion_eval/.build/stm32h723/out/ \
     KCONFIG_CONFIG=tools/motion_eval/.build/stm32h723/.config olddefconfig  # after copying the seed config there
make OUT=... KCONFIG_CONFIG=... -j8
```

`make` is incremental and runs under an flock on every call; runs read a
content-addressed copy (`.build/<name>/klipper-<sha256>.dict`), so parallel
runs never see a dictionary being rewritten.

## Output

- `print_time_s`: first trapq move start to last trapq move end.
- `commanded`: trapq moves as appended by the toolhead, per-axis peak
  velocity/accel and velocity jumps (corners taken at nonzero speed).
- `executed`: the motion the stepper kinematics receive after
  `kin_shaper.c`, reproduced as `sum_j w_j * cmd(t + s_j)` (shaper pulses, or a
  smoother kernel sampled on the time grid and applied by FFT). Every decoded
  step from the MCU stream is checked against this model (step at the midpoint
  crossing, at most `MAX_STEPCOMPRESS_ERROR` off); a mismatch raises
  `ModelMismatch`.
  - `peak_accel_mm_s2`: bounded part of the executed acceleration.
  - `max_velocity_jump_mm_s`: instantaneous velocity steps left after shaping
    (pulse shapers pass a fraction of each corner jump through; smoothers none).
  - `xy_path_deviation_model` / `_steps`: largest XY distance of the executed
    path (model, or the step positions) from the commanded segments within the
    shaper's time support.
  - `corners`: per vertex (a direction change between trapq moves), the larger
    of the executed path's distance to the commanded legs and the vertex's
    distance to the executed path; count, max, p99, p50 and the worst corner.
  - `final_position_model` / `_steps`.
- `phase_time`: time in accel, cruise and decel phases and between moves.
- `corner_deviation`: the `[corner_deviation]` status (junction counters,
  `corner_accel_x/y`) when the config has that section, else null.
- `timing`, `moves_planned`: wall time spent in lookahead (`add_move`/`flush`)
  and in the junction limiter, and the number of planned moves.
- `steppers.*.model_vs_steps`: worst model-vs-step error and its allowed bound.
- `samples` (with `--sample-dt`): commanded and executed position, velocity,
  acceleration and step positions over time.
