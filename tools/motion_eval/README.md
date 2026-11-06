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
uv run python -m tools.motion_eval.corpus.build_corpus  # regenerate corpus/
```

The gcode must not home: the harness puts `SET_KINEMATIC_POSITION` at `--start`
in front of it and `M400` after it. One MCU, linear kinematics (cartesian,
corexy, ...) and a shaper that does not change after motion starts are required;
anything else fails with an error.

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
- `corner_deviation`: the `[corner_deviation]` status (junction counters and
  `max_deviation`) when the config has that section, else null.
- `timing`, `moves_planned`: wall time spent in lookahead (`add_move`/`flush`)
  and in the junction limiter, and the number of planned moves.
- `steppers.*.model_vs_steps`: worst model-vs-step error and its allowed bound.
- `samples` (with `--sample-dt`): commanded and executed position, velocity,
  acceleration and step positions over time.
