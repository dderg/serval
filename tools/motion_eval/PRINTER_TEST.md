# Printer test: `[corner_deviation]` at the owner operating point

This procedure checks the offline findings of FINDINGS.md, "Owner operating point (50k, scv 50, 200/130 Hz)", on the owner's printer. The printer runs corexy, max_accel 50 000 mm/s², smooth_mzv at X 200 Hz / Y 130 Hz, and minimum_cruise_ratio 0.5. The offline claims under test:

1. **Executed acceleration overshoots at corners.** At scv 50, smooth_mzv turns each corner velocity step into an acceleration burst. On the owner corpus the executed peak is 1.62× max_accel on X and 1.22× on Y. It is 1.15× at scv 20 and 1.02× at scv 5. With the burst bounded at 1.03 × max_accel it is 1.012× (FINDINGS.md's corner_deviation row, run while that bound was the default; it is now `max_corner_accel: 51500`). The bound is opt-in now. The default floor mode no longer limits the burst, and its corpus peak is about 1.48× X / 1.24× Y.
2. **Ringing of the tuned mode does not grow with scv.** For an oscillator at the shaper frequency (ζ 0.05), the residual is 8–10 µm from scv 5 to 80. Untuned higher modes do grow: at 2.5× f the residual is 13 µm at scv 5, 21 µm at scv 50 and 15 µm with the bounded corner_deviation. At 4× f it is 1.7 → 3.5 µm.
3. **The bounded corner_deviation is about 5.4% slower than scv 50** (owner corpus 105.8 s against 100.4 s). The unbounded floor default is +0.7% (101.1 s); the printer_test spans below compare them file by file.

The cap mode is removed from `[corner_deviation]`: on fuzzed corner clusters it rounded corners 5–17% beyond its deviation limit. Besides scv, the protocol therefore runs floor (the default, unbounded burst) and bounded (max_corner_accel 51500).

The hardware answers what the harness cannot. Does the toolhead see the burst? Do real modes ring as the oscillator model says? Do the motors keep up with the burst, or skip?

## Files

`uv run python -m tools.motion_eval.printer_test` regenerates everything below, deterministically. It replans every file through `run_klippy` on `configs/owner.cfg`. It asserts that:
- the motion stays at least 50 mm inside the 350×350 bed;
- every corner is followed by a constant-velocity window of at least 5 periods of 0.8 × 130 Hz;
- each file aligns unambiguously against its own plan;
- `stress_scv50` reaches at least 1.6× max_accel;
- `corners_bounded` and `stress_bounded` stay at or below max_corner_accel, i.e. 1.03× max_accel per axis.

| file | setting | motion | measured span |
|---|---|---|---|
| `printer_test/corners_scv5.gcode` | scv 5 | travel only, Z 10 | 11.21 s |
| `printer_test/corners_scv20.gcode` | scv 20 | travel only, Z 10 | 11.14 s |
| `printer_test/corners_scv50.gcode` | scv 50 | travel only, Z 10 | 11.04 s |
| `printer_test/corners_floor.gcode` | `[corner_deviation]` (unbounded default) | travel only, Z 10 | 11.07 s |
| `printer_test/corners_bounded.gcode` | `[corner_deviation]` max_corner_accel 51500 | travel only, Z 10 | 11.21 s |
| `printer_test/stress_scv50.gcode` | scv 50 | **extrudes** reference marks; stress travel at Z 10 | 46.19 s |
| `printer_test/stress_floor.gcode` | `[corner_deviation]` (unbounded default) | **extrudes** reference marks; stress travel at Z 10 | 49.75 s |
| `printer_test/stress_bounded.gcode` | `[corner_deviation]` max_corner_accel 51500 | **extrudes** reference marks; stress travel at Z 10 | 52.87 s |

Every file starts with `; motion_eval` headers:
- `start`;
- `setting`;
- `corner_deviation`, which selects the replay config: `none`, `floor` (the section with no options), or `bound<mm/s²>` for max_corner_accel, such as `bound51500`. Any other value fails;
- `measured_moves`, the planned move range between the two accelerometer macros;
- `sync_s`, the length of the opening sync motion.

`accel_residual` reads these headers to replan the file under the same setting.

### Corner pattern

Every corner pattern starts at X175 Y175 Z10 with a sync motion: seven out-and-back taps on X and Y, of different lengths and with different pauses between them. That signature has exactly one alignment against the accelerometer record; the second-best correlation is 0.31 of the best.

The pattern itself is a sequence of closed loops through the centre, one per speed:

| loop | turn | speeds mm/s | what scv 50 does there |
|---|---|---|---|
| square | 90° | 50, 200, 300 | junction 50 mm/s: a 50 mm/s step on each axis |
| diamond (square at 45°) | 90° | 50, 200 | steps on both axes at once |
| octagon | 45° | 120, 300 | junction 112 mm/s: a 79 mm/s step on one axis |
| star {8/3} | 135° | 200 | junction 25 mm/s |
| reversals X, Y, diagonal | 180° | 200 | full stop |

Each leg cruises 1.5 × 48 ms, plus room to accelerate. The constant-velocity window after each corner therefore holds at least 5 periods of a 104 Hz mode (0.8 × 130 Hz) after the smoother's support. The pattern has 125 corners and 126 windows. The executed peak acceleration of the plan, as a multiple of max_accel:

| file | X | Y | corexy motor a / b |
|---|---|---|---|
| corners_scv5 | 1.014 | 1.000 | 1.414 / 1.343 |
| corners_scv20 | 1.128 | 1.000 | 1.430 / 1.504 |
| corners_scv50 | 1.418 | 1.072 | 1.651 / 1.858 |
| corners_floor | **1.780** | 1.046 | 1.717 / **2.094** |
| corners_bounded | 1.005 | 1.000 | 1.414 / 1.323 |

On corexy, motor a moves with x + y and motor b with x − y. A diagonal move at max_accel therefore already asks one motor for 1.41×, with no burst.

**The unbounded floor default bursts like scv does, and here more than scv 50.** Floor mode references junction speed to the shaper's stop floor, not to scv. On the octagon's 45° turns it admits junctions up to 200 mm/s (scv 50: 112 mm/s), so the per-axis velocity steps, and the bursts they become, are larger. The X peak (1.780×) sits on such a 45° vertex. **The bounded run is the low-burst A/B**: max_corner_accel 51 500 keeps the plan at 1.005× X.

### Stress pattern

Each stress file prints three things:
1. On layer 1 (Z 0.2) at X115 Y175: a 30 mm prime line, a 20 mm cross, and the main scales of two verniers.
2. Then the stress block, travel only at Z 10 around X215 Y175, repeated 8 times.
3. Then the vernier scales on layer 1 and a second cross on layer 2 (Z 0.4), on top of the first.

The stress blocks, with the executed peak of each block on its own (generator output):

| block | geometry | scv 50: X / Y / motor a / motor b | floor (unbounded): X / Y / motor a / motor b | bounded: X / Y / motor a / motor b |
|---|---|---|---|---|
| zigzag_x | 20 × 6 mm lines along X, 0.1 mm jogs, 200 mm/s | 1.498 / 0.466 / 1.710 / 1.710 | 0.822 / 0.284 / 1.106 / 1.106 | 0.822 / 0.284 / 1.106 / 1.106 |
| zigzag_y | 20 × 12 mm lines along Y, 1 mm jogs, 400 mm/s | 1.063 / 1.245 / 1.610 / 1.611 | 1.091 / 1.240 / 1.609 / 1.609 | 0.788 / 1.002 / 1.112 / 1.112 |
| zigzag_diag_a | 6 mm lines along +45°, 0.1 mm jogs, 400 mm/s | 1.448 / 1.183 / **2.224** / 0.981 | 0.707 / 0.761 / 1.468 / 0.422 | 0.707 / 0.761 / 1.468 / 0.422 |
| zigzag_diag_b | the same along −45° | 1.448 / 1.183 / 0.981 / **2.223** | 0.707 / 0.761 / 0.422 / 1.468 | 0.707 / 0.761 / 0.422 / 1.468 |
| circle | r 2.5 mm, 0.7 mm chords (22-gon), 400 mm/s, 3 turns | **2.164** / 1.309 / 2.227 / **2.301** | **2.232** / 0.992 / 2.306 / **2.474** | 0.674 / 0.477 / 0.634 / 0.785 |
| reverse_x, reverse_y | 12 mm out and back, 400 mm/s | 1.000 / 1.000 | 1.000 / 1.000 | 1.000 / 1.000 |

**Whole file:**
- `stress_scv50` reaches an executed peak of 2.164× max_accel on X (108 000 mm/s²), 1.309× on Y, and 2.23× / 2.30× on the corexy motors.
- `stress_floor` (unbounded) reaches 2.232× on X, 1.240× on Y, and 2.31× / 2.47× on the motors.
- `stress_bounded` stays at 1.005× X, 1.003× Y and 1.468× per motor.

The jog length and speeds come from a sweep over line length (3–20 mm), jog (0.02–3 mm), speed (100–800 mm/s) and circle radius / chord. Each value is the one that maximised the executed peak.

A note on FINDINGS.md: 90° corners alone peak at 1.42× (corners_turn90) to 1.50× (zigzag with a 0.1 mm jog). The 1.62× there comes from voron_top_layers, from a travel move along a polyline arc of radius ≈ 2.5 mm at 330 mm/s. The sweep shows that this mechanism goes further:
- At scv 50 the junction limit lets a 22-gon of radius 2.5 mm run at 344 mm/s.
- That is under the centripetal limit (v²/r = 0.95× max_accel), but every chord also accelerates and decelerates at max_accel, and each junction is a 94 mm/s step.
- smooth_mzv's peaked kernel does not average these steps out, so the executed acceleration reaches 2.16×.

Unbounded floor mode drives the same circle at least as hard (2.23× X). The bounded run slows it until it peaks at 0.67×; this is where the bounded setting loses most of its time, and where scv 50 and the unbounded floor ask most of the motors.

## Prerequisites

1. **Accelerometer on the toolhead**, configured in printer.cfg, with `axes_map` set so that `accel_x` and `accel_y` read printer X and Y with the right signs. A wrong sign does not pass silently: `accel_residual` refuses a record whose axis correlates negatively with the plan ("check axes_map").
   - Use an ADXL345 (`rate: 3200`, its default, ±16 g) or an LIS2DW (1600 Hz, ±16 g).
   - Do not use an MPU-9250/6050: Klipper runs it at ±8 g, which is 78 000 mm/s², and the scv 50 bursts clip it. accel_residual fails on a clipped record.
2. **The printer config matches `configs/owner.cfg`**: corexy, `max_accel: 50000`, `max_velocity: 800`, `minimum_cruise_ratio: 0.5`, `[input_shaper] shaper_type: smooth_mzv`, `shaper_freq_x: 200`, `shaper_freq_y: 130`. If anything differs, copy owner.cfg, change it to match, and pass it as `--config`. The replay uses its [printer] and [input_shaper] sections.
3. **Two macros** in printer.cfg. Use your chip's name: `CHIP=adxl345` for an `[adxl345]` section, `CHIP=toolhead` for `[adxl345 toolhead]`.

   ```
   [gcode_macro MOTION_EVAL_ACCEL_START]
   gcode:
     ACCELEROMETER_MEASURE CHIP=adxl345

   [gcode_macro MOTION_EVAL_ACCEL_STOP]
   gcode:
     ACCELEROMETER_MEASURE CHIP=adxl345 NAME={params.NAME}
   ```

   The files call these macros themselves, so the record covers exactly the measured block. Both calls follow an `M400`. Starting or stopping a measurement flushes the lookahead queue, so the live plan matches the offline one only if the toolhead is already at rest there.
4. motan (`scripts/motan/data_logger.py`) can record the same accelerometer stream together with the trapq, and `motan_graph.py` plots them. That is useful for looking at a single corner. `accel_residual` reads only the ACCELEROMETER_MEASURE CSV.

## Config per setting

`[corner_deviation]` rejects any `SQUARE_CORNER_VELOCITY` in SET_VELOCITY_LIMIT. Group the runs so that each config change happens once.

| run | printer.cfg | after the edit |
|---|---|---|
| corners_scv5, corners_scv20, corners_scv50, stress_scv50 | no `[corner_deviation]` section (the file sets scv itself) | `FIRMWARE_RESTART` |
| corners_floor, stress_floor | `[corner_deviation]` with no options (unbounded default) | `FIRMWARE_RESTART` |
| corners_bounded, stress_bounded | `[corner_deviation]` plus `max_corner_accel: 51500` | `FIRMWARE_RESTART` |

`CORNER_DEVIATION_REPORT` and the `corner_accel_x/y` status (worst admitted burst) let you check on the printer that the bounded section is the one loaded.

The scv files leave square_corner_velocity at their own value until the next restart. The last one, scv 50, is the owner's usual value.

## Running a file

1. Home (`G28`). The corner files are travel-only at Z 10. Clear the bed area from X110 to X240, Y110 to Y240.
2. Print the file from virtual_sdcard (upload it in Mainsail/Fluidd and print it). Do not paste it into the console or print it over a slow link: if the gcode supply stalls, the live planner flushes early, and the plan no longer matches. accel_residual detects that and fails ("does not follow the global offset").
3. The file runs this sequence:
   1. `G90`, `M83`, move to X175 Y175 Z10 (the `start` header);
   2. `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=<scv>` (scv files only);
   3. `M400`, `MOTION_EVAL_ACCEL_START`, `G4 P250` (a quiet lead-in, used to measure the noise floor);
   4. the sync and the pattern;
   5. `G4 P100`, `M400`, `MOTION_EVAL_ACCEL_STOP NAME=<file stem>`.
4. Klippy writes the record to its temp dir on the host: `/tmp/adxl345-corners_scv50.csv`, or `/tmp/adxl345-<chip>-corners_scv50.csv` for a named chip. The console prints the path: "Writing raw accelerometer data to … file". Copy it off before a reboot clears /tmp:

   ```
   scp pi@printer:/tmp/adxl345-*.csv ./records/
   ```

Running each corner file twice gives a repeatability check: the two rows should agree.

## Analysis

```
uv run python -m tools.motion_eval.accel_residual \
    records/adxl345-corners_scv5.csv    tools/motion_eval/printer_test/corners_scv5.gcode \
    records/adxl345-corners_scv20.csv   tools/motion_eval/printer_test/corners_scv20.gcode \
    records/adxl345-corners_scv50.csv   tools/motion_eval/printer_test/corners_scv50.gcode \
    records/adxl345-corners_floor.csv   tools/motion_eval/printer_test/corners_floor.gcode \
    records/adxl345-corners_bounded.csv tools/motion_eval/printer_test/corners_bounded.gcode
```

Add `--config my_printer.cfg` if the printer differs from owner.cfg. Add `--accel A` if the print ran after a `SET_VELOCITY_LIMIT ACCEL=A`. The output is one markdown row per pair.

| column | meaning |
|---|---|
| offset s | CSV time minus plan time; any value is fine |
| ρ x / y | correlation of the low-passed record with the plan, per axis. Below 0.7 fails |
| sync peak / 2nd | normalised sync correlation, best and best elsewhere. A 2nd at 0.7 of the best or more fails as ambiguous |
| X/Y residual µm p50 / p99 / max | ringing displacement after corners, over the constant-velocity windows |
| X/Y ring Hz (fit/fallback) | median fitted ringing frequency, and how many windows used the fit or the fallback |
| X/Y corner accel / max_accel | measured peak toolhead acceleration within the smoother's reach of each corner, p50 / p99 / max, with the plan's executed maximum in brackets |

How the residual is computed:
- In a constant-velocity window the carriage does not accelerate. Whatever the accelerometer reads there is the toolhead's lag e″, with gravity and noise removed.
- A damped sinusoid is fitted to each window. The fitted pole converts e″ to e, so each window is reported at the frequency actually ringing.
- A fixed conversion at the shaper frequency would understate a 2.5× mode by 6.25×. The configured shaper frequency is used only as the fallback, for windows where no ring stands NOISE_SIGMAS (4) noise sigmas above the still-toolhead noise. Those windows report the noise floor, about 4σ / (2π f)²: roughly 0.4 µm on X and 0.9 µm on Y for an ADXL345.
- As in oscillator.residuals, the value is the largest |e| over the first period after the window starts.

Accuracy on synthetic data (tests/test_accel_residual.py): ADXL345 at 3200 Hz, 150 mm/s² noise, quantised, one mode per axis at 0.9 × 200 Hz and 2.5 × 130 Hz.
- The time offset comes back within 3.8 µs (an eightieth of a sample).
- Fitted frequencies are within 1% of the true ones.
- Per-window residuals match oscillator.residuals within 0.6% (X) and 0.2% (Y) at the median, 9% in the worst window, and 0.16 µm absolute.

The same synthetic toolhead, run through four corner files, shows what a confirming set of rows looks like. X carries a 0.9× mode (near the tuned frequency); Y carries a 2.5× mode (untuned):

| setting | offset s | ρ x / y | sync peak / 2nd | X residual µm p50 / p99 / max | X ring Hz (fit/fallback) | Y residual µm p50 / p99 / max | Y ring Hz (fit/fallback) | X corner accel / max_accel p50 / p99 / max (plan max) | Y corner accel / max_accel p50 / p99 / max (plan max) |
|---|---|---|---|---|---|---|---|---|---|
| corners_scv5 | 1234.567895 | 1.00 / 1.00 | 1.00 / 0.32 | 3.4 / 9.0 / 9.2 | 180 (117/9) | 4.7 / 7.6 / 7.6 | 325 (120/6) | 0.79 / 1.12 / 1.22 (1.01) | 0.80 / 1.33 / 1.33 (1.00) |
| corners_scv50 | 1234.567895 | 1.00 / 1.00 | 1.00 / 0.31 | 3.8 / 8.2 / 9.8 | 180 (116/10) | 6.2 / 15.0 / 15.0 | 325 (114/12) | 0.81 / 1.32 / 1.32 (1.42) | 0.95 / 1.58 / 1.59 (1.07) |
| corners_floor | 1234.567895 | 1.00 / 1.00 | 1.00 / 0.31 | 3.9 / 12.1 / 12.4 | 180 (121/5) | 5.6 / 11.3 / 11.9 | 325 (114/12) | 0.87 / 1.53 / 1.54 (1.78) | 0.91 / 1.57 / 1.58 (1.05) |
| corners_bounded | 1234.567895 | 1.00 / 1.00 | 1.00 / 0.32 | 3.4 / 9.1 / 9.2 | 180 (117/9) | 3.8 / 8.4 / 8.4 | 325 (120/6) | 0.79 / 1.13 / 1.23 (1.01) | 0.79 / 1.33 / 1.33 (1.00) |

What these rows show:
- The near-tuned X mode rings about the same at scv 5, scv 50 and bounded. The unbounded floor's larger 45° steps raise its X p99 (12.1 against 8–9 µm).
- The 2.5× Y mode doubles at scv 50, sits between at unbounded floor, and returns to scv 5's level with the bound.
- The measured corner acceleration is the plan seen through the toolhead's modes, not the plan itself. A mode near the burst's frequencies amplifies it (Y: 1.59 measured against 1.07 planned); a slower mode smooths it (X: 1.32 against 1.42).
- The motors drive the planned (carriage) acceleration. The measured column tells what the toolhead and the print feel.

### What confirms or refutes the offline findings

Compare rows with each other rather than against the model's absolute µm. The oscillator's ζ and mode are assumptions; the ratios between settings are the claim.

| finding | confirmed if | refuted if |
|---|---|---|
| burst reaches the toolhead (FINDINGS 1.) | the measured corner-accel max and p99 rank like the plan's: unbounded floor and scv 50 highest, then scv 20, then scv 5 ≈ bounded. The plan's maxima on X are 1.78 (floor), 1.42 (scv 50), 1.13 (scv 20), 1.01 (scv 5), 1.005 (bounded); on Y, 1.07 (scv 50), 1.05 (floor), 1.00 for the rest. Absolute measured values carry the toolhead's modes (see the example above) | scv 50 and floor measure no higher than scv 5 and bounded: the burst does not reach the toolhead, because belts or motors filter it, or the steps skip (see the stress test) |
| tuned-mode ringing flat in scv (FINDINGS 2.) | windows ringing near 200 Hz (X) or 130 Hz (Y): residual p50/p99 within run-to-run scatter across scv 5, 20, 50 and bounded. Offline: 8–10 µm at every scv for ζ 0.05 | residual at the shaper frequency rises steadily from scv 5 to scv 50 |
| higher modes grow with the burst | if the fitted ring frequency is ≥ 2× the shaper frequency, residuals grow from scv 5 to scv 50 by about 1.6× (13 → 21 µm offline at 2.5× f), with bounded close to scv 5 (15 µm) | no growth, or bounded worse than scv 50 |

Finding 3, the time cost, is the planner's own arithmetic, and the hardware cannot change it. The planned spans, against scv 50:

| setting | corners | stress |
|---|---|---|
| scv 50 | 11.04 s | 46.19 s |
| floor (unbounded) | 11.07 s (+0.3%) | 49.75 s (+7.7%) |
| bounded | 11.21 s (+1.5%) | 52.87 s (+14.5%) |

The stress gaps are larger than the corpus's (+0.7% floor, +5.4% bounded) because the stress is all small circles and dense zigzags. That is where the bound slows the motion most, and where floor mode stops the sharp turns next to the 0.1 mm jogs.

The fit/fallback counts show how much of the record contained a measurable ring. If most windows fall back, ringing after corners is below the noise floor at every setting. That is also a confirmation of finding 2, at the floor's resolution.

## Stress test: skipped steps and layer shifts

Purpose: find out whether the motors deliver the bursts, or skip.
- Offline, the executed peak of `stress_scv50` reaches 2.16× max_accel on X and 2.30× on a corexy motor.
- The unbounded floor default, `stress_floor`, goes further: 2.23× on X and 2.47× on a motor.
- `stress_bounded` is the A/B: it holds 1.005× cartesian.

Before the stress files:
- Preheat the nozzle and the bed for the loaded filament, home, and run your usual bed leveling/mesh: the reference marks are a first layer.
- The file does not heat anything. Klipper refuses to extrude cold (`min_extrude_temp`), so a cold start fails loudly.

The file then runs these steps:
1. **Before the stress.** On layer 1 (Z 0.2) it prints a prime line, a cross with 10 mm arms at X115 Y175, and the **main scales** of two verniers.
   - The X scale has 11 ticks parallel to Y at a 1.0 mm pitch, 14–18 mm above the cross centre.
   - The Y scale is the same rotated, 14–18 mm right of the centre.
   - Tick 0 of each scale is 2 mm longer.
2. **The stress block**, at Z 10 around X215 Y175, measured with the accelerometer.
3. **After the stress.**
   - On layer 1, the **vernier scales**: 11 ticks at a 0.9 mm pitch, starting where the main ticks end (18–22 mm). Again tick 0 is longer.
   - Then a second cross on **layer 2** (Z 0.4), directly over the first.

Reading:
- **Cross**: one clean cross means no shift. A doubled or stepped line means the toolhead frame moved by more than about a line width (0.4 mm) during the stress.
- **Vernier, per axis**: find the vernier tick that lines up with a main tick. Tick 0 means no shift. Tick +i, counted toward +X on the X scale and toward +Y on the Y scale, means the frame moved i × 0.1 mm toward +X / +Y. The range is ±0.5 mm. Use the cross to rule out a whole-millimetre shift.
- One corexy motor losing a full step moves the toolhead (Δx, Δy) = (±0.1, ±0.1) mm: rotation_distance 40 / 200 steps is 0.2 mm per motor step, and x = (a + b) / 2, y = (a − b) / 2. That shows as tick ±1 on both scales. Losses below 0.1 mm do not show.
- Run `accel_residual` on the stress CSV and its gcode too.
  - Its corner-accel column gives the peaks the toolhead felt. They are not the motor demand. The circle's bursts repeat every chord, about every 2 ms (~490 Hz). In the synthetic stress run, a 180 Hz X mode filtered the planned 2.16× down to a measured 1.33×, while a 325 Hz Y mode amplified the planned 1.31× to 2.52×.
  - An accelerometer cannot see a lost position. After a skip the motion keeps the planned acceleration profile, only displaced, so the alignment still passes. The marks are the position check. A skip can show as an isolated jolt in the record, far above both the plan and the other corners. Look for a measured max that stands out from the p99.

### Safety

- Start at lower acceleration. Before printing the file, run `SET_VELOCITY_LIMIT ACCEL=20000`, then 30000, 40000 and finally 50000 (pass the same `--accel` to accel_residual). The file does not change ACCEL.
- The burst is set by the junction speeds, not by accel, so the ratio to max_accel rises as accel falls. The absolute peak, which is what the motors must deliver, mostly falls with accel, though not on every axis (scv 50 Y at 20 000 is above 30 000). Offline, per stress file:

  | ACCEL | scv 50: X / Y / motor mm/s² | floor (unbounded): X / Y / motor mm/s² | bounded: X / Y / motor mm/s² |
  |---|---|---|---|
  | 20 000 | 43 800 / 49 400 / 56 200 | 38 200 / 24 800 / 41 800 | 39 000 / 24 800 / 42 400 |
  | 30 000 | 64 000 / 45 000 / 81 700 | 49 300 / 37 200 / 64 600 | 44 500 / 37 200 / 58 000 |
  | 40 000 | 103 400 / 62 500 / 109 400 | 69 400 / 49 600 / 85 600 | 47 900 / 44 300 / 63 600 |
  | 50 000 | 108 200 / 65 500 / 115 100 | 111 600 / 62 000 / 123 700 | 50 300 / 50 100 / 73 400 |

  max_corner_accel is an absolute bound: at every ACCEL, stress_bounded stays within 51 500 mm/s² per axis.
- Run stress_bounded first, then stress_floor and stress_scv50. Stay at the printer.
  - Grinding, a sudden bang, or the toolhead visibly jumping is a skipped step. Stop with `M112` (emergency stop).
  - Then re-home before anything else: the position is lost.
- Watch the stepper drivers. TMC drivers can report overtemperature or a short-to-ground under the circle block. The circle runs 3 turns of a 5 mm circle at 344 mm/s, eight times.
- Check the belt tension and the motor current before you conclude that the planner is at fault. A skip at 20 000 says more about the hardware than about scv.

## Reproduce

```
uv run python -m tools.motion_eval.printer_test
uv run pytest tools/motion_eval/tests/test_accel_residual.py -q
```
