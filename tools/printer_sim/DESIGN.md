# printer_sim: a virtual printer behind motion_eval

motion_eval stops at the executed carriage motion: the planned and shaped
trajectory, plus a single-mode oscillator as a ringing proxy. printer_sim
adds the physics after that point, so a question like "does this planner
change ring, bulge or skip" can be answered without the owner's printer.
The chain is:

```
gcode -> klippy (motion_eval capture) -> step streams (X, Y and extruder)
      -> stepper + driver (2) -> rotor -> belts, carriage, gantry, toolhead (1)
      -> nozzle path
      -> extruder stepper -> filament + melt -> flow out of the nozzle (3)
      -> bead along the nozzle path, per-layer height field (4)
      -> wall / corner / width / shift / size metrics, PNG renders
```

Phase 1 built modules 1 and 2; phase 2 built 3 and 4, the metrics and the
renders. Module 5, validation, is a rule rather than a module: no number
below is claimed unless an analytic case behind it passes.

Code:

| file | role |
|---|---|
| `parts.py` | parts list |
| `mechanics.py` | module 1 |
| `motor.py` | module 2, analytic side |
| `extrusion.py` | module 3 |
| `deposition.py` | module 4 |
| `surface.py` | metrics |
| `render.py` | PNGs |
| `kernel.c` + `native.py` + `sim.py` | integration and rasterization |
| `drive.py` | klippy capture to commands |
| `gcodemoves.py` | slicer tags, matched to the planned moves |
| `gcodegen.py` | the test prints |
| `printjob.py` | one print end to end |
| `report.py` | phase-1 table |
| `printcompare.py` | phase-2 table |

Tests are in `tests/`. Commands:

```
PYTHONPATH=. uv run pytest tools/printer_sim/tests
PYTHONPATH=. uv run python -m tools.printer_sim print <gcode> --parts <cfg> --printer <printer.cfg> [--out DIR]
PYTHONPATH=. uv run python -m tools.printer_sim.report
PYTHONPATH=. uv run python -m tools.printer_sim.printcompare
```

The `print` command writes `report.json`, `wall.png` and `top.png`. It
needs:

- the first G0/G1 to give X, Y and Z, which is the start position;
- `;TYPE:WALL-OUTER` and `;LAYER:` tags (what `gcodegen` writes);
- even layers and convex outer walls, for the corner metric.

`motion_eval.capture` now also records the extruder's step stream
(`capture["extruders"]`).

## Inputs

- **Step stream.** `drive.from_gcode` replays a gcode through klippy,
  using the same `accel_residual.replay` the printer protocol uses, and
  takes every motor's recorded step times and positions. Each microstep
  holds its electrical angle until the next one, with no driver
  interpolation. The stream already includes Klipper's step+dir+step
  filter (`SDS_FILTER_TIME`, 750 µs).
- **Executed motion.** The shaped trajectory from motion_eval, sampled
  every 25 µs. It is the reference the nozzle is measured against: an
  infinitely stiff printer would follow it exactly. Its derivative is the
  smooth command rate that the motor damping acts against (see module 2).
  `report.py --continuous` drives the motors with it instead of the
  steps.
- **Parts list.** One file per machine, `parts/<name>.cfg`. Every value
  in it needs a line in its `[sources]` section, or `load` fails. The
  resonance targets, rotation distance and full steps are read from the
  klippy printer.cfg (`[input_shaper] shaper_freq_x/y`, `[stepper_x/y]`),
  so the plan and the model cannot disagree about them.

## Module 1: mechanics

Generalized coordinates, all SI:

- rotor angles θ_a, θ_b;
- the belt-driven carriage x (along the gantry) and the gantry y;
- with a `[local]` section, also the toolhead x_h, y_h, sitting on a
  local spring to the carriage.

Klipper's corexy drives stepper_x with a = x + y and stepper_y with
b = x − y, so the belts stretch by

$$e_a = r\theta_a - (x + y), \qquad e_b = r\theta_b - (x - y),$$

where r = rotation_distance / 2π. The mass matrix is diagonal:

| coordinate | mass |
|---|---|
| each rotor | J |
| x | m_c |
| y | m_c + m_g |
| toolhead, under `[local]` | m_h = toolhead − m_c |

Each spring is an elongation g·q, which adds k g gᵀ to K. That covers
the two belts, and under `[local]` the two local springs. Each motor holds
its rotor to the commanded angle through a magnetic spring. Linearized,
it adds k_mag = K·I·p (N·m/rad) on the rotor's diagonal (module 2).

**Damping is stiffness-proportional:** C = β K, the magnetic spring
included. This lumps material, belt and eddy-current losses in the parts
that flex. It has two useful properties:

- It puts no drag on steady motion. A dashpot to the frame would load the
  motors at cruise (37 N at 0.5 m/s, for the value it took to give Y
  ζ = 0.1). That is not physical, so it was rejected.
- The mode shapes stay the undamped ones, and mode i is damped by
  ζ_i = β ω_i / 2 exactly (test). One measured ratio therefore fixes β,
  and the rest follow: Y gets ζ = 0.1 · 130/200 = 0.065.

### Calibration

The calibration data is the owner's measured resonances, 200/130 Hz.
There are two readings of them, chosen per parts file.

**Reading A** (no `[local]`): 200/130 Hz are the belt modes, i.e. the
peaks of the accelerometer's response to commanded motion. That response
is `mechanics.frf`: sensor motion over commanded motion, which equals
SHAPER_CALIBRATE's accel/accel ratio.

- X fits whichever of belt stiffness and toolhead mass the file leaves
  out.
- Y fits the gantry mass.
- β gives the X mode the file's `damping_ratio`.

**Reading B** (`[local]`): 200/130 Hz are toolhead-local modes (hotend
and mount compliance, gantry twist) sitting above the belt modes.

- Belts, masses and gantry come from the parts list.
- X and Y fit the local springs so that the measured frequencies are
  undamped modes. The solve is exact, a rank-one update of K − ω²M (test).
- Whether those modes are also the largest response peaks, as a shaper
  tuned to them assumes, is reported (`response_peaks`) but not fitted.

**The owner's SHAPER_CALIBRATE CSVs tell A and B apart.** They hold the
full PSD per axis. Under A there is one dominant peak per axis, at
200/130 Hz. Under B with Voron belts the dominant peaks are near
66/33 Hz, and 200/130 Hz appear only as small secondary peaks (table
below). The CSVs are the one-time calibration input this model needs.
With them, the fit becomes the whole response curve rather than two peak
frequencies: the belt mode, the local mode and β per axis.

### Consistency check: what 200/130 Hz implies about the hardware

The motor's magnetic spring is in series with the belts. Two numbers
follow from that:

- Stock 24 V / 0.8 A: k_mag = K·I·p = 0.176 N·m × 50 = 8.8 N·m/rad.
  Reflected to the belt, k_mag/r² = 217 N/mm per motor.
- The rotor (84.5 g·cm², LDO datasheet) reflects to J/r² = 0.209 kg per
  motor.

A rotor on its own magnetic spring rings at √(k_mag/J)/2π: 162 Hz at
0.8 A, 242 Hz at 1.77 A.

| parts | rigid-belt ceiling of the X peak (433 g) | can reach 200 Hz? |
|---|---|---|
| stock 24 V / 0.8 A | 113 Hz | no, at no toolhead mass: the rotor alone is 162 Hz |
| 48 V / 1.77 A | 168 Hz | only with a toolhead ≤ ~0.19 kg and very stiff belts |

Reading A therefore needs the user-approved pin: belts at 3× the 48 V
magnetic stiffness, 1.44×10⁶ N/m per belt. That is 34 times the
Gates-derived 4.2×10⁴ N/m of a 2.4 m loop of 9 mm belt. The fitted
toolhead is then 165 g, and the gantry 654 g.

Published Voron 2.4 fits are X 63–91 Hz and Y 38–44 Hz
(printer-params.md §6). Reading B with Voron parts lands at 66/33 Hz.
So the owner's 200/130 Hz is either:

- a stiffer, lighter machine than a Voron (reading A), or
- local modes sitting above Voron-like belt modes (reading B). But a
  shaper tuned to 200/130 Hz would then leave those belt modes unshaped,
  and the model predicts large ringing and skipping (table).

## Module 2: stepper motor and driver

Per motor, with electrical angle φ = pθ (p = 50 rotor teeth for 1.8°):

$$v_a = R i_a + L\,\dot i_a - K\omega\sin\varphi,\qquad v_b = R i_b + L\,\dot i_b + K\omega\cos\varphi$$
$$T = K(i_b\cos\varphi - i_a\sin\varphi) - T_d\sin 4\varphi$$

- K = T_hold / (√2·I_rated). The holding torque is quoted with both
  phases at the rated current. The same K is the back-EMF constant in
  V·s/rad.
- Current amplitude I = √2 × run_current (RMS). The torque ceiling is
  therefore T_hold × run_current / I_rated: 0.176 N·m at 0.8 A, 0.389 N·m
  at 1.77 A.

**Driver.** A current chopper. Each integration step, it applies the
voltage that would bring each phase to the sine-table target at the end
of the step (dead-beat over the exact RL response), clipped to ±supply.
While the supply suffices, the current follows the command and
T = K·I·sin δ, where δ = φ_cmd − φ is the load angle. At speed, back-EMF
and ωL use up the supply and the current falls short and lags. That is
the torque–speed curve, derived rather than tabulated.

**Damping.** The motor's share of the damping, β·k_mag, is a lead of the
current vector by β times the speed error, against the smooth command
rate. It linearizes to the dashpot of module 1, and by construction it
cannot lift the torque above K·I. A plain dashpot to the command could
add torque beyond K·I once the rotor lags. A dashpot to the step stream
itself is a train of impulses.

**Pole slip.** The static stability limit is |δ| = π/2, where torque
peaks. A rotor pushed past π falls into the next stable point, 2π away,
which is 4 full steps or 0.8 mm of belt. The report counts these
crossings, `sim.pole_slips`, as the lost steps. It also reports the
largest |δ| (margin) and the final nozzle offset after a settle tail
(layer shift).

**Analytic torque curve.** `motor.pullout_torque` maximizes K·I·sin ψ
over current magnitude and angle, subject to the phasor voltage equation
|(R + jω_e L) I e^{jψ} + jKω| ≤ V. Per-phase clipping puts the simulated
motor between a sine of amplitude V and a square wave whose fundamental
is 4V/π (`pullout_bracket`).

| 2504AC | 0–1200 rpm (0–800 mm/s at r = 6.37 mm) | 2400 rpm (1600 mm/s) |
|---|---|---|
| 48 V / 1.77 A | 0.389 N·m (flat) | 0.347–0.388 N·m |
| 24 V / 0.8 A | 0.176 N·m (flat) | 0.085–0.129 N·m |

**Against the one published curve** (LDO-42STH40-2004MAH, 0.9°, digitized
±10 mN·m). The plot does not state its drive voltage, current or
microstep mode, so it cannot validate the model to ±10 mN·m:

| drive assumed | where it matches | where it misses |
|---|---|---|
| 36 V, 2.0 A RMS | 600–840 rpm falloff (model 249/192/178 mN·m vs plot 245/205/170) | plateau: model 350 vs plot 265–296 mN·m |
| 24 V | — | too low at speed |
| 48 V | — | too high at speed |

This is recorded as an open item, not a pass. The owner's
current-vs-skip test (protocol below) is what can close it.

## Numerics

Semi-implicit Euler at 2.5 µs (10 substeps per 25 µs sample):

1. currents (exact RL step under the clipped dead-beat voltage);
2. torques;
3. velocities;
4. positions.

Stiffness-proportional damping puts ζ = 0.27–0.4 on the 550–800 Hz modes,
and ωdt ≤ 0.013 at the stiffest mode, so the scheme is far from its
stability limit. Error is first order in dt, and the tests bound it by
Richardson differences at dt, dt/2 and dt/4.

Integration runs in C (`kernel.c`, built by `native.py` into `.build/`
and cached by source hash), because the hot spot was measured:

| version | cost per substep | per simulated second | full table (4 parts × 6 files, ~225 s of motion per parts set) |
|---|---|---|---|
| Python loop (a throwaway timing script, 4 DOF) | 7.4 µs | 3.0 s | ~45 min of one core |
| C kernel | — | 0.06 s | 71 s wall with 6 jobs, klippy replays included |

## Module 3: extrusion

- **Extruder rotor.** The motor of module 2 (LDO-36STH20 pancake, 0.5 A,
  24 V) drives the rotor through lever = rotation_distance / gear_ratio /
  2π (filament per radian; CW2 22.68 at 50:10). Its command is the
  extruder's recorded step stream, so Kalico's pressure advance and its
  `pressure_advance_smooth_time` smoothing are already in it.
- **Filament and melt.** They form one spring compressed by
  c = u − e_out. Its stiffness is k = back_pressure · A_f / τ, where
  back_pressure is the steady filament force per unit volumetric flow.
- **Nozzle.** It passes filament at ė_out = c / τ, capped at
  max_flow / A_f. With the rotor on its command this is exactly the
  first-order lag pressure advance assumes:

$$u = e_{out} + \tau\,\dot e_{out}$$

  Serval's `flow_metric.melted` makes the same lag (re-implemented, not
  imported).
- **Rotor load.** The filament force k·c loads the rotor. When the rotor
  cannot follow, it slips like an XY motor (pole slips, load angle).
- **Integration.** C (`ps_extruder_run`), on the XY grid.
- **Defaults.** The planning config's `pressure_advance` is set to the
  parts' τ (0.03 s). back_pressure (1 N per mm³/s) and the pancake's
  inertia are assumptions, flagged in the parts files.

## Module 4: deposition

**Bead.** The nozzle path (module 1) is cut into pieces at least half a
cell long. Each piece carries the volume the nozzle passed meanwhile
(module 3), so its bead is A = V / L in cross-section, flattened to the
layer height h. Width:

$$w = A/h$$

w is capped at max_width (1 mm) where the nozzle all but stops, giving a
blob taller than h.

**Rasterization.** `ps_deposit` spreads each piece's volume evenly over
the grid cells (20 µm) within w/2 of it.

- A line's cross-section comes out half-elliptical: 4h/π at the centre,
  with edges at ±w/2.
- The grid holds exactly the volume laid.
- Layers are assigned by the executed z and laid at their nominal height,
  with no interaction between layers.
- The top surface is, per cell, the base of the highest covering layer
  plus that layer's bead.

**Metrics** (`surface.py`), against the slicer's intent: the commanded
outer-wall line plus w_nominal/2.

| metric | definition |
|---|---|
| Ghosting | On every straight outer wall, with corner zones of two widths cut off: the nozzle's normal offset n·(p − a), resampled every 10 µm and linearly detrended. Amplitude √2·rms; spatial frequency from the Hann-windowed spectrum peak. |
| Line width | w = flow / (speed · h) along the same walls: mean, relative std, p1/p99. |
| Layer shift / size error | Per layer, the walls' mean edge offsets fit a translation plus a uniform outline offset. Layer shift is the largest translation from layer 0; size error is twice the mean outline offset. |
| Corner bulge / under-fill | On every fifth layer and the top, within 1.5 mm of each outer vertex: material outside the intended outline (bulge, mm³), and uncovered cells of the outer bead's band × h (under-fill). An exact capsule bead already rounds a 90° corner. |

**Renders.**

- `wall.png`: the outer edge offset (bead width included) unrolled along
  the perimeter versus height.
- `top.png`: the shaded surface height relative to the nominal top.

## Module 5: validation

| claim | test | tolerance |
|---|---|---|
| corexy rigid motion stretches no belt; K symmetric and PSD | `test_mechanics` | float |
| X and Y commands do not cross-couple | `test_mechanics` | 1e-12 relative |
| reading A puts the response peaks at 200/130 Hz | `test_mechanics` | 1e-6 |
| reading B puts undamped modes at 200/130 Hz (rank-one exact) | `test_mechanics` | 1e-9 |
| ζ_i = βω_i/2 on every mode, from the state matrix | `test_mechanics` | 1e-7 |
| stock 24 V / 0.8 A cannot reach 200 Hz (rotor alone below it) | `test_mechanics` | exact |
| unshaped accel step into one mode: textbook lag and decay e^{−2πζ/√(1−ζ²)} per period | `test_ringing` | Richardson bound |
| ZV at the true frequency and damping cancels the residual (≤ 1% of unshaped, within the numerical bound) | `test_ringing` | Richardson bound |
| below K·I the rotor holds, above it slips | `test_motor` | 3× the raised-cosine ramp residual |
| slip speed lies in the voltage-limited torque bracket | `test_motor` | α·(1/f_n + L/R) speed lag |
| a slip leaves whole electrical cycles (4 full steps) lost | `test_motor` | e⁻¹⁰ settle |
| step command holds each step; stays within ½ step (1 near SDS-filtered reversals) of the motion | `test_drive` | stepcompress bounds |
| ideal-rotor melt equals the analytic first-order lag u = e_out + τ ė_out | `test_extrusion` | Richardson |
| steady flow equals commanded flow with the real motor; steady force = back_pressure · flow | `test_extrusion` | 1e-3 |
| flow never exceeds the hotend cap | `test_extrusion` | exact |
| net volume out equals the filament commanded, up to the detent's static error | `test_extrusion` | asin(T_d/KI)/p |
| deposited volume equals the volume the nozzle passed | `test_deposition` | 1e-12 |
| steady straight line has width flow/(v h) | `test_deposition` | one cell |
| feed step: width follows the analytic first-order transient with τ | `test_deposition` | one cell + footprint blur |
| injected sinusoid A sin(2πft) reads back as ghosting A at f/v | `test_deposition` | 1/(πN) bias, half a frequency bin |

## Not modeled, so it still needs hardware

- **Frame.** Rigid and fixed. A rigid frame on compliant feet does not
  change nozzle-to-bed motion, because the bed rides on the same frame.
  Frame flex, gantry racking/yaw and twist beyond the lumped local spring
  are not modeled.
- **Belts.**
  - Stiffness is constant. Really it is EA/L₁ + EA/L₂ and varies with
    position, up to 3–5× near the ends.
  - No tooth meshing (20T polygon effect), pulley or idler eccentricity,
    or belt mass and span vibration.
- **Friction.** Rail friction and preload, and Coulomb friction anywhere.
- **Motor.**
  - Cogging beyond the sin 4φ detent; non-sinusoidal back-EMF and
    torque harmonics; magnetic saturation above rated current.
  - Iron and eddy losses as a speed-dependent drag (lumped into β).
  - Temperature: R rises ~40% hot, which lowers high-speed torque.
- **Driver.**
  - The real chopper period (20–40 kHz) and its current ripple.
  - StealthChop's voltage mode.
  - TMC MicroPlyer interpolation: steps are held, not interpolated.
  - Sense-resistor and sine-table errors; supply sag under load.
- **Step timing.** MCU timer jitter. The host-scheduled step times are
  used, which already include stepcompress error.
- **Air drag and cable chain or umbilical forces.**
- **The accelerometer's own mount and its 3.2 kHz sampling.** The sim
  reports the sensor coordinate directly.
- **Z and the bed.** Z moves are taken as executed; the bed is flat and
  rigid.
- **Extrusion.**
  - The melt is linear, while real polymer melts shear-thin: R_h falls
    with flow, so the right pressure advance changes with speed.
  - No temperature or heater limits. The hotend's flow limit is a hard
    cap, not melting capacity.
  - No die swell, no gear slip on the filament, no grinding, and no
    Bowden tube.
  - back_pressure, which sets the extruder load, is assumed.
- **Deposition.**
  - Beads do not interact: no spreading into neighbours, no squish
    against the previous layer, no sag, no surface tension, no cooling
    shrink.
  - Each layer is laid at its nominal height, so under- or
    over-extrusion does not propagate up the part.
  - Oozing in travel is only what the melt lag passes; there is no
    stringing, no wipe and no retraction physics.

## Validating against the real printer (once)

1. **SHAPER_CALIBRATE CSVs per axis** (the full PSD). Fit `frf` to the
   whole curve, not just the peaks. This decides reading A vs B, gives β
   per axis, and gives any second peak. It is the only calibration input
   the model needs beyond the parts list.
2. **Parts.** Motor part number, driver, supply voltage and run_current,
   toolhead mass (on a scale) and gantry type. They drop into one
   `parts/<owner>.cfg`.
3. **Printer protocol corner files with the accelerometer**
   (`accel_residual.py`). Compare the measured corner acceleration and
   residual per corner with the sim's sensor coordinate on the same
   gcode. Agreement within the measurement scatter validates modules 1
   and 2 on the owner's real trajectories.
4. **Skip threshold.** Run the stress files at the protocol's ACCEL
   ladder with the reference marks. The first ACCEL that shifts a mark,
   compared with the sim's first pole slip at that ACCEL, validates the
   torque curve where it matters. One run at a lowered run_current gives
   a second point.
5. **Prints, for modules 3 and 4.** Print `gcodegen`'s tower and cube
   with the same printer.cfg (`printcompare` writes them next to each
   result). Compare with the metrics:
   - wall waviness and wavelength, from a profilometer or a raking-light
     photo;
   - corner bulge;
   - size, measured with calipers against `size_error_mm`;
   - the predicted under-extrusion at wall ends from
     `pressure_advance_smooth_time` (phase 2 results).

## Phase 1 results

Default parts are the Voron-class list in
`~/Developer/kv2-notes/printer-params.md`: LDO-42STH48-2504AC, Gates GT2
9 mm, Stealthburner + CW2 433 g, damping ratio 0.1 (Klipper's default).
The four cases:

- **A, 48 V / 1.77 A**: calibrated, with the 3× belt pin; toolhead fitted
  to 165 g.
- **A, 24 V / 0.8 A**: cannot be calibrated (above), so its motors drive
  the A-48 V mechanics. Resonances are then 136/91 Hz, stated.
- **B**, both drivers: Voron belts (4.2×10⁴ N/m), toolhead 433 g with
  333 g on the carriage (assumed), gantry 1.35 kg, local springs fitted.

All runs use the step stream, owner.cfg (50 000 mm/s², smooth_mzv
200/130 Hz) and the printer_test gcodes.

Columns:

- **residual**: per constant-velocity window (cruise minus the kernel
  reach), the largest |nozzle − executed motion|, as p50 / p99 / max over
  the windows. This includes the ±6 µm microstep quantization.
- **max error**: over the whole file, corners included.
- **load angle**: the largest |δ| in electrical degrees; 90° is the
  torque peak.
- **slips**: pole slips per motor (each is 4 full steps).
- **final offset**: the nozzle's offset after the settle tail.

| parts | file | residual X p50/p99/max µm | residual Y p50/p99/max µm | max error X/Y µm | load angle A/B ° | slips A/B | final offset X/Y mm |
|---|---|---|---|---|---|---|---|
| A, 48 V / 1.77 A | corners_scv50 | 6.0 / 17.6 / 17.6 | 7.3 / 17.9 / 18.8 | 46 / 96 | 46 / 46 | 0 / 0 | -0.00 / -0.00 |
| A, 48 V / 1.77 A | corners_bounded81000 | 5.8 / 17.7 / 17.7 | 7.1 / 16.0 / 16.2 | 45 / 93 | 40 / 45 | 0 / 0 | -0.00 / -0.00 |
| A, 48 V / 1.77 A | stress_scv50 | 3.0 / 13.0 / 15.1 | 5.4 / 21.5 / 25.9 | 50 / 97 | 48 / 48 | 0 / 0 | 0.00 / 0.00 |
| A, 48 V / 1.77 A | stress_floor | 4.2 / 13.0 / 15.3 | 6.9 / 21.5 / 25.8 | 44 / 94 | 45 / 45 | 0 / 0 | -0.00 / 0.00 |
| A, 48 V / 1.77 A | stress_bounded | 4.0 / 12.9 / 14.9 | 5.6 / 21.4 / 25.9 | 44 / 93 | 45 / 44 | 0 / 0 | 0.00 / 0.00 |
| A, 48 V / 1.77 A | stress_bounded81000 | 4.7 / 13.0 / 15.3 | 7.1 / 21.4 / 25.8 | 44 / 94 | 45 / 45 | 0 / 0 | 0.00 / 0.00 |
| A, 24 V / 0.8 A (A-48 V mechanics) | corners_scv50 | lost (mm) | lost (mm) | 35.7 / 82.2 mm | — | 822 / 460 | 28.80 / 52.80 |
| A, 24 V / 0.8 A (A-48 V mechanics) | corners_bounded81000 | lost (mm) | lost (mm) | 64.9 / 122.6 mm | — | 798 / 590 | 64.80 / 122.40 |
| A, 24 V / 0.8 A (A-48 V mechanics) | stress_scv50 | lost (mm) | lost (mm) | 284.6 / 445.3 mm | — | 11781 / 11098 | -159.60 / 440.40 |
| A, 24 V / 0.8 A (A-48 V mechanics) | stress_floor | lost (mm) | lost (mm) | 292.9 / 422.3 mm | — | 11982 / 11540 | -104.00 / 134.40 |
| A, 24 V / 0.8 A (A-48 V mechanics) | stress_bounded | lost (mm) | lost (mm) | 291.7 / 580.0 mm | — | 10558 / 10867 | 155.60 / 574.00 |
| A, 24 V / 0.8 A (A-48 V mechanics) | stress_bounded81000 | lost (mm) | lost (mm) | 582.1 / 781.1 mm | — | 11679 / 11319 | 490.40 / -699.20 |
| B, 48 V / 1.77 A | corners_scv50 | lost (mm) | lost (mm) | 17.3 / 13.7 mm | — | 269 / 163 | 8.80 / 1.60 |
| B, 48 V / 1.77 A | corners_bounded81000 | lost (mm) | lost (mm) | 17.7 / 11.8 mm | — | 189 / 101 | 7.20 / -4.81 |
| B, 48 V / 1.77 A | stress_scv50 | lost (mm) | lost (mm) | 211.0 / 485.6 mm | — | 9333 / 9361 | -60.80 / 410.40 |
| B, 48 V / 1.77 A | stress_floor | lost (mm) | lost (mm) | 436.6 / 642.8 mm | — | 9327 / 9301 | 316.00 / 540.80 |
| B, 48 V / 1.77 A | stress_bounded | lost (mm) | lost (mm) | 198.0 / 581.6 mm | — | 7841 / 8395 | 188.80 / 495.20 |
| B, 48 V / 1.77 A | stress_bounded81000 | lost (mm) | lost (mm) | 179.1 / 391.2 mm | — | 8801 / 8726 | 45.20 / 386.00 |
| B, 24 V / 0.8 A | corners_scv50 | lost (mm) | lost (mm) | 142.9 / 156.0 mm | — | 1155 / 978 | 142.80 / 155.60 |
| B, 24 V / 0.8 A | corners_bounded81000 | lost (mm) | lost (mm) | 149.6 / 261.5 mm | — | 1093 / 1075 | 122.40 / -204.78 |
| B, 24 V / 0.8 A | stress_scv50 | lost (mm) | lost (mm) | 737.1 / 1589.0 mm | — | 14082 / 14058 | -527.20 / -1559.20 |
| B, 24 V / 0.8 A | stress_floor | lost (mm) | lost (mm) | 549.2 / 1502.4 mm | — | 13671 / 12864 | 42.00 / -668.40 |
| B, 24 V / 0.8 A | stress_bounded | lost (mm) | lost (mm) | 1017.2 / 1105.0 mm | — | 12673 / 12616 | 994.00 / -999.60 |
| B, 24 V / 0.8 A | stress_bounded81000 | lost (mm) | lost (mm) | 289.6 / 985.1 mm | — | 13498 / 13097 | 170.00 / 530.80 |

Reading the table:

- **Consistency check, not a planner finding.** Every parts set except
  A-48 V predicts skipping at 50 000 mm/s² on ordinary moves:
  - A-24 V's motors lack torque: 0.176 N·m against ~0.2 N·m for a plain
    50 000 mm/s² Y move with the fitted masses.
  - B's 1.78 kg Y mass needs ~0.35 of 0.389 N·m before any burst, and the
    66/33 Hz belt modes are left unshaped.

  The owner prints at 50 000 mm/s² without reported skipping, so neither
  is his machine. The sim says he runs something like 48 V and higher
  current, with a light, stiff motion system, or the shaper frequencies
  are not his belt modes.
- **Under A-48 V, the only consistent set,** the motors deliver the
  bursts. The largest load angle on the stress files is 48° (74% of
  pull-out), with no slips. scv 50, floor, bounded and bounded81000 differ
  by at most 2 µm in residual p99 and 6° in load angle. In this model the
  bursts are absorbed by the rotor inertia and the belt/magnetic
  compliance, not delivered to the nozzle.

The same A-48 V runs driven by the continuous executed motion (`--continuous`): residual p50 drops by about half and p99/max barely move. So the p50 is mostly microstep quantization. The p99/max is [inference, not separated by frequency] the 130 Hz Y mode (ζ 0.065 in the model against the shaper's assumed 0.1) and the unshaped 550/800 Hz modes. A-48 V does not slip either way.

| file | residual X p50/p99/max µm | residual Y p50/p99/max µm | load angle A/B ° |
|---|---|---|---|
| corners_scv50 | 3.4 / 17.2 / 18.2 | 6.3 / 11.8 / 11.9 | 45 / 42 |
| corners_bounded81000 | 3.3 / 17.2 / 18.2 | 5.9 / 10.6 / 11.0 | 38 / 41 |
| stress_scv50 | 1.8 / 12.8 / 13.5 | 2.7 / 21.7 / 25.6 | 42 / 42 |
| stress_floor | 2.8 / 12.8 / 14.1 | 4.3 / 21.7 / 25.6 | 40 / 40 |
| stress_bounded | 2.8 / 12.8 / 14.1 | 4.4 / 21.5 / 25.3 | 40 / 40 |
| stress_bounded81000 | 2.8 / 12.8 / 14.1 | 4.3 / 21.7 / 25.6 | 40 / 40 |

## Phase 2 results

Prints from `gcodegen`:

- **cube**: 20 mm, 30 layers, 3 perimeters, outer walls 300 mm/s, inner
  walls and ±45° fill 450 mm/s (25% sparse, solid bottom and top).
- **tower**: a hollow 40 mm square, 45 layers, 2 perimeters, outer walls
  300 / 450 / 600 mm/s in thirds of the height.

All prints use 0.2 mm layers and 0.45 mm lines, and owner.cfg at
50 000 mm/s² with smooth_mzv 200/130 Hz. Pressure advance is 0.03 s (the
parts' melt τ) at Kalico's default 0.04 s smoothing. Settings: scv 50,
`[corner_deviation]` default (floor), `max_corner_accel: 81000`, scv 5.
Renders and per-run reports are under
`tools/printer_sim/.build/prints/<print>/<parts>/<setting>/` (`wall.png`,
`top.png`, `report.json`, plus the gcode and printer.cfg used).

| print | parts | setting | print time s | ghosting µm median / max by outer-wall speed | corner bulge / under-fill mm³ (mean) | line width rel. std | layer shift mm | size error mm | slips A / B / E |
|---|---|---|---|---|---|---|---|---|---|
| cube | A-48 V | scv50 | 54.03 | 300 mm/s: 2.7 / 4.9 | 0.0275 / 0.0115 | 0.070 | 0.000 | -0.019 | 0 / 0 / 0 |
| cube | A-48 V | floor | 55.00 | 300 mm/s: 3.3 / 5.0 | 0.0282 / 0.0114 | 0.071 | 0.000 | -0.019 | 0 / 0 / 0 |
| cube | A-48 V | bounded81000 | 55.00 | 300 mm/s: 3.3 / 5.0 | 0.0282 / 0.0114 | 0.071 | 0.000 | -0.019 | 0 / 0 / 0 |
| cube | A-48 V | scv5 | 56.01 | 300 mm/s: 1.6 / 4.3 | 0.0340 / 0.0148 | 0.081 | 0.000 | -0.022 | 0 / 0 / 0 |
| cube | B-48 V | scv50 | 54.03 | lost steps: laid 707 mm outside the part | — | — | — | — | 12561 / 10289 / 0 |
| cube | B-48 V | floor | 55.00 | lost steps: laid 593 mm outside the part | — | — | — | — | 12926 / 10025 / 0 |
| cube | B-48 V | bounded81000 | 55.00 | lost steps: laid 593 mm outside the part | — | — | — | — | 12926 / 10025 / 0 |
| cube | B-48 V | scv5 | 56.01 | lost steps: laid 470 mm outside the part | — | — | — | — | 12186 / 10080 / 0 |
| tower | A-48 V | scv50 | 44.15 | 300 mm/s: 2.0 / 3.5, 450 mm/s: 2.1 / 3.4, 600 mm/s: 1.8 / 3.6 | 0.0514 / 0.0061 | 0.069 | 0.001 | -0.015 | 0 / 0 / 0 |
| tower | A-48 V | floor | 44.86 | 300 mm/s: 2.2 / 3.7, 450 mm/s: 2.5 / 4.9, 600 mm/s: 2.1 / 3.9 | 0.0524 / 0.0061 | 0.069 | 0.001 | -0.015 | 0 / 0 / 0 |
| tower | A-48 V | bounded81000 | 44.86 | 300 mm/s: 2.2 / 3.7, 450 mm/s: 2.5 / 4.9, 600 mm/s: 2.1 / 3.9 | 0.0524 / 0.0061 | 0.069 | 0.001 | -0.015 | 0 / 0 / 0 |
| tower | A-48 V | scv5 | 45.29 | 300 mm/s: 1.1 / 2.5, 450 mm/s: 1.5 / 4.4, 600 mm/s: 2.0 / 3.7 | 0.0572 / 0.0076 | 0.074 | 0.001 | -0.017 | 0 / 0 / 0 |
| tower | B-48 V | scv50 | 44.15 | lost steps: laid 238 mm outside the part | — | — | — | — | 2995 / 1710 / 0 |
| tower | B-48 V | floor | 44.86 | lost steps: laid 437 mm outside the part | — | — | — | — | 2810 / 1577 / 0 |
| tower | B-48 V | bounded81000 | 44.86 | lost steps: laid 460 mm outside the part | — | — | — | — | 3020 / 1829 / 0 |
| tower | B-48 V | scv5 | 45.29 | lost steps: laid 318 mm outside the part | — | — | — | — | 3006 / 1437 / 0 |

Reading it:

- **B-48 V loses thousands of steps on every print**, as in phase 1: its
  1.78 kg Y mass cannot be accelerated at 50 000 mm/s² by 0.389 N·m.
  This is a consistency check against the parts set, not a planner
  result.
- **Under A-48 V nothing slips**, the extruder included (load angle ≤ 61°
  at 54 mm³/s on the tower's 600 mm/s band).
- **Ghosting is 1–5 µm under every setting.** The shaper sits exactly on
  this model's modes. scv 5 rings least (1.1–2.0 µm median) and corner
  deviation most (2.1–3.3 µm), and the max differs by at most 1.5 µm.
- **Print time.** corner_deviation costs 1.6% (tower) and 1.8% (cube)
  against scv 50, and scv 5 costs 2.6% / 3.7%.
- **Corner bulge.** scv 5 bulges more at corners (0.034 against 0.028 mm³
  on the cube, 0.057 against 0.051–0.052 on the tower) because it stops
  longer in each corner while the melt keeps flowing.
- **floor and max_corner_accel 81000 plan identically here.** Junction
  speeds are equal on every move (checked); 90° corners at 300–600 mm/s
  never reach the 81 000 bound.
- **The dominant wall artifact is not ringing.** It is under-extrusion at
  every wall's ends: the edge sits up to 60 µm inside over 5–8 mm, and
  line widths reach p1 0.34 mm against 0.45 mm. The extruder's motion is
  smoothed over `pressure_advance_smooth_time` (0.04 s, 12–24 mm at
  these speeds) while XY follows the 5 ms shaper, so flow lags the
  nozzle's acceleration and deceleration. It is the same under every
  corner setting.
- **Lowering that smoothing to 0.005 s is worse in this model.** The
  pressure-advance spikes then exceed the pancake motor: 1838 extruder
  pole slips on the tower, and 30% of the volume missing. So the
  prediction to check on the printer is: wall-end under-extrusion at
  0.04 s, and extruder skipping well below it.

