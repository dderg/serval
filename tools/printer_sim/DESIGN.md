# printer_sim: a virtual printer behind motion_eval

motion_eval stops at the executed carriage motion: the planned and shaped
trajectory, plus a single-mode oscillator as a ringing proxy. printer_sim
adds the physics after that point, so a question like "does this planner
change ring, bulge or skip" can be answered without the owner's printer.
The chain is:

```
gcode -> klippy (motion_eval capture) -> step stream per motor
      -> stepper + driver (2) -> rotor -> belts, carriage, gantry, toolhead (1)
      -> nozzle path -> [extruder + melt (3)] -> bead deposition, height field (4)
```

Phase 1 (this commit) is modules 1 and 2, with validation tests and an
end-to-end table. Modules 3 and 4 are specified here and not built yet.
Module 5, validation, is a rule rather than a module: no number below is
claimed unless an analytic case behind it passes.

Code: `parts.py` (parts list), `mechanics.py` (module 1), `motor.py`
(module 2, analytic side), `kernel.c` + `native.py` + `sim.py`
(integration), `drive.py` (gcode to commands), `report.py` (end-to-end).
Tests are in `tests/`. Run them with
`PYTHONPATH=. uv run pytest tools/printer_sim/tests` and the table with
`PYTHONPATH=. uv run python -m tools.printer_sim.report`.

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

## Module 3 (planned): extrusion

The extruder stepper is module 2 again, through the gear ratio
(rotation_distance 22.68, 50:10). It drives filament length e_m.

- Solid filament between the gear and the melt is a spring
  k_f = E·A/L_f (PLA 8.66 kN·L_f⁻¹). It is far stiffer than the melt, per
  printer-params §5.
- The melt chamber stores compliance C, and the nozzle passes
  Q = p / R_h. Together they give the first-order model Klipper's
  pressure advance assumes:

$$e_{cmd} = e_{out} + K_{PA}\,\dot e_{out},\qquad K_{PA} = R_h C$$

  This is the same discretization as serval's `flow_metric.melted`
  (re-implemented, not imported). The volumetric flow out is
  Q = A_f ė_out.
- The melt limit is the hotend's maximum flow (Revo 13–32 mm³/s at
  0.4 mm). It caps Q, and the excess raises pressure, which raises the
  extruder load torque F·r_gear and can slip the extruder in module 2.
- Validation: steady flow equals the commanded flow; the step response
  time constant equals K_PA; the volume integral is conserved.

## Module 4 (planned): deposition

- **Bead.** Along the simulated nozzle path with speed v, the bead
  cross-section is A = Q / v. Width follows the stadium model
  A = (w − h)h + πh²/4 at layer height h.
- **Height field.** Accumulated per layer on a 10–20 µm grid. Each
  sample's volume Q·dt is spread over its stadium footprint.
- **Metrics**, computed against the slicer's intended outline (the
  commanded path offset by half the nominal width):
  - wall waviness: amplitude and spatial frequency along the arc length;
  - corner bulge and under-fill: edge excursion near vertices;
  - line-width variation;
  - layer shift: edge offset between layers;
  - dimensional error.
- **Renders.** The outer-wall offset unrolled over (s, z), which reads
  like a ringing-tower photo, and a shaded top surface, both as PNG.
- **Validation:** deposited volume equals extruded volume to float
  precision; a straight line at constant flow gives constant width; a
  synthetic sinusoidal path offset of known amplitude and wavelength is
  recovered by the waviness metric.

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
| bead volume conserved | module 4 | planned |

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
- **Z, the bed, the extruder, melt and deposition.** Modules 3 and 4 are
  planned.

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
5. **Prints, for modules 3 and 4.** A ringing tower and a corner test.
   Compare measured wave amplitude, wavelength and corner bulge with the
   deposition metrics.

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
