# D. Klipper / Kalico Ecosystem: Prior Art on SCV and Junction Deviation

Research date: 2026-04-16. Scope: Klipper, Kalico, DangerKlipper, notable forks,
Discourse, and Shake&Tune — community history on the junction-deviation /
square-corner-velocity problem and any prior attempt at real corner blending.

## 1. Executive Summary

- **No fork has implemented real arc blending.** Every attempt so far stays inside
  the Grbl-derived "virtual radius" junction-deviation model (SCV reparameterization,
  smooth-cutoff heuristics, per-axis limits). The closest thing to blending in the
  record is a 2019 aside from Butyugin — "I also consider experimenting with
  explicit cornering by approximating corners with arcs ... and/or bezier curves"
  ([#2030](https://github.com/Klipper3d/klipper/issues/2030#issuecomment-540331197)) —
  which was never pursued.
- **O'Connor's bar for replacing JD is explicit and high**: "A change from
  junction_deviation to a new algorithm would require widespread test results
  showing noticeable improvement. The JD algo is a heuristic and I don't doubt it
  can be improved, but for all its faults, it is a widely used and 'battle tested'
  algorithm." ([#4228](https://github.com/Klipper3d/klipper/issues/4228#issuecomment-830892480))
  He has also stated he personally will not work on it.
- **Butyugin has confirmed SCV is non-physical.** "square_corner_velocity model
  is not really particularly physical (it calculates and uses the cornering radius
  that's just a model and does not exist in practice)"
  ([Discourse 7298](https://klipper.discourse.group/t/square-corner-velocity-what-is-the-reasonable-range-of-values/7298)) —
  but also explicitly rejects patching the formula because "in that case, it is
  no longer based on any physical model or assumptions"
  ([#4228](https://github.com/Klipper3d/klipper/issues/4228)).
- **Richfelker (#4228) and Piezoid (Discourse 3970) both hit a wall** not because
  their math was wrong but because of stated maintainer priorities: minimize
  user-facing knobs, avoid breaking backward compatibility, do not trust
  improvements that aren't proven by ~10 independent users on common hardware.
- **Kalico already carries Piezoid's `limited_corexy` / `limited_cartesian`**
  (PR #4, Jan 2024), which does exactly what upstream refused:  runtime per-axis
  accel limits and cornering that recomputes JD when accel changes. This is the
  only shipped Klipper-family code that bends the SCV model at all. It does
  **not** do arc blending — it still calls the same `calc_junction()`.

## 2. Known Issues Deep-Dive

### 2.1 Klipper Issue #468 — JD replaced with SCV (July 2018)

<https://github.com/Klipper3d/klipper/issues/468>

**O'Connor's original rationale** (verbatim, from the issue body):

> 1. It is effectively a "magic number" as there is little intuition that users
>    can apply to pick a good value.
> 2. The junction_deviation is combined with max_accel to determine actual
>    cornering speeds, and this is not obvious.
> 3. The junction_deviation parameter has a non-linear impact to cornering speed
>    and that is not obvious.
> 4. I fear the default junction_deviation is too high to be a good default.

When a user (hg42) argued JD was more intuitive because it has a physical distance
interpretation, O'Connor corrected him:

> The junction deviation algorithm does not set a distance that the tool path may
> deviate. Klipper follows the g-code movement commands it receives and commands
> the toolhead to follow the exact tool path requested.
>
> The junction deviation algorithm is used to determine a cornering velocity. A
> velocity that the toolhead needs to slow down to (instead of zero) during the
> junction between two moves. **It results in an instantaneous velocity change of
> the toolhead during that junction.**

This is load-bearing: **Klipper accepts instantaneous velocity discontinuities
at junctions as a first-class part of the design** — not a bug to be fixed by
blending. SCV was framed as a way to *choose* how big that discontinuity may
be, not eliminate it. Any design that eliminates the discontinuity needs to
explicitly address this point.

### 2.2 Klipper Issue #4228 — "Sharp corners and smooth circles are mutually exclusive"

<https://github.com/Klipper3d/klipper/issues/4228>

Richfelker (April 2021) made the exact observation that motivates us: at scv=5 the
36-gon approximation of a circle is capped to ~50 mm/s, producing visible
stutter from accel/decel oscillations and pressure-advance swings. He proposed
a smooth phase-out of the JD limit at small turn angles.

**Butyugin's rejection of formula-patching** (the one we must address):

> I don't think that 'patching' an existing formula is the right way to go - in
> that case, it is no longer based on any physical model or assumptions.

He then offered an alternative physical model (toolhead momentum):

> one may want to fix how much momentum a printer can instantaneously take from
> a toolhead to change its direction. For a turn theta ... the momentum change
> is 2*m*v*cos(theta/2). If we set scv == 5mm/s, we could use that formula to
> determine an appropriate speed for other angles.

and showed the momentum and JD curves are nearly identical — concluding the
current behavior is essentially correct and that the "real problem" is
elsewhere:

> contrary to your desire, it is indeed more "difficult" for the printer to
> turn 10 degrees (170 degrees angle between path segments) at 100 mm/sec than
> to turn 90 degrees at 5 mm/sec, and the junction deviation model does not
> permit that.

**O'Connor's features-of-JD summary** (the "why we keep JD" list):

> 1. It's output doesn't depend on the low-level kinematics. This was important
>    to me, as in my experience, the main limiting factor to quality prints is
>    the interface between nozzle and print. In general, the steppers have oodles
>    of torque. Getting plastic in the rice place and getting it to adhere is the
>    challenge. So, I felt the "jerk" model was not a good choice, and the JD
>    algorithm is more appropriate.
> 2. It is a widely deployed and battle tested algorithm.
> 3. It results in slower speeds with acute angles, much higher speeds when
>    going in nearly the same direction, and the change of cornering velocity
>    limit with respect to angle is smooth.
> 4. It is efficient to calculate.

**O'Connor's reframing of SCV** (the point our design must co-opt, not fight):

> FWIW, I think of square_corner_velocity as a mechanism for managing extruder
> flow rate. If the printer decelerates to zero on each corner then we get
> terrible results because the extruder can't completely stop the flow of
> plastic. The pressure_advance system can help with managing flow, but PA works
> best when there is still some flow ...
>
> So, I know some people try to tune square_corner_velocity for print times, but
> in my experience that doesn't really do much. If needed, I'd look to tune
> square_corner_velocity based on extruder performance.

**The closing verdict** (what any successor must clear):

> A change from junction_deviation to a new algorithm would require widespread
> test results showing noticeable improvement. The JD algo is a heuristic and
> I don't doubt it can be improved, but for all its faults, it is a widely used
> and "battle tested" algorithm.

And on individual engagement:

> FWIW, this isn't something I, personally, plan to work on.

The issue was closed without any code change.

### 2.3 Klipper Issue #5227 — SCV's hidden coupling to max_accel (Feb 2022)

<https://github.com/Klipper3d/klipper/issues/5227>

hoffbaked filed this, correctly identifying that because SCV is converted to a
`junction_deviation` using the config's `max_accel` once at startup, `SET_VELOCITY_LIMIT
ACCEL=...` (common in resonance testing) does **not** scale the cornering speed,
producing surprising asymmetric results. It was **auto-closed by Sineos / the
GitIssueBot** with no technical response:

> Discussions or questions about the code (if you intend to develop or improve
> Klipper) are best placed on Discord https://klipper.discourse.group as the Devs
> are reading there too. GH is only used to share development results...

This is actually the bug that Butyugin later fixed in PR #5821 (Oct 2022,
"toolhead: Capture current junction_deviation in a Move class"). From #5821:

> @dmbutyugin: in this case, the mainline Klipper computes the junction velocity
> approx equal to 1.58 mm/sec (assuming scv = 5 mm/sec). However, if there was no
> second `SET_VELOCITY_LIMIT` command, it would be 5 mm/sec. With this fix, the
> cornering velocity will be computed as 5 mm/sec in this case. I think this is
> a more appropriate behavior in this case.
>
> @KevinOConnor: I agree there is a "corner case" here. It's not clear to me that
> we need to "fix" this, but I also don't see any issues with your proposed code.

So the coupling bug was eventually fixed in mainline; the hidden coupling
*concept* (SCV only makes sense given a particular max_accel) remains.

### 2.4 Discourse #3970 — Piezoid's Proportional Acceleration Control (Aug 2022)

<https://klipper.discourse.group/t/proportional-acceleration-control/3970>

Maël Kerbiriou ("Piezoid") proposed (a) runtime `accel` override independent of
config `max_accel`, (b) renaming `max_accel_to_decel` to a cruise-ratio
parameter, (c) making SCV scale with sqrt(accel) when accel changes at runtime.
Quoting his opener:

> I make this topic for judging interest and to discuss the naming, compatibility
> and user experience issues. ... This is my last attempt, and if nothing comes
> out of it, I'll shut up forever on these issues.

Butyugin's first-line response: use macros.

> First, I understand that you want to have some relative control of different
> acceleration and cornering parameters, right? If that is the case, I think you
> can achieve the desired effect perfectly with macros and macro variables

On the wider idea Butyugin raised the oft-cited "it's a knob you can't turn"
concern:

> My biggest concern is not that 'it's just another knob', but that it's a knob
> that's unsafe to turn.

O'Connor's response (verbatim, on renaming and on knobs):

> Technically, max_accel is an upper bound. Although most moves will use
> max_accel as the acceleration, the kinematics and extruder can (and do) limit
> the acceleration of some moves. If this setting is causing confusion to users,
> then I'd recommend trying to resolve with documentation updates. (Actually
> renaming the option would break many working setups.)
>
> max_accel_to_decel is a "goofy" system. I'd happily replace it with an improved
> system. ... It's not clear to me that max_accel_to_decel should scale with
> max_accel.

On another Piezoid suggestion (a config-accel vs runtime-accel split):

> FWIW, I fear this would be "adding a knob" - "config accel knob" vs "runtime
> accel knob".

Outcome: partial win. O'Connor eventually accepted the rename idea and landed
PR #6418 (Dec 2023) replacing `max_accel_to_decel` with `minimum_cruise_ratio`.
The proportional-accel part and sqrt(accel)-scaled SCV were dropped. Piezoid
kept them in his personal fork (see §4).

### 2.5 Discourse #7298 — SCV reasonable range (Mar 2023 + Aug 2024 Butyugin reply)

<https://klipper.discourse.group/t/square-corner-velocity-what-is-the-reasonable-range-of-values/7298>

The full Butyugin quote on the non-physicality of SCV (Aug 2024):

> square_corner_velocity model is not really particularly physical (it
> calculates and uses the cornering radius that's just a model and does not
> exist in practice). Two reasons it is used in Klipper is because we (a) need
> a model to calculate motion for segmented smooth surfaces, where toolhead
> changes the direction at the corner only very slightly (though Klipper also
> calculates centripetal acceleration for such cases), and (b) because linear
> pressure advance model does not work very well for low velocities, so
> decelerating too much at sharper corners will lead to poor quality of such
> corners because oftentimes linear pressure advance cannot stop oozing from
> the nozzle at low speeds.
>
> So, in short, if you reduce square_corner_velocity, you get more physically
> sound kinematics calculations at sharp corners, but you'll get worse results
> on corners due to pressure advance. If you increase square_corner_velocity,
> pressure advance works better at sharp corners, but the kinematics of the
> toolhead at corners becomes "borked" ... Also if you use input shaping,
> increasing square_corner_velocity rapidly increases smoothing from input
> shaping.
>
> Therefore, there is not 'one size fits all' answer. Klipper uses an scv==5 as
> a good enough middle ground.

Key admission: **SCV is the knob where three unrelated concerns collide**
(kinematic sanity, linear-PA minimum-flow floor, input-shaper smoothing).
Our design must replace all three concerns, not just one.

### 2.6 Shake&Tune Issue #10 — Calibrator mismatch

<https://github.com/Frix-x/klippain-shaketune/issues/10>

This one turned out **not** to be about the SCV model at all. User celtare21
reported AXES_SHAPER_CALIBRATION recommending shaper 'ei' @ 49.6 Hz while
Klipper's built-in SHAPER_CALIBRATE recommended 'mzv' @ 77.6 Hz on the same
axis. Root cause was a file-system race: Shake&Tune used Python `shutil.move`
instead of the OS `mv`, with different sync semantics, leading to incomplete
CSV files being read by the analyzer.

> @Frix-x: it's probably due to corrupted files or incomplete writes, because
> sometimes the OS didn't have time to write the whole CSV file on slow file
> systems.

Closed as fixed. Not relevant to our motion-planner work. The *real* S&T
architectural question is in §5.

## 3. Additional Issues and Threads Found

- **Klipper #255** (bmc0, Mar 2018) — "Potential oversight in calc_junction()".
  First pointed out that `R` (virtual radius) is unbounded as the number of
  segments approximating a circle increases. His proposed fix was committed as
  a4439b93 ("toolhead: Limit junction speed of short moves"). This is the
  pre-history of the #4228 oscillation problem.
- **Klipper #1997** (O'Connor, Sep 2019) — "Smoothed pressure advance and
  extruder cornering support". Merged. Introduces the tight coupling between
  SCV and PA's smooth_time that Butyugin cites in Discourse 7298.
- **Klipper #2030 / #57 / #1776** — S-curve acceleration experiments.
  Abandoned ("more people have reported a degradation in print quality or no
  change than people that have reported an improvement"). Directly relevant
  quote from Butyugin in this thread, Oct 2019:
  > FWIW, I also consider experimenting with explicit cornering by approximating
  > corners with arcs (for AO=2) and/or bezier curves (for AO=2,4,6) with fixed
  > precision. As a positive, it should work well with constant acceleration
  > mode too.

  Never implemented. Our design should cite this as precedent: **the project's
  own kinematics expert identified arc/Bezier corner blending as a plausible
  direction but never had the reviewer bandwidth to pursue it.**
- **Klipper #5349** (viesturz, Mar 2022) — "Jerkiness on smooth curves". User
  reports exactly the #4228 symptom on a remote-direct extruder. Auto-closed
  by GitIssueBot. The maintainers' bot makes unsolicited motion-planner reports
  on GitHub effectively un-actionable — they have to go through Discourse.
- **Klipper #6747** (Butyugin, Nov 2024) — "Fixed junction deviation calculation
  for straight segments". Long, unusually revealing exchange. Butyugin's
  proposal computes radius-of-curvature directly rather than limiting
  `junction_cos_theta` to `-0.999999`. O'Connor objects:
  > I'm concerned in may trade one unusual "corner case" for another one.
  > Specifically, it'll close a loop-hole where we may pessimize speeds if
  > acceleration changes midway through a straight line movement, but I fear
  > it opens a loop-hole where a slicer could emit an arc with a series of
  > infinitesimal moves such that each of those moves appears to be "close to
  > straight" which then results in the arc proceeding with no slow-down at
  > all. ... It seems this code would basically say "if it seems the junction
  > is mostly straight then assume it is straight and don't limit speed at all".
  > But that's seems backwards to me - if we're unsure about the math we should
  > round down the junction speeds, not round them up.

  Maxim (MRX8024) weighed in with arc-parser data:
  > I am also stupefied by the current way of determining the speed on circles.
  > The worse your gcode file is, the larger length of polygons it have, the
  > slower the circles/turns that are so fond of adding to all 3d models now will
  > be printed. But I do not know the right alternatives, unfortunately.

  Butyugin's counter that nails the philosophical split with O'Connor:
  > My approach is based on the following observation: for smooth curves, as
  > the segmentation becomes finer, both move_d and cos(theta/2) go to 0, but
  > their ratio does not ... Thus, by checking the ratio move_d / cos(theta/2),
  > we can tell apart the cases "a curve with finite curvature, just very fine"
  > and "(sufficiently) straight segments" regardless of how fine the
  > segmentation is, and without having to play with limits too much.

  Was eventually merged (Kalico carries it as b0e18b37).
- **Discourse #4334** (csJosh, Oct 2022) — Same bug as #5227/#5821, observed
  independently via `SET_VELOCITY_LIMIT` experiments.
- **Discourse #5820** (lukes, Jan 2023) — "Simple path planner". An external
  user writing their own planner hitting the same "huge jumps in axis velocity
  at direction changes" that JD allows. Documents that the behavior is
  surprising even to people implementing planners from scratch.
- **Discourse #16865, #24301** — Recurring user complaints through 2024-2025
  about curves being slow or harsh. Consistently answered with "update to
  pickup PR #6747" and "SCV is not a print-time tuning knob".
- **Discourse #24335** (Sineos, Jul 2025) — "The Myth of G2/G3 Arc Commands".
  The maintainers' current party line: arcs don't help because everything gets
  segmented anyway. Worth pre-empting: our design must be clear that it does
  **not** require G2/G3 and works on G1-segmented arcs.

## 4. Existing Forks / Branches Touching This Area

**This is the most important question, per the brief.**

| Fork | Branch | What it does | Relevance |
| --- | --- | --- | --- |
| `Piezoid/klipper` | `work-peraxis_pr` / `work-peraxis-scv` | Per-axis accel/vel limits; recomputes JD when accel changes mid-path; makes SCV scale independently of max_accel. | **Merged into Kalico as `limited_corexy`/`limited_cartesian` (commit 1d8ae0b4, "Independent X & Y Accelerations").** Still uses JD; not arc blending. |
| `dmbutyugin/klipper` | `scurve-pa`, `scurve-shaping`, `scurve-smoothing`, `work-scurve-20180620` | Several s-curve experiments; none replace JD/SCV. | Stale. Kevin's feedback in #2030/#1776 killed momentum. |
| `richfelker/klipper` | only `master`, unmaintained since April 2021 | The #4228 filer. **No fork branch with his proposed smooth SCV-phase-out.** Piezoid ported his idea as commit `76ba4bee "toolhead: cornering without limited acceleration — Credits to @richfelker"` on `work-peraxis`. | Piezoid's 5-line patch is the only shipped code for richfelker's idea. |
| `KalicoCrew/kalico` | `main` | Carries PR #4 (limited_* kinematics) + #6747 + `RESET_VELOCITY_LIMIT` + Piezoid's `minimum_cruise_ratio`. The `rackrobo_dev` branch has a `skip junction` commit (`0d78d1a0`), but it is for the Powercore wire-EDM kinematic — not motion planning. | This is us. **No arc-blending work in-flight.** |
| `DangerKlippers/danger-klipper` | `master` | Unmaintained since 2024-12-11. Essentially the predecessor that Kalico absorbed. Also no arc blending. | Dead branch. |

**Fork-network sweep:** highest-star Klipper forks
(bigtreetech, naikymen/klipper-for-cnc, Desuuuu, dmbutyugin, dockterj, Arksine,
garethky, Piezoid, alchemyEngine, mental405) — none touch `calc_junction()` beyond
what's already covered here. The Creality K1 fork (`K1-Klipper`) is a vendor
stepper-timing fork. `naikymen/klipper-for-cnc` is focused on CNC G-code
support (G2/G3, cutter comp), not on rewriting cornering. No other fork in the
top 40 by stars has a WIP branch name containing `arc`, `blend`, `bezier`, or
`corner`.

**Conclusion: there is no prior art for real arc blending in the Klipper
ecosystem.** The one mention of it (Butyugin, 2019) never produced code. This
is a genuinely unexplored avenue — which is both an opportunity and a warning.

### 4.1 Piezoid's cornering-without-acceleration patch (5 lines)

Piezoid's `76ba4bee` (June 2023) implements richfelker's "don't accelerate between
curve segments" idea in five lines. It drops the centripetal-acceleration term
from the `min(...)` inside `calc_junction()` so curve segments take a cap on
`max_cruise_v2` rather than the usual sqrt(accel·d) ramp-in. This keeps toolhead
speed *constant* across a curve instead of oscillating. It's the smallest
existing patch that addresses the #4228 symptom, and it is **already within
Kalico's orbit** (Piezoid = author of `limited_*` kinematics in Kalico). Worth
benchmarking before we commit to a rewrite — if 5 lines fix 80% of the user pain,
our rewrite has a very high bar.

## 5. Shake&Tune Status

Shake&Tune computes `max_accel` recommendations via Klipper's own
`calibrate_shaper.ShaperCalibrate` — specifically by solving, for a chosen
shaper, the max acceleration at which the commanded-vs-smoothed position
error stays below a threshold. The **user supplies `SCV` as an input to the
script** (`--scv=...`, default pulled from `[printer]` config). Raising SCV
rapidly raises the error budget the shaper has to absorb, so the fitted
`max_accel` collapses — in extreme cases to zero, as #222 demonstrates:

> @Frix-x (#222, Aug 2025): Max_accel is computed based on all the machine's
> parameters and tries to find a max_accel that still allows for acceptable
> smoothing values. However, in your case, you have such a high SCV that it's
> impossible to achieve low smoothing, even at a very low acceleration.
> Therefore, the highest acceleration that can be achieved while maintaining
> acceptable smoothing is zero.
>
> You shouldn't use an SCV that high, as it serves no purpose and will just
> make your prints brittle with defects in the corners and infill anchors. It's
> a bit like having too high a pressure advance. A good rule of thumb is to
> usually use your maximum print acceleration and divide it by 1000, with a
> maximum clipped value of around 12-15.

**Architectural implication for us:** Shake&Tune's entire max_accel
recommendation model is *downstream of SCV*. If we remove SCV as a
user-facing concept, the existing calibration ecosystem (Shake&Tune, Klippain,
every "Ellis tuning guide" on the internet) **recommends max_accel assuming a
specific SCV setting we no longer have**. We must provide a clear migration
target: either (a) expose a compatibility "effective SCV" derived from the
new corner-blending parameters, or (b) ship an updated calibrator that works
against the new model. Option (b) is technically cleaner but requires
coordination with Frix-x.

Frix-x has shown no sign of working on a new model; there is no Shake&Tune
issue or discussion about it. He relies entirely on upstream Klipper's
calibration machinery. Breaking that machinery *without a replacement* will
block adoption.

## 6. Arguments Our Design Must Pre-empt

Synthesized from the above. Each is an O'Connor- or Butyugin-flavored
objection we will receive verbatim.

1. **"Don't add knobs."** (O'Connor, Discourse 3970). Any new parameter must
   (a) replace at least one existing parameter, and (b) have an obvious
   physical meaning. Plan: eliminate `square_corner_velocity` and introduce a
   **single** blend radius or max-centripetal-accel limit — measurable in mm
   or mm/s², not heuristic.
2. **"The JD algorithm is battle-tested; a replacement needs ~10 users with
   measurable improvement before it lands."** (O'Connor, #4228). Plan: ship
   behind a config flag from day one; collect before/after input-shaper
   smoothing plots, extrusion-uniformity photos on curves, and total-print-time
   deltas from multiple hardware classes; invite explicit testing on Discourse
   with a tracking thread.
3. **"SCV already manages extruder flow rate; don't break that."** (O'Connor,
   #4228; Butyugin, Discourse 7298). Plan: the new model must produce a
   non-zero minimum speed at sharp corners (equivalent of today's SCV floor)
   so linear pressure advance still works. Arc blending naturally does this
   for moderate corners; for very sharp corners we retain a configurable
   minimum-velocity floor.
4. **"Don't patch the formula — that loses the physical basis."** (Butyugin,
   #4228). Plan: frame our derivation as *replacing* the centripetal-acceleration
   model with an *explicit blend-arc tangential/centripetal-acceleration*
   model (same physics, honest geometry). Be able to say: our limit is
   `v² <= a_c · R_blend` where `R_blend` is a real geometric radius of a real
   blend arc, not a virtual one. This preempts the "it's no longer based on any
   physical model" critique that killed richfelker's smooth-cutoff idea.
5. **"Slicers emit pathological sequences of infinitesimal moves; a permissive
   model will be exploited."** (O'Connor, #6747). Plan: the blend-geometry
   approach sidesteps this: if the merged blend arc cannot fit within the
   shorter of two adjacent move lengths, we degrade gracefully to the current
   JD-style point junction. Document this fallback up front.
6. **"Axes are independent; small turns still mean big perpendicular axis
   velocity changes."** (O'Connor, #4228, citing a 10° turn at 200 mm/s → 35
   mm/s perpendicular start). Plan: the blend-arc replaces a point with a
   finite-time centripetal segment, which *is* the physical answer to this
   objection — each axis's acceleration is bounded because the centripetal
   accel during the arc is bounded. Make this an explicit pre-emption, not an
   aside.
7. **"It shouldn't depend on low-level kinematics."** (O'Connor, #4228 #1).
   Plan: the blend radius is computed in toolhead Cartesian space; per-axis
   accel limits still compose downstream exactly as today (and as Kalico's
   `limited_*` modules already do).
8. **"Don't break Shake&Tune / existing calibration guides."** (implicit, §5).
   Plan: ship a compatibility helper that reports an "effective SCV" for a
   given blend config so existing Shake&Tune runs still produce sane
   max_accel. Coordinate with Frix-x *before* deprecating SCV.
9. **"Kevin will not work on it personally."** (O'Connor, #4228). This is a
   Kalico-fork reality, not a Klipper-mainline constraint. Our upstream goal
   should be a clean, tested, maintainer-adoptable change; our short-term
   goal is to ship in Kalico and let field data make the case.

## 7. References

- Klipper #468 – <https://github.com/Klipper3d/klipper/issues/468>
- Klipper #255 – <https://github.com/Klipper3d/klipper/issues/255>
- Klipper #1997 – <https://github.com/Klipper3d/klipper/issues/1997>
- Klipper #2030 – <https://github.com/Klipper3d/klipper/issues/2030>
- Klipper #4228 – <https://github.com/Klipper3d/klipper/issues/4228>
- Klipper #5227 – <https://github.com/Klipper3d/klipper/issues/5227>
- Klipper #5349 – <https://github.com/Klipper3d/klipper/issues/5349>
- Klipper #5821 – <https://github.com/Klipper3d/klipper/pull/5821>
- Klipper #6418 – <https://github.com/Klipper3d/klipper/pull/6418>
- Klipper #6747 – <https://github.com/Klipper3d/klipper/pull/6747>
- Discourse 3970 – <https://klipper.discourse.group/t/proportional-acceleration-control/3970>
- Discourse 4334 – <https://klipper.discourse.group/t/calc-junction-numerical-issue-stright-line-move/4334>
- Discourse 5820 – <https://klipper.discourse.group/t/simple-path-planner/5820>
- Discourse 7298 – <https://klipper.discourse.group/t/square-corner-velocity-what-is-the-reasonable-range-of-values/7298>
- Discourse 16865 – <https://klipper.discourse.group/t/square-corner-velocity-at-high-speeds/16865>
- Discourse 24301 – <https://klipper.discourse.group/t/square-corner-velocity-and-smoothness-of-movement/24301>
- Discourse 24335 – <https://klipper.discourse.group/t/the-myth-of-g2-g3-arc-commands/24335>
- Shake&Tune #10 – <https://github.com/Frix-x/klippain-shaketune/issues/10>
- Shake&Tune #222 – <https://github.com/Frix-x/klippain-shaketune/issues/222>
- Piezoid fork branches – <https://github.com/Piezoid/klipper/branches>
- Piezoid per-axis SCV branch – <https://github.com/Piezoid/klipper/tree/work-peraxis-scv>
- Piezoid "cornering without limited acceleration" patch (credits richfelker) – <https://github.com/Piezoid/klipper/commit/76ba4bee>
- Kalico "Independent X & Y Accelerations" (merges Piezoid's limited_*) – commit 1d8ae0b4 in KalicoCrew/kalico
- Kalico carries #6747 – commit b0e18b37 in KalicoCrew/kalico
- DangerKlippers/danger-klipper (archived-in-effect) – <https://github.com/DangerKlippers/danger-klipper>
