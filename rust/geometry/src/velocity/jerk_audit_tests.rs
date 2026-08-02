use super::curved::{self, top_speed_ceiling};
use super::disk::Kinematics;
use super::profile::StraightPhase;
use super::*;
use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{Move, MoveContext, VelocityLimits, line_move};
use crate::path::CurvatureProfile;

const EXTRUDER_AXIS: usize = 3;
const FLOW_RATIO: f64 = 0.05;
const STRAIGHT_KAPPA_EPS: f64 = 1e-12;
const EXACT_JERK_REL_TOL: f64 = 1e-9;
const VECTOR_JERK_REL_TOL: f64 = 1e-2;

#[derive(Clone, Copy)]
struct Machine {
    feed: f64,
    accel: f64,
    corner_accel: f64,
    deviation: f64,
    jerk: f64,
}

impl Machine {
    fn user() -> Self {
        Self {
            feed: 300.0,
            accel: 60_000.0,
            corner_accel: f64::INFINITY,
            deviation: 0.05,
            jerk: 1.5e8,
        }
    }

    fn accel(self, accel: f64) -> Self {
        Self { accel, ..self }
    }

    fn corner_accel(self, corner_accel: f64) -> Self {
        Self {
            corner_accel,
            ..self
        }
    }

    fn jerk(self, jerk: f64) -> Self {
        Self { jerk, ..self }
    }

    fn ctx(&self, line_no: u32) -> MoveContext {
        MoveContext {
            extruder_axis: EXTRUDER_AXIS,
            feedrate_mm_s: self.feed,
            limits: VelocityLimits::try_new(self.feed, self.accel, self.deviation, self.jerk)
                .unwrap()
                .with_corner_accel(self.corner_accel),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        }
    }

    fn polyline(&self, pts: &[[f64; 3]]) -> Vec<Move> {
        pts.windows(2)
            .enumerate()
            .map(|(i, p)| {
                let len = crate::vec3::dist(p[0], p[1]);
                line_move(p[0], p[1], FLOW_RATIO * len, self.ctx(i as u32 + 1)).unwrap()
            })
            .collect()
    }
}

fn square_perimeter(m: Machine, side: f64) -> Vec<Move> {
    m.polyline(&[
        [0.0, 0.0, 0.0],
        [side, 0.0, 0.0],
        [side, side, 0.0],
        [0.0, side, 0.0],
        [0.0, 0.0, 0.0],
    ])
}

fn wedge(m: Machine, leg: f64, theta_deg: f64) -> Vec<Move> {
    let theta = theta_deg.to_radians();
    let (sin, cos) = libm::sincos(std::f64::consts::PI - theta);
    m.polyline(&[
        [-leg, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [leg * cos, leg * sin, 0.0],
    ])
}

fn faceted_arc(m: Machine, radius: f64, facets: usize, sweep_deg: f64) -> Vec<Move> {
    let sweep = sweep_deg.to_radians();
    let mut pts = vec![[-20.0, -radius, 0.0]];
    for i in 0..=facets {
        let a = -std::f64::consts::FRAC_PI_2 + sweep * (i as f64) / (facets as f64);
        let (sin, cos) = libm::sincos(a);
        pts.push([radius * cos, radius * sin, 0.0]);
    }
    let last = *pts.last().unwrap();
    pts.push([last[0] + 20.0, last[1], 0.0]);
    m.polyline(&pts)
}

fn zigzag(m: Machine, teeth: usize, dx: f64, dy: f64) -> Vec<Move> {
    let mut pts = vec![[0.0, 0.0, 0.0]];
    for i in 0..teeth {
        let x = dx * (i as f64 + 1.0);
        let y = if i % 2 == 0 { dy } else { 0.0 };
        pts.push([x, y, 0.0]);
    }
    m.polyline(&pts)
}

fn plan_for(moves: &[Move]) -> (FitOutcome, VelocityProfile) {
    let fitted = fit_corners(moves, CornerFitConfig::default()).expect("fit");
    let profile = plan_velocity_warm_start(
        &fitted,
        1e-7,
        f64::INFINITY,
        f64::INFINITY,
        BoundaryState::REST,
    )
    .expect("plan");
    (fitted, profile)
}

fn kappa_abs_at(seg: &Segment, s: f64) -> f64 {
    seg.kappa(s).abs()
}

fn dkappa_abs_ds_at(seg: &Segment, s: f64) -> f64 {
    let kappa = seg.kappa(s);
    let sigma = seg.dkappa_ds(s);
    if kappa > 0.0 {
        sigma
    } else if kappa < 0.0 {
        -sigma
    } else {
        sigma.abs()
    }
}

fn is_straight(seg: &Segment) -> bool {
    let (_, kappa_peak) = seg.kappa_peak();
    kappa_peak <= STRAIGHT_KAPPA_EPS && seg.dkappa_ds(0.0).abs() <= STRAIGHT_KAPPA_EPS
}

fn normal_jerk(kappa_abs: f64, dkappa_abs_ds: f64, v: f64, a_t: f64) -> f64 {
    dkappa_abs_ds * v * v * v + 3.0 * kappa_abs * v * a_t
}

fn tangential_jerk(kappa_abs: f64, v: f64, a_t_dot: f64) -> f64 {
    a_t_dot - kappa_abs * kappa_abs * v * v * v
}

#[derive(Clone, Copy)]
struct Worst {
    line_no: u32,
    s: f64,
    v: f64,
    a_t: f64,
    kappa: f64,
    sigma: f64,
    ratio: f64,
}

impl Worst {
    fn keep(slot: &mut Option<Worst>, candidate: Worst) {
        if slot.is_none_or(|w| candidate.ratio > w.ratio) {
            *slot = Some(candidate);
        }
    }

    fn describe(slot: Option<Worst>) -> String {
        match slot {
            None => "none".to_string(),
            Some(w) => format!(
                "{:.4}x at line {} s={:.6} v={:.3} a={:.1} kappa={:.5} sigma={:.4}",
                w.ratio, w.line_no, w.s, w.v, w.a_t, w.kappa, w.sigma
            ),
        }
    }
}

struct Audit {
    jerk: f64,
    normal: Option<Worst>,
    straight: Option<Worst>,
    vector: Option<Worst>,
    time_s: f64,
    curved_samples: usize,
    straight_phases: usize,
}

impl Audit {
    fn summary(&self) -> String {
        format!(
            "j_max={:.4e} time={:.6}s curved_samples={} straight_phases={}\n  normal   {}\n  straight {}\n  vector   {}",
            self.jerk,
            self.time_s,
            self.curved_samples,
            self.straight_phases,
            Worst::describe(self.normal),
            Worst::describe(self.straight),
            Worst::describe(self.vector),
        )
    }

    fn ratio(slot: Option<Worst>) -> f64 {
        slot.map_or(0.0, |w| w.ratio)
    }
}

fn audit(fitted: &FitOutcome, profile: &VelocityProfile) -> Audit {
    let mut out = Audit {
        jerk: 0.0,
        normal: None,
        straight: None,
        vector: None,
        time_s: profile.report.traversal_time_s,
        curved_samples: 0,
        straight_phases: 0,
    };
    for (fit, vel) in fitted.moves.iter().zip(&profile.moves) {
        let Some(seg) = fit.segment.spatial.as_ref() else {
            continue;
        };
        let j_max = vel.jerk;
        out.jerk = j_max;
        let line_no = vel.source.start_line;
        if is_straight(seg) {
            out.straight_phases += vel.phases.len();
            for p in &vel.phases {
                Worst::keep(
                    &mut out.straight,
                    Worst {
                        line_no,
                        s: p.s0,
                        v: p.v0,
                        a_t: p.a0,
                        kappa: 0.0,
                        sigma: 0.0,
                        ratio: p.j.abs() / j_max,
                    },
                );
            }
            continue;
        }
        out.curved_samples += vel.samples.len();
        for w in &vel.samples {
            let kappa = kappa_abs_at(seg, w.s);
            let sigma = dkappa_abs_ds_at(seg, w.s);
            Worst::keep(
                &mut out.normal,
                Worst {
                    line_no,
                    s: w.s,
                    v: w.v,
                    a_t: w.a,
                    kappa,
                    sigma,
                    ratio: normal_jerk(kappa, sigma, w.v, w.a).abs() / j_max,
                },
            );
        }
        for pair in vel.samples.windows(2) {
            let (lo, hi) = (pair[0], pair[1]);
            let ds = hi.s - lo.s;
            let v = 0.5 * (lo.v + hi.v);
            if ds <= 0.0 || v <= 0.0 {
                continue;
            }
            let dt = ds / v;
            let a_t = 0.5 * (lo.a + hi.a);
            let s = 0.5 * (lo.s + hi.s);
            let kappa = kappa_abs_at(seg, s);
            let sigma = dkappa_abs_ds_at(seg, s);
            let j_n = normal_jerk(kappa, sigma, v, a_t);
            let j_t = tangential_jerk(kappa, v, (hi.a - lo.a) / dt);
            Worst::keep(
                &mut out.vector,
                Worst {
                    line_no,
                    s,
                    v,
                    a_t,
                    kappa,
                    sigma,
                    ratio: libm::hypot(j_t, j_n) / j_max,
                },
            );
        }
    }
    out
}

fn audit_moves(moves: &[Move]) -> Audit {
    let (fitted, profile) = plan_for(moves);
    audit(&fitted, &profile)
}

/// A recorded overrun: `shape` bounds the components of the jerk vector the
/// tangential control cannot touch (`κ²v³` along the tangent as the centripetal
/// component swings, `σv³ + 3κva` along the normal as the curvature ramps),
/// computed pointwise from the emitted state and the fitted geometry — no
/// differencing, so the number is unarguable. `vector` bounds the whole vector
/// with the tangential rate differenced across grid cells, which understates a
/// bang-bang switch and therefore never overstates the overrun.
///
/// The planner currently caps corner speed on the acceleration disk alone, so
/// both are over the limit wherever a blend's curvature ramp is tight relative
/// to the jerk budget. These bounds are what the planner does today; they exist
/// to make a change's effect on jerk legible and to catch silent worsening.
/// [`blend_jerk_target`] carries the invariant the planner should hold.
fn assert_recorded(a: &Audit, label: &str, shape: f64, vector: f64) {
    assert!(
        Audit::ratio(a.straight) <= 1.0 + EXACT_JERK_REL_TOL,
        "{label}: straight phase jerk exceeds limit\n{}",
        a.summary()
    );
    assert!(
        Audit::ratio(a.normal) <= shape,
        "{label}: normal jerk overrun grew past the recorded {shape}\n{}",
        a.summary()
    );
    assert!(
        Audit::ratio(a.vector) <= vector,
        "{label}: vector jerk overrun grew past the recorded {vector}\n{}",
        a.summary()
    );
}

#[test]
fn straight_run_jerk_is_exactly_the_limit() {
    let m = Machine::user();
    let a = audit_moves(&m.polyline(&[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0]]));
    assert!(a.straight_phases > 0, "{}", a.summary());
    assert_eq!(a.curved_samples, 0, "{}", a.summary());
    assert!(
        (Audit::ratio(a.straight) - 1.0).abs() <= EXACT_JERK_REL_TOL,
        "instrument floor moved\n{}",
        a.summary()
    );
}

#[test]
fn user_square_perimeter_jerk_overrun_is_recorded() {
    let a = audit_moves(&square_perimeter(Machine::user(), 30.0));
    assert!(a.curved_samples > 0, "{}", a.summary());
    assert_recorded(&a, "square perimeter at user config", 1.03, 0.93);
}

#[test]
fn low_jerk_square_perimeter_jerk_overrun_is_recorded() {
    let m = Machine::user().jerk(100_000.0).accel(3_000.0);
    let a = audit_moves(&square_perimeter(m, 30.0));
    assert_recorded(&a, "square perimeter at low jerk", 16.7, 15.3);
}

#[test]
fn corner_angle_jerk_overrun_is_recorded() {
    // The overrun tracks how tight the blend's curvature ramp is: a 30 deg
    // corner spends 2.3x the budget on `sigma*v^3` alone, a 150 deg one 0.3x.
    let recorded = [
        (30.0, 2.28, 2.03),
        (90.0, 1.03, 0.93),
        (120.0, 0.70, 0.77),
        (150.0, 0.31, 1.03),
    ];
    for (theta_deg, shape, vector) in recorded {
        let a = audit_moves(&wedge(Machine::user(), 20.0, theta_deg));
        assert_recorded(&a, &format!("wedge {theta_deg} deg"), shape, vector);
    }
}

#[test]
fn faceted_arc_run_jerk_overrun_is_recorded() {
    let a = audit_moves(&faceted_arc(Machine::user(), 8.0, 24, 90.0));
    assert_recorded(&a, "faceted arc run", 1.01, 1.01);
}

#[test]
fn short_segment_zigzag_jerk_overrun_is_recorded() {
    let a = audit_moves(&zigzag(Machine::user(), 24, 0.8, 0.35));
    assert_recorded(&a, "short segment zigzag", 0.58, 1.03);
}

/// The invariant the planner should hold, and the acceptance gate for enforcing
/// it: the whole jerk vector within budget at every corner, *and* raising the
/// acceleration never slowing the print — the corner keeps its jerk-paced speed
/// while the straights take the extra budget. `j = 1.5e8` is time-neutral at
/// 60k for this geometry, so the sweep isolates exactly that trade.
#[test]
#[ignore = "the planner caps corner speed on the acceleration disk alone; the \
            jerk-domain cap `v <= cbrt(j / hypot(kappa^2, dkappa/ds))` is not \
            enforced yet. See the accompanying findings for the derivation, the \
            measured entry speeds, and why a bare pointwise cap is not enough."]
fn blend_jerk_target() {
    let base = Machine::user();
    let mut prev: Option<(f64, f64)> = None;
    let mut trace = String::new();
    for accel in [60_000.0, 80_000.0, 100_000.0, 140_000.0, 200_000.0] {
        let a = audit_moves(&square_perimeter(base.accel(accel), 30.0));
        trace.push_str(&format!("accel={accel:.0}\n{}\n", a.summary()));
        let label = format!("accel {accel:.0}");
        assert!(
            Audit::ratio(a.normal) <= 1.0 + EXACT_JERK_REL_TOL,
            "{label}: normal jerk component exceeds limit\n{trace}"
        );
        assert!(
            Audit::ratio(a.vector) <= 1.0 + VECTOR_JERK_REL_TOL,
            "{label}: vector jerk exceeds limit\n{trace}"
        );
        if let Some((prev_accel, prev_time)) = prev {
            assert!(
                a.time_s <= prev_time * (1.0 + 1e-9),
                "raising accel {prev_accel:.0} -> {accel:.0} slowed the print\n{trace}"
            );
        }
        prev = Some((accel, a.time_s));
    }
}

/// Curved members' `(s, v, a)` samples, keyed by source line.
fn curve_profiles(fitted: &FitOutcome, profile: &VelocityProfile) -> Vec<(u32, Vec<VelSample>)> {
    fitted
        .moves
        .iter()
        .zip(&profile.moves)
        .filter(|(fit, _)| {
            fit.segment
                .spatial
                .as_ref()
                .is_some_and(|seg| !is_straight(seg))
        })
        .map(|(_, vel)| (vel.source.start_line, vel.samples.clone()))
        .collect()
}

/// Raising `accel` while `corner_accel` stays put must leave every corner's
/// trajectory exactly as it was and never lengthen the print: the extra budget
/// goes to the straights' ramps, which is the only place it can go without
/// speeding a corner up.
#[test]
fn corner_accel_pins_the_corner_trajectory_while_straights_speed_up() {
    let pinned = 60_000.0;
    let base = Machine::user().accel(pinned).corner_accel(pinned);
    let (fit_ref, prof_ref) = plan_for(&square_perimeter(base, 30.0));
    let reference = curve_profiles(&fit_ref, &prof_ref);
    assert!(!reference.is_empty());

    let mut previous = prof_ref.report.traversal_time_s;
    for accel in [80_000.0, 100_000.0, 140_000.0, 200_000.0] {
        let m = base.accel(accel).corner_accel(pinned);
        let (fit, prof) = plan_for(&square_perimeter(m, 30.0));
        let got = curve_profiles(&fit, &prof);
        assert_eq!(
            got.len(),
            reference.len(),
            "corner count changed at {accel}"
        );
        for ((line, want), (_, have)) in reference.iter().zip(&got) {
            assert_eq!(
                want.len(),
                have.len(),
                "line {line} sample count at {accel}"
            );
            for (w, h) in want.iter().zip(have) {
                assert!(
                    (w.s - h.s).abs() <= 1e-9 && (w.v - h.v).abs() <= 1e-6 * (1.0 + w.v),
                    "line {line} corner trajectory moved at accel {accel}: \
                     s={:.7} v {:.6} -> {:.6}",
                    w.s,
                    w.v,
                    h.v
                );
            }
        }
        let time = prof.report.traversal_time_s;
        assert!(
            time <= previous * (1.0 + 1e-9),
            "raising accel to {accel} lengthened the print: {previous:.6} -> {time:.6}"
        );
        previous = time;
    }
}

#[test]
fn extreme_corner_accel_still_plans_promptly() {
    // A corner acceleration far below the straights' is an ordinary slow-corner
    // config, not a new regime — but it is the one knob a user can wind to an
    // extreme, so it gets a terminating-and-bounded check of its own.
    for corner_accel in [60_000.0, 1_000.0, 50.0, 1.0] {
        let m = Machine::user().accel(200_000.0).corner_accel(corner_accel);
        let (fitted, profile) = plan_for(&square_perimeter(m, 30.0));
        let phases: usize = profile.moves.iter().map(|v| v.phases.len()).sum();
        let samples: usize = profile.moves.iter().map(|v| v.samples.len()).sum();
        assert!(
            phases < 100_000 && samples < 1_000_000,
            "corner_accel {corner_accel}: profile blew up to {phases} phases / {samples} samples"
        );
        assert!(profile.report.traversal_time_s.is_finite());
        let _ = fitted;
    }
}

const CORNER_SWEEP_ACCELS: [f64; 7] = [
    20_000.0, 60_000.0, 80_000.0, 100_000.0, 140_000.0, 200_000.0, 300_000.0,
];
const CORNER_SWEEP_JERKS: [f64; 3] = [3.0e7, 6.0e7, 1.5e8];
const CORNER_SIDE_MM: f64 = 30.0;
const JERK_SAMPLES_PER_PHASE: usize = 33;
const APEX_PLATEAU_REL_TOL: f64 = 1e-6;

fn corner_halves(m: Machine) -> Vec<Kinematics> {
    let moves = m.polyline(&[
        [0.0, 0.0, 0.0],
        [CORNER_SIDE_MM, 0.0, 0.0],
        [CORNER_SIDE_MM, CORNER_SIDE_MM, 0.0],
    ]);
    let fitted = fit_corners(&moves, CornerFitConfig::default()).expect("fit");
    let halves: Vec<Kinematics> = fitted
        .moves
        .iter()
        .filter_map(|mv| match &mv.segment.spatial {
            Some(Segment::Clothoid(c)) => Some(Kinematics {
                length: c.s_len(),
                accel: m.accel,
                jerk: m.jerk,
                kappa0: c.kappa_endpoints().0,
                sigma: c.dkappa_ds(0.0),
                flat_ceiling: m.feed,
            }),
            _ => None,
        })
        .collect();
    assert_eq!(
        halves.len(),
        2,
        "the 90 degree corner must fit to a biclothoid"
    );
    halves
}

fn chain_jerk_worsts(kin: &Kinematics, chain: &[StraightPhase]) -> (Option<Worst>, Option<Worst>) {
    let mut shape = None;
    let mut exact = None;
    for p in chain {
        for i in 0..=JERK_SAMPLES_PER_PHASE {
            let tau = p.dt * (i as f64) / (JERK_SAMPLES_PER_PHASE as f64);
            let (s, v, a) = p.state_at(tau);
            let kappa = kin.kappa0 + kin.sigma * s;
            let j_n = normal_jerk(kappa, kin.sigma, v, a);
            let j_t = tangential_jerk(kappa, v, p.j);
            let at = |ratio| Worst {
                line_no: 0,
                s,
                v,
                a_t: a,
                kappa,
                sigma: kin.sigma,
                ratio,
            };
            Worst::keep(&mut shape, at(j_n.abs() / kin.jerk));
            Worst::keep(&mut exact, at(libm::hypot(j_t, j_n) / kin.jerk));
        }
    }
    (shape, exact)
}

struct CornerRow {
    accel: f64,
    jerk: f64,
    kappa_peak: f64,
    apex_v: f64,
    disk_apex_v: f64,
    jerk_rail_v: f64,
    entry_v: f64,
    entry_a: f64,
    exit_v: f64,
    exit_a: f64,
    time_s: f64,
    shape_ratio: f64,
    exact_ratio: f64,
    phases: usize,
}

impl CornerRow {
    fn jerk_binds(&self) -> bool {
        self.apex_v < self.disk_apex_v * (1.0 - APEX_PLATEAU_REL_TOL)
    }

    fn binding(&self) -> &'static str {
        if self.apex_v >= Machine::user().feed * (1.0 - APEX_PLATEAU_REL_TOL) {
            "flat"
        } else if self.jerk_binds() {
            "jerk"
        } else {
            "accel"
        }
    }

    fn row(&self) -> String {
        format!(
            "{:>7.0} {:>8.2e} {:>9.5} {:>10.4} {:>10.4} {:>10.4} {:>7} {:>10.4} {:>9.1e} {:>10.4} {:>9.1e} {:>10.6} {:>8.4} {:>8.4} {:>6}",
            self.accel,
            self.jerk,
            self.kappa_peak,
            self.apex_v,
            self.disk_apex_v,
            self.jerk_rail_v,
            self.binding(),
            self.entry_v,
            self.entry_a,
            self.exit_v,
            self.exit_a,
            self.time_s,
            self.shape_ratio,
            self.exact_ratio,
            self.phases,
        )
    }
}

const CORNER_TABLE_HEADER: &str = "  accel    j_max  kappa_pk    apex_v  disk_apex  jerk_rail   binds    entry_v   entry_a     exit_v    exit_a     time_s    shape    exact phases";

fn measure_corner(accel: f64, jerk: f64) -> Result<CornerRow, String> {
    let halves = corner_halves(Machine::user().accel(accel).jerk(jerk));
    let (entry_half, exit_half) = (&halves[0], &halves[1]);
    let kappa_peak = (entry_half.kappa0 + entry_half.sigma * entry_half.length).abs();
    let apex_v = top_speed_ceiling(entry_half).min(top_speed_ceiling(exit_half));
    let apex = (apex_v, 0.0);

    let entry = curved::entry_requirement(entry_half, apex)
        .map_err(|e| format!("entry_requirement failed: {e:?}"))?;
    let into_corner = curved::curved_chain(entry_half, entry, apex)
        .map_err(|e| format!("entry half chain failed: {e:?}"))?;
    let handoff = curved::curved_reach(exit_half, apex);
    let out_of_corner = curved::curved_chain(exit_half, apex, handoff)
        .map_err(|e| format!("exit half chain failed: {e:?}"))?;

    let (shape_in, exact_in) = chain_jerk_worsts(entry_half, &into_corner);
    let (shape_out, exact_out) = chain_jerk_worsts(exit_half, &out_of_corner);
    let peak = |a: Option<Worst>, b: Option<Worst>| {
        a.map_or(0.0, |w| w.ratio).max(b.map_or(0.0, |w| w.ratio))
    };

    Ok(CornerRow {
        accel,
        jerk,
        kappa_peak,
        apex_v,
        disk_apex_v: (accel / kappa_peak).sqrt(),
        jerk_rail_v: libm::cbrt(
            jerk / libm::hypot(kappa_peak * kappa_peak, entry_half.sigma.abs()),
        ),
        entry_v: entry.0,
        entry_a: entry.1,
        exit_v: handoff.0,
        exit_a: handoff.1,
        time_s: into_corner.iter().chain(&out_of_corner).map(|p| p.dt).sum(),
        shape_ratio: peak(shape_in, shape_out),
        exact_ratio: peak(exact_in, exact_out),
        phases: into_corner.len() + out_of_corner.len(),
    })
}

fn corner_sweep() -> (Vec<Vec<CornerRow>>, String) {
    let mut by_jerk = Vec::new();
    let mut table = format!("{CORNER_TABLE_HEADER}\n");
    for jerk in CORNER_SWEEP_JERKS {
        let mut rows = Vec::new();
        for accel in CORNER_SWEEP_ACCELS {
            match measure_corner(accel, jerk) {
                Ok(row) => {
                    table.push_str(&row.row());
                    table.push('\n');
                    rows.push(row);
                }
                Err(why) => {
                    table.push_str(&format!("{accel:>7.0} {jerk:>8.2e}  SOLVER ERROR: {why}\n"));
                }
            }
        }
        table.push('\n');
        by_jerk.push(rows);
    }
    (by_jerk, table)
}

/// Measurement, not a gate: the closed-form curved solver driven directly over
/// the reference 90 degree corner, so the acceptance answer is known before any
/// integration. Ignored because it is an instrument, and its cost belongs to
/// whoever asks for the number.
#[test]
#[ignore = "measurement instrument: run with --ignored --nocapture for the corner sweep table"]
fn curved_corner_sweep_table() {
    let (by_jerk, table) = corner_sweep();
    println!("{table}");
    assert_eq!(
        by_jerk.iter().map(Vec::len).sum::<usize>(),
        CORNER_SWEEP_ACCELS.len() * CORNER_SWEEP_JERKS.len(),
        "the solver refused part of the sweep\n{table}"
    );
}

/// The project's acceptance criterion, measured on the solver alone: raising
/// `max_accel` from a corner the acceleration disk limits into one the jerk ball
/// limits must never lengthen the corner, must never spend more jerk than the
/// budget, and must hold the apex speed flat once the jerk rail takes over.
#[test]
#[ignore = "acceptance measurement against the closed-form solver; no production caller yet"]
fn raising_accel_never_lengthens_the_corner() {
    let (by_jerk, table) = corner_sweep();
    for rows in &by_jerk {
        for pair in rows.windows(2) {
            let (lo, hi) = (&pair[0], &pair[1]);
            assert!(
                hi.time_s <= lo.time_s * (1.0 + 1e-9),
                "j={:.2e}: accel {:.0} -> {:.0} lengthened the corner {:.6}s -> {:.6}s\n{table}",
                lo.jerk,
                lo.accel,
                hi.accel,
                lo.time_s,
                hi.time_s
            );
            assert!(
                hi.apex_v >= lo.apex_v * (1.0 - APEX_PLATEAU_REL_TOL),
                "j={:.2e}: accel {:.0} -> {:.0} lowered the apex {:.4} -> {:.4}\n{table}",
                lo.jerk,
                lo.accel,
                hi.accel,
                lo.apex_v,
                hi.apex_v
            );
            if lo.jerk_binds() {
                assert!(
                    hi.jerk_binds()
                        && (hi.apex_v - lo.apex_v).abs() <= APEX_PLATEAU_REL_TOL * lo.apex_v,
                    "j={:.2e}: apex did not plateau past the jerk switch: {:.4} -> {:.4}\n{table}",
                    lo.jerk,
                    lo.apex_v,
                    hi.apex_v
                );
            }
        }
        for row in rows {
            assert!(
                row.exact_ratio <= 1.0 + EXACT_JERK_REL_TOL,
                "j={:.2e} accel={:.0}: emitted chain spends {:.4}x the jerk budget\n{table}",
                row.jerk,
                row.accel,
                row.exact_ratio
            );
        }
    }
}

/// The 90 degree blend the repro names, and the one the `clothoid/printer`
/// snapshot config names — whose own comment says the clothoid should ride at
/// flat acceleration with its jerk limit.
fn flat_ride_machines() -> [Machine; 2] {
    [
        Machine {
            feed: 190.0,
            accel: 70_000.0,
            corner_accel: f64::INFINITY,
            deviation: 0.05,
            jerk: 1.5e8,
        },
        Machine {
            feed: 300.0,
            accel: 1_000.0,
            corner_accel: f64::INFINITY,
            deviation: crate::corner_deviation_from_scv(5.0, 1_000.0),
            jerk: 6.0e5,
        },
    ]
}

fn chain_seconds(chain: &[StraightPhase]) -> f64 {
    chain.iter().map(|p| p.dt).sum()
}

fn idles(chain: &[StraightPhase]) -> bool {
    chain.iter().any(|p| p.j == 0.0 && p.a0 == 0.0)
}

fn assert_states_join(chain: &[StraightPhase], what: &str) {
    for pair in chain.windows(2) {
        let (s, v, a) = pair[0].end_state();
        let next = &pair[1];
        let joins = |lhs: f64, rhs: f64, scale: f64| (lhs - rhs).abs() <= 1e-9 * (scale + 1.0);
        assert!(
            joins(s, next.s0, s.abs())
                && joins(v, next.v0, v.abs())
                && joins(a, next.a0, a.abs())
                && joins(pair[0].t0 + pair[0].dt, next.t0, next.t0.abs()),
            "{what}: phase boundary is discontinuous, {:?} hands on to {next:?}",
            pair[0]
        );
    }
}

/// A blend half spends tangential authority without reversing the corner's
/// velocity dip. Coasting is accepted only when it remains above the apex speed
/// and measurably beats the certified constant-acceleration pass.
#[test]
fn a_blend_half_emits_a_monotone_authority_spending_pass() {
    for m in flat_ride_machines() {
        let halves = corner_halves(m);
        let (into_half, out_of_half) = (&halves[0], &halves[1]);
        assert!(
            into_half.sigma != 0.0 && out_of_half.sigma != 0.0,
            "a blend half with constant curvature has no descending ceiling to ride"
        );
        let apex_v = top_speed_ceiling(into_half).min(top_speed_ceiling(out_of_half));
        let apex = (apex_v, 0.0);
        let entry = curved::entry_requirement(into_half, apex).expect("entry requirement");
        let handoff = curved::curved_reach(out_of_half, apex);

        for (half, from, to, what) in [
            (into_half, entry, apex, "into the corner"),
            (out_of_half, apex, handoff, "out of the corner"),
        ] {
            let emitted =
                curved::curved_chain(half, from, to).expect("the emitted chain closes the half");
            if idles(&emitted) {
                let flat = curved::certified_flat_chain(half, from, to)
                    .expect("a coasting candidate requires a flat reference");
                assert!(
                    chain_seconds(&emitted) < 0.999 * chain_seconds(&flat)
                        && emitted
                            .iter()
                            .filter(|p| p.j == 0.0 && p.a0 == 0.0)
                            .all(|p| p.v0 > apex_v * 1.01),
                    "accel={} jerk={} {what}: coasting did not beat its reference: \
                     emitted={emitted:?}, flat={flat:?}",
                    m.accel,
                    m.jerk
                );
            }
            let descending = from.0 >= to.0;
            let sign_tol = 1e-9 * m.accel;
            assert!(
                emitted.iter().all(|p| {
                    let a1 = p.end_state().2;
                    if descending {
                        p.a0 <= sign_tol && a1 <= sign_tol
                    } else {
                        p.a0 >= -sign_tol && a1 >= -sign_tol
                    }
                }),
                "accel={} jerk={} {what}: emitted speed reverses the corner dip: {emitted:?}",
                m.accel,
                m.jerk
            );
            let peak_tangential_accel = emitted
                .iter()
                .map(|p| p.a0.abs().max(p.end_state().2.abs()))
                .fold(0.0_f64, f64::max);
            let exact_jerk = chain_jerk_worsts(half, &emitted)
                .1
                .expect("emitted chain has no jerk samples")
                .ratio;
            assert!(
                peak_tangential_accel >= 0.25 * m.accel && exact_jerk >= 0.85,
                "accel={} jerk={} {what}: left authority unused: tangential accel \
                 {peak_tangential_accel}, jerk ratio {exact_jerk}",
                m.accel,
                m.jerk
            );
            assert_states_join(&emitted, what);
        }
    }
}

#[test]
fn flat_clothoid_winds_each_edge_to_its_local_jerk_authority() {
    let machine = Machine {
        feed: 150.0,
        accel: 1_000.0,
        corner_accel: f64::INFINITY,
        deviation: crate::corner_deviation_from_scv(8.0, 1_000.0),
        jerk: 100_000.0,
    };
    let half = corner_halves(machine)[0].clone();
    let apex = (top_speed_ceiling(&half), 0.0);
    let entry = curved::entry_requirement(&half, apex).expect("entry requirement");
    let chain = curved::certified_flat_chain(&half, entry, apex).expect("flat chain");
    let winds: Vec<f64> = chain
        .iter()
        .filter_map(|phase| (phase.j != 0.0).then_some(phase.j.abs()))
        .collect();
    assert_eq!(winds.len(), 2);
    assert!(winds[0] > 0.7 * machine.jerk, "{winds:?}");
    assert!(winds[1] > 0.99 * machine.jerk, "{winds:?}");
    assert!(winds[1] > 1.2 * winds[0], "{winds:?}");
}

#[test]
fn clothoid_snapshot_trail_brakes_through_both_halves() {
    let machine = flat_ride_machines()[1];
    let moves = machine.polyline(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]);
    let (fitted, profile) = plan_for(&moves);
    let curved: Vec<_> = fitted
        .moves
        .iter()
        .zip(&profile.moves)
        .filter(|(mv, _)| matches!(mv.segment.spatial, Some(Segment::Clothoid(_))))
        .collect();
    assert_eq!(curved.len(), 2, "the snapshot corner must be a biclothoid");
    let into = curved[0].1;
    let out = curved[1].1;
    let apex_v = into.exit_v;
    assert!(
        into.entry_v > 1.5 * apex_v && out.exit_v > 1.5 * apex_v,
        "the clothoid envelope did not carry speed into the trail brake: {} -> {apex_v} -> {}",
        into.entry_v,
        out.exit_v
    );
    let into_a = into.phases.first().expect("empty entry half").a0;
    let out_a = out.phases.last().expect("empty exit half").end_state().2;
    assert!(
        into_a < -0.75 * machine.accel && out_a > 0.75 * machine.accel,
        "the straight-to-clothoid seams dropped the trail brake: {into_a}, {out_a}"
    );
    assert!(
        into.phases
            .iter()
            .all(|p| p.a0 <= 0.0 && p.end_state().2 <= 0.0)
            && out
                .phases
                .iter()
                .all(|p| p.a0 >= 0.0 && p.end_state().2 >= 0.0),
        "tangential acceleration stopped braking through the corner: {:?}, {:?}",
        into.phases,
        out.phases
    );
}
#[test]
fn fast_clothoid_snapshot_carries_full_acceleration_through_the_seams() {
    let machine = Machine {
        feed: 1_000.0,
        accel: 70_000.0,
        corner_accel: f64::INFINITY,
        deviation: crate::corner_deviation_from_scv(70.0, 70_000.0),
        jerk: 1.0e11,
    };
    let moves = machine.polyline(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]);
    let (fitted, profile) = plan_for(&moves);
    let curved: Vec<_> = fitted
        .moves
        .iter()
        .zip(&profile.moves)
        .filter(|(mv, _)| matches!(mv.segment.spatial, Some(Segment::Clothoid(_))))
        .collect();
    assert_eq!(
        curved.len(),
        2,
        "the fast snapshot corner must be a biclothoid"
    );
    let into = curved[0].1;
    let out = curved[1].1;
    assert!(
        into.entry_v > 100.0 && out.exit_v > 100.0,
        "the fast envelope left reachable corner speed unused: {} -> {} -> {}",
        into.entry_v,
        into.exit_v,
        out.exit_v
    );
    let into_a = into.phases.first().expect("empty entry half").a0;
    let out_a = out.phases.last().expect("empty exit half").end_state().2;
    assert!(
        into_a < -0.99 * machine.accel && out_a > 0.99 * machine.accel,
        "the fast envelope dropped acceleration at the straight seams: {into_a}, {out_a}"
    );
}
