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
    deviation: f64,
    jerk: f64,
}

impl Machine {
    fn user() -> Self {
        Self {
            feed: 300.0,
            accel: 60_000.0,
            deviation: 0.05,
            jerk: 1.5e8,
        }
    }

    fn accel(self, accel: f64) -> Self {
        Self { accel, ..self }
    }

    fn jerk(self, jerk: f64) -> Self {
        Self { jerk, ..self }
    }

    fn ctx(&self, line_no: u32) -> MoveContext {
        MoveContext {
            extruder_axis: EXTRUDER_AXIS,
            feedrate_mm_s: self.feed,
            limits: VelocityLimits::try_new(self.feed, self.accel, self.deviation, self.jerk)
                .unwrap(),
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
