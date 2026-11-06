use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{Move, MoveVelocity, RailPair, ScalarLaw};

/// How far apart, relative to the speed, the velocities either side of a
/// handover may lie and still count as one: past it the velocity steps, and
/// the acceleration's variation there is unbounded.
const HANDOVER_SPEED_TOLERANCE: f64 = 1e-9;

/// One stretch of a held move in time — a phase, or one knot pair of a
/// disk rail — as the variation of its planned acceleration reads it: the
/// acceleration and velocity at either end, per axis a bound on the rate
/// its acceleration changes at inside it, and on a rail the pair whose
/// tangential acceleration's own variation adds to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PhaseSpan {
    pub t0: f64,
    pub t1: f64,
    pub accel: [[f64; 3]; 2],
    pub velocity: [[f64; 3]; 2],
    pub rate: [f64; 3],
    pub rail: Option<RailStretch>,
}

/// A disk rail's knot pair starting `origin` after its phase: its
/// tangential acceleration's variation, along the heading, at most `lean`
/// of it on each axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct RailStretch {
    pair: RailPair,
    origin: f64,
    lean: [f64; 3],
}

impl RailStretch {
    fn variation(&self, from: f64, to: f64) -> [f64; 3] {
        let tangential = self
            .pair
            .acceleration_variation(from - self.origin, to - self.origin);
        self.lean.map(|share| share * tangential)
    }
}

impl PhaseSpan {
    /// A stretch of rest over `[t0, t1]`.
    pub fn rest(t0: f64, t1: f64) -> Self {
        Self {
            t0,
            t1,
            accel: [[0.0; 3]; 2],
            velocity: [[0.0; 3]; 2],
            rate: [0.0; 3],
            rail: None,
        }
    }
}

/// The stretches of move `m` starting at `t0`. Along the path the planned
/// acceleration `a_t·t̂ + v²κ·n̂` of a planar segment changes at
/// `ȧ_t·t̂ + (a_t·vκ − v³κ²)·t̂ + (2·a_t·vκ + v³·κ′)·n̂`, past the tangential
/// rate within `3·|a_t|·v̄κ̄ + v̄³·(|κ′| + κ̄²)`, `v̄` the stretch's fastest
/// speed and `κ̄` the segment's peak curvature, its axis share at most the
/// plane's lean on it. A constant acceleration has no tangential rate; a
/// disk rail's is read pair by pair off its interpolant
/// ([`RailPair::acceleration_variation`]), where its speed stays within
/// `(v_lo + v_hi + |a|·h)/2`.
pub(super) fn phase_spans(m: &Move, velocity: &MoveVelocity, t0: f64) -> Vec<PhaseSpan> {
    let duration: f64 = velocity.phases.iter().map(|phase| phase.dt).sum();
    let Some(segment) = &m.segment.spatial else {
        return vec![PhaseSpan::rest(t0, t0 + duration)];
    };
    let origin = velocity.phases.first().map_or(0.0, |phase| phase.s0);
    let length = segment.s_len();
    let local = |s: f64| (s - origin).clamp(0.0, length);
    let (_, kappa_peak) = segment.kappa_peak();
    let kappa_peak = kappa_peak.abs();
    let sigma = segment.dkappa_ds(0.0).abs();
    let lean: [f64; 3] = std::array::from_fn(|axis| segment.lean_on(axis).plane);
    let turning_rate = |accel: f64, fastest: f64| {
        let magnitude = 3.0 * accel * fastest * kappa_peak
            + fastest * fastest * fastest * (sigma + kappa_peak * kappa_peak);
        lean.map(|share| magnitude * share)
    };
    let ends = |(s, v, a): (f64, f64, f64)| {
        (
            accel_at(segment, local(s), v, a),
            velocity_at(segment, local(s), v),
        )
    };
    let mut spans = Vec::with_capacity(velocity.phases.len());
    let mut elapsed = 0.0;
    for phase in &velocity.phases {
        let begins = t0 + elapsed;
        elapsed += phase.dt;
        let finishes = t0 + elapsed;
        match phase.law {
            ScalarLaw::ConstAccel { a0 } => {
                let (start, finish) = (phase.state_at(phase.t0), phase.end_state());
                let ((accel_lo, velocity_lo), (accel_hi, velocity_hi)) =
                    (ends(start), ends(finish));
                spans.push(PhaseSpan {
                    t0: begins,
                    t1: finishes,
                    accel: [accel_lo, accel_hi],
                    velocity: [velocity_lo, velocity_hi],
                    rate: turning_rate(a0.abs(), start.1.abs().max(finish.1.abs())),
                    rail: None,
                });
            }
            ScalarLaw::DiskRail { .. } => {
                let first = spans.len();
                for pair in phase.rail_pairs(0.0, phase.dt) {
                    let peak = pair.acceleration_bound();
                    let fastest = 0.5 * (pair.v_lo + pair.v_hi + peak * (pair.t_hi - pair.t_lo));
                    let knot = |tau: f64, s: f64, v: f64| {
                        ends((phase.s0 + s, v, pair.acceleration_at(tau)))
                    };
                    let (accel_lo, velocity_lo) = knot(pair.t_lo, pair.s_lo, pair.v_lo);
                    let (accel_hi, velocity_hi) = knot(pair.t_hi, pair.s_hi, pair.v_hi);
                    spans.push(PhaseSpan {
                        t0: begins + pair.t_lo,
                        t1: begins + pair.t_hi,
                        accel: [accel_lo, accel_hi],
                        velocity: [velocity_lo, velocity_hi],
                        rate: turning_rate(peak, fastest),
                        rail: Some(RailStretch {
                            pair,
                            origin: begins,
                            lean,
                        }),
                    });
                }
                assert!(
                    spans.len() > first || phase.dt == 0.0,
                    "turning guard: a disk rail of {} s holds no knot pair",
                    phase.dt
                );
                if let Some(last) = spans[first..].last_mut() {
                    last.t1 = finishes;
                }
            }
        }
    }
    spans
}

fn accel_at(segment: &Segment, s: f64, v: f64, a: f64) -> [f64; 3] {
    let (heading, turning) = (segment.heading_at(s), segment.dheading_ds(s));
    std::array::from_fn(|axis| a * heading[axis] + v * v * turning[axis])
}

fn velocity_at(segment: &Segment, s: f64, v: f64) -> [f64; 3] {
    segment.heading_at(s).map(|component| v * component)
}

/// A bound on each axis's total variation of the planned acceleration over
/// the cell `[from, to)`, read off `spans`, the stretches in time order with
/// no gap between them, covering it: inside each stretch its rate times the
/// time spent there plus a rail pair's own variation there, at each
/// handover in the cell the step there, infinite where the velocity steps.
/// A handover at `to` is the next cell's: the cells of a grid count each
/// one once.
pub(super) fn variation_over<'a>(
    spans: impl Iterator<Item = &'a PhaseSpan>,
    (from, to): (f64, f64),
) -> [f64; 3] {
    let mut total = [0.0; 3];
    let mut before: Option<&PhaseSpan> = None;
    let mut covered = f64::NEG_INFINITY;
    for span in spans.skip_while(|span| span.t1 < from) {
        if span.t0 >= to {
            break;
        }
        if let Some(before) = before.filter(|_| span.t0 >= from) {
            let speed = before.velocity[1]
                .iter()
                .chain(&span.velocity[0])
                .fold(0.0_f64, |most, component| most.max(component.abs()));
            for axis in 0..3 {
                let step = (span.accel[0][axis] - before.accel[1][axis]).abs();
                let kink = (span.velocity[0][axis] - before.velocity[1][axis]).abs();
                total[axis] += if kink > HANDOVER_SPEED_TOLERANCE * (1.0 + speed) {
                    f64::INFINITY
                } else {
                    step
                };
            }
        }
        let (lo, hi) = (span.t0.max(from), span.t1.min(to));
        if hi > lo {
            let tangential = span.rail.map_or([0.0; 3], |rail| rail.variation(lo, hi));
            for axis in 0..3 {
                total[axis] += span.rate[axis] * (hi - lo) + tangential[axis];
            }
        }
        covered = covered.max(span.t1);
        before = Some(span);
    }
    assert!(
        covered >= to,
        "turning guard: the phases held cover the cell [{from}, {to}) only up to t={covered}"
    );
    total
}
