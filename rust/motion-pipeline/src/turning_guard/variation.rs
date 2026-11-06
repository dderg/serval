use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{Move, MoveVelocity, ScalarLaw};

/// How far apart, relative to the speed, the velocities either side of a
/// handover may lie and still count as one: past it the velocity steps, and
/// the acceleration's variation there is unbounded.
const HANDOVER_SPEED_TOLERANCE: f64 = 1e-9;

/// One phase of a held move in time, as the variation of its planned
/// acceleration reads it: the acceleration and velocity at either end, and
/// per axis a bound on the rate its acceleration changes at inside it,
/// infinite where none is known.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PhaseSpan {
    pub t0: f64,
    pub t1: f64,
    pub accel: [[f64; 3]; 2],
    pub velocity: [[f64; 3]; 2],
    pub rate: [f64; 3],
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
        }
    }
}

/// The phases of move `m` starting at `t0`. On a constant tangential
/// acceleration `a_t` the planned acceleration `a_t·t̂ + v²κ·n̂` of a
/// planar segment changes at
/// `|(a_t·vκ − v³κ²)·t̂ + (2·a_t·vκ + v³·κ′)·n̂| ≤ 3·|a_t|·v̄κ̄ + v̄³·(|κ′| + κ̄²)`,
/// `v̄` the phase's fastest speed and `κ̄` the segment's peak curvature, its
/// axis share at most the plane's lean on it; a disk rail's is not bounded
/// here.
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
    let mut elapsed = 0.0;
    velocity
        .phases
        .iter()
        .map(|phase| {
            let start = phase.state_at(phase.t0);
            let finish = phase.end_state();
            let begins = t0 + elapsed;
            elapsed += phase.dt;
            let rate = match phase.law {
                ScalarLaw::ConstAccel { a0 } => {
                    let fastest = start.1.abs().max(finish.1.abs());
                    let magnitude = 3.0 * a0.abs() * fastest * kappa_peak
                        + fastest * fastest * fastest * (sigma + kappa_peak * kappa_peak);
                    lean.map(|share| magnitude * share)
                }
                ScalarLaw::DiskRail { .. } => [f64::INFINITY; 3],
            };
            PhaseSpan {
                t0: begins,
                t1: t0 + elapsed,
                accel: [
                    accel_at(segment, local(start.0), start.1, start.2),
                    accel_at(segment, local(finish.0), finish.1, finish.2),
                ],
                velocity: [
                    velocity_at(segment, local(start.0), start.1),
                    velocity_at(segment, local(finish.0), finish.1),
                ],
                rate,
            }
        })
        .collect()
}

fn accel_at(segment: &Segment, s: f64, v: f64, a: f64) -> [f64; 3] {
    let (heading, turning) = (segment.heading_at(s), segment.dheading_ds(s));
    std::array::from_fn(|axis| a * heading[axis] + v * v * turning[axis])
}

fn velocity_at(segment: &Segment, s: f64, v: f64) -> [f64; 3] {
    segment.heading_at(s).map(|component| v * component)
}

/// A bound on each axis's total variation of the planned acceleration over
/// the cell `[from, to)`, read off `spans`, the phases in time order with
/// no gap between them, covering it: inside each phase its rate times the
/// time spent there, at each handover in the cell the step there, infinite
/// where the velocity steps or a rate is unknown. A handover at `to` is the
/// next cell's: the cells of a grid count each one once.
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
        let overlap = span.t1.min(to) - span.t0.max(from);
        if overlap > 0.0 {
            for axis in 0..3 {
                total[axis] += span.rate[axis] * overlap;
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
