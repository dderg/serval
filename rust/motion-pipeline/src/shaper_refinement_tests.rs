use super::{AxisFit, MIN_DEVICE_INTERVAL_S, TrackSignal, fit_axis_from_signal};
use crate::lowering::FitTol;

#[derive(Clone, Copy)]
struct Motion {
    velocity: f64,
    acceleration: f64,
}

struct VelocityStep {
    at: f64,
    before: Motion,
    after: Motion,
}

impl TrackSignal for VelocityStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = t - self.at;
        let side = if t < self.at { self.before } else { self.after };
        (
            side.velocity * tau + side.acceleration * tau * tau / 2.0,
            side.velocity + side.acceleration * tau,
            side.acceleration,
        )
    }
}

fn narrowest_knot_span(track: &nurbs::ScalarNurbs) -> f64 {
    let mut knots = track.knots().to_vec();
    knots.dedup();
    knots
        .windows(2)
        .map(|span| span[1] - span[0])
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn bisecting_toward_an_unseeded_velocity_step_keeps_pieces_device_wide() {
    let at = 0.994_644_409_797_3;
    let step = VelocityStep {
        at,
        before: Motion {
            velocity: -215.828_87,
            acceleration: 11_700.5,
        },
        after: Motion {
            velocity: -215.757_83,
            acceleration: 11_809.0,
        },
    };
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start: 0.982_839_222_588_5,
            t_end: 1.002_872_787_996_2,
            seed_breakpoints: &[0.994_616_767_877, 0.994_655_895_934_5],
            fit_tol: FitTol {
                pos_mm: 0.0025,
                accel_mm_s2: 25.0,
            },
            fit_context: "unseeded velocity step",
            velocity_continuous: false,
        },
        &step,
    )
    .expect("a velocity step refines down to the device interval and fits");
    let narrowest = narrowest_knot_span(&track);
    assert!(
        narrowest >= MIN_DEVICE_INTERVAL_S,
        "refinement emitted a {narrowest:e} s piece, below the {MIN_DEVICE_INTERVAL_S:e} s \
         device interval"
    );
}
