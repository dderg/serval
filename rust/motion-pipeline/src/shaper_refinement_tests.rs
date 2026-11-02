use super::{AxisFit, MIN_DEVICE_INTERVAL_S, TrackSignal, fit_axis_from_signal};
use crate::lowering::FitTol;
use crate::types::PostProcessError;

#[derive(Clone, Copy)]
struct Motion {
    velocity: f64,
    acceleration: f64,
}

struct AccelerationStep {
    at: f64,
    velocity: f64,
    before: f64,
    after: f64,
}

impl TrackSignal for AccelerationStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = t - self.at;
        let acceleration = if t < self.at { self.before } else { self.after };
        (
            self.velocity * tau + acceleration * tau * tau / 2.0,
            self.velocity + acceleration * tau,
            acceleration,
        )
    }
}

struct RampToRest {
    at: f64,
    ramp_s: f64,
    approach: Motion,
}

impl TrackSignal for RampToRest {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let Motion {
            velocity,
            acceleration,
        } = self.approach;
        let tau = t - self.at;
        if tau < 0.0 {
            return (
                velocity * tau + acceleration * tau * tau / 2.0,
                velocity + acceleration * tau,
                acceleration,
            );
        }
        let ramp = tau.min(self.ramp_s);
        let deceleration = velocity / self.ramp_s;
        if tau < self.ramp_s {
            (
                velocity * ramp - deceleration * ramp * ramp / 2.0,
                velocity - deceleration * ramp,
                -deceleration,
            )
        } else {
            (velocity * self.ramp_s / 2.0, 0.0, 0.0)
        }
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
fn bisecting_toward_an_unseeded_acceleration_step_keeps_pieces_device_wide() {
    let step = AccelerationStep {
        at: 0.994_644_409_797_3,
        velocity: -215.828_87,
        before: 11_700.5,
        after: 11_809.0,
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
            fit_context: "unseeded acceleration step",
            velocity_continuous: false,
        },
        &step,
    )
    .expect("an acceleration step refines down to the device interval and fits");
    let narrowest = narrowest_knot_span(&track);
    assert!(
        narrowest >= MIN_DEVICE_INTERVAL_S,
        "refinement emitted a {narrowest:e} s piece, below the {MIN_DEVICE_INTERVAL_S:e} s \
         device interval"
    );
}

#[test]
fn a_picometre_ramp_to_rest_at_the_resolution_floor_fails_loudly() {
    let ramp = RampToRest {
        at: 4.340_821_412_13,
        ramp_s: 4e-14,
        approach: Motion {
            velocity: 0.1275,
            acceleration: -155.84,
        },
    };
    let fit = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start: 4.33,
            t_end: 4.35,
            seed_breakpoints: &[ramp.at - 0.7e-5, ramp.at + 1.3e-5],
            fit_tol: FitTol {
                pos_mm: 0.0025,
                accel_mm_s2: 50.0,
            },
            fit_context: "picometre ramp to rest",
            velocity_continuous: false,
        },
        &ramp,
    );
    match fit {
        Err(PostProcessError::VelocityStepAtFitFloor {
            axis,
            fit_context,
            t_start,
            t_end,
            velocity_change,
            ..
        }) => {
            assert_eq!((axis, fit_context), (3, "picometre ramp to rest"));
            assert!(
                t_start <= ramp.at && ramp.at + ramp.ramp_s <= t_end,
                "the reported span [{t_start}, {t_end}] misses the ramp at {}",
                ramp.at
            );
            assert!(
                (velocity_change + 0.1275).abs() < 1e-3,
                "the reported velocity change {velocity_change} is not the ramp's"
            );
        }
        Err(other) => panic!("a ramp to rest failed with an unrelated error: {other}"),
        Ok(track) => panic!(
            "a ramp to rest narrower than the resolution floor was accepted as {} knots",
            track.knots().len()
        ),
    }
}
