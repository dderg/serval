use std::sync::Arc;

use nurbs::ScalarNurbs;
use trajectory::{ClockedMotorSpan, MotorGroup, MotorSpan};

use super::{Slope, StepRootCursor};
use crate::{MotorConfig, StepEncoder};

#[test]
fn endpoint_roundoff_does_not_hide_a_monotonic_spline() {
    let duration = 0.0045;
    let curve = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, duration, duration, duration],
        vec![139.7, 140.000_000_000_000_5, 139.999_999_999_999_8],
    )
    .expect("a quadratic spline with endpoint-scale roundoff");
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Spline {
                curve: Arc::new(curve),
                summed_scale: 1.0,
            }]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    let view =
        ClockedMotorSpan::try_new(signal, 0.0, duration, 0.0, duration, 1_000.0, 520_000_000.0)
            .expect("a clocked motor span");
    let config = MotorConfig {
        oid: 2,
        microstep_distance: 0.008,
        invert_dir: false,
        cycles_per_second: 520_000_000.0,
        encoder: StepEncoder::Classic {
            max_error_ticks: 13_000,
        },
        min_rearm_cycles: 0,
    };
    let cursor = StepRootCursor::new(&config);

    assert_eq!(
        cursor
            .certified_slope(0, &view, view.start_clock, view.end_clock)
            .unwrap(),
        Some(Slope::Rising)
    );
}
