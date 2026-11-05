use nurbs::algebra::PiecewisePolynomialKernel;

use super::*;

const REACH_S: f64 = 0.02;
const PATH_DEVIATION_MM: f64 = 0.005;

fn box_kernel(reach_s: f64) -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly(vec![0.5 / reach_s], (-reach_s, reach_s))
}

fn travel() -> SettleDemand {
    SettleDemand {
        ratio: 0.0,
        advance_slope_s: 0.0,
        advance_curvature_s2_per_mm: 0.0,
        own_jerk_per_accel_hz: f64::INFINITY,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 5000.0,
        extrude_only_velocity_mm_s: 50.0,
        extrude_only_accel_mm_s2: 5000.0,
    }
}

fn cell_s() -> f64 {
    REACH_S / SETTLE_CELLS as f64
}

/// The box kernel `1/2R` on `(−R, R)` leaves the shaped path within
/// `A·(R − h)³/12R` of the rest point `h` past a stop under `A`.
#[test]
fn a_box_kernel_settles_at_the_first_cell_its_cubic_tail_fits_the_radius() {
    let settle = RestSettle::of_kernels(&[&box_kernel(REACH_S)], PATH_DEVIATION_MM).settle();
    let radius = SETTLE_SHARE_OF_PATH_DEVIATION * PATH_DEVIATION_MM;
    assert_eq!(settle.radius_mm, radius);
    let accel = 10_000.0;
    let distance = |h: f64| accel * (REACH_S - h).powi(3) / (12.0 * REACH_S);
    for side in &settle.sides {
        let hold = side.hold_s(radius, accel, &travel());
        assert!(distance(hold) <= radius * (1.0 + 1e-4), "{hold}");
        assert!(distance(hold - cell_s()) > radius, "{hold}");
        assert!(hold < REACH_S, "{hold}");
    }
}

#[test]
fn without_a_shaped_leader_nothing_settles() {
    let settle = RestSettleId::UNSHAPED.settle();
    for side in &settle.sides {
        assert_eq!(side.hold_s(settle.radius_mm, 50_000.0, &travel()), 0.0);
    }
}

#[test]
fn the_extruder_settling_motion_holds_longer_where_it_lacks_room_and_fully_with_none() {
    let settle = RestSettle::of_kernels(&[&box_kernel(REACH_S)], PATH_DEVIATION_MM).settle();
    let side = &settle.sides[0];
    let accel = 10_000.0;
    let settled = side.hold_s(settle.radius_mm, accel, &travel());
    let extruding = SettleDemand {
        ratio: 0.05,
        max_velocity_mm_s: f64::INFINITY,
        max_accel_mm_s2: 5020.0,
        ..travel()
    };
    let tight = side.hold_s(settle.radius_mm, accel, &extruding);
    assert!(tight > settled && tight < REACH_S, "{settled} {tight}");
    let none = SettleDemand {
        max_accel_mm_s2: 5000.0,
        ..extruding
    };
    assert_eq!(side.hold_s(settle.radius_mm, accel, &none), REACH_S);
}

#[test]
fn the_wider_settle_holds_at_least_as_long_as_either() {
    let short = RestSettle::of_kernels(&[&box_kernel(0.5 * REACH_S)], PATH_DEVIATION_MM);
    let long = RestSettle::of_kernels(&[&box_kernel(REACH_S)], 2.0 * PATH_DEVIATION_MM);
    let wider = short.wider(long).settle();
    let (short, long) = (short.settle(), long.settle());
    assert_eq!(wider.radius_mm, short.radius_mm.min(long.radius_mm));
    for accel in [1_000.0, 10_000.0, 100_000.0] {
        for side in 0..2 {
            let hold =
                |settle: &RestSettle| settle.sides[side].hold_s(wider.radius_mm, accel, &travel());
            assert!(hold(&wider) >= hold(&short).max(hold(&long)), "{accel}");
        }
    }
    assert!(wider.check().is_ok());
}
