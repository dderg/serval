use super::*;
use nurbs::bezier::BezierPiece;

const SAMPLES: usize = 40_000;
const CEILING: f64 = 150.0;
const ACCELS: [f64; 3] = [2_000.0, 20_000.0, 80_000.0];

fn boxed(half_width: f64) -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly(vec![0.5 / half_width], (-half_width, half_width))
}

fn piece(u_start: f64, u_end: f64, coeffs: Vec<f64>) -> BezierPiece {
    BezierPiece {
        u_start,
        u_end,
        coeffs,
    }
}

/// `(1 − |u|/T)/T` on `[−T, T]`.
fn triangle(half_width: f64) -> PiecewisePolynomialKernel {
    let peak = 1.0 / half_width;
    let slope = peak / half_width;
    PiecewisePolynomialKernel::from_pieces(vec![
        piece(-half_width, 0.0, vec![0.0, slope]),
        piece(0.0, half_width, vec![peak, -slope]),
    ])
    .expect("a contiguous kernel")
}

/// A kernel with negative lobes either side of a positive core, its mass
/// above 1.
fn lobed(half_width: f64) -> PiecewisePolynomialKernel {
    let third = half_width / 3.0;
    let core = 1.3 / (2.0 * third);
    let lobe = -0.15 / (2.0 * third);
    PiecewisePolynomialKernel::from_pieces(vec![
        piece(-half_width, -third, vec![lobe]),
        piece(-third, third, vec![core]),
        piece(third, half_width, vec![lobe]),
    ])
    .expect("a contiguous kernel")
}

/// `∫|w(u)|·min(C, v + A·|u|) du` by the midpoint rule over each piece.
fn sampled_shaped_speed(kernel: &PiecewisePolynomialKernel, planned: f64, accel: f64) -> f64 {
    kernel
        .pieces
        .iter()
        .flat_map(|piece| {
            let width = (piece.u_end - piece.u_start) / SAMPLES as f64;
            (0..SAMPLES).map(move |step| {
                let u = piece.u_start + (step as f64 + 0.5) * width;
                piece.evaluate(u).abs() * CEILING.min(planned + accel * u.abs()) * width
            })
        })
        .sum()
}

fn kernels() -> Vec<(&'static str, PiecewisePolynomialKernel)> {
    vec![
        ("box", boxed(0.004)),
        ("triangle", triangle(0.009)),
        ("lobed", lobed(0.006)),
    ]
}

#[test]
fn the_spill_bounds_the_shaped_speed_and_never_exceeds_the_linear_bound() {
    for (name, kernel) in kernels() {
        let spill = LeaderSpill::of_kernels(&[&kernel]).spill();
        let (mass, moment) = kernel.absolute_moments();
        for accel in ACCELS {
            for planned in (0..=30).map(|step| CEILING * f64::from(step) / 30.0) {
                let bound = spill.shaped_speed(planned, CEILING, accel);
                let sampled = sampled_shaped_speed(&kernel, planned, accel);
                assert!(
                    bound >= sampled - 1e-6 * CEILING,
                    "{name}: {bound} under the sampled {sampled} at {planned} mm/s, {accel} mm/s²"
                );
                let linear = (mass * planned + moment * accel).min(mass * CEILING);
                assert!(
                    bound <= linear + 1e-9 * CEILING,
                    "{name}: {bound} over the linear {linear} at {planned} mm/s, {accel} mm/s²"
                );
            }
        }
    }
}

#[test]
fn the_shaped_speed_reaches_the_ceilings_spill_and_slopes_off_strictly_below_it() {
    for (name, kernel) in kernels() {
        let spill = LeaderSpill::of_kernels(&[&kernel]).spill();
        for accel in ACCELS {
            assert_eq!(
                spill.shaped_speed(CEILING, CEILING, accel),
                spill.mass * CEILING
            );
            let speeds: Vec<f64> = (0..=300)
                .map(|step| spill.shaped_speed(CEILING * f64::from(step) / 300.0, CEILING, accel))
                .collect();
            assert!(
                speeds.windows(2).all(|pair| pair[0] < pair[1]),
                "{name}: the shaped speed stalls under {accel} mm/s²"
            );
        }
    }
}

#[test]
fn the_bound_overshoots_the_shaped_speed_by_at_most_a_cells_spill() {
    for (name, kernel) in kernels() {
        let spill = LeaderSpill::of_kernels(&[&kernel]).spill();
        for accel in ACCELS {
            for planned in (0..=30).map(|step| CEILING * f64::from(step) / 30.0) {
                let bound = spill.shaped_speed(planned, CEILING, accel);
                let sampled = sampled_shaped_speed(&kernel, planned, accel);
                assert!(
                    bound - sampled <= accel * spill.mass * spill.cell_s + 1e-6 * CEILING,
                    "{name}: {bound} over the sampled {sampled} at {planned} mm/s, {accel} mm/s²"
                );
            }
        }
    }
}

#[test]
fn an_unshaped_path_moves_at_its_planned_speed() {
    for accel in ACCELS {
        for planned in [0.0, 37.0, CEILING] {
            let shaped = LeaderSpillId::UNSHAPED
                .spill()
                .shaped_speed(planned, CEILING, accel);
            assert!(
                (shaped - planned).abs() <= 1e-12 * CEILING,
                "{shaped} against {planned}"
            );
        }
    }
}

#[test]
fn a_wider_spill_bounds_both() {
    let narrow = LeaderSpill::of_kernels(&[&boxed(0.003)]);
    let wide = LeaderSpill::of_kernels(&[&lobed(0.008)]);
    let unshaped = LeaderSpillId::UNSHAPED;
    for (a, b) in [(narrow, wide), (wide, narrow), (narrow, unshaped)] {
        let wider = a.wider(b).spill();
        assert!(wider.check().is_ok(), "{wider:?}");
        for accel in ACCELS {
            for planned in (0..=30).map(|step| CEILING * f64::from(step) / 30.0) {
                let bound = wider.shaped_speed(planned, CEILING, accel);
                for spill in [a.spill(), b.spill()] {
                    assert!(bound >= spill.shaped_speed(planned, CEILING, accel) - 1e-9 * CEILING);
                }
            }
        }
    }
    assert_eq!(narrow.wider(narrow), narrow);
}
