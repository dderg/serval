use super::*;
use crate::turning_guard::test_motion::{Leaders, Motion, Rng, cross, random_motion};

const POINTS_PER_CELL: usize = 4;
/// The rounding the exact and the gridded sums each carry, by quantity:
/// both add terms many orders above what they cancel to.
const VELOCITY_ROUNDING: f64 = 1e-6;
const ACCEL_ROUNDING: f64 = 1e-3;
const JERK_ROUNDING: f64 = 1.0;

struct Exact {
    velocity: [f64; 3],
    accel: [f64; 3],
    jerk: [f64; 3],
}

impl Exact {
    fn at(leaders: &Leaders, motion: &Motion, t: f64) -> Self {
        let (velocity, accel, jerk) = motion.shaped(&leaders.axes(), t);
        Self {
            velocity,
            accel,
            jerk,
        }
    }
}

fn assert_within(what: &str, got: f64, (least, most): (f64, f64), t: f64, rounding: f64) {
    let tol = 1e-9 * got.abs().max(least.abs()).max(most.abs()) + rounding;
    assert!(
        got >= least - tol && got <= most + tol,
        "{what} {got} at t={t} leaves its bound [{least}, {most}]"
    );
}

fn assert_samples_bound_the_exact_motion(sample: &ShapedSample, exact: &Exact, t: f64) {
    for axis in 0..3 {
        for (what, got, truth, miss, rounding) in [
            (
                "velocity",
                sample.velocity[axis],
                exact.velocity[axis],
                sample.velocity_miss[axis],
                VELOCITY_ROUNDING,
            ),
            (
                "acceleration",
                sample.accel[axis],
                exact.accel[axis],
                sample.accel_miss[axis],
                ACCEL_ROUNDING,
            ),
            (
                "jerk",
                sample.jerk[axis],
                exact.jerk[axis],
                sample.jerk_miss[axis],
                JERK_ROUNDING,
            ),
        ] {
            assert_within(
                &format!("the sampled {what} error on axis {axis}"),
                got - truth,
                (-miss, miss),
                t,
                rounding,
            );
        }
    }
}

fn assert_cell_bounds_the_exact_motion(cell: &CellMotion, exact: &Exact, t: f64) {
    let speed = norm(exact.velocity);
    let cross_norm = norm(cross(exact.accel, exact.velocity));
    assert_within(
        "the shaped speed",
        speed,
        (cell.speed_floor, f64::INFINITY),
        t,
        VELOCITY_ROUNDING,
    );
    assert_within(
        "the shaped |a|",
        norm(exact.accel),
        (0.0, cell.accel_ceiling),
        t,
        ACCEL_ROUNDING,
    );
    assert_within(
        "the shaped |a × v|",
        cross_norm,
        (0.0, cell.cross_ceiling),
        t,
        ACCEL_ROUNDING * speed.max(1.0),
    );
    assert_within(
        "the shaped |j|",
        norm(exact.jerk),
        (0.0, cell.jerk_ceiling),
        t,
        JERK_ROUNDING,
    );
    if speed < ADVANCE_VALLEY_SPEED_MM_S {
        return;
    }
    assert_within(
        "the shaped acceleration along the velocity",
        dot(exact.accel, exact.velocity) / speed,
        cell.tangential_accel,
        t,
        ACCEL_ROUNDING,
    );
    assert_within(
        "the shaped jerk along the velocity",
        dot(exact.jerk, exact.velocity) / speed,
        cell.tangential_jerk,
        t,
        JERK_ROUNDING,
    );
    assert_within(
        "the turning a⊥²/|v|",
        cross_norm * cross_norm / (speed * speed * speed),
        (0.0, cell.turning_ceiling()),
        t,
        ACCEL_ROUNDING,
    );
}

/// Every grid sample bounds the exact shaped motion at its instant within
/// its radii, and every cell's bounds hold at every instant inside it.
fn check_predictor(h: f64, cases: u64, seed: u64) {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(h);
    let (back, ahead) = kernels.sample_span();
    let mut rng = Rng::new(seed);
    for _ in 0..cases {
        let motion = random_motion(&mut rng);
        let (first, last) = (
            (-0.01 / h).floor() as i64,
            ((motion.end() + 0.01) / h).ceil() as i64,
        );
        let grid = motion.grid(h, first - back - 2..last + ahead + 2);
        let shaped = |n: i64| {
            shaped_at(&kernels, &grid, n)
                .unwrap_or_else(|| panic!("the grid holds every cell t={} reads", n as f64 * h))
        };
        let mut from = shaped(first);
        for n in first..last {
            let to = shaped(n + 1);
            let t = n as f64 * h;
            assert_samples_bound_the_exact_motion(&from, &Exact::at(&leaders, &motion, t), t);
            let cell = CellMotion::between(&kernels, &from, &to);
            for point in 0..=POINTS_PER_CELL {
                let t = (n as f64 + point as f64 / POINTS_PER_CELL as f64) * h;
                assert_cell_bounds_the_exact_motion(&cell, &Exact::at(&leaders, &motion, t), t);
            }
            from = to;
        }
    }
}

#[test]
fn the_coarse_grid_bounds_the_exact_shaped_motion_over_every_cell() {
    check_predictor(2.5e-4, 40, 0x5AD0_0001);
}

#[test]
fn the_zoomed_grid_bounds_the_exact_shaped_motion_over_every_cell() {
    check_predictor(2.5e-4 / 8.0, 4, 0x5AD0_0002);
}
