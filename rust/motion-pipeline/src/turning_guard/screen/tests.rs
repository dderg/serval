use super::*;
use crate::turning_guard::test_motion::{Leaders, Motion, Rng, cross, norm, random_motion};

const H: f64 = 2.5e-4;
const CASES: u64 = 400;
const SAMPLES_PER_BLOCK: usize = 40;
const DEMAND: AdvanceDemand = AdvanceDemand {
    ratio: 0.035,
    slope_s: 0.03,
    curvature_s2_per_mm: 2e-5,
};

/// The advance's demand on the exact shaped motion at `t`, `None` in a
/// valley, where the advance lays its lead out instead of reading the path
/// speed through.
fn exact_demand(leaders: &Leaders, motion: &Motion, t: f64) -> Option<f64> {
    let (v, a, j) = motion.shaped(&leaders.axes(), t);
    let speed = norm(v);
    if speed < ADVANCE_VALLEY_SPEED_MM_S {
        return None;
    }
    let along = |x: [f64; 3]| (x[0] * v[0] + x[1] * v[1] + x[2] * v[2]) / speed;
    let tangential = along(a);
    let turning = norm(cross(a, v)).powi(2) / speed.powi(3);
    let flow = DEMAND.ratio * (tangential + DEMAND.slope_s * (along(j) + turning));
    Some(flow.abs() + DEMAND.curvature_s2_per_mm * (DEMAND.ratio * tangential).powi(2))
}

/// A random piece of a random motion as a stretch of grid cells, with the
/// summaries of the planned time the kernels read over it.
fn random_stretch(
    rng: &mut Rng,
    kernels: &FollowerKernels,
) -> (Motion, (i64, i64), Vec<MoveSummary>) {
    let motion = random_motion(rng);
    let piece_ends = motion.breaks();
    let index = (rng.unit() * (piece_ends.len() - 1) as f64) as usize;
    let stretch = (
        (piece_ends[index] / H).floor() as i64,
        (piece_ends[index + 1] / H).ceil() as i64,
    );
    let (back, ahead) = kernels.sample_span();
    let window = motion.summaries(
        (stretch.0 - back) as f64 * H,
        (stretch.1 + ahead) as f64 * H,
    );
    (motion, stretch, window)
}

fn instants((t0, t1): (f64, f64)) -> impl Iterator<Item = f64> {
    (0..=SAMPLES_PER_BLOCK).map(move |step| t0 + (t1 - t0) * step as f64 / SAMPLES_PER_BLOCK as f64)
}

/// Every block the screen proves asks no more than the limit at any of its
/// instants of the exact shaped motion, at limits either side of what the
/// stretch asks at worst.
#[test]
fn a_block_the_screen_proves_never_asks_more_than_the_limit_of_the_exact_shaped_motion() {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(H);
    let mut rng = Rng::new(0x005C_2EE7);
    let (mut proven, mut refused) = (0, 0);
    for _ in 0..CASES {
        let (motion, stretch, window) = random_stretch(&mut rng, &kernels);
        let blocks = Blocks::new(&kernels, stretch);
        let worst: Vec<f64> = (0..blocks.count())
            .map(|block| {
                instants(blocks.instants(block))
                    .filter_map(|t| exact_demand(&leaders, &motion, t))
                    .fold(0.0_f64, f64::max)
            })
            .collect();
        let limit = worst.iter().fold(0.0_f64, |a, b| a.max(*b)) * rng.between(0.5, 4.0);
        if limit == 0.0 {
            continue;
        }
        let verdict = screen(
            &kernels,
            &window,
            stretch,
            &DEMAND,
            |axes| leaders.jerk_gain(axes),
            limit,
        );
        for (block, worst) in worst.iter().enumerate() {
            let first_cell = stretch.0 + (block as i64) * blocks.cells;
            if !verdict.proves(first_cell) {
                refused += 1;
                continue;
            }
            proven += 1;
            assert!(
                *worst <= limit,
                "the screen proves block {block} whose exact shaped motion asks {worst} mm/s² \
                 over {limit}: {window:#?}"
            );
        }
    }
    assert!(
        proven > 100 && refused > 100,
        "proven {proven}, refused {refused}: the cases do not reach both sides"
    );
}

/// The jerk bound of each block holds the exact convolution's jerk on
/// every shaped axis at every instant of the block, and an isolated step
/// meets it: the bound is the largest of `|Δa|·|w|` over the block, which
/// the exact jerk reaches where the block holds the kernel's peak.
#[test]
fn the_switch_jerk_bound_holds_the_exact_shaped_jerk_on_every_axis() {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(H);
    let mut rng = Rng::new(0x0005_1EC7);
    let mut tightest = f64::INFINITY;
    for _ in 0..CASES {
        let (motion, stretch, window) = random_stretch(&mut rng, &kernels);
        let blocks = Blocks::new(&kernels, stretch);
        let mut bound = JerkBound::new(&kernels, blocks);
        for summary in &window {
            bound.add(summary);
        }
        for (block, sums) in bound.finish().iter().enumerate() {
            let mut seen = [0.0_f64; 3];
            for t in instants(blocks.instants(block)) {
                let (_, _, jerk) = motion.shaped(&leaders.axes(), t);
                for axis in 0..3 {
                    seen[axis] = seen[axis].max(jerk[axis].abs());
                }
            }
            for axis in 0..2 {
                assert!(
                    seen[axis] <= sums[axis] * (1.0 + 1e-9) + 1e-6,
                    "block {block}, axis {axis}: the exact shaped jerk reaches {} over a bound of \
                     {}: {window:#?}",
                    seen[axis],
                    sums[axis]
                );
                if seen[axis] > 0.0 {
                    tightest = tightest.min(sums[axis] / seen[axis]);
                }
            }
            assert_eq!(sums[2], 0.0, "the unfollowed axis reads no jerk");
        }
    }
    assert!(
        tightest < 1.02,
        "no block comes within 2% of its bound: tightest {tightest}"
    );
}
