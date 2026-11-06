use super::*;
use crate::turning_guard::test_motion::{Leaders, Motion, Rng, cross, norm, random_motion};

const H: f64 = 2.5e-4;
const CASES: u64 = 400;
const SAMPLES_PER_MOVE: usize = 400;
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

/// A random piece of a random motion as a stretch on the grid, with the
/// summaries of the planned time the kernels read over it.
fn random_stretch(
    rng: &mut Rng,
    kernels: &FollowerKernels,
) -> (Motion, (f64, f64), Vec<MoveSummary>) {
    let motion = random_motion(rng);
    let piece_ends = motion.breaks();
    let index = (rng.unit() * (piece_ends.len() - 1) as f64) as usize;
    let stretch = (
        (piece_ends[index] / H).floor() * H,
        (piece_ends[index + 1] / H).ceil() * H,
    );
    let (back, ahead) = kernels.sample_span();
    let window = motion.summaries(stretch.0 - back as f64 * H, stretch.1 + ahead as f64 * H);
    (motion, stretch, window)
}

fn instants((t0, t1): (f64, f64)) -> impl Iterator<Item = f64> {
    (0..=SAMPLES_PER_MOVE).map(move |step| t0 + (t1 - t0) * step as f64 / SAMPLES_PER_MOVE as f64)
}

#[test]
fn a_move_the_screen_passes_never_asks_more_than_the_limit_of_the_exact_shaped_motion() {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(H);
    let mut rng = Rng::new(0x005C_2EE7);
    let mut checked = 0;
    for _ in 0..CASES {
        let (motion, stretch, window) = random_stretch(&mut rng, &kernels);
        let worst = instants(stretch)
            .filter_map(|t| exact_demand(&leaders, &motion, t))
            .fold(0.0_f64, f64::max);
        if worst == 0.0 {
            continue;
        }
        checked += 1;
        assert!(
            !passes(
                &kernels,
                window.iter().copied(),
                stretch,
                &DEMAND,
                |axes| leaders.jerk_gain(axes),
                worst * (1.0 - 1e-9),
            ),
            "the screen passes a stretch whose exact shaped motion asks {worst} mm/s²: \
             {window:#?}"
        );
    }
    assert!(
        checked * 2 > CASES as usize,
        "only {checked} cases asked anything"
    );
}

/// The jerk bound holds the exact convolution's jerk on every shaped axis
/// at every instant of the stretch, and an isolated step meets it: the
/// bound is the largest of `|Δa|·|w|` over the stretch, which the exact
/// jerk reaches where the stretch holds the kernel's peak.
#[test]
fn the_switch_jerk_bound_holds_the_exact_shaped_jerk_on_every_axis() {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(H);
    let mut rng = Rng::new(0x0005_1EC7);
    let mut tightest = f64::INFINITY;
    for _ in 0..CASES {
        let (motion, stretch, window) = random_stretch(&mut rng, &kernels);
        let mut bound = JerkBound::new(&kernels, stretch);
        for summary in &window {
            bound.add(summary);
        }
        let per_axis = bound.finish().per_axis;
        let mut seen = [0.0_f64; 3];
        for t in instants(stretch) {
            let (_, _, jerk) = motion.shaped(&leaders.axes(), t);
            for axis in 0..3 {
                seen[axis] = seen[axis].max(jerk[axis].abs());
            }
        }
        for axis in 0..2 {
            assert!(
                seen[axis] <= per_axis[axis] * (1.0 + 1e-9) + 1e-6,
                "axis {axis}: the exact shaped jerk reaches {} over a bound of {}: {window:#?}",
                seen[axis],
                per_axis[axis]
            );
            if seen[axis] > 0.0 {
                tightest = tightest.min(per_axis[axis] / seen[axis]);
            }
        }
        assert_eq!(per_axis[2], 0.0, "the unfollowed axis reads no jerk");
    }
    assert!(
        tightest < 1.02,
        "no case comes within 2% of its bound: tightest {tightest}"
    );
}
