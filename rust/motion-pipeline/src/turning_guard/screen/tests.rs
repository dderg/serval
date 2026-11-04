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

#[test]
fn a_move_the_screen_passes_never_asks_more_than_the_limit_of_the_exact_shaped_motion() {
    let leaders = Leaders::new();
    let kernels = leaders.grid_kernels(H);
    let mut rng = Rng::new(0x5C2E_E7);
    let mut checked = 0;
    for _ in 0..CASES {
        let motion = random_motion(&mut rng);
        let piece_ends = motion.breaks();
        let index = (rng.unit() * (piece_ends.len() - 1) as f64) as usize;
        let stretch = (piece_ends[index], piece_ends[index + 1]);
        let worst = (0..=SAMPLES_PER_MOVE)
            .filter_map(|step| {
                let t = stretch.0 + (stretch.1 - stretch.0) * step as f64 / SAMPLES_PER_MOVE as f64;
                exact_demand(&leaders, &motion, t)
            })
            .fold(0.0_f64, f64::max);
        if worst == 0.0 {
            continue;
        }
        checked += 1;
        let (from, to) = leaders.read(stretch);
        let window = motion.summaries(from, to);
        assert!(
            !passes(
                &kernels,
                window.iter().copied(),
                &DEMAND,
                |axes| leaders.jerk_gain(axes),
                worst * (1.0 - 1e-9),
            ),
            "the screen passes a stretch whose exact shaped motion asks {worst} mm/s²: \
             {window:#?}"
        );
    }
    assert!(checked * 2 > CASES as usize, "only {checked} cases asked anything");
}
