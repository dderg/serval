//! Census of the velocity envelope's per-member reachability over the Neptune
//! corpus: every window the seam fuzzer draws from, fitted and planned once,
//! with the per-cause breakdown of the members whose chain could not be planned
//! between the two boundary states the envelope fixed for them.

use std::sync::LazyLock;

use crossbeam_channel::unbounded;
use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{BoundaryState, EntryReachability, InfeasibilityTally, Move, plan_velocity_stops};
use motion_core::seam_test_harness::{default_stream_config, parse_gcode_to_moves};
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::types::{StreamConfig, StreamInput};

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

const WINDOW: usize = 40;
const MIN_WINDOW: usize = 4;

static CORPUS: LazyLock<Vec<Move>> =
    LazyLock::new(|| parse_gcode_to_moves(NEPTUNE, default_stream_config().limits));

fn fit(window: &[Move], config: StreamConfig) -> Vec<Move> {
    let (tx, rx) = unbounded();
    let mut driver = FitStage::new(config.corner).into_driver(tx);
    for m in window {
        assert!(
            driver.feed(StreamInput::Move(m.clone())),
            "fit stage refused a move the corpus produced"
        );
    }
    driver.finish();
    drop(driver);
    rx.into_iter()
        .filter_map(|item| match item {
            StreamInput::Move(m) => Some(m),
            _ => None,
        })
        .collect()
}

/// The planner's own rest anchoring: a seam is anchored unless both sides have
/// spatial bodies and the path is tangent-continuous across it.
fn stop_before(moves: &[Move], config: &StreamConfig) -> Vec<bool> {
    (0..moves.len())
        .map(|i| {
            if i == 0 {
                return false;
            }
            let (Some(a), Some(b)) = (&moves[i - 1].segment.spatial, &moves[i].segment.spatial)
            else {
                return true;
            };
            let t_in = a.heading_at(a.s_len());
            let t_out = b.heading_at(0.0);
            let cos_theta = t_in[0] * t_out[0] + t_in[1] * t_out[1] + t_in[2] * t_out[2];
            libm::acos(cos_theta.clamp(-1.0, 1.0)) > config.corner.theta_min_rad
        })
        .collect()
}

#[derive(Default)]
struct Census {
    windows: u32,
    reach: EntryReachability,
}

impl Census {
    fn absorb(&mut self, r: &EntryReachability) {
        self.windows += 1;
        self.reach.straight_members += r.straight_members;
        self.reach.curved_members += r.curved_members;
        self.reach.unreachable += r.unreachable;
        self.reach.no_admissible_entry += r.no_admissible_entry;
        self.reach.accel_window_empty += r.accel_window_empty;
        add_tally(&mut self.reach.straight, &r.straight);
        add_tally(&mut self.reach.curved, &r.curved);
    }
}

fn add_tally(into: &mut InfeasibilityTally, from: &InfeasibilityTally) {
    into.length_too_short += from.length_too_short;
    into.unwind_over_ceiling += from.unwind_over_ceiling;
    into.unwind_below_rest += from.unwind_below_rest;
    into.accel_over_limit += from.accel_over_limit;
    into.speed_change_without_authority += from.speed_change_without_authority;
    into.length_not_closed += from.length_not_closed;
    into.non_finite += from.non_finite;
    into.unbounded_jerk_with_accel_boundary += from.unbounded_jerk_with_accel_boundary;
    into.uncertified_phase += from.uncertified_phase;
    into.other += from.other;
}

fn census() -> Census {
    let corpus = &*CORPUS;
    let config = default_stream_config();
    let n = corpus.len();
    assert!(n > MIN_WINDOW, "corpus is too small to window");
    let mut out = Census::default();
    for start in 0..(n - MIN_WINDOW) {
        let end = (start + WINDOW).min(n);
        let fitted = fit(&corpus[start..end], config);
        if fitted.len() < MIN_WINDOW {
            continue;
        }
        let stops = stop_before(&fitted, &config);
        let plan = plan_velocity_stops(
            &fitted,
            &stops,
            config.integration_tol,
            config.max_extrude_only_velocity_mm_s,
            config.max_extrude_only_accel_mm_s2,
            BoundaryState::REST,
        )
        .unwrap_or_else(|e| panic!("window [{start}..{end}) failed to plan: {e:?}"));
        out.absorb(&plan.report.reachability);
    }
    out
}

fn clothoid_members(moves: &[Move]) -> usize {
    moves
        .iter()
        .filter(|m| matches!(m.segment.spatial, Some(Segment::Clothoid(_))))
        .count()
}

/// Every member the envelope plans must be reachable from the entry state its
/// predecessor hands it. Until this holds the envelope chains cannot replace the
/// marched profile, so the number is a gate, not a diagnostic.
#[test]
fn every_envelope_member_is_reachable_from_its_required_entry_state() {
    let c = census();
    let r = &c.reach;
    let plans = r.member_plans();
    println!(
        "windows {} member-plans {} (straight {} curved {})\n  \
         unreachable {} ({:.2}%)  no_admissible_entry {}  accel_window_empty {}\n  \
         straight: len_short {} unwind_ceil {} unwind_rest {} accel_lim {} no_authority {} \
         not_closed {} nonfinite {} unbounded_jerk {} uncertified {} other {}\n  \
         curved:   len_short {} unwind_ceil {} unwind_rest {} accel_lim {} no_authority {} \
         not_closed {} nonfinite {} unbounded_jerk {} uncertified {} other {}",
        c.windows,
        plans,
        r.straight_members,
        r.curved_members,
        r.unreachable,
        100.0 * f64::from(r.unreachable) / f64::from(plans.max(1)),
        r.no_admissible_entry,
        r.accel_window_empty,
        r.straight.length_too_short,
        r.straight.unwind_over_ceiling,
        r.straight.unwind_below_rest,
        r.straight.accel_over_limit,
        r.straight.speed_change_without_authority,
        r.straight.length_not_closed,
        r.straight.non_finite,
        r.straight.unbounded_jerk_with_accel_boundary,
        r.straight.uncertified_phase,
        r.straight.other,
        r.curved.length_too_short,
        r.curved.unwind_over_ceiling,
        r.curved.unwind_below_rest,
        r.curved.accel_over_limit,
        r.curved.speed_change_without_authority,
        r.curved.length_not_closed,
        r.curved.non_finite,
        r.curved.unbounded_jerk_with_accel_boundary,
        r.curved.uncertified_phase,
        r.curved.other,
    );
    assert!(plans > 5_000, "the census lost its corpus: {plans} plans");
    assert_eq!(
        r.unreachable, 0,
        "{} of {plans} member-plans cannot be entered at the state the envelope requires",
        r.unreachable
    );
    assert_eq!(
        r.no_admissible_entry, 0,
        "{} members named no entry requirement at all",
        r.no_admissible_entry
    );
}

#[test]
fn the_corpus_windows_carry_curved_members() {
    let corpus = &*CORPUS;
    let fitted = fit(
        &corpus[0..WINDOW.min(corpus.len())],
        default_stream_config(),
    );
    assert!(
        clothoid_members(&fitted) > 0,
        "a window with no blend would make the curved census vacuous"
    );
}
