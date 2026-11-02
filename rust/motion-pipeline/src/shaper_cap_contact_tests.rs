//! How many pieces a shaped leader spends around a curvature-cap contact.

use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::bezier::extract_bezier_pieces;
use trajectory::{AxisChainSet, CompiledChain, ContinuousAxis, ShapedSignal};

use crate::Pipeline;
use crate::types::{BaseItem, StreamConfig, TrajectoryItem};

const MAX_VELOCITY: f64 = 300.0;
const MAX_ACCEL: f64 = 25_000.0;
const CORNER_DEVIATION: f64 = 0.05;
const FEEDRATE: f64 = 200.0;
const SMOOTH_TIME: f64 = 0.010;
const LEADER: usize = 0;

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(MAX_VELOCITY, MAX_ACCEL, CORNER_DEVIATION, f64::INFINITY).unwrap()
}

fn config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 64,
        limits: limits(),
    }
}

fn square_moves() -> Vec<geometry::Move> {
    [[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0], [0.0, 0.0]]
        .windows(2)
        .enumerate()
        .map(|(line, pair)| {
            line_move(
                [pair[0][0], pair[0][1], 0.0],
                [pair[1][0], pair[1][1], 0.0],
                0.0,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: FEEDRATE,
                    limits: limits(),
                    source: SourceRange {
                        start_line: line as u32,
                        end_line: line as u32,
                    },
                },
            )
            .unwrap()
        })
        .collect()
}

fn bell_chain() -> CompiledChain {
    CompiledChain::compile(&[trajectory::PostProcessorInstance::new(
        "shaper",
        &trajectory::algos::SmoothBell,
        vec![SMOOTH_TIME],
    )])
    .expect("smooth bell compiles")
}

fn xy_bell_chains() -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            bell_chain(),
            bell_chain(),
            CompiledChain::default(),
            CompiledChain::default(),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

struct Replay {
    cap_contacts: Vec<f64>,
    leader_knots: Vec<f64>,
}

fn replay(chains: &AxisChainSet) -> Replay {
    let mut pipeline = Pipeline::new(config(), chains.clone(), vec![0.0; 4], 0.0);
    let Pipeline { front, shaper } = &mut pipeline;
    let mut cap_contacts = Vec::new();
    let mut leader_knots = Vec::new();
    let mut emit = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            let ContinuousAxis::Spline(track) = &segment.axes[LEADER] else {
                panic!("the shaped leader is not a spline");
            };
            leader_knots.extend(
                extract_bezier_pieces(track)
                    .iter()
                    .flat_map(|piece| [piece.u_start, piece.u_end]),
            );
        }
        true
    };
    let mut shape = |item: BaseItem| {
        if let BaseItem::Seg(segment) = &item {
            if let ContinuousAxis::Analytic { span, .. } = &segment.axes[LEADER] {
                cap_contacts.extend(span.cap_contact_times());
            }
        }
        shaper.feed(item, &mut emit)
    };
    for movement in square_moves() {
        assert!(front.feed(movement.into(), &mut shape));
    }
    assert!(front.finish(&mut shape));
    assert!(shaper.finish(&mut emit));
    leader_knots.sort_by(f64::total_cmp);
    leader_knots.dedup();
    Replay {
        cap_contacts,
        leader_knots,
    }
}

/// Window around a contact's image through one kernel cut that holds the
/// images of the materialize seeds graded toward it.
const CONTACT_IMAGE_RADIUS_S: f64 = 2e-4;

/// Shaped-leader knots one contact image may cost: its own break plus a
/// refinement split toward a neighbouring image.
const KNOTS_PER_CONTACT_IMAGE: usize = 2;

fn count_within(sorted: &[f64], centre: f64) -> usize {
    let lo = sorted.partition_point(|&t| t < centre - CONTACT_IMAGE_RADIUS_S);
    let hi = sorted.partition_point(|&t| t <= centre + CONTACT_IMAGE_RADIUS_S);
    hi - lo
}

/// Graded materialize seeds only let the source fit follow the square-root
/// shoulder of a cap contact; their knots carry fit residuals the kernel
/// smooths away, so the shaped leader breaks at a contact's images and not
/// at the images of every seed graded toward it.
#[test]
fn a_cap_contact_costs_the_shaped_leader_a_bounded_number_of_pieces() {
    let chains = xy_bell_chains();
    let replay = replay(&chains);
    assert!(
        !replay.cap_contacts.is_empty(),
        "the square's corners must touch the curvature cap"
    );
    let kernel = chains.chains[LEADER].kernel().expect("bell kernel");
    let cuts: Vec<f64> = ShapedSignal::kernel_cut_boundaries(kernel).collect();
    let mut images: Vec<f64> = replay
        .cap_contacts
        .iter()
        .flat_map(|&contact| cuts.iter().map(move |&cut| contact + cut))
        .collect();
    images.sort_by(f64::total_cmp);
    images.dedup();
    for &image in &images {
        let knots = count_within(&replay.leader_knots, image);
        let contact_images = count_within(&images, image);
        assert!(
            knots <= KNOTS_PER_CONTACT_IMAGE * contact_images,
            "the shaped leader spends {knots} knots within {CONTACT_IMAGE_RADIUS_S} s of the \
             contact image at {image} s, which holds {contact_images} contact images"
        );
    }
}
