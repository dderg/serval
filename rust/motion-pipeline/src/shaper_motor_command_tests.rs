use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::bezier::{BezierPiece, extract_bezier_pieces};
use trajectory::{AxisChainSet, CompiledChain, ContinuousAxis, ContinuousSegment};

use super::materialize_changed_sources;
use crate::Pipeline;
use crate::types::{BaseItem, StreamConfig, TrajectoryItem};

const MAX_VELOCITY: f64 = 300.0;
const MAX_ACCEL: f64 = 3000.0;
const SQUARE_CORNER_VELOCITY: f64 = 5.0;
const FEEDRATE: f64 = 150.0;
const SMOOTH_TIME: f64 = 0.0015;
const MODE_FREQUENCY_HZ: f64 = 40.0;
const MODE_DAMPING_RATIO: f64 = 0.1;
const GAUSS_NODES: usize = 12;
const SAMPLES_PER_SEGMENT: usize = 97;

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(
        MAX_VELOCITY,
        MAX_ACCEL,
        geometry::corner_deviation_from_scv(SQUARE_CORNER_VELOCITY, MAX_ACCEL),
        f64::INFINITY,
    )
    .unwrap()
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

fn corner_moves() -> Vec<geometry::Move> {
    [[0.0, 0.0], [40.0, 0.0], [40.0, 40.0]]
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

fn mode_inverse_chain() -> CompiledChain {
    CompiledChain::compile(&[
        trajectory::PostProcessorInstance::new(
            "slew",
            &trajectory::algos::SmoothBell,
            vec![SMOOTH_TIME],
        ),
        trajectory::PostProcessorInstance::new(
            "belt",
            &trajectory::algos::ModeInverse,
            vec![MODE_FREQUENCY_HZ, MODE_DAMPING_RATIO],
        ),
    ])
    .expect("kernel + mode_inverse compiles")
}

fn xy_mode_inverse_chains() -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            mode_inverse_chain(),
            mode_inverse_chain(),
            CompiledChain::default(),
            CompiledChain::default(),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

struct Replay {
    shaper_input: Vec<ContinuousSegment>,
    motor_command: Vec<ContinuousSegment>,
}

fn replay(chains: &AxisChainSet, moves: &[geometry::Move]) -> Replay {
    let config = config();
    let mut pipeline = Pipeline::new(config, chains.clone(), vec![0.0; 4], 0.0);
    let Pipeline { front, shaper } = &mut pipeline;
    let mut shaper_input = Vec::new();
    let mut motor_command = Vec::new();
    let mut emit = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            motor_command.push(segment);
        }
        true
    };
    let mut shape = |item: BaseItem| {
        if let BaseItem::Seg(segment) = &item {
            let mut materialized = segment.clone();
            materialize_changed_sources(&mut materialized, chains, config.fit_tol())
                .expect("shaper input materializes");
            shaper_input.push(materialized);
        }
        shaper.feed(item, &mut emit)
    };
    for movement in moves {
        assert!(front.feed(movement.clone().into(), &mut shape));
    }
    assert!(front.finish(&mut shape));
    assert!(shaper.finish(&mut emit));
    Replay {
        shaper_input,
        motor_command,
    }
}

fn gauss_legendre(count: usize) -> Vec<(f64, f64)> {
    let legendre = |x: f64| {
        let (mut previous, mut current) = (1.0, x);
        for order in 2..=count {
            let next = ((2 * order - 1) as f64 * x * current - (order - 1) as f64 * previous)
                / order as f64;
            previous = current;
            current = next;
        }
        let slope = count as f64 * (x * current - previous) / (x * x - 1.0);
        (current, slope)
    };
    (0..count)
        .map(|index| {
            let mut x =
                libm::cos(std::f64::consts::PI * (index as f64 + 0.75) / (count as f64 + 0.5));
            for _ in 0..64 {
                let (value, slope) = legendre(x);
                let step = value / slope;
                x -= step;
                if step.abs() <= 1e-16 {
                    break;
                }
            }
            let (_, slope) = legendre(x);
            (x, 2.0 / ((1.0 - x * x) * slope * slope))
        })
        .collect()
}

struct KernelDerivatives {
    orders: [BezierPiece; 4],
}

impl KernelDerivatives {
    fn of(chain: &CompiledChain) -> Self {
        let kernel = chain.kernel().expect("chain has a kernel");
        assert_eq!(
            kernel.pieces.len(),
            1,
            "the reference convolves a single-piece kernel"
        );
        let k0 = kernel.pieces[0].clone();
        let k1 = k0.differentiate();
        let k2 = k1.differentiate();
        let k3 = k2.differentiate();
        for edge in [k0.u_start, k0.u_end] {
            assert!(k0.evaluate(edge).abs() <= 1e-9 && k1.evaluate(edge).abs() <= 1e-6);
        }
        Self {
            orders: [k0, k1, k2, k3],
        }
    }

    fn half_width(&self) -> f64 {
        0.5 * (self.orders[0].u_end - self.orders[0].u_start)
    }

    fn at(&self, order: usize, tau: f64) -> f64 {
        let piece = &self.orders[order];
        if tau < piece.u_start || tau > piece.u_end {
            return 0.0;
        }
        piece.evaluate(tau)
    }
}

struct SourceSignal {
    position: Vec<BezierPiece>,
    velocity: Vec<BezierPiece>,
    acceleration: Vec<BezierPiece>,
}

impl SourceSignal {
    fn of(segments: &[ContinuousSegment], axis: usize, reach: f64) -> Self {
        let hold = |value: f64, u_start: f64, u_end: f64| BezierPiece {
            u_start,
            u_end,
            coeffs: vec![value],
        };
        let mut position: Vec<BezierPiece> = Vec::new();
        for segment in segments {
            let ContinuousAxis::Spline(track) = &segment.axes[axis] else {
                panic!("materialized shaper input is a spline");
            };
            for piece in extract_bezier_pieces(track) {
                if piece.u_end <= piece.u_start {
                    continue;
                }
                if let Some(previous) = position.last() {
                    if piece.u_start > previous.u_end {
                        let held = previous.evaluate(previous.u_end);
                        position.push(hold(held, previous.u_end, piece.u_start));
                    }
                }
                position.push(piece);
            }
        }
        let first = position.first().expect("source has pieces");
        let before = hold(
            first.evaluate(first.u_start),
            first.u_start - reach,
            first.u_start,
        );
        let last = position.last().expect("source has pieces");
        let after = hold(last.evaluate(last.u_end), last.u_end, last.u_end + reach);
        position.insert(0, before);
        position.push(after);
        let velocity: Vec<BezierPiece> = position.iter().map(BezierPiece::differentiate).collect();
        let acceleration = velocity.iter().map(BezierPiece::differentiate).collect();
        Self {
            position,
            velocity,
            acceleration,
        }
    }
}

struct CommandReference {
    kernel: KernelDerivatives,
    source: SourceSignal,
    nodes: Vec<(f64, f64)>,
    k1: f64,
    k2: f64,
}

impl CommandReference {
    fn new(chain: &CompiledChain, segments: &[ContinuousSegment], axis: usize) -> Self {
        let Some(trajectory::ChainStage::DerivativeGains { k1, k2 }) = chain.trailing_transform()
        else {
            panic!("chain ends in motor-side derivative gains");
        };
        let kernel = KernelDerivatives::of(chain);
        Self {
            source: SourceSignal::of(segments, axis, 4.0 * kernel.half_width()),
            kernel,
            nodes: gauss_legendre(GAUSS_NODES),
            k1: *k1,
            k2: *k2,
        }
    }

    fn convolve(&self, pieces: &[BezierPiece], kernel_order: usize, t: f64) -> f64 {
        let reach = self.kernel.half_width();
        pieces
            .iter()
            .filter_map(|piece| {
                let lo = piece.u_start.max(t - reach);
                let hi = piece.u_end.min(t + reach);
                (hi > lo).then(|| {
                    let (mid, half) = (0.5 * (lo + hi), 0.5 * (hi - lo));
                    half * self
                        .nodes
                        .iter()
                        .map(|&(node, weight)| {
                            let s = mid + half * node;
                            weight * self.kernel.at(kernel_order, t - s) * piece.evaluate(s)
                        })
                        .sum::<f64>()
                })
            })
            .sum()
    }

    fn seam_steps(&self, t: f64) -> Vec<(f64, f64, f64)> {
        let reach = self.kernel.half_width();
        let source = &self.source;
        (1..source.position.len())
            .filter_map(|index| {
                let seam = source.position[index].u_start;
                let step = |pieces: &[BezierPiece]| {
                    pieces[index].evaluate(seam) - pieces[index - 1].evaluate(seam)
                };
                ((t - seam).abs() < reach)
                    .then(|| (t - seam, step(&source.position), step(&source.velocity)))
            })
            .collect()
    }

    fn shaped_derivatives(&self, t: f64) -> [f64; 5] {
        let source = &self.source;
        let steps = self.seam_steps(t);
        let stepped = |order: usize, with_velocity_step: bool| -> f64 {
            steps
                .iter()
                .map(|&(tau, position_step, velocity_step)| {
                    let position_term = position_step * self.kernel.at(order, tau);
                    if with_velocity_step {
                        velocity_step * self.kernel.at(order - 1, tau) + position_term
                    } else {
                        position_term
                    }
                })
                .sum()
        };
        [
            self.convolve(&source.position, 0, t),
            self.convolve(&source.velocity, 0, t) + stepped(0, false),
            self.convolve(&source.acceleration, 0, t) + stepped(1, true),
            self.convolve(&source.acceleration, 1, t) + stepped(2, true),
            self.convolve(&source.acceleration, 2, t) + stepped(3, true),
        ]
    }

    fn command_position_and_acceleration(&self, t: f64) -> (f64, f64) {
        let [p, v, a, j, s] = self.shaped_derivatives(t);
        (p + self.k1 * v + self.k2 * a, a + self.k1 * j + self.k2 * s)
    }
}

#[test]
fn mode_inverse_motor_command_holds_the_fit_tolerance_against_its_exact_signal() {
    let chains = xy_mode_inverse_chains();
    let replayed = replay(&chains, &corner_moves());
    let tolerance = config().fit_tol();
    for (axis, name) in [(0, "x"), (1, "y")] {
        let reference = CommandReference::new(&chains.chains[axis], &replayed.shaper_input, axis);
        let (mut worst_position, mut worst_acceleration) = ((0.0_f64, 0.0), (0.0_f64, 0.0));
        for segment in &replayed.motor_command {
            for sample in 0..SAMPLES_PER_SEGMENT {
                let t = segment.t_start
                    + (segment.t_end - segment.t_start) * (sample as f64 + 0.5)
                        / SAMPLES_PER_SEGMENT as f64;
                let emitted = segment.eval_axis(axis, t).expect("motor command evaluates");
                let (position, acceleration) = reference.command_position_and_acceleration(t);
                let position_error = (emitted.position - position).abs();
                let acceleration_error = (emitted.acceleration - acceleration).abs();
                if position_error > worst_position.0 {
                    worst_position = (position_error, t);
                }
                if acceleration_error > worst_acceleration.0 {
                    worst_acceleration = (acceleration_error, t);
                }
            }
        }
        assert!(
            worst_position.0 <= tolerance.pos_mm,
            "{name} motor command misses its exact signal by {} mm at t={}, tolerance {}",
            worst_position.0,
            worst_position.1,
            tolerance.pos_mm
        );
        assert!(
            worst_acceleration.0 <= tolerance.accel_mm_s2,
            "{name} motor command acceleration misses its exact signal by {} mm/s² at t={}, \
             tolerance {}",
            worst_acceleration.0,
            worst_acceleration.1,
            tolerance.accel_mm_s2
        );
    }
}

fn stepped_source() -> nurbs::ScalarNurbs {
    nurbs::bezier::bezier_pieces_to_discontinuous_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![0.0, 10.0, 3.0, 0.0],
        },
        BezierPiece {
            u_start: 0.5,
            u_end: 1.0,
            coeffs: vec![5.75 + 1e-3, 15.0, -4.0, 2.0],
        },
    ])
}

fn convolved<'a>(
    kernel: &'a nurbs::algebra::PiecewisePolynomialKernel,
    track: &nurbs::ScalarNurbs,
    gains: Option<(f64, f64)>,
) -> super::ShapedSignalSource<'a> {
    let table = super::AxisSignalTable::from_tracks(
        std::iter::once(track),
        super::SignalBounds {
            first_t: 0.0,
            last_t: 1.0,
            boundary: super::FitBoundary::StreamBoundary,
            lookahead: super::Lookahead::Forced,
        },
    );
    let table = match gains {
        Some((k1, k2)) => table.with_derivative_gains(k1, k2),
        None => table,
    };
    super::ShapedSignalSource::new(kernel, table, track.knots().to_vec(), 0.5)
}

#[test]
fn a_gained_convolution_carries_the_impulses_of_its_input_steps() {
    use super::TrackSignal;
    let chain = mode_inverse_chain();
    let kernel = chain.kernel().expect("mode_inverse rides a kernel");
    let Some(trajectory::ChainStage::DerivativeGains { k1, k2 }) = chain.trailing_transform()
    else {
        panic!("chain ends in motor-side derivative gains");
    };
    let (k1, k2) = (*k1, *k2);
    let track = stepped_source();
    let raw = convolved(kernel, &track, None);
    let gained = convolved(kernel, &track, Some((k1, k2)));
    let (raw, gained) = (raw.signal(), gained.signal());
    let worst = (0..=200)
        .map(|sample| 0.499 + 0.002 * (f64::from(sample) + 0.37) / 200.0)
        .map(|t| {
            let (p, v, a) = raw.eval_pva(t);
            (gained.eval(t) - (p + k1 * v + k2 * a)).abs()
        })
        .fold(0.0, f64::max);
    assert!(
        worst <= 1e-12,
        "the gained convolution misses x + k1·ẋ + k2·ẍ of the shaped signal by {worst} mm"
    );
}
