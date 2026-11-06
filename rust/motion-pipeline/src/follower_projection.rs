use std::borrow::Cow;
use std::cell::RefCell;
use std::sync::{Arc, LazyLock};

use geometry::MIN_DEVICE_INTERVAL_S;
use nurbs::ScalarNurbs;
use nurbs::bezier::{
    BezierPiece, bezier_piece_ends, bezier_pieces_to_nurbs, extract_bezier_pieces,
};
use trajectory::{
    AxisChainSet, ChainStage, CompiledChain, ContinuousAxis, ContinuousSegment,
    RelativeSplinePiece, ShapedSignal,
};

use crate::lowering::{AccelBudget, FitTol, SeamContinuity, riding_tol_scale};
use crate::shaper::{
    AxisFit, AxisSignalTable, ConvolutionSignal, CruiseEnds, Enclosure, ExactConvolution,
    ExactKernel, ExemptAccelerations, FitBoundary, Lookahead, SEGMENT_TIME_EPS_S, ShapedBreaks,
    ShiftedTrackSignal, SignalBounds, TrackSignal, apply_derivative_gains_to_track,
    fit_axis_from_signal, fit_resolution, fit_seed_times, unit_kernel_mass,
};
use crate::types::PostProcessError;
use crate::worker_pool::WorkerPool;

mod enclosure;
mod extrude_only;
mod ratio_ramp;
mod rest;
use ratio_ramp::{RampLeg, RatioSmoothing};

const INTEGRAL_TOL_MM: f64 = 1e-10;
const ODOMETER_RATE_TOL_MM_S: f64 = 1e-6;
const INTEGRAL_MAX_DEPTH: u32 = 24;
const GRID_DEDUP_EPS_S: f64 = 1e-12;
const SPAN_MIN_LEN_MM: f64 = 1e-12;
/// A raw leader chord this long proves their arc, which the odometer
/// integrates far finer, longer than [`SPAN_MIN_LEN_MM`].
const MOVING_CHORD_MM: f64 = 1e-6;
/// The relative rounding a leader speed read off its pieces is trusted to,
/// of the magnitude of the terms it sums.
const SPEED_ROUNDING: f64 = 1e-9;
const SPAN_LOOKUP_SLACK_MM: f64 = 1e-6;
const PVA_MEMO_SLOTS: usize = 64;
const BASIS_CONVERSION_ROUNDOFF_ENVELOPE: f64 = (1_u64 << 20) as f64;
const ODOMETER_ALIGNMENT_ULPS: f64 = (1_u64 << 12) as f64;
/// The share of the leaders' first or last piece next to an endpoint at
/// rest over which the follower reads their speed as leaving or reaching
/// zero at the endpoint's acceleration ([`SpeedZeroSliver`]): within one
/// piece, so a sliver never reads a brake into the cruise before it.
const SPEED_ZERO_SLIVER_FRACTION: f64 = 1e-6;
/// Relative mismatch between a sliver's speed rate and the leaders' own
/// at its inner edge that rounding alone leaves where the leaders brake
/// straight into the rest.
const SLIVER_RATE_ROUNDING: f64 = 1e-9;
const COMPONENT_ZERO_PROBES: usize = 16;
/// The share of the acceleration tolerance under which a slow leader
/// component counts as at rest ([`RestResolution`]).
const REST_ACCELERATION_SHARE: f64 = 1.0 / (1_u64 << 30) as f64;
fn follower_fit_tol(fit_tol: FitTol, position_scale: f64) -> FitTol {
    FitTol {
        pos_mm: fit_tol.pos_mm * position_scale,
        ..fit_tol
    }
}

/// What a follower's source fit spends on acceleration. An advance applied
/// before projection makes the source the advanced signal, read on the
/// shaped path: its acceleration carries `k·r·jerk` of the path while the
/// extruder moves a fraction of its position tolerance, and nothing after
/// it differentiates the fit, so it spends its position budget
/// ([`AccelBudget::PositionEquivalent`]). Where no kernel follows, that fit
/// is the motor's command, which spends it only within the follower's
/// acceleration limit; a kernel after it smooths the advanced signal's
/// acceleration, far past the limit, into the command. Every other
/// follower fit keeps the absolute budget. The plain projection trailing
/// derivative gains differentiate would turn its velocity error `δ'` into
/// `k·δ'` of command position, which the position-equivalent budget does
/// not bound; a kernel's output carries only the path's own acceleration,
/// which the absolute budget resolves in long pieces, and spending its
/// position budget there moves the bead the melt lag lays.
fn source_accel_budget(chain: &CompiledChain) -> AccelBudget {
    if LeadingAdvance::of(chain).is_none() {
        return AccelBudget::Absolute;
    }
    AccelBudget::PositionEquivalent {
        accel_limit_mm_s2: match chain.kernel() {
            Some(_) => f64::INFINITY,
            None => chain.ratio_ramp_rule().max_accel_mm_s2,
        },
    }
}

/// Rebuild every projected-follower track from its leaders' *toolhead*
/// motion: the kernel-convolved signal, before any trailing derivative-gain
/// stage. Trailing gains shape the motor command (a mode-inverse
/// counter-drive), which the physical toolhead — the thing the follower must
/// track — does not perform; the shaper applies them only after projection.
///
/// A smoothing kernel reshapes the commanded path itself, so the shaped
/// leader signal is the trajectory the follower rides. Each segment's
/// extrusion demand is laid out over the shaped arc the toolhead covers
/// during that segment, and the follower extrudes it as the shaped path
/// traverses it: its velocity is `r(s_shaped(t)) · |v_shaped(t)|` and its
/// position the running integral. Demand spans and the traversal odometer
/// measure the same shaped path, so ratio transitions stay glued to the
/// geometry — a travel↔extrude boundary fires where the shaped path crosses
/// the seam (mid corner cut), never displaced by the path length the kernel
/// smoothed away, which would otherwise accumulate without bound and land
/// full-flow steps at cruise speed. The extruded amount tracks the shaped
/// path's true distance — short of the commanded total by the corner-cut
/// length. Extrude-only moves ride no spatial path; their raw track adds in
/// directly.
///
/// The follower's own chain applies on top of the projection in the stage
/// order the leaders use: derivative-gain stages before the kernel act on the
/// projected track, the kernel convolves that signal, and trailing stages act
/// on the convolution. The convolution window reaches past the committed
/// region, so projection runs permanently ahead through `out`'s full frontier
/// (`out.len() >= commit_count` segments), caching one fitted pre-kernel
/// track per raw segment; each is computed exactly once, which keeps every
/// emit reading bit-identical convolution inputs.
#[derive(Default)]
pub(crate) struct ProjectionTiming {
    pub ingest_us: u128,
    pub ingests: usize,
    pub source_fit_us: u128,
    pub source_fits: usize,
    pub breakpoints_us: u128,
    pub kernel_fit_us: u128,
    pub kernel_fits: usize,
    pub kernel_fit_max_us: u128,
}

impl ProjectionTiming {
    pub fn record_stages(&self, projection_us: u128) {
        use crate::timing::{Stage, record_stage};
        record_stage(Stage::FollowerProjection, projection_us);
        record_stage(Stage::FollowerIngest, self.ingest_us);
        record_stage(Stage::FollowerSourceFit, self.source_fit_us);
        record_stage(Stage::FollowerBreakpoints, self.breakpoints_us);
        record_stage(Stage::FollowerKernelFit, self.kernel_fit_us);
    }

    pub fn detail(&self) -> String {
        format!(
            "ingest_us={} ingests={} source_fit_us={} source_fits={} breakpoints_us={} \
             kernel_fit_us={} kernel_fits={} kernel_fit_max_us={}",
            self.ingest_us,
            self.ingests,
            self.source_fit_us,
            self.source_fits,
            self.breakpoints_us,
            self.kernel_fit_us,
            self.kernel_fits,
            self.kernel_fit_max_us
        )
    }
}

struct SourceProjection {
    track: ScalarNurbs,
    s_end: f64,
    e_end_relative: f64,
    semantic_cuts: Vec<f64>,
    input_start: SeamInput,
    input_end: SeamInput,
    /// Advance steps inside the segment, in time order, which `track`
    /// leaves out.
    window_advance_steps: Vec<AdvanceStep>,
    /// The position tolerance scale the segment is fit and welded to
    /// ([`riding_tol_scale`]).
    tol_scale: f64,
    /// Whether a span the source's reach reads retracts
    /// ([`FollowerState::retracts_within`]).
    reach_retracts: bool,
}

/// The position step an advance before projection makes where the follower
/// velocity steps — an extrusion-ratio step taken at speed, at a segment
/// join or where the shaped path crosses a ratio-span boundary inside one.
/// The fitted tracks are continuous, so whatever assembles them puts the
/// step back for a kernel after the advance to smooth.
#[derive(Debug, Clone, Copy)]
struct AdvanceStep {
    t: f64,
    velocity_before: f64,
    velocity_after: f64,
    step_mm: f64,
}

/// Relative follower-velocity mismatch two evaluations of one continuous
/// signal at a seam can show from rounding alone; measured mismatches sit
/// near 1e-12.
const ADVANCE_STEP_VELOCITY_ROUNDING: f64 = 1e-9;

/// Follower velocity below which a stitch across a time gap counts as at
/// rest.
const REST_VELOCITY_MM_S: f64 = 1e-9;

impl AdvanceStep {
    fn is_rounding(&self) -> bool {
        (self.velocity_after - self.velocity_before).abs()
            <= ADVANCE_STEP_VELOCITY_ROUNDING
                * self
                    .velocity_before
                    .abs()
                    .max(self.velocity_after.abs())
                    .max(1.0)
    }

    fn between(stage: &ChainStage, t: f64, velocity_before: f64, velocity_after: f64) -> Self {
        Self {
            t,
            velocity_before,
            velocity_after,
            step_mm: advance_offset(stage, velocity_after) - advance_offset(stage, velocity_before),
        }
    }
}

/// What spares the motor the discontinuities a leading advance makes. Every
/// projected follower ramps its ratio, so the advance reads only the
/// velocity [`Table::Advance`] lays along the leaders' path either way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AdvanceSmoothing {
    /// A ratio ramp ahead of it: the laid ratio is C1, so the advance has no
    /// step to carry and nothing after it would spread one; a step that
    /// would reach the motor is refused. The advance reads only the
    /// extrusion laid along the leaders' path, as Klipper advances only the
    /// extruding moves that travel: a retract or prime the raw track makes
    /// on its own, or a retract the toolhead makes while it moves, pushes no
    /// bead to lead, and advancing either would step its velocity at every
    /// acceleration change of its trapezoid.
    RatioRamp,
    /// A ratio ramp ahead of it and a kernel after: the advance reads the
    /// path's velocity alone, as on a ramp, and the kernel spreads what
    /// steps remain and bounds the extruder's acceleration.
    RatioRampAndKernel,
}

#[derive(Clone, Copy)]
struct LeadingAdvance<'a> {
    stage: &'a ChainStage,
    smoothing: AdvanceSmoothing,
    /// How far a velocity step the advance makes of a leader's acceleration
    /// step may reach the motor of a ratio-ramped follower, see
    /// [`CompiledChain::instantaneous_corner_velocity`].
    instantaneous_corner_velocity: f64,
}

impl<'a> LeadingAdvance<'a> {
    /// The advance a chain applies before its kernel, or with no kernel at
    /// all. A linear one commutes with the kernel, yet applied to the fitted
    /// source it would turn the fit's velocity error `δ'` into `k·δ'` of
    /// command, and the melt lag the advance models hands `δ'` back to the
    /// nozzle flow whole: a bead error of `δ'/(r·v)`, largest where a corner
    /// slows. The source fits the advanced signal instead, whose position
    /// error that lag filters. An acceleration gain, which only a chain
    /// built outside configuration places before a kernel, stays on the
    /// kernel input, where the convolution differentiates it exactly.
    fn of(chain: &'a CompiledChain) -> Option<Self> {
        let acceleration_gain = |stage: &&ChainStage| matches!(stage, ChainStage::DerivativeGains { k2, .. } if *k2 != 0.0);
        let stage = chain
            .leading_transform()
            .filter(|stage| chain.kernel().is_none() || !acceleration_gain(stage))?;
        Some(Self {
            stage,
            smoothing: match chain.kernel() {
                None => AdvanceSmoothing::RatioRamp,
                Some(_) => AdvanceSmoothing::RatioRampAndKernel,
            },
            instantaneous_corner_velocity: chain.instantaneous_corner_velocity(),
        })
    }

    /// The follower velocity, acceleration and jerk the advance reads at
    /// `t`: the extrusion laid along the path, never the extrude-only motion
    /// folded into the follower.
    fn read(&self, sig: &FollowerSignal, t: f64, read: &PathRead) -> (f64, f64, f64) {
        sig.ramped_read(t, read)
    }

    fn read_velocity(&self, sig: &FollowerSignal, t: f64) -> f64 {
        sig.ramped_advance_read(t).0
    }

    fn read_acceleration(&self, sig: &FollowerSignal, t: f64) -> f64 {
        sig.ramped_advance_read(t).1
    }

    fn read_seam_velocity(&self, input: &SeamInput) -> f64 {
        input
            .ramped
            .expect("a ratio-ramped seam records what its advance reads")
            .read_velocity
    }

    /// No kernel follows the advance: a step it makes reaches the motor.
    fn is_ramped(&self) -> bool {
        self.smoothing == AdvanceSmoothing::RatioRamp
    }
}

fn ramped_position_step(axis: usize, step: &AdvanceStep) -> PostProcessError {
    PostProcessError::RatioRampedPositionStep {
        axis,
        t: step.t,
        step_mm: step.step_mm,
        velocity_before: step.velocity_before,
        velocity_after: step.velocity_after,
    }
}

/// One raw segment's follower projection inputs: the shaped leader signal it
/// rides, what its ingest read off those leaders, its own raw track, and
/// where on the shaped path it starts.
#[derive(Clone, Copy)]
struct FollowerSource<'a> {
    shaped: &'a ContinuousSegment,
    raw: &'a ContinuousSegment,
    axis: usize,
    leaders: &'a [usize],
    reads: &'a LeaderReads,
    state: &'a FollowerState,
    s_start: f64,
    s_length: f64,
}

/// Fits one segment's projected follower source. Its pieces meet on the
/// signal's own velocity: the plain source's always, an advanced one's once
/// its in-window advance steps are taken out for the kernel after it to
/// carry, which configuration and the shaper guarantee it has. A
/// ratio-ramped advance has no kernel after it, so it fits only a signal
/// with no such step and no velocity step beyond the fit's resolution.
fn fit_source_projection(
    source: FollowerSource,
    fit_tol: FitTol,
    leading: Option<LeadingAdvance>,
) -> Result<SourceProjection, PostProcessError> {
    let FollowerSource {
        shaped,
        raw,
        axis,
        leaders,
        reads,
        state,
        s_start,
        s_length,
    } = source;
    let raw_axis = &raw.axes[axis];
    let (t_start, t_end) = projection_support(raw, shaped, axis, leaders);
    let sig = FollowerSignal::riding(
        shaped,
        raw,
        axis,
        ShapedLeaders::with_reads(shaped, leaders, reads),
        state,
        Odometer {
            start: s_start,
            length: s_length,
        },
        0.0,
    )
    .with_rests(fit_tol);
    let tol_scale = riding_tol_scale(&raw.followers, axis, sig.ratio_bounds.ratio);
    let tolerance = follower_fit_tol(fit_tol, tol_scale * 0.5);
    let breakpoints = sig.construction_breakpoints(raw_axis, tolerance.accel_mm_s2);
    let input_state = |t| {
        let s = sig.s_owned(t);
        let (ratio, ratio_slope) = sig.ratio_and_slope(s);
        SeamInput {
            velocity: sig.deriv(t),
            ratio,
            leader_speed: sig.shaped_speed(t),
            s,
            ratio_slope,
            ramped: leading.map(|advance| RampedSeam {
                read_velocity: sig.ramped_advance_read(t).0,
                read_slope: sig.ramped_read_slope(s),
                advanced_velocity: AdvancedFollowerSignal::new(&sig, advance, &[], tolerance)
                    .eval_pva(t)
                    .1,
            }),
        }
    };
    let input_start = input_state(t_start);
    let input_end = input_state(t_end);
    let steps_beyond_rounding = match leading {
        Some(advance) => advance_steps_at(advance, &sig, &breakpoints.fit_seeds, (t_start, t_end))
            .filter(|step| !step.is_rounding())
            .collect::<Vec<_>>(),
        None => Vec::new(),
    };
    let unexplained = |steps: &[AdvanceStep]| {
        steps
            .iter()
            .find(|step| !sig.leader_residual_explains(step, (t_start, t_end)))
            .map(|step| ramped_position_step(axis, step))
    };
    if let Some(advance) = leading {
        if let Some(refused) = advance
            .is_ramped()
            .then(|| unexplained(&steps_beyond_rounding))
            .flatten()
        {
            return Err(refused);
        }
    }
    let (track, window_advance_steps) = if let Some(value) = sig.constant_value() {
        let track = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![value, value],
        }]);
        (track, steps_beyond_rounding)
    } else {
        let fit = AxisFit {
            axis,
            t_start,
            t_end,
            seed_breakpoints: &breakpoints.fit_seeds,
            fit_tol: tolerance,
            fit_context: if leading.is_some() {
                "advanced_follower_source"
            } else {
                "follower_source"
            },
            seam_continuity: SeamContinuity::Velocity,
        };
        match leading {
            Some(advance) => {
                let resolution = fit_resolution(
                    &fit,
                    &AdvancedFollowerSignal::new(&sig, advance, &steps_beyond_rounding, tolerance),
                )?;
                let mut seeds = breakpoints.fit_seeds.clone();
                seeds.extend(sig.advance_velocity_steps_on_the_grid(advance, resolution.velocity));
                seeds.sort_by(f64::total_cmp);
                seeds.dedup();
                let mut step_candidates = [seeds.as_slice(), sig.grid.as_slice()].concat();
                step_candidates.sort_by(f64::total_cmp);
                step_candidates.dedup();
                let steps = advance_steps_at(advance, &sig, &step_candidates, (t_start, t_end))
                    .filter(|step| {
                        !step.is_rounding()
                            || (!advance.is_ramped() && step.step_mm.abs() > resolution.position)
                    })
                    .collect::<Vec<_>>();
                let advanced = AdvancedFollowerSignal::new(&sig, advance, &steps, tolerance);
                if advance.is_ramped() {
                    advanced.refuse_onset_steps(axis, (t_start, t_end))?;
                    if let Some(refused) = unexplained(&steps) {
                        return Err(refused);
                    }
                    advanced.refuse_velocity_steps(
                        axis,
                        &step_candidates,
                        (t_start, t_end),
                        resolution.velocity
                            + UnshapedLeaderStep::of(
                                advance,
                                sig.ratio_bounds.ratio,
                                fit_tol.accel_mm_s2,
                            )
                            .allowance(),
                    )?;
                }
                let track = fit_axis_from_signal(
                    AxisFit {
                        seed_breakpoints: &seeds,
                        ..fit
                    },
                    &advanced,
                )?;
                (track, steps)
            }
            None => (fit_axis_from_signal(fit, &sig)?, steps_beyond_rounding),
        }
    };
    let e_end_relative = sig.eval(t_end);
    Ok(SourceProjection {
        track,
        s_end: sig.s_end(),
        e_end_relative,
        semantic_cuts: breakpoints.semantic,
        input_start,
        input_end,
        window_advance_steps,
        tol_scale,
        reach_retracts: sig.reach_retracts,
    })
}

/// Every follower velocity discontinuity inside a segment is a fit seed by
/// construction (ratio-span boundaries are bisected onto the exact time the
/// odometer crosses them; raw breaks come from the raw axis knots), and an
/// advance turns each into a position step. So do the leaders' own knots,
/// where their stored pieces meet with a velocity mismatch inversely
/// proportional to their duration: rounding for the follower's velocity,
/// yet through the advance a position step that spans down to the fit floor
/// cannot absorb.
fn advance_steps_at<'a>(
    advance: LeadingAdvance<'a>,
    sig: &'a FollowerSignal,
    times: &'a [f64],
    (t_start, t_end): (f64, f64),
) -> impl Iterator<Item = AdvanceStep> + 'a {
    times
        .iter()
        .copied()
        .filter(move |&t| t > t_start && t < t_end)
        .map(move |t| {
            AdvanceStep::between(
                advance.stage,
                t,
                advance.read_velocity(sig, trajectory::next_toward(t, t_start)),
                advance.read_velocity(sig, trajectory::next_toward(t, t_end)),
            )
        })
}

/// One emit window's projection inputs: the raw segments, the shaped leader
/// frontier they ride, the chains and tolerance the followers are fitted
/// with, and the shaper's worker pool the fits fan out on.
#[derive(Clone, Copy)]
pub(crate) struct ProjectionWindow<'a> {
    pub base: &'a [ContinuousSegment],
    pub frontier: &'a [ContinuousSegment],
    pub chains: &'a AxisChainSet,
    pub fit_tol: FitTol,
    pub lookahead: Lookahead,
    pub pool: &'a WorkerPool,
}

/// Queues a source fit for every segment the ratio smoothing now holds
/// final. Taken after each leg the smoothing lays, what is taken — and so
/// how far back a valley laid later may reach — follows the legs alone,
/// never how many of them one emit window brings.
fn take_final_sources(
    state: &mut FollowerState,
    (base, frontier): (&[ContinuousSegment], &[ContinuousSegment]),
    axis: usize,
    source_jobs: &mut Vec<(usize, ratio_ramp::PendingSegment)>,
) {
    for pending in state.smoothing.take_final(&state.spans) {
        let index = base[..frontier.len()]
            .partition_point(|raw| raw.t_end < pending.t_end - GRID_DEDUP_EPS_S);
        assert!(
            index < frontier.len() && (base[index].t_end - pending.t_end).abs() <= GRID_DEDUP_EPS_S,
            "follower axis {axis}: the segment ending at {} waited on its ratio smoothing past \
             the shaped frontier",
            pending.t_end
        );
        source_jobs.push((index, pending));
    }
}

/// Projects every follower over the frontier, then commits the segments of
/// `out` every follower holds final: a ratio-ramped follower holds back a
/// segment until no width node or seam still to come reaches into it, and
/// a kernel after the ramp until its window finds every segment it reads
/// projected (see [`final_ramped_commits`]). `out` is cut to the committed
/// prefix; a forced lookahead closes every ramp, so it commits all of it.
pub(crate) fn project_followers(
    window: ProjectionWindow,
    out: &mut Vec<ContinuousSegment>,
    states: &mut Vec<FollowerState>,
    timing: &mut ProjectionTiming,
) -> Result<(), PostProcessError> {
    let ProjectionWindow {
        base,
        frontier,
        chains,
        fit_tol,
        lookahead,
        pool,
    } = window;
    let requested = out.len();
    assert!(frontier.len() >= requested);
    if states.len() < chains.n_axes() {
        states.resize_with(chains.n_axes(), FollowerState::default);
    }
    let mut commit_count = requested;
    let mut committing = Vec::new();
    for (axis, leaders) in chains.projected_followers() {
        let chain = &chains.chains[axis];
        let kernel = chain.kernel();
        let leading = LeadingAdvance::of(chain);
        let source_tol = FitTol {
            accel_budget: source_accel_budget(chain),
            ..fit_tol
        };
        let rule = chain.ratio_ramp_rule();
        let state = &mut states[axis];
        let mut source_jobs = Vec::new();
        {
            state.start_projecting();
            let ingest_started = crate::timing::stopwatch();
            let first_new = state.ingested_through_t.map_or(0, |through| {
                base[..frontier.len()]
                    .partition_point(|raw| raw.t_end <= through + GRID_DEDUP_EPS_S)
            });
            let reads = ingest_reads(
                &base[first_new..frontier.len()],
                &frontier[first_new..],
                leaders,
                pool,
            );
            timing.ingests += reads.len();
            for (offset, read) in reads.into_iter().enumerate() {
                let i = first_new + offset;
                let table_start = state.s_ingested_end;
                let shaped_ds =
                    state.ingest(&base[i], &frontier[i], axis, leaders, &base[i + 1..], &read);
                let s_start = state.aligned_span_start(state.s_queued_end);
                let leg = RampLeg {
                    shaped: &frontier[i],
                    leaders,
                    shaped_leaders: &read.shaped_leaders,
                    support: projection_support(&base[i], &frontier[i], axis, leaders),
                    start_s: table_start,
                    rests: !read.raw_moves,
                };
                state
                    .smoothing
                    .lay((&state.spans, &state.advance_spans), &leg, &rule);
                state.smoothing.queue(
                    base[i].t_end,
                    (s_start, shaped_ds),
                    state.s_ingested_end,
                    read.shaped_leaders.into_reads(read.odometer),
                );
                state.s_queued_end = s_start + shaped_ds;
                take_final_sources(state, (base, frontier), axis, &mut source_jobs);
            }
            if lookahead.clamps_past_signal() {
                state.smoothing.close_at(state.s_ingested_end);
                take_final_sources(state, (base, frontier), axis, &mut source_jobs);
            }
            timing.ingest_us += ingest_started.elapsed_us();
        }
        let source_started = crate::timing::stopwatch();
        timing.source_fits += source_jobs.len();
        let workers = if cfg!(target_arch = "wasm32") {
            1
        } else {
            std::thread::available_parallelism().map_or(1, |cores| {
                cores.get().saturating_sub(1).max(1).min(source_jobs.len())
            })
        };
        let state_ref: &FollowerState = &*state;
        let source_fits = pool
            .map_indexed(
                workers,
                source_jobs.len(),
                || (),
                |(), job| {
                    let (index, pending) = &source_jobs[job];
                    (
                        *index,
                        fit_source_projection(
                            FollowerSource {
                                shaped: &frontier[*index],
                                raw: &base[*index],
                                axis,
                                leaders,
                                reads: &pending.reads,
                                state: state_ref,
                                s_start: pending.s_start,
                                s_length: pending.s_length,
                            },
                            source_tol,
                            leading,
                        ),
                    )
                },
            )
            .expect("follower source fit thread panicked");
        timing.source_fit_us += source_started.elapsed_us();
        let mut source_fits = source_fits.into_iter().peekable();
        for i in 0..frontier.len() {
            let raw = &base[i];
            if axis >= raw.axes.len() {
                return Err(PostProcessError::AxisCountMismatch {
                    expected: chains.n_axes(),
                    got: raw.axes.len(),
                });
            }
            if state
                .projected_through_t
                .is_some_and(|through| raw.t_end <= through + GRID_DEDUP_EPS_S)
            {
                continue;
            }
            if let Some(through) = state.projected_through_t {
                assert!(
                    raw.t_start >= through - GRID_DEDUP_EPS_S,
                    "follower projection saw an out-of-order segment: t_start {} \
                     before projected-through {through}",
                    raw.t_start,
                );
            }
            let awaiting_ramps = source_fits.peek().is_none_or(|(index, _)| *index != i);
            if awaiting_ramps {
                break;
            }
            let raw_axis = &raw.axes[axis];
            let (t_start, t_end) = projection_support(raw, &frontier[i], axis, leaders);
            assert!(
                t_start < t_end,
                "follower axis {axis} has no support in segment"
            );
            let base_position = state.e_end.unwrap_or_else(|| axis_pva(raw_axis, t_start).0);
            let (source_index, source_result) = source_fits
                .next()
                .expect("every unprojected follower segment has a source fit");
            assert_eq!(
                source_index, i,
                "follower source fit index {source_index} does not match segment {i}"
            );
            let SourceProjection {
                track,
                s_end,
                e_end_relative,
                semantic_cuts,
                input_start,
                input_end,
                window_advance_steps,
                tol_scale,
                reach_retracts,
            } = source_result?;
            state.s_shaped = s_end;
            state.e_end = Some(base_position + e_end_relative);
            if !track.control_points().iter().all(|v| v.is_finite()) {
                return Err(PostProcessError::NonFiniteSample {
                    axis,
                    t: raw.t_start,
                });
            }
            let join_advance_step = match leading {
                Some(advance) => {
                    let step = join_advance_step(
                        advance,
                        axis,
                        state.projected_input_end,
                        (t_start, input_start),
                    );
                    if advance.is_ramped() {
                        let unexplained = step.filter(|step| {
                            state.projected_input_end.is_none_or(|(_, before)| {
                                !before.explains_ramped_read_step(&input_start, step)
                            })
                        });
                        if let Some(step) = unexplained {
                            return Err(ramped_position_step(axis, &step));
                        }
                        refuse_ramped_join_velocity_step(
                            advance,
                            axis,
                            fit_tol.accel_mm_s2,
                            state.projected_input_end,
                            (t_start, input_start),
                        )?;
                    }
                    step.map_or(0.0, |step| step.step_mm)
                }
                None => 0.0,
            };
            let track_start = nurbs::eval::eval(&track, t_start);
            let output_base = state.projected_output_end.unwrap_or(base_position)
                + join_advance_step
                - track_start;
            let window_steps_mm = window_advance_steps
                .iter()
                .map(|step| step.step_mm)
                .sum::<f64>();
            state.projected_output_end =
                Some(output_base + nurbs::eval::eval(&track, t_end) + window_steps_mm);
            state.projected_input_end = Some((t_end, input_end));
            state.projected_through_t = Some(t_end);
            state.projected.push(ProjSeg {
                t_start,
                t_end,
                base: output_base,
                input_start,
                input_end,
                track,
                semantic_cuts,
                join_advance_step,
                window_advance_steps,
                tol_scale,
                reach_retracts,
                extrude_only: if raw.spatial_path {
                    Vec::new()
                } else {
                    extrude_only::extrude_only_pieces(axis, raw_axis, (t_start, t_end))
                },
            });
        }
        assert!(
            source_fits.next().is_none(),
            "follower source fit has no matching projection segment"
        );
        let kernel_lo = match kernel {
            Some(kernel) if !lookahead.clamps_past_signal() => kernel.support().0,
            _ => 0.0,
        };
        commit_count = commit_count.min(final_ramped_commits(
            state,
            (base, frontier),
            commit_count,
            kernel_lo,
            (axis, leaders),
        ));
        committing.push((axis, leaders));
    }
    assert!(
        commit_count == requested || !lookahead.clamps_past_signal(),
        "a forced follower projection left {} of {requested} segments short of final",
        requested - commit_count
    );
    if !lookahead.clamps_past_signal() {
        commit_count = crate::shaper::whole_groups(base, commit_count);
    }
    out.truncate(commit_count);
    for (axis, leaders) in committing {
        let chain = &chains.chains[axis];
        let kernel = chain.kernel();
        let defer_linear_prefix =
            LeadingAdvance::of(chain).is_none() && chain.follower_linear_transform();
        let state = &mut states[axis];
        let kernel_tracks = match kernel {
            Some(kernel) if commit_count > 0 => {
                let targets = (0..commit_count)
                    .map(|i| {
                        let support = projection_support(&base[i], &frontier[i], axis, leaders);
                        KernelTarget {
                            support,
                            tol_scale: state.cached_projection(support.0, support.1).tol_scale,
                        }
                    })
                    .collect::<Vec<_>>();
                let fit = KernelFit {
                    axis,
                    chain,
                    kernel,
                    defer_linear_prefix,
                    fit_tol,
                    lookahead,
                };
                let kernel_started = crate::timing::stopwatch();
                timing.kernel_fits += targets.len();
                let groups = crate::shaper::fit_groups(
                    &base[..commit_count],
                    lookahead.clamps_past_signal(),
                );
                let workers = if cfg!(target_arch = "wasm32") {
                    1
                } else {
                    std::thread::available_parallelism().map_or(1, |cores| {
                        cores.get().saturating_sub(1).max(1).min(groups.len())
                    })
                };
                let state_ref: &FollowerState = &*state;
                let fitted = pool
                    .map_indexed(
                        workers,
                        groups.len(),
                        || (),
                        |(), index| fit.group_alone(state_ref, &targets[groups[index].clone()]),
                    )
                    .expect("follower kernel fit thread panicked");
                let mut tracks = Vec::with_capacity(targets.len());
                for result in fitted {
                    let (group_tracks, window) = result?;
                    timing.kernel_fit_max_us = timing.kernel_fit_max_us.max(window.fit_us);
                    timing.breakpoints_us += window.breakpoints_us;
                    tracks.extend(group_tracks);
                }
                timing.kernel_fit_us += kernel_started.elapsed_us();
                Some(tracks)
            }
            _ => None,
        };
        let mut emitted_pv = state.emitted_output_pv;
        let mut committed_input_end = state.committed_input_end;
        let support = |i: usize| projection_support(&base[i], &frontier[i], axis, leaders);
        let mut group_weld: Option<SeamWeld> = None;
        for group in
            crate::shaper::fit_groups(&base[..commit_count], lookahead.clamps_past_signal())
        {
            for i in group.clone() {
                let (t_start, t_end) = support(i);
                let cached = state.cached_projection(t_start, t_end);
                let (input_start, input_end) = (cached.input_start, cached.input_end);
                let (base_position, mut pieces) = kernel_tracks.as_ref().map_or_else(
                    || (cached.base, extract_bezier_pieces(&cached.track)),
                    |tracks| tracks[i].clone(),
                );
                let law_step = |previous_input: Option<SeamInput>| {
                    if kernel.is_some() {
                        return 0.0;
                    }
                    previous_input
                        .expect("an emitted endpoint always records its pre-transform state")
                        .emitted_velocity_step(&input_start)
                };
                if i == group.start {
                    group_weld = None;
                    if let Some(previous) = emitted_pv {
                        let group_tol_scale = group
                            .clone()
                            .map(|j| {
                                let (start, end) = support(j);
                                state.cached_projection(start, end).tol_scale
                            })
                            .fold(f64::INFINITY, f64::min);
                        group_weld = Some(SeamWeld::spanning(
                            WeldWindow {
                                axis,
                                start: t_start,
                                end: support(group.end - 1).1,
                                tol: follower_fit_tol(fit_tol, group_tol_scale),
                            },
                            previous,
                            law_step(committed_input_end),
                            base_position,
                            &pieces,
                        )?);
                    }
                }
                if let Some(weld) = group_weld {
                    pieces = weld.apply(pieces);
                }
                if i > group.start {
                    let previous = emitted_pv.expect("an earlier target of the group emitted");
                    let weld = SeamWeld::spanning(
                        WeldWindow {
                            axis,
                            start: t_start,
                            end: t_end,
                            tol: follower_fit_tol(fit_tol, cached.tol_scale),
                        },
                        previous,
                        law_step(committed_input_end),
                        base_position,
                        &pieces,
                    )?;
                    pieces = weld.apply(pieces);
                }
                committed_input_end = Some(input_end);
                let (run_end, velocity_end) = piece_run_end_pv(&pieces);
                emitted_pv = Some((base_position + run_end, velocity_end));
                Arc::make_mut(&mut out[i].axes)[axis] = ContinuousAxis::PiecewiseRelativeSpline(
                    localize_pieces(base_position, &pieces),
                );
            }
        }
        state.emitted_output_pv = emitted_pv;
        state.committed_input_end = committed_input_end;
        if commit_count > 0 {
            let emitted_through = base[commit_count - 1].t_end;
            let back = kernel.map_or(0.0, |k| k.support().1.max(0.0));
            state.trim_projected(emitted_through - back);
        }
        state.prune_spans();
    }
    Ok(())
}

/// How many of the first `requested` segments a ratio-ramped follower can
/// commit: each one's projection is laid, so no width node or seam still to
/// come reaches into it, and a kernel after the ramp, reaching `-kernel_lo`
/// past a segment's end (zero where the window clamps past the signal),
/// finds that window projected too.
fn final_ramped_commits(
    state: &FollowerState,
    (base, frontier): (&[ContinuousSegment], &[ContinuousSegment]),
    requested: usize,
    kernel_lo: f64,
    (axis, leaders): (usize, &[usize]),
) -> usize {
    let Some(through) = state.projected_through_t else {
        return 0;
    };
    (0..requested)
        .take_while(|&i| {
            projection_support(&base[i], &frontier[i], axis, leaders).1 - kernel_lo <= through
        })
        .count()
}

/// Whether a fitted follower output must be corrected back to monotone: a
/// nonnegative extrusion demand may never emit a retraction, but a track the
/// derivative-gain stages have already shaped legitimately reverses.
#[derive(Clone, Copy, PartialEq, Eq)]
enum OutputMonotonicity {
    Enforced,
    Free,
}

/// One committed segment's follower-kernel fit target: the window the
/// output covers and the tolerance scale of its own extrusion.
#[derive(Clone, Copy)]
struct KernelTarget {
    support: (f64, f64),
    tol_scale: f64,
}

struct KernelWindowTiming {
    fit_us: u128,
    breakpoints_us: u128,
}

/// What every target's follower-kernel fit shares within one emit.
struct KernelFit<'a> {
    axis: usize,
    chain: &'a CompiledChain,
    kernel: &'a nurbs::algebra::PiecewisePolynomialKernel,
    defer_linear_prefix: bool,
    fit_tol: FitTol,
    lookahead: Lookahead,
}

impl KernelFit<'_> {
    /// The kernel output over each target of one fit group from the
    /// projected segments the group's window reads alone, assembled from the
    /// first of them and offset at the group's start, so the fits are the
    /// same whichever other segments share its emit or are still cached
    /// around it. Returns each target's pieces with the absolute base they
    /// sit on.
    fn group_alone(
        &self,
        state: &FollowerState,
        group: &[KernelTarget],
    ) -> Result<(Vec<(f64, Vec<BezierPiece>)>, KernelWindowTiming), PostProcessError> {
        let Self {
            axis,
            chain,
            kernel,
            defer_linear_prefix,
            fit_tol,
            lookahead,
        } = *self;
        let (target_start, target_end) = (
            group.first().expect("a fit group holds targets").support.0,
            group.last().expect("a fit group holds targets").support.1,
        );
        let (k_lo, k_hi) = kernel.support();
        let (read_lo, read_hi) = (target_start - k_hi, target_end - k_lo);
        let projected = &state.projected;
        let lo = projected
            .partition_point(|segment| segment.t_start < read_lo)
            .saturating_sub(1);
        let hi = (projected.partition_point(|segment| segment.t_end <= read_hi) + 1)
            .min(projected.len());
        let window = &projected[lo..hi];
        let first = window.first().expect("cache covers commits");
        let last = window.last().expect("cache covers commits");
        let (first_t, last_t) = (first.t_start, last.t_end);
        if read_lo < first_t && state.projected_trimmed {
            return Err(PostProcessError::MissingHistory { axis, t: read_lo });
        }
        if read_hi > last_t && !lookahead.clamps_past_signal() {
            return Err(PostProcessError::MissingLookahead { axis, t: read_hi });
        }
        let mut input_pieces = Vec::new();
        let mut semantic_cuts = Vec::new();
        let mut input_steps = Vec::new();
        let mut carried = 0.0;
        let mut input_end = None;
        for segment in window {
            if let Some(previous_end) = input_end {
                assert!(
                    segment.t_start >= previous_end,
                    "projected follower segments overlap"
                );
                if segment.t_start > previous_end {
                    input_pieces.push(BezierPiece {
                        u_start: previous_end,
                        u_end: segment.t_start,
                        coeffs: vec![carried],
                    });
                }
            }
            let pieces = extract_bezier_pieces(&segment.track);
            let head = pieces.first().expect("a projected track has pieces").coeffs[0];
            let segment_offset = input_end.map_or(0.0, |_| carried - head);
            let join_step = input_end.map_or(0.0, |_| segment.join_advance_step);
            if join_step != 0.0 {
                input_steps.push((segment.t_start, join_step));
            }
            let pieces = with_window_steps(
                pieces,
                &segment.window_advance_steps,
                segment_offset + join_step,
                &mut input_steps,
            );
            let tail = pieces.last().expect("a projected track has pieces");
            carried = polynomial_pva(&tail.coeffs, tail.u_end - tail.u_start).0;
            input_pieces.extend(pieces);
            semantic_cuts.extend_from_slice(&segment.semantic_cuts);
            semantic_cuts.extend([segment.t_start, segment.t_end]);
            input_end = Some(segment.t_end);
        }
        semantic_cuts.sort_by(f64::total_cmp);
        semantic_cuts.dedup();
        for piece in &mut input_pieces {
            piece.coeffs.resize(piece.coeffs.len().max(6), 0.0);
        }
        let nonnegative_demand = !window.iter().any(|segment| segment.reach_retracts);
        let group_tol = follower_fit_tol(
            fit_tol,
            group
                .iter()
                .map(|target| target.tol_scale)
                .fold(f64::INFINITY, f64::min),
        );
        assert_piece_seams(axis, "projected input", &input_pieces, &input_steps);
        if nonnegative_demand {
            project_monotone(axis, "projected input", &mut input_pieces, group_tol);
        }
        let unified_input_degree = input_pieces
            .iter()
            .map(|piece| piece.degree())
            .max()
            .expect("projected follower input is empty");
        for piece in &mut input_pieces {
            piece.coeffs.resize(unified_input_degree + 1, 0.0);
        }
        let at_target = &input_pieces[input_pieces
            .partition_point(|piece| piece.u_end <= target_start)
            .min(input_pieces.len() - 1)];
        let input_offset = polynomial_pva(&at_target.coeffs, target_start - at_target.u_start).0;
        for piece in &mut input_pieces {
            piece.coeffs[0] -= input_offset;
        }
        let window_base = first.base + input_offset * unit_kernel_mass(kernel);
        let mut gained_input = chain.leading_transform().is_some();
        if defer_linear_prefix {
            if let Some(ChainStage::DerivativeGains { k1, k2 }) = chain.transform() {
                input_pieces = extrude_only::without_extrude_only_gain(
                    extract_bezier_pieces(&apply_derivative_gains_to_track(
                        &nurbs::bezier::bezier_pieces_to_discontinuous_nurbs(&input_pieces),
                        *k1,
                        *k2,
                    )),
                    window.iter().flat_map(|segment| &segment.extrude_only),
                    (*k1, *k2),
                );
                gained_input = true;
            }
        }
        let gained_pieces = input_pieces;
        assert!(
            gained_pieces.iter().all(|piece| {
                [0.0, piece.u_end - piece.u_start]
                    .into_iter()
                    .flat_map(|tau| {
                        let pva = polynomial_pva(&piece.coeffs, tau);
                        [pva.0, pva.1, pva.2]
                    })
                    .all(f64::is_finite)
            }),
            "follower axis {axis} gained input has non-finite one-sided P/V/A"
        );
        let kernel_degree = kernel
            .pieces
            .iter()
            .map(|piece| piece.degree())
            .max()
            .expect("shaper kernel has no pieces");
        let table = AxisSignalTable::from_pieces(
            &gained_pieces,
            SignalBounds {
                first_t,
                last_t,
                boundary: if read_lo < first_t {
                    FitBoundary::StreamBoundary
                } else {
                    FitBoundary::Interior
                },
                lookahead: if read_hi > last_t {
                    lookahead
                } else {
                    Lookahead::Required
                },
            },
        )
        .with_piece_moments(kernel_degree);
        let input_degree = table.max_degree();
        let exact_input_breaks = gained_pieces
            .iter()
            .flat_map(|piece| [piece.u_start, piece.u_end])
            .collect::<Vec<_>>();
        let shaped_break_seeds = if gained_input {
            &exact_input_breaks
        } else {
            &semantic_cuts
        };
        let breaks_started = crate::timing::stopwatch();
        let shaped_breaks = ShapedBreaks::new(kernel, shaped_break_seeds);
        let breakpoints_us = breaks_started.elapsed_us();
        let input_jumps = table.input_jumps().to_vec();
        let exact_kernel = ExactKernel::new(kernel);
        let table = Arc::new(table);
        let eval_table = Arc::clone(&table);
        let moment_table = Arc::clone(&table);
        let piece_hint = std::cell::Cell::new(0);
        let sig = ConvolutionSignal::new(
            ShapedSignal::new_from_polynomial_evaluator(
                kernel,
                move |t| eval_table.eval_hinted(t, &piece_hint),
                exact_input_breaks,
                input_degree,
                move |lo, hi, degree, origin, orders| {
                    moment_table.integrate_moments(lo, hi, degree, origin, orders)
                },
                input_jumps,
            ),
            ExactConvolution {
                table: &table,
                kernel: &exact_kernel,
                impulses: &[],
                position_offset: 0.0,
            },
        );
        let output_monotonicity = if nonnegative_demand && !gained_input {
            OutputMonotonicity::Enforced
        } else {
            OutputMonotonicity::Free
        };
        let mut fitted: Vec<(f64, Vec<BezierPiece>)> = Vec::with_capacity(group.len());
        let mut fit_us = 0;
        for target in group {
            let target_tol = follower_fit_tol(fit_tol, target.tol_scale);
            let seeds = shaped_breaks.select_seeds(
                fit_seed_times(
                    axis,
                    target.support.0,
                    target.support.1,
                    shaped_breaks.times(),
                )?,
                Some(&table),
                target_tol,
                CruiseEnds::Anchored,
            );
            let (local_base, pieces, target_us) = fit_kernel_window(
                axis,
                target.support.0,
                target.support.1,
                &seeds,
                &sig,
                target_tol,
                output_monotonicity,
            )?;
            fit_us += target_us;
            let base_position = match fitted.last() {
                Some((previous_base, previous)) => {
                    previous_base + piece_run_end_pv(previous).0 - piece_run_start_pv(&pieces).0
                }
                None => window_base + local_base - piece_run_start_pv(&pieces).0,
            };
            fitted.push((base_position, pieces));
        }
        Ok((
            fitted,
            KernelWindowTiming {
                fit_us,
                breakpoints_us,
            },
        ))
    }
}

fn fit_kernel_window<S: TrackSignal>(
    axis: usize,
    start: f64,
    end: f64,
    shaped_breaks: &[f64],
    sig: &S,
    target_tol: FitTol,
    monotonicity: OutputMonotonicity,
) -> Result<(f64, Vec<BezierPiece>, u128), PostProcessError> {
    let started = crate::timing::stopwatch();
    let local_base = TrackSignal::eval(sig, start);
    let local = ShiftedTrackSignal::new(sig, local_base);
    let track = fit_axis_from_signal(
        AxisFit {
            axis,
            t_start: start,
            t_end: end,
            seed_breakpoints: shaped_breaks,
            fit_tol: target_tol,
            fit_context: "follower_kernel",
            seam_continuity: SeamContinuity::Velocity,
        },
        &local,
    )?;
    let mut pieces = extract_bezier_pieces(&track);
    if monotonicity == OutputMonotonicity::Enforced {
        project_monotone(axis, "shaped output", &mut pieces, target_tol);
    }
    Ok((local_base, pieces, started.elapsed_us()))
}

/// A follower's pre-transform state at one side of a seam. Its velocity is
/// `ratio · leader_speed` plus whatever the raw track adds on its own, so a
/// velocity difference between two sides splits into the follower's own
/// step (a ratio change or a raw step, which the seam must carry) and the
/// part the leaders' speed residual explains (which a weld absorbs). Each
/// side reads the ratio at its own odometer position `s`, with the ratio's
/// path slope there.
#[derive(Debug, Clone, Copy)]
struct SeamInput {
    velocity: f64,
    ratio: f64,
    leader_speed: f64,
    s: f64,
    ratio_slope: f64,
    ramped: Option<RampedSeam>,
}

/// What a ratio-ramped advance reads at a seam, its path slope, and the
/// velocity the advance makes of it, which no kernel smooths after it.
#[derive(Debug, Clone, Copy)]
struct RampedSeam {
    read_velocity: f64,
    read_slope: f64,
    advanced_velocity: f64,
}

impl SeamInput {
    fn signal_velocity_step(&self, right: &Self) -> f64 {
        right.velocity - self.velocity - right.ratio * (right.leader_speed - self.leader_speed)
    }

    /// How far a velocity laid as `slope_of` along the path differs across
    /// the seam because the two segments read it at odometer positions the
    /// span alignment lets sit apart.
    fn misalignment(&self, right: &Self, slope_of: impl Fn(&Self) -> f64) -> f64 {
        (right.s - self.s).abs()
            * slope_of(self).abs().max(slope_of(right).abs())
            * self.leader_speed.abs().max(right.leader_speed.abs())
    }

    /// Whether a ratio-ramped advance's read velocity steps across the join
    /// to `right` by no more than the odometer alignment and the leaders'
    /// speed residual there explain: a position step the stitch carries as
    /// it carries a kernel chain's, far under any the guard exists for.
    fn explains_ramped_read_step(&self, right: &Self, step: &AdvanceStep) -> bool {
        let read_slope = |side: &Self| side.ramped.map_or(0.0, |ramped| ramped.read_slope);
        let explained = self.misalignment(right, read_slope)
            + self.ratio.abs().max(right.ratio.abs())
                * (right.leader_speed - self.leader_speed).abs();
        (step.velocity_after - step.velocity_before).abs()
            <= explained
                + ADVANCE_STEP_VELOCITY_ROUNDING
                    * step
                        .velocity_before
                        .abs()
                        .max(step.velocity_after.abs())
                        .max(1.0)
    }

    /// The velocity step a kernel-free emission carries at the seam as a
    /// C0 corner rather than weld: the follower's own step, and on a
    /// ratio-ramped advance also what the leaders' acceleration rounding
    /// makes of the advanced velocity there — a weld spreading it over a
    /// segment shorter than a millisecond would exceed the acceleration
    /// budget, and [`refuse_ramped_join_velocity_step`] has bounded it.
    fn emitted_velocity_step(&self, right: &Self) -> f64 {
        match (self.ramped, right.ramped) {
            (Some(left_ramped), Some(right_ramped)) => {
                right_ramped.advanced_velocity
                    - left_ramped.advanced_velocity
                    - right.ratio * (right.leader_speed - self.leader_speed)
            }
            _ => self.signal_velocity_step(right),
        }
    }
}

/// Segment tracks are stitched end to start, which is exact only where the
/// commanded signal is position-continuous. An advance applied before
/// projection turns the follower's velocity difference at a join into a
/// position step: each segment's advanced track gains `advance(v)` between
/// its own end states, so the totals telescope to the unadvanced extrusion
/// only if the stitch carries every join's difference — the follower's own
/// step (an extrusion-ratio change at speed) and the leaders' fit residual
/// alike. The output base takes it and a kernel after the advance smooths
/// it. The velocity difference itself is the seam's business: the emission
/// carries the follower's own step as a C0 corner and welds the residual
/// ([`SeamInput::signal_velocity_step`]), neither of which moves the total.
/// A time gap between the segments (a dwell after a drain) is legal only at
/// rest, where there is nothing to carry.
fn join_advance_step(
    advance: LeadingAdvance,
    axis: usize,
    previous_end: Option<(f64, SeamInput)>,
    (t_start, after): (f64, SeamInput),
) -> Option<AdvanceStep> {
    let (previous_t_end, before) = previous_end?;
    if (previous_t_end - t_start).abs() > SEGMENT_TIME_EPS_S {
        assert!(
            before.velocity.abs() <= REST_VELOCITY_MM_S
                && after.velocity.abs() <= REST_VELOCITY_MM_S,
            "follower axis {axis} stitches a segment starting at {t_start} onto one ending at \
             {previous_t_end} while moving ({} -> {} mm/s); a time gap between follower \
             segments is legal only at rest",
            before.velocity,
            after.velocity
        );
        return None;
    }
    Some(AdvanceStep::between(
        advance.stage,
        t_start,
        advance.read_seam_velocity(&before),
        advance.read_seam_velocity(&after),
    ))
    .filter(|step| !step.is_rounding())
}

/// A kernel-free emission carries a follower's own velocity step at a join
/// as a C0 corner ([`SeamInput::signal_velocity_step`]) and welds what the
/// advanced velocity differs by on top, straight to the motor. A ratio ramp
/// leaves the projection no step of its own, and the advance none beyond
/// its allowance for the leaders' acceleration steps
/// ([`UnshapedLeaderStep`]), so a larger one on a
/// ramped follower is refused.
fn refuse_ramped_join_velocity_step(
    advance: LeadingAdvance,
    axis: usize,
    leader_accel_tol: f64,
    previous_end: Option<(f64, SeamInput)>,
    (t_start, after): (f64, SeamInput),
) -> Result<(), PostProcessError> {
    let Some((previous_t_end, before)) = previous_end else {
        return Ok(());
    };
    if (previous_t_end - t_start).abs() > SEGMENT_TIME_EPS_S {
        return Ok(());
    }
    let own_step = before.signal_velocity_step(&after);
    let rounding = ADVANCE_STEP_VELOCITY_ROUNDING
        * before.velocity.abs().max(after.velocity.abs()).max(1.0)
        + before.misalignment(&after, |side| side.ratio_slope);
    if own_step.abs() > rounding {
        return Err(PostProcessError::RatioRampedVelocityStep {
            axis,
            t: t_start,
            velocity_before: before.velocity,
            velocity_after: before.velocity + own_step,
            resolution: rounding,
        });
    }
    let (Some(before_ramped), Some(after_ramped)) = (before.ramped, after.ramped) else {
        return Ok(());
    };
    let (advanced_before, advanced_after) = (
        before_ramped.advanced_velocity,
        after_ramped.advanced_velocity,
    );
    let allowance = UnshapedLeaderStep::of(
        advance,
        before.ratio.abs().max(after.ratio.abs()),
        leader_accel_tol,
    )
    .allowance();
    if (advanced_after - advanced_before).abs() > rounding + allowance {
        return Err(PostProcessError::RatioRampedVelocityStep {
            axis,
            t: t_start,
            velocity_before: advanced_before,
            velocity_after: advanced_after,
            resolution: rounding + allowance,
        });
    }
    Ok(())
}

/// The velocity change an advance may make of what an unshaped leader's
/// acceleration does faster than a fit resolves — a step, or the square
/// root it leaves a curvature cap by: the follower's instantaneous corner
/// velocity, which the planner keeps an unshaped leader's steps within,
/// plus what the leaders' own rounding makes. A shaped leader is fitted
/// within `leader_accel_tol` on either side of its knots, so its
/// acceleration may step by twice that there, which the projection scales
/// by the ratio and the advance by its slope.
///
/// At a square-root onset the exemption covers the sliver where the
/// leader's singular jerk `J/√τ`, through the advance's gain `G`, alone
/// passes the follower's acceleration tolerance: `τ < (G·J/accel_tol)²`
/// ([`enclosure::OnsetSliver`]). There the acceleration is not bounded, the
/// fit's velocity holds the allowance, and the singular term's change
/// across the sliver, `2·G·J·√τ`, must stay within it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct UnshapedLeaderStep {
    allowance: f64,
}

impl UnshapedLeaderStep {
    fn of(advance: LeadingAdvance, ratio_bound: f64, leader_accel_tol: f64) -> Self {
        Self {
            allowance: advance.instantaneous_corner_velocity
                + 2.0 * leader_accel_tol * ratio_bound * steepest_advance_slope(advance.stage),
        }
    }

    #[cfg(test)]
    pub(crate) const fn with_allowance(allowance: f64) -> Self {
        Self { allowance }
    }

    pub(crate) fn allowance(self) -> f64 {
        self.allowance
    }

    /// `accuracy` for an enclosure piece under the exemption: the hull holds
    /// its velocity to the allowance, so the piece's own bound takes the
    /// share of it any enclosure takes of its budget.
    pub(crate) fn enclosure_accuracy(self, accuracy: [f64; 3]) -> [f64; 3] {
        [
            accuracy[0],
            accuracy[1].min(self.allowance * crate::shaper::hull::ENCLOSURE_BUDGET_SHARE),
            accuracy[2],
        ]
    }

    /// Whether a velocity change of `change` stays within the allowance.
    pub(crate) fn admits(self, change: f64) -> bool {
        change.abs() <= self.allowance
    }
}

/// The velocity step projected follower `axis` of `chains` may take at
/// once, its leaders fitted within `leader_accel_tol` and its ratio within
/// `ratio_bound`: the [`UnshapedLeaderStep`] allowance where an advance
/// with no kernel rides an unshaped leader, whose acceleration steps the
/// advance turns into velocity steps; zero elsewhere, every ratio step
/// being ramped over the time [`CompiledChain::ratio_ramp_rule`] derives.
#[must_use]
pub fn follower_velocity_step_allowance(
    chains: &AxisChainSet,
    axis: usize,
    ratio_bound: f64,
    leader_accel_tol: f64,
) -> f64 {
    let rides_an_unshaped_leader = chains
        .projected_followers()
        .find(|&(follower, _)| follower == axis)
        .is_some_and(|(_, leaders)| {
            leaders
                .iter()
                .any(|&leader| chains.chains[leader].kernel().is_none())
        });
    LeadingAdvance::of(&chains.chains[axis])
        .filter(|advance| advance.is_ramped() && rides_an_unshaped_leader)
        .map_or(0.0, |advance| {
            UnshapedLeaderStep::of(advance, ratio_bound, leader_accel_tol).allowance()
        })
}

/// What every seam of leader `leader`'s fit must carry of its shaped signal
/// for the followers of `chains` that read it. An advance with no kernel
/// after it ([`LeadingAdvance::is_ramped`]) reads the leader's acceleration
/// as velocity, so an acceleration step at a seam would reach the
/// follower's motor as a velocity step.
pub(crate) fn leader_seam_continuity(chains: &AxisChainSet, leader: usize) -> SeamContinuity {
    let read_by_kernel_free_advance = chains
        .projected_followers()
        .filter(|(_, leaders)| leaders.contains(&leader))
        .any(|(follower, _)| {
            LeadingAdvance::of(&chains.chains[follower]).is_some_and(|advance| advance.is_ramped())
        });
    if read_by_kernel_free_advance {
        SeamContinuity::Acceleration
    } else {
        SeamContinuity::Velocity
    }
}

fn advance_offset(stage: &ChainStage, velocity: f64) -> f64 {
    match stage {
        ChainStage::DerivativeGains { k1, .. } => k1 * velocity,
        ChainStage::NonlinearAdvance(advance) => advance.advance(velocity),
        ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
    }
}

fn advance_slope(stage: &ChainStage, velocity: f64) -> f64 {
    match stage {
        ChainStage::DerivativeGains { k1, .. } => *k1,
        ChainStage::NonlinearAdvance(advance) => advance.slope(velocity),
        ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
    }
}

/// Both saturating shapes have unit slope at the origin and flatten away
/// from it, so the law is steepest there.
fn steepest_advance_slope(stage: &ChainStage) -> f64 {
    match stage {
        ChainStage::DerivativeGains { k1, .. } => k1.abs(),
        ChainStage::NonlinearAdvance(advance) => {
            advance.linear_advance.abs()
                + (advance.nonlinear_offset / advance.linearization_velocity).abs()
        }
        ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
    }
}

/// The convolution-relevant cuts of a projected follower source, split by
/// role: `semantic` carries the discontinuities the shaped output inherits
/// (raw axis knots, support grid, speed-zero slivers, ratio-span
/// boundaries), `fit_seeds` adds the component velocity roots that only help
/// the pre-kernel fit resolve the projected signal.
struct FollowerBreakpoints {
    semantic: Vec<f64>,
    fit_seeds: Vec<f64>,
}

/// One raw segment's projected pre-kernel follower track, cached so the
/// follower's own convolution windows read identical inputs on every emit.
#[derive(Debug)]
struct ProjSeg {
    t_start: f64,
    t_end: f64,
    base: f64,
    input_start: SeamInput,
    input_end: SeamInput,
    track: ScalarNurbs,
    semantic_cuts: Vec<f64>,
    /// The advance step at the join with the previous segment, which the
    /// track's own fit leaves out.
    join_advance_step: f64,
    /// The advance steps inside the segment, also left out of `track`.
    window_advance_steps: Vec<AdvanceStep>,
    /// The position tolerance scale the segment's source was fit to.
    tol_scale: f64,
    /// Whether a span the segment's source reach reads retracts.
    reach_retracts: bool,
    /// The extrude-only motion folded into `track`, which a kernel's linear
    /// advance leaves unadvanced.
    extrude_only: Vec<extrude_only::ExtrudeOnlyPiece>,
}

/// One segment's stretch of path: shaped arc length `[s0, s1]` carrying a
/// linearly ramped follower ratio, with `e0` the cumulative projected
/// extrusion at its start.
#[derive(Debug, Clone, Copy)]
struct RatioSpan {
    s0: f64,
    s1: f64,
    r0: f64,
    r1: f64,
    e0: f64,
}

impl RatioSpan {
    fn ratio_at_offset(&self, ds: f64) -> f64 {
        self.r0 + (self.r1 - self.r0) * ds / (self.s1 - self.s0)
    }

    fn ratio_at(&self, s: f64) -> f64 {
        self.ratio_at_offset(s - self.s0)
    }

    fn ratio_slope(&self) -> f64 {
        (self.r1 - self.r0) / (self.s1 - self.s0)
    }

    fn e_at(&self, s: f64) -> f64 {
        self.e0 + self.e_past(s - self.s0)
    }

    fn e_past(&self, ds: f64) -> f64 {
        self.r0 * ds + 0.5 * (self.r1 - self.r0) * ds * ds / (self.s1 - self.s0)
    }

    /// The span as the advance reads it, `(s0, s1, r0, r1)`: its ratio held
    /// at zero wherever it is negative, cut where a ramp crosses zero.
    fn advance_parts(&self) -> [Option<(f64, f64, f64, f64)>; 2] {
        let held = [
            Some((self.s0, self.s1, self.r0.max(0.0), self.r1.max(0.0))),
            None,
        ];
        if self.r0 * self.r1 >= 0.0 {
            return held;
        }
        let zero_at = self.s0 + (self.s1 - self.s0) * self.r0 / (self.r0 - self.r1);
        if !(zero_at > self.s0 && zero_at < self.s1) {
            return held;
        }
        [
            Some((self.s0, zero_at, self.r0.max(0.0), 0.0)),
            Some((zero_at, self.s1, 0.0, self.r1.max(0.0))),
        ]
    }

    fn retracts(&self) -> bool {
        self.r0 < 0.0 || self.r1 < 0.0
    }
}

/// Which of a follower's span tables a read takes: the one the projection
/// lays, or the one a pressure advance reads, which holds the laid ratio at
/// zero wherever it is negative. Klipper advances only the moves that
/// extrude while the toolhead travels; a retract the toolhead makes while
/// it moves, a wipe, pushes no bead to lead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Table {
    Laid,
    Advance,
}

/// Per-follower streaming state: the extrusion-per-shaped-distance table and
/// the shaped path odometer, carried across emit windows so the projected
/// track stays continuous through window boundaries, resets, and chain swaps.
#[derive(Debug, Default)]
pub(crate) struct FollowerState {
    active: bool,
    spans: Vec<RatioSpan>,
    /// [`Table::Advance`]: `spans` with every negative ratio held at zero.
    advance_spans: Vec<RatioSpan>,
    ingested_through_t: Option<f64>,
    s_ingested_end: f64,
    s_shaped: f64,
    /// Where on the shaped path the last leg queued for projection ends:
    /// the next one starts there.
    s_queued_end: f64,
    e_end: Option<f64>,
    projected_output_end: Option<f64>,
    /// Time and follower input state where `projected_output_end` was
    /// taken, so the next stitch can tell what an advance makes of the join.
    projected_input_end: Option<(f64, SeamInput)>,
    emitted_output_pv: Option<(f64, f64)>,
    committed_input_end: Option<SeamInput>,
    projected: Vec<ProjSeg>,
    projected_through_t: Option<f64>,
    projected_trimmed: bool,
    smoothing: RatioSmoothing,
}

impl FollowerState {
    /// A `Reset` restarts the timeline and relabels the follower odometer to
    /// a fresh origin: every reset flavor either re-seeds the MCU step
    /// counters at that same origin (stream_open, set_position, home_drip) or
    /// happens flow-free mid-homing (trip re-anchor), so the pre-reset
    /// odometer labels are void.
    pub(crate) fn reset_timeline(&mut self) {
        self.spans.clear();
        self.advance_spans.clear();
        self.ingested_through_t = None;
        self.s_ingested_end = 0.0;
        self.s_shaped = 0.0;
        self.s_queued_end = 0.0;
        self.e_end = None;
        self.projected_output_end = None;
        self.projected_input_end = None;
        self.emitted_output_pv = None;
        self.committed_input_end = None;
        self.projected.clear();
        self.projected_through_t = None;
        self.projected_trimmed = false;
        self.smoothing.clear();
    }

    /// Drops the cached projected tracks without forgetting how far the
    /// projection has advanced — used when the shaper drops its own raw
    /// history at an era boundary (the stream is at rest there, so the
    /// stream-boundary edge clamp is exact for the next windows).
    pub(crate) fn clear_projected_history(&mut self) {
        self.projected.clear();
        self.projected_trimmed = false;
    }

    pub(crate) fn is_active(&self) -> bool {
        self.active
    }

    /// Marks the follower as projected onto its leaders' shaped path.
    fn start_projecting(&mut self) {
        self.active = true;
    }

    fn cached_projection(&self, t_start: f64, t_end: f64) -> &ProjSeg {
        let idx = self
            .projected
            .partition_point(|p| p.t_end <= t_start + GRID_DEDUP_EPS_S);
        let p = self
            .projected
            .get(idx)
            .unwrap_or_else(|| panic!("no cached projected track covering [{t_start}, {t_end}]"));
        assert!(
            (p.t_start - t_start).abs() <= SEGMENT_TIME_EPS_S
                && (p.t_end - t_end).abs() <= SEGMENT_TIME_EPS_S,
            "cached projected track [{}, {}] misaligned with segment [{t_start}, {t_end}]",
            p.t_start,
            p.t_end,
        );
        p
    }

    fn trim_projected(&mut self, keep_after: f64) {
        let drop = self.projected.partition_point(|p| p.t_end < keep_after);
        if drop > 0 {
            self.projected.drain(..drop);
            self.projected_trimmed = true;
        }
    }

    /// Appends one segment's ratio spans over the shaped arc it covers. A
    /// segment where the raw toolhead rests still moves when shaped: the
    /// kernel drains the previous move into it (its tail, extruded at the
    /// previous ratio) and starts the next move in it (its lead, extruded at
    /// the next one). Both are measured as the shaped leaders' distance
    /// from the rest point at the segment's edges, so the move on each side
    /// extrudes the material of the path it is shaped onto. Any shaped arc
    /// beyond them joins one of them ([`Self::push_rest_arcs`]): a separate
    /// idle span would step the ratio, and so the follower's velocity, where
    /// the leaders are already moving.
    fn ingest(
        &mut self,
        raw: &ContinuousSegment,
        shaped: &ContinuousSegment,
        axis: usize,
        leaders: &[usize],
        upcoming: &[ContinuousSegment],
        read: &IngestRead,
    ) -> f64 {
        let IngestRead {
            shaped_ds,
            ref odometer,
            raw_moves,
            ..
        } = *read;
        if let Some(through) = self.ingested_through_t {
            assert!(
                raw.t_end > through + GRID_DEDUP_EPS_S,
                "follower span ingestion saw an already-ingested segment: \
                 t_end {} within ingested-through {}",
                raw.t_end,
                through
            );
            assert!(
                raw.t_start >= through - GRID_DEDUP_EPS_S,
                "follower span ingestion saw an out-of-order segment: \
                 t_start {} before ingested-through {}",
                raw.t_start,
                through
            );
        }
        self.ingested_through_t = Some(raw.t_end);
        if shaped_ds <= SPAN_MIN_LEN_MM {
            return shaped_ds;
        }
        if raw_moves {
            let (r0, r1) = raw
                .followers
                .iter()
                .find(|f| f.axis_index == axis)
                .filter(|_| raw.spatial_path)
                .map_or((0.0, 0.0), |f| (f.ratio, f.ratio_end));
            self.push_span(shaped_ds, r0, r1);
            return shaped_ds;
        }
        let tail = leader_distance(shaped, raw, leaders, SegmentEdge::Start).min(shaped_ds);
        let lead = leader_distance(shaped, raw, leaders, SegmentEdge::End).min(shaped_ds - tail);
        let idle = shaped_ds - tail - lead;
        let tail_ratio = self.spans.last().map_or(0.0, |span| span.r1);
        if lead <= SPAN_LOOKUP_SLACK_MM && idle <= SPAN_LOOKUP_SLACK_MM {
            self.push_span(shaped_ds, tail_ratio, tail_ratio);
            return shaped_ds;
        }
        let lead_ratio = upcoming
            .iter()
            .find(|seg| seg.spatial_path)
            .and_then(|seg| seg.followers.iter().find(|f| f.axis_index == axis))
            .map_or(0.0, |f| f.ratio);
        let rest_arc = odometer.covered_until(arrival_at_rest(raw, shaped, leaders));
        self.push_rest_arcs((tail, idle, lead), rest_arc, (tail_ratio, lead_ratio));
        shaped_ds
    }

    /// Lays a rest segment's arcs, the ratio stepping from the tail's to
    /// the lead's only `rest_arc` into the segment, where the leaders reach
    /// the rest point. The tail's straight distance from the rest
    /// point falls short of that arc where the shaped axes curve into the
    /// rest unlike each other or the fit wanders toward it: the tail joins
    /// what of the idle arc the leaders cover before they rest
    /// ([`Self::push_tail`]), the lead the rest of it ([`Self::push_lead`]).
    /// A tail too short to lay its own span leaves the whole idle arc to the
    /// lead.
    fn push_rest_arcs(
        &mut self,
        (tail, idle, lead): (f64, f64, f64),
        rest_arc: f64,
        (tail_ratio, lead_ratio): (f64, f64),
    ) {
        if tail <= SPAN_MIN_LEN_MM {
            self.push_lead(lead, idle, lead_ratio);
            return;
        }
        let tail_idle = (rest_arc - tail).clamp(0.0, idle);
        self.push_tail(tail, tail_idle, tail_ratio);
        self.push_lead(lead, idle - tail_idle, lead_ratio);
    }

    /// Lays a tail's material, `tail_ratio × tail`, over the tail joined with
    /// the idle arc, opening on the tail ratio and falling linearly from
    /// there: to `tail_ratio·(2·tail/joined − 1)` at the rest where the idle
    /// arc is no longer than the tail, else to zero over twice the tail and
    /// flat at zero after, the deepest it goes without changing sign. Its
    /// slope steps where the spill starts, the furthest from the rest the
    /// width field ramps it, and the ratio only at the rest. A tail with no
    /// idle arc or no ratio lays one flat span.
    fn push_tail(&mut self, tail: f64, idle: f64, tail_ratio: f64) {
        let joined = tail + idle;
        if idle <= SPAN_MIN_LEN_MM || tail_ratio == 0.0 {
            self.push_span(joined, tail_ratio, tail_ratio);
            return;
        }
        let ramp = joined.min(2.0 * tail);
        self.push_span(ramp, tail_ratio, tail_ratio * (2.0 * tail / ramp - 1.0));
        if joined - ramp > SPAN_MIN_LEN_MM {
            self.push_span(joined - ramp, 0.0, 0.0);
        }
    }

    /// Lays a lead's material, `lead_ratio × lead`, over the lead joined with
    /// the idle arc, opening and closing on the lead ratio. The ratio ramps
    /// linearly into a valley and back out over equal widths: with the idle
    /// arc no longer than the lead, a V reaching just as deep as the idle's
    /// share of material; with a longer idle arc, ramps one lead wide each
    /// around a flat floor at zero, the deepest a valley goes without
    /// changing sign. The two coincide where the lead and idle arcs are
    /// equal. A lead with no ratio lays one flat span: its valley would only
    /// cut the arc into extra breakpoints, down to slivers where the lead is
    /// a few picometres.
    fn push_lead(&mut self, lead: f64, idle: f64, lead_ratio: f64) {
        let joined = lead + idle;
        if idle <= SPAN_MIN_LEN_MM || lead_ratio == 0.0 {
            if joined > SPAN_MIN_LEN_MM {
                self.push_span(joined, lead_ratio, lead_ratio);
            }
            return;
        }
        let ramp = (0.5 * joined).min(lead);
        let floor = joined - 2.0 * ramp;
        let valley_ratio = lead_ratio * (lead - ramp) / (joined - ramp);
        if ramp > SPAN_MIN_LEN_MM {
            self.push_span(ramp, lead_ratio, valley_ratio);
        }
        if floor > SPAN_MIN_LEN_MM {
            self.push_span(floor, valley_ratio, valley_ratio);
        }
        if ramp > SPAN_MIN_LEN_MM {
            self.push_span(ramp, valley_ratio, lead_ratio);
        }
    }

    /// A span shorter than the odometer's float resolution cannot advance
    /// `s_ingested_end` (`s0 + len` rounds back to `s0`); pushing it would
    /// mint a zero-width span whose `e_at(s1)` is 0/0, poisoning every
    /// later span's cumulative `e0`. Its extrusion is below one odometer
    /// ulp, so dropping it drops nothing representable.
    fn push_span(&mut self, len: f64, r0: f64, r1: f64) {
        let s1 = self.s_ingested_end + len;
        if s1 == self.s_ingested_end {
            return;
        }
        let e0 = self.spans.last().map_or(0.0, |span| span.e_at(span.s1));
        let span = RatioSpan {
            s0: self.s_ingested_end,
            s1,
            r0,
            r1,
            e0,
        };
        self.spans.push(span);
        for (s0, s1, r0, r1) in span.advance_parts().into_iter().flatten() {
            let e0 = self
                .advance_spans
                .last()
                .map_or(0.0, |span| span.e_at(span.s1));
            self.advance_spans.push(RatioSpan { s0, s1, r0, r1, e0 });
        }
        self.s_ingested_end = s1;
    }

    /// Drops the spans behind the committed path, but for those a window
    /// there still averages over.
    fn prune_spans(&mut self) {
        let committed = self.s_shaped - SPAN_LOOKUP_SLACK_MM;
        let keep_after = committed - self.smoothing.reach_back(committed);
        let keep_from = self.spans.partition_point(|span| span.s1 < keep_after);
        self.spans.drain(..keep_from);
        let keep_advance_from = self
            .advance_spans
            .partition_point(|span| span.s1 < keep_after);
        self.advance_spans.drain(..keep_advance_from);
        self.smoothing.prune_before(committed);
    }

    /// Cumulative projected spatial extrusion at shaped path distance `s`.
    /// A kernel with negative lobes can overshoot the ingested path by a
    /// whisker at a terminal flush; the terminal ratio extends it.
    fn spans_e(&self, s: f64) -> f64 {
        self.table_e(Table::Laid, s)
            .map_or(0.0, |table| table + self.smoothing.correction(s).value)
    }

    fn span_origin(&self, s: f64) -> SpanOrigin {
        SpanOrigin {
            s,
            table_e: self.table_e(Table::Laid, s).unwrap_or(0.0),
            smoothing_e: self.smoothing.correction(s).value,
        }
    }

    /// [`Self::spans_e`] at `distance` past `origin`, less its value at
    /// `origin`. Path length and cumulative extrusion both run tens of
    /// millimetres deep into a print, so the difference of two absolute
    /// reads carries rounding at that depth, which over a nanosecond fit
    /// span is a seam mismatch the stored track turns into acceleration.
    /// Every term here is a difference of nearby quantities instead.
    fn spans_e_since(&self, origin: &SpanOrigin, distance: OdometerDistance) -> f64 {
        self.table_e_since(origin, distance)
            + (self.smoothing.correction(distance.s).value - origin.smoothing_e)
    }

    fn table_e_since(&self, origin: &SpanOrigin, distance: OdometerDistance) -> f64 {
        let Some(first) = self.spans.first() else {
            return 0.0;
        };
        let OdometerDistance { s, offset } = distance;
        assert!(
            s >= first.s0 - SPAN_LOOKUP_SLACK_MM,
            "shaped path odometer {s} fell behind the pruned span table \
             starting at {}",
            first.s0
        );
        let past = |boundary: f64| (origin.s - boundary) + offset;
        let idx = self.spans.partition_point(|span| span.s1 < s);
        match self.spans.get(idx) {
            Some(span) => (span.e0 - origin.table_e) + span.e_past(past(span.s0).max(0.0)),
            None => {
                let last = self.spans.last().expect("non-empty spans");
                (last.e_at(last.s1) - origin.table_e) + last.r1 * past(last.s1)
            }
        }
    }

    fn spans_of(&self, table: Table) -> &[RatioSpan] {
        match table {
            Table::Laid => &self.spans,
            Table::Advance => &self.advance_spans,
        }
    }

    fn table_e(&self, table: Table, s: f64) -> Option<f64> {
        let spans = self.spans_of(table);
        let first = spans.first()?;
        assert!(
            s >= first.s0 - SPAN_LOOKUP_SLACK_MM,
            "shaped path odometer {s} fell behind the pruned span table \
             starting at {}",
            first.s0
        );
        let idx = spans.partition_point(|span| span.s1 < s);
        Some(match spans.get(idx) {
            Some(span) => span.e_at(s.max(span.s0)),
            None => {
                let last = spans.last().expect("non-empty spans");
                last.e_at(last.s1) + last.r1 * (s - last.s1)
            }
        })
    }

    /// Projected extrusion accumulated between two shaped-path offsets
    /// measured from `s_base`. Every term is a difference of nearby
    /// quantities, so the result carries no ulp of the cumulative `e0` the
    /// spans are stacked on — the whole point, since a drain span an
    /// odometer tick wide sits on tens of millimetres of cumulative
    /// extrusion and `spans_e(s1) - spans_e(s0)` would be pure rounding.
    fn spans_delta_e(&self, s_base: f64, offset_start: f64, offset_end: f64) -> f64 {
        let Some(first) = self.spans.first() else {
            return 0.0;
        };
        let sign = if offset_end < offset_start { -1.0 } else { 1.0 };
        let floor = first.s0 - s_base;
        let lo = offset_start.min(offset_end).max(floor);
        let hi = offset_start.max(offset_end).max(floor);
        let mut delta = 0.0;
        for span in &self.spans {
            let span_lo = span.s0 - s_base;
            if span_lo >= hi {
                break;
            }
            let span_hi = span.s1 - s_base;
            let a = lo.clamp(span_lo, span_hi);
            let b = hi.clamp(span_lo, span_hi);
            if b > a {
                delta += (b - a)
                    * 0.5
                    * (span.ratio_at_offset(a - span_lo) + span.ratio_at_offset(b - span_lo));
            }
        }
        let last = self.spans.last().expect("non-empty spans");
        let tail = last.s1 - s_base;
        if hi > tail {
            delta += last.r1 * (hi - lo.max(tail));
        }
        if !self.smoothing.is_inert() {
            delta += self.smoothing.correction(s_base + hi).value
                - self.smoothing.correction(s_base + lo).value;
        }
        sign * delta
    }

    fn ratio_and_slope(&self, s: f64, side: SpanSide) -> (f64, f64) {
        self.ratio_and_slope_on(Table::Laid, s, side)
    }

    fn ratio_and_slope_on(&self, table: Table, s: f64, side: SpanSide) -> (f64, f64) {
        let (ratio, slope) = self.table_ratio_and_slope(table, s, side);
        let correction = self.smoothing.correction_of(table, s);
        (ratio + correction.d1, slope + correction.d2)
    }

    fn table_ratio_and_slope(&self, table: Table, s: f64, side: SpanSide) -> (f64, f64) {
        let spans = self.spans_of(table);
        let idx = match side {
            SpanSide::After => spans.partition_point(|span| span.s1 <= s),
            SpanSide::Before => spans.partition_point(|span| span.s1 < s),
        };
        match spans.get(idx) {
            Some(span) if s >= span.s0 => (span.ratio_at(s), span.ratio_slope()),
            Some(span) => (span.r0, 0.0),
            None => (spans.last().map_or(0.0, |span| span.r1), 0.0),
        }
    }

    /// What `table` and its smoothing lay at `s`, the smoothing's
    /// correction read once: [`Self::spans_e`], [`Self::ratio_and_slope`]
    /// and `d²r/ds²` (the table's spans are linear, so only the smoothing
    /// bends it).
    fn laid_on(&self, table: Table, s: f64, side: SpanSide) -> LaidRead {
        let (ratio, slope) = self.table_ratio_and_slope(table, s, side);
        let correction = self.smoothing.correction_of(table, s);
        LaidRead {
            e: self
                .table_e(table, s)
                .map_or(0.0, |table| table + correction.value),
            ratio: ratio + correction.d1,
            slope: slope + correction.d2,
            curvature: correction.d3,
        }
    }

    /// [`Self::laid_on`] of the laid table at `distance` past `origin`, its
    /// extrusion read as [`Self::spans_e_since`] reads it.
    fn laid_since(
        &self,
        origin: &SpanOrigin,
        distance: OdometerDistance,
        side: SpanSide,
    ) -> LaidRead {
        let (ratio, slope) = self.table_ratio_and_slope(Table::Laid, distance.s, side);
        let correction = self.smoothing.correction_of(Table::Laid, distance.s);
        LaidRead {
            e: self.table_e_since(origin, distance) + (correction.value - origin.smoothing_e),
            ratio: ratio + correction.d1,
            slope: slope + correction.d2,
            curvature: correction.d3,
        }
    }

    /// Whether a span the smoothed table reads anywhere on `[s_lo, s_hi]`
    /// retracts: a window there averages the table no further than its
    /// width either side, so nothing laid past that reach decides it.
    fn retracts_within(&self, s_lo: f64, s_hi: f64) -> bool {
        let lo = s_lo - SPAN_LOOKUP_SLACK_MM;
        let hi = s_hi + SPAN_LOOKUP_SLACK_MM;
        let (reach_lo, reach_hi) = (
            lo - self.smoothing.reach_back(lo),
            hi + self.smoothing.reach_back(hi),
        );
        self.spans
            .iter()
            .any(|span| span.retracts() && span.s1 > reach_lo && span.s0 < reach_hi)
    }

    fn aligned_span_start(&self, s: f64) -> f64 {
        let envelope = span_alignment_envelope(s);
        self.spans
            .iter()
            .flat_map(|span| [span.s0, span.s1])
            .filter(|&boundary| boundary > s && boundary - s <= envelope)
            .fold(s, f64::max)
    }

    fn owned_span_end(&self, s_start: f64, s_end: f64) -> f64 {
        let envelope = span_alignment_envelope(s_end);
        self.spans
            .iter()
            .flat_map(|span| [span.s0, span.s1])
            .filter(|&boundary| boundary <= s_end && s_end - boundary < envelope)
            .fold(s_end, |end, boundary| end.min(next_lower_float(boundary)))
            .max(s_start)
    }
}

/// Which span a table read at a span boundary takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SpanSide {
    /// The span starting there.
    After,
    /// The span ending there.
    Before,
}

impl SpanSide {
    /// The side a segment owning `[s_start, s_end]` reads its start on:
    /// its own span after it, or, owning no path, the span before.
    fn opening(s_start: f64, s_end: f64) -> Self {
        if s_end > s_start {
            Self::After
        } else {
            Self::Before
        }
    }
}

fn projection_support(
    raw: &ContinuousSegment,
    shaped: &ContinuousSegment,
    axis: usize,
    leaders: &[usize],
) -> (f64, f64) {
    let (axis_start, axis_end) = raw.axes[axis].domain();
    let (mut start, mut end) = (raw.t_start.max(axis_start), raw.t_end.min(axis_end));
    for &leader in leaders {
        let (leader_start, leader_end) = shaped.axes[leader].domain();
        start = start.max(leader_start);
        end = end.min(leader_end);
    }
    assert!(
        start < end,
        "follower axis {axis} has no continuous projection support"
    );
    (start, end)
}

fn axis_pva(axis: &ContinuousAxis, t: f64) -> (f64, f64, f64) {
    let pva = axis
        .eval_pva(t)
        .unwrap_or_else(|error| panic!("continuous follower evaluator failed at {t}: {error}"));
    (pva.position, pva.velocity, pva.acceleration)
}

fn axis_breakpoints(axis: &ContinuousAxis) -> Vec<f64> {
    match axis {
        ContinuousAxis::Spline(curve) | ContinuousAxis::RelativeSpline { curve, .. } => {
            bezier_piece_ends(curve).collect()
        }
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => pieces
            .iter()
            .flat_map(|piece| bezier_piece_ends(&piece.curve))
            .collect(),
        ContinuousAxis::Analytic { span, .. } => {
            let mut breaks = Vec::with_capacity(span.phases.len() + 2);
            breaks.push(span.t_start);
            breaks.extend(span.phase_seam_times());
            breaks.push(span.t_end);
            breaks
        }
        ContinuousAxis::Hold { .. } | ContinuousAxis::Nudge(_) | ContinuousAxis::Buzz { .. } => {
            Vec::new()
        }
    }
}

fn leader_arc_length(seg: &ContinuousSegment, leaders: &[usize]) -> f64 {
    shaped_leader_arc_length(seg, leaders, &ShapedLeaders::of(seg, leaders)).0
}

/// Whether the raw leaders of `raw` cover more than [`SPAN_MIN_LEN_MM`]: at
/// once where the chord across their support does by far, as no arc is
/// shorter than its chord, else by their arc length.
fn raw_leaders_move(raw: &ContinuousSegment, leaders: &[usize]) -> bool {
    let (start, end) = leader_support(raw, leaders);
    let chord = leaders
        .iter()
        .map(|&leader| {
            let travel = axis_pva(&raw.axes[leader], end).0 - axis_pva(&raw.axes[leader], start).0;
            travel * travel
        })
        .sum::<f64>()
        .sqrt();
    chord > MOVING_CHORD_MM || leader_arc_length(raw, leaders) > SPAN_MIN_LEN_MM
}

/// The arc length the leaders of `seg` cover, with the odometer leaves it
/// sums.
fn shaped_leader_arc_length(
    seg: &ContinuousSegment,
    leaders: &[usize],
    shaped_leaders: &ShapedLeaders,
) -> (f64, OdometerLeaves) {
    let grid = axis_grid(seg, leaders);
    let mut leaves = Vec::new();
    let length = grid
        .windows(2)
        .map(|window| {
            let mut length = 0.0;
            odometer_leaves(
                &|t| shaped_leaders.speed(t),
                window[0],
                window[1],
                &mut |end, leaf| {
                    length += leaf.covered_until(end);
                    leaves.push((end, leaf));
                },
            );
            length
        })
        .sum();
    (length, OdometerLeaves { grid, leaves })
}

/// One newly ingested segment as ingest reads it: its shaped leaders, the
/// arc length they cover with the odometer leaves summing it, and whether
/// its raw leaders move.
struct IngestRead<'a> {
    shaped_leaders: ShapedLeaders<'a>,
    shaped_ds: f64,
    odometer: OdometerLeaves,
    raw_moves: bool,
}

/// [`IngestRead`] for each newly ingested segment. The reads are pure
/// per-segment work, so they fan out across cores; results are ordered by
/// segment and bit-identical to serial evaluation.
fn ingest_reads<'a>(
    raw: &[ContinuousSegment],
    shaped: &'a [ContinuousSegment],
    leaders: &'a [usize],
    pool: &WorkerPool,
) -> Vec<IngestRead<'a>> {
    assert_eq!(raw.len(), shaped.len());
    let workers = if cfg!(target_arch = "wasm32") {
        1
    } else {
        std::thread::available_parallelism().map_or(1, |cores| {
            cores.get().saturating_sub(1).max(1).min(raw.len())
        })
    };
    pool.map_indexed(
        workers,
        raw.len(),
        || (),
        |(), i| {
            let shaped_leaders = ShapedLeaders::of(&shaped[i], leaders);
            let (shaped_ds, odometer) =
                shaped_leader_arc_length(&shaped[i], leaders, &shaped_leaders);
            IngestRead {
                shaped_leaders,
                shaped_ds,
                odometer,
                raw_moves: raw_leaders_move(&raw[i], leaders),
            }
        },
    )
    .expect("leader arc length thread panicked")
}

#[derive(Clone, Copy)]
enum SegmentEdge {
    Start,
    End,
}

/// How far apart two segments' leaders are at their shared start or end.
fn leader_distance(
    a: &ContinuousSegment,
    b: &ContinuousSegment,
    leaders: &[usize],
    edge: SegmentEdge,
) -> f64 {
    let t = match edge {
        SegmentEdge::Start => leaders
            .iter()
            .fold(a.t_start.max(b.t_start), |start, &axis| {
                start
                    .max(a.axes[axis].domain().0)
                    .max(b.axes[axis].domain().0)
            }),
        SegmentEdge::End => leaders.iter().fold(a.t_end.min(b.t_end), |end, &axis| {
            end.min(a.axes[axis].domain().1)
                .min(b.axes[axis].domain().1)
        }),
    };
    leaders
        .iter()
        .map(|&axis| {
            let delta = axis_pva(&a.axes[axis], t).0 - axis_pva(&b.axes[axis], t).0;
            delta * delta
        })
        .sum::<f64>()
        .sqrt()
}

/// Where every one of `axes` of `seg` is continuously defined.
fn leader_support(seg: &ContinuousSegment, axes: &[usize]) -> (f64, f64) {
    let (mut support_start, mut support_end) = (seg.t_start, seg.t_end);
    for &axis in axes {
        let (start, end) = seg.axes[axis].domain();
        support_start = support_start.max(start);
        support_end = support_end.min(end);
    }
    assert!(
        support_start < support_end,
        "axes have no shared continuous support"
    );
    (support_start, support_end)
}

fn axis_grid(seg: &ContinuousSegment, axes: &[usize]) -> Vec<f64> {
    let (support_start, support_end) = leader_support(seg, axes);
    let mut grid = vec![support_start, support_end];
    for &axis in axes {
        grid.extend(
            axis_breakpoints(&seg.axes[axis])
                .iter()
                .copied()
                .filter(|&time| time > support_start && time < support_end),
        );
    }
    grid.sort_by(f64::total_cmp);
    grid.dedup_by(|left, right| (*left - *right).abs() <= GRID_DEDUP_EPS_S);
    grid
}

#[derive(Clone, Copy)]
struct SpeedZeroSliver {
    inner: f64,
    zero_time: f64,
    accel_norm: f64,
    interior_sign: f64,
}

impl SpeedZeroSliver {
    /// Whether the sliver holds the window `[lo, hi]` whose interior reads
    /// it: the window lies on its side of its inner edge.
    fn holds_window(&self, lo: f64, hi: f64) -> bool {
        if self.interior_sign > 0.0 {
            hi <= self.inner
        } else {
            lo >= self.inner
        }
    }

    fn deriv_at(&self, t: f64) -> f64 {
        let sign = if t > self.zero_time {
            1.0
        } else if t < self.zero_time {
            -1.0
        } else {
            self.interior_sign
        };
        sign * self.accel_norm
    }
}

#[derive(Debug, Clone)]
struct LeaderPieces {
    base: f64,
    pieces: Vec<BezierPiece>,
}

impl LeaderPieces {
    fn of(axis: &ContinuousAxis) -> Option<Self> {
        let candidate = match axis {
            ContinuousAxis::Spline(curve) => Some(Self {
                base: 0.0,
                pieces: extract_bezier_pieces(curve),
            }),
            ContinuousAxis::RelativeSpline {
                base_position,
                curve,
            } => Some(Self {
                base: *base_position,
                pieces: extract_bezier_pieces(curve),
            }),
            ContinuousAxis::PiecewiseRelativeSpline(relative_pieces) => {
                let mut pieces = Vec::new();
                for relative in relative_pieces.iter() {
                    for mut piece in extract_bezier_pieces(&relative.curve) {
                        piece.coeffs[0] += relative.base_position;
                        pieces.push(piece);
                    }
                }
                let contiguous = pieces
                    .windows(2)
                    .all(|pair| pair[0].u_end == pair[1].u_start);
                contiguous.then_some(Self { base: 0.0, pieces })
            }
            _ => None,
        }?;
        candidate.matches(axis).then_some(candidate)
    }

    /// Whether every piece reads what the axis does at its far Gauss
    /// node: the power basis's conversion error grows along the piece, so
    /// an ill-conditioned piece shows there.
    fn matches(&self, axis: &ContinuousAxis) -> bool {
        const PROBE: f64 = 0.788_675_134_594_812_9;
        self.pieces.iter().all(|piece| {
            let duration = piece.u_end - piece.u_start;
            if !(duration.is_finite() && duration > 0.0) {
                return false;
            }
            let t = piece.u_start + PROBE * duration;
            let (position, velocity, acceleration) =
                polynomial_pva(&piece.coeffs, t - piece.u_start);
            let cached = [self.base + position, velocity, acceleration];
            let direct = axis_pva(axis, t);
            cached
                .into_iter()
                .zip([direct.0, direct.1, direct.2])
                .all(|(left, right)| {
                    left.is_finite()
                        && right.is_finite()
                        && (left - right).abs() <= 1e-8 * left.abs().max(right.abs()).max(1.0)
                })
        })
    }

    fn piece_at(&self, t: f64) -> Option<&BezierPiece> {
        let index = self
            .pieces
            .partition_point(|piece| piece.u_end < t)
            .min(self.pieces.len().saturating_sub(1));
        let piece = self.pieces.get(index)?;
        let slack = 1e-9 * (piece.u_end - piece.u_start).max(1.0);
        (t >= piece.u_start - slack && t <= piece.u_end + slack).then_some(piece)
    }

    /// The piece `at` reads, when it also reads `[lo, hi]` around `at`.
    fn piece_over(&self, lo: f64, at: f64, hi: f64) -> Option<&BezierPiece> {
        let piece = self.piece_at(at)?;
        let slack = 1e-9 * (piece.u_end - piece.u_start).max(1.0);
        (lo >= piece.u_start - slack && hi <= piece.u_end + slack).then_some(piece)
    }

    fn pva(&self, t: f64) -> Option<(f64, f64, f64)> {
        let piece = self.piece_at(t)?;
        let (p, v, a) = polynomial_pva(&piece.coeffs, t - piece.u_start);
        Some((self.base + p, v, a))
    }

    fn pvaj(&self, t: f64) -> Option<(f64, f64, f64, f64)> {
        let piece = self.piece_at(t)?;
        let (p, v, a, j) = polynomial_pvaj(&piece.coeffs, t - piece.u_start);
        Some((self.base + p, v, a, j))
    }
}

/// The odometer leaves over one segment's leader knot grid, in order, each
/// with the end it covers to.
#[derive(Debug)]
struct OdometerLeaves {
    grid: Vec<f64>,
    leaves: Vec<(f64, OdometerLeaf)>,
}

impl OdometerLeaves {
    /// The arc the leaders cover from the segment's start until `t`.
    fn covered_until(&self, t: f64) -> f64 {
        let mut covered = 0.0;
        for (end, leaf) in &self.leaves {
            if t <= *end {
                return covered + leaf.covered_until(t);
            }
            covered += leaf.covered_until(*end);
        }
        covered
    }
}

/// Probes per leader knot interval the leaders of a rest segment are
/// searched with for where they reach the rest point.
const REST_POINT_PROBES_PER_INTERVAL: usize = 8;
const REST_POINT_REFINEMENTS: usize = 64;

/// When the shaped leaders of `shaped` first stop closing on where the raw
/// ones of `raw` rest: the first probe their distance from it grows after,
/// refined by golden-section search over the probes either side. The fit's
/// back-and-forth around the rest point after that changes nothing of how
/// they arrived.
fn arrival_at_rest(raw: &ContinuousSegment, shaped: &ContinuousSegment, leaders: &[usize]) -> f64 {
    let rest_at = leader_support(raw, leaders).0;
    let distance_sq = |t: f64| {
        leaders
            .iter()
            .map(|&axis| {
                let delta =
                    axis_pva(&shaped.axes[axis], t).0 - axis_pva(&raw.axes[axis], rest_at).0;
                delta * delta
            })
            .sum::<f64>()
    };
    let grid = axis_grid(shaped, leaders);
    let mut probes: Vec<f64> = grid
        .windows(2)
        .flat_map(|window| {
            (0..REST_POINT_PROBES_PER_INTERVAL).map(move |k| {
                window[0]
                    + (window[1] - window[0]) * k as f64 / REST_POINT_PROBES_PER_INTERVAL as f64
            })
        })
        .collect();
    probes.push(grid[grid.len() - 1]);
    let distances: Vec<f64> = probes.iter().map(|&t| distance_sq(t)).collect();
    let nearest = distances
        .windows(2)
        .position(|pair| pair[1] > pair[0])
        .unwrap_or(distances.len() - 1);
    let (mut lo, mut hi) = (
        probes[nearest.saturating_sub(1)],
        probes[(nearest + 1).min(probes.len() - 1)],
    );
    let golden = 0.5 * (5.0_f64.sqrt() - 1.0);
    for _ in 0..REST_POINT_REFINEMENTS {
        let (left, right) = (hi - golden * (hi - lo), lo + golden * (hi - lo));
        if distance_sq(left) <= distance_sq(right) {
            hi = right;
        } else {
            lo = left;
        }
    }
    0.5 * (lo + hi)
}

/// What ingest read off one shaped segment's leaders that its source fit
/// reads again.
#[derive(Debug)]
struct LeaderReads {
    pieces: Vec<Option<LeaderPieces>>,
    odometer: OdometerLeaves,
}

/// The leaders a [`ShapedLeaders::speed_floor`] bounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FloorLeaders {
    /// Every leader, so one no cached piece reads across the interval
    /// leaves no floor.
    Every,
    /// Those a cached piece reads across the interval: the part of the
    /// velocity they carry is never faster than the whole.
    Cached,
}

struct ShapedLeaders<'a> {
    leaders: &'a [usize],
    axes: Vec<&'a ContinuousAxis>,
    pieces: Cow<'a, [Option<LeaderPieces>]>,
    odometer: Option<&'a OdometerLeaves>,
}

impl<'a> ShapedLeaders<'a> {
    fn of(seg: &'a ContinuousSegment, leaders: &'a [usize]) -> Self {
        let axes: Vec<&'a ContinuousAxis> =
            leaders.iter().map(|&leader| &seg.axes[leader]).collect();
        let pieces = axes.iter().map(|axis| LeaderPieces::of(axis)).collect();
        Self {
            leaders,
            axes,
            pieces: Cow::Owned(pieces),
            odometer: None,
        }
    }

    /// The leaders of `seg` read through `reads`, which ingest took from
    /// [`Self::of`] over the same segment.
    fn with_reads(
        seg: &'a ContinuousSegment,
        leaders: &'a [usize],
        reads: &'a LeaderReads,
    ) -> Self {
        assert_eq!(
            leaders.len(),
            reads.pieces.len(),
            "cached leader pieces belong to a different leader set"
        );
        Self {
            leaders,
            axes: leaders.iter().map(|&leader| &seg.axes[leader]).collect(),
            pieces: Cow::Borrowed(&reads.pieces),
            odometer: Some(&reads.odometer),
        }
    }

    fn into_reads(self, odometer: OdometerLeaves) -> LeaderReads {
        LeaderReads {
            pieces: self.pieces.into_owned(),
            odometer,
        }
    }

    fn pva(&self, index: usize, t: f64) -> (f64, f64, f64) {
        if let Some(Some(pieces)) = self.pieces.get(index) {
            if let Some(value) = pieces.pva(t) {
                return value;
            }
        }
        axis_pva(self.axes[index], t)
    }

    /// Every leader's state at `t`, to jerk where every leader reads a
    /// cached piece: the piece's position, velocity and acceleration are
    /// [`Self::pva`]'s and its jerk [`Self::pvaj`]'s.
    fn motion(&self, t: f64) -> LeaderMotion {
        let leaders = self.axes.len();
        assert!(
            leaders <= MAX_LEADERS,
            "a follower rides at most {MAX_LEADERS} leaders, got {leaders}"
        );
        let mut states = [(0.0, 0.0, 0.0, 0.0); MAX_LEADERS];
        let mut jerks_read = true;
        for (index, state) in states.iter_mut().take(leaders).enumerate() {
            let cached = match self.pieces.get(index) {
                Some(Some(pieces)) => pieces.pvaj(t),
                _ => None,
            };
            *state = cached.unwrap_or_else(|| {
                jerks_read = false;
                let (position, velocity, acceleration) = axis_pva(self.axes[index], t);
                (position, velocity, acceleration, 0.0)
            });
        }
        LeaderMotion {
            states,
            leaders,
            jerks_read,
        }
    }

    fn speed(&self, t: f64) -> f64 {
        norm_of((0..self.axes.len()).map(|index| self.pva(index, t).1))
    }

    /// A lower bound on the speed of `bounded` leaders over `[lo, hi]`, and
    /// so on the leaders' own: their velocity's tangent line at `at`, at its
    /// nearest to rest over the interval, less the most the pieces' higher
    /// terms add to it and their evaluation's rounding.
    fn speed_floor(&self, lo: f64, at: f64, hi: f64, bounded: FloorLeaders) -> Option<f64> {
        let leaders = self.axes.len();
        assert!(
            leaders <= MAX_LEADERS,
            "a follower rides at most {MAX_LEADERS} leaders, got {leaders}"
        );
        let (back, ahead) = (at - lo, hi - at);
        let reach = back.max(ahead);
        let mut velocity = [0.0; MAX_LEADERS];
        let mut acceleration = [0.0; MAX_LEADERS];
        let mut remainder_squared = 0.0;
        let mut magnitude = 0.0;
        for (index, (axis, pieces)) in self.axes.iter().zip(self.pieces.iter()).enumerate() {
            if matches!(axis, ContinuousAxis::Hold { .. }) {
                continue;
            }
            let Some(piece) = pieces
                .as_ref()
                .and_then(|pieces| pieces.piece_over(lo, at, hi))
            else {
                match bounded {
                    FloorLeaders::Every => return None,
                    FloorLeaders::Cached => continue,
                }
            };
            let shifted = taylor_shifted(&piece.coeffs, at - piece.u_start);
            velocity[index] = shifted.get(1).copied().unwrap_or(0.0);
            acceleration[index] = 2.0 * shifted.get(2).copied().unwrap_or(0.0);
            let remainder = shifted
                .iter()
                .enumerate()
                .skip(3)
                .rev()
                .fold(0.0, |acc, (power, coeff)| {
                    acc * reach + power as f64 * coeff.abs()
                })
                * reach
                * reach;
            remainder_squared += remainder * remainder;
            let farthest = (lo - piece.u_start).abs().max((hi - piece.u_start).abs());
            magnitude += piece
                .coeffs
                .iter()
                .enumerate()
                .skip(1)
                .rev()
                .fold(0.0, |acc, (power, coeff)| {
                    acc * farthest + power as f64 * coeff.abs()
                });
        }
        let (velocity, acceleration) = (&velocity[..leaders], &acceleration[..leaders]);
        let dot = |left: &[f64], right: &[f64]| -> f64 {
            left.iter().zip(right).map(|(l, r)| l * r).sum()
        };
        let rate_squared = dot(acceleration, acceleration);
        let nearest = if rate_squared > 0.0 {
            (-dot(velocity, acceleration) / rate_squared).clamp(-back, ahead)
        } else {
            0.0
        };
        let tangent_speed = velocity
            .iter()
            .zip(acceleration)
            .map(|(v, a)| {
                let along = v + a * nearest;
                along * along
            })
            .sum::<f64>()
            .sqrt();
        Some(tangent_speed - remainder_squared.sqrt() - SPEED_ROUNDING * magnitude)
    }

    fn never_moves(&self, index: usize, t0: f64, t1: f64, rest: RestResolution) -> bool {
        let bounds = self.axes[index].pva_bounds(t0, t1).unwrap_or_else(|error| {
            panic!("leader {index} has no velocity bounds over [{t0}, {t1}]: {error}")
        });
        rest.holds(bounds.velocity_min, bounds.acceleration_abs_max)
            && rest.holds(bounds.velocity_max, bounds.acceleration_abs_max)
    }

    /// The leader's state to third order, read from the same representation
    /// as [`Self::pva`]: the path-speed derivatives divide by the speed, so
    /// mixing two evaluations of one leader amplifies their rounding
    /// difference wherever the speed is tiny.
    fn pvaj(&self, index: usize, t: f64) -> (f64, f64, f64, f64) {
        if let Some(Some(pieces)) = self.pieces.get(index) {
            if let Some(value) = pieces.pvaj(t) {
                return value;
            }
        }
        let pvaj = self.axes[index]
            .eval_pvaj(t)
            .expect("follower leader has exact derivatives");
        (pvaj.position, pvaj.velocity, pvaj.acceleration, pvaj.jerk)
    }

    fn speed_second(&self, t: f64, speed: f64) -> f64 {
        let (states, leaders) = self.pvaj_states(t);
        speed_second_of(&states[..leaders], speed)
    }

    fn pvaj_states(&self, t: f64) -> ([(f64, f64, f64, f64); MAX_LEADERS], usize) {
        let leaders = self.axes.len();
        assert!(
            leaders <= MAX_LEADERS,
            "a follower rides at most {MAX_LEADERS} leaders, got {leaders}"
        );
        let mut states = [(0.0, 0.0, 0.0, 0.0); MAX_LEADERS];
        for (index, state) in states.iter_mut().take(leaders).enumerate() {
            *state = self.pvaj(index, t);
        }
        (states, leaders)
    }

    fn accel_norm(&self, t: f64) -> f64 {
        norm_of((0..self.axes.len()).map(|index| self.pva(index, t).2))
    }
}

const MAX_LEADERS: usize = 3;

/// The Euclidean norm of the leaders' `components`, in leader order.
fn norm_of(components: impl Iterator<Item = f64>) -> f64 {
    components
        .map(|component| component * component)
        .sum::<f64>()
        .sqrt()
}

/// The leaders' states at one time, which their speed, its rate and their
/// acceleration's norm read, and, when `jerks_read`, the speed's second
/// rate too.
#[derive(Clone, Copy)]
struct LeaderMotion {
    states: [(f64, f64, f64, f64); MAX_LEADERS],
    leaders: usize,
    jerks_read: bool,
}

impl LeaderMotion {
    fn speed(&self) -> f64 {
        norm_of(self.states[..self.leaders].iter().map(|state| state.1))
    }

    fn speed_deriv(&self, speed: f64) -> f64 {
        if speed == 0.0 {
            return 0.0;
        }
        self.states[..self.leaders]
            .iter()
            .map(|&(_, velocity, acceleration, _)| velocity / speed * acceleration)
            .sum::<f64>()
    }

    fn accel_norm(&self) -> f64 {
        norm_of(self.states[..self.leaders].iter().map(|state| state.2))
    }

    /// [`ShapedLeaders::speed_second`] at `speed`, where the jerks are read.
    fn speed_second(&self, speed: f64) -> Option<f64> {
        self.jerks_read
            .then(|| speed_second_of(&self.states[..self.leaders], speed))
    }
}

/// `d²|v|/dt² = |a⊥|²/|v| + v̂·j`, read through the unit direction
/// `v̂ = v/|v|` before any product: near rest the leaders' velocity is
/// small enough that `|v|³` and `|v|²|a|²` underflow to zero long before
/// the curvature they describe does. `|a⊥|²` is summed through Lagrange's
/// identity as the squared pairwise cross terms of `v̂` and `a`: the
/// difference form cancels to rounding times `|a|²`, which the division by
/// a near-rest speed would amplify.
fn speed_second_of(states: &[(f64, f64, f64, f64)], speed: f64) -> f64 {
    let mut normal_squared = 0.0;
    for (index, &(_, v_i, a_i, _)) in states.iter().enumerate() {
        for &(_, v_j, a_j, _) in &states[index + 1..] {
            let normal = v_i / speed * a_j - v_j / speed * a_i;
            normal_squared += normal * normal;
        }
    }
    let direction_dot_jerk = states
        .iter()
        .map(|&(_, v, _, j)| v / speed * j)
        .sum::<f64>();
    normal_squared / speed + direction_dot_jerk
}

/// The odometer's leaves: each knot's time, the distance to it, and the
/// leaders' speed there.
struct OdometerKnots {
    t: Vec<f64>,
    s: Vec<f64>,
    leaves: Vec<OdometerLeaf>,
    leaf_magnitude: f64,
}

impl OdometerKnots {
    fn starting_at(t: f64, capacity: usize) -> Self {
        let mut knots = Self {
            t: Vec::with_capacity(capacity),
            s: Vec::with_capacity(capacity),
            leaves: Vec::with_capacity(capacity),
            leaf_magnitude: 0.0,
        };
        knots.t.push(t);
        knots.s.push(0.0);
        knots
    }

    fn push_leaf(&mut self, end: f64, leaf: OdometerLeaf) {
        let value = leaf.covered_until(end);
        let length = self.length() + value;
        self.t.push(end);
        self.s.push(length);
        self.leaves.push(leaf);
        self.leaf_magnitude += value.abs();
    }

    fn length(&self) -> f64 {
        *self.s.last().expect("odometer knots seeded")
    }

    fn leaves(&self) -> usize {
        self.leaves.len()
    }

    fn reassociation_bound(&self) -> f64 {
        self.leaves() as f64 * f64::EPSILON * self.leaf_magnitude
    }

    fn scale_to(&self, ingested: f64) -> f64 {
        let integrated = self.length();
        let start = self.t[0];
        let end = self.t[self.t.len() - 1];
        if integrated == 0.0 {
            assert_eq!(
                ingested, 0.0,
                "follower leaders stand still over [{start}, {end}] but the span table \
                 ingested {ingested} mm"
            );
            return 1.0;
        }
        let bound = self.reassociation_bound();
        assert!(
            (integrated - ingested).abs() <= bound,
            "follower odometer integrates {integrated} mm over [{start}, {end}] but the span \
             table ingested {ingested} mm, beyond the {bound} mm that reordering its {} leaves \
             can account for",
            self.leaves()
        );
        ingested / integrated
    }
}

#[derive(Clone, Copy)]
struct Odometer {
    start: f64,
    length: f64,
}

#[derive(Clone, Copy)]
struct RatioBounds {
    ratio: f64,
    slope: f64,
}

impl RatioBounds {
    /// The ratio bounds over the spans a signal owning `[s_start,
    /// s_owned_end]` reads, never one starting past its end: whether the
    /// next segment's spans are laid yet must not reach the fit.
    fn ridden(state: &FollowerState, s_start: f64, s_owned_end: f64) -> Self {
        let ridden = state
            .spans
            .iter()
            .filter(|span| span.s1 >= s_start - SPAN_LOOKUP_SLACK_MM && span.s0 < s_owned_end);
        let table = Self {
            ratio: ridden
                .clone()
                .map(|span| span.r0.abs().max(span.r1.abs()))
                .chain(
                    [
                        (s_start, SpanSide::opening(s_start, s_owned_end)),
                        (s_owned_end, SpanSide::Before),
                    ]
                    .map(|(s, side)| state.ratio_and_slope(s, side).0.abs()),
                )
                .fold(0.0_f64, f64::max),
            slope: ridden
                .map(|span| span.ratio_slope().abs())
                .fold(0.0_f64, f64::max),
        };
        if !state.smoothing.touches(s_start, s_owned_end) {
            return table;
        }
        let (ratio, slope) = state
            .smoothing
            .ratio_bounds(&state.spans, s_start, s_owned_end);
        Self {
            ratio: table.ratio.max(ratio),
            slope: table.slope.max(slope),
        }
    }
}

/// What the span table and its smoothing lay at one shaped path distance.
#[derive(Clone, Copy)]
struct LaidRead {
    /// The extrusion laid up to the distance read, from the origin the read
    /// is taken against: the table's own for [`FollowerState::laid_on`], the
    /// segment's for [`FollowerState::laid_since`].
    e: f64,
    ratio: f64,
    slope: f64,
    curvature: f64,
}

/// The shaped path distance at one time, absolute for table lookups and as
/// the offset from the segment's odometer start the integration produced.
#[derive(Clone, Copy)]
struct OdometerDistance {
    s: f64,
    offset: f64,
}

/// What [`FollowerState::spans_e`] reads at the segment's odometer start,
/// split into the table's and the smoothing's share.
#[derive(Clone, Copy)]
struct SpanOrigin {
    s: f64,
    table_e: f64,
    smoothing_e: f64,
}

/// The follower's path state at one time, read once for every quantity
/// derived from it: where the odometer stands, how fast it runs and turns,
/// and what is laid there.
#[derive(Clone, Copy)]
struct PathRead {
    motion: LeaderMotion,
    speed: f64,
    rate: f64,
    rate_deriv: f64,
    s: f64,
    laid: LaidRead,
}

pub(crate) struct FollowerSignal<'a> {
    state: &'a FollowerState,
    e_start: f64,
    s_start: f64,
    /// The odometer's start plus the length it was scaled to, the sum the
    /// projection loop carries to the next segment's start, so a segment
    /// starts where its predecessor ends whether one emit held both or not.
    s_end: f64,
    e_spans_start: f64,
    span_origin: SpanOrigin,
    leaders: ShapedLeaders<'a>,
    raw_delta: Option<(&'a ContinuousAxis, f64)>,
    grid: Vec<f64>,
    knots: OdometerKnots,
    odometer_scale: f64,
    s_owned_end: f64,
    ratio_bounds: RatioBounds,
    /// Whether a span the signal's reach reads retracts, so that its
    /// advance reads [`Table::Advance`] there.
    reach_retracts: bool,
    start_sliver: Option<SpeedZeroSliver>,
    end_sliver: Option<SpeedZeroSliver>,
    pva_memo: PvaMemo,
    /// The enclosure over the whole signal, which every rung of the fit
    /// asks again.
    whole_enclosure: enclosure::WholeEnclosure,
    /// The leaders and the raw track as the enclosure reads them.
    axis_windows: std::cell::OnceCell<Option<enclosure::FollowerAxisWindows<'a>>>,
    /// See [`Self::ratio_crossings`].
    ratio_crossings: std::cell::OnceCell<Vec<(f64, f64)>>,
    /// The leaders' rests inside the support, in order ([`rest`]).
    rests: Vec<rest::Rest>,
}

impl<'a> FollowerSignal<'a> {
    /// The follower track of `raw` riding the shaped path of `shaped`, whose
    /// leaders `shaped_leaders` reads.
    fn riding(
        shaped: &'a ContinuousSegment,
        raw: &'a ContinuousSegment,
        axis: usize,
        shaped_leaders: ShapedLeaders<'a>,
        state: &'a FollowerState,
        odometer: Odometer,
        e_start: f64,
    ) -> Self {
        let leaders = shaped_leaders.leaders;
        assert!(
            leaders
                .iter()
                .zip(&shaped_leaders.axes)
                .all(|(&leader, &read)| std::ptr::eq(read, &shaped.axes[leader])),
            "the shaped leaders read another segment than the one the follower rides"
        );
        let (t0, t1) = projection_support(raw, shaped, axis, leaders);
        let raw_delta =
            (!raw.spatial_path).then(|| (&raw.axes[axis], axis_pva(&raw.axes[axis], t0).0));
        let mut grid = vec![t0, t1];
        grid.extend(
            axis_grid(shaped, leaders)
                .into_iter()
                .filter(|&t| t > t0 && t < t1),
        );
        grid.sort_by(f64::total_cmp);
        grid.dedup_by(|left, right| (*left - *right).abs() <= GRID_DEDUP_EPS_S);
        let mut knots = OdometerKnots::starting_at(grid[0], grid.len());
        match shaped_leaders
            .odometer
            .filter(|ingested| ingested.grid == grid)
        {
            Some(ingested) => {
                for &(end, leaf) in &ingested.leaves {
                    knots.push_leaf(end, leaf);
                }
            }
            None => {
                for w in grid.windows(2) {
                    odometer_leaves(
                        &|t| shaped_leaders.speed(t),
                        w[0],
                        w[1],
                        &mut |end, leaf| {
                            knots.push_leaf(end, leaf);
                        },
                    );
                }
            }
        }
        let odometer_scale = knots.scale_to(odometer.length);
        let s_end = odometer.start + odometer.length;
        let s_owned_end = state.owned_span_end(odometer.start, s_end);
        let ratio_bounds = RatioBounds::ridden(state, odometer.start, s_owned_end);
        let reach_retracts = state.retracts_within(odometer.start, s_end);

        let mut sig = Self {
            state,
            e_start,
            s_start: odometer.start,
            s_end,
            e_spans_start: state.spans_e(odometer.start),
            span_origin: state.span_origin(odometer.start),
            leaders: shaped_leaders,
            raw_delta,
            grid,
            knots,
            odometer_scale,
            s_owned_end: f64::INFINITY,
            ratio_bounds,
            reach_retracts,
            start_sliver: None,
            end_sliver: None,
            pva_memo: PvaMemo::default(),
            whole_enclosure: enclosure::WholeEnclosure::default(),
            axis_windows: std::cell::OnceCell::new(),
            ratio_crossings: std::cell::OnceCell::new(),
            rests: Vec::new(),
        };
        let (start_sliver, end_sliver) = sig.speed_zero_slivers();
        sig.start_sliver = start_sliver;
        sig.end_sliver = end_sliver;
        sig.s_owned_end = s_owned_end;
        sig
    }

    /// The signal reading the leaders' rests, at speeds their fits to
    /// `leader_tol` cannot resolve ([`rest`]).
    fn with_rests(mut self, leader_tol: FitTol) -> Self {
        self.rests = self.find_rests(self.rest_speed(leader_tol));
        self
    }

    fn endpoint_sliver(
        &self,
        endpoint: f64,
        slack: f64,
        interior_sign: f64,
    ) -> Option<SpeedZeroSliver> {
        let inner = endpoint + interior_sign * slack;
        let speed = self.leaders.speed(endpoint);
        let secant = (self.leaders.speed(inner) - speed) / (inner - endpoint);
        (speed <= secant.abs() * slack).then(|| SpeedZeroSliver {
            inner,
            zero_time: if speed == 0.0 {
                endpoint
            } else {
                endpoint - speed / secant
            },
            accel_norm: self.leaders.accel_norm(endpoint),
            interior_sign,
        })
    }

    fn speed_zero_slivers(&self) -> (Option<SpeedZeroSliver>, Option<SpeedZeroSliver>) {
        let [t0, first_knot, ..] = self.grid[..] else {
            unreachable!("a follower signal's grid holds both ends of its support")
        };
        let [.., last_knot, t1] = self.grid[..] else {
            unreachable!("a follower signal's grid holds both ends of its support")
        };
        (
            self.endpoint_sliver(t0, SPEED_ZERO_SLIVER_FRACTION * (first_knot - t0), 1.0),
            self.endpoint_sliver(t1, SPEED_ZERO_SLIVER_FRACTION * (t1 - last_knot), -1.0),
        )
    }

    fn speed_zero_sliver(&self, t: f64) -> Option<SpeedZeroSliver> {
        self.start_sliver
            .filter(|sliver| t <= sliver.inner)
            .or_else(|| self.end_sliver.filter(|sliver| t >= sliver.inner))
    }

    /// How far the sliver's speed rate steps from the leaders' own where it
    /// takes over.
    fn sliver_rate_step(&self, sliver: &SpeedZeroSliver) -> f64 {
        let motion = self.leaders.motion(sliver.inner);
        (sliver.deriv_at(sliver.inner) - motion.speed_deriv(motion.speed())).abs()
    }

    /// The sliver's speed rate is the leaders' where it takes over, to
    /// within rounding: its inner edge is no break of the signal.
    fn sliver_joins_the_leaders(&self, sliver: &SpeedZeroSliver) -> bool {
        self.sliver_rate_step(sliver) <= SLIVER_RATE_ROUNDING * sliver.accel_norm.max(1.0)
    }

    fn shaped_speed(&self, t: f64) -> f64 {
        self.odometer_scale * self.leaders.speed(t)
    }

    fn odometer_rate(&self, t: f64) -> (f64, f64) {
        let motion = self.leaders.motion(t);
        let speed = motion.speed();
        (
            self.odometer_scale * speed,
            self.odometer_scale * self.leader_speed_deriv(t, &motion, speed),
        )
    }

    fn leader_speed_deriv(&self, t: f64, motion: &LeaderMotion, speed: f64) -> f64 {
        if let Some(sliver) = self.speed_zero_sliver(t) {
            return sliver.deriv_at(t);
        }
        match self.rest_at(t) {
            Some(rest) => self.rest_rates(rest, t).0,
            None => motion.speed_deriv(speed),
        }
    }

    fn leader_speed_second_deriv(&self, t: f64, motion: &LeaderMotion, speed: f64) -> f64 {
        if self.speed_zero_sliver(t).is_some() {
            return 0.0;
        }
        if let Some(rest) = self.rest_at(t) {
            return self.rest_rates(rest, t).1;
        }
        if speed == 0.0 {
            return 0.0;
        }
        motion
            .speed_second(speed)
            .unwrap_or_else(|| self.leaders.speed_second(t, speed))
    }

    fn path_read(&self, t: f64) -> PathRead {
        let motion = self.leaders.motion(t);
        let speed = motion.speed();
        let distance = self.owned_distance(t);
        PathRead {
            motion,
            speed,
            rate: self.odometer_scale * speed,
            rate_deriv: self.odometer_scale * self.leader_speed_deriv(t, &motion, speed),
            s: distance.s,
            laid: self.laid_since(distance),
        }
    }

    fn pva_at(&self, t: f64) -> (f64, f64, f64) {
        let (rate, rate_deriv) = self.odometer_rate(t);
        self.pva_laid(
            t,
            (rate, rate_deriv),
            self.laid_since(self.owned_distance(t)),
        )
    }

    fn pva_read(&self, t: f64, read: &PathRead) -> (f64, f64, f64) {
        self.pva_laid(t, (read.rate, read.rate_deriv), read.laid)
    }

    fn pva_laid(&self, t: f64, (rate, rate_deriv): (f64, f64), laid: LaidRead) -> (f64, f64, f64) {
        let (raw_p, raw_v, raw_a) = self.raw_delta.map_or((0.0, 0.0, 0.0), |(axis, at_start)| {
            let (position, velocity, acceleration) = axis_pva(axis, t);
            (position - at_start, velocity, acceleration)
        });
        (
            self.e_start + laid.e + raw_p,
            laid.ratio * rate + raw_v,
            laid.slope * rate * rate + laid.ratio * rate_deriv + raw_a,
        )
    }

    fn path_deriv(&self, t: f64) -> f64 {
        let (ratio, _) = self.ratio_and_slope(self.s_owned(t));
        ratio * self.shaped_speed(t)
    }

    fn path_second_deriv(&self, t: f64) -> f64 {
        let (rate, rate_deriv) = self.odometer_rate(t);
        let (ratio, slope) = self.ratio_and_slope(self.s_owned(t));
        slope * rate * rate + ratio * rate_deriv
    }

    fn rate_second(&self, t: f64, read: &PathRead) -> f64 {
        self.odometer_scale * self.leader_speed_second_deriv(t, &read.motion, read.speed)
    }

    /// The table the advance reads: [`Table::Advance`], which is the laid
    /// table itself wherever that retracts nowhere within the reach.
    fn advance_table(&self) -> Table {
        if self.reach_retracts {
            Table::Advance
        } else {
            Table::Laid
        }
    }

    /// `d(R·W)/ds` at `s`: the path slope of the ratio a ratio-ramped
    /// advance reads.
    fn ramped_read_slope(&self, s: f64) -> f64 {
        let (ratio, slope) = self.ratio_and_slope_on(self.advance_table(), s);
        let (weight, weight_slope, _) = self.state.smoothing.valley_weight(s);
        slope * weight + ratio * weight_slope
    }

    /// Whether an advance step inside the segment is no more than the
    /// leaders' own pieces explain where they meet with a speed residual,
    /// which the ratio scales: rounding a weld at the next join absorbs.
    fn leader_residual_explains(&self, step: &AdvanceStep, (t_start, t_end): (f64, f64)) -> bool {
        let residual = (self.shaped_speed(trajectory::next_toward(step.t, t_end))
            - self.shaped_speed(trajectory::next_toward(step.t, t_start)))
        .abs();
        (step.velocity_after - step.velocity_before).abs()
            <= self.ratio_bounds.ratio * residual
                + ADVANCE_STEP_VELOCITY_ROUNDING
                    * step
                        .velocity_before
                        .abs()
                        .max(step.velocity_after.abs())
                        .max(1.0)
    }

    /// What a ratio-ramped advance reads at `t`: the velocity `R(s)·ṡ` the
    /// projection lays along the leaders' path and its first two time
    /// derivatives, leaving out what the raw track moves on its own. `R`
    /// is the ratio [`Table::Advance`] lays, weighted down to zero at every
    /// turnaround of the leaders ([`RatioSmoothing::valley_weight`]).
    fn ramped_advance_read(&self, t: f64) -> (f64, f64, f64) {
        self.ramped_read(t, &self.path_read(t))
    }

    fn ramped_read(&self, t: f64, read: &PathRead) -> (f64, f64, f64) {
        let PathRead {
            rate,
            rate_deriv,
            s,
            laid,
            ..
        } = *read;
        let laid = match self.advance_table() {
            Table::Laid => laid,
            Table::Advance => self.laid_on(Table::Advance, s),
        };
        let rate_second = self.rate_second(t, read);
        let (weight, weight_slope, weight_curvature) = self.state.smoothing.valley_weight(s);
        let weighted = laid.ratio * weight;
        let weighted_slope = laid.slope * weight + laid.ratio * weight_slope;
        let weighted_curvature = laid.curvature * weight
            + 2.0 * laid.slope * weight_slope
            + laid.ratio * weight_curvature;
        (
            weighted * rate,
            weighted_slope * rate * rate + weighted * rate_deriv,
            weighted * rate_second
                + 3.0 * weighted_slope * rate * rate_deriv
                + weighted_curvature * rate * rate * rate,
        )
    }

    fn s_offset(&self, t: f64) -> f64 {
        let start = self.grid[0];
        let end = self.grid[self.grid.len() - 1];
        let scale = t.abs().max(start.abs()).max(end.abs());
        let slack = 1e-12_f64.max(8.0 * f64::EPSILON * scale);
        assert!(
            t >= start - slack && t <= end + slack,
            "follower distance query {t} outside [{start}, {end}]"
        );
        let t = if t < start {
            start
        } else if t > end {
            end
        } else {
            t
        };
        let integrated = match self.knots.t.binary_search_by(|knot| knot.total_cmp(&t)) {
            Ok(index) => self.knots.s[index],
            Err(insertion) => {
                assert!(
                    insertion > 0 && insertion < self.knots.t.len(),
                    "follower distance query {t} outside [{}, {}]",
                    self.grid[0],
                    self.grid[self.grid.len() - 1]
                );
                let index = insertion - 1;
                self.knots.s[index] + self.knots.leaves[index].covered_until(t)
            }
        };
        integrated * self.odometer_scale
    }
    fn s_at(&self, t: f64) -> f64 {
        self.s_start + self.s_offset(t)
    }
    /// Which side of a span boundary the signal reads the table on at `s`:
    /// at its owned end, the span it lays from, never the one after, which
    /// the next segment owns and which may not be laid yet.
    fn side_at(&self, s: f64) -> SpanSide {
        if s >= self.s_owned_end {
            SpanSide::Before
        } else {
            SpanSide::After
        }
    }

    fn ratio_and_slope(&self, s: f64) -> (f64, f64) {
        self.ratio_and_slope_on(Table::Laid, s)
    }

    fn ratio_and_slope_on(&self, table: Table, s: f64) -> (f64, f64) {
        self.state.ratio_and_slope_on(table, s, self.side_at(s))
    }

    fn laid_since(&self, distance: OdometerDistance) -> LaidRead {
        self.state
            .laid_since(&self.span_origin, distance, self.side_at(distance.s))
    }

    fn laid_on(&self, table: Table, s: f64) -> LaidRead {
        self.state.laid_on(table, s, self.side_at(s))
    }

    fn s_owned(&self, t: f64) -> f64 {
        self.owned_distance(t).s
    }
    fn s_owned_offset(&self, t: f64) -> f64 {
        self.owned_distance(t).offset
    }
    /// The odometer readings `t` may take: at least the boundary of the
    /// last ratio crossing at or before it, below that of the next. At a
    /// crawl the odometer steps back, its leaf's rate running under a speed
    /// smaller than the rate's tolerance, and would otherwise read a
    /// boundary's formula again after its crossing or before it.
    fn crossing_hold(&self, t: f64) -> (f64, f64) {
        let crossings = self.ratio_crossings();
        let next = crossings.partition_point(|&(time, _)| time <= t);
        let floor = next
            .checked_sub(1)
            .map_or(f64::NEG_INFINITY, |last| crossings[last].1);
        let ceiling = crossings.get(next).map_or(f64::INFINITY, |&(_, boundary)| {
            trajectory::next_toward(boundary, f64::NEG_INFINITY)
        });
        (floor, ceiling)
    }
    /// The odometer's reading at `t`, held to its ratio crossings
    /// ([`Self::crossing_hold`]) and to the owned end.
    fn owned_distance(&self, t: f64) -> OdometerDistance {
        let (floor, ceiling) = self.crossing_hold(t);
        let offset = self.s_offset(t);
        let s = self.s_start + offset;
        let held = if s < floor {
            floor - self.s_start
        } else if s > ceiling {
            ceiling - self.s_start
        } else {
            offset
        };
        OdometerDistance {
            s: s.clamp(floor, ceiling).min(self.s_owned_end),
            offset: held.min(self.s_owned_end - self.s_start),
        }
    }
    fn s_end(&self) -> f64 {
        self.s_end
    }
    /// The exact value of an identically-constant window — a follower whose
    /// span table commands zero extrusion ratio across the whole owned range
    /// and whose raw axis contributes no extrude-only motion (every travel
    /// and homing move). Constant input needs no fit: the kernel-shaped
    /// leader lattice would otherwise seed hundreds of pieces of flat spline,
    /// starving real-time drip streams on small hosts.
    fn constant_value(&self) -> Option<f64> {
        if let Some((axis, _)) = self.raw_delta {
            if !matches!(axis, ContinuousAxis::Hold { .. }) {
                return None;
            }
        }
        let s_lo = self.s_start;
        let s_hi = self.s_end().max(self.s_owned_end.min(f64::MAX));
        let overlapping_zero = self
            .state
            .spans
            .iter()
            .filter(|span| span.s1 >= s_lo - SPAN_LOOKUP_SLACK_MM && span.s0 < self.s_owned_end)
            .all(|span| span.r0 == 0.0 && span.r1 == 0.0);
        let tail_zero = match self
            .state
            .spans
            .iter()
            .rev()
            .find(|span| span.s0 < self.s_owned_end)
        {
            Some(last) if self.s_owned_end > last.s1 => last.r1 == 0.0,
            _ => true,
        };
        let smoothed = self
            .state
            .smoothing
            .touches(s_lo - SPAN_LOOKUP_SLACK_MM, s_hi + SPAN_LOOKUP_SLACK_MM);
        (overlapping_zero && tail_zero && !smoothed).then(|| self.eval(self.grid[0]))
    }

    /// The times inside the support the odometer first reads a span or
    /// smoothing boundary, each beside that boundary, in order: the
    /// formula the table lays, or the one the advance reads, changes there,
    /// the signal reading the one before up to the time and the one after
    /// from it on ([`Self::crossing_hold`]). Every boundary of the laid
    /// table bounds a span of the advance's too. A crossing is held no
    /// earlier than the one before, so the times rise with the boundaries
    /// however the odometer steps back between.
    fn ratio_crossings(&self) -> &[(f64, f64)] {
        self.ratio_crossings.get_or_init(|| {
            let s_end = self.s_end();
            let support_start = *self.grid.first().expect("follower grid is seeded");
            let support_end = *self.grid.last().expect("follower grid is seeded");
            let mut boundaries = self
                .state
                .spans_of(self.advance_table())
                .iter()
                .flat_map(|span| [span.s0, span.s1])
                .chain(self.state.smoothing.bends(
                    self.s_start - SPAN_LOOKUP_SLACK_MM,
                    s_end + SPAN_LOOKUP_SLACK_MM,
                ))
                .filter(|&s| {
                    s >= self.s_start - SPAN_LOOKUP_SLACK_MM && s <= s_end + SPAN_LOOKUP_SLACK_MM
                })
                .collect::<Vec<_>>();
            boundaries.sort_by(f64::total_cmp);
            boundaries.dedup();
            let start_reading = self.s_at(support_start);
            let mut crossings = Vec::new();
            let mut earliest = support_start;
            for boundary in boundaries.into_iter().filter(|&b| b > start_reading) {
                let crossing = self
                    .crossing_of(boundary, support_start, support_end)
                    .max(earliest);
                earliest = crossing;
                if crossing - support_start > GRID_DEDUP_EPS_S
                    && support_end - crossing > GRID_DEDUP_EPS_S
                {
                    crossings.push((crossing, boundary));
                }
            }
            crossings
        })
    }

    /// A float of `(lo, hi]` the odometer reads `boundary` at, the float
    /// before it below, by bisection from `lo` reading below it; `hi` where
    /// the odometer stays below throughout.
    fn crossing_of(&self, boundary: f64, mut lo: f64, mut hi: f64) -> f64 {
        loop {
            let mid = 0.5 * lo + 0.5 * hi;
            if mid <= lo || mid >= hi {
                return hi;
            }
            if self.s_at(mid) < boundary {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }

    fn construction_breakpoints(
        &self,
        raw_axis: &ContinuousAxis,
        acceleration_tol: f64,
    ) -> FollowerBreakpoints {
        let raw_breaks = axis_breakpoints(raw_axis);
        let mut breaks = raw_breaks.clone();
        breaks.push(*self.grid.first().expect("follower grid is seeded"));
        breaks.push(*self.grid.last().expect("follower grid is seeded"));
        let support_start = *self.grid.first().expect("follower grid is seeded");
        let support_end = *self.grid.last().expect("follower grid is seeded");
        for sliver in [self.start_sliver, self.end_sliver].into_iter().flatten() {
            if sliver.inner > support_start + GRID_DEDUP_EPS_S
                && sliver.inner < support_end - GRID_DEDUP_EPS_S
                && !self.sliver_joins_the_leaders(&sliver)
            {
                breaks.push(sliver.inner);
            }
        }
        breaks.extend(
            self.ratio_crossings()
                .iter()
                .map(|&(time, _)| time)
                .filter(|&time| self.speed_zero_sliver(time).is_none()),
        );
        breaks.extend(
            self.rests
                .iter()
                .map(|rest| rest.split)
                .filter(|&t| t > support_start && t < support_end),
        );
        breaks.sort_by(f64::total_cmp);
        breaks.dedup_by(|left, right| (*left - *right).abs() <= GRID_DEDUP_EPS_S);
        for b in breaks.iter_mut() {
            for &rb in &raw_breaks {
                if (*b - rb).abs() <= GRID_DEDUP_EPS_S && *b != rb {
                    *b = rb;
                }
            }
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let semantic = breaks.clone();
        let velocity_resolution = acceleration_tol * MIN_DEVICE_INTERVAL_S;
        breaks.extend(self.acceleration_steps_on_the_grid(acceleration_tol));
        breaks.extend(self.velocity_steps_on_the_grid(velocity_resolution));
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let rest = RestResolution {
            velocity: velocity_resolution,
            acceleration: acceleration_tol * REST_ACCELERATION_SHARE,
        };
        let zeros = self.velocity_component_zeros(&breaks, support_start, support_end, rest);
        for zero in zeros {
            let dominated = breaks.iter().any(|&b| (b - zero).abs() <= GRID_DEDUP_EPS_S);
            if !dominated {
                breaks.push(zero);
            }
        }
        breaks.sort_by(f64::total_cmp);
        FollowerBreakpoints {
            semantic,
            fit_seeds: breaks,
        }
    }

    /// Interior leader-grid times where the follower's own acceleration
    /// steps by more than `tolerance`: a shaped leader's pieces join with
    /// their acceleration free to jump, and the projection inherits the
    /// step. A fit span straddling one cannot hold the tolerance; every
    /// other grid point is smooth enough to fit across, and seeding all of
    /// them would multiply the follower's fit and kernel spans.
    ///
    /// The projection is `ë = r(s)·s̈ + r'(s)·ṡ²` plus raw-axis motion, and
    /// raw-axis and ratio-span breaks are seeded on their own. At a leader
    /// seam its step is therefore at most `max|r|·|Δs̈| + max|r'|·|Δṡ²|`,
    /// with the path speed `ṡ` and its rate `s̈ = v·a/|v|` read off the
    /// leaders alone; only seams where that bound exceeds the tolerance pay
    /// for evaluating the projection itself on both sides. The ratio bounds
    /// cover every span the signal rides and what the table holds beyond
    /// its ends.
    fn acceleration_steps_on_the_grid(&self, tolerance: f64) -> Vec<f64> {
        self.grid_acceleration_steps(
            tolerance,
            |t| self.second_deriv(t),
            |_, step| step > tolerance,
        )
    }

    /// Interior leader-grid times where a leading advance turns the
    /// acceleration step `Δa` it reads into a velocity step `a'(v)·Δa`
    /// wider than `velocity_resolution`. The velocity it reads is
    /// continuous there, so the step is the advance slope at that velocity
    /// times `Δa`, and only seams whose `Δa` bound times the steepest slope
    /// the law reaches clears the resolution pay for evaluating it.
    fn advance_velocity_steps_on_the_grid(
        &self,
        advance: LeadingAdvance,
        velocity_resolution: f64,
    ) -> Vec<f64> {
        self.grid_acceleration_steps(
            velocity_resolution / steepest_advance_slope(advance.stage),
            |t| advance.read_acceleration(self, t),
            |t, step| {
                advance_slope(advance.stage, advance.read_velocity(self, t)).abs() * step
                    > velocity_resolution
            },
        )
    }
    fn velocity_steps_on_the_grid(&self, velocity_resolution: f64) -> Vec<f64> {
        let ratio_bound = self.ratio_bounds.ratio;
        let interior = &self.grid[1..self.grid.len() - 1];
        interior
            .iter()
            .copied()
            .filter(|&t| {
                let before = trajectory::next_toward(t, f64::NEG_INFINITY);
                let after = trajectory::next_toward(t, f64::INFINITY);
                let path_speed_step = self.odometer_scale
                    * (self.leaders.speed(after) - self.leaders.speed(before)).abs();
                ratio_bound * path_speed_step > velocity_resolution
                    && (self.deriv(after) - self.deriv(before)).abs() > velocity_resolution
            })
            .collect()
    }

    fn grid_acceleration_steps(
        &self,
        step_bound_threshold: f64,
        acceleration: impl Fn(f64) -> f64,
        step_exceeds: impl Fn(f64, f64) -> bool,
    ) -> Vec<f64> {
        let RatioBounds {
            ratio: ratio_bound,
            slope: slope_bound,
        } = self.ratio_bounds;
        let path_rate = |t: f64| {
            let (speed_squared, power) = (0..self.leaders.axes.len()).fold(
                (0.0_f64, 0.0_f64),
                |(speed_squared, power), index| {
                    let (_, velocity, acceleration) = self.leaders.pva(index, t);
                    (
                        speed_squared + velocity * velocity,
                        power + velocity * acceleration,
                    )
                },
            );
            let speed = speed_squared.sqrt();
            let rate = if speed > 0.0 { power / speed } else { 0.0 };
            let scale = self.odometer_scale;
            (scale * scale * speed_squared, scale * rate)
        };
        let interior = &self.grid[1..self.grid.len() - 1];
        interior
            .iter()
            .copied()
            .filter(|&t| {
                let before = trajectory::next_toward(t, f64::NEG_INFINITY);
                let after = trajectory::next_toward(t, f64::INFINITY);
                let (speed_squared_before, rate_before) = path_rate(before);
                let (speed_squared_after, rate_after) = path_rate(after);
                let bound = ratio_bound * (rate_after - rate_before).abs()
                    + slope_bound * (speed_squared_after - speed_squared_before).abs();
                bound > step_bound_threshold
                    && step_exceeds(t, (acceleration(after) - acceleration(before)).abs())
            })
            .collect()
    }

    fn velocity_component_zeros(
        &self,
        sorted_breaks: &[f64],
        support_start: f64,
        support_end: f64,
        rest: RestResolution,
    ) -> Vec<f64> {
        let mut nodes = Vec::with_capacity(sorted_breaks.len() + 2);
        nodes.push(support_start);
        nodes.extend(
            sorted_breaks
                .iter()
                .copied()
                .filter(|&t| t > support_start && t < support_end),
        );
        nodes.push(support_end);
        let mut zeros = Vec::new();
        let moving_leaders = (0..self.leaders.axes.len()).filter(|&leader| {
            !self
                .leaders
                .never_moves(leader, support_start, support_end, rest)
        });
        for leader in moving_leaders {
            let direction = |t| {
                let (_, velocity, acceleration) = self.leaders.pva(leader, t);
                rest.direction(velocity, acceleration)
            };
            for window in nodes.windows(2) {
                isolate_velocity_zeros(&direction, window[0], window[1], &mut zeros);
            }
        }
        zeros
    }
}

/// A leader component at rest as far as the follower's fit can tell: its
/// velocity under the fit's velocity resolution and its acceleration under
/// a share of the acceleration tolerance so small that a stop made at it
/// stays negligible even through an advance. A resting leader's fit moves
/// at the rounding of its exact signal with either sign, so rest is read
/// by this resolution and never by that sign.
#[derive(Clone, Copy, Debug)]
struct RestResolution {
    velocity: f64,
    acceleration: f64,
}

impl RestResolution {
    fn holds(self, velocity: f64, acceleration: f64) -> bool {
        velocity.abs() <= self.velocity && acceleration.abs() <= self.acceleration
    }

    /// The sign a component moves with, `None` at rest. A velocity exactly
    /// zero off rest is a reversal landing on a float, which moves the way
    /// it accelerates.
    fn direction(self, velocity: f64, acceleration: f64) -> Option<f64> {
        if self.holds(velocity, acceleration) {
            return None;
        }
        Some(if velocity == 0.0 {
            acceleration.signum()
        } else {
            velocity.signum()
        })
    }
}

/// Seeds every reversal between probes moving opposite ways, and every
/// time a component starts or stops resting. The times a resting component
/// is probed at are arbitrary, so rest itself seeds nothing.
fn isolate_velocity_zeros(
    direction_at: &impl Fn(f64) -> Option<f64>,
    lo: f64,
    hi: f64,
    zeros: &mut Vec<f64>,
) {
    if hi <= lo {
        return;
    }
    let span = hi - lo;
    let mut previous_time = lo;
    let mut previous_direction = direction_at(lo);
    for probe in 1..=COMPONENT_ZERO_PROBES {
        let time = if probe == COMPONENT_ZERO_PROBES {
            hi
        } else {
            lo + span * (probe as f64 / COMPONENT_ZERO_PROBES as f64)
        };
        let direction = direction_at(time);
        match (previous_direction, direction) {
            (None, None) => {}
            (Some(_), None) => {
                zeros.push(refine_rest_boundary(direction_at, time, previous_time));
            }
            (None, Some(_)) => {
                zeros.push(refine_rest_boundary(direction_at, previous_time, time));
            }
            (Some(previous), Some(current)) if previous != current => {
                zeros.push(refine_velocity_zero(
                    direction_at,
                    previous_time,
                    time,
                    previous,
                ));
            }
            (Some(_), Some(_)) => {}
        }
        previous_time = time;
        previous_direction = direction;
    }
}

fn refine_velocity_zero(
    direction_at: &impl Fn(f64) -> Option<f64>,
    mut lo: f64,
    mut hi: f64,
    lo_direction: f64,
) -> f64 {
    loop {
        let mid = 0.5 * lo + 0.5 * hi;
        if mid <= lo || mid >= hi {
            return hi;
        }
        match direction_at(mid) {
            None => return mid,
            Some(direction) if direction == lo_direction => lo = mid,
            Some(_) => hi = mid,
        }
    }
}

/// The first time off rest next to it, between a resting time and a moving
/// one in either order: where a component starts or stops moving, which
/// is no crossing but changes the path speed's law all the same.
fn refine_rest_boundary(
    direction_at: &impl Fn(f64) -> Option<f64>,
    mut resting: f64,
    mut moving: f64,
) -> f64 {
    loop {
        let mid = 0.5 * resting + 0.5 * moving;
        if mid == resting || mid == moving {
            return moving;
        }
        match direction_at(mid) {
            None => resting = mid,
            Some(_) => moving = mid,
        }
    }
}

impl TrackSignal for FollowerSignal<'_> {
    fn eval(&self, t: f64) -> f64 {
        let spans = self
            .state
            .spans_e_since(&self.span_origin, self.owned_distance(t));
        let raw = self
            .raw_delta
            .map_or(0.0, |(axis, at_start)| axis_pva(axis, t).0 - at_start);
        self.e_start + spans + raw
    }

    /// `eval` carries the stream's cumulative extrusion, tens of millimetres
    /// deep into a print; the difference of two of those samples over a span
    /// an odometer tick wide is rounding noise. The span table integrates the
    /// ratio over the shaped distance actually travelled instead, and the raw
    /// track contributes its own relative delta.
    fn position_delta(&self, (t0, _): (f64, f64), (t1, _): (f64, f64)) -> f64 {
        let spans = self.state.spans_delta_e(
            self.s_start,
            self.s_owned_offset(t0),
            self.s_owned_offset(t1),
        );
        let raw = self
            .raw_delta
            .map_or(0.0, |(axis, _)| axis_pva(axis, t1).0 - axis_pva(axis, t0).0);
        spans + raw
    }

    fn deriv(&self, t: f64) -> f64 {
        let raw = self.raw_delta.map_or(0.0, |(axis, _)| axis_pva(axis, t).1);
        self.path_deriv(t) + raw
    }

    fn second_deriv(&self, t: f64) -> f64 {
        let raw = self.raw_delta.map_or(0.0, |(axis, _)| axis_pva(axis, t).2);
        self.path_second_deriv(t) + raw
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        self.pva_memo.get_or(t, |t| self.pva_at(t))
    }
    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        let support = (self.grid[0], self.grid[self.grid.len() - 1]);
        self.whole_enclosure
            .over((lo, hi), accuracy, support, |(lo, hi), accuracy| {
                self.enclosure(lo, hi, accuracy)
            })
    }
    fn diagnostic(&self, t: f64) -> Option<String> {
        let s = self.s_owned(t);
        let span_index = self.state.spans.partition_point(|span| span.s1 <= s);
        let span = self.state.spans.get(span_index);
        let (speed, speed_deriv) = self.odometer_rate(t);
        let (ratio, slope) = self.ratio_and_slope(s);
        let leaders = self
            .leaders
            .axes
            .iter()
            .enumerate()
            .map(|(index, axis)| (axis_pva(axis, t), self.leaders.pva(index, t), axis.domain()))
            .collect::<Vec<_>>();
        let raw = self
            .raw_delta
            .map(|(axis, _)| (axis_pva(axis, t), axis.domain()));
        Some(format!(
            "follower t={t} s={s} span_index={span_index} span={span:?} \
             ratio={ratio} slope={slope} speed={speed} speed_deriv={speed_deriv} \
             leaders={leaders:?} raw={raw:?}"
        ))
    }
}

/// The follower signal with the advance applied, less the advance steps
/// inside the segment, which the assembly of the fitted tracks carries. At
/// a step's own time the advance reads the velocity before the step, so the
/// signal there is its left limit whichever side the follower evaluates —
/// at a leader knot it reads the piece before, past a ratio-span boundary
/// the span after.
struct AdvancedFollowerSignal<'a, 'b> {
    source: &'a FollowerSignal<'b>,
    advance: LeadingAdvance<'a>,
    steps: &'a [AdvanceStep],
    pva_memo: PvaMemo,
    /// The acceleration tolerance the advanced follower is fitted to.
    accel_tol: f64,
    unshaped_step: UnshapedLeaderStep,
    /// The enclosure over the whole signal, which every rung of the fit
    /// asks again.
    whole_enclosure: enclosure::WholeEnclosure,
}

impl<'a, 'b> AdvancedFollowerSignal<'a, 'b> {
    fn new(
        source: &'a FollowerSignal<'b>,
        advance: LeadingAdvance<'a>,
        steps: &'a [AdvanceStep],
        fit_tol: FitTol,
    ) -> Self {
        Self {
            source,
            advance,
            steps,
            pva_memo: PvaMemo::default(),
            accel_tol: fit_tol.accel_mm_s2,
            unshaped_step: UnshapedLeaderStep::of(
                advance,
                source.ratio_bounds.ratio,
                fit_tol.accel_mm_s2,
            ),
            whole_enclosure: enclosure::WholeEnclosure::default(),
        }
    }

    fn pva(&self, t: f64) -> (f64, f64, f64) {
        let read = self.source.path_read(t);
        let (p, v, a) = self.source.pva_read(t, &read);
        let (read_v, read_a, read_j) = self.advance.read(self.source, t, &read);
        let input = self.advance_input(t, read_v);
        let p = p - input.steps_taken_out;
        match self.advance.stage {
            ChainStage::DerivativeGains { k1, k2 } => {
                assert_eq!(*k2, 0.0, "leading acceleration gain is not supported");
                (p + k1 * input.velocity, v + k1 * read_a, a + k1 * read_j)
            }
            ChainStage::NonlinearAdvance(advance) => (
                p + advance.advance(input.velocity),
                v + advance.slope(read_v) * read_a,
                a + advance.slope(read_v) * read_j + advance.curvature(read_v) * read_a * read_a,
            ),
            ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
        }
    }
}

/// The last position, velocity and acceleration a signal evaluated, by
/// the bits of their time: the fit asks a signal again at times it has
/// probed.
struct PvaMemo(RefCell<[Option<(u64, (f64, f64, f64))>; PVA_MEMO_SLOTS]>);

impl Default for PvaMemo {
    fn default() -> Self {
        Self(RefCell::new([None; PVA_MEMO_SLOTS]))
    }
}

impl PvaMemo {
    fn get_or(&self, t: f64, eval: impl FnOnce(f64) -> (f64, f64, f64)) -> (f64, f64, f64) {
        let key = t.to_bits();
        let slot = ((key ^ key.rotate_right(32)) as usize) & (PVA_MEMO_SLOTS - 1);
        if let Some((stored_key, value)) = self.0.borrow()[slot] {
            if stored_key == key {
                return value;
            }
        }
        let value = eval(t);
        self.0.borrow_mut()[slot] = Some((key, value));
        value
    }
}

struct AdvanceInput {
    velocity: f64,
    steps_taken_out: f64,
}

impl AdvancedFollowerSignal<'_, '_> {
    /// Refuses a velocity change across any of the window's square-root
    /// onset slivers ([`UnshapedLeaderStep`]) its allowance does not admit:
    /// with no kernel after the advance, the motor would be asked to make
    /// it.
    fn refuse_onset_steps(
        &self,
        axis: usize,
        (t_start, t_end): (f64, f64),
    ) -> Result<(), PostProcessError> {
        for sliver in self.onset_slivers(t_start, t_end).unwrap_or_default() {
            if !self.unshaped_step.admits(sliver.singular_change) {
                return Err(PostProcessError::UnshapedLeaderOnsetVelocityStep {
                    axis,
                    from: sliver.from,
                    to: sliver.to,
                    change: sliver.singular_change,
                    allowance: self.unshaped_step.allowance(),
                });
            }
        }
        Ok(())
    }

    /// Refuses a velocity step wider than `resolution` at any of `times`
    /// inside the window: with no kernel after the advance, the motor
    /// would be asked to make it.
    fn refuse_velocity_steps(
        &self,
        axis: usize,
        times: &[f64],
        (t_start, t_end): (f64, f64),
        resolution: f64,
    ) -> Result<(), PostProcessError> {
        for &t in times.iter().filter(|&&t| t > t_start && t < t_end) {
            let before = self.eval_pva(trajectory::next_toward(t, t_start)).1;
            let after = self.eval_pva(trajectory::next_toward(t, t_end)).1;
            if (after - before).abs() > resolution {
                return Err(PostProcessError::RatioRampedVelocityStep {
                    axis,
                    t,
                    velocity_before: before,
                    velocity_after: after,
                    resolution,
                });
            }
        }
        Ok(())
    }

    fn advance_input(&self, t: f64, velocity: f64) -> AdvanceInput {
        self.steps.iter().take_while(|step| step.t <= t).fold(
            AdvanceInput {
                velocity,
                steps_taken_out: 0.0,
            },
            |input, step| {
                if step.t == t {
                    AdvanceInput {
                        velocity: step.velocity_before,
                        ..input
                    }
                } else {
                    AdvanceInput {
                        steps_taken_out: input.steps_taken_out + step.step_mm,
                        ..input
                    }
                }
            },
        )
    }
}

impl TrackSignal for AdvancedFollowerSignal<'_, '_> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        self.pva_memo.get_or(t, |t| self.pva(t))
    }

    fn exempt_accelerations(&self, lo: f64, hi: f64) -> Option<ExemptAccelerations> {
        let slivers = self.onset_slivers(lo, hi)?;
        (!slivers.is_empty()).then(|| ExemptAccelerations {
            stretches: slivers
                .iter()
                .map(|sliver| (sliver.from, sliver.to))
                .collect(),
            velocity: self.unshaped_step.allowance(),
        })
    }

    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        let grid = &self.source.grid;
        let support = (grid[0], grid[grid.len() - 1]);
        self.whole_enclosure
            .over((lo, hi), accuracy, support, |(lo, hi), accuracy| {
                self.enclosure(lo, hi, accuracy)
            })
    }

    fn position_delta(&self, (t0, _): (f64, f64), (t1, _): (f64, f64)) -> f64 {
        let delta = self.source.position_delta((t0, 0.0), (t1, 0.0));
        let start = self.advance_input(t0, self.advance.read_velocity(self.source, t0));
        let end = self.advance_input(t1, self.advance.read_velocity(self.source, t1));
        let (v0, v1) = (start.velocity, end.velocity);
        delta - (end.steps_taken_out - start.steps_taken_out)
            + match self.advance.stage {
                ChainStage::DerivativeGains { k1, .. } => k1 * (v1 - v0),
                ChainStage::NonlinearAdvance(advance) => advance.advance(v1) - advance.advance(v0),
                ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
            }
    }
}

/// Every emitted follower piece carries its own position origin: the fitted
/// polynomial keeps only the excursion it accumulates inside its own knot
/// span, and `base_position` holds the cumulative print position at the
/// piece's start. A long target no longer regains a large relative carrier —
/// the value a fitted curve would have to resolve against never exceeds one
/// piece's own travel — and the seams stay bit-exact because each origin is
/// the running sum of the preceding pieces' own increments.
pub(crate) fn localize_pieces(base: f64, pieces: &[BezierPiece]) -> Arc<[RelativeSplinePiece]> {
    let first = pieces.first().expect("follower emit has no fitted pieces");
    let mut origin = base + first.coeffs[0];
    let mut out = Vec::with_capacity(pieces.len());
    for piece in pieces {
        let mut local = piece.clone();
        local.coeffs[0] = 0.0;
        let travel = polynomial_pv(&local.coeffs, local.u_end - local.u_start).0;
        out.push(RelativeSplinePiece {
            base_position: origin,
            curve: Arc::new(bezier_pieces_to_nurbs(std::slice::from_ref(&local))),
            t_start: local.u_start,
            t_end: local.u_end,
        });
        origin += travel;
    }
    Arc::from(out)
}

fn piece_run_start_pv(pieces: &[BezierPiece]) -> (f64, f64) {
    let first = pieces.first().expect("a fitted target has pieces");
    polynomial_pv(&first.coeffs, 0.0)
}

fn piece_run_end_pv(pieces: &[BezierPiece]) -> (f64, f64) {
    let last = pieces.last().expect("a fitted target has pieces");
    polynomial_pv(&last.coeffs, last.u_end - last.u_start)
}
/// Every emitted follower track continues one the pipeline already handed
/// downstream, and adjacent stretches are fitted independently, so each one
/// opens a fit residual away from the state already emitted. Welding only
/// position leaves that residual's slope behind as a velocity step at the
/// seam — a batch boundary in the leading case, a committed segment boundary
/// inside the batch.
///
/// The unique cubic with `c(0) = dp`, `c'(0) = dv` and `c(L) = c'(L) = 0`
/// carries the whole correction: the stretch opens on the emitted position
/// *and* velocity, its settled total is untouched, and it closes on the
/// natural endpoint state, so nothing downstream inherits a residual ramp and
/// the correction cannot accumulate. `L` is the whole window unless a
/// velocity residual spread over it would displace the track past the
/// tolerance — `4·|dv|·L/27` grows with the window — in which case the weld
/// closes as early as that displacement allows.
#[derive(Clone, Copy)]
struct SeamWeld {
    origin: f64,
    length: f64,
    closes_inside: Option<f64>,
    coeffs: [f64; 4],
}

#[derive(Clone, Copy)]
struct WeldWindow {
    axis: usize,
    start: f64,
    end: f64,
    tol: FitTol,
}

const WELD_REACH_BISECTIONS: usize = 64;

/// A residual this many ulps of the emitted position is the seam's
/// rounding, whatever acceleration closing it over a sliver window reads as.
const WELD_ROUNDING_ULPS: f64 = 4.0;

/// The acceleration a weld spends closing [`WELD_ROUNDING_ULPS`] of
/// `position` over `length` (`6·dp/L²`): over a window of nanoseconds it
/// passes any budget, so a weld is refused only beyond it.
fn weld_rounding_acceleration(position: f64, length: f64) -> f64 {
    6.0 * WELD_ROUNDING_ULPS * f64::EPSILON * position.abs().max(1.0) / (length * length)
}

impl SeamWeld {
    fn spanning(
        window: WeldWindow,
        emitted: (f64, f64),
        law_step: f64,
        base_position: f64,
        pieces: &[BezierPiece],
    ) -> Result<Self, PostProcessError> {
        let WeldWindow {
            axis,
            start,
            end,
            tol,
        } = window;
        let length = end - start;
        assert!(
            length > 0.0,
            "follower weld window [{start}, {end}] has no length"
        );
        let (run_start, velocity_start) = piece_run_start_pv(pieces);
        let dp = emitted.0 - (base_position + run_start);
        let dv = emitted.1 + law_step - velocity_start;
        let whole = Self::over(start, length, (dp, dv));
        let weld = if whole.max_displacement() <= tol.pos_mm {
            whole
        } else {
            Self::closing_within(start, length, (dp, dv), tol.pos_mm).unwrap_or(whole)
        };
        let displacement = weld.max_displacement();
        let acceleration = weld.max_acceleration();
        let acceleration_budget = tol
            .accel_mm_s2
            .max(weld_rounding_acceleration(emitted.0, weld.length));
        if displacement > tol.pos_mm || acceleration > acceleration_budget {
            return Err(PostProcessError::SeamWeld {
                axis,
                t_start: start,
                t_end: weld.closes_inside.unwrap_or(end),
                displacement_mm: displacement,
                position_budget_mm: tol.pos_mm,
                acceleration_mm_s2: acceleration,
                acceleration_budget_mm_s2: tol.accel_mm_s2,
                emitted_position: emitted.0,
                emitted_velocity: emitted.1,
                fitted_position: base_position + run_start,
                fitted_velocity: velocity_start,
            });
        }
        Ok(weld)
    }

    fn over(origin: f64, length: f64, (dp, dv): (f64, f64)) -> Self {
        let cubic = (2.0 * dp + dv * length) / (length * length * length);
        let quadratic = -(3.0 * dp + 2.0 * dv * length) / (length * length);
        Self {
            origin,
            length,
            closes_inside: None,
            coeffs: [dp, dv, quadratic, cubic],
        }
    }

    /// The longest weld shorter than `window` that keeps its displacement
    /// within `budget`, if any length does.
    fn closing_within(origin: f64, window: f64, residual: (f64, f64), budget: f64) -> Option<Self> {
        let (mut fits, mut exceeds) = (0.0_f64, window);
        for _ in 0..WELD_REACH_BISECTIONS {
            let length = 0.5 * (fits + exceeds);
            if Self::over(origin, length, residual).max_displacement() <= budget {
                fits = length;
            } else {
                exceeds = length;
            }
        }
        (fits > 0.0).then(|| Self {
            closes_inside: Some(origin + fits),
            ..Self::over(origin, fits, residual)
        })
    }

    fn correction_at(&self, x: f64) -> f64 {
        let [c0, c1, c2, c3] = self.coeffs;
        c0 + x * (c1 + x * (c2 + x * c3))
    }

    fn max_displacement(&self) -> f64 {
        let length = self.length;
        let [_, c1, c2, c3] = self.coeffs;
        let (a, b, c) = (3.0 * c3, 2.0 * c2, c1);
        let mut candidates = vec![0.0, length];
        if a == 0.0 {
            if b != 0.0 {
                candidates.push(-c / b);
            }
        } else {
            let discriminant = b * b - 4.0 * a * c;
            if discriminant >= 0.0 {
                let root = discriminant.sqrt();
                candidates.push((-b + root) / (2.0 * a));
                candidates.push((-b - root) / (2.0 * a));
            }
        }
        candidates
            .into_iter()
            .filter(|&x| (0.0..=length).contains(&x))
            .map(|x| self.correction_at(x).abs())
            .fold(0.0, f64::max)
    }

    fn max_acceleration(&self) -> f64 {
        let [_, _, c2, c3] = self.coeffs;
        (2.0 * c2)
            .abs()
            .max((2.0 * c2 + 6.0 * c3 * self.length).abs())
    }

    /// `pieces` with the correction added over the weld, split where a
    /// weld closing inside the window ends.
    fn apply(&self, pieces: Vec<BezierPiece>) -> Vec<BezierPiece> {
        let Some(close) = self.closes_inside else {
            return pieces
                .into_iter()
                .map(|mut piece| {
                    self.raise(&mut piece);
                    piece
                })
                .collect();
        };
        let mut welded = Vec::with_capacity(pieces.len() + 1);
        for piece in pieces {
            if piece.u_start >= close {
                welded.push(piece);
            } else if piece.u_end > close {
                let (mut head, tail) = nurbs::bezier::split_piece_at(&piece, close);
                self.raise(&mut head);
                welded.push(head);
                welded.push(tail);
            } else {
                let mut piece = piece;
                self.raise(&mut piece);
                welded.push(piece);
            }
        }
        welded
    }

    fn raise(&self, piece: &mut BezierPiece) {
        if piece.coeffs.len() < 4 {
            piece.coeffs.resize(4, 0.0);
        }
        let d = piece.u_start - self.origin;
        let [c0, c1, c2, c3] = self.coeffs;
        piece.coeffs[0] += c0 + d * (c1 + d * (c2 + d * c3));
        piece.coeffs[1] += c1 + d * (2.0 * c2 + 3.0 * c3 * d);
        piece.coeffs[2] += c2 + 3.0 * c3 * d;
        piece.coeffs[3] += c3;
    }
}

/// Lifts every piece whose velocity dips under the fit's velocity resolution
/// back to a monotone one by projecting its velocity's Bernstein coefficients
/// onto the nonnegative ones. The velocity is read off the piece's own
/// derivative, never off differences of its positions, so the offset a piece
/// rides at (a batch's carried position) cannot change what it becomes, and
/// an end velocity within the resolution of zero is rest, not a retraction.
/// The projection may lift the piece by the deepest retreat it removes plus
/// the position tolerance, read with the offset taken out, and lower it by
/// the tolerance; a piece it would move further keeps its dip: clamping a
/// high-degree piece's Bernstein velocity and carrying the deficit forward
/// can lift a steep rise far above the source it corrects.
fn project_monotone(
    axis: usize,
    label: &str,
    pieces: &mut [nurbs::bezier::BezierPiece],
    tol: FitTol,
) {
    let velocity_resolution = tol.accel_mm_s2 * MIN_DEVICE_INTERVAL_S;
    let total_at_end = |pieces: &[nurbs::bezier::BezierPiece]| {
        pieces
            .last()
            .map(|piece| polynomial_pva(&piece.coeffs, piece.u_end - piece.u_start).0)
            .expect("follower piece run is non-empty")
    };
    let source_total = total_at_end(pieces);
    for piece in pieces.iter_mut() {
        let h = piece.u_end - piece.u_start;
        let degree = piece.degree();
        if h <= SEGMENT_TIME_EPS_S || degree < 3 {
            continue;
        }
        let probe_tau = |u: f64| 0.5 * (u + 1.0) * h;
        let sweep = 4 * degree;
        let dips = (0..=sweep).any(|step| {
            let tau = h * step as f64 / sweep as f64;
            polynomial_pva(&piece.coeffs, tau).1 < -velocity_resolution
        });
        if !dips {
            continue;
        }
        let mut derivative = piece_velocity(piece).to_bernstein();
        let ends = [0, degree - 1];
        if ends
            .iter()
            .any(|&end| derivative[end] < -velocity_resolution)
        {
            continue;
        }
        let mut perturbation = 0.0_f64;
        for end in ends {
            let rested = derivative[end].max(0.0);
            perturbation = perturbation.max(rested - derivative[end]);
            derivative[end] = rested;
        }
        let mut deficit = 0.0_f64;
        for rate in &mut derivative[1..degree - 1] {
            let available = *rate + deficit;
            let projected = available.max(0.0);
            deficit = available - projected;
            perturbation = perturbation.max((projected - *rate).abs());
            *rate = projected;
        }
        if deficit < 0.0 {
            continue;
        }
        let accel_envelope = tol.accel_mm_s2 + 2.0 * (degree - 1) as f64 * perturbation / h;
        let lift_budget = deepest_retreat(&piece.coeffs, h, sweep) + tol.pos_mm;
        let source = piece.clone();
        let corrected_piece = integrate_velocity(
            piece.coeffs[0],
            &nurbs::bezier::BezierPiece::from_bernstein(&derivative, piece.u_start, piece.u_end),
        );
        let within_budget = crate::lowering::LADDER_PROBES_U.iter().all(|&u| {
            let corrected = polynomial_pva(&corrected_piece.coeffs, probe_tau(u));
            let uncorrected = polynomial_pva(&source.coeffs, probe_tau(u));
            [corrected.0, corrected.1, corrected.2]
                .into_iter()
                .all(f64::is_finite)
                && (-tol.pos_mm..=lift_budget).contains(&(corrected.0 - uncorrected.0))
                && corrected.2.abs() <= uncorrected.2.abs() + accel_envelope
        });
        if within_budget {
            *piece = corrected_piece;
        }
    }
    let corrected_total = total_at_end(pieces);
    assert!(
        (corrected_total - source_total).abs() <= tol.pos_mm,
        "follower axis {axis} {label} monotone correction moved the total from \
         {source_total} to {corrected_total}"
    );
}

/// How far the polynomial `coeffs` falls below the highest it has reached,
/// at most, sampled `sweep` times over `[0, h]` with its offset taken out.
fn deepest_retreat(coeffs: &[f64], h: f64, sweep: usize) -> f64 {
    let mut rise = coeffs.to_vec();
    rise[0] = 0.0;
    (0..=sweep)
        .map(|step| polynomial_pva(&rise, h * step as f64 / sweep as f64).0)
        .fold((f64::NEG_INFINITY, 0.0_f64), |(peak, deepest), position| {
            let peak = peak.max(position);
            (peak, deepest.max(peak - position))
        })
        .1
}

fn piece_velocity(piece: &BezierPiece) -> BezierPiece {
    BezierPiece {
        u_start: piece.u_start,
        u_end: piece.u_end,
        coeffs: (1..piece.coeffs.len())
            .map(|power| power as f64 * piece.coeffs[power])
            .collect(),
    }
}

fn integrate_velocity(start_position: f64, velocity: &BezierPiece) -> BezierPiece {
    BezierPiece {
        u_start: velocity.u_start,
        u_end: velocity.u_end,
        coeffs: std::iter::once(start_position)
            .chain(
                velocity
                    .coeffs
                    .iter()
                    .enumerate()
                    .map(|(power, &coefficient)| coefficient / (power + 1) as f64),
            )
            .collect(),
    }
}

fn polynomial_pv(coeffs: &[f64], tau: f64) -> (f64, f64) {
    let (position, velocity, _) = polynomial_pva(coeffs, tau);
    (position, velocity)
}

/// `coeffs`, a polynomial in `τ`, as one in `τ − at`.
fn taylor_shifted(coeffs: &[f64], at: f64) -> Vec<f64> {
    let mut shifted = coeffs.to_vec();
    let degree = shifted.len().saturating_sub(1);
    for done in 0..degree {
        for index in (done..degree).rev() {
            shifted[index] += at * shifted[index + 1];
        }
    }
    shifted
}

fn polynomial_pva(coeffs: &[f64], tau: f64) -> (f64, f64, f64) {
    let (mut p, mut v, mut a) = (0.0_f64, 0.0_f64, 0.0_f64);
    for &coefficient in coeffs.iter().rev() {
        a = nurbs::fmadd(a, tau, v);
        v = nurbs::fmadd(v, tau, p);
        p = nurbs::fmadd(p, tau, coefficient);
    }
    (p, v, 2.0 * a)
}

/// One segment's fitted pieces, raised by `offset` and by every window step
/// at and after its time, each step's time recorded in `steps_at`.
fn with_window_steps(
    pieces: Vec<BezierPiece>,
    steps: &[AdvanceStep],
    offset: f64,
    steps_at: &mut Vec<(f64, f64)>,
) -> Vec<BezierPiece> {
    let mut pieces = split_at_steps(pieces, steps);
    let mut pending = steps.iter().peekable();
    let mut stepped = offset;
    for piece in &mut pieces {
        while let Some(step) = pending.next_if(|step| step.t <= piece.u_start) {
            assert_eq!(
                step.t, piece.u_start,
                "advance step at {} was not split onto a piece start",
                step.t
            );
            stepped += step.step_mm;
            steps_at.push((piece.u_start, step.step_mm));
        }
        piece.coeffs[0] += stepped;
    }
    assert!(
        pending.next().is_none(),
        "advance step past the end of its segment's pieces"
    );
    pieces
}

/// Splits the pieces that straddle a step time there, so every step lands
/// on a piece start: seed coalescing may have merged the step's seed into a
/// neighbour.
fn split_at_steps(pieces: Vec<BezierPiece>, steps: &[AdvanceStep]) -> Vec<BezierPiece> {
    let mut out = Vec::with_capacity(pieces.len() + steps.len());
    for mut piece in pieces {
        for step in steps {
            if step.t > piece.u_start && step.t < piece.u_end {
                let (head, tail) = nurbs::bezier::split_piece_at(&piece, step.t);
                out.push(head);
                piece = tail;
            }
        }
        out.push(piece);
    }
    out
}

fn polynomial_pvaj(coeffs: &[f64], tau: f64) -> (f64, f64, f64, f64) {
    let (mut p, mut v, mut a, mut j) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for &coefficient in coeffs.iter().rev() {
        j = nurbs::fmadd(j, tau, a);
        a = nurbs::fmadd(a, tau, v);
        v = nurbs::fmadd(v, tau, p);
        p = nurbs::fmadd(p, tau, coefficient);
    }
    (p, v, 2.0 * a, 6.0 * j)
}

fn next_lower_float(value: f64) -> f64 {
    if value == 0.0 {
        f64::from_bits((1_u64 << 63) | 1)
    } else if value > 0.0 {
        f64::from_bits(value.to_bits() - 1)
    } else {
        f64::from_bits(value.to_bits() + 1)
    }
}

fn span_alignment_envelope(s: f64) -> f64 {
    INTEGRAL_TOL_MM.max(ODOMETER_ALIGNMENT_ULPS * f64::EPSILON * s.abs())
}

fn basis_conversion_same(
    left: f64,
    right: f64,
    degree: usize,
    left_duration: f64,
    right_duration: f64,
) -> bool {
    let duration_condition = left_duration.max(right_duration) / left_duration.min(right_duration);
    if !duration_condition.is_finite() {
        return false;
    }
    let degree_terms = (degree + 1) as f64;
    (left - right).abs()
        <= BASIS_CONVERSION_ROUNDOFF_ENVELOPE
            * degree_terms
            * degree_terms
            * duration_condition
            * f64::EPSILON
            * left.abs().max(right.abs()).max(1.0)
}
/// Every seam of `pieces` must be position-continuous except where `steps`
/// lists the step it carries.
fn assert_piece_seams(
    axis: usize,
    label: &str,
    pieces: &[nurbs::bezier::BezierPiece],
    steps: &[(f64, f64)],
) {
    for (index, pair) in pieces.windows(2).enumerate() {
        let left_h = pair[0].u_end - pair[0].u_start;
        let right_h = pair[1].u_end - pair[1].u_start;
        let seam = pair[0].u_end;
        let step = steps
            .iter()
            .filter(|&&(t, _)| t == seam)
            .map(|&(_, step)| step)
            .sum::<f64>();
        let left = polynomial_pv(&pair[0].coeffs, left_h);
        let left = (left.0 + step, left.1);
        let right = polynomial_pv(&pair[1].coeffs, 0.0);
        let degree = pair[0].degree().max(pair[1].degree());
        assert!(
            basis_conversion_same(left.0, right.0, degree, left_h, right_h)
                && left.1.is_finite()
                && right.1.is_finite(),
            "follower axis {axis} {label} seam {index} at {seam}: left {left:?} \
             degree {} duration {left_h}, right {right:?} degree {} duration {right_h}",
            pair[0].degree(),
            pair[1].degree(),
        );
    }
}
/// The 15-point Gauss–Kronrod rule on `[-1, 1]`: its nodes from the end
/// in, the centre last, their Kronrod weights, and the weights of the
/// 7-point Gauss rule on the odd-indexed ones it embeds.
const KRONROD_NODES: [f64; 8] = [
    0.991_455_371_120_812_6,
    0.949_107_912_342_758_5,
    0.864_864_423_359_769_1,
    0.741_531_185_599_394_4,
    0.586_087_235_467_691_1,
    0.405_845_151_377_397_2,
    0.207_784_955_007_898_5,
    0.0,
];
const KRONROD_WEIGHTS: [f64; 8] = [
    0.022_935_322_010_529_225,
    0.063_092_092_629_978_55,
    0.104_790_010_322_250_18,
    0.140_653_259_715_525_92,
    0.169_004_726_639_267_9,
    0.190_350_578_064_785_4,
    0.204_432_940_075_298_9,
    0.209_482_141_084_727_83,
];
const GAUSS_WEIGHTS: [f64; 4] = [
    0.129_484_966_168_869_7,
    0.279_705_391_489_276_7,
    0.381_830_050_505_118_9,
    0.417_959_183_673_469_4,
];

const KRONROD_POINTS: usize = 15;

/// The Kronrod nodes on `[-1, 1]` in ascending order.
fn kronrod_abscissae() -> [f64; KRONROD_POINTS] {
    let mut abscissae = [0.0; KRONROD_POINTS];
    for (index, &node) in KRONROD_NODES[..7].iter().enumerate() {
        abscissae[index] = -node;
        abscissae[KRONROD_POINTS - 1 - index] = node;
    }
    abscissae
}

/// `f` at the Kronrod nodes of `[a, b]`, in ascending order.
fn kronrod_samples(f: &impl Fn(f64) -> f64, a: f64, b: f64) -> [f64; KRONROD_POINTS] {
    let centre = 0.5 * (a + b);
    let half = 0.5 * (b - a);
    let mut samples = [0.0; KRONROD_POINTS];
    samples[7] = f(centre);
    for (index, &node) in KRONROD_NODES[..7].iter().enumerate() {
        let offset = half * node;
        samples[index] = f(centre - offset);
        samples[KRONROD_POINTS - 1 - index] = f(centre + offset);
    }
    samples
}

/// The Kronrod and the embedded Gauss estimates of `∫f` over an interval
/// `2·half` long, from `f` at its Kronrod nodes.
fn kronrod_and_gauss(samples: &[f64; KRONROD_POINTS], half: f64) -> (f64, f64) {
    let mut kronrod = KRONROD_WEIGHTS[7] * samples[7];
    let mut gauss = GAUSS_WEIGHTS[3] * samples[7];
    for (index, &weight) in KRONROD_WEIGHTS[..7].iter().enumerate() {
        let pair = samples[index] + samples[KRONROD_POINTS - 1 - index];
        kronrod += weight * pair;
        if index % 2 == 1 {
            gauss += GAUSS_WEIGHTS[index / 2] * pair;
        }
    }
    (kronrod * half, gauss * half)
}

/// Adaptive Gauss–Kronrod: `rule` estimates `[a, b]` under its share of
/// the tolerance and returns a leaf once it converges, or unconditionally
/// when told it is the last chance; each leaf is handed to `leaf` with its
/// end, in order. Simpson's halves can agree by chance on a speed that
/// bends often, Kronrod and Gauss all but never do.
fn adaptive_gauss_kronrod<L>(
    rule: &impl Fn(f64, f64, f64, bool) -> Option<L>,
    (a, b): (f64, f64),
    tol: f64,
    depth: u32,
    leaf: &mut impl FnMut(f64, L),
) {
    if let Some(converged) = rule(a, b, tol, depth == 0) {
        leaf(b, converged);
        return;
    }
    let middle = 0.5 * (a + b);
    adaptive_gauss_kronrod(rule, (a, middle), 0.5 * tol, depth - 1, leaf);
    adaptive_gauss_kronrod(rule, (middle, b), 0.5 * tol, depth - 1, leaf);
}

fn integrate(f: &impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    let mut total = 0.0;
    if b - a > 0.0 {
        adaptive_gauss_kronrod(
            &|a, b, tol, last_chance| {
                let (kronrod, gauss) = kronrod_and_gauss(&kronrod_samples(f, a, b), 0.5 * (b - a));
                (last_chance || (kronrod - gauss).abs() <= tol).then_some(kronrod)
            },
            (a, b),
            INTEGRAL_TOL_MM,
            INTEGRAL_MAX_DEPTH,
            &mut |_, value| total += value,
        );
    }
    total
}

/// Maps `f` at the Kronrod nodes to the Chebyshev coefficients of the
/// degree-14 polynomial through them.
static KRONROD_INTERPOLATION: LazyLock<[[f64; KRONROD_POINTS]; KRONROD_POINTS]> =
    LazyLock::new(|| {
        let abscissae = kronrod_abscissae();
        let mut vandermonde = [[0.0; KRONROD_POINTS]; KRONROD_POINTS];
        for (row, &x) in vandermonde.iter_mut().zip(&abscissae) {
            row[0] = 1.0;
            row[1] = x;
            for degree in 2..KRONROD_POINTS {
                row[degree] = 2.0 * x * row[degree - 1] - row[degree - 2];
            }
        }
        inverse(vandermonde)
    });

/// Gauss–Jordan inversion with partial pivoting; panics on a singular
/// matrix.
fn inverse<const N: usize>(mut matrix: [[f64; N]; N]) -> [[f64; N]; N] {
    let mut inverse = [[0.0; N]; N];
    for (index, row) in inverse.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    for column in 0..N {
        let pivot = (column..N)
            .max_by(|&left, &right| {
                matrix[left][column]
                    .abs()
                    .total_cmp(&matrix[right][column].abs())
            })
            .expect("a column below the diagonal is never empty");
        assert!(matrix[pivot][column] != 0.0, "inverting a singular matrix");
        matrix.swap(column, pivot);
        inverse.swap(column, pivot);
        let scale = 1.0 / matrix[column][column];
        for entry in 0..N {
            matrix[column][entry] *= scale;
            inverse[column][entry] *= scale;
        }
        for row in 0..N {
            if row == column {
                continue;
            }
            let factor = matrix[row][column];
            for entry in 0..N {
                matrix[row][entry] -= factor * matrix[column][entry];
                inverse[row][entry] -= factor * inverse[column][entry];
            }
        }
    }
    inverse
}

/// `Σ coefficients[k]·T_k(x)` by Clenshaw's recurrence.
fn chebyshev_sum(coefficients: &[f64], x: f64) -> f64 {
    let (mut next, mut after) = (0.0, 0.0);
    for &coefficient in coefficients[1..].iter().rev() {
        (next, after) = (2.0 * x * next - after + coefficient, next);
    }
    x * next - after + coefficients[0]
}

/// One leaf of the odometer: the integral, from the leaf's start, of the
/// polynomial through the leaders' speed at its Kronrod nodes. Its total is
/// the Kronrod estimate, and it grows at that polynomial's value, which the
/// leaf only accepts where it stays on the leaders' speed: the follower's
/// position must follow its velocity for its fit to close.
#[derive(Debug, Clone, Copy)]
struct OdometerLeaf {
    centre: f64,
    half: f64,
    covered: [f64; KRONROD_POINTS + 1],
}

impl OdometerLeaf {
    fn through(samples: &[f64; KRONROD_POINTS], a: f64, b: f64) -> (Self, [f64; KRONROD_POINTS]) {
        let mut rate = [0.0; KRONROD_POINTS];
        for (coefficient, row) in rate.iter_mut().zip(KRONROD_INTERPOLATION.iter()) {
            *coefficient = row
                .iter()
                .zip(samples)
                .map(|(weight, sample)| weight * sample)
                .sum();
        }
        let mut covered = [0.0; KRONROD_POINTS + 1];
        let beyond = |degree: usize| rate.get(degree).copied().unwrap_or(0.0);
        covered[1] = rate[0] - 0.5 * beyond(2);
        for degree in 2..=KRONROD_POINTS {
            covered[degree] = (rate[degree - 1] - beyond(degree + 1)) / (2.0 * degree as f64);
        }
        covered[0] = -(1..=KRONROD_POINTS)
            .map(|degree| {
                if degree % 2 == 0 {
                    covered[degree]
                } else {
                    -covered[degree]
                }
            })
            .sum::<f64>();
        let leaf = Self {
            centre: 0.5 * (a + b),
            half: 0.5 * (b - a),
            covered,
        };
        (leaf, rate)
    }

    /// The distance the leaders cover from the leaf's start to `t`: never
    /// negative, though the Chebyshev sum rounds to a few ulps below zero
    /// next to the start, where the signal would read the table before
    /// its odometer's start.
    fn covered_until(&self, t: f64) -> f64 {
        let x = ((t - self.centre) / self.half).clamp(-1.0, 1.0);
        (self.half * chebyshev_sum(&self.covered, x)).max(0.0)
    }
}

/// Where a leaf's rate is checked against the leaders' speed: midway
/// between each pair of neighbouring Kronrod nodes, and between the
/// outermost ones and the leaf's ends. Not at the ends themselves, where
/// the leaders may cross a seam whose velocity steps by its fit's
/// rounding.
static ODOMETER_RATE_CHECKS: LazyLock<[f64; KRONROD_POINTS + 1]> = LazyLock::new(|| {
    let mut bounded = [0.0; KRONROD_POINTS + 2];
    bounded[0] = -1.0;
    bounded[KRONROD_POINTS + 1] = 1.0;
    bounded[1..=KRONROD_POINTS].copy_from_slice(&kronrod_abscissae());
    let mut checks = [0.0; KRONROD_POINTS + 1];
    for (check, pair) in checks.iter_mut().zip(bounded.windows(2)) {
        *check = 0.5 * (pair[0] + pair[1]);
    }
    checks
});

/// The path `speed` covers over `[a, b]`, leaf by leaf, each converged on
/// its total and on its rate. Where the leaders' speed bends sharply the
/// total alone converges on intervals over which a leaf's rate runs
/// tenths of a micrometre per second off the speed.
fn odometer_leaves(
    speed: &impl Fn(f64) -> f64,
    a: f64,
    b: f64,
    leaf: &mut impl FnMut(f64, OdometerLeaf),
) {
    if b - a <= 0.0 {
        return;
    }
    adaptive_gauss_kronrod(
        &|a, b, tol, last_chance| {
            let samples = kronrod_samples(speed, a, b);
            let half = 0.5 * (b - a);
            let (kronrod, gauss) = kronrod_and_gauss(&samples, half);
            if !last_chance && (kronrod - gauss).abs() > tol {
                return None;
            }
            let (odometer_leaf, rate) = OdometerLeaf::through(&samples, a, b);
            let centre = 0.5 * (a + b);
            let follows_speed = ODOMETER_RATE_CHECKS.iter().all(|&x| {
                (chebyshev_sum(&rate, x) - speed(centre + half * x)).abs() <= ODOMETER_RATE_TOL_MM_S
            });
            (last_chance || follows_speed).then_some(odometer_leaf)
        },
        (a, b),
        INTEGRAL_TOL_MM,
        INTEGRAL_MAX_DEPTH,
        leaf,
    );
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod projection_fuzz;
