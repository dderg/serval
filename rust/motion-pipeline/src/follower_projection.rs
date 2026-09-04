use std::cell::RefCell;
use std::sync::Arc;

use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{
    AxisChainSet, ChainStage, CompiledChain, ContinuousAxis, ContinuousSegment,
    RelativeSplinePiece, ShapedSignal,
};

use crate::lowering::{FitTol, follower_tol_scale};
use crate::shaper::{
    AxisSignalTable, SEGMENT_TIME_EPS_S, ShiftedTrackSignal, TrackSignal, analytic_phase_boundary,
    apply_derivative_gains_to_track, apply_nonlinear_advance_to_track, fit_axis_from_signal,
    shaped_signal_breakpoints,
};
use crate::types::PostProcessError;

const INTEGRAL_TOL_MM: f64 = 1e-10;
const INTEGRAL_MAX_DEPTH: u32 = 24;
const GRID_DEDUP_EPS_S: f64 = 1e-12;
const SPAN_MIN_LEN_MM: f64 = 1e-12;
const SPAN_LOOKUP_SLACK_MM: f64 = 1e-6;
const PVA_MEMO_SLOTS: usize = 64;
const BASIS_CONVERSION_ROUNDOFF_ENVELOPE: f64 = (1_u64 << 20) as f64;
const ODOMETER_ALIGNMENT_ULPS: f64 = (1_u64 << 12) as f64;
const SPEED_ZERO_SLIVER_FRACTION: f64 = 1e-6;
const COMPONENT_ZERO_PROBES: usize = 16;
fn follower_fit_tol(fit_tol: FitTol, position_scale: f64) -> FitTol {
    FitTol {
        pos_mm: fit_tol.pos_mm * position_scale,
        accel_mm_s2: fit_tol.accel_mm_s2,
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
}

fn fit_source_projection(
    shaped: &ContinuousSegment,
    raw: &ContinuousSegment,
    axis: usize,
    leaders: &[usize],
    state: &FollowerState,
    s_start: f64,
    fit_tol: FitTol,
) -> Result<SourceProjection, PostProcessError> {
    let raw_axis = &raw.axes[axis];
    let (t_start, t_end) = projection_support(raw, shaped, axis, leaders);
    let sig = FollowerSignal::new(shaped, raw, axis, leaders, state, s_start, 0.0);
    let breakpoints = sig.construction_breakpoints(raw_axis);
    let track = match sig.constant_value() {
        Some(value) => bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![value, value],
        }]),
        None => fit_axis_from_signal(
            axis,
            t_start,
            t_end,
            &breakpoints.fit_seeds,
            &sig,
            follower_fit_tol(fit_tol, follower_tol_scale(&raw.followers, axis) * 0.5),
            "follower_source",
        )?,
    };
    let e_end_relative = nurbs::eval::eval(&track.as_view(), t_end);
    Ok(SourceProjection {
        track,
        s_end: sig.s_end(),
        e_end_relative,
        semantic_cuts: breakpoints.semantic,
    })
}

pub(crate) fn project_followers(
    base: &[ContinuousSegment],
    frontier: &[ContinuousSegment],
    out: &mut [ContinuousSegment],
    commit_count: usize,
    force: bool,
    chains: &AxisChainSet,
    fit_tol: FitTol,
    states: &mut Vec<FollowerState>,
    timing: &mut ProjectionTiming,
) -> Result<(), PostProcessError> {
    assert!(frontier.len() >= commit_count && out.len() == commit_count);
    if states.len() < chains.n_axes() {
        states.resize_with(chains.n_axes(), FollowerState::default);
    }
    for (axis, leaders) in chains.projected_followers() {
        let chain = &chains.chains[axis];
        let kernel = chain.stages.iter().find_map(|stage| match stage {
            ChainStage::SmoothKernel(kernel) => Some(kernel),
            ChainStage::DerivativeGains { .. } | ChainStage::NonlinearAdvance(_) => None,
        });
        let defer_linear_prefix = kernel.is_some()
            && chain.stages.iter().all(|stage| {
                matches!(
                    stage,
                    ChainStage::DerivativeGains { .. } | ChainStage::SmoothKernel(_)
                )
            });
        let leaders_transformed = leaders.iter().any(|&leader| {
            chains
                .chains
                .get(leader)
                .is_some_and(CompiledChain::is_zero_support_only)
        });
        let leaders_changed = leaders_transformed
            || base.iter().zip(frontier).any(|(raw, shaped)| {
                leaders
                    .iter()
                    .any(|&leader| raw.axes[leader] != shaped.axes[leader])
            });
        let state = &mut states[axis];
        let projecting = leaders_changed || state.active;
        if !projecting && chain.is_empty() {
            continue;
        }
        let mut source_jobs = Vec::new();
        if projecting {
            state.active = true;
            let ingest_started = crate::timing::stopwatch();
            let first_new = state.ingested_through_t.map_or(0, |through| {
                base[..frontier.len()]
                    .partition_point(|raw| raw.t_end <= through + GRID_DEDUP_EPS_S)
            });
            let lengths = leader_arc_lengths(
                &base[first_new..frontier.len()],
                &frontier[first_new..],
                leaders,
            );
            timing.ingests += lengths.len();
            let mut next_s = state.s_shaped;
            for (offset, &arc) in lengths.iter().enumerate() {
                let i = first_new + offset;
                let shaped_ds =
                    state.ingest(&base[i], &frontier[i], axis, leaders, &base[i + 1..], arc);
                let s_start = state.aligned_span_start(next_s);
                source_jobs.push((i, s_start));
                next_s = s_start + shaped_ds;
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
        let mut source_fits = if workers > 1 {
            let next_job = std::sync::atomic::AtomicUsize::new(0);
            std::thread::scope(|scope| {
                let next_job = &next_job;
                let source_jobs = &source_jobs;
                let handles = (0..workers)
                    .map(|_| {
                        scope.spawn(move || {
                            let mut done = Vec::new();
                            loop {
                                let job =
                                    next_job.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                let Some(&(index, s_start)) = source_jobs.get(job) else {
                                    return done;
                                };
                                done.push((
                                    index,
                                    fit_source_projection(
                                        &frontier[index],
                                        &base[index],
                                        axis,
                                        leaders,
                                        state_ref,
                                        s_start,
                                        fit_tol,
                                    ),
                                ));
                            }
                        })
                    })
                    .collect::<Vec<_>>();
                handles
                    .into_iter()
                    .flat_map(|handle| handle.join().expect("follower source fit thread panicked"))
                    .collect::<Vec<_>>()
            })
        } else {
            source_jobs
                .iter()
                .map(|&(index, s_start)| {
                    (
                        index,
                        fit_source_projection(
                            &frontier[index],
                            &base[index],
                            axis,
                            leaders,
                            state_ref,
                            s_start,
                            fit_tol,
                        ),
                    )
                })
                .collect()
        };
        source_fits.sort_by_key(|(index, _)| *index);
        timing.source_fit_us += source_started.elapsed_us();
        let mut source_fits = source_fits.into_iter();
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
            let raw_axis = &raw.axes[axis];
            let (t_start, t_end) = projection_support(raw, &frontier[i], axis, leaders);
            assert!(
                t_start < t_end,
                "follower axis {axis} has no support in segment"
            );
            let base_position = state.e_end.unwrap_or_else(|| axis_pva(raw_axis, t_start).0);
            let (projected, projected_cuts) = if projecting {
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
                } = source_result?;
                state.s_shaped = s_end;
                state.e_end = Some(base_position + e_end_relative);
                (track, Some(semantic_cuts))
            } else {
                (
                    fit_continuous_axis(axis, raw_axis, base_position, t_start, t_end, fit_tol)?,
                    None,
                )
            };
            let input_start = pvaj_of_track(&projected, t_start);
            let input_end = pvaj_of_track(&projected, t_end);
            let track = apply_leading_stages(
                chain,
                axis,
                projected,
                follower_fit_tol(fit_tol, follower_tol_scale(&raw.followers, axis)),
                defer_linear_prefix,
            )?;
            if !track.control_points().iter().all(|v| v.is_finite()) {
                return Err(PostProcessError::NonFiniteSample {
                    axis,
                    t: raw.t_start,
                });
            }
            let semantic_cuts = projected_cuts.unwrap_or_else(|| piece_boundaries(&track));
            let track_start = nurbs::eval::eval(&track.as_view(), t_start);
            let output_base = state.projected_output_end.unwrap_or(base_position) - track_start;
            state.projected_output_end =
                Some(output_base + nurbs::eval::eval(&track.as_view(), t_end));
            state.projected_through_t = Some(t_end);
            state.projected.push(ProjSeg {
                t_start,
                t_end,
                base: output_base,
                input_start,
                input_end,
                track,
                semantic_cuts,
            });
        }
        assert!(
            source_fits.next().is_none(),
            "follower source fit has no matching projection segment"
        );
        let kernel_tracks = match kernel {
            Some(kernel) if commit_count > 0 => {
                let first = state.projected.first().expect("cache covers commits");
                let last = state.projected.last().expect("cache covers commits");
                let (first_t, last_t) = (first.t_start, last.t_end);
                let mut batch_base = first.base;
                let supports = (0..commit_count)
                    .map(|i| projection_support(&base[i], &frontier[i], axis, leaders))
                    .collect::<Vec<_>>();
                let target_start = supports.first().expect("commit_count > 0").0;
                let target_end = supports.last().expect("commit_count > 0").1;
                let (k_lo, k_hi) = kernel.support();
                let need_lo = target_start - k_hi;
                let need_hi = target_end - k_lo;
                if need_lo < first_t && state.projected_trimmed {
                    return Err(PostProcessError::MissingHistory { axis, t: need_lo });
                }
                if need_hi > last_t && !force {
                    return Err(PostProcessError::MissingLookahead { axis, t: need_hi });
                }
                let mut input_pieces = Vec::new();
                let mut semantic_cuts = Vec::new();
                let mut carried = 0.0;
                let mut input_end = None;
                for segment in &state.projected {
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
                    let mut pieces = extract_bezier_pieces(&segment.track);
                    let head = pieces.first().expect("a projected track has pieces").coeffs[0];
                    let segment_offset = input_end.map_or(0.0, |_| carried - head);
                    for piece in &mut pieces {
                        piece.coeffs[0] += segment_offset;
                    }
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
                let nonnegative_demand = state
                    .spans
                    .iter()
                    .all(|span| span.r0 >= 0.0 && span.r1 >= 0.0);
                if projecting {
                    assert_piece_seams(axis, "projected input", &input_pieces);
                    if nonnegative_demand {
                        let correction_tol = follower_fit_tol(
                            fit_tol,
                            base.iter()
                                .map(|segment| follower_tol_scale(&segment.followers, axis))
                                .fold(1.0, f64::min),
                        );
                        project_monotone(
                            axis,
                            "projected input",
                            &mut input_pieces,
                            correction_tol,
                        );
                    }
                }
                let unified_input_degree = input_pieces
                    .iter()
                    .map(|piece| piece.degree())
                    .max()
                    .expect("projected follower input is empty");
                for piece in &mut input_pieces {
                    piece.coeffs.resize(unified_input_degree + 1, 0.0);
                }
                let mut kernel_input = bezier_pieces_to_nurbs(&input_pieces);
                let input_offset = nurbs::eval::eval(&kernel_input.as_view(), target_start);
                for piece in &mut input_pieces {
                    piece.coeffs[0] -= input_offset;
                }
                kernel_input = bezier_pieces_to_nurbs(&input_pieces);
                batch_base += input_offset;
                let mut gained_input = chain
                    .stages
                    .iter()
                    .take_while(|stage| !matches!(stage, ChainStage::SmoothKernel(_)))
                    .any(|stage| !matches!(stage, ChainStage::SmoothKernel(_)));
                if defer_linear_prefix {
                    for stage in &chain.stages {
                        if let ChainStage::DerivativeGains { k1, k2 } = stage {
                            kernel_input = apply_derivative_gains_to_track(&kernel_input, *k1, *k2);
                            gained_input = true;
                        }
                    }
                }
                let gained_pieces = extract_bezier_pieces(&kernel_input);
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
                let table = Arc::new(
                    AxisSignalTable::from_tracks(
                        std::iter::once(&kernel_input),
                        first_t,
                        last_t,
                        !state.projected_trimmed,
                        force,
                    )
                    .with_piece_moments(kernel_degree),
                );
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
                let shaped_breaks = shaped_signal_breakpoints(kernel, shaped_break_seeds);
                timing.breakpoints_us += breaks_started.elapsed_us();
                let make_sig = || {
                    let eval_table = Arc::clone(&table);
                    let moment_table = Arc::clone(&table);
                    ShapedSignal::new_from_polynomial_evaluator(
                        kernel,
                        move |t| eval_table.eval(t),
                        exact_input_breaks.clone(),
                        input_degree,
                        move |lo, hi, degree, origin, moments| {
                            moment_table.integrate_moments(lo, hi, degree, origin, moments)
                        },
                    )
                };
                let tol_scale = (0..commit_count)
                    .map(|i| follower_tol_scale(&base[i].followers, axis))
                    .fold(1.0, f64::min);
                let target_tol = follower_fit_tol(fit_tol, tol_scale);
                let monotone_output = nonnegative_demand && !gained_input;
                let kernel_started = crate::timing::stopwatch();
                timing.kernel_fits += supports.len();
                let workers = if cfg!(target_arch = "wasm32") {
                    1
                } else {
                    std::thread::available_parallelism().map_or(1, |cores| {
                        cores.get().saturating_sub(1).max(1).min(supports.len())
                    })
                };
                let mut fitted: Vec<(
                    usize,
                    Result<(f64, Vec<BezierPiece>, u128), PostProcessError>,
                )> = if workers > 1 {
                    let next_window = std::sync::atomic::AtomicUsize::new(0);
                    std::thread::scope(|scope| {
                        let next_window = &next_window;
                        let supports = &supports;
                        let make_sig = &make_sig;
                        let shaped_breaks = &shaped_breaks;
                        let handles: Vec<_> = (0..workers)
                            .map(|_| {
                                scope.spawn(move || {
                                    let sig = make_sig();
                                    let mut done = Vec::new();
                                    loop {
                                        let index = next_window
                                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                        let Some(&(start, end)) = supports.get(index) else {
                                            return done;
                                        };
                                        done.push((
                                            index,
                                            fit_kernel_window(
                                                axis,
                                                start,
                                                end,
                                                shaped_breaks,
                                                &sig,
                                                target_tol,
                                                monotone_output,
                                            ),
                                        ));
                                    }
                                })
                            })
                            .collect();
                        handles
                            .into_iter()
                            .flat_map(|handle| {
                                handle.join().expect("follower kernel fit thread panicked")
                            })
                            .collect()
                    })
                } else {
                    let sig = make_sig();
                    supports
                        .iter()
                        .enumerate()
                        .map(|(index, &(start, end))| {
                            (
                                index,
                                fit_kernel_window(
                                    axis,
                                    start,
                                    end,
                                    &shaped_breaks,
                                    &sig,
                                    target_tol,
                                    monotone_output,
                                ),
                            )
                        })
                        .collect()
                };
                fitted.sort_by_key(|(index, _)| *index);
                let mut bases = Vec::with_capacity(supports.len());
                let mut tracks: Vec<Vec<BezierPiece>> = Vec::with_capacity(supports.len());
                for (_, result) in fitted {
                    let (local_base, pieces, window_us) = result?;
                    timing.kernel_fit_max_us = timing.kernel_fit_max_us.max(window_us);
                    bases.push(local_base);
                    tracks.push(pieces);
                }
                timing.kernel_fit_us += kernel_started.elapsed_us();
                bases[0] -= piece_run_start_pv(&tracks[0]).0;
                for i in 1..tracks.len() {
                    bases[i] = bases[i - 1] + piece_run_end(&tracks[i - 1])
                        - piece_run_start_pv(&tracks[i]).0;
                }
                for (i, pair) in tracks.windows(2).enumerate() {
                    let t = supports[i].1;
                    let left_piece = pair[0].last().expect("a fitted target has pieces");
                    let right_piece = pair[1].first().expect("a fitted target has pieces");
                    let left = polynomial_pv(&left_piece.coeffs, t - left_piece.u_start);
                    let right = polynomial_pv(&right_piece.coeffs, t - right_piece.u_start);
                    assert!(
                        [left.0, left.1, right.0, right.1]
                            .into_iter()
                            .all(f64::is_finite)
                            && basis_conversion_same(
                                bases[i] + left.0,
                                bases[i + 1] + right.0,
                                left_piece.degree().max(right_piece.degree()),
                                supports[i].1 - supports[i].0,
                                supports[i + 1].1 - supports[i + 1].0,
                            ),
                        "follower axis {axis} target seam {i} at {t}: left {left:?} over base \
                         {}, right {right:?} over base {}",
                        bases[i],
                        bases[i + 1]
                    );
                }
                Some((batch_base, bases, tracks))
            }
            _ => None,
        };
        let mut emitted_pv = state.emitted_output_pv;
        let mut committed_input_end = state.committed_input_end;
        let emit_window = (commit_count > 0).then(|| {
            (
                projection_support(&base[0], &frontier[0], axis, leaders).0,
                projection_support(
                    &base[commit_count - 1],
                    &frontier[commit_count - 1],
                    axis,
                    leaders,
                )
                .1,
            )
        });
        let mut batch_weld: Option<SeamWeld> = None;
        for i in 0..commit_count {
            let raw = &base[i];
            let (t_start, t_end) = projection_support(raw, &frontier[i], axis, leaders);
            let cached = state.cached_projection(t_start, t_end);
            let (input_start, input_end) = (cached.input_start, cached.input_end);
            let (base_position, mut pieces) = kernel_tracks.as_ref().map_or_else(
                || (cached.base, extract_bezier_pieces(&cached.track)),
                |(batch_base, bases, tracks)| (*batch_base + bases[i], tracks[i].clone()),
            );
            let law_step = |previous_input: Option<Pvaj4>| {
                if kernel.is_some() {
                    return 0.0;
                }
                let previous = previous_input
                    .expect("an emitted endpoint always records its pre-transform state");
                let shared_v = previous.1;
                let (_, _, a_r, j_r) = input_start;
                let (_, _, a_l, j_l) = previous;
                chain_output_velocity(chain, shared_v, a_r, j_r)
                    - chain_output_velocity(chain, shared_v, a_l, j_l)
            };
            if i == 0 {
                if let (Some(previous), Some((emit_start, emit_end))) = (emitted_pv, emit_window) {
                    batch_weld = Some(SeamWeld::spanning(
                        emit_start,
                        emit_end,
                        previous,
                        law_step(committed_input_end),
                        base_position,
                        &pieces,
                    ));
                }
            }
            if let Some(weld) = batch_weld {
                for piece in &mut pieces {
                    weld.apply(piece);
                }
            }
            if i > 0 {
                let previous = emitted_pv.expect("an earlier commit emitted its endpoint state");
                let weld = SeamWeld::spanning(
                    t_start,
                    t_end,
                    previous,
                    law_step(committed_input_end),
                    base_position,
                    &pieces,
                );
                for piece in &mut pieces {
                    weld.apply(piece);
                }
            }
            committed_input_end = Some(input_end);
            let (run_end, velocity_end) = piece_run_end_pv(&pieces);
            emitted_pv = Some((base_position + run_end, velocity_end));
            Arc::make_mut(&mut out[i].axes)[axis] =
                ContinuousAxis::PiecewiseRelativeSpline(localize_pieces(base_position, &pieces));
        }
        state.emitted_output_pv = emitted_pv;
        state.committed_input_end = committed_input_end;
        if commit_count > 0 {
            let emitted_through = base[commit_count - 1].t_end;
            let back = kernel.map_or(0.0, |k| k.support().1.max(0.0));
            state.trim_projected(emitted_through - back);
        }
        if projecting {
            state.prune_spans();
        }
    }
    Ok(())
}

fn fit_kernel_window<S: TrackSignal>(
    axis: usize,
    start: f64,
    end: f64,
    shaped_breaks: &[f64],
    sig: &S,
    target_tol: FitTol,
    monotone_output: bool,
) -> Result<(f64, Vec<BezierPiece>, u128), PostProcessError> {
    let started = crate::timing::stopwatch();
    let local_base = TrackSignal::eval(sig, start);
    let local = ShiftedTrackSignal::new(sig, local_base);
    let track = fit_axis_from_signal(
        axis,
        start,
        end,
        shaped_breaks,
        &local,
        target_tol,
        "follower_kernel",
    )?;
    let mut pieces = extract_bezier_pieces(&track);
    if monotone_output {
        project_monotone(axis, "shaped output", &mut pieces, target_tol);
    }
    Ok((local_base, pieces, started.elapsed_us()))
}

type Pvaj4 = (f64, f64, f64, f64);

fn pvaj_of_track(track: &ScalarNurbs, t: f64) -> Pvaj4 {
    let mut state = [nurbs::eval::eval(&track.as_view(), t), 0.0, 0.0, 0.0];
    let mut current = track.clone();
    for slot in state.iter_mut().skip(1) {
        if current.degree() == 0 {
            break;
        }
        current = nurbs::eval::derivative(&current);
        *slot = nurbs::eval::eval(&current.as_view(), t);
    }
    (state[0], state[1], state[2], state[3])
}

/// The transformed output velocity the pre-kernel chain commands for a
/// one-sided input `(a, j)` at a shared seam velocity `v` — the seam-side law
/// value, so a weld can tell the law's own velocity step (driven by the input
/// acceleration jumping across the seam) apart from the pipeline's fit
/// residual (which lives in `v` and stays welded).
fn chain_output_velocity(chain: &CompiledChain, v: f64, a: f64, j: f64) -> f64 {
    let (mut v, mut a, j) = (v, a, j);
    for stage in &chain.stages {
        match stage {
            ChainStage::SmoothKernel(_) => break,
            ChainStage::DerivativeGains { k1, k2 } => {
                let (nv, na) = (v + k1 * a + k2 * j, a + k1 * j);
                (v, a) = (nv, na);
            }
            ChainStage::NonlinearAdvance(adv) => {
                let (nv, na) = (
                    v + adv.slope(v) * a,
                    adv.curvature(v) * a * a + adv.slope(v) * j + a,
                );
                (v, a) = (nv, na);
            }
        }
    }
    let _ = (a, j);
    v
}

/// The chain stages ahead of the follower's kernel (all of them when it has
/// none), applied to the projected track — the convolution's input, matching
/// the leader convention where pre-kernel stages bake into the raw track.
fn apply_leading_stages(
    chain: &CompiledChain,
    axis: usize,
    mut track: ScalarNurbs,
    fit_tol: FitTol,
    defer_linear_prefix: bool,
) -> Result<ScalarNurbs, PostProcessError> {
    for stage in &chain.stages {
        match stage {
            ChainStage::SmoothKernel(_) => break,
            ChainStage::DerivativeGains { k1, k2 } => {
                if !defer_linear_prefix {
                    track = apply_derivative_gains_to_track(&track, *k1, *k2);
                }
            }
            ChainStage::NonlinearAdvance(adv) => {
                track = apply_nonlinear_advance_to_track(axis, &track, *adv, fit_tol)?;
            }
        }
    }
    Ok(track)
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
    input_start: Pvaj4,
    input_end: Pvaj4,
    track: ScalarNurbs,
    semantic_cuts: Vec<f64>,
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
        let ds = s - self.s0;
        self.e0 + self.r0 * ds + 0.5 * (self.r1 - self.r0) * ds * ds / (self.s1 - self.s0)
    }
}

/// Per-follower streaming state: the extrusion-per-shaped-distance table and
/// the shaped path odometer, carried across emit windows so the projected
/// track stays continuous through window boundaries, resets, and chain swaps.
#[derive(Debug, Default)]
pub(crate) struct FollowerState {
    active: bool,
    spans: Vec<RatioSpan>,
    ingested_through_t: Option<f64>,
    s_ingested_end: f64,
    s_shaped: f64,
    e_end: Option<f64>,
    projected_output_end: Option<f64>,
    emitted_output_pv: Option<(f64, f64)>,
    committed_input_end: Option<Pvaj4>,
    projected: Vec<ProjSeg>,
    projected_through_t: Option<f64>,
    projected_trimmed: bool,
}

impl FollowerState {
    /// A `Reset` restarts the timeline and relabels the follower odometer to
    /// a fresh origin: every reset flavor either re-seeds the MCU step
    /// counters at that same origin (stream_open, set_position, home_drip) or
    /// happens flow-free mid-homing (trip re-anchor), so the pre-reset
    /// odometer labels are void.
    pub(crate) fn reset_timeline(&mut self) {
        self.spans.clear();
        self.ingested_through_t = None;
        self.s_ingested_end = 0.0;
        self.s_shaped = 0.0;
        self.e_end = None;
        self.projected_output_end = None;
        self.emitted_output_pv = None;
        self.committed_input_end = None;
        self.projected.clear();
        self.projected_through_t = None;
        self.projected_trimmed = false;
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

    fn ingest(
        &mut self,
        raw: &ContinuousSegment,
        shaped: &ContinuousSegment,
        axis: usize,
        leaders: &[usize],
        upcoming: &[ContinuousSegment],
        (shaped_ds, raw_ds): (f64, f64),
    ) -> f64 {
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
        if raw_ds > SPAN_MIN_LEN_MM {
            let (r0, r1) = raw
                .followers
                .iter()
                .find(|f| f.axis_index == axis)
                .filter(|_| raw.spatial_path)
                .map_or((0.0, 0.0), |f| (f.ratio, f.ratio_end));
            self.push_span(shaped_ds, r0, r1);
            return shaped_ds;
        }
        let tail = leader_distance(shaped, raw, leaders).min(shaped_ds);
        let lead = shaped_ds - tail;
        let tail_ratio = self.spans.last().map_or(0.0, |span| span.r1);
        if lead <= SPAN_LOOKUP_SLACK_MM {
            self.push_span(shaped_ds, tail_ratio, tail_ratio);
            return shaped_ds;
        }
        if tail > SPAN_MIN_LEN_MM {
            self.push_span(tail, tail_ratio, tail_ratio);
        }
        let lead_ratio = upcoming
            .iter()
            .find(|seg| seg.spatial_path)
            .and_then(|seg| seg.followers.iter().find(|f| f.axis_index == axis))
            .map_or(0.0, |f| f.ratio);
        self.push_span(lead, lead_ratio, lead_ratio);
        shaped_ds
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
        self.spans.push(RatioSpan {
            s0: self.s_ingested_end,
            s1,
            r0,
            r1,
            e0,
        });
        self.s_ingested_end = s1;
    }

    fn prune_spans(&mut self) {
        let keep_from = self
            .spans
            .partition_point(|span| span.s1 < self.s_shaped - SPAN_LOOKUP_SLACK_MM);
        self.spans.drain(..keep_from);
    }

    /// Cumulative projected spatial extrusion at shaped path distance `s`.
    /// A kernel with negative lobes can overshoot the ingested path by a
    /// whisker at a terminal flush; the terminal ratio extends it.
    fn spans_e(&self, s: f64) -> f64 {
        let Some(first) = self.spans.first() else {
            return 0.0;
        };
        assert!(
            s >= first.s0 - SPAN_LOOKUP_SLACK_MM,
            "shaped path odometer {s} fell behind the pruned span table \
             starting at {}",
            first.s0
        );
        let idx = self.spans.partition_point(|span| span.s1 < s);
        match self.spans.get(idx) {
            Some(span) => span.e_at(s.max(span.s0)),
            None => {
                let last = self.spans.last().expect("non-empty spans");
                last.e_at(last.s1) + last.r1 * (s - last.s1)
            }
        }
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
        sign * delta
    }

    fn ratio_and_slope(&self, s: f64) -> (f64, f64) {
        let idx = self.spans.partition_point(|span| span.s1 <= s);
        match self.spans.get(idx) {
            Some(span) if s >= span.s0 => (span.ratio_at(s), span.ratio_slope()),
            Some(span) => (span.r0, 0.0),
            None => (self.spans.last().map_or(0.0, |span| span.r1), 0.0),
        }
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
        ContinuousAxis::Spline(curve) => extract_bezier_pieces(curve)
            .into_iter()
            .map(|piece| piece.u_end)
            .collect(),
        ContinuousAxis::RelativeSpline { curve, .. } => extract_bezier_pieces(curve)
            .into_iter()
            .map(|piece| piece.u_end)
            .collect(),
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => pieces
            .iter()
            .flat_map(|piece| {
                extract_bezier_pieces(&piece.curve)
                    .into_iter()
                    .map(|bezier| bezier.u_end)
            })
            .collect(),
        ContinuousAxis::Analytic { span, .. } => {
            let mut breaks = Vec::with_capacity(span.phases.len() + 2);
            breaks.push(span.t_start);
            breaks.extend(
                span.phases
                    .iter()
                    .take(span.phases.len().saturating_sub(1))
                    .map(|phase| analytic_phase_boundary(span.t_start, phase.end_time())),
            );
            breaks.push(span.t_end);
            breaks
        }
        ContinuousAxis::Hold { .. } | ContinuousAxis::Nudge(_) | ContinuousAxis::Buzz { .. } => {
            Vec::new()
        }
    }
}

struct AxisSignal<'a> {
    axis: &'a ContinuousAxis,
    base: f64,
}

impl TrackSignal for AxisSignal<'_> {
    fn eval(&self, t: f64) -> f64 {
        axis_pva(self.axis, t).0 - self.base
    }

    fn deriv(&self, t: f64) -> f64 {
        axis_pva(self.axis, t).1
    }

    fn second_deriv(&self, t: f64) -> f64 {
        axis_pva(self.axis, t).2
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (position, velocity, acceleration) = axis_pva(self.axis, t);
        (position - self.base, velocity, acceleration)
    }
}

fn fit_continuous_axis(
    axis_index: usize,
    axis: &ContinuousAxis,
    base: f64,
    t_start: f64,
    t_end: f64,
    fit_tol: FitTol,
) -> Result<ScalarNurbs, PostProcessError> {
    let breakpoints = axis_breakpoints(axis);
    fit_axis_from_signal(
        axis_index,
        t_start,
        t_end,
        &breakpoints,
        &AxisSignal { axis, base },
        fit_tol,
        "follower_axis",
    )
}

fn leader_arc_length(seg: &ContinuousSegment, leaders: &[usize]) -> f64 {
    let speed = |t: f64| {
        leaders
            .iter()
            .map(|&axis| {
                let velocity = axis_pva(&seg.axes[axis], t).1;
                velocity * velocity
            })
            .sum::<f64>()
            .sqrt()
    };
    axis_grid(seg, leaders)
        .windows(2)
        .map(|window| integrate(&speed, window[0], window[1]))
        .sum()
}

/// Shaped and raw leader arc lengths for each newly ingested segment. The
/// integrals are pure per-segment work, so they fan out across cores; results
/// are ordered by segment and bit-identical to serial evaluation.
fn leader_arc_lengths(
    raw: &[ContinuousSegment],
    shaped: &[ContinuousSegment],
    leaders: &[usize],
) -> Vec<(f64, f64)> {
    assert_eq!(raw.len(), shaped.len());
    let measure = |i: usize| {
        (
            leader_arc_length(&shaped[i], leaders),
            leader_arc_length(&raw[i], leaders),
        )
    };
    let workers = if cfg!(target_arch = "wasm32") {
        1
    } else {
        std::thread::available_parallelism().map_or(1, |cores| {
            cores.get().saturating_sub(1).max(1).min(raw.len())
        })
    };
    if workers <= 1 {
        return (0..raw.len()).map(measure).collect();
    }
    let next_job = std::sync::atomic::AtomicUsize::new(0);
    let mut out: Vec<(usize, (f64, f64))> = std::thread::scope(|scope| {
        let next_job = &next_job;
        let measure = &measure;
        let handles = (0..workers)
            .map(|_| {
                scope.spawn(move || {
                    let mut done = Vec::new();
                    loop {
                        let job = next_job.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        if job >= raw.len() {
                            return done;
                        }
                        done.push((job, measure(job)));
                    }
                })
            })
            .collect::<Vec<_>>();
        handles
            .into_iter()
            .flat_map(|handle| handle.join().expect("leader arc length thread panicked"))
            .collect()
    });
    out.sort_by_key(|(index, _)| *index);
    out.into_iter().map(|(_, lengths)| lengths).collect()
}

fn leader_distance(a: &ContinuousSegment, b: &ContinuousSegment, leaders: &[usize]) -> f64 {
    let t = leaders
        .iter()
        .fold(a.t_start.max(b.t_start), |start, &axis| {
            start
                .max(a.axes[axis].domain().0)
                .max(b.axes[axis].domain().0)
        });
    leaders
        .iter()
        .map(|&axis| {
            let delta = axis_pva(&a.axes[axis], t).0 - axis_pva(&b.axes[axis], t).0;
            delta * delta
        })
        .sum::<f64>()
        .sqrt()
}

fn axis_grid(seg: &ContinuousSegment, axes: &[usize]) -> Vec<f64> {
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

    fn matches(&self, axis: &ContinuousAxis) -> bool {
        const PROBES: [f64; 3] = [0.211_324_865_405_187_13, 0.5, 0.788_675_134_594_812_9];
        self.pieces.iter().all(|piece| {
            let duration = piece.u_end - piece.u_start;
            duration.is_finite()
                && duration > 0.0
                && PROBES.into_iter().all(|fraction| {
                    let t = piece.u_start + fraction * duration;
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
                                && (left - right).abs()
                                    <= 1e-8 * left.abs().max(right.abs()).max(1.0)
                        })
                })
        })
    }

    fn pva(&self, t: f64) -> Option<(f64, f64, f64)> {
        let index = self
            .pieces
            .partition_point(|piece| piece.u_end < t)
            .min(self.pieces.len().saturating_sub(1));
        let piece = self.pieces.get(index)?;
        let slack = 1e-9 * (piece.u_end - piece.u_start).max(1.0);
        if t < piece.u_start - slack || t > piece.u_end + slack {
            return None;
        }
        let (p, v, a) = polynomial_pva(&piece.coeffs, t - piece.u_start);
        Some((self.base + p, v, a))
    }
}

pub(crate) struct FollowerSignal<'a> {
    state: &'a FollowerState,
    e_start: f64,
    s_start: f64,
    e_spans_start: f64,
    shaped_axes: Vec<&'a ContinuousAxis>,
    leader_pieces: Vec<Option<LeaderPieces>>,
    raw_delta: Option<(&'a ContinuousAxis, f64)>,
    grid: Vec<f64>,
    dense_t: Vec<f64>,
    dense_s: Vec<f64>,
    s_owned_end: f64,
    start_sliver: Option<SpeedZeroSliver>,
    end_sliver: Option<SpeedZeroSliver>,
    pva_memo: RefCell<[Option<(u64, (f64, f64, f64))>; PVA_MEMO_SLOTS]>,
}

impl<'a> FollowerSignal<'a> {
    fn new(
        shaped: &'a ContinuousSegment,
        raw: &'a ContinuousSegment,
        axis: usize,
        leaders: &[usize],
        state: &'a FollowerState,
        s_start: f64,
        e_start: f64,
    ) -> Self {
        let (t0, t1) = projection_support(raw, shaped, axis, leaders);
        let shaped_axes: Vec<&'a ContinuousAxis> =
            leaders.iter().map(|&leader| &shaped.axes[leader]).collect();
        let leader_pieces = shaped_axes
            .iter()
            .map(|axis| LeaderPieces::of(axis))
            .collect();
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

        let mut sig = Self {
            state,
            e_start,
            s_start,
            e_spans_start: state.spans_e(s_start),
            shaped_axes,
            leader_pieces,
            raw_delta,
            grid,
            dense_t: Vec::new(),
            dense_s: Vec::new(),
            s_owned_end: f64::INFINITY,
            start_sliver: None,
            end_sliver: None,
            pva_memo: RefCell::new([None; PVA_MEMO_SLOTS]),
        };
        let (start_sliver, end_sliver) = sig.speed_zero_slivers();
        sig.start_sliver = start_sliver;
        sig.end_sliver = end_sliver;
        let mut dense_t = Vec::with_capacity(sig.grid.len());
        let mut dense_s = Vec::with_capacity(sig.grid.len());
        dense_t.push(sig.grid[0]);
        dense_s.push(0.0);
        let mut acc = 0.0;
        for w in sig.grid.windows(2) {
            integrate_recording(
                &|t| sig.shaped_speed(t),
                w[0],
                w[1],
                &mut acc,
                &mut dense_t,
                &mut dense_s,
            );
        }
        sig.dense_t = dense_t;
        sig.dense_s = dense_s;
        sig.s_owned_end = state.owned_span_end(sig.s_start, sig.s_end());
        sig
    }

    fn leader_pva(&self, index: usize, t: f64) -> (f64, f64, f64) {
        if let Some(Some(pieces)) = self.leader_pieces.get(index) {
            if let Some(value) = pieces.pva(t) {
                return value;
            }
        }
        axis_pva(self.shaped_axes[index], t)
    }

    fn raw_shaped_speed(&self, t: f64) -> f64 {
        (0..self.shaped_axes.len())
            .map(|index| {
                let velocity = self.leader_pva(index, t).1;
                velocity * velocity
            })
            .sum::<f64>()
            .sqrt()
    }

    fn raw_shaped_speed_deriv(&self, t: f64, speed: f64) -> f64 {
        if speed == 0.0 {
            return 0.0;
        }
        (0..self.shaped_axes.len())
            .map(|index| {
                let (_, velocity, acceleration) = self.leader_pva(index, t);
                velocity * acceleration
            })
            .sum::<f64>()
            / speed
    }

    fn raw_shaped_accel_norm(&self, t: f64) -> f64 {
        (0..self.shaped_axes.len())
            .map(|index| {
                let acceleration = self.leader_pva(index, t).2;
                acceleration * acceleration
            })
            .sum::<f64>()
            .sqrt()
    }

    fn endpoint_sliver(
        &self,
        endpoint: f64,
        slack: f64,
        interior_sign: f64,
    ) -> Option<SpeedZeroSliver> {
        let inner = endpoint + interior_sign * slack;
        let speed = self.raw_shaped_speed(endpoint);
        let secant = (self.raw_shaped_speed(inner) - speed) / (inner - endpoint);
        (speed <= secant.abs() * slack).then(|| SpeedZeroSliver {
            inner,
            zero_time: if speed == 0.0 {
                endpoint
            } else {
                endpoint - speed / secant
            },
            accel_norm: self.raw_shaped_accel_norm(endpoint),
            interior_sign,
        })
    }

    fn speed_zero_slivers(&self) -> (Option<SpeedZeroSliver>, Option<SpeedZeroSliver>) {
        let t0 = self.grid[0];
        let t1 = self.grid[self.grid.len() - 1];
        let slack = SPEED_ZERO_SLIVER_FRACTION * (t1 - t0);
        (
            self.endpoint_sliver(t0, slack, 1.0),
            self.endpoint_sliver(t1, slack, -1.0),
        )
    }

    fn speed_zero_sliver(&self, t: f64) -> Option<SpeedZeroSliver> {
        self.start_sliver
            .filter(|sliver| t <= sliver.inner)
            .or_else(|| self.end_sliver.filter(|sliver| t >= sliver.inner))
    }

    fn shaped_speed(&self, t: f64) -> f64 {
        self.raw_shaped_speed(t)
    }

    fn shaped_speed_deriv_from_speed(&self, t: f64, speed: f64) -> f64 {
        match self.speed_zero_sliver(t) {
            Some(sliver) => sliver.deriv_at(t),
            None => self.raw_shaped_speed_deriv(t, speed),
        }
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
        match self.dense_t.binary_search_by(|dense| dense.total_cmp(&t)) {
            Ok(index) => self.dense_s[index],
            Err(insertion) => {
                assert!(
                    insertion > 0 && insertion < self.dense_t.len(),
                    "follower distance query {t} outside [{}, {}]",
                    self.grid[0],
                    self.grid[self.grid.len() - 1]
                );
                let index = insertion - 1;
                self.dense_s[index] + integrate(&|u| self.shaped_speed(u), self.dense_t[index], t)
            }
        }
    }
    fn s_at(&self, t: f64) -> f64 {
        self.s_start + self.s_offset(t)
    }
    fn s_owned(&self, t: f64) -> f64 {
        self.s_at(t).min(self.s_owned_end)
    }
    fn s_owned_offset(&self, t: f64) -> f64 {
        self.s_offset(t).min(self.s_owned_end - self.s_start)
    }
    fn s_end(&self) -> f64 {
        self.s_start + self.dense_s.last().copied().expect("odometer knots seeded")
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
            .filter(|span| {
                span.s1 >= s_lo - SPAN_LOOKUP_SLACK_MM && span.s0 <= s_hi + SPAN_LOOKUP_SLACK_MM
            })
            .all(|span| span.r0 == 0.0 && span.r1 == 0.0);
        let tail_zero = match self.state.spans.last() {
            Some(last) if s_hi + SPAN_LOOKUP_SLACK_MM > last.s1 => last.r1 == 0.0,
            _ => true,
        };
        (overlapping_zero && tail_zero).then(|| self.eval(self.grid[0]))
    }

    fn construction_breakpoints(&self, raw_axis: &ContinuousAxis) -> FollowerBreakpoints {
        let raw_breaks = axis_breakpoints(raw_axis);
        let mut breaks = raw_breaks.clone();
        breaks.push(*self.grid.first().expect("follower grid is seeded"));
        breaks.push(*self.grid.last().expect("follower grid is seeded"));
        let s_end = self.s_end();
        let support_start = *self.grid.first().expect("follower grid is seeded");
        let support_end = *self.grid.last().expect("follower grid is seeded");
        for sliver in [self.start_sliver, self.end_sliver].into_iter().flatten() {
            if sliver.inner > support_start + GRID_DEDUP_EPS_S
                && sliver.inner < support_end - GRID_DEDUP_EPS_S
            {
                breaks.push(sliver.inner);
            }
        }
        let mut boundaries = self
            .state
            .spans
            .iter()
            .flat_map(|span| [span.s0, span.s1])
            .filter(|&s| {
                s >= self.s_start - SPAN_LOOKUP_SLACK_MM && s <= s_end + SPAN_LOOKUP_SLACK_MM
            })
            .collect::<Vec<_>>();
        boundaries.sort_by(f64::total_cmp);
        boundaries.dedup();
        for boundary in boundaries {
            if self.s_at(support_start) >= boundary {
                continue;
            }
            let mut lo = support_start;
            let mut hi = support_end;
            loop {
                let mid = 0.5 * lo + 0.5 * hi;
                if mid <= lo || mid >= hi {
                    break;
                }
                if self.s_at(mid) < boundary {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            if hi - support_start > GRID_DEDUP_EPS_S && support_end - hi > GRID_DEDUP_EPS_S {
                breaks.push(hi);
            }
        }
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
        let zeros = self.velocity_component_zeros(&breaks, support_start, support_end);
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

    fn velocity_component_zeros(
        &self,
        sorted_breaks: &[f64],
        support_start: f64,
        support_end: f64,
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
        for axis in &self.shaped_axes {
            for window in nodes.windows(2) {
                isolate_velocity_zeros(axis, window[0], window[1], &mut zeros);
            }
        }
        zeros
    }
}

fn isolate_velocity_zeros(axis: &ContinuousAxis, lo: f64, hi: f64, zeros: &mut Vec<f64>) {
    if hi <= lo {
        return;
    }
    let span = hi - lo;
    let mut previous_time = lo;
    let mut previous_velocity = axis_pva(axis, lo).1;
    if previous_velocity == 0.0 {
        zeros.push(lo);
    }
    for probe in 1..=COMPONENT_ZERO_PROBES {
        let time = if probe == COMPONENT_ZERO_PROBES {
            hi
        } else {
            lo + span * (probe as f64 / COMPONENT_ZERO_PROBES as f64)
        };
        let velocity = axis_pva(axis, time).1;
        if velocity == 0.0 {
            zeros.push(time);
        } else if previous_velocity != 0.0 && previous_velocity.signum() != velocity.signum() {
            zeros.push(refine_velocity_zero(
                axis,
                previous_time,
                time,
                previous_velocity,
            ));
        }
        previous_time = time;
        previous_velocity = velocity;
    }
}

fn refine_velocity_zero(axis: &ContinuousAxis, mut lo: f64, mut hi: f64, lo_velocity: f64) -> f64 {
    loop {
        let mid = 0.5 * lo + 0.5 * hi;
        if mid <= lo || mid >= hi {
            return hi;
        }
        let velocity = axis_pva(axis, mid).1;
        if velocity == 0.0 {
            return mid;
        }
        if velocity.signum() == lo_velocity.signum() {
            lo = mid;
        } else {
            hi = mid;
        }
    }
}

impl TrackSignal for FollowerSignal<'_> {
    fn eval(&self, t: f64) -> f64 {
        let spans = self.state.spans_e(self.s_owned(t)) - self.e_spans_start;
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
        let (ratio, _) = self.state.ratio_and_slope(self.s_owned(t));
        let raw = self.raw_delta.map_or(0.0, |(axis, _)| axis_pva(axis, t).1);
        ratio * self.shaped_speed(t) + raw
    }

    fn second_deriv(&self, t: f64) -> f64 {
        let speed = self.shaped_speed(t);
        let (ratio, slope) = self.state.ratio_and_slope(self.s_owned(t));
        let raw = self.raw_delta.map_or(0.0, |(axis, _)| axis_pva(axis, t).2);
        slope * speed * speed + ratio * self.shaped_speed_deriv_from_speed(t, speed) + raw
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let key = t.to_bits();
        let slot = ((key ^ key.rotate_right(32)) as usize) & (PVA_MEMO_SLOTS - 1);
        if let Some((stored_key, value)) = self.pva_memo.borrow()[slot] {
            if stored_key == key {
                return value;
            }
        }
        let s = self.s_owned(t);
        let speed = self.shaped_speed(t);
        let (ratio, slope) = self.state.ratio_and_slope(s);
        let spans = self.state.spans_e(s) - self.e_spans_start;
        let (raw_p, raw_v, raw_a) = self.raw_delta.map_or((0.0, 0.0, 0.0), |(axis, at_start)| {
            let (position, velocity, acceleration) = axis_pva(axis, t);
            (position - at_start, velocity, acceleration)
        });
        let value = (
            self.e_start + spans + raw_p,
            ratio * speed + raw_v,
            slope * speed * speed + ratio * self.shaped_speed_deriv_from_speed(t, speed) + raw_a,
        );
        self.pva_memo.borrow_mut()[slot] = Some((key, value));
        value
    }
    fn diagnostic(&self, t: f64) -> Option<String> {
        let s = self.s_owned(t);
        let span_index = self.state.spans.partition_point(|span| span.s1 <= s);
        let span = self.state.spans.get(span_index);
        let speed = self.shaped_speed(t);
        let speed_deriv = self.shaped_speed_deriv_from_speed(t, speed);
        let (ratio, slope) = self.state.ratio_and_slope(s);
        let leaders = self
            .shaped_axes
            .iter()
            .enumerate()
            .map(|(index, axis)| (axis_pva(axis, t), self.leader_pva(index, t), axis.domain()))
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

/// Every emitted follower piece carries its own position origin: the fitted
/// polynomial keeps only the excursion it accumulates inside its own knot
/// span, and `base_position` holds the cumulative print position at the
/// piece's start. A long target no longer regains a large relative carrier —
/// the value a fitted curve would have to resolve against never exceeds one
/// piece's own travel — and the seams stay bit-exact because each origin is
/// the running sum of the preceding pieces' own increments.
fn localize_pieces(base: f64, pieces: &[BezierPiece]) -> Arc<[RelativeSplinePiece]> {
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

fn piece_boundaries(track: &ScalarNurbs) -> Vec<f64> {
    let mut breaks = extract_bezier_pieces(track)
        .iter()
        .flat_map(|piece| [piece.u_start, piece.u_end])
        .collect::<Vec<_>>();
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    breaks
}

fn piece_run_start_pv(pieces: &[BezierPiece]) -> (f64, f64) {
    let first = pieces.first().expect("a fitted target has pieces");
    polynomial_pv(&first.coeffs, 0.0)
}

fn piece_run_end_pv(pieces: &[BezierPiece]) -> (f64, f64) {
    let last = pieces.last().expect("a fitted target has pieces");
    polynomial_pv(&last.coeffs, last.u_end - last.u_start)
}

fn piece_run_end(pieces: &[BezierPiece]) -> f64 {
    pieces.last().map_or(0.0, |last| {
        polynomial_pv(&last.coeffs, last.u_end - last.u_start).0
    })
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
/// the correction cannot accumulate.
#[derive(Clone, Copy)]
struct SeamWeld {
    origin: f64,
    coeffs: [f64; 4],
}

impl SeamWeld {
    fn spanning(
        start: f64,
        end: f64,
        emitted: (f64, f64),
        law_step: f64,
        base_position: f64,
        pieces: &[BezierPiece],
    ) -> Self {
        let length = end - start;
        assert!(
            length > 0.0,
            "follower weld window [{start}, {end}] has no length"
        );
        let (run_start, velocity_start) = piece_run_start_pv(pieces);
        let dp = emitted.0 - (base_position + run_start);
        let dv = emitted.1 + law_step - velocity_start;
        let cubic = (2.0 * dp + dv * length) / (length * length * length);
        let quadratic = -(3.0 * dp + 2.0 * dv * length) / (length * length);
        Self {
            origin: start,
            coeffs: [dp, dv, quadratic, cubic],
        }
    }

    fn apply(&self, piece: &mut BezierPiece) {
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

fn project_monotone(
    axis: usize,
    label: &str,
    pieces: &mut [nurbs::bezier::BezierPiece],
    tol: FitTol,
) {
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
        let probe_tau = |u: f64| 0.5 * (u + 1.0) * h;
        let sweep = 4 * degree;
        let dips = (0..=sweep).any(|step| {
            let tau = h * step as f64 / sweep as f64;
            polynomial_pva(&piece.coeffs, tau).1 < 0.0
        });
        if h <= SEGMENT_TIME_EPS_S || !dips {
            continue;
        }
        let bernstein = piece.to_bernstein();
        let mut derivative = bernstein
            .windows(2)
            .map(|pair| degree as f64 * (pair[1] - pair[0]) / h)
            .collect::<Vec<_>>();
        if degree < 3 || derivative[0] < 0.0 || derivative[degree - 1] < 0.0 {
            continue;
        }
        let mut deficit = 0.0_f64;
        let mut perturbation = 0.0_f64;
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
        let source = piece.clone();
        let mut reintegrated = vec![bernstein[0]; degree + 1];
        for i in 0..degree {
            reintegrated[i + 1] = reintegrated[i] + derivative[i] * h / degree as f64;
        }
        let corrected_piece =
            nurbs::bezier::BezierPiece::from_bernstein(&reintegrated, piece.u_start, piece.u_end);
        let within_budget = crate::lowering::LADDER_PROBES_U.iter().all(|&u| {
            let corrected = polynomial_pva(&corrected_piece.coeffs, probe_tau(u));
            let uncorrected = polynomial_pva(&source.coeffs, probe_tau(u));
            [corrected.0, corrected.1, corrected.2]
                .into_iter()
                .all(f64::is_finite)
                && corrected.0 - uncorrected.0 >= -tol.pos_mm
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

fn polynomial_pv(coeffs: &[f64], tau: f64) -> (f64, f64) {
    let (position, velocity, _) = polynomial_pva(coeffs, tau);
    (position, velocity)
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
fn assert_piece_seams(axis: usize, label: &str, pieces: &[nurbs::bezier::BezierPiece]) {
    for (index, pair) in pieces.windows(2).enumerate() {
        let left_h = pair[0].u_end - pair[0].u_start;
        let right_h = pair[1].u_end - pair[1].u_start;
        let left = polynomial_pv(&pair[0].coeffs, left_h);
        let right = polynomial_pv(&pair[1].coeffs, 0.0);
        let seam = pair[0].u_end;
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
fn integrate(f: &impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    if b - a <= 0.0 {
        return 0.0;
    }
    let m = 0.5 * (a + b);
    let (fa, fm, fb) = (f(a), f(m), f(b));
    let whole = (b - a) / 6.0 * (fa + 4.0 * fm + fb);
    adaptive_simpson(
        f,
        a,
        b,
        fa,
        fm,
        fb,
        whole,
        INTEGRAL_TOL_MM,
        INTEGRAL_MAX_DEPTH,
    )
}

/// `integrate`, but the converged leaves of the adaptive recursion are kept
/// as odometer knots: `dense_t`/`dense_s` receive each leaf's right endpoint
/// with the running prefix sum in `acc`, so later point queries integrate
/// only the short residual past the nearest knot instead of re-descending
/// the whole refinement tree from a grid boundary.
fn integrate_recording(
    f: &impl Fn(f64) -> f64,
    a: f64,
    b: f64,
    acc: &mut f64,
    dense_t: &mut Vec<f64>,
    dense_s: &mut Vec<f64>,
) {
    if b - a <= 0.0 {
        return;
    }
    let m = 0.5 * (a + b);
    let (fa, fm, fb) = (f(a), f(m), f(b));
    let whole = (b - a) / 6.0 * (fa + 4.0 * fm + fb);
    adaptive_simpson_recording(
        f,
        a,
        b,
        fa,
        fm,
        fb,
        whole,
        INTEGRAL_TOL_MM,
        INTEGRAL_MAX_DEPTH,
        acc,
        dense_t,
        dense_s,
    );
}

#[allow(clippy::too_many_arguments)]
fn adaptive_simpson_recording(
    f: &impl Fn(f64) -> f64,
    a: f64,
    b: f64,
    fa: f64,
    fm: f64,
    fb: f64,
    whole: f64,
    tol: f64,
    depth: u32,
    acc: &mut f64,
    dense_t: &mut Vec<f64>,
    dense_s: &mut Vec<f64>,
) {
    let m = 0.5 * (a + b);
    let (lm, rm) = (0.5 * (a + m), 0.5 * (m + b));
    let (flm, frm) = (f(lm), f(rm));
    let left = (m - a) / 6.0 * (fa + 4.0 * flm + fm);
    let right = (b - m) / 6.0 * (fm + 4.0 * frm + fb);
    let delta = left + right - whole;
    if depth == 0 || delta.abs() <= 15.0 * tol {
        *acc += left + right + delta / 15.0;
        dense_t.push(b);
        dense_s.push(*acc);
        return;
    }
    adaptive_simpson_recording(
        f,
        a,
        m,
        fa,
        flm,
        fm,
        left,
        0.5 * tol,
        depth - 1,
        acc,
        dense_t,
        dense_s,
    );
    adaptive_simpson_recording(
        f,
        m,
        b,
        fm,
        frm,
        fb,
        right,
        0.5 * tol,
        depth - 1,
        acc,
        dense_t,
        dense_s,
    );
}

#[allow(clippy::too_many_arguments)]
fn adaptive_simpson(
    f: &impl Fn(f64) -> f64,
    a: f64,
    b: f64,
    fa: f64,
    fm: f64,
    fb: f64,
    whole: f64,
    tol: f64,
    depth: u32,
) -> f64 {
    let m = 0.5 * (a + b);
    let (lm, rm) = (0.5 * (a + m), 0.5 * (m + b));
    let (flm, frm) = (f(lm), f(rm));
    let left = (m - a) / 6.0 * (fa + 4.0 * flm + fm);
    let right = (b - m) / 6.0 * (fm + 4.0 * frm + fb);
    let delta = left + right - whole;
    if depth == 0 || delta.abs() <= 15.0 * tol {
        return left + right + delta / 15.0;
    }
    adaptive_simpson(f, a, m, fa, flm, fm, left, 0.5 * tol, depth - 1)
        + adaptive_simpson(f, m, b, fm, frm, fb, right, 0.5 * tol, depth - 1)
}

#[cfg(test)]
mod tests;
