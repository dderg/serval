#![cfg(any(test, feature = "test-support"))]

use std::collections::BTreeMap;

pub use geometry::Move;
use geometry::{CornerFitConfig, VelocityLimits};
use trajectory::{AxisChainSet, ClockedMotorSpan, ContinuousSegment};

use crate::enqueue::enqueue_segment;
use crate::mcu_config::{McuAxisConfig, McuHardware};
use crate::pump::{
    JUNCTION_POSITION_FATAL_MM, JUNCTION_POSITION_LOG_MM, JunctionTracker, MAX_LEAD_SECS,
};
use crate::types::AxisKey;
use motion_pipeline::{Pipeline, StreamConfig, TrajectoryItem};

const HARNESS_MCU_ID: u32 = 0;
const HARNESS_MCU_FREQ_HZ: f64 = 64.0e6;
const HARNESS_T0_SECS: f64 = 1.0;

#[must_use]
pub fn default_stream_config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 1e-3,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(
            100.0,
            1000.0,
            geometry::corner_deviation_from_scv(5.0, 1000.0),
            f64::INFINITY,
        )
        .expect("bench limits (max_v=100 accel=1000 scv=5 jerk=inf) are valid"),
        step_capacity: None,
    }
}

fn harness_mcu_configs() -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        ethercat: false,
        mcu_id: HARNESS_MCU_ID,
        axes: vec![0, 1, 2],
        hw: McuHardware {
            kinematics: 1,
            max_motor_velocity: vec![f64::INFINITY; 3],
            ..Default::default()
        },
        ..Default::default()
    }]
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeamDescriptor {
    pub mcu_id: u32,
    pub axis: u8,
    pub delta_mm: f64,
    pub prev_pos: f64,
    pub next_pos: f64,
    pub prev_host_t: f64,
    pub next_host_t: f64,
    pub prev_source_line: u32,
    pub next_source_line: u32,
    pub vel_jump: Option<f64>,
    pub commit_index: usize,
}

impl SeamDescriptor {
    #[must_use]
    pub fn is_fatal(&self) -> bool {
        self.delta_mm >= JUNCTION_POSITION_FATAL_MM
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SeamReport {
    pub boundaries: Vec<SeamDescriptor>,
    pub moves: usize,
    pub segments: usize,
    pub commits: usize,
}

impl SeamReport {
    #[must_use]
    pub fn fatal(&self) -> usize {
        self.boundaries.iter().filter(|b| b.is_fatal()).count()
    }

    #[must_use]
    pub fn worst(&self) -> f64 {
        self.boundaries
            .iter()
            .map(|b| b.delta_mm)
            .fold(0.0, f64::max)
    }

    #[must_use]
    pub fn worst_fatal(&self) -> Option<&SeamDescriptor> {
        self.boundaries
            .iter()
            .filter(|b| b.is_fatal())
            .max_by(|a, b| a.delta_mm.total_cmp(&b.delta_mm))
    }
}

/// The moves the live host submits for `source`, through the shared snapshot
/// G-code interpreter (`pipeline_snapshot::waypoints::parse_gcode`). Panics
/// on G-code the snapshot refuses and on a stop at rest, which a move list
/// cannot carry.
#[must_use]
pub fn parse_gcode_to_moves(source: &str, limits: VelocityLimits) -> Vec<Move> {
    let waypoints =
        pipeline_snapshot::waypoints::parse_gcode(source, &pipeline_snapshot::config_base(limits))
            .unwrap_or_else(|e| panic!("seam harness: {e}"));
    let steps = pipeline_snapshot::plan_steps(&waypoints, limits)
        .unwrap_or_else(|e| panic!("seam harness: {e}"));
    let mut moves = Vec::with_capacity(steps.len());
    for step in steps {
        match step {
            pipeline_snapshot::PlanStep::Move(m) => moves.push(m),
            pipeline_snapshot::PlanStep::Rest { dwell_s } => panic!(
                "seam harness: the G-code stops at rest (dwell {dwell_s} s) after move {}; a move list cannot carry it",
                moves.len()
            ),
        }
    }
    moves
}

pub struct LiveReplay {
    pub config: StreamConfig,
    pub chains: AxisChainSet,
    pub home: Vec<f64>,
    pub moves: Vec<Move>,
}

impl LiveReplay {
    pub fn new(source: &str, planner: &planner_config::PlannerConfig) -> Result<Self, String> {
        let config = motion_pipeline::live_config::stream_config(planner)?;
        let chains = planner.compile_active_chains().map_err(|e| e.to_string())?;
        let extruder = motion_pipeline::live_config::extruder_caps_on(
            &motion_pipeline::live_config::extruder_caps(planner)?,
            pipeline_snapshot::EXTRUDER_AXIS,
        );
        let moves = parse_gcode_to_moves(
            source,
            motion_pipeline::live_config::move_limits(planner, extruder)?,
        );
        Ok(Self {
            config,
            chains,
            home: pipeline_snapshot::rest_position_before(&moves),
            moves,
        })
    }

    #[must_use]
    pub fn pipeline(&self) -> Pipeline {
        Pipeline::new(
            self.config.clone(),
            self.chains.clone(),
            self.home.clone(),
            0.0,
        )
    }
}

fn span_velocity(span: &ClockedMotorSpan, t: f64) -> f64 {
    span.signal
        .eval_pva(t)
        .expect("dispatched span evaluates inside its own stream domain")
        .velocity
}

struct Ingestor {
    mcu_configs: Vec<McuAxisConfig>,
    tracker: JunctionTracker,
    prev_last: BTreeMap<AxisKey, ClockedMotorSpan>,
    first_enqueue: bool,
    report: SeamReport,
}

impl Ingestor {
    fn new() -> Self {
        Self {
            mcu_configs: harness_mcu_configs(),
            tracker: JunctionTracker::default(),
            prev_last: BTreeMap::new(),
            first_enqueue: true,
            report: SeamReport::default(),
        }
    }

    fn ingest(&mut self, segments: &[ContinuousSegment], commit_index: usize) {
        for seg in segments {
            self.report.segments += 1;
            let epoch = if self.first_enqueue {
                crate::anchor::StreamEpoch::Reposition
            } else {
                crate::anchor::StreamEpoch::Continuation
            };
            self.first_enqueue = false;
            let msgs = enqueue_segment(
                seg,
                &self.mcu_configs,
                &crate::enqueue::EnqueueCtx {
                    epoch_freq: &|_| None,
                    lane_is_phase: &|_| false,
                    t0: HARNESS_T0_SECS,
                    epoch,
                    host_now: 0.0,
                    lead_secs: MAX_LEAD_SECS,
                    project_exact: |_mcu, hs: f64| hs * HARNESS_MCU_FREQ_HZ,
                    clock_freq_hz: &|_| HARNESS_MCU_FREQ_HZ,
                },
            )
            .expect("harness lanes enqueue without a continuous error");
            for msg in msgs {
                if msg.epoch.position_redefined() {
                    self.prev_last.remove(&msg.key);
                }
                if let Some(seam) =
                    self.tracker
                        .observe_msg(msg.key, &msg.spans, msg.epoch, msg.source_line)
                {
                    if seam.jump() >= JUNCTION_POSITION_LOG_MM {
                        let first = msg.spans.first().unwrap();
                        let vel_jump = self.prev_last.get(&msg.key).map(|prev| {
                            (span_velocity(prev, prev.stream_t_end)
                                - span_velocity(first, first.stream_t_start))
                            .abs()
                        });
                        self.report.boundaries.push(SeamDescriptor {
                            mcu_id: seam.key.mcu_id,
                            axis: seam.key.axis,
                            delta_mm: seam.jump(),
                            prev_pos: seam.prev_end_pos,
                            next_pos: seam.next_start_pos,
                            prev_host_t: seam.prev_end_host,
                            next_host_t: seam.next_start_host,
                            prev_source_line: seam.prev_source_line,
                            next_source_line: seam.next_source_line,
                            vel_jump,
                            commit_index,
                        });
                    }
                }
                if msg
                    .spans
                    .first()
                    .is_some_and(|span| span.signal.motor_mask == 0)
                {
                    self.prev_last
                        .insert(msg.key, msg.spans.last().unwrap().clone());
                }
            }
        }
    }
}

pub fn run_schedule(source: &str, config: StreamConfig) -> SeamReport {
    let moves = parse_gcode_to_moves(source, config.limits);
    run_moves(&moves, config)
}

/// Replay the moves through the full streaming pipeline and observe every
/// emitted segment seam. Emission boundaries are the pipeline's own (finality
/// barrier, input-empty drains); the thread interleaving between feeding and
/// the stages varies them run to run, which is exactly the surface the seam
/// checks guard.
pub fn run_moves(moves: &[Move], config: StreamConfig) -> SeamReport {
    run_moves_with_chains(moves, config, AxisChainSet::default(), None)
}

/// `run_moves` with post-processing chains active and, optionally, a `Drain`
/// injected after every `drain_every` moves — the cadence the pacer uses when
/// the feed runs dry mid-print.
pub fn run_moves_with_chains(
    moves: &[Move],
    config: StreamConfig,
    chains: AxisChainSet,
    drain_every: Option<usize>,
) -> SeamReport {
    let n_moves = moves.len();
    let segs = collect_shaped_segments_scripted(moves, config, chains, drain_every);

    let mut ingestor = Ingestor::new();
    ingestor.ingest(&segs, 0);
    let mut report = ingestor.report;
    report.moves = n_moves;
    report.commits = 1;
    report
}

pub fn collect_shaped_segments_scripted(
    moves: &[Move],
    config: StreamConfig,
    chains: AxisChainSet,
    drain_every: Option<usize>,
) -> Vec<ContinuousSegment> {
    let mut script: Vec<motion_pipeline::StreamInput> = Vec::new();
    for (i, m) in moves.iter().cloned().enumerate() {
        script.push(m.into());
        if drain_every.is_some_and(|n| i % n == n - 1) {
            script.push(motion_pipeline::StreamInput::Drain);
        }
    }
    collect_shaped_segments_from_script(script, config, chains)
}

/// Feed an explicit input script (moves interleaved with `Drain`/`Control`
/// tokens — the shapes the ingress produces for M400 / G4 boundaries) through
/// the full streaming pipeline and return the shaped segments it emits.
pub fn collect_shaped_segments_from_script(
    script: Vec<motion_pipeline::StreamInput>,
    config: StreamConfig,
    chains: AxisChainSet,
) -> Vec<ContinuousSegment> {
    let home =
        pipeline_snapshot::rest_position_before(script.iter().filter_map(|item| match item {
            motion_pipeline::StreamInput::Move(m) => Some(m),
            _ => None,
        }));
    let mut pipeline = Pipeline::new(config, chains, home, 0.0);
    let mut segs = Vec::new();
    let mut output = |item| {
        if let TrajectoryItem::Seg(seg) = item {
            segs.push(seg);
        }
        true
    };
    for item in script {
        assert!(pipeline.feed(item, &mut output));
    }
    assert!(pipeline.finish(&mut output));
    segs
}

#[cfg(test)]
mod tests;
