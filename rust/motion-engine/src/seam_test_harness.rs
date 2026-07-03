#![cfg(any(test, feature = "test-support"))]

use std::collections::BTreeMap;

pub use geometry::Move;
use geometry::path::lowering::PositionProfile;
use geometry::{ChainFitConfig, VelocityLimits};
use runtime::piece_ring::PieceEntry;
use trajectory::{AxisChainSet, ShapedSegment};

use crate::classify::build_move;
use crate::enqueue::enqueue_segment;
use crate::mcu_config::{McuAxisConfig, McuCaps};
use crate::pump::{
    JUNCTION_POSITION_FATAL_MM, JUNCTION_POSITION_LOG_MM, JunctionTracker, MAX_LEAD_SECS,
};
use crate::types::AxisKey;
use motion_pipeline::{StreamConfig, setup_stages};

const HARNESS_MCU_ID: u32 = 0;
const HARNESS_MCU_FREQ_HZ: f64 = 1.0e6;
const EXTRUDER_AXIS: usize = 3;

#[must_use]
pub fn default_stream_config() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 1e-3,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(100.0, 1000.0, 5.0, 100_000.0)
            .expect("bench limits (max_v=100 accel=1000 scv=5 jerk=100000) are valid"),
    }
}

fn harness_mcu_configs() -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        mcu_id: HARNESS_MCU_ID,
        axes: vec![0, 1, 2],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }]
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeamDescriptor {
    pub mcu_id: u32,
    pub axis: u8,
    pub delta_mm: f32,
    pub prev_pos: f32,
    pub next_pos: f32,
    pub prev_host_t: f64,
    pub next_host_t: f64,
    pub prev_source_line: u32,
    pub next_source_line: u32,
    pub vel_jump: Option<f32>,
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
    pub fn worst(&self) -> f32 {
        self.boundaries
            .iter()
            .map(|b| b.delta_mm)
            .fold(0.0, f32::max)
    }

    #[must_use]
    pub fn worst_fatal(&self) -> Option<&SeamDescriptor> {
        self.boundaries
            .iter()
            .filter(|b| b.is_fatal())
            .max_by(|a, b| a.delta_mm.total_cmp(&b.delta_mm))
    }
}

struct PosTracker {
    pos: [f64; 3],
    feed: f64,
    absolute: bool,
    established: bool,
}

impl PosTracker {
    fn new() -> Self {
        Self {
            pos: [0.0; 3],
            feed: 80.0,
            absolute: true,
            established: false,
        }
    }

    fn apply(
        &mut self,
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        f: Option<f64>,
    ) -> Option<([f64; 3], f64, f64, f64)> {
        if let Some(fv) = f {
            self.feed = fv / 60.0;
        }
        let target = if self.absolute {
            [
                x.unwrap_or(self.pos[0]),
                y.unwrap_or(self.pos[1]),
                z.unwrap_or(self.pos[2]),
            ]
        } else {
            [
                self.pos[0] + x.unwrap_or(0.0),
                self.pos[1] + y.unwrap_or(0.0),
                self.pos[2] + z.unwrap_or(0.0),
            ]
        };
        let d = [
            target[0] - self.pos[0],
            target[1] - self.pos[1],
            target[2] - self.pos[2],
        ];
        let start = self.pos;
        self.pos = target;
        if !self.established {
            self.established = true;
            return None;
        }
        Some((start, d[0], d[1], d[2]))
    }
}

#[must_use]
pub fn parse_gcode_to_moves(source: &str, limits: VelocityLimits) -> Vec<Move> {
    let mut p = PosTracker::new();
    let mut moves = Vec::new();
    let mut submitted: u32 = 0;
    for tok in gcode::lex(source) {
        let Ok(t) = tok else { continue };
        let gcode::Token::Command {
            letter,
            major,
            params,
            ..
        } = t
        else {
            continue;
        };
        if letter != b'G' {
            continue;
        }
        match major {
            0 | 1 => {
                let Some((start, dx, dy, dz)) =
                    p.apply(params.x(), params.y(), params.z(), params.f())
                else {
                    continue;
                };
                if dx.abs() < 1e-9 && dy.abs() < 1e-9 && dz.abs() < 1e-9 {
                    continue;
                }
                match build_move(
                    start,
                    dx,
                    dy,
                    dz,
                    EXTRUDER_AXIS,
                    0.0,
                    limits,
                    p.feed,
                    submitted,
                ) {
                    Ok(m) => {
                        moves.push(m);
                        submitted += 1;
                    }
                    Err(_) => continue,
                }
            }
            90 => p.absolute = true,
            91 => p.absolute = false,
            _ => {}
        }
    }
    moves
}

fn endpoint_velocity_out(p: &PieceEntry) -> f32 {
    p.vel_end()
}

fn endpoint_velocity_in(p: &PieceEntry) -> f32 {
    p.vel_start()
}

struct Ingestor {
    mcu_configs: Vec<McuAxisConfig>,
    tracker: JunctionTracker,
    prev_last: BTreeMap<AxisKey, PieceEntry>,
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

    fn ingest(&mut self, segments: &[ShapedSegment], commit_index: usize) {
        for seg in segments {
            self.report.segments += 1;
            let fresh = self.first_enqueue;
            self.first_enqueue = false;
            let msgs = enqueue_segment(
                seg,
                &self.mcu_configs,
                0.0,
                fresh,
                0.0,
                MAX_LEAD_SECS,
                |_mcu, hs| (hs * HARNESS_MCU_FREQ_HZ) as u64,
                None,
            );
            for msg in msgs {
                if msg.fresh_stream {
                    self.prev_last.remove(&msg.key);
                }
                if let Some(seam) = self.tracker.observe_msg(
                    msg.key,
                    &msg.pieces,
                    msg.fresh_stream,
                    msg.source_line,
                    Some(HARNESS_MCU_FREQ_HZ),
                ) {
                    if seam.jump() >= JUNCTION_POSITION_LOG_MM {
                        let first_piece = &msg.pieces.first().unwrap().0;
                        let vel_jump = self.prev_last.get(&msg.key).map(|prev| {
                            (endpoint_velocity_out(prev) - endpoint_velocity_in(first_piece)).abs()
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
                if msg.pieces.first().is_some_and(|(p, _)| p.motor_mask == 0) {
                    self.prev_last.insert(msg.key, msg.pieces.last().unwrap().0);
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
    let n_moves = moves.len();
    let segs = collect_shaped_segments(moves, config);

    let mut ingestor = Ingestor::new();
    ingestor.ingest(&segs, 0);
    let mut report = ingestor.report;
    report.moves = n_moves;
    report.commits = 1;
    report
}

/// Feed the moves through the full streaming pipeline and return the shaped
/// segments it emits — the trajectory enqueue would dispatch.
pub fn collect_shaped_segments(moves: &[Move], config: StreamConfig) -> Vec<ShapedSegment> {
    let home = moves
        .first()
        .and_then(|m| m.segment.spatial.as_ref())
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let handle = setup_stages(config, AxisChainSet::default(), home.to_vec(), 0.0);
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ShapedSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let motion_pipeline::ShapedItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    for m in moves.iter().cloned() {
        handle
            .input
            .send(m.into())
            .expect("pipeline input closed while feeding — a stage died");
    }
    drop(handle.input);
    collector.join().expect("pipeline collector panicked")
}

#[cfg(test)]
mod tests;
