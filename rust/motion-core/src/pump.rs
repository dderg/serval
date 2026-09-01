pub use crate::types::AxisKey;

mod barrier_ledger;
mod diag;
mod drip;
mod junction;
mod memstat;
mod messages;
mod pump_loop;
mod sample_sink;
mod sched;
mod stall;
mod stepcompress_sink;
mod transit_trace;
mod wire_sink;

pub use drip::{DRIP_WINDOW_SECS, DripArm};
pub use junction::{
    JUNCTION_POSITION_FATAL_MM, JUNCTION_POSITION_LOG_MM, JunctionSeam, JunctionTracker,
    junction_jumps,
};
pub use messages::{
    BundleLimits, BuzzLane, BuzzParams, BuzzRoute, BuzzStart, BuzzToken, BuzzTransport, BuzzWave,
    DrainTick, EnqueueMsg, HeartbeatMsg, HistoryRecorder, PumpCallbacks, PumpMsg, RetiredBy,
    SendError, SpanSink,
};
#[cfg(test)]
pub(crate) use pump_loop::pump_past_guard_secs;
pub use pump_loop::{MAX_LEAD_SECS, PUMP_DATA_CHANNEL_CAP, run_pump};
pub use sample_sink::{
    RetiredRuns, SAMPLE_BACKLOG_CEILING_RUNS, SAMPLE_LANE_PIECE_WINDOW, SampleEndpoint,
    SampleLaneConfig, SamplePacer, SamplePositionQuery, build_sample_endpoint,
};
pub use sched::{
    AxisFrame, AxisQueue, FramePlan, ReleaseHorizons, Schedule, append_spans_merging_holds,
    schedule,
};
pub use stepcompress_sink::{
    BACKLOG_CEILING_FRAMES, ClockSource, FrameEgress, MOVE_SLOT_RESERVE, StepLaneConfig,
    StepcompressEndpoint, StepcompressPacer, build_endpoint,
};
pub use transit_trace::emit_fault_snapshot;
pub use wire_sink::{EtherCatRing, LANE_GROUP_PHASE, LANE_GROUP_PULSE, RingFiller, WireSink};
pub mod skew_monitor;

#[cfg(test)]
mod drip_tests;
#[cfg(test)]
mod heartbeat_credit_tests;
#[cfg(test)]
mod hold_merge_seam_tests;
#[cfg(test)]
mod lane_rejoin_tests;
#[cfg(test)]
mod memstat_tests;
#[cfg(test)]
mod sched_tests;
#[cfg(test)]
mod skew_monitor_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod transit_trace_tests;
