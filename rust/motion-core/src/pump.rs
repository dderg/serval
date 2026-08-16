pub use crate::types::AxisKey;

mod diag;
mod drip;
mod junction;
mod margin;
mod memstat;
mod messages;
mod pump_loop;
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
    BundleLimits, EnqueueMsg, HeartbeatMsg, HistoryRecorder, PieceSink, PumpCallbacks, PumpMsg,
    SendError,
};
pub use pump_loop::{MAX_LEAD_SECS, PUMP_DATA_CHANNEL_CAP, run_pump};
pub use sched::{
    AxisFrame, AxisQueue, FramePlan, Schedule, SeamBasis, append_pieces_merging_holds, schedule,
};
pub use stepcompress_sink::{
    BACKLOG_CEILING_FRAMES, ClockSource, FrameEgress, MOVE_SLOT_RESERVE, StepcompressEndpoint,
    StepcompressPacer, build_endpoint,
};
pub use transit_trace::emit_fault_snapshot;
#[cfg(test)]
pub(crate) use wire_sink::pushpieces_retransmit_serial;
pub use wire_sink::{McuTransport, WireSink};

#[cfg(test)]
mod drip_tests;
#[cfg(test)]
mod hold_merge_seam_tests;
#[cfg(test)]
mod margin_tests;
#[cfg(test)]
mod memstat_tests;
#[cfg(test)]
mod sched_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod transit_trace_tests;
