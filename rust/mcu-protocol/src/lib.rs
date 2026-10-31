#![forbid(unsafe_code)]

pub mod bootstrap;
pub mod codec;
pub mod messages;

pub use bootstrap::IdentifyResponse;
pub use codec::{Decode, DecodeError, Encode};
pub use messages::{
    ClaimHandshakeReply, EndstopTrip, FaultEvent, LaneDepth, LaneRun, McuLog, MessageKind,
    PushSampleRuns, PushSampleRunsResponse, QuerySampleGrid, RuntimeCapsResponse,
    SampleGridResponse, SetpointSample, SlaveState, SlaveStatus, StatusHeartbeat, Stop,
    StopResponse,
};

include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));

pub const PROTO_VERSION: u8 = 0x01;

// Channel discriminator mirrors MCU_CHANNEL_PIECES in src/mcu_transport_dispatch.c.
pub const MCU_CHANNEL_PIECES: u8 = 0x02;

// result_codes mirror the MCU dispatch result table in
// src/mcu_transport_dispatch.c — keep in sync. Canonical numeric values are
// FaultCode in rust/runtime-contract/src/error.rs.
pub mod result_codes {
    pub const OK: i32 = 0;
    pub const RING_FULL: i32 = -309;
    pub const STREAM_HALTED: i32 = -142;
    // Mirrors ERR_PIECES_WHILE_HALTED in rust/ethercat-rt/src/stream_halt.rs — the
    // EtherCAT endpoint's sample-run gate, distinct from the MCU runtime's -142.
    pub const EC_PIECES_WHILE_HALTED: i32 = -315;
}

pub const PER_MESSAGE_HEADER_LEN: usize = 7;

#[cfg(test)]
mod tests;
