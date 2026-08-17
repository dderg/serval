#![forbid(unsafe_code)]

pub mod bootstrap;
pub mod codec;
pub mod messages;

pub use bootstrap::{Identify, IdentifyResponse};
pub use codec::{Decode, DecodeError, Encode};
pub use messages::{
    ClaimHandshakeReply, EndstopTrip, FaultEvent, LaneDepth, LaneRun, McuLog, MessageKind,
    PushPieces, PushPiecesResponse, PushSampleRuns, PushSampleRunsResponse, QuerySampleGrid,
    RuntimeCapsResponse, SampleGridResponse, SetpointSample, SlaveState, SlaveStatus,
    StatusHeartbeat, Stop, StopResponse,
};

include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));

pub const PROTO_VERSION: u8 = 0x01;

// Channel discriminators mirror MCU_CHANNEL_* in src/mcu_transport_dispatch.c — keep in sync.
pub const MCU_CHANNEL_CONTROL: u8 = 0x00;
pub const MCU_CHANNEL_EVENTS: u8 = 0x01;
pub const MCU_CHANNEL_PIECES: u8 = 0x02;

// result_codes mirror RUNTIME_ERR_* in src/mcu_transport_dispatch.c — keep in sync.
// Canonical numeric values are defined in rust/runtime/src/error.rs.
pub mod result_codes {
    pub const OK: i32 = 0;
    pub const RING_FULL: i32 = -309;
    pub const STREAM_HALTED: i32 = -142;
    // Mirrors ERR_PIECES_WHILE_HALTED in rust/ethercat-rt/src/stream_halt.rs —
    // the EtherCAT endpoint's piece gate, distinct from the MCU runtime's -142.
    pub const EC_PIECES_WHILE_HALTED: i32 = -315;
    pub const INVALID_ARG: i32 = -26;
}

pub const PER_MESSAGE_HEADER_LEN: usize = 7;

#[cfg(test)]
mod tests;
