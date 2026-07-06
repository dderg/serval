pub mod bootstrap;
pub mod connection;
pub mod demux;
pub mod frame;
pub mod klipper_frame;
pub mod transport;
pub mod wire_helpers;

pub use bootstrap::{
    BOOTSTRAP_IDENTIFY_LEN, BOOTSTRAP_IDENTIFY_RESPONSE_LEN, IdentifyResponse,
    decode_identify_response, encode_identify,
};
pub use connection::Connection;
#[cfg(any(test, feature = "test-util"))]
pub use connection::MockConnection;
pub use demux::{Demuxer, Frame, KlipperFrame, PollOutcome, StreamError};
pub use frame::{
    CHANNEL_CONTROL, CHANNEL_EVENTS, FRAME_SYNC, FrameError, decode_frame, encode_frame,
};
pub use mcu_protocol::{MessageKind, PROTO_VERSION, SCHEMA_HASH};
pub use transport::{ConnectionState, EpochChange, McuTransport, Transport, TransportError};
