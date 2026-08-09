pub mod bootstrap;
pub mod demux;
pub mod frame;
pub mod klipper_frame;
pub mod wire_helpers;

pub use bootstrap::{
    BOOTSTRAP_IDENTIFY_RESPONSE_LEN, IdentifyResponse, decode_identify_response, encode_identify,
};
pub use demux::{Demuxer, Frame, KlipperFrame, PollOutcome, StreamError};
pub use frame::{
    CHANNEL_CONTROL, CHANNEL_EVENTS, FRAME_SYNC, FrameError, decode_frame, encode_frame,
};
pub use mcu_protocol::{MessageKind, PROTO_VERSION, SCHEMA_HASH};
