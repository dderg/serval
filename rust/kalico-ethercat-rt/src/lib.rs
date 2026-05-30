//! Host-side EtherCAT motion-node endpoint: decodes the kalico-native piece
//! stream and streams CSP position to an A6-EC servo over EtherCAT/DC.
pub mod curves;
pub mod ffi;
pub mod scale;
pub mod server;
pub mod wire;
