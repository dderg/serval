//! Host-side EtherCAT motion-node endpoint: decodes the kalico-native piece
//! stream and streams CSP position to an A6-EC servo over EtherCAT/DC.
pub mod scale;
pub mod wire;
pub mod curves;
pub mod ffi;
