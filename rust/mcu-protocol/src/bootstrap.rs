pub const IDENTIFY_BODY_LEN: usize = 1;

// IdentifyResponse body (81 bytes, frozen):
//  0     proto_version : u8
//  1..5  firmware_ver  : u32_le
//  5..25 build_hash    : [u8; 20]
// 25..57 schema_hash   : [u8; 32]
// 57..61 reset_epoch   : u32_le
// 61..69 capabilities  : u64_le
// 69..81 mcu_serial    : [u8; 12]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifyResponse {
    pub proto_version: u8,
    pub firmware_ver: u32,
    pub build_hash: [u8; 20],
    pub schema_hash: [u8; 32],
    pub reset_epoch: u32,
    pub capabilities: u64,
    pub mcu_serial: [u8; 12],
}

pub const IDENTIFY_RESPONSE_BODY_LEN: usize = 81;

impl IdentifyResponse {
    pub fn encode_body(&self, out: &mut Vec<u8>) {
        let arr = self.encode_body_to_array();
        out.extend_from_slice(&arr);
    }

    pub fn encode_body_to_array(&self) -> [u8; IDENTIFY_RESPONSE_BODY_LEN] {
        let mut b = [0u8; IDENTIFY_RESPONSE_BODY_LEN];
        b[0] = self.proto_version;
        b[1..5].copy_from_slice(&self.firmware_ver.to_le_bytes());
        b[5..25].copy_from_slice(&self.build_hash);
        b[25..57].copy_from_slice(&self.schema_hash);
        b[57..61].copy_from_slice(&self.reset_epoch.to_le_bytes());
        b[61..69].copy_from_slice(&self.capabilities.to_le_bytes());
        b[69..81].copy_from_slice(&self.mcu_serial);
        b
    }

    #[allow(clippy::range_plus_one)]
    pub fn decode_body(buf: &[u8]) -> Result<Self, BootstrapDecodeError> {
        if buf.len() != IDENTIFY_RESPONSE_BODY_LEN {
            return Err(BootstrapDecodeError::WrongLength {
                expected: IDENTIFY_RESPONSE_BODY_LEN,
                got: buf.len(),
            });
        }
        let mut build_hash = [0u8; 20];
        build_hash.copy_from_slice(&buf[5..25]);
        let mut schema_hash = [0u8; 32];
        schema_hash.copy_from_slice(&buf[25..57]);
        let mut mcu_serial = [0u8; 12];
        mcu_serial.copy_from_slice(&buf[69..81]);
        Ok(Self {
            proto_version: buf[0],
            firmware_ver: u32::from_le_bytes(buf[1..5].try_into().expect("range checked above")),
            build_hash,
            schema_hash,
            reset_epoch: u32::from_le_bytes(buf[57..61].try_into().expect("range checked above")),
            capabilities: u64::from_le_bytes(buf[61..69].try_into().expect("range checked above")),
            mcu_serial,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootstrapDecodeError {
    WrongLength { expected: usize, got: usize },
}

impl core::fmt::Display for BootstrapDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::WrongLength { expected, got } => write!(
                f,
                "bootstrap message wrong length: expected {expected} bytes, got {got}"
            ),
        }
    }
}

impl std::error::Error for BootstrapDecodeError {}

#[cfg(test)]
mod tests;
