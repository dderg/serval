use crate::thermal::HEATERS;

pub const AUX_RECORD_LEN: usize = 44;
pub const INPUTS: usize = 3;
pub const FLAG_MCO_OK: u8 = 1 << 0;
pub const FLAG_DAC_CLAMPED: [u8; HEATERS] = [1 << 1, 1 << 2];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AuxRecord {
    pub time: u64,
    pub mco: u32,
    pub samples: u32,
    pub high: [u32; INPUTS],
    pub temp_centi: [i16; HEATERS],
    pub dac: [u16; HEATERS],
    pub config_id: u16,
    pub endstops: u8,
    pub flags: u8,
    pub dedge: u8,
    pub tmc_writes: u8,
    pub tmc_reads: u8,
    pub tmc_rejects: u8,
}

impl AuxRecord {
    pub fn encode(&self) -> [u8; AUX_RECORD_LEN] {
        let mut out = [0u8; AUX_RECORD_LEN];
        let mut at = 0;
        let mut put = |bytes: &[u8]| {
            out[at..at + bytes.len()].copy_from_slice(bytes);
            at += bytes.len();
        };
        put(&self.time.to_le_bytes());
        put(&self.mco.to_le_bytes());
        put(&self.samples.to_le_bytes());
        for high in self.high {
            put(&high.to_le_bytes());
        }
        for temp in self.temp_centi {
            put(&temp.to_le_bytes());
        }
        for code in self.dac {
            put(&code.to_le_bytes());
        }
        put(&self.config_id.to_le_bytes());
        put(&[
            self.endstops,
            self.flags,
            self.dedge,
            self.tmc_writes,
            self.tmc_reads,
            self.tmc_rejects,
        ]);
        debug_assert_eq!(at, AUX_RECORD_LEN);
        out
    }
}
