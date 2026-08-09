pub const PHASE_LUT_SIZE: usize = 1024;

pub const COIL_AMPLITUDE: i16 = 248;

include!(concat!(env!("OUT_DIR"), "/phase_lut_table.rs"));

#[cfg(test)]
mod tests;
