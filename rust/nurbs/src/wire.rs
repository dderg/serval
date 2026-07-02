pub const FORMAT_VERSION_V1: u8 = 0x01;

#[allow(dead_code)]
const MAX_SCALAR_ALIGN_BYTES: usize = core::mem::align_of::<f64>();
pub const SCALAR_HEADER_BYTES: usize = 8;
pub const VECTOR_HEADER_BYTES: usize = 8;
pub const ARC_LENGTH_HEADER_BYTES: usize = 8;

const _: () = {
    assert!(SCALAR_HEADER_BYTES % MAX_SCALAR_ALIGN_BYTES == 0);
    assert!(VECTOR_HEADER_BYTES % MAX_SCALAR_ALIGN_BYTES == 0);
    assert!(ARC_LENGTH_HEADER_BYTES % MAX_SCALAR_ALIGN_BYTES == 0);
};
