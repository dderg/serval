mod router;

pub use router::{
    ClockRecordSnapshot, DEGRADED_CLOCK_RECORD_AGE_SECS, MAX_CLOCK_RECORD_AGE_SECS, McuHandle,
    PassthroughRouter, RouterError,
};
