#![allow(unsafe_code)]

#[cfg(feature = "hw")]
use crate::ffi;
use crate::ffi::EcTelemetry;

pub(super) trait DriveChain {
    fn cycle_time_ns(&self) -> u64;
    fn cycle(&mut self) -> (i32, i64);
    fn enable(&mut self, slot: usize) -> i32;
    fn disable(&mut self, slot: usize);
    fn shutdown(&mut self);
    fn set_target_position(&mut self, slot: usize, counts: i32);
    fn set_velocity_offset(&mut self, slot: usize, counts_per_s: i32);
    fn set_torque_offset(&mut self, slot: usize, tenths_pct: i16);
    fn position_actual(&self, slot: usize) -> i32;
    fn velocity_actual(&self, slot: usize) -> i32;
    fn torque_actual(&self, slot: usize) -> i16;
    fn error_code(&self, slot: usize) -> u16;
    fn telemetry(&self, slot: usize) -> EcTelemetry;
    fn dump_al_state(&self);

    fn reanchor_count(&self) -> u32 {
        0
    }

    /// (wake_late_ns, recv_ns, send_ns) of the last exchange — the stage
    /// breakdown that attributes a frame-timing spike to kernel wakeup
    /// latency vs the polled bus receive vs the send path.
    fn cycle_stage_ns(&self) -> (i64, i64, i64) {
        (0, 0, 0)
    }

    fn disable_all(&mut self, num_slaves: usize) {
        for s in 0..num_slaves {
            self.disable(s);
        }
    }

    fn shutdown_and_exit(&mut self, num_slaves: usize) -> ! {
        self.disable_all(num_slaves);
        self.shutdown();
        std::process::exit(1);
    }
}

#[cfg(feature = "hw")]
pub(super) struct FfiDriveChain;

#[cfg(feature = "hw")]
fn c_slot(slot: usize) -> std::os::raw::c_int {
    std::os::raw::c_int::try_from(slot).expect("slave slot index exceeds c_int")
}

#[cfg(feature = "hw")]
impl DriveChain for FfiDriveChain {
    fn cycle_time_ns(&self) -> u64 {
        unsafe { ffi::ec_rt_cycle_time_ns() }
    }

    fn cycle(&mut self) -> (i32, i64) {
        let mut toff = 0i64;
        let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };
        (wkc, toff)
    }

    fn reanchor_count(&self) -> u32 {
        unsafe { ffi::ec_rt_reanchor_count() }
    }
    fn cycle_stage_ns(&self) -> (i64, i64, i64) {
        let (mut wake_late, mut recv, mut send) = (0i64, 0i64, 0i64);
        unsafe { ffi::ec_rt_cycle_stage_ns(&mut wake_late, &mut recv, &mut send) };
        (wake_late, recv, send)
    }

    fn enable(&mut self, slot: usize) -> i32 {
        unsafe { ffi::ec_rt_enable(c_slot(slot)) }
    }

    fn disable(&mut self, slot: usize) {
        unsafe { ffi::ec_rt_disable(c_slot(slot)) }
    }

    fn shutdown(&mut self) {
        unsafe { ffi::ec_rt_shutdown() }
    }

    fn set_target_position(&mut self, slot: usize, counts: i32) {
        unsafe { ffi::ec_rt_set_target_position(c_slot(slot), counts) }
    }

    fn set_velocity_offset(&mut self, slot: usize, counts_per_s: i32) {
        unsafe { ffi::ec_rt_set_velocity_offset(c_slot(slot), counts_per_s) }
    }

    fn set_torque_offset(&mut self, slot: usize, tenths_pct: i16) {
        unsafe { ffi::ec_rt_set_torque_offset(c_slot(slot), tenths_pct) }
    }

    fn position_actual(&self, slot: usize) -> i32 {
        unsafe { ffi::ec_rt_get_position_actual(c_slot(slot)) }
    }

    fn velocity_actual(&self, slot: usize) -> i32 {
        unsafe { ffi::ec_rt_get_velocity_actual(c_slot(slot)) }
    }

    fn torque_actual(&self, slot: usize) -> i16 {
        unsafe { ffi::ec_rt_get_torque_actual(c_slot(slot)) }
    }

    fn error_code(&self, slot: usize) -> u16 {
        unsafe { ffi::ec_rt_get_error_code(c_slot(slot)) }
    }

    fn telemetry(&self, slot: usize) -> EcTelemetry {
        let mut t = EcTelemetry::default();
        unsafe { ffi::ec_rt_get_telemetry(c_slot(slot), &mut t) };
        t
    }

    fn dump_al_state(&self) {
        unsafe { ffi::ec_rt_dump_al_state() }
    }
}
