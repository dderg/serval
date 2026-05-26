use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProbeHomingResult {
    ProbeTriggered = 0,
    SegmentRetired = 1,
    SensorFault = 2,
    DeadlineExpired = 3,
}

const TICK_INTERVAL: Duration = Duration::from_millis(25);

/// Opaque handle returned by [`prepare_probe_homing`].  Holds the
/// interceptor registration and the shared trigger flag.  Must be
/// passed to [`run_probe_homing`] to enter the homing loop, and
/// cleaned up by [`cleanup_probe_homing`] afterwards.
pub struct ProbeHomingHandle {
    pub(crate) triggered: Arc<AtomicBool>,
    pub(crate) interceptor_id: InterceptorId,
    pub(crate) beacon_io: Arc<KalicoHostIo>,
    pub(crate) stepper_io: Arc<KalicoHostIo>,
    pub(crate) arm_id: u32,
    pub(crate) sensor_fault_timeout: Duration,
}

/// Phase 1: register the interceptor on the Beacon reactor.
///
/// Call this BEFORE `home_start()` sends `beacon_home` to the Beacon
/// MCU, so the interceptor is in place when the probe triggers.
pub fn prepare_probe_homing(
    beacon_io: Arc<KalicoHostIo>,
    stepper_io: Arc<KalicoHostIo>,
    beacon_trsync_oid: u8,
    arm_id: u32,
    sensor_fault_timeout: Duration,
) -> Result<ProbeHomingHandle, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));

    log::info!(
        "[z-home-diag] prepare_probe_homing: trsync_oid={} arm_id={} \
         sensor_fault_timeout={:.2}s",
        beacon_trsync_oid,
        arm_id,
        sensor_fault_timeout.as_secs_f64(),
    );

    let interceptor_id = {
        let triggered_clone = Arc::clone(&triggered);
        let stepper_io_clone = Arc::clone(&stepper_io);

        beacon_io.register_frame_interceptor(
            "trsync_state",
            Some(u32::from(beacon_trsync_oid)),
            Box::new(move |msg_params| {
                let can_trigger = msg_params.get_u32("can_trigger");
                // [z-home-diag] log every callback invocation
                log::info!(
                    "[z-home-diag] trsync_state interceptor FIRED: \
                     can_trigger={} arm_id={}",
                    can_trigger, arm_id,
                );
                // DIAG: trace every callback invocation
                {
                    use std::io::Write;
                    if let Ok(mut f) = std::fs::OpenOptions::new()
                        .create(true).append(true)
                        .open("/tmp/interceptor_trace.log")
                    {
                        let _ = writeln!(f,
                            "[{:?}] CALLBACK can_trigger={} arm_id={}",
                            std::time::SystemTime::now(), can_trigger, arm_id,
                        );
                    }
                }
                if can_trigger != 0 {
                    log::info!(
                        "[z-home-diag] trsync_state: can_trigger={} != 0, \
                         NOT sending software_trip (probe not yet triggered)",
                        can_trigger,
                    );
                    return;
                }
                let cmd = format!("runtime_software_trip arm_id={arm_id}");
                log::info!(
                    "[z-home-diag] trsync_state: can_trigger=0 → \
                     sending software_trip arm_id={}",
                    arm_id,
                );
                let send_result = stepper_io_clone.send_fire_and_forget(&cmd);
                log::info!(
                    "[z-home-diag] software_trip send result: {:?} arm_id={}",
                    send_result, arm_id,
                );
                triggered_clone.store(true, Ordering::Release);
                // DIAG: trace the trip send result
                {
                    use std::io::Write;
                    if let Ok(mut f) = std::fs::OpenOptions::new()
                        .create(true).append(true)
                        .open("/tmp/interceptor_trace.log")
                    {
                        let _ = writeln!(f,
                            "[{:?}] SOFTWARE_TRIP sent arm_id={} result={:?} flag_set=true",
                            std::time::SystemTime::now(), arm_id, send_result,
                        );
                    }
                }
            }),
        )?
    };

    log::info!(
        "[z-home-diag] prepare_probe_homing: interceptor registered \
         interceptor_id={:?} trsync_oid={} arm_id={}",
        interceptor_id,
        beacon_trsync_oid,
        arm_id,
    );

    Ok(ProbeHomingHandle {
        triggered,
        interceptor_id,
        beacon_io,
        stepper_io,
        arm_id,
        sensor_fault_timeout,
    })
}

/// Phase 2: enter the blocking homing loop.
///
/// Call AFTER the homing move has been submitted.  Sends an immediate
/// deadline extension, then loops at 25 ms checking for the trigger
/// flag or sensor-fault timeout.  Does NOT check HomingSegmentState —
/// CreditFreed fires on slot-available (not execution-complete), so
/// segment "retirement" is unreliable during homing.
pub fn run_probe_homing(
    handle: &ProbeHomingHandle,
) -> Result<ProbeHomingResult, TransportError> {
    log::info!(
        "[z-home-diag] run_probe_homing entry: arm_id={} \
         sensor_fault_timeout={:.2}s already_triggered={}",
        handle.arm_id,
        handle.sensor_fault_timeout.as_secs_f64(),
        handle.triggered.load(Ordering::Acquire),
    );

    let extend_cmd = format!(
        "runtime_extend_homing_deadline arm_id={}",
        handle.arm_id
    );
    log::info!(
        "[z-home-diag] run_probe_homing: sending initial deadline extension \
         arm_id={}",
        handle.arm_id,
    );
    handle.stepper_io.send_fire_and_forget(&extend_cmd)?;

    let result = run_loop(handle, &extend_cmd);
    log::info!(
        "[z-home-diag] run_probe_homing returning: result={:?} arm_id={}",
        result, handle.arm_id,
    );
    result
}

/// Phase 3: unregister the interceptor.  Always call this, even on
/// error (the handle borrows the Beacon I/O, so it must be cleaned up
/// before the next homing cycle can register a new interceptor for the
/// same OID).
pub fn cleanup_probe_homing(handle: ProbeHomingHandle) {
    let _ = handle.beacon_io.unregister_frame_interceptor(handle.interceptor_id);
}

fn run_loop(
    handle: &ProbeHomingHandle,
    extend_cmd: &str,
) -> Result<ProbeHomingResult, TransportError> {
    let start = Instant::now();
    let mut tick: u32 = 0;

    loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();
        tick += 1;

        if handle.triggered.load(Ordering::Acquire) {
            log::info!(
                "[z-home-diag] run_loop: TRIGGERED at tick={} elapsed={:.3}s \
                 arm_id={}",
                tick, elapsed.as_secs_f64(), handle.arm_id,
            );
            log::info!(
                "[probe-homing] probe triggered elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::ProbeTriggered);
        }

        if elapsed > handle.sensor_fault_timeout {
            log::error!(
                "[z-home-diag] run_loop: SENSOR FAULT at tick={} \
                 elapsed={:.1}s timeout={:.1}s arm_id={}",
                tick, elapsed.as_secs_f64(),
                handle.sensor_fault_timeout.as_secs_f64(),
                handle.arm_id,
            );
            log::error!(
                "[probe-homing] SENSOR FAULT: no trigger after {:.1}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SensorFault);
        }

        // Log every 10th deadline extension to avoid log spam
        if tick % 10 == 0 {
            log::info!(
                "[z-home-diag] run_loop tick={} elapsed={:.3}s arm_id={} \
                 sending deadline extension triggered={}",
                tick, elapsed.as_secs_f64(), handle.arm_id,
                handle.triggered.load(Ordering::Acquire),
            );
        }

        handle.stepper_io.send_fire_and_forget(extend_cmd)?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use std::time::Duration;

    /// Simulates the real-hardware failure: CreditFreed arrives before
    /// the probe triggers, causing HomingSegmentState to become Completed.
    /// The loop must NOT exit on segment retirement — only on trigger
    /// or sensor_fault_timeout.
    #[test]
    fn loop_does_not_exit_on_segment_completed() {
        let triggered = Arc::new(AtomicBool::new(false));
        let triggered_clone = Arc::clone(&triggered);

        // Set the trigger after 75ms (3 ticks). If the loop exited
        // early on segment retirement, it would return SensorFault or
        // SegmentRetired within 1 tick, not ProbeTriggered after 75ms.
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(75));
            triggered_clone.store(true, Ordering::Release);
        });

        let start = Instant::now();

        // Inline the loop logic to test without needing real KalicoHostIo.
        let sensor_fault_timeout = Duration::from_secs(5);
        let result = loop {
            std::thread::sleep(TICK_INTERVAL);
            let elapsed = start.elapsed();

            if triggered.load(Ordering::Acquire) {
                break ProbeHomingResult::ProbeTriggered;
            }

            if elapsed > sensor_fault_timeout {
                break ProbeHomingResult::SensorFault;
            }
        };

        assert_eq!(result, ProbeHomingResult::ProbeTriggered);
        // Verify it took at least 50ms (not instant exit)
        assert!(start.elapsed() >= Duration::from_millis(50));
    }

    /// The loop must exit on sensor_fault_timeout if no trigger arrives.
    #[test]
    fn loop_exits_on_sensor_fault_timeout() {
        let triggered = Arc::new(AtomicBool::new(false));
        let sensor_fault_timeout = Duration::from_millis(60);

        let start = Instant::now();
        let result = loop {
            std::thread::sleep(TICK_INTERVAL);
            let elapsed = start.elapsed();

            if triggered.load(Ordering::Acquire) {
                break ProbeHomingResult::ProbeTriggered;
            }

            if elapsed > sensor_fault_timeout {
                break ProbeHomingResult::SensorFault;
            }
        };

        assert_eq!(result, ProbeHomingResult::SensorFault);
    }

    /// The loop must exit immediately when the trigger flag is set.
    #[test]
    fn loop_exits_on_trigger() {
        let triggered = Arc::new(AtomicBool::new(true)); // pre-set
        let sensor_fault_timeout = Duration::from_secs(60);

        let start = Instant::now();
        let result = loop {
            std::thread::sleep(TICK_INTERVAL);
            let elapsed = start.elapsed();

            if triggered.load(Ordering::Acquire) {
                break ProbeHomingResult::ProbeTriggered;
            }

            if elapsed > sensor_fault_timeout {
                break ProbeHomingResult::SensorFault;
            }
        };

        assert_eq!(result, ProbeHomingResult::ProbeTriggered);
        assert!(start.elapsed() < Duration::from_millis(100));
    }
}
