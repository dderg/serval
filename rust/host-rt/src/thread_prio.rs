//! Elevate a deadline-carrying host thread to SCHED_FIFO.
//!
//! The host process runs unprivileged; SCHED_FIFO for it requires
//! RLIMIT_RTPRIO > 0 (LimitRTPRIO in the klipper systemd unit). Elevation is
//! best-effort: a bench without the limit keeps working at SCHED_OTHER, it
//! just stays exposed to CPU-burst starvation — the warn says how to fix it.

/// Priorities sit far below the EtherCAT endpoint's FIFO 80: these threads
/// have millisecond deadlines, not the endpoint's 250 us cycle.
pub const PUMP_RT_PRIORITY: i32 = 10;

#[cfg(not(target_os = "linux"))]
pub fn elevate_current_thread(_priority: i32, _name: &str) {}

#[cfg(target_os = "linux")]
pub fn elevate_current_thread(priority: i32, name: &str) {
    let param = libc::sched_param {
        sched_priority: priority,
    };
    let rc = unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
    if rc == 0 {
        tracing::info!(
            subsystem = "motion",
            event = "thread_rt_elevated",
            thread = name,
            priority,
            "thread elevated to SCHED_FIFO"
        );
    } else {
        tracing::warn!(
            subsystem = "motion",
            event = "thread_rt_denied",
            thread = name,
            priority,
            errno = rc,
            "SCHED_FIFO denied — thread stays SCHED_OTHER and can be starved \
             by CPU bursts; grant RLIMIT_RTPRIO (LimitRTPRIO= in the klipper \
             systemd unit)"
        );
    }
}
