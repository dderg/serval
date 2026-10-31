//! New threads inherit the creator's SCHED_FIFO policy, priority, and CPU
//! pin — and go_realtime runs on the thread that later spawns every helper.
//! Two placement rules follow:
//!
//! * Helpers that never touch the EtherCAT master (file I/O, channel drains)
//!   call [`demote_to_normal_scheduling`]: SCHED_OTHER on the housekeeping
//!   cores, so the FIFO DC thread preempts them unconditionally and they stay
//!   off the isolated (`isolcpus`/`nohz_full`) cores.
//!
//! * The CoE mailbox helper shares the EtherCAT master with the DC loop, but
//!   the in-kernel IgH master serializes that access itself and the SDO call
//!   sleeps, so the helper uses [`demote_to_normal_scheduling`] too.

#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
pub fn demote_to_normal_scheduling() {
    unsafe {
        let param = libc::sched_param { sched_priority: 0 };
        let rc = libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_OTHER, &param);
        if rc != 0 {
            panic!("ec-rt helper thread: SCHED_OTHER demotion failed (errno {rc})");
        }
        let isolated = isolated_cpus();
        let mut cpus: libc::cpu_set_t = std::mem::zeroed();
        let mut any = false;
        for cpu in 0..(8 * std::mem::size_of::<libc::cpu_set_t>()) {
            if !isolated.contains(&cpu) {
                libc::CPU_SET(cpu, &mut cpus);
                any = true;
            }
        }
        if !any {
            for cpu in 0..(8 * std::mem::size_of::<libc::cpu_set_t>()) {
                libc::CPU_SET(cpu, &mut cpus);
            }
        }
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &cpus);
    }
}

#[cfg(target_os = "linux")]
fn isolated_cpus() -> Vec<usize> {
    parse_cpu_list(
        std::fs::read_to_string("/sys/devices/system/cpu/isolated")
            .unwrap_or_default()
            .trim(),
    )
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn parse_cpu_list(list: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in list.split(',').filter(|p| !p.is_empty()) {
        match part.split_once('-') {
            Some((lo, hi)) => {
                if let (Ok(lo), Ok(hi)) = (lo.trim().parse::<usize>(), hi.trim().parse::<usize>()) {
                    cpus.extend(lo..=hi);
                }
            }
            None => {
                if let Ok(cpu) = part.trim().parse() {
                    cpus.push(cpu);
                }
            }
        }
    }
    cpus
}

#[cfg(not(target_os = "linux"))]
pub fn demote_to_normal_scheduling() {}

#[cfg(test)]
mod tests;

/// Nonvoluntary context switches of the calling thread — nonzero deltas on
/// the DC thread mean something preempted SCHED_FIFO-80, pointing a
/// frame-timing spike at the kernel (stop_machine, higher-prio kthread)
/// rather than at any stage of the cycle loop.
#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
pub fn thread_nonvoluntary_ctx_switches() -> i64 {
    let mut usage = std::mem::MaybeUninit::<libc::rusage>::zeroed();
    let rc = unsafe { libc::getrusage(libc::RUSAGE_THREAD, usage.as_mut_ptr()) };
    if rc == 0 {
        unsafe { usage.assume_init() }.ru_nivcsw
    } else {
        0
    }
}

#[cfg(not(target_os = "linux"))]
pub fn thread_nonvoluntary_ctx_switches() -> i64 {
    0
}
