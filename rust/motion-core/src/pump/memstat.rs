#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct MemPressureSample {
    pub(super) majflt: u64,
    pub(super) vm_swap_kb: u64,
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(super) fn parse_thread_stat_majflt(stat: &str) -> Result<u64, String> {
    const MAJFLT_INDEX_AFTER_COMM: usize = 9;
    let after_comm = stat
        .rsplit_once(')')
        .ok_or_else(|| format!("no ')' after comm in thread stat {stat:?}"))?
        .1;
    let field = after_comm
        .split_ascii_whitespace()
        .nth(MAJFLT_INDEX_AFTER_COMM)
        .ok_or_else(|| format!("thread stat has too few fields for majflt: {stat:?}"))?;
    field
        .parse()
        .map_err(|e| format!("majflt field {field:?} is not a u64 ({e}) in thread stat {stat:?}"))
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(super) fn parse_status_vm_swap_kb(status: &str) -> Result<u64, String> {
    let value = status
        .lines()
        .find_map(|line| line.strip_prefix("VmSwap:"))
        .ok_or("no VmSwap line in /proc/self/status")?;
    let number = value
        .split_ascii_whitespace()
        .next()
        .ok_or_else(|| format!("empty VmSwap value {value:?}"))?;
    number
        .parse()
        .map_err(|e| format!("VmSwap value {number:?} is not a u64 ({e})"))
}

#[cfg(target_os = "linux")]
mod imp {
    use std::fs::File;
    use std::os::unix::fs::FileExt;

    const PROC_READ_BUF_LEN: usize = 16 * 1024;

    pub(super) struct ProcFiles {
        thread_stat: File,
        process_status: File,
        buf: Box<[u8; PROC_READ_BUF_LEN]>,
    }

    impl ProcFiles {
        pub(super) fn open() -> Result<Self, String> {
            Ok(Self {
                thread_stat: File::open("/proc/thread-self/stat")
                    .map_err(|e| format!("open /proc/thread-self/stat: {e}"))?,
                process_status: File::open("/proc/self/status")
                    .map_err(|e| format!("open /proc/self/status: {e}"))?,
                buf: Box::new([0u8; PROC_READ_BUF_LEN]),
            })
        }

        pub(super) fn read_sample(&mut self) -> Result<super::MemPressureSample, String> {
            let stat = read_from_start(&self.thread_stat, &mut self.buf[..])
                .map_err(|e| format!("read /proc/thread-self/stat: {e}"))?;
            let majflt = super::parse_thread_stat_majflt(stat)?;
            let status = read_from_start(&self.process_status, &mut self.buf[..])
                .map_err(|e| format!("read /proc/self/status: {e}"))?;
            let vm_swap_kb = super::parse_status_vm_swap_kb(status)?;
            Ok(super::MemPressureSample { majflt, vm_swap_kb })
        }
    }

    fn read_from_start<'a>(file: &File, buf: &'a mut [u8]) -> Result<&'a str, String> {
        let mut total = 0;
        loop {
            let n = file
                .read_at(&mut buf[total..], total as u64)
                .map_err(|e| e.to_string())?;
            if n == 0 {
                break;
            }
            total += n;
            if total == buf.len() {
                return Err(format!("content exceeds {}-byte buffer", buf.len()));
            }
        }
        std::str::from_utf8(&buf[..total]).map_err(|e| e.to_string())
    }
}

#[cfg(target_os = "linux")]
pub(super) struct MemPressureProbe {
    state: ProbeState,
}

#[cfg(target_os = "linux")]
enum ProbeState {
    Unopened,
    Open(imp::ProcFiles),
    Unavailable,
}

#[cfg(target_os = "linux")]
impl MemPressureProbe {
    pub(super) fn new() -> Self {
        Self {
            state: ProbeState::Unopened,
        }
    }

    pub(super) fn sample(&mut self) -> Option<MemPressureSample> {
        if matches!(self.state, ProbeState::Unopened) {
            self.state = match imp::ProcFiles::open() {
                Ok(files) => ProbeState::Open(files),
                Err(e) => {
                    log_unavailable(&e);
                    ProbeState::Unavailable
                }
            };
        }
        let ProbeState::Open(files) = &mut self.state else {
            return None;
        };
        match files.read_sample() {
            Ok(sample) => Some(sample),
            Err(e) => {
                log_unavailable(&e);
                self.state = ProbeState::Unavailable;
                None
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn log_unavailable(error: &str) {
    tracing::warn!(
        subsystem = "motion",
        event = "pump_memstat_unavailable",
        error,
        "[pump-send] memory-pressure diagnostic unavailable — pump_send_blocked events \
         will carry no majflt/VmSwap evidence: {error}"
    );
}

#[cfg(not(target_os = "linux"))]
pub(super) struct MemPressureProbe;

#[cfg(not(target_os = "linux"))]
impl MemPressureProbe {
    pub(super) fn new() -> Self {
        Self
    }

    pub(super) fn sample(&mut self) -> Option<MemPressureSample> {
        None
    }
}
