//! A playhead clock the test controls (#1006). It stands still while any
//! thread of the test process runs or waits for a CPU, and jumps to the
//! earliest timed wait once every one of them is blocked, as the
//! simulator's virtual clock does. What the pacer decides on it follows
//! from the order of events alone: a loaded machine slows the test down
//! but cannot starve the pipeline of playhead time. Only `sleep` and the
//! ingress's timed waits cost time; computing is free.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};

use super::super::StreamMsg;
use super::super::playhead_clock::PlayheadClock;
use crate::lock_ext::LockExt;

/// How often the driver looks again while a thread runs or none waits.
const DRIVER_POLL: Duration = Duration::from_micros(100);

/// Threads whose work never feeds back into the pipeline, woken on the
/// wall clock: on a loaded machine the dispatch record's writer, flushing
/// every 100 ms, is always waiting for a CPU, and the clock would never
/// move.
const UNTIMED_THREADS: [&str; 1] = ["dispatch-record"];

pub(super) struct VirtualClock {
    state: Mutex<Waits>,
}

struct Waits {
    now: Instant,
    next_id: u64,
    pending: BTreeMap<u64, TimedWait>,
}

struct TimedWait {
    deadline: Instant,
    wake: Sender<()>,
}

impl VirtualClock {
    /// The clock and the thread that advances it, which ends with the
    /// clock's last owner.
    pub(super) fn start() -> Arc<Self> {
        let clock = Arc::new(Self {
            state: Mutex::new(Waits {
                now: Instant::now(),
                next_id: 0,
                pending: BTreeMap::new(),
            }),
        });
        let driven = Arc::downgrade(&clock);
        std::thread::Builder::new()
            .name("virtual-clock".into())
            .spawn(move || drive(&driven))
            .expect("spawn the virtual clock driver");
        clock
    }

    pub(super) fn now(&self) -> Instant {
        self.state.lock_ok().now
    }

    pub(super) fn sleep(&self, duration: Duration) {
        let (id, woken) = self.wait_until(self.now() + duration);
        woken
            .recv()
            .expect("the driver wakes every wait it removes");
        self.forget(id);
    }

    pub(super) fn elapsed_since(&self, earlier: Instant) -> Duration {
        self.now().saturating_duration_since(earlier)
    }

    fn wait_until(&self, deadline: Instant) -> (u64, Receiver<()>) {
        let (wake, woken) = crossbeam_channel::bounded(1);
        let mut waits = self.state.lock_ok();
        let id = waits.next_id;
        waits.next_id += 1;
        waits.pending.insert(id, TimedWait { deadline, wake });
        (id, woken)
    }

    fn forget(&self, id: u64) {
        self.state.lock_ok().pending.remove(&id);
    }

    /// Moves to the earliest deadline and wakes every wait due by then;
    /// false when nothing waits on time.
    fn advance(&self) -> bool {
        let mut waits = self.state.lock_ok();
        let Some(earliest) = waits.pending.values().map(|wait| wait.deadline).min() else {
            return false;
        };
        waits.now = waits.now.max(earliest);
        let now = waits.now;
        waits.pending.retain(|_, wait| {
            if wait.deadline > now {
                return true;
            }
            let _ = wait.wake.try_send(());
            false
        });
        true
    }

    fn has_waits(&self) -> bool {
        !self.state.lock_ok().pending.is_empty()
    }
}

impl host_rt::clock::Clock for VirtualClock {
    fn now(&self) -> Instant {
        VirtualClock::now(self)
    }
}

impl PlayheadClock for VirtualClock {
    fn now(&self) -> Instant {
        VirtualClock::now(self)
    }

    fn recv_until(
        &self,
        rx: &Receiver<StreamMsg>,
        deadline: Instant,
    ) -> Result<StreamMsg, RecvTimeoutError> {
        if PlayheadClock::now(self) >= deadline {
            return rx.try_recv().map_err(|error| match error {
                crossbeam_channel::TryRecvError::Empty => RecvTimeoutError::Timeout,
                crossbeam_channel::TryRecvError::Disconnected => RecvTimeoutError::Disconnected,
            });
        }
        let (id, woken) = self.wait_until(deadline);
        let received = crossbeam_channel::select! {
            recv(rx) -> msg => msg.map_err(|_| RecvTimeoutError::Disconnected),
            recv(woken) -> _ => Err(RecvTimeoutError::Timeout),
        };
        self.forget(id);
        received
    }
}

fn drive(clock: &Weak<VirtualClock>) {
    let driver = own_thread_id();
    loop {
        let Some(clock) = clock.upgrade() else {
            return;
        };
        if clock.has_waits() && every_other_thread_blocked(&driver) && clock.advance() {
            continue;
        }
        drop(clock);
        std::thread::sleep(DRIVER_POLL);
    }
}

fn own_thread_id() -> String {
    let link = std::fs::read_link("/proc/thread-self").expect("read /proc/thread-self");
    link.file_name()
        .and_then(|tid| tid.to_str())
        .expect("/proc/thread-self ends in the thread id")
        .to_string()
}

/// Every other thread blocked across two passes in which none was
/// scheduled: a thread woken between the passes is runnable in the second
/// or ran, which its timeslice count shows.
fn every_other_thread_blocked(driver: &str) -> bool {
    let Some(first) = blocked_threads(driver) else {
        return false;
    };
    blocked_threads(driver).is_some_and(|second| second == first)
}

/// Each other thread's id and timeslice count, or `None` when one is
/// running or runnable.
fn blocked_threads(driver: &str) -> Option<Vec<(String, u64)>> {
    let mut threads = Vec::new();
    for entry in std::fs::read_dir("/proc/self/task").expect("list /proc/self/task") {
        let tid = entry.expect("read a /proc/self/task entry").file_name();
        let tid = tid.to_str().expect("thread ids are ASCII").to_string();
        if tid == driver {
            continue;
        }
        let task = format!("/proc/self/task/{tid}");
        let Ok(stat) = std::fs::read_to_string(format!("{task}/stat")) else {
            continue;
        };
        let (name, fields) = stat
            .split_once('(')
            .and_then(|(_, rest)| rest.rsplit_once(')'))
            .expect("a thread's stat holds its name in parentheses");
        if UNTIMED_THREADS.contains(&name) {
            continue;
        }
        let state = fields
            .trim_start()
            .chars()
            .next()
            .expect("a thread's stat holds its state after the name");
        if matches!(state, 'R' | 'D') {
            return None;
        }
        let Ok(schedstat) = std::fs::read_to_string(format!("{task}/schedstat")) else {
            continue;
        };
        let timeslices = schedstat
            .split_whitespace()
            .nth(2)
            .and_then(|count| count.parse().ok())
            .expect("schedstat holds the timeslice count third");
        threads.push((tid, timeslices));
    }
    threads.sort_unstable();
    Some(threads)
}
