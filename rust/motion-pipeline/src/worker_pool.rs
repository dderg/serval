//! The shaper's persistent fan-out threads. Fitting a window splits into
//! independent jobs; spawning threads per call cost more than many of the
//! jobs, so the shaper keeps idle helpers parked on a queue instead and hands
//! each fan-out to the calling thread plus whichever helpers are free.

use std::any::Any;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};
use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, unbounded};

type Task<'a> = dyn Fn() + Sync + 'a;
type Panic = Box<dyn Any + Send>;

/// The most helpers a pool keeps. klippy runs under `mlockall`, so every
/// helper pins a full stack and its own malloc arena for the life of the
/// process; on the dense replay at 16 cores, throughput stopped growing at
/// one helper and three already cover a four-core host's spare cores.
const HELPERS_MAX: usize = 3;

/// Helper threads shared by every fan-out of one shaper. They are spawned on
/// the first fan-out, so they inherit the scheduling of the thread that runs
/// the shaper, and a shaper that never fans out never spawns any.
pub(crate) struct WorkerPool {
    helper_count: usize,
    helpers: OnceLock<Helpers>,
}

struct Helpers {
    calls: Sender<Arc<Call>>,
    threads: Vec<JoinHandle<()>>,
}

/// One fan-out. Helpers may take its task only while it is open; the caller
/// closes it and waits for every helper that took it before the task's
/// borrow ends.
struct Call {
    state: Mutex<CallState>,
    left: Condvar,
}

struct CallState {
    task: Option<&'static Task<'static>>,
    running: usize,
    panic: Option<Panic>,
}

impl Call {
    fn lock(&self) -> MutexGuard<'_, CallState> {
        self.state
            .lock()
            .expect("shaper worker call state poisoned")
    }

    fn help(&self) {
        let task = {
            let mut state = self.lock();
            let Some(task) = state.task else { return };
            state.running += 1;
            task
        };
        let outcome = catch_unwind(AssertUnwindSafe(task));
        let mut state = self.lock();
        state.running -= 1;
        if let Err(panic) = outcome {
            state.panic.get_or_insert(panic);
        }
        if state.running == 0 {
            self.left.notify_all();
        }
    }

    fn close_and_wait(&self) -> Option<Panic> {
        let mut state = self.lock();
        state.task = None;
        while state.running > 0 {
            state = self
                .left
                .wait(state)
                .expect("shaper worker call state poisoned");
        }
        state.panic.take()
    }
}

struct OpenCall<'a>(&'a Call);

impl Drop for OpenCall<'_> {
    fn drop(&mut self) {
        self.0.close_and_wait();
    }
}

impl WorkerPool {
    /// A pool sized to the cores the calling thread does not occupy, up to
    /// [`HELPERS_MAX`].
    pub(crate) fn new() -> Self {
        Self::with_helpers(
            std::thread::available_parallelism()
                .map_or(0, |cores| (cores.get() - 1).min(HELPERS_MAX)),
        )
    }

    pub(crate) fn with_helpers(helper_count: usize) -> Self {
        Self {
            helper_count,
            helpers: OnceLock::new(),
        }
    }

    fn helpers(&self) -> &Helpers {
        self.helpers.get_or_init(|| {
            let (calls, queue) = unbounded();
            let threads = (0..self.helper_count)
                .map(|index| {
                    let queue: Receiver<Arc<Call>> = queue.clone();
                    std::thread::Builder::new()
                        .name(format!("kalico-shaper-{index}"))
                        .spawn(move || {
                            for call in queue {
                                call.help();
                            }
                        })
                        .expect("spawn shaper worker thread")
                })
                .collect();
            Helpers { calls, threads }
        })
    }

    /// Runs `run_job` for every index in `0..jobs` on up to `participants`
    /// threads — the caller and free helpers — pulling indices from one
    /// shared counter, and returns the results in index order. Each
    /// participant builds its own `make_state` before its first job. With one
    /// participant everything runs inline and a panic unwinds straight
    /// through; with more, a panic on any participant is caught, the others
    /// finish, and it returns as `Err` the way `JoinHandle::join` reports a
    /// panicked thread.
    pub(crate) fn map_indexed<S, T: Send>(
        &self,
        participants: usize,
        jobs: usize,
        make_state: impl Fn() -> S + Sync,
        run_job: impl Fn(&S, usize) -> T + Sync,
    ) -> std::thread::Result<Vec<T>> {
        if participants <= 1 {
            let state = make_state();
            return Ok((0..jobs).map(|index| run_job(&state, index)).collect());
        }
        let next_job = AtomicUsize::new(0);
        let done = Mutex::new(Vec::with_capacity(jobs));
        let participate = || {
            let mut index = next_job.fetch_add(1, Ordering::Relaxed);
            if index >= jobs {
                return;
            }
            let state = make_state();
            let mut local = Vec::new();
            while index < jobs {
                local.push((index, run_job(&state, index)));
                index = next_job.fetch_add(1, Ordering::Relaxed);
            }
            done.lock()
                .expect("shaper worker results poisoned")
                .append(&mut local);
        };
        self.fan_out(participants, &participate)?;
        let mut done = done.into_inner().expect("shaper worker results poisoned");
        assert_eq!(done.len(), jobs, "shaper workers dropped jobs");
        done.sort_by_key(|(index, _)| *index);
        Ok(done.into_iter().map(|(_, result)| result).collect())
    }

    /// Runs `task` on the calling thread and on up to `participants - 1`
    /// helpers at once. Helpers reach `task` with its lifetime erased, which
    /// is sound because `OpenCall` closes the call and waits out every helper
    /// that took it before this returns or unwinds, and a closed call hands
    /// the task to no one.
    #[allow(unsafe_code)]
    fn fan_out(&self, participants: usize, task: &Task<'_>) -> std::thread::Result<()> {
        let helpers = self.helpers();
        let task = unsafe { std::mem::transmute::<&Task<'_>, &'static Task<'static>>(task) };
        let call = Arc::new(Call {
            state: Mutex::new(CallState {
                task: Some(task),
                running: 0,
                panic: None,
            }),
            left: Condvar::new(),
        });
        let open = OpenCall(&call);
        for _ in 1..participants.min(helpers.threads.len() + 1) {
            helpers
                .calls
                .send(Arc::clone(&call))
                .expect("shaper worker queue closed");
        }
        let own = catch_unwind(AssertUnwindSafe(task));
        std::mem::forget(open);
        let helper_panic = call.close_and_wait();
        own?;
        helper_panic.map_or(Ok(()), Err)
    }
}

impl Drop for WorkerPool {
    fn drop(&mut self) {
        let Some(Helpers { calls, threads }) = self.helpers.take() else {
            return;
        };
        drop(calls);
        for thread in threads {
            thread.join().expect("shaper worker thread panicked");
        }
    }
}

#[cfg(test)]
#[path = "worker_pool_tests.rs"]
mod tests;
