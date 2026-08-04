use std::cell::RefCell;
use std::ffi::c_int;

use host_rt::memory_lock::{MEMORY_LOCK_FLAGS, MemoryLockDenied, ProcessMemoryLock};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Mlockall(c_int),
    PipelineThreadsStarted,
}

thread_local! {
    static STEPS: RefCell<Vec<Step>> = const { RefCell::new(Vec::new()) };
    static VERDICT: RefCell<Result<(), i32>> = const { RefCell::new(Ok(())) };
}

fn recording_mlockall(flags: c_int) -> Result<(), i32> {
    STEPS.with_borrow_mut(|steps| steps.push(Step::Mlockall(flags)));
    VERDICT.with_borrow(|v| *v)
}

fn deny(errno: i32) {
    VERDICT.with_borrow_mut(|v| *v = Err(errno));
}

fn steps() -> Vec<Step> {
    STEPS.with_borrow(Clone::clone)
}

fn lock() -> ProcessMemoryLock {
    STEPS.with_borrow_mut(Vec::clear);
    VERDICT.with_borrow_mut(|v| *v = Ok(()));
    ProcessMemoryLock::new(recording_mlockall)
}

#[test]
fn locks_both_current_and_future_mappings() {
    lock().engage().expect("fake mlockall succeeds");

    assert_eq!(
        steps(),
        vec![Step::Mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE)]
    );
    assert_eq!(MEMORY_LOCK_FLAGS, libc::MCL_CURRENT | libc::MCL_FUTURE);
}

#[test]
fn pipeline_threads_start_only_after_the_lock_is_held() {
    let lock = lock();

    let started = lock
        .start_pipeline_threads(|| {
            STEPS.with_borrow_mut(|steps| steps.push(Step::PipelineThreadsStarted));
            "pipeline"
        })
        .expect("lock granted");

    assert_eq!(started, "pipeline");
    assert_eq!(
        steps(),
        vec![
            Step::Mlockall(MEMORY_LOCK_FLAGS),
            Step::PipelineThreadsStarted
        ]
    );
}

#[test]
fn denied_lock_keeps_pipeline_threads_from_spawning() {
    let lock = lock();
    deny(libc::ENOMEM);

    let outcome = lock.start_pipeline_threads(|| {
        STEPS.with_borrow_mut(|steps| steps.push(Step::PipelineThreadsStarted));
    });

    assert_eq!(
        outcome,
        Err(MemoryLockDenied {
            errno: libc::ENOMEM
        })
    );
    assert_eq!(steps(), vec![Step::Mlockall(MEMORY_LOCK_FLAGS)]);
}

#[test]
fn denial_reports_os_error_and_how_to_raise_the_limit() {
    let lock = lock();
    deny(libc::ENOMEM);

    let message = lock.engage().expect_err("lock denied").to_string();

    assert!(
        message.contains("mlockall(MCL_CURRENT|MCL_FUTURE) failed"),
        "{message}"
    );
    assert!(
        message.contains(&std::io::Error::from_raw_os_error(libc::ENOMEM).to_string()),
        "{message}"
    );
    assert!(
        message.contains(&format!("errno {}", libc::ENOMEM)),
        "{message}"
    );
    assert!(message.contains("RLIMIT_MEMLOCK"), "{message}");
    assert!(message.contains("LimitMEMLOCK=infinity"), "{message}");
}

#[test]
fn repeated_startup_issues_the_syscall_once() {
    let lock = lock();

    lock.engage().expect("first engage");
    lock.engage().expect("second engage");
    lock.start_pipeline_threads(|| {}).expect("third engage");

    assert_eq!(steps(), vec![Step::Mlockall(MEMORY_LOCK_FLAGS)]);
}

#[test]
fn a_denied_lock_stays_denied_without_retrying() {
    let lock = lock();
    deny(libc::EPERM);

    let first = lock.engage().expect_err("first denial");
    VERDICT.with_borrow_mut(|v| *v = Ok(()));
    let second = lock.engage().expect_err("verdict is latched");

    assert_eq!(first, MemoryLockDenied { errno: libc::EPERM });
    assert_eq!(second, first);
    assert_eq!(steps(), vec![Step::Mlockall(MEMORY_LOCK_FLAGS)]);
}
