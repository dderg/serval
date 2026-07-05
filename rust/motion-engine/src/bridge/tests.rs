use std::os::unix::io::FromRawFd;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Weak};

use host_rt::host_io::{McuHostIo, McuHostIoConfig};
use host_rt::mcu_serial_conn::McuSerialConn;

use crate::config::PlannerConfig;
use crate::worker::{DispatchError, StreamWorkerHandle};
use trajectory::ShapedSegment;

use super::{McuConnection, PyMotionEngine};

fn open_pty() -> (libc::c_int, String) {
    let mut master: libc::c_int = 0;
    let mut slave: libc::c_int = 0;
    #[allow(unsafe_code)]
    let path = unsafe {
        let mut name_buf = [0i8; 256];
        let r = libc::openpty(
            &mut master,
            &mut slave,
            name_buf.as_mut_ptr(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert_eq!(r, 0, "openpty failed: {}", std::io::Error::last_os_error());
        let cstr = std::ffi::CStr::from_ptr(name_buf.as_ptr());
        let p = cstr.to_str().expect("pts path is utf-8").to_owned();
        libc::close(slave);
        p
    };
    (master, path)
}

fn host_io_on_pty(slave_path: &str) -> (Arc<McuHostIo>, Weak<McuHostIo>) {
    #[allow(unsafe_code)]
    let port: Box<dyn serialport::SerialPort> = unsafe {
        let cpath = std::ffi::CString::new(slave_path).unwrap();
        let fd = libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_NOCTTY);
        assert!(
            fd >= 0,
            "open({slave_path}) failed: {}",
            std::io::Error::last_os_error()
        );
        Box::new(serialport::TTYPort::from_raw_fd(fd))
    };
    let io = McuHostIo::from_port_skip_identify(port, McuHostIoConfig::default());
    let arc = Arc::new(io);
    let weak = Arc::downgrade(&arc);
    (arc, weak)
}

fn serial_mcu_conn(label: &str, host_io: Arc<McuHostIo>) -> McuConnection {
    McuConnection {
        label: label.to_owned(),
        host_io: Some(host_io),
        runtime_rx_priority: None,
        runtime_rx_bulk: None,
        runtime_caps: None,
        identify_caps: 0,
        mcu_transport_supported: false,
        ethercat_socket: None,
        ethercat_slot_axes: Vec::new(),
        endpoint_process: None,
        endpoint_conn: None,
    }
}

fn insert_mcu(engine: &PyMotionEngine, handle: u32, conn: McuConnection) {
    engine
        .mcus
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(handle, conn);
}

fn mcus_is_empty(engine: &PyMotionEngine) -> bool {
    engine
        .mcus
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .is_empty()
}

fn seed_pump_thread(engine: &PyMotionEngine) -> Arc<std::sync::atomic::AtomicBool> {
    let (tx, rx) = crossbeam_channel::unbounded::<crate::pump::PumpMsg>();
    let exited = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let exited_thread = Arc::clone(&exited);
    let handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            for msg in rx {
                if matches!(msg, crate::pump::PumpMsg::Shutdown) {
                    break;
                }
            }
            exited_thread.store(true, std::sync::atomic::Ordering::SeqCst);
        })
        .expect("spawn test pump thread");
    *engine.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(tx);
    *engine.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    exited
}

#[test]
fn shutdown_releases_pty_and_joins_threads() {
    let engine = PyMotionEngine::new();
    let (master_fd, slave_path) = open_pty();

    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&engine, 1, serial_mcu_conn("mcu", io_arc));
    assert!(
        io_weak.upgrade().is_some(),
        "host_io must be alive pre-shutdown"
    );

    let pump_exited = seed_pump_thread(&engine);

    engine.shutdown();

    assert!(
        io_weak.upgrade().is_none(),
        "shutdown() must drop the last Arc<McuHostIo> — its Drop closes the \
         pts fd (TIOCEXCL release); a surviving Arc means a leaked fd → EBUSY"
    );
    assert!(
        pump_exited.load(std::sync::atomic::Ordering::SeqCst),
        "pump thread must have received Shutdown and exited (joined, not leaked)"
    );
    assert!(
        engine
            .pump_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "pump_thread handle must be taken (None) after join"
    );
    assert!(
        mcus_is_empty(&engine),
        "mcus map must be empty after shutdown"
    );
    assert!(
        engine.shut_down.load(std::sync::atomic::Ordering::SeqCst),
        "shut_down flag must be latched"
    );

    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}

#[test]
fn shutdown_releases_ethercat_socket_and_child() {
    use std::io::Read;
    use std::os::unix::net::UnixStream;

    let engine = PyMotionEngine::new();

    let (conn_stream, peer_stream) = UnixStream::pair().expect("socketpair must be available");
    let native = McuSerialConn::from_stream(conn_stream).expect("from_stream");

    let child = std::process::Command::new("sh")
        .args(["-c", "sleep 30"])
        .spawn()
        .expect("sh must be available");
    let child_pid = child.id();

    let conn = McuConnection {
        label: "ec".to_owned(),
        host_io: None,
        runtime_rx_priority: None,
        runtime_rx_bulk: None,
        runtime_caps: None,
        identify_caps: 0,
        mcu_transport_supported: true,
        ethercat_socket: Some("/tmp/kalico_test_ec.sock".to_owned()),
        ethercat_slot_axes: Vec::new(),
        endpoint_process: Some(child),
        endpoint_conn: Some(Arc::new(native)),
    };
    insert_mcu(&engine, 7, conn);

    engine.shutdown();

    let (done_tx, done_rx) = std::sync::mpsc::channel::<std::io::Result<usize>>();
    std::thread::spawn(move || {
        let mut peer = peer_stream;
        let mut buf = [0u8; 16];
        let _ = done_tx.send(peer.read(&mut buf));
    });
    let n = done_rx
        .recv_timeout(std::time::Duration::from_secs(5))
        .expect("peer read must complete (socket must be closed by shutdown)")
        .expect("peer read after conn close");
    assert_eq!(
        n, 0,
        "peer must see EOF (0 bytes) after endpoint_conn dropped"
    );

    #[allow(unsafe_code)]
    let alive = unsafe { libc::kill(child_pid as libc::pid_t, 0) };
    assert_eq!(
        alive, -1,
        "endpoint child (pid {child_pid}) must be reaped (kill(pid,0) → ESRCH)"
    );
    assert_eq!(
        std::io::Error::last_os_error().raw_os_error(),
        Some(libc::ESRCH),
        "child must be gone (ESRCH), not merely unsignalable"
    );

    assert!(
        mcus_is_empty(&engine),
        "mcus map must be empty after shutdown"
    );
}

#[test]
fn double_shutdown_is_safe() {
    let engine = PyMotionEngine::new();
    let (master_fd, slave_path) = open_pty();
    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&engine, 1, serial_mcu_conn("mcu", io_arc));

    engine.shutdown();
    assert!(
        io_weak.upgrade().is_none(),
        "first shutdown releases host_io"
    );
    assert!(engine.shut_down.load(std::sync::atomic::Ordering::SeqCst));

    engine.shutdown();
    assert!(mcus_is_empty(&engine));

    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}

fn counting_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<AtomicUsize>,
) {
    let counter = Arc::new(AtomicUsize::new(0));
    let c = Arc::clone(&counter);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            c.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });
    (cb, counter)
}

fn noop_nudge_dispatch()
-> Arc<dyn Fn(u32, &crate::nudge::NudgePiece) -> Result<(), DispatchError> + Send + Sync> {
    Arc::new(|_mcu_id: u32, _np: &crate::nudge::NudgePiece| Ok(()))
}

fn relaxed_planner_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.fit_tolerance_mm = 0.05;
    c
}

fn test_limits() -> geometry::VelocityLimits {
    geometry::VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap()
}

fn stream_config_from(cfg: &PlannerConfig) -> (motion_pipeline::StreamConfig, Vec<f64>) {
    let sc = motion_pipeline::StreamConfig {
        chain: geometry::ChainFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: cfg.fit_tolerance_mm,
        fit_tol_accel_mm_s2: cfg.fit_tolerance_accel_mm_s2,
        max_buffer_moves: 64,
        limits: test_limits(),
    };
    (sc, vec![0.0; cfg.axis_registry.n_axes().max(3)])
}

#[test]
fn shutdown_takes_and_joins_planner() {
    let engine = PyMotionEngine::new();
    let (dispatch, _counter) = counting_dispatch();
    let (sc, home) = stream_config_from(&PlannerConfig::default());
    *engine.planner.lock().unwrap_or_else(|p| p.into_inner()) = Some(StreamWorkerHandle::spawn(
        sc,
        trajectory::AxisChainSet::default(),
        home,
        dispatch,
        noop_nudge_dispatch(),
        Arc::default(),
    ));

    assert!(
        engine
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some(),
        "planner must be seeded pre-shutdown"
    );

    engine.shutdown();

    assert!(
        engine
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "shutdown() must take() the planner out of the Mutex and join it — a \
         surviving Some means the kalico-planner thread leaked across restart"
    );
}

#[test]
fn shutdown_joins_planner_before_dropping_pump_receiver() {
    let engine = PyMotionEngine::new();

    let (pump_tx, pump_rx) = crossbeam_channel::unbounded::<crate::pump::PumpMsg>();
    let pump_tx_for_engine = pump_tx.clone();

    let saw_pump_gone = Arc::new(AtomicBool::new(false));
    let saw_pump_gone_cb = Arc::clone(&saw_pump_gone);
    let dispatch_count = Arc::new(AtomicUsize::new(0));
    let dispatch_count_cb = Arc::clone(&dispatch_count);
    let dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            dispatch_count_cb.fetch_add(1, Ordering::SeqCst);
            let hb = crate::pump::PumpMsg::Heartbeat(crate::pump::HeartbeatMsg {
                mcu_id: 0,
                retired_counts: Vec::new(),
            });
            if pump_tx.send(hb).is_err() {
                saw_pump_gone_cb.store(true, Ordering::SeqCst);
            }
            Ok(())
        });

    let (sc, home) = stream_config_from(&relaxed_planner_config());
    let planner = StreamWorkerHandle::spawn(
        sc,
        trajectory::AxisChainSet::default(),
        home,
        dispatch,
        noop_nudge_dispatch(),
        Arc::default(),
    );
    planner
        .submit_move(
            crate::classify::build_move([0.0; 3], 50.0, 0.0, 0.0, 0, 0.0, test_limits(), 200.0, 0)
                .unwrap(),
        )
        .unwrap();
    let engine = Arc::new(engine);
    *engine.planner.lock().unwrap_or_else(|p| p.into_inner()) = Some(planner);

    let pump_handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            for msg in &pump_rx {
                if matches!(msg, crate::pump::PumpMsg::Shutdown) {
                    break;
                }
            }
            drop(pump_rx);
        })
        .expect("spawn test pump thread");
    *engine.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_handle);
    *engine.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx_for_engine);

    let stop = Arc::new(AtomicBool::new(false));
    let stop_sub = Arc::clone(&stop);
    let engine_sub = Arc::clone(&engine);
    let submitter = std::thread::Builder::new()
        .name("test-submitter".into())
        .spawn(move || {
            let mut start = [50.0, 0.0, 0.0];
            while !stop_sub.load(Ordering::SeqCst) {
                {
                    let guard = engine_sub.planner.lock().unwrap_or_else(|p| p.into_inner());
                    let Some(p) = guard.as_ref() else {
                        break; // shutdown() took the planner; stop submitting.
                    };
                    let m = crate::classify::build_move(
                        start,
                        50.0,
                        0.0,
                        0.0,
                        0,
                        0.0,
                        test_limits(),
                        200.0,
                        0,
                    )
                    .unwrap();
                    if p.submit_move(m).is_err() {
                        break;
                    }
                    start[0] += 50.0;
                }
                std::thread::sleep(std::time::Duration::from_millis(3));
            }
        })
        .expect("spawn test submitter");

    std::thread::sleep(std::time::Duration::from_millis(300));
    assert!(
        dispatch_count.load(Ordering::SeqCst) > 0,
        "planner must have fired at least one dispatch (else the test does not \
         exercise the ordering window)"
    );

    engine.shutdown();

    stop.store(true, Ordering::SeqCst);
    submitter.join().expect("submitter join");

    assert!(
        !saw_pump_gone.load(Ordering::SeqCst),
        "planner observed a dropped pump during teardown — this is the abort() \
         path that leaks the pts fd; shutdown() must join the planner BEFORE \
         dropping the pump's Receiver"
    );
    assert!(
        engine
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "planner must be taken+joined by shutdown()"
    );
}

#[test]
fn shutdown_does_not_abort_on_detached_ethercat_weak() {
    use runtime::piece_ring::PieceEntry;
    use std::collections::HashMap;
    use std::time::Duration;

    use crate::pump::{EnqueueMsg, McuTransport, PumpMsg, WireSink, run_pump};
    use crate::types::AxisKey;

    const EC_MCU_ID: u32 = 42;

    let detached_weak: std::sync::Weak<host_rt::mcu_serial_conn::McuSerialConn> =
        std::sync::Weak::new();

    let fatal_fired = Arc::new(AtomicBool::new(false));
    let fatal_flag = Arc::clone(&fatal_fired);

    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(EC_MCU_ID, McuTransport::EtherCat(detached_weak));
            m
        },
        timeout: Duration::from_millis(50),
        freq_of: Arc::new(|_| None),
    };

    let mcu_clock_of = |_mcu_id: u32| -> Option<(u64, f64)> { Some((1, 1.0)) };

    let (pump_tx, control_rx) = crossbeam_channel::unbounded::<PumpMsg>();
    let (data_tx, data_rx) = crossbeam_channel::unbounded::<EnqueueMsg>();

    let pump_handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            run_pump(
                control_rx,
                data_rx,
                sink,
                |_key| 256_u32,
                mcu_clock_of,
                move |_key: AxisKey| {
                    fatal_flag.store(true, Ordering::SeqCst);
                },
                |_key: AxisKey, _n: u32| {},
                |_key: AxisKey, _n: u32| {},
                |_msg: String| {},
                Arc::new(AtomicU64::new(0)),
            );
        })
        .expect("spawn test pump thread");

    let pieces_to_enqueue = vec![(
        PieceEntry {
            start_time: 1_000_000,
            duration: 0.001,
            ..PieceEntry::zeroed()
        },
        1.0_f64,
    )];
    data_tx
        .send(EnqueueMsg {
            key: AxisKey {
                mcu_id: EC_MCU_ID,
                axis: 0,
            },
            pieces: pieces_to_enqueue,
            fresh_stream: false,
            lead_secs: 0.0,
            source_line: u32::MAX,
        })
        .expect("enqueue must succeed before shutdown");

    std::thread::sleep(Duration::from_millis(30));

    let engine = Arc::new(PyMotionEngine::new());
    *engine.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx);
    *engine.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_handle);

    let (dispatch, _counter) = counting_dispatch();
    let (sc, home) = stream_config_from(&relaxed_planner_config());
    *engine.planner.lock().unwrap_or_else(|p| p.into_inner()) = Some(StreamWorkerHandle::spawn(
        sc,
        trajectory::AxisChainSet::default(),
        home,
        dispatch,
        noop_nudge_dispatch(),
        Arc::default(),
    ));

    engine.shutdown();

    assert!(
        !fatal_fired.load(Ordering::SeqCst),
        "on_fatal_transport must never fire during shutdown(): pump must exit \
         via PumpMsg::Shutdown before it can touch the dead EtherCAT transport. \
         A true flag means the old abort() path would have killed the process and \
         leaked the pts fd."
    );
    assert!(
        engine
            .pump_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "pump thread handle must be taken (joined) by shutdown()"
    );
    assert!(
        engine
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "planner must be taken+joined by shutdown()"
    );
}

#[test]
fn register_ethercat_mcu_seeds_nominal_clock_freq() {
    use std::os::unix::net::UnixStream;

    use host_rt::mcu_serial_conn::McuSerialConn;

    let engine = PyMotionEngine::new();
    const RAW: u32 = 77;

    let (conn_stream, _peer) = UnixStream::pair().expect("socketpair");
    let conn = McuSerialConn::from_stream(conn_stream).expect("from_stream");
    let child = std::process::Command::new("true")
        .spawn()
        .expect("spawn true");

    engine.register_ethercat_mcu(RAW, "servo", "/tmp/test.sock", child, conn, vec![0]);

    assert!(
        engine
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains_key(&RAW),
        "mcus must contain the raw handle after register_ethercat_mcu"
    );
    assert_eq!(
        engine
            .nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&RAW)
            .copied(),
        Some(1_000_000_000_u32),
        "nominal_clock_freqs must contain 1 GHz for the ethercat raw handle; \
         removing the insert from register_ethercat_mcu must cause this to fail"
    );
}

#[test]
fn partial_state_teardown_at_exit() {
    let engine = PyMotionEngine::new();
    let (master_fd, slave_path) = open_pty();
    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&engine, 1, serial_mcu_conn("mcu0", io_arc));

    engine.shutdown();

    assert!(
        io_weak.upgrade().is_none(),
        "the one attached MCU's host_io fd must be released even when the \
         engine was only partially attached"
    );
    assert!(mcus_is_empty(&engine));

    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}

#[test]
fn report_ethercat_endpoint_death_latches_203_and_first_cause_wins() {
    let latch: Arc<std::sync::Mutex<std::collections::HashMap<u32, String>>> =
        Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
    let first = super::report_ethercat_endpoint_death(&latch, 5, "conn EOF");
    // A later writer (e.g. the supervisor after the pump already latched) must
    // not overwrite the first surfaced cause.
    let second = super::report_ethercat_endpoint_death(&latch, 5, "later transport fatal");
    assert!(
        first,
        "the first call latches the cause and arms the backstop"
    );
    assert!(!second, "a later writer does not re-latch (returns false)");
    let msg = latch
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .remove(&5)
        .expect("a cause must be latched");
    assert!(
        msg.contains("(fault -203)"),
        "must carry the -203 code: {msg}"
    );
    assert!(msg.contains("conn EOF"), "first cause must win: {msg}");
    assert!(
        !msg.contains("later transport fatal"),
        "a later writer must not overwrite the first cause: {msg}"
    );
}
