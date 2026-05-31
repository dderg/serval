//! Unit tests for [`super::resolve_dispatch_target`].
//!
//! These tests verify the helper's contract: an `mcu_id` present in `nodes`
//! but absent from `dispatch_ios` (an EtherCAT node) must resolve to
//! `Ok(Some((node, None)))` — dispatched with no serial io — not skipped.
//! This guards against re-introducing the pre-fix `dispatch_ios`-keyed skip.
//!
//! Scope: these tests cover the helper itself. That the dispatch closure
//! actually calls this helper is verified at the closure's single call site.

use std::collections::HashMap;
use std::sync::{Arc, Weak};

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::unix_native_conn::UnixNativeConn;

use crate::motion_node::{EtherCatNode, MotionNode};
use crate::planner::DispatchError;
use crate::slot_pool::SharedSlotPool;

use super::resolve_dispatch_target;

// ── helpers ──────────────────────────────────────────────────────────────────

/// Stand up an `EtherCatNode` over a paired Unix socket.  The peer end is
/// intentionally dropped immediately: the test only inspects the return value
/// of `resolve_dispatch_target`, never calls `load_and_push`.
fn ethercat_node() -> Arc<dyn MotionNode> {
    let (client, _server) = std::os::unix::net::UnixStream::pair()
        .expect("UnixStream::pair");
    Arc::new(EtherCatNode::new(
        Arc::new(UnixNativeConn::from_stream(client)),
        Arc::new(CreditCounter::new(8)),
        Arc::new(SharedSlotPool::new(16)),
    ))
}

// ── EtherCAT dispatch guard ───────────────────────────────────────────────────

/// PRIMARY REGRESSION GUARD (Task 5).
///
/// An EtherCAT mcu_id appears in `nodes` but NOT in `dispatch_ios` — the
/// expected steady-state for every EtherCAT-routed MCU.
///
/// `resolve_dispatch_target` must return `Ok(Some((node, None)))`:
/// - `Some` → the plan is dispatched, not skipped.
/// - inner `None` → no serial `KalicoHostIo`; EtherCAT path.
///
/// If this test fails, the regression from before Task 5 has been
/// reintroduced: EtherCAT plans are being silently dropped.
#[test]
fn ethercat_mcu_in_nodes_only_is_dispatched_with_no_serial_io() {
    let mcu_id: u32 = 42;

    let mut nodes: HashMap<u32, Arc<dyn MotionNode>> = HashMap::new();
    nodes.insert(mcu_id, ethercat_node());

    // dispatch_ios is empty — EtherCAT MCUs have no serial KalicoHostIo.
    let dispatch_ios = HashMap::new();

    let result = resolve_dispatch_target(mcu_id, &nodes, &dispatch_ios);

    let pair = result
        .expect("must not return Err for an EtherCAT node")
        .expect("must return Some — EtherCAT plan must NOT be skipped");

    let (_node, serial_io) = pair;
    assert!(
        serial_io.is_none(),
        "EtherCAT node must have serial_io == None (no KalicoHostIo on the socket path)"
    );
}

// ── Skip path ────────────────────────────────────────────────────────────────

/// An mcu_id that is present in neither `nodes` nor `dispatch_ios` is a
/// configuration bug (a plan whose origin MCU was never registered).  The
/// function returns `Ok(None)` so the caller can `continue` past this plan
/// without panicking the planner thread.
#[test]
fn unknown_mcu_id_yields_none_skip() {
    let nodes: HashMap<u32, Arc<dyn MotionNode>> = HashMap::new();
    let dispatch_ios = HashMap::new();

    let result = resolve_dispatch_target(99, &nodes, &dispatch_ios);

    assert!(
        matches!(result, Ok(None)),
        "unknown mcu_id must produce Ok(None) (caller skips)"
    );
}

// ── ConnectionDropped ────────────────────────────────────────────────────────

/// A serial stepper MCU whose `Weak<KalicoHostIo>` has been dropped (the MCU
/// disconnected while the planner was mid-batch) must return
/// `Err(DispatchError::ConnectionDropped)`.  Uses `Weak::new()` which always
/// fails to upgrade, simulating a fully-dropped Arc.
#[test]
fn dead_serial_weak_returns_connection_dropped() {
    let mcu_id: u32 = 7;

    let mut nodes: HashMap<u32, Arc<dyn MotionNode>> = HashMap::new();
    nodes.insert(mcu_id, ethercat_node()); // node type doesn't matter here

    // Weak::new() always fails to upgrade — simulates a dropped Arc<KalicoHostIo>.
    let dead_weak: Weak<kalico_host_rt::host_io::KalicoHostIo> = Weak::new();
    let credit = Arc::new(CreditCounter::new(4));
    let pool = Arc::new(SharedSlotPool::new(8));

    let mut dispatch_ios = HashMap::new();
    dispatch_ios.insert(mcu_id, (dead_weak, credit, pool));

    let result = resolve_dispatch_target(mcu_id, &nodes, &dispatch_ios);

    assert!(
        matches!(result, Err(DispatchError::ConnectionDropped(id)) if id == mcu_id),
        "dead Weak must produce ConnectionDropped({mcu_id})"
    );
}

// ── Orthogonality: extra nodes do not pollute EtherCAT result ────────────────

/// Two MCUs are registered in `nodes`: one EtherCAT (mcu_id=10) and one serial
/// stepper with a dead Weak (mcu_id=11).  Querying the EtherCAT node must
/// still yield `Ok(Some((_, None)))` regardless of the serial node's presence.
#[test]
fn ethercat_mcu_unaffected_by_sibling_serial_mcu_in_dispatch_ios() {
    let ec_id: u32 = 10;
    let serial_id: u32 = 11;

    let mut nodes: HashMap<u32, Arc<dyn MotionNode>> = HashMap::new();
    nodes.insert(ec_id, ethercat_node());
    nodes.insert(serial_id, ethercat_node()); // stand-in node for the serial MCU

    let dead_weak: Weak<kalico_host_rt::host_io::KalicoHostIo> = Weak::new();
    let mut dispatch_ios = HashMap::new();
    dispatch_ios.insert(
        serial_id,
        (dead_weak, Arc::new(CreditCounter::new(4)), Arc::new(SharedSlotPool::new(8))),
    );

    // EtherCAT lookup must ignore the serial entry entirely.
    let result = resolve_dispatch_target(ec_id, &nodes, &dispatch_ios);
    let (_node, serial_io) = result
        .expect("no error for EtherCAT")
        .expect("EtherCAT must not be skipped even with a sibling serial entry");
    assert!(
        serial_io.is_none(),
        "EtherCAT node must have serial_io == None even when a serial sibling exists"
    );
}
