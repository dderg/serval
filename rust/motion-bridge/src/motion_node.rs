//! `MotionNode`: a clock-synced motion output the dispatch closure drives as a
//! peer. The trait surface is intentionally minimal — the per-MCU clock-base
//! arithmetic stays in the dispatch closure (`bridge.rs`); a node only answers
//! "what is now, in your clock domain?" (`now_clock`), "how fast does that
//! clock tick?" (`clock_freq`), and "load these curves and push this segment"
//! (`load_and_push`).
//!
//! `StepperMcuNode` (serial `KalicoHostIo`) and `EtherCatNode` (same-host
//! `UnixNativeConn`) are the two implementations. The EtherCAT node shares the
//! host's `CLOCK_MONOTONIC`, so its clock domain is nanoseconds with no
//! drift — `clock_freq() == 1e9`, `now_clock() == monotonic_ns()`.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant};

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::host_io::KalicoHostIo;
use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::passthrough_queue::PassthroughRouter;
use kalico_host_rt::producer;
use kalico_host_rt::unix_native_conn::UnixNativeConn;

use crate::dispatch::McuPushPlan;
use crate::planner::DispatchError;
use crate::slot_pool::SharedSlotPool;
use crate::types::mcu_handle_from_raw;

/// Maximum time `load_and_push_via` blocks waiting for a free curve slot.
/// Mirrors the private `DEFAULT_SLOT_ACQUIRE_TIMEOUT` in `bridge.rs`.
const DEFAULT_SLOT_ACQUIRE_TIMEOUT: Duration = Duration::from_secs(60);

/// A clock-synced motion output peer.
pub trait MotionNode: Send + Sync {
    /// Current time in this node's clock domain (ticks).
    fn now_clock(&self) -> Result<u64, DispatchError>;
    /// This node's clock frequency in ticks/second.
    fn clock_freq(&self) -> f64;
    /// Load the plan's curves into the node's pool and push the segment.
    fn load_and_push(&self, plan: McuPushPlan) -> Result<(), DispatchError>;
}

/// Read the host-wide monotonic clock in nanoseconds. Shared by every process
/// on the machine (unlike `std::time::Instant`, whose epoch is per-process),
/// so it is the common time base between this host and the EtherCAT endpoint.
fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, writable `timespec`; `CLOCK_MONOTONIC` is
    // available on every supported platform (Linux, macOS). The call writes
    // only `ts` and returns 0 on success, -1 on error. We ignore the return
    // value — a `clock_gettime` failure is unrecoverable at this call site
    // and would produce a zero timestamp, which the caller's monotonicity
    // assertion in tests would immediately catch.
    #[allow(unsafe_code)]
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

/// EtherCAT RT endpoint as a motion node: same-host Unix socket, shared
/// monotonic clock (`CLOCK_MONOTONIC`, nanosecond resolution).
pub struct EtherCatNode {
    conn: Arc<UnixNativeConn>,
    credit: Arc<CreditCounter>,
    slot_pool: Arc<SharedSlotPool>,
}

impl std::fmt::Debug for EtherCatNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EtherCatNode").finish_non_exhaustive()
    }
}

impl EtherCatNode {
    pub fn new(
        conn: Arc<UnixNativeConn>,
        credit: Arc<CreditCounter>,
        slot_pool: Arc<SharedSlotPool>,
    ) -> Self {
        Self {
            conn,
            credit,
            slot_pool,
        }
    }
}

impl MotionNode for EtherCatNode {
    fn now_clock(&self) -> Result<u64, DispatchError> {
        Ok(monotonic_ns())
    }

    fn clock_freq(&self) -> f64 {
        1.0e9
    }

    fn load_and_push(&self, plan: McuPushPlan) -> Result<(), DispatchError> {
        load_and_push_via(self.conn.as_ref(), &self.credit, &self.slot_pool, plan)
    }
}

/// Shared load+push unit of work over any `NativeCall` peer. Allocates a slot
/// per curve, loads it, registers the slots against the segment id, then
/// pushes the segment. Releases all allocated slots on any failure. Lifted
/// verbatim from the dispatch closure's inner loop (`bridge.rs`), parameterised
/// over the connection — so `StepperMcuNode` and `EtherCatNode` share one
/// implementation.
pub(crate) fn load_and_push_via(
    io: &dyn NativeCall,
    credit: &CreditCounter,
    slot_pool: &SharedSlotPool,
    mut plan: McuPushPlan,
) -> Result<(), DispatchError> {
    let mut allocated_slots: Vec<u16> = Vec::with_capacity(plan.curves_to_load.len());
    let mut seg_err: Option<DispatchError> = None;

    for i in 0..plan.curves_to_load.len() {
        let axis_idx = plan.curves_to_load[i].0;
        let curve_params = plan.curves_to_load[i].1.clone();

        let alloc_result = slot_pool
            .alloc_blocking(DEFAULT_SLOT_ACQUIRE_TIMEOUT)
            .ok_or_else(|| DispatchError::SlotPoolExhausted {
                mcu_id: plan.mcu_id,
                capacity: slot_pool.capacity(),
                in_flight: slot_pool.in_flight_count(),
            });

        let (slot, slot_gen) = match alloc_result {
            Ok(v) => v,
            Err(e) => {
                seg_err = Some(e);
                break;
            }
        };
        allocated_slots.push(slot);

        match producer::load_curve(
            io,
            slot,
            axis_idx as u8,
            &curve_params,
            producer::DEFAULT_LOAD_CURVE_TIMEOUT,
        ) {
            Ok(handle) => {
                plan.set_handle(axis_idx, handle);
            }
            Err(e) => {
                seg_err = Some(DispatchError::LoadCurve {
                    mcu_id: plan.mcu_id,
                    slot,
                    seg_id: plan.params.id,
                    axis: axis_idx,
                    host_gen: slot_gen,
                    detail: e.to_string(),
                });
                break;
            }
        }
    }

    if let Some(err) = seg_err {
        for s in &allocated_slots {
            slot_pool.release(*s);
        }
        return Err(err);
    }

    // Register slots BEFORE push (slot_pool.rs requirement: slots must be
    // registered before the MCU can retire them via credit_freed events).
    for slot in &allocated_slots {
        slot_pool.register_segment(*slot, plan.params.id);
    }

    match producer::push_segment(io, credit, &plan.params) {
        Ok(_info) => Ok(()),
        Err(e) => {
            for s in &allocated_slots {
                slot_pool.release(*s);
            }
            Err(DispatchError::PushSegment {
                mcu_id: plan.mcu_id,
                detail: e.to_string(),
            })
        }
    }
}

/// The serial stepper MCU as a motion node. Holds the same per-MCU state the
/// dispatch closure used to capture inline; `clock_freq`/`now_clock` reproduce
/// the closure's logic verbatim. `now_clock` returns the MCU's current widened
/// clock (the schedule-base rebasing stays in the dispatch closure).
pub struct StepperMcuNode {
    pub mcu_id: u32,
    io: Weak<KalicoHostIo>,
    credit: Arc<CreditCounter>,
    slot_pool: Arc<SharedSlotPool>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    router: Arc<Mutex<PassthroughRouter>>,
    fallback_counter: Arc<AtomicU64>,
    warned_mcus: Arc<Mutex<HashSet<u32>>>,
}

impl std::fmt::Debug for StepperMcuNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StepperMcuNode")
            .field("mcu_id", &self.mcu_id)
            .finish_non_exhaustive()
    }
}

impl StepperMcuNode {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mcu_id: u32,
        io: Weak<KalicoHostIo>,
        credit: Arc<CreditCounter>,
        slot_pool: Arc<SharedSlotPool>,
        clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
        router: Arc<Mutex<PassthroughRouter>>,
        fallback_counter: Arc<AtomicU64>,
        warned_mcus: Arc<Mutex<HashSet<u32>>>,
    ) -> Self {
        Self {
            mcu_id,
            io,
            credit,
            slot_pool,
            clock_freqs,
            router,
            fallback_counter,
            warned_mcus,
        }
    }
}

impl MotionNode for StepperMcuNode {
    /// Returns the MCU's clock frequency in ticks/second.
    ///
    /// Verbatim lift of the `freq` lookup from `bridge.rs` (~line 2233).
    /// Falls back to 1 MHz with a one-shot per-MCU `log::warn!` if
    /// `set_clock_est` has not installed a valid frequency yet.
    fn clock_freq(&self) -> f64 {
        self.clock_freqs
            .lock()
            .unwrap()
            .get(&self.mcu_id)
            .copied()
            .filter(|f| *f > 0.0)
            .unwrap_or_else(|| {
                self.fallback_counter.fetch_add(1, Ordering::Relaxed);
                let first_for_mcu = {
                    let mut warned = self
                        .warned_mcus
                        .lock()
                        .unwrap_or_else(|p| p.into_inner());
                    warned.insert(self.mcu_id)
                };
                if first_for_mcu {
                    log::warn!(
                        "motion-bridge: MCU {} clock frequency not installed; \
                         using 1 MHz fallback for relative segment timing. \
                         SET_CLOCK_EST not yet wired by klippy?",
                        self.mcu_id
                    );
                }
                1_000_000.0
            })
    }

    /// Returns the MCU's current widened clock (ticks).
    ///
    /// Verbatim lift of the `now_clock` acquisition loop from `bridge.rs`
    /// (~line 2278–2308). Polls `PassthroughRouter::compute_ack_clock` until
    /// it returns a non-zero value, sleeping 10 ms between retries. Returns
    /// `DispatchError::ClockSyncTimeout` if clock-sync does not establish
    /// within 5 seconds.
    ///
    /// The schedule-base rebasing arithmetic (`lead_cycles`, `schedule_state`)
    /// stays in the dispatch closure and is NOT part of this method.
    fn now_clock(&self) -> Result<u64, DispatchError> {
        let mcu_h = mcu_handle_from_raw(self.mcu_id);
        let wait_start = Instant::now();
        let now_clock = loop {
            let r = self
                .router
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let n = r
                .compute_ack_clock(mcu_h)
                .map_err(|e| DispatchError::ComputeAckClock(e.to_string()))?;
            drop(r);
            if n > 0 {
                break n;
            }
            if wait_start.elapsed() > Duration::from_secs(5) {
                return Err(DispatchError::ClockSyncTimeout {
                    mcu_id: self.mcu_id,
                    mcu_handle: mcu_h,
                });
            }
            std::thread::sleep(Duration::from_millis(10));
        };
        Ok(now_clock)
    }

    fn load_and_push(&self, plan: McuPushPlan) -> Result<(), DispatchError> {
        let io = self
            .io
            .upgrade()
            .ok_or(DispatchError::ConnectionDropped(self.mcu_id))?;
        load_and_push_via(io.as_ref(), &self.credit, &self.slot_pool, plan)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ethercat_node_clock_domain_is_monotonic_ns() {
        let (a, _b) = std::os::unix::net::UnixStream::pair().unwrap();
        let node = EtherCatNode::new(
            Arc::new(UnixNativeConn::from_stream(a)),
            Arc::new(CreditCounter::new(8)),
            Arc::new(SharedSlotPool::new(16)),
        );
        assert_eq!(node.clock_freq(), 1.0e9);
        let t0 = node.now_clock().unwrap();
        let t1 = node.now_clock().unwrap();
        assert!(t1 >= t0, "monotonic clock must not go backwards");
    }
}
