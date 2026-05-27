//! Test harness for MCU motion contract tests.
//!
//! `McuTestBench` wraps the ISR state, engine, curve pool, step queues,
//! and shared state into a single struct with a clean API. Tests interact
//! through high-level methods (`push_segment`, `tick_for_ms`, `cancel`)
//! rather than raw pointer juggling.

#![allow(unsafe_code, clippy::unwrap_used)]

use core::ptr::addr_of_mut;
use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::c_segment_queue;
use runtime::clock::WidenState;
use runtime::config::EMode;
use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::{IsrState, SharedState};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TRACE_RING_N, TraceSample};

type EngineImpl = Engine<NoopPa, NoopIs>;

const H7_CLOCK_HZ: u32 = 520_000_000;
const H7_SAMPLE_RATE_HZ: u32 = 40_000;

const F446_CLOCK_HZ: u32 = 180_000_000;
const F446_SAMPLE_RATE_HZ: u32 = 20_000;

pub struct McuTestBench {
    pub isr: IsrState,
    pub shared: SharedState,
    pub pool: CurvePool,
    step_queues: Box<[StepQueue; 4]>,
    queue_producer: c_segment_queue::Producer<Segment>,
    clock_hz: u32,
    sample_rate_hz: u32,
    raw_cyccnt: u32,
    next_slot: usize,
}

impl McuTestBench {
    pub fn new_h7() -> Self {
        Self::new(H7_CLOCK_HZ, H7_SAMPLE_RATE_HZ)
    }

    #[allow(dead_code)]
    pub fn new_f446() -> Self {
        Self::new(F446_CLOCK_HZ, F446_SAMPLE_RATE_HZ)
    }

    fn new(clock_hz: u32, sample_rate_hz: u32) -> Self {
        let mut engine = EngineImpl::new(clock_hz, sample_rate_hz);

        let binding = StepperBindingRust {
            stepper_oid: 0,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        };
        engine.configure_axis(0, StepMode::Pulse, 0.0125, &[binding]);
        engine.configure_kinematics(1.0);

        let mut step_queues = Box::new([
            StepQueue::new(),
            StepQueue::new(),
            StepQueue::new(),
            StepQueue::new(),
        ]);
        let queue_ptrs = [
            addr_of_mut!(step_queues[0]),
            addr_of_mut!(step_queues[1]),
            addr_of_mut!(step_queues[2]),
            addr_of_mut!(step_queues[3]),
        ];
        engine.test_install_step_queues(queue_ptrs);

        c_segment_queue::reset();
        let queue_consumer = c_segment_queue::Consumer::<Segment>::new();
        let queue_producer = c_segment_queue::Producer::<Segment>::new();

        let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
            Box::leak(Box::new(Queue::new()));
        let (trace_producer, _trace_consumer) = trace_queue.split();

        let isr = IsrState {
            queue_consumer,
            trace_producer,
            engine,
            widen_state: WidenState::default(),
            pending_segment: None,
        };

        // trace_queue is leaked intentionally — it must live for 'static
        // because heapless::spsc::split() requires it.
        let _ = trace_queue;

        Self {
            isr,
            shared: SharedState::new(),
            pool: CurvePool::new(),
            step_queues,
            queue_producer,
            clock_hz,
            sample_rate_hz,
            raw_cyccnt: 0,
            next_slot: 0,
        }
    }

    /// Cycles per ISR tick.
    pub fn cycles_per_sample(&self) -> u32 {
        self.clock_hz / self.sample_rate_hz
    }

    /// Convert milliseconds to MCU clock cycles.
    pub fn ms_to_cycles(&self, ms: f64) -> u64 {
        (ms / 1000.0 * f64::from(self.clock_hz)) as u64
    }

    /// Current widened clock value (what the ISR sees as "now").
    pub fn now_cycles(&self) -> u64 {
        runtime::clock::read_widened_now(&self.shared)
    }

    // ── Curve loading ──────────────────────────────────────────────────

    /// Load a single-piece linear Bézier curve from `start_mm` to `end_mm`
    /// over `duration_sec`. Returns a handle to the loaded curve.
    pub fn load_linear_curve(
        &mut self,
        start_mm: f32,
        end_mm: f32,
        duration_sec: f32,
    ) -> CurveHandle {
        let delta = end_mm - start_mm;
        let piece = WirePiece {
            bp0_bits: start_mm.to_bits(),
            bp1_bits: (start_mm + delta / 3.0).to_bits(),
            bp2_bits: (start_mm + 2.0 * delta / 3.0).to_bits(),
            bp3_bits: end_mm.to_bits(),
            duration_bits: duration_sec.to_bits(),
        };
        let slot = self.next_slot;
        self.next_slot += 1;
        self.pool
            .try_alloc_and_load(slot, &[piece])
            .expect("curve pool alloc failed")
    }

    // ── Segment pushing ────────────────────────────────────────────────

    /// Push a segment with an X curve (and optionally Y). t_start is set
    /// to `now + 1ms` lead time. Returns the segment ID.
    pub fn push_segment_xy(
        &mut self,
        id: u32,
        x_handle: CurveHandle,
        y_handle: CurveHandle,
    ) -> u32 {
        let lead = self.ms_to_cycles(1.0);
        let now = self.now_cycles();
        let t_start = now + lead;
        let duration = self.ms_to_cycles(100.0);
        self.push_segment_raw(id, x_handle, y_handle, t_start, duration)
    }

    /// Push a segment with fully explicit timing.
    pub fn push_segment_raw(
        &mut self,
        id: u32,
        x_handle: CurveHandle,
        y_handle: CurveHandle,
        t_start: u64,
        duration: u64,
    ) -> u32 {
        let mut seg = Segment {
            id,
            x_handle,
            y_handle,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start,
            t_end: t_start + duration,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            flags: 0,
            _pad: [0; 1],
            extrusion_ratio: 0.0,
            consumers_remaining: 0,
        };
        seg.consumers_remaining = Segment::compute_consumers_remaining(
            seg.kinematics,
            seg.x_handle,
            seg.y_handle,
            seg.z_handle,
            seg.e_handle,
        );
        self.queue_producer
            .enqueue(seg)
            .expect("segment queue full");
        id
    }

    // ── Ticking ────────────────────────────────────────────────────────

    /// Advance the ISR clock by one sample tick.
    pub fn tick_one(&mut self) {
        self.raw_cyccnt = self.raw_cyccnt.wrapping_add(self.cycles_per_sample());
        runtime::tick::isr_sample_tick(
            &mut self.isr,
            &self.shared,
            &self.pool,
            self.raw_cyccnt,
        );
        // Drain step queues to prevent overflow (mimics the per-axis consumer).
        for i in 0..4 {
            let q_ptr = addr_of_mut!(self.step_queues[i]);
            unsafe {
                while runtime::step_queue::pop(q_ptr).is_some() {}
            }
        }
    }

    /// Tick for approximately `ms` milliseconds of simulated time.
    pub fn tick_for_ms(&mut self, ms: f64) {
        let n_samples =
            (ms / 1000.0 * f64::from(self.sample_rate_hz)).ceil() as u64;
        for _ in 0..n_samples {
            self.tick_one();
        }
    }

    // ── Queries ────────────────────────────────────────────────────────

    /// Last retired segment ID.
    pub fn retired_through(&self) -> u32 {
        self.shared
            .retired_through_segment_id
            .load(Ordering::Acquire)
    }

    /// X-axis stepper position count (signed microsteps).
    pub fn x_step_count(&self) -> i32 {
        self.isr.engine.stepping_axes[0].steppers[0]
            .position_count
            .load(Ordering::Acquire)
    }

    /// Whether the engine currently has an armed segment.
    pub fn has_current_segment(&self) -> bool {
        self.isr.engine.debug_current_is_some()
    }

    /// Whether a fault has been latched.
    pub fn has_fault(&self) -> bool {
        self.shared.last_error.load(Ordering::Acquire) != 0
    }

    /// The fault code, or 0 if none.
    pub fn fault_code(&self) -> i32 {
        self.shared.last_error.load(Ordering::Acquire)
    }
}
