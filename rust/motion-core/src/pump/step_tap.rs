//! Dev-only record of every classic step frame an mcu was sent, for the bench
//! analyzer to replay into commanded step clocks. Enabled by
//! [`STEP_TAP_DIR_ENV`]; when unset nothing is built and the egress is not
//! wrapped. A writer that cannot keep up is fatal, never lossy.

use super::SendError;
use super::stepcompress_sink::{ClockSource, FrameEgress};
use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError, bounded};
use host_rt::host_io::parser::ArgValue;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use trajectory::ClockedMotorSpan;

pub const STEP_TAP_DIR_ENV: &str = "SERVAL_STEP_TAP_DIR";
pub(crate) const TAP_MAGIC: [u8; 8] = *b"SVTAP1\0\0";
pub(crate) const RECORD_LEN: usize = 32;
const PAYLOAD_OFFSET: usize = 16;
const CHANNEL_RECORDS: usize = 1 << 16;
const IDLE_FLUSH: Duration = Duration::from_millis(100);
const PROJECTION_SLACK_TICKS: f64 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum RecordKind {
    Mcu = 1,
    Lane = 2,
    ResetStepClock = 3,
    SetNextStepDir = 4,
    QueueStep = 5,
    QueueStepHp = 6,
    Barrier = 7,
    SetPosition = 8,
    Projection = 9,
    Halt = 10,
}

/// One fixed-size record: kind | flags | reserved u16 | oid u32 | the mcu's
/// projected clock when it was recorded u64 | 16-byte payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TapRecord(pub(crate) [u8; RECORD_LEN]);

impl TapRecord {
    fn new(kind: RecordKind, oid: u32, clock: u64) -> Self {
        let mut bytes = [0u8; RECORD_LEN];
        bytes[0] = kind as u8;
        bytes[4..8].copy_from_slice(&oid.to_le_bytes());
        bytes[8..16].copy_from_slice(&clock.to_le_bytes());
        Self(bytes)
    }

    fn with_flags(mut self, flags: u8) -> Self {
        self.0[1] = flags;
        self
    }

    fn put(mut self, offset: usize, bytes: &[u8]) -> Self {
        let at = PAYLOAD_OFFSET + offset;
        self.0[at..at + bytes.len()].copy_from_slice(bytes);
        self
    }
}

#[cfg(test)]
impl TapRecord {
    pub(crate) fn kind(&self) -> u8 {
        self.0[0]
    }

    pub(crate) fn oid(&self) -> u32 {
        u32::from_le_bytes(self.0[4..8].try_into().expect("4 bytes"))
    }

    pub(crate) fn payload(&self) -> &[u8] {
        &self.0[PAYLOAD_OFFSET..]
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TapLane {
    pub oid: u32,
    pub axis: u32,
    pub motor_in_axis: u32,
    pub microstep_distance: f64,
    pub invert_dir: bool,
}

pub(crate) struct StepTap {
    mcu_id: u32,
    tx: Sender<TapRecord>,
    clock_of: ClockSource,
    fault: Mutex<Option<String>>,
    projections: Mutex<Vec<(u32, f64, f64)>>,
}

impl StepTap {
    pub(crate) fn open(
        dir: &Path,
        mcu_id: u32,
        clock_freq: f64,
        lanes: &[TapLane],
        clock_of: ClockSource,
    ) -> Result<Arc<Self>, String> {
        let path = dir.join(format!("step_tap_mcu{mcu_id}.bin"));
        let mut file = BufWriter::new(
            File::create(&path)
                .map_err(|e| format!("step tap: cannot create {}: {e}", path.display()))?,
        );
        file.write_all(&TAP_MAGIC)
            .map_err(|e| format!("step tap: cannot write {}: {e}", path.display()))?;
        let (tx, rx) = bounded(CHANNEL_RECORDS);
        std::thread::Builder::new()
            .name(format!("step-tap-mcu{mcu_id}"))
            .spawn(move || write_records(&rx, file))
            .map_err(|e| format!("step tap: cannot spawn writer: {e}"))?;
        let tap = Arc::new(Self::from_sender(tx, mcu_id, clock_of));
        tap.record_header(clock_freq, lanes);
        Ok(tap)
    }

    pub(crate) fn from_sender(tx: Sender<TapRecord>, mcu_id: u32, clock_of: ClockSource) -> Self {
        Self {
            mcu_id,
            tx,
            clock_of,
            fault: Mutex::new(None),
            projections: Mutex::new(Vec::new()),
        }
    }

    pub(crate) fn record_header(&self, clock_freq: f64, lanes: &[TapLane]) {
        let clock = self.clock_now();
        self.push(
            TapRecord::new(RecordKind::Mcu, 0, clock)
                .put(0, &self.mcu_id.to_le_bytes())
                .put(8, &clock_freq.to_le_bytes()),
        );
        for lane in lanes {
            self.push(
                TapRecord::new(RecordKind::Lane, lane.oid, clock)
                    .with_flags(u8::from(lane.invert_dir))
                    .put(0, &lane.axis.to_le_bytes())
                    .put(4, &lane.motor_in_axis.to_le_bytes())
                    .put(8, &lane.microstep_distance.to_le_bytes()),
            );
        }
    }

    pub(crate) fn wrap(self: &Arc<Self>, inner: FrameEgress) -> FrameEgress {
        let tap = Arc::clone(self);
        Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
            inner(frames)?;
            tap.record_frames(frames)
        })
    }

    pub(crate) fn record_frames(
        &self,
        frames: &[(&'static str, Vec<(String, ArgValue)>)],
    ) -> Result<(), SendError> {
        let clock = self.clock_now();
        for (name, args) in frames {
            match encode_frame(name, args, clock) {
                Ok(Some(record)) => self.push(record),
                Ok(None) => {}
                Err(message) => self.latch(message),
            }
        }
        self.check()
    }

    pub(crate) fn note_views(&self, oid: u32, views: &[ClockedMotorSpan]) {
        for view in views {
            let freq = view.clock_freq_hz;
            let offset = view.start_clock_exact - view.stream_t_start * freq;
            if self.projection_changed(oid, freq, offset) {
                self.push(
                    TapRecord::new(RecordKind::Projection, oid, self.clock_now())
                        .put(0, &freq.to_le_bytes())
                        .put(8, &offset.to_le_bytes()),
                );
            }
        }
    }

    pub(crate) fn record_halt(&self, oid: u32) {
        self.push(TapRecord::new(RecordKind::Halt, oid, self.clock_now()));
    }

    pub(crate) fn projection_changed(&self, oid: u32, freq: f64, offset: f64) -> bool {
        let mut known = self.projections.lock().expect("step tap projection lock");
        match known.iter_mut().find(|(o, _, _)| *o == oid) {
            Some(entry)
                if entry.1 == freq && (entry.2 - offset).abs() <= PROJECTION_SLACK_TICKS =>
            {
                false
            }
            Some(entry) => {
                *entry = (oid, freq, offset);
                true
            }
            None => {
                known.push((oid, freq, offset));
                true
            }
        }
    }

    fn clock_now(&self) -> u64 {
        match (self.clock_of)(self.mcu_id) {
            Some((clock, _)) => clock,
            None => {
                self.latch(format!(
                    "mcu {} has no clock estimate to stamp",
                    self.mcu_id
                ));
                0
            }
        }
    }

    fn push(&self, record: TapRecord) {
        match self.tx.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => self.latch(format!(
                "writer backlog full ({CHANNEL_RECORDS} records); the tap fell behind"
            )),
            Err(TrySendError::Disconnected(_)) => {
                self.latch("writer stopped; see the step-tap error log".to_string());
            }
        }
    }

    fn latch(&self, message: String) {
        let mut fault = self.fault.lock().expect("step tap fault lock");
        fault.get_or_insert(message);
    }

    pub(crate) fn check(&self) -> Result<(), SendError> {
        match self.fault.lock().expect("step tap fault lock").as_ref() {
            Some(message) => Err(SendError::Fatal(format!(
                "step tap mcu {}: {message}",
                self.mcu_id
            ))),
            None => Ok(()),
        }
    }
}

fn write_records(rx: &Receiver<TapRecord>, mut file: BufWriter<File>) {
    loop {
        let outcome = match rx.recv_timeout(IDLE_FLUSH) {
            Ok(record) => file.write_all(&record.0),
            Err(RecvTimeoutError::Timeout) => file.flush(),
            Err(RecvTimeoutError::Disconnected) => {
                if let Err(error) = file.flush() {
                    tracing::error!(event = "step_tap_write_failed", %error, "[step-tap] final flush failed");
                }
                return;
            }
        };
        if let Err(error) = outcome {
            tracing::error!(event = "step_tap_write_failed", %error, "[step-tap] write failed; stopping");
            return;
        }
    }
}

fn int_arg(name: &str, args: &[(String, ArgValue)], key: &str) -> Result<i64, String> {
    args.iter()
        .find(|(arg, _)| arg == key)
        .and_then(|(_, value)| match value {
            ArgValue::Int(v) => Some(*v),
            _ => None,
        })
        .ok_or_else(|| format!("{name} frame carries no integer `{key}`"))
}

fn narrow<T: TryFrom<i64>>(name: &str, key: &str, value: i64) -> Result<T, String> {
    T::try_from(value).map_err(|_| format!("{name} `{key}` {value} does not fit its wire width"))
}

pub(crate) fn encode_frame(
    name: &str,
    args: &[(String, ArgValue)],
    clock: u64,
) -> Result<Option<TapRecord>, String> {
    let get = |key: &str| int_arg(name, args, key);
    let oid = || -> Result<u32, String> { narrow(name, "oid", get("oid")?) };
    let record = match name {
        "kalico_wire_probe" => return Ok(None),
        "reset_step_clock" => TapRecord::new(RecordKind::ResetStepClock, oid()?, clock).put(
            0,
            &narrow::<u32>(name, "clock", get("clock")?)?.to_le_bytes(),
        ),
        "set_next_step_dir" => TapRecord::new(RecordKind::SetNextStepDir, oid()?, clock)
            .put(0, &[narrow::<u8>(name, "dir", get("dir")?)?]),
        "queue_step" => TapRecord::new(RecordKind::QueueStep, oid()?, clock)
            .put(
                0,
                &narrow::<u32>(name, "interval", get("interval")?)?.to_le_bytes(),
            )
            .put(
                4,
                &narrow::<u16>(name, "count", get("count")?)?.to_le_bytes(),
            )
            .put(6, &narrow::<i16>(name, "add", get("add")?)?.to_le_bytes()),
        "queue_step_hp" => TapRecord::new(RecordKind::QueueStepHp, oid()?, clock)
            .put(
                0,
                &narrow::<u32>(name, "interval", get("interval")?)?.to_le_bytes(),
            )
            .put(
                4,
                &narrow::<u16>(name, "count", get("count")?)?.to_le_bytes(),
            )
            .put(6, &narrow::<i32>(name, "add", get("add")?)?.to_le_bytes())
            .put(
                10,
                &narrow::<i32>(name, "add2", get("add2")?)?.to_le_bytes(),
            )
            .put(14, &[narrow::<u8>(name, "shift", get("shift")?)?]),
        "stepcompress_barrier" => TapRecord::new(RecordKind::Barrier, oid()?, clock)
            .put(0, &narrow::<u32>(name, "seq", get("seq")?)?.to_le_bytes()),
        "stepcompress_set_position" => TapRecord::new(RecordKind::SetPosition, oid()?, clock)
            .put(0, &narrow::<i32>(name, "pos", get("pos")?)?.to_le_bytes()),
        _ => {
            return Err(format!(
                "unrecognised frame `{name}`; the tap cannot replay it"
            ));
        }
    };
    Ok(Some(record))
}
