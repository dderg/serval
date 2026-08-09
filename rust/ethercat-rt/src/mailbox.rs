//! Off-loop executor for CoE mailbox traffic.
//!
//! SDO transactions block for milliseconds to seconds (mailbox round trips,
//! the drive's internal EEPROM save, the master's per-attempt SDO timeout). The
//! DC loop must keep process data flowing every cycle — a slave in OP drops
//! to SAFE-OP (ErC1.1, emergency 0x8700) when cyclic frames pause past its
//! sync watchdog. So mailbox work runs on this dedicated thread while the DC
//! loop keeps cycling; the EtherCAT master serializes access between the DC
//! loop and mailbox traffic, so the cyclic loop never blocks on an SDO.
//!
//! Requests execute strictly in submission order (single worker, FIFO
//! channel), preserving write-then-readback semantics per client call.
//!
//! Because the in-kernel IgH master does that serialization itself — the SDO is
//! a blocking `ecrt_master_sdo_*` call that sleeps while the master FSM pumps
//! the mailbox over its own cycles — the worker is safe to run as plain
//! SCHED_OTHER on the housekeeping cores ([`WorkerScheduling::Normal`], the
//! default). The pinned low-priority SCHED_FIFO companion
//! ([`WorkerScheduling::RealtimeCompanion`], selected by `--mailbox-cpu`)
//! exists for the SOEM-style master where the SDO busy-polls a raw socket
//! shared with the DC loop, so a mid-transaction deschedule traps the cyclic
//! frame; see [`crate::thread_prio::assume_companion_rt_scheduling`].

use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::thread::JoinHandle;

use mcu_protocol::messages::{SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse};

use crate::sdo::{execute_sdo_read, execute_sdo_write, SdoBus};
use crate::thread_prio::{assume_companion_rt_scheduling, demote_to_normal_scheduling};

pub enum WorkerScheduling {
    RealtimeCompanion { cpu: usize, priority: i32 },
    Normal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LimitEntry {
    pub slot: u8,
    pub ferr_counts: u32,
    pub torque_tenth_pct: u16,
}

pub enum MailboxRequest {
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
    },
    WriteLimits {
        correlation_id: u32,
        entries: Vec<LimitEntry>,
        restore: bool,
    },
}

pub enum MailboxReply {
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
        resp: SdoReadResponse,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
        resp: SdoWriteResponse,
    },
    WriteLimits {
        correlation_id: u32,
        rc: i32,
        entries: Vec<LimitEntry>,
        restore: bool,
    },
}

pub struct MailboxWorker {
    requests: Sender<MailboxRequest>,
    replies: Receiver<MailboxReply>,
    handle: Option<JoinHandle<()>>,
}

impl MailboxWorker {
    pub fn spawn<B, L>(mut bus: B, mut write_limits: L, scheduling: WorkerScheduling) -> Self
    where
        B: SdoBus + Send + 'static,
        L: FnMut(u8, u32, u16) -> i32 + Send + 'static,
    {
        let (req_tx, req_rx) = channel::<MailboxRequest>();
        let (rep_tx, rep_rx) = channel::<MailboxReply>();
        let handle = std::thread::Builder::new()
            .name("ec-rt-mailbox".into())
            .spawn(move || {
                match scheduling {
                    WorkerScheduling::RealtimeCompanion { cpu, priority } => {
                        assume_companion_rt_scheduling(cpu, priority)
                    }
                    WorkerScheduling::Normal => demote_to_normal_scheduling(),
                }
                while let Ok(req) = req_rx.recv() {
                    let reply = match req {
                        MailboxRequest::SdoRead {
                            correlation_id,
                            msg,
                        } => MailboxReply::SdoRead {
                            correlation_id,
                            resp: execute_sdo_read(&mut bus, &msg),
                            msg,
                        },
                        MailboxRequest::SdoWrite {
                            correlation_id,
                            msg,
                        } => MailboxReply::SdoWrite {
                            correlation_id,
                            resp: execute_sdo_write(&mut bus, &msg),
                            msg,
                        },
                        MailboxRequest::WriteLimits {
                            correlation_id,
                            entries,
                            restore,
                        } => {
                            let mut rc = 0;
                            for e in &entries {
                                rc = write_limits(e.slot, e.ferr_counts, e.torque_tenth_pct);
                                if rc != 0 {
                                    break;
                                }
                            }
                            MailboxReply::WriteLimits {
                                correlation_id,
                                rc,
                                entries,
                                restore,
                            }
                        }
                    };
                    if rep_tx.send(reply).is_err() {
                        return;
                    }
                }
            })
            .expect("spawn ec-rt-mailbox thread");
        Self {
            requests: req_tx,
            replies: rep_rx,
            handle: Some(handle),
        }
    }

    /// Queue a mailbox transaction; never blocks. Panics if the worker thread
    /// died — that is a bug, not a runtime condition to recover from.
    pub fn submit(&self, req: MailboxRequest) {
        self.requests
            .send(req)
            .expect("ec-rt-mailbox thread is gone");
    }

    /// Non-blocking poll for one completed transaction; call from the DC loop
    /// each cycle until it returns None.
    pub fn try_recv(&self) -> Option<MailboxReply> {
        match self.replies.try_recv() {
            Ok(reply) => Some(reply),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                panic!("ec-rt-mailbox thread is gone");
            }
        }
    }
}

impl Drop for MailboxWorker {
    fn drop(&mut self) {
        let (sink, _) = channel();
        let _ = std::mem::replace(&mut self.requests, sink);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests;
