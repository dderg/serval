//! Drive-off endpoint: the real command dispatch and DC loop over a simulated
//! drive chain and an in-memory object dictionary. Same wire protocol as the
//! hardware endpoint, no EtherCAT master.
use std::sync::atomic::{AtomicI16, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ethercat_rt::claim::{
    all_slaves_reply, parse_fail_bringup, single_slave_reply, wait_for_claim,
};
use ethercat_rt::cli::parse_slaves;
use ethercat_rt::endpoint::{self, SimConfig, SimDrive, SIGTERM_RECEIVED};
use ethercat_rt::sdo::{DictObject, DictSdoBus, SdoBus};
use ethercat_rt::server::FrameServer;
use ethercat_rt::wire::claim_handshake_reply_frame;
use mcu_protocol::messages::SlaveState;

const STUB_CYCLE_NS: i64 = 1_000_000;
/// Per-slot tracking lag, alternating so every slot's telemetry — and so
/// every capture block — differs from its neighbour's.
const STUB_FOLLOWING_ERROR: [i32; 2] = [40, -25];

/// Read count of the object dictionary, so a test can assert how many SDO
/// probe/verify round trips a write cost. Answered without counting itself.
const STUB_PROBE_COUNTER_INDEX: u16 = 0x5FFF;
/// Writing 6077h injects a measured torque into the simulated drive, which is
/// what the sensorless endstop trips on.
const TXPDO_TORQUE_ACTUAL_INDEX: u16 = 0x6077;

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn stub_object_dictionary() -> DictSdoBus {
    DictSdoBus::new([
        (
            (0x2002, 0),
            DictObject {
                size: 2,
                value: [100, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x2003, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: Some(500),
            },
        ),
        (
            (0x2010, 1),
            DictObject {
                size: 4,
                value: [0; 4],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x6041, 0),
            DictObject {
                size: 2,
                value: [0x37, 0x02, 0, 0],
                read_only: true,
                unsigned_clamp_max: None,
            },
        ),
        (
            (TXPDO_TORQUE_ACTUAL_INDEX, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
    ])
}

/// The stub's fake drive as seen from the CoE mailbox: the object dictionary
/// plus the read-counter probe and the torque injection hook.
struct StubSdoBus {
    dict: DictSdoBus,
    torques: Arc<Vec<AtomicI16>>,
}

impl SdoBus for StubSdoBus {
    fn read(&mut self, slot: u8, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        if index == STUB_PROBE_COUNTER_INDEX {
            return Ok((4, self.dict.read_count.to_le_bytes()));
        }
        self.dict.read(slot, index, subindex)
    }

    fn write(&mut self, slot: u8, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        self.dict.write(slot, index, subindex, bytes)?;
        if index == TXPDO_TORQUE_ACTUAL_INDEX {
            let mut le = [0u8; 2];
            le.copy_from_slice(&bytes[..2]);
            let torque = i16::from_le_bytes(le);
            for cell in self.torques.iter() {
                cell.store(torque, Ordering::Relaxed);
            }
        }
        Ok(())
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());

    let fail_slave: Option<u8> = match parse_fail_bringup(&args) {
        Ok(v) => v,
        Err(msg) => {
            eprintln!("ec-rt-stub: {msg}");
            eprintln!("Usage: ethercat-rt-stub [--socket PATH] [--fail-bringup slave=N]");
            std::process::exit(2);
        }
    };
    let slaves = parse_slaves(&args).unwrap_or_else(|e| {
        eprintln!("ec-rt-stub: bad --slave config: {e}");
        std::process::exit(2);
    });
    let num_slaves = slaves.len();
    let fail_enable = args.iter().any(|a| a == "--fail-enable");
    let drive_fault_after: Option<u32> =
        arg_val(&args, "--drive-fault-after-cycles").and_then(|s| s.parse().ok());

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket} (NO HARDWARE)");

    endpoint::install_sigterm_handler();

    let claim_deadline = std::time::Instant::now() + Duration::from_secs(10);
    let Some(cid) = wait_for_claim(&mut server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt-stub")
    else {
        eprintln!("ec-rt-stub: bridge did not send ClaimHandshake within 10 s; aborting");
        std::process::exit(1);
    };

    if let Some(slave_idx) = fail_slave {
        let reply = single_slave_reply(slave_idx, SlaveState::Offline, 0);
        server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
        eprintln!("ec-rt-stub: --fail-bringup: sent Offline for slave {slave_idx}, exiting");
        std::process::exit(1);
    }

    server.respond(&claim_handshake_reply_frame(
        cid,
        &all_slaves_reply(num_slaves, SlaveState::Ok, 0),
    ));
    eprintln!("ec-rt-stub: handshake ok, entering stub loop");

    let drive = SimDrive::paced(num_slaves, STUB_CYCLE_NS as u64)
        .with_following_error(
            (0..num_slaves)
                .map(|s| STUB_FOLLOWING_ERROR[s % STUB_FOLLOWING_ERROR.len()])
                .collect(),
        )
        .with_enable_rc(if fail_enable {
            ethercat_rt::torque::ERR_ENABLE_FAILED
        } else {
            0
        })
        .with_fault_after_writes(drive_fault_after);
    let bus = StubSdoBus {
        dict: stub_object_dictionary(),
        torques: drive.torque_handle(),
    };
    let mut ctx = endpoint::sim_endpoint(
        SimConfig {
            server,
            live_tap_socket: &format!("{socket}.live"),
            slave_axes: slaves.iter().map(|s| s.axis).collect(),
            counts_per_mm: slaves.iter().map(|s| s.counts_per_mm).collect(),
            cycle_ns: STUB_CYCLE_NS,
            telemetry_period: u64::MAX,
        },
        drive,
        bus,
    );
    endpoint::run(&mut ctx);
}
