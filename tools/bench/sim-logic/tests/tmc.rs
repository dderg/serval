use bench_sim_logic::tmc::{crc8, reg, reply_wait_ticks, Reply, TmcBus, GSTAT_RESET};

const BIT_NS: u32 = 25_000;
const SAMPLE_NS: u32 = 100_000;
const KLIPPER_WINDOW_BITS: u32 = 64;

const READ_IFCNT_ADDR0: [u8; 4] = [0xF5, 0x00, 0x02, 0xC8];
const READ_CHOPCONF_ADDR2: [u8; 4] = [0xF5, 0x02, 0x6C, 0xD6];
const WRITE_CHOPCONF_ADDR1: [u8; 8] = [0xF5, 0x01, 0xEC, 0x30, 0x01, 0x00, 0x53, 0xDA];
const REPLY_IFCNT_1: [u8; 8] = [0x05, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x01, 0xC5];
const REPLY_CHOPCONF: [u8; 8] = [0x05, 0xFF, 0x6C, 0x30, 0x01, 0x00, 0x53, 0x02];

fn feed(bus: &mut TmcBus, bytes: &[u8]) -> Option<Reply> {
    let mut reply = None;
    for &byte in bytes {
        if let Some(r) = bus.on_byte(byte) {
            assert!(reply.is_none(), "one reply per frame");
            reply = Some(r);
        }
    }
    reply
}

fn read(addr: u8, register: u8) -> [u8; 4] {
    let mut frame = [0xF5, addr, register, 0];
    frame[3] = crc8(&frame[..3]);
    frame
}

fn write(addr: u8, register: u8, value: u32) -> [u8; 8] {
    let v = value.to_be_bytes();
    let mut frame = [0xF5, addr, register | 0x80, v[0], v[1], v[2], v[3], 0];
    frame[7] = crc8(&frame[..7]);
    frame
}

fn value_of(reply: &Reply) -> u32 {
    u32::from_be_bytes([
        reply.frame[3],
        reply.frame[4],
        reply.frame[5],
        reply.frame[6],
    ])
}

#[test]
fn the_crc_matches_klippys_encoder() {
    assert_eq!(read(0, reg::IFCNT), READ_IFCNT_ADDR0);
    assert_eq!(read(2, reg::CHOPCONF), READ_CHOPCONF_ADDR2);
    assert_eq!(write(1, reg::CHOPCONF, 0x3001_0053), WRITE_CHOPCONF_ADDR1);
}

#[test]
fn a_write_counts_in_ifcnt_and_reads_back_byte_exact() {
    let mut bus = TmcBus::default();
    assert_eq!(value_of(&feed(&mut bus, &read(1, reg::IFCNT)).unwrap()), 0);
    assert_eq!(feed(&mut bus, &WRITE_CHOPCONF_ADDR1), None);
    assert_eq!(
        feed(&mut bus, &read(1, reg::IFCNT)).unwrap().frame,
        REPLY_IFCNT_1
    );
    assert_eq!(
        feed(&mut bus, &read(1, reg::CHOPCONF)).unwrap().frame,
        REPLY_CHOPCONF
    );
    assert_eq!(
        bus.dedge_mask(),
        0b0010,
        "0x30010053 sets dedge on address 1"
    );
    assert_eq!(value_of(&feed(&mut bus, &read(0, reg::IFCNT)).unwrap()), 0);
    assert_eq!((bus.writes, bus.reads, bus.rejects), (1, 4, 0));
}

#[test]
fn gstat_reports_a_power_on_reset_and_clears_on_write_one() {
    let mut bus = TmcBus::default();
    assert_eq!(bus.register(3, reg::GSTAT), GSTAT_RESET);
    feed(&mut bus, &write(3, reg::GSTAT, 0b110));
    assert_eq!(bus.register(3, reg::GSTAT), GSTAT_RESET);
    feed(&mut bus, &write(3, reg::GSTAT, GSTAT_RESET));
    assert_eq!(value_of(&feed(&mut bus, &read(3, reg::GSTAT)).unwrap()), 0);
    assert_eq!(bus.register(3, reg::IFCNT), 2);
}

#[test]
fn ifcnt_wraps_at_eight_bits() {
    let mut bus = TmcBus::default();
    for _ in 0..257 {
        feed(&mut bus, &write(0, reg::GCONF, 1));
    }
    assert_eq!(bus.register(0, reg::IFCNT), 1);
}

#[test]
fn a_bad_crc_is_ignored_and_counted() {
    let mut bus = TmcBus::default();
    let mut corrupt = WRITE_CHOPCONF_ADDR1;
    corrupt[5] ^= 1;
    assert_eq!(feed(&mut bus, &corrupt), None);
    assert_eq!(bus.register(1, reg::IFCNT), 0);
    assert_eq!(bus.register(1, reg::CHOPCONF), 0);
    assert_eq!(bus.rejects, 1);
}

#[test]
fn stray_bytes_and_idle_gaps_resynchronise_the_parser() {
    let mut bus = TmcBus::default();
    assert_eq!(feed(&mut bus, &[0x00, 0xFF]), None);
    feed(&mut bus, &WRITE_CHOPCONF_ADDR1[..5]);
    bus.on_idle();
    assert_eq!(bus.rejects, 3);
    assert_eq!(feed(&mut bus, &read(1, reg::IFCNT)).unwrap().frame[6], 0);
}

#[test]
fn addresses_beyond_the_four_drivers_get_no_reply() {
    let mut bus = TmcBus::default();
    assert_eq!(feed(&mut bus, &read(4, reg::IFCNT)), None);
    assert_eq!(bus.rejects, 1);
}

#[test]
fn ioin_reports_a_tmc2209() {
    let mut bus = TmcBus::default();
    let ioin = value_of(&feed(&mut bus, &read(0, reg::IOIN)).unwrap());
    assert_eq!(ioin >> 24, 0x21);
}

#[test]
fn the_reply_waits_senddelay_bit_times() {
    let mut bus = TmcBus::default();
    let delay = |bus: &mut TmcBus| feed(bus, &read(2, reg::IFCNT)).unwrap().delay_bits;
    assert_eq!(delay(&mut bus), 8);
    for (senddelay, bits) in [(1u32, 8u16), (2, 24), (3, 24), (4, 40), (15, 120)] {
        feed(&mut bus, &write(2, reg::SLAVECONF, senddelay << 8));
        assert_eq!(delay(&mut bus), bits, "senddelay {senddelay}");
    }
}

#[test]
fn a_false_sync_byte_costs_one_frame_and_the_retry_after_idle_parses() {
    let mut bus = TmcBus::default();
    let mut stream = vec![0x05];
    stream.extend_from_slice(&READ_IFCNT_ADDR0);
    assert_eq!(feed(&mut bus, &stream), None);
    bus.on_idle();
    assert_eq!(bus.reads, 0);
    assert!(bus.rejects >= 1);
    assert_eq!(feed(&mut bus, &READ_IFCNT_ADDR0).unwrap().frame[6], 0);
}

#[test]
fn a_bad_crc_on_a_read_gets_no_reply() {
    let mut bus = TmcBus::default();
    let mut corrupt = READ_CHOPCONF_ADDR2;
    corrupt[3] ^= 0x80;
    assert_eq!(feed(&mut bus, &corrupt), None);
    assert_eq!((bus.reads, bus.rejects), (0, 1));
    assert!(feed(&mut bus, &READ_CHOPCONF_ADDR2).is_some());
}

#[test]
fn back_to_back_frames_parse_without_an_idle_gap() {
    let mut bus = TmcBus::default();
    let mut stream = WRITE_CHOPCONF_ADDR1.to_vec();
    stream.extend_from_slice(&WRITE_CHOPCONF_ADDR1);
    feed(&mut bus, &stream);
    assert_eq!(bus.register(1, reg::IFCNT), 2);
    assert_eq!(bus.rejects, 0);
}

#[test]
fn an_overrun_drops_the_frame_in_progress() {
    let mut bus = TmcBus::default();
    feed(&mut bus, &WRITE_CHOPCONF_ADDR1[..3]);
    bus.on_overrun();
    assert_eq!(bus.rejects, 1);
    feed(&mut bus, &WRITE_CHOPCONF_ADDR1);
    assert_eq!(bus.register(1, reg::IFCNT), 1);
}

#[test]
fn an_overrun_between_frames_still_counts() {
    let mut bus = TmcBus::default();
    bus.on_overrun();
    assert_eq!(bus.rejects, 1);
}

#[test]
fn rejects_wrap_like_the_other_counters() {
    let mut bus = TmcBus::default();
    for _ in 0..300 {
        bus.on_overrun();
    }
    assert_eq!(bus.rejects, (300 % 256) as u8);
}

#[test]
fn the_reply_starts_inside_klippers_receive_window() {
    let mut bus = TmcBus::default();
    feed(&mut bus, &write(0, reg::SLAVECONF, 2 << 8));
    let delay = feed(&mut bus, &read(0, reg::IFCNT)).unwrap().delay_bits;
    let ticks = reply_wait_ticks(delay, BIT_NS, SAMPLE_NS);
    let earliest_ns = (ticks - 1) * SAMPLE_NS;
    let latest_ns = ticks * SAMPLE_NS;
    assert!(
        earliest_ns >= u32::from(delay) * BIT_NS,
        "never before SENDDELAY"
    );
    assert!(latest_ns <= KLIPPER_WINDOW_BITS * BIT_NS, "{latest_ns} ns");
}

#[test]
fn the_wait_never_undercuts_senddelay() {
    for bits in [8u16, 24, 40, 56, 120] {
        let ticks = reply_wait_ticks(bits, BIT_NS, SAMPLE_NS);
        assert!(
            (ticks - 1) * SAMPLE_NS >= u32::from(bits) * BIT_NS,
            "{bits} bits"
        );
        assert!(
            (ticks - 2) * SAMPLE_NS < u32::from(bits) * BIT_NS,
            "{bits} bits"
        );
    }
}
