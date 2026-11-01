use bench_sim_logic::tmc::{crc8, reg, Reply, TmcBus, GSTAT_RESET};

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
