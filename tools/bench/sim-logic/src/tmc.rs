pub const DRIVERS: usize = 4;
pub const REPLY_LEN: usize = 8;
const REGISTERS: usize = 128;
const WRITE_FLAG: u8 = 0x80;
const SYNC_NIBBLE: u8 = 0x05;
const SYNC_MASK: u8 = 0x0F;
const REPLY_ADDRESS: u8 = 0xFF;
const READ_LEN: usize = 4;
const WRITE_LEN: usize = 8;
const VERSION_2209: u32 = 0x21;
const SENDDELAY_UNIT_BITS: u16 = 8;

pub mod reg {
    pub const GCONF: u8 = 0x00;
    pub const GSTAT: u8 = 0x01;
    pub const IFCNT: u8 = 0x02;
    pub const SLAVECONF: u8 = 0x03;
    pub const IOIN: u8 = 0x06;
    pub const TSTEP: u8 = 0x12;
    pub const CHOPCONF: u8 = 0x6C;
    pub const DRV_STATUS: u8 = 0x6F;
}

pub const GSTAT_RESET: u32 = 1 << 0;
pub const IOIN_PDN_UART: u32 = 1 << 6;
pub const CHOPCONF_DEDGE: u32 = 1 << 29;
pub const TSTEP_STANDSTILL: u32 = 0xF_FFFF;
const SLAVECONF_SENDDELAY_SHIFT: u32 = 8;
const SLAVECONF_SENDDELAY_MASK: u32 = 0xF;

pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        let mut b = byte;
        for _ in 0..8 {
            crc = if (crc >> 7) ^ (b & 1) != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
            b >>= 1;
        }
    }
    crc
}

#[derive(Clone, Copy, Debug)]
struct Driver {
    regs: [u32; REGISTERS],
    ifcnt: u8,
}

impl Driver {
    fn powered_on() -> Self {
        let mut regs = [0u32; REGISTERS];
        regs[usize::from(reg::GSTAT)] = GSTAT_RESET;
        Self { regs, ifcnt: 0 }
    }

    fn read(&self, register: u8) -> u32 {
        match register {
            reg::IFCNT => u32::from(self.ifcnt),
            reg::IOIN => (VERSION_2209 << 24) | IOIN_PDN_UART,
            reg::TSTEP => TSTEP_STANDSTILL,
            reg::DRV_STATUS => 0,
            _ => self.regs[usize::from(register)],
        }
    }

    fn write(&mut self, register: u8, value: u32) {
        let slot = &mut self.regs[usize::from(register)];
        match register {
            reg::GSTAT => *slot &= !value,
            reg::IFCNT | reg::IOIN | reg::DRV_STATUS => {}
            _ => *slot = value,
        }
        self.ifcnt = self.ifcnt.wrapping_add(1);
    }

    fn reply_delay_bits(&self) -> u16 {
        let senddelay = (self.regs[usize::from(reg::SLAVECONF)] >> SLAVECONF_SENDDELAY_SHIFT)
            & SLAVECONF_SENDDELAY_MASK;
        SENDDELAY_UNIT_BITS * (2 * (senddelay as u16 / 2) + 1)
    }
}

pub fn reply_wait_ticks(delay_bits: u16, bit_ns: u32, tick_ns: u32) -> u32 {
    (u32::from(delay_bits) * bit_ns).div_ceil(tick_ns) + 1
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reply {
    pub frame: [u8; REPLY_LEN],
    pub delay_bits: u16,
}

#[derive(Clone, Copy, Debug)]
pub struct TmcBus {
    drivers: [Driver; DRIVERS],
    rx: [u8; WRITE_LEN],
    rx_len: usize,
    pub writes: u8,
    pub reads: u8,
    pub rejects: u8,
}

impl Default for TmcBus {
    fn default() -> Self {
        Self {
            drivers: [Driver::powered_on(); DRIVERS],
            rx: [0; WRITE_LEN],
            rx_len: 0,
            writes: 0,
            reads: 0,
            rejects: 0,
        }
    }
}

impl TmcBus {
    pub fn register(&self, address: u8, register: u8) -> u32 {
        self.drivers[usize::from(address)].read(register)
    }

    pub fn dedge_mask(&self) -> u8 {
        self.drivers
            .iter()
            .enumerate()
            .filter(|(_, d)| d.regs[usize::from(reg::CHOPCONF)] & CHOPCONF_DEDGE != 0)
            .fold(0, |mask, (address, _)| mask | (1 << address))
    }

    pub fn on_overrun(&mut self) {
        self.reject();
    }

    pub fn on_reply_collision(&mut self) {
        self.rejects = self.rejects.wrapping_add(1);
    }

    pub fn on_idle(&mut self) {
        if self.rx_len != 0 {
            self.reject();
        }
    }

    pub fn on_byte(&mut self, byte: u8) -> Option<Reply> {
        if self.rx_len == 0 && byte & SYNC_MASK != SYNC_NIBBLE {
            self.rejects = self.rejects.wrapping_add(1);
            return None;
        }
        self.rx[self.rx_len] = byte;
        self.rx_len += 1;
        if self.rx_len < 3 || self.rx_len < self.frame_len() {
            return None;
        }
        let len = self.rx_len;
        self.rx_len = 0;
        self.complete(len)
    }

    fn frame_len(&self) -> usize {
        if self.rx[2] & WRITE_FLAG != 0 {
            WRITE_LEN
        } else {
            READ_LEN
        }
    }

    fn reject(&mut self) {
        self.rx_len = 0;
        self.rejects = self.rejects.wrapping_add(1);
    }

    fn complete(&mut self, len: usize) -> Option<Reply> {
        let frame = &self.rx[..len];
        let address = frame[1];
        let register = frame[2] & !WRITE_FLAG;
        if crc8(&frame[..len - 1]) != frame[len - 1] || usize::from(address) >= DRIVERS {
            self.rejects = self.rejects.wrapping_add(1);
            return None;
        }
        let driver = &mut self.drivers[usize::from(address)];
        if len == WRITE_LEN {
            let value = u32::from_be_bytes([frame[3], frame[4], frame[5], frame[6]]);
            driver.write(register, value);
            self.writes = self.writes.wrapping_add(1);
            return None;
        }
        let value = driver.read(register).to_be_bytes();
        let mut reply = [
            SYNC_NIBBLE,
            REPLY_ADDRESS,
            register,
            value[0],
            value[1],
            value[2],
            value[3],
            0,
        ];
        reply[REPLY_LEN - 1] = crc8(&reply[..REPLY_LEN - 1]);
        self.reads = self.reads.wrapping_add(1);
        Some(Reply {
            frame: reply,
            delay_bits: driver.reply_delay_bits(),
        })
    }
}
