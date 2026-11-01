#[derive(Clone, Copy, Debug)]
pub struct Extend16 {
    last: u16,
    value: u64,
}

impl Extend16 {
    pub fn new(raw: u16) -> Self {
        Self {
            last: raw,
            value: 0,
        }
    }

    pub fn update(&mut self, raw: u16) -> u64 {
        self.value += u64::from(raw.wrapping_sub(self.last));
        self.last = raw;
        self.value
    }

    pub fn value(&self) -> u64 {
        self.value
    }
}
