use crate::CHANNELS;

pub const PINS: usize = 4;
pub const MAX_WALLS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wall {
    pub pin: u8,
    pub active_high: bool,
    pub at_or_below: bool,
    pub coeffs: [i8; CHANNELS],
    pub threshold: i32,
}

impl Wall {
    pub fn triggered(&self, positions: &[i32; CHANNELS]) -> bool {
        let sum: i64 = self
            .coeffs
            .iter()
            .zip(positions)
            .map(|(&c, &p)| i64::from(c) * i64::from(p))
            .sum();
        if self.at_or_below {
            sum <= i64::from(self.threshold)
        } else {
            sum >= i64::from(self.threshold)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WallError {
    PinOutOfRange(u8),
    TooManyWalls,
    MixedPolarity(u8),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Walls {
    walls: [Option<Wall>; MAX_WALLS],
    count: usize,
    active_high: u8,
    used: u8,
}

impl Walls {
    pub fn push(&mut self, wall: Wall) -> Result<(), WallError> {
        if usize::from(wall.pin) >= PINS {
            return Err(WallError::PinOutOfRange(wall.pin));
        }
        if self.count == MAX_WALLS {
            return Err(WallError::TooManyWalls);
        }
        let bit = 1u8 << wall.pin;
        if self.used & bit != 0 && (self.active_high & bit != 0) != wall.active_high {
            return Err(WallError::MixedPolarity(wall.pin));
        }
        self.used |= bit;
        if wall.active_high {
            self.active_high |= bit;
        }
        self.walls[self.count] = Some(wall);
        self.count += 1;
        Ok(())
    }

    pub fn idle_levels(&self) -> u8 {
        self.used & !self.active_high
    }

    pub fn levels(&self, positions: &[i32; CHANNELS]) -> u8 {
        let mut triggered = 0u8;
        for wall in self.walls[..self.count].iter().flatten() {
            if wall.triggered(positions) {
                triggered |= 1 << wall.pin;
            }
        }
        self.idle_levels() ^ triggered
    }
}
