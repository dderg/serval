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
    pub fn only_channel(&self) -> Option<usize> {
        let mut used = self.coeffs.iter().enumerate().filter(|(_, &c)| c != 0);
        match (used.next(), used.next()) {
            (Some((channel, _)), None) => Some(channel),
            _ => None,
        }
    }

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
        self.levels_for(self.triggered(positions))
    }

    pub fn levels_for(&self, triggered_pins: u8) -> u8 {
        self.idle_levels() ^ triggered_pins
    }

    pub fn triggered(&self, positions: &[i32; CHANNELS]) -> u8 {
        self.triggered_where(positions, |_| true)
    }

    pub fn triggered_involving(&self, positions: &[i32; CHANNELS], channel: usize) -> u8 {
        self.triggered_where(positions, |wall| wall.coeffs[channel] != 0)
    }

    pub fn triggered_by_channel_alone(&self, positions: &[i32; CHANNELS], channel: usize) -> u8 {
        self.triggered_where(positions, |wall| wall.only_channel() == Some(channel))
    }

    fn triggered_where(&self, positions: &[i32; CHANNELS], select: impl Fn(&Wall) -> bool) -> u8 {
        let mut pins = 0u8;
        for wall in self.walls[..self.count].iter().flatten() {
            if select(wall) && wall.triggered(positions) {
                pins |= 1 << wall.pin;
            }
        }
        pins
    }
}
