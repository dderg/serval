pub const MIN_DIR_STEP_TICKS: u64 = 9;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ambiguous {
    pub step: u64,
    pub edge: u64,
}

pub struct DirMerge<'a> {
    edges: &'a [(u64, bool)],
    next: usize,
    level: bool,
}

impl<'a> DirMerge<'a> {
    pub fn new(edges: &'a [(u64, bool)], level: bool) -> Self {
        Self {
            edges,
            next: 0,
            level,
        }
    }

    pub fn sign_at(&mut self, time: u64) -> Result<i32, Ambiguous> {
        while let Some(&(edge, level)) = self.edges.get(self.next) {
            if edge >= time {
                break;
            }
            self.level = level;
            self.next += 1;
        }
        let neighbours = [
            self.next.checked_sub(1).and_then(|i| self.edges.get(i)),
            self.edges.get(self.next),
        ];
        for &(edge, _) in neighbours.into_iter().flatten() {
            if edge.abs_diff(time) < MIN_DIR_STEP_TICKS {
                return Err(Ambiguous { step: time, edge });
            }
        }
        Ok(if self.level { 1 } else { -1 })
    }

    pub fn finish(self) -> bool {
        self.edges.last().map_or(self.level, |&(_, level)| level)
    }
}
