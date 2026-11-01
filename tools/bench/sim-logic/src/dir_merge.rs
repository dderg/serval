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

    pub fn sign_at(&mut self, time: u64) -> i32 {
        while let Some(&(edge, level)) = self.edges.get(self.next) {
            if edge >= time {
                break;
            }
            self.level = level;
            self.next += 1;
        }
        if self.level {
            1
        } else {
            -1
        }
    }

    pub fn finish(self) -> bool {
        self.edges.last().map_or(self.level, |&(_, level)| level)
    }
}
