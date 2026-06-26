use crate::frontend::Move;

use super::CornerFitConfig;
use super::vec3::{norm, sub};

pub(super) mod kappa_signal;
pub(super) mod position_greedy;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HeartKind {
    #[default]
    PositionGreedy,
    KappaSignal,
}

impl HeartKind {
    pub(super) fn build(self) -> Box<dyn Heart> {
        match self {
            HeartKind::PositionGreedy => Box::new(position_greedy::PositionGreedy),
            HeartKind::KappaSignal => Box::new(kappa_signal::KappaSignal),
        }
    }
}

/// A maximal sub-run of consecutive line legs (local indices into the chain,
/// inclusive) the heart judges reconstructable as one circular arc. The causal
/// driver turns each span into a bare arc, eases it into its straight neighbours,
/// and inherits its boundary curvature forward so the element stream stays G2.
pub(super) trait Heart {
    fn arc_spans(
        &self,
        chain: &[Move],
        tol: f64,
        min_run: usize,
        corner: CornerFitConfig,
    ) -> Vec<(usize, usize)>;
}

/// Cumulative chord arc length `s` and signed turning angle `theta` of a vertex
/// polyline in a fixed plane, sampled at each vertex. `s[0] = 0`, `theta[0] = 0`;
/// `theta[i]` is the accumulated heading change from the first chord.
pub(super) fn turning_signal(verts: &[[f64; 3]], plane_normal: [f64; 3]) -> (Vec<f64>, Vec<f64>) {
    use super::vec3::{cross, dot};
    let mut s = vec![0.0_f64];
    let mut theta = vec![0.0_f64];
    if verts.len() < 2 {
        return (s, theta);
    }
    let mut prev_dir = sub(verts[1], verts[0]);
    let mut acc_s = 0.0;
    let mut acc_theta = 0.0;
    for w in verts.windows(2) {
        let chord = sub(w[1], w[0]);
        let len = norm(chord);
        acc_s += len;
        let cur_dir = chord;
        let cs = dot(cross(prev_dir, cur_dir), plane_normal);
        let cc = dot(prev_dir, cur_dir);
        acc_theta += cs.atan2(cc);
        s.push(acc_s);
        theta.push(acc_theta);
        prev_dir = cur_dir;
    }
    (s, theta)
}
