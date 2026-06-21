use crate::multi::GridStrategy;
use crate::topp::chain::MAX_JUNCTION_SPACING_RATIO;
use nurbs::VectorNurbs;

pub(crate) fn compute_n(strategy: &GridStrategy, curve: &VectorNurbs<f64, 3>) -> usize {
    match *strategy {
        GridStrategy::Fixed(n) => n,
        GridStrategy::Adaptive {
            min_n,
            max_n,
            target_grid_spacing_mm,
        } => {
            debug_assert!(
                target_grid_spacing_mm > 0.0,
                "target_grid_spacing_mm must be > 0; got {target_grid_spacing_mm}"
            );
            let l = control_polygon_length_mm(curve);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let n = (l / target_grid_spacing_mm).ceil() as usize;
            n.clamp(min_n, max_n)
        }
    }
}

pub(crate) fn classify_absorbed(
    ns: &[usize],
    curves: &[&VectorNurbs<f64, 3>],
    max_n: Option<usize>,
) -> Vec<bool> {
    let n = ns.len();
    let mut absorbed = vec![false; n];
    if n < 2 {
        return absorbed;
    }
    let lengths: Vec<f64> = curves
        .iter()
        .map(|c| control_polygon_length_mm(c))
        .collect();
    let h = |n_seg: usize, l: f64| -> f64 {
        if n_seg <= 1 {
            l
        } else {
            l / (n_seg - 1) as f64
        }
    };
    let finest = |i: usize| -> f64 { h(max_n.unwrap_or(ns[i]), lengths[i]) };
    for i in 0..n {
        let unresolvable = |nb: usize| -> bool {
            lengths[i] <= 0.0 || finest(nb) / lengths[i] > MAX_JUNCTION_SPACING_RATIO
        };
        let left_bad = i > 0 && unresolvable(i - 1);
        let right_bad = i + 1 < n && unresolvable(i + 1);
        if left_bad || right_bad {
            absorbed[i] = true;
        }
    }
    if absorbed.iter().all(|&a| a) {
        let longest = (0..n)
            .max_by(|&a, &b| lengths[a].total_cmp(&lengths[b]))
            .unwrap();
        absorbed[longest] = false;
    }
    absorbed
}

pub(crate) fn reconcile_junction_n(
    ns: &mut [usize],
    curves: &[&VectorNurbs<f64, 3>],
    max_n: Option<usize>,
    absorbed: &[bool],
) {
    debug_assert_eq!(ns.len(), curves.len());
    debug_assert_eq!(ns.len(), absorbed.len());
    let live: Vec<usize> = (0..ns.len()).filter(|&i| !absorbed[i]).collect();
    if live.len() < 2 {
        return;
    }

    let lengths: Vec<f64> = curves
        .iter()
        .map(|c| control_polygon_length_mm(c))
        .collect();

    let h = |n: usize, l: f64| -> f64 { if n <= 1 { l } else { l / (n - 1) as f64 } };

    let reconcile_pair =
        |n_left: &mut usize, n_right: &mut usize, l_left: f64, l_right: f64, cap: Option<usize>| {
            let hl = h(*n_left, l_left);
            let hr = h(*n_right, l_right);

            if hl <= 0.0 || hr <= 0.0 {
                return;
            }

            let ratio = hl / hr;
            if ratio > MAX_JUNCTION_SPACING_RATIO {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n_needed = (l_left / (hr * MAX_JUNCTION_SPACING_RATIO)).ceil() as usize + 1;
                let n_new = match cap {
                    Some(c) => n_needed.min(c).max(*n_left),
                    None => n_needed.max(*n_left),
                };
                *n_left = n_new;
                let hl_new = h(*n_left, l_left);
                if hl_new > hr * MAX_JUNCTION_SPACING_RATIO {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let n_right_max =
                        (l_right * MAX_JUNCTION_SPACING_RATIO / hl_new).floor() as usize + 1;
                    *n_right = (*n_right).min(n_right_max).max(2);
                }
            } else if hr / hl > MAX_JUNCTION_SPACING_RATIO {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n_needed = (l_right / (hl * MAX_JUNCTION_SPACING_RATIO)).ceil() as usize + 1;
                let n_new = match cap {
                    Some(c) => n_needed.min(c).max(*n_right),
                    None => n_needed.max(*n_right),
                };
                *n_right = n_new;
                let hr_new = h(*n_right, l_right);
                if hr_new > hl * MAX_JUNCTION_SPACING_RATIO {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let n_left_max =
                        (l_left * MAX_JUNCTION_SPACING_RATIO / hr_new).floor() as usize + 1;
                    *n_left = (*n_left).min(n_left_max).max(2);
                }
            }
        };

    for w in live.windows(2) {
        let (i, j) = (w[0], w[1]);
        let (left, right) = ns.split_at_mut(j);
        reconcile_pair(&mut left[i], &mut right[0], lengths[i], lengths[j], max_n);
    }

    for w in live.windows(2).rev() {
        let (i, j) = (w[0], w[1]);
        let (left, right) = ns.split_at_mut(j);
        reconcile_pair(&mut left[i], &mut right[0], lengths[i], lengths[j], max_n);
    }
}

fn control_polygon_length_mm(curve: &VectorNurbs<f64, 3>) -> f64 {
    let cps = curve.control_points();
    cps.windows(2)
        .map(|w| {
            let dx = w[1][0] - w[0][0];
            let dy = w[1][1] - w[0][1];
            let dz = w[1][2] - w[0][2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

#[cfg(test)]
mod tests;
