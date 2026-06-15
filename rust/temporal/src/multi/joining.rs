use crate::TopProfile;
use crate::multi::parallel::fan_out_solves;
use crate::multi::{BatchError, JoiningStatus};
use crate::topp::chain::ChainGrid;

const MAX_SWEEPS: u32 = 10;

pub(crate) fn join_until_converged(
    chain_grids: &[ChainGrid],
    states: &mut [ChainState],
    corner_caps: &[f64],
    n_threads: usize,
) -> Result<(u32, JoiningStatus), BatchError> {
    for sweep in 1..=MAX_SWEEPS {
        let dirty_count = bidirectional_junction_sweep(states, corner_caps);
        if dirty_count == 0 {
            if states.iter().all(|s| !s.dirty) {
                return Ok((sweep, JoiningStatus::Converged));
            }
            let last_dirty_count = states.iter().filter(|s| s.dirty).count();
            return Ok((
                sweep,
                JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count },
            ));
        }
        fan_out_solves(chain_grids, states, n_threads)?;
    }
    let last_dirty = states.iter().filter(|s| s.dirty).count();
    Ok((
        MAX_SWEEPS,
        JoiningStatus::CappedAtMaxSweeps {
            last_dirty_count: last_dirty,
        },
    ))
}

pub(crate) struct ChainState {
    pub v_start: f64,
    pub v_end: f64,
    pub a_start: Option<f64>,
    pub profile: Option<TopProfile>,
    pub dirty: bool,
}

pub(crate) fn bidirectional_junction_sweep(
    states: &mut [ChainState],
    corner_caps: &[f64],
) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;

    for k in 0..corner_caps.len() {
        let target = corner_caps[k]
            .min(states[k].v_end)
            .min(states[k + 1].v_start);

        if (target - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = target;
            states[k].dirty = true;
            dirty_count += 1;
        }
        if (target - states[k + 1].v_start).abs() > EPS_VEL {
            states[k + 1].v_start = target;
            states[k + 1].dirty = true;
            dirty_count += 1;
        }
    }

    dirty_count
}

const TAIL_EXCHANGE_MAX: u32 = 3;
const TAIL_EXCHANGE_REL_TOL: f64 = 1e-3;
const TAIL_SAMPLES: usize = 32;

fn profile_time_map(profile: &TopProfile) -> Vec<f64> {
    let mut t = vec![0.0];
    for w in profile.samples.windows(2) {
        let v_sum = (w[0].v + w[1].v).max(1e-12);
        t.push(t.last().unwrap() + 2.0 * (w[1].s - w[0].s) / v_sum);
    }
    t
}

fn axis_velocity_at(profile: &TopProfile, chain: &ChainGrid, t_map: &[f64], tau: f64) -> [f64; 3] {
    let n = profile.samples.len();
    let total = *t_map.last().unwrap();
    let tau = tau.clamp(0.0, total);
    let j = t_map.partition_point(|&tj| tj <= tau).clamp(1, n - 1);
    let frac = (tau - t_map[j - 1]) / (t_map[j] - t_map[j - 1]).max(1e-12);
    let v = profile.samples[j - 1].v + (profile.samples[j].v - profile.samples[j - 1].v) * frac;
    let cp = if frac < 0.5 {
        chain.geom[j - 1].c_prime
    } else {
        chain.geom[j].c_prime
    };
    [cp[0] * v, cp[1] * v, cp[2] * v]
}

fn sample_boundary_window(
    profile: &TopProfile,
    chain: &ChainGrid,
    width: f64,
    from_end: bool,
) -> crate::FollowerHistory {
    let t_map = profile_time_map(profile);
    let total = *t_map.last().unwrap();
    let dt = width / TAIL_SAMPLES as f64;
    let mut axis_velocity: [Vec<f64>; 3] = Default::default();
    for m in 0..TAIL_SAMPLES {
        let offset = (m as f64 + 0.5) * dt;
        let tau = if from_end { total - offset } else { offset };
        let v = axis_velocity_at(profile, chain, &t_map, tau);
        for alpha in 0..3 {
            axis_velocity[alpha].push(v[alpha]);
        }
    }
    crate::FollowerHistory { dt, axis_velocity }
}

fn max_kernel_half_support(chain: &ChainGrid) -> f64 {
    chain
        .axis_kernels
        .iter()
        .flatten()
        .map(|k| k.support().1)
        .fold(0.0, f64::max)
}

/// The shaper window spans chain junctions (full stops): the neighbor chain's
/// ramp contributes to shaped speed near the boundary. After joining has
/// converged, re-solve each windowed chain with its neighbors' boundary-window
/// velocity samples as constants, iterated until total times settle.
pub(crate) fn exchange_follower_tails(
    chain_grids: &mut [ChainGrid],
    states: &mut [ChainState],
    n_threads: usize,
) -> Result<(), BatchError> {
    let n = chain_grids.len();
    if n < 2 || !chain_grids.iter().any(ChainGrid::has_active_windows) {
        return Ok(());
    }
    let mut prev_times: Vec<f64> = states
        .iter()
        .map(|s| {
            s.profile
                .as_ref()
                .expect("joining solved every chain")
                .total_time
        })
        .collect();
    for pass in 1..=TAIL_EXCHANGE_MAX {
        let tails: Vec<Option<(crate::FollowerHistory, crate::FollowerHistory)>> = (0..n)
            .map(|c| {
                if !chain_grids[c].has_active_windows() {
                    return None;
                }
                let width = max_kernel_half_support(&chain_grids[c]);
                let left = (c > 0).then(|| {
                    sample_boundary_window(
                        states[c - 1].profile.as_ref().unwrap(),
                        &chain_grids[c - 1],
                        width,
                        true,
                    )
                });
                let right = (c + 1 < n).then(|| {
                    sample_boundary_window(
                        states[c + 1].profile.as_ref().unwrap(),
                        &chain_grids[c + 1],
                        width,
                        false,
                    )
                });
                Some((left.unwrap_or_default(), right.unwrap_or_default()))
            })
            .collect();
        for (c, tail) in tails.into_iter().enumerate() {
            let Some((left, right)) = tail else { continue };
            if c > 0 {
                chain_grids[c].follower_history = Some(left);
            }
            if c + 1 < n {
                chain_grids[c].follower_terminal = Some(right);
            }
            states[c].dirty = true;
        }
        fan_out_solves(chain_grids, states, n_threads)?;
        let mut worst: (usize, f64) = (0, 0.0);
        for (c, state) in states.iter().enumerate() {
            let t = state.profile.as_ref().unwrap().total_time;
            let rel = (t - prev_times[c]).abs() / prev_times[c].max(1e-9);
            if rel > worst.1 {
                worst = (c, rel);
            }
            prev_times[c] = t;
        }
        if worst.1 <= TAIL_EXCHANGE_REL_TOL {
            return Ok(());
        }
        if pass == TAIL_EXCHANGE_MAX {
            return Err(BatchError::TailExchangeDiverged {
                passes: pass,
                chain: worst.0,
                rel_change: worst.1,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
