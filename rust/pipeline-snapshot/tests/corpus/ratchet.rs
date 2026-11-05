//! What known engine bugs leave on a case, pinned per class by count and
//! worst value. A case fails on anything outside a pinned class and
//! whenever a pinned class grows, worsens or shrinks, so the pins tighten
//! as fixes land; the failure prints the numbers to pin.

use std::fmt::Debug;

/// One class's tally on a case: how many, and the worst of them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tally<K> {
    pub class: K,
    pub count: usize,
    pub worst: f64,
}

pub fn tally<K: PartialEq + Copy>(
    observations: impl IntoIterator<Item = (K, f64)>,
) -> Vec<Tally<K>> {
    let mut tallies: Vec<Tally<K>> = Vec::new();
    for (class, value) in observations {
        match tallies.iter_mut().find(|seen| seen.class == class) {
            Some(seen) => {
                seen.count += 1;
                seen.worst = seen.worst.max(value);
            }
            None => tallies.push(Tally {
                class,
                count: 1,
                worst: value,
            }),
        }
    }
    tallies
}

/// Where `observed` departs from `pinned`, `printed` rounding a worst value
/// as the pins state it: one line per class to pin, re-pin or unpin.
pub fn departures<K: PartialEq + Copy + Debug>(
    observed: &[Tally<K>],
    pinned: &[Tally<K>],
    printed: impl Fn(K, f64) -> f64,
) -> Vec<String> {
    let mut departures: Vec<String> = observed
        .iter()
        .filter(|seen| {
            !pinned.iter().any(|pin| {
                pin.class == seen.class
                    && pin.count == seen.count
                    && pin.worst == printed(seen.class, seen.worst)
            })
        })
        .map(|seen| {
            format!(
                "pin {:?}: count {}, worst {}",
                seen.class,
                seen.count,
                printed(seen.class, seen.worst)
            )
        })
        .collect();
    departures.extend(
        pinned
            .iter()
            .filter(|pin| !observed.iter().any(|seen| seen.class == pin.class))
            .map(|pin| format!("unpin {:?}: none left", pin.class)),
    );
    departures
}

pub fn rounded(value: f64, decimals: i32) -> f64 {
    let scale = 10_f64.powi(decimals);
    (value * scale).round() / scale
}
