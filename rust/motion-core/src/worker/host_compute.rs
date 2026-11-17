//! A host slower than the one running the engine: the simulator's stand-in
//! for a Pi, whose clock stands still while host threads compute. Set
//! [`HOST_COMPUTE_SCALE_ENV`] to `<scale>` and every pipeline thread waits
//! out `scale` times the CPU time it spends before handing each item on.

use motion_pipeline::ComputeCharge;

pub(crate) const HOST_COMPUTE_SCALE_ENV: &str = "SERVAL_HOST_COMPUTE_SCALE";

pub(crate) fn from_env() -> Option<ComputeCharge> {
    let spec = std::env::var(HOST_COMPUTE_SCALE_ENV).ok()?;
    let charge = parse(&spec).unwrap_or_else(|error| panic!("{HOST_COMPUTE_SCALE_ENV}: {error}"));
    tracing::warn!(
        subsystem = "motion",
        event = "host_compute_charged",
        scale = charge.scale,
        "[ingress] planning waits out scaled CPU time on purpose ({HOST_COMPUTE_SCALE_ENV})"
    );
    Some(charge)
}

pub(crate) fn parse(spec: &str) -> Result<ComputeCharge, String> {
    let scale: f64 = spec
        .trim()
        .parse()
        .map_err(|e| format!("scale {spec:?}: {e}"))?;
    if !(scale.is_finite() && scale > 0.0) {
        return Err(format!("scale must be finite and positive, got {scale}"));
    }
    Ok(ComputeCharge {
        scale,
        thread_cpu: host_rt::clock::thread_cpu_time,
    })
}

#[cfg(test)]
mod tests;
