use crate::mcu_config::{McuAxisConfig, McuCaps};

pub(crate) fn resolve_motion_caps(
    caps: Option<mcu_protocol::messages::RuntimeCapsResponse>,
    label: &str,
    handle: u32,
) -> Result<McuCaps, String> {
    caps.map(McuCaps::from).ok_or_else(|| {
        format!(
            "no runtime caps for {label} MCU (handle={handle}) — cannot size piece rings; \
             firmware not flashed or QueryRuntimeCaps failed at attach"
        )
    })
}

pub(crate) fn require_events_dir_for_mcu_transport(
    mcu_transport: bool,
    events_dir: Option<&std::path::Path>,
    mcu_label: &str,
) -> Result<(), String> {
    if mcu_transport && events_dir.is_none() {
        return Err(format!(
            "attach_serial({mcu_label}): init_logging must be called before \
             attach_serial for a kalico-native MCU — the dedicated \
             mcu-*.jsonl writer cannot be installed without an events_dir. \
             All McuLog events would be silently discarded to the general \
             runtime_rx channel with no NDJSON output, which violates the \
             observability spec (§4, Decision C). Call init_logging first."
        ));
    }
    Ok(())
}

pub(crate) fn axis_ring_depth(total_pieces: u32, num_axes: u32) -> u32 {
    (total_pieces / num_axes.max(1)).max(1)
}

pub(crate) fn drip_cohort_participants(configs: &[McuAxisConfig]) -> Vec<crate::types::AxisKey> {
    configs
        .iter()
        .flat_map(|cfg| {
            cfg.axes.iter().map(move |&a| crate::types::AxisKey {
                mcu_id: cfg.mcu_id,
                axis: a as u8,
            })
        })
        .collect()
}

pub(crate) fn ring_depth_for_axis_inner(
    configs: &[crate::mcu_config::McuAxisConfig],
    mcu_handle: u32,
    axis: u8,
) -> Result<u16, String> {
    let cfg = configs
        .iter()
        .find(|c| c.mcu_id == mcu_handle)
        .ok_or_else(|| {
            format!(
                "ring_depth_for_axis: unknown mcu_handle {mcu_handle} \
                 (init_planner not yet called?)"
            )
        })?;
    let axis_usize = usize::from(axis);
    if !cfg.axes.contains(&axis_usize) {
        return Err(format!(
            "ring_depth_for_axis: axis {axis} is not configured on mcu_handle \
             {mcu_handle} (configured axes: {:?})",
            cfg.axes
        ));
    }
    let depth = axis_ring_depth(cfg.caps.total_pieces() as u32, cfg.axes.len() as u32);
    if depth > u32::from(u16::MAX) {
        return Err(format!(
            "ring depth {depth} exceeds u16::MAX (65535) for mcu {mcu_handle} axis {axis}; \
             a >65535-piece ring would need >2 MB of SRAM and is impossible here — \
             check total_piece_memory configuration"
        ));
    }
    #[allow(clippy::cast_possible_truncation)]
    Ok(depth as u16)
}
