use crate::mcu_config::McuAxisConfig;

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
