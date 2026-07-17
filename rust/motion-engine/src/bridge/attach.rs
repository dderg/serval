use super::{
    Arc, McuHostIo, PyMotionEngine, PyResult, PyRuntimeError, mcu_handle_from_raw,
    query_runtime_caps, require_events_dir_for_mcu_transport,
};
use crate::lock_ext::LockExt;

impl PyMotionEngine {
    pub(super) fn try_reuse_existing_connection(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        klippy_non_critical: bool,
        expect_native: bool,
    ) -> PyResult<bool> {
        let existing_io: Option<Arc<McuHostIo>> = {
            let mcus = self.mcus.lock_ok();
            mcus.get(&mcu_handle)
                .and_then(|conn| conn.host_io.as_ref().map(Arc::clone))
        };
        if let Some(io) = existing_io {
            if io.is_alive() {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "attach_reuse_connection",
                    serial_path,
                    "attach_serial: reusing existing connection (reactor alive, skipping close/reopen)"
                );

                let (rx_priority, rx_bulk) = io.take_runtime_event_subscription().map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "attach_serial: runtime_event re-subscribe: {e:?}"
                    ))
                })?;

                let (mcu_transport_supported, identify_caps) = if !expect_native {
                    tracing::info!(
                        subsystem = "mcu-comms",
                        event = "attach_identify_skipped_reuse",
                        serial_path,
                        "attach_serial: kalico identify skipped on reuse (plugin-attached peripheral, not declared via an [mcu] section)"
                    );
                    (false, 0u64)
                } else {
                    match io.kalico_identify(std::time::Duration::from_secs(5)) {
                        Ok(out) => {
                            tracing::info!(
                                subsystem = "mcu-comms",
                                event = "attach_reidentified",
                                serial_path,
                                reset_epoch = out.reset_epoch,
                                capabilities = out.capabilities,
                                "attach_serial: kalico re-identified (reset_epoch/caps as hex)"
                            );
                            (true, out.capabilities)
                        }
                        Err(e) => {
                            tracing::warn!(
                                subsystem = "mcu-comms",
                                event = "attach_identify_timeout_reuse",
                                serial_path,
                                error = %e,
                                "attach_serial: kalico_identify timed out on reuse; treating as Klipper-protocol-only"
                            );
                            (false, 0u64)
                        }
                    }
                };

                let runtime_caps = if mcu_transport_supported {
                    match query_runtime_caps(&io, std::time::Duration::from_secs(2)) {
                        Ok(caps) => {
                            tracing::debug!(
                                subsystem = "mcu-comms",
                                event = "attach_runtime_caps_reuse",
                                serial_path,
                                total_piece_memory = caps.total_piece_memory,
                                "[caps-trace] attach_serial reuse: runtime caps"
                            );
                            Some(caps)
                        }
                        Err(e) => {
                            return Err(PyRuntimeError::new_err(format!(
                                "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                                 ({e}) — a kalico-native MCU must report runtime caps; \
                                 firmware is too old, mismatched, or not flashed. \
                                 Refusing to attach with guessed caps."
                            )));
                        }
                    }
                } else {
                    None
                };

                let critical = mcu_transport_supported && !klippy_non_critical;
                io.set_critical(critical);
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "attach_criticality_reuse",
                    serial_path,
                    critical,
                    mcu_transport = mcu_transport_supported,
                    klippy_non_critical,
                    "attach_serial: reuse — criticality set"
                );

                self.with_mcu(
                    mcu_handle,
                    |h| format!("attach_serial: unknown mcu_handle {h}"),
                    |conn| {
                        conn.runtime_rx_priority = Some(rx_priority);
                        conn.runtime_rx_bulk = Some(rx_bulk);
                        conn.runtime_caps = runtime_caps;
                        conn.identify_caps = identify_caps;
                        conn.mcu_transport_supported = mcu_transport_supported;
                        Ok(())
                    },
                )?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub(super) fn register_freshly_attached_mcu(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        mcu_label: &str,
        klippy_non_critical: bool,
        expect_native: bool,
        host_io: McuHostIo,
    ) -> PyResult<()> {
        let (rx_priority, rx_bulk, mcu_transport_supported, identify_caps, runtime_caps) =
            identify_and_query_caps(&host_io, serial_path, expect_native)?;

        let critical = mcu_transport_supported && !klippy_non_critical;
        host_io.set_critical(critical);
        tracing::info!(
            subsystem = "mcu-comms",
            event = "attach_criticality",
            serial_path,
            critical,
            mcu_transport = mcu_transport_supported,
            klippy_non_critical,
            "attach_serial: criticality set"
        );

        let host_io_arc = Arc::new(host_io);

        self.wire_mcu_log_hook(mcu_transport_supported, mcu_handle, mcu_label, &host_io_arc)?;

        self.with_mcu(
            mcu_handle,
            |h| format!("attach_serial: unknown mcu_handle {h}"),
            |conn| {
                conn.host_io = Some(host_io_arc);
                conn.runtime_rx_priority = Some(rx_priority);
                conn.runtime_rx_bulk = Some(rx_bulk);
                conn.runtime_caps = runtime_caps;
                conn.identify_caps = identify_caps;
                conn.mcu_transport_supported = mcu_transport_supported;
                Ok(())
            },
        )
    }

    fn wire_mcu_log_hook(
        &self,
        mcu_transport_supported: bool,
        mcu_handle: u32,
        mcu_label: &str,
        host_io_arc: &Arc<McuHostIo>,
    ) -> PyResult<()> {
        {
            let events_dir_guard = self.events_dir.lock_ok();
            require_events_dir_for_mcu_transport(
                mcu_transport_supported,
                events_dir_guard.as_deref(),
                mcu_label,
            )
            .map_err(PyRuntimeError::new_err)?;
        }

        if mcu_transport_supported {
            let events_dir_guard = self.events_dir.lock_ok();
            if let Some(ref dir) = *events_dir_guard {
                use crate::logging::writer::{
                    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL, RotatingJsonlWriter,
                };
                let source = mcu_label.to_owned();
                let jsonl_path = dir.join(format!("{source}.jsonl"));
                match RotatingJsonlWriter::new(
                    &jsonl_path,
                    DEFAULT_MAX_BYTES,
                    DEFAULT_BACKUP_COUNT,
                    FSYNC_INTERVAL,
                ) {
                    Ok(writer) => {
                        let sink = crate::mcu_log::spawn_jsonl_writer_thread(writer, &source);
                        let mcu_h = mcu_handle_from_raw(mcu_handle);
                        let hook = crate::mcu_log::build_mcu_log_hook(
                            Arc::clone(&self.router),
                            mcu_h,
                            sink,
                            source,
                        );
                        host_io_arc.set_mcu_log_hook(Box::new(hook));
                    }
                    Err(e) => {
                        tracing::warn!(
                            subsystem = "mcu-comms",
                            event = "attach_mcu_log_open_failed",
                            jsonl_path = %jsonl_path.display(),
                            error = %e,
                            "attach_serial: mcu-log: failed to open jsonl writer"
                        );
                    }
                }
            } else {
                unreachable!(
                    "attach_serial: events_dir is None for a kalico-native MCU \
                     — require_events_dir_for_mcu_transport should have \
                     rejected this call before reaching hook wiring"
                );
            }
        }
        Ok(())
    }
}

#[allow(clippy::type_complexity)]
fn identify_and_query_caps(
    host_io: &McuHostIo,
    serial_path: &str,
    expect_native: bool,
) -> PyResult<(
    std::sync::mpsc::Receiver<host_rt::host_io::runtime_events::RuntimeEvent>,
    std::sync::mpsc::Receiver<host_rt::host_io::runtime_events::RuntimeEvent>,
    bool,
    u64,
    Option<mcu_protocol::messages::RuntimeCapsResponse>,
)> {
    let (rx_priority, rx_bulk) = host_io.take_runtime_event_subscription().map_err(|e| {
        PyRuntimeError::new_err(format!("attach_serial: runtime_event subscribe: {e:?}"))
    })?;

    let (mcu_transport_supported, identify_caps) = if !expect_native {
        tracing::info!(
            subsystem = "mcu-comms",
            event = "attach_identify_skipped",
            serial_path,
            "attach_serial: kalico identify skipped (plugin-attached peripheral, not declared via an [mcu] section)"
        );
        (false, 0u64)
    } else {
        match host_io.kalico_identify(std::time::Duration::from_secs(5)) {
            Ok(out) => {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "attach_identified",
                    serial_path,
                    reset_epoch = out.reset_epoch,
                    capabilities = out.capabilities,
                    "attach_serial: kalico identified (reset_epoch/caps as hex)"
                );
                (true, out.capabilities)
            }
            Err(e) => {
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "attach_identify_timeout",
                    serial_path,
                    error = %e,
                    "attach_serial: kalico_identify timed out; continuing attach as a Klipper-protocol-only MCU"
                );
                (false, 0u64)
            }
        }
    };

    let runtime_caps = if mcu_transport_supported {
        match query_runtime_caps(host_io, std::time::Duration::from_secs(2)) {
            Ok(caps) => {
                tracing::debug!(
                    subsystem = "mcu-comms",
                    event = "attach_runtime_caps",
                    serial_path,
                    total_piece_memory = caps.total_piece_memory,
                    "[caps-trace] attach_serial: runtime caps"
                );
                Some(caps)
            }
            Err(e) => {
                return Err(PyRuntimeError::new_err(format!(
                    "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                     ({e}) — a kalico-native MCU must report runtime caps; \
                     firmware is too old, mismatched, or not flashed. \
                     Refusing to attach with guessed caps."
                )));
            }
        }
    } else {
        None
    };

    Ok((
        rx_priority,
        rx_bulk,
        mcu_transport_supported,
        identify_caps,
        runtime_caps,
    ))
}
