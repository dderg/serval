use super::{
    Arc, DataDictionary, EngineEvent, MsgProtoParser, NotifyId, PassthroughEntry, Py, PyAny,
    PyDict, PyMotionEngine, PyResult, PyRuntimeError, Python, cq_id_from_raw, mcu_handle_from_raw,
    pymethods, router_err, stats_to_pydict,
};
use pyo3::prelude::*;

#[pymethods]
impl PyMotionEngine {
    fn alloc_command_queue(&self, handle: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let qid = router
            .alloc_command_queue(mcu_handle_from_raw(handle))
            .map_err(router_err)?;
        Ok(qid.raw())
    }
    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_send(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<()> {
        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, NotifyId::none());
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .push(mcu_handle_from_raw(mcu), cq_id_from_raw(queue), entry)
            .map_err(router_err)?;
        Ok(())
    }
    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_query(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<u64> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let events_ref = Arc::clone(&self.events);
        let mcu_raw = mcu;

        let nid = router
            .register_notify(
                mcu_h,
                Box::new(move |resp| {
                    let ev = EngineEvent {
                        kind: "query_response".to_owned(),
                        mcu: mcu_raw,
                        notify_id: 0,
                        response_bytes: resp.bytes,
                        sent_time: resp.sent_time,
                        receive_time: resp.receive_time,
                    };
                    events_ref
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push_back(ev);
                }),
            )
            .map_err(router_err)?;

        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, nid);
        router
            .push(mcu_h, cq_id_from_raw(queue), entry)
            .map_err(router_err)?;

        Ok(nid.raw())
    }
    #[pyo3(signature = (_mcu, _queue, _data, _timeout))]
    fn passthrough_send_wait_ack(
        &self,
        _mcu: u32,
        _queue: u32,
        _data: &[u8],
        _timeout: f64,
    ) -> PyResult<Vec<u8>> {
        Err(pyo3::exceptions::PyNotImplementedError::new_err(
            "send_wait_ack requires reactor thread — deferred to Phase 2",
        ))
    }
    #[pyo3(signature = (mcu, name, oid, callback))]
    fn passthrough_register_handler(
        &self,
        mcu: u32,
        name: &str,
        oid: u32,
        callback: Py<PyAny>,
    ) -> PyResult<()> {
        self.handlers
            .lock()
            .unwrap()
            .insert((mcu, name.to_owned(), oid), callback);
        Ok(())
    }
    fn passthrough_register_flush_callback(&self, mcu: u32, callback: Py<PyAny>) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let cb: Box<dyn Fn() + Send> = Box::new(move || {
            Python::attach(|py| {
                if let Err(e) = callback.call0(py) {
                    e.print(py);
                }
            });
        });

        router
            .register_flush_callback(mcu_h, cb)
            .map_err(router_err)?;
        Ok(())
    }
    fn poll_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let mut events = self.events.lock().unwrap_or_else(|p| p.into_inner());
        match events.pop_front() {
            Some(ev) => Ok(Some(ev.to_pydict(py)?)),
            None => Ok(None),
        }
    }
    fn add_config_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_config_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }
    fn add_init_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_init_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }
    fn add_restart_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_restart_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }
    fn begin_config_phase(&self, mcu: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .begin_config_phase(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }
    fn next_config_entry(&self, mcu: u32) -> PyResult<Option<Vec<u8>>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .next_config_entry(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }
    fn get_stats(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let stats = router
            .get_stats(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;
        stats_to_pydict(py, &stats)
    }
    fn set_msgproto_dict(&self, dict_json: &[u8]) -> PyResult<()> {
        let json_str = std::str::from_utf8(dict_json)
            .map_err(|e| PyRuntimeError::new_err(format!("dict_json utf8: {e}")))?;
        let dict: DataDictionary = serde_json::from_str(json_str)
            .map_err(|e| PyRuntimeError::new_err(format!("dict json parse: {e}")))?;
        let parser = MsgProtoParser::from_dictionary(dict)
            .map_err(|e| PyRuntimeError::new_err(format!("parser build: {e:?}")))?;
        *self.parser.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(parser));
        Ok(())
    }
    fn get_identify_data(&self, mcu_handle: u32) -> PyResult<Vec<u8>> {
        let io = self.host_io_for_mcu("get_identify_data", mcu_handle)?;
        Ok(io.raw_identify_bytes().to_vec())
    }
    #[pyo3(signature = (mcu_handle, msg, response, timeout_s = 5.0))]
    fn engine_call(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        msg: &str,
        response: &str,
        timeout_s: f64,
    ) -> PyResult<Py<PyDict>> {
        use std::time::Duration;

        let io = self.host_io_for_mcu("engine_call", mcu_handle)?;

        let msg_owned = msg.to_owned();
        let response_owned = response.to_owned();
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(
                &msg_owned,
                &response_owned,
                Duration::from_secs_f64(timeout_s),
            )
            .map_err(|e| PyRuntimeError::new_err(format!("engine_call: {e}")))
        })?;

        let d = PyDict::new(py);
        for (k, v) in &params.fields {
            use host_rt::transport::MessageValue;
            match v {
                MessageValue::U32(n) => d.set_item(k, n)?,
                MessageValue::I32(n) => d.set_item(k, n)?,
                MessageValue::U64(n) => d.set_item(k, n)?,
                MessageValue::Bytes(b) => {
                    d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                }
                MessageValue::String(s) => d.set_item(k, s)?,
            }
        }
        if params.sent_time_raw != 0.0 {
            d.set_item("#sent_time_raw", params.sent_time_raw)?;
            d.set_item("#receive_time_raw", params.recv_time_raw)?;
        }
        Ok(d.unbind())
    }
    fn take_runtime_event(&self, py: Python<'_>, mcu_handle: u32) -> PyResult<Option<Py<PyDict>>> {
        use host_rt::host_io::runtime_events::RuntimeEvent;

        let event = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "take_runtime_event: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            let mut taken = None;
            for lane in [&mut conn.runtime_rx_priority, &mut conn.runtime_rx_bulk] {
                if let Some(rx) = lane.as_mut() {
                    if let Ok(ev) = rx.try_recv() {
                        taken = Some(ev);
                        break;
                    }
                }
            }
            match taken {
                Some(ev) => ev,
                None => return Ok(None),
            }
        };

        let d = PyDict::new(py);
        match event {
            RuntimeEvent::Status(s) => {
                d.set_item("type", "status")?;
                d.set_item("engine_status", s.engine_status)?;
                d.set_item("queue_depth", s.queue_depth)?;
                d.set_item("current_segment_id", s.current_segment_id)?;
                d.set_item("last_fault", s.last_fault)?;
                d.set_item("fault_detail", s.fault_detail)?;
                d.set_item("retired_through_segment_id", s.retired_through_segment_id)?;
            }
            RuntimeEvent::CreditFreed(c) => {
                d.set_item("type", "credit_freed")?;
                d.set_item("retired_through_segment_id", c.retired_through_segment_id)?;
                d.set_item("free_slots", c.free_slots)?;
            }
            RuntimeEvent::Fault(f) => {
                d.set_item("type", "fault")?;
                d.set_item("fault_code", f.fault_code)?;
                d.set_item("fault_detail", f.fault_detail)?;
                d.set_item("segment_id", f.segment_id)?;
                d.set_item("synthesized", f.synthesized)?;
            }
            RuntimeEvent::Trace(_) => {
                return Ok(None);
            }
            RuntimeEvent::Heartbeat { .. } => {
                return Ok(None);
            }
            RuntimeEvent::EndstopTrip(t) => {
                d.set_item("type", "endstop_trip")?;
                d.set_item("endstop_id", t.endstop_id)?;
                d.set_item("trip_clock", t.trip_clock)?;
                self.handle_endstop_trip(mcu_handle, t.endstop_id, t.trip_clock);
            }
            RuntimeEvent::UnknownOutput { format, msg } => {
                d.set_item("type", "output")?;
                d.set_item("format", format)?;
                d.set_item("msg", msg)?;
            }
            RuntimeEvent::PassthroughResponse { name, params } => {
                d.set_item("type", "response")?;
                d.set_item("name", name)?;
                for (k, v) in &params.fields {
                    use host_rt::transport::MessageValue;
                    match v {
                        MessageValue::U32(n) => d.set_item(k, *n)?,
                        MessageValue::I32(n) => d.set_item(k, *n)?,
                        MessageValue::U64(n) => d.set_item(k, *n)?,
                        MessageValue::Bytes(b) => {
                            d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                        }
                        MessageValue::String(s) => d.set_item(k, s)?,
                    }
                }
                if params.sent_time_raw != 0.0 {
                    d.set_item("#sent_time_raw", params.sent_time_raw)?;
                    d.set_item("#receive_time_raw", params.recv_time_raw)?;
                }
            }
            RuntimeEvent::McuLog(_) => {
                return Ok(None);
            }
        }
        Ok(Some(d.unbind()))
    }
    fn engine_get_clock_async(&self, mcu_handle: u32) -> PyResult<()> {
        let io = self.host_io_for_mcu("engine_get_clock_async", mcu_handle)?;

        io.get_clock_async().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("engine_get_clock_async: {e}"))
        })
    }
    #[pyo3(signature = (mcu_handle, msg))]
    fn engine_send(&self, mcu_handle: u32, msg: &str) -> PyResult<()> {
        let io = self.host_io_for_mcu("engine_send", mcu_handle)?;
        io.send_fire_and_forget(msg)
            .map_err(|e| PyRuntimeError::new_err(format!("engine_send: {e}")))
    }
    fn engine_mark_expected_disconnect(&self, mcu_handle: u32) -> PyResult<()> {
        let io = self.host_io_for_mcu("engine_mark_expected_disconnect", mcu_handle)?;
        io.mark_expected_disconnect()
            .map_err(|e| PyRuntimeError::new_err(format!("engine_mark_expected_disconnect: {e}")))
    }
    #[pyo3(signature = (mcu, freq, offset, last_clock, host_now_raw))]
    fn set_clock_est(
        &self,
        _py: Python<'_>,
        mcu: u32,
        freq: f64,
        offset: f64,
        last_clock: u64,
        host_now_raw: f64,
    ) -> PyResult<()> {
        self.clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq);

        use std::sync::atomic::{AtomicUsize, Ordering as AOrd};
        static SET_CLOCK_EST_CALLS: AtomicUsize = AtomicUsize::new(0);
        let call_n = SET_CLOCK_EST_CALLS.fetch_add(1, AOrd::Relaxed);
        if call_n < 5 || call_n % 100 == 0 {
            tracing::debug!(
                subsystem = "engine",
                event = "set_clock_est",
                call_n,
                mcu,
                freq = freq as u64,
                offset,
                last_clock,
                "[engine-trace] set_clock_est"
            );
        }
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .set_clock_est_rebased(
                mcu_handle_from_raw(mcu),
                freq,
                offset,
                last_clock,
                host_now_raw,
            )
            .map_err(router_err)?;
        Ok(())
    }
    #[pyo3(signature = (mcu, freq_hz))]
    fn set_nominal_clock_freq(&self, mcu: u32, freq_hz: u32) -> PyResult<()> {
        if freq_hz == 0 {
            return Err(PyRuntimeError::new_err(
                "set_nominal_clock_freq: freq_hz must be nonzero",
            ));
        }
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq_hz);
        Ok(())
    }
    fn extract_old(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let (sent, received) = router
            .extract_old(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;

        let d = PyDict::new(py);

        let sent_list: Vec<Py<PyDict>> = sent
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        let received_list: Vec<Py<PyDict>> = received
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        d.set_item("sent", sent_list)?;
        d.set_item("received", received_list)?;
        Ok(d.unbind())
    }
}
