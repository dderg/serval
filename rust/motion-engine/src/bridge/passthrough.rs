use super::{
    Arc, DataDictionary, MsgProtoParser, Py, PyDict, PyMotionEngine, PyResult, PyRuntimeError,
    Python, mcu_handle_from_raw, pymethods, router_err,
};
use crate::lock_ext::LockExt;
use host_rt::host_io::parser::ArgValue;
use host_rt::transport::MessageParams;
use pyo3::prelude::*;

/// Command argument as klippy passes it: int (covers bool), str (enum/pin
/// names), or bytes/bytearray/list-of-int (buffer params).
#[derive(FromPyObject)]
pub(crate) enum PyArgValue {
    Int(i64),
    Str(String),
    Bytes(Vec<u8>),
}

impl From<PyArgValue> for ArgValue {
    fn from(v: PyArgValue) -> Self {
        match v {
            PyArgValue::Int(i) => ArgValue::Int(i),
            PyArgValue::Str(s) => ArgValue::Str(s),
            PyArgValue::Bytes(b) => ArgValue::Bytes(b),
        }
    }
}

fn owned_args(args: Vec<(String, PyArgValue)>) -> Vec<(String, ArgValue)> {
    args.into_iter().map(|(k, v)| (k, v.into())).collect()
}

fn params_to_pydict(py: Python<'_>, params: &MessageParams) -> PyResult<Py<PyDict>> {
    let d = PyDict::new(py);
    for (k, v) in &params.fields {
        use host_rt::transport::MessageValue;
        match v {
            MessageValue::U32(n) => d.set_item(k, n)?,
            MessageValue::I32(n) => d.set_item(k, n)?,
            MessageValue::U64(n) => d.set_item(k, n)?,
            MessageValue::Bytes(b) => d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?,
            MessageValue::String(s) => d.set_item(k, s)?,
        }
    }
    if params.sent_time_raw != 0.0 {
        d.set_item("#sent_time_raw", params.sent_time_raw)?;
        d.set_item("#receive_time_raw", params.recv_time_raw)?;
    }
    Ok(d.unbind())
}

#[pymethods]
impl PyMotionEngine {
    fn set_msgproto_dict(&self, dict_json: &[u8]) -> PyResult<()> {
        let json_str = std::str::from_utf8(dict_json)
            .map_err(|e| PyRuntimeError::new_err(format!("dict_json utf8: {e}")))?;
        let dict: DataDictionary = serde_json::from_str(json_str)
            .map_err(|e| PyRuntimeError::new_err(format!("dict json parse: {e}")))?;
        let parser = MsgProtoParser::from_dictionary(dict)
            .map_err(|e| PyRuntimeError::new_err(format!("parser build: {e:?}")))?;
        *self.parser.lock_ok() = Some(Arc::new(parser));
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

        params_to_pydict(py, &params)
    }
    #[pyo3(signature = (mcu_handle, name, args, response, timeout_s = 5.0))]
    fn engine_call_args(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        name: &str,
        args: Vec<(String, PyArgValue)>,
        response: &str,
        timeout_s: f64,
    ) -> PyResult<Py<PyDict>> {
        use std::time::Duration;

        let io = self.host_io_for_mcu("engine_call_args", mcu_handle)?;
        let name_owned = name.to_owned();
        let response_owned = response.to_owned();
        let args = owned_args(args);
        let params = py.detach(|| -> PyResult<_> {
            io.call_args(
                &name_owned,
                &args,
                &response_owned,
                Duration::from_secs_f64(timeout_s),
            )
            .map_err(|e| PyRuntimeError::new_err(format!("engine_call_args: {e}")))
        })?;

        params_to_pydict(py, &params)
    }
    #[pyo3(signature = (mcu_handle, name, args))]
    fn engine_send_args(
        &self,
        mcu_handle: u32,
        name: &str,
        args: Vec<(String, PyArgValue)>,
    ) -> PyResult<()> {
        let io = self.host_io_for_mcu("engine_send_args", mcu_handle)?;
        io.send_args(name, &owned_args(args))
            .map_err(|e| PyRuntimeError::new_err(format!("engine_send_args: {e}")))
    }
    fn take_runtime_event(&self, py: Python<'_>, mcu_handle: u32) -> PyResult<Option<Py<PyDict>>> {
        use host_rt::host_io::runtime_events::RuntimeEvent;

        let event = {
            let mut mcus = self.mcus.lock_ok();
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
                let fields = params_to_pydict(py, &params)?;
                d.update(fields.bind(py).as_mapping())?;
                d.set_item("type", "response")?;
                d.set_item("name", name)?;
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
        self.clock_freqs.lock_ok().insert(mcu, freq);

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
        let mut router = self.router.lock_ok();
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
        self.nominal_clock_freqs.lock_ok().insert(mcu, freq_hz);
        Ok(())
    }
}
