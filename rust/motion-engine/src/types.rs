use host_rt::passthrough_queue::McuHandle;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

pub(crate) fn mcu_handle_from_raw(raw: u32) -> McuHandle {
    McuHandle::from_raw(raw)
}
