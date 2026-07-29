pub const CANBUS_ID_ADMIN: u32 = 0x3f0;
pub const CANBUS_ID_ADMIN_RESP: u32 = 0x3f1;
pub const CANBUS_ID_DATA_BASE: u32 = 0x100;
pub const CAN_MAX_DLEN: usize = 8;
pub const CANBUS_UUID_LEN: usize = 6;
pub const NODEID_FIRST: u8 = 0x40;
pub const NODEID_LAST: u8 = 0x7f;

const CANBUS_CMD_QUERY_UNASSIGNED: u8 = 0x00;
const CANBUS_CMD_SET_KLIPPER_NODEID: u8 = 0x01;
const CANBUS_RESP_NEED_NODEID: u8 = 0x20;
const CANBUS_APP_KLIPPER: u8 = 0x01;

pub fn tx_id(nodeid: u8) -> u32 {
    u32::from(nodeid) * 2 + CANBUS_ID_DATA_BASE
}

pub fn rx_id(nodeid: u8) -> u32 {
    tx_id(nodeid) + 1
}

pub fn query_unassigned_payload() -> [u8; 1] {
    [CANBUS_CMD_QUERY_UNASSIGNED]
}

pub fn set_nodeid_payload(uuid: u64, nodeid: u8) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = CANBUS_CMD_SET_KLIPPER_NODEID;
    out[1..1 + CANBUS_UUID_LEN].copy_from_slice(&uuid_bytes(uuid));
    out[7] = nodeid;
    out
}

pub fn uuid_bytes(uuid: u64) -> [u8; CANBUS_UUID_LEN] {
    let be = uuid.to_be_bytes();
    let mut out = [0u8; CANBUS_UUID_LEN];
    out.copy_from_slice(&be[8 - CANBUS_UUID_LEN..]);
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnassignedNode {
    pub uuid: u64,
    pub klipper_application: bool,
}

pub fn parse_unassigned_response(data: &[u8]) -> Option<UnassignedNode> {
    if data.len() < CAN_MAX_DLEN || data[0] != CANBUS_RESP_NEED_NODEID {
        return None;
    }
    let mut be = [0u8; 8];
    be[8 - CANBUS_UUID_LEN..].copy_from_slice(&data[1..1 + CANBUS_UUID_LEN]);
    Some(UnassignedNode {
        uuid: u64::from_be_bytes(be),
        klipper_application: data[7] == CANBUS_APP_KLIPPER,
    })
}

pub fn payload_chunks(bytes: &[u8]) -> impl Iterator<Item = &[u8]> {
    bytes.chunks(CAN_MAX_DLEN)
}

#[cfg(target_os = "linux")]
pub use linux::CanLink;

#[cfg(target_os = "linux")]
mod linux {
    use super::{
        CAN_MAX_DLEN, CANBUS_ID_ADMIN, CANBUS_ID_ADMIN_RESP, NODEID_FIRST,
        parse_unassigned_response, payload_chunks, query_unassigned_payload, rx_id,
        set_nodeid_payload, tx_id,
    };
    use std::collections::VecDeque;
    use std::io;
    use std::time::{Duration, Instant};

    const AF_CAN: libc::c_int = 29;
    const CAN_RAW: libc::c_int = 1;
    const SOL_CAN_RAW: libc::c_int = 100 + CAN_RAW;
    const CAN_RAW_FILTER: libc::c_int = 1;
    const SIOCGIFINDEX: libc::c_ulong = 0x8933;
    const CAN_SFF_MASK: u32 = 0x7ff;
    const IFREQ_NAME_LEN: usize = 16;
    const DISCOVERY_QUERY_INTERVAL: Duration = Duration::from_millis(100);

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct SockAddrCan {
        can_family: u16,
        can_ifindex: libc::c_int,
        rx_id: u32,
        tx_id: u32,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct IfReq {
        name: [u8; IFREQ_NAME_LEN],
        payload: [u8; 24],
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct CanFrame {
        can_id: u32,
        can_dlc: u8,
        pad: u8,
        res0: u8,
        res1: u8,
        data: [u8; CAN_MAX_DLEN],
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct CanFilter {
        can_id: u32,
        can_mask: u32,
    }

    fn last_error() -> io::Error {
        io::Error::last_os_error()
    }

    fn open_can_socket() -> io::Result<libc::c_int> {
        #[allow(unsafe_code)]
        let fd = unsafe { libc::socket(AF_CAN, libc::SOCK_RAW, CAN_RAW) };
        if fd < 0 {
            return Err(last_error());
        }
        Ok(fd)
    }

    fn if_index(fd: libc::c_int, interface: &str) -> io::Result<libc::c_int> {
        if interface.len() >= IFREQ_NAME_LEN {
            return Err(io::Error::other(format!(
                "canbus interface name too long: {interface}"
            )));
        }
        let mut req = IfReq {
            name: [0u8; IFREQ_NAME_LEN],
            payload: [0u8; 24],
        };
        req.name[..interface.len()].copy_from_slice(interface.as_bytes());
        #[allow(unsafe_code)]
        let rc = unsafe { libc::ioctl(fd, SIOCGIFINDEX, &raw mut req) };
        if rc < 0 {
            return Err(io::Error::new(
                last_error().kind(),
                format!("SIOCGIFINDEX({interface}): {}", last_error()),
            ));
        }
        Ok(libc::c_int::from_ne_bytes([
            req.payload[0],
            req.payload[1],
            req.payload[2],
            req.payload[3],
        ]))
    }

    fn bind_can(fd: libc::c_int, ifindex: libc::c_int) -> io::Result<()> {
        let addr = SockAddrCan {
            can_family: AF_CAN as u16,
            can_ifindex: ifindex,
            rx_id: 0,
            tx_id: 0,
        };
        #[allow(unsafe_code)]
        let rc = unsafe {
            libc::bind(
                fd,
                (&raw const addr).cast::<libc::sockaddr>(),
                std::mem::size_of::<SockAddrCan>() as libc::socklen_t,
            )
        };
        if rc < 0 {
            return Err(last_error());
        }
        Ok(())
    }

    fn set_filters(fd: libc::c_int, filters: &[CanFilter]) -> io::Result<()> {
        #[allow(unsafe_code)]
        let rc = unsafe {
            libc::setsockopt(
                fd,
                SOL_CAN_RAW,
                CAN_RAW_FILTER,
                filters.as_ptr().cast::<libc::c_void>(),
                std::mem::size_of_val(filters) as libc::socklen_t,
            )
        };
        if rc < 0 {
            return Err(last_error());
        }
        Ok(())
    }

    fn poll_readable(fd: libc::c_int, timeout: Duration) -> io::Result<bool> {
        let mut pfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };
        let millis = timeout.as_millis().min(i32::MAX as u128) as libc::c_int;
        #[allow(unsafe_code)]
        let rc = unsafe { libc::poll(&raw mut pfd, 1, millis) };
        if rc < 0 {
            let e = last_error();
            if e.kind() == io::ErrorKind::Interrupted {
                return Ok(false);
            }
            return Err(e);
        }
        Ok(rc > 0)
    }

    fn recv_frame(fd: libc::c_int) -> io::Result<CanFrame> {
        let mut frame = CanFrame {
            can_id: 0,
            can_dlc: 0,
            pad: 0,
            res0: 0,
            res1: 0,
            data: [0u8; CAN_MAX_DLEN],
        };
        #[allow(unsafe_code)]
        let n = unsafe {
            libc::read(
                fd,
                (&raw mut frame).cast::<libc::c_void>(),
                std::mem::size_of::<CanFrame>(),
            )
        };
        if n < 0 {
            return Err(last_error());
        }
        if n as usize != std::mem::size_of::<CanFrame>() {
            return Err(io::Error::other(format!(
                "short CAN frame read: {n} bytes, expected {}",
                std::mem::size_of::<CanFrame>()
            )));
        }
        Ok(frame)
    }

    fn send_frame(fd: libc::c_int, can_id: u32, payload: &[u8]) -> io::Result<()> {
        if payload.len() > CAN_MAX_DLEN {
            return Err(io::Error::other(format!(
                "classic CAN payload overflow: {} bytes",
                payload.len()
            )));
        }
        let mut frame = CanFrame {
            can_id,
            can_dlc: payload.len() as u8,
            pad: 0,
            res0: 0,
            res1: 0,
            data: [0u8; CAN_MAX_DLEN],
        };
        frame.data[..payload.len()].copy_from_slice(payload);
        #[allow(unsafe_code)]
        let n = unsafe {
            libc::write(
                fd,
                (&raw const frame).cast::<libc::c_void>(),
                std::mem::size_of::<CanFrame>(),
            )
        };
        if n < 0 {
            return Err(last_error());
        }
        if n as usize != std::mem::size_of::<CanFrame>() {
            return Err(io::Error::other(format!(
                "short CAN frame write: {n} bytes"
            )));
        }
        Ok(())
    }

    fn close_fd(fd: libc::c_int) {
        #[allow(unsafe_code)]
        unsafe {
            libc::close(fd);
        }
    }

    pub struct CanLink {
        fd: libc::c_int,
        tx_id: u32,
        rx_id: u32,
        nodeid: u8,
        timeout: Duration,
        pending: VecDeque<u8>,
    }

    impl CanLink {
        pub fn open(interface: &str, uuid: u64, discovery_timeout: Duration) -> io::Result<Self> {
            let fd = open_can_socket()?;
            let guard = FdGuard(fd);
            let ifindex = if_index(fd, interface)?;
            bind_can(fd, ifindex)?;
            set_filters(
                fd,
                &[CanFilter {
                    can_id: CANBUS_ID_ADMIN_RESP,
                    can_mask: CAN_SFF_MASK,
                }],
            )?;

            let nodeid = NODEID_FIRST;
            discover(fd, uuid, discovery_timeout)?;
            send_frame(fd, CANBUS_ID_ADMIN, &set_nodeid_payload(uuid, nodeid))?;

            let tx = tx_id(nodeid);
            let rx = rx_id(nodeid);
            set_filters(
                fd,
                &[
                    CanFilter {
                        can_id: CANBUS_ID_ADMIN_RESP,
                        can_mask: CAN_SFF_MASK,
                    },
                    CanFilter {
                        can_id: rx,
                        can_mask: CAN_SFF_MASK,
                    },
                ],
            )?;

            std::mem::forget(guard);
            Ok(Self {
                fd,
                tx_id: tx,
                rx_id: rx,
                nodeid,
                timeout: Duration::from_millis(100),
                pending: VecDeque::new(),
            })
        }

        pub fn nodeid(&self) -> u8 {
            self.nodeid
        }

        pub fn tx_id(&self) -> u32 {
            self.tx_id
        }

        pub fn rx_id(&self) -> u32 {
            self.rx_id
        }

        fn drain_pending(&mut self, buf: &mut [u8]) -> usize {
            let n = self.pending.len().min(buf.len());
            for slot in buf.iter_mut().take(n) {
                *slot = self.pending.pop_front().expect("pending byte");
            }
            n
        }
    }

    struct FdGuard(libc::c_int);

    impl Drop for FdGuard {
        fn drop(&mut self) {
            close_fd(self.0);
        }
    }

    fn discover(fd: libc::c_int, uuid: u64, discovery_timeout: Duration) -> io::Result<()> {
        let deadline = Instant::now() + discovery_timeout;
        loop {
            send_frame(fd, CANBUS_ID_ADMIN, &query_unassigned_payload())?;
            let window = Instant::now() + DISCOVERY_QUERY_INTERVAL;
            while Instant::now() < window {
                let remaining = window.saturating_duration_since(Instant::now());
                if !poll_readable(fd, remaining)? {
                    continue;
                }
                let frame = recv_frame(fd)?;
                if frame.can_id & CAN_SFF_MASK != CANBUS_ID_ADMIN_RESP {
                    continue;
                }
                let Some(node) = parse_unassigned_response(&frame.data[..]) else {
                    continue;
                };
                if node.uuid != uuid {
                    continue;
                }
                if !node.klipper_application {
                    return Err(io::Error::other(format!(
                        "canbus uuid {uuid:012x} answered with a non-Klipper application marker; \
                         node is running bootloader or foreign firmware"
                    )));
                }
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "canbus uuid {uuid:012x} did not answer discovery within {discovery_timeout:?}"
                    ),
                ));
            }
        }
    }

    impl crate::host_io::byte_link::ByteLink for CanLink {
        fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
            self.timeout = timeout;
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if buf.is_empty() {
                return Ok(0);
            }
            if !self.pending.is_empty() {
                return Ok(self.drain_pending(buf));
            }
            let deadline = Instant::now() + self.timeout;
            loop {
                let remaining = deadline.saturating_duration_since(Instant::now());
                if !poll_readable(self.fd, remaining)? {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "canbus read timed out",
                        ));
                    }
                    continue;
                }
                let frame = recv_frame(self.fd)?;
                if frame.can_id & CAN_SFF_MASK != self.rx_id {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "canbus read timed out",
                        ));
                    }
                    continue;
                }
                let len = usize::from(frame.can_dlc).min(CAN_MAX_DLEN);
                let copied = len.min(buf.len());
                buf[..copied].copy_from_slice(&frame.data[..copied]);
                self.pending.extend(&frame.data[copied..len]);
                return Ok(copied);
            }
        }

        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let mut sent = 0usize;
            for chunk in payload_chunks(buf) {
                match send_frame(self.fd, self.tx_id, chunk) {
                    Ok(()) => sent += chunk.len(),
                    Err(e) => {
                        if sent > 0 {
                            return Ok(sent);
                        }
                        return Err(e);
                    }
                }
            }
            Ok(sent)
        }

        fn out_queue(&self) -> io::Result<Option<u32>> {
            Ok(None)
        }
    }

    impl Drop for CanLink {
        fn drop(&mut self) {
            close_fd(self.fd);
        }
    }
}

#[cfg(test)]
mod tests;
