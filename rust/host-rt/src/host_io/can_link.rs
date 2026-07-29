pub const CANBUS_ID_ADMIN: u32 = 0x3f0;
pub const CANBUS_ID_ADMIN_RESP: u32 = 0x3f1;
pub const CANBUS_ID_DATA_BASE: u32 = 0x100;
pub const CAN_MAX_DLEN: usize = 8;
pub const CANBUS_UUID_LEN: usize = 6;
pub const NODEID_FIRST: u8 = 0x40;
pub const NODEID_LAST: u8 = 0x7f;

const CANBUS_CMD_QUERY_UNASSIGNED: u8 = 0x00;
const CANBUS_CMD_QUERY_EXTENDED: u8 = 0x01;
const CANBUS_CMD_SET_KLIPPER_NODEID: u8 = 0x01;
const CANBUS_RESP_NEED_NODEID: u8 = 0x20;
const CANBUS_RESP_HAVE_NODEID: u8 = 0x21;
const CANBUS_APP_KLIPPER: u8 = 0x01;
const CANBUS_APP_KALICO: u8 = 0x07;

pub fn tx_id(nodeid: u8) -> u32 {
    u32::from(nodeid) * 2 + CANBUS_ID_DATA_BASE
}

pub fn rx_id(nodeid: u8) -> u32 {
    tx_id(nodeid) + 1
}

pub fn query_extended_payload() -> [u8; 2] {
    [CANBUS_CMD_QUERY_UNASSIGNED, CANBUS_CMD_QUERY_EXTENDED]
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
pub enum NodeAssignment {
    Unassigned,
    AlreadyAssigned(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiscoveredNode {
    pub uuid: u64,
    pub assignment: NodeAssignment,
}

pub fn parse_query_response(data: &[u8]) -> Option<DiscoveredNode> {
    if data.len() < CAN_MAX_DLEN {
        return None;
    }
    let assignment = match data[0] {
        CANBUS_RESP_NEED_NODEID => {
            if data[7] != CANBUS_APP_KLIPPER && data[7] != CANBUS_APP_KALICO {
                return None;
            }
            NodeAssignment::Unassigned
        }
        CANBUS_RESP_HAVE_NODEID => NodeAssignment::AlreadyAssigned(data[7]),
        _ => return None,
    };
    let mut be = [0u8; 8];
    be[8 - CANBUS_UUID_LEN..].copy_from_slice(&data[1..1 + CANBUS_UUID_LEN]);
    Some(DiscoveredNode {
        uuid: u64::from_be_bytes(be),
        assignment,
    })
}

pub const CAN_FRAME_SIZE: usize = 16;
pub const CANFD_FRAME_SIZE: usize = 72;
pub const CANFD_MAX_DLEN: usize = 64;

const CANFD_BRS_FLAG: u8 = 0x01;
const CANFD_PAYLOAD_SIZES: [usize; 7] = [64, 48, 32, 24, 20, 16, 12];
const CANFD_FLAGS_OFFSET: usize = 5;
const CAN_DATA_OFFSET: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameFormat {
    Classic,
    Fd,
}

impl FrameFormat {
    fn frame_size(self) -> usize {
        match self {
            Self::Classic => CAN_FRAME_SIZE,
            Self::Fd => CANFD_FRAME_SIZE,
        }
    }

    fn max_dlen(self) -> usize {
        match self {
            Self::Classic => CAN_MAX_DLEN,
            Self::Fd => CANFD_MAX_DLEN,
        }
    }
}

pub fn from_mtu(mtu: usize) -> std::io::Result<FrameFormat> {
    match mtu {
        CAN_FRAME_SIZE => Ok(FrameFormat::Classic),
        CANFD_FRAME_SIZE => Ok(FrameFormat::Fd),
        other => Err(std::io::Error::other(format!(
            "canbus interface MTU {other} is neither classic CAN ({CAN_FRAME_SIZE}) nor CAN-FD \
             ({CANFD_FRAME_SIZE})"
        ))),
    }
}

pub fn fd_chunk_len(available: usize) -> usize {
    if available <= CAN_MAX_DLEN {
        return available;
    }
    for size in CANFD_PAYLOAD_SIZES {
        if available >= size {
            return size;
        }
    }
    CAN_MAX_DLEN
}

pub fn chunk_len(format: FrameFormat, available: usize) -> usize {
    match format {
        FrameFormat::Classic => available.min(CAN_MAX_DLEN),
        FrameFormat::Fd => fd_chunk_len(available),
    }
}

#[derive(Debug)]
pub struct EncodedFrame {
    bytes: [u8; CANFD_FRAME_SIZE],
    format: FrameFormat,
}

impl EncodedFrame {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.format.frame_size()]
    }

    pub fn format(&self) -> FrameFormat {
        self.format
    }
}

pub fn encode_frame(can_id: u32, payload: &[u8]) -> std::io::Result<EncodedFrame> {
    if payload.len() > CANFD_MAX_DLEN {
        return Err(std::io::Error::other(format!(
            "CAN payload overflow: {} bytes exceeds the {CANFD_MAX_DLEN}-byte FD limit",
            payload.len()
        )));
    }
    let format = if payload.len() > CAN_MAX_DLEN {
        FrameFormat::Fd
    } else {
        FrameFormat::Classic
    };
    let mut bytes = [0u8; CANFD_FRAME_SIZE];
    bytes[..4].copy_from_slice(&can_id.to_ne_bytes());
    bytes[4] = payload.len() as u8;
    if format == FrameFormat::Fd {
        bytes[CANFD_FLAGS_OFFSET] = CANFD_BRS_FLAG;
    }
    bytes[CAN_DATA_OFFSET..CAN_DATA_OFFSET + payload.len()].copy_from_slice(payload);
    Ok(EncodedFrame { bytes, format })
}

pub fn decode_frame(datagram: &[u8]) -> std::io::Result<(u32, &[u8])> {
    let format = match datagram.len() {
        CAN_FRAME_SIZE => FrameFormat::Classic,
        CANFD_FRAME_SIZE => FrameFormat::Fd,
        other => {
            return Err(std::io::Error::other(format!(
                "CAN datagram length {other} is neither a classic frame ({CAN_FRAME_SIZE}) nor an \
                 FD frame ({CANFD_FRAME_SIZE})"
            )));
        }
    };
    let can_id = u32::from_ne_bytes([datagram[0], datagram[1], datagram[2], datagram[3]]);
    let len = usize::from(datagram[4]);
    if len > format.max_dlen() {
        return Err(std::io::Error::other(format!(
            "CAN frame declares {len} payload bytes, above the {} the datagram can hold",
            format.max_dlen()
        )));
    }
    Ok((can_id, &datagram[CAN_DATA_OFFSET..CAN_DATA_OFFSET + len]))
}

#[cfg(target_os = "linux")]
pub use linux::CanLink;

#[cfg(target_os = "linux")]
mod linux {
    use super::{
        CANBUS_ID_ADMIN, CANBUS_ID_ADMIN_RESP, CANFD_FRAME_SIZE, FrameFormat, NODEID_FIRST,
        NODEID_LAST, NodeAssignment, chunk_len, decode_frame, encode_frame, from_mtu,
        parse_query_response, query_extended_payload, rx_id, set_nodeid_payload, tx_id,
    };
    use std::collections::{HashMap, VecDeque};
    use std::io;
    use std::sync::Mutex;
    use std::time::{Duration, Instant};

    static ASSIGNED_NODEIDS: Mutex<Option<HashMap<(String, u64), u8>>> = Mutex::new(None);

    /// One node id per (interface, uuid), stable across re-attach. A second node
    /// on the same bus must not be handed an id another node already answers on:
    /// the incumbent would see a foreign uuid claim its id and shut down.
    fn allocate_nodeid(interface: &str, uuid: u64) -> io::Result<u8> {
        let mut guard = ASSIGNED_NODEIDS
            .lock()
            .map_err(|_| io::Error::other("canbus nodeid registry poisoned"))?;
        let registry = guard.get_or_insert_with(HashMap::new);
        let key = (interface.to_owned(), uuid);
        if let Some(existing) = registry.get(&key) {
            return Ok(*existing);
        }
        let taken: Vec<u8> = registry
            .iter()
            .filter(|((iface, _), _)| iface == interface)
            .map(|(_, id)| *id)
            .collect();
        let free = (NODEID_FIRST..=NODEID_LAST).find(|id| !taken.contains(id));
        let Some(nodeid) = free else {
            return Err(io::Error::other(format!(
                "canbus {interface}: no free node id in {NODEID_FIRST:#x}..={NODEID_LAST:#x} \
                 for uuid {uuid:012x}"
            )));
        };
        registry.insert(key, nodeid);
        Ok(nodeid)
    }

    const AF_CAN: libc::c_int = 29;
    const CAN_RAW: libc::c_int = 1;
    const SOL_CAN_RAW: libc::c_int = 100 + CAN_RAW;
    const CAN_RAW_FILTER: libc::c_int = 1;
    const CAN_RAW_FD_FRAMES: libc::c_int = 5;
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

    fn interface_mtu(interface: &str) -> io::Result<usize> {
        let path = format!("/sys/class/net/{interface}/mtu");
        let raw = std::fs::read_to_string(&path)
            .map_err(|e| io::Error::new(e.kind(), format!("read {path}: {e}")))?;
        raw.trim()
            .parse::<usize>()
            .map_err(|e| io::Error::other(format!("parse {path} ({:?}): {e}", raw.trim())))
    }

    fn enable_fd_frames(fd: libc::c_int) -> io::Result<()> {
        let enable: libc::c_int = 1;
        #[allow(unsafe_code)]
        let rc = unsafe {
            libc::setsockopt(
                fd,
                SOL_CAN_RAW,
                CAN_RAW_FD_FRAMES,
                (&raw const enable).cast::<libc::c_void>(),
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        if rc < 0 {
            return Err(io::Error::new(
                last_error().kind(),
                format!("CAN_RAW_FD_FRAMES: {}", last_error()),
            ));
        }
        Ok(())
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

    struct Datagram {
        bytes: [u8; CANFD_FRAME_SIZE],
        len: usize,
    }

    impl Datagram {
        fn as_bytes(&self) -> &[u8] {
            &self.bytes[..self.len]
        }
    }

    fn recv_datagram(fd: libc::c_int) -> io::Result<Datagram> {
        let mut bytes = [0u8; CANFD_FRAME_SIZE];
        #[allow(unsafe_code)]
        let n = unsafe {
            libc::read(
                fd,
                bytes.as_mut_ptr().cast::<libc::c_void>(),
                CANFD_FRAME_SIZE,
            )
        };
        if n < 0 {
            return Err(last_error());
        }
        Ok(Datagram {
            bytes,
            len: n as usize,
        })
    }

    fn send_frame(fd: libc::c_int, can_id: u32, payload: &[u8]) -> io::Result<()> {
        let frame = encode_frame(can_id, payload)?;
        let wire = frame.as_bytes();
        #[allow(unsafe_code)]
        let n = unsafe { libc::write(fd, wire.as_ptr().cast::<libc::c_void>(), wire.len()) };
        if n < 0 {
            return Err(last_error());
        }
        if n as usize != wire.len() {
            return Err(io::Error::other(format!(
                "short CAN frame write: {n} bytes, expected {}",
                wire.len()
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
        format: FrameFormat,
        interface_format: FrameFormat,
        interface: String,
        timeout: Duration,
        pending: VecDeque<u8>,
    }

    impl CanLink {
        pub fn open(interface: &str, uuid: u64, discovery_timeout: Duration) -> io::Result<Self> {
            let fd = open_can_socket()?;
            let guard = FdGuard(fd);
            let ifindex = if_index(fd, interface)?;
            let mtu = interface_mtu(interface)?;
            let interface_format = from_mtu(mtu)?;
            let format = FrameFormat::Classic;
            tracing::info!(
                subsystem = "mcu-comms",
                event = "canbus_frame_mode",
                interface,
                mtu,
                interface_fd_capable = interface_format == FrameFormat::Fd,
                "canbus opened classic; FD awaits the mcu advertising CANBUS_DATA_FREQUENCY"
            );
            bind_can(fd, ifindex)?;
            set_filters(
                fd,
                &[CanFilter {
                    can_id: CANBUS_ID_ADMIN_RESP,
                    can_mask: CAN_SFF_MASK,
                }],
            )?;

            let nodeid = allocate_nodeid(interface, uuid)?;
            let prior_assignment = discover(fd, uuid, discovery_timeout)?;
            tracing::info!(
                subsystem = "mcu-comms",
                event = "canbus_discovered",
                interface,
                uuid = format!("{uuid:012x}"),
                prior = ?prior_assignment,
                nodeid,
                "canbus node answered admin query; assigning nodeid"
            );
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
                format,
                interface_format,
                interface: interface.to_owned(),
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

        pub fn frame_format(&self) -> FrameFormat {
            self.format
        }

        pub fn try_enable_fd(&mut self, mcu_data_rate_hz: u32) -> io::Result<bool> {
            if self.format == FrameFormat::Fd {
                return Ok(true);
            }
            let mcu_capable = mcu_data_rate_hz > 0;
            let interface_capable = self.interface_format == FrameFormat::Fd;
            if !mcu_capable || !interface_capable {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "canbus_fd_declined",
                    interface = %self.interface,
                    mcu_data_rate_hz,
                    interface_capable,
                    "staying on classic CAN framing"
                );
                return Ok(false);
            }
            enable_fd_frames(self.fd)?;
            self.format = FrameFormat::Fd;
            tracing::info!(
                subsystem = "mcu-comms",
                event = "canbus_fd_enabled",
                interface = %self.interface,
                mcu_data_rate_hz,
                "mcu advertises a CAN-FD data phase; switching to 64-byte framing"
            );
            Ok(true)
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

    fn discover(
        fd: libc::c_int,
        uuid: u64,
        discovery_timeout: Duration,
    ) -> io::Result<NodeAssignment> {
        let deadline = Instant::now() + discovery_timeout;
        loop {
            send_frame(fd, CANBUS_ID_ADMIN, &query_extended_payload())?;
            let window = Instant::now() + DISCOVERY_QUERY_INTERVAL;
            while Instant::now() < window {
                let remaining = window.saturating_duration_since(Instant::now());
                if !poll_readable(fd, remaining)? {
                    continue;
                }
                let datagram = recv_datagram(fd)?;
                let (can_id, payload) = decode_frame(datagram.as_bytes())?;
                if can_id & CAN_SFF_MASK != CANBUS_ID_ADMIN_RESP {
                    continue;
                }
                let Some(node) = parse_query_response(payload) else {
                    continue;
                };
                if node.uuid != uuid {
                    continue;
                }
                return Ok(node.assignment);
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "canbus uuid {uuid:012x} did not answer an extended admin query within \
                         {discovery_timeout:?}: node absent, powered down, or running firmware \
                         without the extended-query admin command"
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
                let datagram = recv_datagram(self.fd)?;
                let (can_id, payload) = decode_frame(datagram.as_bytes())?;
                if can_id & CAN_SFF_MASK != self.rx_id {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "canbus read timed out",
                        ));
                    }
                    continue;
                }
                let copied = payload.len().min(buf.len());
                buf[..copied].copy_from_slice(&payload[..copied]);
                self.pending.extend(&payload[copied..]);
                return Ok(copied);
            }
        }

        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let mut sent = 0usize;
            while sent < buf.len() {
                let remainder = &buf[sent..];
                let take = chunk_len(self.format, remainder.len());
                match send_frame(self.fd, self.tx_id, &remainder[..take]) {
                    Ok(()) => sent += take,
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

        fn try_enable_fd(&mut self, mcu_data_rate_hz: u32) -> io::Result<bool> {
            CanLink::try_enable_fd(self, mcu_data_rate_hz)
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
