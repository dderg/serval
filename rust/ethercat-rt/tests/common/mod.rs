use std::thread;
use std::time::{Duration, Instant};

use host_rt::mcu_serial_conn::McuSerialConn;

pub fn connect_until(path: &str, deadline: Instant) -> McuSerialConn {
    loop {
        match McuSerialConn::connect(path) {
            Ok(connection) => return connection,
            Err(_) if Instant::now() < deadline => thread::sleep(Duration::from_millis(10)),
            Err(error) => panic!("connect to {path} failed: {error}"),
        }
    }
}
