use std::io;
use std::time::Duration;

pub trait ByteLink: Send {
    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write(&mut self, buf: &[u8]) -> io::Result<usize>;
    fn out_queue(&self) -> io::Result<Option<u32>>;
    fn wire_time(&self, bytes: usize) -> io::Result<Option<Duration>>;
    fn configure_wire_rates(&mut self, nominal_rate_hz: u32, data_rate_hz: u32) {
        let _ = (nominal_rate_hz, data_rate_hz);
    }
    fn try_enable_fd(&mut self, mcu_data_rate_hz: u32) -> io::Result<bool> {
        let _ = mcu_data_rate_hz;
        Ok(false)
    }
}

impl<T: serialport::SerialPort + ?Sized> ByteLink for Box<T> {
    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        serialport::SerialPort::set_timeout(&mut **self, timeout)
            .map_err(|e| io::Error::other(e.to_string()))
    }

    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        io::Read::read(&mut **self, buf)
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        io::Write::write(&mut **self, buf)
    }

    fn out_queue(&self) -> io::Result<Option<u32>> {
        serialport::SerialPort::bytes_to_write(&**self)
            .map(Some)
            .map_err(|e| io::Error::other(e.to_string()))
    }

    fn wire_time(&self, bytes: usize) -> io::Result<Option<Duration>> {
        let baud = serialport::SerialPort::baud_rate(&**self)
            .map_err(|e| io::Error::other(e.to_string()))?;
        if baud == 0 {
            return Ok(None);
        }
        Ok(Some(Duration::from_secs_f64(
            bytes as f64 * 10.0 / f64::from(baud),
        )))
    }
}
