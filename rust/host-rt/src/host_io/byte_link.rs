use std::io;
use std::time::Duration;

pub trait ByteLink: Send {
    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write(&mut self, buf: &[u8]) -> io::Result<usize>;
    /// Kernel-side unsent byte count; None when the link cannot observe it.
    fn out_queue(&self) -> io::Result<Option<u32>>;
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
}
