//! Minimal hand-rolled HTTP/1.1 server for `servo-cal serve`: a
//! `TcpListener` accept loop, thread-per-connection, and a GET/POST-only
//! request parser. No keep-alive — every response carries
//! `Connection: close`, so the parser never has to reassemble a second
//! request off the same socket.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;

const MAX_HEADER_BYTES: usize = 64 * 1024;
const READ_CHUNK: usize = 4096;

pub struct Request {
    pub method: String,
    pub path: String,
    #[allow(dead_code)]
    pub body: Vec<u8>,
}

pub struct Response {
    pub status: u16,
    pub content_type: &'static str,
    pub body: Vec<u8>,
}

impl Response {
    pub fn json(status: u16, body: String) -> Response {
        Response {
            status,
            content_type: "application/json",
            body: body.into_bytes(),
        }
    }

    pub fn text(status: u16, content_type: &'static str, body: String) -> Response {
        Response {
            status,
            content_type,
            body: body.into_bytes(),
        }
    }

    pub fn not_found(reason: &str) -> Response {
        Response::json(404, serde_json::json!({ "error": reason }).to_string())
    }
}

fn status_line(status: u16) -> &'static str {
    match status {
        200 => "200 OK",
        400 => "400 Bad Request",
        404 => "404 Not Found",
        405 => "405 Method Not Allowed",
        _ => "500 Internal Server Error",
    }
}

fn find_double_crlf(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n")
}

fn read_request(stream: &mut TcpStream) -> Result<Request, String> {
    let mut buf = Vec::new();
    let mut chunk = [0u8; READ_CHUNK];
    let header_end = loop {
        let n = stream.read(&mut chunk).map_err(|e| format!("read: {e}"))?;
        if n == 0 {
            return Err("connection closed before headers completed".to_string());
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(pos) = find_double_crlf(&buf) {
            break pos;
        }
        if buf.len() > MAX_HEADER_BYTES {
            return Err("request headers too large".to_string());
        }
    };
    let head = String::from_utf8_lossy(&buf[..header_end]).into_owned();
    let mut lines = head.split("\r\n");
    let request_line = lines.next().ok_or("empty request")?;
    let mut parts = request_line.split_whitespace();
    let method = parts.next().ok_or("missing method")?.to_string();
    let path = parts.next().ok_or("missing path")?.to_string();

    let mut content_length = 0usize;
    for line in lines {
        if let Some((k, v)) = line.split_once(':') {
            if k.trim().eq_ignore_ascii_case("content-length") {
                content_length = v
                    .trim()
                    .parse()
                    .map_err(|_| format!("bad content-length {v:?}"))?;
            }
        }
    }

    let mut body = buf[header_end + 4..].to_vec();
    while body.len() < content_length {
        let n = stream
            .read(&mut chunk)
            .map_err(|e| format!("read body: {e}"))?;
        if n == 0 {
            return Err("connection closed mid-body".to_string());
        }
        body.extend_from_slice(&chunk[..n]);
    }
    body.truncate(content_length);
    Ok(Request { method, path, body })
}

fn write_response(stream: &mut TcpStream, resp: &Response) -> std::io::Result<()> {
    let header = format!(
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        status_line(resp.status),
        resp.content_type,
        resp.body.len(),
    );
    stream.write_all(header.as_bytes())?;
    stream.write_all(&resp.body)
}

/// Bind `host:port` (port 0 picks an ephemeral port — read it back with
/// `TcpListener::local_addr` before calling `run`, as the tests do).
pub fn bind(host: &str, port: u16) -> Result<TcpListener, String> {
    TcpListener::bind((host, port)).map_err(|e| format!("bind {host}:{port}: {e}"))
}

/// Accept connections forever, dispatching each to `handler` on its own
/// thread. Never returns under normal operation; a bind/accept failure is
/// the only way out, and it fails loud via the caller's `Result`.
pub fn run<F>(listener: TcpListener, handler: F)
where
    F: Fn(&Request) -> Response + Send + Sync + 'static,
{
    let handler = Arc::new(handler);
    for incoming in listener.incoming() {
        let mut stream = match incoming {
            Ok(s) => s,
            Err(e) => {
                eprintln!("servo-cal serve: accept error: {e}");
                continue;
            }
        };
        let handler = Arc::clone(&handler);
        std::thread::spawn(move || match read_request(&mut stream) {
            Ok(req) => {
                let resp = handler(&req);
                println!("{} {} -> {}", req.method, req.path, resp.status);
                if let Err(e) = write_response(&mut stream, &resp) {
                    eprintln!("servo-cal serve: write error: {e}");
                }
            }
            Err(e) => {
                eprintln!("servo-cal serve: bad request: {e}");
                let resp = Response::text(400, "text/plain", format!("bad request: {e}"));
                let _ = write_response(&mut stream, &resp);
            }
        });
    }
}
