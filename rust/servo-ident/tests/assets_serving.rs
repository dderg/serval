//! The embedded web bundle must actually be servable: `/` and every asset
//! `build.rs` embedded come back 200, non-empty, with the right MIME — a
//! broken frontend build pipeline fails here, not in a browser.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::time::Duration;

use servo_ident::{assets, http, serve};

struct HttpResult {
    status: u16,
    content_type: String,
    body: Vec<u8>,
}

fn get(port: u16, path: &str) -> HttpResult {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect");
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let req = format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n");
    stream.write_all(req.as_bytes()).unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let split = raw
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .expect("header/body split");
    let head = String::from_utf8_lossy(&raw[..split]).to_string();
    let body = raw[split + 4..].to_vec();
    let status: u16 = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .expect("status line");
    let content_type = head
        .lines()
        .find_map(|l| l.strip_prefix("Content-Type: "))
        .unwrap_or("")
        .to_string();
    HttpResult {
        status,
        content_type,
        body,
    }
}

fn spawn_server() -> u16 {
    let root = std::env::temp_dir().join(format!(
        "servo_cal_assets_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&root).unwrap();
    let listener = http::bind("127.0.0.1", 0).expect("bind ephemeral port");
    let port = listener.local_addr().unwrap().port();
    let captures_root: PathBuf = root;
    std::thread::spawn(move || {
        http::run(listener, move |req| serve::handle(&captures_root, req));
    });
    port
}

#[test]
fn root_serves_the_bundled_index() {
    let port = spawn_server();
    let resp = get(port, "/");
    assert_eq!(resp.status, 200);
    assert_eq!(resp.content_type, "text/html; charset=utf-8");
    let text = String::from_utf8(resp.body).expect("index.html is utf-8");
    assert!(text.contains("<!doctype html>"), "not an html document");
    assert!(text.contains("servo-cal"), "index.html lost its title");
}

#[test]
fn every_embedded_asset_serves_with_its_mime() {
    let port = spawn_server();
    assert!(
        !assets::BUILT_ASSETS.is_empty(),
        "build.rs embedded no assets"
    );
    for asset in assets::BUILT_ASSETS {
        let resp = get(port, &format!("/{}", asset.path));
        assert_eq!(resp.status, 200, "{}: status", asset.path);
        assert_eq!(resp.content_type, asset.mime, "{}: mime", asset.path);
        assert!(!resp.body.is_empty(), "{}: empty body", asset.path);
        assert_eq!(resp.body, asset.body, "{}: served bytes differ", asset.path);
    }
}

#[test]
fn index_references_only_embedded_assets() {
    let index = std::str::from_utf8(assets::index_html().body).expect("index.html utf-8");
    let mut referenced = Vec::new();
    for attr in ["src=\"", "href=\""] {
        for (pos, _) in index.match_indices(attr) {
            let rest = &index[pos + attr.len()..];
            let url = &rest[..rest.find('"').expect("closing quote")];
            if url.starts_with("http") || url.starts_with('#') {
                continue;
            }
            referenced.push(url.trim_start_matches("./").trim_start_matches('/'));
        }
    }
    let has_ext = |wanted: &str| {
        referenced
            .iter()
            .any(|u| u.rsplit_once('.').is_some_and(|(_, ext)| ext == wanted))
    };
    assert!(
        has_ext("js"),
        "index.html references no script bundle: {referenced:?}"
    );
    assert!(
        has_ext("css"),
        "index.html references no stylesheet: {referenced:?}"
    );
    for url in referenced {
        assert!(
            assets::built(url).is_some(),
            "index.html references {url:?} but it is not in BUILT_ASSETS"
        );
    }
}
