//! The dashboard SPA, bundled by `build.rs` (bun builds `web/` into
//! `OUT_DIR`) and embedded here — the served bytes are always the built
//! output of the checked-in sources, never a stale `dist/`.

pub struct Asset {
    pub path: &'static str,
    pub mime: &'static str,
    pub body: &'static [u8],
}

include!(concat!(env!("OUT_DIR"), "/embedded_assets.rs"));

pub fn built(path: &str) -> Option<&'static Asset> {
    BUILT_ASSETS.iter().find(|a| a.path == path)
}

pub fn index_html() -> &'static Asset {
    built("index.html").expect("build.rs asserts index.html is in the bundle")
}
