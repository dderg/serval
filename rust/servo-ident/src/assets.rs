//! The dashboard SPA, embedded so `servo-cal serve` ships as one self
//! contained binary — no build step, no CDN, no framework.

pub const INDEX_HTML: &str = include_str!("web/index.html");
pub const APP_JS: &str = include_str!("web/app.js");
pub const APP_CSS: &str = include_str!("web/app.css");
