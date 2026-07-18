//! The dashboard SPA, embedded so `servo-cal serve` ships as one self
//! contained binary — no build step, no CDN, no framework.

pub const INDEX_HTML: &str = include_str!("web/index.html");
pub const APP_CSS: &str = include_str!("web/app.css");

pub const JS_MODULES: &[(&str, &str)] = &[
    ("api.js", include_str!("web/js/api.js")),
    ("boot.js", include_str!("web/js/boot.js")),
    ("charts-core.js", include_str!("web/js/charts-core.js")),
    ("console.js", include_str!("web/js/console.js")),
    ("docs.js", include_str!("web/js/docs.js")),
    ("drive.js", include_str!("web/js/drive.js")),
    ("dynamics.js", include_str!("web/js/dynamics.js")),
    ("live.js", include_str!("web/js/live.js")),
    ("metrics.js", include_str!("web/js/metrics.js")),
    ("moonraker.js", include_str!("web/js/moonraker.js")),
    ("peaks.js", include_str!("web/js/peaks.js")),
    ("runs.js", include_str!("web/js/runs.js")),
    ("shell.js", include_str!("web/js/shell.js")),
    ("state.js", include_str!("web/js/state.js")),
    ("strain.js", include_str!("web/js/strain.js")),
];

pub fn js_module(name: &str) -> Option<&'static str> {
    JS_MODULES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, src)| *src)
}

/// Third-party libraries staged for the SPA but not yet imported by it —
/// see `web/vendor/VERSIONS.md` for what each file is and where it came
/// from. `(file name, MIME type, source)`.
pub const VENDOR_ASSETS: &[(&str, &str, &str)] = &[
    (
        "htm-preact-standalone-3.1.1.mjs",
        "application/javascript",
        include_str!("web/vendor/htm-preact-standalone-3.1.1.mjs"),
    ),
    (
        "uPlot-1.6.32.esm.js",
        "application/javascript",
        include_str!("web/vendor/uPlot-1.6.32.esm.js"),
    ),
    (
        "uPlot-1.6.32.min.css",
        "text/css",
        include_str!("web/vendor/uPlot-1.6.32.min.css"),
    ),
];

pub fn vendor_asset(name: &str) -> Option<(&'static str, &'static str)> {
    VENDOR_ASSETS
        .iter()
        .find(|(n, _, _)| *n == name)
        .map(|(_, content_type, src)| (*content_type, *src))
}

/// Every JS module concatenated, for tests that grep the served sources
/// for required function declarations.
pub fn all_js() -> String {
    JS_MODULES
        .iter()
        .map(|(_, src)| *src)
        .collect::<Vec<_>>()
        .join("\n")
}
