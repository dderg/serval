//! The dashboard SPA sources, embedded from the `web/` bun package.
//! TODO(next phase): serve the bundled `web/dist/` output instead of the
//! raw TypeScript modules — until then `/js/*` serves the .ts sources
//! under their old .js names so the asset table and its tests keep working.

pub const INDEX_HTML: &str = include_str!("../web/index.html");
pub const APP_CSS: &str = include_str!("../web/app.css");

pub const JS_MODULES: &[(&str, &str)] = &[
    ("api.js", include_str!("../web/src/api.ts")),
    ("boot.js", include_str!("../web/src/boot.ts")),
    ("charts-core.js", include_str!("../web/src/charts-core.ts")),
    ("console.js", include_str!("../web/src/console.ts")),
    ("docs.js", include_str!("../web/src/docs.ts")),
    ("drive.js", include_str!("../web/src/drive.ts")),
    ("dynamics.js", include_str!("../web/src/dynamics.ts")),
    ("live.js", include_str!("../web/src/live.ts")),
    ("metrics.js", include_str!("../web/src/metrics.ts")),
    ("moonraker.js", include_str!("../web/src/moonraker.ts")),
    ("peaks.js", include_str!("../web/src/peaks.ts")),
    ("runs.js", include_str!("../web/src/runs.ts")),
    ("shell.js", include_str!("../web/src/shell.ts")),
    ("state.js", include_str!("../web/src/state.ts")),
    ("store.js", include_str!("../web/src/store.ts")),
    ("strain.js", include_str!("../web/src/strain.ts")),
    ("uplot-chart.js", include_str!("../web/src/uplot-chart.ts")),
];

pub fn js_module(name: &str) -> Option<&'static str> {
    JS_MODULES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, src)| *src)
}

/// Third-party libraries now come from npm via the `web/` package; nothing
/// is vendored anymore. Kept until the serve pipeline switches to `dist/`.
pub const VENDOR_ASSETS: &[(&str, &str, &str)] = &[];

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
