use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::SystemTime;

const BUN_HINT: &str =
    "the servo-ident web UI is built with bun at compile time; install bun: https://bun.sh";

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let web_dir = manifest_dir.join("web");
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=web/src");
    println!("cargo:rerun-if-changed=web/index.html");
    println!("cargo:rerun-if-changed=web/app.css");
    println!("cargo:rerun-if-changed=web/package.json");
    println!("cargo:rerun-if-changed=web/bun.lock");

    let bun = which_bun();
    ensure_node_modules(&bun, &web_dir);

    let dist_dir = out_dir.join("web-dist");
    if dist_dir.exists() {
        std::fs::remove_dir_all(&dist_dir).expect("clear stale web-dist in OUT_DIR");
    }
    run(
        Command::new(&bun)
            .args(["run", "build"])
            .env("SERVO_WEB_OUTDIR", &dist_dir)
            .current_dir(&web_dir),
        "bun run build",
    );

    let table = asset_table(&dist_dir);
    std::fs::write(out_dir.join("embedded_assets.rs"), table).expect("write embedded_assets.rs");
}

fn which_bun() -> PathBuf {
    let candidates = ["bun", "/opt/homebrew/bin/bun", "/usr/local/bin/bun"];
    for candidate in candidates {
        if Command::new(candidate)
            .arg("--version")
            .output()
            .is_ok_and(|out| out.status.success())
        {
            return PathBuf::from(candidate);
        }
    }
    panic!("bun not found on PATH or in /opt/homebrew/bin, /usr/local/bin; {BUN_HINT}");
}

fn ensure_node_modules(bun: &Path, web_dir: &Path) {
    let node_modules = web_dir.join("node_modules");
    let lockfile = web_dir.join("bun.lock");
    let install_needed = match mtime(&node_modules) {
        None => true,
        Some(modules_mtime) => {
            mtime(&lockfile).expect("web/bun.lock must exist (bun install writes it)")
                > modules_mtime
        }
    };
    if install_needed {
        run(
            Command::new(bun)
                .args(["install", "--frozen-lockfile"])
                .current_dir(web_dir),
            "bun install --frozen-lockfile",
        );
    }
}

fn mtime(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

fn run(command: &mut Command, label: &str) {
    let status = command
        .status()
        .unwrap_or_else(|e| panic!("failed to spawn {label}: {e}; {BUN_HINT}"));
    if !status.success() {
        panic!("{label} failed with {status}; {BUN_HINT}");
    }
}

fn mime_for(name: &str) -> &'static str {
    let extension = name.rsplit_once('.').map(|(_, ext)| ext);
    match extension {
        Some("html") => "text/html; charset=utf-8",
        Some("css") => "text/css",
        Some("js") => "application/javascript",
        Some("map") => "application/json",
        Some("svg") => "image/svg+xml",
        Some("png") => "image/png",
        Some("woff2") => "font/woff2",
        _ => panic!("built asset {name:?} has no known MIME type; extend mime_for in build.rs"),
    }
}

fn asset_table(dist_dir: &Path) -> String {
    let mut names: Vec<String> = std::fs::read_dir(dist_dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dist_dir.display()))
        .map(|entry| {
            let entry = entry.expect("read dist entry");
            assert!(
                entry.path().is_file(),
                "unexpected non-file in web dist: {}",
                entry.path().display()
            );
            entry.file_name().into_string().expect("utf-8 asset name")
        })
        .collect();
    names.sort();
    assert!(
        names.iter().any(|n| n == "index.html"),
        "web build produced no index.html in {}",
        dist_dir.display()
    );

    let mut rows = String::new();
    for name in &names {
        let path = dist_dir.join(name);
        rows.push_str(&format!(
            "    Asset {{ path: {name:?}, mime: {mime:?}, body: include_bytes!({fs_path:?}) }},\n",
            mime = mime_for(name),
            fs_path = path.to_str().expect("utf-8 OUT_DIR path"),
        ));
    }
    format!("pub const BUILT_ASSETS: &[Asset] = &[\n{rows}];\n")
}
