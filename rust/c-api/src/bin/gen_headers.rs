fn canonicalize_untagged_enums(src: &str) -> String {
    let mut paragraphs: Vec<&str> = src.split("\n\n").collect();

    let enum_positions: Vec<usize> = (0..paragraphs.len())
        .filter(|&i| fieldless_enum_name(paragraphs[i]).is_some())
        .collect();

    if enum_positions.len() < 2 {
        return paragraphs.join("\n\n");
    }

    let mut enum_paragraphs: Vec<&str> = enum_positions.iter().map(|&i| paragraphs[i]).collect();
    enum_paragraphs.sort_by_key(|p| fieldless_enum_name(p).unwrap());

    for (slot, paragraph) in enum_positions.iter().zip(enum_paragraphs) {
        paragraphs[*slot] = paragraph;
    }

    paragraphs.join("\n\n")
}

fn fieldless_enum_name(paragraph: &str) -> Option<String> {
    let mut lines = paragraph.lines();

    let first_non_empty = loop {
        match lines.next() {
            None => return None,
            Some(line) => {
                let trimmed = line.trim();
                if trimmed.is_empty() {
                    continue;
                }
                if trimmed.starts_with("/**") || trimmed.starts_with("/*") {
                    loop {
                        match lines.next() {
                            None => return None,
                            Some(comment_line) => {
                                if comment_line.contains("*/") {
                                    break;
                                }
                            }
                        }
                    }
                    continue;
                }
                break trimmed;
            }
        }
    };

    if !first_non_empty.starts_with("enum ") || !first_non_empty.ends_with(" {") {
        return None;
    }

    let after_enum = &first_non_empty["enum ".len()..];
    let name = after_enum.strip_suffix(" {")?;

    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return None;
    }

    let expected_typedef = format!("typedef uint8_t {name};");
    let has_typedef = paragraph
        .lines()
        .any(|l| l.trim() == expected_typedef.as_str());

    if has_typedef {
        Some(name.to_owned())
    } else {
        None
    }
}

fn write_canonical(bindings: cbindgen::Bindings, out_path: &str) {
    let mut buf: Vec<u8> = Vec::new();
    bindings.write(&mut buf);
    let raw = String::from_utf8(buf).expect("cbindgen output is UTF-8");
    let canonical = canonicalize_untagged_enums(&raw);
    std::fs::write(out_path, canonical).expect("write header");
}

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    if !cfg!(feature = "header-runtime") {
        eprintln!("error: invoke with --features header-runtime");
        std::process::exit(1);
    }
    let cfg = cbindgen::Config::from_file(format!("{crate_dir}/cbindgen-runtime.toml"))
        .expect("cbindgen-runtime.toml should be parseable");
    let bindings = cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(cfg)
        .generate()
        .expect("runtime.h generation failed");
    write_canonical(bindings, &format!("{crate_dir}/include/runtime.h"));
    println!("Generated runtime.h");
}

#[cfg(test)]
#[path = "gen_headers/tests.rs"]
mod tests;
