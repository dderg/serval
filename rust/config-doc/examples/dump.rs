//! Parity-harness dump: parse a config file and print every
//! section/option/value in a canonical line format for diffing against the
//! Python configparser pipeline.

fn main() {
    let path = std::env::args().nth(1).expect("usage: dump <config.cfg>");
    let data = config_doc::Document::parse(
        &std::fs::read_to_string(&path)
            .expect("readable input")
            .replace("\r\n", "\n"),
        &path,
    );
    let doc = match data {
        Ok(doc) => doc,
        Err(e) => {
            println!("ERROR: {e}");
            return;
        }
    };
    for section in doc.section_names() {
        for option in doc.section(section).expect("iterating").option_names() {
            match doc.get(section, option) {
                Ok((value, _refs)) => {
                    println!("{section}\x1f{option}\x1f{value:?}");
                }
                Err(e) => println!("{section}\x1f{option}\x1fERROR: {e}"),
            }
        }
    }
}
