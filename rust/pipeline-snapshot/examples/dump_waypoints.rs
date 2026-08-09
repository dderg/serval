// Prints the parsed waypoints of a gcode file as JSON rows, for parity checks
// against snapshots/harness.py::parse_gcode.
//
//   cargo run -p pipeline-snapshot --example dump_waypoints -- <file.gcode> <max_velocity> <max_accel>

use pipeline_snapshot::waypoints::parse_gcode;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: dump_waypoints <file.gcode> <max_velocity>");
    let max_velocity: f64 = args
        .next()
        .expect("usage: dump_waypoints <file.gcode> <max_velocity> <max_accel>")
        .parse()
        .expect("max_velocity must be a number");
    let max_accel: f64 = args
        .next()
        .expect("usage: dump_waypoints <file.gcode> <max_velocity> <max_accel>")
        .parse()
        .expect("max_accel must be a number");
    let text = std::fs::read_to_string(&path).expect("cannot read gcode file");
    match parse_gcode(&text, max_velocity, max_accel) {
        Ok(wp) => {
            for (x, y, z, e, f, a) in wp {
                println!("[{x:?},{y:?},{z:?},{e:?},{f:?},{a:?}]");
            }
        }
        Err(e) => {
            eprintln!("parse error: {e}");
            std::process::exit(1);
        }
    }
}
