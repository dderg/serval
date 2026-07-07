use std::env;
use std::fs;
use std::process;

use motion_core::seam_test_harness::{default_stream_config, run_schedule};

fn axis_name(axis: u8) -> &'static str {
    match axis {
        0 => "X",
        1 => "Y",
        2 => "Z",
        _ => "?",
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: repro_junction <in.gcode>");
        process::exit(1);
    }
    let in_path = &args[1];
    let source = fs::read_to_string(in_path).unwrap_or_else(|e| {
        eprintln!("cannot read {in_path}: {e}");
        process::exit(1);
    });

    let report = run_schedule(&source, default_stream_config());

    for b in report.boundaries.iter().filter(|b| b.is_fatal()).take(20) {
        println!(
            "FATAL {} |Δ|={:.5}mm  prev={:.5} next={:.5}  t={:.6} lines {}->{}",
            axis_name(b.axis),
            b.delta_mm,
            b.prev_pos,
            b.next_pos,
            b.next_host_t,
            b.prev_source_line,
            b.next_source_line
        );
    }
    println!(
        "\nmoves={} segments={}  FATAL(>=0.1)={} worst=|Δ|={:.5}mm",
        report.moves,
        report.segments,
        report.fatal(),
        report.worst()
    );
}
