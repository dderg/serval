//! Usage: ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rotation-distance F] [--rt-cpu N] [--rt-prio N]
//!        [--velocity-ff] [--dynamics-profile PATH] [--torque-clamp-pct F]

fn main() {
    let args = ethercat_rt::cli::Args::parse();
    let mut ctx = ethercat_rt::endpoint::bringup(args);
    ethercat_rt::endpoint::run(&mut ctx);
}
