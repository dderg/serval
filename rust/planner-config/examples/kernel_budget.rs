//! Print each smoothing kernel's corner-deviation share at a given accel —
//! the number `validate_corner_budget` checks a config's corner_deviation
//! against. Usage: kernel_budget <max_accel> <type> <param>=<value>...

fn main() {
    let mut args = std::env::args().skip(1);
    let accel: f64 = args.next().expect("accel").parse().expect("accel f64");
    let ty = args.next().expect("post_processor type");
    let params: Vec<(String, f64)> = args
        .map(|kv| {
            let (k, v) = kv.split_once('=').expect("param as key=value");
            (k.to_owned(), v.parse().expect("param value f64"))
        })
        .collect();

    let axes = ["x", "y", "z"]
        .map(|name| planner_config::AxisDecl {
            name: name.into(),
            follows: vec![],
            motors: vec![],
            post_processors: if name == "x" {
                vec!["k".into()]
            } else {
                vec![]
            },
        })
        .to_vec();
    let registry = planner_config::AxisRegistry::try_new(axes).expect("registry");
    let decls = vec![planner_config::PostProcessorDecl {
        name: "k".into(),
        ty,
        params,
    }];
    let set = planner_config::PostProcessorSet::try_new(&registry, &decls).expect("set");
    let chains = set.compile(&registry).expect("compile");
    for chain in &chains.chains {
        let dev = geometry::kernel_corner_deviation_mm(chain.kernel_variance_s2(), accel);
        println!("kernel_deviation_mm={dev:.6}");
    }
}
