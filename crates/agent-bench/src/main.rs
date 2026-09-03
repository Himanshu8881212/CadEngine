//! Runner for the CAD Code agent-surface ruler: evaluates the seed criteria against
//! the JSON surface, prints a red/green table, and writes the versioned scorecard to
//! `data/bench/scorecard.json` (run from the repo root, e.g. `cargo run -p agent-bench`).

use agent_bench::{run_all, score, scorecard_json};

fn main() {
	let criteria = run_all();
	let s = score(&criteria);

	println!("CAD Code — agent-surface ruler (seed)\n");
	for c in &criteria {
		println!("  [{}] {:<11} {:<24} {}", if c.passed { "PASS" } else { "  · " }, c.dim.name(), c.id, c.desc);
	}
	println!("\n  per dimension:");
	for (dim, p, t) in &s.per_dim {
		println!("    {:<12} {}/{}", dim, p, t);
	}
	println!(
		"\n  agent surface {:.1}/10  ({}/{} criteria)   kernel 9.0 (frozen)   composite {:.2}/10",
		s.agent_surface, s.passed, s.total, s.composite
	);

	let card = scorecard_json(&criteria, &s);
	let out = "data/bench/scorecard.json";
	match std::fs::write(out, serde_json::to_string_pretty(&card).unwrap()) {
		Ok(()) => println!("\n  wrote {out}"),
		Err(e) => eprintln!("\n  could not write {out} (run from the repo root): {e}"),
	}
}
