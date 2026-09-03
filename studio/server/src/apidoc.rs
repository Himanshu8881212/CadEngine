// Copyright (c) LMCAD. Licensed under the MIT License.

//! Backend of the `describe_api` harness tool: the live op catalogue (a real
//! in-process `describe` program run) plus per-op documentation sections
//! extracted from the repo-root `API.md`.
//!
//! This closes the blind-agent live-fire P0 gap (PROGRESS.md): the kernel's
//! `describe` op only answers existence, and `API.md` is the only parameter
//! source — so the harness serves BOTH from one tool. Extraction is written
//! against API.md's real heading format: each op is documented under a
//! level-3 heading whose FIRST backticked token is the op name, e.g.
//! ``### `box` `` or ``### `implicit` — nestable expression trees (…)``.

use std::path::Path;
use std::sync::OnceLock;

use serde_json::Value;

/// The op catalogue `(count, names)`, resolved by actually executing a
/// `describe` program through the kernel executor — the same path the
/// model's own programs take, so the list cannot drift from what runs.
/// Cached process-wide after the first successful run (`scratch_dir` is the
/// executor's out-dir; `describe` writes nothing there).
pub fn op_catalogue(scratch_dir: &Path) -> Result<(u64, Vec<String>), String> {
	static CATALOGUE: OnceLock<(u64, Vec<String>)> = OnceLock::new();
	if let Some(c) = CATALOGUE.get() {
		return Ok(c.clone());
	}
	let report = kernel_api::run_program(r#"{"ops": [{"id": "catalogue", "op": "describe"}]}"#, scratch_dir);
	let measures = report
		.ops
		.first()
		.and_then(|op| op.measures.as_ref())
		.ok_or_else(|| "describe produced no measures — kernel executor misbehaving".to_string())?;
	let count = measures
		.get("count")
		.and_then(Value::as_u64)
		.ok_or_else(|| "describe measures carry no 'count'".to_string())?;
	let ops: Vec<String> = measures
		.get("ops")
		.and_then(Value::as_array)
		.ok_or_else(|| "describe measures carry no 'ops' list".to_string())?
		.iter()
		.filter_map(Value::as_str)
		.map(str::to_string)
		.collect();
	Ok(CATALOGUE.get_or_init(|| (count, ops)).clone())
}

/// Extract the `API.md` documentation section for `op`, or `None` when the
/// op has no section. `None` is a real answer, not an error: API.md documents
/// most but not all of the op surface (the `asm_*` family and the density-grid
/// samplers are prose-only as of 2026-08), and the tool layer turns `None`
/// into the "probe params carefully" note. Which ops are undocumented is not
/// pinned here — `tools/audit_docs.py` reports the live list.
///
/// A section starts at a level-3 heading whose first backticked token equals
/// `op` (trailing prose after the token is allowed) and runs until the next
/// heading of depth ≤ 3 — so `####` sub-headings (e.g. `implicit`'s tree
/// grammar) stay inside. Fenced code blocks are tracked so a `#` line inside
/// ```` ``` ```` never starts or ends a section. Exact-token matching means
/// `o_ring_face_gland` never matches `o_ring_face_gland_racetrack`.
pub fn extract_section(api_md: &str, op: &str) -> Option<String> {
	let mut lines: Vec<&str> = Vec::new();
	let mut in_section = false;
	let mut in_fence = false;
	for line in api_md.lines() {
		if line.trim_start().starts_with("```") {
			in_fence = !in_fence;
			if in_section {
				lines.push(line);
			}
			continue;
		}
		if !in_fence && line.starts_with('#') {
			let depth = line.chars().take_while(|&c| c == '#').count();
			if depth <= 3 {
				if in_section {
					break; // next section — done
				}
				if depth == 3 && heading_names_op(line, op) {
					in_section = true;
					lines.push(line);
				}
				continue;
			}
		}
		if in_section {
			lines.push(line);
		}
	}
	if lines.is_empty() {
		None
	} else {
		Some(lines.join("\n").trim_end().to_string())
	}
}

/// True iff a `###` heading line's FIRST backticked token is exactly `op`.
fn heading_names_op(line: &str, op: &str) -> bool {
	let rest = line.trim_start_matches('#').trim_start();
	let Some(start) = rest.find('`') else { return false };
	let rest = &rest[start + 1..];
	let Some(end) = rest.find('`') else { return false };
	&rest[..end] == op
}

#[cfg(test)]
mod tests {
	use super::*;

	/// The real repo-root API.md (two levels above this crate's manifest).
	fn api_md() -> String {
		let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.parent()
			.unwrap()
			.parent()
			.unwrap()
			.to_path_buf();
		std::fs::read_to_string(root.join("API.md")).expect("repo-root API.md exists")
	}

	/// Extraction against the REAL API.md: documented ops return their own
	/// section (heading + param table) without bleeding into the next one,
	/// prefix-named ops don't cross-match, `####` sub-headings stay inside
	/// their parent section, and an op with no section honestly returns `None`
	/// (the tool layer turns that into the "probe params carefully" note).
	///
	/// UPDATED 2026-07-30: `support_report`, `list_faces`, `list_edges` and
	/// `coincident_fit` used to be pinned here as known-undocumented — this
	/// test was the record of that gap. The doc auditor
	/// (`tools/audit_docs.py`) flagged them as absent from a reference that
	/// claims to cover all 160 ops, and they were written with executed
	/// examples, so the pin flips to asserting they are PRESENT.
	///
	/// UPDATED 2026-08-08: the `None` branch used to be pinned to ONE named op
	/// (`clearance`) chosen as "the honest known-missing case". That made the
	/// test a hostage of the docs: writing the missing section — which is
	/// exactly the improvement the auditor asks for — broke the test, and it
	/// did (`clearance` gained a section this pass). The name was never the
	/// contract. The contract is the BICONDITIONAL, so it is now asserted over
	/// the whole live op surface: `extract_section` returns a section starting
	/// with that op's heading iff API.md carries one, for every op `describe`
	/// serves. The expectation side is computed by an independent oracle (a
	/// plain `"### \`op\`"` line-prefix scan, deliberately not reusing this
	/// module's fence/depth machinery), so the two disagree the moment
	/// extraction breaks — and the `None` branch stays exercised by whatever is
	/// genuinely undocumented, without naming it.
	#[test]
	fn extracts_real_sections_and_reports_missing_ones_honestly() {
		let md = api_md();
		let boxs = extract_section(&md, "box").unwrap_or_default();
		let tear = extract_section(&md, "teardrop_hole").unwrap_or_default();
		let gland = extract_section(&md, "o_ring_face_gland").unwrap_or_default();
		let implicit = extract_section(&md, "implicit").unwrap_or_default();
		assert!(
			boxs.starts_with("### `box`")
				&& boxs.contains("low corner")
				&& !boxs.contains("### `cylinder`")
				&& tear.starts_with("### `teardrop_hole`")
				&& tear.contains("`through`")
				&& tear.contains("```json")
				&& !tear.contains("### `board_mount`")
				&& gland.starts_with("### `o_ring_face_gland`")
				&& !gland.contains("### `o_ring_face_gland_racetrack`")
				&& implicit.starts_with("### `implicit`")
				&& implicit.contains("#### Tree grammar")
				&& extract_section(&md, "support_report").is_some_and(|s| s.starts_with("### `support_report`"))
				&& extract_section(&md, "list_faces").is_some_and(|s| s.starts_with("### `list_faces`"))
				&& extract_section(&md, "list_edges").is_some_and(|s| s.starts_with("### `list_edges`"))
				&& extract_section(&md, "coincident_fit").is_some_and(|s| s.starts_with("### `coincident_fit`"))
				&& extract_section(&md, "no_such_op_lol").is_none(),
			"API.md section extraction contract broken.\nbox: {:?}…\nteardrop: {:?}…\ngland: {:?}…\nimplicit: {:?}…\nsupport_report present: {} (expected true since 2026-07-30)",
			&boxs.chars().take(80).collect::<String>(),
			&tear.chars().take(80).collect::<String>(),
			&gland.chars().take(80).collect::<String>(),
			&implicit.chars().take(80).collect::<String>(),
			extract_section(&md, "support_report").is_some(),
		);

		// The biconditional, over every op the live surface serves.
		let (_, ops) = op_catalogue(&std::env::temp_dir()).expect("describe runs in-process");
		let mut disagreements: Vec<String> = Vec::new();
		let mut undocumented: Vec<&str> = Vec::new();
		for op in &ops {
			let heading = format!("### `{op}`");
			let documented = md.lines().any(|l| l.starts_with(&heading));
			if !documented {
				undocumented.push(op);
			}
			match (documented, extract_section(&md, op)) {
				(true, Some(s)) if s.starts_with(&heading) => {}
				(false, None) => {}
				(true, Some(s)) => disagreements.push(format!(
					"{op}: API.md has the heading but the section starts {:?}",
					&s.chars().take(40).collect::<String>()
				)),
				(true, None) => disagreements.push(format!("{op}: API.md has the heading but extraction returned None")),
				(false, Some(_)) => disagreements.push(format!("{op}: API.md has NO heading but extraction returned a section")),
			}
		}
		assert!(
			disagreements.is_empty(),
			"extract_section disagrees with API.md's own headings for {} of {} ops: {:?}",
			disagreements.len(),
			ops.len(),
			&disagreements[..disagreements.len().min(8)],
		);
		// Not an assertion — a note, so closing the last gap never fails the test.
		eprintln!("API.md op sections: {}/{} documented; undocumented: {undocumented:?}", ops.len() - undocumented.len(), ops.len());
	}

	/// The catalogue comes from a REAL in-process describe run: count matches
	/// the list length, and the surface includes constructors, feature cuts
	/// and `describe` itself.
	#[test]
	fn op_catalogue_is_the_live_describe_surface() {
		let (count, ops) = op_catalogue(&std::env::temp_dir()).expect("describe runs in-process");
		assert!(
			count as usize == ops.len()
				&& count >= 100
				&& ops.iter().any(|o| o == "box")
				&& ops.iter().any(|o| o == "teardrop_hole")
				&& ops.iter().any(|o| o == "describe"),
			"describe catalogue should be self-consistent and complete: count {count}, len {}, has box/teardrop_hole/describe: {}/{}/{}",
			ops.len(),
			ops.iter().any(|o| o == "box"),
			ops.iter().any(|o| o == "teardrop_hole"),
			ops.iter().any(|o| o == "describe"),
		);
	}
}
