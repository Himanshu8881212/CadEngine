// Copyright (c) LMCAD. Licensed under the MIT License.

//! The `catalog` cargo feature (docs/OP_USAGE.md): the hardware-catalog op families no shipped
//! campaign uses are compiled behind it. On by default — a default build must still execute
//! every one of them; a `--no-default-features` build must refuse them with `unknown_op` and a
//! message that names the feature, never the "not one of the N supported ops" typo message.

use kernel_api::{run_program, ErrorKind, Report, CATALOG_OP_NAMES, OP_COUNT, OP_NAMES};
use serde_json::json;
use std::path::Path;

fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

fn scratch(tag: &str) -> std::path::PathBuf {
	let dir = std::env::temp_dir().join(format!("lmcad_catalog_feature_{tag}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	dir
}

#[test]
fn catalog_list_is_consistent_with_the_describe_tables() {
	assert!(!CATALOG_OP_NAMES.is_empty(), "the gated list must not be empty");
	assert_eq!(OP_COUNT, OP_NAMES.len(), "OP_COUNT must match the OP_NAMES table of THIS build");
	let in_table = CATALOG_OP_NAMES.iter().filter(|n| OP_NAMES.contains(n)).count();
	if cfg!(feature = "catalog") {
		assert_eq!(in_table, CATALOG_OP_NAMES.len(), "with `catalog` on, every gated op is in OP_NAMES");
	} else {
		assert_eq!(in_table, 0, "with `catalog` off, no gated op may be advertised by `describe`");
	}
}

#[cfg(feature = "catalog")]
#[test]
fn every_catalog_op_executes_when_the_feature_is_on() {
	let dir = scratch("on");
	for name in CATALOG_OP_NAMES {
		let r = run(&dir, json!([{"id": "x", "op": name}]));
		if let Some(e) = r.ops.iter().find(|o| o.id == "x").and_then(|o| o.error.as_ref()) {
			assert_ne!(e.kind, ErrorKind::UnknownOp, "'{name}' is behind `catalog` but the feature is ON — {e:?}");
		}
	}
	let _ = std::fs::remove_dir_all(&dir);
}

#[cfg(not(feature = "catalog"))]
#[test]
fn every_catalog_op_is_refused_by_name_when_the_feature_is_off() {
	let dir = scratch("off");
	for name in CATALOG_OP_NAMES {
		let r = run(&dir, json!([{"id": "x", "op": name, "frame": 17, "body_len": 40.0}]));
		let e = r.ops.iter().find(|o| o.id == "x").and_then(|o| o.error.as_ref()).unwrap_or_else(|| panic!("'{name}' must fail — {r:#?}"));
		assert_eq!(e.kind, ErrorKind::UnknownOp, "'{name}' must be refused as unknown_op — {e:?}");
		assert!(
			e.message.contains("behind the `catalog` cargo feature") && !e.message.contains("not one of the"),
			"'{name}' must be refused BY NAME as compiled out, not as a typo — got {:?}",
			e.message
		);
	}
	// A real typo keeps the enumerating message, so the two refusals stay distinguishable.
	let r = run(&dir, json!([{"id": "t", "op": "nema_motr"}]));
	let e = r.ops.iter().find(|o| o.id == "t").and_then(|o| o.error.as_ref()).expect("typo must fail");
	assert!(e.kind == ErrorKind::UnknownOp && e.message.contains("not one of the"), "typo message must enumerate — {e:?}");
	let _ = std::fs::remove_dir_all(&dir);
}
