// Copyright (c) LMCAD. Licensed under the MIT License.

//! The `implicit` op (BAR.md I6): the CSG Node algebra as nestable JSON plus
//! the scalar-field expression language, end-to-end through `run_program` with
//! NO direct Rust geometry calls — grammar breadth, structured error paths
//! with JSON paths to the bad subtree, and the I6 acceptance: the
//! hybrid_showcase M10 helical-thread bolt rebuilt from PURE JSON against a
//! Rust-built reference meshed by the same kernel.

use std::path::PathBuf;
use std::sync::Arc;

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use kernel_implicit::{
	dual_contour_narrowband, make_manifold, manifold_dual_contour, Aabb, Cuboid, Cylinder as VoxCylinder, Gyroid,
	Node, Resolution, Sdf, Vec3,
};
use serde_json::{json, Value};

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_implicit_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

/// The report entry for op `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report
		.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
}

/// The `volume` measure of op `id`.
fn vol(report: &Report, id: &str) -> f64 {
	entry(report, id).measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN)
}

/// True when op `id` reported a watertight mesh.
fn watertight(report: &Report, id: &str) -> bool {
	entry(report, id).measures.as_ref().map(|m| m["watertight"] == json!(true)).unwrap_or(false)
}

/// (1) Grammar breadth: every combinator and every leaf shape of the implicit
/// expression tree executes green in ONE program — smooth/fillet/chamfer
/// blends, all rigid transforms and patterns (with a 2×/5-sphere volume
/// anchor), beam lattices in both construction forms, pipes and helix pipes,
/// `expr_sdf` leaves (with an exact prism + sphere volume anchor), and the
/// field-modulated `lerp` (anchored by the exact r=9 midpoint sphere) and
/// `offset_by`. The kernel math under each node is proven in kernel-implicit's
/// own suite; THIS test pins that the JSON grammar wires every name through.
#[test]
fn implicit_grammar_breadth_program() {
	let dir = out_dir("breadth");
	let hex_circum = 4.0 / (std::f64::consts::PI / 6.0).cos();
	// The hex-prism field language idiom: max of three |cos·x + sin·y| − af/2
	// half-plane pairs and the two end planes (1-Lipschitz by construction).
	let hex_walls = (0..3)
		.map(|k| {
			let a = k as f64 * std::f64::consts::PI / 3.0;
			json!({"op": "sub",
				"a": {"op": "abs", "arg": {"op": "add",
					"a": {"op": "mul", "a": "x", "b": a.cos()},
					"b": {"op": "mul", "a": "y", "b": a.sin()}}},
				"b": 4.0})
		})
		.collect::<Vec<_>>();
	let hex_expr = json!({"op": "max",
		"a": {"op": "max", "a": hex_walls[0], "b": {"op": "max", "a": hex_walls[1], "b": hex_walls[2]}},
		"b": {"op": "max", "a": {"op": "neg", "arg": "z"}, "b": {"op": "sub", "a": "z", "b": 6.0}}});

	let program = json!({"ops": [
		// Smooth blends over sphere/box/cylinder.
		{"id": "blend", "op": "implicit", "voxel": 0.4,
			"expr": {"op": "smooth_intersection", "k": 1,
				"a": {"op": "smooth_difference", "k": 1,
					"a": {"op": "smooth_union", "k": 2,
						"a": {"shape": "sphere", "center": [0, 0, 8], "radius": 8},
						"b": {"shape": "box", "min": [-8, -8, -8], "max": [8, 8, 8]}},
					"b": {"shape": "cylinder", "a": [0, 0, -12], "b": [0, 0, 16], "radius": 3}},
				"b": {"shape": "sphere", "center": [0, 0, 0], "radius": 10}}},
		// True-radius fillet/chamfer feature blends.
		{"id": "feat", "op": "implicit", "voxel": 0.4,
			"expr": {"op": "chamfer_difference", "r": 1,
				"a": {"op": "fillet_difference", "r": 1,
					"a": {"op": "chamfer_union", "r": 1,
						"a": {"op": "fillet_union", "r": 2,
							"a": {"shape": "box", "min": [-10, -10, 0], "max": [10, 10, 6]},
							"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 14], "radius": 4}},
						"b": {"shape": "box", "min": [6, -4, 0], "max": [14, 4, 4]}},
					"b": {"shape": "cylinder", "a": [0, 0, 8], "b": [0, 0, 15], "radius": 2}},
				"b": {"shape": "box", "min": [-11, -11, -1], "max": [-7, -7, 7]}}},
		// Rigid transforms + offset/shell + a plane-clipped hemisphere.
		{"id": "xform", "op": "implicit", "voxel": 0.4,
			"expr": {"op": "union",
				"a": {"op": "union",
					"a": {"op": "rotate", "axis": [0, 1, 0], "degrees": 45, "center": [0, 0, 5],
						"in": {"shape": "cone", "a": [0, 0, 0], "b": [0, 0, 10], "ra": 4, "rb": 1.5}},
					"b": {"op": "union",
						"a": {"op": "scale", "factor": 1.2,
							"in": {"shape": "torus", "center": [30, 0, 5], "axis": [0, 0, 1], "major": 6, "minor": 2}},
						"b": {"op": "translate", "offset": [60, 0, 0],
							"in": {"shape": "capsule", "a": [0, 0, 0], "b": [0, 0, 6], "radius": 2}}}},
				"b": {"op": "union",
					"a": {"op": "union",
						"a": {"op": "offset", "t": 1.5, "in": {"shape": "sphere", "center": [90, 0, 5], "radius": 4}},
						"b": {"op": "shell", "t": 1.5, "in": {"shape": "box", "min": [110, -6, 0], "max": [122, 6, 10]}}},
					"b": {"op": "intersection",
						"a": {"shape": "sphere", "center": [140, 0, 5], "radius": 6},
						"b": {"shape": "plane", "point": [140, 0, 5], "normal": [0, 0, 1]}}}}},
		// Mirror + patterns, anchored: 2 × r3 spheres + (2 + 3) × r2 spheres.
		{"id": "patterns", "op": "implicit", "voxel": 0.25,
			"expr": {"op": "union",
				"a": {"op": "mirror", "point": [0, 0, 0], "normal": [1, 0, 0],
					"in": {"shape": "sphere", "center": [8, 0, 0], "radius": 3}},
				"b": {"op": "union",
					"a": {"op": "linear_pattern", "step": [8, 0, 0], "count": 2,
						"in": {"shape": "sphere", "center": [0, 30, 0], "radius": 2}},
					"b": {"op": "circular_pattern", "center": [40, 0, 0], "axis": [0, 0, 1], "count": 3,
						"in": {"shape": "sphere", "center": [46, 0, 0], "radius": 2}}}}},
		// Beam lattices: one octet cell (junction-rich ⇒ manifold mesher) and an
		// explicit tapered-strut tripod graph.
		{"id": "lattice", "op": "implicit", "voxel": 0.35, "mesher": "manifold",
			"expr": {"shape": "beam_lattice", "min": [0, 0, 0], "max": [10, 10, 10],
				"cell": "octet", "cell_size": 10, "radius": 1.4}},
		{"id": "tripod", "op": "implicit", "voxel": 0.3, "mesher": "manifold",
			"expr": {"shape": "beam_lattice",
				"nodes": [[0, 0, 0], [6, 0, 8], [-6, 3, 8], [0, -6, 8]],
				"struts": [[0, 1, 1.2, 0.8], [0, 2, 1.2, 0.8], [0, 3, 1.2, 0.8]]}},
		// Pipes: varying-radius polyline + a helix.
		{"id": "pipes", "op": "implicit", "voxel": 0.25,
			"expr": {"op": "union",
				"a": {"shape": "pipe", "path": [[0, 0, 0], [10, 0, 4], [20, 0, 0]], "radii": [2, 3, 2]},
				"b": {"shape": "helix_pipe", "center": [40, 0, 0], "axis": [0, 0, 1],
					"r_helix": 6, "pitch": 4, "turns": 2, "radius": 1.2, "samples_per_turn": 32}}},
		// expr_sdf leaves: the exact hex prism (AF 8 × 6 tall) plus an r=4 sphere
		// written as length3/sqrt/min — both 1-Lipschitz with exact volumes.
		{"id": "expr", "op": "implicit", "voxel": 0.3,
			"expr": {"op": "union",
				"a": {"shape": "expr_sdf", "lipschitz_bound": 1.0,
					"min": [-hex_circum, -hex_circum, 0.0], "max": [hex_circum, hex_circum, 6.0],
					"expr": hex_expr},
				"b": {"shape": "expr_sdf", "lipschitz_bound": 1.0,
					"min": [-4.0, -4.0, 16.0], "max": [4.0, 4.0, 24.0],
					"expr": {"op": "sub",
						"a": {"op": "length3", "a": "x", "b": "y", "c": {"op": "sub", "a": "z", "b": 20.0}},
						"b": {"op": "sqrt", "arg": {"op": "min", "a": 16.0, "b": 25.0}}}}}},
		// lerp at the exact constant midpoint weight: concentric r6/r12 spheres
		// blend to EXACTLY the r9 sphere (field exercises sin/cos/div/clamp).
		{"id": "lerp9", "op": "implicit", "voxel": 0.25,
			"expr": {"op": "lerp",
				"a": {"shape": "sphere", "center": [0, 0, 0], "radius": 6},
				"b": {"shape": "sphere", "center": [0, 0, 0], "radius": 12},
				"field": {"op": "clamp",
					"value": {"op": "div",
						"a": {"op": "add", "a": {"op": "sin", "arg": 0.0}, "b": {"op": "cos", "arg": 0.0}},
						"b": 2.0},
					"lo": 0.0, "hi": 1.0}}},
		// offset_by: the z-graded shell (wall 2 → 4 mm) — the documented
		// slowly-varying-field contract, meshed dense via the manifold extractor.
		{"id": "graded", "op": "implicit", "voxel": 0.3, "mesher": "manifold",
			"expr": {"op": "difference",
				"a": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 40], "radius": 10},
				"b": {"op": "offset_by", "max_abs": 6,
					"in": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 40], "radius": 10},
					"field": {"op": "neg", "arg": {"op": "add", "a": 2.0,
						"b": {"op": "mul", "a": 0.05, "b": {"op": "clamp", "value": "z", "lo": 0.0, "hi": 40.0}}}}}},
			"file": "graded_shell.stl"}
	]});

	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let all_watertight =
		["blend", "feat", "xform", "patterns", "lattice", "tripod", "pipes", "expr", "lerp9", "graded"]
			.iter()
			.all(|id| watertight(&report, id));
	let sphere = |r: f64| 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
	let patterns_want = 2.0 * sphere(3.0) + 5.0 * sphere(2.0);
	let expr_want = 3.0f64.sqrt() / 2.0 * 64.0 * 6.0 + sphere(4.0);
	let lerp_want = sphere(9.0);
	let (patterns_got, expr_got, lerp_got) = (vol(&report, "patterns"), vol(&report, "expr"), vol(&report, "lerp9"));
	assert!(
		report.ok
			&& all_watertight
			&& (patterns_got - patterns_want).abs() / patterns_want < 0.04
			&& (expr_got - expr_want).abs() / expr_want < 0.03
			&& (lerp_got - lerp_want).abs() / lerp_want < 0.02
			&& std::fs::metadata(dir.join("graded_shell.stl")).map(|m| m.len() > 0).unwrap_or(false),
		"implicit grammar breadth: ok={} all_watertight={all_watertight} patterns={patterns_got:.1} (want {patterns_want:.1}) expr={expr_got:.1} (want {expr_want:.1}) lerp9={lerp_got:.1} (want {lerp_want:.1}) report={report:#?}",
		report.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (1b) The NATIVE `voronoi_lattice` leaf end-to-end through the JSON grammar:
/// a seed cloud (built here by a fixed deterministic LCG — no scipy, no rand
/// crate) becomes an open-cell foam BALL SHELL, intersected with a sphere
/// shell and meshed watertight by the manifold extractor — proving the airless
/// foam builds with ZERO Python. The `< 5 seeds` guard is a loud structured
/// `invalid_param` that names the `seeds` field.
#[test]
fn voronoi_lattice_foam_from_pure_json() {
	let dir = out_dir("voronoi");
	// A fixed, deterministic seed cloud in the ±11 cube (LCG — reproducible, no
	// randomness at test time).
	let mut s = 0x1357_9BDF_2468_ACE0_u64;
	let mut next = || {
		s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		-11.0 + 22.0 * ((s >> 33) as f64) / ((1u64 << 31) as f64)
	};
	let seeds: Vec<Value> = (0..48).map(|_| json!([next(), next(), next()])).collect();
	let shell = json!({"op": "difference",
		"a": {"shape": "sphere", "center": [0, 0, 0], "radius": 10},
		"b": {"shape": "sphere", "center": [0, 0, 0], "radius": 6}});
	let program = json!({"ops": [
		{"id": "foam", "op": "implicit", "voxel": 0.3, "mesher": "manifold", "file": "voronoi_foam_shell.stl",
			"expr": {"op": "intersection",
				"a": {"shape": "voronoi_lattice", "seeds": seeds, "radius": 0.9,
					"min": [-12, -12, -12], "max": [12, 12, 12]},
				"b": shell}}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);

	// The <5-seeds guard is a loud structured invalid_param naming 'seeds'.
	let bad = json!({"ops": [
		{"id": "x", "op": "implicit", "voxel": 0.5,
			"expr": {"shape": "voronoi_lattice", "seeds": [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
				"radius": 1, "min": [-2, -2, -2], "max": [2, 2, 2]}}]});
	let badreport = run_program(&serde_json::to_string(&bad).expect("serialize"), &dir);
	let bad_err = badreport.ops[0].error.as_ref().unwrap_or_else(|| panic!("expected an error, got {badreport:#?}"));

	assert!(
		report.ok
			&& watertight(&report, "foam")
			&& vol(&report, "foam") > 0.0
			&& !badreport.ok
			&& bad_err.kind == ErrorKind::InvalidParam
			&& bad_err.message.contains("seeds")
			&& bad_err.message.contains("at least 5"),
		"voronoi_lattice from JSON: ok={} watertight={} vol={:.1} | bad_kind={:?} bad_msg={:?}",
		report.ok,
		watertight(&report, "foam"),
		vol(&report, "foam"),
		bad_err.kind,
		bad_err.message
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) Implicit error paths are STRUCTURED and carry the JSON path to the bad
/// subtree: unknown names deep in the tree, a rejected Lipschitz declaration,
/// a non-finite field caught at a probe point BEFORE meshing, an unbounded
/// tree without a domain, a bad export extension, an out-of-range pattern
/// count, a node with both 'shape' and 'op', and an empty (disjoint) result.
#[test]
fn implicit_error_paths_are_structured() {
	let dir = out_dir("errors");
	let run = |program: Value| run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let sphere = json!({"shape": "sphere", "center": [0, 0, 0], "radius": 5});

	// Each case: (program, expected kind, message needles).
	let cases: Vec<(Value, ErrorKind, Vec<&str>)> = vec![
		// (a) Unknown shape, nested two levels deep — the path names the subtree.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"op": "union", "a": sphere,
					"b": {"op": "difference", "a": {"shape": "sphre", "center": [0,0,0], "radius": 3}, "b": sphere}}}]}),
			ErrorKind::InvalidParam,
			vec!["expr.b.a", "unknown shape 'sphre'"],
		),
		// (b) Unknown combinator with the supported list.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"op": "unoin", "a": sphere, "b": sphere}}]}),
			ErrorKind::InvalidParam,
			vec!["at expr", "unknown combinator 'unoin'"],
		),
		// (c) lipschitz_bound <= 0 is rejected with the contract in the message.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"shape": "expr_sdf", "expr": "z", "lipschitz_bound": 0.0,
					"min": [-5,-5,-5], "max": [5,5,5]}}]}),
			ErrorKind::InvalidParam,
			vec!["lipschitz_bound", "zero set preserved"],
		),
		// (d) A division pole inside the domain is caught at a probe point
		// BEFORE meshing, naming the expression's JSON path.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"shape": "expr_sdf", "lipschitz_bound": 1.0,
					"min": [-5,-5,-5], "max": [5,5,5],
					"expr": {"op": "sub", "a": {"op": "div", "a": 1.0, "b": "z"}, "b": 2.0}}}]}),
			ErrorKind::InvalidParam,
			vec!["expr.expr", "probe point"],
		),
		// (e) An unbounded tree (bare plane) without an explicit domain.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"shape": "plane", "point": [0,0,0], "normal": [0,0,1]}}]}),
			ErrorKind::InvalidParam,
			vec!["unbounded", "domain"],
		),
		// (f) Only .stl / .3mf exports.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5, "expr": sphere, "file": "out.obj"}]}),
			ErrorKind::InvalidParam,
			vec![".stl or .3mf"],
		),
		// (g) Pattern count outside 1..=4096.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"op": "linear_pattern", "step": [5,0,0], "count": 0, "in": sphere}}]}),
			ErrorKind::InvalidParam,
			vec!["count", "1..=4096"],
		),
		// (h) A node may carry 'shape' or 'op', never both.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"shape": "sphere", "op": "union", "center": [0,0,0], "radius": 5}}]}),
			ErrorKind::InvalidParam,
			vec!["either 'shape' or 'op'"],
		),
		// (i) A disjoint intersection has empty bounds — caught loudly BEFORE
		// meshing, never a silent empty file.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"expr": {"op": "intersection",
					"a": sphere,
					"b": {"shape": "sphere", "center": [50, 0, 0], "radius": 5}}}]}),
			ErrorKind::InvalidParam,
			vec!["empty bounds"],
		),
		// (j) An empty result under an EXPLICIT domain meshes zero triangles —
		// loud invalid_geometry instead of a bound-less success.
		(
			json!({"ops": [{"id": "x", "op": "implicit", "voxel": 0.5,
				"domain": {"min": [-10, -10, -10], "max": [10, 10, 10]},
				"expr": {"op": "intersection",
					"a": sphere,
					"b": {"shape": "sphere", "center": [50, 0, 0], "radius": 5}}}]}),
			ErrorKind::InvalidGeometry,
			vec!["did not mesh watertight"],
		),
	];

	let results: Vec<(ErrorKind, bool)> = cases
		.iter()
		.map(|(program, _, needles)| {
			let r = run(program.clone());
			let e = r.ops[0].error.as_ref().unwrap_or_else(|| panic!("expected a structured error, got {r:#?}"));
			(e.kind, needles.iter().all(|n| e.message.contains(n)))
		})
		.collect();
	let want: Vec<(ErrorKind, bool)> = cases.iter().map(|(_, kind, _)| (*kind, true)).collect();
	assert_eq!(
		results, want,
		"implicit error paths must carry the expected kind and name the offending subtree (cases in declaration order)"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// An `expr_sdf` leaf's declared `lipschitz_bound` is now sample-verified — but
/// only where a wrong bound actually bites. The narrow-band mesher prunes by the
/// Lipschitz contract, so an under-declared bound would silently tear holes; it
/// must be REFUSED there. The dense (`manifold`) mesher samples every cell and
/// needs no bound, so the SAME under-declaration must still mesh. `length3-8` is
/// a true distance field (`|∇| = 1`); declaring `0.2` under-states it 5×.
#[test]
fn expr_sdf_under_declared_lipschitz_is_caught_on_narrowband_only() {
	let dir = out_dir("lipschitz");
	let run = |program: Value| run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let prog = |l: f64, mesher: &str| {
		json!({"ops": [{"id": "s", "op": "implicit", "voxel": 0.6, "mesher": mesher,
			"expr": {"op": "intersection",
				"a": {"shape": "expr_sdf", "lipschitz_bound": l, "min": [-12,-12,-12], "max": [12,12,12],
					"expr": {"op": "sub", "a": {"op": "length3", "a": "x", "b": "y", "c": "z"}, "b": 8.0}},
				"b": {"shape": "box", "min": [-12,-12,-12], "max": [12,12,12]}}}]})
	};
	let truthful = run(prog(1.0, "narrowband"));
	let under_nb = run(prog(0.2, "narrowband"));
	let under_dense = run(prog(0.2, "manifold"));
	let under_err = under_nb.ops[0].error.as_ref();
	assert!(
		truthful.ok
			&& !under_nb.ok
			&& under_err.map(|e| e.kind == ErrorKind::InvalidParam && e.message.contains("UNDER-stated")).unwrap_or(false)
			&& under_dense.ok,
		"under-declared expr_sdf lipschitz_bound must be refused on narrow-band, accepted on dense: \
		 truthful.ok={} under_nb.ok={} under_nb.err={:?} under_dense.ok={}",
		truthful.ok,
		under_nb.ok,
		under_err.map(|e| &e.message),
		under_dense.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// All six TPMS families are reachable from the flat op program via the `tpms`
/// shape leaf (network mode) and each meshes watertight — the three families
/// added alongside the original gyroid/Schwarz-P/diamond (Neovius, Schoen I-WP,
/// Fischer-Koch S) are now first-class on the AI-callable surface, not just the
/// Rust API.
#[test]
fn tpms_families_mesh_watertight_through_the_op_surface() {
	let dir = out_dir("tpms_ops");
	let run = |program: Value| run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let kinds = ["gyroid", "schwarz_p", "diamond", "neovius", "schoen_iwp", "fischer_koch_s"];
	let mut report = String::new();
	let mut all_ok = true;
	for k in kinds {
		let prog = json!({"ops": [{"id": "lat", "op": "implicit", "voxel": 0.5, "mesher": "manifold",
			"expr": {"op": "intersection",
				"a": {"shape": "tpms", "kind": k, "mode": "network", "cell": 7, "level": 0, "min": [-12,-12,-12], "max": [12,12,12]},
				"b": {"shape": "box", "min": [-10,-10,-10], "max": [10,10,10]}}}]});
		let r = run(prog);
		let m = entry(&r, "lat").measures.as_ref();
		let wt = m.and_then(|m| m["watertight"].as_bool()).unwrap_or(false);
		let tris = m.and_then(|m| m["triangles"].as_u64()).unwrap_or(0);
		all_ok &= r.ok && wt && tris > 0;
		report += &format!("\n  {k}: ok={} watertight={wt} tris={tris}", r.ok);
	}
	assert!(all_ok, "every TPMS family must mesh watertight through the `tpms` op-surface leaf:{report}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// The NAMED `tpms` op (discoverability twin of the tree leaf): every family
/// meshes watertight with route `"voxel_implicit"` and writes its file; sheet
/// mode works; and the two loud-error contracts hold — a bad `kind` names the
/// six families (`invalid_param`), sheet mode without a positive `level` is
/// refused. Same parser as the leaf, so this pins the WIRING, not new math.
#[test]
#[cfg(feature = "catalog")]
fn tpms_named_op_all_families_and_errors() {
	let dir = out_dir("tpms_named");
	let run = |program: Value| run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let kinds = ["gyroid", "schwarz_p", "diamond", "neovius", "schoen_iwp", "fischer_koch_s"];
	let mut report = String::new();
	let mut all_ok = true;
	for k in kinds {
		let prog = json!({"ops": [{"id": "lat", "op": "tpms", "kind": k, "min": [-10,-10,-10], "max": [10,10,10],
			"cell": 7, "voxel": 0.5, "file": format!("{k}.stl")}]});
		let r = run(prog);
		let m = entry(&r, "lat").measures.as_ref();
		let wt = m.and_then(|m| m["watertight"].as_bool()).unwrap_or(false);
		let route = m.and_then(|m| m["route"].as_str()).unwrap_or("<missing>").to_string();
		let file_ok = dir.join(format!("{k}.stl")).exists();
		all_ok &= r.ok && wt && route == "voxel_implicit" && file_ok;
		report += &format!("\n  {k}: ok={} watertight={wt} route={route} file={file_ok}", r.ok);
	}
	// Sheet mode: a Schoen I-WP sheet with a real wall meshes watertight too.
	let sheet = run(json!({"ops": [{"id": "lat", "op": "tpms", "kind": "schoen_iwp", "mode": "sheet", "level": 0.8,
		"min": [-10,-10,-10], "max": [10,10,10], "cell": 8, "voxel": 0.5, "file": "iwp_sheet.stl"}]}));
	let sheet_ok = sheet.ok && watertight(&sheet, "lat");
	report += &format!("\n  schoen_iwp sheet: ok={sheet_ok}");
	// Loud errors: unknown family; sheet without a level.
	let bad_kind = run(json!({"ops": [{"id": "lat", "op": "tpms", "kind": "spaghetti", "min": [0,0,0], "max": [10,10,10],
		"cell": 5, "file": "x.stl"}]}));
	let bad_kind_ok = !bad_kind.ok
		&& entry(&bad_kind, "lat").error.as_ref().map(|e| e.kind == ErrorKind::InvalidParam && e.message.contains("gyroid")).unwrap_or(false);
	let bad_sheet = run(json!({"ops": [{"id": "lat", "op": "tpms", "kind": "gyroid", "mode": "sheet", "min": [0,0,0],
		"max": [10,10,10], "cell": 5, "file": "y.stl"}]}));
	let bad_sheet_ok = !bad_sheet.ok
		&& entry(&bad_sheet, "lat").error.as_ref().map(|e| e.kind == ErrorKind::InvalidParam).unwrap_or(false);
	report += &format!("\n  bad kind loud={bad_kind_ok} sheet-without-level loud={bad_sheet_ok}");
	assert!(
		all_ok && sheet_ok && bad_kind_ok && bad_sheet_ok,
		"the named `tpms` op must mesh all six families watertight (route voxel_implicit), support sheet mode, and refuse bad kinds / missing sheet level loudly:{report}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

// --- The I6 acceptance: the hybrid_showcase helical thread from PURE JSON ------

/// JSON scalar-AST builders (`sub`/`add`/`mul`/`max` shorthands).
fn jsub(a: Value, b: Value) -> Value {
	json!({"op": "sub", "a": a, "b": b})
}
fn jadd(a: Value, b: Value) -> Value {
	json!({"op": "add", "a": a, "b": b})
}
fn jmul(a: Value, b: Value) -> Value {
	json!({"op": "mul", "a": a, "b": b})
}
fn jmax(a: Value, b: Value) -> Value {
	json!({"op": "max", "a": a, "b": b})
}

/// The hybrid_showcase `HelicalThreadSdf` idiom, duplicated VERBATIM as the
/// independent Rust-built reference for the I6 acceptance: in helical
/// coordinates (radius, axial offset to the nearest turn) the swept ISO-like
/// trapezoid is a fixed convex quad — its field is the max of the four edge
/// half-planes, clamped to the threaded span.
struct HelicalThreadSdf {
	shank_r: f32,
	z0: f32,
	z1: f32,
	pitch: f32,
	depth: f32,
}

impl Sdf for HelicalThreadSdf {
	fn distance(&self, p: Vec3) -> f32 {
		let rad = (p.x * p.x + p.y * p.y).sqrt();
		let theta = p.y.atan2(p.x);
		let mut u = (p.z - self.z0 - self.pitch * theta / std::f32::consts::TAU).rem_euclid(self.pitch);
		if u > self.pitch * 0.5 {
			u -= self.pitch;
		}
		let (ra, rc) = (self.shank_r - 0.3, self.shank_r + self.depth);
		let (bw, cw) = (self.pitch * 0.43, self.pitch * 0.08);
		let v = [[ra, -bw], [rc, -cw], [rc, cw], [ra, bw]];
		let mut d = f32::NEG_INFINITY;
		for i in 0..4 {
			let (a, b) = (v[i], v[(i + 1) % 4]);
			let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
			let inv_len = 1.0 / (ex * ex + ey * ey).sqrt();
			d = d.max(((rad - a[0]) * ey - (u - a[1]) * ex) * inv_len);
		}
		d.max(self.z0 - p.z).max(p.z - self.z1)
	}

	fn bounds(&self) -> Aabb {
		let r = self.shank_r + self.depth;
		Aabb::from_center_half_extent(
			Vec3::new(0.0, 0.0, (self.z0 + self.z1) * 0.5),
			Vec3::new(r, r, (self.z1 - self.z0) * 0.5 + self.pitch),
		)
	}
}

/// The hybrid_showcase `HexPrismSdf` reference twin (max of three flat
/// half-plane pairs and two end planes — exact zero set, 1-Lipschitz).
struct HexPrismSdf {
	af: f32,
	z0: f32,
	z1: f32,
}

impl Sdf for HexPrismSdf {
	fn distance(&self, p: Vec3) -> f32 {
		let mut d = f32::NEG_INFINITY;
		for k in 0..3 {
			let a = k as f32 * std::f32::consts::PI / 3.0;
			d = d.max((p.x * a.cos() + p.y * a.sin()).abs() - self.af * 0.5);
		}
		d.max(self.z0 - p.z).max(p.z - self.z1)
	}

	fn bounds(&self) -> Aabb {
		let r = self.af * 0.5 / (std::f32::consts::PI / 6.0).cos();
		Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, (self.z0 + self.z1) * 0.5), Vec3::new(r, r, (self.z1 - self.z0) * 0.5))
	}
}

/// (3) **The I6 falsifiable acceptance**: the hybrid_showcase M10×1.5
/// machine-bolt implicit twin — Ø10 shank, AF16 hex head, and the REAL
/// helical ISO-form thread — rebuilt through PURE JSON (no Rust geometry):
/// the thread is the helical-coordinate trapezoid written in the scalar-field
/// language (`atan2` unwrap, branchless `mod` recentering, four edge
/// half-planes under `max`), the hex head is an `expr_sdf` of six half-planes,
/// and the body is a `cylinder` leaf, all unioned and narrow-band extracted.
/// The JSON result must be watertight with a volume within 2% of the
/// Rust-built reference (the showcase's custom `Sdf` structs, duplicated
/// above, meshed inline by the SAME kernel at the same voxel and domain).
#[test]
fn pure_json_helical_thread_bolt_matches_rust_reference() {
	let dir = out_dir("bolt");
	// M10×1.5: shank Ø10 × 40, thread pitch 1.5 depth 0.85 over z 2..28, head AF16 z 40..46.4.
	let (shank_r, z0, z1, pitch, depth) = (5.0f64, 2.0f64, 28.0f64, 1.5f64, 0.85f64);
	let (ra, rc) = (shank_r - 0.3, shank_r + depth);
	let (bw, cw) = (pitch * 0.43, pitch * 0.08);
	let voxel = 0.08;

	// The thread field in the JSON scalar language. Helical coordinates:
	//   rad = length2(x, y),  θ = atan2(y, x),
	//   u = mod(z − pitch·θ/2π + P/2 − z0, P) − P/2   (branchless recentering
	//       into [−P/2, P/2) — replaces the reference's `if u > P/2 { u −= P }`;
	//       continuous across the θ branch cut because the jump is exactly P).
	// Then the max of the CCW quad's four edge half-planes (unit normals as
	// constants) and the two span planes. Declared Lipschitz bound 1.5: each
	// half-plane is α·rad + β·u + c with α² + β² = 1, |∇rad| = 1 and
	// |∇u| ≤ √(1 + (P/(2π·rad))²) ≈ 1.003 over the quad's radial band, so the
	// crude triangle-inequality bound |α| + |β|·1.003 ≤ √2·1.003 < 1.5 holds.
	let rad = json!({"op": "length2", "a": "x", "b": "y"});
	let theta = json!({"op": "atan2", "y": "y", "x": "x"});
	let tau = std::f64::consts::TAU;
	let u = jsub(
		json!({"op": "mod",
			"a": jadd(jsub(json!("z"), jmul(theta, json!(pitch / tau))), json!(pitch / 2.0 - z0)),
			"b": pitch}),
		json!(pitch / 2.0),
	);
	let quad = [[ra, -bw], [rc, -cw], [rc, cw], [ra, bw]];
	let planes: Vec<Value> = (0..4)
		.map(|i| {
			let (a, b) = (quad[i], quad[(i + 1) % 4]);
			let (er, eu) = (b[0] - a[0], b[1] - a[1]);
			let inv = 1.0 / (er * er + eu * eu).sqrt();
			jsub(
				jmul(jsub(rad.clone(), json!(a[0])), json!(eu * inv)),
				jmul(jsub(u.clone(), json!(a[1])), json!(er * inv)),
			)
		})
		.collect();
	let thread = jmax(
		jmax(jmax(planes[0].clone(), planes[1].clone()), jmax(planes[2].clone(), planes[3].clone())),
		jmax(jsub(json!(z0), json!("z")), jsub(json!("z"), json!(z1))),
	);

	// The AF16 hex head as an expr_sdf (1-Lipschitz: unit-normal half-planes).
	let hex_walls: Vec<Value> = (0..3)
		.map(|k| {
			let a = k as f64 * std::f64::consts::PI / 3.0;
			jsub(
				json!({"op": "abs", "arg": jadd(jmul(json!("x"), json!(a.cos())), jmul(json!("y"), json!(a.sin())))}),
				json!(8.0),
			)
		})
		.collect();
	let hex = jmax(
		jmax(hex_walls[0].clone(), jmax(hex_walls[1].clone(), hex_walls[2].clone())),
		jmax(jsub(json!(40.0), json!("z")), jsub(json!("z"), json!(46.4))),
	);
	let hex_circum = 8.0 / (std::f64::consts::PI / 6.0).cos();

	let domain = (json!([-9.3, -9.3, -0.2]), json!([9.3, 9.3, 46.6]));
	let program = json!({"ops": [
		{"id": "bolt", "op": "implicit", "voxel": voxel,
			"domain": {"min": domain.0, "max": domain.1},
			"expr": {"op": "union",
				"a": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 40], "radius": 5},
				"b": {"op": "union",
					"a": {"shape": "expr_sdf", "expr": hex, "lipschitz_bound": 1.0,
						"min": [-hex_circum, -hex_circum, 40.0], "max": [hex_circum, hex_circum, 46.4]},
					"b": {"shape": "expr_sdf", "expr": thread, "lipschitz_bound": 1.5,
						"min": [-rc, -rc, z0], "max": [rc, rc, z1]}}}}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let json_vol = vol(&report, "bolt");

	// The Rust-built reference: the showcase twin meshed inline by the same
	// kernel — same extractor, voxel and domain.
	let twin = Node::primitive(VoxCylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 40.0), 5.0))
		.union(Node::primitive(HexPrismSdf { af: 16.0, z0: 40.0, z1: 46.4 }))
		.union(Node::primitive(HelicalThreadSdf { shank_r: 5.0, z0: 2.0, z1: 28.0, pitch: 1.5, depth: 0.85 }));
	let rust_domain = Aabb::new(Vec3::new(-9.3, -9.3, -0.2), Vec3::new(9.3, 9.3, 46.6));
	let reference = dual_contour_narrowband(&twin, rust_domain, Resolution::VoxelSize(voxel as f32));
	let ref_vol = reference.signed_volume();

	println!(
		"I6 bolt @ voxel {voxel}: pure-JSON vol {json_vol:.3} mm³ vs Rust reference {ref_vol:.3} mm³ (Δ {:+.4}%)",
		(json_vol / ref_vol - 1.0) * 100.0
	);
	assert!(
		report.ok
			&& watertight(&report, "bolt")
			&& reference.is_watertight()
			&& ref_vol > 0.0
			&& (json_vol - ref_vol).abs() / ref_vol < 0.02,
		"I6: pure-JSON M10 thread bolt vs Rust reference: ok={} json_watertight={} json_vol={json_vol:.2} ref_watertight={} ref_vol={ref_vol:.2} (Δ {:+.3}%) report={report:#?}",
		report.ok,
		watertight(&report, "bolt"),
		reference.is_watertight(),
		(json_vol / ref_vol - 1.0) * 100.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (4) The graded-lattice acceptance: a gyroid ∩ box whose wall thickness is
/// driven by an expression field through `offset_by` (0.02·(z + 20) — half-
/// thickness ramps 0.6 → 1.4 mm bottom → top, field gradient 0.02 ≪ 1 per the
/// documented contract), extracted by Manifold DC. The JSON result must be
/// watertight, match the Rust-built reference (closure field, same kernel,
/// same grid) within 2%, and hold MORE volume than the ungraded lattice —
/// proof the data-driven grading really thickens the walls.
#[test]
fn graded_gyroid_lattice_program_matches_rust_reference() {
	let dir = out_dir("graded_gyroid");
	let program = json!({"ops": [
		{"id": "lat", "op": "implicit", "voxel": 0.8, "mesher": "manifold",
			"domain": {"min": [-20, -20, -20], "max": [20, 20, 20]},
			"expr": {"op": "intersection",
				"a": {"op": "offset_by", "max_abs": 0.8,
					"in": {"shape": "gyroid", "min": [-20, -20, -20], "max": [20, 20, 20],
						"scale": 0.35, "thickness": 0.6},
					"field": {"op": "mul", "a": 0.02, "b": {"op": "add", "a": "z", "b": 20.0}}},
				"b": {"shape": "box", "min": [-20, -20, -20], "max": [20, 20, 20]}}}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let json_vol = vol(&report, "lat");

	// Rust reference: the identical tree through the library API (closure
	// field), plus the ungraded baseline for the does-it-actually-grade check.
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(20.0));
	let mesh_of = |node: &Node| {
		let mut m = manifold_dual_contour(node, region, Resolution::VoxelSize(0.8));
		if !m.is_watertight() {
			m = make_manifold(&m);
		}
		m
	};
	let graded = Node::primitive(Gyroid::new(region, 0.35, 0.6))
		.offset_by(Arc::new(|p: Vec3| 0.02 * (p.z + 20.0)), 0.8)
		.intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(20.0))));
	let reference = mesh_of(&graded);
	let ref_vol = reference.signed_volume();
	let ungraded = Node::primitive(Gyroid::new(region, 0.35, 0.6)).intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(20.0))));
	let base_vol = mesh_of(&ungraded).signed_volume();

	println!(
		"graded gyroid: pure-JSON vol {json_vol:.0} mm³ vs Rust reference {ref_vol:.0} mm³ (Δ {:+.4}%), ungraded {base_vol:.0} mm³ ({:.2}× grading gain)",
		(json_vol / ref_vol - 1.0) * 100.0,
		json_vol / base_vol
	);
	assert!(
		report.ok
			&& watertight(&report, "lat")
			&& reference.is_watertight()
			&& ref_vol > 0.0
			&& (json_vol - ref_vol).abs() / ref_vol < 0.02
			&& json_vol > 1.2 * base_vol,
		"graded gyroid: ok={} json_watertight={} json_vol={json_vol:.0} ref_vol={ref_vol:.0} (Δ {:+.3}%) ungraded={base_vol:.0} report={report:#?}",
		report.ok,
		watertight(&report, "lat"),
		(json_vol / ref_vol - 1.0) * 100.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}
