// Copyright (c) LMCAD. Licensed under the MIT License.

//! Measures and assertions — the interrogation half of the API. The measures say
//! what a solid or mesh *is* (`validate`, the volumes, `mass_properties`,
//! `bounding_box`, `wall_thickness`, `draft_analysis`, `mesh_components`); the
//! assertions are the in-program oracles that make a program FAIL on unmet intent
//! (`assert`, `assert_disjoint`, `coincident_fit`, `clearance`, `support_report`),
//! together with the `describe` / `list_faces` / `list_edges` probes.

use std::collections::{BTreeMap, BTreeSet};

use kernel_brep::math::DVec3;
use kernel_core::math::Vec3;
use kernel_core::{check_mesh, Mesh, ThicknessOptions, ThicknessSample};
use serde_json::{json, Value};

use crate::interp::{err, fetch_measurable, fetch_solid, EnvValue, Measurable, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{dv3, polygon_area, polygon_centroid, v3a};

/// The witness block `validate` reports when `geometric_ok` is false: the two
/// crossing triangles, a point on the crossing, and the total pair count.
pub(crate) fn self_intersection_json(w: &kernel_core::mesh::SelfIntersection) -> Value {
	json!({
		"triangles": w.triangles,
		"point": [w.point.x, w.point.y, w.point.z],
		"pairs": w.pairs,
	})
}

/// Validate the two knobs of the connectivity oracle. Shared by `mesh_components`
/// and `assert`, so the gate and its diagnostic can never be tuned differently.
pub(crate) fn connectivity_tolerances(op_id: &str, tol: f64, weld_tol: f64) -> Result<(), OpError> {
	if !(tol.is_finite() && tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
	}
	if !(weld_tol.is_finite() && weld_tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': weld_tol must be a positive weld scale in mm")));
	}
	Ok(())
}

/// The connected-body count plus the receipt that says whether it can be
/// believed.
///
/// # The trust rule
///
/// Union-find over welded triangles answers "how many connected pieces is this
/// SURFACE in". That is the part's body count only when the surface is the whole
/// boundary of the part. A bound solid is closed and manifold by construction
/// (every solid-producing op is gated through `validate`), so if its measurement
/// tessellation has boundary edges the faceter has dropped geometry, and the
/// count is then counting faceter cracks. Reporting that number as a body count
/// is precisely the confident-wrong-answer this engine refuses to give, so the
/// op FAILS instead, and says what to gate meanwhile.
///
/// A bound MESH is different: openness is a property of the data, not a defect
/// of the measurement, so an open mesh is measured and reported honestly with
/// `watertight: false` for `require` to gate.
pub(crate) fn connectivity_measures(
	op_id: &str,
	mesh: &Mesh,
	tol: f64,
	weld_tol: f64,
	source: &str,
) -> Result<serde_json::Map<String, Value>, OpError> {
	// Topology only — `check_mesh` would also run the self-intersection sweep,
	// which is orders of magnitude more expensive and answers a question this
	// gate does not ask. (`validate` is where self-intersection is paid for, on
	// demand.) These two are edge-hash passes, linear in triangle count.
	let boundary_edges = mesh.boundary_edge_count();
	let watertight = mesh.is_two_manifold();
	let components = mesh.component_count(weld_tol as f32);
	// Openings are the only defect that can break the count. A winding
	// inconsistency (`non_orientable`) leaves every triangle in place and every
	// vertex shared, so connectivity is untouched — it is reported, never
	// refused. (It USED to refuse: `boundary_edge_count` counted non-orientable
	// edges as boundary edges until 2026-08-08, which turned this guard on 11
	// shipped part programs whose tessellations are closed.)
	if source == "solid" && boundary_edges > 0 {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': the connectivity oracle cannot be trusted on this solid — tessellating it at tol {tol} mm left {} boundary edges ({} triangles), so the measurement surface is NOT closed and its component count ({components}) counts faceter cracks, not severed bodies. A bound solid is closed by construction, so this is a tessellation defect, not a geometry defect (a planar face carrying inner/hole loops is the known case: `extrude_with_holes` and `import_step` pockets). Gate this part with `validate` (closed / manifold / shells) meanwhile, and/or `export_stl` it and run this measure on the export's bound mesh — the exported mesh IS what prints",
				boundary_edges,
				mesh.triangle_count()
			),
		));
	}
	let mut m = serde_json::Map::new();
	m.insert("components".into(), json!(components));
	m.insert("is_one_body".into(), json!(components == 1));
	m.insert("triangles".into(), json!(mesh.triangle_count()));
	m.insert("tol".into(), json!(tol));
	m.insert("weld_tol".into(), json!(weld_tol));
	m.insert("watertight".into(), json!(watertight));
	m.insert("boundary_edges".into(), json!(boundary_edges));
	// Reported, not gated: this is what `watertight: false` means whenever
	// `boundary_edges` is 0, and without it that pair is unexplained. Another
	// edge-hash pass, not `check_mesh` (which would also pay for the
	// self-intersection sweep this gate does not ask about).
	let non_orientable = mesh.non_orientable_edge_count();
	m.insert("non_orientable_edges".into(), json!(non_orientable));
	if non_orientable > 0 {
		// A nonzero count must be locatable, not just countable: midpoints of
		// the first few offending edges, for aiming a fix or a disclosure.
		m.insert("non_orientable_witness".into(), json!(mesh.non_orientable_edge_witnesses(8)));
	}
	m.insert("source".into(), json!(source));
	Ok(m)
}

/// The `describe` entry for the universal `require` gate — identical for every
/// op, because `require` IS identical for every op.
pub(crate) fn universal_require_param() -> Value {
	json!({ "name": crate::require::REQUIRE_KEY, "type": "object", "required": false, "doc": crate::require::REQUIRE_DOC })
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::Validate { input } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			// A bound mesh has no B-rep record to validate; report the triangle
			// topology under the same key names, plus the mesh-only counts, and say
			// `source: "mesh"` so no reader mistakes one for the other.
			let s = match target {
				Measurable::Solid(s) => s,
				Measurable::Mesh(m) => {
					// `closed` is CLOSURE — no openings — and nothing else, so it can
					// never contradict the `boundary_edges` printed beside it. It used
					// to be `check_mesh().watertight`, which folds orientability and
					// vertex-manifoldness in, and a mesh with a flipped triangle then
					// reported `closed: false` next to `boundary_edges: 0` in the same
					// receipt. Everything the old `closed` covered still gates through
					// `manifold`, so `valid` (closed AND manifold) is unchanged.
					let r = check_mesh(m);
					let closed = r.boundary_edges == 0 && m.triangle_count() > 0;
					let manifold =
						r.non_manifold_edges == 0 && r.non_orientable_edges == 0 && r.non_manifold_vertices == 0;
					let witness = m.self_intersection_witness();
					let mut out = json!({
						"closed": closed,
						"manifold": manifold,
						"valid": closed && manifold,
						"triangles": m.triangle_count(),
						"boundary_edges": r.boundary_edges,
						"non_manifold_edges": r.non_manifold_edges,
						"non_orientable_edges": r.non_orientable_edges,
						"non_manifold_vertices": r.non_manifold_vertices,
						"geometric_ok": witness.is_none(),
						"source": "mesh",
					});
					if let Some(w) = &witness {
						out["self_intersection"] = self_intersection_json(w);
					}
					return Ok(Outcome::measures(out));
				}
			};
			let v = kernel_brep::validate(s);
			// M2 trust: `geometric_ok` is the geometric-validity flag (no self-intersection),
			// distinct from the topological validity above — a solid can be closed+manifold yet
			// self-overlapping with a silently-wrong volume. self_intersects() tessellates and is
			// O(tri²)-ish, so it is computed here on the EXPLICIT validate op (on demand), not on
			// every bind. false ⇒ measure the fit / re-route; do not trust the volume as exact.
			//
			// A bare `false` is not actionable, and a validity flag nobody can act on
			// is a validity flag everybody learns to ignore (theme T15 — three
			// campaigns shipped `geometric_ok:false` disclosed as "unexplained").
			// When the flag trips, the report now carries the WITNESS: which two
			// triangles cross, where in space, and how many pairs do it.
			let witness = kernel_brep::tessellate_default(s).self_intersection_witness();
			let mut out = json!({
				"closed": v.closed,
				"manifold": v.manifold,
				"euler_characteristic": v.euler_characteristic,
				"genus": v.genus,
				"shells": v.shells,
				"valid": v.is_valid(),
				"geometric_ok": witness.is_none(),
				"source": "solid",
			});
			if let Some(w) = &witness {
				out["self_intersection"] = self_intersection_json(w);
			}
			Ok(Outcome::measures(out))
		}
		OpKind::Volume { input } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			// Provenance (M2): `volume` is the tessellated (faceted) volume — use `exact_volume`
			// or `mass_properties` for the analytic value where the faces carry analytic surfaces.
			// A bound mesh's enclosed volume is only defined when it is watertight;
			// a leaky mesh gets a refusal, never a plausible number.
			match target {
				Measurable::Solid(s) => {
					Ok(Outcome::measures(json!({ "volume": kernel_brep::volume(s), "provenance": "faceted", "source": "solid" })))
				}
				Measurable::Mesh(m) => {
					// Edge topology only: whether the surface closes is an O(T) question,
					// and `check_mesh` would additionally run the self-intersection sweep
					// that `validate` exists to pay for.
					if !m.is_two_manifold() {
						return Err(err(
							ErrorKind::InvalidGeometry,
							format!(
								"op '{op_id}': '{input}' is a mesh with {} boundary edges and {} edges not shared by exactly two triangles — an open or non-manifold surface encloses no volume, so there is no number to report. Heal it (`import_mesh` with heal, or re-mesh through the voxel route) or measure its `bounding_box` instead",
								m.boundary_edge_count(),
								m.non_manifold_edge_count()
							),
						));
					}
					Ok(Outcome::measures(json!({ "volume": m.signed_volume(), "provenance": "faceted", "source": "mesh" })))
				}
			}
		}
		OpKind::ExactVolume { input } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			// Analytic where faces carry a quadric/torus surface; degrades to faceted per untagged face.
			Ok(Outcome::measures(json!({ "exact_volume": kernel_brep::exact_volume(s), "provenance": "analytic" })))
		}
		OpKind::MassProperties { input } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let m = kernel_brep::mass_properties(s);
			// `inertia_tensor` is the FULL 3×3 inertia tensor rows [[Ixx,Ixy,Ixz],…] about the
			// center of mass at unit density (mm⁵) — balance/imbalance analysis needs the
			// products of inertia, which the diagonal alone cannot carry. Convention: standard
			// dynamics tensor, off-diagonals are −∫xy dV etc. (glam stores columns; the tensor
			// is symmetric, rows are emitted explicitly). `inertia_diag` stays for compatibility.
			let i = &m.inertia;
			Ok(Outcome::measures(json!({
				"volume": m.volume,
				"center_of_mass": [m.center_of_mass.x, m.center_of_mass.y, m.center_of_mass.z],
				"inertia_diag": [m.inertia.x_axis.x, m.inertia.y_axis.y, m.inertia.z_axis.z],
				"inertia_tensor": [
					[i.x_axis.x, i.y_axis.x, i.z_axis.x],
					[i.x_axis.y, i.y_axis.y, i.z_axis.y],
					[i.x_axis.z, i.y_axis.z, i.z_axis.z],
				],
				"provenance": "analytic",
			})))
		}
		OpKind::BoundingBox { input, envelope } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let b = match target {
				Measurable::Solid(s) => kernel_brep::measure::bounding_box(s),
				// The mesh's own extent — for an exported print file this is the
				// envelope check that matters, not the solid's.
				Measurable::Mesh(m) => {
					let a = m.aabb();
					(!m.is_empty() && a.is_valid()).then(|| kernel_brep::measure::BoundingBox {
						min: DVec3::new(a.min.x as f64, a.min.y as f64, a.min.z as f64),
						max: DVec3::new(a.max.x as f64, a.max.y as f64, a.max.z as f64),
					})
				}
			};
			let b = b.ok_or_else(|| {
				err(ErrorKind::InvalidGeometry, format!("op '{op_id}': 'bounding_box' has no finite geometry to measure"))
			})?;
			let (mn, mx, sz, c) = (b.min, b.max, b.size(), b.center());
			let mut m = serde_json::Map::new();
			m.insert("source".into(), json!(target.source()));
			m.insert("min".into(), json!([mn.x, mn.y, mn.z]));
			m.insert("max".into(), json!([mx.x, mx.y, mx.z]));
			m.insert("size".into(), json!([sz.x, sz.y, sz.z]));
			m.insert("center".into(), json!([c.x, c.y, c.z]));
			m.insert("diagonal".into(), json!(b.diagonal()));
			if let Some(e) = envelope {
				let ev = dv3(e);
				m.insert("fits_within".into(), json!(b.fits_within(ev)));
				m.insert("fits_within_rotated".into(), json!(b.fits_within_rotated(ev)));
			}
			Ok(Outcome::measures(Value::Object(m)))
		}
		OpKind::WallThickness { input, flag_below, exclude_wedge_deg } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if let Some(deg) = exclude_wedge_deg {
				if !(deg.is_finite() && deg > 0.0 && deg <= 180.0) {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': exclude_wedge_deg must be a material dihedral angle in (0, 180] degrees"),
					));
				}
			}
			let t = kernel_brep::wall_thickness_with(s, ThicknessOptions { flag_below, exclude_wedge_deg });
			// Every thinness statistic is over the COUNTED samples — all of them
			// without a wedge exclusion, the non-wedge ones with it; the wedge
			// readings are reported apart (`thin_area_wedge`, `thin_wedge_witness`).
			// The samples are area-uniform, so the percentiles are AREA
			// percentiles. `min_thickness` is still edge noise on an acute body
			// without the exclusion (FRICTION #17); the robust signals are the
			// percentiles and `thin_area`.
			let mut finite: Vec<f64> = t.samples.iter().filter(|s| !s.wedge && s.thickness.is_finite()).map(|s| s.thickness).collect();
			finite.sort_unstable_by(f64::total_cmp);
			let pct = |p: f64| -> Value {
				if finite.is_empty() {
					Value::Null
				} else {
					json!(finite[((finite.len() - 1) as f64 * p).round() as usize])
				}
			};
			// The thinnest flagged samples of a bucket, so a nonzero area is
			// locatable: `{"at": [x, y, z], "thickness": t}`, thinnest first.
			let witness = |wedge: bool| -> Value {
				let mut flagged: Vec<&ThicknessSample> =
					t.samples.iter().filter(|s| s.wedge == wedge && s.thickness < flag_below).collect();
				flagged.sort_by(|a, b| a.thickness.total_cmp(&b.thickness));
				let points: Vec<Value> = flagged
					.iter()
					.take(8)
					.map(|s| json!({ "at": [s.point.x as f64, s.point.y as f64, s.point.z as f64], "thickness": s.thickness }))
					.collect();
				Value::Array(points)
			};
			let mut m = json!({
				"min_thickness": t.min_thickness,
				"p05_thickness": pct(0.05),
				"median_thickness": pct(0.5),
				"thin_area": t.thin_area,
				"flag_below": flag_below,
				"sampled_triangles": t.thickness.len(),
				"samples": t.samples.len(),
				"thin_witness": witness(false),
			});
			if let Some(deg) = exclude_wedge_deg {
				m["exclude_wedge_deg"] = json!(deg);
				m["thin_area_wedge"] = json!(t.thin_area_wedge);
				m["thin_area_total"] = json!(t.thin_area + t.thin_area_wedge);
				m["thin_wedge_witness"] = witness(true);
			}
			Ok(Outcome::measures(m))
		}
		OpKind::DraftAnalysis { input, pull, min_deg } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pull = dv3(pull);
			if pull.length_squared() <= f64::EPSILON {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': pull direction must be a non-zero vector")));
			}
			let d = kernel_brep::draft_analysis(s, pull, min_deg);
			Ok(Outcome::measures(json!({
				"min_draft_deg": d.min_draft_deg,
				"low_draft_area": d.low_draft_area,
				"undercut_area": d.undercut_area,
			})))
		}
		OpKind::MeshComponents { input, tol, weld_tol } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			connectivity_tolerances(op_id, tol, weld_tol)?;
			// Raw exact tessellation (never the voxel heal): connectivity of the
			// modelled surfaces is the question, and welding is what makes
			// coincident-but-unshared boolean vertices count as one point.
			let mesh = target.mesh(tol);
			let mut m = connectivity_measures(op_id, &mesh, tol, weld_tol, target.source())?;
			m.insert("provenance".into(), json!("faceted"));
			Ok(Outcome::measures(Value::Object(m)))
		}

		OpKind::Assert { input, volume_within, exact_volume_within, genus, shells, components, closed, manifold, valid, tol, weld_tol } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let any_check = volume_within.is_some()
				|| exact_volume_within.is_some()
				|| genus.is_some() || shells.is_some() || components.is_some()
				|| closed.is_some() || manifold.is_some() || valid.is_some();
			if !any_check {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': assert has no checks — give at least one of volume_within / exact_volume_within / genus / shells / components / closed / manifold / valid"),
				));
			}
			connectivity_tolerances(op_id, tol, weld_tol)?;
			// A bound MESH has no B-rep topology: genus / shells / exact_volume are
			// records of the solid model, and inventing them from triangles would be
			// exactly the plausible-looking number this surface refuses to produce.
			// The mesh-meaningful checks (components / closed / manifold / valid /
			// volume_within) are answered from the mesh itself.
			let s = match target {
				Measurable::Solid(s) => Some(s),
				Measurable::Mesh(_) => {
					for (name, present) in
						[("genus", genus.is_some()), ("shells", shells.is_some()), ("exact_volume_within", exact_volume_within.is_some())]
					{
						if present {
							return Err(err(
								ErrorKind::WrongType,
								format!(
									"op '{op_id}': assert '{name}' needs a bound SOLID — '{input}' is a mesh, which carries no B-rep topology or analytic surfaces. On a mesh assert components / closed / manifold / valid / volume_within instead"
								),
							));
						}
					}
					None
				}
			};
			let v = s.map(kernel_brep::validate);
			// Closed / manifold for a mesh come from edge topology alone; running the
			// full `check_mesh` here would drag in the self-intersection sweep, which
			// `assert` never reports and which is the expensive part of that call.
			let mesh_report = match target {
				Measurable::Mesh(m) => Some((m.boundary_edge_count() == 0 && !m.is_empty(), m.is_two_manifold())),
				Measurable::Solid(_) => None,
			};
			let mut measures = serde_json::Map::new();
			let mut failures: Vec<String> = Vec::new();
			let mut within = |what: &str, measured: f64, spec: &crate::program::WithinSpec| -> Result<(), OpError> {
				let half_width = match (spec.abs, spec.percent) {
					(Some(abs), None) if abs.is_finite() && abs >= 0.0 => abs,
					(None, Some(pct)) if pct.is_finite() && pct >= 0.0 => spec.target.abs() * pct / 100.0,
					_ => {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': {what}: exactly one of 'abs' / 'percent' is required (a finite non-negative tolerance)"),
						));
					}
				};
				if (measured - spec.target).abs() > half_width {
					failures.push(format!("{what}: measured {measured} is outside {} ± {half_width}", spec.target));
				}
				Ok(())
			};
			if let Some(spec) = &volume_within {
				let measured = match (s, &target) {
					(Some(s), _) => kernel_brep::volume(s),
					(None, Measurable::Mesh(m)) => m.signed_volume(),
					(None, _) => unreachable!("a non-solid target is a mesh"),
				};
				within("volume_within", measured, spec)?;
				measures.insert("volume".to_string(), json!(measured));
			}
			if let (Some(spec), Some(s)) = (&exact_volume_within, s) {
				let measured = kernel_brep::exact_volume(s);
				within("exact_volume_within", measured, spec)?;
				measures.insert("exact_volume".to_string(), json!(measured));
			}
			let mut equals = |what: &str, measured: Value, expected: Value| {
				if measured != expected {
					failures.push(format!("{what}: measured {measured}, expected {expected}"));
				}
				measures.insert(what.to_string(), measured);
			};
			if let (Some(g), Some(v)) = (genus, &v) {
				equals("genus", json!(v.genus), json!(g));
			}
			if let (Some(n), Some(v)) = (shells, &v) {
				equals("shells", json!(v.shells), json!(n));
			}
			if let Some(n) = components {
				// The single-body oracle (FRICTION #24): union-find over welded
				// triangle connectivity — `shells` counts B-rep records and cannot
				// catch a severed part, while this cannot see a severance narrower
				// than `weld_tol`. They are COMPLEMENTARY; neither dominates.
				let mesh = target.mesh(tol);
				let m = connectivity_measures(op_id, &mesh, tol, weld_tol, target.source())?;
				equals("components", m["components"].clone(), json!(n));
			}
			// closed / manifold / valid come from the B-rep record for a solid and
			// from the triangle topology for a mesh — same question, same answer
			// shape, measured where the geometry actually lives.
			if let Some(c) = closed {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.closed,
					(None, Some((closed, _))) => *closed,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("closed", json!(measured), json!(c));
			}
			if let Some(m) = manifold {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.manifold,
					(None, Some((_, manifold))) => *manifold,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("manifold", json!(measured), json!(m));
			}
			if let Some(ok) = valid {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.is_valid(),
					(None, Some((closed, manifold))) => *closed && *manifold,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("valid", json!(measured), json!(ok));
			}
			if failures.is_empty() {
				Ok(Outcome::measures(Value::Object(measures)))
			} else {
				Err(err(ErrorKind::AssertFailed, format!("op '{op_id}': assert failed: {}", failures.join("; "))))
			}
		}
		OpKind::AssertDisjoint { a, b, min_clearance, tol } => {
			let ta = fetch_measurable(env, all_ids, op_id, "a", &a)?;
			let tb = fetch_measurable(env, all_ids, op_id, "b", &b)?;
			if !(tol.is_finite() && tol > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
			}
			if !(min_clearance.is_finite() && min_clearance >= 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': min_clearance must be a finite non-negative gap in mm")));
			}
			// Raw exact tessellations: vertices lie on the true surfaces, and a
			// distance query needs no watertightness — never the voxel heal.
			let ma = ta.mesh(tol);
			let mb = tb.mesh(tol);
			let distance = ma.min_distance(&mb);
			if distance > min_clearance {
				Ok(Outcome::measures(
					json!({ "distance": distance, "min_clearance": min_clearance, "tol": tol, "source": [ta.source(), tb.source()] }),
				))
			} else {
				Err(err(
					ErrorKind::AssertFailed,
					format!(
						"op '{op_id}': assert_disjoint failed: surface distance {distance} mm ≤ required clearance {min_clearance} mm — '{a}' and '{b}' touch or interfere"
					),
				))
			}
		}

		OpKind::CoincidentFit { a, b } => {
			// Advisory pre-check the agent runs BEFORE a boolean to avoid the coincident-fit
			// hazard (audit V4). Wires the existing kernel scan; refuses nothing — the hard
			// hang backstop is the server request timeout (V6).
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			Ok(Outcome::measures(json!({ "coincident_fit": kernel_brep::detect_coincident_fit(sa, sb) })))
		}
		OpKind::SupportReport { input, build_dir, overhang_deg } => {
			// M5: FDM support-necessity audit — wires the existing Mesh::support_free_report.
			// Accepts a bound mesh so the audit can be run on the file that actually
			// prints (an export's healed mesh is not the solid's tessellation).
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let mesh = target.mesh(0.05);
			let up = Vec3::new(build_dir[0] as f32, build_dir[1] as f32, build_dir[2] as f32);
			let r = mesh.support_free_report(up, overhang_deg as f32, 0.2);
			Ok(Outcome::measures(json!({
				"support_free": r.steep_area < 1e-6,
				"bed_area": r.bed_area,
				"bridge_area": r.bridge_area,
				"steep_area": r.steep_area,
				"total_area": r.total_area,
				"max_bridge_span": r.max_bridge_span,
				"provenance": "faceted",
				"source": target.source(),
			})))
		}
		OpKind::Clearance { a, b, tol } => {
			// M5: non-asserting clearance/interference — the measuring twin of assert_disjoint.
			let ta = fetch_measurable(env, all_ids, op_id, "a", &a)?;
			let tb = fetch_measurable(env, all_ids, op_id, "b", &b)?;
			if !(tol.is_finite() && tol > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
			}
			let ma = ta.mesh(tol);
			let mb = tb.mesh(tol);
			let distance = ma.min_distance(&mb);
			// overlap_volume runs an EXACT boolean intersection, so it needs two exact
			// solids, and it is skipped on the coincident-fit hazard (a press-fit) so a
			// clearance query can't trigger the coincident-fit boolean hang (V4).
			// `overlap_volume: null` used to arrive with no explanation attached —
			// a null that does not say why is indistinguishable from a bug, so the
			// reason is now a first-class field and is never absent when the value is.
			let (overlap, hazard, reason) = match (&ta, &tb) {
				(Measurable::Solid(sa), Measurable::Solid(sb)) => {
					let hazard = kernel_brep::detect_coincident_fit(sa, sb);
					if hazard {
						(None, true, Some("coincident_fit_hazard: the operands share a flush/press-fit face pair, and the exact intersection across it is the known boolean-hang case (V4) — measure the fit analytically (measure_dimension diameter) instead"))
					} else {
						match kernel_brep::overlap_volume(sa, sb) {
							Some(v) => (Some(v), false, None),
							// The exact arrangement can fail on posed/near-degenerate
							// pairs while the meshes overlap plainly (friction
							// folding_book_stand F5: `overlap_volume: null` on an
							// interfering posed pair). Fall back to the mesh-level
							// boolean of the already-tessellated operands — a faceted
							// estimate, labelled as such, instead of a null.
							None => {
								let common = kernel_brep::mesh_intersection(&ma, &mb);
								let v = common.signed_volume().abs();
								(
									Some(v),
									false,
									Some("the exact boolean intersection did not produce a measurable body for this operand pair — `overlap_volume` is the FACETED mesh-boolean volume of the tessellated operands at `tol` (an estimate, not the analytic overlap); gate `exact_volume` on an explicit `intersection` body when the exact number matters"),
								)
							}
						}
					}
				}
				_ => (
					None,
					false,
					Some("overlap_volume needs two exact solids; at least one operand is a bound MESH, which carries no exact boolean — `distance` is measured on the meshes and is the honest answer here"),
				),
			};
			// With no overlap volume the only evidence is the surface gap. A gap of
			// exactly 0 on faceted operands is CONTACT within the faceting, not proof
			// of interpenetration, so it is reported as such rather than as a boolean.
			let interfering = overlap.map(|v| v > 1e-9).unwrap_or(distance < 1e-6);
			let mut m = json!({
				"distance": distance,
				"interfering": interfering,
				"overlap_volume": overlap,
				"coincident_fit_hazard": hazard,
				"tol": tol,
				"provenance": "faceted",
				"source": [ta.source(), tb.source()],
			});
			if let Some(r) = reason {
				m["overlap_volume_reason"] = json!(r);
			}
			Ok(Outcome::measures(m))
		}
		OpKind::Describe { name } => {
			// M3: self-describe the op surface from the single authoritative catalogue (discover.rs),
			// which is compile-forced complete via op_tag — the list cannot drift from what runs.
			// With `name`, a real op also gets its parameter specs (generated OP_PARAMS table);
			// no-arg describe stays names+count (the full 139-op param dump would be huge) and
			// advertises `params_available` so callers know to ask per-op.
			match name {
				Some(n) => {
					let params = crate::discover::op_params(&n);
					let mut m = json!({ "name": n, "exists": params.is_some() });
					if let Some(specs) = params {
						// The generated per-op table PLUS the universal params every op
						// accepts, so `describe` is the complete answer to "what may I
						// pass here" — a param advertised nowhere is a param nobody uses.
						let mut list: Vec<Value> = specs
							.iter()
							.map(|p| {
								let mut spec = json!({ "name": p.name, "type": p.ty, "required": p.required, "doc": p.doc });
								if !p.aliases.is_empty() {
									spec["aliases"] = json!(p.aliases);
								}
								spec
							})
							.collect();
						list.push(universal_require_param());
						m["params"] = Value::Array(list);
					}
					Ok(Outcome::measures(m))
				}
				None => Ok(Outcome::measures(json!({
					"count": crate::discover::OP_COUNT,
					"ops": crate::discover::OP_NAMES,
					"params_available": true,
					"universal_params": [universal_require_param()],
				}))),
			}
		}
		OpKind::ListFaces { input } => {
			// M4 loop: enumerate faces as references (analytic descriptor + a witness point), read
			// from the existing kernel topology — no build, no geometry change.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let faces: Vec<Value> = s
				.faces()
				.enumerate()
				.map(|(i, fid)| {
					let (kind, descriptor) = match s.face(fid).surface {
						kernel_brep::Surface::Plane { origin, normal } => ("plane", json!({ "normal": v3a(normal), "point": v3a(origin) })),
						kernel_brep::Surface::Cylinder { origin, axis, radius } => ("cylinder", json!({ "axis": v3a(axis), "point": v3a(origin), "radius": radius })),
						kernel_brep::Surface::Sphere { center, radius } => ("sphere", json!({ "center": v3a(center), "radius": radius })),
						kernel_brep::Surface::Cone { apex, axis, half_angle } => ("cone", json!({ "apex": v3a(apex), "axis": v3a(axis), "half_angle": half_angle })),
						kernel_brep::Surface::Torus { center, axis, major, minor } => ("torus", json!({ "center": v3a(center), "axis": v3a(axis), "major": major, "minor": minor })),
					};
					let poly = s.face_polygon(fid);
					let area = if kind == "plane" { Some(polygon_area(&poly)) } else { None };
					json!({ "index": i, "type": kind, "descriptor": descriptor, "witness": v3a(polygon_centroid(&poly)), "area": area })
				})
				.collect();
			Ok(Outcome::measures(json!({ "count": faces.len(), "faces": faces })))
		}
		OpKind::ListEdges { input } => {
			// M4 loop: enumerate edges as references (midpoint witness + chord length).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let edges: Vec<Value> = s
				.edges()
				.enumerate()
				.map(|(i, eid)| {
					let he = s.half_edge(s.edge(eid).half_edge);
					let a = s.position(he.origin);
					let b = s.position(s.half_edge(he.next).origin);
					json!({ "index": i, "midpoint": v3a((a + b) * 0.5), "length": (a - b).length(), "curved": s.edge_curve(eid).is_some() })
				})
				.collect();
			Ok(Outcome::measures(json!({ "count": edges.len(), "edges": edges })))
		}

		_ => unreachable!("ops::measure: op routed to the wrong family"),
	}
}
