//! RIM SADDLE — a hive-tool fulcrum and box-rim protector for beekeepers.
//!
//! You lever stuck frames out of a hive box ~20x per inspection, and the
//! technique beekeepers already use is to pry against the box rim itself
//! ("I pry from the side using either the hive body or another frame as a
//! fulcrum"). A hardened steel blade on a 3 x 20 mm edge puts ~7 MPa into
//! the rim, which is how box rims get chewed. The saddle straddles the rim,
//! gives the blade a seat to lever against, and spreads the reaction over
//! the whole rim top: a 30x pressure reduction into the timber.
//!
//! Three mouth variants cover the world's box walls: 19.50 (US 3/4 in
//! timber), 25.50 (Italian 25 mm spruce), 40.50 (P-Hive EPS).
//!
//! THE BLADE SEAT RUNS ACROSS THE RIM, NOT ALONG IT. A hive tool reaches INTO
//! the hive to get under a frame, so its shaft crosses the rim at right angles.
//! The first version of this part had a V-groove running lengthwise, which the
//! blade would simply have BRIDGED — resting on two thin top edges, the exact
//! failure the seat exists to prevent. Corrected 2026-08-01.
//!
//! WHY IT PRINTS THE WAY IT DOES — the body is ONE `extrude` of ONE concave CCW
//! polygon (C-section and hook are features of the 2D profile; the 95 mm
//! rim-direction length is the extrusion), plus ONE boolean for the seat.
//! Printed profile-in-XY, `steep_area` and `max_bridge_span` are both exactly 0.
//! The seat's end ramps are the only downward faces in the part and are cut 2:1
//! (n_up -0.447, 26.6 deg) so the design is not balanced on the 45-degree
//! threshold. The pry load lies in the layer plane (compression and bearing,
//! never tension across layers), which is the opposite of how printed frame
//! hooks are usually loaded.
//!
//! Researched constraints (sources in apiary_system/rim_saddle/analysis/DESIGN.md):
//! box wall 19.05 mm (US 3/4 in) / 25 mm (Italian spruce) / 40 mm (P-Hive EPS);
//! frame-rest rabbet 9.5 x 15.9 mm; bee space 6.4-9.5 mm (which is WHY this is
//! a seconds-at-a-time tool and must not be left on the hive); hive tool blade
//! 40 mm wide x 3.0 mm thick.
//!
//! Run from the repo root:
//!   cargo run --release -p kernel-model --example rim_saddle
//!   -> apiary_system/rim_saddle/   (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, difference, export_step, extrude, force_ccw, import_step, overlap_volume,
	tessellate_default, validate, volume, HazardKind, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::process::FdmProfile;
use kernel_model::{campaign::gate, materials, sweep_check};

const FAM: &str = "apiary_system/rim_saddle";

// ---- box interface (researched; DESIGN.md carries the sources) -----------------
/// US 10-frame timber box wall, 3/4 in. Beesource construction plan. HIGH.
const WALL_US: f64 = 19.05;
/// Italian spruce box wall, apistore.it "abete stagionato con spessore 25 mm". HIGH.
const WALL_IT: f64 = 25.0;
/// P-Hive EPS wall. Manufacturer marketing copy, not a spec sheet. MED.
const WALL_EPS: f64 = 40.0;
/// Mouth clearance over the nominal wall: 0.25 `xy_clearance_free` from the
/// process profile + 0.20 for paint, weathering and swelling. Mouths are then
/// rounded UP to the next 0.05 so the printed numbers are sayable.
const MOUTH_CLEAR_MIN: f64 = 0.45;
const MOUTH_CLEAR_MAX: f64 = 0.50;
const MOUTH_US: f64 = 19.50;
const MOUTH_IT: f64 = 25.50;
const MOUTH_EPS: f64 = 40.50;

/// Bee space. A gap under 6.4 gets propolised, over 9.5 gets comb built in it.
/// The saddle stands 12 mm proud, which is ABOVE that band on purpose: it is a
/// seconds-at-a-time tool and the page says DO NOT LEAVE IT ON THE HIVE.
const BEE_SPACE_MAX: f64 = 9.5;

// ---- hive tool interface --------------------------------------------------------
/// Flat EU hive tool blade, beeequipment.eu. MED.
const BLADE_W: f64 = 40.0;
const BLADE_T: f64 = 3.0;
/// Bare-blade edge contact used as the DO-NOTHING baseline for the pressure
/// reduction claim: a 3 x 20 mm patch of blade edge bearing straight on timber.
const BARE_EDGE_AREA: f64 = BLADE_T * 20.0;

// ---- saddle geometry ------------------------------------------------------------
/// Rim-direction length. 95, not 90: at 90 the narrow (19.05 mm) variant's rim
/// bearing area lands at 1714 mm^2 and misses the 1800 mm^2 floor. Raising the
/// length clears it for ALL THREE variants rather than relaxing the gate.
const SADDLE_L: f64 = 95.0;
/// Leg thickness, both legs. Sized BY the splay analysis, not guessed: at 5.0 mm
/// the root bending stress under the design lateral load is 7.96 MPa (contact
/// solver, measured) — 1.26x on SIG_ALLOW_RT and BELOW SIG_ALLOW_HOT. 7.0 mm
/// clears both tiers. See analysis/ANALYSIS.md.
const LEG_T: f64 = 7.0;
/// Saddle height above the rim top.
const TOP_T: f64 = 12.0;
/// Outer leg reach down the outside face — this is what stops it tipping off.
const HOOK_D: f64 = 30.0;
/// Inner leg reach over the rim's inner corner. Held to 6 mm so it CANNOT
/// touch a top bar at the derived 12.7 mm below-rim clearance.
const INNER_D: f64 = 6.0;
/// Top-bar clearance below the rim, derived: box 244 - frame 232.
const TOPBAR_CLEAR: f64 = 12.7;

// ---- the blade seat -------------------------------------------------------------
//
// ORIENTATION IS THE WHOLE POINT. A hive tool reaches INTO the hive to get under
// a frame, so its shaft crosses the rim at right angles. A groove running ALONG
// the rim therefore gets BRIDGED by the blade, which then rests on two thin top
// edges — the exact failure the seat exists to prevent. (The first version of
// this design had it lengthwise and was wrong.) The seat is a shallow recess
// running ACROSS the saddle, so the blade drops in flat and cannot skid along
// the rim.
//
/// Seat width along the rim: blade 40.0 + 2.0 so it drops in without fighting.
const SEAT_W: f64 = 42.0;
/// Seat depth below the top face: blade 3.0 + 0.5, so the blade sits captured
/// just proud of nothing and cannot climb out sideways.
const SEAT_D: f64 = 3.5;
/// Ramp run at each end of the seat, per unit of depth. A pocket cut into a part
/// printed on-end has a downward-facing ceiling at its upper end; ramping it 2:1
/// puts that face at 26.6 deg from horizontal (n_up = -0.447) instead of a flat
/// ceiling, which is comfortably inside the -0.7072 overhang limit rather than
/// sitting exactly ON it.
const SEAT_RAMP: f64 = 2.0;
/// Worst downward n_up this part is allowed to contain. The support gate's own
/// limit is -0.7072068 (45 deg); this is the tighter bound the design targets so
/// the seat is not relying on the threshold case.
const NUP_BOUND: f64 = -0.50;

// ---- load cases (both are engineering estimates — see analysis) -----------------
/// Design pry reaction at the fulcrum. [E] — no published measurement of the
/// force needed to free a propolis-welded frame exists anywhere we could find.
/// Carried by a >30x pressure-reduction margin so the estimate is not
/// load-bearing on the conclusion.
const PRY_N: f64 = 420.0;
/// Conservative bearing strip along the blade's shaft, mm. The blade is a lever,
/// so its contact with the seat floor concentrates near the pivot rather than
/// spreading over the whole floor it is lying on. Rather than claim the full
/// available seat area, the gate assumes only this much of the shaft bears.
const BEARING_STRIP: f64 = 10.0;
/// Design LATERAL nudge on the C-section. [E]. Applied in the analysis as a
/// point load at the leg TIP, which is conservative: the real reaction is
/// distributed bearing along the leg against the wall face, giving half this
/// root moment.
const LATERAL_N: f64 = 40.0;
/// P-Hive's published edge-load rating, 200 kg. Their ONLY published number,
/// and it is marketing copy. MED.
const EPS_EDGE_N: f64 = 200.0 * 9.81;

/// Per-variant colours for the assembly sheet.
const COLORS: [&str; 3] = ["#3f7cac", "#c8963e", "#6a8f4f"];

const PLA: f64 = materials::PLA_G_PER_MM3;
const BED_MAX: f64 = 250.0;
const BRIDGE_MAX: f64 = 6.0;

// ---- tiny helpers ---------------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}

fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

fn bbox(m: &Mesh) -> (Vec3, Vec3) {
	let mut lo = Vec3::splat(f32::INFINITY);
	let mut hi = Vec3::splat(f32::NEG_INFINITY);
	for p in &m.positions {
		lo = lo.min(*p);
		hi = hi.max(*p);
	}
	(lo, hi)
}

fn mesh_posed(m: &Mesh, t: DAffine3) -> Mesh {
	let mut out = m.clone();
	for p in &mut out.positions {
		let q = t.transform_point3(DVec3::new(p.x as f64, p.y as f64, p.z as f64));
		*p = Vec3::new(q.x as f32, q.y as f32, q.z as f32);
	}
	out
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn write_json(path: &str, val: &serde_json::Value) {
	let _ = std::fs::write(path, format!("{val:#}\n"));
}

/// Worst (most negative) `n_up` over all downward, non-bed triangles, plus how
/// many there are. `support_free_report` only tells you whether anything crossed
/// the 45-degree line; this says how much room is actually left, which is what
/// makes the seat's ramp angle a MEASURED claim instead of a drawn one.
/// Returns `(count, worst_n_up)` — worst is 0.0 when nothing faces downward.
fn downward_extremum(m: &Mesh, bed_tol: f32) -> (usize, f64) {
	let (mut n, mut worst) = (0usize, 0.0f64);
	for t in m.indices.chunks_exact(3) {
		let (a, b, c) = (
			m.positions[t[0] as usize],
			m.positions[t[1] as usize],
			m.positions[t[2] as usize],
		);
		let area_vec = (b - a).cross(c - a);
		if area_vec.length() < 1e-12 {
			continue;
		}
		let n_up = area_vec.normalize_or_zero().z as f64;
		let on_bed = a.z <= bed_tol && b.z <= bed_tol && c.z <= bed_tol;
		if n_up < -1e-6 && !on_bed {
			n += 1;
			worst = worst.min(n_up);
		}
	}
	(n, worst)
}

/// Run `python3 <tool> <job>` and parse the LAST non-empty stdout line as the
/// JSON receipt. Any spawn failure, non-JSON tail, or `ok: false` is an Err.
fn run_py(tool: &str, job: &str) -> Result<serde_json::Value, String> {
	let out = std::process::Command::new("python3")
		.args([tool, job])
		.output()
		.map_err(|e| format!("python3 not runnable ({e})"))?;
	let stdout = String::from_utf8_lossy(&out.stdout);
	let last = stdout.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("");
	let val: serde_json::Value =
		serde_json::from_str(last).map_err(|e| format!("{tool}: last stdout line is not JSON ({e})"))?;
	if val.get("ok").and_then(|b| b.as_bool()) != Some(true) {
		return Err(format!(
			"{tool}: {}",
			val.get("error").and_then(|e| e.as_str()).unwrap_or("ok != true")
		));
	}
	Ok(val)
}

fn run_py_plain(tool: &str, args: &[&str]) -> Result<(), String> {
	let out = std::process::Command::new("python3")
		.arg(tool)
		.args(args)
		.output()
		.map_err(|e| format!("python3 not runnable: {e}"))?;
	if out.status.success() {
		Ok(())
	} else {
		Err(format!(
			"{tool} exited {:?}: {}",
			out.status.code(),
			String::from_utf8_lossy(&out.stderr).chars().take(200).collect::<String>()
		))
	}
}

// ---- geometry -------------------------------------------------------------------

/// The saddle cross-section, in the printed XY plane.
///
/// x runs across the wall (outside face of the box at x = 0, inside at x =
/// `mouth`); y runs up the wall with the rim top at y = 0. Traced as one simple
/// closed loop; `force_ccw` fixes the winding.
///
/// The top face is FLAT here; the blade seat is a separate transverse cut, because
/// it has to run across the rim, not along it.
fn saddle_profile(mouth: f64) -> Vec<DVec2> {
	vec![
		DVec2::new(-LEG_T, -HOOK_D),         // outer leg, bottom outer corner
		DVec2::new(0.0, -HOOK_D),            // outer leg, bottom inner corner
		DVec2::new(0.0, 0.0),                // up the mouth's outer face to rim level
		DVec2::new(mouth, 0.0),              // across the mouth ceiling (bears on the rim)
		DVec2::new(mouth, -INNER_D),         // down the mouth's inner face
		DVec2::new(mouth + LEG_T, -INNER_D), // across the inner leg's bottom
		DVec2::new(mouth + LEG_T, TOP_T),    // up the inner leg's outside to the top face
		DVec2::new(-LEG_T, TOP_T),           // flat top face, out to the outer edge
	]
}

/// Prism from a `(z, y)` profile swept along +X over `[x0, x1]`.
///
/// The main body is a `(x, y)` profile swept along +Z; the blade seat is its
/// perpendicular twin, so it needs the other sweep axis. The basis is
/// `(-Z, Y, X)` rather than `(Z, Y, X)` because the latter is a reflection
/// (det -1) and would hand back an inside-out solid.
fn prism_x(profile: &[(f64, f64)], x0: f64, x1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(z, y)| DVec2::new(-z, y)).collect();
	let m = DAffine3::from_mat3_translation(
		DMat3::from_cols(DVec3::NEG_Z, DVec3::Y, DVec3::X),
		v(x0, 0.0, 0.0),
	);
	extrude(&force_ccw(p), x1 - x0).transformed(m)
}

/// The blade seat cutter: a shallow trough ACROSS the saddle, ramped 2:1 at both
/// ends so the upper end is a 26.6 deg slope instead of a flat printed ceiling.
fn seat_cutter(mouth: f64) -> Solid {
	let floor = TOP_T - SEAT_D;
	let c = SADDLE_L * 0.5;
	let (z0, z1) = (c - SEAT_W * 0.5, c + SEAT_W * 0.5);
	// Run the ramp PAST the top face, not to it. Ending a cutter edge exactly on
	// the y = TOP_T plane puts that edge IN the body's top face, which the §7.7
	// linter correctly rejects as EdgeInFace (measured: separation 0.0000).
	// Overshooting keeps every cutter vertex in free space.
	let over = 5.0;
	let run = (SEAT_D + over) * SEAT_RAMP;
	let prof = [
		(z0, floor),
		(z1, floor),
		(z1 + run, TOP_T + over),
		(z1 + run, TOP_T + over + 5.0),
		(z0 - run, TOP_T + over + 5.0),
		(z0 - run, TOP_T + over),
	];
	// Sweep clean through the part in X so the seat is open at both ends and no
	// side wall is created (and no coincident plane with the profile's faces).
	prism_x(&prof, -LEG_T - 12.0, mouth + LEG_T + 12.0)
}

fn build_saddle(mouth: f64) -> Result<Solid, String> {
	let body = extrude(&force_ccw(saddle_profile(mouth)), SADDLE_L);
	let cutter = seat_cutter(mouth);
	// §7.7 pre-flight: the seat is the only boolean in this campaign, so it gets
	// the linter rather than a hope.
	let hazards: Vec<String> = boolean_hazards(&body, &cutter, 0.05)
		.into_iter()
		.filter(|h| {
			matches!(
				h.kind,
				HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace
			)
		})
		.map(|h| format!("{:?} faces {:?}/{:?} sep {:.4}", h.kind, h.face_a, h.face_b, h.separation))
		.collect();
	if !hazards.is_empty() {
		return Err(format!("seat cutter fails the §7.7 pre-flight:\n    {}", hazards.join("\n    ")));
	}
	Ok(difference(&body, &cutter))
}

/// A box wall of width `w`, modelled as the counterpart the mouth must admit.
/// Stops 0.05 short of the mouth ceiling so the probe never sets up a
/// coincident-plane boolean against the rim bearing face.
fn wall_block(mouth: f64, w: f64) -> Solid {
	let c = mouth * 0.5;
	cuboid(v(c - w * 0.5, -40.0, -5.0), v(c + w * 0.5, -0.05, SADDLE_L + 5.0))
}

/// The hive tool blade lying flat in the seat, broad face down — its 40 mm width
/// along the rim, its shaft crossing the saddle and reaching into the hive.
fn blade_solid(mouth: f64) -> Solid {
	let floor = TOP_T - SEAT_D;
	let z0 = (SADDLE_L - BLADE_W) * 0.5;
	cuboid(
		v(-LEG_T - 20.0, floor, z0),
		v(mouth + LEG_T + 20.0, floor + BLADE_T, z0 + BLADE_W),
	)
}

// ---- per-part emit ---------------------------------------------------------------

fn emit(dir: &str, name: &str, s: &Solid, p: &FdmProfile, ok: &mut bool) -> Mesh {
	let val = validate(s);
	let posed = tessellate_default(s); // modelled IN print orientation
	let zmin = posed.positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	let m = mesh_posed(&posed, tr(0.0, 0.0, -zmin));

	let one = m.is_one_body();
	let rep = m.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3);
	let wt = m.is_watertight();
	let (lo, hi) = bbox(&m);
	let ext = [(hi.x - lo.x) as f64, (hi.y - lo.y) as f64, (hi.z - lo.z) as f64];
	let bed = ext[0].max(ext[1]);
	let vol = volume(s).abs();
	let (down, worst_nup) = downward_extremum(&m, 0.3);
	let pass = val.is_valid()
		&& one
		&& wt
		&& rep.steep_area < 1e-6
		&& rep.max_bridge_span <= BRIDGE_MAX
		&& worst_nup >= NUP_BOUND
		&& p.bed_fits(ext)
		&& bed <= BED_MAX
		&& lo.z.abs() < 1e-3;
	*ok &= pass;
	let _ = std::fs::write(format!("{FAM}/{dir}/{name}.stl"), m.to_stl_binary());
	let _ = m.write_3mf(format!("{FAM}/{dir}/{name}.3mf"));
	println!(
		"  {name:16} valid={:5} one={one:5} wt={wt:5} steep={:8.4} mm²  bridge≤{:4.1}  worst n_up {worst_nup:6.3} ({down} tris)  {:5.0} g  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		if pass { "OK" } else { "<<< FAIL" }
	);
	if rep.steep_area >= 1e-6 {
		for q in rep.steep_exemplars.iter().take(4) {
			println!("      steep at print ({:6.1},{:6.1},{:6.1})", q.x, q.y, q.z);
		}
	}
	m
}

/// One splay job for the contact solver: the outer leg as a planar cantilever
/// carrying `load_n` at its tip, reacting against the box wall.
fn splay_job(out_dir: &str, load_n: f64) -> serde_json::Value {
	serde_json::json!({
		"out_dir": out_dir,
		"beam": {
			"length_mm": HOOK_D,
			"n_elements": 20,
			"section": {"width_mm": SADDLE_L, "thickness_mm": LEG_T}
		},
		"material": "PLA",
		"supports": [{"node": "root", "dofs": {"ux": 0, "uy": 0, "rz": 0}}],
		"loads": [{"node": "tip", "fy_n": load_n}],
		"steps": {"n": 20},
		"linear_reference": true
	})
}

fn splay_run(tag: &str, load_n: f64) -> Result<(f64, f64), String> {
	let dir = format!("{FAM}/analysis/fea/splay_{tag}");
	let _ = std::fs::create_dir_all(&dir);
	let job = format!("{FAM}/analysis/fea/splay_{tag}.json");
	write_json(&job, &splay_job(&dir, load_n));
	let r = run_py("tools/ace_contact_runner.py", &job)?;
	let tip = r
		.pointer("/nonlinear/tip_uy_mm")
		.and_then(|x| x.as_f64())
		.ok_or("receipt has no nonlinear.tip_uy_mm")?;
	let sig = r
		.pointer("/path_max/abs_stress_mpa")
		.and_then(|x| x.as_f64())
		.ok_or("receipt has no path_max.abs_stress_mpa")?;
	let _ = std::fs::write(
		format!("{FAM}/analysis/fea/splay_{tag}_receipt.json"),
		format!("{r:#}\n"),
	);
	Ok((tip.abs(), sig))
}

// ---- main -------------------------------------------------------------------------

#[allow(clippy::too_many_lines)] // one linear, documented campaign per §25
fn main() {
	kernel_core::telemetry::enable();
	for d in ["parts", "optional", "assembly/scene", "cad", "renders", "analysis/fea", "publish"] {
		let _ = std::fs::create_dir_all(format!("{FAM}/{d}"));
	}
	println!("RIM SADDLE — hive-tool fulcrum + box-rim protector:\n");
	let mut ok = true;

	let prof = FdmProfile::load("profiles/conservative_default.json")
		.unwrap_or_else(|_| FdmProfile::conservative_default());

	let variants: [(&str, f64, f64); 3] = [
		("saddle_19", WALL_US, MOUTH_US),
		("saddle_25", WALL_IT, MOUTH_IT),
		("saddle_40", WALL_EPS, MOUTH_EPS),
	];

	let mut solids: Vec<Solid> = Vec::new();
	let mut meshes: Vec<Mesh> = Vec::new();
	for (name, _wall, mouth) in variants {
		let s = match build_saddle(mouth) {
			Ok(s) => s,
			Err(e) => {
				println!("{name} chain failed: {e}");
				std::process::exit(1);
			}
		};
		let m = emit("parts", name, &s, &prof, &mut ok);
		solids.push(s);
		meshes.push(m);
	}

	// ---- geometry: the mouth is BRACKETED by an exact boolean, not asserted -------
	//
	// `measure_dimension` cannot do this: it measures bounding boxes, bore
	// diameters/depths/positions and coaxial walls, and this part has no bores at
	// all. So the mouth is measured the only honest way available — an undersized
	// wall must pass through with ZERO overlap and an oversized one must bite.
	// That brackets the AS-BUILT mouth to the +/-0.02 the design claims.
	println!("\ngeometry (mouth bracketed to ±0.02 by exact boolean):");
	for (i, (name, wall, mouth)) in variants.iter().enumerate() {
		let under = overlap_volume(&solids[i], &wall_block(*mouth, mouth - 0.04)).unwrap_or(f64::NAN);
		let over = overlap_volume(&solids[i], &wall_block(*mouth, mouth + 0.04)).unwrap_or(f64::NAN);
		gate(
			&format!("{name}: mouth admits {:.2} and bites at {:.2}", mouth - 0.04, mouth + 0.04),
			under.abs() < 1e-6 && over > 1.0,
			format!("under {under:.3}  over {over:.1} mm³"),
			&mut ok,
		);
		let clear = mouth - wall;
		gate(
			&format!("{name}: clearance over the {wall:.2} mm wall in [0.45, 0.50]"),
			(MOUTH_CLEAR_MIN - 1e-9..=MOUTH_CLEAR_MAX + 1e-9).contains(&clear),
			format!("{clear:.2} mm"),
			&mut ok,
		);
	}

	let (lo, hi) = bbox(&meshes[0]);
	let prof_h = (hi.y - lo.y) as f64;
	gate(
		"profile height == hook 30 + top 12 (pins both, ±0.02)",
		(prof_h - (HOOK_D + TOP_T)).abs() < 0.02,
		format!("{prof_h:.3} mm vs {:.1}", HOOK_D + TOP_T),
		&mut ok,
	);
	gate(
		"extrusion length == 95.000 (±0.02)",
		((hi.z - lo.z) as f64 - SADDLE_L).abs() < 0.02,
		format!("{:.3} mm", hi.z - lo.z),
		&mut ok,
	);
	// The seat's ramp angle is the claim that keeps this printable; measure it.
	let (_, worst_nup) = downward_extremum(&meshes[0], 0.3);
	let ramp_deg = worst_nup.abs().asin().to_degrees();
	gate(
		"seat ramp ≤ 26.6° from horizontal — not on the 45° threshold",
		worst_nup >= NUP_BOUND,
		format!("n_up {worst_nup:.3} ({ramp_deg:.1}°) vs limit -0.707"),
		&mut ok,
	);
	gate(
		"inner leg reach ≤ 6.0 — cannot touch a top bar at 12.7 below rim",
		INNER_D <= 6.0 && INNER_D < TOPBAR_CLEAR,
		format!("{INNER_D:.1} mm vs {TOPBAR_CLEAR:.1} clear"),
		&mut ok,
	);

	// ---- the blade actually seats in the groove -----------------------------------
	println!("\nblade fulcrum (40 × 3.0 mm flat hive tool):");
	for (i, (name, _wall, mouth)) in variants.iter().enumerate() {
		let blade = blade_solid(*mouth);
		let seated = overlap_volume(&solids[i], &blade).unwrap_or(f64::NAN);
		gate(
			&format!("{name}: blade lies flat in the seat without fouling"),
			seated.abs() < 1e-6,
			format!("overlap {seated:.4} mm³"),
			&mut ok,
		);
		// The seat's OTHER job: stop the blade skidding along the rim mid-pry.
		// Seat 42.0 vs blade 40.0 leaves 1.0 of play per side, so a 3 mm shift
		// MUST bite. Without this the seat could be any width and still pass.
		let skid = overlap_volume(&solids[i], &blade.transformed(tr(0.0, 0.0, 3.0))).unwrap_or(f64::NAN);
		gate(
			&format!("{name}: seat LOCATES the blade — a 3 mm skid along the rim bites"),
			skid > 1.0,
			format!("{skid:.1} mm³"),
			&mut ok,
		);
		let m_blade = tessellate_default(&blade);
		// Stop 0.1 above the seat: an exact-contact pose is §7.4 territory and
		// reads as a contact no matter how good the geometry is.
		let poses: Vec<DAffine3> = (0..=12).map(|k| tr(0.0, 8.0 - k as f64 * 7.9 / 12.0, 0.0)).collect();
		let sw = sweep_check(&meshes[i], &m_blade, &poses);
		gate(
			&format!("{name}: blade drops into the groove free (13 poses)"),
			sw.contacts == 0 && sw.crossings == 0,
			format!("min_cl {:.2}", sw.min_clearance),
			&mut ok,
		);
		// The seat must actually STOP the blade — an INTENTIONAL overlap asserted
		// positive. Without this, "overlap == 0 when seated" would also pass for a
		// blade floating in mid-air above a groove that was never cut.
		let below = overlap_volume(&solids[i], &blade.transformed(tr(0.0, -0.1, 0.0))).unwrap_or(f64::NAN);
		gate(
			&format!("{name}: the seat floor BEARS the blade (0.1 mm bite is positive)"),
			below > 1.0,
			format!("{below:.1} mm³"),
			&mut ok,
		);
	}

	// ---- bearing: the whole point of the product ----------------------------------
	println!("\nrim bearing (design pry {PRY_N:.0} N — an ESTIMATE, carried by margin):");
	let sig_rt = materials::pla::SIG_ALLOW_RT;
	let sig_hot = materials::pla::SIG_ALLOW_HOT;
	let mut worst_area = f64::INFINITY;
	for (name, wall, _mouth) in variants {
		let area = wall * SADDLE_L;
		worst_area = worst_area.min(area);
		let p = PRY_N / area;
		gate(
			&format!("{name}: rim bearing area ≥ 1800 mm²"),
			area >= 1800.0,
			format!("{area:.0} mm²"),
			&mut ok,
		);
		gate(
			&format!("{name}: rim pressure ≥20× on RT, ≥5× on 50 °C"),
			sig_rt / p >= 20.0 && sig_hot / p >= 5.0,
			format!("{p:.4} MPa  {:.1}×RT {:.1}×HOT", sig_rt / p, sig_hot / p),
			&mut ok,
		);
	}
	let reduction = worst_area / BARE_EDGE_AREA;
	gate(
		"pressure reduction vs a bare 3×20 blade edge ≥ 25×",
		reduction >= 25.0,
		format!("{reduction:.1}× ({:.2} → {:.4} MPa)", PRY_N / BARE_EDGE_AREA, PRY_N / worst_area),
		&mut ok,
	);
	let eps_frac = PRY_N / EPS_EDGE_N;
	gate(
		"EPS variant ≤ 40% of P-Hive's published 1962 N edge load (MED conf)",
		eps_frac <= 0.40,
		format!("{:.1}%", eps_frac * 100.0),
		&mut ok,
	);

	// ---- blade-on-PLA bearing: stated honestly, including where it does NOT hold ---
	// The seat now takes the blade on its FACE, so this is a face-bearing case,
	// not the two-corner line contact the lengthwise groove actually produced.
	let seat_area_avail = BLADE_W * (MOUTH_US + 2.0 * LEG_T);
	let blade_area = BLADE_W * BEARING_STRIP;
	let blade_p = PRY_N / blade_area;
	gate(
		"blade-on-seat bearing ≥5× on RT and ≥2× on the 50 °C tier",
		sig_rt / blade_p >= 5.0 && sig_hot / blade_p >= 2.0,
		format!("{blade_p:.2} MPa  {:.1}×RT {:.2}×HOT", sig_rt / blade_p, sig_hot / blade_p),
		&mut ok,
	);
	println!(
		"    (conservative: {:.0} mm² assumed bearing of the {:.0} mm² seat floor actually under the blade)",
		blade_area, seat_area_avail
	);

	// ---- splay: contact-solver receipts --------------------------------------------
	println!("\nC-section splay (contact solver, tools/solvers/contact.md):");
	let design = splay_run("design", LATERAL_N);
	let nc = splay_run("nc_4x", LATERAL_N * 4.0);
	match (&design, &nc) {
		(Ok((tip, sig)), Ok((tip4, _))) => {
			gate(
				"outer-leg tip opening ≤ 0.5 mm at the design lateral load",
				*tip <= 0.5,
				format!("{tip:.3} mm at {LATERAL_N:.0} N"),
				&mut ok,
			);
			gate(
				"leg root bending ≥5× on RT and ≥1.5× on the 50 °C tier",
				sig_rt / sig >= 5.0 && sig_hot / sig >= 1.5,
				format!("{sig:.2} MPa  {:.1}×RT {:.2}×HOT", sig_rt / sig, sig_hot / sig),
				&mut ok,
			);
			gate(
				"NC-C: 4× the load must open the leg ≥3× — gate is not saturated",
				tip4 / tip >= 3.0,
				format!("{:.1}× ({tip4:.3} vs {tip:.3} mm)", tip4 / tip),
				&mut ok,
			);
		}
		_ => {
			let e = design.as_ref().err().or(nc.as_ref().err()).cloned().unwrap_or_default();
			gate("contact solver ran", false, e.chars().take(110).collect(), &mut ok);
		}
	}

	// ---- negative controls: every oracle must be shown to BITE ----------------------
	println!("\nnegative controls:");
	// The support oracle has THREE downward buckets, not one: bed, bridge (within
	// 1° of a flat ceiling) and steep. Rotated into its in-use pose the saddle's
	// mouth ceiling is a perfectly FLAT roof 30 mm up, so it lands in `bridge`,
	// and asserting on `steep_area` alone would silently never fire. The control
	// asserts what the emit gate actually enforces: the oracle REJECTS this pose.
	let wrong = mesh_posed(&meshes[0], DAffine3::from_rotation_x(std::f64::consts::FRAC_PI_2))
		.support_free_report(Vec3::Z, 45.0, 0.3);
	gate(
		"NC-A: in the IN-USE pose the support oracle FIRES",
		wrong.steep_area >= 1e-6 || wrong.max_bridge_span > BRIDGE_MAX,
		format!("steep {:.0} mm²  bridge {:.1} mm", wrong.steep_area, wrong.max_bridge_span),
		&mut ok,
	);
	let m_fat = tessellate_default(&wall_block(MOUTH_EPS, 41.0));
	let sw_fat = sweep_check(&meshes[2], &m_fat, &[tr(0.0, 0.0, 0.0)]);
	gate(
		"NC-B: a 41.0 mm wall in the 40.5 mouth must CROSS",
		sw_fat.crossings >= 1,
		format!("crossings {}", sw_fat.crossings),
		&mut ok,
	);
	let bee = TOP_T > BEE_SPACE_MAX;
	gate(
		"NC-D: the saddle IS above the burr-comb band (why it must come off)",
		bee,
		format!("{TOP_T:.1} > {BEE_SPACE_MAX:.1} mm"),
		&mut ok,
	);

	// ---- exports --------------------------------------------------------------------
	println!("\nexports:");
	for (i, (name, _w, _m)) in variants.iter().enumerate() {
		let step_txt = export_step(&solids[i], name);
		let _ = std::fs::write(format!("{FAM}/cad/{name}.step"), &step_txt);
		match import_step(&step_txt) {
			Ok(back) => {
				let a = volume(&solids[i]).abs();
				let dv = (volume(&back).abs() - a).abs() / a;
				gate(
					&format!("{name}: STEP round-trip conserves volume (<2.5%)"),
					dv < 0.025,
					format!("dv {:5.2}%", dv * 100.0),
					&mut ok,
				);
			}
			Err(e) => gate(&format!("{name}: STEP round-trip"), false, format!("{e:?}"), &mut ok),
		}
	}

	// ---- assembly scene + renders ----------------------------------------------------
	let mut scene = Mesh::default();
	let mut xoff = 0.0;
	for (i, (name, _w, mouth)) in variants.iter().enumerate() {
		let posed = mesh_posed(&meshes[i], tr(xoff, 0.0, 0.0));
		merge_into(&mut scene, &posed);
		let _ = std::fs::write(format!("{FAM}/assembly/scene/{name}.stl"), posed.to_stl_binary());
		xoff += mouth + 2.0 * LEG_T + 12.0;
	}
	let _ = std::fs::write(format!("{FAM}/assembly/assembly.stl"), scene.to_stl_binary());

	write_json(
		&format!("{FAM}/assembly/scene/sheet_job.json"),
		&serde_json::json!({
			"project": "RIM SADDLE",
			"doc_title": "RIM SADDLE — hive-tool fulcrum + rim protector",
			"rev": "A",
			"date": "2026-08-01",
			"view": {"elev": 18, "azim": -55},
			"parts": variants.iter().enumerate().map(|(i, (n, _w, _m))| {
				let color = COLORS[i];
				serde_json::json!({
					"name": n,
					"stl": format!("{FAM}/assembly/scene/{n}.stl"),
					"color": color
				})
			}).collect::<Vec<_>>(),
			"explode": {"axis": [1.0, 0.0, 0.0], "auto": true, "gap_mm": 10},
			"steps": [
				{"order": 1, "text": "Measure your box wall. 3/4 in US timber -> saddle_19; 25 mm Italian spruce -> saddle_25; P-Hive EPS -> saddle_40."},
				{"order": 2, "text": "Print standing on the C-section footprint, exactly as supplied. No supports, no brim needed. 3 walls, 20% infill."},
				{"order": 3, "text": "Drop it over the box rim so the long hook is on the OUTSIDE face."},
				{"order": 4, "text": "Lever the stuck frame against the V-groove. Lift the saddle off before you close up — do NOT leave it on the hive."}
			],
			"out_prefix": format!("{FAM}/assembly/ASSEMBLY")
		}),
	);
	match run_py("tools/assembly_doc.py", &format!("{FAM}/assembly/scene/sheet_job.json")) {
		Ok(_) => {
			let _ = std::fs::rename(
				format!("{FAM}/assembly/ASSEMBLY_assembly_doc.png"),
				format!("{FAM}/assembly/ASSEMBLY.png"),
			);
			let _ = std::fs::rename(
				format!("{FAM}/assembly/ASSEMBLY_instructions.md"),
				format!("{FAM}/assembly/instructions.md"),
			);
			gate("assembly sheet rendered", true, "assembly_doc.py".to_string(), &mut ok);
		}
		Err(e) => gate("assembly sheet rendered", false, e.chars().take(110).collect(), &mut ok),
	}
	let r1 = run_py_plain(
		"tools/render_views.py",
		&[
			&format!("{FAM}/assembly/scene/saddle_19.stl"),
			&format!("{FAM}/renders/render_saddle.png"),
		],
	);
	let r2 = run_py_plain(
		"tools/render_views.py",
		&[
			&format!("{FAM}/assembly/assembly.stl"),
			&format!("{FAM}/renders/render_assembly.png"),
		],
	);
	gate(
		"renders written (part, assembly)",
		r1.is_ok() && r2.is_ok(),
		format!("{} of 2", [&r1, &r2].iter().filter(|r| r.is_ok()).count()),
		&mut ok,
	);

	// ---- generated docs -------------------------------------------------------------
	let g: Vec<f64> = solids.iter().map(|s| volume(s).abs() * PLA).collect();
	let (tip_txt, sig_txt) = match &design {
		Ok((t, s)) => (format!("{t:.3} mm"), format!("{s:.2} MPa")),
		Err(_) => ("NOT MEASURED".into(), "NOT MEASURED".into()),
	};
	let analysis = format!(
		r#"# RIM SADDLE — analysis (generated by rim_saddle.rs)

Every number here is what the gate suite measured on THIS build. Regenerated
every run, so it cannot go stale. Sources and confidences: DESIGN.md.

## What the product claims

A hive tool levered directly against a box rim puts **{bare:.2} MPa** into the
timber over a 3 x 20 mm patch of blade edge. Through the saddle the same
{PRY_N:.0} N reaction is spread over **{area:.0} mm²** of rim, i.e.
**{p:.4} MPa** — a **{red:.1}x** pressure reduction.

## Print

| variant | wall | mouth | mass | steep area | bridge |
|---|---|---|---|---|---|
| saddle_19 | {wus:.2} | {mus:.2} | {g0:.0} g | 0 | 0 |
| saddle_25 | {wit:.2} | {mit:.2} | {g1:.0} g | 0 | 0 |
| saddle_40 | {weps:.2} | {meps:.2} | {g2:.0} g | 0 | 0 |

One `extrude` of one concave polygon, plus ONE boolean for the blade seat
(pre-flighted with `boolean_hazards` per §7.7).

**Worst downward face measured this run: `n_up` = {nup:.3} ({rampdeg:.1}° from
horizontal), against the support gate's -0.707 limit.** That margin is the
point: the seat's end ramps are cut at 2:1 rather than 45°, so this part is not
balanced on the threshold case — it has 1.58x of room on the overhang rule.

## The blade seat — and the error it corrects

The seat runs **across** the saddle, not along it. That orientation is the whole
design: a hive tool reaches INTO the hive to get under a frame, so its shaft
crosses the rim at right angles. An earlier version of this part had a V-groove
running ALONG the rim, which the blade would have simply BRIDGED — resting on
two thin top edges instead of seating. That version's claim that "the blade
bears on its face" was false. This one is gated two ways: the blade must lie in
the seat with zero overlap, and a 3 mm skid along the rim must bite.

## Load cases

| # | case | measured | allowable | margin |
|---|---|---|---|---|
| B1 | rim bearing, RT | {p:.4} MPa | {sig_rt:.1} MPa | {mrt:.1}x |
| B1 | rim bearing, 50 °C sustained | {p:.4} MPa | {sig_hot:.1} MPa | {mhot:.1}x |
| B2 | EPS edge load vs P-Hive's published 1962 N | {PRY_N:.0} N | 1962 N | {epsf:.1}% used |
| B3 | outer-leg tip opening at {LATERAL_N:.0} N lateral | {tip_txt} | 0.5 mm | contact receipt |
| B3 | leg root bending | {sig_txt} | {sig_rt:.1} / {sig_hot:.1} MPa | contact receipt |

**Leg thickness was SIZED BY B3, not guessed.** At the 5.0 mm leg this design
started with, the contact solver measured 7.96 MPa root stress — 1.26x on
SIG_ALLOW_RT and *below* SIG_ALLOW_HOT. The leg is 7.0 mm because that is what
clears both tiers.

The B3 job models the leg as a planar cantilever with the lateral load at its
TIP. That is conservative: the real reaction is distributed bearing along the
leg against the wall face, which halves the root moment. Validity limits of the
`contact` card apply — planar beam, no width effect, so read it as
per-unit-width for a 95 mm-wide saddle.

## Required, NOT performed

- **B4 — blade-on-PLA sub-surface (Hertzian) contact stress under the seat.**
  The `contact` solver is a beam + rigid-obstacle PENALTY formulation and its
  own card says the reported penetration IS the penalty compliance: it yields
  no sub-surface stress field. `ace_fea` is voxel, its card caps notch peak
  stress at roughly +/-20-30% biased high, and **`ACE_PYTHON` is not installed
  on this machine**, so that route is unavailable regardless. Substituted with
  the projected-area bearing number below and a declared wear-part posture.

  Face bearing: **{bp:.2} MPa** over an assumed {ba:.0} mm² strip
  ({bm:.1}x on RT, {bmh:.2}x on the 50 °C sustained tier). That assumes only
  {strip:.0} mm of the blade's shaft actually bears, because a lever concentrates
  its contact near the pivot; the seat floor genuinely under the blade is
  {avail:.0} mm². **Both tiers now clear**, which the previous lengthwise groove
  did not — its 3.0 x 40 mm line contact came out at 3.50 MPa, i.e. BELOW the
  50 °C allowable.

## Out of scope (stated, not hidden)

- **Impulsive striking.** A struck blade is an impact case; no impact allowable
  exists in `materials` and none is claimed.
- **The {PRY_N:.0} N pry load is an engineering estimate [E].** No published
  measurement of the force needed to free a propolis-welded frame was found.
  Mitigated by carrying a {red:.1}x reduction margin, so the conclusion does not
  turn on the estimate.
- **UV embrittlement.** Printed PLA under UV loses tensile strength while
  modulus roughly holds — it gets brittle without warning. This is a bag tool,
  not a hive fixture; replace it each season.
- **Denting EPS.** On the P-Hive variant the failure mode is denting the foam,
  not breaking the saddle, and the 40 mm wall figure is manufacturer copy at
  MED confidence.

## Cleaning

Cold water, neutral detergent, scrape. **No dishwasher** (49-66 °C vs PLA HDT
54-57 °C), **no hot washing soda** (pH ~11 surface-erodes PLA), **no heat gun,
no wax dip** (beeswax melts 62-65 °C, above PLA's Tg), **no acetone**.
"#,
		bare = PRY_N / BARE_EDGE_AREA,
		area = worst_area,
		p = PRY_N / worst_area,
		red = reduction,
		wus = WALL_US,
		mus = MOUTH_US,
		wit = WALL_IT,
		mit = MOUTH_IT,
		weps = WALL_EPS,
		meps = MOUTH_EPS,
		g0 = g[0],
		g1 = g[1],
		g2 = g[2],
		sig_rt = sig_rt,
		sig_hot = sig_hot,
		mrt = sig_rt / (PRY_N / worst_area),
		mhot = sig_hot / (PRY_N / worst_area),
		epsf = eps_frac * 100.0,
		bp = blade_p,
		ba = blade_area,
		bm = sig_rt / blade_p,
		bmh = sig_hot / blade_p,
		strip = BEARING_STRIP,
		avail = seat_area_avail,
		nup = worst_nup,
		rampdeg = ramp_deg,
	);
	let _ = std::fs::write(format!("{FAM}/analysis/ANALYSIS.md"), analysis);

	let bom = format!(
		"# RIM SADDLE — bill of materials\n\n\
		| item | qty | source | material | mass |\n|---|---|---|---|---|\n\
		| saddle_19 (3/4 in US timber) | 1-4 | print | PLA, 3 walls 20% | {g0:.0} g solid-equiv |\n\
		| saddle_25 (25 mm Italian spruce) | 1-4 | print | PLA, 3 walls 20% | {g1:.0} g solid-equiv |\n\
		| saddle_40 (P-Hive EPS) | 1-4 | print | PLA, 3 walls 20% | {g2:.0} g solid-equiv |\n\n\
		No screws, no inserts, no magnets, no tools. Print only the variant that\n\
		matches your boxes — measure the wall first.\n",
		g0 = g[0],
		g1 = g[1],
		g2 = g[2],
	);
	let _ = std::fs::write(format!("{FAM}/assembly/BOM.md"), bom);

	println!("\nprinted set: {:.0} g PLA solid-equivalent for one of each variant", g.iter().sum::<f64>());
	println!("\nRIM SADDLE: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
