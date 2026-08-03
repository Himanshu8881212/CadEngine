// Copyright (c) LMCAD. Licensed under the MIT License.

//! `/api/catalog` — the standard-parts families behind the PARTS button.
//!
//! This is a **UI schema over the real kernel catalog** (`kernel_model::parts`
//! exposed as `kernel-api` ops, documented op-by-op in `API.md` §"Standard
//! parts catalog"): each family names the op, its parameters, defaults and
//! table bounds, and the front-end instantiates one by submitting an ordinary
//! work order to `/api/run` — there is no second build path. Defaults are the
//! executed `API.md` examples, so "insert with defaults" is always a part that
//! builds. The catalog test runs one family end-to-end to keep this honest.

use axum::Json;
use serde::Serialize;
use serde_json::{json, Value};

/// One parameter of a catalog family.
#[derive(Clone, Serialize)]
pub struct CatalogParam {
	/// JSON field name on the op.
	pub name: String,
	/// `"number"`, `"int"`, `"bool"` or `"string"`.
	pub kind: String,
	/// Whether the op requires it.
	pub required: bool,
	/// Default value (an executed-example value).
	pub default: Value,
	/// Lower bound, when the standard's table has one.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub min: Option<f64>,
	/// Upper bound, when the standard's table has one.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub max: Option<f64>,
	/// The exact stocked values, when the parameter is table-bound.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub options: Option<Vec<Value>>,
	/// Human meaning (units are mm unless stated).
	pub meaning: String,
}

/// One standard-parts family.
#[derive(Clone, Serialize)]
pub struct CatalogFamily {
	/// The `kernel-api` op name (`{"op": ...}` in a work order).
	pub op: String,
	/// Display title.
	pub title: String,
	/// Display category (groups the browser).
	pub category: String,
	/// One-line description (standard + honesty notes live in API.md).
	pub summary: String,
	/// Parameter schema.
	pub params: Vec<CatalogParam>,
}

fn p(name: &str, kind: &str, required: bool, default: Value, meaning: &str) -> CatalogParam {
	CatalogParam { name: name.into(), kind: kind.into(), required, default, min: None, max: None, options: None, meaning: meaning.into() }
}

fn num(name: &str, default: f64, min: f64, max: f64, meaning: &str) -> CatalogParam {
	CatalogParam { min: Some(min), max: Some(max), ..p(name, "number", true, json!(default), meaning) }
}

fn int(name: &str, default: i64, min: f64, max: f64, meaning: &str) -> CatalogParam {
	CatalogParam { min: Some(min), max: Some(max), ..p(name, "int", true, json!(default), meaning) }
}

fn table(name: &str, kind: &str, default: Value, options: &[Value], meaning: &str) -> CatalogParam {
	CatalogParam { options: Some(options.to_vec()), ..p(name, kind, true, default, meaning) }
}

fn m_sizes(default: f64, sizes: &[f64]) -> CatalogParam {
	let options: Vec<Value> = sizes.iter().map(|s| json!(s)).collect();
	table("m", "number", json!(default), &options, "nominal thread size (the 10 of M10)")
}

/// The catalog: every family the PARTS browser offers. Op names, parameter
/// names, defaults and table bounds mirror `API.md` §"Standard parts catalog".
pub fn families() -> Vec<CatalogFamily> {
	let fam = |op: &str, title: &str, category: &str, summary: &str, params: Vec<CatalogParam>| CatalogFamily {
		op: op.into(),
		title: title.into(),
		category: category.into(),
		summary: summary.into(),
		params,
	};
	let m38 = &[3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0];
	let m312 = &[3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0];
	vec![
		// --- Gears & motion -----------------------------------------------------------
		fam("spur_gear", "Spur gear", "Gears & motion", "ISO 53 involute spur gear, bored, optional DIN 6885-1 keyway.", vec![
			num("module", 2.0, 0.5, 6.0, "gear module m (tooth size)"),
			int("teeth", 20, 8.0, 120.0, "tooth count z"),
			num("face_width", 10.0, 1.0, 60.0, "axial width"),
			num("bore", 8.0, 1.0, 60.0, "bore diameter (keep bore/2 + t2 < m(z/2 - 1.25))"),
			CatalogParam { required: false, min: Some(5.0), max: Some(30.0), ..p("pressure_angle_deg", "number", false, json!(20.0), "pressure angle, degrees") },
			p("keyway", "bool", false, json!(false), "cut the DIN 6885-1 hub keyway (bore must be in the 6-75 mm table)"),
		]),
		fam("gear_rack", "Gear rack", "Gears & motion", "ISO 53 / DIN 867 basic rack: straight flanks, whole teeth, pitch line y = 3m.", vec![
			num("module", 2.0, 0.5, 6.0, "module m"),
			num("length", 100.0, 10.0, 500.0, "bar length"),
			num("width", 10.0, 2.0, 60.0, "face width (extrusion)"),
			CatalogParam { required: false, min: Some(5.0), max: Some(30.0), ..p("pressure_angle_deg", "number", false, json!(20.0), "pressure angle, degrees") },
		]),
		fam("internal_gear", "Internal (ring) gear", "Gears & motion", "Involute tooth spaces cut into a rim bore; exact conjugate of a spur_gear pinion.", vec![
			num("module", 2.0, 0.5, 6.0, "module m"),
			int("teeth", 36, 8.0, 200.0, "ring tooth count"),
			num("face_width", 8.0, 1.0, 60.0, "axial width"),
			num("rim_od", 84.0, 10.0, 500.0, "rim outer diameter (> m*(teeth + 2.5))"),
		]),
		fam("gt2_pulley", "GT2 pulley", "Gears & motion", "GT2 2 mm-pitch timing pulley, optionally flanged.", vec![
			int("teeth", 20, 10.0, 80.0, "groove count"),
			num("belt_width", 6.0, 3.0, 15.0, "toothed band width"),
			num("bore", 5.0, 2.0, 15.0, "bore diameter"),
			p("flanged", "bool", false, json!(true), "add retaining flanges"),
		]),
		fam("chain_sprocket", "Chain sprocket", "Gears & motion", "ANSI B29.1 roller-chain sprocket plate (e.g. #25: pitch 6.35, roller 3.302).", vec![
			num("pitch", 6.35, 4.0, 20.0, "chain pitch P"),
			num("roller_d", 3.302, 2.0, 12.0, "nominal roller diameter Dr"),
			int("teeth", 12, 6.0, 60.0, "tooth count"),
			num("bore", 5.0, 2.0, 25.0, "bore diameter (keep well inside the root circle)"),
		]),
		// --- Fasteners ----------------------------------------------------------------
		fam("hex_bolt", "Hex bolt", "Fasteners", "ISO 4017 hex-head bolt body (threads not modelled - exact assembly envelope).", vec![
			m_sizes(10.0, m38),
			num("length", 30.0, 4.0, 200.0, "shank length"),
		]),
		fam("hex_nut", "Hex nut", "Fasteners", "ISO 4032 hex nut, bored at the nominal diameter.", vec![m_sizes(5.0, m38)]),
		fam("washer", "Washer", "Fasteners", "ISO 7089 plain washer (~DIN 125 A).", vec![m_sizes(5.0, m38)]),
		fam("spring_washer", "Spring washer", "Fasteners", "DIN 127 B split lock washer (free height 2s, 15 deg gap).", vec![m_sizes(5.0, m312)]),
		fam("socket_head_cap_screw", "Socket-head cap screw", "Fasteners", "DIN 912 / ISO 4762 body with the real hex socket pocket.", vec![
			m_sizes(5.0, m38),
			num("length", 16.0, 4.0, 150.0, "under-head shank length"),
		]),
		fam("flat_head_screw", "Flat-head screw", "Fasteners", "ISO 10642 countersunk socket screw (90 deg head); pairs with countersink_hole.", vec![
			m_sizes(5.0, m38),
			num("length", 16.0, 6.0, 150.0, "overall length (tip to head top)"),
		]),
		fam("button_head_screw", "Button-head screw", "Fasteners", "ISO 7380 button-head socket screw.", vec![
			m_sizes(5.0, m312),
			num("length", 16.0, 4.0, 120.0, "under-head shank length"),
		]),
		fam("set_screw", "Set screw (grub)", "Fasteners", "DIN 916 cup-point set screw: hex socket + cup recess.", vec![
			m_sizes(6.0, m312),
			num("length", 10.0, 3.0, 60.0, "overall length (must hold cup + socket + 0.5 mm web)"),
		]),
		fam("lock_nut", "Nyloc lock nut", "Fasteners", "DIN 985 nylon-insert lock nut body (note DIN widths: M10 -> 17 AF).", vec![m_sizes(10.0, m38)]),
		fam("threaded_rod", "Threaded rod", "Fasteners", "DIN 976-1 style studding with half-pitch end chamfers.", vec![
			m_sizes(8.0, m38),
			num("length", 60.0, 10.0, 1000.0, "rod length"),
		]),
		fam("standoff", "Hex standoff", "Fasteners", "Female-female hex standoff at the conventional wrench size.", vec![
			m_sizes(3.0, &[2.0, 2.5, 3.0, 4.0, 5.0, 6.0]),
			num("length", 12.0, 3.0, 100.0, "standoff length"),
		]),
		fam("shoulder_bolt", "Shoulder bolt", "Fasteners", "ISO 7379 hexagon-socket shoulder screw (ground pivot shoulder).", vec![
			table("shoulder_d", "number", json!(8.0), &[json!(6.5), json!(8.0), json!(10.0), json!(13.0), json!(16.0)], "shoulder diameter (table size)"),
			num("shoulder_len", 20.0, 4.0, 120.0, "ground shoulder length"),
		]),
		// --- Shafts, keys & pins ------------------------------------------------------
		fam("shaft", "Shaft", "Shafts & pins", "Plain shaft along +Z, optional DIN 6885 form-A keyway auto-sized for d.", vec![
			num("d", 8.0, 2.0, 75.0, "shaft diameter"),
			num("length", 40.0, 5.0, 1000.0, "shaft length"),
		]),
		fam("parallel_key", "Parallel key", "Shafts & pins", "DIN 6885 form-A key (round ends), lying flat on z = 0.", vec![
			num("b", 6.0, 2.0, 20.0, "key width"),
			num("h", 6.0, 2.0, 12.0, "key height"),
			num("l", 25.0, 6.0, 100.0, "overall length (keep l > b)"),
		]),
		fam("dowel_pin", "Dowel pin", "Shafts & pins", "ISO 2338 parallel dowel pin with insertion chamfers.", vec![
			table("d", "number", json!(6.0), &[json!(1.0), json!(1.5), json!(2.0), json!(2.5), json!(3.0), json!(4.0), json!(5.0), json!(6.0), json!(8.0), json!(10.0), json!(12.0)], "pin diameter (table size)"),
			num("length", 24.0, 4.0, 120.0, "overall length (must exceed the two chamfers)"),
		]),
		fam("circlip_external", "Circlip (external)", "Shafts & pins", "DIN 471 external retaining ring, drawn installed; mate with circlip_groove_external.", vec![
			table("shaft_d", "number", json!(20.0), &[json!(8.0), json!(10.0), json!(12.0), json!(15.0), json!(20.0), json!(25.0), json!(30.0)], "nominal shaft diameter (table size)"),
		]),
		fam("circlip_internal", "Circlip (internal)", "Shafts & pins", "DIN 472 internal retaining ring (lugs inward); mate with circlip_groove_internal.", vec![
			table("bore_d", "number", json!(32.0), &[json!(16.0), json!(20.0), json!(22.0), json!(26.0), json!(32.0), json!(35.0), json!(42.0), json!(47.0)], "nominal bore diameter (table size)"),
		]),
		// --- Bearings & linear motion ---------------------------------------------------
		fam("deep_groove_bearing", "Deep-groove bearing", "Bearings & linear", "Ball-bearing body (d x D x B annulus) with witness grooves; drop into a bearing_seat.", vec![
			table("designation", "string", json!("608"), &[json!("603"), json!("608"), json!("625"), json!("688"), json!("6000"), json!("6001"), json!("6804")], "bearing designation"),
		]),
		fam("flanged_bearing", "Flanged bearing", "Bearings & linear", "Flanged miniature bearing body, flange face at z = 0.", vec![
			table("designation", "string", json!("F608"), &[json!("F608"), json!("F623")], "bearing designation"),
		]),
		fam("thrust_bearing", "Thrust bearing", "Bearings & linear", "511-series thrust ball-bearing envelope (ISO 104 boundary dims).", vec![
			table("designation", "string", json!("51100"), &[json!("51100"), json!("51101")], "bearing designation"),
		]),
		fam("linear_bearing_lmuu", "Linear bearing (LMxUU)", "Bearings & linear", "LM-series linear ball-bearing envelope with retaining-ring grooves.", vec![
			table("bore", "number", json!(8.0), &[json!(8.0), json!(12.0)], "shaft diameter (LM8UU / LM12UU)"),
		]),
		fam("mgn12_rail", "MGN12 rail", "Bearings & linear", "HIWIN MGN12 profile-rail envelope with M3 countersunk holes on the 25 mm pitch.", vec![
			num("length", 200.0, 25.0, 1000.0, "rail length (>= 25)"),
		]),
		// --- Motors & frames -------------------------------------------------------------
		fam("nema_motor", "NEMA stepper motor", "Motors & frames", "Simplified NEMA body (faceplate at z = 0, pilot + shaft along +Z) for clearance work.", vec![
			table("frame", "int", json!(17), &[json!(17), json!(23)], "NEMA frame number"),
			num("body_len", 40.0, 20.0, 120.0, "body length below the faceplate"),
		]),
		fam("nema_mount_plate", "NEMA mount plate", "Motors & frames", "Square bracket plate: pilot register bore + four ISO 273 clearance holes.", vec![
			table("frame", "int", json!(17), &[json!(17), json!(23)], "NEMA frame number"),
			num("thickness", 5.0, 2.0, 20.0, "plate thickness"),
			num("margin", 4.0, 0.0, 30.0, "extra width beyond the motor face per side"),
		]),
		fam("extrusion_2020", "2020 extrusion", "Motors & frames", "2020 V-slot aluminium extrusion stock (composite profile, ~0.48 kg/m).", vec![
			num("length", 100.0, 10.0, 2000.0, "stick length"),
		]),
		fam("extrusion_3030", "3030 extrusion", "Motors & frames", "3030 T-slot extrusion stock (8 mm slots, M8 core).", vec![
			num("length", 80.0, 10.0, 2000.0, "stick length"),
		]),
		// --- Lead screws, springs & seals -------------------------------------------------
		fam("lead_screw_tr8", "Tr8 lead screw", "Lead screws & springs", "Tr8 trapezoidal lead-screw body (DIN 103 envelope; thread form documented, not cut).", vec![
			num("length", 300.0, 10.0, 1000.0, "screw length"),
			table("lead", "number", json!(8.0), &[json!(2.0), json!(4.0), json!(8.0)], "lead per turn (1/2/4-start, pitch 2)"),
		]),
		fam("compression_spring", "Compression spring", "Lead screws & springs", "Round-wire helix, plain open ends; refused when coils would touch.", vec![
			num("wire_d", 2.0, 0.3, 8.0, "wire diameter"),
			num("outer_d", 16.0, 2.0, 80.0, "coil outside diameter (> 2*wire_d)"),
			num("pitch", 6.0, 0.5, 40.0, "axial advance per turn (> wire_d)"),
			num("turns", 5.0, 1.0, 60.0, "active turns (may be fractional)"),
		]),
		fam("o_ring", "O-ring (AS568)", "Lead screws & springs", "AS568 O-ring at its free nominal size (exact analytic torus).", vec![
			table("dash", "int", json!(214), &[json!(10), json!(12), json!(14), json!(16), json!(18), json!(20), json!(110), json!(112), json!(115), json!(120), json!(210), json!(214), json!(218), json!(222), json!(325)], "AS568 dash number"),
		]),
	]
}

/// GET `/api/catalog` — the families + a UI hint for the insert flow.
pub async fn catalog_endpoint() -> Json<Value> {
	let families = families();
	Json(json!({
		"count": families.len(),
		"families": families,
		"how_to_instantiate": "POST /api/run with {\"ops\": [{\"id\": \"part\", \"op\": <op>, ...params}, {\"id\": \"v\", \"op\": \"volume\", \"in\": \"part\"}, {\"id\": \"stl\", \"op\": \"export_stl\", \"in\": \"part\", \"file\": \"part.stl\"}]}",
	}))
}
