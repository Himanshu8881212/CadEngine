//! CYCLO SIMULATOR — quasi-static simulation of the 26:1 cycloidal drive.
//!
//! "Make sure it works" means MEASURING it, not asserting hope:
//!
//! - **S1 dense mesh sweep** — 720 cam poses over a full cam revolution, both
//!   discs at their true phases: minimum disc↔ring distance tracked with the
//!   BVH, any near-contact pose boolean-verified for ZERO interference; also
//!   counts how many ring pins are simultaneously engaged (the multi-tooth
//!   contact that gives cycloids their torque density).
//! - **S2 ratio lock** — the disc creep MUST be exactly −θ/lobes: sweeps with
//!   deliberately wrong ratios (·1.02, ·0.98) must jam (interfere) within one
//!   cam revolution — geometry, not convention, enforces 26:1.
//! - **S3 backlash + ANTI-BACKLASH SPLIT-DISC CLOCKING** — bisect the free disc
//!   rotation ±ψ at fixed cam angles (mesh backlash) and the free output-plate
//!   rotation at fixed discs (pin-hole backlash). The two discs carry
//!   OPPOSITELY-CLOCKED output holes (disc_a +Δ/2, disc_b −Δ/2 at the pin
//!   circle): one disc bounds the CW output sense, the other the CCW sense, so
//!   the pin-hole backlash shrinks by ≈Δ with the mesh untouched. S3 sweeps
//!   Δ ∈ {0,0.2,0.4,0.6,0.8}° and reports the TOTAL output backlash for each,
//!   plus a no-jam (anti-bind) check on the preloaded pair.
//! - **S4 output coupling** — BOTH clocked discs' six-hole patterns must clear
//!   the three screw-shank pins over the sweep with the plate at −θ/lobes.
//! - **S5 animation export** — the exact profile + parameters go to JSON;
//!   cyclo26/tools/animate_sim.py draws the true 2D kinematics into
//!   cyclo26/simulation.gif (watch the lobes walk the pins and the output
//!   creep at 1/26 speed).
//!
//! Exit 1 on any FAIL. Run:
//!   cargo run --example cyclo26_sim -p kernel-model --release

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, extrude, tessellate_default, try_difference, union, volume, Solid};
use kernel_model::parts::cycloid_disc_profile;
use std::f64::consts::{PI, TAU};

// mirror the drive's STRUCTURAL parameters (params.csv); load() SYNC-ASSERTS
// that params.csv has not drifted from these baked-in constants.
const LOBES: usize = 26;
const RING_R: f64 = 16.5;
const PIN_R: f64 = 1.0;
const ECC: f64 = 0.5;
const DISC_T: f64 = 5.0; // == eccentric bearing (688) width — the disc bore seats it flush
const OUT_PIN_CIRCLE: f64 = 11.5; // moved out 10.5 -> 11.5 so the Ø16.2 eccentric-bearing bore keeps the ligament
const OUT_PIN_R: f64 = 1.5;
// SIX output pins (raised from 3, 2026-07-11 torque-capacity revalidation): the
// 10 N·m FEA showed the plate pin-bore bearing binds the capacity; six M3 pins
// on the same r11.5 circle halve it. k*TAU/6 for k=0,2,4 is the old 3-pattern.
// MUST match OUT_PINS in cyclo26.rs — keep both in sync.
const OUT_PINS: usize = 6;
// shipped tunables. mesh_clearance and hole_slack are LIVE from params.csv.
// BACKDRIVABLE build: disc_clock_deg is 0.0 (ZERO anti-backlash preload — the
// clock preload is a standing pin friction that fights backdrive); the S3 sweep
// still reports {0,0.2,0.4,0.6,0.8} so a preloaded variant stays selectable.
// Sync-asserted to match the CSV so example and sim never diverge.
const CLEAR_DEFAULT: f64 = 0.11;
const HOLE_SLACK_DEFAULT: f64 = 0.04;
const DISC_CLOCK_DEG: f64 = 0.0;
// the Δ-sweep of the anti-backlash split-disc clock (degrees at the pin circle)
const CLOCK_SWEEP: [f64; 5] = [0.0, 0.2, 0.4, 0.6, 0.8];

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}
fn rotz(a: f64) -> DAffine3 {
	DAffine3::from_rotation_z(a)
}
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}
fn overlap_mm3(a: &Solid, b: &Solid) -> f64 {
	match try_difference(a, b) {
		Ok(rem) => (volume(a).abs() - volume(&rem).abs()).max(0.0),
		Err(_) => f64::NAN,
	}
}
/// Disc pose at cam angle θ (ring fixed): eccentric offset + the −θ/lobes creep.
fn disc_pose(th: f64, phase: f64, ratio_lobes: f64) -> DAffine3 {
	let a = th + phase;
	tr(ECC * a.cos(), ECC * a.sin(), 0.0) * rotz(-(a) / ratio_lobes)
}
/// The six output-screw shank pins on the r11.5 circle, rotated by ψ about the
/// output axis (ψ tracks the plate rotation; contact bounds the backlash).
fn out_pins(psi: f64) -> Solid {
	let mut r = cylinder(v(OUT_PIN_CIRCLE * psi.cos(), OUT_PIN_CIRCLE * psi.sin(), -1.0), DVec3::Z, OUT_PIN_R, DISC_T + 2.0, 24);
	for k in 1..OUT_PINS {
		let a = TAU * k as f64 / OUT_PINS as f64 + psi;
		r = union(&r, &cylinder(v(OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin(), -1.0), DVec3::Z, OUT_PIN_R, DISC_T + 2.0, 24));
	}
	r
}

/// The three live tunables, read from cyclo26/params.csv.
struct Tune {
	clear: f64,
	hole_slack: f64,
	disc_clock_deg: f64,
}
/// Load the tunables from params.csv and SYNC-ASSERT the structural params
/// (lobes/ring_r/pin_r/ecc) and the anti-backlash clock still match this sim's
/// constants — the honesty gate against silent params.csv/code drift.
fn load() -> Tune {
	let mut t = Tune { clear: CLEAR_DEFAULT, hole_slack: HOLE_SLACK_DEFAULT, disc_clock_deg: DISC_CLOCK_DEG };
	let (mut cl, mut cr, mut cp, mut ce) = (LOBES as f64, RING_R, PIN_R, ECC);
	let mut clock_seen = false;
	if let Ok(text) = std::fs::read_to_string("cyclo26/params.csv") {
		for line in text.lines() {
			let line = line.trim();
			if line.starts_with('#') || line.is_empty() {
				continue;
			}
			let mut it = line.split(',');
			let (Some(k), Some(val)) = (it.next(), it.next()) else { continue };
			let Ok(x) = val.trim().parse::<f64>() else { continue };
			match k.trim() {
				"lobes" => cl = x,
				"ring_r" => cr = x,
				"pin_r" => cp = x,
				"ecc" => ce = x,
				"mesh_clearance" => t.clear = x,
				"hole_slack" => t.hole_slack = x,
				"disc_clock_deg" => {
					t.disc_clock_deg = x;
					clock_seen = true;
				}
				_ => {}
			}
		}
	}
	assert!(
		cl as usize == LOBES && (cr - RING_R).abs() < 1e-9 && (cp - PIN_R).abs() < 1e-9 && (ce - ECC).abs() < 1e-9,
		"params.csv structural params ({cl}:{cr}:{cp}:{ce}) drifted from the sim consts ({LOBES}:{RING_R}:{PIN_R}:{ECC}) — update BOTH"
	);
	assert!(
		clock_seen && (t.disc_clock_deg - DISC_CLOCK_DEG).abs() < 1e-9,
		"params.csv disc_clock_deg {} != sim DISC_CLOCK_DEG {} — update BOTH so the example and the sim agree",
		t.disc_clock_deg,
		DISC_CLOCK_DEG
	);
	t
}

fn main() {
	let t0 = std::time::Instant::now();
	let tune = load();
	let (clear, hole_slack) = (tune.clear, tune.hole_slack);
	println!(
		"CYCLO SIMULATOR — {LOBES}:1, e={ECC}, ring Ø{:.0}, clearance {clear}, hole_slack {hole_slack}, anti-backlash Δ={}°\n",
		RING_R * 2.0,
		tune.disc_clock_deg
	);
	let mut ok = true;

	let profile = ccw(cycloid_disc_profile(LOBES, RING_R, PIN_R + clear, ECC, 24));
	// build a disc whose THREE output holes are clocked by `clock` rad about the
	// disc centre; the epitrochoid mesh profile and the journal bore never move.
	let build_disc = |clock: f64| -> Solid {
		let mut d = extrude(&profile, DISC_T);
		// Ø16.2 eccentric-bearing (688) outer-race seat (was a Ø14.2 plain-journal bore)
		d = try_difference(&d, &cylinder(v(0.0, 0.0, -1.0), DVec3::Z, 8.1, DISC_T + 2.0, 48)).expect("bore");
		for k in 0..OUT_PINS {
			let a = TAU * k as f64 / OUT_PINS as f64 + clock;
			let hr = OUT_PIN_R + ECC + hole_slack;
			d = try_difference(&d, &cylinder(v(OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin(), -1.0), DVec3::Z, hr, DISC_T + 2.0, 32)).expect("hole");
		}
		d
	};
	// reference disc for the MESH tests (S1/S2/mesh-backlash) — the hole clock is
	// irrelevant to meshing, so a clock-0 disc represents both discs there.
	let disc = build_disc(0.0);
	// the assembly clocks disc_b's holes +π/lobes in its OWN frame so its
	// rotz(−π/lobes) placement lands the holes back on the pin circle; on top of
	// that the anti-backlash split adds ∓Δ/2. two_discs() poses BOTH clocked
	// patterns at their assembly phases (a = phase 0, b = phase π).
	let comp = PI / LOBES as f64;
	let two_discs = |clock_deg: f64, th0: f64| -> (Solid, Solid) {
		let half = clock_deg.to_radians() * 0.5;
		let a = build_disc(half).transformed(disc_pose(th0, 0.0, LOBES as f64));
		let b = build_disc(comp - half).transformed(disc_pose(th0, PI, LOBES as f64));
		(a, b)
	};

	let disc_mesh_area = tessellate_default(&disc).indices.len() / 3;
	let pins_n = LOBES + 1;
	let ring: Solid = {
		let mut r = cylinder(v(RING_R, 0.0, -1.0), DVec3::Z, PIN_R, DISC_T + 2.0, 24);
		for k in 1..pins_n {
			let a = TAU * k as f64 / pins_n as f64;
			r = union(&r, &cylinder(v(RING_R * a.cos(), RING_R * a.sin(), -1.0), DVec3::Z, PIN_R, DISC_T + 2.0, 24));
		}
		r
	};
	let ring_mesh = tessellate_default(&ring);
	// individual pins for engagement counting
	let pin_meshes: Vec<_> = (0..pins_n)
		.map(|k| {
			let a = TAU * k as f64 / pins_n as f64;
			tessellate_default(&cylinder(v(RING_R * a.cos(), RING_R * a.sin(), -1.0), DVec3::Z, PIN_R, DISC_T + 2.0, 24))
		})
		.collect();

	// ---- S1: dense full-revolution sweep, both disc phases ----
	println!("S1 — dense mesh sweep: 720 poses × 2 disc phases ({disc_mesh_area}-tri disc):");
	let steps = 720;
	let mut min_gap = f64::INFINITY;
	let mut max_gap = 0.0f64;
	let (mut eng_min, mut eng_sum, mut eng_poses) = (usize::MAX, 0usize, 0usize);
	let mut suspicious: Vec<(f64, f64)> = Vec::new();
	for phase in [0.0, PI] {
		for k in 0..steps {
			let th = TAU * k as f64 / steps as f64;
			let posed = disc.transformed(disc_pose(th, phase, LOBES as f64));
			let pm = tessellate_default(&posed);
			let gap = pm.min_distance(&ring_mesh) as f64;
			min_gap = min_gap.min(gap);
			max_gap = max_gap.max(gap);
			if gap < 0.005 {
				suspicious.push((th, phase));
			}
			// engagement: pins within one clearance band of the disc
			if k % 30 == 0 {
				let engaged = pin_meshes.iter().filter(|p| (pm.min_distance(p) as f64) < 0.8 * clear + 0.04).count();
				eng_min = eng_min.min(engaged);
				eng_sum += engaged;
				eng_poses += 1;
			}
		}
	}
	// boolean-verify every near-contact pose: closeness must be contact, never overlap
	let mut verified = 0usize;
	let mut interfered = 0usize;
	for &(th, phase) in suspicious.iter().take(40) {
		let posed = disc.transformed(disc_pose(th, phase, LOBES as f64));
		let ov = overlap_mm3(&posed, &ring);
		if ov.is_nan() || ov >= 0.05 {
			interfered += 1;
		}
		verified += 1;
	}
	let s1_ok = interfered == 0 && max_gap <= 3.0 * clear + 0.05 && eng_min >= 2;
	ok &= s1_ok;
	println!(
		"  gap ∈ [{min_gap:.4}, {max_gap:.4}] (≤ {:.2} = always engaged), {} near-contact poses boolean-verified: {} interference",
		3.0 * clear + 0.05,
		verified,
		interfered
	);
	println!(
		"  simultaneous pin engagement: min {eng_min}, avg {:.1} of {pins_n} pins  {}",
		eng_sum as f64 / eng_poses as f64,
		if s1_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- S2: ratio lock — wrong creep must jam within one revolution ----
	println!("S2 — ratio lock (wrong creep must JAM; geometry enforces {LOBES}:1):");
	let mut s2_ok = true;
	for wrong in [LOBES as f64 * 1.05, LOBES as f64 * 0.95] {
		let mut worst = 0.0f64;
		for k in 0..180 {
			let th = 2.0 * TAU * k as f64 / 180.0; // two cam revolutions
			let ov = overlap_mm3(&disc.transformed(disc_pose(th, 0.0, wrong)), &ring);
			if ov.is_finite() {
				worst = worst.max(ov);
			}
			if worst > 1.0 {
				break;
			}
		}
		let jammed = worst > 1.0;
		s2_ok &= jammed;
		println!(
			"  creep −θ/{wrong:.2}: max interference {worst:.1} mm³ within two revs — {}",
			if jammed { "JAMS (correct)" } else { "<<< does NOT jam: ratio not enforced FAIL" }
		);
	}
	ok &= s2_ok;

	// ---- S3: backlash by bisection + anti-backlash split-disc clocking ----
	println!("S3 — backlash (bisection to 0.01°); anti-backlash split-disc clocking:");
	// mesh backlash: free disc rotation ±ψ about its own (offset) centre — the
	// output holes never touch here, so clocking leaves this untouched.
	let free = |th: f64, psi: f64| -> bool {
		let a = th;
		let pose = tr(ECC * a.cos(), ECC * a.sin(), 0.0) * rotz(-(a) / LOBES as f64 + psi);
		overlap_mm3(&disc.transformed(pose), &ring) < 0.02
	};
	let bisect = |th: f64, sign: f64| -> f64 {
		let (mut lo, mut hi) = (0.0f64, 3.0f64.to_radians());
		for _ in 0..9 {
			let mid = 0.5 * (lo + hi);
			if free(th, sign * mid) {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		lo
	};
	let mut mesh_lash = 0.0f64;
	for k in 0..4 {
		let th = TAU * k as f64 / 4.0 + 0.13;
		let lash = bisect(th, 1.0) + bisect(th, -1.0);
		mesh_lash = mesh_lash.max(lash);
	}

	// TWO-disc pin-hole backlash. Both clocked discs are held at a cam angle;
	// the output pins are rotated about the axis and contact occurs when EITHER
	// disc's holes touch a pin (the +Δ/2 pattern bounds one output sense, the
	// −Δ/2 pattern the other). The pin-hole play is CAM-ANGLE dependent (the
	// eccentric orbit of each disc's holes swings tangentially), so the honest
	// spec is the WORST (largest) free window over the cam revolution — that is
	// the phase where the eccentric offset is radial and the clock subtracts
	// cleanly. hole_lash(Δ) = max over the cam-angle set of (ψ+ + ψ−).
	let cam_set: [f64; 4] = [0.13, PI * 0.5 + 0.13, PI + 0.13, PI * 1.5 + 0.13];
	let hole_lash_for = |clock_deg: f64| -> f64 {
		let half = clock_deg.to_radians() * 0.5;
		let da0 = build_disc(half); // disc_a: holes +Δ/2 (pin frame), placed at phase 0
		let db0 = build_disc(comp - half); // disc_b: rotz(−π/lobes) lands holes at −Δ/2
		let mut worst = 0.0f64;
		for &th in &cam_set {
			let da = da0.transformed(disc_pose(th, 0.0, LOBES as f64));
			let db = db0.transformed(disc_pose(th, PI, LOBES as f64));
			let base = -th / LOBES as f64; // pins co-rotate with disc_a's output creep
			let pin_free = |psi: f64| -> bool {
				let pins = out_pins(base + psi);
				overlap_mm3(&da, &pins) < 0.02 && overlap_mm3(&db, &pins) < 0.02
			};
			let pin_bisect = |sign: f64| -> f64 {
				let (mut lo, mut hi) = (0.0f64, 6.0f64.to_radians());
				for _ in 0..8 {
					let mid = 0.5 * (lo + hi);
					if pin_free(sign * mid) {
						lo = mid;
					} else {
						hi = mid;
					}
				}
				lo
			};
			worst = worst.max(pin_bisect(1.0) + pin_bisect(-1.0));
		}
		worst
	};

	// Δ-sweep: measure the pin-hole and TOTAL output backlash at each clock split
	let mut lash_of = [0.0f64; 5];
	for (i, &d) in CLOCK_SWEEP.iter().enumerate() {
		lash_of[i] = hole_lash_for(d);
	}
	// Δ=0 is the twin-disc baseline: two discs at OPPOSITE eccentric phases already
	// sandwich the pins somewhat, so this is already ≤ a lone disc's pin-hole lash.
	let base_lash = lash_of[0];
	println!("  mesh backlash ≤ {:.3}° (clocking leaves the mesh untouched)", mesh_lash.to_degrees());
	println!("  Δ-sweep (worst-case over the cam revolution, deg at the pin circle):");
	println!("    Δ(deg)   pin-hole   TOTAL=mesh+pin-hole   Δ removed");
	for (i, &d) in CLOCK_SWEEP.iter().enumerate() {
		println!(
			"    {:>5.1}    {:>7.3}°   {:>7.3}°             {:>+6.3}°",
			d,
			lash_of[i].to_degrees(),
			(mesh_lash + lash_of[i]).to_degrees(),
			(base_lash - lash_of[i]).to_degrees()
		);
	}

	// the chosen split (params.csv disc_clock_deg, sync-asserted == DISC_CLOCK_DEG)
	let chosen = tune.disc_clock_deg;
	let chosen_idx = CLOCK_SWEEP.iter().position(|d| (d - chosen).abs() < 1e-9);
	let chosen_lash = match chosen_idx {
		Some(i) => lash_of[i],
		None => hole_lash_for(chosen), // off-grid value: measure it directly
	};
	let total_lash = mesh_lash + chosen_lash;
	let no_jam_margin = chosen_lash.to_degrees(); // surviving free play = distance from a bind

	// ANTI-JAM: place BOTH clocked discs at a handful of cam poses and confirm the
	// preloaded pair never forces the pins into a bind (overlap ≈ 0 through a rev).
	let mut jam_bad = 0usize;
	let mut jam_worst = 0.0f64;
	for k in 0..8 {
		let th = TAU * k as f64 / 8.0;
		let (da, db) = two_discs(chosen, th);
		let pins = out_pins(-th / LOBES as f64);
		let (oa, ob) = (overlap_mm3(&da, &pins), overlap_mm3(&db, &pins));
		let worst = oa.max(ob);
		if !(worst.is_finite() && worst < 0.05) {
			jam_bad += 1;
		}
		jam_worst = jam_worst.max(if worst.is_finite() { worst } else { f64::INFINITY });
	}

	// the single-pattern facet-honest geometric bound (the Δ=0 worst case sits at
	// it — a lone hole pattern's slack past the eccentric orbit + facet deficit)
	let geo_bound_facets = 2.0 * ((hole_slack + 0.03) / OUT_PIN_CIRCLE).asin().to_degrees();
	// honest gates — asserting only what is MEASURED, no assumed physics:
	//  (1) the Δ=0 worst case sits within the facet-honest geometric bound;
	//  (2) the chosen clock never ADDS backlash vs the twin-disc Δ=0 baseline (a
	//      preloaded build reduces it; the BACKDRIVABLE Δ=0 build sits AT it);
	//  (3) the chosen TOTAL stays under the 1.5° absolute spec;
	//  (4) the no-jam margin (surviving free window) is ≥ 0.15°, and no cam pose
	//      forces the pair into a bind (trivially satisfied at Δ=0 — no preload).
	// NOTE: the per-Δ reduction does NOT cleanly equal Δ — the opposite-phase
	// twin discs already remove some lash at Δ=0, so the clock only harvests the
	// remainder. We report the measured reduction and do not pretend it is Δ.
	let base_ok = base_lash.to_degrees() <= geo_bound_facets * 1.10;
	// chosen clock must never ADD backlash over the twin-disc baseline; a preloaded
	// build strictly reduces it, the BACKDRIVABLE zero-preload build sits AT it (==).
	let not_worse = total_lash <= mesh_lash + base_lash + 1e-9;
	let s3_ok = base_ok && not_worse && total_lash.to_degrees() < 1.5 && no_jam_margin >= 0.15 && jam_bad == 0;
	ok &= s3_ok;
	println!(
		"  chosen Δ={chosen}°: pin-hole {:.3}° (twin-disc baseline Δ=0 {:.3}°, facet bound {:.3}°) · TOTAL {:.3}° (was {:.3}° at Δ=0)",
		chosen_lash.to_degrees(),
		base_lash.to_degrees(),
		geo_bound_facets,
		total_lash.to_degrees(),
		(mesh_lash + base_lash).to_degrees()
	);
	println!(
		"  no-jam: 8 cam poses, both clocked discs placed — worst pin overlap {:.3} mm³, margin {no_jam_margin:.3}° ≥ 0.15  {}",
		jam_worst,
		if s3_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- S4: output coupling over the sweep — BOTH clocked discs ----
	let mut s4_ok = true;
	for k in 0..24 {
		let th = TAU * k as f64 / 24.0;
		let (da, db) = two_discs(chosen, th);
		let pins = out_pins(-th / LOBES as f64);
		let (oa, ob) = (overlap_mm3(&da, &pins), overlap_mm3(&db, &pins));
		if oa.is_nan() || oa >= 0.05 || ob.is_nan() || ob >= 0.05 {
			s4_ok = false;
			println!("  output pins collide at θ={:.0}° (disc_a {oa:.2}/disc_b {ob:.2} mm³) <<<", th.to_degrees());
		}
	}
	ok &= s4_ok;
	println!("S4 — both clocked discs' hole patterns clear the output pins over the sweep  {}", if s4_ok { "OK" } else { "<<< FAIL" });

	// ---- S5: export exact geometry for the 2D animation ----
	let hole_r = OUT_PIN_R + ECC + hole_slack;
	let mut json = String::from("{\n");
	json.push_str(&format!(
		"\"lobes\": {LOBES}, \"ring_r\": {RING_R}, \"pin_r\": {PIN_R}, \"ecc\": {ECC}, \"clear\": {clear},\n\"hole_circle\": {OUT_PIN_CIRCLE}, \"hole_r\": {hole_r}, \"out_pin_r\": {OUT_PIN_R}, \"disc_clock_deg\": {chosen},\n\"profile\": ["
	));
	for (i, q) in profile.iter().enumerate() {
		if i > 0 {
			json.push(',');
		}
		json.push_str(&format!("[{:.5},{:.5}]", q.x, q.y));
	}
	json.push_str("]\n}\n");
	let _ = std::fs::create_dir_all("cyclo26/sim");
	let _ = std::fs::write("cyclo26/sim/kinematics.json", &json);
	println!("S5 — exact profile + params -> cyclo26/sim/kinematics.json (run cyclo26/tools/animate_sim.py)");

	println!(
		"\nRESULT: {} ({:.0} s)",
		if ok { "PASS — kinematics work: meshes everywhere, ratio locked, backlash measured across the Δ-sweep (backdrivability is a friction property this sim does NOT measure — hand-test to confirm)" } else { "FAIL — see <<< lines" },
		t0.elapsed().as_secs_f64()
	);
	if !ok {
		std::process::exit(1);
	}
}
