// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Screw-family breadth**: ISO 10642 countersunk (flat-head) socket screws, ISO 7380
//! button-head socket screws, DIN 916 cup-point set screws, DIN 985 nyloc lock nuts,
//! threaded rod and hex standoffs. As across the library, these are the analytically
//! exact *bodies* (threads are not modelled — see [`super::threads`] for modelled ISO
//! threads); drive sockets are cut as real hexagonal pockets so clearance checks and
//! renders see them. Dimension tables are copied from the published standards with the
//! source cited next to each table; all values mm, hex sizes across flats.

use super::hexagon_across_flats;
use super::threads::iso_coarse_pitch;
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, revolve, union, Solid};

/// One row of the ISO 10642 (≈ DIN 7991) hexagon-socket countersunk screw table:
/// `(thread Ø d, head Ø dk, head height k, socket across-flats s, socket depth t)`.
/// Source: ISO 10642 dimension table as published at fasteners.eu/standards/iso/10642
/// (dk/k/t max, s nominal; mm). The 90° head angle means the cone rise is exactly
/// `(dk − d)/2`; `k` is the table's slightly larger max envelope.
const ISO10642: [(f64, f64, f64, f64, f64); 8] = [
	(3.0, 6.0, 1.7, 2.0, 1.2),
	(4.0, 8.0, 2.3, 2.5, 1.8),
	(5.0, 10.0, 2.8, 3.0, 2.3),
	(6.0, 12.0, 3.3, 4.0, 2.5),
	(8.0, 16.0, 4.4, 5.0, 3.5),
	(10.0, 20.0, 5.5, 6.0, 4.4),
	(12.0, 24.0, 6.5, 8.0, 4.6),
	(16.0, 30.0, 7.5, 10.0, 5.3),
];

/// One row of the ISO 7380 button-head socket screw table:
/// `(thread Ø d, head Ø dk, head height k, socket across-flats s, socket depth t)`.
/// Source: ISO 7380 dimension table as published at fasteners.eu/standards/iso/7380
/// (dk/k max, s nominal, t min; mm).
const ISO7380: [(f64, f64, f64, f64, f64); 7] = [
	(3.0, 5.7, 1.65, 2.0, 1.04),
	(4.0, 7.6, 2.2, 2.5, 1.3),
	(5.0, 9.5, 2.75, 3.0, 1.56),
	(6.0, 10.5, 3.3, 4.0, 2.08),
	(8.0, 14.0, 4.4, 5.0, 2.6),
	(10.0, 17.5, 5.5, 6.0, 3.12),
	(12.0, 21.0, 6.6, 8.0, 4.16),
];

/// One row of the DIN 916 (≈ ISO 4029) cup-point set-screw table:
/// `(thread Ø d, socket across-flats s, socket depth t, cup Ø dv)`.
/// Source: DIN 916 dimension table as published at fasteners.eu/standards/din/916
/// (s nominal, t min, dv max; mm).
const DIN916: [(f64, f64, f64, f64); 7] = [
	(3.0, 1.5, 1.2, 1.4),
	(4.0, 2.0, 1.5, 2.0),
	(5.0, 2.5, 2.0, 2.5),
	(6.0, 3.0, 2.0, 3.0),
	(8.0, 4.0, 3.0, 5.0),
	(10.0, 5.0, 4.0, 6.0),
	(12.0, 6.0, 4.5, 8.0),
];

/// One row of the DIN 985 prevailing-torque (nylon-insert) lock-nut table:
/// `(thread Ø d, across-flats s, overall height h, metal hex height m, bearing Ø dw)`.
/// Source: DIN 985 dimension table as published at fasteners.eu/standards/din/985
/// (h max, m min, dw; mm). Note DIN widths: M10 → 17, M12 → 19 (unlike ISO 4032's 16/18).
const DIN985: [(f64, f64, f64, f64, f64); 8] = [
	(3.0, 5.5, 4.0, 2.4, 4.6),
	(4.0, 7.0, 5.0, 2.9, 5.9),
	(5.0, 8.0, 5.0, 3.2, 6.9),
	(6.0, 10.0, 6.0, 4.0, 8.9),
	(8.0, 13.0, 8.0, 5.5, 11.6),
	(10.0, 17.0, 10.0, 6.5, 15.6),
	(12.0, 19.0, 12.0, 8.0, 17.4),
	(16.0, 24.0, 16.0, 10.5, 22.5),
];

/// Hex-standoff (spacer) across-flats by thread size: standoffs conventionally take
/// the hex-nut wrench size — `(thread Ø d, across-flats s)`. Source: the ISO 4032 /
/// DIN 934 nut widths (fasteners.eu/standards/iso/4032) as used by the common
/// M2–M6 spacer catalogs (e.g. Würth, Keystone: M2.5 → AF 5, M3 → AF 5.5).
const STANDOFF_AF: [(f64, f64); 6] = [(2.0, 4.0), (2.5, 5.0), (3.0, 5.5), (4.0, 7.0), (5.0, 8.0), (6.0, 10.0)];

/// Match a nominal metric size against a table keyed by its first column.
fn lookup<const N: usize, T: Copy>(table: &[T; N], m: f64, key: impl Fn(&T) -> f64) -> Option<T> {
	table.iter().find(|row| (key(row) - m).abs() < 1e-9).copied()
}

/// ISO 10642 head dimensions for nominal size `m` (3–16): `(head Ø dk, head height k,
/// socket s, socket depth t)` in mm; `None` outside the table.
pub fn iso10642_dims(m: f64) -> Option<(f64, f64, f64, f64)> {
	lookup(&ISO10642, m, |r| r.0).map(|(_, dk, k, s, t)| (dk, k, s, t))
}

/// Cut the hexagonal drive socket (across-flats `s`, depth `t`) into the planar top
/// face of `body` at height `z_top`; the prism overshoots upward so the cut is clean.
fn cut_hex_socket(body: &Solid, s: f64, t: f64, z_top: f64) -> Solid {
	let socket = extrude(&hexagon_across_flats(s), t + 1.0).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, z_top - t)));
	difference(body, &socket)
}

/// An **ISO 10642 countersunk (flat-head) socket screw** body for nominal Ø `m`
/// (M3–M16) and overall `length` (countersunk screws measure tip-to-head-top):
/// a Ø`m` shank capped by the exact 90° conical head — rise `(dk − m)/2` to the
/// table's head Ø `dk` — with the hex drive socket (`s` × `t`) cut into the flat top.
/// Pairs with the DIN 74 form F countersink of [`kernel_brep::holes::countersink_hole`].
/// One revolved profile plus the socket pocket: closed, manifold, genus 0. The thread
/// is not modelled. `None` outside the table or when `length` cannot contain the head.
pub fn flat_head_screw(m: f64, length: f64) -> Option<Solid> {
	let (dk, _k, s, t) = iso10642_dims(m)?;
	let hc = (dk - m) * 0.5; // exact 90° cone rise
						  // NaN-safe: the conjunction refuses non-finite lengths too.
	if !(length > hc + t && length.is_finite()) {
		return None;
	}
	let profile = [
		DVec2::new(0.0, 0.0),
		DVec2::new(m * 0.5, 0.0),
		DVec2::new(m * 0.5, length - hc),
		DVec2::new(dk * 0.5, length),
		DVec2::new(0.0, length),
	];
	Some(cut_hex_socket(&revolve(&profile, 48), s, t, length))
}

/// ISO 7380 head dimensions for nominal size `m` (3–12): `(head Ø dk, head height k,
/// socket s, socket depth t)` in mm; `None` outside the table.
pub fn iso7380_dims(m: f64) -> Option<(f64, f64, f64, f64)> {
	lookup(&ISO7380, m, |r| r.0).map(|(_, dk, k, s, t)| (dk, k, s, t))
}

/// An **ISO 7380 button-head socket screw** body for nominal Ø `m` (M3–M12) and
/// under-head shank `length`: a Ø`m` shank under a spherical-cap head of base Ø `dk`
/// and height `k` (dome radius `(dk²/4 + k²) / 2k`, the unique sphere through the
/// head rim and crown), with the hex socket sunk through the crown. One revolved
/// profile (dome arc sampled at 12 stations) plus the socket pocket — the pocket
/// pierces the curved crown, the kernel's robust cut-across-curved-wall route.
/// Closed, manifold, genus 0; thread not modelled. `None` outside the table.
pub fn button_head_screw(m: f64, length: f64) -> Option<Solid> {
	let (dk, k, s, t) = iso7380_dims(m)?;
	if !(length > 0.0 && length.is_finite()) {
		return None;
	}
	let big_r = (dk * dk * 0.25 + k * k) / (2.0 * k); // dome (spherical-cap) radius
	let zc = length + k - big_r; // dome centre on the axis
	let mut profile = vec![DVec2::new(0.0, 0.0), DVec2::new(m * 0.5, 0.0), DVec2::new(m * 0.5, length)];
	// Dome arc from the rim (dk/2, length) up to the crown (0, length + k).
	let a0 = (dk * 0.5).atan2(length - zc); // polar angle from +z at the rim
	for i in 0..=12 {
		let a = a0 * (1.0 - i as f64 / 12.0);
		profile.push(DVec2::new(big_r * a.sin(), zc + big_r * a.cos()));
	}
	Some(cut_hex_socket(&revolve(&profile, 48), s, t, length + k))
}

/// DIN 916 dimensions for nominal size `m` (3–12): `(socket s, socket depth t,
/// cup Ø dv)` in mm; `None` outside the table.
pub fn din916_dims(m: f64) -> Option<(f64, f64, f64)> {
	lookup(&DIN916, m, |r| r.0).map(|(_, s, t, dv)| (s, t, dv))
}

/// A **DIN 916 hexagon-socket set screw (grub screw), cup point**, for nominal Ø `m`
/// (M3–M12) and `length`: a headless Ø`m` body with the hex socket (`s` × `t`) in the
/// top face and the cup recess in the bottom face. Honest approximation: the cup is
/// modelled as a 120°-included conical recess of the table's mouth Ø `dv` (depth
/// `dv/2 · tan 30°`) — the standard draws an annular crater whose exact wall profile
/// is manufacturer-specific; mouth Ø and the body envelope are the table values. The
/// body is built at the nominal Ø (thread not modelled). Genus 0; `None` outside the
/// table or when the cup, socket and a 0.5 mm web cannot fit in `length`.
pub fn set_screw(m: f64, length: f64) -> Option<Solid> {
	let (s, t, dv) = din916_dims(m)?;
	let cup = dv * 0.5 * 30.0_f64.to_radians().tan(); // 120°-included cone depth
	if !(length > cup + t + 0.5 && length.is_finite()) {
		return None;
	}
	let profile =
		[DVec2::new(0.0, cup), DVec2::new(dv * 0.5, 0.0), DVec2::new(m * 0.5, 0.0), DVec2::new(m * 0.5, length), DVec2::new(0.0, length)];
	Some(cut_hex_socket(&revolve(&profile, 48), s, t, length))
}

/// DIN 985 dimensions for nominal size `m` (3–16): `(across-flats s, overall height h,
/// metal hex height m_hex, bearing Ø dw)` in mm; `None` outside the table.
pub fn din985_dims(m: f64) -> Option<(f64, f64, f64, f64)> {
	lookup(&DIN985, m, |r| r.0).map(|(_, s, h, mh, dw)| (s, h, mh, dw))
}

/// A **DIN 985 nyloc (nylon-insert) lock nut** body for nominal Ø `m` (M3–M16): the
/// hexagonal wrench section (across-flats `s`) up to the metal height `m_hex`, topped
/// by the insert collar — a revolved Ø`dw` ring up to the overall height `h` with a
/// 45° crown chamfer — and bored through at the nominal thread Ø.
///
/// Honest approximations: the real crown is a smooth dome rolled over the nylon ring;
/// here it is a straight Ø`dw` collar (the table's bearing-surface Ø, which keeps the
/// collar strictly inside the hex flats) with a chamfered rim, and the nylon ring
/// itself is not modelled as a separate body — `s`, `h`, `m_hex`, `dw` are the table
/// values. Genus 1; `None` outside the table.
pub fn lock_nut(m: f64) -> Option<Solid> {
	let (s, h, mh, dw) = din985_dims(m)?;
	let c = 0.3 * (h - mh); // crown chamfer, well under the collar wall (dw − m)/2
	let hex = extrude(&hexagon_across_flats(s), mh);
	let collar =
		[DVec2::new(0.0, mh), DVec2::new(dw * 0.5, mh), DVec2::new(dw * 0.5, h - c), DVec2::new(dw * 0.5 - c, h), DVec2::new(0.0, h)];
	let body = union(&hex, &revolve(&collar, 48));
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, m * 0.5, h + 2.0, 48);
	Some(difference(&body, &bore))
}

/// A length of **metric threaded rod** (studding, DIN 976-1 style) for nominal Ø `m`
/// (M3–M16, the ISO 261 coarse table) and any `length`: a Ø`m` cylinder with 45° end
/// chamfers of half a pitch — the standard's deburred chamfered ends. The thread is
/// not modelled (project convention: catalog bodies are the exact assembly envelopes;
/// see [`super::threads::iso_thread_solid`] for a modelled ridge). One revolved
/// profile, genus 0; `None` outside the pitch table or for degenerate lengths.
pub fn threaded_rod(m: f64, length: f64) -> Option<Solid> {
	let pitch = iso_coarse_pitch(m)?;
	let c = 0.5 * pitch;
	if !(length > 2.0 * c && length.is_finite()) {
		return None;
	}
	let r = m * 0.5;
	let profile = [
		DVec2::new(0.0, 0.0),
		DVec2::new(r - c, 0.0),
		DVec2::new(r, c),
		DVec2::new(r, length - c),
		DVec2::new(r - c, length),
		DVec2::new(0.0, length),
	];
	Some(revolve(&profile, 48))
}

/// A **female–female hex standoff (spacer)** for nominal thread Ø `m` (M2–M6) and
/// `length`: a hexagonal prism at the conventional wrench size for `m` (the hex-nut
/// across-flats — M2.5 → AF 5, M3 → AF 5.5, …), bored through at the nominal Ø.
/// The internal thread is not modelled. Genus 1; `None` outside the AF table or for
/// degenerate lengths.
pub fn standoff(m: f64, length: f64) -> Option<Solid> {
	let af = lookup(&STANDOFF_AF, m, |r| r.0).map(|(_, s)| s)?;
	if !(length > 0.0 && length.is_finite()) {
		return None;
	}
	Some(super::fasteners::hex_nut(af, length, m))
}

/// One row of the ISO 7379 hexagon-socket **shoulder screw** table: `(shoulder Ø d1,
/// thread Ø d3, thread length b, head Ø dk, head height k, socket across-flats s)`.
/// Source: the ISO 7379 dimension table as reproduced by the fastener vendors
/// (Bossard BN 1364 / fasteners.eu ISO 7379; mm) — note the standard's distinctive
/// shoulder sizes 6.5 and 13 (ground f9 shoulders, one size over their thread).
const ISO7379: [(f64, f64, f64, f64, f64, f64); 5] = [
	(6.5, 5.0, 9.75, 10.0, 4.5, 3.0),
	(8.0, 6.0, 11.25, 13.0, 5.5, 4.0),
	(10.0, 8.0, 13.25, 16.0, 7.0, 5.0),
	(13.0, 10.0, 16.4, 18.0, 9.0, 6.0),
	(16.0, 12.0, 18.4, 24.0, 11.0, 8.0),
];

/// ISO 7379 dimensions for a `shoulder_d` of 6.5, 8, 10, 13 or 16: `(thread Ø d3,
/// thread length b, head Ø dk, head height k, socket s)`; `None` outside the table.
pub fn iso7379_dims(shoulder_d: f64) -> Option<(f64, f64, f64, f64, f64)> {
	lookup(&ISO7379, shoulder_d, |r| r.0).map(|(_, d3, b, dk, k, s)| (d3, b, dk, k, s))
}

/// An **ISO 7379 hexagon-socket shoulder screw** body: thread tip at z = 0 (stem at
/// the thread major Ø — the helix is not modelled, as across the library), the
/// ground Ø`shoulder_d` shoulder from `b` to `b + shoulder_len` (the length you
/// order; it bushes a bearing or pivots a link), and the table's head with its hex
/// socket on top. The socket is cut to `k/2` — ISO 7379's socket-depth column is
/// not reproduced here (display proportion, documented). One revolved profile plus
/// the socket pocket: closed, manifold, genus 0. The DIN 76-style thread-relief
/// undercut at the shoulder step is omitted (honest simplification). `None`
/// outside the table or for a degenerate `shoulder_len`.
pub fn shoulder_bolt(shoulder_d: f64, shoulder_len: f64) -> Option<Solid> {
	let (d3, b, dk, k, s) = iso7379_dims(shoulder_d)?;
	if !(shoulder_len > 0.0 && shoulder_len.is_finite()) {
		return None;
	}
	let top = b + shoulder_len + k;
	let profile = [
		DVec2::new(0.0, 0.0),
		DVec2::new(d3 * 0.5, 0.0),
		DVec2::new(d3 * 0.5, b),
		DVec2::new(shoulder_d * 0.5, b),
		DVec2::new(shoulder_d * 0.5, b + shoulder_len),
		DVec2::new(dk * 0.5, b + shoulder_len),
		DVec2::new(dk * 0.5, top),
		DVec2::new(0.0, top),
	];
	Some(cut_hex_socket(&revolve(&profile, 48), s, k * 0.5, top))
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::parts::hexagon_area;
	use kernel_brep::{tessellate_adaptive_tol, validate, volume};
	use std::f64::consts::PI;

	/// `(volume, validity, watertight)` of a part, for the snapshot asserts below.
	/// Watertightness is checked on the adaptive 10 µm tessellation — the library's
	/// primary export route (same as the `threads` tests): the default-path stitcher
	/// is known to crack on some valid boolean outputs (e.g. the M8 button crown's
	/// hex-socket seam) where the adaptive path is watertight.
	fn measure(s: &Solid) -> (f64, bool, i64) {
		let v = validate(s);
		(volume(s).abs(), v.closed && v.manifold && tessellate_adaptive_tol(s, 0.01).is_watertight(), v.genus)
	}

	#[test]
	fn flat_head_screws_match_iso10642_and_lose_exactly_cone_offcut_and_socket() {
		// M5×16 and M10×30 (overall length): volume = shank π r²(L − rise) + the 90°
		// head frustum − the hex socket; 1% covers the 48-gon faceting. The M5 socket
		// is 3 AF × 2.3 deep per the table.
		for (m, len) in [(5.0, 16.0), (10.0, 30.0)] {
			let (dk, _k, s, t) = iso10642_dims(m).expect("table row");
			let screw = flat_head_screw(m, len).expect("table size");
			let (vol, sound, genus) = measure(&screw);
			let (r, rk, hc) = (m * 0.5, dk * 0.5, (dk - m) * 0.5);
			let frustum = PI * hc / 3.0 * (r * r + r * rk + rk * rk);
			let expected = PI * r * r * (len - hc) + frustum - hexagon_area(s) * t;
			assert!(
				sound && genus == 0 && (vol - expected).abs() / expected < 0.01,
				"ISO 10642 M{m}×{len}: want watertight genus-0 ~{expected:.1}mm³; got genus={genus} vol={vol:.1}"
			);
		}
		assert!(
			flat_head_screw(7.0, 20.0).is_none() && flat_head_screw(5.0, 3.0).is_none(),
			"M7 (out of table) and a 3 mm M5 (shorter than its head) must be refused"
		);
	}

	#[test]
	fn shoulder_bolts_step_thread_shoulder_head_per_iso7379() {
		// Shoulder Ø8 × 20 (M6 thread) and Ø13 × 30 (M10): three stacked 48-gon
		// cylinders (thread at major Ø × b, ground shoulder, head) minus the k/2 hex
		// socket — closed form to 1e-6 (every wall is an exact polygon prism); spans
		// exactly b + L + k tall and dk/2 wide. Ø12 (not an ISO 7379 size — the
		// standard jumps 10 → 13) must be refused.
		use kernel_brep::VertexId;
		let c48 = 24.0 * (2.0 * PI / 48.0).sin();
		for (d1, len) in [(8.0, 20.0), (13.0, 30.0)] {
			let (d3, b, dk, k, s) = iso7379_dims(d1).expect("table row");
			let bolt = shoulder_bolt(d1, len).expect("table size");
			let (vol, sound, genus) = measure(&bolt);
			let ring = |d: f64, h: f64| c48 * d * d * 0.25 * h;
			let expected = ring(d3, b) + ring(d1, len) + ring(dk, k) - hexagon_area(s) * (k * 0.5);
			let (mut rmax, mut zmax) = (0.0_f64, 0.0_f64);
			for i in 0..bolt.vertex_count() as u32 {
				let p = bolt.position(VertexId(i));
				rmax = rmax.max((p.x * p.x + p.y * p.y).sqrt());
				zmax = zmax.max(p.z);
			}
			assert!(
				sound && genus == 0
					&& (vol - expected).abs() / expected < 1e-6
					&& (rmax - dk * 0.5).abs() < 1e-9
					&& (zmax - (b + len + k)).abs() < 1e-9,
				"ISO 7379 Ø{d1}×{len}: want watertight genus-0, exactly {expected:.3}mm³, Ø{dk} × {} tall; got genus={genus} vol={vol:.3} rmax={rmax} zmax={zmax}",
				b + len + k
			);
		}
		assert!(
			shoulder_bolt(12.0, 20.0).is_none() && shoulder_bolt(8.0, f64::NAN).is_none(),
			"Ø12 (between the standard's 10 and 13) and NaN lengths must be refused"
		);
	}

	#[test]
	fn button_head_screws_match_iso7380_with_a_spherical_crown() {
		// M5×16 and M8×20 (under-head length): volume = shank + spherical cap
		// (π k (3a² + k²) / 6) − socket prism; 1.5% covers faceting plus the sliver of
		// socket prism that exits through the crown near the apex.
		for (m, len) in [(5.0, 16.0), (8.0, 20.0)] {
			let (dk, k, s, t) = iso7380_dims(m).expect("table row");
			let screw = button_head_screw(m, len).expect("table size");
			let (vol, sound, genus) = measure(&screw);
			let a = dk * 0.5;
			let cap = PI * k * (3.0 * a * a + k * k) / 6.0;
			let expected = PI * (m * 0.5) * (m * 0.5) * len + cap - hexagon_area(s) * t;
			assert!(
				sound && genus == 0 && (vol - expected).abs() / expected < 0.015,
				"ISO 7380 M{m}×{len}: want watertight genus-0 ~{expected:.1}mm³; got genus={genus} vol={vol:.1}"
			);
		}
		assert!(button_head_screw(16.0, 30.0).is_none(), "ISO 7380 stops at M12");
	}

	#[test]
	fn set_screws_carry_socket_and_cup_within_the_din916_envelope() {
		// M6×10 and M10×16: volume = π r² L − the 120° cup cone − the hex socket (1%).
		for (m, len) in [(6.0, 10.0), (10.0, 16.0)] {
			let (s, t, dv) = din916_dims(m).expect("table row");
			let screw = set_screw(m, len).expect("table size");
			let (vol, sound, genus) = measure(&screw);
			let cup_depth = dv * 0.5 * 30.0_f64.to_radians().tan();
			let cup = PI * (dv * 0.5).powi(2) * cup_depth / 3.0;
			let expected = PI * (m * 0.5) * (m * 0.5) * len - cup - hexagon_area(s) * t;
			assert!(
				sound && genus == 0 && (vol - expected).abs() / expected < 0.01,
				"DIN 916 M{m}×{len}: want watertight genus-0 ~{expected:.1}mm³; got genus={genus} vol={vol:.1}"
			);
		}
		assert!(set_screw(6.0, 2.0).is_none(), "a 2 mm M6 grub cannot hold its cup + socket");
	}

	#[test]
	fn lock_nuts_stack_the_din985_collar_on_the_hex_and_bore_through() {
		// M10 (DIN width 17!) and M5: volume = hex·m_hex + collar ring (minus the
		// Pappus chamfer ring) − the Ø m bore; genus 1; 1% band.
		for m in [10.0, 5.0] {
			let (s, h, mh, dw) = din985_dims(m).expect("table row");
			let nut = lock_nut(m).expect("table size");
			let (vol, sound, genus) = measure(&nut);
			let c = 0.3 * (h - mh);
			let chamfer_ring = 2.0 * PI * (dw * 0.5 - c / 3.0) * (c * c * 0.5);
			let expected = hexagon_area(s) * mh + PI * (dw * 0.5).powi(2) * (h - mh) - chamfer_ring - PI * (m * 0.5).powi(2) * h;
			assert!(
				sound && genus == 1 && (vol - expected).abs() / expected < 0.01,
				"DIN 985 M{m}: want watertight genus-1 ~{expected:.1}mm³; got genus={genus} vol={vol:.1}"
			);
		}
		assert_eq!(din985_dims(10.0).map(|r| r.0), Some(17.0), "DIN 985 keeps the 17 mm M10 width");
		assert!(lock_nut(7.0).is_none(), "M7 is not a DIN 985 size");
	}

	#[test]
	fn threaded_rod_and_standoffs_are_the_plain_table_envelopes() {
		// M8 rod × 60: chamfers are half the 1.25 pitch; volume closed-form (1%).
		// M3 standoff × 12: a bored AF 5.5 hex prism, exact areas (1%).
		let rod = threaded_rod(8.0, 60.0).expect("M8 rod");
		let (rv, rs, rg) = measure(&rod);
		let (r, c) = (4.0, 0.625);
		let frustum = PI * c / 3.0 * (r * r + r * (r - c) + (r - c) * (r - c));
		let rod_expected = PI * r * r * (60.0 - 2.0 * c) + 2.0 * frustum;
		let spacer = standoff(3.0, 12.0).expect("M3 spacer");
		let (sv, ss, sg) = measure(&spacer);
		let spacer_expected = (hexagon_area(5.5) - PI * 1.5 * 1.5) * 12.0;
		assert!(
			rs && rg == 0
				&& (rv - rod_expected).abs() / rod_expected < 0.01
				&& ss && sg == 1
				&& (sv - spacer_expected).abs() / spacer_expected < 0.01
				&& threaded_rod(7.0, 50.0).is_none()
				&& standoff(8.0, 20.0).is_none(),
			"M8×60 rod ~{rod_expected:.0}mm³ (got {rv:.0}, genus {rg}) and M3×12 standoff ~{spacer_expected:.1}mm³ (got {sv:.1}, genus {sg}); M7 rod / M8 standoff refused"
		);
	}
}
