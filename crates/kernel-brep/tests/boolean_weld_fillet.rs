//! Acceptance + honest limitation for filleting edges CREATED BY a boolean — the
//! FRICTION #20 territory (edge-feature witness resolution after booleans).
//!
//! Dogfooding an L (plate ∪ upright) shows the feared failure (re-tessellation
//! fragmenting a planar face into coplanar pieces so the seam edge can't be
//! resolved) does NOT occur for this case: the plate keeps all six faces and
//! every weld-edge name resolves to exactly one fragment. What IS limited is the
//! fillet itself — `fillet_edge` rounds **convex** dihedrals only (by design;
//! see fillet.rs), so the convex weld edges fillet to a valid solid while the
//! concave inside-corner edge honestly returns [`FilletError::Unsupported`].
//! That concave straight-edge fillet is the real remaining gap, distinct from
//! the fragmentation framing. If it is ever implemented, this test's
//! `concave_unsupported >= 1` expectation will flag that the limitation lifted.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, fillet_edge_near, union, validate, EdgeName, FaceName, FaceSource, FilletError};

fn fname(src: FaceSource, f: u32) -> FaceName {
	FaceName { operand: src, source_face: f }
}

#[test]
fn boolean_weld_edges_fillet_when_convex_and_refuse_concave_without_fragmenting() {
	let plate = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
	let upright = cuboid(DVec3::ZERO, DVec3::new(8.0, 40.0, 50.0));
	let u = union(&plate, &upright);
	assert!(validate(&u).is_valid(), "L-union must be a valid solid: {:?}", validate(&u));

	// No coplanar fragmentation: the plate (operand A) keeps its six cuboid faces.
	let plate_faces = u.faces().filter(|&f| u.face_source(f) == Some(FaceSource::OperandA)).count();

	// Midpoint of the concave weld line (upright side x=8 meets plate top z=8).
	let witness = DVec3::new(8.0, 20.0, 8.0);
	let (mut convex_ok, mut concave_unsupported, mut max_frags) = (0usize, 0usize, 0usize);
	for a in 0..6u32 {
		for b in 0..6u32 {
			let name = EdgeName::new(fname(FaceSource::OperandA, a), fname(FaceSource::OperandB, b));
			let frags = u.edges_named(name).len();
			if frags == 0 {
				continue;
			}
			max_frags = max_frags.max(frags);
			match fillet_edge_near(&u, name, 1.0, witness) {
				Ok(s) if validate(&s).is_valid() => convex_ok += 1,
				Err(FilletError::Unsupported) => concave_unsupported += 1,
				other => panic!("weld edge A{a}-B{b}: unexpected fillet outcome {other:?} (want a valid solid or Unsupported)"),
			}
		}
	}

	assert!(
		plate_faces == 6 && max_frags == 1 && convex_ok >= 3 && concave_unsupported >= 1,
		"boolean weld-edge fillet: plate kept {plate_faces}/6 faces (no coplanar fragmentation) and each weld name resolved to \
		 {max_frags} fragment(s); {convex_ok} convex weld edges filleted to valid solids and {concave_unsupported} concave weld \
		 edges returned Unsupported (fillet_edge is convex-only; concave straight-edge fillets are the documented remaining gap)"
	);
}
