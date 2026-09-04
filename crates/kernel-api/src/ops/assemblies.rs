// Copyright (c) LMCAD. Licensed under the MIT License.

//! In-program assembly ops — thin delegations to [`crate::asmops`], which holds the
//! instance/mate/DOF-honest-solve state machine and the assembly exports.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use crate::interp::{EnvValue, Outcome};
use crate::program::OpKind;
use crate::report::OpError;

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	asm: &mut crate::asmops::AsmProgramState,
	out_dir: &Path,
	input_base: &Path,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::AsmInstance { solid, name, translate, rotate, material } => {
			crate::asmops::instance(asm, env, all_ids, op_id, &solid, &name, &translate, &rotate, &material)
		}
		OpKind::AsmInstanceMesh { file, name, translate, rotate, material } => {
			crate::asmops::instance_mesh(asm, input_base, op_id, &file, &name, &translate, &rotate, &material)
		}
		OpKind::AsmMate {
			kind,
			a,
			b,
			a_point,
			b_point,
			a_dir,
			b_dir,
			a_axis_point,
			a_axis_dir,
			b_axis_point,
			b_axis_dir,
			distance,
			degrees,
		} => crate::asmops::mate(
			asm,
			op_id,
			&kind,
			&a,
			&b,
			&a_point,
			&b_point,
			&a_dir,
			&b_dir,
			&a_axis_point,
			&a_axis_dir,
			&b_axis_point,
			&b_axis_dir,
			&distance,
			&degrees,
		),
		OpKind::AsmMateAxis { a, a_witness, b, b_witness, distance } => {
			crate::asmops::mate_axis(asm, env, all_ids, op_id, &a, a_witness, &b, b_witness, &distance)
		}
		OpKind::AsmMateFace { a, a_witness, b, b_witness, offset } => {
			crate::asmops::mate_face(asm, env, all_ids, op_id, &a, a_witness, &b, b_witness, &offset)
		}
		OpKind::AsmSolve { iterations, max_residual, allow_unconverged } => {
			crate::asmops::solve(asm, op_id, &iterations, &max_residual, allow_unconverged)
		}
		OpKind::AsmContacts { window, tol } => crate::asmops::contacts(asm, env, all_ids, op_id, &window, &tol),
		OpKind::AsmInterferenceVolume { a, b, voxel } => crate::asmops::interference_volume(asm, env, all_ids, op_id, &a, &b, &voxel),
		OpKind::AsmMassProperties {} => crate::asmops::mass_properties(asm, env, all_ids, op_id),
		OpKind::AsmExport { file, parts_dir, tol, voxel } => {
			crate::asmops::export(asm, env, all_ids, op_id, out_dir, &file, &parts_dir, &tol, &voxel)
		}
		OpKind::AsmExportStep { file } => crate::asmops::export_step(asm, env, all_ids, op_id, out_dir, &file),
		OpKind::AsmSave { file, name, parts_dir } => crate::asmops::save(asm, env, all_ids, op_id, out_dir, &file, &name, &parts_dir),
		OpKind::GearTrainPoses { sun_teeth, ring1_teeth, planet_a_teeth, planet_b_teeth, ring2_teeth, n_planets, module, theta_deg } => {
			crate::asmops::gear_train_poses(
				op_id,
				sun_teeth,
				ring1_teeth,
				planet_a_teeth,
				planet_b_teeth,
				ring2_teeth,
				n_planets,
				module,
				theta_deg,
			)
		}

		_ => unreachable!("ops::assemblies: op routed to the wrong family"),
	}
}
