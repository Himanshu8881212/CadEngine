#!/usr/bin/env python3
"""test_aux_tools.py — regression tests for the documentation / render / audit
tools and the density-field staleness guard.

    python3 tools/test_aux_tools.py            # all
    python3 tools/test_aux_tools.py -k parity  # substring filter

Every test here FAILS on the pre-2026-08-08 tools. Each names the friction entry
it pins. No network, no clock, no campaign directory is read or written; every
artifact goes to a temp dir that is removed on exit.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/
sys.path.insert(0, HERE)
import _layout  # noqa: E402
_layout.add_import_paths()

from _stl import load_stl, write_stl  # noqa: E402
import analysis_sheet  # noqa: E402
import voxelize_stl as vox  # noqa: E402

PY = sys.executable
TESTS = []


def test(fn):
	TESTS.append(fn)
	return fn


def run_tool(tool, *args):
	p = subprocess.run([PY, str(_layout.find_tool(tool)), *args], capture_output=True, text=True)
	line = [l for l in p.stdout.strip().splitlines() if l.strip()]
	try:
		rec = json.loads(line[-1]) if line else None
	except json.JSONDecodeError:
		rec = None
	return p.returncode, rec, p.stdout, p.stderr


# --------------------------------------------------------------- fixtures --
def ring(r, z, n=48):
	a = np.linspace(0, 2 * np.pi, n, endpoint=False)
	return np.stack([r * np.cos(a), r * np.sin(a), np.full(n, z)], 1)


def tube_stl(path, r_out=15.0, r_in=5.0, z0=0.0, z1=12.0,
             z_out_split=6.0, z_in_split=6.5, n=48):
	"""Annular tube carrying an explicit VERTEX RING at `z_in_split` on the bore
	wall — exactly what an exact-B-rep STL export emits at the mid-height of a
	cylindrical face, and what used to delete that slice's bore contour."""
	tris = []
	for r, zs, flip in ((r_out, [z0, z_out_split, z1], False),
	                    (r_in, [z0, z_in_split, z1], True)):
		for k in range(len(zs) - 1):
			A, B = ring(r, zs[k], n), ring(r, zs[k + 1], n)
			for i in range(n):
				j = (i + 1) % n
				for t in ((A[i], A[j], B[j]), (A[i], B[j], B[i])):
					tris.append(t[::-1] if flip else t)
	for z, flip in ((z0, True), (z1, False)):
		O, I = ring(r_out, z, n), ring(r_in, z, n)
		for i in range(n):
			j = (i + 1) % n
			for t in ((O[i], O[j], I[j]), (O[i], I[j], I[i])):
				tris.append(t[::-1] if flip else t)
	write_stl(path, np.array(tris, dtype=np.float64))


def box_stl(path, lo=(0, 0, 0), hi=(10, 10, 10), rot_deg=0.0):
	lo, hi = np.array(lo, float), np.array(hi, float)
	v = []
	for d in range(3):
		for s in (0, 1):
			ax = [0, 1, 2]
			ax.remove(d)
			u, w = ax

			def P(uu, ww, d=d, s=s, u=u, w=w):
				p = np.zeros(3)
				p[d] = hi[d] if s else lo[d]
				p[u], p[w] = uu, ww
				return p
			q = [P(lo[u], lo[w]), P(hi[u], lo[w]), P(hi[u], hi[w]), P(lo[u], hi[w])]
			v += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
	tris = np.array(v, dtype=np.float64)
	if rot_deg:
		th = np.deg2rad(rot_deg)
		R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
		tris = tris @ R.T
	write_stl(path, tris)


def polygon_truth(r_out, r_in, n, origin, h, shape):
	"""Independent oracle: matplotlib point-in-polygon on the exact n-gon
	annulus the tessellation describes. Nothing in tools/ is involved."""
	from matplotlib.path import Path
	xs = origin[0] + (np.arange(shape[0]) + 0.5) * h
	ys = origin[1] + (np.arange(shape[1]) + 0.5) * h
	X, Y = np.meshgrid(xs, ys, indexing="ij")
	P = np.stack([X.ravel(), Y.ravel()], 1)
	def poly(r):
		a = np.linspace(0, 2 * np.pi, n, endpoint=False)
		return np.stack([r * np.cos(a), r * np.sin(a)], 1)
	inside = Path(poly(r_out)).contains_points(P) & ~Path(poly(r_in)).contains_points(P)
	return inside.reshape(shape[0], shape[1])


# ------------------------------------------------------ parity-fill (T9/F8) --
@test
def test_parity_fill_vertex_ring_on_slice_centre(tmp):
	"""horn F8 / cubesat F4: a vertex ring landing exactly on a slice centre used
	to erase the bore contour (slice reads solid, or vanishes entirely)."""
	stl = os.path.join(tmp, "tube.stl")
	tube_stl(stl)                       # bore ring at z = 6.5 = a slice centre at h = 1.0
	origin, h, shape = np.array([-15.0, -15.0, 0.0]), 1.0, (30, 30, 12)
	mat = vox.parity_fill(load_stl(stl), origin, h, shape)
	per_slice = [int(mat[:, :, k].sum()) for k in range(shape[2])]
	assert len(set(per_slice)) == 1, f"slice-dependent fill (degeneracy leaked): {per_slice}"
	# dead centre of a wide-open Ø10 bore must never be material
	assert not mat[15, 15, :].any(), "phantom material inside an open bore"
	truth = polygon_truth(15.0, 5.0, 48, origin, h, shape)
	for k in range(shape[2]):
		assert (mat[:, :, k] == truth).all(), f"slice {k} != independent polygon oracle"


@test
def test_parity_fill_axis_aligned_box_is_exact(tmp):
	"""A 10x10x10 box on a 1 mm grid is 1000 cells. The old rule gave 900."""
	stl = os.path.join(tmp, "box.stl")
	box_stl(stl)
	mat = vox.parity_fill(load_stl(stl), np.array([0.0, 0.0, 0.0]), 1.0, (10, 10, 10))
	assert int(mat.sum()) == 1000, f"axis-aligned box filled {int(mat.sum())}/1000"
	assert mat.all()


@test
def test_parity_fill_nondegenerate_is_unchanged(tmp):
	"""The half-open rule must only change DEGENERATE configurations: with no
	vertex on any slice/scanline it agrees with the historical `da*db < 0` rule."""
	stl = os.path.join(tmp, "boxr.stl")
	box_stl(stl, lo=(-5, -5, -5), hi=(5, 5, 5), rot_deg=31.7)
	tris = load_stl(stl)
	origin, h, shape = np.array([-8.0, -8.0, -6.0]), 0.7, (24, 24, 18)
	new = vox.parity_fill(tris, origin, h, shape)
	old = _legacy_parity_fill(tris, origin, h, shape)
	assert (new == old).all(), f"{int((new ^ old).sum())} cells differ on a non-degenerate mesh"


def _legacy_parity_fill(tris, lo, h, shape):
	"""The pre-fix algorithm, verbatim, kept ONLY as the comparison baseline."""
	nx, ny, nz = shape
	mat = np.zeros((nx, ny, nz), dtype=bool)
	xs = lo[0] + (np.arange(nx) + 0.5) * h
	ys = lo[1] + (np.arange(ny) + 0.5) * h
	zmin, zmax = tris[:, :, 2].min(1), tris[:, :, 2].max(1)
	for k in range(nz):
		z = lo[2] + (k + 0.5) * h
		sel = tris[(zmin < z) & (zmax > z)]
		if not len(sel):
			continue
		segs = []
		for t in sel:
			pts = []
			for i in range(3):
				a, b = t[i], t[(i + 1) % 3]
				da, db = a[2] - z, b[2] - z
				if da * db < 0:
					s = da / (da - db)
					pts.append((a + s * (b - a))[:2])
			if len(pts) == 2:
				segs.append(pts)
		if not segs:
			continue
		S = np.array(segs)
		y1, y2 = S[:, 0, 1], S[:, 1, 1]
		for j in range(ny):
			yj = ys[j]
			cross = (y1 - yj) * (y2 - yj) < 0
			if not cross.any():
				continue
			cs = S[cross]
			tp = (yj - cs[:, 0, 1]) / (cs[:, 1, 1] - cs[:, 0, 1])
			xi = np.sort(cs[:, 0, 0] + tp * (cs[:, 1, 0] - cs[:, 0, 0]))
			mat[:, j, k] |= (np.searchsorted(xi, xs, side="right") % 2).astype(bool)
	return mat


# ------------------------------------------------- air topology (T9 / F13) --
@test
def test_air_topology_open_bore_is_connected(tmp):
	"""horn F8: a wide-open through-bore was reported SEVERED at a commensurate
	pitch. It must read connected, and the seed volume must be joinable."""
	stl = os.path.join(tmp, "tube.stl")
	tube_stl(stl)
	job = {"stl": stl, "voxel_mm": 1.0, "wall_margin_mm": 0,
	       "seeds": {"bottom": [0.0, 0.0, 1.0], "top": [0.0, 0.0, 11.0]},
	       "require_connected": [["bottom", "top"]],
	       "receipt": os.path.join(tmp, "air.json")}
	jp = os.path.join(tmp, "air.json.job")
	json.dump(job, open(jp, "w"))
	code, rec, _, err = run_tool("air_topology_audit.py", jp)
	assert rec is not None, err
	assert rec["ok"] is True and code == 0, rec
	assert rec["connected"]["bottom<->top"] is True, rec
	# horn F13: seed_labels / sizes_cm3 could not be joined; these two can be
	assert rec["seed_sizes_cm3"]["bottom"] == rec["seed_sizes_cm3"]["top"], rec
	assert str(rec["seed_labels"]["bottom"]) in rec["component_sizes_cm3"], rec
	assert len(rec["component_sizes_cm3"]) == rec["components"], rec
	# cubesat F11: the job's `receipt` key must actually produce a file
	assert os.path.exists(job["receipt"]), "job 'receipt' key was ignored"


@test
def test_air_topology_seed_in_material_is_refused(tmp):
	"""A seed pointing at a wall must not masquerade as `connected: false`."""
	stl = os.path.join(tmp, "tube.stl")
	tube_stl(stl)
	jp = os.path.join(tmp, "airbad.json")
	json.dump({"stl": stl, "voxel_mm": 1.0, "wall_margin_mm": 0,
	           "seeds": {"wall": [10.0, 0.0, 6.0], "top": [0.0, 0.0, 11.0]},
	           "require_connected": [["wall", "top"]]}, open(jp, "w"))
	code, rec, _, _ = run_tool("air_topology_audit.py", jp)
	assert code == 1 and rec["ok"] is False, rec
	assert "seed placement refused" in rec["error"] and "wall" in rec["error"], rec


@test
def test_air_topology_exit_1_on_disconnected(tmp):
	stl = os.path.join(tmp, "sealed.stl")
	box_stl(stl, hi=(20, 20, 20))          # solid: the two seeds land in material
	jp = os.path.join(tmp, "sealed.json")
	json.dump({"stl": stl, "voxel_mm": 1.0, "wall_margin_mm": 0,
	           "seeds": {"a": [1.0, 1.0, 1.0]}, "require_connected": [["a", "a"]]}, open(jp, "w"))
	code, rec, _, _ = run_tool("air_topology_audit.py", jp)
	assert code == 1 and rec["ok"] is False, (code, rec)


# ---------------------------------------------- staleness guard (gripper F8) --
@test
def test_voxelize_stamps_and_verifies_freshness(tmp):
	stl = os.path.join(tmp, "box.stl")
	box_stl(stl)
	npy = os.path.join(tmp, "field.npy")
	jp = os.path.join(tmp, "vox.json")
	job = {"stl": stl, "origin_mm": [0, 0, 0], "voxel_mm": 1.0, "shape": [10, 10, 10], "out": npy}
	json.dump(job, open(jp, "w"))
	code, rec, _, err = run_tool("voxelize_stl.py", jp)
	assert code == 0 and rec["ok"], (rec, err)
	assert rec["geometry_hash"].startswith("mesh:sha256:"), rec
	assert rec["field_hash"].startswith("density:sha256:"), rec
	assert os.path.exists(vox.provenance_path(npy)), "no freshness sidecar written"

	code, rec, _, _ = run_tool("voxelize_stl.py", "--verify", jp)
	assert code == 0 and rec["fresh"] is True, rec

	# THE gripper F8 scenario: geometry is amended, the field is not re-voxelised
	box_stl(stl, hi=(11, 10, 10))
	code, rec, _, _ = run_tool("voxelize_stl.py", "--verify", jp)
	assert code == 1 and rec["ok"] is False and rec["fresh"] is False, rec
	assert "source_changed" in rec["reasons"], rec

	# a field with no record is UNKNOWN, never assumed fresh
	os.remove(vox.provenance_path(npy))
	code, rec, _, _ = run_tool("voxelize_stl.py", "--verify-field", npy)
	assert code == 1 and rec["fresh"] is None and "no_provenance" in rec["reasons"], rec


@test
def test_voxelize_verify_catches_a_changed_grid(tmp):
	stl = os.path.join(tmp, "box.stl")
	box_stl(stl)
	npy = os.path.join(tmp, "f.npy")
	jp = os.path.join(tmp, "v.json")
	json.dump({"stl": stl, "origin_mm": [0, 0, 0], "voxel_mm": 1.0,
	           "shape": [10, 10, 10], "out": npy}, open(jp, "w"))
	run_tool("voxelize_stl.py", jp)
	json.dump({"stl": stl, "origin_mm": [0, 0, 0], "voxel_mm": 0.5,
	           "shape": [20, 20, 20], "out": npy}, open(jp, "w"))
	code, rec, _, _ = run_tool("voxelize_stl.py", "--verify", jp)
	assert code == 1 and "grid_changed" in rec["reasons"], rec


@test
def test_voxelize_is_deterministic(tmp):
	stl = os.path.join(tmp, "box.stl")
	box_stl(stl, rot_deg=17.0)
	out = []
	for i in (0, 1):
		d = os.path.join(tmp, f"run{i}")
		os.makedirs(d, exist_ok=True)
		npy = os.path.join(d, "field.npy")     # same NAME, different run dir
		jp = os.path.join(d, "job.json")
		json.dump({"stl": stl, "origin_mm": [-9, -9, 0], "voxel_mm": 0.6,
		           "shape": [30, 30, 18], "out": npy}, open(jp, "w"))
		run_tool("voxelize_stl.py", jp)
		out.append(open(npy, "rb").read())
		out.append(json.dumps(json.load(open(vox.provenance_path(npy)))["field"], sort_keys=True))
	assert out[0] == out[2], "voxel field is not byte-reproducible"
	assert out[1] == out[3], "provenance sidecar is not byte-reproducible"


# --------------------------------------------------- analysis_sheet (F7/F8) --
def _sheet_job(tmp, **over):
	stl = os.path.join(tmp, "box.stl")
	if not os.path.exists(stl):
		box_stl(stl)
	npy = os.path.join(tmp, "stress.npy")
	if not os.path.exists(npy):
		f = np.zeros((10, 10, 10), np.float32)
		f[5, 5, 5] = 1.92e7
		np.save(npy, f)
	job = {"title": "t", "date": "2026-08-08", "out": os.path.join(tmp, "s.png"),
	       "results": [["a", "b"]],
	       "panels": [{"kind": "view", "stl": stl,
	                   "loads": [{"at": [5, 5, 5], "dir": [0, 0, -1]}]},
	                  {"kind": "field", "caption": "stress", "stl": stl, "npy": npy,
	                   "origin_mm": [0, 0, 0], "voxel_mm": 1.0, "unit": "MPa"}]}
	job.update(over)
	jp = os.path.join(tmp, "sheet.json")
	json.dump(job, open(jp, "w"))
	return jp, job


@test
def test_analysis_sheet_load_without_label(tmp):
	"""cubesat F7: `loads[].label` is an annotation, not a requirement."""
	jp, _ = _sheet_job(tmp)
	code, rec, _, err = run_tool("analysis_sheet.py", jp)
	assert code == 0 and rec["ok"], (rec, err)


@test
def test_analysis_sheet_missing_key_is_a_named_refusal(tmp):
	"""The failure must arrive as a RECEIPT on stdout, naming panel and key —
	not as a bare traceback on stderr with an empty stdout (the silence class:
	the documented `| tail -1 > f.json` idiom then writes a 0-byte receipt)."""
	jp, job = _sheet_job(tmp)
	job["panels"][0]["loads"][0].pop("at")
	json.dump(job, open(jp, "w"))
	code, rec, out, _ = run_tool("analysis_sheet.py", jp)
	assert code == 1, code
	assert rec is not None and rec["ok"] is False, out
	assert "panel 0 load 0" in rec["error"] and "'at'" in rec["error"], rec


@test
def test_analysis_sheet_unit_conversion(tmp):
	"""cubesat F8: `unit` was decorative, so a Pa field labelled MPa drew a bar
	reading 1.92e+07. `field_unit` makes the conversion real and checked."""
	jp, job = _sheet_job(tmp)
	job["panels"][1]["field_unit"] = "Pa"
	json.dump(job, open(jp, "w"))
	code, rec, _, _ = run_tool("analysis_sheet.py", jp)
	assert code == 0 and rec["ok"], rec
	assert rec["fields"][0]["scale"] == 1e-6, rec
	assert "warnings" not in rec, rec

	job["panels"][1]["field_unit"] = "m"                       # length -> pressure
	json.dump(job, open(jp, "w"))
	code, rec, _, _ = run_tool("analysis_sheet.py", jp)
	assert code == 1 and "dimension mismatch" in rec["error"], rec

	job["panels"][1]["field_unit"] = "Pa"
	job["panels"][1]["scale"] = 1.0                            # disagrees with Pa->MPa
	json.dump(job, open(jp, "w"))
	code, rec, _, _ = run_tool("analysis_sheet.py", jp)
	assert code == 1 and "unit_scale_conflict" in rec["error"], rec


@test
def test_analysis_sheet_unverified_unit_warns(tmp):
	jp, _ = _sheet_job(tmp)
	_, rec, _, _ = run_tool("analysis_sheet.py", jp)
	assert any("LABEL only" in w for w in rec.get("warnings", [])), rec


@test
def test_units_table_refuses_temperature_offsets():
	assert analysis_sheet.convert_factor("Pa", "MPa") == 1e-6
	assert analysis_sheet.convert_factor("m", "mm") == 1000.0
	for bad in (("K", "degC"), ("m", "N"), ("Pa", "furlong")):
		try:
			analysis_sheet.convert_factor(*bad)
		except ValueError:
			continue
		raise AssertionError(f"convert_factor{bad} should have refused")


# ------------------------------------------------- assembly_doc (F11 / F12) --
def _asm_job(tmp, **over):
	a, b = os.path.join(tmp, "box.stl"), os.path.join(tmp, "boxr.stl")
	if not os.path.exists(a):
		box_stl(a)
	if not os.path.exists(b):
		box_stl(b, lo=(-5, -5, -5), hi=(5, 5, 5), rot_deg=31.7)
	job = {"parts": [{"name": "base", "stl": a}, {"name": "lid", "stl": b}],
	       "explode": {"axis": [0, 0, 1], "auto": True, "gap_mm": 8},
	       "steps": [{"order": 1, "text": "Short step."}],
	       "date": "2026-08-08", "max_px": 1200,
	       "doc_title": "fixed title",     # so two out_prefixes render the same sheet
	       "out_prefix": os.path.join(tmp, "asm")}
	job.update(over)
	jp = os.path.join(tmp, "asm.json")
	json.dump(job, open(jp, "w"))
	return jp, job


@test
def test_assembly_doc_axis_name_and_view_list(tmp):
	"""ball F6 / singulator F12(a): `axis: "z"`; horn F11: `view: [elev, azim]`.
	Both must render, and render the SAME sheet as the canonical spellings."""
	jp, job = _asm_job(tmp, view={"elev": 18, "azim": -60})
	code, rec, _, err = run_tool("assembly_doc.py", jp)
	assert code == 0 and rec["ok"], (rec, err)
	canonical = open(rec["png"], "rb").read()

	job["explode"]["axis"] = "z"
	job["view"] = [18, -60]
	job["out_prefix"] = os.path.join(tmp, "asm2")
	json.dump(job, open(jp, "w"))
	code, rec2, _, _ = run_tool("assembly_doc.py", jp)
	assert code == 0 and rec2["ok"], rec2
	assert open(rec2["png"], "rb").read() == canonical, "'z' and [0,0,1] disagree"
	assert rec2["explode_axis"] == [0.0, 0.0, 1.0], rec2

	job["explode"]["axis"] = "q"
	json.dump(job, open(jp, "w"))
	code, rec3, _, _ = run_tool("assembly_doc.py", jp)
	assert code == 1 and "explode.axis" in rec3["error"], rec3

	job["explode"]["axis"] = "z"
	job["view"] = "sideways"
	json.dump(job, open(jp, "w"))
	code, rec4, _, _ = run_tool("assembly_doc.py", jp)
	assert code == 1 and "view" in rec4["error"], rec4


@test
def test_assembly_doc_page_grows_instead_of_refusing(tmp):
	"""singulator F12(b): a documentation-rich assembly was refused outright. The
	page grows; capping the growth restores the old refusal, with its height."""
	prose = ("Seat the part fully against the previous assembly and verify the fit is square "
	         "before continuing; the shoulder must bottom out on the boss face, and any rock "
	         "at this stage means the pocket needs a light deburr, not more force. ")
	parts = [{"name": f"p{i:02d}", "stl": os.path.join(tmp, "box.stl")} for i in range(10)]
	jp, job = _asm_job(tmp, parts=parts,
	                   explode={"axis": "z", "spacing_mm": 14},
	                   steps=[{"order": i + 1, "text": f"Step {i + 1}. " + prose} for i in range(8)],
	                   out_prefix=os.path.join(tmp, "grow"))
	code, rec, _, err = run_tool("assembly_doc.py", jp)
	assert code == 0 and rec["ok"], (rec, err)
	assert rec["page_grew"] is True and rec["page_h_in"] > 10.0, rec

	job["max_page_h_in"] = 10.0
	json.dump(job, open(jp, "w"))
	code, rec2, _, _ = run_tool("assembly_doc.py", jp)
	assert code == 1 and "maximum page height" in rec2["error"], rec2


@test
def test_assembly_doc_short_job_does_not_grow(tmp):
	jp, _ = _asm_job(tmp, out_prefix=os.path.join(tmp, "small"))
	_, rec, _, _ = run_tool("assembly_doc.py", jp)
	assert rec["page_h_in"] == 10.0 and rec["page_grew"] is False, rec


# ------------------------------------------------------ render_sheet (F10/F4) --
@test
def test_render_sheet_resolves_job_relative_paths(tmp):
	"""gripper F10: a job carrying `parts/x.stl` must rebuild from the repo root,
	not only from the one directory the README does not name."""
	part = os.path.join(tmp, "part")
	os.makedirs(os.path.join(part, "parts"), exist_ok=True)
	os.makedirs(os.path.join(part, "programs", "jobs"), exist_ok=True)
	box_stl(os.path.join(part, "parts", "p.stl"))
	jp = os.path.join(part, "programs", "jobs", "r.json")
	json.dump({"stl": "parts/p.stl", "out": "renders/p.png", "date": "2026-08-08"},
	          open(jp, "w"))
	cwd = os.getcwd()
	try:
		os.chdir(part)                                   # the old happy path
		code, rec, _, _ = run_tool("render_sheet.py", "programs/jobs/r.json")
		assert code == 0 and rec["ok"], rec
		assert "notes" not in rec, "CWD resolution must stay silent (unchanged)"
		os.chdir(tmp)                                    # the F10 path
		code, rec, _, err = run_tool("render_sheet.py", jp)
		assert code == 0 and rec["ok"], (rec, err)
		assert any("job file's directory" in n for n in rec["notes"]), rec
	finally:
		os.chdir(cwd)


@test
def test_render_sheet_missing_input_names_the_roots(tmp):
	jp = os.path.join(tmp, "bad.json")
	json.dump({"stl": "parts/nope.stl", "out": "x.png"}, open(jp, "w"))
	code, rec, _, _ = run_tool("render_sheet.py", jp)
	assert code == 1 and rec["ok"] is False, rec
	assert "cwd" in rec["error"] and "job file" in rec["error"], rec


@test
def test_render_sheet_overlay_legend_clears_the_meta_line(tmp):
	"""din_rail F4: with a long combined title the legend overprinted the meta
	string. It must move to its own strip and SAY so."""
	stls = []
	for i in range(5):
		p = os.path.join(tmp, f"a_rather_long_part_name_{i}.stl")
		box_stl(p, lo=(i, 0, 0), hi=(i + 8, 8, 8))
		stls.append(p)
	jp = os.path.join(tmp, "ov.json")
	json.dump({"stls": stls, "out": os.path.join(tmp, "ov.png"), "date": "2026-08-08"},
	          open(jp, "w"))
	code, rec, _, err = run_tool("render_sheet.py", jp)
	assert code == 0 and rec["ok"], (rec, err)
	assert any("legend moved" in n for n in rec.get("notes", [])), rec


@test
def test_render_sheet_is_deterministic(tmp):
	stl = os.path.join(tmp, "box.stl")
	if not os.path.exists(stl):
		box_stl(stl)
	jp = os.path.join(tmp, "det.json")
	digests = []
	for i in (0, 1):
		out = os.path.join(tmp, f"det{i}.png")
		json.dump({"stl": stl, "out": out, "date": "2026-08-08"}, open(jp, "w"))
		run_tool("render_sheet.py", jp)
		digests.append(open(out, "rb").read())
	assert digests[0] == digests[1], "render_sheet is not byte-reproducible"


# ---------------------------------------------------------- bom_audit (F5) --
def _step_tree(path, names):
	L = ["ISO-10303-21;", "DATA;"]
	for n, inst in names.items():
		L.append(f"#1=PRODUCT('{n}','x','',(#2));")           # 1 metadata occurrence
		for _ in range(inst):
			L.append(f"#9=NEXT_ASSEMBLY_USAGE_OCCURRENCE('{n}','x','',#1,#2,$);")
		L.append("")
	L += ["ENDSEC;", "END-ISO-10303-21;"]
	os.makedirs(os.path.dirname(path), exist_ok=True)
	open(path, "w").write("\n".join(L))


@test
def test_bom_audit_is_job_driven(tmp):
	"""digest F5: the tool hardcoded one project's STEP trees and hardware table."""
	root = os.path.join(tmp, "bom")
	_step_tree(os.path.join(root, "A", "ASSEMBLY.step"),
	           {"hw_motor": 1, "hw_m3x10": 4, "hw_special_a": 2})
	_step_tree(os.path.join(root, "B", "ASSEMBLY.step"),
	           {"hw_motor": 1, "hw_m3x10": 6})
	job = {"assemblies": [{"name": "A", "step": "A/ASSEMBLY.step"},
	                      {"name": "B", "step": "B/ASSEMBLY.step"}],
	       "calibrate_with": "hw_motor",
	       "bom": {"hw_motor": {"label": "motor"},
	               "hw_m3x10": {"label": "M3x10", "expect": 10},
	               "hw_special_a": {"label": "A only", "only": ["A"]}}}
	jp = os.path.join(root, "job.json")
	json.dump(job, open(jp, "w"))
	code, rec, _, err = run_tool("bom_audit.py", jp)
	assert code == 0 and rec["ok"], (rec, err)
	assert rec["family_totals"] == {"hw_m3x10": 10, "hw_motor": 2, "hw_special_a": 2}, rec

	job["bom"]["hw_special_a"]["only"] = ["B"]               # now misplaced
	json.dump(job, open(jp, "w"))
	code, rec, _, _ = run_tool("bom_audit.py", jp)
	assert code == 1 and rec["ok"] is False, rec
	assert any("restricted to" in f for f in rec["findings"]), rec

	_step_tree(os.path.join(root, "B", "ASSEMBLY.step"),
	           {"hw_motor": 1, "hw_m3x10": 6, "hw_rogue": 1})
	job["bom"]["hw_special_a"]["only"] = ["A"]
	json.dump(job, open(jp, "w"))
	code, rec, _, _ = run_tool("bom_audit.py", jp)
	assert code == 1 and any("NOT in the unified BOM" in f for f in rec["findings"]), rec


@test
def test_bom_audit_example_job_is_wellformed(tmp):
	p = subprocess.run([PY, str(_layout.find_tool("bom_audit.py")), "--example"],
	                   capture_output=True, text=True)
	assert p.returncode == 0
	job = json.loads(p.stdout)
	assert [a["name"] for a in job["assemblies"]] == ["cyclo26", "harmonic26", "planetary26"]
	assert job["bom"]["hw_dowel_2x20"]["only"] == ["cyclo26"]


# --------------------------------------------------------------- --help (F9) --
@test
def test_help_never_crashes():
	"""digest F9: the doc tools treated `--help` as a job file and stack-traced."""
	for tool in ("render_sheet.py", "analysis_sheet.py", "assembly_doc.py",
	             "production_dossier.py", "motion_gif.py", "document_bundle.py",
	             "air_topology_audit.py", "voxelize_stl.py", "bom_audit.py",
	             "render_views.py", "stress_to_density.py"):
		p = subprocess.run([PY, str(_layout.find_tool(tool)), "--help"],
		                   capture_output=True, text=True)
		assert p.returncode == 0, f"{tool} --help exited {p.returncode}: {p.stderr[-300:]}"
		assert "Traceback" not in p.stderr, f"{tool} --help traced back"
		assert len(p.stdout.strip()) > 40, f"{tool} --help printed nothing useful"


# ---------------------------------------------------------------- runner --
def main(argv):
	filt = None
	if "-k" in argv:
		filt = argv[argv.index("-k") + 1]
	failed = []
	for fn in TESTS:
		if filt and filt not in fn.__name__:
			continue
		tmp = tempfile.mkdtemp(prefix="auxtools_")
		try:
			if fn.__code__.co_argcount:
				fn(tmp)
			else:
				fn()
			print(f"ok    {fn.__name__}")
		except Exception as e:  # noqa: BLE001
			failed.append((fn.__name__, e))
			print(f"FAIL  {fn.__name__}: {type(e).__name__}: {e}")
		finally:
			shutil.rmtree(tmp, ignore_errors=True)
	print(f"\n{len(TESTS) - len(failed)}/{len(TESTS)} passed")
	return 1 if failed else 0


if __name__ == "__main__":
	sys.exit(main(sys.argv[1:]))
