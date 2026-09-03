# Owner E — aux tools + staleness — fixlog

Owned: tools/{analysis_sheet,render_sheet,render_views,assembly_doc,production_dossier,
air_topology_audit,voxelize_stl,bom_audit,document_bundle,motion_gif,stress_to_density}.py

Scratch/repro dir: /private/tmp/claude-501/-Users-himanshu-Work-New-LMCAD/eef3ec62-58a7-4682-b52d-7dd8bef08b42/scratchpad/repro

## REPRODUCED (before any edit)

### R1 — parity-fill slice-plane degeneracy (the false-seal / phantom-solid class)
Synthetic tube: outer Ø30, bore Ø10, z 0..12, with an explicit VERTEX RING at
z = 6.5 on the bore wall (exactly what exact-B-rep STL exports emit at the
mid-height of a cylindrical face) and at z = 6.0 on the outer wall.
Grid voxel_mm 1.0 -> slice centres at z = k+0.5, so z=6.5 IS a slice centre.

- `voxelize_stl.py`  -> slice k=6 has 656 solid voxels vs 592 on every other
  slice, and `rho[15,15,6]` (dead centre of a wide-open Ø10 bore) is **True**.
  The bore is FILLED WITH PHANTOM MATERIAL on that layer.
- `air_topology_audit.py` -> `{"ok": false, "components": 8,
  "seed_labels": {"bottom": 3, "top": 4}, "connected": {"bottom<->top": false}}`
  i.e. it reports a completely open through-bore as SEVERED. exit 0.
- If BOTH walls' rings land on the slice (tube.stl), the whole slice voxelizes
  to ZERO solid — a whole layer of material vanishes.

Root cause (identical code in both files): the contour builder keeps an edge
only when `da*db < 0`, so a triangle with a vertex exactly ON the slice plane
(`da == 0`) contributes at most one crossing and is dropped by `len(pts)==2`.
The y-scanline has the same defect (`(y1-yj)*(y2-yj) < 0`).

### R2 — no freshness guard on the density field
`voxelize_stl.py` writes a bare `.npy`. Nothing records the source STL's hash,
so a consumer cannot tell a fresh field from one built before the last geometry
amendment (gripper F8: shipped 62552 vs fresh 64328 solid voxels, f1 70.54 ->
65.84 Hz). `geometry_hash` exists in receipts but is read by NOTHING.

### R3 — analysis_sheet: bare KeyError on a load with no `label`; no unit conversion
### R4 — assembly_doc: `view` list crashes; `explode.axis` string crashes;
        long step prose is REFUSED on a fixed-height page
### R5 — render_sheet: job-relative paths resolved against CWD only; header
        legend collides with the right-aligned meta string
### R6 — `--help` treated as a job path by every doc tool (crash, no receipt)
### R7 — bom_audit.py hardcoded to cyclo26/harmonic26/planetary26
### R8 — air_topology_audit ignores the job's `receipt` key; `seed_labels` and
        `sizes_cm3` cannot be joined (sizes_cm3 is sorted+truncated to 8)

## LANDED

### 1. `voxelize_stl.py` — rewritten (parity fill + freshness)
- `parity_fill(tris, origin, h, shape)` is now the ONE shared fill; half-open
  sign rule `(d > 0)` on BOTH the slice plane and the y scanline.
- PROVED against an independent oracle (matplotlib.path point-in-polygon on the
  analytic 48-gon annulus): new fill = 628 cells = ground truth EXACTLY;
  old fill = 592 (under-filled by 36 on EVERY slice, degenerate or not).
  Axis-aligned 10×10×10 box at h=1: old 900, new 1000 (= truth).
  Rotated box (no degeneracies): old 3045, new 3045 — IDENTICAL.
  => the change touches ONLY degenerate configurations, and always corrects them.
- Freshness: writes `<out>.provenance.json` (schema lmcad.field.provenance.v1)
  with source sha256 + `mesh:sha256:` + `density:sha256:` in provenance.py's OWN
  vocabulary, so the hash is comparable with the ACE runners' `geometry_hash`.
  `check_field_freshness()` is the consumer entry point; `--verify` / `--verify-field`
  are its CLI. Stale => ok:false + exit 1. Absent sidecar => ok:false "no_provenance"
  (unknown reported as unknown, never as fresh).
- `--help`; `--verify`; job keys `provenance` (default true), `verify_source`.

### 2. `air_topology_audit.py` — false-seal fixed + brought into the checker family
- imports `voxelize_stl.parity_fill` (one fill, not two copies).
- REAL SHIPPED HORN at the pitch that used to lie (`voxel_mm 1.0`):
  before `{"ok": false, "components": 13, "seed_labels": {"throat": 0, "mouth": 2},
  "connected": {"throat<->mouth": false}}`
  after  `{"ok": true, "components": 10, "seed_labels": {"throat": 2, "mouth": 2},
  "connected": {"throat<->mouth": true}, "seed_sizes_cm3": {"throat": 1921.49, ...}}`
  (1921.49 cm3 vs the campaign's 1946.9 cm3 closed-form bore integral — 1.3 %).
- REAL SHIPPED CUBESAT FRAME (F4 phantom sealed void): `components 2 →
  1`, `sizes_cm3 [1058.79, 0.01] → [1074.81]`. Same root cause, different campaign.
- honours the job's `receipt` key via `_receipt.emit` (cubesat F11), exits 1 on
  ok:false, `--help`.
- `component_sizes_cm3` {label: cm3, COMPLETE} and `seed_sizes_cm3` {seed: cm3}
  close horn F13's un-joinable `seed_labels`/`sizes_cm3`.
- a seed landing in material / outside the domain is now a REFUSAL, not a
  `connected:false` verdict.

### 3. `analysis_sheet.py`
- `validate_job()` refuses by NAME ("panel 0 load 0: missing required key 'at'
  (only 'label' is optional)"); `loads[].label` is now OPTIONAL like
  `fixture.label`. `cli()` wraps everything so a failure prints
  `{"ok": false, "error": ...}` on STDOUT and exits 1 — before, the traceback
  went to stderr and stdout was EMPTY, so the documented `| tail -1 > f.json`
  idiom wrote a 0-byte receipt and reported success.
- REFUTED (evidence): the "PRINTS THE TRACEBACK YET EXITS 0" half of cubesat F7
  is the PIPE, not the tool. Reconstruction of the old main tail:
  `python3 oldcrash.py` -> exit 1; `python3 oldcrash.py | tail -1` -> exit 0.
  The real defect is the missing stdout receipt line, which is what is fixed.
- UNITS table + `field_unit`: Pa->MPa computed and cross-checked against `scale`
  (`unit_scale_conflict` refusal), dimension mismatch refused, K->°C refused as
  an offset. `unit` with nothing to verify it now raises a receipt `warnings`
  entry. Receipt carries `fields:[{panel,unit,field_unit,scale}]`.
- determinism: same job twice -> identical md5.

### 4. `render_sheet.py`
- `resolve_job_root()`: base_dir -> CWD -> job-file dir and its ancestors
  (<=6 up); a root qualifies only if EVERY relative input exists under it;
  ambiguity and not-found are REFUSALS naming the roots tried; the winning root
  is named by KIND in the receipt `notes` (byte-comparable). Output follows the
  same root. gripper F10 repro now passes from the repo root AND the part dir.
- BACKWARD-COMPAT PROOF: re-rendering the SHIPPED gripper job reproduces
  `renders/sheet_palm.png` byte-identically (md5 de990069… both).
- overlay legend: measures the meta string's left edge and, when the inline
  legend would collide, moves it to its own full-width strip under the rule
  (grid gives up exactly that height). din_rail F4's 5-STL sheet re-rendered
  and inspected: no overlap. Receipt notes the move.
- `--help`.

### 5. `assembly_doc.py`
- `parse_axis` (3-vector OR 'x'/'y'/'z', optionally signed) and `parse_view`
  ({elev,azim} OR [elev,azim]); both refuse by NAME. 'z' and [0,0,1] render a
  byte-identical sheet (asserted in the test).
- the page GROWS 16:10 -> 16:`max_page_h_in` (default 20 in, 0.5 in steps) when
  the measured steps/BOM do not fit, instead of refusing a legal job; capping it
  restores the old refusal, now naming the height it reached.
- the step layout is measured ONCE and the height search is pure arithmetic;
  font-size probing is lazy (1 measurement instead of 7 in the common case).
- receipt gains `page_h_in`, `page_grew`, `steps_fs`. `--help`.

### 6. `bom_audit.py` — rewritten job-file driven (digest F5)
`{assemblies:[{name, step}], bom:{name:{label, only?, expect?}}, name_pattern?,
calibrate_with?, overhead?}`; findings for not-in-BOM / used-outside-`only` /
declared-but-absent / total != `expect`; `_receipt.emit`; exit 1 on findings.
`--example` prints the OLD hardcoded cyclo26/harmonic26/planetary26 project as a
runnable job, so nothing is lost.

### 7. `--help` (digest F9) + receipts
`--help` added to render_sheet, analysis_sheet, assembly_doc, production_dossier,
motion_gif, air_topology_audit, voxelize_stl, bom_audit, render_views.
`motion_gif` and `render_views` also gained an `{ok:false,error}` stdout receipt
+ exit 1 (they had none); `render_views` gained an `{ok:true,...}` receipt and
argument validation.
`render_sheet.text_w_px` is memoized (pure function; outputs unchanged).

## BACKWARD-COMPATIBILITY EVIDENCE (re-render vs the SHIPPED artifact)
- render_sheet: 36 shipped jobs swept -> **27 byte-identical, 0 errors, 3 differ**,
  and all 3 are multi-STL OVERLAY sheets whose receipts now say
  "overlay legend moved out of the header band" (din_rail, wrist, rotor) — i.e.
  exactly and only the collision the fix targets.
- assembly_doc: 3/3 shipped sheets byte-identical (singulator, mirror, horn).
- analysis_sheet: 3/3 shipped sheets byte-identical (cubesat, mirror, turgo).
- LOUD EXCEPTION: `voxelize_stl` occupancy CHANGES for any mesh with a vertex on
  a slice centre or scanline — including every axis-aligned part. The old fill
  was wrong (see the oracle numbers above). Any campaign re-voxelising will get a
  different, CORRECT field; the cookbook's "VERIFIED box -> solid_voxels 2320"
  example must be re-measured.

## TESTS
`tools/test_aux_tools.py` — 24 tests, all green
(`python3 tools/test_aux_tools.py`, `-k <substr>` to filter). Each pins a named
friction entry; the parity tests compare against an independent
matplotlib point-in-polygon oracle and against the verbatim legacy algorithm.

## CROSS-OWNER REQUESTS (not my files — described, not edited)
1. ACE voxel runners (`ace_fea_runner` / `ace_modal_runner` / `ace_thermal_runner`
   / `ace_buckling_runner` / `graded_infill_runner`): call
   `voxelize_stl.check_field_freshness(npy, stl_path=..., grid={...})` before
   consuming a density `.npy`; put `fresh` + `reasons` in the receipt and REFUSE
   on `fresh: false`. Add the same `--verify` mode for program-meshed jobs and
   record the INPUT hash under one uniform field name (gripper F11).
2. `campaign/digests/tools_cookbook.md`: air_topology_audit joins the "exit 1 on
   failure" AND the "receipt persistence" rows; `sizes_cm3` is sorted+truncated
   (use `component_sizes_cm3`/`seed_sizes_cm3`); assembly_doc `view` and
   `explode.axis` shapes + `max_page_h_in`; analysis_sheet `field_unit` and
   optional `loads[].label`; render_sheet `base_dir`; bom_audit is job-driven;
   voxelize_stl sidecar + `--verify`; and RE-MEASURE the box example.
3. `campaign/OPERATOR_BRIEF.md`: the documented `tool job.json | tail -1 > f.json`
   idiom converts a tool's exit 1 into shell success AND, when the tool dies
   before printing, writes a 0-byte receipt. Recommend `set -o pipefail` or
   `tool job.json > f.json` with the receipt parsed from the file.
4. Campaign notes now describing defects that no longer exist (no edit made):
   the horn's `gen_air_jobs.py` "safe pitch" search, and the cubesat A10
   phantom-sealed-void adjudication.

## DEFERRED (real, not fixed)
- `assembly_doc.wrap_text` measures each growing candidate line with a fresh
  TextPath, so an 8-step x 230-char sheet costs ~60 s. Fixing it means summing
  per-word widths, which changes wrap points and would break the byte-identity
  of the three shipped sheets. Needs a deliberate re-baseline, not a silent one.
