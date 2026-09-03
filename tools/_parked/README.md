# tools/_parked — orphaned tools, kept but not wired

Parked on 2026-09-02. Each file here was verified (`grep -rn <name>` over the
whole repository, `tools/` excluded) to be referenced by **nothing**: no
campaign `run_all.sh`, no CI workflow, no nightly gate, no registry row, no
digest, no doc. They are not deleted — the history and the ideas are worth
keeping — but they are no longer part of the tools/ surface: `tools/_parked/`
is not scanned by `analyzer_registry.py`, not discovered by `nightly.sh`, and
not run by any workflow. Nothing in here has a receipt contract you may quote.

| file | what it was for | why it is parked |
|---|---|---|
| `fea_report.py` | Runs a campaign's `manifest.json` list of voxelize+FEA jobs and rewrites the receipts table between `<!-- fea-receipts:begin/end -->` markers in its `FEA.md`, so no FEA number is hand-pasted. | Predates `document_bundle.py` / `analysis_sheet.py`, which regenerate analysis prose from receipts; no campaign carries an `FEA.md` manifest any more. Referenced by nothing. |
| `sim_design_evaluator.py` | A `param_optimize` command evaluator: builds a stepped shaft for `(d, r)`, meshes it body-fitted (ACE `fea_tet`) and returns mass + fillet peak so the optimizer's objective is real converged physics, not the voxel proxy. | The simulation-driven-design loop it served was a 2026-07 experiment; nothing invokes it and its 8–11 min pin (`sim_design_validation.py`) never joined the registry or CI. Referenced only by its own job/pin files, parked alongside. |
| `sim_design_shaft_job.json` | The `param_optimize` job for that loop (2400 N axial, 20 MPa fillet cap, minimize mass over `d`, `r`). | Input to the parked evaluator/pin only. |
| `sim_design_validation.py` | The pin for the loop above: feasible design found, mass reduced, constraint active, mesh-convergence trust delta bounded (<= 12 %). | Never registered as a pin (no registry row names it) and never run by CI or nightly; needs ACE + gmsh and ~10 min. |
| `sim_generative_verify.py` | Closes the generative (SIMP) loop with an honest body-fitted tet10 verification of the as-built STL and reports the voxel-vs-body-fitted stress gap — or reports, truthfully, that gmsh cannot consume the staircased surface-nets STL (the measured 2026-07-18 outcome). | An integration experiment whose finding (the loop is OPEN on surface-nets STLs) is recorded in its docstring; `ace_optimize`'s own pin and `ace_fea_tet` cover what is validated today. Referenced by nothing but the reconstruct script below. |
| `sim_generative_reconstruct.py` | The follow-up: marching-cubes + Taubin reconstruction of the SIMP density iso-surface into a smooth watertight mesh the tet10 pipeline can mesh, then the same verification. Needs scikit-image + gmsh. | Same experiment; not on any campaign or gate path. Parked with `sim_generative_verify.py`, which it imports (`from sim_generative_verify import ...` — both live here, so the import still resolves when run from this directory). |

`ace_fea_kt_tet_validation.py` was on the candidate list but is **not** an
orphan: it is the registered validation pin of `ace_fea_tet`
(`tools/analyzer_registry.py`, `tools/manifests/ace_fea_tet.manifest.json`) and
`analyzer_registry.py --run-pins` executes it on the physics gate. It stays
in the validation surface.

To revive a parked tool, move it back into the surface it belongs to and add
the registry row / gate / doc reference that makes it non-orphan — a tool that
nothing references is a tool nothing re-proves.
