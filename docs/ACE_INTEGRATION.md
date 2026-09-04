# ACE ↔ LMCAD — the physics/geometry marriage

**Status change, 2026-09-04: the physics moved in-tree.** LMCAD no longer
imports anything from an external ACE checkout. The FEA / modal / buckling /
tet10 / DfAM / printability / convergence solvers now live in this repository
at [`tools/analyzers/physics/`](../tools/analyzers/physics/), and the split of
labour is the other way round from here on: **ACE keeps its LLM agent layer and
calls LMCAD as its geometry + physics backend.**

## Why it moved

Two concrete failures, both fixed by the move:

- **Hosted CI could not run the analyzers at all.** Every `ace_*_runner.py`
  began with `sys.path.insert(0, ACE_ROOT)` and imported `engine.*`. On a
  GitHub-hosted runner that is `ModuleNotFoundError: No module named 'engine'`,
  so `analysis-gate` was red and only a self-hosted box could grade the
  analyzers.
- **The pinned revision existed on exactly one laptop.** `tools/ACE_REVISION`
  named ACE commit `f9202e727cbca8d33a2488eb9d3efa2e8d7ee6d0`, which was never
  pushed anywhere. Nobody outside that machine could reproduce an FEA number in
  a shipped campaign — the receipts said "validated" against a revision no
  reader could fetch.

Both were the same defect: a reproducibility claim that depended on a second,
private repository. The pin is now **this repository's own history** — the
commit that carries a receipt also carries the solver that produced it.

## Licensing — this directory is Apache-2.0, the rest of LMCAD is MIT

The moved code did not originate here and was **not relicensed**. LMCAD is MIT;
every `.py` file under `tools/analyzers/physics/` remains under the **Apache
License, Version 2.0**, copyright 2026 The ACE Authors. The licence and the
attribution travel with the code:

- `tools/analyzers/physics/LICENSE-APACHE-2.0` — the full Apache-2.0 text.
- `tools/analyzers/physics/NOTICE` — origin, the source commit
  (`f9202e727cbca8d33a2488eb9d3efa2e8d7ee6d0`), the move date, and the exact
  list of what changed in the move.

If you redistribute LMCAD, keep both files. The move changed file locations,
the module paths named in imports and docstrings, two docstring sentences that
described PicoGK as the geometry backend, and one hard-coded absolute path. The
numerics are untouched, which is why every validation pin still passes with the
same numbers.

## The package

`tools/analyzers/physics/` — 4,219 lines, numpy + scipy only (gmsh, trimesh and
pyamg are optional and lazily imported):

| module | was | what it is |
|---|---|---|
| `fea.py` | `engine/verify/fea.py` | hex8 linear-elastic reference FEA + modal |
| `buckling.py` | `engine/verify/buckling.py` | linear (eigenvalue) buckling |
| `fea_tet.py` | `engine/verify/fea_tet.py` | body-fitted tet10 solver |
| `convergence.py` | `engine/verify/convergence.py` | mesh-refinement study |
| `printability.py` | `engine/verify/printability.py` | STL print-readiness audit |
| `selectors.py` | `engine/verify/selectors.py` | geometric region resolver |
| `mesh_ir.py` | `engine/verify/mesh_ir.py` | tet mesh IR (gmsh front end) |
| `dfam.py` | `engine/verify/dfam.py` | DfAM audit of the voxel field |
| `sampling.py` | `engine/lmcad.py` | the LMCAD geometry bridge (below) |
| `__init__.py` | `engine/verify/__init__.py` | the public re-exports |

ACE's `engine/lmcad.py` became `sampling.py` because a module named for LMCAD,
inside LMCAD, says nothing; it is the part-sampling bridge. Everything else kept
its name, and the `engine.verify` package flattened into `physics` because that
package *is* the physics.

Import it as `physics.*` — `tools/analyzers/_ace.py` puts `tools/analyzers/` on
`sys.path`, so `from physics.fea import reference_fea` resolves with no
environment variables at all. `ACE_ROOT` and `ACE_PYTHON` are gone.

## The geometry contract (unchanged)

**The entire load-bearing interface is two numpy arrays**: `solid_fraction.npy`
(float32, C-order, `(nx,ny,nz)`, `rho[i,j,k]` with `i↔x`, voxel centers
`origin+(idx+0.5)h`, mm) + `region_kind.npy` (strings). LMCAD is the geometry
authority: exact B-rep + implicit hybrid, STEP, assemblies, parts catalog, FDM
print gates.

- LMCAD `kernel-api` ops (`API.md`, "ACE / voxel-physics bridge"):
  - `sample_density_grid` — any bound solid (winding-number MeshSdf) or
    implicit tree → `solid_fraction.npy` in the exact contract, supersampled
    fractions.
  - `mesh_density_grid` — optimized density `.npy` → redistanced level-set →
    watertight narrow-band mesh (fails loudly if not watertight).
  - Round-trip pinned by `crates/kernel-api/tests/bridge.rs` (volume within the
    voxel skin).
- `physics/sampling.py`: `sample_part` (LMCAD JSON ops → validated array),
  `region_kind_from_regions` (regions resolved by the same selector engine the
  FEA uses), `emit_stl_gated` (kernel-enforced watertightness).
- Proven end-to-end: L-bracket built with exact B-rep booleans →
  `validate_solid_fraction_array` PASS → `reference_fea` solves 64k elements
  (25.85 MPa, 2.862 mm) → `emit_stl_gated` watertight, 29,248 triangles.

## Who keeps what, now

| | LMCAD keeps | ACE keeps |
|---|---|---|
| geometry | exact B-rep, booleans, STEP, catalogs, gates | — |
| physics | hex8 FEA, modal, buckling, tet10, DfAM, convergence | — |
| optimization | SIMP loop (`ace_optimize_runner.py`) | BESO/vision-critic loops |
| agents | — | the LLM specialists, planning, V&V narrative |

ACE calls LMCAD as its geometry **and** physics backend: the analyzers are
plain CLI tools with a receipt contract (`tools/_receipt.py`), so an agent
shells out to them and reads one JSON line.

## Reproducing an FEA number

```sh
cargo build --release -p kernel-api
uv venv --python 3.11 .analysis-venv
uv pip install --python .analysis-venv/bin/python --require-hashes -r tools/requirements-analysis.lock
.analysis-venv/bin/python tools/analyzers/ace_fea_runner.py job.json --out receipt.json
```

Nothing else is needed. `LMCAD_REQUIRE_REPRODUCIBLE_ANALYSIS=1` still refuses
before meshing if the checkout is dirty, the lock is missing, `PYTHONPATH` is
set, or a package is imported from outside the selected Python prefix — but a
clean LMCAD checkout now satisfies the whole contract on its own, which is what
makes the hosted gate possible. See `docs/ANALYSIS_TIERS.md`.

## Job-dict conventions worth remembering

Loads/fixtures use `kind` + `region_selector` + `magnitude`/`direction` (NOT
`type`/`selector` — the schema validators catch program-level drift but
`reference_fea` kwargs are permissive).
