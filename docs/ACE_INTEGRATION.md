# ACE ↔ LMCAD — the physics/geometry marriage

ACE (`~/Work/ACE`) is a natural-language → topology-optimized
STL pipeline: 5 LLM specialists + a benchmark-tested pure-numpy physics
stack (hex8 voxel FEA, modal, buckling, DfAM) + SIMP/BESO optimization
loops. Its geometry kernel was LEAP71 PicoGK (C#, voxel-only). LMCAD is
the geometry authority here: exact B-rep + implicit hybrid, STEP,
assemblies, parts catalog, FDM print gates.

**The entire load-bearing interface is two numpy arrays** (ACE
`agents/_schema.md §4`): `solid_fraction.npy` (float32, C-order,
`(nx,ny,nz)`, `rho[i,j,k]` with `i↔x`, voxel centers `origin+(idx+0.5)h`,
mm) + `region_kind.npy` (strings). ACE's FEA/optimizer/4-of-5 agents never
touch C# — only the Designer's voxel sampler did.

## What exists now (Phase A, working)

- LMCAD `kernel-api` ops (`API.md` "ACE / voxel-physics bridge"):
  - `sample_density_grid` — any bound solid (winding-number MeshSdf) or
    implicit tree → `solid_fraction.npy` in the exact ACE contract,
    supersampled fractions.
  - `mesh_density_grid` — optimized density `.npy` → redistanced level-set
    → watertight narrow-band mesh (fails loudly if not watertight);
    reports the ACE `emit_stl` contract fields.
  - Round-trip pinned by `crates/kernel-api/tests/bridge.rs` (volume
    within the voxel skin).
- ACE `engine/lmcad.py` shim: `sample_part` (LMCAD JSON ops → validated
  array), `region_kind_from_regions` (regions resolved by ACE's own
  selector engine — removes the C# duplication), `emit_stl_gated`
  (drop-in `render.emit_stl` with kernel-enforced watertightness).
- Proven end-to-end: L-bracket built with exact B-rep booleans →
  `validate_solid_fraction_array` PASS → `reference_fea` solves 64k
  elements (25.85 MPa, 2.862 mm) → `emit_stl_gated` watertight, 29,248
  triangles. Zero C#.

## Both worlds, kept

| | LMCAD keeps | ACE keeps |
|---|---|---|
| geometry | exact B-rep, booleans, STEP, catalogs, gates | — |
| physics | — | hex8 FEA, modal, buckling, DfAM, convergence |
| optimization | — | SIMP/BESO loops, vision critic, V&V |
| agents | JSON API written for LLM planning | 4 of 5 specialists unchanged |

## Remaining phases

- **B — Designer retarget**: rewrite `agents/designer.system.md` (67
  PicoGK/C# mentions) to emit LMCAD JSON programs via `engine/lmcad.py`;
  drop `spec/geometry.cs`/`Program.cs`/csproj generation. The
  `initial/*.npy` gate (`validate_initial_arrays`) needs no change.
- **C — gated emit**: point the Optimizer template's `render.emit_stl` at
  `lmcad.emit_stl_gated` (per-part `render.py` one-liner).
- **D — reverse bridge**: rate LMCAD's drive parts (cyclo/harmonic/
  planetary) with ACE's `reference_fea` — real stress fields replacing
  hand formulas (`kernel_model::rate` stays as the first-order sanity
  layer).
- Optional: retire `engine/picogk_inspect.py`'s dotnet SDF probe (LMCAD
  `signed_clearance`/DfAM covers it); PicoGK stays vendored for legacy
  parts until B lands.

ACE-side dict conventions worth remembering: loads/fixtures use
`kind` + `region_selector` + `magnitude`/`direction` (NOT `type`/`selector`
— the schema validators catch program-level drift but `reference_fea`
kwargs are permissive).
