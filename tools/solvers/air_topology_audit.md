# air_topology_audit — is the functional INTERNAL AIR one connected path?

**Runner**: `tools/analyzers/air_topology_audit.py` ·
`python3 tools/analyzers/air_topology_audit.py job.json` (also `--help`; the
pre-2026-09-02 path `tools/air_topology_audit.py` is a forwarding shim).
**Gates**: none — see Benchmark gates and Validity limits.
**Status**: no gate suite. **Tier**: **Demonstrated**
(`python3 tools/analyzer_registry.py --tier air_topology_audit` →
`{"tier": "Demonstrated", "gate_suite": null, "manifest": null, "pins": []}`,
`tier_reason`: "Demonstrated by the registry rule: Validated requires a present
manifest AND a present validation pin").

Every other geometry gate in this tree checks the MATERIAL. `watertight`,
`geometric_ok`, `exact_volume`, a mesh check — all of them are statements about
the solid. A speaker line, a manifold, a duct, a cooling channel is a part whose
function lives in the VOID, and a void can be severed by a wall the material
gates are perfectly happy with. That is the TL-91 defect the runner's docstring
names (2026-07-09): a design that passed watertight and geometric_ok shipped a
channel that did not connect. This tool voxelizes the STL, flood-labels the
internal air, and asserts that the named seed points sit in ONE component.

It is a topology gate, not a flow model. It answers "is there a path", never
"how much passes through it".

## What it actually computes

Occupancy: the mesh's own bounding box is voxelized at pitch `h = voxel_mm` by
per-slice parity fill — `voxelize_stl.parity_fill`
(`tools/analyzers/voxelize_stl.py`), the same single fill the `.npy` bridge uses,
loaded through the shared binary-STL reader `tools/_stl.py`. Grid shape is
`ceil((hi - lo)/h)` per axis; `origin_mm` is the mesh's `lo` corner and is
echoed on the receipt as `grid`.

Audited domain: an axis-aligned interior box inset by `m = round(wall_margin_mm
/ h)` voxels on every axis, so the outside air is excluded by construction. Each
face named in `open_faces` (`"y+"`, `"x-"`, …) is a face some later part seals;
the domain is extended to within 1 voxel of it instead of being inset there.

Air = `domain AND NOT material`. Components come from `scipy.ndimage.label` with
its default structuring element — **6-connectivity** (face neighbours only;
diagonal voxel touches do NOT join two channels). Volumes are
`cell_count * h**3 / 1000` cm³.

Seeds are world points mapped to `floor((v - lo)/h)`. Connectivity for each
`require_connected` pair is label equality — `ok` is the AND over all pairs.

The `front_openings_face` census is separate and 2-D: over the wall slab of
`m - 1` voxels behind the named face, a cell is an opening iff it is air through
the WHOLE slab (`wall.all(axis)`), and the through-cells are then labelled and
their sizes reported descending in `openings_mm2`.

Assumptions baked in: watertight input (parity fill has no meaning otherwise);
one uniform pitch; axis-aligned boxes for the domain and the face census;
staircased curved walls; a channel narrower than one voxel reads as severed.

## The contract (job → receipt)

Job keys that matter: `stl` (path), `voxel_mm` (default 1.0), `wall_margin_mm`
(default 7), `open_faces` (list of face names), `seeds` (`{name: [x,y,z]}`),
`require_connected` (`[[seed_a, seed_b], …]`), `front_openings_face`, and
`receipt` (the receipt is ALSO written there — `tools/_receipt.py` rules;
cubesat F11 was a run where that key was ignored and the evidence existed
nowhere).

Receipt (LAST non-empty stdout line; logging goes to stderr):
`ok`, `components`, `sizes_cm3` (DESCENDING-SORTED, **TOP-8**, and therefore NOT
indexable by a seed label), `seed_labels` (raw label ids),
`component_sizes_cm3` (`{label: cm3}` for EVERY component),
`seed_sizes_cm3` (`{seed name: cm3 of its component}`), `connected`
(`{"a<->b": bool}`), `openings_mm2`, `grid` (`origin_mm`, `voxel_mm`, `shape`).
The last two size keys exist because `sizes_cm3` and `seed_labels` looked
joinable and were not (horn F13) — use them for any size a seed is named in.

Refusals: a seed that lands in MATERIAL, or outside the audited domain, or
outside the mesh bounding box, is a **refusal**, not a `connected: false`
verdict — "these are disconnected" and "you pointed at a wall" would otherwise
be the same receipt. The refusal surfaces as `ok:false` with `error` beginning
`seed placement refused:` and naming the seed.

Exit codes — read this before wiring a shell gate. This runner does **not**
implement the three-way split in `tools/_receipt.py`
(0 ok / 1 could-not-run / 2 ran-and-refused). Its `main()` returns **0** when
`ok` is true and **1** in every other case: a disconnected required pair, a
refused seed, and an unreadable job all exit 1. Nonzero still means "do not
quote this receipt", but a caller cannot branch on the exit code to tell a
verdict from a breakage — branch on `error` / `ok` instead. It also emits no
`error_kind` slug, so a gate matching `refusal.*` will not match anything here.

## Benchmark gates (measured, frozen)

**None. This analyzer has no benchmark gate suite.** The registry records
`gate: no` for it (`python3 tools/analyzer_registry.py --tier
air_topology_audit` returns `"gate_suite": null`), and there is no manifest
under `tools/manifests/` and no validation script under `tools/validation/`.

What exists instead is three regression tests in
`tools/tests/test_aux_tools.py` (`python3 tools/tests/test_aux_tools.py -k air`;
all three pass in this checkout):

| test | what it proves | what it does NOT pin |
|---|---|---|
| `test_air_topology_open_bore_is_connected` | on a synthetic annular tube carrying an explicit vertex ring on a slice centre, a wide-open through-bore reads `connected: true` at `voxel_mm` 1.0 and exits 0; `seed_sizes_cm3` for both seeds are equal; `len(component_sizes_cm3) == components`; the job's `receipt` key writes a file | any volume against an independent reference — the tube is compared only to itself |
| `test_air_topology_seed_in_material_is_refused` | a seed at a wall gives `ok:false`, exit 1, and an `error` naming the seed — not `connected: false` | nothing numeric |
| `test_air_topology_exit_1_on_disconnected` | a solid box with a required pair exits 1 with `ok:false` | nothing numeric |

These are behavioural regressions against named campaign defects (horn F8, horn
F13, cubesat F11). They prove the tool reproduces its own past verdicts. **No
number this tool returns — no `sizes_cm3`, no `component_sizes_cm3`, no
`openings_mm2` — is validated against an independent reference.** The registry's
own note says as much: "deterministic, defect-taught, but not pinned against a
measured flow/acoustic reference" (`tools/analyzer_registry.py`).

## Validity limits / out of scope

- **`openings_mm2` is a raw voxel-face COUNT, not an area in mm².** The
  `front_openings_face` branch reports `int(s)` straight from the labelled
  component sizes with no `h**2` factor, so the key means mm² only at
  `voxel_mm = 1.0`. At any other pitch multiply by `voxel_mm**2` yourself, and
  say which you quoted.
- **The opening census needs a wall slab at least 2 voxels deep.** It inspects
  `m - 1 = round(wall_margin_mm/voxel_mm) - 1` voxels; at `m <= 1` that slab is
  empty and `all()` over a zero-length axis is True for every cell, so the whole
  face reads as one opening. Do not run the census with a sub-2-voxel margin.
- **A job with an empty `require_connected` always returns `ok: true`.** The
  verdict is the AND over the pairs you named; naming none asserts nothing. The
  gate is only as strong as the seed list.
- **Discretisation error is not characterised.** No study in this repo
  establishes how `voxel_mm` relates to the smallest channel this tool resolves,
  or how `sizes_cm3` converges as the pitch is refined. There is no convergence
  study and no pitch-sensitivity band. Until one exists, treat every volume it
  reports as unbounded error, choose a pitch several voxels smaller than the
  narrowest designed passage, and gate on the CONNECTIVITY verdict — not on a
  volume.
- **Not a flow, acoustic, or pressure-drop model.** It says a path exists. Cross
  section, tortuosity, surface roughness, and anything downstream of them are
  outside it — those are a new solver or a declared gap, not an inference from
  this receipt.
- **Requires a watertight mesh.** Parity fill on a leaking mesh is meaningless
  and this tool does not check watertightness for you; gate that first.
- **Curved walls staircase** and axis-aligned domain boxes cannot follow a
  non-axis-aligned part. The error that introduces is not characterised here.
- **NumPy + SciPy are hard dependencies** (`scipy.ndimage`), unlike the
  stdlib-only checkers.

## When to use it

Any part whose function is the void: speaker enclosures and horn/transmission
lines, manifolds, ducts, cooling and coolant channels, siphons, syringe and
pipette paths, anything with a designed internal cavity that must reach a port.
Run it right after the mesh gates, seed one point in every chamber and one in
every port, and name every pair that MUST be joined. If the part's function
depends on two cavities being SEPARATE, note that this tool has no
"must-not-connect" assertion — read the `connected` map yourself and gate on it
explicitly.
