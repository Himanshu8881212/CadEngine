# LMCAD Design Guide — the Operator's Book

How to design real parts and assemblies with this engine, cold, through its
public surfaces only. Written for an LLM operator and for a careful human —
both work the same way here: send JSON, read receipts, gate on measures.
Companions: [`API.md`](API.md) is the op-by-op reference (161 ops, every
parameter and default); this guide is the *method* — what to reach for, in
what order, and how to know you are right.

**The credibility law of this document: every fenced JSON program, part file
and assembly file below was executed against `kernel-api` (current main)
before inclusion, and every quoted number is a measured value from those
runs — never a prediction.** Deliberately-failing examples are marked and
their error text is quoted verbatim. A mechanical extractor re-runs every
fenced block before each revision of this guide ships.

Two reading paths:

- **Human, cover to cover**: read Part I, type the §3.1 work order in, then
  work forward. Parts II–V each open with the decision guidance and put the
  reference detail behind it. Budget an afternoon.
- **AI, lookup**: jump by section anchor. The coverage map in Appendix A
  routes every op family to its guide section and its API.md heading, and the
  161-op arithmetic is reconciled there. Failure-first lookups: §23 (error
  kinds, verbatim) and §24 (known limits).

Contents:

- **Part I — Orientation**: §1 mental model · §2 human on-ramp · §3 quickstart
- **Part II — The work-order language**: §4 grammar · §5 solid constructors ·
  §6 sketches & solver · §7 booleans · §8 fillets/chamfers/holes ·
  §9 placement & patterns · §10 measures & assertions
- **Part III — The implicit half**: §11 expression trees · §12 scalar language
  & `expr_sdf` · §13 field modulation · §14 the pillow trap · §15 threads
- **Part IV — Recipes, assemblies, libraries, catalog**: §16 `.lmcpart` ·
  §17 hybrid parts & voxel selection · §18 `.lmcasm` · §19 the library ·
  §20 the catalog
- **Part V — Shipping & survival**: §21 exports · §22 print-readiness ·
  §23 failure playbook · §24 limits ledger · §25 campaign cookbook
- **Appendix A — coverage map (161 ops reconciled)**

---

# Part I — Orientation

## 1. Mental model

The engine speaks four kinds of file. Keep their nouns and verbs straight and
everything else follows.

| artifact | metaphor | grammar | verb | mutability |
|---|---|---|---|---|
| JSON program (`{"ops": […]}`) | **work order** | op list, executed top-to-bottom | `kernel-api run` | write per task; cheap, disposable |
| `.lmcpart` | **recipe** | parametric feature tree (`Document`) — geometry is **never stored**, it re-evaluates deterministically on load | `load_part`, `kernel-api asm` | the living source file; hand-edit, diff, version |
| `.lmcasm` | **floor plan** | instances of recipes at rigid poses + mates + named states | `kernel-api asm` | the living assembly source |
| STL / STEP / 3MF / `bom.json` + `bom.csv` | **blueprint** | dead export | (output of the above) | never edited; always regenerable |

A **work order** is imperative: "build this, cut that, measure it, export it,
and *fail loudly* if any expectation is unmet." It binds nothing across runs —
state lives in the recipe and floor-plan files it loads and writes. A
**recipe** is declarative: parameters, features, labels, configurations; the
kernel's deterministic rebuild (R5) re-evaluates it to the bit-identical solid
every time. A **floor plan** seeds poses but its **mates are the authority** —
they re-solve on every load, so a hand-edited pose snaps back to a consistent
assembly. **Blueprints** are for printers, machinists and CAD interop; if you
find yourself editing one, you took a wrong turn — edit the recipe and
re-export.

### 1.1 The three geometry representations

One kernel, two halves and a bridge. Choose per feature, not per project:

| you need | use | why |
|---|---|---|
| machined interfaces: bores, seats, flanges, fillets/chamfers, hole patterns; π-exact volumes; STEP | **exact B-rep** (constructors §5, booleans §7, hole wizard §8, catalog §20) | analytic surface tags survive booleans; `exact_volume` and STEP read them; receipts are exact |
| lattices/TPMS, threads, organic blends, graded walls, self-intersecting unions, shells | **implicit** (`implicit` expression trees §11–15, `gyroid_block`; Document `GyroidLattice`/`BeamLatticeFill`/`PipeFeat`/`Shell`/smooth booleans §16.4) | closed-form fields mesh watertight where exact arithmetic is impossible (a helical thread buried in a shank self-intersects — no exact boolean can stitch that) |
| exact skin + organic core in ONE part | **hybrid** (Document `HybridFuse` §17; Rust `hybrid_boolean`) | untouched exact faces are kept verbatim on the `ExactStitch` route; the `Healed` route resamples everything and *says so* |

The voxel **heal** is the safety net under all of it: any solid the exact
tessellator cannot mesh watertight is re-meshed through its winding-number SDF
— and the report names the route (`"route": "exact"` vs `"voxel_healed"`),
never silently.

### 1.2 The receipts-first doctrine

This kernel never asks to be trusted; it hands you receipts.

- The **report is the only output contract**: stdout JSON, one entry per op,
  exit code 0 iff every op succeeded. Parse it; never scrape logs.
- Every solid-producing op is gated through `validate()` — invalid geometry is
  never bound silently; execution stops at the first failure with one
  root-cause error (no cascades).
- Numbers you care about are **measures** (`volume`, `genus`, `route`,
  `watertight`, `residual`, fit limits, gland dimensions…). Claims you depend
  on become **`assert` ops inside the program**, so acceptance criteria travel
  with the design instead of living in an external grep.
- **The validity gate is a topology check, not a geometric-truth oracle.**
  Three measured §5.3 traps make the point: a hole loop that crosses its outer
  boundary binds a *valid* solid with a *wrong* volume; a negative sphere
  radius silently absolutizes; an assembly export can report
  `"watertight": false` while the run exits 0 (§18.5). In each case one
  `assert`/measure catches what the gate cannot. Measure what you mean.

## 2. If you are a human — the ten-minute on-ramp

No CAD background needed; no Rust knowledge needed beyond one build command.

1. **Install the Rust toolchain** (one-time): <https://rustup.rs> — accept the
   defaults. Any platform; this repo needs nothing else.
2. **Build the CLI** (one-time, from the repo root; takes a few minutes):

   ```bash
   cargo build -p kernel-api --release
   ```

   The binary lands at `target/release/kernel-api`.
3. **Make your first part.** Copy this into a file named `first.json` — it is
   a *work order*: build a 60×40×8 mm plate, drill an 8 mm hole through it,
   prove the result is sane, write a 3D-printable STL:

   ```json
   {"ops": [
   	{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [60, 40, 8]},
   	{"id": "bore", "op": "drill", "in": "plate", "at": [30, 20, 8],
   	 "axis": [0, 0, -1], "d": 8, "through": 8},
   	{"id": "gate", "op": "assert", "in": "bore", "genus": 1, "valid": true,
   	 "exact_volume_within": {"target": 18797.9, "abs": 0.1}},
   	{"id": "stl", "op": "export_stl", "in": "bore", "file": "first_plate.stl"}
   ]}
   ```

   Run it:

   ```bash
   target/release/kernel-api run first.json --out-dir out/
   ```

   Exit code 0 and a JSON report on stdout = every op passed, including the
   `gate`: genus 1 means "exactly one tunnel through the solid", and the
   volume window is the closed form 60·40·8 − π·4²·8 = 18 797.88 mm³ — the
   kernel's `exact_volume` recovers the *analytic* bore volume from the hole
   wall's surface tag, not the faceted approximation. `out/first_plate.stl`
   is watertight and printable. (Executed: exit 0, measured exact_volume
   18 797.8761 mm³, 408 triangles, `"route": "exact"`.)
4. **Break it on purpose** (30 seconds, the most useful habit here): change
   `"genus": 1` to `"genus": 2` and re-run. The program exits 1 and the
   report names exactly what is false: `assert failed: genus: measured 1,
   expected 2`. That is the whole working style — declare what must be true,
   let the engine refuse anything less.
5. **Where to go next**: §3 quickstart (five canonical work orders), then §5
   onward as reference. All lengths are mm, all angles on the JSON surface
   are degrees, profiles wind counter-clockwise. When something fails, the
   error's `kind` indexes into §23. When you want a wheel, check §20's
   catalog before modelling one — `{"op": "spur_gear", …}` is one line.

## 3. Quickstart — five work orders

Build once, then:

```bash
target/release/kernel-api run  program.json  --out-dir out/
target/release/kernel-api asm  assembly.lmcasm --out-dir out/asm
```

Exit code 0 iff everything passed. All lengths are **mm**, all angles on this
surface are **degrees**.

### 3.1 Catalog part → measure → export

```json
{"ops": [
	{"id": "gear", "op": "spur_gear", "module": 2, "teeth": 20, "face_width": 10, "bore": 8, "keyway": true},
	{"id": "topo", "op": "validate", "in": "gear"},
	{"id": "xv", "op": "exact_volume", "in": "gear"},
	{"id": "gate", "op": "assert", "in": "gear", "genus": 1, "closed": true, "manifold": true},
	{"id": "stl", "op": "export_stl", "in": "gear", "file": "gear.stl"},
	{"id": "step", "op": "export_step", "in": "gear", "file": "gear.step"}
]}
```

Executed: exit 0; `topo` reports `genus: 1` (the bore), `xv` reports
`exact_volume: 11731.378…` (π-exact bore via the surface tags), and the STL
ships on the exact route, watertight:

```json
{"id": "stl", "ok": true, "measures": {"route": "exact", "triangles": 2272, "watertight": true}}
```

### 3.2 Sketch → extrude

The solver receipt tells you whether your constraint set actually pins the
profile — read `state` and `free_dof`, not vibes:

```json
{"ops": [
	{"id": "profile", "op": "sketch",
	 "points": [[1, 2], [55, 3], [58, 44], [-2, 38]],
	 "segments": [[0, 1], [1, 2], [2, 3], [3, 0]],
	 "constraints": [
		{"kind": "fixed", "point": 0, "at": [0, 0]},
		{"kind": "horizontal", "a": 0, "b": 1},
		{"kind": "distance", "a": 0, "b": 1, "distance": 60},
		{"kind": "vertical", "a": 0, "b": 3},
		{"kind": "distance", "a": 0, "b": 3, "distance": 40},
		{"kind": "horizontal", "a": 3, "b": 2},
		{"kind": "vertical", "a": 1, "b": 2}
	 ]},
	{"id": "plate", "op": "sketch_extrude", "sketch": "profile", "height": 8},
	{"id": "gate", "op": "assert", "in": "plate",
	 "volume_within": {"target": 19200, "percent": 0.01}, "genus": 0, "valid": true},
	{"id": "stl", "op": "export_stl", "in": "plate", "file": "plate.stl"}
]}
```

Executed: the deliberately-skewed seed points converge in 4 iterations to
residual 6.6e-19, `state: "well_constrained"`, `free_dof: 0`; the gate measures
volume 19199.9999997 against the 60×40×8 target. The full solver method —
including under- and over-constrained runs — is §6.

### 3.3 Load a hand-written recipe + hole wizard

A minimal `.lmcpart` is JSON you can type. Save as `spacer.lmcpart` next to
the program (relative `file` paths resolve against the **program file's own
directory**, so the pair stays relocatable):

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "spacer",
	"created_with": "written by hand",
	"document": {
		"params": {"h": 8.0},
		"features": [
			{"Box": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
			         "size": [{"Literal": 30.0}, {"Literal": 20.0}, {"Param": "h"}]},
			 "label": "blank"},
			{"Cylinder": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
			              "radius": {"Literal": 4.0}, "height": {"Literal": 12.0}}},
			{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
		],
		"root": 2,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "spacer", "op": "load_part", "file": "spacer.lmcpart"},
	{"id": "seats", "op": "bolt_circle", "in": "spacer", "center": [15, 10, 8],
	 "axis": [0, 0, -1], "circle_d": 16, "n": 4, "start_deg": 45,
	 "hole": {"kind": "counterbore", "m": 3}},
	{"id": "gate", "op": "assert", "in": "seats", "genus": 5, "valid": true},
	{"id": "stl", "op": "export_stl", "in": "seats", "file": "spacer_seated.stl"}
]}
```

Executed: exit 0, genus 5 (bore + four counterbored holes), and the wizard
echoes the table it used so you can pose mating hardware without reading the
kernel: `"hole": {"clearance_d": 3.4, "counterbore_d": 6.5,
"counterbore_depth": 3.5, "fit": "medium", "m": 3.0}`.

### 3.4 Library lifecycle (admission-gated)

```json
{"ops": [
	{"id": "admit", "op": "library_add", "dir": "lib",
	 "part": {
		"format": "lmc-part", "version": 1, "units": "mm", "name": "bushing",
		"created_with": "design-guide",
		"document": {
			"params": {"outer_r": 12.0, "bore_r": 4.0, "h": 10.0},
			"features": [
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "outer_r"}, "height": {"Param": "h"}}},
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "bore_r"}, "height": {"Literal": 200.0}}},
				{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
			],
			"root": 2, "suppressed": []
		}
	 },
	 "meta": {
		"name": "bushing", "version": 1, "category": "spacers",
		"tags": ["bearing", "sleeve"],
		"description": "parametric plain bushing with a through bore",
		"provenance": {"author": "design-guide-ai", "date": "2026-06-11"},
		"params": [
			{"name": "outer_r", "units": "mm", "default": 12.0, "min": 8.0, "max": 16.0},
			{"name": "bore_r",  "units": "mm", "default": 4.0,  "min": 2.0, "max": 5.0},
			{"name": "h",       "units": "mm", "default": 10.0, "min": 4.0, "max": 40.0}
		]
	 }},
	{"id": "find", "op": "library_search", "dir": "lib", "text": "bushing"},
	{"id": "bush", "op": "library_instantiate", "dir": "lib", "name": "bushing",
	 "params": {"outer_r": 14.0, "h": 20.0}},
	{"id": "v", "op": "exact_volume", "in": "bush"},
	{"id": "retire", "op": "library_deprecate", "dir": "lib", "name": "bushing"},
	{"id": "still_builds", "op": "library_instantiate", "dir": "lib", "name": "bushing"},
	{"id": "rm", "op": "library_remove", "dir": "lib", "name": "bushing"}
]}
```

Executed, exit 0. The receipts tell the whole I7 story: `admit` reports
`gate_samples: 10, gate_rebuilds: 2` (defaults + range corners + midpoint,
each built twice and compared volume-bit-deterministically) and
`volume_at_defaults: 3995.449…`; `v` reports 11309.7335529 — exactly
`π·20·(14² − 4²)`; `still_builds` carries `"deprecated": true` plus a warning
string (existing references keep building); `rm` removes only because no
`.lmcasm` in the directory references the entry (else `dependents_exist`,
provoked in §19.3).

### 3.5 The reference assembly — gearbox

The in-repo 15:1 two-stage gearbox (`gearbox/`) is the worked floor-plan
project: 20 `.lmcpart` recipes, 21 part programs (one per part, plus a
catalog-built circlip), a 37-instance `.lmcasm`, and a design-intent contact
allowlist. Run the official surface:

```bash
target/release/kernel-api asm gearbox/gearbox.lmcasm --out-dir out/asm > asm_report.json
python3 gearbox/check_asm.py asm_report.json        # design-intent layer
# or everything at once: cd gearbox && ./run_all.sh
```

Executed for this edition (pinned current-main binary): exit 0; `load` 37
instances / 4 mates / state `exploded`; `mates` residual 1.44e-12; `bom` 20
grouped lines; per-instance STLs with per-part routes (the housing base ships
`voxel_healed` at 971,736 triangles, watertight — FRICTION #19, §24); merged
export 1,010,204 triangles watertight; `contacts` 78 pairs in the 1 mm
window, 56 touching. `check_asm.py` then proves **52/52 designed contacts, 0
unexpected**, tightest must-clear gap 0.050 mm (the designed gear-flank
backlash — theory says 0.051), tolerating 4 known phantom pairs that
`gearbox/check_artifacts.json` re-proves disjoint by exact booleans every
run (§10.3, §24).

---
# Part II — The work-order language

## 4. The grammar, precisely

A program is `{"ops": [{"id": …, "op": …, …}, …]}`. Rules, all enforced and
all verified by execution:

- Ops run **in order**; each `id` is unique; later ops reference earlier
  results via `in` / `a` / `b` / `sketch`. First failure stops the run.
- **Geometry-producing ops bind; measure/export/assert/design-math ops bind
  nothing.** Referencing a non-binding op is a loud `missing_ref`:
  > `op 'v' param 'in': 'blob' is a measure/export op and binds no geometry`
- Units mm; **angles in degrees everywhere on this surface** (`degrees`,
  `*_deg`). Bores/shanks are **diameters**; hex sizes across flats.
- `extrude` profiles must be **CCW** simple polygons. Executed CW: the op
  fails `invalid_geometry` ("closed=false manifold=false … refusing to bind an
  invalid solid"). `extrude_with_holes`, `extrude_tapered`, `revolve` and the
  sketch sweeps re-wind input automatically.
- Unknown fields are ignored — including **misspelled optional params**, which
  silently leave the default in effect. Executed (§5.2): a Ø7×10 cylinder with
  `"segemnts": 64` reports the 32-segment faceted volume 382.377, not the
  64-segment 384.227. Misspelling a *required* param is a loud
  `invalid_param`. When in doubt, check a measure.
- Top-level unknown keys are tolerated too (the gearbox programs carry a
  `"_comment"` field).
- An **empty boolean result is an op failure** (`invalid_param`), not an empty
  solid — a disjoint `intersection` cannot bind. To *prove* disjointness, use
  `assert_disjoint` (distance-based, tessellation-accurate) or the exact
  route: `union` (or `union_all`) + `assert {"shells": N}` (§7.3).
- `pose` is the general placement op — rotation about an arbitrary axis
  (optional `center`), THEN `translate`. At least one part is required;
  executed empty: `invalid_param: pose needs 'translate' and/or 'rotate' — an
  empty pose would be a no-op`.
- `assert` needs at least one check; executed empty: `invalid_param: assert
  has no checks — give at least one of volume_within / exact_volume_within /
  genus / shells / closed / manifold / valid`.
- `load_part` resolves relative paths against the **program file's directory**
  (CLI), so programs and their recipes travel together. Export `file` paths
  and library `dir`s join `--out-dir`.

## 5. Solid constructors — all eleven (loft and sweep in both worlds)

The nine core constructor ops (`box`, `cylinder`, `sphere`, `cone`, `torus`,
`extrude`, `extrude_with_holes`, `extrude_tapered`, `revolve`) all validate
their result and refuse to bind a broken or empty solid. Two more solid
builders — **loft** and **sweep** — are first-class ops as well (op forms in
API.md); §5.4 shows their *Document-feature* form, which is what you want
when the shape must stay parametric.

**Segment counts and the two volumes.** Curved walls are faceted (`segments`,
`u`/`v`, `ring_segments`/`tube_segments`) but each facet carries its exact
analytic surface tag — `exact_volume` and STEP read the tag, `volume` reads
the facets. Executed, Ø7 × 10 mm pin:

```json
{"ops": [
	{"id": "pin24", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 3.5, "height": 10, "segments": 24},
	{"id": "v24", "op": "volume", "in": "pin24"},
	{"id": "xv", "op": "exact_volume", "in": "pin24"},
	{"id": "pin96", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 3.5, "height": 10, "segments": 96},
	{"id": "v96", "op": "volume", "in": "pin96"}
]}
```

| receipt | value | meaning |
|---|---|---|
| `v24` | 380.4640 | 24-gon prism (−1.14% vs π) |
| `v96` | 384.5704 | 96-gon prism (−0.07%) |
| `xv` | 384.8451 = π·3.5²·10 | exact, **independent of segments** |

Guidance: segments buy *silhouette* quality (STL cosmetics, clearance scans),
never measurement accuracy — gate volumes on `exact_volume_within` and leave
segments at defaults (32 walls / 48–24 torus / 64 revolve) unless an export
or a mesh-distance check needs a finer silhouette. Defaults and parameter
tables: API.md "Solid constructors".

### 5.1 One executed example each

`box` — the §2 plate. `cylinder` — above. The other seven:

```json
{"ops": [
	{"id": "ball", "op": "sphere", "center": [0, 0, 20], "radius": 6, "u": 48, "v": 24},
	{"id": "xv", "op": "exact_volume", "in": "ball"},
	{"id": "gate", "op": "assert", "in": "ball", "exact_volume_within": {"target": 904.7786842338603, "abs": 0.001}, "genus": 0}
]}
```

Executed: exact_volume 904.77868423 — `(4/3)π·6³` to ten digits, from a
48×24 faceting (the sphere tag carries the truth).

```json
{"ops": [
	{"id": "tip", "op": "cone", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 5, "height": 12},
	{"id": "xv", "op": "exact_volume", "in": "tip"},
	{"id": "mp", "op": "mass_properties", "in": "tip"}
]}
```

Executed: exact_volume 314.15927 = 100π exactly; centroid z = 3.0000000
(h/4 from the base — `mass_properties` is analytic for cones, §10.1).

```json
{"ops": [
	{"id": "ring", "op": "torus", "center": [0, 0, 0], "axis": [0, 0, 1], "major": 20, "minor": 5},
	{"id": "xv", "op": "exact_volume", "in": "ring"},
	{"id": "topo", "op": "validate", "in": "ring"}
]}
```

Executed: genus 1 (a torus is the unit donut), exact_volume 9869.6044 =
`2π²·20·5²` (Pappus) exactly.

```json
{"ops": [
	{"id": "lbar", "op": "extrude", "profile": [[0,0], [30,0], [30,10], [10,10], [10,25], [0,25]], "height": 6},
	{"id": "gate", "op": "assert", "in": "lbar", "volume_within": {"target": 2700, "abs": 0.001}, "genus": 0}
]}
```

Executed: 2700.000 exactly — planar-faced solids need no exact/faceted
distinction. Profile is CCW; CW is refused (§5.3).

```json
{"ops": [
	{"id": "frame", "op": "extrude_with_holes",
	 "outer": [[0,0], [40,0], [40,30], [0,30]],
	 "holes": [[[8,8], [18,8], [18,22], [8,22]], [[24,8], [34,8], [34,22], [24,22]]],
	 "height": 6},
	{"id": "gate", "op": "assert", "in": "frame", "volume_within": {"target": 5520, "abs": 0.001}, "genus": 2}
]}
```

Executed: genus = hole count = 2, volume (1200 − 2·140)·6 = 5520 exactly.
Hole loops are re-wound automatically but must lie **strictly inside**
`outer` — see the §5.3 trap for what happens when they do not.

```json
{"ops": [
	{"id": "boss", "op": "extrude_tapered", "profile": [[0,0], [30,0], [30,20], [0,20]], "height": 10, "draft_deg": 2},
	{"id": "v", "op": "volume", "in": "boss"},
	{"id": "da", "op": "draft_analysis", "in": "boss", "pull": [0, 0, 1], "min_deg": 1}
]}
```

Executed: volume 5827.02 (the 2° inset eats 173 mm³ of the 6000 prism) and
`draft_analysis` reads back `min_draft_deg: 2.0000` with zero undercut area —
build the mold check into the same program as the part.

```json
{"ops": [
	{"id": "pulley_blank", "op": "revolve",
	 "profile": [[6,0], [22,0], [22,4], [18,7], [18,9], [22,12], [22,16], [6,16]],
	 "segments": 96},
	{"id": "topo", "op": "validate", "in": "pulley_blank"},
	{"id": "xv", "op": "exact_volume", "in": "pulley_blank"}
]}
```

Executed: a concave multi-segment `(r, z)` profile (bore, V-groove) revolves
clean — genus 1, exact_volume 19955.3965 = **6352π exactly** (each profile
edge carries its cylinder/cone/plane tag; this is the R1 robustness fix
holding). Remember `revolve` reads `[radius, z]` pairs about the **Z axis**,
radii ≥ 0.

### 5.2 The misspelled-optional trap, executed

```json
{"ops": [
	{"id": "pin", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 3.5, "height": 10, "segemnts": 64},
	{"id": "v", "op": "volume", "in": "pin"}
]}
```

Executed: exit 0, volume **382.377** — the 32-segment default, because
`segemnts` is an unknown field and was ignored. The correctly-spelled program
measures 384.227. Two defenses: gate on `exact_volume_within` (tag-exact, so
typos in faceting params cannot shift it), and when a faceted number matters,
assert it.

### 5.3 Degenerate input — what the gate catches, and three honest traps

Every constructor was deliberately fed garbage. What the gate catches
(executed, one per constructor):

| provocation | result |
|---|---|
| `box` with `max.z == min.z` (zero thickness) | `invalid_geometry`: *"box failed validate(): closed=false manifold=false … refusing to bind an invalid solid"* |
| `cylinder` with `axis: [0,0,0]` | `invalid_geometry` (NaN positions fail the manifold check) |
| `cone` with `height: 0` | `invalid_param`: *"cone produced an empty solid — degenerate input, parameters outside the op's documented domain…"* |
| `torus` with `minor: 8 > major: 5` | `invalid_param` (empty-solid gate) |
| `extrude` with a CW profile | `invalid_geometry`: *"extrude failed validate(): closed=false manifold=false genus=6 … refusing to bind an invalid solid"* — wind CCW |
| `extrude_tapered`, concave L-profile at `draft_deg: 45`, height 12 | `invalid_param` (the inset self-intersects and collapses to empty) |
| `revolve` of `[[8,0],[0,6],[8,12]]` (isolated on-axis apex) | `invalid_param` (an isolated apex would pinch — documented domain) |

And three traps the gate does **not** catch — all measured, all caught by a
one-line measure instead:

1. **`sphere` with `radius: -2` binds.** Executed: exit 0, a valid r = 2
   sphere (faceted volume 32.98). The sign is silently absolutized. Assert
   the volume you meant.
2. **A shallow draft on a concave profile binds.** Executed: the L-profile at
   `draft_deg: 2`, height 10 extrudes tapered to a *valid* solid — the
   documented domain of `extrude_tapered` is convex profiles, and outside it
   you are relying on the empty-solid gate to catch only the *collapsing*
   cases. Stay convex, or validate dimensions you care about.
3. **A hole loop crossing the outer boundary binds a valid-topology, wrong-
   geometry solid.** Executed: a 20×20×5 plate with a "hole" half outside the
   outer loop exits 0, `validate` says closed/manifold/genus 1 — and `volume`
   reads **2333.33 mm³, more than the 2000 mm³ blank** (the self-overlapping
   triangulation double-counts). The loop contract is *strictly inside*;
   `assert volume_within` is the tripwire that catches a violation.

```json
{"ops": [
	{"id": "plate", "op": "extrude_with_holes",
	 "outer": [[0,0], [20,0], [20,20], [0,20]],
	 "holes": [[[15,15], [30,15], [30,30], [15,30]]], "height": 5},
	{"id": "v", "op": "volume", "in": "plate"},
	{"id": "topo", "op": "validate", "in": "plate"}
]}
```

(The trap-3 program above, kept runnable: exit 0 with the wrong-volume
receipt — the point is that *you* must put the volume gate in.)

### 5.4 Loft and sweep — Document features, loaded via `load_part`

`LoftSolid` and `SweepSolid` now have op-surface twins (`loft` / `sweep`,
one-shot forms — see API.md); the `.lmcpart` Document form shown here is the
*parametric* route, re-evaluated on every load (full recipe grammar: §16).
Save as `loft_funnel.lmcpart`:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "loft_funnel",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"top_w": 16.0},
		"features": [
			{"LoftSolid": {"sections": [
				[[{"Literal": -20.0}, {"Literal": -20.0}, {"Literal": 0.0}],
				 [{"Literal": 20.0}, {"Literal": -20.0}, {"Literal": 0.0}],
				 [{"Literal": 20.0}, {"Literal": 20.0}, {"Literal": 0.0}],
				 [{"Literal": -20.0}, {"Literal": 20.0}, {"Literal": 0.0}]],
				[[{"Literal": -8.0}, {"Literal": -8.0}, {"Literal": 30.0}],
				 [{"Param": "top_w"}, {"Literal": -8.0}, {"Literal": 30.0}],
				 [{"Param": "top_w"}, {"Literal": 8.0}, {"Literal": 30.0}],
				 [{"Literal": -8.0}, {"Literal": 8.0}, {"Literal": 30.0}]]
			]}, "label": "hopper shell"}
		],
		"root": 0,
		"suppressed": []
	}
}
```

And `sweep_bend.lmcpart` — a closed profile swept along a 3-leg path with
rotation-minimizing frames, capped ends:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "sweep_bend",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {},
		"features": [
			{"SweepSolid": {
				"profile": [[{"Literal": -2.0}, {"Literal": -2.0}, {"Literal": 0.0}],
				            [{"Literal": 2.0}, {"Literal": -2.0}, {"Literal": 0.0}],
				            [{"Literal": 2.0}, {"Literal": 2.0}, {"Literal": 0.0}],
				            [{"Literal": -2.0}, {"Literal": 2.0}, {"Literal": 0.0}]],
				"path": [[{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				         [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 20.0}],
				         [{"Literal": 14.0}, {"Literal": 0.0}, {"Literal": 34.0}],
				         [{"Literal": 34.0}, {"Literal": 0.0}, {"Literal": 34.0}]]},
			 "label": "bent square duct"}
		],
		"root": 0,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "funnel", "op": "load_part", "file": "loft_funnel.lmcpart"},
	{"id": "vf", "op": "volume", "in": "funnel"},
	{"id": "tf", "op": "validate", "in": "funnel"},
	{"id": "duct", "op": "load_part", "file": "sweep_bend.lmcpart"},
	{"id": "vd", "op": "volume", "in": "duct"},
	{"id": "td", "op": "validate", "in": "duct"}
]}
```

Executed: the loft measures **27840.000** — exactly
`30·∫(40−16t)(40−24t)dt`, the closed-form linear-loft volume, because planar
quads loft to planar-faceted walls; the duct measures 908.31 (4×4 profile ×
59.8 mm path, minus the two mitred-corner bites), both genus 0 and valid.
Contract corners: all sections share one point count and winding, ≥ 2
sections of ≥ 3 points; the sweep path is open, ≥ 2 points; a
self-overlapping sweep (tight helix) builds a self-intersecting solid —
route that through the mesh/heal path (§17), not `load_part`. Loft/sweep
points are `Dim`s, so any coordinate can be a `{"Param": …}` (the funnel's
`top_w` drives its top-section width).

## 6. Sketches and the constraint solver

The 2D sketch subsystem is a Levenberg–Marquardt solver with a rank-revealing
degree-of-freedom analysis. The receipt is the method: every `sketch` op
reports `residual`, `iterations`, `converged`, `dof`, `rank`, `free_dof`,
`redundant`, and `state` ∈ `under_constrained` / `well_constrained` /
`over_constrained`.

- **`dof`** = 2 × points (every point floats in the plane).
- **`rank`** = independent constraint equations; **`free_dof`** = dof − rank —
  how many directions the profile can still move; **`redundant`** counts
  dependent (but consistent) constraint rows.
- The 11 constraint kinds (`fixed`, `coincident`, `horizontal`, `vertical`,
  `distance`, `parallel`, `perpendicular`, `equal_length`, `tangent`,
  `angle`, `symmetric`) are tabled with field names in API.md "Sketch ops".

The full flow — sketch → solve diagnostics → extrude — is §3.2 (4 iterations
to residual 6.6e-19, `free_dof: 0`). What the other two states look like:

### 6.1 Under-constrained is allowed — and labelled

```json
{"ops": [
	{"id": "bracket", "op": "sketch",
	 "points": [[0, 0], [50, 0], [50, 30], [0, 30]],
	 "segments": [[0, 1], [1, 2], [2, 3], [3, 0]],
	 "constraints": [
		{"kind": "fixed", "point": 0, "at": [0, 0]},
		{"kind": "horizontal", "a": 0, "b": 1},
		{"kind": "distance", "a": 0, "b": 1, "distance": 50}
	 ]},
	{"id": "pad", "op": "sketch_extrude", "sketch": "bracket", "height": 5},
	{"id": "v", "op": "volume", "in": "pad"}
]}
```

Executed: `state: "under_constrained"`, `rank: 4`, `free_dof: 4` — the top
edge floats. The extrude still runs (volume 7500 from the seed positions):
legal, and exactly how you sketch exploratively. The discipline: before a
dimension *matters*, drive `free_dof` to 0 and let `state:
"well_constrained"` be an asserted fact of the build (it is in the §3.2
program — re-run after any edit and the receipt re-proves the profile).

### 6.2 Over-constrained fails loudly, with the diagnosis

```json
{"ops": [
	{"id": "impossible", "op": "sketch",
	 "points": [[0, 0], [50, 0]],
	 "segments": [[0, 1]],
	 "constraints": [
		{"kind": "fixed", "point": 0, "at": [0, 0]},
		{"kind": "fixed", "point": 1, "at": [50, 0]},
		{"kind": "distance", "a": 0, "b": 1, "distance": 80}
	 ]}
]}
```

Executed, exit 1, kind `sketch_failed`:

> `op 'impossible': constraints did not converge (residual 3.000e2 after 4
> iterations, state over_constrained) — they are conflicting or inconsistent`

The residual is the tell: 3.000e2 = (80 − 50)² · ⅓-ish of the squared error
the solver could not shed. Fix the *conflict* (here: two pins and an
incompatible distance), never by deleting whichever constraint was listed
last — read which dimensions disagree.

### 6.3 Circles extrude to exact cylinders; arcs build slots

A sketch whose only profile is one standalone circle extrudes to an
**analytic cylinder** — tag-exact, not a polygon prism:

```json
{"ops": [
	{"id": "disc", "op": "sketch",
	 "points": [[0, 0], [9.5, 0]],
	 "circles": [{"center": 0, "radius_point": 1}],
	 "constraints": [
		{"kind": "fixed", "point": 0, "at": [0, 0]},
		{"kind": "distance", "a": 0, "b": 1, "distance": 10}
	 ]},
	{"id": "puck", "op": "sketch_extrude", "sketch": "disc", "height": 4},
	{"id": "xv", "op": "exact_volume", "in": "puck"},
	{"id": "gate", "op": "assert", "in": "puck", "exact_volume_within": {"target": 1256.6370614359173, "abs": 1e-6}}
]}
```

Executed: exact_volume 1256.63706142 vs the symbolic 400π = 1256.63706144 —
the 1.2e-8 gap is the *solver's* convergence (the radius point lands at
10 − 5e-11 after 8 LM iterations, residual 1.7e-20), not faceting. Lesson:
solver-driven dimensions are converged numerics; gate them with windows ≥
1e-6, not bit-equality.

Arc edges (`arcs: [{a, b, center, ccw}]`, center is a construction point)
close profiles through circular segments — executed slot link, two arcs +
two segments: genus 0, volume 4721.70 (the analytic stadium is 4744.78; the
arcs facet at the sketch's tessellation). Order the boundary CCW overall;
`ccw` controls which way each arc bulges.

### 6.4 `sketch_revolve`

Same domain rules as `revolve` (the sketch's `(x, y)` read as `(r, z)`,
r ≥ 0) — one op turns a §6.3-style solved section into a lathe part.
Runnable example: API.md "Sketch ops"; the op-surface twin is §5.1's
revolve.

## 7. Booleans

Four ops — `union`, `difference`, `intersection`, `union_all` — exact
planar-arrangement booleans with **persistent face naming**: every result
face remembers which operand face it came from, which is why a fillet witness
still resolves after three more cuts (§8) and why `exact_volume` stays
π-exact through hole after hole when the cut walls keep their analytic tags
— but not as a law: a hole-wizard bore through a plate measured 6e-5
relative off the closed form (cold-start audit), so volume-gate bored
bodies with a tolerance band, never equality. Coplanar contact, faces with inner loops,
and cuts crossing previously-cut curved walls are in scope (the R1–R4
robustness work; chain-fuzz 100.0% at N=2000, floors 98 — `docs/ROBUSTNESS.md`).

### 7.1 The working trio, in one executed chain

```json
{"ops": [
	{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
	{"id": "dome", "op": "sphere", "center": [15, 10, 10], "radius": 6, "u": 24, "v": 12},
	{"id": "body", "op": "union", "a": "plate", "b": "dome"},
	{"id": "drill_tool", "op": "cylinder", "base": [15, 10, -1], "axis": [0, 0, 1], "radius": 3, "height": 20},
	{"id": "holed", "op": "difference", "a": "body", "b": "drill_tool"},
	{"id": "zone", "op": "box", "min": [0, 0, 0], "max": [15, 20, 20]},
	{"id": "left_half", "op": "intersection", "a": "holed", "b": "zone"},
	{"id": "topo", "op": "validate", "in": "holed"},
	{"id": "v_holed", "op": "exact_volume", "in": "holed"},
	{"id": "v_left", "op": "exact_volume", "in": "left_half"}
]}
```

Executed: the bored dome-plate is genus 1 with exact_volume **6011.0916**
(plate + spherical cap − plane-trimmed bore − sphere-trimmed bore crown — all
recovered from tags that survived two booleans), and the intersection's half
measures **3005.5458 — exactly half**, symmetric to 13 digits. Tool-building
conventions visible above: overshoot cutting tools past both faces (the
drill runs −1 → 19) so no coplanar membrane is left behind.

### 7.2 Empty results are op failures (and that is a feature)

Executed, both directions:

- disjoint `intersection` → `invalid_param`: *"intersection produced an empty
  solid — degenerate input, parameters outside the op's documented domain, or
  an empty boolean result (e.g. a disjoint intersection); see API.md"*
- `difference` whose tool swallows the body (small − big) → same kind, named
  on the `difference` op.

There is deliberately no "empty solid" value in this engine — nothing
downstream could validate against it. Prove non-overlap positively instead
(§7.3, §10.3).

### 7.3 `union_all` + `shells` — the N-body no-contact proof

```json
{"ops": [
	{"id": "a", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]},
	{"id": "b", "op": "box", "min": [10, 0, 0], "max": [15, 5, 5]},
	{"id": "c", "op": "box", "min": [20, 0, 0], "max": [25, 5, 5]},
	{"id": "all", "op": "union_all", "in": ["a", "b", "c"]},
	{"id": "no_contact", "op": "assert", "in": "all", "shells": 3, "volume_within": {"target": 375, "abs": 0.001}}
]}
```

Executed: one solid, three shells, 375.000 mm³. Disjoint bodies keep their
own shells through a union, so `shells: N` is an exact-arithmetic proof that
N bodies do not touch — tessellation-independent, unlike `assert_disjoint`
(which gives you the *distance*, §10.3). The gearbox uses exactly this for
gear-mesh interleave proofs and to re-arbitrate phantom contact-scan pairs
(`gearbox/check_artifacts.json`).

### 7.4 Coplanar faces — measured today, and the honest history

Coplanar-contact unions are in the supported scope. Executed on current main,
both canonical stacked cases:

```json
{"ops": [
	{"id": "lower", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
	{"id": "upper", "op": "box", "min": [5, 5, 10], "max": [25, 15, 18]},
	{"id": "stacked", "op": "union", "a": "lower", "b": "upper"},
	{"id": "gate", "op": "assert", "in": "stacked", "volume_within": {"target": 7600, "abs": 0.001}, "genus": 0, "shells": 1, "valid": true}
]}
```

```json
{"ops": [
	{"id": "lower", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
	{"id": "upper", "op": "box", "min": [0, 0, 10], "max": [30, 20, 18]},
	{"id": "tower", "op": "union", "a": "lower", "b": "upper"},
	{"id": "gate", "op": "assert", "in": "tower", "volume_within": {"target": 10800, "abs": 0.001}, "genus": 0, "shells": 1, "valid": true}
]}
```

Both exit 0 with exact volumes (7600 / 10800), one shell, genus 0 — the
boss-on-plate and flush-stack cases work. Honest history: a pre-mop-up branch
showed process-seed sensitivity on some coplanar stacked-primitive unions
(the catalog grew a workaround); the two probes above pass deterministically
on current main, but a *forest* of coplanar partial overlaps is still the
least-margin corner of the arrangement — prefer a 0.01 mm embedment when you
control the dimensions, and keep `valid: true` gates on stacked unions.
Document-feature patterns carry the same advice (§9.3, §16.4).

### 7.5 `try_*` checked booleans — where they live

On the JSON surface **every boolean is already checked**: results are gated
through `validate()` and an invalid or empty result fails the op (§7.2) —
there is nothing extra to call. The `try_` spelling exists on the **Rust
surface** (`kernel_brep::try_union` / `try_difference` / `try_intersection`):
the identical boolean, byte-for-byte, but returning
`Result<Solid, BooleanError>` where the error carries the machine-readable
`Validity` report — for Rust embedders who would otherwise have to remember
to validate. Same guardrail, two bindings. (Source: `kernel-brep/src/checked.rs`.)

Which to reach for (Rust surface, 2026-07-28 additions included): a single
authoring boolean → **`try_*_sealed`** (topology AND tessellation proven, the
mesh comes back with the solid); a chain of 10+ ops → **`ChainLog::seal()`**;
risky inputs before either → **`boolean_hazards`** (§7.7); routed pipelines
that must degrade gracefully → `policy::boolean_with_policy`. Plain `try_*`
remains the minimal guardrail; `detect_coincident_fit` predates the hazard
linter and survives as its single-question shortcut.

### 7.6 When the result tessellates exact vs heals

A boolean *binds* an exact B-rep — routing only happens at export/measure
time. `export_stl`/`export_3mf` tessellate the result on the exact adaptive
path; if that mesh is watertight it ships `"route": "exact"`, else the
winding-number heal re-meshes at `voxel` (§21.1). Every §7 example above
ships exact. The one in-repo part that heals is the 64-feature gearbox
housing (§24, FRICTION #19) — valid B-rep, leaky tessellation, honest route
receipt.

### 7.7 Boolean hygiene — the pre-flight checklist (learned 2026-07-28)

The RESPOOL campaign bisected three phase-dependent failures out of one
21-op chain and distilled them into rules. All three detectors now live in
`kernel_brep::boolean_hazards(a, b, tol)` (Rust surface) — run it while
authoring and it names the hazard with a location instead of leaving you a
blind bisect three ops later; `try_*_sealed` and `ChainLog` (below) are the
matching gates.

The checklist's detectors are executed in
`kernel-brep/tests/hazards_linter.rs` — the flush stack, the 0.02 sliver,
and the on-meridian cutter below are that test's literal cases:

```rust
let flush = boolean_hazards(&plate, &boss_flush, 0.05);   // CoincidentPlanes (info)
let sliver = boolean_hazards(&plate, &boss_sliver, 0.05); // NearCoincidentPlanes (fix!)
let on_grid = boolean_hazards(&tube(120), &cutter, 0.05); // EdgeInFace (meridian hit)
```

1. **Respect the facet grid of a revolve.** Curved bands facet along
   meridians at `k·360°/segments` from θ=0. A cutter **side plane lying ON a
   meridian** degenerated the arrangement outright (sector cut `[171°,249°]`
   at SEG=120, pitch 3°); a **small embedded union straddling one** left a
   valid B-rep whose default tessellation cracked (a detent bump astride the
   112.5° meridian at SEG=128). Keep boolean features off the meridians, and
   pick `segments` divisible by a pattern's count so every +360/n copy shares
   one facet phase (RESPOOL: SEG=126 for a ×3 pattern).
2. **Embed ≥ 0.1 mm — or coincide exactly; never the sliver between.** Exact
   coincidence cancels (§7.4, supported); gaps/overlaps in the ~1e-6–0.1 mm
   band mint needle faces (FRICTION #23's family). A cutter floor 0.05 under
   a face it meant to meet cost RESPOOL two invalid rebuilds.
3. **Keep cutter edges out of coincident-plane overlaps.** A cutter whose
   bottom was flush-coincident with a face while its INNER EDGE lay inside
   that face's region flipped a chain invalid three ops later (the §7.4
   "forest of partial coplanar overlaps" corner, made concrete). Extend the
   cutter until every face of it is fully in air or fully in material.
4. **Chain with receipts.** `ChainLog::start(..)?.seal()` validates *and*
   tessellation-checks after every op and refuses past the first bad step by
   name — the bisect harness you'd otherwise write by hand, built in.
5. **Valid ≠ tessellatable.** `try_*` proves topology; a valid result can
   still tessellate leaky (§7.6). `try_*_sealed` closes that gap: it returns
   the solid *with* its verified-watertight default mesh, or a `SealedError`
   carrying boundary/non-manifold edge counts.
6. **De-fragment planes when you're done.** Boolean chains can leave one
   plane as many adjacent faces (a pads-on-plate union measured 65 faces for
   a 16-face shape). `kernel_brep::coalesce_coplanar` merges them back —
   volume-exact, islands preserved — as a FINISHING pass (the rebuild resets
   provenance names, so run it after feature edits, not between them).

## 8. Fillets, chamfers, and the hole wizard

### 8.1 Edge fillets and chamfers — witness-addressed

No edge IDs cross the JSON surface; you point **near** an edge with a witness
coordinate, and resolution is guarded: a witness farther than `max_distance`
(default 10% of the bbox diagonal) from every edge fails rather than grabbing
something far away. Executed pair on one plate — round one top edge, bevel
the opposite:

```json
{"ops": [
	{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
	{"id": "soft", "op": "fillet_edge_near", "in": "plate", "witness": [15, 0, 10], "radius": 2},
	{"id": "v1", "op": "volume", "in": "soft"},
	{"id": "beveled", "op": "chamfer_edge_near", "in": "soft", "witness": [15, 20, 10], "radius": 2},
	{"id": "v2", "op": "volume", "in": "beveled"}
]}
```

Executed: 6000 → 5974.0965 (fillet removes ≈ (1−π/4)r²·L = 25.75, faceted
slightly deeper) → 5914.0965 (the 45° chamfer removes exactly r²L/2 = 60.000
— chamfers are planar, hence exact in the faceted volume too).

**Trap (measured in the cold-start audit): edge features after booleans
fragment the next witness.** On a plain box, four sequential corner chamfers
work fine. On a *boolean result*, the first `chamfer_edge_near` succeeds —
then the **next** one fails with `not a straight edge between two
perpendicular planar faces`, even for a clean corner nowhere near the cut:
each chamfer/boolean re-tessellation fragments the flat side faces, and edge
resolution stops finding one straight edge between two whole planes. The
working order is **ease edges on primitives first, boolean last** (the same
fuse-first family of rules as §14's datum re-clamp). Engine-side face
re-coalescing is on the frontier ledger (docs/FRICTION.md #20).

Honest scope (`feature_failed` otherwise, both executed):

- witness 170 mm off the part:
  > `witness [200, 0, 10] matched no edge — nearest edge is 170.000 mm away
  > (limit 3.742; pass max_distance to widen)`
- a curved edge (cylinder rim):
  > `fillet_edge_near: the edge near the witness is not a straight edge
  > between two perpendicular planar faces (the supported scope; for
  > cylindrical rims use fillet_circular_rim)`

### 8.2 Rim fillets — exact torus bands, convex on the op surface

`fillet_circular_rim` rolls the exact **torus** band around a circular convex
rim (cylindrical wall meets planar cap) — on bare cylinders and on bosses
fused onto other bodies:

```json
{"ops": [
	{"id": "plinth", "op": "box", "min": [-15, -15, 0], "max": [15, 15, 8]},
	{"id": "post", "op": "cylinder", "base": [0, 0, 8], "axis": [0, 0, 1], "radius": 6, "height": 12, "segments": 48},
	{"id": "bossed", "op": "union", "a": "plinth", "b": "post"},
	{"id": "soft_rim", "op": "fillet_circular_rim", "in": "bossed", "witness": [6, 0, 20], "radius": 1.5, "arc_segments": 6},
	{"id": "xv", "op": "exact_volume", "in": "soft_rim"},
	{"id": "topo", "op": "validate", "in": "soft_rim"}
]}
```

Executed: exact_volume 8539.9814 on a boss **fused through a boolean** —
the rim was found on the union result (persistent naming again), and the
fillet band carries the exact torus tag.

Two scope facts, both measured:

- **Convex rims only on this op.** The concave wall-meets-plate root junction
  is out of scope. The concave *bore-lip* case exists as the Document
  feature `CircularRimFillet {…, "concave": true}` — executed in §16.6 with
  its own honest scope.
- **The rim picker has no distance guard — it snaps to the nearest
  *qualifying* rim.** Executed: the same program with the witness moved to
  the concave root (`[6, 0, 8]`) exits 0 and fillets the TOP rim 12 mm away
  (identical exact_volume 8539.9814). A witness near a non-qualifying edge
  does not fail; it silently selects the nearest convex rim. Check a measure
  whenever the witness was not placed exactly on the rim you meant.

### 8.3 The hole wizard — all seven cuts

Real ISO/DIN dimension tables, machining conventions: `at` on the entry face,
`axis` INTO the material, cutters overshoot 0.5 mm so no membranes are left.
The seven cuts: `drill`, `clearance_hole`, `counterbore_hole`, `countersink_hole`,
`tap_drill_hole`, `bolt_circle` (repeats any of the five on a bolt-circle
diameter), `bearing_seat`. Five of them in one executed block — blind drill,
through drill, close-fit clearance, counterbore, countersink, blind tap
pilot:

```json
{"ops": [
	{"id": "block", "op": "box", "min": [0, 0, 0], "max": [90, 40, 12]},
	{"id": "h1", "op": "drill", "in": "block", "at": [10, 12, 12], "axis": [0, 0, -1], "d": 6, "depth": 7},
	{"id": "h2", "op": "drill", "in": "h1", "at": [10, 28, 12], "axis": [0, 0, -1], "d": 6, "through": 12},
	{"id": "h3", "op": "clearance_hole", "in": "h2", "at": [28, 20, 12], "axis": [0, 0, -1], "m": 5, "fit": "close"},
	{"id": "h4", "op": "counterbore_hole", "in": "h3", "at": [46, 20, 12], "axis": [0, 0, -1], "m": 5},
	{"id": "h5", "op": "countersink_hole", "in": "h4", "at": [64, 20, 12], "axis": [0, 0, -1], "m": 5},
	{"id": "h6", "op": "tap_drill_hole", "in": "h5", "at": [82, 20, 12], "axis": [0, 0, -1], "m": 6, "depth": 9},
	{"id": "gate", "op": "assert", "in": "h6", "genus": 4, "valid": true},
	{"id": "xv", "op": "exact_volume", "in": "h6"}
]}
```

Executed, exit 0 — read the echoes like a drawing schedule:

| op | echo (the table row actually cut) |
|---|---|
| `h1` blind drill | `depth: 7, point_depth: 8.803` — the 118° drill point reaches 1.8 mm deeper than the full-Ø depth; plan wall thickness against `point_depth` |
| `h2` through drill | `through: 12` |
| `h3` | `clearance_d: 5.3` (ISO 273 *close*; medium = 5.5, coarse = 5.8) |
| `h4` | `counterbore_d: 10.0, counterbore_depth: 5.8` (DIN 974-1 — an M5 cap screw sits flush) |
| `h5` | `countersink_d: 12.5` (DIN 74-1 form F, 90°) |
| `h6` tap pilot | `pilot_d: 5.0, pitch: 1.0` — Ø = m − pitch; **the thread itself is never modelled** on the exact half (cut a real one implicitly: §15) |

Genus 4 = the four *through* cuts; blind holes add no tunnels. The exact
volume (41155.85) is π-exact through all six cuts. `bolt_circle` (§3.3,
§10.2) and `bearing_seat` (§10.2, §20.2) complete the seven.

Counterbore + thread in one part is the §16.2 pad: a Document `Hole` feature
drills the bore, the §3.3 program adds the counterbored circle — and §15 cuts
a *real* helical thread where one is actually needed.

### 8.4 What "persistent" means here — and what survives edits

Two different persistence mechanisms, often confused:

1. **Witness resolution on the op surface** — works because boolean results
   carry persistent *face* names (operand + source face). A fillet finds its
   edge on a five-cut part (§8.2's union; the §3.1 gear keyway corners). But
   a work order has no edit history: re-running an edited program re-resolves
   witnesses against the new geometry — nearest-edge semantics, §8.2 trap.
2. **Persistent topological *edge names* in the Document** — the `.lmcpart`
   `Fillet`/`Chamfer` features store `EdgeName` (the pair of face names that
   meet at the edge), not coordinates. The name re-attaches across parameter
   edits; if an upstream boolean later *splits* the named edge, the optional
   `near` witness disambiguates the fragment instead of failing. Executed
   demonstration with measured deltas — §16.6: the same stored name fillets
   a 30 mm edge, then the hand-edited 44 mm variant, machine-exactly.

Use the op surface for one-shot machining; use Document features when the
fillet must *survive* a future re-dimension.

## 9. Placement and patterns

### 9.1 The three placement ops

`translate` (vector), `rotate_z` (degrees about world Z through the origin),
and `pose` — rotation about an **arbitrary** axis through an optional
`center`, THEN a translation; chain two `pose` ops for composed rotations.
All three re-validate and transform the analytic tags too. Executed, with
`mass_properties` as the motion receipt (a 20×10×6 bar, CoM starts at
(10, 5, 3)):

```json
{"ops": [
	{"id": "bar", "op": "box", "min": [0, 0, 0], "max": [20, 10, 6]},
	{"id": "slid", "op": "translate", "in": "bar", "offset": [40, 0, 0]},
	{"id": "com1", "op": "mass_properties", "in": "slid"},
	{"id": "turned", "op": "rotate_z", "in": "bar", "degrees": 90},
	{"id": "com2", "op": "mass_properties", "in": "turned"},
	{"id": "tipped", "op": "pose", "in": "bar",
	 "rotate": {"axis": [0, 1, 0], "degrees": -90, "center": [20, 0, 0]},
	 "translate": [0, 0, 4]},
	{"id": "com3", "op": "mass_properties", "in": "tipped"}
]}
```

Executed: CoM (50, 5, 3) after the slide; (−5, 10, 3) after the turn; and
(17, 5, −6) after the pose — rotate-about-center first, translate second,
exactly (and the inertia diagonal permutes (1.36e4, 4.36e4, 5.0e4) →
(5.0e4, 4.36e4, 1.36e4) with the tip). Conventions: right-handed about
`axis`; a `.lmcasm` instance pose is exactly `rotate` + `translate` (§18.1).
A real posing example with a gear (`Rx(−90°)` then offset) is in API.md
"Features & transforms".

### 9.2 Where patterns live (one-shot ops vs parametric features)

| you want | reach for | executed at |
|---|---|---|
| a hole pattern | `bolt_circle` (any wizard cut × n on a BCD) | §3.3, §10.2 |
| repeated *solids*, one-shot | ops `linear_pattern` / `polar_pattern` (fold clones by exact union; count includes the original) and `mirror` (does NOT auto-union) | API.md |
| repeated *solids*, parametric | Document `LinearPattern` / `CircularPattern` / `Mirror` | §9.3 |
| repeated *fields* (lattice cells, cage bars, fin arrays) | implicit `linear_pattern` / `circular_pattern` / `mirror` combinators (count ≤ 4096) | §11.4 |
| repeated *parts* in an assembly | `.lmcasm` instances (each its own pose/BOM line) | §18 |

The op-surface arrays are one-shot folds for programs; when the array must
*re-evaluate* as dimensions change, use the Document features — that is what
the recipe grammar is for.

### 9.3 Document patterns, executed

Save as `post_row.lmcpart` — a base strip, a post, a 4× linear pattern of the
post, the union, and a mirror of the whole assembly side (Document grammar
detail in §16; note `Mirror` = original ∪ reflection in ONE feature):

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "post_row",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"pitch": 12.0},
		"features": [
			{"Box": {"center": [{"Literal": 18.0}, {"Literal": 0.0}, {"Literal": 2.0}],
			         "size": [{"Literal": 60.0}, {"Literal": 24.0}, {"Literal": 4.0}]},
			 "label": "base strip"},
			{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 9.0}],
			              "radius": {"Literal": 3.0}, "height": {"Literal": 10.0}}},
			{"LinearPattern": {"input": 1, "count": 4,
			                   "step": [{"Param": "pitch"}, {"Literal": 0.0}, {"Literal": 0.0}]},
			 "label": "post row"},
			{"Boolean": {"op": "Union", "a": 0, "b": 2}, "label": "strip + posts"},
			{"Mirror": {"input": 3,
			            "plane_point": [{"Literal": 0.0}, {"Literal": -13.0}, {"Literal": 0.0}],
			            "plane_normal": [{"Literal": 0.0}, {"Literal": 1.0}, {"Literal": 0.0}]},
			 "label": "mirrored pair", "notes": "plane y=-13 keeps a 2 mm gap: the two halves stay disjoint shells"}
		],
		"root": 4,
		"suppressed": []
	}
}
```

And `lug_flange.lmcpart` — a hub with six lugs on a `CircularPattern`
(**`angle` is RADIANS in the Document grammar**, one of its three unit/shape
asymmetries, §16.3):

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "lug_flange",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"lug_r": 2.5},
		"features": [
			{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 4.0}],
			              "radius": {"Literal": 18.0}, "height": {"Literal": 8.0}}, "label": "hub"},
			{"Cylinder": {"center": [{"Literal": 12.0}, {"Literal": 0.0}, {"Literal": 10.0}],
			              "radius": {"Param": "lug_r"}, "height": {"Literal": 4.0}}, "label": "lug 0"},
			{"CircularPattern": {"input": 1, "count": 6,
			                     "axis_point": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
			                     "axis_dir": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 1.0}],
			                     "angle": {"Literal": 1.0471975511965976}},
			 "label": "lug ring", "notes": "angle is RADIANS in the Document grammar: 1.0471975512 = 60 deg"},
			{"Boolean": {"op": "Union", "a": 0, "b": 2}}
		],
		"root": 3,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "row", "op": "load_part", "file": "post_row.lmcpart"},
	{"id": "row_gate", "op": "assert", "in": "row", "shells": 2, "valid": true},
	{"id": "row_xv", "op": "exact_volume", "in": "row"},
	{"id": "flange", "op": "load_part", "file": "lug_flange.lmcpart"},
	{"id": "flange_xv", "op": "exact_volume", "in": "flange"}
]}
```

Executed: the post row measures 13781.9467 = 2 × (5760 + 4·π·3²·10) machine-
exact (mirror doubles it; the two halves are disjoint, hence `shells: 2`);
the flange measures 8614.2471 = π(18²·8 + 6·2.5²·4) exactly. Pattern
discipline (from the feature docs, and the reason these examples are spaced):
keep copies from *sharing face planes with each other* — the pattern fuses
copies with booleans, and a coplanar-overlap forest is the §7.4 thin-margin
corner. Copies seated ON a base plane (posts on the strip) are the supported
coplanar-contact case.

## 10. Measures and assertions — the whole receipt vocabulary

Six measure ops (record numbers) and two assertion ops (enforce them). All
require a bound solid (`wrong_type` on a sketch).

### 10.1 The six measures

| op | reports | doctrine |
|---|---|---|
| `validate` | `closed`, `manifold`, `euler_characteristic`, `genus`, `shells`, `valid` | every solid op already gates on this; the op exists to RECORD topology (genus = through-tunnels — the strongest one-number shape check) |
| `volume` | faceted enclosed volume | exact for planar solids; segment-dependent on curved ones (§5) |
| `exact_volume` | tag-recovered analytic volume | π-exact on tagged quadrics; falls back to facets for untagged faces — the default volume gate. Through booleans, cut walls can drop to chord level (measured: 6e-5 relative on a bored plate) — gate with a band |
| `mass_properties` | `volume`, `center_of_mass`, `inertia_diag` | unit density — multiply by yours. Analytic second moments for cylindrical/spherical/conical faces; torus second moments are tessellation-level (documented) |
| `wall_thickness` | `min_thickness`, `p05_thickness`, `median_thickness`, `thin_area`, `sampled_triangles` vs your `flag_below` | judge by `thin_area` + percentiles; `min_thickness` reads oblique corner rays (measured: 2.86 on a part whose true minimum wall is 5 — §10.2) |
| `draft_analysis` | `min_draft_deg`, `low_draft_area`, `undercut_area` against `pull`/`min_deg` | walls parallel to pull are 0°; executed on a 2° tapered boss: `min_draft_deg: 2.0000, undercut_area: 0` (§5.1) |

Executed `mass_properties` reference (the §5.1 cone): volume 100π, centroid
(0, 0, 3.0000000), inertia diagonal (2874.557, 2874.557, 2356.194) about the
CoM — Izz = (3/10)·m·r² analytically. Use it for balance checks the same way
§9.1 uses it as a motion receipt.

### 10.2 The assertion-first method — a complete measured contract

Write the checks before the geometry. Decide what must be true — topology,
an analytic volume window, wall thickness, clearances — put them in the
program as `assert` ops, then build until the program exits 0. The
acceptance criteria ride with the design forever; any later edit that breaks
them fails loudly. A complete executed example — a 608 bearing wall:

```json
{"ops": [
	{"id": "wall", "op": "box", "min": [0, 0, 0], "max": [40, 40, 12]},
	{"id": "seat", "op": "bearing_seat", "in": "wall", "at": [20, 20, 12], "axis": [0, 0, -1], "bearing": "608"},
	{"id": "bolts", "op": "bolt_circle", "in": "seat", "center": [20, 20, 12], "axis": [0, 0, -1], "circle_d": 31, "n": 4, "start_deg": 45, "hole": {"kind": "clearance", "m": 3}},
	{"id": "gate_topology", "op": "assert", "in": "bolts", "genus": 5, "valid": true},
	{"id": "gate_volume", "op": "assert", "in": "bolts", "exact_volume_within": {"target": 15219.7, "percent": 0.1}},
	{"id": "gate_walls", "op": "wall_thickness", "in": "bolts", "flag_below": 2},
	{"id": "shaft", "op": "shaft", "d": 8, "length": 30},
	{"id": "shaft_posed", "op": "pose", "in": "shaft", "translate": [20, 20, -9]},
	{"id": "gate_shaft_clears", "op": "assert_disjoint", "a": "bolts", "b": "shaft_posed", "min_clearance": 3, "tol": 0.01},
	{"id": "stl", "op": "export_stl", "in": "bolts", "file": "bearing_wall.stl"}
]}
```

Executed: exit 0. The volume gate's target is closed-form
(`19200 − π(11²·7 + 7.5²·5 + 4·1.7²·12)` = 15219.70) and the kernel measures
15219.6964 — the surface tags survive all the cuts. `gate_shaft_clears`
measures 3.466 mm (the Ø8 shaft inside the Ø15 shoulder bore, minus the
shaft's entry chamfer). The `wall_thickness` receipt shows the doctrine:
`thin_area: 0.0` at flag 2 and `p05_thickness: 5.0` say the design is sound
while `min_thickness: 2.86` is oblique-ray corner noise.

`assert` checks available: `volume_within` / `exact_volume_within` (each
`{"target", "abs"|"percent"}` — exactly one tolerance form), `genus`,
`shells`, `closed`, `manifold`, `valid`. All present checks are evaluated and
every failure is listed in one message (executed in §2: *"assert failed:
genus: measured 1, expected 2"*).

### 10.3 `assert_disjoint` vs the exact route

`assert_disjoint` passes iff the measured surface distance **exceeds**
`min_clearance` (default 0) — the exit-0 proof of non-interference an empty
`intersection` cannot give, *and* it hands you the distance. It measures raw
exact tessellations at `tol` (default 0.01), so treat the answer as accurate
to about `tol` and keep `min_clearance` ≳ `tol` for hard proofs. For
tessellation-independent proofs, the exact route is `union` + `assert
shells` (§7.3). Both patterns in the wild: §10.2's shaft clearance (3.466
mm), §20.3's gear backlash (0.0834 mm), and the gearbox's 52-contact
allowlist (§18.6).

Two assertion patterns worth stealing from the gearbox:

- **Mesh-clearance proof**: pose two gears at the mounted centre distance,
  `union` them, `assert shells == 2` — teeth interleave without contact, both
  as-assembled and half-pitch-rolled (`gearbox/programs/check_mesh_stage*.json`).
- **Exact disjointness despite mesh artifacts**: where the contact scan reads
  a phantom touch through a known tessellation leak, re-prove the pair with
  `pose → union → assert shells == 2` — the exact boolean is the truth
  channel (`gearbox/check_artifacts.json`).

---
# Part III — The implicit half

### 10.4 The witness-fixture pattern (stronger than a direct measure)

Direct measures exist now — `bounding_box` (envelope/`fits_within`) and
`measure_dimension` (point_point / face_face / diameter) — and are the right
tool for envelope and single-dimension receipts. But a *positional* claim
("this Ø5.2 bore exists open, on THIS axis, at THIS height") is still best
proven by the strongest indirect proof —
invented unprompted by the cold-start audit session, now canon — is the
**witness fixture**: build a virtual gauge part at the exact specified
geometry and let shell topology be the verdict. To prove a Ø5.2 bore exists
at exactly z = 14: pose a Ø5.0×40 pin on that axis, `union` it with the
part, `assert shells == 2` (it must float free, clearing the bore wall by
0.1 mm) — one assertion proves diameter *and* position *and* that the bore
is actually open. A pin 0.3 mm off-axis or oversize would weld into one
shell and fail. Gauges cost nothing; they never ship. Combine with
`assert_disjoint` when you also want the measured gap as a receipt.

## 11. The `implicit` op — trees of leaves and combinators

`implicit` meshes a recursive CSG expression tree watertight at `voxel`
resolution and (optionally) writes a file (`.stl`/`.3mf` by extension). **It
binds no solid** — its products are the file and the measures (`triangles`,
`watertight`, `healed`, `volume`). A passing anchor (executed: 26,784
triangles, watertight, volume 5547.5):

```json
{"ops": [
	{"id": "blob", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "smooth_union", "k": 3,
		"a": {"shape": "sphere", "center": [0, 0, 12], "radius": 8},
		"b": {"shape": "box", "min": [-10, -10, 0], "max": [10, 10, 10]}},
	 "file": "blob.stl"}
]}
```

When implicit beats B-rep: anything whose exact arrangement would
self-intersect or has no closed form — threads (§15), TPMS lattices and
graded walls (§13), metaball blends, shells of arbitrary solids, helical
anything. When B-rep beats implicit: every machined interface — you give up
analytic tags, π-exact volumes, STEP, and fit-grade dimensional accuracy the
moment you voxelize (a voxel mesh is honest to ~the voxel size; an exact
bore is honest to f64).

### 11.1 The grammar's one structural rule

A tree node is either a **leaf** `{"shape": …}` or a **combinator**
`{"op": …}` over child trees (`a`/`b`, or `in` for single-child ops). The
authoritative vocabularies below are quoted from the interpreter's own
rejection messages — each provoked by execution (unknown names `sphre`,
`sphere`-as-op, `tan`), so the lists cannot drift from the binary:

> `unknown shape 'sphre' — supported shapes: sphere, box, cylinder, cone,
> capsule, torus, plane, gyroid, beam_lattice, pipe, helix_pipe, expr_sdf`

> `unknown combinator 'sphere' — supported combinators: union, intersection,
> difference, smooth_union, smooth_intersection, smooth_difference,
> fillet_union, fillet_difference, chamfer_union, chamfer_difference, offset,
> shell, translate, rotate, scale, mirror, linear_pattern, circular_pattern,
> offset_by, lerp`

> `unknown scalar op 'tan' — supported scalar ops: add, sub, mul, div, min,
> max, mod, atan2, neg, abs, sqrt, sin, cos, clamp, length2, length3`

That is the complete surface: **12 leaves, 20 combinators, 16 scalar ops**
plus the variables `"x"`/`"y"`/`"z"` and bare-number constants
(cross-checked against `kernel-api/src/implicit.rs`, the parser itself).
Every parse error carries the **JSON path to the bad subtree** (`at
expr.b.a: …`) — fix exactly there. Parameter tables per node: API.md
"Implicit / hybrid".

### 11.2 All twelve leaves, executed

`sphere`, `box`, `cylinder` appear above and in §11.3–§14. The remaining
nine in two programs:

```json
{"ops": [
	{"id": "funnel", "op": "implicit", "voxel": 0.3,
	 "expr": {"op": "union",
		"a": {"shape": "cone", "a": [0, 0, 0], "b": [0, 0, 14], "ra": 10, "rb": 3},
		"b": {"shape": "torus", "center": [0, 0, 0], "axis": [0, 0, 1], "major": 9, "minor": 1.5}}},
	{"id": "wedge", "op": "implicit", "voxel": 0.3,
	 "expr": {"op": "intersection",
		"a": {"shape": "box", "min": [0, 0, 0], "max": [20, 12, 10]},
		"b": {"shape": "plane", "point": [0, 0, 10], "normal": [0.4472135955, 0, 0.894427191]}}},
	{"id": "manifold_pipe", "op": "implicit", "voxel": 0.3,
	 "expr": {"shape": "pipe",
		"path": [[-12, 0, 2], [-12, 0, 12], [0, 0, 18], [12, 0, 12], [12, 0, 2]],
		"radii": [3, 2.5, 2, 2.5, 3]}},
	{"id": "spring", "op": "implicit", "voxel": 0.25,
	 "expr": {"shape": "helix_pipe", "center": [0, 0, 0], "axis": [0, 0, 1],
		"r_helix": 8, "pitch": 6, "turns": 3, "radius": 1.5}},
	{"id": "octet", "op": "implicit", "voxel": 0.3, "mesher": "manifold",
	 "expr": {"shape": "beam_lattice", "min": [0, 0, 0], "max": [24, 24, 24],
		"cell": "octet", "cell_size": 12, "radius": 1.0}},
	{"id": "tripod", "op": "implicit", "voxel": 0.3, "mesher": "manifold",
	 "expr": {"shape": "beam_lattice",
		"nodes": [[0, 0, 14], [-8, -5, 0], [8, -5, 0], [0, 9, 0]],
		"struts": [[0, 1, 1.0, 2.2], [0, 2, 1.0, 2.2], [0, 3, 1.0, 2.2]]}}
]}
```

Executed, all watertight unhealed. Reading the receipts as leaf lessons:

| id | leaf lesson | measured |
|---|---|---|
| `funnel` | `cone` is a capped frustum (`rb: 0` = sharp, `ra == rb` = cylinder); the `torus` collar is *buried* (major 9, not tangent 10) — tangent-contact unions pinch (first attempt at major 10 failed `invalid_geometry` with `non_manifold_edges: 25`; bury or gap, never kiss) | V 2290.9 |
| `wedge` | `plane` is an UNBOUNDED half-space ("inside" is opposite the normal) — legal only under an intersection with something bounded, or an explicit `domain`; analytic wedge volume 1200, voxel measures 1198.2 (voxel-grade, −0.15%) | V 1198.2 |
| `manifold_pipe` | `radii` (one per path point) tapers the tube vertex-to-vertex; `radius` alone is the constant form (exactly one of the two) | V 1017.7 |
| `spring` | `helix_pipe`: cooling channels and springs without sweeping math; `samples_per_turn` (default 64, ≥ 8) controls helix smoothness | V 1089.3 |
| `octet` | cell-fill form (`min/max/cell/cell_size/radius`); junction-rich ⇒ `"mesher": "manifold"` (§11.5); fills whole cells from the low corner, ≤ 16384 cells | V 5093.6, 283,524 tris |
| `tripod` | graph form (`nodes` + `struts` `[a, b, ra, rb]` — tapering struts); your own truss topology as data | V 488.8 |

The 12th leaf, `expr_sdf`, has its own contract — §12. `gyroid` is executed
in §13.2 (graded) and §16.4 (as a Document feature, with the watertightness
caveat measured).

### 11.3 Seam and cut treatments — the blend-family sampler

One geometry (12×12×8 pad, Ø10 post), four union treatments — and the same
four as cuts. Executed, volumes as the receipt of what each treatment adds
or removes:

```json
{"ops": [
	{"id": "hard", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "union",
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 8]},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 20], "radius": 5}}},
	{"id": "filleted", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "fillet_union", "r": 3,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 8]},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 20], "radius": 5}}},
	{"id": "chamfered", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "chamfer_union", "r": 3,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 8]},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 20], "radius": 5}}},
	{"id": "blobbed", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "smooth_union", "k": 3,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 8]},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 20], "radius": 5}}}
]}
```

| treatment | volume | character |
|---|---|---|
| `union` (hard) | 5550.3 | crease — min(a, b), never fails |
| `fillet_union r 3` | 5732.7 | TRUE constant-radius quarter-round collar (+182) |
| `smooth_union k 3` | 5695.6 | polynomial blob — blend size tracks field magnitude, organic, dimension-vague (+145) |
| `chamfer_union r 3` | 5953.3 | 45° flat collar — fills the most (+403, the r²/2 triangle vs the fillet's (1−π/4)r²) |

```json
{"ops": [
	{"id": "hard_cut", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "difference",
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 10]},
		"b": {"shape": "cylinder", "a": [0, 0, -1], "b": [0, 0, 11], "radius": 4}}},
	{"id": "smooth_cut", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "smooth_difference", "k": 2.5,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 10]},
		"b": {"shape": "cylinder", "a": [0, 0, -1], "b": [0, 0, 11], "radius": 4}}},
	{"id": "fillet_cut", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "fillet_difference", "r": 2.5,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 10]},
		"b": {"shape": "cylinder", "a": [0, 0, -1], "b": [0, 0, 11], "radius": 4}}},
	{"id": "chamfer_cut", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "chamfer_difference", "r": 2.5,
		"a": {"shape": "box", "min": [-12, -12, 0], "max": [12, 12, 10]},
		"b": {"shape": "cylinder", "a": [0, 0, -1], "b": [0, 0, 11], "radius": 4}}}
]}
```

Executed: 5256.4 (hard) / 5198.2 (smooth, rounds the bore mouth) / 5180.2
(fillet, true-radius mouth) / 5066.8 (chamfer, widest mouth). Decision rule:
**fillet/chamfer when a drawing would dimension the blend; smooth when you
want flesh**. And read §14 before using any of them near parallel surfaces.

### 11.4 Transform & pattern combinators, executed

```json
{"ops": [
	{"id": "tower", "op": "implicit", "voxel": 0.35,
	 "expr": {"op": "union",
		"a": {"op": "rotate", "axis": [0, 0, 1], "degrees": 45,
			"in": {"op": "translate", "offset": [0, 0, 8],
				"in": {"shape": "box", "min": [-6, -6, 0], "max": [6, 6, 8]}}},
		"b": {"op": "scale", "factor": 1.5,
			"in": {"shape": "box", "min": [-6, -6, 0], "max": [6, 6, 5.34]}}}},
	{"id": "pair", "op": "implicit", "voxel": 0.35,
	 "expr": {"op": "mirror", "point": [14, 0, 0], "normal": [1, 0, 0],
		"in": {"shape": "capsule", "a": [0, 0, 2.5], "b": [8, 0, 2.5], "radius": 2.5}}},
	{"id": "rowcol", "op": "implicit", "voxel": 0.35,
	 "expr": {"op": "circular_pattern", "center": [0, 0, 0], "axis": [0, 0, 1], "count": 8,
		"in": {"op": "linear_pattern", "step": [0, 0, 7], "count": 3,
			"in": {"op": "translate", "offset": [16, 0, 0],
				"in": {"shape": "sphere", "center": [0, 0, 3], "radius": 3}}}}}
]}
```

Executed receipts: `tower` 3745.1 (a 45°-twisted stack — `rotate` takes
degrees + optional `center`, `scale` is uniform about the origin); `pair`
446.0 = two capsules (`mirror` = child ∪ reflection — also the 12th leaf
demo: `capsule` is a sphere-swept segment); `rowcol` 2721.9 — patterns
*nest*: 3 spheres stacked by `linear_pattern`, ringed 8× by
`circular_pattern` = 24 spheres ≈ 24·113.1 mm³. `step_degrees` defaults to
360/count. Cost model: domain repetition — each copy costs one child
evaluation per query — hence the executed cap:

> `at expr: field 'count' must be in 1..=4096, got 5000`

### 11.5 Mesher choice

`"mesher": "narrowband"` (default) is fast and surface-area-scaled but
**requires ≤ 1-Lipschitz fields** (which `expr_sdf`'s normalization gives you
*if your bound is truthful*, §12.2). `"mesher": "manifold"` samples densely,
assumes only continuity, and resolves TPMS/lattice pinch saddles — use it for
junction-rich geometry (octet lattices, gyroids — both executed above at
`manifold`), for steep modulation fields (§13.1), and for fields you cannot
bound.

### 11.6 `gyroid_block` — the one-shot lattice op

The pre-assembled sibling of the `gyroid` leaf: cube of half-extent `half`
at `center`, TPMS at `scale` (rad/mm — cell period 2π/scale) and `thickness`
(wall half-thickness), meshed by Manifold Dual Contouring, written straight
to STL, healed once if needed (`healed: true` in the measures is your
marginal-resolution warning), `invalid_geometry` if still leaky. Binds
nothing. Parameters: API.md "Implicit / hybrid". For graded walls or lattice
∩ part intersections, use the `implicit` op's `gyroid` leaf (§13.2) or the
Document features (§16.4) instead.

## 12. The scalar expression language and the `expr_sdf` contract

### 12.1 The language

Inside `expr_sdf.expr` and the `field` of `offset_by`/`lerp`:

- **Constants are bare JSON numbers** (`1.25`, not `{"const": 1.25}`).
- **The query point is the bare strings `"x"`, `"y"`, `"z"`** (mm). Executed
  with `{"var": "x"}` instead: `invalid_param … at expr.expr.a: missing
  required field 'op'`.
- Operators are objects: `add/sub/mul/div/min/max {a, b}`, `mod {a, b}`
  (Euclidean — result in `[0, b)`, the periodic-pattern workhorse),
  `neg/abs/sqrt/sin/cos {arg}`, `clamp {value, lo, hi}` (never errors, even
  for lo > hi).
- **`length2` takes `a`/`b`** — `{"op": "length2", "a": "x", "b": "y"}` is
  the cylindrical radius. Executed with `x`/`y` keys instead:
  `invalid_param … missing required field 'a'`.
- **`atan2` takes `y`/`x`** — named, order-proof:
  `{"op": "atan2", "y": "y", "x": "x"}` (radians). Executed with `a`/`b`:
  `invalid_param … missing required field 'y'`.
- `length3 {a, b, c}`.

In SDF-land, remember: `max` = intersection, `min` = union, negate = invert.
A complete original example using **thirteen of the sixteen ops in one
field** (the other three — `mod`, `atan2`, `length3` — carry §15's thread) —
a wavy-rimmed dish: a hand-rolled sphere shell (`sqrt(x² + y² + (z+20)²) −
24`, written with `sqrt/add/mul/sub` to show `length3`'s expansion), wave
amplitude ramped by height (`clamp(div(z, 4), 0, 1)`), floor-clamped with
`neg`, brim attached with `min`:

```json
{"ops": [
	{"id": "dish", "op": "implicit", "voxel": 0.3,
	 "expr": {
		"shape": "expr_sdf",
		"lipschitz_bound": 1.8,
		"min": [-26.5, -26.5, 0], "max": [26.5, 26.5, 6],
		"expr": {
			"op": "min",
			"a": {"op": "max",
				"a": {"op": "sub",
					"a": {"op": "sqrt",
						"arg": {"op": "add",
							"a": {"op": "add",
								"a": {"op": "mul", "a": "x", "b": "x"},
								"b": {"op": "mul", "a": "y", "b": "y"}},
							"b": {"op": "mul",
								"a": {"op": "sub", "a": "z", "b": -20},
								"b": {"op": "sub", "a": "z", "b": -20}}}},
					"b": {"op": "add", "a": 24.0,
						"b": {"op": "mul",
							"a": {"op": "mul", "a": 0.4,
								"b": {"op": "clamp",
									"value": {"op": "div", "a": "z", "b": 4},
									"lo": 0, "hi": 1}},
							"b": {"op": "mul",
								"a": {"op": "sin", "arg": {"op": "mul", "a": 0.7, "b": "x"}},
								"b": {"op": "cos", "arg": {"op": "mul", "a": 0.7, "b": "y"}}}}}},
				"b": {"op": "neg", "arg": "z"}},
			"b": {"op": "max",
				"a": {"op": "sub", "a": {"op": "length2", "a": "x", "b": "y"}, "b": 26},
				"b": {"op": "sub", "a": {"op": "abs", "arg": {"op": "sub", "a": "z", "b": 0.8}}, "b": 0.8}}
		}
	 },
	 "file": "wavy_dish.stl"}
]}
```

Executed: 104,500 triangles, watertight, volume 3820.42. The Lipschitz
arithmetic behind `1.8` (do this every time, §12.2): the sphere term is
1-Lipschitz; the wave term's gradient is ≤ 0.4·0.7·√2 ≈ 0.40 in-plane plus
0.4/4 = 0.1 vertical; max-combining with the 1-Lipschitz clamps keeps
sup|∇| ≤ ~1.5; declared 1.8 with margin.

### 12.2 The `expr_sdf` contract fields are REQUIRED — every clause provoked

The leaf is `{"shape": "expr_sdf", "expr": <scalar tree>, "lipschitz_bound":
L, "min": [x,y,z], "max": [x,y,z]}`:

- `lipschitz_bound` is **required** (executed without it: `invalid_param …
  at expr: missing required field 'lipschitz_bound'`). Declare a truthful
  `L ≥ sup|∇expr|`; the kernel evaluates `expr / L` — zero set unchanged,
  slope normalized.
- `min`/`max` come **together or not at all** (executed with only `min`:
  `'min' and 'max' bounds must be given together (or both omitted for an
  unbounded field)`). They declare where the surface lives and feed the
  automatic meshing domain (tree bounds padded by 3·voxel).
- Omitting both is legal **only** when something else bounds the tree.
  Executed unbounded and undomained:
  > `invalid_param: the expression tree is unbounded (a bare 'plane' or a
  > bounds-less 'expr_sdf' leaf) — intersect it with a bounded shape, give the
  > expr_sdf leaf min/max bounds, or pass an explicit 'domain'`
- A 5×5×5 **degeneracy probe** runs before extraction. Executed with a `1/z`
  pole in the domain:
  > `invalid_param: at expr.expr: expression evaluates to inf at probe point
  > [-8, -8, 0] — the field must stay finite over the meshing domain (clamp
  > denominators, keep sqrt arguments non-negative, never mod by 0)`

  The probe is a tripwire, not a proof — clamp your denominators anyway.

### 12.3 The Lipschitz honesty trap — measured spectrum

Over-declaring `lipschitz_bound` is safe (slightly slower). **Under-declaring
is not reliably caught.** Executed today, a true-distance r = 8 sphere
(volume 2144.7 analytic) at voxel 0.5 under four dishonest bounds:

| declared bound (truth: 1.0) | result |
|---|---|
| 0.9, 0.5, 0.2, 0.1 | **exit 0, watertight, volume 2146.35** — silently correct *this time*, because block seeding happened to land on the surface even with the field's apparent distances inflated 10× |
| 0.05 | exit 1, `invalid_geometry`: *"the implicit tree did not mesh watertight at voxel 0.5 (triangles=0, …)"* — pruning deleted the entire surface, which at least fails loudly |

The dangerous middle ground is **partial pruning: silent holes** in exactly
the blocks whose seeds missed. The kernel cannot verify your calculus; the
bound is your engineering statement. If no bound is practical, switch to
`"mesher": "manifold"` (no Lipschitz assumption, §11.5).

## 13. Field modulation — `offset_by`, `lerp`, graded lattices

### 13.1 The four modulation moves, executed

```json
{"ops": [
	{"id": "inflated", "op": "implicit", "voxel": 0.35,
	 "expr": {"op": "offset", "t": 1.5,
		"in": {"shape": "box", "min": [-8, -8, 0], "max": [8, 8, 10]}}},
	{"id": "hollow", "op": "implicit", "voxel": 0.35,
	 "expr": {"op": "shell", "t": 1.0,
		"in": {"shape": "sphere", "center": [0, 0, 0], "radius": 12}}},
	{"id": "morph", "op": "implicit", "voxel": 0.4,
	 "expr": {"op": "intersection",
		"a": {"op": "lerp",
			"a": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 30], "radius": 10},
			"b": {"shape": "box", "min": [-10, -10, 0], "max": [10, 10, 30]},
			"field": {"op": "clamp", "value": {"op": "div", "a": "z", "b": 30}, "lo": 0, "hi": 1}},
		"b": {"shape": "box", "min": [-16, -16, 0], "max": [16, 16, 30]}}},
	{"id": "rippled", "op": "implicit", "voxel": 0.35, "mesher": "manifold",
	 "expr": {"op": "offset_by", "max_abs": 1.2,
		"in": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 24], "radius": 8},
		"field": {"op": "mul", "a": 1.0, "b": {"op": "sin", "arg": {"op": "mul", "a": 0.8, "b": "z"}}}}}
]}
```

| id | move | measured |
|---|---|---|
| `inflated` | `offset t` — uniform inflate (+) / deflate (−); rounds convex corners by construction | V 4600.2 (box 2560 grown 1.5 all round) |
| `hollow` | `shell t` — hollow wall of **total thickness 2·t** straddling the surface | V 3627.7 = (4/3)π(13³ − 11³) exactly the ±1 mm shell |
| `morph` | `lerp(a, b, field)` — pointwise distance blend, weight clamp(field, 0, 1): a cylinder that becomes a box over its height (round-to-square transition pieces, solid-to-lattice gradients) | V 10575.0 |
| `rippled` | `offset_by(field, max_abs)` — the surface moves outward by field(p) mm, clamped: a sin-rippled barrel | V 5059.0 |

The modulation caveat (same as the Rust API): the modulated result is only
`(1 + |∇field|)`-Lipschitz. `rippled`'s field slope reaches 0.8 — too steep
for the narrow-band contract, so it is extracted with `"mesher": "manifold"`
above; *as a rule keep graded fields to a few % per mm for narrowband, or go
manifold*. (`morph`'s ramp is 1/30 per mm — narrowband-safe, and its lerp
endpoints are true SDFs.)

### 13.2 Graded gyroid — the density-gradient workhorse, executed

Uniform vs graded, same 30 mm puck (gyroid `scale` 0.45 rad/mm ⇒ ~14 mm
cells, wall half-thickness 0.7), grade = `0.02·(30 − z)`: +0.6 mm walls at
the bottom fading to nominal at the top — clamped by `max_abs: 0.6`, slope
0.02 ≪ 1 honoring the contract, extracted manifold (TPMS = junction-rich):

```json
{"ops": [
	{"id": "uniform", "op": "implicit", "voxel": 0.5, "mesher": "manifold",
	 "domain": {"min": [-15, -15, 0], "max": [15, 15, 30]},
	 "expr": {"op": "intersection",
		"a": {"shape": "gyroid", "min": [-15, -15, 0], "max": [15, 15, 30],
		      "scale": 0.45, "thickness": 0.7},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 30], "radius": 14}}},
	{"id": "graded", "op": "implicit", "voxel": 0.5, "mesher": "manifold",
	 "domain": {"min": [-15, -15, 0], "max": [15, 15, 30]},
	 "expr": {"op": "intersection",
		"a": {"op": "offset_by", "max_abs": 0.6,
			"in": {"shape": "gyroid", "min": [-15, -15, 0], "max": [15, 15, 30],
			       "scale": 0.45, "thickness": 0.7},
			"field": {"op": "mul", "a": 0.02, "b": {"op": "sub", "a": 30, "b": "z"}}},
		"b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 30], "radius": 14}}}
]}
```

Executed: uniform 3703.5 mm³ → graded 6512.6 mm³ (1.76× material, biased to
the loaded end), both watertight at voxel 0.5. The persistable Document twin
of this exact pattern (`GyroidLattice` + `LinearGrade`) is §16.4 — use the
op for exploration, the feature for the part file. The grade-law honesty
note applies to both: the graded field is only `(1 +
|per_unit|·|axis|)`-Lipschitz — keep slopes a few % per mm.

## 14. The pillow trap: blends bulge wherever surfaces run parallel

`fillet_union`/`smooth_union` are **global field operations**, not local seam
treatments: everywhere the two operands' surfaces sit within the blend radius
of each other, material is added — including *buried* faces and *parallel*
walls nowhere near the visible seam. The receipts stay green (watertight,
valid); only geometric probes catch it. First measured in the iPhone-stand
project (`iphone_stand/DESIGN.md` §4 v4.1 and §8 finding 3: a 0.6 mm panel
step pillowed ~1.6 mm proud; a bed face bulged 0.46 mm downward). Minimal
reproduction, executed for this guide:

```json
{"ops": [
	{"id": "hard", "op": "implicit", "voxel": 0.25,
	 "expr": {"op": "union",
		"a": {"shape": "box", "min": [-20, -20, 0], "max": [20, 20, 6]},
		"b": {"shape": "box", "min": [-10, -10, 2.5], "max": [10, 10, 12]}},
	 "file": "pillow_hard.stl"},
	{"id": "pillowed", "op": "implicit", "voxel": 0.25,
	 "expr": {"op": "fillet_union", "r": 5,
		"a": {"shape": "box", "min": [-20, -20, 0], "max": [20, 20, 6]},
		"b": {"shape": "box", "min": [-10, -10, 2.5], "max": [10, 10, 12]}},
	 "file": "pillow_bulged.stl"},
	{"id": "reclamped", "op": "implicit", "voxel": 0.25,
	 "expr": {"op": "intersection",
		"a": {"op": "fillet_union", "r": 5,
			"a": {"shape": "box", "min": [-20, -20, 0], "max": [20, 20, 6]},
			"b": {"shape": "box", "min": [-10, -10, 2.5], "max": [10, 10, 12]}},
		"b": {"shape": "plane", "point": [0, 0, 0], "normal": [0, 0, -1]}},
	 "file": "pillow_reclamped.stl"}
]}
```

The riser's bottom face (z = 2.5) is buried inside the base and runs
parallel to the bed face (z = 0), 2.5 mm away — inside the r = 5 collar.
Executed receipts and STL bed-plane probe (z_min measured from each exported
STL with a 10-line script over the binary triangles):

| op | volume (mm³) | STL z_min | verdict |
|---|---|---|---|
| `hard` | 11998.8 (analytic 12000) | **0.0000** | hard ops never bulge |
| `pillowed` | 12730.0 (**+731**) | **−0.4428** | the bed face pillowed 0.44 mm below the build plane — `ok: true, watertight: true` throughout |
| `reclamped` | 12465.6 | **0.0000** | the fix |

The operating rule, straight from the measured fix: **fuse first, cut last**
— apply soft unions before the precision cuts, then re-clamp every datum
plane with a *hard* op (`intersection` with a half-space, hard `difference`)
as the final shaping step. And probe datums: a bbox/z_min check on the
export, a `volume_within` window against the hard-union volume — receipts
alone will smile through a pillow.

## 15. Threads and helical features — the implicit half's signature move

Real helical threads self-intersect when buried in a shank, so no exact
boolean can build them — the implicit extraction fuses them watertight (BAR
I6: an M10×1.5 bolt rebuilt from pure JSON to <0.0001% of its Rust
reference; full program in API.md). The reusable idiom, compact and executed
— a Ø8 stud with a 60°-ish V-thread, pitch 1.25, cut as a helical groove:

```json
{"ops": [
	{"id": "stud", "op": "implicit", "voxel": 0.12,
	 "domain": {"min": [-4.4, -4.4, -0.4], "max": [4.4, 4.4, 16.4]},
	 "expr": {
		"op": "difference",
		"a": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 16], "radius": 4},
		"b": {
			"shape": "expr_sdf",
			"lipschitz_bound": 1.7,
			"min": [-4.2, -4.2, 1.8], "max": [4.2, 4.2, 14.2],
			"expr": {
				"op": "max",
				"a": {"op": "max",
					"a": {"op": "sub", "a": 2.0, "b": "z"},
					"b": {"op": "sub", "a": "z", "b": 14.0}},
				"b": {"op": "max",
					"a": {"op": "sub", "a": 3.25,
						"b": {"op": "length2", "a": "x", "b": "y"}},
					"b": {"op": "sub",
						"a": {"op": "abs", "arg": {"op": "sub",
							"a": {"op": "mod",
								"a": {"op": "sub", "a": "z",
									"b": {"op": "mul", "a": 0.198943678865,
										"b": {"op": "atan2", "y": "y", "x": "x"}}},
								"b": 1.25},
							"b": 0.625}},
						"b": {"op": "mul", "a": 0.6,
							"b": {"op": "sub",
								"a": {"op": "length2", "a": "x", "b": "y"},
								"b": 3.25}}}}}
		}
	 },
	 "file": "threaded_stud.stl"}
]}
```

Executed: watertight, `healed: false`, 125,328 triangles, volume 728.30 mm³
(plain stud 804.25 — the groove removed what a V-groove should). Anatomy,
line by line:

1. **Helical unwrap**: `u = mod(z − (P/2π)·atan2(y,x), P)` — axial offset to
   the current turn, `P = 1.25`, `P/2π = 0.198943678865`. Continuous across
   the atan2 branch cut because the jump is exactly one pitch.
2. **Recenter branchlessly**: `abs(u − P/2)` = distance from the groove
   centreline, no conditionals.
3. **Flank slope**: groove half-width grows `0.6` per mm of radius above the
   root (`length2(x,y) − 3.25`) — a V whose included angle is
   `2·atan(0.6) ≈ 62°`. At the crest (r = 4) the half-width is 0.45 < P/2, so
   lands of material remain between turns.
4. **Span and root clamps** as `max` terms (intersection in SDF-land):
   `z ∈ [2, 14]`, `r ≥ 3.25`.
5. **Lipschitz accounting** (do this every time): `|∇(z − kθ)| ≤
   √(1 + (k/r)²) ≈ 1.002` over the band (r ≥ 3.25), the radial term adds 0.6,
   the clamps are 1-Lipschitz → sup|∇| ≤ 1.602; declared **1.7** with margin.
   The kernel divides by it — geometry unchanged, pruning safe.
6. For a thread **ridge** (a bolt, not a groove): build the flank planes of
   the trapezoid in the `(r, u)` plane and `union` it onto the shank — the
   union self-intersects and only the implicit extraction can fuse it; see
   the M10 program in API.md ("The I6 proof").

Voxel guidance for threads: ≥ ~3 voxels across the thread depth — 0.12 here
for a 0.65-deep groove; 0.06–0.08 is the resin-grade band the M10 showcase
used (§17.4).

---
# Part IV — Recipes, assemblies, libraries, catalog

## 16. `.lmcpart` — the recipe grammar, complete

### 16.1 The envelope and every Document field

The envelope's `format` (`"lmc-part"`), `version` (1) and `units` (`"mm"`)
are contract fields, checked before the payload — non-mm files are refused,
never rescaled. `name` and `created_with` are echoed as `load_part` measures.
The `document` payload:

| field | meaning | demonstrated |
|---|---|---|
| `params` | `{name: number}` — the public knobs | §16.2 (and every part file in this guide) |
| `features` | ordered array; each entry one `Feature` variant, optionally with `"label"` and `"notes"` beside it (the I5 human⇄AI handoff fields — inert to geometry, serialized next to the variant where a hand-editor expects them) | §16.2 |
| `root` | which feature is the part's result (omit = the last) | §16.2 table |
| `suppressed` | feature indices toggled off — **modifier** features (single-input: fillet, chamfer, hole, transform, pattern, shell…) fall back to their input; suppressing a generative feature (primitive, boolean operand) is a no-op for solids that never referenced it | §16.2 table |
| `configs` | named parameter-override sets — one model, several variants (`BTreeMap`, byte-stable saves) | §16.2 table |
| `active_config` | which override set evaluation uses (omit for base; an unknown name resolves to no overrides) | §16.2 table |

Dimensions are `Dim`s: `{"Literal": 4.0}` or `{"Param": "h"}` — no arithmetic
in the file; derive values in your head or add a parameter. Saves are
byte-stable (sorted keys), so recipes git-diff like code.

### 16.2 Every field exercised — one file, four measured variants

`pad.lmcpart` — a box + hole-wizard drill whose width is config-driven:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "drilled_pad",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"pad_w": 40.0, "bore_d": 6.0},
		"features": [
			{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 4.0}],
			         "size": [{"Param": "pad_w"}, {"Literal": 24.0}, {"Literal": 8.0}]},
			 "label": "pad", "notes": "mounting pad; width is the config-driven dimension"},
			{"Hole": {"input": 0, "kind": "Drill", "m_or_d": {"Param": "bore_d"},
			          "at": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 8.0}],
			          "axis": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": -1.0}]},
			 "label": "axle bore"}
		],
		"root": 1,
		"suppressed": [],
		"configs": {"wide": {"pad_w": 60.0}}
	}
}
```

```json
{"ops": [
	{"id": "pad", "op": "load_part", "file": "pad.lmcpart"},
	{"id": "xv", "op": "exact_volume", "in": "pad"},
	{"id": "topo", "op": "validate", "in": "pad"}
]}
```

Executed via `load_part` + `exact_volume` on four hand-edited variants:

| variant (the hand edit) | exact_volume | validate |
|---|---|---|
| as written | 7453.805 (= 7680 − π·3²·8, machine-exact) | genus 1 |
| `"active_config": "wide"` added | 11293.805 (= 11520 − π·3²·8) | genus 1 |
| `"suppressed": [1]` | 7680.0 exactly | genus 0 — the hole modifier fell back to its input |
| `"root": 0` | 7680.0 exactly | genus 0 — the result is the un-drilled feature 0 |

That is the whole edit-model: parameters, configurations, suppression and
root selection are *file edits*, and the deterministic rebuild does the rest.

### 16.3 Document grammar ≠ op grammar — the three corners that bite

1. Document sketches wrap indices in objects (`"segments": [{"a": 0, "b":
   1}]`, `"arcs": [{"a", "b", "center", "ccw"}]`, `"circles": [{"center",
   "radius_point"}]`), and constraints are externally-tagged PascalCase
   (`{"Fixed": {"point": 0, "at": [0, 0]}}`) — unlike the op surface's bare
   pairs and `"kind"`-tagged forms. Executed: §16.5's `drafted_pad`.
2. The op surface is degrees everywhere; the Document's `ExtrudeSketch.draft`
   and `CircularPattern.angle` are **radians** (executed: §16.5, §9.3).
3. `Transform.xform` is 12 floats, **column-major** (three basis columns,
   then translation); `.lmcasm` quaternions are `[x, y, z, w]`. Executed:
   §16.6's `pin_pair`.

Feature-variant names are PascalCase (`"Box"`, `"Hole"`, `"kind": "Drill"`,
`"Boolean": {"op": "Difference"}`), unlike the snake_case op surface.

### 16.4 The complete feature table — all 30 variants

Authoritative against `kernel_model::Feature` (lib.rs) on current main.
"Half" says where the feature evaluates: **B** = exact B-rep
(`load_part`-able), **V** = voxel/implicit only (meshes via `.lmcasm` /
`Document::mesh`; `load_part` refuses, §16.7), **B+V** = both.

| # | variant | half | one-liner | executed |
|---|---|---|---|---|
| 1 | `Box` | B+V | centred box, `center`+`size` | §3.3, §16.2 |
| 2 | `Sphere` | B+V | centred sphere | (form mirrors Box; covered by op §5.1) |
| 3 | `Cylinder` | B+V | axis +Z, `center` is the body centroid (spans center.z ± h/2) | §3.3, §9.3 |
| 4 | `FilletedCylinder` | B | cylinder with rolled top rim (parametric boss/pin) | §16.6 |
| 5 | `ChamferedCylinder` | B | cylinder with beveled top rim | §16.6 |
| 6 | `Boolean` | B+V | `{"op": "Union"/"Difference"/"Intersection", "a", "b"}` over feature ids | §3.3, §16.2 |
| 7 | `SmoothUnion` | V | blended union, `blend` radius | §16.7 (refusal class) |
| 8 | `SmoothDifference` | V | blended carve | (same class as 7) |
| 9 | `SmoothIntersection` | V | blended common volume | (same class as 7) |
| 10 | `Gyroid` | V | TPMS block, `center`+`size` form | superseded by 27 for new work |
| 11 | `Transform` | B+V | rigid+uniform-scale, 12-float column-major | §16.6 |
| 12 | `Fillet` | B | constant-radius round on a **persistent EdgeName**, optional `near` disambiguator | §16.6 |
| 13 | `Chamfer` | B | flat bevel, same naming semantics | (same mechanism as 12) |
| 14 | `ExtrudeSketch` | B | Document-sketch extrude: `dims` overrides + radian `draft` | §16.5 |
| 15 | `LinearPattern` | B+V | count × step copies, boolean-fused | §9.3 |
| 16 | `Mirror` | B+V | original ∪ reflection across a plane | §9.3 |
| 17 | `CircularPattern` | B+V | count copies about an axis, radian `angle` | §9.3 |
| 18 | `Shell` | V | hollow to wall `thickness` (outer faces preserved) | §16.7 (refusal class) |
| 19 | `Hole` | B | the hole wizard as a feature (`kind`: Drill/Clearance/Counterbore/Countersink/Tap; `fit` only on clearance-family, `depth` only on drill/tap — wrong combinations fail loudly) | §16.2 |
| 20 | `CircularRimFillet` | B | exact-torus rim fillet; `concave: true` = bore exit lip | §16.6 |
| 21 | `LoftSolid` | B | section-stack loft, `Dim` points | §5.4 |
| 22 | `SweepSolid` | B | profile along path, RMF, capped | §5.4 |
| 23 | `CatalogPart` | B | any §20 standard part as one feature | gearbox recipes (e.g. `gearbox/parts/*.lmcpart`) |
| 24 | `ORingGroove` | B | AS568 shaft gland cut | cut twin tabled §20.3 (runnable example in API.md; same kernel function) |
| 25 | `CirclipGroove` | B | DIN 471/472 groove, `internal` flag | cut twins tabled §20.3 (API.md examples) |
| 26 | `HeatsetBoss` | B | insert boss + undersized pocket | op twin executed §20.3 |
| 27 | `GyroidLattice` | V | corner-form TPMS with optional `LinearGrade` law | §16.7 |
| 28 | `BeamLatticeFill` | V | cubic/octet cell fill of a region | §17.2 (hybrid operand) |
| 29 | `PipeFeat` | V | polyline tube, per-vertex radii | §17.1 |
| 30 | `HybridFuse` | B+V | one cross-representation boolean (§17) | §17.1 |

B-rep-only features are honestly absent from the implicit preview
(`Document::evaluate` passes fillets through unrounded, skips sketches), and
voxel-only features are honestly absent from `evaluate_brep` — neither half
fakes the other.

### 16.5 `ExtrudeSketch` — Document sketches, dims overrides, radian draft

Save as `drafted_pad.lmcpart`. The sketch is the §3.2 rectangle in Document
spelling; `dims: [[2, {"Param": "w"}]]` re-targets constraint #2 (the width
`Distance`) from the parameter table **before solving**, so the parameter
drives the *profile*, not just the height; `draft` is the one radian field:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "drafted_pad",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"w": 50.0},
		"features": [
			{"ExtrudeSketch": {
				"sketch": {
					"points": [[0.0, 0.0], [48.0, 1.0], [49.0, 30.0], [-1.0, 29.0]],
					"segments": [{"a": 0, "b": 1}, {"a": 1, "b": 2}, {"a": 2, "b": 3}, {"a": 3, "b": 0}],
					"arcs": [],
					"circles": [],
					"constraints": [
						{"Fixed": {"point": 0, "at": [0.0, 0.0]}},
						{"Horizontal": {"a": 0, "b": 1}},
						{"Distance": {"a": 0, "b": 1, "distance": 50.0}},
						{"Vertical": {"a": 0, "b": 3}},
						{"Distance": {"a": 0, "b": 3, "distance": 30.0}},
						{"Horizontal": {"a": 3, "b": 2}},
						{"Vertical": {"a": 1, "b": 2}}
					]
				},
				"height": {"Literal": 12.0},
				"dims": [[2, {"Param": "w"}]],
				"draft": {"Literal": 0.0349066}
			}, "label": "drafted pad", "notes": "draft is RADIANS in the Document grammar: 0.0349066 = 2 deg"}
		],
		"root": 0,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "pad", "op": "load_part", "file": "drafted_pad.lmcpart"},
	{"id": "v", "op": "volume", "in": "pad"},
	{"id": "da", "op": "draft_analysis", "in": "pad", "pull": [0, 0, 1], "min_deg": 1.5}
]}
```

Executed: volume 17600.52, `min_draft_deg: 2.0000`; the hand-edited
`"w": 70.0` variant re-solves the *sketch* and measures 24699.95 — the
profile moved, not just a scale factor. (Draft note: with nonzero draft only
the outer boundary is drafted — documented in the feature.)

### 16.6 Topological persistence + the cylinder-family features, executed

`filleted_bar.lmcpart` — the §8.4 persistent-name demonstration. The
`Fillet` stores the edge as a pair of face names; on the canonical box the
faces are numbered 0=−Z 1=+Z 2=−Y 3=+Y 4=−X 5=+X, so `[{Primitive, 1},
{Primitive, 2}]` is the front-top edge — *as a name, not a position*:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "filleted_bar",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"w": 30.0},
		"features": [
			{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 5.0}],
			         "size": [{"Param": "w"}, {"Literal": 20.0}, {"Literal": 10.0}]},
			 "label": "bar"},
			{"Fillet": {"input": 0,
			            "edge": [{"operand": "Primitive", "source_face": 1},
			                     {"operand": "Primitive", "source_face": 2}],
			            "radius": {"Literal": 2.0}},
			 "label": "soft front edge",
			 "notes": "edge named (+Z cap, -Y wall): survives any w edit"}
		],
		"root": 1,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "bar", "op": "load_part", "file": "filleted_bar.lmcpart"},
	{"id": "xv", "op": "exact_volume", "in": "bar"},
	{"id": "topo", "op": "validate", "in": "bar"}
]}
```

Executed: exact_volume **5974.2478** = 6000 − (1−π/4)·2²·30. Hand-edit
`"w": 44.0` and re-load (executed): **8762.2301** = 8800 − (1−π/4)·2²·44 —
the same stored name re-attached to the now-44-mm edge. *That* is what
"persistent topological naming" buys: the feature survives re-dimensioning.
If a later boolean splits the named edge into fragments, the optional
`"near": [x, y, z]` `Dim`-triple picks the fragment (instead of failing as
ambiguous); `Chamfer` works identically with a planar bevel.

`pin_pair.lmcpart` — `FilletedCylinder` + `ChamferedCylinder` (parametric
rounded/beveled pins, base at the local origin) + the 12-float `Transform`:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "pin_pair",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {},
		"features": [
			{"FilletedCylinder": {"radius": {"Literal": 6.0}, "height": {"Literal": 12.0},
			                      "fillet": {"Literal": 2.0}}, "label": "rounded-top pin"},
			{"ChamferedCylinder": {"radius": {"Literal": 6.0}, "height": {"Literal": 12.0},
			                       "chamfer": {"Literal": 2.0}}, "label": "beveled-top pin"},
			{"Transform": {"input": 1, "xform": [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0,  20.0, 0.0, 0.0]},
			 "notes": "12 floats, COLUMN-major: x-axis, y-axis, z-axis, translation"},
			{"Boolean": {"op": "Union", "a": 0, "b": 2}, "label": "side-by-side pair"}
		],
		"root": 3,
		"suppressed": []
	}
}
```

`bore_lip.lmcpart` — the **concave** rim fillet (`concave: true` →
`fillet_circular_rim_concave`, the bore-exit-lip kernel). Honest scope, from
the kernel doc and an executed rejection: the cap structure must be what a
*boolean bore cut* emits — this octagonal sketch-extruded pad drilled by a
`Cylinder` difference qualifies; a `Hole`-wizard drill through a plain
cylinder cap does **not** yet (first attempt refused with the §16.7
voxel-half message — the feature failed to evaluate, loudly, rather than
returning an unrounded solid):

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "bore_lip",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"lip_r": 1.0},
		"features": [
			{"ExtrudeSketch": {
				"sketch": {
					"points": [[12.0, 0.0], [8.4853, 8.4853], [0.0, 12.0], [-8.4853, 8.4853],
					           [-12.0, 0.0], [-8.4853, -8.4853], [0.0, -12.0], [8.4853, -8.4853]],
					"segments": [{"a": 0, "b": 1}, {"a": 1, "b": 2}, {"a": 2, "b": 3}, {"a": 3, "b": 4},
					             {"a": 4, "b": 5}, {"a": 5, "b": 6}, {"a": 6, "b": 7}, {"a": 7, "b": 0}],
					"arcs": [], "circles": [], "constraints": []
				},
				"height": {"Literal": 8.0},
				"dims": [],
				"draft": {"Literal": 0.0}
			}, "label": "octagonal pad"},
			{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 4.0}],
			              "radius": {"Literal": 5.0}, "height": {"Literal": 12.0}}},
			{"Boolean": {"op": "Difference", "a": 0, "b": 1}, "label": "drilled pad"},
			{"CircularRimFillet": {"input": 2,
			                       "near": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 9.0}],
			                       "radius": {"Param": "lip_r"}, "concave": true},
			 "label": "rounded bore exit lip",
			 "notes": "concave: true selects the bore-lip kernel (fillet_circular_rim_concave)"}
		],
		"root": 3,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "pins", "op": "load_part", "file": "pin_pair.lmcpart"},
	{"id": "pins_gate", "op": "assert", "in": "pins", "shells": 2, "valid": true},
	{"id": "pins_xv", "op": "exact_volume", "in": "pins"},
	{"id": "lip", "op": "load_part", "file": "bore_lip.lmcpart"},
	{"id": "lip_xv", "op": "exact_volume", "in": "lip"},
	{"id": "lip_topo", "op": "validate", "in": "lip"}
]}
```

Executed: pin pair 2617.36 mm³, two shells; bore lip genus 1, exact_volume
**2622.9936** — the lip fillet shed 7.0 mm³ of the sharp corner ring into
the bore mouth, machine-exact (the Wave-1 "concave bore-rim torus fillets"
capability, persisted).

### 16.7 Voxel-half recipes cannot enter the solid environment — by contract

A TPMS/shell/smooth-boolean/pipe-rooted recipe has no exact B-rep, so
`load_part` refuses it. Executed with `damper.lmcpart` (§16.8):

> `invalid_param: op 'damper': the part's feature tree produced no exact
> B-rep (voxel-half-only features — shell, gyroid, smooth booleans — cannot
> enter the solid environment)`

Voxel-half recipes live in **assemblies** (the `.lmcasm` runner meshes them
through `Document::mesh` and routes honestly — §16.8) or behind a
`HybridFuse` that re-enters the exact world (§17).

### 16.8 `GyroidLattice` with a grade law — the persisted graded damper

A Rust grading closure cannot persist, so the file form is the declarative
`LinearGrade`: `field(p) = offset + per_unit·(axis·p)`, clamped to
`±max_abs`; the lattice surface moves outward by the field (negative
carves). Keep the slope a few % per mm (§13.2's law). Save as
`damper.lmcpart` — a 30×30×20 gyroid puck, stiff bottom (+0.25 mm) to soft
top (−0.25 mm), clipped by a cylinder:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "graded_damper",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {"cell_scale": 0.55, "wall": 1.3},
		"features": [
			{"GyroidLattice": {
				"region": [[{"Literal": -15.0}, {"Literal": -15.0}, {"Literal": 0.0}],
				           [{"Literal": 15.0}, {"Literal": 15.0}, {"Literal": 20.0}]],
				"scale": {"Param": "cell_scale"},
				"thickness": {"Param": "wall"},
				"grade": {
					"axis": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 1.0}],
					"per_unit": {"Literal": -0.025},
					"offset": {"Literal": 0.25},
					"max_abs": {"Literal": 0.3}
				}},
			 "label": "graded web", "notes": "stiff bottom (+0.25 mm) to soft top (-0.25 mm)"},
			{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 10.0}],
			              "radius": {"Literal": 14.0}, "height": {"Literal": 20.0}}},
			{"Boolean": {"op": "Intersection", "a": 0, "b": 1}, "label": "puck"}
		],
		"root": 2,
		"suppressed": []
	}
}
```

Run it through a one-instance floor plan, `damper_run.lmcasm`:

```json
{
	"format": "lmc-asm",
	"version": 1,
	"units": "mm",
	"name": "damper_run",
	"instances": [
		{"name": "damper", "source": {"path": "damper.lmcpart"},
		 "pose": {"translation": [0.0, 0.0, 0.0]}}
	],
	"mates": []
}
```

Executed, `kernel-api asm damper_run.lmcasm`: at the default `--voxel 0.4`
the damper exports `route: "voxel_healed"`, 131,132 triangles,
**`watertight: false`** (TPMS saddle pinches — and the asm run still exits
0: *gate on the receipt*, §18.5); re-run at `--voxel 0.25` → 339,888
triangles, **`watertight: true`**. Rule of thumb: walls/struts need ≥ ~3
voxels across, junction-rich shapes more — refine until the receipt says
watertight (§17.4 has the selection table).

## 17. Hybrid parts — `HybridFuse`, routes, rails, and the voxel knob

### 17.1 One cross-representation boolean

`{"HybridFuse": {"brep": id, "field": id, "op":
"Union"|"Difference"|"Intersection", "voxel": Dim}}` stitches an exact B-rep
operand with an implicit operand (the field side is meshed at `voxel` for
the seam; `voxel` ≤ 0 auto-picks ≈ 1/96 of the relevant bounding diagonal).
Route semantics, honestly:

- **`ExactStitch`**: untouched exact faces are kept *verbatim*
  (bit-identical, provenance-tagged); the result is a real `Solid` that
  **feeds downstream B-rep features** and `load_part`.
- **`Healed`** (or refused): everything is voxel-resampled; the exact half
  returns nothing, so `load_part` refuses the recipe (§16.7 message) — but
  `Document::mesh` (the `.lmcasm` route) still delivers the watertight mesh
  with the route named.
- **The 50k-triangle operand rail** (`HYBRID_EXACT_MAX_OPERAND_TRIS`): an
  operand mesh denser than 50,000 triangles self-demotes to the heal with
  the reason `operand mesh too dense for the exact arrangement … re-mesh
  coarser or accept the heal`. The fuse `voxel` controls operand density —
  coarsen it to re-enter the exact route, or accept the heal.

Executed both ways. A plate + implicit pipe handle stitches exactly — save
as `handle_pad.lmcpart`:

```json
{
	"format": "lmc-part",
	"version": 1,
	"units": "mm",
	"name": "handle_pad",
	"created_with": "DESIGN_GUIDE.md by hand",
	"document": {
		"params": {},
		"features": [
			{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 3.0}],
			         "size": [{"Literal": 40.0}, {"Literal": 20.0}, {"Literal": 6.0}]},
			 "label": "exact base plate"},
			{"PipeFeat": {
				"path": [[{"Literal": -10.0}, {"Literal": 0.0}, {"Literal": 2.0}],
				         [{"Literal": -10.0}, {"Literal": 0.0}, {"Literal": 12.0}],
				         [{"Literal": 10.0}, {"Literal": 0.0}, {"Literal": 12.0}],
				         [{"Literal": 10.0}, {"Literal": 0.0}, {"Literal": 2.0}]],
				"radii": [{"Literal": 2.5}, {"Literal": 2.5}, {"Literal": 2.5}, {"Literal": 2.5}]},
			 "label": "carry handle (implicit pipe)"},
			{"HybridFuse": {"brep": 0, "field": 1, "op": "Union", "voxel": {"Literal": 0.5}},
			 "label": "fused"}
		],
		"root": 2,
		"suppressed": []
	}
}
```

```json
{"ops": [
	{"id": "pad", "op": "load_part", "file": "handle_pad.lmcpart"},
	{"id": "topo", "op": "validate", "in": "pad"},
	{"id": "stl", "op": "export_stl", "in": "pad", "file": "handle_pad.stl"}
]}
```

Executed: **genus 1** (the handle arch makes a real tunnel through the fused
solid), export `route: "exact"`, 5,492 triangles, watertight — an implicit
pipe is now part of an exact solid, loadable, cuttable, STEP-able.

### 17.2 The self-demoting heal and the remedy chain

The same recipe with an octet `BeamLatticeFill` crown instead demoted to the
heal (seam complexity past the stitcher's budget): `load_part` refused with
the §16.7 voxel-half message, and the `.lmcasm` route shipped it
`voxel_healed` — watertight at `--voxel 0.2` (311,724 triangles),
`watertight: false` at 0.3 (measured for this guide's v1 edition; behavior
class re-confirmed by §16.8's damper on current main). The remedy chain when
a fuse does not go the way you wanted, in order:

1. **Wanted exact, got healed** → coarsen the fuse `voxel` (operand density
   is the usual demotion reason — the 50k rail), simplify the seam region,
   or keep the lattice as its own `.lmcasm` instance instead of fusing.
2. **Healed and leaky** → refine the *runner's* `--voxel` (§16.8: false at
   0.4 → true at 0.25); junction-rich fields may also need the manifold
   mesher (automatic on the Document mesh path).
3. **Either way** — the route and its reason are in the receipts
   (`Document::hybrid_fuse_result` Rust-side; per-instance `route` in asm
   reports). Never infer the route from looks; read it.

### 17.3 The datum rule for fused parts

Blends bulge (§14). When a `HybridFuse`/smooth-union part carries a datum
plane (a bed face, a mating face), **fuse first, cut last**: do the soft
fusion, then re-clamp the datum with a hard boolean as the final feature —
and put a probe (bbox z_min, `volume_within`) in the part's check program.
Measured end-to-end in §14 (−0.4428 mm pillow, exact 0.0000 after the
clamp).

### 17.4 Voxel-size selection table

Voxel cost scales ~1/voxel² (narrow-band, surface-dominated) to ~1/voxel³
(dense/manifold paths). Measured anchors from this repo's shipped artifacts:

| intent | voxel (mm) | evidence (all measured) |
|---|---|---|
| draft/iteration loop | 0.5–0.8 | §13.2 gyroids watertight at 0.5 (manifold); API.md graded lattice pinned at 0.8 |
| FDM production (0.2 mm layers, ~100 mm part) | 0.3–0.4 | iPhone stand shipped at 0.40 — 458k triangles, chord error ≪ a layer line (`iphone_stand/DESIGN.md` §8.4); damper watertight at 0.25 (§16.8) |
| resin/SLA production, fine threads | 0.06–0.12 | §15 stud at 0.12 (Ø8, 0.65-deep thread); the M10 bolt reference at 0.08 agreed with Rust to <0.0001%, 0.06 = the showcase's resin-grade choice (API.md I6) |
| sub-voxel fits | do not — switch representation | B-rep tessellation at `tol` measures a 0.5 mm fit as 0.4976 (§18.3); voxel meshes are honest only to ~the voxel |

Refinement triggers, both directions: leaky receipt → finer (§16.8);
hybrid demoted to heal → coarser fuse voxel (§17.2). Keep ≥ ~3 voxels across
every wall/strut; `healed: true` on a passing op is the marginal-resolution
warning (measured on a 0.05 mm-wall `gyroid_block` at 0.5 — shipped
watertight only via the heal).

### 17.5 Complexity rails that protect you

All loud or documented (`docs/NUMERICS.md`, API.md): dense meshers cap at 2²⁸
conceptual cells (the narrow-band path indexes up to 2⁴⁴ — it never
allocates the lattice); implicit `linear_pattern`/`circular_pattern` count ≤
4096 (provoked, §11.4); `beam_lattice` cell fills ≤ 16384 cells; pipe/helix
≤ 100,000 segments; `samples_per_turn` 8…1024. Model near the origin in mm —
the f32 voxel side is honest to ~1e6 mm; the f64 B-rep side re-centres
booleans automatically beyond |centre| > 1e7.

## 18. `.lmcasm` — assemblies, complete

> **2026-07-17: you usually don't hand-write these anymore.** The assembly
> surface is now first-class **in-program ops** — `asm_instance`,
> `asm_mate`/`asm_mate_axis`/`asm_mate_face`, the DOF-honest `asm_solve`,
> `asm_contacts`, `asm_export`/`asm_export_step`, and `asm_save`, which writes
> the `.lmcasm` for you (program-built parts become `{"mesh": …}` sources —
> a new source kind alongside `path`/`part`/`asm_path`). Mate kinds grew
> `Angle`, `AxisDistance` (the gear center-distance mate) and `Fixed`. See
> API.md "Assembly ops (in-program)". This section remains the file-format
> ground truth — everything below still loads and runs.

### 18.1 Anatomy: instances, poses, mates, states, suppression

Instances (each a recipe **by relative path**, **embedded inline** as a
full part envelope, a sub-assembly by `asm_path`, or a triangle-mesh file by
`{"mesh": "part.stl"}` — measured honestly on its welded mesh) at rigid
poses, plus mates and optional named states.
The complete grammar in one executed file — all four mate types, an
instance-level suppression, a state with its own suppression set. Save as
`mates_tour.lmcasm` (it reuses §3.3's `spacer.lmcpart` as the path-sourced
base):

```json
{
	"format": "lmc-asm",
	"version": 1,
	"units": "mm",
	"name": "mates_tour",
	"instances": [
		{"name": "base", "source": {"path": "spacer.lmcpart"},
		 "pose": {"translation": [0.0, 0.0, 0.0]}},
		{"name": "pin",
		 "source": {"part": {
			"format": "lmc-part", "version": 1, "units": "mm", "name": "pin",
			"created_with": "inline",
			"document": {
				"params": {},
				"features": [
					{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 10.0}],
					              "radius": {"Literal": 3.5}, "height": {"Literal": 20.0}}}
				],
				"root": 0, "suppressed": []
			}
		 }},
		 "pose": {"translation": [14.2, 9.5, 0.4]}},
		{"name": "cap",
		 "source": {"part": {
			"format": "lmc-part", "version": 1, "units": "mm", "name": "cap",
			"created_with": "inline",
			"document": {
				"params": {},
				"features": [
					{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 1.5}],
					         "size": [{"Literal": 12.0}, {"Literal": 12.0}, {"Literal": 3.0}]}}
				],
				"root": 0, "suppressed": []
			}
		 }},
		 "pose": {"translation": [15.0, 10.0, 14.2]}},
		{"name": "spare_pin", "suppressed": true,
		 "source": {"part": {
			"format": "lmc-part", "version": 1, "units": "mm", "name": "pin",
			"created_with": "inline",
			"document": {
				"params": {},
				"features": [
					{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 10.0}],
					              "radius": {"Literal": 3.5}, "height": {"Literal": 20.0}}}
				],
				"root": 0, "suppressed": []
			}
		 }},
		 "pose": {"translation": [60.0, 0.0, 0.0]}}
	],
	"mates": [
		{"Concentric": {
			"a": 0, "a_axis_point": [15.0, 10.0, 0.0], "a_axis_dir": [0.0, 0.0, 1.0],
			"b": 1, "b_axis_point": [0.0, 0.0, 0.0], "b_axis_dir": [0.0, 0.0, 1.0]}},
		{"Coincident": {
			"a": 1, "a_point": [0.0, 0.0, 0.0],
			"b": 0, "b_point": [15.0, 10.0, 0.0]}},
		{"Parallel": {
			"a": 2, "a_dir": [0.0, 0.0, 1.0],
			"b": 0, "b_dir": [0.0, 0.0, 1.0]}},
		{"Distance": {
			"a": 2, "a_point": [0.0, 0.0, 0.0],
			"b": 0, "b_point": [15.0, 10.0, 8.0], "distance": 12.0}}
	],
	"states": {
		"service": {
			"poses": [
				{"translation": [0.0, 0.0, 0.0]},
				{"translation": [15.0, 10.0, 30.0]},
				{"translation": [15.0, 10.0, 60.0]},
				{"translation": [60.0, 0.0, 0.0]}
			],
			"suppressed": [2]
		}
	}
}
```

Grammar facts (each exercised above or in §18.3):

- **Poses**: `{"translation": [x,y,z], "rotation": [qx,qy,qz,qw]}` — rotation
  omitted = identity; quaternions `[x, y, z, w]` (Rx(−90°) =
  `[-0.7071068, 0, 0, 0.7071068]`); rigid only — scale is refused on save
  (`BadPose`).
- **Mates** are externally-tagged PascalCase variants over instance indices,
  geometry in each instance's **local** frame, directions normalized
  defensively. The complete set (authoritative against
  `kernel_model::constraints::Constraint`): `Coincident {a, a_point, b,
  b_point}` (point-on-point), `Distance {…, distance}` (point separation ≥
  0), `Parallel {a, a_dir, b, b_dir}` (parallel *or* anti-parallel — the
  solver rotates toward the closer one, never forcing a 180° flip),
  `Concentric {a, a_axis_point, a_axis_dir, b, …}` (axes collinear).
  **Stored poses are only the solver's seed; the mates are the authority** —
  re-solved on every load.
- **States** are pose/suppression snapshots: one pose per instance
  (count-checked on save) plus optional `suppressed` indices.
  Instance-level `"suppressed": true` keeps the instance in the file but
  out of geometry, BOM and contacts.
- `suppressed` and `states` are omitted from saved files when empty.

### 18.2 The runner's report anatomy — every step, executed

`kernel-api asm mates_tour.lmcasm --out-dir out/` (flags: `--base-dir` for
part resolution, `--tol` 0.05 chord, `--voxel` 0.4 heal, `--window` 1.0
contact scan). Exit 0; the steps in order, with this run's measured
receipts:

| entry id | what it proves here |
|---|---|
| `load` | `instances: 4, mates: 4, states: ["service"], suppressed: [3]` — file parsed, parts rebuilt |
| `mates` | `residual: 1.34e-12` against the runner's 1e-6 gate — the deliberately-off seeds (pin at (14.2, 9.5, 0.4)) snapped to the bore axis; >1e-6 would FAIL the run (`assert_failed`). Also a `per_mate` residual list and the DOF block: `dof: {instances: 4, grounded_instances: 1, constraint_rows: 10, rank: 8, redundant_rows: 2, free_dof: 10, verdict: "under_constrained (10 free DOF)"}` — read `verdict`/`free_dof` the way you read the sketch solver's (§6): under-constrained is a *diagnosis*, not a failure |
| `bom` | 3 grouped flat lines (suppressed spare excluded): `cap ×1`, `pin ×1`, `spacer ×1` with `"params": "h=8"` — same-named parts at different parameters stay distinct lines (§18.4); since BOM v2 the file wraps them in the `bom/2` envelope with a `tree` view and `bom.csv` (§18.7) |
| `export:00:base` … | one world-posed STL per instance with the honest per-part route: all `"route": "exact"` here, 160/124/12 triangles; `export:03:spare_pin` reports `{"suppressed": true}` and writes nothing |
| `export:assembly` | merged mesh: `instances: 3, triangles: 296, watertight: true` |
| `export:assembly_step` | the runner also writes AP214 assembly STEP automatically: `parts: 3, bytes: 47650, skipped: []` → `mates_tour_assembly.step` (suppressed instances are skipped by name) |
| `contacts` | the §18.3 receipt below |
| `state:service` | the state applied + exported (`suppressed: 1` — the cap is out in this state), then assembled poses restored |

### 18.3 The contacts receipt — designed fits read back

```json
{"pairs": [{"a": "base", "b": "pin", "distance": 0.4975906014442444,
            "i": 0, "j": 1, "touching": false},
           {"a": "pin", "b": "cap", "distance": 0.0,
            "i": 1, "j": 2, "touching": true}],
 "tol": 0.05, "touching": 1, "window": 1.0}
```

B-rep parts are measured on their **raw exact tessellation** at `--tol`, so
the designed Ø7-in-Ø8 fit reads 0.4976 — sub-voxel fits are real
measurements, accurate to about the chord tolerance. The pin↔cap pair is the
*designed* face-on-face seat (cap mated 12 mm above the bore top = pin top):
`touching: true` is the designed-contact / interference *class* — the scan
cannot distinguish a perfect seat from an overlap. Distinguishing is the
exact route's job (`union` + `shells`, §10.3) or the design-intent layer's
(§18.6).

### 18.4 BOM grouping

`bom.json` groups by `(part name, parameter values)`: the §3.3 spacer's line
carries `"params": "h=8"`; instantiate the same recipe at h=10 elsewhere and
it becomes a second line. Suppressed instances are excluded (absent
material). The 37-instance gearbox groups to 20 lines (§18.6). Since BOM v2
the file is `{"schema": "bom/2", "flat": […], "tree": […]}` — `flat` is this
grouping (plus the optional engineering columns of §18.7), `tree` mirrors
the assembly structure, and a fixed-column `bom.csv` lands alongside.

### 18.5 Assembly receipts to gate on

`load.instances/mates/states`, `mates.residual` (the runner itself fails
above 1e-6), per-export `route` and `watertight`, `contacts.touching`. Note
(measured, §16.8): a non-watertight *instance export* does not fail the asm
run — the receipt says `"watertight": false` and the exit stays 0, unlike
program-level `export_stl` which fails (`invalid_geometry`) if even the heal
stays leaky. Your pipeline decides; the gearbox's `check_asm.py` is the
model of that caller-policy layer.

### 18.6 The joinery doctrine: contacts ≠ connections

The contact scan proves parts *touch*; it cannot prove they are *attached*.
Resting a stack of parts on each other passes every clearance check and
falls apart in your hand. The tri-benchmark
(`legacy/kernel-model-examples/tri_benchmark.rs`, kept out of the build since
2026-09) encodes the lesson — steal
its three moves:

- **Recess/spigot registration**: the B-rep base carries a 2 mm-deep recess
  that *seats* the damper with 0.3 mm radial clearance, and the cap's
  underside recess registers over the damper's top disc — parts locate each
  other instead of merely resting.
- **Real mating faces for lattices**: solid end discs unioned onto the
  gyroid puck give the lattice flat annular faces to clamp against — a raw
  TPMS edge is not a joint.
- **A fastener path**: a Ø9 clearance bore through the whole stack so one M8
  threaded rod clamps it — every "assembly" needs at least one answer to
  "what holds this together?".

Then *assert the design intent*: the tri-benchmark requires exactly 2
designed contacts; the gearbox allowlists all 52 designed contacts and fails
on any unexpected touch (`check_asm.py`), with `MUST_CLEAR` pairs (gear
flanks at the designed backlash) proven positive-distance. Joinery on the
parts, intent in the checks.

### 18.7 Nested assemblies and BOM v2 — sub-assemblies, `meta`, mass

An instance can source a whole assembly by path, to any depth — each file
resolves its own part sources against its own directory:

```json
{"name": "stack_in", "source": {"asm_path": "asm/shaft_input.lmcasm"},
 "pose": [1,0,0, 0,1,0, 0,0,1, -41,0,38]}
```

The worked artifact is the gearbox's nested variant
(`gearbox/gearbox_nested.lmcasm` regrouping the three shaft stacks; run by
`run_all.sh`). All receipts below are measured from that run.

**Semantics** — a sub-assembly solves its *own* mates at its own load, then
enters the parent as **one rigid unit** at the instance pose. Parent-level
mates and states address top-level instances only (v2 limit: no mating to a
sub-assembly's internal member, no un-suppressing members suppressed inside
the sub's own file). The reported `mates` residual is the max across all
levels (measured 1.27e-12). Leaf parts get hierarchical names everywhere —
contacts report pairs like `base ↔ stack_in/spacer9_0`, instance exports
write `parts/02_stack_in_shaft_in.stl` — and the nested gearbox is held to
the *identical* 52-designed-contact allowlist as the flat one (measured:
52/52, tightest must-clear gap 0.050 mm). Suppressing a sub-assembly
instance drops its entire branch from geometry, contacts and BOM. An include
cycle is refused loudly, naming the whole chain (provoked in
`tests/nesting.rs`):

```
sub-assembly cycle: '…/b.lmcasm' is already being loaded by this include
chain (b.lmcasm -> a.lmcasm -> b.lmcasm) — an assembly cannot contain
itself, directly or through intermediates
```

**Engineering metadata** — a `.lmcpart` optionally carries a `meta` block
(absent ⇒ byte-identical pre-v2 saves, fully backward compatible):

```json
"meta": {"part_number": "608ZZ",
         "material": {"name": "steel", "density_g_cm3": 7.85},
         "make_or_buy": "buy"}
```

`bom.json` (schema `bom/2`) then carries, per flat line, the grouping of
§18.4 plus `part_number`, `material`, `unit_mass_g`, `line_mass_g`,
`make_or_buy` and an honest `volume_source`: `"exact"` when the mass came
from the analytic B-rep volume, `"mesh"` when it had to come from the voxel
route. The measured bearing line:

```json
{"name": "bearing_608", "count": 6, "params": "", "part_number": "608ZZ",
 "material": {"name": "steel", "density_g_cm3": 7.85},
 "volume_source": "exact", "unit_mass_g": 18.126204213049718,
 "line_mass_g": 108.7572252782983, "make_or_buy": "buy"}
```

— `7.85·π·(11²−4²)·7/1000` to 1e-14 (pinned at 1e-9 in `check_asm.py`). Mass
honesty cuts both ways: the BOM prices the **model**, and the modelled
envelope ring deliberately overstates a real shielded 608ZZ (≈12 g) — if you
need datasheet mass, that is procurement data, not geometry.

The `tree` view mirrors the structure with rollup counts — measured: 15
top-level nodes, branches `stack_in: 8, stack_mid: 9, stack_out: 8`, tree
total **37 = flat total 37** (20 flat lines) — and `bom.csv` (fixed column
order, one header row) is the ERP hand-off. Both files are byte-identical
across independent runs (asserted in-tree and re-pinned by the gearbox
check).

**What BOM v2 is not** (scope, on purpose): no revisions/ECO/release states,
no suppliers or cost — part lifecycle lives in PLM-land (or your git
discipline), outside the kernel. Sub-assemblies are path-referenced only (no
inline assembly envelope yet).

## 19. The library — your own vocabulary, admission-gated

The catalog (§20) is the kernel's vocabulary; the **library** is yours. A
library is a plain directory — one stored `.lmcpart` per admitted entry
(file name `{name}-v{version}.lmcpart`) plus a byte-stable `index.json` — so
git-version it and every admission, deprecation and removal is auditable and
reversible. The five ops (`library_add`, `library_search`,
`library_instantiate`, `library_deprecate`, `library_remove`) run end-to-end
in §3.4; this section is the curation method and the two refusals you must
know.

### 19.1 What the admission gate buys you (and what it does not)

`library_add` builds the candidate at the declared **defaults**, sampled
**range corners** (all 2ⁿ min/max combinations, capped at 16 by a
deterministic spread that always keeps all-min, all-max and each
single-parameter extreme) and the **midpoint**; each sample must be a closed
manifold AND rebuild volume-bit-deterministically (two evaluations,
identical to the last bit). Per-sample measures land in the index as
evidence (§3.4 measured: `gate_samples: 10, gate_rebuilds: 2`). Honest
scope: the gate proves the *sampled points*, not every value between them —
declare ranges you actually intend, not the widest you can imagine.

### 19.2 A real rejection, executed

The §3.4 bushing re-declared with a careless `bore_r` range (max 12 ≥
outer_r min 8) — the gate finds the corner where the bore swallows the body:

```json
{"ops": [
	{"id": "admit", "op": "library_add", "dir": "vault",
	 "part": {
		"format": "lmc-part", "version": 1, "units": "mm", "name": "overdrilled",
		"created_with": "design-guide",
		"document": {
			"params": {"outer_r": 12.0, "bore_r": 4.0},
			"features": [
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "outer_r"}, "height": {"Literal": 10.0}}},
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "bore_r"}, "height": {"Literal": 200.0}}},
				{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
			],
			"root": 2, "suppressed": []
		}
	 },
	 "meta": {
		"name": "overdrilled", "version": 1,
		"provenance": {"author": "design-guide-ai", "date": "2026-06-11"},
		"params": [
			{"name": "outer_r", "units": "mm", "default": 12.0, "min": 8.0, "max": 16.0},
			{"name": "bore_r",  "units": "mm", "default": 4.0,  "min": 2.0, "max": 12.0}
		]
	 }}
]}
```

Executed, exit 1, kind `admission_rejected` — and nothing was admitted:

> `op 'admit': library_add: admission gate: sample corner_lh (bore_r=12,
> outer_r=8) failed to build: the rebuild produced an EMPTY solid — the
> dimensions degenerate at these values (e.g. a cut consumes the whole body)`

Fix the *declaration* (shrink `bore_r.max`), not the gate.

### 19.3 Dependents, deprecation, removal — the curation rules

One `(name, version)` is **immutable** once admitted; changed geometry is a
new version; unversioned instantiate takes the highest.
`library_instantiate` rejects unknown/out-of-range params loudly — the
declared interface is the contract. Retirement is two-stage, both executed:

1. **Deprecate** (§3.4): hidden from search, still builds, instantiate
   carries `"deprecated": true` + a warning string. Idempotent.
2. **Remove** refuses while any `.lmcasm` in the directory references the
   entry's stored file by path. Executed end-to-end: admit `bushing` v1 into
   `vault` (the §3.4 candidate, fresh directory — measured the same:
   `gate_samples: 10`, `volume_at_defaults: 3995.4498`):

   ```json
   {"ops": [
   	{"id": "admit", "op": "library_add", "dir": "vault",
   	 "part": {
   		"format": "lmc-part", "version": 1, "units": "mm", "name": "bushing",
   		"created_with": "design-guide",
   		"document": {
   			"params": {"outer_r": 12.0, "bore_r": 4.0, "h": 10.0},
   			"features": [
   				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
   				              "radius": {"Param": "outer_r"}, "height": {"Param": "h"}}},
   				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
   				              "radius": {"Param": "bore_r"}, "height": {"Literal": 200.0}}},
   				{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
   			],
   			"root": 2, "suppressed": []
   		}
   	 },
   	 "meta": {
   		"name": "bushing", "version": 1, "category": "spacers",
   		"tags": ["bearing", "sleeve"],
   		"description": "parametric plain bushing with a through bore",
   		"provenance": {"author": "design-guide-ai", "date": "2026-06-11"},
   		"params": [
   			{"name": "outer_r", "units": "mm", "default": 12.0, "min": 8.0, "max": 16.0},
   			{"name": "bore_r",  "units": "mm", "default": 4.0,  "min": 2.0, "max": 5.0},
   			{"name": "h",       "units": "mm", "default": 10.0, "min": 4.0, "max": 40.0}
   		]
   	 }}
   ]}
   ```

   Then drop this floor plan beside it as `vault/uses_bushing.lmcasm`:

   ```json
   {
   	"format": "lmc-asm",
   	"version": 1,
   	"units": "mm",
   	"name": "uses_bushing",
   	"instances": [
   		{"name": "sleeve", "source": {"path": "bushing-v1.lmcpart"},
   		 "pose": {"translation": [0.0, 0.0, 0.0]}}
   	],
   	"mates": []
   }
   ```

   ```json
   {"ops": [
   	{"id": "rm", "op": "library_remove", "dir": "vault", "name": "bushing"}
   ]}
   ```

   Executed, exit 1, kind `dependents_exist`:

   > `op 'rm': library_remove: refusing to remove 'bushing': referenced by
   > uses_bushing.lmcasm (pass force to remove anyway; git history keeps it
   > recoverable)`

   With `"force": true` (executed after the refusal): exit 0,
   `removed_files: ["bushing-v1.lmcpart"]` — git is your undo.

```json
{"ops": [
	{"id": "rm", "op": "library_remove", "dir": "vault", "name": "bushing", "force": true}
]}
```

Dates are **caller-supplied** (`meta.provenance.date`) — the kernel never
stamps clock time, so identical programs write identical bytes.

## 20. The standard-parts catalog

48 standard parts, 13 standard feature cuts, 7 hole-wizard cuts, 7
design-math lookups — built from published ISO/DIN/ANSI/vendor tables (cited
as `const` tables in `kernel-model/src/parts/`), with honest approximation
notes per op (threads are never modelled on catalog bodies — they are exact
assembly/clearance envelopes; display conventions documented per part).
Conventions: parts build at the origin along +Z — place with `pose`; bores
and shanks are **diameters**; a size outside its table is a loud
`invalid_param` naming the supported sizes (executed: `hex_bolt: M7 is not
in the ISO 4017 table (supported: M3, M4, M5, M6, M8, M10, M12, M16)`);
`bore_d` is accepted as an alias for `bore` on
`spur_gear`/`gt2_pulley`/`chain_sprocket` so the op and Document grammars
interchange. Per-op parameter tables: API.md "Standard parts catalog".
Deliberate scope limits: consumables, raw stock and made-to-order fasteners are
out of scope for the parametric catalog.

### 20.1 The full family table (48 parts) with key parameters

| family | op (key params) |
|---|---|
| fasteners (15) | `hex_bolt` (m, length) · `hex_nut` (m) · `washer` (m) · `socket_head_cap_screw` (m, length) · `flat_head_screw` (m, length) · `button_head_screw` (m, length) · `set_screw` (m, length) · `lock_nut` (m) · `threaded_rod` (m, length) · `standoff` (m, length) · `shoulder_bolt` (shoulder_d, shoulder_len) · `spring_washer` (m) · `dowel_pin` (d, length) · `circlip_external` (shaft_d) · `circlip_internal` (bore_d) |
| power transmission (13) | `spur_gear` (module, teeth, face_width, bore, keyway?) · `internal_gear` (module, teeth, face_width, rim_od) · `gear_rack` (module, length, width) · `gt2_pulley` (teeth, belt_width, bore, flanged?) · `chain_sprocket` (pitch, roller_d, teeth, bore) · `shaft` (d, length, keyway?) · `parallel_key` (b, h, l) · `jaw_coupling_hub` (od, bore) · `jaw_coupling_spider` (od) · `set_screw_coupling` (bore1, bore2) · `clamp_coupling` (bore1, bore2) · `lead_screw_tr8` (length, lead) · `lead_screw_nut_tr8` () |
| bearings & linear motion (10) | `deep_groove_bearing` (designation: 603/608/625/688/6000/6001/6804) · `flanged_bearing` (F608/F623) · `thrust_bearing` (51100/51101) · `kp08_pillow_block` () · `linear_bearing_lmuu` (bore: 8/12) · `sc8uu_block` () · `shaft_support_sk8` () · `shaft_support_shf8` () · `mgn12_rail` (length) · `mgn12_carriage` () |
| motors (2) | `nema_motor` (frame: 17/23, body_len) · `nema_mount_plate` (frame, thickness, margin) |
| sealing & fluid (4) | `o_ring` (dash) · `o_ring_cord` (ring_id, cord_d) · `pipe_boss_g` (designation G1/8…G1/2, wall, length) · `hose_barb` (hose_id, barbs) |
| springs & structure (4) | `compression_spring` (wire_d, outer_d, pitch, turns) · `extrusion_2020` (length) · `extrusion_3030` (length) · `tnut_2020` () |

(15 + 13 + 10 + 2 + 4 + 4 = 48.)

### 20.2 One executed instantiation per family

**Fasteners** — screw + nyloc + washer, with a stack-fit proof:

```json
{"ops": [
	{"id": "screw", "op": "socket_head_cap_screw", "m": 5, "length": 16},
	{"id": "nut", "op": "lock_nut", "m": 5},
	{"id": "wash", "op": "washer", "m": 5},
	{"id": "v_screw", "op": "exact_volume", "in": "screw"},
	{"id": "stack_nut", "op": "pose", "in": "nut", "translate": [0, 0, -21.0]},
	{"id": "fit_check", "op": "assert_disjoint", "a": "screw", "b": "stack_nut", "tol": 0.005}
]}
```

Executed: screw 561.54 mm³ (the drive socket is a real pocket); the nut
posed 21 mm below the head plane sits a measured 16.000 mm clear of the
screw tip — the gap you would close when threading it on. (Catalog bodies carry no
threads — clearance studies are exactly what they are for.)

**Gears** — internal ring + pinion at *designed backlash*, plus a rack:

```json
{"ops": [
	{"id": "pinion", "op": "spur_gear", "module": 2, "teeth": 12, "face_width": 10, "bore": 8},
	{"id": "ring", "op": "internal_gear", "module": 2, "teeth": 36, "face_width": 10, "rim_od": 84},
	{"id": "pinion_mounted", "op": "pose", "in": "pinion", "translate": [23.6, 0, 0.5]},
	{"id": "backlash", "op": "assert_disjoint", "a": "ring", "b": "pinion_mounted", "tol": 0.005, "min_clearance": 0.05},
	{"id": "rack", "op": "gear_rack", "module": 2, "length": 100, "width": 10},
	{"id": "v_rack", "op": "volume", "in": "rack"}
]}
```

Executed: flank gap **0.0834 mm** at centre distance 23.6 (conjugate C = 24
backed off 0.4 — internal meshes gain backlash as C *decreases*). At the
exact conjugate C the involutes touch within µm and the union welds — assert
*designed* clearances, not theoretical ones. Rack volume 5892.98.

**Bearings & seats** — the nominal line-to-line contract plus the ISO 286
allowance flow:

```json
{"ops": [
	{"id": "wall", "op": "box", "min": [0, 0, 0], "max": [40, 40, 12]},
	{"id": "seat", "op": "bearing_seat", "in": "wall", "at": [20, 20, 12], "axis": [0, 0, -1], "bearing": "608"},
	{"id": "fit", "op": "iso286_fit", "d": 22, "fit": "H7/p6"},
	{"id": "b608", "op": "deep_groove_bearing", "designation": "608"},
	{"id": "seated", "op": "pose", "in": "b608", "translate": [20, 20, 5]},
	{"id": "nominal_is_line_to_line", "op": "union", "a": "seat", "b": "seated"},
	{"id": "gate", "op": "assert", "in": "nominal_is_line_to_line", "shells": 1, "valid": true}
]}
```

Executed: the seat echo (pocket Ø22 × 7, shoulder Ø15) matches the bearing
body exactly — the union of the dropped-in bearing and the wall merges to
ONE shell because nominal-on-nominal is line-to-line *by design*
(press/slip allowance is the caller's). The `iso286_fit` receipt supplies
it: H7/p6 at Ø22 → `clearance: [-0.035, -0.001]` — an interference press
fit; apply the offset to your bore, not the catalog body.

**Couplings** — stepped rigid + slit clamp, genus as the fingerprint:

```json
{"ops": [
	{"id": "hub_a", "op": "jaw_coupling_hub", "od": 25, "bore": 8},
	{"id": "spider", "op": "jaw_coupling_spider", "od": 25},
	{"id": "rigid", "op": "set_screw_coupling", "bore1": 5, "bore2": 8},
	{"id": "topo_rigid", "op": "validate", "in": "rigid"},
	{"id": "clampc", "op": "clamp_coupling", "bore1": 8, "bore2": 10},
	{"id": "topo_clamp", "op": "validate", "in": "clampc"}
]}
```

Executed: rigid coupling genus 5 (through-bore + 4 set-screw cross-holes),
clamp coupling genus 4 — the catalog's documented topology contracts,
asserted as built. (The jaw pair + spider assemble with 2°/0.2 mm designed
play — proven by a kernel test, ready for your `.lmcasm`.)

**Linear motion** — leadscrew + nut + rail + carriage + round bearing:

```json
{"ops": [
	{"id": "screw", "op": "lead_screw_tr8", "length": 120, "lead": 8},
	{"id": "nut", "op": "lead_screw_nut_tr8"},
	{"id": "rail", "op": "mgn12_rail", "length": 75},
	{"id": "topo_rail", "op": "validate", "in": "rail"},
	{"id": "car", "op": "mgn12_carriage"},
	{"id": "lm8", "op": "linear_bearing_lmuu", "bore": 8},
	{"id": "v_lm8", "op": "exact_volume", "in": "lm8"}
]}
```

Executed: the 75 mm rail carries genus 3 — three countersunk mounting holes
on the catalog 25 mm pitch (genus = hole count is the rail's contract);
LM8UU 2999.34 mm³ with its two retaining-ring grooves.

**Springs & structure** — V-slot stock + tee nut + spring:

```json
{"ops": [
	{"id": "rail", "op": "extrusion_2020", "length": 100},
	{"id": "v", "op": "volume", "in": "rail"},
	{"id": "tnut", "op": "tnut_2020"},
	{"id": "spring", "op": "compression_spring", "wire_d": 2, "outer_d": 16, "pitch": 6, "turns": 5},
	{"id": "topo_spring", "op": "validate", "in": "spring"}
]}
```

Executed: 100 mm of 2020 measures 17818.5 mm³ → 0.481 kg/m in aluminium —
on the published ~0.48 figure (the honesty note: simplified sharp-corner
profile, dimensionally correct); the helical spring is genus 0 and validates
(refused if coils would touch: `pitch ≤ wire_d`).

**Sealing & fluid**:

```json
{"ops": [
	{"id": "seal", "op": "o_ring", "dash": 214},
	{"id": "xv_seal", "op": "exact_volume", "in": "seal"},
	{"id": "boss", "op": "pipe_boss_g", "designation": "G1/4", "wall": 2.5, "length": 12},
	{"id": "topo_boss", "op": "validate", "in": "boss"},
	{"id": "barb", "op": "hose_barb", "hose_id": 6, "barbs": 3},
	{"id": "topo_barb", "op": "validate", "in": "barb"}
]}
```

Executed: AS568-214 measures 876.877 mm³ — the exact analytic torus
(2π²·R·r² of the dash dimensions); G1/4 boss and 3-barb stem both genus 1
(tap-drill convention — the helix is yours to tap).

**Motors & boards** — motor + plate, clearance-proven:

```json
{"ops": [
	{"id": "motor", "op": "nema_motor", "frame": 17, "body_len": 40},
	{"id": "plate", "op": "nema_mount_plate", "frame": 17, "thickness": 5, "margin": 4},
	{"id": "plate_up", "op": "pose", "in": "plate", "translate": [0, 0, 0.1]},
	{"id": "clears", "op": "assert_disjoint", "a": "motor", "b": "plate_up", "tol": 0.005},
	{"id": "v_plate", "op": "exact_volume", "in": "plate"}
]}
```

Executed: the plate lifted 0.1 mm off the faceplate clears the motor by
0.0975 mm — pilot Ø22.2 vs pilot boss Ø22 leaves the designed 0.1 radial;
plate exact_volume 10533.49 with all five bores π-exact.

### 20.3 The 13 standard feature cuts

Catalog-driven features machined into a prior solid (`at` + `axis` placed;
grooves use the lathe-style ring cutter; host should be the standard's
nominal Ø — each op documents its clearance envelope):
`heatset_insert_boss`, `circlip_groove_external`, `circlip_groove_internal`,
`o_ring_groove`, `o_ring_face_gland`, `o_ring_face_gland_racetrack`,
`nema_mount_cut`, `servo_pocket`, `tr8_nut_trap`, `pc4_port`,
`teardrop_hole`, `board_mount`, `bridged_counterbore`. Parameter tables and
a runnable example each: API.md "Standard feature cuts". Three executed
here — the print-oriented trio on one lid (§22 uses them as method):

```json
{"ops": [
	{"id": "lid", "op": "box", "min": [0, 0, 0], "max": [60, 30, 10]},
	{"id": "boss", "op": "heatset_insert_boss", "in": "lid", "at": [15, 15, 10], "axis": [0, 0, 1], "m": 3},
	{"id": "axle", "op": "teardrop_hole", "in": "boss", "at": [38, 30, 5], "axis": [0, -1, 0], "up": [0, 0, 1], "d": 4, "through": 30},
	{"id": "cb", "op": "bridged_counterbore", "in": "axle", "at": [50, 15, 10], "axis": [0, 0, -1], "m": 5, "through": 10, "bridge": 0.3},
	{"id": "gate", "op": "assert", "in": "cb", "genus": 1, "valid": true},
	{"id": "v", "op": "volume", "in": "cb"}
]}
```

Executed: genus 1 — read it as the printability story: the heat-set boss
adds a blind pocket (no tunnel), the teardrop axle is the one tunnel
(45°-crowned so it prints unsupported), and the bridged counterbore is
*deliberately not through* — a 0.3 mm membrane seals the clearance bore for
support-free printing; you drill it out after (the kernel test asserts genus
0 vs the wizard's genus 1 for exactly this reason). Volume 17403.55.
Placement gotchas worth knowing before you reach for the other ten:
`board_mount`'s corner-anchored rpi/arduino patterns mirror in y when cut
from a top face (−Z axis maps pattern x/y to world (X, −Y) — documented
face-frame caveat); gland cuts echo their full Parker-derived dimension set
(§20.4).

### 20.4 The 7 design-math lookups — all executed

Pure cited-table/closed-form calculations; they bind no geometry (a
reference to one is `missing_ref`), the numbers come back as measures:

```json
{"ops": [
	{"id": "fit", "op": "iso286_fit", "d": 8, "fit": "H7/g6"},
	{"id": "belt", "op": "gt2_belt", "center_distance": 100, "t1": 20, "t2": 36},
	{"id": "c_back", "op": "gt2_center_distance", "belt_teeth": 158, "t1": 20, "t2": 36},
	{"id": "m4", "op": "heatset_spec", "m": 4},
	{"id": "gland", "op": "metric_cord_gland", "cord_d": 2},
	{"id": "cord", "op": "racetrack_cord_length", "x_len": 100, "y_len": 60, "corner_r": 8},
	{"id": "g14", "op": "pipe_thread_g", "designation": "G1/4"}
]}
```

Executed, the receipts (each a design number you would otherwise compute by
hand):

| id | receipt |
|---|---|
| `fit` | Ø8 H7/g6 → hole [0, +0.015], shaft [−0.014, −0.005], clearance [0.005, 0.029] — a sliding fit |
| `belt` | C=100, 20T/36T → pitch_length 256.259, commercial belt = 128 teeth |
| `c_back` | 158T belt on the same pulleys runs taut at C = 129.900 |
| `m4` | M4 insert → pilot Ø5.6, pocket 9.1 deep, boss Ø11.2 (cut flush pockets with `drill` when you don't want the boss) |
| `gland` | Ø2 cord → groove 1.5 deep × 2.7925 wide (25% squeeze / 75% fill — Parker static mid-band, metric rows derived & documented) |
| `cord` | 100×60 r8 racetrack → cut 306.265 mm of cord (+ vendor compression allowance) |
| `g14` | G1/4 → major Ø13.157, 19 TPI, pitch 1.337, tap drill Ø11.8 |

The supported fits for `iso286_fit`: H7/g6, H7/h6, H7/k6, H7/n6, H7/p6,
H7/s6, H8/f7 (hole-basis preferred set, d ≤ 120; negative clearance =
interference, used in §20.2's bearing flow).

---
# Part V — Shipping & survival

## 21. Exports and interop

### 21.1 The three export ops, executed

`file` paths join `--out-dir` (absolute passes through); parents are
created; the report's `file` is the path actually written. One revolved knob
through all three:

```json
{"ops": [
	{"id": "knob", "op": "revolve",
	 "profile": [[0,0], [9,0], [9,3], [4,3], [4,14], [7,17], [5.5,20], [0,20]],
	 "segments": 64},
	{"id": "coarse", "op": "export_stl", "in": "knob", "file": "knob_coarse.stl", "tol": 0.1},
	{"id": "fine", "op": "export_stl", "in": "knob", "file": "knob_fine.stl", "tol": 0.005},
	{"id": "as3mf", "op": "export_3mf", "in": "knob", "file": "knob.3mf"},
	{"id": "asstep", "op": "export_step", "in": "knob", "file": "knob.step"}
]}
```

Executed: `tol` drives the adaptive tessellation — 768 triangles at 0.1 mm
chord vs 2560 at 0.005; the 3MF carries the same exact-route mesh; the STEP
is **not a mesh** — grepping the written file counts 128 CYLINDRICAL_SURFACE
+ 128 CONICAL_SURFACE + 192 PLANE entities (the revolve's exact per-edge
surfaces; counts are per-sector entity instances).

What each format preserves:

| op | format | preserves | loses |
|---|---|---|---|
| `export_stl` | binary STL | watertight facets at `tol` | everything else — no units, no curves, no names |
| `export_3mf` | 3MF (zip/XML) | same mesh, mm units explicit | analytic surfaces |
| `export_step` | STEP **AP203** | exact analytic surfaces (plane/cylinder/sphere/cone/torus), circular edges as CIRCLE, product name = file stem | nothing geometric on tagged solids; faceted faces of untagged regions export as planar patches |

### 21.2 Watertight gating — programs vs assemblies (different, by design)

- **Program exports** (`export_stl`/`export_3mf`): exact tessellation at
  `tol`; watertight → ships `"route": "exact"`; leaky → healed through the
  winding-number SDF at `voxel` (default 0.3) → `"route": "voxel_healed"`;
  *still* leaky → the op FAILS (`invalid_geometry`) — a program never writes
  a garbage file.
- **Assembly instance exports**: never fail the run for leakiness — the
  receipt carries `"watertight": false` and exit stays 0 (measured: §16.8's
  damper at `--voxel 0.4`). Rationale: a 37-part run should not die on one
  marginal lattice; the per-instance receipt is the gate, and your policy
  layer (§18.5) decides.

`export_step` does not tessellate and has no routing — it writes the B-rep
as-is.

### 21.3 STEP scope, AP242, and the Rust-only formats — honest inventory

Reachable from JSON programs: STL, 3MF, STEP AP203 (above). The wider
mesh/B-rep I/O lives on the **Rust surface** (no JSON op yet — listed here
so you know where the capability boundary runs; everything below is
API-documented in the crates):

- `kernel_core::Mesh`: `write_obj`, `write_glb` (glTF binary), `write_3mf`,
  `write_stl_binary`; readers for STL/OBJ/PLY/3MF (PLY is read-only — there
  is no PLY writer).
- `kernel_brep::export_step_ap242`: the AP242 edition-1 envelope of the same
  exact-surface writer (`export_step` = AP203 with the complete
  product structure). Honest scope per the module docs: geometry +
  product identity — PMI/GD&T and the AP242-specific capability modules are
  absent, which a conforming AP242 reader treats as simply not present.
- STEP **import** (`kernel_brep::step_import`) round-trips this kernel's own
  exports exactly (incl. trimmed-NURBS faces, Wave-5); the third-party
  exporter corpus is an open frontier (§24, item 9).
- Assembly STEP export exists Rust-side (Wave-5 `assembly export`); the
  `.lmcasm` runner currently writes STL only.

If you need OBJ/glTF from a program today: export STL and convert, or call
the Rust API. Do not guess at unlisted ops — the §23 `unknown_op` error
names the live count (161 today) and points at `describe`.

## 22. Print-readiness method

The shipped FDM artifact (`iphone_stand/` — stand.json, receipt.json,
committed STL/3MF and renders) is the worked example of designing FOR the
printer with receipts; its `tools/` directory is the model audit kit. The
method it encodes, distilled:

1. **Design printable features in, from the catalog**: `teardrop_hole` for
   horizontal bores (45° crown, no supports — the lower 270° keeps the exact
   nominal circle so pins still locate), `bridged_counterbore` for
   downward-facing pockets (sacrificial membrane, drill after — genus
   receipt proves the membrane: §20.3, executed), `heatset_insert_boss`
   with the correctly *undersized* Ruthex pocket (`heatset_spec` when you
   need the numbers without the boss). All three executed in §20.3 on one
   lid.
2. **Pick the voxel from the §17.4 table** (FDM 0.3–0.4 mm at 0.2 mm
   layers), then **drive the watertight receipt green** (§16.8: false at
   0.4 → true at 0.25 — for lattices the receipt, not the table, has the
   final word).
3. **Wall-thickness gate**: `wall_thickness` with `flag_below` = your
   minimum printable wall; judge `thin_area` + `p05_thickness` (§10.1's
   doctrine). The stand's lip twin measured `thin_area 0.0` at flag 1.6
   before shipping.
4. **Overhang audit on the exported STL**: a triangle whose outward normal
   has nz < −cos 45° above the bed plane is a violation; gate on violating
   *area* calibrated against a negative control. That is
   `iphone_stand/tools/overhang_audit.py` (~60 lines, STL-in, exit-coded) —
   copy it; it aborts on inside-out meshes by checking the signed volume
   first. In-engine, the same audit is the `support_report` op — but
   `describe` ships **empty `doc` strings** for its two parameters, and both
   conventions are counter-intuitive enough that four campaigns wrote wrong
   orientation prose (one shipped a render of the wrong bed). Measured
   2026-08-08 and authoritative:
   - **`build_dir` points AWAY from the bed** (it is the layer-growth
     direction), so `[0,0,1]` puts the bed at min-Z. Verified on an L-bracket:
     `bed_area 50.0` (the foot) at `[0,0,1]`, `bed_area 200.0` (the two top
     faces) at `[0,0,-1]`. `render_sheet`'s `build_dir` is the same
     convention — if the bed view looks upside-down, the claim is backwards.
   - **A LARGER `overhang_deg` is MORE permissive** (default 45). A downward
     face is counted in `steep_area` iff its tilt from `build_dir` *exceeds*
     `overhang_deg`; verified on lofted frusta (a 63.435° wall is steep at 63,
     clean at 64; a 45° wall is steep at 44, clean at 45). A "second, stricter
     reading" is a **smaller** number. The comparison is strict on f32, so
     never set `overhang_deg` to a modelled face angle.
   - `steep_area == 0.0` is the support gate; a horizontal underside is
     `bridge_area`, not `steep_area`, so `support_free: true` does **not**
     mean "no bridging" — always quote `max_bridge_span` beside it, and note
     that it is the *short way across* the bridging region (an under-read for
     a cantilever, which is not a bridge).
   Full measured tables: `campaign/digests/ops_core.md` §11a.
5. **Datum-plane probe for anything blended**: §14's pillow check (bbox
   z_min == bed plane) — the stand shipped only after a standalone bed-planarity
   assertion, because receipts alone smiled through a 0.46 mm pillow.
6. **Fits**: FDM at 0.4 mm nozzle wants ~0.2–0.3 mm designed radial
   clearance on push fits (the gearbox prints at 0.2–0.25 design gaps);
   resin holds the §17.4 fine band — and *measure* every fit you care about
   with `assert_disjoint`/`contacts` instead of trusting the slicer.

## 23. Failure playbook

Execution stops at the first failing op; the error has a machine-matchable
`kind` and a message naming the op, parameter and values. Every row below
except `internal` (a caught panic — not honestly provocable) was **provoked
deliberately against the current binary and its message captured verbatim**
during the writing of this guide.

| kind | meaning | typical fix |
|---|---|---|
| `parse` | file not JSON / not `{"ops": […]}`, reported on id `$program` — *"program is not valid JSON: expected ident at line 1 column 2"* | fix the envelope |
| `unknown_op` | `op` names nothing — *"unknown op 'cube' — not one of the 161 supported ops; call the `describe` op to enumerate them"* | check spelling (`box`, not `cube`); `describe` lists the catalogue |
| `duplicate_id` | *"this id was already used by an earlier op — ids must be unique"* | ids unique per program |
| `missing_ref` | *"no result named 'nope' — it must be the id of an earlier geometry-producing op"*; also fired when referencing a measure/export op (*"binds no geometry"*) | reference a binding op; remember `implicit`, measures, exports, design-math bind nothing |
| `wrong_type` | *"'sq' is a sketch, expected a solid"* | route sketches through `sketch_extrude`/`sketch_revolve` first |
| `invalid_param` | missing/malformed required param; degenerate input (§5.3 table); **empty boolean result** (*"intersection produced an empty solid — … e.g. a disjoint intersection"*); out-of-table size (names the table, §20); non-loadable `.lmcpart` (incl. the voxel-half refusal, §16.7); every `implicit`-tree parse error with its JSON path (§11–12); pattern/lattice caps (§17.5) | the message tells you which; for empty booleans check your poses — for proofs of emptiness use `assert_disjoint` / `assert shells` |
| `feature_failed` | fillet/chamfer witness or scope: *"witness [200, 0, 10] matched no edge — nearest edge is 170.000 mm away (limit 3.742; pass max_distance to widen)"*; *"the edge near the witness is outside the supported scope — supported: CONVEX straight edges between two planar faces (any convex dihedral angle…) via fillet_edge_near/chamfer_edge_near, and convex circular rims via fillet_circular_rim. Concave junctions (inside corners…) are out of scope for BOTH"* (re-measured 2026-07-09 — the message now names the real constraint, convexity, and suggests the explicit quarter-round cove workaround); radius does not fit | use the right fillet op for the edge kind (§8); move the witness onto the edge; shrink the radius |
| `sketch_failed` | *"constraints did not converge (residual 3.000e2 after 4 iterations, state over_constrained) — they are conflicting or inconsistent"*; open/degenerate profiles | find the conflicting dimensions (§6.2); close the loop |
| `invalid_geometry` | op ran, result failed `validate()` — e.g. CW extrude: *"extrude failed validate(): closed=false manifold=false … refusing to bind an invalid solid"*; an implicit tree that stayed leaky: *"the implicit tree did not mesh watertight at voxel 0.5 (triangles=0, …)"* (also the all-pruned Lipschitz case, §12.3); an export leaky even after healing | wind profiles CCW; refine `voxel` / switch mesher (§11.5, §17.4); bury tangent contacts (§11.2) |
| `admission_rejected` | library gate: *"sample corner_lh (bore_r=12, outer_r=8) failed to build: the rebuild produced an EMPTY solid — the dimensions degenerate at these values (e.g. a cut consumes the whole body)"* | shrink the declared ranges to what actually builds (§19.2) |
| `dependents_exist` | `library_remove` refused: *"refusing to remove 'bushing': referenced by uses_bushing.lmcasm (pass force to remove anyway; git history keeps it recoverable)"* | deprecate instead, or `"force": true` under version control (§19.3) |
| `assert_failed` | *"assert failed: genus: measured 1, expected 2"*; *"assert_disjoint failed: surface distance 0 mm ≤ required clearance 0 mm — 'seat' and 'dropped' touch or interfere"*; also the asm mate-residual gate (§18.2) | the design does not meet the declared intent: fix the geometry or fix a wrong expectation, never delete the gate |
| `io` | *"cannot read 'work/does_not_exist.lmcpart': No such file or directory (os error 2)"*; unwritable export path | check paths; `load_part` resolves relative to the program file |
| `internal` | a caught kernel panic | treat as a kernel bug; report it with the program |

Beyond the kinds, the **silent** failure modes this guide measured — the
ones no exit code flags — and their tripwires:

| silent mode | tripwire |
|---|---|
| misspelled optional param → default in effect (§5.2) | `exact_volume_within` / assert any measure the param drives |
| negative sphere radius → absolutized (§5.3) | volume window |
| hole loop crossing outer → valid topology, wrong volume (§5.3) | volume window |
| rim-fillet witness snapping to a far qualifying rim (§8.2) | volume/measure after the fillet |
| under-declared Lipschitz bound → possible silent holes (§12.3) | declare honestly; manifold mesher; volume window |
| blend pillow on parallel surfaces — receipts green (§14) | datum probe (bbox/z_min), volume window; fuse-first-cut-last |
| `touching: true` cannot distinguish seat from overlap (§18.3) | `union` + `assert shells` on the pair |
| hole wizard cuts with zero edge-proximity awareness — a countersink tangent to a wall or 0.25 mm from a feature raises nothing (cold-start audit) | `wall_thickness` gate + a volume window after every wizard cut |
| edge features after booleans: first `chamfer_edge_near` works, the next fails on fragmented planes (§8.4) | ease primitives first, boolean last; treat the loud `feature_failed` as the design-order signal |
| valid B-rep, leaky default tessellation — booleans fighting a revolve's facet grid (§7.7; RESPOOL 2026-07-28) | `try_*_sealed` at the op, `ChainLog::seal()` on the chain, `boolean_hazards` before it |
| vertex-only mesh measurements — a cuboid has no mid-height vertices, so a z-window vertex scan reads the wrong silhouette (RESPOOL rib-envelope gate) | `Mesh::radial_extent` (clips triangles to the band; exact max, interior-foot-aware min) |

## 24. Limits — the current ledger

The engine is graded 9.5/10 on the falsifiable ladder in
[`docs/BAR.md`](docs/BAR.md) (677/0 tests, fuzz 100% at N=2000 with floors at
98, five product acceptances green). These are the *known, documented* edges
— the enhancement ledger, not hidden failure modes. Design with them in
mind:

1. **NURBS through booleans/fillets — one slice open (2026-07-30).**
   Trimmed B-spline faces tessellate, round-trip STEP exactly, and export.
   A freeform face can now be a boolean operand for **planar half-space cuts
   only** (`try_freeform_boolean`, difference/intersection): exact-surface,
   tolerance-curve routing — the patch stays exact (control net bit-identical
   in both halves), the intersection curve is chord-refined to a stated
   tolerance (default 1e-4 × bbox diagonal, carried in the result), and the
   solid is gated watertight or withheld. Volume tracks an independent
   exact-patch oracle to ~0.1% (the operand's own tessellation error); the two
   halves re-sum to the operand at 1e-5, converging to 1.1e-7 with refinement.
   Everything else — quadric tools, solid tools, freeform∩freeform, multi-patch
   operands, unions — refuses BY NAME. Fillets on freeform faces remain
   unsupported. Concrete blocker for the quadric drill: a cylinder produces a
   closed ISLAND crossing in the patch chart, needing inner-ring chart
   trimming (the plane case is guaranteed boundary-to-boundary).
2. **FRICTION #19 — housing-tessellation leak.** The exact adaptive
   tessellation of one 64-feature gearbox housing goes leaky (post-Wave-5
   triangulator), so its STL ships `voxel_healed` (measured in §3.5's run:
   971,736 triangles, watertight) and the mesh-distance contact scan reads 4
   phantom touching pairs — each re-proven disjoint by exact booleans every
   run (`check_artifacts.json`). The B-rep itself is valid; the leak is
   mesh-side. Pattern to copy: when a contact receipt surprises you,
   arbitrate with the exact route (§10.3).
3. **Mirror-symmetric hole-row stitcher degeneracy.** When two hole rows
   land exactly equidistant from a panel's edges, the B-rep stays perfectly
   valid but BOTH tessellation routes crack at the symmetric stitch; a
   1.5 mm datum shift heals it, and the hybrid router still delivers a
   watertight voxel mesh (pinned honestly in
   `kernel-model/src/parts/boards.rs`). If a `board_mount`-style pattern
   reports a leaky exact route on a symmetric panel, nudge the datum or
   accept the heal.
4. **Fuzz corpus: 10,000/10,000 as of 2026-07-30** (was 99.98%). Both
   residual seeds were root-caused and fixed, and the ledger's own hypothesis
   about them (sagitta-scale seam interleaving) was DISPROVED in the process:
   the real causes were a degenerate-edge guard comparing a *squared* length
   against a plain epsilon (exempting sub-3.2e-5 mm edges from T-junction
   healing) and a seam-corner duplicate pair sitting 1.051e-7 mm apart — 5%
   outside the weld ball, hence unstitchable by construction. Both seeds now
   replay in the DEFAULT suite and the Level-9 floor is an exact
   10,000/10,000 pin ([`docs/ROBUSTNESS.md`](docs/ROBUSTNESS.md)). The
   practical reading is unchanged and still load-bearing: every solid op
   gates on `validate()` and fails loudly rather than binding garbage.
5. **Blend pillowing is physics, not a bug — but it is a permanent design
   hazard.** `fillet_union`/`smooth_union` add material wherever operand
   surfaces run parallel within the blend radius, including buried faces and
   datum planes, with green receipts (measured −0.4428 mm bed-plane bulge,
   §14; first caught in the field by `iphone_stand` v4.1's probes, two
   incarnations). Operating rule lives in §14/§17.3: fuse first, cut last,
   probe datums.
6. **Validity ≠ geometric truth at the input boundary.** The §5.3 measured
   traps (negative radius absolutized; out-of-contract hole loops binding
   wrong-volume solids; concave tapers passing at shallow draft) — the gate
   guarantees closed/manifold, not that your input meant what you meant.
   Assert measures.
7. **Scan classes, not certainties.** `contacts.touching` is a class
   (designed contact OR interference, §18.3); `wall_thickness.min_thickness`
   is corner noise (§10.1); `assert_disjoint` is accurate to `tol` (§10.3).
   Each has its exact-route or statistical sibling — use it.
8. **Simulation bridge (I8): BUILT at the tools/campaign level; JSON
   load-case ops still absent.** Six benchmark-gated solvers live in the
   registry `tools/solvers/` — structural voxel FEA + SIMP, heat conduction
   (steady + transient), modal, linear buckling, nonlinear/contact
   (corotational beam), and fatigue — each with a card stating its measured
   benchmark error and validity limits (§25.7). The loop closes both ways:
   `tools/analyzers/stress_to_density.py` + `kernel_implicit::grid_field` turn a real
   stress field into graded geometry, and `kernel_model::loads` turns an
   assembly load path into per-part FEA boundary conditions. What is still
   absent: FEA/CFD **ops on the JSON surface** (a load case cannot be
   declared in a program), and any flow solver at all — CFD is a declared
   gap, not a capability. On the JSON surface the analysis vocabulary
   remains mass properties, wall thickness, draft and clearance receipts
   (torus second moments are tessellation-level; cylinder/sphere/cone are
   analytic).
9. **Domains out of scope**: sheet metal / casting / CNC **process
   profiles** (declared siblings that refuse loudly — casting's draft and
   undercut half exists as `draft_analysis`); PLY write, OBJ/glTF from
   JSON programs, `.lmcasm` STEP export (§21.3). Dimensioned 2-D **drawings
   are no longer out of scope** — `kernel_model::drawing` ships orthographic
   and section views with sampled hidden-line removal, dimensions that each
   trace to a named model measure, and deterministic SVG + DXF R12 output;
   what it does NOT do is GD&T, surface finish, thread callouts, auxiliary
   views, or standards-compliant automatic dimension placement. STEP frontier beyond the
   exact round-trip: sphere poles/periodic regions, assemblies on the JSON
   surface, and the third-party exporter import corpus. Numerical contracts you
   inherit from [`docs/NUMERICS.md`](docs/NUMERICS.md): determinism is
   per-platform (not cross-libm), the GPU path is tolerance-equivalent but
   never bit-authoritative, and `expr_sdf` honesty is yours (§12.3).
10. **Coplanar-overlap forests** (§7.4): supported and passing on the
    canonical cases, deterministically — but the least-margin corner of the
    arrangement; prefer small embedments where you control the dimensions.
11. **Validity does not imply CONNECTEDNESS** (found 2026-07-31, the hardest
    silent-wrongness class in the ledger). A part severed into two floating
    lumps is **valid** (each lump is a closed orientable solid), **watertight**
    (every edge used twice), and **plausibly measured** (`volume` sums both
    lumps) — and `Solid::shell_count()` reports **1**, because it counts B-rep
    shell RECORDS, not connected geometry. In the field this passed the support
    audit, keep-out and insertion sweeps, stress sections, STEP round-trip, and
    rendered convincingly; a human caught it by noticing a gap in a top view.
    The oracle is `Mesh::component_count` / `Mesh::is_one_body`
    (`kernel-core/tests/connectivity.rs` pins that it disagrees with the other
    two exactly when they are blind). **Gate it on every campaign**, and treat
    any tapering cutter as the hazard: its apex must stay strictly INSIDE the
    material, never run out through a face.

When a capability is partial, the engine's contract — and this guide's — is
to say so in the receipt rather than degrade silently. Read the receipts.

---

## 25. Campaign cookbook — gate-driven Rust examples

The shipped print campaigns (DOVESTACK `drawer_system.rs`, POOLDOCK
`pool_*.rs`, RESPOOL `respool.rs` — all parked, uncompiled, in
`legacy/kernel-model-examples/` since 2026-09; its README says how to restore
one)
share one architecture that this guide's JSON surface does not cover: a flat
Rust `main` that builds parts with the `kernel_brep` API, then **re-proves
every claim on every run** and exits non-zero on any FAIL. If you are
authoring a new physical part, copy this shape. (The examples are the
executed references for the architecture; the newer helpers named below —
`boolean_hazards`, `ChainLog`, `try_*_sealed`, `sweep_check`,
`radial_extent` — postdate them and carry their own executed tests in
`kernel-brep/tests/{hazards_linter,chain_and_sealed,mesh_measures}.rs` and
`kernel-model/tests/sweep_check.rs`.)

1. **Consts first.** Every dimension is a named const with the WHY in a
   comment; derived values are expressions of other consts, never re-typed
   literals — one edit propagates or the build fails loudly.
2. **A gate accumulator.** `let mut ok = true;` and a `gate(label, pass,
   detail, &mut ok)` printer: each check prints one table row with its
   measured value and `OK`/`<<< FAIL`; the last line is the verdict and
   `process::exit(if ok {0} else {1})`.
3. **Per-part emit.** validate → **connectivity** (`Mesh::is_one_body`) →
   pose to print orientation → drop to z=0 → `support_free_report(Z, 45.0,
   0.3)` (gate `steep_area < 1e-6`, bound `max_bridge_span`; the report's
   `steep_exemplars`/`bridge_patches` name WHERE) → watertight → bed-fit →
   write STL/3MF. Connectivity is a **third oracle**, independent of the other
   two: a part cut into two floating lumps is valid, watertight, plausibly
   measured, and passes every downstream gate — see §24 item 11.
4. **A negative control per oracle.** Audit a part in a deliberately wrong
   orientation and assert the support gate FIRES; drop a mating part at a
   wrong angle and assert the interference oracle fires. A gate that cannot
   fail is not a gate.
5. **Kinematics on posed pairs.** `kernel_model::sweep_check` for dense
   insertion/twist paths (mesh distance + sampled penetration), then an
   exact `overlap_volume` gate at every load-bearing pose (seat, lock,
   retention-proof with an INTENTIONAL overlap asserted positive). Lift
   contact poses ~0.05–0.1: exact-contact booleans are §7.4 territory.
6. **Fit gauges, not hopes.** Model the counterpart (a rail tube, a refill
   core at nominal AND worst-case) and gate clearance/bite numerically; add
   printable coupons so the user can verify the two critical fits in
   minutes before a long print.
7. **Arithmetic load cases, honestly labeled.** Closed-form stress checks
   against derated allowables (state the derating chain), a hot tier when
   heat is in play, out-of-scope failure modes listed by name — and
   `kernel_model::materials` for densities. FEA (tools/analyzers/ace_fea_runner.py)
   sharpens, never replaces, the closed-form. The analysis SET itself is a
   research output: the research pass freezes a per-artifact **analysis
   plan** (governing physics + failure modes → required analyses), and
   ANALYSIS.md must answer every item one of three ways — receipts from an
   existing solver; a NEW solver written first and proven against
   closed-form benchmarks + a convergence check (a written solver is
   guilty until its own gates are green — the ACE FEA/SIMP pair is the
   precedent); or **"required, NOT performed"** in bold with the reason.
   Silence about a required analysis is the one forbidden outcome.
8. **Boolean hygiene per §7.7** — `boolean_hazards` while authoring,
   `ChainLog::seal()` when a chain grows past ~10 ops.
9. **Ship receipts.** The example writes its own outputs (the standard
   folder layout of step 1,
   ASSEMBLY.*, an analysis doc generated FROM the live numbers) so nothing
   quotable can go stale; `tools/publish/assembly_doc.py` renders the sheet.

### 25.1 The implicit toolbox — campaign-facing Rust APIs (2026-07-29 wave)

Six capabilities campaigns call directly (Rust level; none are JSON ops
yet). Every row is pinned by the named test — quote the test, not this
table, if they ever disagree.

| capability | API | contract & pinned proof |
|---|---|---|
| strut lattices | `kernel_implicit::strut::{StrutLattice, StrutKind::{Bcc,Fcc,Octet}, graph_lattice, pipe_path}` | exactly 1-Lipschitz min-capsule field; periodic 27-image tiling with in-module equality proof (seam jump ≤ 1.2e-3, tiled meshes closed); solid fractions 19.8/22.1/39.3% @ cell 10, r 1 (`kernel-implicit/tests/strut.rs`) |
| simulation fields | `kernel_implicit::grid_field::GridField::{from_npy_file, normalized, into_grade_law, into_scalar_field}` | NPY v1–v3, refuses NaN/Fortran/shape surprises; trilinear, border-clamped; emits the SAME closure type `offset_by`/`lerp` consume. Stress-graded gyroid pin: high-stress half 1.42× material; real RESPOOL FEA field round-tripped via `tools/analyzers/stress_to_density.py` (`tests/grid_field.rs`) |
| surface textures | `kernel_implicit::texture::{displaced, Texture::{Knurl,Stipple,Noise}}` | displacement with DERIVED Lipschitz constants, field renormalized ÷L′ so `DistanceBound` stays sound; dense vs narrow-band volumes bit-equal (`tests/texture_text.rs`) |
| text emboss/engrave | `kernel_implicit::text::{text_field, text_advance}` | Hershey Simplex capsule strokes (provenance + license in-module), exactly 1-Lipschitz; engrave pin 128.05 vs 128.04 mm³ analytic (0.01%) (`tests/texture_text.rs`) |
| shell / offset | `kernel_model::shell::{offset_mesh, shell_mesh, offset_to_solid, shell_to_solid}` | voxel route, labeled honestly; −0.10…−0.19% vs exact Steiner analytics; cavity survives to B-rep (shells=2) (`kernel-model/tests/shell_offset.rs`) |
| reverse bridge v1 | `kernel_model::reverse::{mesh_to_solid, implicit_to_solid}` | implicit/mesh → faceted B-rep, volume-conservation gate, counts-carrying refusals; STEP round-trip drift exactly 0.0 mm³ (`tests/reverse_bridge.rs`) |
| reverse bridge v2 (2026-07-30) | `kernel_model::reverse::{mesh_to_solid_recovered, implicit_to_solid_recovered}` · `kernel_brep::recover::recover_quadrics` | analytic quadric recovery WITH face collapse on all four quadrics, via interior-refined tessellation of merged faces (parameter-space charts — the boundary-ring-only contract is lifted). Measured: implicit cylinder 1326→**24** faces, STEP **5.74 MB→180 KB (31.9×)**, re-import drift **0.0000%**; implicit sphere 14 979→**6** chart sextants; cone 24 710→**30**; builder torus 4608→**16** (4×4 azimuth×tube quadrants). A full-wrap sphere/torus legitimately stays split into charts (a single-loop face cannot be a periodic wrap). Honest caveat: a MESHER-derived torus still drops to retag — coincident chords leave 4-triangle edges, and watertightness is the refusal gate. Hex-prism negative control rejected at 21.4× tol by the closed-form sagitta. Provenance now SURVIVES the rebuild, so this may run mid-chain (`kernel-brep/tests/{recover,curved_faces}.rs`, `tests/reverse_bridge.rs`) |
| sparse fields (2026-07-30) | `kernel_implicit::sparse::{SparseGrid, OctreeGrid}` | Lipschitz-safe tile allocation (proof in-module): 200 mm domain @ 0.4 mm = 22.6 MB vs 503 MB dense (4.49%), build evals 9.75% of dense, cache error 5.06e-4 mm, mesh-through-cache volume delta 0.00002%; octree = evaluation cache only (T-junction caveat stated) (`tests/sparse.rs`) |
| interrogation probes | `kernel_model::reverse::thin_wall_report` · `kernel_brep::holes::min_ligament` | sampled thin-wall estimate (under-reports ≤ one cell, documented) · advisory bore-wall ligament echo, 2.000 mm pin (`tests/reverse_bridge.rs`, `kernel-brep/tests/holes.rs`) |

Routing honesty carries over: everything mesh-borne here is voxel-accurate
(±½ voxel class); `reverse` v1 hands faceted operands, v2 recovers quadric
carriers where they fit within tol and reports the residual. Engine-wide
riders landed 2026-07-30: intra-arrangement threading (booleans ~2× on
heavy chains at 8 cores, byte-identical to sequential BY CONSTRUCTION,
`LMCAD_BREP_THREADS`, parity + threaded-40× pinned — docs/NUMERICS.md) and
GPU narrow-band extraction (`extract_narrow_band` in `kernel-gpu`, a crate
parked unbuilt in `legacy/kernel-gpu/` since 2026-09: preview
path, domains beyond the dense 2²⁸-cell cap delivered watertight, CPU
stays bit-authoritative). The five capabilities on this JSON surface since
2026-07-30: `offset_solid`, `shell_solid`, `solid_from_implicit`,
`thin_wall`, `min_ligament` + tree leaves `strut_lattice`/`pipe_path`/
`text`, combinator `displace`, and the `{"grid":…}` field source (all
executed in API.md).

### 25.7 The analysis plan — the required-analyses law

Step 7 above, stated in full, because it is the rule that makes a campaign an
*engineering* deliverable rather than a geometry deliverable.

**The set of analyses is itself a research output.** The research pass
(the research procedure) does not only freeze dimensions; from the
governing physics and the artifact class's known failure modes it freezes
WHICH analyses this artifact requires. A bracket may need one static check.
A nozzle needs isentropic flow, thermal, structural and flow separation. The
plan is written into `analysis/DESIGN.md` before the geometry is trusted.

**Every item on that plan must be answered in `analysis/ANALYSIS.md` in
exactly one of three ways:**

1. **Receipts from a solver that exists** — cite the run, the manifest, and
   the measured margin.
2. **A solver written for it** — permitted and expected, but a written solver
   is GUILTY UNTIL ITS OWN GATES ARE GREEN: closed-form benchmarks with a
   measured convergence check, negative controls that fire, and a
   meta-negative-control proving the suite can fail. It then joins the
   registry (below) so the next campaign inherits it.
3. **"Required, NOT performed"** — in bold, with the reason. An honest gap is
   a legitimate deliverable; a hidden one is not.

**Silence about a required analysis is the one forbidden outcome.** A reader
must never have to guess whether an analysis was skipped or merely omitted
from the write-up.

**The solver registry** (`tools/solvers/`, index in its `README.md`) is the
shop's accumulated analysis capability — one card per solver: physics,
governing equations, discretization, I/O contract, CURRENT measured benchmark
numbers, validity limits, and when to use it. Read the card before quoting a
solver; if the card's limits exclude your case, that is a "NOT performed" row
or a new solver, not a shrug. Solvers are voxel-discretized and their cards
say what they cannot see (staircased stress concentrations, layer adhesion,
imperfection-driven buckling) — quote the limit alongside the number.

**Time-dependent allowables.** A part under sustained load is a creep case,
not a static one: gate against `kernel_model::materials::pla::creep_allowable_mpa`
(temperature × duration, conservative rounding, 0.0 above the tabulated hot
tier) and state the duration you designed for. The static allowables describe
a load applied and removed, which is not what a shelf bracket or a spool in a
warm dryer experiences.

# Appendix A — Coverage map: every op family → guide section → API.md

The op count is mechanical: `op_tag` in `kernel-api/src/discover.rs` is a
compile-forced exhaustive match over `OpKind` (regenerated by
`tools/gen_discover.py`, pinned by `tests/describe.rs`; the `unknown_op`
error message and the `describe` op both say 160). Re-derived
2026-07-30 by extracting every `op_tag` arm and grouping:

**11 + 3 + 4 + 11 + 14 + 3 + 2 + 3 + 10 + 4 + 5 + 48 + 13 + 13 + 7 + 7 + 3
= 161.**

| family (count) | ops | guide | API.md heading |
|---|---|---|---|
| solid constructors (11) | box, cylinder, sphere, cone, torus, extrude, extrude_with_holes, extrude_tapered, revolve, loft, sweep | §5 (core nine executed §5.1–5.3; loft/sweep §5.4 — now first-class ops as well as Document features) | "Solid constructors" |
| sketch ops (3) | sketch, sketch_extrude, sketch_revolve | §6 (revolve variant §6.4 → API example) | "Sketch ops" |
| booleans (4) | union, difference, intersection, union_all | §7 (all four executed; `try_*` Rust note §7.5) | "Booleans" |
| features & transforms (11) | fillet_edge_near, chamfer_edge_near, fillet_circular_rim, translate, rotate_x, rotate_y, rotate_z, pose, mirror, linear_pattern, polar_pattern | §8.1–8.2, §9.1 (original six executed; rotations/mirror/patterns echo §9.3's Document forms) | "Features & transforms" |
| measures (14) | validate, volume, exact_volume, mass_properties, bounding_box, wall_thickness, draft_analysis, mesh_components, coincident_fit, support_report, clearance, measure_dimension, thin_wall, min_ligament | §10.1–10.2 (core six executed); §22 support_report; §25.1 thin_wall/min_ligament (2026-07-30, executed API.md) | "Measures" (thin_wall/min_ligament under "Implicit / hybrid") |
| assertions (2) | assert, assert_disjoint | §10.2–10.3 (both executed, incl. failures) | "Assertions" |
| exports (3) | export_stl, export_step, export_3mf | §21 (all three executed) | "Exports" |
| implicit / hybrid (10) | implicit, gyroid_block, tpms, hybrid_boolean, shell, offset_solid, shell_solid, solid_from_implicit, sample_density_grid, mesh_density_grid | §11–15 (tree exhaustively; gyroid_block §11.6; tpms 2026-07-12); §17 hybrid; §25.1 voxel-route solid trio 2026-07-30 (executed API.md) | "Implicit / hybrid" |
| discovery (3) | describe, list_faces, list_edges | self-describing surface (M3): `describe` serves the 161-op catalogue + per-op param tables over the wire (pinned `tests/describe.rs`) | "Discovery & introspection" |
| native formats & imports (4) | load_part, import_step, import_mesh, mesh_carve | §16 (load_part throughout; refusal §16.7); §21.3 STEP import; §17 mesh routes | "Native formats" / "Imports" |
| parts library (5) | library_add, library_search, library_instantiate, library_deprecate, library_remove | §3.4 + §19 (all five executed, both refusals provoked) | "Parts library" |
| standard parts (48) | §20.1 full table | §20.2 (one executed program per family group; plus spur_gear §3.1, shaft §10.2) | "Standard parts catalog" |
| standard feature cuts (13) | §20.3 list | §20.3 (3 executed here; the gland cuts echo §20.4's executed metric-cord table; remainder: API.md runnable examples) | "Standard feature cuts" |
| assembly ops (13) | asm_instance, asm_instance_mesh, asm_mate, asm_mate_axis, asm_mate_face, asm_solve, asm_contacts, asm_interference_volume, asm_mass_properties, asm_export, asm_export_step, asm_save, gear_train_poses | §18 (the `.lmcasm` runner is the doctrine anchor; these are its op-surface siblings) | "Assembly ops (in-program)" |
| design-math lookups (7) | gt2_belt, gt2_center_distance, iso286_fit, heatset_spec, metric_cord_gland, racetrack_cord_length, pipe_thread_g | §20.4 (all seven executed in one program) | "Design-math lookups" |
| hole wizard (7) | drill, clearance_hole, counterbore_hole, countersink_hole, tap_drill_hole, bolt_circle, bearing_seat | §8.3 (five in one executed program), bolt_circle §3.3/§10.2, bearing_seat §10.2/§20.2 | "Hole wizard" |
| threads (3) | thread_spec, thread_ridge, export_threaded | §15 (the implicit half's signature move; spec/ridge/export trio) | "Modelled ISO threads" |

Non-op surfaces with their own sections: the `.lmcasm` runner (§18; API.md
"The assembly surface"), the `.lmcpart` Document grammar — 30 feature
variants, all tabled §16.4, the Document-only ones executed (§5.4 loft/
sweep, §9.3 patterns/mirror, §16.5 ExtrudeSketch, §16.6 persistent fillet/
rounded cylinders/Transform/concave rim, §16.8 GyroidLattice grade,
§17.1 HybridFuse/PipeFeat) — and the implicit tree grammar (17 leaves + 21
combinators + 16 scalar ops + the `{"grid":…}` field source as of
2026-07-30: the original 12 leaves + tpms + strut_lattice/pipe_path/text
and the displace combinator, §11.1/§25.1, every name interpreter-provoked;
original leaves and combinators executed across §11–15, the 2026-07-30
additions executed in API.md; scalar coverage: 13/16 in §12.1's dish +
mod/atan2/length3 in §15's thread).

*Written and executed against the current main binary (`kernel-api`,
pinned for this edition). Every fenced JSON block above was re-extracted
from this exact document and re-run before commit; the quoted reports are
unedited excerpts from those runs.*
