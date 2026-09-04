# production_dossier — what does this design cost, weigh, and how many plates and hours is it?

**Runner**: `tools/publish/production_dossier.py` (shim: `tools/production_dossier.py`) ·
`python3 tools/publish/production_dossier.py job.json`
**Gates**: `python3 tools/validation/production_dossier_validation.py`
(shim: `tools/production_dossier_validation.py`) · `python3 tools/tests/test_aux_tools.py`
**Status**: pin green — re-run 2026-09-04 in this worktree: *"production_dossier
validation: ALL PINS OK"*; `test_aux_tools.py` 24/24 passed. Read the gate table
before trusting that second command: its only `production_dossier` coverage is
the `--help` robustness pin.
**Tier**: **Validated** (`python3 tools/analyzer_registry.py --tier
production_dossier`; `kind` = reporting — *Validated bookkeeping*: the pin proves
the mesh maths and the packing are exact against analytic boxes, and proves
nothing physical).

Every campaign ships a BOM, and a BOM that lies about mass lies about cost, about
print time, and about whether the design fits on the bed at all. This tool reads
the actual exported STLs, computes their true mesh volume and area, applies a
stated shell+infill model to get the grams the printer will really extrude,
prices and times every line, packs the made parts onto plates, and writes
`bom_dossier.json` + `bom_dossier.csv` to `out_dir`.

The reason the printed-mass model exists rather than `rho * V`: solid mesh grams
are **not** what the printer extrudes for thick sections — a 50 mm PLA cube is
155 g solid and 57 g printed at 20 % infill. Quoting the solid number is how a
design ends up promising 4.6 kg of filament it does not need.

## What it actually computes

Closed forms over the STL triangle soup, plus a heuristic mass/time model and a
2-D packing pass. No physics of any kind.

    volume:     V = | sum_tris (v0 . (v1 x v2)) / 6 |      (divergence theorem, |sum| taken)
    area:       A = sum_tris |(v1 - v0) x (v2 - v0)| / 2   (both sides of every wall)
    shell:      t_shell = perimeters * line_width + top_bottom_layers * layer_h / 2
                V_shell = min(V, A * t_shell)
    printed:    printed_g = rho * [ V_shell + infill * max(0, V - V_shell) ] / 1e6
                solid_g   = rho * V / 1e6
    time:       t_h = printed_g / 12 * speed_factor  per part  + 0.25 h setup per plate
    cost:       unit_cost = printed_g * price_per_kg / 1000 ;  line_cost = unit_cost * qty
    thick warn: solid_g > 2 * printed_g
    packing:    first-fit-decreasing shelf packing (long side desc), 0/90 rotation,
                `spacing_mm` to every neighbour AND to the bed edges

Defaults `{perimeters 3, line_width 0.45, layer_h 0.2, top_bottom_layers 4,
infill 0.20}` give `t_shell` = 1.75 mm; bed default 220 × 220 × 250 mm,
`spacing_mm` 5, filament 25/kg. Densities come from `tools/material_db.json`
(inline `density_kg_m3` overrides win); a material with no density from either
source is refused.

Two assumptions carry the whole result. **The mesh is assumed watertight and
consistently wound** — garbage in, garbage grams, with no warning here; gate
`watertight` on the export receipt first. And **the top/bottom skin stacks are
smeared over ALL surfaces at half thickness**, which is a stated simplification
(only up/down-facing regions really carry them), not a slicer emulation.

## The contract

Job:

```
{"out_dir": "...",                                  # REQUIRED
 "parts": [                                         # REQUIRED
   {"name","stl","material","qty"?,"material_required"?,"print_notes"?,"print_params"?},
   {"name","buy": true,"qty"?,"part_number"?,"unit_price"?,"source"?,"wear"?}],
 "bed"?: {"x":220,"y":220,"z":250}, "spacing_mm"?: 5,
 "density_kg_m3"?: {...}, "print_params"?: {...},
 "filament_price_per_kg"?: 25, "print_speed_factor"?: 1.0,
 "emit_plates"?: true, "date"?: "..."}              # date is a job string, never the clock
```

Receipt (LAST non-empty stdout line; the human table goes to stderr):
`{ok, parts[], totals, plates[], warnings, bed, spacing_mm,
filament_price_per_kg, material_db_used, mass_model, time_model,
volume_method}`, where each made line carries `volume_mm3`, `surface_mm2`,
`density_kg_m3` + `density_source`, `t_shell_mm`, `shell_vol_mm3`,
`solid_g_per_unit`, `printed_g_per_unit` (= `grams_per_unit`, the number that
drives cost and time), `print_time_h_per_unit`, `footprint_mm`, `height_mm`,
`unit_cost`, `line_cost`, `thick_section_warning`; and `totals` carries
`n_plates`, `total_print_time_h`, `total_made_cost`, `total_buy_cost`,
`total_cost`, `buy_lines_tbd`, `total_cost_note`. The `mass_model`,
`time_model` and `volume_method` strings ship the honesty bands **inside** the
receipt. Side effects: `bom_dossier.json` and `.csv` in `out_dir`, plus per-plate
STLs and `plate_layout.png` when `emit_plates` is on (the PNG needs matplotlib).

Buy lines without `unit_price` are **TBD and EXCLUDED** from the numeric total;
`buy_lines_tbd` names them and `total_cost_note` says how many were left out.

Failure is **exit 1** with `{ok:false, error}` — this tool has its own `main`
and, unlike the analyzers, does **not** distinguish exit 1 from exit 2. A part
taller than `bed.z`, or whose footprint cannot fit an empty bed in either 0/90
rotation with spacing, refuses the whole job and writes no dossier.

## Benchmark gates (measured, frozen)

All numbers below come from `tools/validation/production_dossier_validation.py`,
which writes analytic 12-triangle box STLs and re-derives every expectation
in-file from the manifest's closed forms. The registry names
`tools/tests/test_aux_tools.py` as this analyzer's gate suite; **that suite's
only `production_dossier` coverage is `test_help_never_crashes`** (`--help` must
exit 0, print something useful, and not traceback). No numeric result of this
tool is pinned there.

| gate | what it pins | where |
|---|---|---|
| box arithmetic | 30 × 20 × 10 mm PLA box → V 6000 mm³, A 2200 mm², `t_shell` 1.75 mm, `V_shell` min(6000, 3850) = 3850, printed volume 4280 mm³ → **5.3072 g printed / 7.44 g solid**, cost 0.13268 at 25/kg, 0.442267 h, footprint [30, 20] × 10 mm, 1 plate → 0.692267 h total; density 1240 from `tools/material_db.json` | pin 1, validation script |
| thick-section warning | 50 mm cube: V 125000, A 15000 → printed 46000 mm³ = **57.04 g** vs solid **155.0 g** → warning fires on the line AND in `warnings` | pin 2 |
| shell cap | 30 × 20 × 1 mm plate: A·t_shell 2275 > V 600 → `V_shell` capped at 600, `printed_g == solid_g == 0.744 g` (a part thinner than its own shell prints fully dense) | pin 3 |
| TBD buy lines | 4 × 0.05 = 0.20 summed; an unpriced `bearing` line has `line_cost: null`, is excluded from `total_cost` 0.33, and is named in `total_cost_note` ("EXCLUDES 1 TBD buy line(s): bearing") | pin 4 |
| packing | qty 4 of the box on a 60 × 60 × 250 bed at 5 mm spacing → **2 plates, 2 parts each**; every placement inside the 5 mm margin and mutually separated; total 4 × 0.442267 + 2 × 0.25 = 2.269 h; each emitted plate STL's volume = 2 × 6000 mm³ | pin 5 |
| refusal | a 300 mm tall part vs bed z 250 → `ok:false`, exit 1, "exceeds bed z", and **no `bom_dossier.json` left behind** | pin 6 |
| determinism | two runs of one job (same `out_dir`) produce a byte-identical receipt | pin 7 |
| error band | volume and area exact to 1e-6 relative — **measured 0.0**; mass/cost/time exact to the receipt's rounding — measured within 5e-4; packing exact plate count and placements; `last_measured` 2026-09-02 | `validation.error_band`, `tools/manifests/production_dossier.manifest.json` |

## Validity limits / out of scope

- **The ±30 % mass and ±50 % time bands are DECLARED honesty bands, not
  measured error bands.** Nothing in this repo compares `printed_g` or
  `print_time_h` against a slicer's output or a real print. **Not
  characterised**: the true error of the shell/infill model and of the
  12 g/h time rule is unquantified here — treat both as planning figures, take
  the slicer's number when one exists, and never gate a claim on either. What
  *is* pinned is that the model computes what it says it computes.
- **A non-watertight or inconsistently wound STL yields a wrong volume with no
  warning.** The tool cannot detect it; check the export receipt's `watertight`
  flag upstream.
- **Packing uses axis-aligned footprint bounding boxes**, not true outlines, so
  an L-shaped or nestable set is over-estimated in plate count (conservative,
  never the reverse), and the shelf FFD is not an optimal packer.
- The time heuristic models no travel, no retraction, no geometry-dependent
  speed, and assumes the 0.2 mm layer class; `print_speed_factor` is a user
  input, not a calibration.
- **Bookkeeping, not physics**: no strength, no thermal, no warp, no support
  material, no purge/prime waste, and no slicer emulation of any kind.
- Failure exits **1** only; do not write a negative control that distinguishes
  1 from 2 on this tool.

## When to use it

At the end of every campaign, and again whenever the geometry changes: it is the
`assembly/` folder's BOM source (`bom_dossier.{csv,json}`, whose CSV row order is
the balloon numbering used by `tools/publish/assembly_doc.py`). Reach for it
mid-design too, as a cheap reality check — the thick-section warning is the
fastest way to learn that a part wants hollowing, and the packing refusal is the
fastest way to learn a part does not fit the bed before you slice it.
