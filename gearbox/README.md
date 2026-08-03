# Parametric 2-stage reduction gearbox (15:1) — LMCAD dogfood build

A complete, printable gearbox designed end-to-end through the kernel's **public surfaces
only**: every part is a native `.lmcpart` parametric document, every part is
validated/measured/exported by a `kernel-api` CLI program, and the whole machine is a
`.lmcasm` assembly with poses, mates and an exploded state — in two equivalent builds,
the flat `gearbox.lmcasm` and the nested `gearbox_nested.lmcasm` (the three shaft stacks
as `asm/shaft_*.lmcasm` sub-assemblies). The matching papercut log is
[`../docs/FRICTION.md`](../docs/FRICTION.md).

```
input 608 ──┐   stage 1: z12 ── z60   (m 1.25)   i₁ = 5.0   C₁ = 45.00 mm
            │   stage 2: z15 ── z45   (m 1.50)   i₂ = 3.0   C₂ = 45.00 mm
output 608 ─┘   overall i = 15.0 : 1             mounted C* = 45.15 mm (both stages)
```

## Gear train

Centre distance: `C = m·(z₁+z₂)/2` → stage 1: `1.25·(12+60)/2 = 45.000`, stage 2:
`1.50·(15+45)/2 = 45.000`. Both stages are mounted at `C* = C + 0.15 mm`: involutes
tolerate centre-distance growth, trading it for backlash — circumferential
`j_t = 2·ΔC·tan 20° = 0.109 mm`, i.e. ≈ 0.051 mm per flank normal gap. The assembly
contact scan measures the tightest flank gap at **0.050 mm** (`kernel-api asm` +
`check_asm.py`), matching theory to a micron.

| gear | module | z | PA | pitch Ø | tip Ø | root Ø | face | bore | hub keyway | mesh phase |
|---|---|---|---|---|---|---|---|---|---|---|
| S1 pinion (`gear_s1_pinion`) | 1.25 | 12 | 20° | 15.00 | 17.50 | 11.875 | 12 | Ø8 H7 | DIN 6885 2×2 | 0° |
| S1 wheel (`gear_s1_wheel`) | 1.25 | 60 | 20° | 75.00 | 77.50 | 71.875 | 10 | Ø8 H7 | DIN 6885 2×2 | +3° |
| S2 pinion (`gear_s2_pinion`) | 1.50 | 15 | 20° | 22.50 | 25.50 | 18.75 | 14 | Ø8 H7 | DIN 6885 2×2 | +3° |
| S2 wheel (`gear_s2_wheel`) | 1.50 | 45 | 20° | 67.50 | 70.50 | 63.75 | 12 | Ø8 H7 | DIN 6885 2×2 | −1° |

Mesh phasing (kernel convention: tooth 0 and hub keyway both centred on +X): with the
pinion tooth on the line of centres, the z=60 wheel (even, tooth at 180°) must turn half a
pitch (+3°); the z=45 wheel natively presents a space. The +3° rides the intermediate
shaft, so its keyway — and therefore `gear_s2_pinion` — sits at +3° too; conjugate
transfer puts the output wheel at `−3°·(15/45) = −1°`. The note in each shaft's pose
quaternion is exactly this chain. z12 < 17 teeth means undercut is required but not
modelled (kernel's documented ISO 53 approximation) — acceptable for printed gears.

## Architecture

Three Ø8 shafts on parallel Y-axes at height z=48, x = 0 / 45.15 / 90.30, in a drafted
open-top housing with a flat lid:

- **housing_base** — outer shell drafted 1.5° and cavity drafted 2.0° (both widest at the
  z=91 parting plane, `ExtrudeSketch` swept downward), 12 mm flange band with 8 heat-set
  pockets + 2 dowel press holes, 6 bearing bosses Ø34, six 608 seats (Ø22×7 pocket from
  the outside to a web shoulder + Ø16 race-clearance web bore), 4 M3 heat-set accessory
  bosses on the floor. Genus 6 (= 3 axes × 2 walls), 457.2 cm³.
- **housing_lid** — 8 mm plate, 8× DIN 912 M4 counterbores (DIN 974: Ø8 × 4.8), 2 dowel
  slip holes, and a racetrack O-ring groove 2.7 × 1.5 mm in the seal face for a Ø2.0 EPDM
  cord (~392 mm loop, ~25 % squeeze). Genus 10.
- **shafts** — Ø8 catalog blanks with DIN 6885 (form B) keyseats and DIN 471 circlip
  grooves outboard of the entry bearings (input front, output rear); the intermediate
  shaft is captive between the webs via the spacer/gear stack.
- **bearings** — 6× 608 (8×22×7), outer races seated against the web shoulders, pressed
  from outside; modelled as exact envelope rings (no rolling-element catalog part —
  FRICTION #8).
- **axial stacks** (race-to-race): printed spacer tubes Ø12/Ø8.4 — input 9 | g1p(12) | 23;
  intermediate 10 | g1w(10) | g2p(14, hubs butt) | 10; output 21 | g2w(12) | 11.

## Fits (ISO 286, `programs/check_fits.json`)

| joint | fit | hole (mm) | shaft (mm) | clearance (mm) |
|---|---|---|---|---|
| shaft in 608 bore (Ø8) | H7/k6 | +0.000/+0.015 | +0.001/+0.010 | −0.010…+0.014 (transition) |
| 608 OD in housing (Ø22) | H7/h6 | +0.000/+0.021 | −0.013/0 | 0…+0.034 |
| gear bores on shaft (Ø8) | H7/h6 | +0.000/+0.015 | −0.009/0 | 0…+0.024 |
| dowel press in base (Ø4) | H7/n6 | +0.000/+0.012 | +0.008/+0.016 | −0.016…+0.004 (press) |
| dowel slip in lid (Ø4) | H7/g6 | +0.000/+0.012 | −0.012/−0.004 | +0.004…+0.024 |

Keys are nominal 2×2 DIN 6885 (t1 1.2 shaft / t2 1.0 hub, 0.2 top clearance). Printed
parts: apply your printer's horizontal compensation; the model is nominal.

## Bill of materials

Printed / machined (modelled, `parts/*.lmcpart` → STL+STEP in `out/`):

| part | qty | note |
|---|---|---|
| housing_base | 1 | 457.2 cm³ model volume |
| housing_lid | 1 | 119.3 cm³ |
| gear_s1_pinion / s1_wheel / s2_pinion / s2_wheel | 1+1+1+1 | ISO 53 involute |
| shaft_input (Ø8×73) / _intermediate (Ø8×55) / _output (Ø8×82) | 1+1+1 | steel stock preferred |
| spacer Ø12/Ø8.4 × 9, 10(×2), 11, 21, 23 | 6 | printed |
| key 2×2 × 8(×3), 10(×2), 12(×1) | 6 | DIN 6885 form B |

Purchased:

| item | qty |
|---|---|
| 608 bearing (608ZZ) | 6 |
| DIN 912 M4×12 | 8 |
| ISO 2338 dowel Ø4×12 | 2 |
| DIN 471 circlip Ø8 | 2 |
| M4 heat-set insert (Ruthex, Ø5.6 pilot) | 8 |
| M3 heat-set insert (Ø4.0 pilot) | 4 |
| Ø2.0 EPDM O-ring cord, ≈392 mm loop | 1 |

The machine-readable BOM is written by `kernel-api asm gearbox.lmcasm` to
`out/asm/bom.json` + `out/asm/bom.csv` (spacer/key variants group by their `len`
parameter). It is **BOM v2** (`schema: "bom/2"`): a grouped `flat` view plus a `tree`
view mirroring the assembly structure, with the five metal parts carrying engineering
metadata from their `.lmcpart` `meta` blocks (stamped by `generate.py`) — part number,
material, make-or-buy, and mass = density × the kernel's volume with an honest
`volume_source` label (`"exact"` for all five: analytic B-rep volumes):

| meta-stamped part | part number | material | sourcing | unit mass |
|---|---|---|---|---|
| shaft_input / _intermediate / _output | GBX-SH-IN / -MID / -OUT | steel 7.85 g/cm³ | make | 28.4 / 21.3 / 32.0 g |
| bearing_608 | 608ZZ | steel 7.85 g/cm³ | buy | 18.1 g * |
| key_2x2_8 | DIN6885B-2×2×8 | brass 8.4 g/cm³ | buy | 0.27 g |

\* the 608 is modelled as its solid **envelope ring** (FRICTION #8), so density ×
envelope volume intentionally **overstates** a real 608ZZ (≈12 g — steel plus cage and
air): the BOM mass is honest about the modelled geometry, not a datasheet lookup.

### Nested variant (`gearbox_nested.lmcasm`, assembly-nesting v2)

The same 37 parts at the same world poses, regrouped: each shaft's drivetrain stack
(shaft, gears, bearings, spacers, keys) becomes a path-referenced sub-assembly —
`asm/shaft_input.lmcasm` (8 parts), `asm/shaft_intermediate.lmcasm` (9),
`asm/shaft_output.lmcasm` (8) — placed by `{"asm_path": …}` instances next to the
housing and fasteners (15 top-level units). Each stack solves its **own** gear-on-shaft
concentric mates first, then the parent's mates pin it to the housing bore axis as **one
rigid unit** (the mate's b-geometry is written in the stack's frame: its shaft axis is
+Y through the unit origin). The report names leaf parts hierarchically
(`stack_in/g1p`), the BOM `tree` rolls up 8/9/8 under the three stacks while `flat`
still totals 37, and the contact scan finds the same 52 designed contacts — now across
nesting levels (`base ↔ stack_in/608_in_A`). v2 limits on display: parent mates/states
address top-level units only, so the nested `exploded` state lifts each stack as one
piece (members cannot fly apart axially as in the flat exploded view), and suppressing a
stack instance would drop its whole branch from geometry, contacts and BOM.

## Files & how to run

```
gearbox/
  generate.py            # ALL parameters + emitter for parts/programs/assemblies
  parts/*.lmcpart        # 20 parametric part documents (5 with BOM v2 "meta" blocks)
  programs/p_*.json      # one CLI program per part: load → validate → measure → export
  programs/check_*.json  # fits, stage-1/2 mesh proofs, swept-envelope clearance, evidence
  gearbox.lmcasm         # flat: 37 instances, 4 mates, named "exploded" state
  gearbox_nested.lmcasm  # nested: 15 top-level units (3 asm_path sub-assemblies)
  asm/shaft_*.lmcasm     # the three shaft-stack sub-assemblies (8/9/8 parts, own mates)
  check_asm.py           # design-intent contact allowlist + nested BOM v2 intent check
  run_all.sh             # build CLI, run everything, summarize
  out/                   # exports + reports (gitignored)
```

```bash
cd gearbox && ./run_all.sh         # everything below, with a pass/fail summary
# or by hand:
cargo build -p kernel-api --release
../target/release/kernel-api run programs/p_housing_base.json --out-dir out
../target/release/kernel-api asm gearbox.lmcasm --out-dir out/asm > out/asm_report.json
python3 check_asm.py out/asm_report.json
../target/release/kernel-api asm gearbox_nested.lmcasm --out-dir out/asm_nested > out/asm_nested_report.json
python3 check_asm.py out/asm_nested_report.json
```

## Verification summary (all green)

- 21 part programs exit 0: every part **closed, manifold, expected genus** (gears 1,
  shafts/keys/screws/pins 0, spacers/bearing 1, base 6, lid 10), every STL watertight,
  STEP exported. `check_clash_expected_fail.json` exits 1 **by design** (FRICTION #4).
- `check_mesh_stage1/2.json`: posed gear pairs at C*=45.15 union to **2 shells** (teeth
  interleave, no contact) in the as-assembled AND half-pitch-rolled configurations.
- `check_envelopes.json`: housing ∪ all four swept gear envelopes ∪ all three shaft
  envelopes = **2 shells** — the housing clears every rotating part in every rotation.
- `check_fits.json`: the table above.
- `kernel-api asm gearbox.lmcasm` (+ `check_asm.py`): loads, mates re-solve to residual
  ~1e-12, BOM groups, merged + per-instance + exploded-state STLs export, and the contact
  scan finds **52 designed contacts / 0 unexpected**, tightest must-clear gap 0.050 mm
  (g1p↔g1w flanks — the designed backlash). This used to require the `tools/asmcheck`
  Rust workaround; the official surface replaced it (FRICTION #1/#2 resolved-w6).
- `kernel-api asm gearbox_nested.lmcasm` (+ `check_asm.py`): the nested build loads
  recursively (residual is the max across all levels, ~1e-12), flattens to the same 37
  leaf parts under hierarchical names, finds the **same 52 designed contacts** across
  nesting levels, and its BOM v2 reports the 8/9/8 stack tree rollup with the
  meta-derived masses pinned against closed forms (bearing ring, brass key).
- Caught during dogfood by these checks: DIN 974 M4 counterbore is 4.8 deep (not 4.4) —
  screw poses corrected when the seating contacts went missing (FRICTION #9).

## Known modelling limitations (all logged in docs/FRICTION.md)

- Keys/keyseats are form B (square-ended) in the documents — the form-A op exists but has
  no Document twin (#7). Bearings are envelope rings (#8: no rolling-bearing part). Gear
  STLs ship voxel-healed at 0.3 mm (#6). Circlips are BOM-only in the assembly (#7);
  their grooves are modelled. Draft analysis on the finished base reports undercut area
  from the cross-axis bearing bores — true for a 2-plate mold (side cores needed),
  irrelevant for printing.
