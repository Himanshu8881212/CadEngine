# `reference/assembly/` — the worked `.lmcasm` reference

The 15:1 two-stage gearbox that was the W5 dogfood project has been retired as
a *campaign* (design campaigns now live outside this repository). What is kept
here is the part of it the documentation **teaches**: the reference assembly
files themselves, plus the design-intent layer that sits on top of the engine's
contact scan.

Every `.lmcasm` / `.json` / `check_asm.py` here is a genuine artifact of that
campaign. The only edit made when they moved was to repoint each part
reference at the surviving part corpus,
`crates/kernel-model/tests/fixtures/pre_w6_parts/` (see the README there —
those files must not be regenerated). `run_all.sh` and this README were
written at retirement time, replacing the campaign's own driver.

| file | what it demonstrates |
|---|---|
| `gearbox.lmcasm` | a **flat** 37-instance assembly: rigid poses, 4 mates, a named `exploded` state, `meta`-carrying parts for BOM v2 |
| `gearbox_nested.lmcasm` | assembly **nesting (v2)**: the same 37 parts regrouped into three sub-assembly units via `source.asm_path` |
| `asm/shaft_{input,intermediate,output}.lmcasm` | the three sub-assemblies, each solving its **own** gear-on-shaft mates before entering the parent as one rigid unit |
| `check_asm.py` | the **design-intent layer**: an allowlist over the engine's contact report — every touching pair must be a designed fit/seat/butt, the designed-contact count and the must-clear flank gaps are pinned, and the nested run's BOM v2 tree rollup is checked |
| `check_artifacts.json` | exact-boolean disjointness proofs (pose → `union` → `assert {"shells": 2}`) for the five known tessellation-artifact pairs the mesh-distance scan reports as phantom contacts (FRICTION #19) |
| `programs/check_mesh_stage{1,2}.json` | involute mesh assertions: a gear pair as-assembled and half-pitch-rolled, flank gap held to theory |
| `programs/check_clash_expected_fail.json` | a **negative control** — it is *supposed* to exit 1; kept as the documented evidence behind FRICTION #4: the natural "these two posed gears must not intersect" check is an `intersection` whose EMPTY result is the pass condition, but an empty boolean is a loud `invalid_param` failure — which is why `assert_disjoint` exists |

## Running it

Everything at once, from the repository root:

```bash
sh reference/assembly/run_all.sh          # ALL GREEN, exit 0
```

Step by step, with `cargo build -p kernel-api --release` done:

```bash
K=./target/release/kernel-api

# flat: load / mates / BOM v2 / per-instance + merged + STEP exports / contacts
$K asm reference/assembly/gearbox.lmcasm --out-dir out/asm > out/asm_report.json
python3 reference/assembly/check_asm.py out/asm_report.json

# nested (v2): same parts, three sub-assembly units
$K asm reference/assembly/gearbox_nested.lmcasm --out-dir out/asm_nested \
    > out/asm_nested_report.json
python3 reference/assembly/check_asm.py out/asm_nested_report.json

# the exact-boolean phantom-contact proofs (input paths resolve against --out-dir)
$K run reference/assembly/check_artifacts.json --out-dir .

# gear-mesh assertions, and the negative control (this one MUST exit 1)
$K run reference/assembly/programs/check_mesh_stage1.json --out-dir out
$K run reference/assembly/programs/check_mesh_stage2.json --out-dir out
$K run reference/assembly/programs/check_clash_expected_fail.json --out-dir out; \
    [ $? -eq 1 ] && echo "negative control failed as designed"
```

Both `check_asm.py` runs prove **52/52 designed contacts, 0 unexpected**, with
the tightest must-clear gap at 0.050 mm (the designed gear-flank backlash;
involute theory says 0.051). The flat and nested reports are held to the
*identical* allowlist — nested reports name leaves hierarchically
(`stack_in/g1p`) and the allowlist classifies by leaf name.

Note the flat run tessellates ~1M triangles and takes a couple of minutes.

## What is *not* here

The campaign's own build machinery — `generate.py` (which regenerated the
parts and the per-part programs), the 21 per-part programs and the campaign
README — was retired with the campaign; `run_all.sh` here is its surviving
verification half. The parts it produced survive as the back-compat corpus
named above; the engine capabilities it exercised are covered by the workspace
test suite.
