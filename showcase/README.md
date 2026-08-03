# Showcase — the curated LMCAD deliverable

One end-to-end project produced by the LMCAD pipeline (engine driven over MCP: design →
analysis → assembly verification → production dossier). Everything here is a **copy** of
the session outputs, curated down to what a reader needs: the print-oriented STLs, the
human docs, the verdict-relevant receipts, and the saved programs that regenerate or
re-verify every part.

| project | what it demonstrates | key receipts (numbers from the receipts, not from prose) |
|---|---|---|
| [`squatchee_spin/`](squatchee_spin/) | Cap-button propeller, "receipts as marketing" — 3 printed parts, **9.9 g**, no hardware, support-free. v2 field-fix (2026-07-15) answered a real breakage report at the root: the v1 split snap-post (two ~1 mm towers, unprintable as per-layer islands) was **replaced** by a solid Ø4.4 post + push-and-twist bayonet, so retention is *geometric* rather than spring interference; the thick-wall rework took every non-spring wall to ≥2 mm at *lower* snap strain (≤1.72%, was 1.97%). Matched set — no v1/v2 part mixing. | `support_report` steep area **exactly 0.0** on all three parts (worst micro-bridge 2.82 mm); `mass_properties` CG **1.8e-9 mm** off the spin axis → **0.0 g·mm** static imbalance; pivot fit **0.30–0.90 mm** across ±0.15 mm print extremes; retention proven by a **measured negative control** (lifted-while-locked interferes 0.0263 mm³) and release by its twin (untwisted lifts free, 0.1488 mm); `receipts/approach_sweep.csv` 18 clean descent stations, `twist_sweep.csv` 9 clean lock stations, `prop_drop_sweep.csv` 14 clean drop-on stations. |

**Published, and entered in Printables' Cap Hacks contest** (July 2026, 204 entries):
[printables.com/model/1777704-squatchee-spin](https://www.printables.com/model/1777704-squatchee-spin)
— **444 likes · 4.6★ · 1.2K downloads**, 3rd by likes in the field, jury result pending.
The listing text is [`squatchee_spin/printables_listing.md`](squatchee_spin/printables_listing.md),
written *from* the receipts above rather than around them.

That link is the point of this directory. Gates prove internal consistency; 1.2 thousand
strangers' printers prove the rest. v1 shipped, came back with three field complaints,
and v2 answered all three at the root — see the README's field-loop section.

## Layout

```
squatchee_spin/
├── print/       print-oriented STLs (what you slice)
├── docs/        assembly doc + instructions + drawing sheets
├── receipts/    the verdict-relevant machine receipts (curated, not every probe job)
└── programs/    saved run_program/run_assembly programs + analysis job orders
```

## Reproducing

Geometry needs nothing but the kernel CLI:

```sh
cargo build -p kernel-api --release
./target/release/kernel-api run squatchee_spin/programs/prop_program.json      --out-dir out/
./target/release/kernel-api run squatchee_spin/programs/mount_program.json     --out-dir out/
./target/release/kernel-api run squatchee_spin/programs/retainer_program.json  --out-dir out/
./target/release/kernel-api run squatchee_spin/programs/assembly_program.json  --out-dir out/
```

The exports come out byte-identical to the committed `print/*.stl` (the boolean pipeline
is run-deterministic). `assembly_program.json` additionally poses all three parts in the
worn attitude and runs the five clearance checks — including the two controls that make
the retention claim falsifiable:

- `c_retention_neg` — lifted **while locked**: MUST interfere (measured 0.0263 mm³). If
  this ever comes back clean, the part can fall off and the claim is dead.
- `c_release` — same lift with the windows **untwisted**: MUST clear (measured 0.1488 mm).
  If this ever interferes, the part is not serviceable.

The verification job orders re-run through the MCP tools and the Python analysis layer:
balance (`balance_job.json`), tolerance stacks (`tol_*.json` →
`tools/tolerance_stack.py`), motion sweeps (`sweep_*_job.json`), production dossier
(`dossier_job.json` → `tools/production_dossier.py`), drawing sheets (`sheet_*_job.json`
→ `tools/render_sheet.py`), assembly document (`asmdoc_job.json` →
`tools/assembly_doc.py`). Start the MCP server with `./target/release/lmcad-mcp`; see
[`../API.md`](../API.md) for the op reference and [`../DESIGN_GUIDE.md`](../DESIGN_GUIDE.md)
for the operator manual.

Source: copied (never moved) from `studio_out/mcp/capspin`. The live working directory
remains `studio_out/` (gitignored).
