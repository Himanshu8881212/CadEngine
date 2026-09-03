# pre-W6 `.lmcpart` corpus — the backward-compatibility fixtures

These 20 `.lmcpart` documents are **real pre-Wave-6 artifacts**. They were
written by the W5 dogfood campaign (the 15:1 two-stage reference gearbox,
built by a design engineer through public surfaces only — see
`docs/FRICTION.md`) **before** the exotic `Feature` variants, the
`GyroidLattice` `grade` key, the `Tpms` family and the rest of the W6 schema
work existed. That is the whole point of them: they are the only documents in
the tree that were serialized by an older kernel, so they are the only honest
test that `kernel_model::format::load_part` still parses what an older LMCAD
wrote.

## DO NOT REGENERATE OR REFORMAT THESE FILES

Not with a script, not with `jq`, not by round-tripping them through
`save_part`, not to "normalise" whitespace, key order or float spelling.
Regenerating them replaces old bytes with today's bytes, which silently
deletes the exact property the corpus exists to prove. If a new schema change
means one of these no longer loads, that is a **back-compat bug in the
loader**, not a stale fixture — fix the loader.

Adding *more* genuinely-old documents alongside them is fine (bump the count
in the consuming test). Deleting one is not.

## Who consumes them

`crates/kernel-model/tests/exotic_features.rs` →
`beam_lattice_fill_guards_fail_loud_and_old_documents_still_load` asserts:

- all **20** files parse through `load_part`;
- `spacer_10.lmcpart` still rebuilds an exact B-rep with positive volume and
  meta name `spacer_8x12_10`.

They are also the part library behind the preserved reference assembly in
`reference/assembly/` (the `.lmcasm` files there resolve their instance sources
into this directory) and the path used by the studio walkthrough in
`studio/SMOKE.md`.

`spacer_21.lmcpart` in particular carries a real `Dim` (`len`), which is why
it is the part the studio params/`set_dim` demo drives.
