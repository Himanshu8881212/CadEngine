# OP_USAGE.md — which interpreter ops the campaigns actually use

Census taken 2026-09-02 on the main checkout, read-only. **Dated record:** the campaign
directories it counted (`*_system/*`, `gearbox/`) have since been moved out of this
repository — the census numbers below are the ones measured on that date and are not
re-derivable from the current tree. (The gearbox's assembly and evidence programs
survived one wave longer under `reference/assembly/` and were removed on 2026-09-03;
they are in git history at commit `5a70984`.) It answers one question per op:
did any shipped campaign ever name it? The result drives the `catalog` cargo feature of
`kernel-api` (below) and is the evidence behind it.

## Method

- **Op list:** every `OpKind` variant named by the `match kind` in `exec_op`
  (`crates/kernel-api/src/interp.rs`), i.e. every variant the interpreter dispatches — the
  dispatch code, not the docs. That is 161 ops; the family is the banner comment above the
  arm in `exec_op`. (Since 2026-09-03 `exec_op` is a routing table: one arm per family,
  handing off to the matching `crates/kernel-api/src/ops/<family>.rs`. The banners, the
  families and the 161 variants are unchanged — only where each op's body lives moved.)
- **Corpus:** `*_system/*/programs/*.json`, `gearbox/**/*.json` and `showcase/**/*.json` —
  868 JSON files, 394 of them program envelopes with an `ops` array,
  spread over 23 campaign directories (21 `*_system/<part>` campaigns plus `gearbox` and
  `showcase`; `gearbox/**` and `showcase/**` are counted as one directory each). Campaign folders
  with no `programs/*.json` — the Rust-generated ones, parked out of the build in 2026-09 and
  removed from the tree on 2026-09-03 —
  contribute nothing, which is why this is fewer than the number of campaign folders.
- **Counting:** `ops[]` = entries of top-level `"ops"` arrays (what `kernel-api run` dispatches);
  `all` = every `"op": "<name>"` key anywhere in those files, nested sub-programs and
  embedded tool jobs included; `campaigns` = how many of the campaign directories name the op
  at least once. The set of distinct interpreter ops is identical under both counts.

## Summary

| | count |
|---|---:|
| ops the interpreter dispatches | 161 |
| distinct ops used by at least one campaign | 84 |
| never used by any campaign | 77 |
| never used **and** behind the `catalog` feature | 52 |
| never used, kept in core | 25 |
| ops in a `--no-default-features` build | 109 |

**Gating rule.** An op is behind `catalog` iff it has zero campaign usage **and** belongs to one
of the hardware-catalog families named in the cleanup brief: standard parts (NEMA, GT2, TR8
lead screws, bearings, couplings, O-ring grooves, 2020/3030 extrusion and T-nuts, hex/shoulder
bolts, threaded rod, standoffs, spring washers, circlips, shafts and keys, pipe/hose/PC4
fittings, servo pockets, MGN12/SC8UU/SHF8/SK8/KP08 linear hardware, racks, ring gears and
sprockets), the parts library (`library_*`), the lattice ops (`gyroid_block`, `tpms`) and
`sketch_extrude`. Any op a campaign has used stays in core regardless of family (e.g.
`deep_groove_bearing`, `flanged_bearing`, `o_ring`, `o_ring_face_gland`, `circlip_external`,
`spur_gear`, `compression_spring`). Unused ops outside those families also stay in core —
they are engine/measure surface, not catalog: `torus`, `sweep`, `chamfer_edge_near`, `fillet_circular_rim`, `coincident_fit`, `list_faces`, `list_edges`, `shell`, `offset_solid`, `shell_solid`, `thin_wall`, `sample_density_grid`, `mesh_density_grid`, `mesh_carve`, `button_head_screw`, `board_mount`, `bridged_counterbore`, `asm_instance_mesh`, `asm_mate_axis`, `asm_mate_face`, `gear_train_poses`, `racetrack_cord_length`, `counterbore_hole`, `tap_drill_hole`, `bearing_seat`.

`strut_lattice`, `beam_lattice` and `text` from the brief are not interpreter ops — they are
leaves of the `implicit` expression tree, and `implicit` is core (used by 3 campaigns) — so
there was nothing to gate for them.

## Per-op usage, by family

`build` = **core** (always compiled) or **catalog** (compiled only with the default-on
`catalog` feature). Families and order follow `exec_op`.

### Assemblies (in-program) — see asmops.rs

13 in this family — 9 used, 4 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `asm_instance` | 270 | 270 | 6 | core |
| `asm_instance_mesh` | 0 | 0 | 0 | core |
| `asm_mate` | 201 | 201 | 6 | core |
| `asm_mate_axis` | 0 | 0 | 0 | core |
| `asm_mate_face` | 0 | 0 | 0 | core |
| `asm_solve` | 11 | 11 | 6 | core |
| `asm_contacts` | 37 | 37 | 6 | core |
| `asm_interference_volume` | 17 | 17 | 1 | core |
| `asm_mass_properties` | 8 | 8 | 4 | core |
| `asm_export` | 28 | 28 | 4 | core |
| `asm_export_step` | 6 | 6 | 5 | core |
| `asm_save` | 5 | 5 | 5 | core |
| `gear_train_poses` | 0 | 0 | 0 | core |

### Solid primitives & sweeps

11 in this family — 9 used, 2 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `box` | 2928 | 3208 | 19 | core |
| `cylinder` | 1975 | 2211 | 21 | core |
| `sphere` | 18 | 19 | 3 | core |
| `cone` | 17 | 23 | 1 | core |
| `torus` | 0 | 0 | 0 | core |
| `extrude` | 2534 | 2785 | 20 | core |
| `extrude_with_holes` | 19 | 19 | 2 | core |
| `extrude_tapered` | 22 | 22 | 2 | core |
| `revolve` | 78 | 104 | 7 | core |
| `loft` | 30 | 34 | 2 | core |
| `sweep` | 0 | 0 | 0 | core |

### Sketch

3 in this family — 2 used, 1 unused, 1 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `sketch` | 4 | 9 | 1 | core |
| `sketch_extrude` | 0 | 0 | 0 | catalog |
| `sketch_revolve` | 4 | 9 | 1 | core |

### Booleans

4 in this family — 4 used, 0 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `union` | 803 | 970 | 17 | core |
| `difference` | 2163 | 2536 | 22 | core |
| `intersection` | 213 | 256 | 18 | core |
| `union_all` | 663 | 699 | 17 | core |

### Features & transforms

11 in this family — 9 used, 2 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `fillet_edge_near` | 20 | 20 | 1 | core |
| `chamfer_edge_near` | 0 | 0 | 0 | core |
| `fillet_circular_rim` | 0 | 0 | 0 | core |
| `translate` | 3439 | 3753 | 20 | core |
| `rotate_z` | 159 | 178 | 9 | core |
| `rotate_x` | 1253 | 1433 | 12 | core |
| `rotate_y` | 34 | 40 | 6 | core |
| `pose` | 594 | 663 | 15 | core |
| `mirror` | 36 | 38 | 4 | core |
| `linear_pattern` | 79 | 83 | 2 | core |
| `polar_pattern` | 31 | 33 | 3 | core |

### Measures

8 in this family — 8 used, 0 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `validate` | 182 | 183 | 22 | core |
| `volume` | 23 | 23 | 2 | core |
| `exact_volume` | 209 | 210 | 18 | core |
| `mass_properties` | 93 | 93 | 21 | core |
| `bounding_box` | 167 | 167 | 20 | core |
| `wall_thickness` | 106 | 106 | 22 | core |
| `draft_analysis` | 2 | 2 | 2 | core |
| `mesh_components` | 114 | 114 | 18 | core |

### Assertions

8 in this family — 5 used, 3 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `assert` | 559 | 560 | 22 | core |
| `assert_disjoint` | 41 | 41 | 6 | core |
| `coincident_fit` | 0 | 0 | 0 | core |
| `support_report` | 160 | 160 | 20 | core |
| `clearance` | 762 | 799 | 19 | core |
| `describe` | 7 | 7 | 1 | core |
| `list_faces` | 0 | 0 | 0 | core |
| `list_edges` | 0 | 0 | 0 | core |

### Exports

3 in this family — 3 used, 0 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `export_stl` | 209 | 209 | 22 | core |
| `export_step` | 119 | 119 | 20 | core |
| `export_3mf` | 14 | 14 | 7 | core |

### Implicit / hybrid

5 in this family — 1 used, 4 unused, 1 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `gyroid_block` | 0 | 0 | 0 | catalog |
| `implicit` | 26 | 26 | 3 | core |
| `shell` | 0 | 0 | 0 | core |
| `sample_density_grid` | 0 | 0 | 0 | core |
| `mesh_density_grid` | 0 | 0 | 0 | core |

### Voxel-route solid ops & interrogation probes (2026-07-29 implicit wave)

5 in this family — 2 used, 3 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `offset_solid` | 0 | 0 | 0 | core |
| `shell_solid` | 0 | 0 | 0 | core |
| `solid_from_implicit` | 2 | 2 | 1 | core |
| `thin_wall` | 0 | 0 | 0 | core |
| `min_ligament` | 3 | 3 | 2 | core |

### Native formats

1 in this family — 1 used, 0 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `load_part` | 27 | 27 | 1 | core |

### Imports

6 in this family — 4 used, 2 unused, 1 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `measure_dimension` | 27 | 27 | 5 | core |
| `tpms` | 0 | 0 | 0 | catalog |
| `hybrid_boolean` | 1 | 1 | 1 | core |
| `import_step` | 89 | 89 | 14 | core |
| `import_mesh` | 1 | 1 | 1 | core |
| `mesh_carve` | 0 | 0 | 0 | core |

### Parts library (curated, admission-gated; BAR.md I7)

5 in this family — 0 used, 5 unused, 5 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `library_add` | 0 | 0 | 0 | catalog |
| `library_search` | 0 | 0 | 0 | catalog |
| `library_instantiate` | 0 | 0 | 0 | catalog |
| `library_deprecate` | 0 | 0 | 0 | catalog |
| `library_remove` | 0 | 0 | 0 | catalog |

### Standard parts catalog

48 in this family — 14 used, 34 unused, 33 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `spur_gear` | 8 | 8 | 1 | core |
| `hex_bolt` | 0 | 0 | 0 | catalog |
| `hex_nut` | 1 | 1 | 1 | core |
| `washer` | 17 | 17 | 5 | core |
| `socket_head_cap_screw` | 22 | 22 | 4 | core |
| `gt2_pulley` | 0 | 0 | 0 | catalog |
| `chain_sprocket` | 0 | 0 | 0 | catalog |
| `shaft` | 0 | 0 | 0 | catalog |
| `parallel_key` | 0 | 0 | 0 | catalog |
| `dowel_pin` | 27 | 28 | 6 | core |
| `circlip_external` | 3 | 3 | 2 | core |
| `circlip_internal` | 0 | 0 | 0 | catalog |
| `flat_head_screw` | 1 | 1 | 1 | core |
| `button_head_screw` | 0 | 0 | 0 | core |
| `set_screw` | 3 | 3 | 1 | core |
| `lock_nut` | 3 | 3 | 2 | core |
| `threaded_rod` | 0 | 0 | 0 | catalog |
| `standoff` | 0 | 0 | 0 | catalog |
| `compression_spring` | 6 | 6 | 1 | core |
| `extrusion_2020` | 0 | 0 | 0 | catalog |
| `extrusion_3030` | 0 | 0 | 0 | catalog |
| `tnut_2020` | 0 | 0 | 0 | catalog |
| `o_ring` | 20 | 20 | 1 | core |
| `o_ring_cord` | 4 | 4 | 1 | core |
| `jaw_coupling_hub` | 0 | 0 | 0 | catalog |
| `jaw_coupling_spider` | 0 | 0 | 0 | catalog |
| `set_screw_coupling` | 0 | 0 | 0 | catalog |
| `clamp_coupling` | 0 | 0 | 0 | catalog |
| `nema_motor` | 0 | 0 | 0 | catalog |
| `nema_mount_plate` | 0 | 0 | 0 | catalog |
| `linear_bearing_lmuu` | 0 | 0 | 0 | catalog |
| `sc8uu_block` | 0 | 0 | 0 | catalog |
| `shaft_support_sk8` | 0 | 0 | 0 | catalog |
| `shaft_support_shf8` | 0 | 0 | 0 | catalog |
| `mgn12_rail` | 0 | 0 | 0 | catalog |
| `mgn12_carriage` | 0 | 0 | 0 | catalog |
| `deep_groove_bearing` | 1 | 1 | 1 | core |
| `flanged_bearing` | 5 | 5 | 1 | core |
| `thrust_bearing` | 0 | 0 | 0 | catalog |
| `kp08_pillow_block` | 0 | 0 | 0 | catalog |
| `pipe_boss_g` | 0 | 0 | 0 | catalog |
| `hose_barb` | 0 | 0 | 0 | catalog |
| `shoulder_bolt` | 0 | 0 | 0 | catalog |
| `spring_washer` | 0 | 0 | 0 | catalog |
| `lead_screw_tr8` | 0 | 0 | 0 | catalog |
| `lead_screw_nut_tr8` | 0 | 0 | 0 | catalog |
| `gear_rack` | 0 | 0 | 0 | catalog |
| `internal_gear` | 0 | 0 | 0 | catalog |

### Standard feature cuts

13 in this family — 3 used, 10 unused, 8 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `heatset_insert_boss` | 20 | 20 | 1 | core |
| `circlip_groove_external` | 0 | 0 | 0 | catalog |
| `circlip_groove_internal` | 0 | 0 | 0 | catalog |
| `o_ring_groove` | 0 | 0 | 0 | catalog |
| `o_ring_face_gland` | 9 | 10 | 1 | core |
| `o_ring_face_gland_racetrack` | 0 | 0 | 0 | catalog |
| `nema_mount_cut` | 0 | 0 | 0 | catalog |
| `servo_pocket` | 0 | 0 | 0 | catalog |
| `tr8_nut_trap` | 0 | 0 | 0 | catalog |
| `pc4_port` | 0 | 0 | 0 | catalog |
| `teardrop_hole` | 54 | 62 | 6 | core |
| `board_mount` | 0 | 0 | 0 | core |
| `bridged_counterbore` | 0 | 0 | 0 | core |

### Design-math lookups

7 in this family — 3 used, 4 unused, 3 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `gt2_belt` | 0 | 0 | 0 | catalog |
| `gt2_center_distance` | 0 | 0 | 0 | catalog |
| `iso286_fit` | 39 | 39 | 11 | core |
| `heatset_spec` | 12 | 12 | 8 | core |
| `metric_cord_gland` | 1 | 1 | 1 | core |
| `racetrack_cord_length` | 0 | 0 | 0 | core |
| `pipe_thread_g` | 0 | 0 | 0 | catalog |

### Hole wizard

7 in this family — 4 used, 3 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `drill` | 8 | 8 | 2 | core |
| `clearance_hole` | 34 | 40 | 2 | core |
| `counterbore_hole` | 0 | 0 | 0 | core |
| `countersink_hole` | 7 | 7 | 1 | core |
| `tap_drill_hole` | 0 | 0 | 0 | core |
| `bolt_circle` | 9 | 10 | 2 | core |
| `bearing_seat` | 0 | 0 | 0 | core |

### Modelled ISO threads

3 in this family — 3 used, 0 unused, 0 behind `catalog`.

| op | `ops[]` | all | campaigns | build |
|---|---:|---:|---:|---|
| `thread_spec` | 8 | 8 | 4 | core |
| `thread_ridge` | 30 | 38 | 4 | core |
| `export_threaded` | 1 | 1 | 1 | core |

## `"op"` values that are not interpreter ops

These names also appear as `"op"` keys in the corpus, but only inside `implicit` expression
trees (scalar-field math nodes); the interpreter never dispatches them and they are not part of
the count above.

| node | occurrences |
|---|---:|
| `abs` | 34 |
| `add` | 34 |
| `atan2` | 34 |
| `length2` | 102 |
| `max` | 136 |
| `mod` | 34 |
| `mul` | 68 |
| `rotate` | 8 |
| `sub` | 272 |

## The gated list (machine-readable: `kernel_api::CATALOG_OP_NAMES`)

```
nema_motor
nema_mount_plate
nema_mount_cut
gt2_pulley
gt2_belt
gt2_center_distance
lead_screw_tr8
lead_screw_nut_tr8
tr8_nut_trap
thrust_bearing
linear_bearing_lmuu
jaw_coupling_hub
jaw_coupling_spider
set_screw_coupling
clamp_coupling
o_ring_groove
o_ring_face_gland_racetrack
extrusion_2020
extrusion_3030
tnut_2020
hex_bolt
shoulder_bolt
threaded_rod
standoff
spring_washer
circlip_internal
circlip_groove_external
circlip_groove_internal
shaft
parallel_key
pipe_boss_g
pipe_thread_g
hose_barb
pc4_port
servo_pocket
mgn12_rail
mgn12_carriage
sc8uu_block
shaft_support_sk8
shaft_support_shf8
kp08_pillow_block
gear_rack
internal_gear
chain_sprocket
library_add
library_search
library_instantiate
library_deprecate
library_remove
gyroid_block
tpms
sketch_extrude
```

## Campaign directories in the corpus

```
acoustics_system/screw_on_exponential_horn
aerospace_system/cubesat_1u_dev_frame
agriculture_system/jar_top_seed_singulator
assistive_system/ratcheting_cap_wrench
automotive_system/rotor_runout_gauge_bridge
biomedical_system/prosthetic_wrist_quick_disconnect
electronics_system/din_rail_pi4_enclosure
energy_system/turgo_runner
framework_system/l12_mini_case
gearbox
home_it_system/demo_knob
home_it_system/m6_hex_fastener_pair
horology_system/graham_deadbeat_escapement
hydroponics_system/reservoir_topoff_float_valve
laboratory_system/slas_microplate_row_index_stage
magic_system/uphill_roller
marine_system/folding_deck_cleat
optics_system/ball_kinematic_mirror_mount
rail_system/ls45_turnout_throw_lock
robotics_system/iso9409_wedge_flexure_gripper
school_system/folding_book_stand
school_system/rated_desk_hook
showcase
```
