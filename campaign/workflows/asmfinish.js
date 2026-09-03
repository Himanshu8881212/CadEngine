export const meta = {
  name: 'lmcad-assembly-finish',
  description: 'Finish the three remaining assembly campaigns (prove + document + self-check) from their on-disk state',
  phases: [
    { title: 'Prove', detail: 'close kinematics, negative controls, tolerance chains, physics, optimization' },
    { title: 'Finish', detail: 'assembly doc, BOM, renders, generated ANALYSIS/README, section-5 self-check' },
  ],
}

const REPO = '/Users/himanshu/Work/New-LMCAD/cad engine'

// ONLY the three incomplete campaigns. slas_microplate_row_index_stage and
// ls45_turnout_throw_lock are COMPLETE and self-checked — they are deliberately
// absent so no agent is ever dispatched into them again.
const TARGETS = [
  { dir: 'horology_system', name: 'graham_deadbeat_escapement',
    state: 'members built (5 STL + 5 STEP, 24 programs, ~20 receipts) but NO renders and no generated docs — the prove stage is largely unrun. BUILD_LOG ends mid-note about sliver mints and thread export limits.' },
  { dir: 'assistive_system', name: 'ratcheting_cap_wrench',
    state: 'members built (5 STL + 5 STEP) with ~127 receipts across 48 programs — prove work is substantially done and a stage-4 re-run had started (5 part programs + 5 STEP round-trips, exit 0, zero warnings). Audit, close gaps, then document.' },
  { dir: 'hydroponics_system', name: 'reservoir_topoff_float_valve',
    state: 'BUILD_LOG says STAGE 3 DONE: 6 STL + 6 STEP, ~100 receipts across 102 programs, 8 renders already present. Most likely only documentation + the section-5 self-check remain.' },
]

const LAW = (t) => `
ASSEMBLY CAMPAIGN: ${t.name}. Directory: "${REPO}/${t.dir}/${t.name}/"
Repo root has a SPACE — quote every path.
Engine CLI: "${REPO}/target/release/kernel-api" run <prog.json> --out-dir <dir>   |   ... asm <file.lmcasm> --out-dir <dir>
Python tools: python3 "${REPO}/tools/<tool>.py" job.json

ON-DISK STATE WHEN THIS RUN WAS PLANNED: ${t.state}
**AUDIT THE DIRECTORY FIRST — it is the source of truth, not this description.** Read programs/BUILD_LOG.md, list programs/ and receipts/, and re-run what you must to learn what already passes. NEVER redo work that is already complete and receipted; continue from the frontier. Append one dated line to programs/BUILD_LOG.md per meaningful step — you may be killed by a usage limit at any moment and a successor reads that log.

MANDATORY READS: "${REPO}/campaign/OPERATOR_BRIEF.md" (doctrine, the universal 'require' gate param, the 3-code tool exit contract), "${REPO}/campaign/DELIVERABLE_SPEC.md" (section 2 gates incl. asserting BOTH shells and components, section 3 honesty rules, section 4 friction protocol, section 5 self-check), and your own analysis/DESIGN.md (the frozen dimensions, analysis plan, negative-control plan).

DEFECT CLASSES FROM THE PART ROUND — DO NOT REPEAT (see campaign/history/PART_PORTFOLIO_VERDICT.md sections 3 and 5):
- hand-typed literals smuggled into "generated-from-receipts" documents (the #1 defect: every number in ANALYSIS.md/README.md must flow from a receipt file, and say which);
- claims that contradict their own cited receipt;
- gates that cannot fail (one shipped oracle measured a faceter artefact and passed vacuously — verify each oracle actually FIRES);
- creep gated at 23 C for parts that run hotter (production_check takes a duration argument — use it, at the service temperature);
- "Reproducing" sections that do not reproduce.

HARD RULES:
- NEVER edit "${REPO}/crates/", "${REPO}/tools/", the root docs, or ANY other campaign directory. In particular do NOT touch laboratory_system/ or rail_system/ — those campaigns are closed.
- Engine/tool issues -> append a structured entry to "${REPO}/campaign/friction/${t.name}.md" and work around inside your own directory.
- No mutating git commands.
- Receipts, not claims. Record refusals verbatim. Never delete a failing gate: an honest failure SHIPS as a condition limit or a required process step.
- PLA on a Bambu-class printer (0.4 nozzle, 256 mm bed, HDT ~54 C, creep is real). Nothing has been physically printed — say so.
- KNOWN KERNEL DRIFT (do not fight it): kernels built on/after 2026-08-10 refuse the FINAL merged scene-export op of an assembled attitude when exact-contact seats or designed interference put self-intersections in the merged scene soup. Every MEASURE gate still runs and reproduces first. If you hit it: record the verbatim refusal as an ATTEMPTED receipt, annotate the Reproducing section, and proceed — the interference/clearance numbers are the evidence, not the scene STL. A fix is staged for the maintainer's re-baseline.

OUTPUT DISCIPLINE: keep every response small; never hand-write a file over ~150 lines — emit it from a small stdlib-only Python generator in programs/ (the generator ships as the reproducible source). Reach a minimal complete pass first, then improve. Persist every artifact the moment it works.
`

const STATUS = {
  type: 'object',
  properties: {
    assembly: { type: 'string' },
    stage: { type: 'string' },
    ok: { type: 'boolean', description: 'true only if this stage contract is FULLY met' },
    blocked_on: { type: 'string' },
    already_complete: { type: 'array', items: { type: 'string' }, description: 'what you found already done on disk and did NOT redo' },
    work_done: { type: 'array', items: { type: 'string' }, description: 'what this run actually produced' },
    gates_passing: { type: 'array', items: { type: 'string' } },
    gates_failing: { type: 'array', items: { type: 'string' }, description: 'empty for ok:true EXCEPT deliberately-shipped honest failures, which must be listed here with their explanation' },
    key_numbers: { type: 'array', items: { type: 'string' }, description: 'load-bearing measured numbers with units and receipt path' },
    friction_count: { type: 'number' },
  },
  required: ['assembly', 'stage', 'ok', 'blocked_on', 'already_complete', 'work_done', 'gates_passing', 'gates_failing', 'key_numbers', 'friction_count'],
}

phase('Prove')
log('Closing the prove stage on the three remaining assemblies (disk-state driven)...')

const results = await pipeline(
  TARGETS,

  (t) => agent(`${LAW(t)}
YOU ARE THE PROVE STAGE. Bring this campaign to a fully proven state. Skip anything already receipted; close everything else.

1. PER-MEMBER GATES (each printed member, in-program with 'require'): validate (+ genus/shells asserted), assert shells AND assert components, exact_volume_within against a closed form you compute, wall_thickness, support_report at the declared build_dir, bounding_box envelope require fits_within, export_stl require watertight+route, export_step + import_step round-trip within 2.5%. ZERO warnings anywhere.
2. ASSEMBLY: the members placed with mates; mate residual within the runner's gate; read the DOF block and state honestly whether the assembly is well- or under-constrained and why that is correct for a mechanism with intended freedoms. asm_contacts for designed seats; clearance between every non-contacting pair; float exact-contact poses by 0.1 mm.
3. KINEMATICS: posed station sweeps across the full motion range. A sweep proves FREE MOTION ONLY — it is blind to steady interference, so every must-NOT-fit claim lives on exact overlap_volume in the posed failure attitude.
4. NEGATIVE CONTROLS: for each 'cannot' claim, pose the failure attitude and measure the interference (overlap_volume > 0) AND pose the legal path and measure the clearance — ship BOTH numbers. Include at least one ORACLE control proving a gate can fail, and VERIFY IT FIRES this run.
5. TOLERANCE: tolerance_stack CHAIN across >=3 stacked members plus FIT for every interface at +/-0.15 mm extremes. Running fits need >=0.3 mm nominal clearance or a justified printed coupon.
6. PHYSICS: execute the DESIGN.md analysis plan (jobs in programs/, receipts in receipts/), each with its error band and honest tier (query analyzer_registry, never a solver README). Sustained loads gate on the creep table AT THE SERVICE TEMPERATURE AND DURATION. Derate the allowable x0.55 out-of-plane where the load is out-of-plane, and say where. Run every planned refusal and record it verbatim.
7. OPTIMIZATION: run it, bake the optimum back into the geometry, and RE-RUN the full gate suite — the shipped artifacts must be the optimized ones, re-receipted.

Return the structured status.`, { label: `prove:${t.name}`, phase: 'Prove', schema: STATUS, effort: 'high' }),

  (_prev, t) => agent(`${LAW(t)}
YOU ARE THE FINISH STAGE. The prove stage has run; **read the directory to see what it left** (BUILD_LOG.md, receipts/, programs/) rather than assuming. Close any remaining prove-stage hole first — an unrun analysis row or an un-regated optimum is a hole.

1. DOC PACK: assembly_doc (exploded view, ballooned BOM, numbered assembly sequence) + instructions.md; bom_audit and production_dossier for the BOM and print pack; renders via render_views/render_sheet into renders/. 'date' is a literal string input — never the clock, so re-renders are byte-identical.
2. programs/gen_analysis.py (stdlib only) reads receipts/ and REGENERATES analysis/ANALYSIS.md. ZERO hand-typed measurement numbers; every number cites the receipt it came from. Add a guard that fails loudly if a receipt it depends on is missing. Run it.
3. README.md: the mechanism told through measured numbers, contents table, per-member print settings, interface standards with sources, assembly sequence summary, honest limits, and a mandatory "What has NOT been done" section that states plainly that NOTHING HAS BEEN PRINTED. Generate it the same way if it carries numbers; extract the Reproducing section so README and ANALYSIS cannot drift.
4. SELF-CHECK — DELIVERABLE_SPEC section 5, top to bottom, executed FRESH this run: every README Reproducing command run from the repo root; every committed STL/STEP rebuilt byte-identical (cmp); zero warnings; every negative control fired with its number quoted; docs byte-stable across two consecutive regenerations. Fix what fails. Remove orphan scratch files (__pycache__, .DS_Store, temp jobs).

Return the structured status. ok:true ONLY if the section-5 self-check fully passed; list deliberately-shipped honest failures in gates_failing with their explanation. key_numbers: the headline receipts a listing would quote.`, { label: `finish:${t.name}`, phase: 'Finish', schema: STATUS, effort: 'high' }),
)

const done = results.filter(r => r && r.ok).length
log(`${done}/${TARGETS.length} remaining assemblies now report a fully passed self-check.`)
return { targets: TARGETS.map(t => t.name), results }
