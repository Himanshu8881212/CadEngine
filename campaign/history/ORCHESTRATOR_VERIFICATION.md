# Orchestrator independent verification — 2026-08-08

Re-verification of the fix phase by the orchestrator, run independently of the
integrator agent (whose own first harness produced a false green, so its report
was not taken on trust). Every number below was produced by commands run
directly, not quoted from an agent.

## Engine health

| check | command | result |
|---|---|---|
| workspace tests | `cargo test --workspace --release` | **160 suites, 1073 tests, 0 failed** |
| lints | `cargo clippy --workspace --all-targets --release` | **0 warnings / 0 errors** |
| doc drift | `python3 tools/audit_docs.py` | **exit 0** (18 findings, 0 at error severity) |
| campaign dirs untouched by fixes | `git status --porcelain -- '*_system/' showcase/` | **empty** |

## Campaign regression (all 10 parts + shipped showcase)

Harness: `scratchpad/regress.py` — runs every file under `*_system/*/programs/`
and `showcase/squatchee_spin/programs/` that is a real kernel-api program
(top-level `ops` array), then byte-compares every produced `.stl` / `.3mf` /
`.step` against the committed artifact of the same name.

```
programs run ........ 111
byte-identical ...... 93
bytes differ ........  0     <-- the load-bearing number
report warnings ..... 0      <-- across all 111 programs
```

**No geometry regression and no unknown-param warnings anywhere.** The fix phase
was report/tool/gate work, and the bytes confirm it stayed that way.

## Three harness artifacts, recorded so they are not mistaken for defects

My first two harness attempts were themselves the broken thing. Recorded because
the same traps will catch the next person:

1. **Feeding Python job files to `kernel-api`.** `prodcheck_*`, `t_*` (thermal),
   `vox_*`, `joint_check`, `air_*`, `field_probe_*` are tool jobs, not engine
   programs. Filter on a top-level `ops` array.
2. **Isolated `--out-dir` breaks programs that round-trip a file.** 5 programs
   reported `io` / `invalid_geometry` under a temp out-dir. Cause: a program
   writes `programs/_rt_x.step` (resolved against `--out-dir`) then reads
   `_rt_x.step` back (resolved against the PROGRAM's directory). That is
   friction theme **T4, path/root asymmetry** — still open after the fix phase.
   Re-run with `--out-dir` = the part directory, exactly as each README's
   Reproducing section specifies, and all 5 pass:
   `kernel-api run optics_system/ball_kinematic_mirror_mount/programs/part_frame.json --out-dir optics_system/ball_kinematic_mirror_mount` → `ok: true`, working tree unchanged.
3. **"A negative control must exit non-zero" is WRONG for this repo.** 16
   programs were flagged by that assumption. The house pattern poses the failure
   attitude and proves the interference *positively* — `union` + `assert
   shells==1` (the bodies weld into one shell) plus an `overlap_volume` measure —
   so the program exits **0** and the interference number IS the receipt.
   Verbatim from `marine_system/folding_deck_cleat/programs/nc_interference.json`:
   *"Every cannot-claim is posed in its FAILURE ATTITUDE and measured on exact
   overlap_volume, then enforced with union+assert shells==1. Legal twins are
   asserted disjoint."* An NC that exits non-zero is the `assert_failed` style;
   both are legitimate and a checker must handle both.

## One real observation

`biomedical_system/prosthetic_wrist_quick_disconnect/programs/sweep_motion.json`
did not finish within **420 s** (and exceeded 900 s on the first attempt). It is
a shipped campaign program. Not a regression — but a motion sweep that costs
>7 minutes deserves either a documented cost or a cheaper station count, and it
is the kind of runtime that pushes an operator toward skipping the sweep.
Logged for the maintainer; not fixed here.
