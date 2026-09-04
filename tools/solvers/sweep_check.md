# sweep_check — does the mechanism move through its range without interfering?

**Runner**: `tools/analyzers/sweep_check.py` ·
`python3 tools/analyzers/sweep_check.py job.json [--out PATH]` (the
pre-2026-09-02 path `tools/sweep_check.py` is a forwarding shim).
**Gates**: none — see Benchmark gates and Validity limits.
**Status**: no gate suite. **Tier**: **Demonstrated**
(`python3 tools/analyzer_registry.py --tier sweep_check` →
`{"tier": "Demonstrated", "gate_suite": null, "manifest": null, "pins": []}`,
`tier_reason`: "Demonstrated by the registry rule: Validated requires a present
manifest AND a present validation pin").

A static clearance measure answers "do these two bodies clash in THIS pose".
Almost every assembly failure is about a different question: a shaft that enters
the bore but binds 4 mm in, a cam that clears its follower at rest and hits it
at 130°, a lid that fits closed and fouls the hinge boss halfway open. This tool
answers that one by running the ACTUAL engine at a series of poses and
collecting the clearance measures at each.

It covers both sweep classes with one receipt shape: **insertion** sweeps, where
`t` is position along an insertion axis, and **motion** sweeps, where `t` is a
joint angle or pose parameter.

## What it actually computes

No geometry kernel of its own. The job carries a normal LMCAD work-order
`template` whose ops contain the literal string `"$t"` wherever the sweep
parameter goes — any string equal to `"$t"`, including inside a coordinate
array, is replaced by the numeric station value (the same substitution
semantics as `param_optimize`, and literally
`param_optimize.substitute` from `tools/analyzers/param_optimize.py`).

Stations are an inclusive linspace: `t.from` to `t.to` in `t.steps` samples
(`n == 1` gives just `t.from`). `MAX_STEPS = 200`.

At each station the substituted program is materialised via
`param_optimize.station_dir` — `job["program_dir"]`, else `job["out_dir"]`, else
the job file's own directory, **never a system temp dir** — and run one-shot
through `param_optimize.call_engine`, i.e. the `kernel-api` CLI. Each id in
`watch` is looked up in the report:

- a `clearance` op interferes at `t` iff its `interfering` measure is true;
- a `coincident_fit` op "interferes" iff `coincident_fit` is true (the press-fit
  / coincident-surface hazard), so one receipt shape covers both.

Per watch the tool then reduces the station table to `min_distance` and the `t`
it occurred at, `first_interfering_t` in sweep order, the contiguous
`interfering_t_ranges`, the station count, and `all_stations_interfering`. A
station whose program failed is recorded in `failed_stations` and skipped, not
silently dropped.

Determinism: the engine is run-to-run deterministic (R5) and stations are
visited in order, so the CSV tables are reproducible.

**What a sweep proves.** It is a SAMPLED FREE-MOTION proof. `ok: true` means no
EVALUATED station interfered. It cannot support a must-NOT-fit claim, and the
runner says so on every receipt in `sweep_semantics`. See Validity limits.

## The contract (job → receipt)

Job: `template` (`{"ops": [...]}` with `"$t"` placeholders), `t`
(`{from, to, steps}`, `steps` 1..200), `watch` (non-empty list of clearance /
coincident_fit op ids), `out_dir` (required — the per-watch CSV tables land
there), optional `program_dir`.

Receipt (LAST non-empty stdout line; persisted to
`<out_dir>/sweep_check_receipt.json`, or `--out`, or the job's `receipt` path,
per `tools/_receipt.py`): `ok` (false iff any watch interferes anywhere OR any
station failed), `stations`, `t_from`, `t_to`, `watches` (per id:
`min_distance`, `min_distance_t`, `first_interfering_t`,
`interfering_t_ranges`, `stations`, `all_stations_interfering`, `table_path`),
`failed_stations` (**ALWAYS a list**, empty when none — an absent sibling key
read as "nothing failed", din_rail F3), `interfering_watches`, and
`sweep_semantics`. Each watch also writes
`<out_dir>/<id>_sweep.csv` with columns
`t, distance, interfering, overlap_volume, coincident_fit_hazard`.

Refusals, by exact `error_kind` string:
`refusal.no_free_station` (every evaluated station of every watch interferes —
see below), `refusal.missing_t`, `refusal.bad_steps`, `refusal.no_watch`,
`refusal.missing_out_dir`, `refusal.watch_not_found`,
`refusal.watch_not_a_clearance_op`.

`refusal.no_free_station` is the load-bearing one. A sweep in which no watch
ever saw a clear station observed no free motion at all, so it proved nothing —
and its receipt (`ok:false`, every station interfering) is exactly the shape a
reader mistakes for a proof of interference. It is refused by name instead of
returned as a tidy `ok:false`.

Exit codes follow the shared contract in `tools/_receipt.py`: **0** `ok:true`,
**1** the tool could not run the request, **2** it ran and REFUSED or the
analysis failed. Both signals always agree.

## Benchmark gates (measured, frozen)

**None. This analyzer has no benchmark gate suite.** The registry records
`gate: no` for it (`python3 tools/analyzer_registry.py --tier sweep_check`
returns `"gate_suite": null`); there is no manifest under `tools/manifests/` and
no validation script under `tools/validation/`.

What exists is a set of **behavioural pins** in `tools/tests/test_checkers.py`
(`python3 tools/tests/test_checkers.py`, section `sweep_check.py`), run on two
boxes where one slides along +X over 5 stations. They pin:

- a clear sweep returns `ok:true` at exit 0, with `failed_stations == []` and
  `interfering_watches == []`;
- the receipt states what a sweep can and cannot prove (`sweep_semantics`
  contains "free-motion proof", "does not fit", `exact_volume`);
- an all-stations-interfering sweep is `refusal.no_free_station` at exit 2 with
  `all_stations_interfering: true` — **not** a tidy `ok:false`;
- a missing `t` block is `refusal.missing_t` at exit 2, not a `KeyError`.

Two things to be honest about. First, these are exit-code and receipt-shape
pins: **there is not one numeric band among them** — no distance, no tolerance,
no convergence figure is checked against an independent reference. Second, the
whole section is **engine-backed and SKIPS, loudly, when
`target/release/kernel-api` is not built**; in this checkout the binary is
absent and the suite prints `sweep_check pins (SKIPPED: target/release/kernel-api
not built)` and counts it as a pass. A green `test_checkers.py` therefore does
not by itself mean the sweep pins ran — check the line.

The accuracy of the underlying `clearance` / `distance` measures is the
engine's, not this tool's; nothing here pins it.

## Validity limits / out of scope

- **A sweep proves FREE MOTION ONLY. It is blind to STEADY interference.** This
  is the tool's known blind spot, stated in `campaign/OPERATOR_BRIEF.md`
  ("Sweeps prove free motion ONLY — blind to steady interference. Must-NOT-fit
  claims belong on exact `overlap_volume` in the posed failure attitude") and
  logged as **`campaign/friction/ENGINE.md` #27** (2026-07-31, severity minor,
  with a concrete repro): a body that interferes at EVERY pose never produces a
  free/interfering transition, so there is nothing for the sweep to see. The
  recorded repro — a DRILL HOOK negative control, a grip gauge 4 mm thicker than
  its channel lowered through **13 poses**, interfering **1 mm per side** from
  the moment it enters — reports **crossings = 0** from the sweep while
  `overlap_volume` on the seated pose reports **1680 mm³**. Those numbers belong
  to that logged repro; they are not a gate in this repo. The runner's own
  `refusal.no_free_station` closes the specific case where NO watch ever sees a
  clear station, but it cannot rescue a sweep where one watch is clear and
  another is uniformly bad. **Anything asserting that something does NOT fit
  belongs on the exact oracle: `intersection` + `exact_volume` in the posed
  failure attitude.**
- **It is SAMPLED.** Between two clear stations anything may happen. `ok:true`
  supports "these `stations` poses are clear" and nothing stronger. No study in
  this repo establishes how many stations are enough for a given geometry, or
  bounds the worst-case interference that can hide between two clear samples —
  that resolution/aliasing limit is **not characterised**, so do not gate a
  clearance claim on a station count.
- **`min_distance` is the minimum over SAMPLED stations**, not the true minimum
  of the motion. Treat it as an upper bound on the real minimum clearance, and
  do not quote it as a design clearance.
- **`steps <= 200`**, hard (`MAX_STEPS`). One parameter only: `$t` is scalar, so
  a genuinely two-DOF motion needs a different tool or a nested campaign of
  sweeps.
- **Kinematics are the author's.** The template encodes the pose as a function
  of `t`; nothing checks that this function is the real mechanism's path. A
  wrong parameterisation gives a clean, wrong, `ok:true`.
- **No dynamics, no compliance, no friction, no gravity.** Rigid poses only.
  Interference is geometric, so a part that clears rigidly but binds because it
  deflects is outside this tool.
- **Relative-path caveat.** Station programs resolve relative
  `import_step`/`load_part` paths against `program_dir`/`out_dir`/the job's own
  directory. Note that `campaign/OPERATOR_BRIEF.md` still describes the older
  system-temp behaviour that gripper F4 / turgo F7 / rotor F11 closed; the
  runner source and `param_optimize.station_dir` are current.

## When to use it

Whenever motion, not just fit, is the requirement: shaft-into-bore and
pin-into-slot insertions, hinges and lids through their full swing, cams and
followers, drawer slides, latch actuation, a gripper through its jaw range, an
arm link through its joint limits. Sweep it, keep the CSV, and quote the
station table — and when the claim you need is "this must NOT go in", stop and
use `intersection` + `exact_volume` on the posed failure attitude instead.
