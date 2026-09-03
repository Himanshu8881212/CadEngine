# friction — rated_desk_hook

## F1 — tools_cookbook.md production_check example predates the creep-table gate (2026-08-27)
- symptom: `production_check.py` with `load_character.sustained: true` and no
  duration returned `ok:false`, creep rule `allowable_mpa: 0.0, SF: 0.0`,
  `refusal_kind: "creep_duration_required"`, exit 2 — verbatim detail:
  "sustained load declared but the job states NO design duration
  (`duration_h`, in hours)".
- minimal repro: `{"material":"PLA","max_von_mises_pa":2330000.0,
  "load_character":{"sustained":true},"service_temp_c":23}` →
  `python3 tools/production_check.py job.json`.
- expected vs actual: `campaign/digests/tools_cookbook.md` ("VERIFIED
  gotcha: PLA at 8 MPa sustained → creep allowable is 55×0.2 = 11 MPa, SF
  1.38") and its job schema list no `duration_h` key; the live tool now
  REQUIRES `duration_h` when sustained and gates on the
  `creep.sig_allow_mpa` table, refusing the legacy scalar. The tool's own
  docstring documents the new behavior; the digest is stale.
- workaround used: added `duration_h: 24` (the rating condition) to the job;
  campaign proceeded. Tool behavior is BETTER than the digest — this entry
  is doc drift, not a bug.

## F2 — MCP render_views sandbox rejects campaign paths (2026-08-27)
- symptom: `mcp__lmcad__render_views` with
  `stl: school_system/rated_desk_hook/parts/rated_desk_hook_desk29.stl` →
  "stl '...' not found under the out dir '.../studio_out/mcp'".
- minimal repro: call the MCP tool with any repo-relative path outside
  `studio_out/mcp`.
- expected vs actual: the MCP tool description does not state that `stl`
  must live under `studio_out/mcp`; the CLI twin `tools/render_sheet.py`
  takes CWD-relative paths.
- workaround used: `cp` the STL into `studio_out/mcp/` first (fine), and
  used `tools/render_sheet.py` for the shipped render.

---

## RESOLUTIONS (2026-08-27)

- **F1**: `campaign/digests/tools_cookbook.md` production_check section
  updated — `duration_h` required for sustained loads, the creep TABLE
  governs, the 11 MPa scalar is documented as `legacy_scalar_mpa` only.
- **F2**: `render_views` (studio/mcp) now resolves `stl` against the out dir
  AND the repo root (read-only), and its miss error names both roots and the
  fix. A campaign's `parts/*.stl` renders without copying.
- **F2 SUPERSEDED (2026-09-03)**: `studio/` — the HTTP server, the web IDE,
  `lmcad-tui` and the `lmcad-mcp` MCP server — was removed from the
  repository. There is no MCP surface and no `studio_out/mcp` sandbox any
  more, so the sandbox class of failure cannot recur. Render views with the
  CLI twins, which take ordinary CWD-relative paths:
  `python3 tools/render_views.py job.json` and
  `python3 tools/render_sheet.py job.json`.

---

## OPEN — the campaign is RED against current main (2026-09-03)

- symptom: `sh school_system/rated_desk_hook/run_all.sh` dies at its first
  gate, `hook_desk19`:
  `op 'g_walls': require failed: thin_area: measured 0.40999965369701385, expected 0.0`
  (`set -e`, so nothing after it runs).
- minimal repro: `kernel-api run school_system/rated_desk_hook/programs/hook_desk19.json`
- cause, not a regression from the 2026-09-03 tree cleanup: the
  `wall_thickness` sampler became area-uniform and stratified in the same-day
  fix wave (see docs/CHANGELOG.md, F4), and that wave's own note says coarse
  boolean bodies now report thin bands the old centroid-per-triangle sampler
  missed — "a measurement improvement, not a geometry change". The campaign's
  `thin_area: 0.0` gate was baselined against the OLD sampler.
- verified pre-existing: a `kernel-api` built from commit `5a70984` (before
  the studio/reference/legacy removals) returns the SAME measured value,
  0.40999965369701385, and the same exit 1. The removals changed no Rust code.
- what it needs: a re-baseline of this campaign against the current sampler —
  re-measure `thin_area`, decide whether 0.41 mm² of genuinely thin band is
  acceptable for the part, and either fix the geometry or restate the gate
  (`exclude_wedge_deg` moves acute-wedge readings into `thin_area_wedge` if
  the reading is a lip rather than a wall). Not attempted here: re-baselining
  a campaign is a design decision, not a cleanup.
- the other three campaigns are green on current main: `l12_mini_case`
  (ALL GREEN), `uphill_roller` (ALL GATES GREEN), `folding_book_stand`
  (ALL GATES GREEN).
