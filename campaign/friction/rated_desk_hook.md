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
