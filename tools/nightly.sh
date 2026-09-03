#!/bin/sh
# LMCAD nightly self-exercise — full test suite, clippy, every Python contract
# gate under tools/ and docs/, and the generated-table-in-step check. (The two
# Rust campaign gate suites, respool and drybox_roller, ran here until 2026-09;
# their sources are parked uncompiled in legacy/kernel-model-examples/.)
# Re-proves every machine-verified
# claim in the repo, on both sides of the language line.
# Writes telemetry/nightly/YYYY-MM-DD.md (table + FAIL lines + tails of failing
# logs) and appends one summary line to telemetry/nightly/history.jsonl.
#
# Tolerant of individual failures: every step runs, results are collected, and
# the script exits non-zero if ANY step failed (so cron mail/logs show red).
# Dependencies: sh + coreutils + grep + cargo only. Run from anywhere; it cd's
# to the repo root itself.

cd "$(dirname "$0")/.." || exit 1

# Opt this run into the engine event log (kernel_core::telemetry — writes
# telemetry/engine_log.jsonl; failures land in docs/friction_inbox.jsonl
# regardless, for the lmcad-lessons harvest).
LMCAD_TELEMETRY=1
export LMCAD_TELEMETRY

DATE=$(date +%Y-%m-%d)
NOW=$(date '+%Y-%m-%d %H:%M:%S')
OUT_DIR="telemetry/nightly"
REPORT="$OUT_DIR/$DATE.md"
HISTORY="$OUT_DIR/history.jsonl"
mkdir -p "$OUT_DIR" || exit 1
LOGS=$(mktemp -d "${TMPDIR:-/tmp}/lmcad-nightly.XXXXXX") || exit 1

FAILURES=0

# ---- 1. full test suite ----------------------------------------------------
cargo test --workspace --release >"$LOGS/tests.log" 2>&1
TEST_EXIT=$?
SUITES_OK=$(grep -c '^test result: ok\.' "$LOGS/tests.log")
SUITES_TOTAL=$(grep -c '^test result:' "$LOGS/tests.log")
if [ "$TEST_EXIT" -eq 0 ]; then
	TEST_STATUS=PASS
else
	TEST_STATUS=FAIL
	FAILURES=$((FAILURES + 1))
fi

# ---- 2. clippy (repo rule: stays clean, so warnings count as failure) ------
cargo clippy --workspace --all-targets >"$LOGS/clippy.log" 2>&1
CLIPPY_EXIT=$?
# Per-lint warning lines, excluding cargo's "generated N warnings" roll-ups.
CLIPPY_WARNINGS=$(grep '^warning' "$LOGS/clippy.log" | grep -cv 'generated [0-9][0-9]* warning')
if [ "$CLIPPY_EXIT" -eq 0 ] && [ "$CLIPPY_WARNINGS" -eq 0 ]; then
	CLIPPY_STATUS=PASS
else
	CLIPPY_STATUS=FAIL
	FAILURES=$((FAILURES + 1))
fi

# ---- 3. Python contract gates ----------------------------------------------
# The tools/ half of the repo carries hundreds of executable contracts —
# checker pins, aux-tool pins, doc contracts, the cross-language creep vectors,
# the runner exit/receipt contract, the doc-drift audit. Until 2026-08-08 the
# nightly ran none of them: `cargo test` cannot see a single one, so every
# Python-side guarantee was unwatched and a regression there would surface only
# when a campaign tripped over it. That is the same silence this repo forbids
# in its tools, one level up.
#
# DISCOVERED, not enumerated: every `tools/tests/test_*.py` and `docs/test_*.py`
# is a suite by naming convention, so a suite added tomorrow is watched tomorrow
# without editing this file (tools/tests/ since the 2026-09-02 re-organisation;
# the shims left at tools/test_*.py are pointers, not suites, and are not run
# twice). The gates that need arguments are listed after. Each must exit 0;
# anything else is a failure with its own row.
PY_GATES=""
for f in tools/tests/test_*.py docs/test_*.py; do
	[ -f "$f" ] && PY_GATES="$PY_GATES $f"
done
PY_GATES="$PY_GATES tools/audit_docs.py"
PY_GATES="$PY_GATES tools/tests/materials_crosslang_test.py"
PY_GATES="$PY_GATES tools/analyzer_registry.py::--check"
PY_GATES="$PY_GATES tools/analyzer_registry.py::--check-contract"
PY_GATES="$PY_GATES tools/analyzers/materials.py::--selftest"
PY_GATES="$PY_GATES tools/analyzers/production_check.py::--selftest"
# The hermetic (non-ACE) validation pins of the Validated rules engines.
PY_GATES="$PY_GATES tools/validation/tolerance_stack_validation.py"
PY_GATES="$PY_GATES tools/validation/production_check_validation.py"
PY_GATES="$PY_GATES tools/validation/production_dossier_validation.py"

PY_TOTAL=0
PY_OK=0
PY_FAIL_ROWS=""
for gate in $PY_GATES; do
	script=${gate%%::*}
	arg=""
	case "$gate" in *::*) arg=${gate##*::} ;; esac
	label=$(basename "$script")${arg:+ $arg}
	log="$LOGS/py_$(printf '%s' "$label" | tr -c 'A-Za-z0-9' '_').log"
	if [ -n "$arg" ]; then
		python3 "$script" "$arg" >"$log" 2>&1
	else
		python3 "$script" >"$log" 2>&1
	fi
	rc=$?
	PY_TOTAL=$((PY_TOTAL + 1))
	if [ "$rc" -eq 0 ]; then
		PY_OK=$((PY_OK + 1))
	else
		PY_FAIL_ROWS="$PY_FAIL_ROWS
$label: exit $rc — $(tail -n 3 "$log" | tr '\n' ' ')"
	fi
done
if [ "$PY_OK" -eq "$PY_TOTAL" ]; then
	PY_STATUS=PASS
else
	PY_STATUS=FAIL
	FAILURES=$((FAILURES + 1))
fi

# ---- 4. generated tables are in step with their source ---------------------
# `crates/kernel-api/src/discover.rs` is GENERATED from program.rs. Hand-editing
# one without the other is invisible to every other gate here, so regenerate and
# require that nothing moved.
cp crates/kernel-api/src/discover.rs "$LOGS/discover.before" 2>/dev/null
python3 tools/gen_discover.py >"$LOGS/gen_discover.log" 2>&1
GEN_EXIT=$?
if [ "$GEN_EXIT" -eq 0 ] && cmp -s crates/kernel-api/src/discover.rs "$LOGS/discover.before"; then
	GEN_STATUS=PASS
	GEN_DETAIL="regeneration changed nothing"
else
	GEN_STATUS=FAIL
	GEN_DETAIL="gen_discover exit $GEN_EXIT; discover.rs is NOT what program.rs generates"
	FAILURES=$((FAILURES + 1))
fi

# ---- report ----------------------------------------------------------------
if [ "$FAILURES" -eq 0 ]; then OVERALL=pass; else OVERALL=fail; fi

{
	printf '# LMCAD nightly — %s\n\n' "$DATE"
	printf 'run at %s from `%s`\n\n' "$NOW" "$(pwd)"
	printf '| step | status | detail |\n'
	printf '|---|---|---|\n'
	printf '| cargo test --workspace --release | %s | exit %s, %s/%s suites ok |\n' \
		"$TEST_STATUS" "$TEST_EXIT" "$SUITES_OK" "$SUITES_TOTAL"
	printf '| cargo clippy --workspace --all-targets | %s | exit %s, %s warnings |\n' \
		"$CLIPPY_STATUS" "$CLIPPY_EXIT" "$CLIPPY_WARNINGS"
	printf '| python contract gates | %s | %s/%s gates exit 0 |\n' \
		"$PY_STATUS" "$PY_OK" "$PY_TOTAL"
	printf '| discover.rs in step with program.rs | %s | %s |\n' \
		"$GEN_STATUS" "$GEN_DETAIL"
	printf '\noverall: **%s** (%s failing step(s))\n' "$OVERALL" "$FAILURES"
	if [ -n "$PY_FAIL_ROWS" ]; then
		printf '\n## failing python gates\n\n```%s\n```\n' "$PY_FAIL_ROWS"
	fi
} >"$REPORT"

# Any FAIL lines from the gate tables / test harness, verbatim.
FAIL_LINES=$(
	grep 'FAILED' "$LOGS/tests.log" 2>/dev/null
)
if [ -n "$FAIL_LINES" ]; then
	{
		printf '\n## FAIL lines\n\n```\n'
		printf '%s\n' "$FAIL_LINES"
		printf '```\n'
	} >>"$REPORT"
fi

append_tail() {
	{
		printf '\n### tail: %s\n\n```\n' "$1"
		tail -n 40 "$2"
		printf '```\n'
	} >>"$REPORT"
}
if [ "$TEST_STATUS" = FAIL ]; then append_tail "cargo test" "$LOGS/tests.log"; fi
if [ "$CLIPPY_STATUS" = FAIL ]; then append_tail "cargo clippy" "$LOGS/clippy.log"; fi

# ---- history JSONL (one line per run) --------------------------------------
printf '{"date":"%s","test_exit":%s,"suites_ok":%s,"suites_total":%s,"clippy_exit":%s,"clippy_warnings":%s,"py_gates_ok":%s,"py_gates_total":%s,"discover_in_step":%s,"failures":%s,"overall":"%s"}\n' \
	"$DATE" "$TEST_EXIT" "$SUITES_OK" "$SUITES_TOTAL" "$CLIPPY_EXIT" "$CLIPPY_WARNINGS" \
	"$PY_OK" "$PY_TOTAL" \
	"$([ "$GEN_STATUS" = PASS ] && echo true || echo false)" "$FAILURES" "$OVERALL" >>"$HISTORY"

rm -rf "$LOGS"
printf 'nightly: %s — report %s\n' "$OVERALL" "$REPORT"
if [ "$FAILURES" -eq 0 ]; then
	exit 0
fi
exit 1
