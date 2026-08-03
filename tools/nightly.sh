#!/bin/sh
# LMCAD nightly self-exercise — full test suite, clippy, and the two campaign
# gate suites (respool, drybox_roller), re-proving every machine-verified
# claim. Writes telemetry/nightly/YYYY-MM-DD.md (table + FAIL lines + tails of
# failing logs) and appends one summary line to telemetry/nightly/history.jsonl.
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

# ---- 3. RESPOOL campaign gate suite ----------------------------------------
cargo run --release -p kernel-model --example respool >"$LOGS/respool.log" 2>&1
RESPOOL_EXIT=$?
RESPOOL_VERDICT=$(grep '^RESPOOL:' "$LOGS/respool.log" | tail -n 1)
[ -n "$RESPOOL_VERDICT" ] || RESPOOL_VERDICT="(no verdict line - crashed before gates?)"
if [ "$RESPOOL_EXIT" -eq 0 ]; then
	RESPOOL_STATUS=PASS
else
	RESPOOL_STATUS=FAIL
	FAILURES=$((FAILURES + 1))
fi

# ---- 4. DRYBOX ROLLER campaign gate suite ----------------------------------
cargo run --release -p kernel-model --example drybox_roller >"$LOGS/drybox.log" 2>&1
DRYBOX_EXIT=$?
DRYBOX_VERDICT=$(grep '^DRYBOX ROLLER:' "$LOGS/drybox.log" | tail -n 1)
[ -n "$DRYBOX_VERDICT" ] || DRYBOX_VERDICT="(no verdict line - crashed before gates?)"
if [ "$DRYBOX_EXIT" -eq 0 ]; then
	DRYBOX_STATUS=PASS
else
	DRYBOX_STATUS=FAIL
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
	printf '| example respool | %s | exit %s, %s |\n' \
		"$RESPOOL_STATUS" "$RESPOOL_EXIT" "$RESPOOL_VERDICT"
	printf '| example drybox_roller | %s | exit %s, %s |\n' \
		"$DRYBOX_STATUS" "$DRYBOX_EXIT" "$DRYBOX_VERDICT"
	printf '\noverall: **%s** (%s failing step(s))\n' "$OVERALL" "$FAILURES"
} >"$REPORT"

# Any FAIL lines from the gate tables / test harness, verbatim.
FAIL_LINES=$(
	grep -h '<<< FAIL' "$LOGS/respool.log" "$LOGS/drybox.log" 2>/dev/null
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
if [ "$RESPOOL_STATUS" = FAIL ]; then append_tail "example respool" "$LOGS/respool.log"; fi
if [ "$DRYBOX_STATUS" = FAIL ]; then append_tail "example drybox_roller" "$LOGS/drybox.log"; fi

# ---- history JSONL (one line per run) --------------------------------------
printf '{"date":"%s","test_exit":%s,"suites_ok":%s,"suites_total":%s,"clippy_exit":%s,"clippy_warnings":%s,"respool_exit":%s,"drybox_exit":%s,"failures":%s,"overall":"%s"}\n' \
	"$DATE" "$TEST_EXIT" "$SUITES_OK" "$SUITES_TOTAL" "$CLIPPY_EXIT" "$CLIPPY_WARNINGS" \
	"$RESPOOL_EXIT" "$DRYBOX_EXIT" "$FAILURES" "$OVERALL" >>"$HISTORY"

rm -rf "$LOGS"
printf 'nightly: %s — report %s\n' "$OVERALL" "$REPORT"
if [ "$FAILURES" -eq 0 ]; then
	exit 0
fi
exit 1
