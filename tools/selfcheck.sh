#!/bin/sh
# selfcheck.sh — enable/disable/status/run for the LMCAD nightly self-exercise.
#
# The "self-exercise" is a plain OS crontab entry (NOT an agent, NOT a model,
# NOT Claude Code — just `cargo test` + the campaign examples via
# tools/nightly.sh, writing a dated report to telemetry/nightly/). It exists
# so regressions surface within a day; it is OFF unless you enable it.
#
#   tools/selfcheck.sh status    show whether the nightly entry is installed
#   tools/selfcheck.sh enable    install the 03:17 nightly crontab entry
#   tools/selfcheck.sh disable   remove it
#   tools/selfcheck.sh run       run the self-check once, right now, in foreground
TAG='# lmcad-nightly'
ROOT=$(cd "$(dirname "$0")/.." && pwd)
LINE="17 3 * * * cd \"$ROOT\" && sh tools/nightly.sh >> telemetry/nightly/cron.log 2>&1 $TAG"
PATHLINE="PATH=$HOME/.cargo/bin:/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin $TAG-path"
case "$1" in
	status)
		if crontab -l 2>/dev/null | grep -q "$TAG$"; then echo "nightly self-check: ENABLED (03:17)"; else echo "nightly self-check: disabled"; fi ;;
	enable)
		( crontab -l 2>/dev/null | grep -v "lmcad-nightly"; echo "$PATHLINE"; echo "$LINE" ) | crontab - && echo "enabled — next run 03:17; disable any time with: tools/selfcheck.sh disable" ;;
	disable)
		crontab -l 2>/dev/null | grep -v "lmcad-nightly" | crontab - ; echo "disabled" ;;
	run)
		exec sh "$ROOT/tools/nightly.sh" ;;
	*)
		echo "usage: tools/selfcheck.sh {status|enable|disable|run}"; exit 2 ;;
esac
