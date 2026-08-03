#!/usr/bin/env bash
# Run the whole gearbox build/verify pipeline through the public surfaces.
# Exit 0 iff every program behaves as declared (programs/EXPECT_FAIL lists the
# documented expected-failure evidence programs) and asmcheck passes.
set -u
cd "$(dirname "$0")"
ROOT=..
KAPI=$ROOT/target/release/kernel-api

echo "== build CLI =="
(cd $ROOT && cargo build -p kernel-api --release) || exit 1

echo "== regenerate parts/programs/assembly =="
python3 generate.py || exit 1

mkdir -p out
fail=0
echo "== run programs =="
for p in programs/p_*.json programs/check_*.json; do
	n=$(basename "$p")
	"$KAPI" run "$p" --out-dir out > "out/report_${n%.json}.txt" 2>&1
	got=$?
	want=0
	grep -qx "$n" programs/EXPECT_FAIL 2>/dev/null && want=1
	if [ "$got" -eq "$want" ]; then
		extra=""
		[ "$want" -eq 1 ] && extra="  (expected failure: documented friction evidence)"
		echo "  ok   $n$extra"
	else
		echo "  FAIL $n (exit $got, wanted $want)"
		fail=1
	fi
done

echo "== assembly check (official surface: kernel-api asm, FRICTION #1/#2 resolved-w6) =="
# Exact-boolean disjointness proofs for the known tessellation-artifact pairs
# (FRICTION #19): pose + union + assert shells == 2, all through the program
# surface. Paths in this program resolve against gearbox/ (--out-dir .).
"$KAPI" run check_artifacts.json --out-dir . > out/report_check_artifacts.txt 2>&1
if [ $? -ne 0 ]; then
	echo "  FAIL check_artifacts.json (exact disjointness proofs) — see out/report_check_artifacts.txt"
	fail=1
else
	echo "  ok   check_artifacts.json (5 artifact pairs exact-proven disjoint)"
fi
"$KAPI" asm gearbox.lmcasm --out-dir out/asm > out/asm_report.json
if [ $? -ne 0 ]; then
	echo "  FAIL kernel-api asm (load/mates/BOM/exports/contacts) — see out/asm_report.json"
	fail=1
elif python3 check_asm.py out/asm_report.json > out/check_asm.txt 2>&1; then
	echo "  ok   kernel-api asm + design-intent contact allowlist — see out/asm_report.json, out/check_asm.txt"
else
	echo "  FAIL design-intent contact check — see out/check_asm.txt"
	fail=1
fi

echo "== nested assembly (v2: 3 shaft-stack sub-assemblies via asm_path; BOM v2 masses) =="
# Same 37 parts at the same world poses, regrouped: gearbox_nested.lmcasm
# places asm/shaft_*.lmcasm as rigid units (each solving its own gear-on-shaft
# mates first). check_asm.py holds it to the SAME 52-designed-contact standard
# (hierarchical names like stack_in/g1p classify by leaf) and pins the BOM v2
# tree rollup + the meta-derived masses.
"$KAPI" asm gearbox_nested.lmcasm --out-dir out/asm_nested > out/asm_nested_report.json
if [ $? -ne 0 ]; then
	echo "  FAIL kernel-api asm gearbox_nested.lmcasm — see out/asm_nested_report.json"
	fail=1
elif python3 check_asm.py out/asm_nested_report.json > out/check_asm_nested.txt 2>&1; then
	echo "  ok   nested asm + contacts across levels + BOM v2 tree/masses — see out/check_asm_nested.txt"
else
	echo "  FAIL nested design-intent check — see out/check_asm_nested.txt"
	fail=1
fi

[ "$fail" -eq 0 ] && echo "ALL GREEN" || echo "FAILURES PRESENT"
exit $fail
