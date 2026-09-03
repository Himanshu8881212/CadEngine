#!/usr/bin/env bash
# Run the preserved reference-assembly example end to end, through the public
# surfaces only. Exit 0 iff every step behaves as declared — including the
# negative control, which MUST exit 1.
#
# This is the surviving half of the retired gearbox campaign's run_all.sh: the
# campaign's own regeneration machinery (generate.py, the 21 per-part programs)
# went with the campaign; the assembly, the design-intent layer and the
# evidence programs are kept here. Part sources resolve into
# crates/kernel-model/tests/fixtures/pre_w6_parts/ (see the README there).
set -u
cd "$(dirname "$0")/../.."          # repo root
KAPI=./target/release/kernel-api
HERE=reference/assembly
OUT=${1:-out/reference_assembly}

echo "== build CLI =="
cargo build -p kernel-api --release || exit 1
mkdir -p "$OUT" || exit 1
fail=0

echo "== flat assembly (load / mates / BOM v2 / exports / contacts) =="
"$KAPI" asm "$HERE/gearbox.lmcasm" --out-dir "$OUT/asm" > "$OUT/asm_report.json" 2>&1
if [ $? -ne 0 ]; then
	echo "  FAIL kernel-api asm — see $OUT/asm_report.json"; fail=1
elif python3 "$HERE/check_asm.py" "$OUT/asm_report.json" > "$OUT/check_asm.txt" 2>&1; then
	echo "  ok   kernel-api asm + design-intent contact allowlist — see $OUT/check_asm.txt"
else
	echo "  FAIL design-intent contact check — see $OUT/check_asm.txt"; fail=1
fi

echo "== nested assembly (v2: 3 shaft-stack sub-assemblies via asm_path) =="
# Same 37 parts at the same world poses, regrouped: gearbox_nested.lmcasm places
# asm/shaft_*.lmcasm as rigid units (each solving its own gear-on-shaft mates
# first). check_asm.py holds it to the SAME 52-designed-contact standard
# (hierarchical names like stack_in/g1p classify by leaf) and pins the BOM v2
# tree rollup + the meta-derived masses.
"$KAPI" asm "$HERE/gearbox_nested.lmcasm" --out-dir "$OUT/asm_nested" \
	> "$OUT/asm_nested_report.json" 2>&1
if [ $? -ne 0 ]; then
	echo "  FAIL kernel-api asm gearbox_nested.lmcasm — see $OUT/asm_nested_report.json"; fail=1
elif python3 "$HERE/check_asm.py" "$OUT/asm_nested_report.json" > "$OUT/check_asm_nested.txt" 2>&1; then
	echo "  ok   nested asm + contacts across levels + BOM v2 tree/masses — see $OUT/check_asm_nested.txt"
else
	echo "  FAIL nested design-intent check — see $OUT/check_asm_nested.txt"; fail=1
fi

echo "== exact-boolean disjointness proofs for the phantom contact pairs =="
# FRICTION #19: pose + union + assert shells == 2, all through the program
# surface. Its load_part inputs are repo-root-relative, so --out-dir is the root.
"$KAPI" run "$HERE/check_artifacts.json" --out-dir . > "$OUT/report_check_artifacts.txt" 2>&1
if [ $? -ne 0 ]; then
	echo "  FAIL check_artifacts.json — see $OUT/report_check_artifacts.txt"; fail=1
else
	echo "  ok   check_artifacts.json (5 artifact pairs exact-proven disjoint)"
fi

echo "== gear-mesh assertions =="
for p in check_mesh_stage1 check_mesh_stage2; do
	"$KAPI" run "$HERE/programs/$p.json" --out-dir "$OUT" > "$OUT/report_$p.txt" 2>&1
	if [ $? -ne 0 ]; then echo "  FAIL $p — see $OUT/report_$p.txt"; fail=1
	else echo "  ok   $p"; fi
done

echo "== negative control (MUST exit 1) =="
"$KAPI" run "$HERE/programs/check_clash_expected_fail.json" --out-dir "$OUT" \
	> "$OUT/report_check_clash_expected_fail.txt" 2>&1
if [ $? -eq 1 ]; then
	echo "  ok   check_clash_expected_fail.json failed as designed"
else
	echo "  FAIL check_clash_expected_fail.json did NOT fail — the negative control is dead"
	fail=1
fi

[ "$fail" -eq 0 ] && echo "ALL GREEN" || echo "FAILURES PRESENT"
exit $fail
