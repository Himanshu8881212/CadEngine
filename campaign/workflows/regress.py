#!/usr/bin/env python3
"""Campaign regression + re-baseline census. NON-DESTRUCTIVE by construction.

For each campaign (*_system/* + showcase/squatchee_spin): clone the part
directory into scratch, run every real kernel-api program (top-level "ops"
array) INSIDE the clone (program path and --out-dir both in the clone, so
write-then-read round trips and Reproducing-convention paths work), then
byte-compare every artifact the clone now holds against the untouched original.
The originals are never written.

Modes:
  verify — any byte change or unexpected failure is a problem (exit 1)
  census — byte changes are EXPECTED (tessellation re-baseline); listed per part

House rules (campaign/ORCHESTRATOR_VERIFICATION.md): tool-job JSONs are not
engine programs; negative controls may exit 0 (union+assert: the interference
number is the receipt) or nonzero (assert_failed style) — both legitimate.
"""
import json, subprocess, sys, shutil, filecmp, tempfile
from pathlib import Path

MODE = sys.argv[1] if len(sys.argv) > 1 else "verify"
REPO = Path("/Users/himanshu/Work/New-LMCAD/cad engine")
CLI = REPO / "target/release/kernel-api"
SCRATCH = Path(tempfile.mkdtemp(prefix="lmcad_census_"))

stats = dict(programs=0, passed=0, failed=0, nc_failed_as_designed=0,
             warnings=0, identical=0, changed=0, timeout=0)
changed, problems = [], []

parts = sorted(d for d in REPO.glob("*_system/*") if d.is_dir())
parts.append(REPO / "showcase/squatchee_spin")

ART = (".stl", ".3mf", ".step")

for part in parts:
    if not (part / "programs").is_dir():
        continue
    clone = SCRATCH / part.parent.name / part.name
    shutil.copytree(part, clone, symlinks=True,
                    ignore=shutil.ignore_patterns("__pycache__", ".DS_Store"))

    for prog in sorted((clone / "programs").glob("*.json")):
        try:
            doc = json.loads(prog.read_text())
        except Exception:
            continue
        if not isinstance(doc, dict) or not isinstance(doc.get("ops"), list):
            continue
        stats["programs"] += 1
        name = prog.stem.lower()
        # A negative control announces itself either by filename convention or
        # by declared intent in the program header ("expected REFUSAL ..." in
        # the part/_why string) — filename alone misfiled asm_scene_probe and
        # the *_refuse / backlash_* families.
        header = str(doc.get("part", "")).lower()
        is_nc = name.startswith("nc") or "oracle" in name or "severed" in name \
            or "_neg" in name or "_fail" in name or "_refus" in name \
            or name.startswith("backlash") or "expected refusal" in header \
            or "expected fail" in header
        try:
            r = subprocess.run([str(CLI), "run", str(prog), "--out-dir", str(clone)],
                               capture_output=True, text=True, timeout=420)
        except subprocess.TimeoutExpired:
            stats["timeout"] += 1
            problems.append(f"TIMEOUT: {part.name}/{prog.name}")
            continue
        try:
            rep = json.loads(r.stdout)
        except Exception:
            rep = {}
        stats["warnings"] += sum(len(o.get("warnings", [])) for o in rep.get("ops", []))
        if r.returncode != 0:
            if is_nc:
                stats["nc_failed_as_designed"] += 1
            else:
                stats["failed"] += 1
                kinds = [o.get("error", {}).get("kind") for o in rep.get("ops", []) if not o.get("ok", True)]
                problems.append(f"FAIL {kinds}: {part.name}/{prog.name}")
        else:
            stats["passed"] += 1

    # Compare every shipped artifact against its regenerated twin in the clone.
    for sub in ("parts", "print", "cad"):
        orig_dir = part / sub
        if not orig_dir.is_dir():
            continue
        for orig in sorted(orig_dir.glob("*")):
            if orig.suffix.lower() not in ART:
                continue
            twin = clone / sub / orig.name
            if not twin.exists():
                continue  # program suite didn't regenerate it in this pass
            if filecmp.cmp(str(orig), str(twin), shallow=False):
                stats["identical"] += 1
            else:
                stats["changed"] += 1
                changed.append(f"{part.parent.name}/{part.name}: {sub}/{orig.name}")

print(json.dumps(stats, indent=1))
if changed:
    print("=== BYTE-CHANGED shipped artifacts:")
    print("\n".join(changed))
if problems:
    print("=== PROBLEMS:")
    print("\n".join(problems))
print(f"scratch: {SCRATCH}")
if MODE == "verify" and (changed or [p for p in problems]):
    sys.exit(1)
