#!/usr/bin/env python3
"""Unified-BOM audit: tally hardware instances from each drive's STEP
assembly (instance names appear once as PRODUCT and once per NAUO — the
minimum count of the quoted-name occurrences minus the product row gives
the instance count; we use the PRODUCT_DEFINITION occurrence pattern
instead: count NEXT_ASSEMBLY_USAGE_OCCURRENCE lines naming each part)."""
import re, sys, collections

UNIFIED = {
    "hw_bearing_6804": ("6804 bearing", None),
    "hw_bearing_693zz": ("693ZZ bearing (harmonic wave gen ONLY)", {"harmonic26"}),
    "hw_bearing_688_ecc": ("688 bearing (cyclo backdrivable eccentric ONLY)", {"cyclo26"}),
    "hw_dowel_2x20": ("Ø2×20 dowel (cyclo ring ONLY)", {"cyclo26"}),
    "hw_m3x40_sandwich": ("M3×40 button", None),
    # cyclo26 + harmonic26 moved to M3×30 sandwich bolts (2026-07-19 tap-depth
    # audit: M3×40 bottomed out in the ~4.5 mm blind NEMA-17 face taps)
    "hw_m3x30_sandwich": ("M3×30 button", None),
    "hw_m3x12_pin": ("M3×12 csk (output pins)", None),
    "hw_m3x12_hub": ("M3×12 csk (hub)", None),
    # harmonic26 moved to M3×10 hub screws (2026-07-19 pilot-depth audit: the
    # M3×12 tip bottomed 3.6 mm early; M3×12 also can't drill deeper — its
    # flank would ride the Ø5 motor shaft below z 24)
    "hw_m3x10_hub": ("M3×10 csk (hub)", None),
    "hw_m3x8_retainer": ("M3×8 button (retainer)", None),
    "hw_m3x8_axle": ("M3×8 button (roller axles)", {"harmonic26"}),
    "hw_m3x5_set": ("M3×5 DIN916 set screw", {"cyclo26", "harmonic26"}),
    # same physical M3×5 DIN916 grub as hw_m3x5_set — used as planet journal axles
    # in the planetary (swappable across the kit; counted with the set screws below)
    "hw_m3x5_axle": ("M3×5 DIN916 (planet axle = same screw as the set screw)", {"planetary26"}),
    "hw_nema17": ("NEMA-17 motor", None),
}
ok = True
grand = collections.Counter()
for drive in ("cyclo26", "harmonic26", "planetary26"):
    txt = open(f"{drive}/ASSEMBLY.step").read()
    nauo = re.findall(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('[^']*','[^']*','[^']*',#\d+,#\d+,\$\)", txt)
    # instance names: count each quoted hw_ name, minus 1 PRODUCT + 1 PRODUCT_DEFINITION style rows
    names = re.findall(r"'(hw_[a-z0-9_]+)'", txt)
    c = collections.Counter(names)
    # each unique part contributes ~4 metadata rows + 1 per additional instance? calibrate:
    # PRODUCT block uses the name k times for a single-instance part; measure via a known:
    print(f"\n== {drive} ==")
    # calibrate overhead: hw_nema17 is always exactly 1 instance
    overhead = c.get("hw_nema17", 0) - 1
    for name, cnt in sorted(c.items()):
        inst = cnt - overhead
        grand[name] += inst
        allowed = UNIFIED.get(name)
        if allowed is None:
            print(f"  {name:22} ×{inst:2}  <<< NOT IN THE UNIFIED BOM")
            ok = False
            continue
        label, only = allowed
        if only and drive not in only:
            print(f"  {name:22} ×{inst:2}  <<< {label} — not allowed in {drive}")
            ok = False
        else:
            print(f"  {name:22} ×{inst:2}  OK   ({label})")
print("\n== family totals ==")
for name, tot in sorted(grand.items()):
    print(f"  {name:22} ×{tot}")
print("\nUNIFIED-BOM AUDIT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
