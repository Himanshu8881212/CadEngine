#!/usr/bin/env python3
"""Combine every printable part of all THREE drives (cyclo26, harmonic26,
planetary26) onto ONE bed plate → ALL_DRIVES_PLATE.stl. Parts are already in
audited print orientation; this shelf-packs them (big-first) with clearance,
duplicates _xN parts, drops each to z=0, and reports the footprint so you can
check it against your bed."""
import os, struct, glob, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVES = ["cyclo26", "harmonic26", "planetary26"]
GAP = 6.0
BED_W = float(sys.argv[1]) if len(sys.argv) > 1 else 250.0  # shelf wrap width

def read_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    return [list(struct.unpack("<12f", d[84 + i*50 : 84 + i*50 + 48])) for i in range(n)]

def bbox(tris):
    xs = [v[k] for v in tris for k in (3,6,9)]; ys=[v[k] for v in tris for k in (4,7,10)]; zs=[v[k] for v in tris for k in (5,8,11)]
    return min(xs),max(xs),min(ys),max(ys),min(zs),max(zs)

parts = []
for drive in DRIVES:
    for f in sorted(glob.glob(os.path.join(ROOT, drive, "parts", "*.stl"))):
        name = os.path.basename(f)[:-4]
        m = re.search(r"_x(\d+)$", name); copies = int(m.group(1)) if m else 1
        tris = read_stl(f)
        for c in range(copies):
            parts.append((f"{drive}/{name}" + (f"#{c+1}" if copies>1 else ""), tris))

parts.sort(key=lambda p: -((bbox(p[1])[1]-bbox(p[1])[0])*(bbox(p[1])[3]-bbox(p[1])[2])))
placed, sx, sy, shelf_h = [], 0.0, 0.0, 0.0
for name, tris in parts:
    x0,x1,y0,y1,z0,_ = bbox(tris); w,h = x1-x0+GAP, y1-y0+GAP
    if sx + w > BED_W:
        sy += shelf_h; sx = 0.0; shelf_h = 0.0
    dx,dy,dz = sx-x0, sy-y0, -z0
    sx += w; shelf_h = max(shelf_h, h)
    moved=[]
    for v in tris:
        nv=list(v)
        for k in (3,6,9): nv[k]+=dx
        for k in (4,7,10): nv[k]+=dy
        for k in (5,8,11): nv[k]+=dz
        moved.append(nv)
    placed.append((name, moved))

allt = [v for _,t in placed for v in t]
out = os.path.join(ROOT, "ALL_DRIVES_PLATE.stl")
with open(out, "wb") as fh:
    fh.write(b"all-3-drives print plate".ljust(80, b" ")); fh.write(struct.pack("<I", len(allt)))
    for v in allt: fh.write(struct.pack("<12fH", *v, 0))
x0,x1,y0,y1,z0,z1 = bbox(allt)
print(f"{len(placed)} bodies packed, {len(allt)} triangles")
print(f"FOOTPRINT: {x1-x0:.0f} x {y1-y0:.0f} mm | height {z1-z0:.1f} mm | z-min {z0:.3f}")
for n,_ in placed: pass
by_drive={}
for n,t in placed:
    d=n.split("/")[0]; by_drive[d]=by_drive.get(d,0)+1
print("per drive:", by_drive)
print("wrote", out)
