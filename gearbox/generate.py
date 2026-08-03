#!/usr/bin/env python3
"""Parametric 2-stage reduction gearbox for the LMCAD kernel — single source of truth.

Design-engineer dogfood build (w5): authors ONLY public surfaces:
  parts/*.lmcpart  — native parametric part documents (kernel-model Document JSON)
  programs/*.json  — kernel-api CLI programs (load/validate/measure/export per part + checks)
  gearbox.lmcasm   — top-level assembly (poses, mates, named "exploded" state)

Train:  stage 1  m=1.25  z 12:60  (i=5.0)   C1 = 1.25*(12+60)/2 = 45.000 mm
        stage 2  m=1.50  z 15:45  (i=3.0)   C2 = 1.50*(15+45)/2 = 45.000 mm
        overall i = 15.0; mounted C* = C + 0.15 mm backlash allowance
All dimensions mm, angles deg unless noted. Run:  python3 generate.py
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------------------
# PARAMETERS (the design table lives here)
# ----------------------------------------------------------------------------------
P = {}

# Gear train (ISO 53, 20 deg pressure angle)
P["m1"], P["z1p"], P["z1w"] = 1.25, 12, 60
P["m2"], P["z2p"], P["z2w"] = 1.50, 15, 45
P["pa"] = 20.0
P["backlash_dc"] = 0.15                      # extra centre distance, printed-gear allowance
C1 = P["m1"] * (P["z1p"] + P["z1w"]) / 2.0   # 45.000
C2 = P["m2"] * (P["z2p"] + P["z2w"]) / 2.0   # 45.000
CM1 = C1 + P["backlash_dc"]                  # 45.15 mounted
CM2 = C2 + P["backlash_dc"]                  # 45.15 mounted

# tip / root radii
def tip_r(m, z):  return m * (z / 2.0 + 1.0)
def root_r(m, z): return m * (z / 2.0 - 1.25)
def pitch_r(m, z): return m * z / 2.0

# Face widths (axial, along the shaft = assembly Y)
FW1P, FW1W, FW2P, FW2W = 12.0, 10.0, 14.0, 12.0

# Shaft axes in assembly/world frame: axes parallel to +Y, all at height Z_AXIS
X_IN, X_MID, X_OUT = 0.0, CM1, CM1 + CM2     # 0 / 45.15 / 90.30
Z_AXIS = 48.0

# Shaft & bearing (608: 8 x 22 x 7)
SHAFT_D = 8.0
BRG_OD, BRG_ID, BRG_W = 22.0, 8.0, 7.0
KEY_B, KEY_H, KEY_T1, KEY_T2 = 2.0, 2.0, 1.2, 1.0   # DIN 6885-1 for d in (6,8]

# Housing base (world frame; parting/seal plane z = WTOP)
FLOOR_Z = 5.0            # cavity floor (floor slab thickness)
WTOP = 91.0              # wall top = seal face
CAV = (-14.0, 130.0, 0.0, 38.0)    # cavity rect at the top: x0,x1,y0,y1 (rounded r6)
OUT = (-24.0, 140.0, -10.0, 48.0)  # outer rect at the top (rounded r8)
FLG = (-36.0, 152.0, -22.0, 60.0)  # flange / lid rect (sharp corners)
FLG_Z0, FLG_Z1 = 79.0, 91.0        # flange slab band
BODY_Z1 = 82.0                     # drafted body top (overlaps slab 79..82)
DRAFT_OUT_DEG, DRAFT_CAV_DEG = 1.5, 2.0
LID_Z0, LID_Z1 = 91.0, 99.0

# Bearing wall geometry (per wall, wall A inner face y=0, outer y=-10; wall B mirrored at y=38/48)
BOSS_R, BOSS_LEN = 17.0, 4.0       # inner boss dia 34, face at y=4 / y=34
WEB_Y_A, WEB_Y_B = -3.0, 41.0      # pocket bottoms (outer-race shoulders)
WEB_BORE_R = 8.0                   # dia 16 through bore (shaft + inner-race clearance)

# Axial stack (y): stage-1 plane then stage-2 plane (g1w and g2p butt — same shaft)
Y_G1P = (6.0, 18.0)
Y_G1W = (7.0, 17.0)
Y_G2P = (17.0, 31.0)
Y_G2W = (18.0, 30.0)

# Shafts (length along their local +Z, posed base at world y = SH_*_Y0)
SH_IN_Y0, SH_IN_LEN = -25.0, 73.0    # y -25..48: 15 mm coupling stickout at front
SH_MID_Y0, SH_MID_LEN = -8.0, 55.0   # y -8..47: fully captive
SH_OUT_Y0, SH_OUT_LEN = -9.0, 82.0   # y -9..73: 25 mm stickout at back

# Lid fastening: 8x DIN 912 M4 into heat-set inserts; 2x ISO 2338 dowels dia 4
BOLT_M = 4.0
BOLT_LEN = 12.0
CB_DEPTH_M4 = 4.8   # DIN 974-1 counterbore depth for M4 (kernel holes table; NOT
                    # queryable through the JSON surface — see FRICTION.md)
BOLT_XY = [(-30.0, -16.0), (146.0, -16.0), (146.0, 54.0), (-30.0, 54.0),
           (58.0, -16.0), (58.0, 54.0), (-30.0, 19.0), (146.0, 19.0)]
DOWEL_D, DOWEL_LEN = 4.0, 12.0
DOWEL_XY = [(14.0, -16.0), (132.0, 54.0)]
HEATSET_M4_PILOT, HEATSET_M4_DEPTH = 5.6, 9.5   # Ruthex M4 pilot; insert 8.1 + melt room
DOWEL_BASE_DEPTH = 6.5

# O-ring (face seal, lid underside): 2.0 mm EPDM cord, racetrack groove on the wall midline
ORG_RECT = (-19.0, 135.0, -5.0, 43.0)   # centreline rect
ORG_CR = 7.0                            # centreline corner radius
ORG_W, ORG_DEPTH = 2.7, 1.5             # 25% squeeze on 2.0 cord, ~78% fill

# Accessory heat-set bosses (M3) on the cavity floor
ACC_XY = [(20.0, 10.0), (20.0, 28.0), (70.0, 10.0), (70.0, 28.0)]

# Spacer tubes (printed): OD 12 / ID 8.4, lengths from the axial stack
SPACERS = {  # name -> (length, [world (x, y0) placements])
    "9":  (9.0,  [(X_IN, -3.0)]),
    "10": (10.0, [(X_MID, -3.0), (X_MID, 31.0)]),
    "11": (11.0, [(X_OUT, 30.0)]),
    "21": (21.0, [(X_OUT, -3.0)]),
    "23": (23.0, [(X_IN, 18.0)]),
}
SPC_OD, SPC_ID = 12.0, 8.4

# Keys 2x2 (form B in this design — see FRICTION; document layer has no form-A key/keyway)
KEYS = {  # name -> (length, [(shaft world x, world y0 of key/slot, phase deg)])
    "8":  (8.0,  [(X_IN, -20.0, 0.0), (X_OUT, 53.0, -1.0), (X_MID, 8.0, 3.0)]),
    "10": (10.0, [(X_IN, 7.0, 0.0), (X_OUT, 19.0, -1.0)]),
    "12": (12.0, [(X_MID, 18.0, 3.0)]),
}

# Mesh phases (deg, about each axis; derived from tooth-0-on-+X convention; see README)
PH_IN, PH_MID, PH_OUT = 0.0, 3.0, -1.0

# BOM v2 engineering metadata: the optional .lmcpart "meta" block (part number,
# material density g/cm3, make-or-buy) on the five metal parts. bom.json mass =
# density x ENGINE volume of the MODELLED geometry (volume_source "exact": all
# five are exact B-reps). Honesty note: bearing_608 is the solid ENVELOPE ring
# (FRICTION #8 — no rolling-element part), so its BOM mass (~18.1 g)
# intentionally overstates a real 608ZZ (~12 g of steel + air); the README says
# so next to the number.
STEEL = {"name": "steel", "density_g_cm3": 7.85}
BRASS = {"name": "brass", "density_g_cm3": 8.4}
META = {  # part file stem -> meta block
    "shaft_input":        {"part_number": "GBX-SH-IN",       "material": STEEL, "make_or_buy": "make"},
    "shaft_intermediate": {"part_number": "GBX-SH-MID",      "material": STEEL, "make_or_buy": "make"},
    "shaft_output":       {"part_number": "GBX-SH-OUT",      "material": STEEL, "make_or_buy": "make"},
    "bearing_608":        {"part_number": "608ZZ",           "material": STEEL, "make_or_buy": "buy"},
    "key_2x2_8":          {"part_number": "DIN6885B-2x2x8",  "material": BRASS, "make_or_buy": "buy"},
}

# Circlip grooves (DIN 471 d8: width m=0.9), outboard retention on in/out shafts;
# the clip's inner side face sits flush on the bearing inner-race face.
CLIP_W = 0.9
CLIP_IN_Y = -10.9    # groove spans y -10.9..-10.0 (race face at -10)
CLIP_OUT_Y = 48.0    # groove spans y 48.0..48.9 (race face at 48)


# ----------------------------------------------------------------------------------
# Small helpers: Dim / vectors / documents / programs
# ----------------------------------------------------------------------------------
def L(v):
    return {"Literal": float(v)}

def V3(x, y, z):
    return [L(x), L(y), L(z)]

def rounded_rect(x0, x1, y0, y1, r, seg=8):
    """CCW polygon of a rounded rectangle (convex)."""
    cs = [((x1 - r, y0 + r), -90.0), ((x1 - r, y1 - r), 0.0),
          ((x0 + r, y1 - r), 90.0), ((x0 + r, y0 + r), 180.0)]
    pts = []
    for (cx, cy), a0 in cs:
        for i in range(seg + 1):
            a = math.radians(a0 + 90.0 * i / seg)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    # drop consecutive duplicates
    out = []
    for p in pts:
        if not out or abs(p[0] - out[-1][0]) > 1e-12 or abs(p[1] - out[-1][1]) > 1e-12:
            out.append(p)
    if abs(out[0][0] - out[-1][0]) < 1e-12 and abs(out[0][1] - out[-1][1]) < 1e-12:
        out.pop()
    return out

def sketch_of(poly):
    """An unconstrained Document sketch holding one closed polygon."""
    n = len(poly)
    return {
        "points": [[float(x), float(y)] for x, y in poly],
        "segments": [{"a": i, "b": (i + 1) % n} for i in range(n)],
        "arcs": [], "circles": [], "constraints": [],
    }

def xform_translate(tx, ty, tz):
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, float(tx), float(ty), float(tz)]

def xform_rot_x(deg, tx=0.0, ty=0.0, tz=0.0):
    """Column-major Affine3A (12 floats): rotation about +X then translation."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    # columns = images of basis vectors: X->X, Y->(0,c,s), Z->(0,-s,c)
    return [1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c, float(tx), float(ty), float(tz)]


class Doc:
    """A kernel-model Document under construction (feature list + params)."""

    def __init__(self, params=None):
        self.params = params or {}
        self.features = []

    def add(self, feature, label=None):
        rec = dict(feature)
        if label:
            rec["label"] = label
        self.features.append(rec)
        return len(self.features) - 1

    def box(self, cx, cy, cz, sx, sy, sz, label=None):
        return self.add({"Box": {"center": V3(cx, cy, cz), "size": V3(sx, sy, sz)}}, label)

    def cylinder(self, cx, cy, cz, r, h, label=None):
        """Axis +Z, centred at (cx,cy,cz) -> spans cz +/- h/2."""
        return self.add({"Cylinder": {"center": V3(cx, cy, cz), "radius": L(r), "height": L(h)}}, label)

    def cylinder_y(self, cx, y0, y1, cz, r, label=None):
        """Cylinder along +Y spanning world y0..y1 at (cx, cz)."""
        h = y1 - y0
        cid = self.cylinder(0.0, 0.0, 0.0, r, h)
        # local z in -h/2..h/2; rotate Z->Y (about X by -90), centre at midspan
        return self.add({"Transform": {"input": cid,
                                       "xform": xform_rot_x(-90.0, cx, (y0 + y1) / 2.0, cz)}}, label)

    def boolean(self, op, a, b, label=None):
        return self.add({"Boolean": {"op": op, "a": a, "b": b}}, label)

    def union(self, a, b, label=None):
        return self.boolean("Union", a, b, label)

    def diff(self, a, b, label=None):
        return self.boolean("Difference", a, b, label)

    def extrude(self, poly, height, draft_deg=0.0, label=None):
        return self.add({"ExtrudeSketch": {"sketch": sketch_of(poly), "height": L(height),
                                           "dims": [], "draft": L(math.radians(draft_deg))}}, label)

    def transform(self, input_id, xf, label=None):
        return self.add({"Transform": {"input": input_id, "xform": xf}}, label)

    def hole(self, input_id, kind, m_or_d, at, axis, fit=None, depth=None, label=None):
        h = {"input": input_id, "kind": kind, "m_or_d": L(m_or_d),
             "at": V3(*at), "axis": V3(*axis)}
        if fit is not None:
            h["fit"] = fit
        if depth is not None:
            h["depth"] = L(depth)
        return self.add({"Hole": h}, label)

    def to_part(self, name):
        return {
            "format": "lmc-part", "version": 1, "units": "mm", "name": name,
            "created_with": "gearbox/generate.py (w5 dogfood)",
            "document": {"params": {k: float(v) for k, v in self.params.items()},
                         "features": self.features,
                         "root": len(self.features) - 1, "suppressed": []},
        }


def stamp_meta(env, meta):
    """The part envelope with the BOM v2 "meta" block inserted between
    created_with and document (the same field order the kernel's own
    save_part_with_meta writes)."""
    out = {}
    for k, v in env.items():
        if k == "document":
            out["meta"] = meta
        out[k] = v
    return out


PARTS = {}      # name -> part envelope dict
PROGRAMS = {}   # filename -> program dict
EXPECT_FAIL = set()  # program filenames expected to exit 1 (documented friction evidence)


def part_program(name, extra_ops=None, exact=False, mass=True):
    """Standard per-part program: load -> validate -> measures -> export STL + STEP."""
    ops = [
        {"id": "part", "op": "load_part", "file": f"../parts/{name}.lmcpart"},
        {"id": "topo", "op": "validate", "in": "part"},
        {"id": "vol", "op": "volume", "in": "part"},
    ]
    if exact:
        ops.append({"id": "xvol", "op": "exact_volume", "in": "part"})
    if mass:
        ops.append({"id": "mp", "op": "mass_properties", "in": "part"})
    ops += (extra_ops or [])
    ops += [
        {"id": "stl", "op": "export_stl", "in": "part", "file": f"{name}.stl", "tol": 0.01},
        {"id": "step", "op": "export_step", "in": "part", "file": f"{name}.step"},
    ]
    PROGRAMS[f"p_{name}.json"] = {"ops": ops}


# ----------------------------------------------------------------------------------
# PART: housing_base
# ----------------------------------------------------------------------------------
def build_housing_base():
    d = Doc()
    # Drafted outer body: profile = outer rect AT THE TOP, swept DOWN so the shell
    # narrows toward the floor (release upward, +Z pull). Top lands at BODY_Z1.
    body0 = d.extrude(rounded_rect(*OUT, 8.0), -BODY_Z1, DRAFT_OUT_DEG, "outer shell (drafted)")
    body = d.transform(body0, xform_translate(0, 0, BODY_Z1))
    slab = d.box((FLG[0] + FLG[1]) / 2.0, (FLG[2] + FLG[3]) / 2.0, (FLG_Z0 + FLG_Z1) / 2.0,
                 FLG[1] - FLG[0], FLG[3] - FLG[2], FLG_Z1 - FLG_Z0, "flange slab")
    solid = d.union(body, slab)
    # Drafted cavity cutter: top profile at z=92 (1 mm above seal face), down to the floor.
    cav0 = d.extrude(rounded_rect(*CAV, 6.0), -(WTOP + 1.0 - FLOOR_Z), DRAFT_CAV_DEG, "cavity cutter (drafted)")
    cav = d.transform(cav0, xform_translate(0, 0, WTOP + 1.0))
    solid = d.diff(solid, cav, "hollowed base")
    # Bearing bosses on the inner walls (buried 6 into the wall, protruding to the boss face)
    for x in (X_IN, X_MID, X_OUT):
        b = d.cylinder_y(x, -6.0, BOSS_LEN, Z_AXIS, BOSS_R)
        solid = d.union(solid, b, f"bearing boss A x={x:g}")
        b = d.cylinder_y(x, 38.0 - BOSS_LEN, 44.0, Z_AXIS, BOSS_R)
        solid = d.union(solid, b, f"bearing boss B x={x:g}")
    # Web bores dia 16 straight through both walls + bosses at every axis
    for x in (X_IN, X_MID, X_OUT):
        c = d.cylinder_y(x, -12.0, 50.0, Z_AXIS, WEB_BORE_R)
        solid = d.diff(solid, c, f"web bore x={x:g}")
    # 608 pockets dia 22 x 7 from each outer face to the web shoulder
    for x in (X_IN, X_MID, X_OUT):
        c = d.cylinder_y(x, -11.0, WEB_Y_A, Z_AXIS, BRG_OD / 2.0)
        solid = d.diff(solid, c, f"608 pocket A x={x:g}")
        c = d.cylinder_y(x, WEB_Y_B, 49.0, Z_AXIS, BRG_OD / 2.0)
        solid = d.diff(solid, c, f"608 pocket B x={x:g}")
    # Heat-set insert pockets (M4, Ruthex pilot 5.6) at the 8 lid-bolt positions
    for i, (bx, by) in enumerate(BOLT_XY):
        solid = d.hole(solid, "Drill", HEATSET_M4_PILOT, (bx, by, FLG_Z1), (0, 0, -1),
                       depth=HEATSET_M4_DEPTH, label=f"M4 insert pocket {i}")
    # Dowel press holes (dia 4 H7/n6 side), blind
    for i, (dx, dy) in enumerate(DOWEL_XY):
        solid = d.hole(solid, "Drill", DOWEL_D, (dx, dy, FLG_Z1), (0, 0, -1),
                       depth=DOWEL_BASE_DEPTH, label=f"dowel press hole {i}")
    # Accessory heat-set bosses (M3) on the cavity floor
    for i, (ax, ay) in enumerate(ACC_XY):
        solid = d.add({"HeatsetBoss": {"input": solid, "at": V3(ax, ay, FLOOR_Z),
                                       "axis": V3(0, 0, 1), "m": L(3.0)}},
                      f"M3 accessory boss {i}")
    PARTS["housing_base"] = d.to_part("housing_base")
    part_program("housing_base", extra_ops=[
        {"id": "wt", "op": "wall_thickness", "in": "part", "flag_below": 2.4},
        {"id": "draft", "op": "draft_analysis", "in": "part", "pull": [0, 0, 1], "min_deg": 1.0},
    ])


# ----------------------------------------------------------------------------------
# PART: housing_lid
# ----------------------------------------------------------------------------------
def build_housing_lid():
    d = Doc()
    lid = d.box((FLG[0] + FLG[1]) / 2.0, (FLG[2] + FLG[3]) / 2.0, (LID_Z0 + LID_Z1) / 2.0,
                FLG[1] - FLG[0], FLG[3] - FLG[2], LID_Z1 - LID_Z0, "lid plate")
    # O-ring groove (racetrack ring, 2.7 wide x 1.5 deep) in the underside seal face:
    # ring = outer prism - inner prism, then subtracted from the plate.
    x0, x1, y0, y1 = ORG_RECT
    half = ORG_W / 2.0
    # outer prism z 89.5..92.5 (1.5 air overshoot below the seal face, 1.5 cut depth);
    # inner prism z 88.5..93.5 pierces the outer fully so the ring is clean.
    go = d.extrude(rounded_rect(x0 - half, x1 + half, y0 - half, y1 + half, ORG_CR + half, 10), 3.0,
                   0.0, "groove ring outer")
    go = d.transform(go, xform_translate(0, 0, LID_Z0 - ORG_DEPTH))
    gi = d.extrude(rounded_rect(x0 + half, x1 - half, y0 + half, y1 - half, ORG_CR - half, 10), 5.0,
                   0.0, "groove ring inner")
    gi = d.transform(gi, xform_translate(0, 0, LID_Z0 - ORG_DEPTH - 1.0))
    ring = d.diff(go, gi, "o-ring groove cutter")
    lid = d.diff(lid, ring, "grooved lid")
    # 8x DIN 912 M4 counterbores (DIN 974: dia 8 pocket, 4.4 deep), entry on the top face
    for i, (bx, by) in enumerate(BOLT_XY):
        lid = d.hole(lid, "Counterbore", BOLT_M, (bx, by, LID_Z1), (0, 0, -1),
                     fit="Close", label=f"M4 counterbore {i}")
    # Dowel slip holes (dia 4 H7/g6 side), through
    for i, (dx, dy) in enumerate(DOWEL_XY):
        lid = d.hole(lid, "Drill", DOWEL_D, (dx, dy, LID_Z1), (0, 0, -1),
                     label=f"dowel slip hole {i}")
    PARTS["housing_lid"] = d.to_part("housing_lid")
    part_program("housing_lid", extra_ops=[
        {"id": "wt", "op": "wall_thickness", "in": "part", "flag_below": 2.0},
    ])


# ----------------------------------------------------------------------------------
# PARTS: shafts (catalog shaft + form-B keyway slots via Box + DIN 471 grooves)
# ----------------------------------------------------------------------------------
def keyway_slot(d, solid, z0, z1, label):
    """Form-B keyway slot on the +X side of a dia-8 shaft along +Z: floor at r - t1,
    cutter overshooting the wall radially (x 2.8..4.6)."""
    floor_x = SHAFT_D / 2.0 - KEY_T1                      # 2.8
    outer_x = SHAFT_D / 2.0 + 0.6                         # 4.6
    cut = d.box((floor_x + outer_x) / 2.0, 0.0, (z0 + z1) / 2.0,
                outer_x - floor_x, KEY_B, z1 - z0)
    return d.diff(solid, cut, label)

def circlip_groove(d, solid, z, label):
    return d.add({"CirclipGroove": {"input": solid, "at": V3(0, 0, z),
                                    "axis": V3(0, 0, 1), "d": L(SHAFT_D)}}, label)

def build_shafts():
    # local frame: shaft along +Z from z=0; world y = local z + SH_*_Y0
    # input: coupling slot + g1p slot + front circlip groove
    d = Doc()
    s = d.add({"CatalogPart": {"part": {"Shaft": {"d": L(SHAFT_D), "length": L(SH_IN_LEN)}}}},
              "shaft blank")
    s = keyway_slot(d, s, -20.0 - SH_IN_Y0, -12.0 - SH_IN_Y0, "coupling keyway 2x2x8")
    s = keyway_slot(d, s, 7.0 - SH_IN_Y0, 17.0 - SH_IN_Y0, "pinion-1 keyway 2x2x10")
    s = circlip_groove(d, s, CLIP_IN_Y - SH_IN_Y0, "DIN 471 groove (front)")
    PARTS["shaft_input"] = d.to_part("shaft_input")
    part_program("shaft_input", exact=True)

    # intermediate: g1w slot + g2p slot, captive (no grooves)
    d = Doc()
    s = d.add({"CatalogPart": {"part": {"Shaft": {"d": L(SHAFT_D), "length": L(SH_MID_LEN)}}}},
              "shaft blank")
    s = keyway_slot(d, s, 8.0 - SH_MID_Y0, 16.0 - SH_MID_Y0, "wheel-1 keyway 2x2x8")
    s = keyway_slot(d, s, 18.0 - SH_MID_Y0, 30.0 - SH_MID_Y0, "pinion-2 keyway 2x2x12")
    PARTS["shaft_intermediate"] = d.to_part("shaft_intermediate")
    part_program("shaft_intermediate", exact=True)

    # output: g2w slot + coupling slot + rear circlip groove
    d = Doc()
    s = d.add({"CatalogPart": {"part": {"Shaft": {"d": L(SHAFT_D), "length": L(SH_OUT_LEN)}}}},
              "shaft blank")
    s = keyway_slot(d, s, 19.0 - SH_OUT_Y0, 29.0 - SH_OUT_Y0, "wheel-2 keyway 2x2x10")
    s = keyway_slot(d, s, 53.0 - SH_OUT_Y0, 61.0 - SH_OUT_Y0, "coupling keyway 2x2x8")
    s = circlip_groove(d, s, CLIP_OUT_Y - SH_OUT_Y0, "DIN 471 groove (rear)")
    PARTS["shaft_output"] = d.to_part("shaft_output")
    part_program("shaft_output", exact=True)


# ----------------------------------------------------------------------------------
# PARTS: gears (catalog involute spur gears, bore 8 H7 + DIN 6885 hub keyway)
# ----------------------------------------------------------------------------------
def build_gears():
    for name, m, z, fw in [("gear_s1_pinion", P["m1"], P["z1p"], FW1P),
                           ("gear_s1_wheel", P["m1"], P["z1w"], FW1W),
                           ("gear_s2_pinion", P["m2"], P["z2p"], FW2P),
                           ("gear_s2_wheel", P["m2"], P["z2w"], FW2W)]:
        d = Doc()
        d.add({"CatalogPart": {"part": {"SpurGear": {
            "module": L(m), "teeth": z, "face_width": L(fw), "bore_d": L(SHAFT_D),
            "pressure_angle_deg": L(P["pa"]), "keyway": True}}}},
            f"ISO 53 m{m:g} z{z}")
        PARTS[name] = d.to_part(name)
        part_program(name, exact=True)


# ----------------------------------------------------------------------------------
# PARTS: bearing envelope, spacers, keys, fasteners
# ----------------------------------------------------------------------------------
def build_small_parts():
    # 608 envelope ring (no rolling-element catalog part in the kernel; see FRICTION)
    d = Doc()
    ring = d.cylinder(0, 0, BRG_W / 2.0, BRG_OD / 2.0, BRG_W, "608 envelope")
    bore = d.cylinder(0, 0, BRG_W / 2.0, BRG_ID / 2.0, BRG_W + 2.0)
    d.diff(ring, bore, "bored")
    PARTS["bearing_608"] = d.to_part("bearing_608")
    part_program("bearing_608", exact=True)

    for name, (length, _) in SPACERS.items():
        d = Doc({"len": length})
        t = d.add({"Cylinder": {"center": V3(0, 0, length / 2.0), "radius": L(SPC_OD / 2.0),
                                "height": {"Param": "len"}}}, "tube")
        b = d.cylinder(0, 0, length / 2.0, SPC_ID / 2.0, length + 2.0)
        d.diff(t, b, "bored")
        PARTS[f"spacer_{name}"] = d.to_part(f"spacer_8x12_{name}")
        part_program(f"spacer_{name}", mass=False)

    for name, (length, _) in KEYS.items():
        d = Doc({"len": length})
        d.add({"Box": {"center": [L(length / 2.0), L(0.0), L(KEY_H / 2.0)],
                       "size": [{"Param": "len"}, L(KEY_B), L(KEY_H)]}}, "DIN 6885 B key")
        PARTS[f"key_2x2_{name}"] = d.to_part(f"key_2x2_{name}")
        part_program(f"key_2x2_{name}", mass=False)

    d = Doc()
    d.add({"CatalogPart": {"part": {"SocketHeadCapScrew": {"m": L(BOLT_M), "length": L(BOLT_LEN)}}}},
          "DIN 912 M4x12")
    PARTS["screw_m4x12"] = d.to_part("screw_m4x12")
    part_program("screw_m4x12", mass=False)

    d = Doc()
    d.add({"CatalogPart": {"part": {"DowelPin": {"d": L(DOWEL_D), "length": L(DOWEL_LEN)}}}},
          "ISO 2338 4x12")
    PARTS["dowel_4x12"] = d.to_part("dowel_4x12")
    part_program("dowel_4x12", mass=False)

    # DIN 471 dia-8 circlip: program-op only (no Document catalog variant -> BOM-only
    # purchased part; STL exported for completeness through the op surface).
    PROGRAMS["p_circlip_din471_8.json"] = {"ops": [
        {"id": "clip", "op": "circlip_external", "shaft_d": 8},
        {"id": "topo", "op": "validate", "in": "clip"},
        {"id": "vol", "op": "volume", "in": "clip"},
        {"id": "stl", "op": "export_stl", "in": "clip", "file": "circlip_din471_8.stl"},
    ]}


# ----------------------------------------------------------------------------------
# CHECK programs
# ----------------------------------------------------------------------------------
def build_checks():
    # ISO 286 fits chosen for the design (numbers quoted in README)
    PROGRAMS["check_fits.json"] = {"ops": [
        {"id": "shaft_in_608_bore", "op": "iso286_fit", "d": 8, "fit": "H7/k6"},
        {"id": "608_od_in_housing", "op": "iso286_fit", "d": 22, "fit": "H7/h6"},
        {"id": "gear_bore_on_shaft", "op": "iso286_fit", "d": 8, "fit": "H7/h6"},
        {"id": "dowel_press_in_base", "op": "iso286_fit", "d": 4, "fit": "H7/n6"},
        {"id": "dowel_slip_in_lid", "op": "iso286_fit", "d": 4, "fit": "H7/g6"},
    ]}

    def mesh_check(name, m, zp, zw, fwp, fww, cm, ph_w, roll_p):
        """Gear pair at mounted centre distance; union of the two must stay 2 shells.
        Checked in the gear-local XY frame (programs cannot pose with a general
        rotation -- FRICTION; the assembly frame is this frame rigidly rotated)."""
        ph_w_rolled = ph_w - roll_p * zp / zw
        PROGRAMS[name] = {"ops": [
            {"id": "pinion", "op": "spur_gear", "module": m, "teeth": zp, "face_width": fwp,
             "bore": SHAFT_D, "keyway": True},
            {"id": "wheel0", "op": "spur_gear", "module": m, "teeth": zw, "face_width": fww,
             "bore": SHAFT_D, "keyway": True},
            {"id": "wheel1", "op": "rotate_z", "in": "wheel0", "degrees": ph_w},
            {"id": "wheel", "op": "translate", "in": "wheel1", "offset": [cm, 0, 1.0]},
            {"id": "pair", "op": "union", "a": "pinion", "b": "wheel"},
            {"id": "no_contact_is_2_shells", "op": "validate", "in": "pair"},
            # rolled configuration: pinion turned by roll_p deg, wheel conjugately
            {"id": "pinion_r", "op": "rotate_z", "in": "pinion", "degrees": roll_p},
            {"id": "wheel0_r", "op": "spur_gear", "module": m, "teeth": zw, "face_width": fww,
             "bore": SHAFT_D, "keyway": True},
            {"id": "wheel1_r", "op": "rotate_z", "in": "wheel0_r", "degrees": ph_w_rolled},
            {"id": "wheel_r", "op": "translate", "in": "wheel1_r", "offset": [cm, 0, 1.0]},
            {"id": "pair_r", "op": "union", "a": "pinion_r", "b": "wheel_r"},
            {"id": "rolled_still_2_shells", "op": "validate", "in": "pair_r"},
            {"id": "stl", "op": "export_stl", "in": "pair", "file": f"{name[:-5]}.stl"},
        ]}

    # stage 1: wheel phase +3 deg (z=60 even -> tooth at 180; half pitch = 3 deg)
    mesh_check("check_mesh_stage1.json", P["m1"], P["z1p"], P["z1w"], FW1P, FW1W, CM1,
               PH_MID, roll_p=15.0)  # roll = half a pinion pitch (360/12/2)
    # stage 2, checked AS-ASSEMBLED: the pinion phase rides the intermediate shaft
    # keyway (+3 deg), and conjugate transfer puts the wheel at
    # base(0 for z=45) - 3*(15/45) = -1 deg (PH_OUT). Rolled config: pinion +12
    # (half a pinion pitch) -> wheel rolls a further -12*15/45 = -4.
    PROGRAMS["check_mesh_stage2.json"] = {"ops": [
        {"id": "pinion0", "op": "spur_gear", "module": P["m2"], "teeth": P["z2p"],
         "face_width": FW2P, "bore": SHAFT_D, "keyway": True},
        {"id": "pinion", "op": "rotate_z", "in": "pinion0", "degrees": PH_MID},
        {"id": "wheel0", "op": "spur_gear", "module": P["m2"], "teeth": P["z2w"],
         "face_width": FW2W, "bore": SHAFT_D, "keyway": True},
        {"id": "wheel1", "op": "rotate_z", "in": "wheel0", "degrees": PH_OUT},
        {"id": "wheel", "op": "translate", "in": "wheel1", "offset": [CM2, 0, 1.0]},
        {"id": "pair", "op": "union", "a": "pinion", "b": "wheel"},
        {"id": "no_contact_is_2_shells", "op": "validate", "in": "pair"},
        # rolled by half a pinion pitch (12 deg) -> wheel rolls -12*15/45 = -4
        {"id": "pinion_r", "op": "rotate_z", "in": "pinion", "degrees": 12.0},
        {"id": "wheel_r0", "op": "spur_gear", "module": P["m2"], "teeth": P["z2w"],
         "face_width": FW2W, "bore": SHAFT_D, "keyway": True},
        {"id": "wheel_r1", "op": "rotate_z", "in": "wheel_r0", "degrees": PH_OUT - 4.0},
        {"id": "wheel_r", "op": "translate", "in": "wheel_r1", "offset": [CM2, 0, 1.0]},
        {"id": "pair_r", "op": "union", "a": "pinion_r", "b": "wheel_r"},
        {"id": "rolled_still_2_shells", "op": "validate", "in": "pair_r"},
        {"id": "stl", "op": "export_stl", "in": "pair", "file": "check_mesh_stage2.stl"},
    ]}

    # Whole-box radial clearances: housing + every swept gear envelope + every shaft
    # envelope unioned. Gear envelopes are the exact swept solids of the rotating
    # gears (covers ALL rotations). Envelopes of meshing gears/shafts on one train
    # legitimately overlap each other and merge into ONE rotating-parts shell; the
    # check is that the HOUSING stays a separate shell: expected shells == 2.
    ops = [{"id": "housing", "op": "load_part", "file": "../parts/housing_base.lmcpart"}]
    envs = [
        ("env_g1p", X_IN, Y_G1P, tip_r(P["m1"], P["z1p"])),
        ("env_g1w", X_MID, Y_G1W, tip_r(P["m1"], P["z1w"])),
        ("env_g2p", X_MID, Y_G2P, tip_r(P["m2"], P["z2p"])),
        ("env_g2w", X_OUT, Y_G2W, tip_r(P["m2"], P["z2w"])),
        ("env_shaft_in", X_IN, (SH_IN_Y0, SH_IN_Y0 + SH_IN_LEN), SHAFT_D / 2.0),
        ("env_shaft_mid", X_MID, (SH_MID_Y0, SH_MID_Y0 + SH_MID_LEN), SHAFT_D / 2.0),
        ("env_shaft_out", X_OUT, (SH_OUT_Y0, SH_OUT_Y0 + SH_OUT_LEN), SHAFT_D / 2.0),
    ]
    prev = "housing"
    for i, (nm, x, (y0, y1), r) in enumerate(envs):
        ops.append({"id": nm, "op": "cylinder", "base": [x, y0, Z_AXIS], "axis": [0, 1, 0],
                    "radius": r, "height": y1 - y0, "segments": 48})
        ops.append({"id": f"u{i}", "op": "union", "a": prev, "b": nm})
        prev = f"u{i}"
    ops.append({"id": "housing_clear_of_swept_parts_2_shells", "op": "validate", "in": prev})
    PROGRAMS["check_envelopes.json"] = {"ops": ops}

    # Documented-friction evidence: an empty intersection is the natural "prove no
    # interference" op but it FAILS the program (exit 1) -- kept as evidence.
    PROGRAMS["check_clash_expected_fail.json"] = {"ops": [
        {"id": "pinion", "op": "spur_gear", "module": P["m1"], "teeth": P["z1p"],
         "face_width": FW1P, "bore": SHAFT_D, "keyway": True},
        {"id": "wheel0", "op": "spur_gear", "module": P["m1"], "teeth": P["z1w"],
         "face_width": FW1W, "bore": SHAFT_D, "keyway": True},
        {"id": "wheel1", "op": "rotate_z", "in": "wheel0", "degrees": PH_MID},
        {"id": "wheel", "op": "translate", "in": "wheel1", "offset": [CM1, 0, 0]},
        {"id": "clash", "op": "intersection", "a": "pinion", "b": "wheel"},
    ]}
    EXPECT_FAIL.add("check_clash_expected_fail.json")


# ----------------------------------------------------------------------------------
# ASSEMBLY (.lmcasm)
# ----------------------------------------------------------------------------------
def quat_axis_y_part(phase_deg):
    """Quaternion mapping part local +Z to world +Y (shaft axes), with a phase
    rotation about the local axis first: q = qx(-90) * qz(phase)."""
    s = c = math.sqrt(0.5)
    h = math.radians(phase_deg) / 2.0
    sz, cz = math.sin(h), math.cos(h)
    # q = qx * qz with qx = (-s,0,0,c), qz = (0,0,sz,cz)
    return [-s * cz, s * sz, c * sz, c * cz]

def pose(t, q=None):
    p = {"translation": [float(t[0]), float(t[1]), float(t[2])]}
    if q is not None:
        p["rotation"] = [float(v) for v in q]
    return p


def assembly_instances():
    """The 37 placed instances — name, part path, pose, explode delta — built
    once per caller: the single source shared by the flat gearbox.lmcasm and
    the nested gearbox_nested.lmcasm (identical world poses by construction)."""
    inst = []      # (name, part_file, pose, explode_delta)
    def add(name, part, t, q=None, explode=(0, 0, 0)):
        inst.append({"name": name, "source": {"path": f"parts/{part}.lmcpart"},
                     "pose": pose(t, q)} | {"_explode": explode})

    add("base", "housing_base", (0, 0, 0))
    add("lid", "housing_lid", (0, 0, 0), explode=(0, 0, 60))

    qi = quat_axis_y_part(PH_IN)
    qm = quat_axis_y_part(PH_MID)
    qo = quat_axis_y_part(PH_OUT)
    add("shaft_in", "shaft_input", (X_IN, SH_IN_Y0, Z_AXIS), qi, explode=(0, 0, 70))
    add("shaft_mid", "shaft_intermediate", (X_MID, SH_MID_Y0, Z_AXIS), qm, explode=(0, 0, 95))
    add("shaft_out", "shaft_output", (X_OUT, SH_OUT_Y0, Z_AXIS), qo, explode=(0, 0, 70))

    add("g1p", "gear_s1_pinion", (X_IN, Y_G1P[0], Z_AXIS), qi, explode=(0, 0, 70))
    add("g1w", "gear_s1_wheel", (X_MID, Y_G1W[0], Z_AXIS), qm, explode=(0, 0, 95))
    add("g2p", "gear_s2_pinion", (X_MID, Y_G2P[0], Z_AXIS), qm, explode=(0, 0, 95))
    add("g2w", "gear_s2_wheel", (X_OUT, Y_G2W[0], Z_AXIS), qo, explode=(0, 0, 70))

    for x, tag in [(X_IN, "in"), (X_MID, "mid"), (X_OUT, "out")]:
        add(f"608_{tag}_A", "bearing_608", (x, -10.0, Z_AXIS), quat_axis_y_part(0),
            explode=(0, -30, 0))
        add(f"608_{tag}_B", "bearing_608", (x, WEB_Y_B, Z_AXIS), quat_axis_y_part(0),
            explode=(0, 30, 0))

    for name, (length, places) in SPACERS.items():
        for j, (x, y0) in enumerate(places):
            ex = {X_IN: 70, X_MID: 95, X_OUT: 70}[x]
            add(f"spacer{name}_{j}", f"spacer_{name}", (x, y0, Z_AXIS), quat_axis_y_part(0),
                explode=(0, 0, ex))

    # keys: seated in the shaft slots. Key local: x 0..l (along axis), y +/-1, z 0..2
    # (height). Posed: local +X -> world +Y (shaft axis), local +Z -> radial at the
    # shaft's phase. Rotation = R_axis_y(phase) where columns map X->+Y etc.
    for name, (length, places) in KEYS.items():
        for j, (x, y0, ph) in enumerate(places):
            # column-major rotation: X->(0,1,0); Z->radial(phase); Y completes RH.
            a = math.radians(ph)
            # radial direction of the keyway in world (see README): (cos ph, 0, -sin ph)
            rx, rz = math.cos(a), -math.sin(a)
            # columns: X=(0,1,0), Y = Z x X = (rz*1-0, ...) compute: Y = Zc x Xc
            zc = (rx, 0.0, rz)
            xc = (0.0, 1.0, 0.0)
            yc = (zc[1] * xc[2] - zc[2] * xc[1], zc[2] * xc[0] - zc[0] * xc[2],
                  zc[0] * xc[1] - zc[1] * xc[0])
            q = mat_to_quat([xc, yc, zc])
            # key floor sits on the slot floor at radius 2.8 from the axis
            floor_r = SHAFT_D / 2.0 - KEY_T1
            t = (x + floor_r * rx, y0, Z_AXIS + floor_r * rz)
            ex = {X_IN: 70, X_MID: 95, X_OUT: 70}[x]
            add(f"key{name}_{j}", f"key_2x2_{name}", t, q, explode=(0, 0, ex))

    for i, (bx, by) in enumerate(BOLT_XY):
        # head bottom seats on the counterbore floor at LID_Z1 - CB_DEPTH_M4
        add(f"bolt_{i}", "screw_m4x12", (bx, by, LID_Z1 - CB_DEPTH_M4 - BOLT_LEN),
            explode=(0, 0, 85))
    for i, (dx, dy) in enumerate(DOWEL_XY):
        add(f"dowel_{i}", "dowel_4x12", (dx, dy, FLG_Z1 - DOWEL_BASE_DEPTH),
            explode=(0, 0, 35))
    return inst


def build_assembly():
    inst = assembly_instances()

    # mates: exactly satisfied by the stored poses (residual ~0 on load)
    idx = {e["name"]: i for i, e in enumerate(inst)}
    mates = []
    for tag, x in [("shaft_in", X_IN), ("shaft_mid", X_MID), ("shaft_out", X_OUT)]:
        mates.append({"Concentric": {
            "a": 0, "a_axis_point": [x, 0.0, Z_AXIS], "a_axis_dir": [0.0, 1.0, 0.0],
            "b": idx[tag], "b_axis_point": [0.0, 0.0, 10.0], "b_axis_dir": [0.0, 0.0, 1.0]}})
    mates.append({"Coincident": {
        "a": idx["lid"], "a_point": [FLG[0], FLG[2], LID_Z0],
        "b": 0, "b_point": [FLG[0], FLG[2], WTOP]}})

    exploded = []
    for e in inst:
        t = e["pose"]["translation"]
        dx, dy, dz = e["_explode"]
        p = {"translation": [t[0] + dx, t[1] + dy, t[2] + dz]}
        if "rotation" in e["pose"]:
            p["rotation"] = e["pose"]["rotation"]
        exploded.append(p)

    for e in inst:
        del e["_explode"]
    return {
        "format": "lmc-asm", "version": 1, "units": "mm", "name": "gearbox_2stage_15to1",
        "instances": inst, "mates": mates,
        "states": {"exploded": {"poses": exploded}},
    }


# ----------------------------------------------------------------------------------
# NESTED ASSEMBLY (.lmcasm v2, asm_path sub-assemblies)
# ----------------------------------------------------------------------------------
# The three shaft stacks of the SAME design, regrouped as path-referenced
# sub-assemblies (asm/shaft_*.lmcasm) under gearbox_nested.lmcasm. Same parts,
# same world poses: each stack's local frame has its shaft axis along +Y
# through the origin (= world (X, 0, Z_AXIS)), so member local pose = world
# pose - that offset and the parent places each stack by one translation.
# Stack membership mirrors check_asm.py's SHAFT_OF design table.
STACKS = {  # unit name -> (sub file stem, axis x, member instance names, explode dz)
    "stack_in": ("shaft_input", X_IN,
                 ["shaft_in", "g1p", "608_in_A", "608_in_B",
                  "spacer9_0", "spacer23_0", "key8_0", "key10_0"], 70),
    "stack_mid": ("shaft_intermediate", X_MID,
                  ["shaft_mid", "g1w", "g2p", "608_mid_A", "608_mid_B",
                   "spacer10_0", "spacer10_1", "key8_2", "key12_0"], 95),
    "stack_out": ("shaft_output", X_OUT,
                  ["shaft_out", "g2w", "608_out_A", "608_out_B",
                   "spacer11_0", "spacer21_0", "key8_1", "key10_1"], 70),
}


def build_nested_assembly():
    """gearbox_nested.lmcasm + the three asm/shaft_*.lmcasm sub-assemblies.

    Returns (top_envelope, {sub file name -> sub envelope}). v2 nesting
    semantics on display: each sub-assembly solves its OWN mates (every gear
    concentric on its shaft — exactly satisfied by the stored poses), then the
    parent mates pin each stack to the housing axis as ONE rigid unit, with the
    mate's b-geometry expressed in the stack's frame. The parent's "exploded"
    state poses top-level units only, so a stack rises as one piece — members
    cannot fly apart axially as in the flat exploded view (documented v2
    limit: states/mates cannot address a sub-assembly's internal members)."""
    inst = assembly_instances()
    by_name = {e["name"]: e for e in inst}
    member_of = {m: s for s, (_, _, members, _) in STACKS.items() for m in members}
    assert len(by_name) == len(inst) and set(member_of) <= set(by_name), \
        "STACKS must name real, unique instances"

    subs = {}
    for stack, (stem, x, members, _) in STACKS.items():
        sub_inst = []
        for m in members:
            e = by_name[m]
            t = e["pose"]["translation"]
            local = {"translation": [t[0] - x, t[1], t[2] - Z_AXIS]}
            if "rotation" in e["pose"]:
                local["rotation"] = e["pose"]["rotation"]
            sub_inst.append({"name": m, "source": {"path": "../" + e["source"]["path"]},
                             "pose": local})
        # Internal mates, re-solved inside the unit on every load: each gear
        # concentric on the stack's shaft (member 0). Axis geometry is in each
        # PART's local frame (shaft axis +Z through (0,0,10), gear bore +Z
        # through the origin) — exactly satisfied by the stored poses.
        sub_mates = [{"Concentric": {
            "a": 0, "a_axis_point": [0.0, 0.0, 10.0], "a_axis_dir": [0.0, 0.0, 1.0],
            "b": j, "b_axis_point": [0.0, 0.0, 0.0], "b_axis_dir": [0.0, 0.0, 1.0]}}
            for j, m in enumerate(members) if m.startswith("g")]
        subs[f"{stem}.lmcasm"] = {
            "format": "lmc-asm", "version": 1, "units": "mm", "name": f"{stem}_stack",
            "instances": sub_inst, "mates": sub_mates,
        }

    # Top level: housing + fasteners stay leaf parts; the stacks are asm_path
    # units placed by one translation each (15 instances, 37 leaf parts).
    top_inst, exploded = [], []
    def add_top(entry, explode):
        top_inst.append(entry)
        t = entry["pose"]["translation"]
        p = {"translation": [t[0] + explode[0], t[1] + explode[1], t[2] + explode[2]]}
        if "rotation" in entry["pose"]:
            p["rotation"] = entry["pose"]["rotation"]
        exploded.append(p)

    for e in inst:
        if e["name"] in member_of:
            continue
        if e["name"] == "lid":  # insert the stacks between the housing and the fasteners
            add_top({"name": e["name"], "source": e["source"], "pose": e["pose"]}, e["_explode"])
            for stack, (stem, x, _, dz) in STACKS.items():
                add_top({"name": stack, "source": {"asm_path": f"asm/{stem}.lmcasm"},
                         "pose": pose((x, 0.0, Z_AXIS))}, (0, 0, dz))
            continue
        add_top({"name": e["name"], "source": e["source"], "pose": e["pose"]}, e["_explode"])

    idx = {e["name"]: i for i, e in enumerate(top_inst)}
    mates = []
    for stack, (_, x, _, _) in STACKS.items():
        mates.append({"Concentric": {
            "a": idx["base"], "a_axis_point": [x, 0.0, Z_AXIS], "a_axis_dir": [0.0, 1.0, 0.0],
            # b-geometry in the STACK's frame: its shaft axis is +Y through the origin
            "b": idx[stack], "b_axis_point": [0.0, 0.0, 0.0], "b_axis_dir": [0.0, 1.0, 0.0]}})
    mates.append({"Coincident": {
        "a": idx["lid"], "a_point": [FLG[0], FLG[2], LID_Z0],
        "b": idx["base"], "b_point": [FLG[0], FLG[2], WTOP]}})

    top = {
        "format": "lmc-asm", "version": 1, "units": "mm", "name": "gearbox_2stage_15to1_nested",
        "instances": top_inst, "mates": mates,
        "states": {"exploded": {"poses": exploded}},
    }
    return top, subs


def mat_to_quat(cols):
    """Quaternion [x,y,z,w] from column-major rotation matrix (columns = images)."""
    # rows from columns
    m = [[cols[0][0], cols[1][0], cols[2][0]],
         [cols[0][1], cols[1][1], cols[2][1]],
         [cols[0][2], cols[1][2], cols[2][2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


# ----------------------------------------------------------------------------------
def main():
    build_housing_base()
    build_housing_lid()
    build_shafts()
    build_gears()
    build_small_parts()
    build_checks()

    os.makedirs(os.path.join(HERE, "parts"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "programs"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "asm"), exist_ok=True)
    for name, env in PARTS.items():
        if name in META:
            env = stamp_meta(env, META[name])
        with open(os.path.join(HERE, "parts", f"{name}.lmcpart"), "w") as f:
            json.dump(env, f, indent=1)
            f.write("\n")
    for fname, prog in PROGRAMS.items():
        with open(os.path.join(HERE, "programs", fname), "w") as f:
            json.dump(prog, f, indent=1)
            f.write("\n")
    asm = build_assembly()
    with open(os.path.join(HERE, "gearbox.lmcasm"), "w") as f:
        json.dump(asm, f, indent=1)
        f.write("\n")
    nested, subs = build_nested_assembly()
    for fname, sub in subs.items():
        with open(os.path.join(HERE, "asm", fname), "w") as f:
            json.dump(sub, f, indent=1)
            f.write("\n")
    with open(os.path.join(HERE, "gearbox_nested.lmcasm"), "w") as f:
        json.dump(nested, f, indent=1)
        f.write("\n")
    with open(os.path.join(HERE, "programs", "EXPECT_FAIL"), "w") as f:
        f.write("\n".join(sorted(EXPECT_FAIL)) + "\n")
    print(f"wrote {len(PARTS)} parts ({len(META)} with BOM meta), {len(PROGRAMS)} programs, "
          f"gearbox.lmcasm ({len(asm['instances'])} instances), gearbox_nested.lmcasm "
          f"({len(nested['instances'])} top-level units, {len(subs)} sub-assemblies)")
    print(f"C1={C1} C2={C2} mounted={CM1}/{CM2} ratio="
          f"{(P['z1w']/P['z1p'])*(P['z2w']/P['z2p']):g}:1")


if __name__ == "__main__":
    main()
