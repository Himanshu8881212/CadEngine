#!/usr/bin/env python3
"""design_revisions.py — never lose a design: snapshot, list, diff and restore a campaign's revisions.

A change request ("make the nose longer", "screw the joints") is applied to the
generators IN PLACE, and the old design is gone the moment the pipeline reruns.
This tool keeps every shipped revision of a campaign under `<campaign>/revisions/`
so a design can always be brought back, compared, or shipped again.

Usage:
    python3 tools/publish/design_revisions.py snapshot <campaign> --rev <name> [--note TEXT] [--if-changed] [--light] [--dest DIR] [--out receipt.json]
    python3 tools/publish/design_revisions.py list     <campaign> [--out receipt.json]
    python3 tools/publish/design_revisions.py diff     <campaign> <revA> <revB> [--out receipt.json]
    python3 tools/publish/design_revisions.py restore  <campaign> <rev> [--no-backup] [--out receipt.json]

What a snapshot holds (relative to the campaign directory):
    programs/**            every generator (.py) and every kernel program (.json): the design's SOURCE
    analysis/*.md README.md assembly/*.md assembly/*.csv assembly/*.json publish/*.md
                           the documents of that revision
    parts/*.stl            the print files
    receipts/*.json        the top-level receipts (proofs); solver sub-directories are not copied
    plates/* renders/*.png cad/*.step
                           the deliverables (skipped with --light)
    manifest.json          rev, date, note, md5 + size of every file, the freeze's numbers
                           (programs/design_freeze.json if present) and the mass-budget headline
                           (receipts/mass_budget.json if present)
`revisions/REVISIONS.md` is rewritten after every snapshot: one row per revision with its headline
numbers and the restore command. A revision is never overwritten: snapshotting an existing name refuses.

--if-changed   skip (exit 0, receipt `skipped: true`) when the DESIGN INPUTS (programs/*.py,
               programs/design_freeze.json, parts/*.stl) are byte-identical to the latest revision's —
               the hook for `run_all.sh`: `snapshot <campaign> --rev auto --if-changed` at the end of a
               green run records every design that ever shipped and nothing twice.
--rev auto     names the revision `green_<YYYYMMDD>_<HHMM>`.
restore        copies the revision's files back over the campaign (after snapshotting the current state
               as `pre_restore_<YYYYMMDD>_<HHMM>` unless --no-backup), then prints the rebuild command;
               receipts/renders/plates of the restored revision come back too, but rerun `run_all.sh`
               to regenerate them from the restored generators before claiming anything.

Receipt (last stdout line, and --out): {"ok", "action", "campaign", "rev", "dir", "n_files", "bytes",
"skipped", "changed_since_latest", ...}. Exit 0 ok / 1 refused (missing campaign, duplicate rev,
unknown rev).
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import os
import shutil
import sys
import time

DESIGN_INPUT_GLOBS = ("programs/*.py", "programs/design_freeze.json", "parts/*.stl")
FULL_GLOBS = ("programs/**", "analysis/*.md", "README.md", "assembly/*.md", "assembly/*.csv", "assembly/*.json",
              "publish/*.md", "parts/*.stl", "receipts/*.json", "plates/*", "renders/*.png", "cad/*.step")
LIGHT_SKIP = ("plates/*", "renders/*.png", "cad/*.step")
FREEZE_KEYS = ("span_mm", "root_chord_mm", "tip_chord_mm", "sweep_le_deg", "twist_tip_deg", "twist_exponent",
               "cruise_airspeed_m_s", "x_cg_mm", "x_np_mm", "x_cg_pct_mac", "optimizer_best_L_over_D", "mass_kg")


def _md5(path: str) -> str:
	h = hashlib.md5()
	with open(path, "rb") as f:
		for chunk in iter(lambda: f.read(1 << 20), b""):
			h.update(chunk)
	return h.hexdigest()


def _collect(camp: str, globs) -> list[str]:
	"""Relative paths of the files matching the globs (files only, sorted, no __pycache__, no revisions/)."""
	out = set()
	for g in globs:
		if g.endswith("/**"):
			root = os.path.join(camp, g[:-3])
			if os.path.isdir(root):
				for dp, dns, fns in os.walk(root):
					dns[:] = [d for d in dns if d != "__pycache__"]
					for fn in fns:
						if not fn.endswith(".pyc"):
							out.add(os.path.relpath(os.path.join(dp, fn), camp))
			continue
		d, pat = os.path.split(g)
		root = os.path.join(camp, d) if d else camp
		if not os.path.isdir(root):
			continue
		for fn in os.listdir(root):
			p = os.path.join(root, fn)
			if os.path.isfile(p) and fnmatch.fnmatch(fn, pat):
				out.add(os.path.relpath(p, camp))
	return sorted(r for r in out if not r.startswith("revisions" + os.sep))


def _manifest_files(camp: str, rels) -> dict:
	return {r: {"md5": _md5(os.path.join(camp, r)), "bytes": os.path.getsize(os.path.join(camp, r))} for r in rels}


def _headline(camp: str) -> dict:
	h = {}
	fz = os.path.join(camp, "programs", "design_freeze.json")
	if os.path.isfile(fz):
		try:
			f = json.load(open(fz))
			h["freeze"] = {k: f[k] for k in FREEZE_KEYS if k in f and not isinstance(f[k], (dict, list))}
		except (json.JSONDecodeError, OSError):
			pass
	mb = os.path.join(camp, "receipts", "mass_budget.json")
	if os.path.isfile(mb):
		try:
			m = json.load(open(mb))
			hl = {}
			if "printed_total_g" in m and "buy_total_g" in m:
				hl["all_up_g"] = round(float(m["printed_total_g"]) + float(m["buy_total_g"]), 1)
				hl["printed_g"] = round(float(m["printed_total_g"]), 1)
			for k in ("cg_pct_mac", "battery_center_x_mm"):
				if k in m:
					hl[k] = round(float(m[k]), 2)
			h["mass_budget"] = hl
		except (json.JSONDecodeError, OSError, TypeError, ValueError):
			pass
	return h


def _rev_dir(camp: str, dest: str | None) -> str:
	return dest or os.path.join(camp, "revisions")


def _load_manifests(revdir: str) -> list[dict]:
	out = []
	if not os.path.isdir(revdir):
		return out
	for name in sorted(os.listdir(revdir)):
		mp = os.path.join(revdir, name, "manifest.json")
		if os.path.isfile(mp):
			try:
				m = json.load(open(mp))
				m["_dir"] = os.path.join(revdir, name)
				out.append(m)
			except (json.JSONDecodeError, OSError):
				pass
	out.sort(key=lambda m: m.get("date", ""))
	return out


def _design_signature(files: dict) -> dict:
	"""md5 of the design INPUTS only (the generators, the freeze, the print files)."""
	sig = {}
	for r, v in files.items():
		rn = r.replace(os.sep, "/")
		if any(fnmatch.fnmatch(rn, g) for g in DESIGN_INPUT_GLOBS):
			sig[rn] = v["md5"]
	return sig


def _write_index(revdir: str) -> str:
	ms = _load_manifests(revdir)
	L = ["# Design revisions", "",
	     "Every shipped revision of this campaign, oldest first. A revision is a copy of the design's SOURCE",
	     "(`programs/`), its documents, print files, receipts and deliverables at that moment; it is never",
	     "overwritten. Restore one with the command in its row (the current state is snapshotted first), then",
	     "rerun `run_all.sh` to regenerate the receipts from the restored generators.", "",
	     "| revision | date | note | files | MB | freeze (tip / sweep / washout / twist exp / V) | mass (all-up / printed g) | restore |",
	     "|---|---|---|---|---|---|---|---|"]
	for m in ms:
		fz = m.get("headline", {}).get("freeze", {})
		mb = m.get("headline", {}).get("mass_budget", {})
		fzs = " / ".join(str(fz[k]) for k in ("tip_chord_mm", "sweep_le_deg", "twist_tip_deg", "twist_exponent", "cruise_airspeed_m_s") if k in fz) or "—"
		mbs = f"{mb.get('all_up_g', '—')} / {mb.get('printed_g', '—')}" if mb else "—"
		L.append(f"| **{m['rev']}** | {m.get('date', '')} | {m.get('note', '')} | {m.get('n_files', 0)} | {m.get('bytes', 0) / 1e6:.1f} | {fzs} | {mbs} | "
		         f"`python3 tools/publish/design_revisions.py restore <campaign> {m['rev']}` |")
	L.append("")
	p = os.path.join(revdir, "REVISIONS.md")
	os.makedirs(revdir, exist_ok=True)
	with open(p, "w") as f:
		f.write("\n".join(L))
	return p


def snapshot(camp: str, rev: str, note: str, if_changed: bool, light: bool, dest: str | None) -> dict:
	camp = os.path.abspath(camp)
	if not os.path.isdir(camp):
		return {"ok": False, "action": "snapshot", "error": f"no such campaign directory: {camp}"}
	if rev == "auto":
		rev = "green_" + time.strftime("%Y%m%d_%H%M")
	if not rev or "/" in rev or rev.startswith("."):
		return {"ok": False, "action": "snapshot", "error": f"bad revision name {rev!r}"}
	revdir = _rev_dir(camp, dest)
	target = os.path.join(revdir, rev)
	globs = tuple(g for g in FULL_GLOBS if not (light and g in LIGHT_SKIP))
	rels = _collect(camp, globs)
	files = _manifest_files(camp, rels)
	latest = _load_manifests(revdir)
	changed = True
	if latest:
		changed = _design_signature(files) != _design_signature(latest[-1].get("files", {}))
	if if_changed and not changed:
		rec = {"ok": True, "action": "snapshot", "campaign": camp, "rev": rev, "skipped": True, "changed_since_latest": False,
		       "latest": latest[-1]["rev"], "n_files": len(files), "note": "design inputs identical to the latest revision — nothing recorded"}
		return rec
	if os.path.exists(target):
		return {"ok": False, "action": "snapshot", "campaign": camp, "rev": rev, "error": f"revision {rev!r} exists — revisions are never overwritten; pick a new name"}
	os.makedirs(target, exist_ok=True)
	for r in rels:
		dst = os.path.join(target, r)
		os.makedirs(os.path.dirname(dst), exist_ok=True)
		shutil.copy2(os.path.join(camp, r), dst)
	man = {"rev": rev, "date": time.strftime("%Y-%m-%d %H:%M"), "note": note or "", "campaign": os.path.basename(camp),
	       "light": bool(light), "n_files": len(files), "bytes": sum(v["bytes"] for v in files.values()),
	       "headline": _headline(camp), "files": files}
	with open(os.path.join(target, "manifest.json"), "w") as f:
		json.dump(man, f, indent=1)
	idx = _write_index(revdir)
	return {"ok": True, "action": "snapshot", "campaign": camp, "rev": rev, "dir": target, "n_files": len(files), "bytes": man["bytes"],
	        "skipped": False, "changed_since_latest": changed, "headline": man["headline"], "index": idx}


def list_revs(camp: str, dest: str | None) -> dict:
	camp = os.path.abspath(camp)
	revdir = _rev_dir(camp, dest)
	ms = _load_manifests(revdir)
	rows = [{"rev": m["rev"], "date": m.get("date"), "note": m.get("note"), "n_files": m.get("n_files"), "bytes": m.get("bytes"),
	         "headline": m.get("headline", {}), "dir": m["_dir"]} for m in ms]
	return {"ok": True, "action": "list", "campaign": camp, "n_revisions": len(rows), "revisions": rows,
	        "index": os.path.join(revdir, "REVISIONS.md") if rows else None}


def diff(camp: str, a: str, b: str, dest: str | None) -> dict:
	camp = os.path.abspath(camp)
	revdir = _rev_dir(camp, dest)
	ms = {m["rev"]: m for m in _load_manifests(revdir)}
	if a not in ms or b not in ms:
		return {"ok": False, "action": "diff", "error": f"unknown revision(s): {[r for r in (a, b) if r not in ms]}; known: {sorted(ms)}"}
	fa, fb = ms[a].get("files", {}), ms[b].get("files", {})
	added = sorted(set(fb) - set(fa)); removed = sorted(set(fa) - set(fb))
	changed = sorted(r for r in set(fa) & set(fb) if fa[r]["md5"] != fb[r]["md5"])
	ha, hb = ms[a].get("headline", {}), ms[b].get("headline", {})
	fz = {k: (ha.get("freeze", {}).get(k), hb.get("freeze", {}).get(k)) for k in FREEZE_KEYS
	      if ha.get("freeze", {}).get(k) != hb.get("freeze", {}).get(k) and (k in ha.get("freeze", {}) or k in hb.get("freeze", {}))}
	mb = {k: (ha.get("mass_budget", {}).get(k), hb.get("mass_budget", {}).get(k)) for k in ("all_up_g", "printed_g", "cg_pct_mac", "battery_center_x_mm")
	      if ha.get("mass_budget", {}).get(k) != hb.get("mass_budget", {}).get(k)}
	design_changed = _design_signature(fa) != _design_signature(fb)
	return {"ok": True, "action": "diff", "campaign": camp, "a": a, "b": b, "design_inputs_differ": design_changed,
	        "added": added, "removed": removed, "changed": changed, "n_changed": len(changed),
	        "freeze_deltas": fz, "mass_budget_deltas": mb}


def restore(camp: str, rev: str, backup: bool, dest: str | None) -> dict:
	camp = os.path.abspath(camp)
	revdir = _rev_dir(camp, dest)
	ms = {m["rev"]: m for m in _load_manifests(revdir)}
	if rev not in ms:
		return {"ok": False, "action": "restore", "error": f"unknown revision {rev!r}; known: {sorted(ms)}"}
	pre = None
	if backup:
		pre = snapshot(camp, "pre_restore_" + time.strftime("%Y%m%d_%H%M%S"), f"automatic snapshot before restoring {rev}", False, False, dest)
		if not pre.get("ok"):
			return {"ok": False, "action": "restore", "error": f"could not snapshot the current state first: {pre.get('error')}"}
	src = ms[rev]["_dir"]
	n = 0
	for r in ms[rev].get("files", {}):
		s = os.path.join(src, r)
		if not os.path.isfile(s):
			continue
		d = os.path.join(camp, r)
		os.makedirs(os.path.dirname(d), exist_ok=True)
		shutil.copy2(s, d)
		n += 1
	return {"ok": True, "action": "restore", "campaign": camp, "rev": rev, "n_files": n,
	        "backup_rev": pre["rev"] if pre else None,
	        "next": f"sh {os.path.relpath(os.path.join(camp, 'run_all.sh'))} from the workspace root — regenerate every receipt from the restored generators before claiming anything"}


def main(argv=None) -> int:
	ap = argparse.ArgumentParser(prog="design_revisions.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	sub = ap.add_subparsers(dest="cmd", required=True)
	s = sub.add_parser("snapshot", help="record the campaign's current design as a revision")
	s.add_argument("campaign"); s.add_argument("--rev", required=True, help="revision name, or 'auto'")
	s.add_argument("--note", default=""); s.add_argument("--if-changed", action="store_true")
	s.add_argument("--light", action="store_true", help="skip plates/, renders/, cad/"); s.add_argument("--dest", default=None)
	s.add_argument("--out", default=None)
	l = sub.add_parser("list", help="list the revisions"); l.add_argument("campaign"); l.add_argument("--dest", default=None); l.add_argument("--out", default=None)
	d = sub.add_parser("diff", help="what changed between two revisions"); d.add_argument("campaign"); d.add_argument("a"); d.add_argument("b")
	d.add_argument("--dest", default=None); d.add_argument("--out", default=None)
	r = sub.add_parser("restore", help="bring a revision back (snapshots the current state first)"); r.add_argument("campaign"); r.add_argument("rev")
	r.add_argument("--no-backup", action="store_true"); r.add_argument("--dest", default=None); r.add_argument("--out", default=None)
	a = ap.parse_args(argv)
	if a.cmd == "snapshot":
		rec = snapshot(a.campaign, a.rev, a.note, a.if_changed, a.light, a.dest)
	elif a.cmd == "list":
		rec = list_revs(a.campaign, a.dest)
	elif a.cmd == "diff":
		rec = diff(a.campaign, a.a, a.b, a.dest)
	else:
		rec = restore(a.campaign, a.rev, not a.no_backup, a.dest)
	if a.out:
		os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
		with open(a.out, "w") as f:
			json.dump(rec, f, indent=1)
	print(json.dumps(rec))
	return 0 if rec.get("ok") else 1


if __name__ == "__main__":
	sys.exit(main())
