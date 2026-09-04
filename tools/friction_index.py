#!/usr/bin/env python3
"""friction_index.py — roll the friction logs up into one sortable table.

`campaign/friction/` is the richest record the repo has of what actually fights
back when a model drives this engine: ~190 items across 20+ campaigns. As a
pile of prose it is unusable as a work queue — you cannot sort it by blast
radius, and nothing notices when six campaigns independently hit the SAME
defect. Six independent reports of one broken behaviour is a specification, not
six complaints.

This tool reads every `campaign/friction/*.md`, parses the three machine-read
fields the friction contract requires on every item (DELIVERABLE_SPEC "FRICTION
PROTOCOL"):

    - severity: blocker | major | minor | papercut | note
    - surface:  the one op / tool path / named surface — THE ROLLUP KEY
    - status:   open | partial | fixed — <what fixed it>

and emits `docs/FRICTION_INDEX.md`: one table, grouped by `surface`, ordered by
how many DISTINCT source logs hit it. The output is a pure function of the
inputs — no dates, no run ids — so regenerating it twice is byte-identical and
a stale index is detectable (`--check`, and the `friction` class in
`tools/audit_docs.py`).

Usage:
  python3 tools/friction_index.py                 # regenerate docs/FRICTION_INDEX.md
  python3 tools/friction_index.py --check         # exit 1 if the file on disk is stale
  python3 tools/friction_index.py --stdout        # print, write nothing
  python3 tools/friction_index.py --json          # machine-readable rollup
  python3 tools/friction_index.py --surfaces      # every surface token in use, with counts
  python3 tools/friction_index.py --surface union_all
                                                  # every item filed against one surface
"""

import argparse
import json
import re
import sys
from pathlib import Path

FRICTION_DIR = "campaign/friction"
OUT_REL = "docs/FRICTION_INDEX.md"
SKIP_FILES = {"README.md"}

# logs that are NOT a single campaign: the engine-wide record and the digest
# readers' pass. They are counted as one SOURCE each, and flagged, so a row's
# repeat count is never inflated by a cross-campaign log.
NON_CAMPAIGN = {"ENGINE.md", "_digest_phase_findings.md"}

SEVERITIES = ["blocker", "major", "minor", "papercut", "note"]
SEV_RANK = {s: i for i, s in enumerate(SEVERITIES)}
# `note` is not friction (balancing evidence kept in the log on purpose); it is
# parsed, counted separately, and excluded from every rollup row.
GRADED = [s for s in SEVERITIES if s != "note"]

# `## F1 — title`, `## F5 addendum — title`, `## 7. [MAJOR] title`, `## #23 — title`
ITEM_RE = re.compile(r"^##\s+(#?\d+|[Ff]\d+[a-z]?)(?=[.\s—-]|$)")
FIELD_RE = re.compile(r"^\s*[-*]\s*(severity|surface|status|recurrence of)\s*:\s*(.+?)\s*$",
	re.IGNORECASE)


def slug(text):
	"""GitHub-flavoured heading anchor (same rule as tools/audit_docs.py)."""
	s = text.strip().lower()
	s = re.sub(r"`", "", s)
	s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)
	s = re.sub(r"[^\w\s-]", "", s, flags=re.UNICODE)
	s = re.sub(r"\s+", "-", s)
	return s


def norm_surface(raw):
	"""Canonical form of a surface token: no backticks, no trailing
	punctuation, single-spaced. CASE IS PRESERVED — half these tokens are repo
	paths (`campaign/DESIGN_GUIDE.md`), and a case-folded path is a path that
	does not exist on a case-sensitive filesystem. Grouping is case-insensitive
	(`fold()`); only the display spelling keeps its case."""
	s = raw.strip().strip("`").strip()
	s = re.sub(r"\s+", " ", s)
	return s.rstrip(".,;")


def fold(surface):
	"""The rollup key: two spellings that differ only in case are one surface."""
	return surface.casefold()


class Item:
	def __init__(self, rel, ident, title, line):
		self.rel = rel				# campaign/friction/<file>.md
		self.file = Path(rel).name
		self.ident = ident.lstrip("#")
		self.title = title
		self.line = line
		self.severity = None
		self.surface = None
		self.status = None
		self.recurrence = None

	@property
	def campaign(self):
		return self.file[:-3] if self.file.endswith(".md") else self.file

	@property
	def is_campaign(self):
		return self.file not in NON_CAMPAIGN

	@property
	def fixed(self):
		return bool(self.status) and self.status.lower().startswith("fixed")

	@property
	def partial(self):
		return bool(self.status) and self.status.lower().startswith("partial")

	@property
	def link(self):
		return f"[{self.campaign}#{self.ident}](../{self.rel}#{slug(self.title)})"

	def as_dict(self):
		return {"file": self.rel, "id": self.ident, "title": self.title,
			"line": self.line, "severity": self.severity, "surface": self.surface,
			"status": self.status, "recurrence_of": self.recurrence,
			"is_campaign": self.is_campaign}


def parse_file(root, rel):
	"""Parse one friction log into Items. An `##` heading with no finding
	number is a prose section (RESOLUTIONS, Fix log, Disposition) and is not an
	item."""
	text = (Path(root) / rel).read_text(encoding="utf-8")
	items = []
	seen = set()
	cur = None
	in_fence = False
	for lineno, line in enumerate(text.splitlines(), 1):
		if re.match(r"^\s*(?:```+|~~~+)", line):
			in_fence = not in_fence
			continue
		if in_fence:
			continue
		if line.startswith("## "):
			m = ITEM_RE.match(line)
			if m:
				title = line[3:].strip()
				ident = m.group(1).lstrip("#")
				if ident in seen:
					# `## F5 addendum — …` sits beside `## F5 — …`: disambiguate
					# from the heading itself so the label still points somewhere.
					tail = re.sub(r"[^\w]+", "-", title[m.end() - 3:].strip()).strip("-")
					suffix = tail.split("-")[0].lower() if tail else ""
					n = 2
					cand = f"{ident}-{suffix}" if suffix else f"{ident}-{n}"
					while cand in seen:
						n += 1
						cand = f"{ident}-{n}"
					ident = cand
				seen.add(ident)
				cur = Item(rel, ident, title, lineno)
				items.append(cur)
			else:
				cur = None
			continue
		if cur is None:
			continue
		fm = FIELD_RE.match(line)
		if not fm:
			continue
		key, val = fm.group(1).lower(), fm.group(2).strip()
		if key == "severity" and cur.severity is None:
			cur.severity = val.split()[0].strip("`*_").lower()
		elif key == "surface" and cur.surface is None:
			cur.surface = norm_surface(val)
		elif key == "status" and cur.status is None:
			cur.status = val.strip("`*_")
		elif key == "recurrence of" and cur.recurrence is None:
			cur.recurrence = val.strip("`*_")
	return items


def parse_all(root):
	root = Path(root)
	d = root / FRICTION_DIR
	if not d.is_dir():
		return []
	items = []
	for p in sorted(d.glob("*.md")):
		if p.name in SKIP_FILES:
			continue
		items.append(parse_file(root, p.relative_to(root).as_posix()))
	return [it for group in items for it in group]


def rollup(items):
	"""Group graded items by surface. Returns rows sorted most-repeated first."""
	by_surface = {}
	for it in items:
		if it.severity == "note":
			continue
		key = fold(it.surface or "(no surface field)")
		by_surface.setdefault(key, []).append(it)

	rows = []
	for group in by_surface.values():
		group = sorted(group, key=lambda i: (i.file, i.ident))
		# display spelling: the most-used original, ties broken lexicographically
		spellings = {}
		for i in group:
			s = i.surface or "(no surface field)"
			spellings[s] = spellings.get(s, 0) + 1
		surface = sorted(spellings.items(), key=lambda kv: (-kv[1], kv[0]))[0][0]
		sources = sorted({i.file for i in group})
		campaigns = sorted({i.campaign for i in group if i.is_campaign})
		graded = [i for i in group if i.severity in SEV_RANK]
		worst = min((i.severity for i in graded),
			key=lambda s: SEV_RANK[s], default="ungraded")
		n_fixed = sum(1 for i in group if i.fixed)
		n_partial = sum(1 for i in group if i.partial)
		if n_fixed == len(group):
			state = "fixed"
		elif n_fixed or n_partial:
			state = f"partial ({n_fixed}/{len(group)} fixed)"
		else:
			state = "OPEN"
		rows.append({
			"surface": surface,
			"sources": sources,
			"n_sources": len(sources),
			"campaigns": campaigns,
			"n_campaigns": len(campaigns),
			"items": group,
			"n_items": len(group),
			"worst": worst,
			"state": state,
			"n_fixed": n_fixed,
		})
	rows.sort(key=lambda r: (-r["n_sources"], -r["n_items"],
		SEV_RANK.get(r["worst"], 99), fold(r["surface"])))
	return rows


def stats(items):
	sev = {s: 0 for s in SEVERITIES}
	sev["ungraded"] = 0
	for it in items:
		sev[it.severity if it.severity in sev else "ungraded"] += 1
	graded = [i for i in items if i.severity != "note"]
	return {
		"items_total": len(items),
		"items_graded": len(graded),
		"severity": sev,
		"ungraded": [i for i in items if i.severity not in SEVERITIES],
		"no_surface": [i for i in graded if not i.surface],
		"fixed": sum(1 for i in graded if i.fixed),
		"partial": sum(1 for i in graded if i.partial),
		"open": sum(1 for i in graded if not i.fixed and not i.partial),
	}


BADGE = {"blocker": "**blocker**", "major": "major", "minor": "minor",
	"papercut": "papercut", "ungraded": "**UNGRADED**"}


def render(root, items):
	rows = rollup(items)
	st = stats(items)
	files = sorted({i.rel for i in items})
	campaigns = sorted({i.campaign for i in items if i.is_campaign})
	L = []
	A = L.append

	A("# FRICTION_INDEX.md — every logged friction item, rolled up by surface")
	A("")
	A("**Generated. Do not edit by hand.**")
	A("`python3 tools/friction_index.py` rewrites this file from")
	A("`campaign/friction/*.md`; `python3 tools/friction_index.py --check` fails when")
	A("it has drifted, and `tools/audit_docs.py` runs that check as its `friction`")
	A("class, so an index that no longer matches the logs is a doc-audit finding.")
	A("")
	A("The friction logs are the repo's record of what fights back when a model")
	A("drives this engine. Read as prose they are 20+ separate complaints; read as")
	A("this table they are a work queue ordered by blast radius. A surface that")
	A("several independent campaigns hit is not bad luck — it is a specification for")
	A("a fix.")
	A("")

	A("## What this counts, and what it cannot see")
	A("")
	A("- **One row per `surface` token.** The rollup key is the `surface:` field the")
	A("  friction contract requires on every item. Tokens are compared lowercased")
	A("  with backticks stripped and whitespace collapsed — nothing smarter. Two")
	A("  spellings of the same surface are two rows, and that is the failure mode to")
	A("  watch for: run `--surfaces` before inventing a token.")
	A("- **\"logs\" counts DISTINCT source files, not items.** Three items in one")
	A("  campaign's file are one log. The campaign column excludes the two")
	A("  cross-campaign logs (`ENGINE.md`, `_digest_phase_findings.md`), which are")
	A("  not campaigns; they still count as a log and still appear in the")
	A("  occurrences.")
	A("- **Worst severity is the worst grade any occurrence carries**, and a fixed")
	A("  item keeps the grade it earned — severity records what it cost, `status`")
	A("  records whether it still costs it.")
	A("- **`note` items are excluded from every row.** They are context the authors")
	A("  kept on purpose (\"what worked better than expected\"), not friction.")
	A("- **It cannot see friction nobody logged.** Every campaign that solved a")
	A("  problem silently is invisible here, and the corpus is retrospective: items")
	A("  were graded in one pass against the evidence in each entry, so a grade is")
	A("  an honest reading of the write-up, not a re-run of the failure.")
	A("- **It cannot tell you a row is one BUG.** It tells you several campaigns")
	A("  named one surface. Whether those are one defect or four unrelated ones in")
	A("  the same file is a judgement the reader makes from the linked entries — the")
	A("  index points, it does not diagnose.")
	A("- **`status` is what the logs say.** An item is `fixed` here because an entry,")
	A("  a `RESOLUTIONS` section or a `Fix log` in the same file says so. This tool")
	A("  does not re-verify a fix against the binary.")
	A("")

	A("## Corpus")
	A("")
	A(f"| logs read | {len(files)} files in `campaign/friction/` "
		f"({len(campaigns)} campaigns + {len(files) - len(campaigns)} cross-campaign) |")
	A("|---|---|")
	A(f"| items parsed | {st['items_total']} "
		f"({st['items_graded']} graded, {st['severity']['note']} `note`) |")
	A("| severity | " + " · ".join(
		f"{s} {st['severity'][s]}" for s in GRADED) + " |")
	A(f"| disposition | open {st['open']} · partial {st['partial']} · "
		f"fixed {st['fixed']} |")
	A(f"| distinct surfaces | {len(rows)} |")
	if st["ungraded"]:
		A(f"| **ungraded items** | **{len(st['ungraded'])} — the contract requires "
			"`severity:` on every item** |")
	if st["no_surface"]:
		A(f"| **items with no surface** | **{len(st['no_surface'])}** |")
	A("")

	A("## The rollup — most-repeated surface first")
	A("")
	A("| surface | logs | items | worst | state | occurrences |")
	A("|---|---|---|---|---|---|")
	for r in rows:
		occ = " · ".join(i.link for i in r["items"])
		A(f"| `{r['surface']}` | {r['n_sources']} | {r['n_items']} | "
			f"{BADGE.get(r['worst'], r['worst'])} | {r['state']} | {occ} |")
	A("")

	repeats = [r for r in rows if r["n_sources"] >= 2]
	A("## Read this first: the repeat offenders")
	A("")
	if repeats:
		A(f"{len(repeats)} surfaces were hit by more than one log. They are the top")
		A("of the table above; the ones still open are the queue:")
		A("")
		A("| surface | logs | worst | state |")
		A("|---|---|---|---|")
		for r in repeats:
			A(f"| `{r['surface']}` | {r['n_sources']} | "
				f"{BADGE.get(r['worst'], r['worst'])} | {r['state']} |")
	else:
		A("No surface was hit by more than one log.")
	A("")

	if st["ungraded"]:
		A("## Ungraded items (contract violations)")
		A("")
		A("Every item must carry `severity:` (DELIVERABLE_SPEC, FRICTION PROTOCOL).")
		A("These do not, so they are missing from the rollup's severity column:")
		A("")
		for i in sorted(st["ungraded"], key=lambda x: (x.rel, x.ident)):
			A(f"- `{i.rel}:{i.line}` — {i.ident}")
		A("")

	A("## Using it")
	A("")
	A("```sh")
	A("python3 tools/friction_index.py --surfaces          # every token in use")
	A("python3 tools/friction_index.py --surface union_all # every item on one surface")
	A("python3 tools/friction_index.py --json              # the rollup, machine-readable")
	A("python3 tools/friction_index.py --check             # is this file stale?")
	A("```")
	A("")
	A("Before a campaign logs a new item it checks this table for the surface it is")
	A("about to name; when it finds one, its own entry carries a `recurrence of:`")
	A("line and the final self-check lists the id (DELIVERABLE_SPEC, \"Recurrence")
	A("stated\"). That is what keeps a defect from being rediscovered in silence.")
	return "\n".join(L) + "\n"


def cmd_surfaces(items):
	rows = rollup(items)
	print(f"{'surface':<44} logs items worst      state")
	print("-" * 92)
	for r in sorted(rows, key=lambda r: r["surface"]):
		print(f"{r['surface']:<44} {r['n_sources']:>4} {r['n_items']:>5} "
			f"{r['worst']:<10} {r['state']}")
	print(f"\n{len(rows)} distinct surface token(s).")


def cmd_surface(items, want):
	want = norm_surface(want)
	hits = [i for i in items if (i.surface or "") == want]
	if not hits:
		rows = rollup(items)
		near = [r["surface"] for r in rows if want in r["surface"] or r["surface"] in want]
		print(f"no items filed against surface `{want}`.")
		if near:
			print("did you mean: " + ", ".join(sorted(near)))
		return 1
	print(f"surface `{want}` — {len(hits)} item(s) in "
		f"{len({i.file for i in hits})} log(s)\n")
	for i in sorted(hits, key=lambda x: (x.file, x.ident)):
		print(f"  {i.rel}:{i.line}")
		print(f"    {i.ident}  [{i.severity}]  status: {i.status or '(none)'}")
		print(f"    {i.title}")
		if i.recurrence:
			print(f"    recurrence of: {i.recurrence}")
		print()
	return 0


def main(argv=None):
	ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
		formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("--root", default=None, help="repo root (default: this file's parent's parent)")
	ap.add_argument("--check", action="store_true",
		help="exit 1 if docs/FRICTION_INDEX.md differs from the regenerated text")
	ap.add_argument("--stdout", action="store_true", help="print the index, write nothing")
	ap.add_argument("--json", action="store_true", help="machine-readable rollup")
	ap.add_argument("--surfaces", action="store_true", help="list every surface token in use")
	ap.add_argument("--surface", metavar="TOKEN", help="list every item on one surface")
	args = ap.parse_args(argv)

	root = Path(args.root) if args.root else Path(__file__).resolve().parent.parent
	items = parse_all(root)
	if not items:
		print(f"no friction items found under {root}/{FRICTION_DIR}", file=sys.stderr)
		return 1

	if args.surfaces:
		cmd_surfaces(items)
		return 0
	if args.surface:
		return cmd_surface(items, args.surface)
	if args.json:
		st = stats(items)
		out = {
			"corpus": {"files": sorted({i.rel for i in items}),
				"items": st["items_total"], "graded": st["items_graded"]},
			"severity": st["severity"],
			"disposition": {"open": st["open"], "partial": st["partial"],
				"fixed": st["fixed"]},
			"ungraded": [i.as_dict() for i in st["ungraded"]],
			"rows": [{"surface": r["surface"], "logs": r["n_sources"],
				"campaigns": r["campaigns"], "items": r["n_items"],
				"worst": r["worst"], "state": r["state"],
				"occurrences": [i.as_dict() for i in r["items"]]} for r in rollup(items)],
		}
		print(json.dumps(out, indent=2, sort_keys=False))
		return 0

	text = render(root, items)
	if args.stdout:
		sys.stdout.write(text)
		return 0

	dest = root / OUT_REL
	if args.check:
		have = dest.read_text(encoding="utf-8") if dest.is_file() else None
		if have == text:
			print(f"{OUT_REL} is current ({len(items)} items, "
				f"{len(rollup(items))} surfaces).")
			return 0
		if have is None:
			print(f"{OUT_REL} is MISSING — run `python3 tools/friction_index.py`",
				file=sys.stderr)
		else:
			print(f"{OUT_REL} is STALE — run `python3 tools/friction_index.py`",
				file=sys.stderr)
		return 1

	dest.parent.mkdir(parents=True, exist_ok=True)
	dest.write_text(text, encoding="utf-8")
	st = stats(items)
	print(f"wrote {OUT_REL}: {st['items_total']} items "
		f"({st['items_graded']} graded) from {len({i.rel for i in items})} logs "
		f"→ {len(rollup(items))} surfaces; "
		+ " · ".join(f"{s} {st['severity'][s]}" for s in GRADED))
	if st["ungraded"]:
		print(f"WARNING: {len(st['ungraded'])} item(s) carry no severity",
			file=sys.stderr)
	return 0


if __name__ == "__main__":
	sys.exit(main())
