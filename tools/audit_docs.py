#!/usr/bin/env python3
"""audit_docs.py — mechanical doc-drift auditor for the LMCAD knowledge layer.

The repo claims "the docs are accurate". That claim was false twice already
(Appendix A said 116 ops while `op_tag` was at 155; README still says 142).
This tool makes the claim falsifiable: it derives ground truth from SOURCE and
reports every doc statement that contradicts it, with file:line and severity.

Check classes (each independently reported, each independently self-tested):

  op-count      every numeric "<N> ops" claim in the doc corpus vs the
                mechanical count of `op_tag` match arms in
                crates/kernel-api/src/discover.rs (cross-checked against
                OP_NAMES and OP_COUNT in the same file).
  op-family     DESIGN_GUIDE.md Appendix A: the bold family arithmetic must
                actually sum to the true count, the table's per-family counts
                must match that arithmetic, every op named in a family must be
                a real op, no op may be claimed by two families, and the ops
                NOT enumerated must be exactly accounted for by the rows that
                delegate to a section pointer.
  op-doc        every real op must appear in API.md (the "op-by-op reference").
  path          every file path / crates module path / tests/<name>.rs named in
                the docs must exist.
  section       every §N / §N.M cross-reference must resolve to a DESIGN_GUIDE
                heading; every local markdown link target and #anchor must
                resolve.
  symbol        every kernel_*::path::to::symbol named in the docs must exist
                in that crate's source (grep-level definition scan).
  claim         every .rs file cited as PROOF ("pinned by", "repro", "gated
                by", anything under tests/) must exist AND contain a #[test]
                (or a fn main, for examples/).

Ground rules this tool follows (they are also its honest limits):

  * It never edits anything. Findings are for a human/integrator to fix.
  * Fenced code blocks are excluded from path/section/claim extraction (they
    contain example programs whose output paths need not exist). Symbols are
    additionally read from `rust`-tagged and untagged fences, because that is
    where the API contract snippets live.
  * Symbol checking is grep-level (`pub fn|pub struct|pub enum|pub trait|
    pub const|pub static|pub type|pub mod|pub use` plus bare `fn` for inherent
    methods). It proves a NAME is defined, not that the module path is exact.
    A name defined in a different crate than the doc claims is reported as a
    near-miss, not a hard failure.
  * Un-rooted path tokens (a bare filename, or a tail like `analysis/fea/`)
    resolve if they exist ANYWHERE in the tree. That leniency is deliberate:
    campaign docs legitimately name output files by tail. Fully-rooted tokens
    (`crates/...`) are checked exactly.
  * A multi-segment token is only treated as a repo path when it carries a
    known extension, ends in `/`, or starts at a real top-level entry — this
    keeps prose like `min/max/radius` or `bom/2` out of the report. A
    single-segment filename is only checked when its extension is a SOURCE
    extension (.rs/.py/.sh/.toml/.md/.lock); walkthrough artifacts the reader
    is told to create (`spacer.lmcpart`, `first.json`) are not repo claims.
    Anything under `target/` is a build artifact and is never checked.
  * Op-count claims below --min-op-claim (default 40), preceded by `~`, or on a
    line mentioning "chain" are treated as chain-length idioms, not op-surface
    claims ("a chain past ~10 ops"). Pass --verbose to see every skip, so the
    heuristic is itself auditable.

Usage:
  python3 tools/audit_docs.py                     # human report, exit 1 on error
  python3 tools/audit_docs.py --json              # machine report
  python3 tools/audit_docs.py --fail-on warn      # stricter gate
  python3 tools/audit_docs.py --all-docs          # add docs/*.md to the corpus
  python3 tools/audit_docs.py --self-test         # PROVE the checks can fail
"""

import argparse
import difflib
import json
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path

SEVERITY = {"info": 0, "warn": 1, "error": 2}
CLASSES = ["op-count", "op-family", "op-doc", "path", "section", "symbol", "claim"]

CORE_DOCS = ["AGENTS.md", "CLAUDE.md", "README.md", "API.md", "DESIGN_GUIDE.md"]
CRATE_NAMES = ["kernel-core", "kernel-brep", "kernel-implicit", "kernel-model",
	"kernel-api", "kernel-gpu", "kernel-wasm", "agent-bench"]
PATH_EXTS = {".rs", ".py", ".sh", ".md", ".json", ".jsonl", ".toml", ".ts", ".lock",
	".csv", ".png", ".stl", ".step", ".stp", ".3mf", ".lmcpart", ".lmcasm", ".txt",
	".yaml", ".yml", ".obj", ".ply", ".gltf", ".glb", ".svg", ".gif", ".html", ".rlib"}
# extensions for which a BARE filename (no directory part) is a repo claim
SOURCE_EXTS = {".rs", ".py", ".sh", ".toml", ".md", ".lock", ".ts"}
# dated ledgers: a stale-looking number there may be a legitimate historical
# record ("2026-06-10: 44-op JSON API"), so op-count findings are reported at
# info severity and flagged for human judgement rather than gated on.
HISTORICAL_DOCS = {"docs/BAR.md", "docs/CHANGELOG.md", "docs/FRICTION.md"}
SKIP_DIRS = {".git", "target", "__pycache__", "node_modules", ".venv"}


# --------------------------------------------------------------------------- #
# findings
# --------------------------------------------------------------------------- #

class Finding:
	def __init__(self, cls, severity, where, line, message, detail=None):
		self.cls = cls
		self.severity = severity
		self.where = where
		self.line = line
		self.message = message
		self.detail = detail

	def as_dict(self):
		d = {"class": self.cls, "severity": self.severity, "file": self.where,
			"line": self.line, "message": self.message}
		if self.detail:
			d["detail"] = self.detail
		return d

	def __repr__(self):
		return f"<{self.cls}/{self.severity} {self.where}:{self.line} {self.message}>"


# --------------------------------------------------------------------------- #
# markdown model
# --------------------------------------------------------------------------- #

FENCE_RE = re.compile(r"^\s*(?:```+|~~~+)\s*([A-Za-z0-9_+-]*)\s*$")


class Doc:
	"""A markdown file split into prose lines and fenced code blocks."""

	def __init__(self, root, rel):
		self.rel = rel
		self.path = root / rel
		self.text = self.path.read_text(encoding="utf-8", errors="replace")
		self.lines = self.text.split("\n")
		self._fence_lang = [None] * len(self.lines)	# per line: None = prose
		open_lang = None
		for i, ln in enumerate(self.lines):
			m = FENCE_RE.match(ln)
			if m:
				if open_lang is None:
					open_lang = m.group(1).lower() or "_none_"
					self._fence_lang[i] = open_lang
				else:
					self._fence_lang[i] = open_lang
					open_lang = None
				continue
			self._fence_lang[i] = open_lang

	def prose(self):
		return [(i + 1, ln) for i, ln in enumerate(self.lines) if self._fence_lang[i] is None]

	def code(self, langs):
		out = []
		for i, ln in enumerate(self.lines):
			lang = self._fence_lang[i]
			if lang is not None and lang in langs and not FENCE_RE.match(ln):
				out.append((i + 1, ln))
		return out

	def headings(self):
		out = []
		for i, ln in enumerate(self.lines):
			if self._fence_lang[i] is not None:
				continue
			m = re.match(r"^(#{1,6})\s+(.*?)\s*$", ln)
			if m:
				out.append((i + 1, len(m.group(1)), m.group(2)))
		return out


def slug(text):
	"""GitHub-flavoured heading anchor."""
	s = text.strip().lower()
	s = re.sub(r"`", "", s)
	s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)
	s = re.sub(r"[^\w\s-]", "", s, flags=re.UNICODE)
	s = re.sub(r"\s+", "-", s)
	return s


# --------------------------------------------------------------------------- #
# repository index
# --------------------------------------------------------------------------- #

DEF_RE = re.compile(
	r"^\s*(?:pub(?:\s*\([^)]*\))?\s+)?"
	r"(?:async\s+|unsafe\s+|extern\s+\"[^\"]*\"\s+|const\s+)*"
	r"(fn|struct|enum|trait|type|mod|const|static|union)\s+([A-Za-z_][A-Za-z0-9_]*)")
USE_RE = re.compile(r"^\s*pub\s+use\s+(.+?);")
MACRO_RE = re.compile(r"^\s*macro_rules!\s+([A-Za-z_][A-Za-z0-9_]*)")


class Repo:
	def __init__(self, root):
		self.root = Path(root).resolve()
		self._paths = None
		self._tails = None
		self._symbols = None

	# ---- filesystem index ------------------------------------------------ #

	def _index_paths(self):
		if self._paths is not None:
			return
		paths = set()
		tails = {}
		seen_real = set()
		for dirpath, dirnames, filenames in os.walk(self.root, followlinks=True):
			real = os.path.realpath(dirpath)
			if real in seen_real:
				dirnames[:] = []
				continue
			seen_real.add(real)
			dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
			rel_dir = Path(dirpath).relative_to(self.root)
			entries = list(dirnames) + list(filenames)
			for name in entries:
				rel = (rel_dir / name).as_posix() if rel_dir.as_posix() != "." else name
				paths.add(rel)
				parts = rel.split("/")
				for k in range(1, min(len(parts), 4) + 1):
					tails.setdefault("/".join(parts[-k:]), []).append(rel)
		self._paths = paths
		self._tails = tails

	def exists(self, rel):
		self._index_paths()
		return rel in self._paths

	def tail_matches(self, tail):
		self._index_paths()
		return self._tails.get(tail, [])

	def all_paths(self):
		self._index_paths()
		return self._paths

	# ---- rust symbol index ----------------------------------------------- #

	def _index_symbols(self):
		if self._symbols is not None:
			return
		syms = {}
		crates_dir = self.root / "crates"
		roots = []
		if crates_dir.is_dir():
			roots = [p for p in sorted(crates_dir.iterdir()) if p.is_dir()]
		for crate in roots:
			for rs in crate.rglob("*.rs"):
				if any(part in SKIP_DIRS for part in rs.parts):
					continue
				try:
					lines = rs.read_text(encoding="utf-8", errors="replace").split("\n")
				except OSError:
					continue
				rel = rs.relative_to(self.root).as_posix()
				enum_depth = None		# brace depth of the enum body we are inside
				depth = 0
				for n, ln in enumerate(lines, 1):
					code = ln.split("//")[0]
					opens, closes = code.count("{"), code.count("}")
					m = DEF_RE.match(ln)
					if enum_depth is not None and depth == enum_depth:
						# enum variants: `Name,` / `Name {` / `Name(` / `Name = 1`
						v = re.match(r"\s*([A-Z][A-Za-z0-9_]*)\s*(?:[,({=]|$)", ln)
						if v:
							syms.setdefault(v.group(1), []).append(
								(crate.name, rel, n, "variant"))
					depth += opens - closes
					if enum_depth is not None and depth < enum_depth:
						enum_depth = None
					if m:
						syms.setdefault(m.group(2), []).append((crate.name, rel, n, m.group(1)))
						if m.group(1) == "enum" and "{" in code:
							enum_depth = depth
						continue
					m = MACRO_RE.match(ln)
					if m:
						syms.setdefault(m.group(1), []).append((crate.name, rel, n, "macro"))
						continue
					m = USE_RE.match(ln)
					if m:
						for name in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", m.group(1)):
							syms.setdefault(name, []).append((crate.name, rel, n, "use"))
		self._symbols = syms

	def symbol(self, name):
		self._index_symbols()
		return self._symbols.get(name, [])

	def all_symbols(self):
		self._index_symbols()
		return self._symbols


# --------------------------------------------------------------------------- #
# ground truth: the op surface
# --------------------------------------------------------------------------- #

DISCOVER_REL = "crates/kernel-api/src/discover.rs"


class OpTruth:
	def __init__(self, arms, names, count, findings):
		self.arms = arms
		self.names = names
		self.count = count
		self.findings = findings

	@property
	def n(self):
		return len(self.arms)


def op_truth(repo):
	findings = []
	src = repo.root / DISCOVER_REL
	if not src.is_file():
		findings.append(Finding("op-count", "error", DISCOVER_REL, 0,
			"ground-truth source missing: cannot derive the op count"))
		return OpTruth([], [], None, findings)
	text = src.read_text(encoding="utf-8", errors="replace")
	lines = text.split("\n")

	start = None
	for i, ln in enumerate(lines):
		if re.match(r"\s*pub fn op_tag\b", ln):
			start = i
			break
	arms = []
	if start is None:
		findings.append(Finding("op-count", "error", DISCOVER_REL, 0,
			"`pub fn op_tag` not found — the mechanical op count has no source"))
	else:
		for ln in lines[start:]:
			if re.match(r"^\}", ln):
				break
			m = re.search(r'=>\s*"([a-z0-9_]+)"', ln)
			if m:
				arms.append(m.group(1))

	names = []
	m = re.search(r"pub const OP_NAMES:\s*&\[&str\]\s*=\s*&\[(.*?)\n\];", text, re.S)
	if m:
		names = re.findall(r'"([a-z0-9_]+)"', m.group(1))
	else:
		findings.append(Finding("op-count", "warn", DISCOVER_REL, 0,
			"`OP_NAMES` table not found — cross-check of the catalogue skipped"))

	count = None
	m = re.search(r"pub const OP_COUNT:\s*usize\s*=\s*(\d+)\s*;", text)
	if m:
		count = int(m.group(1))
		lineno = text[:m.start()].count("\n") + 1
		if arms and count != len(arms):
			findings.append(Finding("op-count", "error", DISCOVER_REL, lineno,
				f"OP_COUNT = {count} but `op_tag` has {len(arms)} arms"))
	else:
		findings.append(Finding("op-count", "warn", DISCOVER_REL, 0,
			"`OP_COUNT` const not found"))

	if arms and names and arms != names:
		only_arms = [a for a in arms if a not in set(names)]
		only_names = [n for n in names if n not in set(arms)]
		findings.append(Finding("op-count", "error", DISCOVER_REL, 0,
			f"`op_tag` and OP_NAMES disagree ({len(arms)} vs {len(names)})",
			f"only in op_tag: {only_arms}; only in OP_NAMES: {only_names}"))

	return OpTruth(arms, names, count, findings)


# --------------------------------------------------------------------------- #
# check 1 — op-count claims
# --------------------------------------------------------------------------- #

OPCLAIM_RE = re.compile(r"(\d{1,4})\s*(?:-\s*|\s+)(ops?|operations?)\b", re.I)


def check_op_counts(repo, docs, truth, min_claim, verbose_skips, stats):
	out = []
	if not truth.arms:
		return out
	stats["op-count"] = 0
	true_n = truth.n
	for doc in docs:
		for lineno, text in doc.prose():
			for m in OPCLAIM_RE.finditer(text):
				n = int(m.group(1))
				before = text[max(0, m.start() - 60):m.start()]
				after = text[m.end():m.end() + 60]
				reason = None
				if before.endswith("~") or before.endswith("~ "):
					reason = "approximate (`~`) — chain-length idiom"
				elif "chain" in (before + after).lower():
					reason = "line mentions a boolean chain, not the op surface"
				elif n < min_claim:
					reason = f"below --min-op-claim ({min_claim})"
				if reason:
					if verbose_skips:
						out.append(Finding("op-count", "info", doc.rel, lineno,
							f"skipped candidate '{m.group(0)}': {reason}",
							text.strip()))
					continue
				stats["op-count"] += 1
				if n != true_n:
					hist = doc.rel in HISTORICAL_DOCS or doc.rel.startswith("docs/history/")
					note = (" [dated ledger — may be a legitimate historical record; "
						"human judgement required]" if hist else "")
					out.append(Finding("op-count", "info" if hist else "error",
						doc.rel, lineno,
						f"op-count claim '{m.group(0).strip()}' contradicts source "
						f"({true_n} `op_tag` arms in {DISCOVER_REL}){note}",
						text.strip()))
	return out


# --------------------------------------------------------------------------- #
# check 2 — Appendix A family arithmetic
# --------------------------------------------------------------------------- #

def check_op_families(repo, doc, truth):
	"""DESIGN_GUIDE.md Appendix A: arithmetic, family membership, coverage."""
	out = []
	if not truth.arms:
		return out
	real = set(truth.arms)
	start = None
	for lineno, level, title in doc.headings():
		if re.match(r"^Appendix A\b", title):
			start = lineno
			break
	if start is None:
		out.append(Finding("op-family", "warn", doc.rel, 0,
			"no `Appendix A` heading found — family arithmetic not auditable"))
		return out

	region = [(n, t) for n, t in doc.prose() if n >= start]

	# --- the bold arithmetic (may wrap across lines) ----------------------- #
	arith = None
	joined = "\n".join(t for _, t in region)
	offsets = []
	pos = 0
	for lineno, t in region:
		offsets.append((pos, lineno))
		pos += len(t) + 1
	m = re.search(r"\*\*\s*((?:\d+\s*\+\s*)+\d+)\s*=\s*(\d+)\s*\.?\s*\*\*", joined)
	if m:
		lineno = next((ln for off, ln in reversed(offsets) if off <= m.start()), start)
		summands = [int(x) for x in re.findall(r"\d+", m.group(1))]
		stated = int(m.group(2))
		arith = (lineno, summands, stated)
		snippet = " ".join(m.group(0).split())
		if sum(summands) != stated:
			out.append(Finding("op-family", "error", doc.rel, lineno,
				f"Appendix A arithmetic does not add up: the summands total "
				f"{sum(summands)} but the line claims {stated}", snippet))
		if stated != truth.n:
			out.append(Finding("op-family", "error", doc.rel, lineno,
				f"Appendix A total {stated} contradicts source ({truth.n} "
				f"`op_tag` arms)", snippet))
	if arith is None:
		out.append(Finding("op-family", "warn", doc.rel, start,
			"Appendix A has no `**a + b + ... = N.**` arithmetic line to verify"))

	# --- the family table -------------------------------------------------- #
	rows = []			# (lineno, family, declared_count, ops or None)
	for lineno, text in region:
		if not text.lstrip().startswith("|"):
			continue
		cells = [c.strip() for c in text.strip().strip("|").split("|")]
		if len(cells) < 2:
			continue
		m = re.match(r"^(.*?)\s*\((\d+)\)\s*$", cells[0])
		if not m:
			continue
		family, declared = m.group(1), int(m.group(2))
		ops_cell = cells[1]
		if "§" in ops_cell:
			rows.append((lineno, family, declared, None))
			out.append(Finding("op-family", "info", doc.rel, lineno,
				f"family '{family}' ({declared}) delegates its op list to a section "
				f"pointer ({ops_cell}); membership is verified only by residual count"))
			continue
		ops = [o.strip(" `") for o in ops_cell.split(",") if o.strip(" `")]
		rows.append((lineno, family, declared, ops))

	if not rows:
		out.append(Finding("op-family", "warn", doc.rel, start,
			"Appendix A family table not found (no `| family (N) | ops |` rows)"))
		return out

	declared_total = sum(r[2] for r in rows)
	if declared_total != truth.n:
		out.append(Finding("op-family", "error", doc.rel, rows[0][0],
			f"Appendix A family counts sum to {declared_total}, source has {truth.n} ops"))
	if arith is not None and sorted(arith[1]) != sorted(r[2] for r in rows):
		out.append(Finding("op-family", "error", doc.rel, arith[0],
			"the Appendix A arithmetic summands are not the table's family counts",
			f"arithmetic {sorted(arith[1])} vs table {sorted(r[2] for r in rows)}"))
	elif arith is not None and arith[1] != [r[2] for r in rows]:
		out.append(Finding("op-family", "info", doc.rel, arith[0],
			"the Appendix A arithmetic lists the same family counts in a different "
			"order than the table (sums agree)",
			f"arithmetic {arith[1]} vs table {[r[2] for r in rows]}"))

	seen = {}
	for lineno, family, declared, ops in rows:
		if ops is None:
			continue
		if len(ops) != declared:
			out.append(Finding("op-family", "error", doc.rel, lineno,
				f"family '{family}' declares ({declared}) but lists {len(ops)} ops"))
		for op in ops:
			if op not in real:
				near = difflib.get_close_matches(op, sorted(real), n=1, cutoff=0.75)
				hint = f" (did you mean `{near[0]}`?)" if near else ""
				out.append(Finding("op-family", "error", doc.rel, lineno,
					f"family '{family}' names `{op}`, which is not a real op{hint}"))
			if op in seen:
				out.append(Finding("op-family", "error", doc.rel, lineno,
					f"op `{op}` is claimed by two families: '{seen[op]}' and '{family}'"))
			else:
				seen[op] = family

	listed = set(seen)
	pointer_total = sum(r[2] for r in rows if r[3] is None)
	uncovered = [o for o in truth.arms if o not in listed]
	if len(uncovered) != pointer_total:
		sample = uncovered[:12]
		out.append(Finding("op-family", "error", doc.rel, rows[0][0],
			f"{len(uncovered)} ops are not enumerated by any family, but the "
			f"pointer-delegated families account for only {pointer_total}",
			f"unenumerated (first {len(sample)}): {sample}"))
	return out


# --------------------------------------------------------------------------- #
# check 3 — API.md op coverage
# --------------------------------------------------------------------------- #

def check_op_docs(repo, docs, truth):
	out = []
	if not truth.arms:
		return out
	api = next((d for d in docs if d.rel == "API.md"), None)
	if api is None:
		return out
	heads = set()
	for _, _, title in api.headings():
		m = re.match(r"^`([a-z0-9_]+)`", title)
		if m:
			heads.add(m.group(1))
	body = api.text
	for op in truth.arms:
		if op in heads:
			continue
		if re.search(r"\b" + re.escape(op) + r"\b", body):
			out.append(Finding("op-doc", "info", "API.md", 0,
				f"op `{op}` has no `### \\`{op}\\`` reference heading in API.md "
				f"(mentioned in prose/tables only)"))
		else:
			out.append(Finding("op-doc", "warn", "API.md", 0,
				f"op `{op}` is absent from API.md entirely, which claims to be the "
				f"op-by-op reference for all {truth.n} ops"))
	return out


# --------------------------------------------------------------------------- #
# check 4 — paths
# --------------------------------------------------------------------------- #

INLINE_CODE_RE = re.compile(r"`([^`\n]+)`")
MD_LINK_RE = re.compile(r"\[[^\]]*\]\(\s*([^)\s]+?)\s*\)")
HTML_ATTR_RE = re.compile(r'(?:href|src)="([^"]+)"')
PATH_CHARS_RE = re.compile(r"^[A-Za-z0-9._/{},+-]+$")


def expand_braces(tok):
	m = re.search(r"\{([^{}]*)\}", tok)
	if not m:
		return [tok]
	out = []
	for alt in m.group(1).split(","):
		out.extend(expand_braces(tok[:m.start()] + alt.strip() + tok[m.end():]))
	return out


def looks_like_path(tok, repo=None, as_link=False):
	"""Conservative: only call something a repo-path claim when it really is one.

	`as_link=True` relaxes the heuristics for markdown/HTML link targets — a
	link is an unambiguous navigational claim, so every non-URL target counts.
	"""
	if not tok or not PATH_CHARS_RE.match(tok):
		return False
	if tok.startswith(("http://", "https://", "mailto:", "#")):
		return False
	if "*" in tok or "<" in tok or ">" in tok:
		return False
	body = tok.rstrip("/")
	if not body or body.endswith("."):
		return False
	top = body.split("/")[0]
	if top == "target":
		return False			# build artifact, never in the tree at audit time
	if as_link:
		return True
	last = body.rsplit("/", 1)[-1]
	ext = "." + last.rsplit(".", 1)[1] if "." in last and not last.startswith(".") else ""
	if "/" not in body:
		# bare filename: only source-ish names are repo claims (see module docstring)
		return ext in SOURCE_EXTS
	if ext not in PATH_EXTS and not tok.endswith("/"):
		return False			# `min/max/radius`, `bom/2`, `1/z` — prose, not paths
	if top == ".." or top in CRATE_NAMES:
		return True				# doc-relative escape, or a crate-rooted module path
	# only ROOTED repo paths are claims; `out/plate.stl`, `parts/NN_name.stl` and
	# other walkthrough-relative artifacts are not.
	return repo is not None and repo.exists(top)


def resolve_path(repo, tok, doc_rel):
	"""Return (True, note) if the token names something that exists."""
	body = tok.rstrip("/")
	cands = [body]
	doc_dir = str(Path(doc_rel).parent)
	if doc_dir not in (".", ""):
		cands.append((Path(doc_dir) / body).as_posix())
	if body.split("/")[0] in CRATE_NAMES:
		cands.append("crates/" + body)
	if body.split("/")[0] in ("tests", "src", "examples", "benches"):
		for c in CRATE_NAMES:
			cands.append(f"crates/{c}/{body}")
	for c in cands:
		c = os.path.normpath(c).replace("\\", "/")
		if c.startswith("../"):
			if (repo.root / c).exists():
				return True, "resolved above the repo root"
			continue
		if repo.exists(c):
			return True, None
	hits = repo.tail_matches(body)
	if hits:
		return True, f"resolved by tail match → {hits[0]}"
	return False, None


def _path_tokens(doc):
	"""Yield (lineno, token, kind) for every path-ish reference in prose."""
	for lineno, text in doc.prose():
		for m in MD_LINK_RE.finditer(text):
			yield lineno, m.group(1), "link"
		for m in HTML_ATTR_RE.finditer(text):
			yield lineno, m.group(1), "link"
		# strip link targets so inline-code scan does not double-report
		stripped = MD_LINK_RE.sub(lambda mm: "[]()", text)
		for m in INLINE_CODE_RE.finditer(stripped):
			yield lineno, m.group(1), "code"


def check_paths(repo, docs, stats):
	out = []
	stats["path"] = 0
	for doc in docs:
		for lineno, raw, kind in _path_tokens(doc):
			if raw.startswith("#"):
				continue
			target = raw.split("#", 1)[0]
			if not target:
				continue
			for tok in expand_braces(target):
				if not looks_like_path(tok, repo, as_link=(kind == "link")):
					continue
				stats["path"] += 1
				ok, note = resolve_path(repo, tok, doc.rel)
				if ok:
					continue
				near = difflib.get_close_matches(tok.rstrip("/"),
					[p for p in repo.all_paths() if p.count("/") <= tok.count("/") + 1],
					n=1, cutoff=0.8)
				hint = f"closest existing: {near[0]}" if near else None
				sev = "error" if kind == "link" else "warn"
				out.append(Finding("path", sev, doc.rel, lineno,
					f"referenced path does not exist: `{tok}`"
					+ (" (markdown link target)" if kind == "link" else ""), hint))
	return out


# --------------------------------------------------------------------------- #
# check 5 — section pointers and anchors
# --------------------------------------------------------------------------- #

SECTION_RE = re.compile(r"§\s*(\d+(?:\.\d+)*)(?:\s*[–—-]\s*(\d+(?:\.\d+)*))?")


def design_guide_sections(doc):
	secs = set()
	for _, _, title in doc.headings():
		m = re.match(r"^(\d+(?:\.\d+)*)[.)]?\s+", title)
		if m:
			num = m.group(1).rstrip(".")
			secs.add(num)
			if "." in num:
				secs.add(num.split(".")[0])
	return secs


def check_sections(repo, docs, guide_doc, stats):
	out = []
	stats["section"] = 0
	if guide_doc is None:
		return out
	secs = design_guide_sections(guide_doc)
	if not secs:
		out.append(Finding("section", "warn", guide_doc.rel, 0,
			"no numbered headings found — § pointers cannot be resolved"))
		return out
	for doc in docs:
		for lineno, text in doc.prose():
			for m in SECTION_RE.finditer(text):
				refs = [m.group(1)]
				if m.group(2):
					hi = m.group(2)
					refs.append(hi)
					if "." not in m.group(1) and "." not in hi:
						lo_i, hi_i = int(m.group(1)), int(hi)
						if 0 < hi_i - lo_i < 40:
							refs = [str(x) for x in range(lo_i, hi_i + 1)]
				for ref in refs:
					stats["section"] += 1
					if ref not in secs:
						near = difflib.get_close_matches(ref, sorted(secs), n=1, cutoff=0.6)
						hint = f"nearest existing section: §{near[0]}" if near else None
						out.append(Finding("section", "error", doc.rel, lineno,
							f"§{ref} does not resolve to a heading in {guide_doc.rel}",
							hint))
	# local markdown anchors
	anchors = {d.rel: {slug(t) for _, _, t in d.headings()} for d in docs}
	for doc in docs:
		for lineno, text in doc.prose():
			for m in MD_LINK_RE.finditer(text):
				tgt = m.group(1)
				if tgt.startswith(("http://", "https://", "mailto:")):
					continue
				if "#" not in tgt:
					continue
				fpart, anchor = tgt.split("#", 1)
				if not anchor:
					continue
				owner = doc.rel if not fpart else fpart
				if owner not in anchors:
					continue	# missing-file case is a path finding already
				if anchor.lower() not in anchors[owner]:
					out.append(Finding("section", "warn", doc.rel, lineno,
						f"anchor `#{anchor}` does not match any heading in {owner}"))
	return out


# --------------------------------------------------------------------------- #
# check 6 — symbols
# --------------------------------------------------------------------------- #

SYMPATH_RE = re.compile(
	r"\b(kernel_(?:core|brep|implicit|model|api|gpu|wasm))"
	r"((?:::[A-Za-z_][A-Za-z0-9_]*)+)"
	r"(::\{[^}]*\})?")


def crate_dir_for(krate_snake):
	return krate_snake.replace("_", "-")


def check_symbols(repo, docs, stats):
	out = []
	stats["symbol"] = 0
	for doc in docs:
		lines = doc.prose() + doc.code({"rust", "_none_", "rs"})
		for lineno, text in sorted(set(lines)):
			for m in SYMPATH_RE.finditer(text):
				krate = m.group(1)
				segs = [s for s in m.group(2).split("::") if s]
				tails = []
				if m.group(3):
					inner = m.group(3)[3:-1]
					for part in inner.split(","):
						part = part.strip()
						if part:
							tails.append(segs + [part])
				else:
					tails.append(segs)
				for path_segs in tails:
					stats["symbol"] += 1
					out.extend(_check_one_symbol(repo, doc, lineno, krate, path_segs, text))
	return out


def _check_one_symbol(repo, doc, lineno, krate, segs, text):
	out = []
	full = krate + "::" + "::".join(segs)
	want_crate = crate_dir_for(krate)
	name = segs[-1]
	if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", name):
		return out
	hits = repo.symbol(name)
	if not hits:
		near = difflib.get_close_matches(name, list(repo.all_symbols()), n=1, cutoff=0.85)
		hint = f"closest defined name: {near[0]}" if near else None
		out.append(Finding("symbol", "warn", doc.rel, lineno,
			f"`{full}` — no definition of `{name}` found in any crate "
			f"(grep-level scan)", hint))
		return out
	crates_with = {h[0] for h in hits}
	if want_crate not in crates_with:
		out.append(Finding("symbol", "info", doc.rel, lineno,
			f"`{full}` — `{name}` is defined in {sorted(crates_with)}, not in "
			f"{want_crate} (near-miss: re-export or wrong crate in the doc)",
			f"e.g. {hits[0][1]}:{hits[0][2]}"))
		return out
	# intermediate module segments
	for seg in segs[:-1]:
		if not re.match(r"^[a-z_][a-z0-9_]*$", seg):
			continue
		mod_hits = [h for h in repo.symbol(seg) if h[0] == want_crate and h[3] in ("mod", "use")]
		file_hit = (repo.exists(f"crates/{want_crate}/src/{seg}.rs")
			or repo.exists(f"crates/{want_crate}/src/{seg}"))
		if not mod_hits and not file_hit:
			out.append(Finding("symbol", "info", doc.rel, lineno,
				f"`{full}` — module segment `{seg}` not found in {want_crate} "
				f"(the leaf name exists; the path may be a re-export)"))
	return out


# --------------------------------------------------------------------------- #
# check 7 — claim freshness
# --------------------------------------------------------------------------- #

PROOF_WORDS = re.compile(
	r"\b(pinn?ed|pins?|repro(?:duc\w*)?|proof|proven|proves|gated by|regression|"
	r"acceptance|executed reference|test)\b", re.I)


def check_claims(repo, docs, stats):
	out = []
	stats["claim"] = 0
	for doc in docs:
		for lineno, text in doc.prose():
			stripped = MD_LINK_RE.sub(lambda mm: "[]()", text)
			raw_toks = [m.group(1) for m in INLINE_CODE_RE.finditer(stripped)]
			raw_toks += [m.group(1) for m in MD_LINK_RE.finditer(text)]
			toks = []
			for raw in raw_toks:
				toks.extend(expand_braces(raw.split("#", 1)[0].strip()))
			for tok in toks:
				if not tok.endswith(".rs") or not looks_like_path(tok, repo):
					continue
				is_test_path = "/tests/" in tok or tok.startswith("tests/")
				if not is_test_path and not PROOF_WORDS.search(text):
					continue
				stats["claim"] += 1
				ok, _ = resolve_path(repo, tok, doc.rel)
				if not ok:
					out.append(Finding("claim", "error", doc.rel, lineno,
						f"cited as proof but the file does not exist: `{tok}`",
						text.strip()))
					continue
				real = _first_existing(repo, tok, doc.rel)
				if real is None:
					continue
				body = (repo.root / real).read_text(encoding="utf-8", errors="replace")
				if "#[test]" in body:
					continue
				if "/examples/" in real and re.search(r"\bfn\s+main\s*\(", body):
					continue
				out.append(Finding("claim", "error", doc.rel, lineno,
					f"`{tok}` is cited as proof but contains no `#[test]` "
					f"(resolved to {real})", text.strip()))
	return out


def _first_existing(repo, tok, doc_rel):
	body = tok.rstrip("/")
	cands = [body]
	doc_dir = str(Path(doc_rel).parent)
	if doc_dir not in (".", ""):
		cands.append((Path(doc_dir) / body).as_posix())
	if body.split("/")[0] in CRATE_NAMES:
		cands.append("crates/" + body)
	if body.split("/")[0] in ("tests", "src", "examples", "benches"):
		for c in CRATE_NAMES:
			cands.append(f"crates/{c}/{body}")
	for c in cands:
		c = os.path.normpath(c).replace("\\", "/")
		if repo.exists(c) and (repo.root / c).is_file():
			return c
	for hit in repo.tail_matches(body):
		if (repo.root / hit).is_file():
			return hit
	return None


# --------------------------------------------------------------------------- #
# driver
# --------------------------------------------------------------------------- #

def corpus(repo, all_docs=False):
	rels = []
	for rel in CORE_DOCS:
		if (repo.root / rel).is_file():
			rels.append(rel)
	skills = sorted((repo.root / ".claude" / "skills").glob("*/SKILL.md")) \
		if (repo.root / ".claude" / "skills").is_dir() else []
	rels += [p.relative_to(repo.root).as_posix() for p in skills]
	cmds = sorted((repo.root / ".opencode" / "command").glob("*.md")) \
		if (repo.root / ".opencode" / "command").is_dir() else []
	rels += [p.relative_to(repo.root).as_posix() for p in cmds]
	if all_docs:
		docsdir = repo.root / "docs"
		if docsdir.is_dir():
			rels += [p.relative_to(repo.root).as_posix() for p in sorted(docsdir.glob("*.md"))]
	return [Doc(repo.root, r) for r in rels]


def audit(root, all_docs=False, min_claim=40, verbose_skips=False, only=None):
	repo = Repo(root)
	docs = corpus(repo, all_docs)
	truth = op_truth(repo)
	guide = next((d for d in docs if d.rel == "DESIGN_GUIDE.md"), None)

	stats = {}
	findings = list(truth.findings)
	findings += check_op_counts(repo, docs, truth, min_claim, verbose_skips, stats)
	if guide is not None:
		findings += check_op_families(repo, guide, truth)
	findings += check_op_docs(repo, docs, truth)
	stats["op-doc"] = truth.n
	stats["op-family"] = truth.n
	findings += check_paths(repo, docs, stats)
	findings += check_sections(repo, docs, guide, stats)
	findings += check_symbols(repo, docs, stats)
	findings += check_claims(repo, docs, stats)

	if only:
		findings = [f for f in findings if f.cls in only]
	findings.sort(key=lambda f: (-SEVERITY[f.severity], f.cls, f.where, f.line))
	return repo, truth, docs, findings, stats


def render(truth, docs, findings, fail_on, stats):
	lines = []
	lines.append("LMCAD doc-drift audit")
	lines.append(f"  ground truth : {truth.n} ops (`op_tag` arms in {DISCOVER_REL})")
	lines.append(f"  corpus       : {len(docs)} files — " + ", ".join(d.rel for d in docs))
	lines.append("")
	by_class = {c: [] for c in CLASSES}
	for f in findings:
		by_class.setdefault(f.cls, []).append(f)
	for cls in CLASSES:
		fs = by_class.get(cls, [])
		tally = {s: sum(1 for f in fs if f.severity == s) for s in ("error", "warn", "info")}
		checked = stats.get(cls)
		scope = f" over {checked} reference(s) checked" if checked is not None else ""
		lines.append(f"[{cls}] {len(fs)} finding(s){scope} "
			f"(error {tally['error']} / warn {tally['warn']} / info {tally['info']})")
		for f in fs:
			loc = f"{f.where}:{f.line}" if f.line else f.where
			lines.append(f"  {f.severity.upper():5s} {loc}  {f.message}")
			if f.detail:
				lines.append(f"        · {f.detail}")
		lines.append("")
	gate = [f for f in findings if SEVERITY[f.severity] >= SEVERITY[fail_on]]
	lines.append(f"TOTAL {len(findings)} finding(s); {len(gate)} at or above "
		f"severity '{fail_on}' → exit {1 if gate else 0}")
	return "\n".join(lines), len(gate)


# --------------------------------------------------------------------------- #
# self-test — prove every check can fail
# --------------------------------------------------------------------------- #

FIXTURE_DISCOVER = '''//! fixture
use crate::program::OpKind;

pub fn op_tag(op: &OpKind) -> &'static str {
	match op {
		OpKind::Box { .. } => "box",
		OpKind::Cylinder { .. } => "cylinder",
		OpKind::Union { .. } => "union",
		OpKind::Difference { .. } => "difference",
		OpKind::Volume { .. } => "volume",
		OpKind::ExportStl { .. } => "export_stl",
	}
}

pub const OP_NAMES: &[&str] = &[
	"box",
	"cylinder",
	"union",
	"difference",
	"volume",
	"export_stl",
];

pub const OP_COUNT: usize = 6;
'''

FIXTURE_GUIDE = '''# Fixture Design Guide

Companions: [`API.md`](API.md) is the op-by-op reference (6 ops).
See §1 for the model and §2.1 for booleans.

## 1. Mental model

Nothing here.

## 2. Booleans

Text.

### 2.1 The trio

`kernel_brep::booleans::union` is the entry point. Pinned by
`crates/kernel-brep/tests/fixture_pin.rs`.

# Appendix A — Coverage map

**2 + 2 + 2 = 6.**

| family (count) | ops | guide |
|---|---|---|
| constructors (2) | box, cylinder | §1 |
| booleans (2) | union, difference | §2.1 |
| rest (2) | §2 full table | §2 |
'''

FIXTURE_API = '''# Fixture API

6 ops total.

### `box`
### `cylinder`
### `union`
### `difference`
### `volume`
### `export_stl`
'''

FIXTURE_AGENTS = '''# Fixture agents

- `crates/kernel-brep` — the 6-op surface
- run `tools/fixture.sh`
'''

FIXTURE_SKILL = '''---
name: fixture-skill
description: A fixture skill.
---

# Fixture

Follow DESIGN_GUIDE.md §2.1. Uses `kernel_brep::booleans::union`.
'''

FIXTURE_LIB = '''pub mod booleans;
'''

FIXTURE_BOOLEANS = '''pub fn union(a: u32, b: u32) -> u32 { a + b }
pub fn difference(a: u32, b: u32) -> u32 { a - b }
'''

FIXTURE_TEST = '''#[test]
fn fixture_pin() { assert_eq!(1, 1); }
'''


def build_fixture(dest):
	dest = Path(dest)
	(dest / "crates/kernel-api/src").mkdir(parents=True, exist_ok=True)
	(dest / "crates/kernel-brep/src").mkdir(parents=True, exist_ok=True)
	(dest / "crates/kernel-brep/tests").mkdir(parents=True, exist_ok=True)
	(dest / ".claude/skills/fixture-skill").mkdir(parents=True, exist_ok=True)
	(dest / "tools").mkdir(parents=True, exist_ok=True)
	(dest / "crates/kernel-api/src/discover.rs").write_text(FIXTURE_DISCOVER)
	(dest / "crates/kernel-brep/src/lib.rs").write_text(FIXTURE_LIB)
	(dest / "crates/kernel-brep/src/booleans.rs").write_text(FIXTURE_BOOLEANS)
	(dest / "crates/kernel-brep/tests/fixture_pin.rs").write_text(FIXTURE_TEST)
	(dest / "DESIGN_GUIDE.md").write_text(FIXTURE_GUIDE)
	(dest / "API.md").write_text(FIXTURE_API)
	(dest / "AGENTS.md").write_text(FIXTURE_AGENTS)
	(dest / "CLAUDE.md").write_text("# shim\n\n@AGENTS.md\n")
	(dest / "README.md").write_text("# Fixture\n\nThe surface has 6 ops.\n")
	(dest / ".claude/skills/fixture-skill/SKILL.md").write_text(FIXTURE_SKILL)
	(dest / "tools/fixture.sh").write_text("#!/bin/sh\ntrue\n")
	return dest


def _inject(path, old, new):
	p = Path(path)
	t = p.read_text()
	assert old in t, f"injection anchor not found in {path}: {old!r}"
	p.write_text(t.replace(old, new, 1))


# Each injection: (label, class it must trigger, mutate(root), token that must
# appear in the finding text)
def injections():
	def op_count(root):
		_inject(Path(root) / "README.md", "6 ops", "142 ops")
		return "142"

	def arithmetic(root):
		_inject(Path(root) / "DESIGN_GUIDE.md", "**2 + 2 + 2 = 6.**", "**2 + 3 + 2 = 6.**")
		return "add up"

	def bad_family_op(root):
		_inject(Path(root) / "DESIGN_GUIDE.md", "| booleans (2) | union, difference |",
			"| booleans (2) | union, differance |")
		return "differance"

	def uncovered_op(root):
		d = Path(root) / "crates/kernel-api/src/discover.rs"
		_inject(d, '\t\tOpKind::ExportStl { .. } => "export_stl",',
			'\t\tOpKind::ExportStl { .. } => "export_stl",\n\t\tOpKind::Brand { .. } => "brand_new_op",')
		_inject(d, '\t"export_stl",\n];', '\t"export_stl",\n\t"brand_new_op",\n];')
		_inject(d, "pub const OP_COUNT: usize = 6;", "pub const OP_COUNT: usize = 7;")
		return "not enumerated"

	def dead_path(root):
		_inject(Path(root) / "AGENTS.md", "- run `tools/fixture.sh`",
			"- run `tools/fixture.sh`\n- see `crates/kernel-nope/src/ghost.rs`")
		return "kernel-nope"

	def dead_link(root):
		_inject(Path(root) / "README.md", "The surface has 6 ops.",
			"The surface has 6 ops. See [the ledger](docs/GONE.md).")
		return "docs/GONE.md"

	def dead_section(root):
		_inject(Path(root) / ".claude/skills/fixture-skill/SKILL.md",
			"Follow DESIGN_GUIDE.md §2.1.", "Follow DESIGN_GUIDE.md §2.1 and §9.9.")
		return "§9.9"

	def dead_symbol(root):
		_inject(Path(root) / ".claude/skills/fixture-skill/SKILL.md",
			"Uses `kernel_brep::booleans::union`.",
			"Uses `kernel_brep::booleans::union` and `kernel_brep::booleans::unyon`.")
		return "unyon"

	def wrong_crate_symbol(root):
		_inject(Path(root) / ".claude/skills/fixture-skill/SKILL.md",
			"Uses `kernel_brep::booleans::union`.",
			"Uses `kernel_model::booleans::union`.")
		return "near-miss"

	def vacuous_pin(root):
		(Path(root) / "crates/kernel-brep/tests/fixture_pin.rs").write_text(
			"// TODO: write the test\n")
		return "no `#[test]`"

	def missing_pin(root):
		_inject(Path(root) / "DESIGN_GUIDE.md",
			"`crates/kernel-brep/tests/fixture_pin.rs`",
			"`crates/kernel-brep/tests/never_written.rs`")
		return "does not exist"

	def undocumented_op(root):
		_inject(Path(root) / "API.md", "### `volume`\n", "")
		return "volume"

	return [
		("op-count claim drifts", "op-count", op_count, "142"),
		("Appendix A arithmetic stops summing", "op-family", arithmetic, "add up"),
		("family names a non-existent op", "op-family", bad_family_op, "differance"),
		("new op added, Appendix A not updated", "op-family", uncovered_op, "not enumerated"),
		("doc names a dead crate path", "path", dead_path, "kernel-nope"),
		("markdown link target missing", "path", dead_link, "docs/GONE.md"),
		("§ pointer to a non-existent section", "section", dead_section, "§9.9"),
		("doc names a non-existent symbol", "symbol", dead_symbol, "unyon"),
		("doc names a symbol in the wrong crate", "symbol", wrong_crate_symbol, "near-miss"),
		("cited pin has no #[test]", "claim", vacuous_pin, "#[test]"),
		("cited pin file does not exist", "claim", missing_pin, "does not exist"),
		("op vanishes from API.md", "op-doc", undocumented_op, "volume"),
	]


def _copy_real(root, dest):
	"""Copy the parts of the real tree the auditor reads; symlink the rest."""
	root = Path(root)
	dest = Path(dest)
	dest.mkdir(parents=True, exist_ok=True)
	copy_dirs = ["crates", "docs", ".claude", ".opencode", "tools"]
	for name in copy_dirs:
		src = root / name
		if src.is_dir():
			shutil.copytree(src, dest / name,
				ignore=shutil.ignore_patterns(*SKIP_DIRS), symlinks=True)
	for entry in root.iterdir():
		if entry.name in SKIP_DIRS or entry.name in copy_dirs:
			continue
		target = dest / entry.name
		if target.exists():
			continue
		if entry.is_file():
			shutil.copy2(entry, target)
		else:
			try:
				target.symlink_to(entry, target_is_directory=True)
			except OSError:
				pass
	return dest


def self_test(root, verbose=False):
	"""Two phases: a clean synthetic fixture must be silent; every injected
	drift must make its own check fire. Then the same injections are replayed
	against a COPY of the real tree, where the assertion is a per-class DELTA
	(the real tree has pre-existing findings; a delta proves the check fires
	regardless)."""
	log = []
	failures = []

	# ---- phase 1: synthetic fixture, zero baseline ----------------------- #
	with tempfile.TemporaryDirectory() as tmp:
		fx = build_fixture(Path(tmp) / "clean")
		*_, base, _ = audit(fx)
		hard = [f for f in base if f.severity in ("error", "warn")]
		log.append(f"phase1 clean fixture: {len(base)} finding(s) total, "
			f"{len(hard)} at warn+ (expected 0)")
		if hard:
			failures.append("clean fixture is not silent: " + "; ".join(repr(f) for f in hard))
			for f in hard:
				log.append(f"    unexpected: {f!r}")

	for label, cls, mutate, token in injections():
		with tempfile.TemporaryDirectory() as tmp:
			fx = build_fixture(Path(tmp) / "drifted")
			mutate(fx)
			*_, fs, _ = audit(fx)
			hits = [f for f in fs if f.cls == cls and token.lower() in
				(f.message + " " + (f.detail or "")).lower()]
			ok = bool(hits)
			log.append(f"phase1 inject [{cls}] {label}: "
				f"{'FIRED' if ok else 'SILENT'} ({len(hits)} matching finding(s))")
			if verbose and hits:
				log.append(f"    → {hits[0].severity.upper()} {hits[0].where}:"
					f"{hits[0].line} {hits[0].message}")
			if not ok:
				failures.append(f"injection '{label}' did not trigger class {cls} "
					f"with token {token!r}; got {[repr(f) for f in fs]}")

	# ---- phase 2: real tree copy, per-class delta ------------------------ #
	with tempfile.TemporaryDirectory() as tmp:
		real = _copy_real(root, Path(tmp) / "real")
		*_, base, _ = audit(real)
		base_count = {}
		for f in base:
			base_count[f.cls] = base_count.get(f.cls, 0) + 1
		log.append("phase2 real-tree copy baseline: " + ", ".join(
			f"{c}={base_count.get(c, 0)}" for c in CLASSES))
		real_injections = [
			("op-count", lambda r: _inject(Path(r) / "AGENTS.md", "160-op", "999-op"), "999"),
			("path", lambda r: _inject(Path(r) / "AGENTS.md", "## Layout",
				"## Layout\n- `crates/kernel-ghost/src/nope.rs` — not a thing"), "kernel-ghost"),
			("section", lambda r: _inject(Path(r) / "AGENTS.md", "DESIGN_GUIDE §7.7",
				"DESIGN_GUIDE §7.7 and §99.9"), "§99.9"),
			("symbol", lambda r: _inject(Path(r) / "AGENTS.md", "`ChainLog::seal()`",
				"`ChainLog::seal()` / `kernel_brep::nonexistent_symbol_xyz`"),
				"nonexistent_symbol_xyz"),
			("claim", lambda r: _inject(Path(r) / "AGENTS.md",
				"repro `keyed_pulley_acceptance.rs`",
				"repro `crates/kernel-brep/tests/never_written_xyz.rs`"), "never_written_xyz"),
			("op-family", lambda r: _inject(Path(r) / "DESIGN_GUIDE.md",
				"| booleans (4) | union, difference, intersection, union_all |",
				"| booleans (4) | union, difference, intersection, union_alll |"),
				"union_alll"),
		]
		for cls, mutate, token in real_injections:
			with tempfile.TemporaryDirectory() as tmp2:
				dup = Path(tmp2) / "inj"
				shutil.copytree(real, dup, symlinks=True)
				try:
					mutate(dup)
				except AssertionError as e:
					log.append(f"phase2 [{cls}]: SKIPPED — {e}")
					failures.append(f"phase2 anchor missing for {cls}: {e}")
					continue
				*_, fs, _ = audit(dup)
				after = sum(1 for f in fs if f.cls == cls)
				named = [f for f in fs if f.cls == cls and token.lower() in
					(f.message + " " + (f.detail or "")).lower()]
				grew = after > base_count.get(cls, 0)
				ok = grew and named
				log.append(f"phase2 inject [{cls}] token {token!r}: "
					f"{base_count.get(cls, 0)} → {after} finding(s), "
					f"named={len(named)} {'OK' if ok else 'FAILED'}")
				if not ok:
					failures.append(f"phase2 injection for {cls} did not raise a new "
						f"finding naming {token!r}")

	return log, failures


# --------------------------------------------------------------------------- #

def main(argv=None):
	ap = argparse.ArgumentParser(description=__doc__,
		formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("--root", default=None, help="repo root (default: parent of tools/)")
	ap.add_argument("--json", action="store_true", help="machine-readable findings")
	ap.add_argument("--fail-on", default="error", choices=list(SEVERITY),
		help="exit 1 when any finding is at or above this severity (default: error)")
	ap.add_argument("--all-docs", action="store_true", help="also audit docs/*.md")
	ap.add_argument("--only", action="append", choices=CLASSES,
		help="restrict to one or more check classes (repeatable)")
	ap.add_argument("--min-op-claim", type=int, default=40,
		help="numbers below this are treated as chain-length idioms (default: 40)")
	ap.add_argument("--verbose", action="store_true",
		help="also report op-count candidates skipped by the heuristics")
	ap.add_argument("--self-test", action="store_true",
		help="prove every check can fail (injected drift on a fixture and on a "
			"copy of the real tree); exit 0 only if every check fires")
	args = ap.parse_args(argv)

	root = Path(args.root).resolve() if args.root else Path(__file__).resolve().parent.parent

	if args.self_test:
		log, failures = self_test(root, verbose=args.verbose)
		if args.json:
			print(json.dumps({"log": log, "failures": failures,
				"ok": not failures}, indent=2))
		else:
			print("audit_docs.py --self-test")
			for line in log:
				print("  " + line)
			print()
			if failures:
				print(f"SELF-TEST FAILED ({len(failures)} problem(s)):")
				for f in failures:
					print("  - " + f)
			else:
				print("SELF-TEST PASSED — every check class fired on injected drift, "
					"and the clean fixture is silent.")
		return 2 if failures else 0

	repo, truth, docs, findings, stats = audit(root, all_docs=args.all_docs,
		min_claim=args.min_op_claim, verbose_skips=args.verbose,
		only=set(args.only) if args.only else None)

	gate = [f for f in findings if SEVERITY[f.severity] >= SEVERITY[args.fail_on]]
	if args.json:
		print(json.dumps({
			"root": str(repo.root),
			"op_truth": {"count": truth.n, "source": DISCOVER_REL,
				"op_count_const": truth.count},
			"corpus": [d.rel for d in docs],
			"references_checked": stats,
			"fail_on": args.fail_on,
			"findings": [f.as_dict() for f in findings],
			"totals": {"all": len(findings), "gating": len(gate)},
		}, indent=2))
	else:
		text, _ = render(truth, docs, findings, args.fail_on, stats)
		print(text)
	return 1 if gate else 0


if __name__ == "__main__":
	sys.exit(main())
