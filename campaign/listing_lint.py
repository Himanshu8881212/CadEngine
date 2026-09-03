#!/usr/bin/env python3
"""listing_lint.py — mechanical checks for a Printables listing written to
campaign/PRINTABLES_LISTING_SPEC.md. Exit 1 on any violation.
Usage: python3 campaign/listing_lint.py <path/to/PRINTABLES_LISTING.md>
Limits verified on the live upload form 2026-09-02 (summary 120 hard); update here."""
import re, sys
NAME_MAX, SUMMARY_MAX, TAGS_MIN, TAGS_MAX, HEADINGS_MIN = 70, 120, 12, 20, 4
REQUIRED = ("name", "summary", "category", "tags", "license", "origin", "ai", "contest", "cover")
ORDER = ["What it is", "Print it", "Assemble it", "Verified", "Not verified", "Credits", "Contest"]

def main(path):
    txt = open(path, encoding="utf-8").read()
    bad = []
    m = re.match(r"^---\n(.*?)\n---\n", txt, re.S)
    if not m:
        print("LINT FAILED: no header block"); return 1
    hdr = {}
    for line in m.group(1).splitlines():
        if ":" in line:
            k, v = line.split(":", 1); hdr[k.strip()] = v.strip()
    for k in REQUIRED:
        if not hdr.get(k): bad.append("header missing '%s'" % k)
    name, summary = hdr.get("name", ""), hdr.get("summary", "")
    if len(name) > NAME_MAX: bad.append("name %d chars > %d" % (len(name), NAME_MAX))
    if len(summary) > SUMMARY_MAX: bad.append("summary %d chars > %d (site hard limit)" % (len(summary), SUMMARY_MAX))
    if summary.count(". ") > 0: bad.append("summary must be one sentence")
    tags = hdr.get("tags", "").split()
    if not (TAGS_MIN <= len(tags) <= TAGS_MAX): bad.append("tags: %d, need %d..%d" % (len(tags), TAGS_MIN, TAGS_MAX))
    for t in tags:
        if not re.fullmatch(r"[a-z0-9]+", t): bad.append("tag must be lowercase letters/digits only (the site rejects hyphens): %s" % t)
    if hdr.get("ai") != "yes-assisted": bad.append("ai must be 'yes-assisted' for engine-made models")
    if hdr.get("origin") not in ("original", "remix"): bad.append("origin must be original|remix")
    if "CC" not in hdr.get("license", "") and "GPL" not in hdr.get("license", ""): bad.append("license not recognised")
    body = txt[m.end():]
    heads = re.findall(r"^##\s+(.+)$", body, re.M)
    if len(heads) < HEADINGS_MIN: bad.append("only %d '##' headings; site builds a TOC at %d" % (len(heads), HEADINGS_MIN))
    pos = [next((i for i, h in enumerate(heads) if h.startswith(o)), None) for o in ORDER]
    if any(p is None for p in pos): bad.append("missing sections: %s" % [o for o, p in zip(ORDER, pos) if p is None])
    elif pos != sorted(pos): bad.append("sections out of template order: %s" % heads)
    if body.count("!") > 1: bad.append("more than one exclamation mark")
    for word in ("robust", "perfect", "amazing", "best ever"):
        if re.search(r"\b%s\b" % word, body, re.I): bad.append("unbacked adjective: %s" % word)
    if "render" not in body.lower(): bad.append("renders must be declared (or state that photos are real)")
    if "AI-assisted" not in body: bad.append("description must state AI-assisted (matches the form declaration)")
    if hdr.get("contest", "none") != "none" and "AI-generated models are not allowed" not in body:
        bad.append("contest listing must quote the contest AI clause in the Contest section")
    if bad:
        print("LINT FAILED (%s)" % path); [print("  -", b) for b in bad]; return 1
    print("listing lint ok: name %d/%d, summary %d/%d, %d tags, %d headings" % (len(name), NAME_MAX, len(summary), SUMMARY_MAX, len(tags), len(heads)))
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
