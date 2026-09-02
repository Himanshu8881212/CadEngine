#!/usr/bin/env python3
"""gen_discover.py — regenerate crates/kernel-api/src/discover.rs from program.rs.

Single source of truth: the `OpKind` enum in crates/kernel-api/src/program.rs.
This script parses its variants AND their fields and emits the WHOLE of
discover.rs — `op_tag` (the compile-forced exhaustive match), `OP_NAMES`,
`OP_COUNT`, `ParamSpec` + `OP_PARAMS` (the per-op parameter table behind
`describe {name}`), and the `op_params` lookup — deterministically: running it
twice produces byte-identical output, and running it against an unchanged
program.rs reproduces the committed discover.rs exactly (CI-checkable with
`git diff --exit-code crates/kernel-api/src/discover.rs`).

Per-field extraction:
  name      wire name after `#[serde(rename = "...")]` (`in`, `bool`, ...)
  ty        a friendly type string: number / int / string / bool / id-ref /
            [x,y,z] / [[x,y]...] / object / ... (Rust types mapped pragmatically;
            String fields named in/a/b/sketch are id-refs; string-serialized
            unit enums like FitSpec are `string`; spec structs are `object`)
  required  false iff the field is Option<T> or carries #[serde(default...)]
  doc       the first sentence of the field's /// doc comment ("" when absent)

Run from anywhere:  python3 tools/gen_discover.py
Exits non-zero (without writing) if it meets a type or variant shape it cannot
map — extend the mapping instead of hand-editing discover.rs.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PROGRAM_RS = REPO / "crates" / "kernel-api" / "src" / "program.rs"
DISCOVER_RS = REPO / "crates" / "kernel-api" / "src" / "discover.rs"

# String fields with these wire names reference a previously bound result id.
ID_REF_NAMES = {"in", "a", "b", "sketch"}
# Unit-variant enums that serde serializes as a plain string value.
STRING_ENUMS = {"MesherSpec", "FitSpec", "BoolOpSpec"}
# Sentence-boundary suppression: tokens whose trailing '.' is an abbreviation.
ABBREVS = {"e.g", "i.e", "etc", "vs", "cf"}


def snake(name: str) -> str:
    out = []
    for i, ch in enumerate(name):
        if ch.isupper():
            if i > 0:
                out.append("_")
            out.append(ch.lower())
        else:
            out.append(ch)
    return "".join(out)


def first_sentence(doc_lines: list[str]) -> str:
    """Join /// lines and cut at the first real sentence boundary."""
    text = " ".join(l.strip() for l in doc_lines).strip()
    i = 0
    while True:
        j = text.find(".", i)
        if j == -1:
            return text
        nxt = text[j + 1] if j + 1 < len(text) else " "
        if not (nxt == " " or j + 1 == len(text)):
            i = j + 1  # mid-token period: `.stl`, `0.5`, the first dot of `e.g.`
            continue
        tok = re.split(r"[\s(]", text[:j])[-1].lower()
        if tok in ABBREVS or (len(tok) == 1 and tok.isalpha()):
            i = j + 1  # abbreviation ("e.g.") — not a sentence end
            continue
        return text[: j + 1]


def friendly(ty: str, wire_name: str) -> str:
    """Map a Rust field type to the friendly string advertised by describe."""
    ty = ty.strip().rstrip(",").strip()
    if ty.startswith("Option<") and ty.endswith(">"):
        return friendly(ty[7:-1], wire_name)
    if ty == "f64":
        return "number"
    if ty in ("usize", "u16", "u32", "u64", "i32", "i64"):
        return "int"
    if ty == "bool":
        return "bool"
    if ty == "String":
        return "id-ref" if wire_name in ID_REF_NAMES else "string"
    m = re.fullmatch(r"\[\s*(\w+)\s*;\s*(\d+)\s*\]", ty)
    if m:
        base, n = m.group(1), int(m.group(2))
        if base == "f64":
            named = {2: "[x,y]", 3: "[x,y,z]"}
            return named.get(n, "[" + ",".join(["number"] * n) + "]")
        return "[" + ",".join([friendly(base, wire_name)] * n) + "]"
    m = re.fullmatch(r"Vec<(.+)>", ty)
    if m:
        return "[" + friendly(m.group(1), wire_name) + "...]"
    if ty == "serde_json::Value" or ty.startswith("BTreeMap<"):
        return "object"
    if ty in STRING_ENUMS:
        return "string"
    if re.fullmatch(r"[A-Z]\w*", ty):
        return "object"  # a *Spec struct / tagged enum — a nested JSON object
    sys.exit(f"gen_discover.py: unmapped Rust type {ty!r} (field {wire_name!r}) — extend friendly()")


def is_optional(ty: str, has_default: bool) -> bool:
    return has_default or ty.strip().startswith("Option<")


def split_top_level(text: str) -> list[str]:
    """Split `a: T, b: U` on commas not nested in [], (), or <>."""
    parts, depth, cur = [], 0, []
    for ch in text:
        if ch in "[(<":
            depth += 1
        elif ch in "])>":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    parts.append("".join(cur))
    return [p.strip() for p in parts if p.strip()]


def parse_fields(body: str) -> list[dict]:
    """Parse a struct-variant body: field docs, serde attrs, declarations."""
    fields: list[dict] = []
    doc: list[str] = []
    rename: str | None = None
    aliases: list[str] = []
    has_default = False
    for raw in body.split("\n"):
        s = raw.strip()
        if not s:
            continue
        if s.startswith("///"):
            doc.append(s[3:].strip())
            continue
        if s.startswith("//"):
            continue
        m = re.fullmatch(r"#\[serde\((.*)\)\]", s)
        if m:
            inner = m.group(1)
            rm = re.search(r'rename\s*=\s*"([^"]+)"', inner)
            if rm:
                rename = rm.group(1)
            if re.search(r"\bdefault\b", inner):
                has_default = True
            aliases += re.findall(r'alias\s*=\s*"([^"]+)"', inner)
            continue
        for decl in split_top_level(s.rstrip(",")):
            dm = re.fullmatch(r"(\w+)\s*:\s*(.+)", decl)
            if not dm:
                sys.exit(f"gen_discover.py: cannot parse field declaration {decl!r}")
            rust_name, ty = dm.group(1), dm.group(2).strip()
            wire = rename if rename else rust_name
            fields.append({
                "name": wire,
                "ty": friendly(ty, wire),
                "required": not is_optional(ty, has_default),
                "doc": first_sentence(doc),
                "aliases": aliases,
            })
            doc, rename, aliases, has_default = [], None, [], False
    return fields


def parse_opkind() -> list[dict]:
    """Extract every OpKind variant: (rust name, wire tag, shape, fields)."""
    src = PROGRAM_RS.read_text(encoding="utf-8")
    lines = src.splitlines()
    start = next(i for i, l in enumerate(lines) if re.match(r"\s*pub enum OpKind\s*\{", l))

    variants: list[dict] = []
    pending_rename: str | None = None
    i, depth = start + 1, 0
    while i < len(lines):
        line = lines[i]
        s = line.strip()
        if depth == 0:
            if s == "}":
                break
            if s.startswith("///") or s.startswith("//") or not s:
                i += 1
                continue
            am = re.fullmatch(r'#\[serde\(rename\s*=\s*"([^"]+)"\)\]', s)
            if am:
                pending_rename = am.group(1)
                i += 1
                continue
            vm = re.match(r"([A-Z][A-Za-z0-9]*)\s*(\{|\(|,)", s)
            if vm:
                name, delim = vm.group(1), vm.group(2)
                tag = pending_rename if pending_rename else snake(name)
                pending_rename = None
                if delim == ",":
                    variants.append({"name": name, "tag": tag, "shape": "unit", "fields": []})
                elif delim == "(":
                    variants.append({"name": name, "tag": tag, "shape": "tuple", "fields": []})
                else:
                    # Struct variant: capture the brace-balanced body (inline or multi-line).
                    chunk = [line[line.index("{") + 1:]]
                    d = 1 + chunk[0].count("{") - chunk[0].count("}")
                    j = i
                    while d > 0:
                        j += 1
                        d += lines[j].count("{") - lines[j].count("}")
                        chunk.append(lines[j])
                    body = "\n".join(chunk)
                    body = body[: body.rindex("}")]  # drop the closing brace (+ trailing `,`)
                    variants.append({"name": name, "tag": tag, "shape": "struct", "fields": parse_fields(body)})
                    depth = 0
                    i = j + 1
                    continue
        depth += line.count("{") - line.count("}")
        i += 1
    return variants


def rs_str(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit(variants: list[dict]) -> str:
    arm_pat = {
        "struct": '\t\tOpKind::{n} {{ .. }} => "{t}",',
        "tuple": '\t\tOpKind::{n}(..) => "{t}",',
        "unit": '\t\tOpKind::{n} => "{t}",',
    }
    arms = "\n".join(arm_pat[v["shape"]].format(n=v["name"], t=v["tag"]) for v in variants)
    names = ",\n".join(f'\t"{v["tag"]}"' for v in variants)
    count = len(variants)

    param_rows = []
    for v in variants:
        if not v["fields"]:
            param_rows.append(f'\t({rs_str(v["tag"])}, &[]),')
            continue
        specs = "\n".join(
            "\t\tParamSpec { name: %s, ty: %s, required: %s, doc: %s, aliases: &[%s] },"
            % (rs_str(f["name"]), rs_str(f["ty"]), "true" if f["required"] else "false", rs_str(f["doc"]),
               ", ".join(rs_str(a) for a in f.get("aliases", [])))
            for f in v["fields"]
        )
        param_rows.append(f'\t({rs_str(v["tag"])}, &[\n{specs}\n\t]),')
    params_table = "\n".join(param_rows)

    return f'''//! Self-describing op surface (M3 Discovery). The op catalogue AND the per-op parameter table
//! are derived from the [`OpKind`] enum: `op_tag` through a **compile-forced exhaustive match**
//! (adding a variant without regenerating fails to compile), [`OP_NAMES`]/[`OP_PARAMS`] as
//! generated tables pinned to it by `tests/describe.rs`. Regenerate this WHOLE file with
//! `python3 tools/gen_discover.py` whenever `program.rs`'s `OpKind` changes — never hand-edit.

use crate::program::OpKind;

/// Canonical wire tag of an op — matches serde's `rename_all = "snake_case"` plus every explicit
/// `#[serde(rename)]`. The match is EXHAUSTIVE (the compiler forces one arm per variant); that is
/// the anti-drift guarantee behind [`OP_NAMES`] and the `describe` op.
pub fn op_tag(op: &OpKind) -> &'static str {{
\tmatch op {{
{arms}
\t}}
}}

/// The authoritative catalogue every supported op tag, in declaration order returned by the
/// `describe` op and generated from the same source as [`op_tag`]. Length is pinned to the variant
/// count by `tests/describe.rs`; every entry is proven executable (never `unknown_op`) there too.
pub const OP_NAMES: &[&str] = &[
{names},
];

/// Number of supported ops. Kept in lockstep with the `OpKind` variant count via [`op_tag`].
pub const OP_COUNT: usize = {count};

/// One parameter of an op, as served by `describe {{name}}`: the JSON wire name (post
/// `#[serde(rename)]` — e.g. `in`), a friendly type string (`number` / `int` / `string` /
/// `bool` / `id-ref` / `[x,y,z]` / `[[x,y]...]` / `object` / ...), whether the field is
/// required (no `Option` and no serde default), the first sentence of its doc comment, and
/// every accepted `#[serde(alias)]` wire spelling (the fail-closed unknown-param check and
/// `describe` both honour aliases — an accepted spelling is never refused as unknown).
#[derive(Clone, Copy, Debug)]
pub struct ParamSpec {{
\tpub name: &'static str,
\tpub ty: &'static str,
\tpub required: bool,
\tpub doc: &'static str,
\tpub aliases: &'static [&'static str],
}}

/// Per-op parameter specs, parallel to [`OP_NAMES`] (same tags, same declaration order — pinned
/// by `tests/describe.rs`). Generated from the `OpKind` field lists by `tools/gen_discover.py`.
pub static OP_PARAMS: &[(&str, &[ParamSpec])] = &[
{params_table}
];

/// The parameter specs of one op by wire tag (`None` for an unknown tag) — the lookup behind
/// `describe {{name}}`.
pub fn op_params(name: &str) -> Option<&'static [ParamSpec]> {{
\tOP_PARAMS.iter().find(|(tag, _)| *tag == name).map(|(_, specs)| *specs)
}}
'''


def main() -> None:
    variants = parse_opkind()
    out = emit(variants)
    n_fields = sum(len(v["fields"]) for v in variants)
    DISCOVER_RS.write_text(out, encoding="utf-8")
    print(f"wrote {DISCOVER_RS.relative_to(REPO)}: {len(variants)} ops, {n_fields} param specs")


if __name__ == "__main__":
    main()
