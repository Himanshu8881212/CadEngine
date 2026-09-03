# PRINTABLES LISTING SPEC — binding for every campaign

Every campaign ships `publish/PRINTABLES_LISTING.md` written to this spec and a
`publish/check_listing.py` that (a) runs `campaign/listing_lint.py` on it and (b) proves
every number in it against a receipt. `run_all.sh` must call the checker. The listing is a
deliverable with an audience; it is finished only when it passes both.

Sources (read 2026-09-02 through the live site, logged-in upload form
`printables.com/model/create`; contest page `printables.com/contest/524`; Prusa blog
"New on Printables: community feed, improved search"; Prusa KB "How do Prusameters
work"; Prusa forum "Best practices for publishing prints"; community template article
jZ993ZD). Re-verify limits when the form changes: the lint carries them as constants.

## 1. The form, field by field (what the site actually asks)

| field | rule on the site | our rule |
|---|---|---|
| Files (drag-drop) | model in .3MF or .STL required; STEP/others allowed; supported: 3mf stl step stp obj amf ply scad f3d fcstd blend dxf svg pdf txt csv zip gcode bgcode png jpg webp… | upload every print STL **and** the STEPs and a `.3mf` print project when we have one; never g-code alone; a zip of "sources" is fine |
| Model name (required) | placeholder "Descriptive names are better"; no hard limit | ≤ 70 characters (cards truncate); pattern `<What it is> for <who/what> — <the hook>`; the object noun first, the searchable product name (e.g. "Framework Laptop 12 Mainboard") verbatim |
| Summary (required) | **120 characters hard limit**, textarea, shows `n/120` | one sentence, ≤ 120 chars, no trailing period needed; it is the search-card subtitle: promise + differentiator, no jargon |
| Main category (required) | single pick from the site tree | pick the narrowest category that a searcher would browse; note the path in the listing header |
| Additional tags | free text, **space-separated**; **the field rejects hyphens** (user-verified 2026-09-02), so a tag is one alphanumeric word | 12–20 single-word tags: object nouns, brand and product words split into their words (`framework laptop12 mainboard`), use-cases, material, mount type, `nosupports`; search indexes name + description + tags, so tags are the only place synonyms live |
| Model origin (required radio) | Original / Remix (must cite source) / Reupload (no Prusameters) | Original for our designs; Remix if any geometry is derived from someone's model (then link it) |
| **Was AI used to create this model?** (required radio) | "Yes — AI-assisted creation" / "No — fully human-made"; "You are responsible for labeling this model correctly. Incorrect or misleading labeling may result in model removal." | **Always "Yes — AI-assisted creation" for anything this engine produced.** Never advise otherwise. |
| NSFW / Political flags | checkboxes | off |
| Description | rich-text editor: headings, bold, lists, tables, images, links, video embeds; 4+ headings → automatic table of contents | our template in §3; paste as headings + lists; upload our renders/sheets inline |
| License (required) | dropdown: Creative Commons family (BY, BY-SA, BY-NC, BY-NC-SA, BY-ND, BY-NC-ND, CC0), GPL/LGPL/BSD, Standard Digital File License… | CC BY 4.0 unless the user says otherwise; must be compatible with any vendor data used (Framework CAD is CC BY 4.0 → attribution line in the description) |
| Photos / renders | jpg png gif webp; first image = cover; cover crop is 4:3 in cards | cover 2000 × 1500 (4:3), ≥ 6 images, order in §4; renders labelled as renders in the caption |
| Print files (optional) | per-file metadata: printer, nozzle, layer height, material, supports, infill, print time, weight | attach when a sliced `.3mf`/g-code for a mainstream printer exists; otherwise put the settings in the description |
| Publish → Compete | the Compete medal appears next to your name on the published page; pick the contest | the user does this; the listing header carries the exact contest name |

## 2. Contest mechanics (read the live T&C every time; these were true on 2026-09-02)

- Up to 5 unique models per user; the model must be **uploaded inside the contest window**.
- The entry must contain at least one printed part and every model needed to complete the project.
- Stays published ≥ 1 year (Prusa may repost it if removed).
- You must be the original creator of the model **and the photos**.
- **"AI-generated models are not allowed. Please submit only designs you created yourself."**
  The site's own form asks for an AI declaration. Both must be answered truthfully.
  **Agent rule:** before any campaign is entered, put this clause and the honest
  declaration ("Yes — AI-assisted creation") in front of the user in the final report and
  in the listing header, and record it in memory. Eligibility is the organiser's call and
  the user's decision; laundering the declaration is forbidden.
- Judging criteria (verbatim headings): overall quality · printability (orientation,
  parts split to avoid supports) · originality · popularity (shares, likes) · assembly
  instructions · photo quality · your other works.
- Physical ownership of the subject (e.g. the mainboard) is not required when the
  organiser provides CAD; a render is accepted ("photo or render, ideally both").

## 3. Description template (headings in this order; ≥ 4 headings so the site builds a TOC)

```
[Cover image: the object in use, 4:3]

<One-line hook, bold.> One sentence that says what the reader gets and the single most
surprising fact (a number, a "no hardware", a "prints in one plate").

## What it is
3–6 bullets. Object, who it is for, the differentiators. Numbers from receipts only.

## Print it
Material, layer height, perimeters, infill, supports (none), bed size needed, plates and
time from the dossier, orientation note per part ("exported in print orientation, do not
rotate"). Files listed with one line each.

## Assemble it
Numbered steps mirroring assembly/ASSEMBLY_instructions.md; link the exploded sheet.

## Verified (what the engine actually checked)
5–8 bullets of receipts in plain words: fits, controls, flexure forces/strains, FEA,
tolerance stacks. Every number must be provable by check_listing.py.

## Not verified / known limits
Honest list: no physical test, assumptions, temperature limits, untested printers.
Renders are declared as renders.

## Credits and license
License line; vendor data attribution (name, © holder, license); "Designed with the LMCAD
engine, AI-assisted" (matches the form declaration).

## Contest
Exact contest name, the rule-by-rule mapping if the contest has design requirements.
```

Rules of tone: verbs, short sentences, no adjectives that a receipt cannot back
("robust", "perfect"), no emoji walls, one exclamation mark per listing at most, no
"please like" (the judges read popularity from real shares).

## 4. Images (cover first)

1. Cover: the object **in use**, 4:3, no text overlay, single subject, plain background.
2. Lid/enclosure open or the part that proves the function.
3. Straight-on view that answers the reader's first question (fit, ports, size).
4. A detail that shows craft (snap, latch, flexure).
5. Optional configuration (mount, stand).
6. Exploded view / assembly sheet.
7. Plate layout or print orientation.
Photos of a real print replace 1–4 the day they exist; until then renders, captioned "render".

## 5. Discoverability and the first 30 days

- Search indexes **name, description, tags**. The product name and the object noun
  must appear in all three. Synonyms go in tags (`case`, `enclosure`, `housing`).
- Prusameter milestones run for 30 days from publish (30 dl/3 likes → 400 dl/20 likes):
  ship the listing complete on day one; edits later do not restart the window.
- The summary is what the card shows: spend the 120 characters on the promise, not on
  the category.
- One listing per project; do not split a set into several models.

## 6. Pre-publish checklist (the lint enforces the mechanical ones)

- [ ] header block present and complete (§7)
- [ ] name ≤ 70 chars, contains the object noun and the product name
- [ ] summary ≤ 120 chars, one sentence
- [ ] 12–20 tags, space-separated, lowercase, letters and digits only (no hyphens), the product's words present
- [ ] license set and compatible with vendor data; attribution line present
- [ ] AI declaration = Yes — AI-assisted creation; origin = Original (or Remix with link)
- [ ] ≥ 4 headings, in the template order
- [ ] every number in the text matched to a receipt by check_listing.py
- [ ] "Not verified" section present and truthful; renders labelled
- [ ] contest T&C re-read on the day; AI clause surfaced to the user
- [ ] files list matches `parts/` and `cad/`

## 7. Header block (machine-checked)

The listing starts with a fenced block the lint parses:

```
---
name: <model name>
summary: <≤120 chars>
category: <site category path>
tags: tagone tagtwo ...          (letters and digits only, space-separated)
license: CC BY 4.0
origin: original
ai: yes-assisted
contest: <exact contest name or none>
cover: renders/<file>.png
---
```
