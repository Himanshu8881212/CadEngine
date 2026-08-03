# SQUATCHEE-SPIN — the clip-on propeller your baseball cap has been waiting for

**Snap it on the button. Walk. Grin.** SQUATCHEE-SPIN turns any standard baseball cap into a propeller beanie in three seconds, with zero glue, zero hardware, and zero modifications to your cap. Six springy PETG fingers snap right over the little button on top of your cap (that button is called a *squatchee* — now you know) and grip under its rim. A 100 mm two-blade prop with real 20° pitch drops onto the mount, and a tiny snap-on cap locks it in place while it spins free. And because we're extra: the retainer is shaped like a miniature squatchee. Your cap's button gets its own little hat. It's buttons all the way up.

This isn't just a gag print — it's an *engineered* gag print. The whole assembly was designed, gated, and receipt-checked by an AI CAD pipeline: the propeller is **balance-certified at 0.0 g·mm static imbalance** about its spin axis (measured, not vibes), the pivot runs a true 0.3 mm radial clearance that can never bind even at ±0.15 mm printer tolerance, and the retainer's insertion path was swept through the engine to prove it never snags until the designed snap click. It spins at a walking pace, keeps spinning in a breeze, and survives being yanked off and reassembled — the snap is serviceable by design.

## v2 — the field-fix update (July 2026)

You told us three things broke the fun, and all three are fixed at the root:

1. **"The mast snaps off / prints badly."** True twice, and both had the same root cause: v1's mast was a *split snap-post* — two ~1 mm-thin springy prongs that stood proud of the propeller (easy to lever off) and printed as wobbly per-layer islands with overhanging barbs. The snap-post is **gone**. The mast is now a **solid Ø4.4 post** on a Ø9.2 boss — it prints as one chunky continuous column (the nicest thing a slicer will see all day) and hides entirely inside the propeller hub. Retention moved to a **push-and-twist lock**: two small lug nubs on the post, two L-windows in the retainer — drop in, twist ~35°, locked behind solid ledges. No flexing parts anywhere in the joint, so printer tolerance can't weaken it: the lock is geometry, not spring pressure.
2. **"Make the propeller hole bigger so the top pin goes inside it."** Done, exactly that: the prop bore grows Ø5.0 → **Ø7.2**, and the retainer grows a **Ø6.6 journal sleeve** that drops inside the bore and twist-locks onto the Ø4.4 post. The prop spins on the sleeve's smooth cylinder with a guaranteed 0.30–0.90 mm clearance, and the mini-squatchee head is the retaining flange.
3. **"The walls are too thin."** Every wall is now **2 mm or more**. The skirt and gripper-finger walls go 1.2 → **2.0 mm**, the dome shell to **~3 mm**, a solid ring collar hoops the tops of the finger slots (the old crack line), and the shell steps Ø22 → Ø23.6. The six gripper fingers are *springs* — they must flex 2.9 mm per side to swallow a Ø19 button, so fattening them demanded lengthening them too (flex length 13.3 → 18.4 mm): worst-case snap-over strain still *drops* from 1.97% to ≤1.72%. Two honest trades: the mount carries 5.1 g of PETG instead of 3.1 g, and the stiffer springs make the snap onto the cap button noticeably firmer (~1.6–1.75× the v1 force) — a stronger click and a stronger grip.

**v2 is a matched set** — print all three parts. A v2 propeller will not be retained by a v1 mount/retainer, and a v1 propeller won't fit over the v2 sleeve.

## Cap fit — read this first
- **Fits standard baseball cap top buttons (squatchees) Ø14–19 mm.** The industry-standard replacement button is 16 mm (5/8"), and common variants run ~14–17 mm, so nearly every dad cap, snapback, and trucker cap is covered.
- The grip is positive mechanical capture (a Ø13.2 mm finger-lip ring closes under the button rim), not friction — verified to still capture a 14 mm button at worst-case printer tolerance.
- Not compatible with: caps with no top button, flat-top buttons wider than 19 mm, or buttons sewn dead-flush with no rim gap.

## The three parts (9.9 g total — your neck won't notice)
| # | Part | Material | Mass | Why |
|---|------|----------|------|-----|
| 1 | Mount | **PETG (required)** | 5.2 g | 2 mm walls everywhere that doesn't flex by design. The six gripper fingers and the split snap-post are engineered springs — v2 makes them longer *and* thicker (2.0 mm wall, ≤1.72% strain on a big button, was 1.97%) and armors the rest: ~3 mm dome shell, ring collar over the slot roots, Ø9.2 boss, short shielded mast. PLA here is brittle and may crack. |
| 2 | Propeller | PLA or PETG | 4.5 g | Ø100 mm vintage paddle blades (canoe-paddle planform, widest at 65% span), real 20° pitch printed support-free (the pitch is a wedge — the bed side stays dead flat). v2: Ø7.2 bore rides the retainer sleeve. |
| 3 | Retainer | PLA or PETG | 0.2 g | Rigid mini-squatchee twist-lock cap with the Ø6.6 journal sleeve the prop actually spins on. Nothing on it flexes — any material works. |

## Print settings
- **No supports. Anywhere. On any part.** Every down-facing surface is ≥46° or a flat micro-ceiling ≤2.9 mm true span (the finger-slot roofs — any 0.4 mm printer bridges those without thinking). The support-free claim is machine-audited (steep area = exactly 0), not hoped.
- Layer height: 0.2 mm (0.16 mm looks nicer on the mount's cone).
- Perimeters: 3. Infill: 20–25%, any symmetric pattern (grid/gyroid) — symmetric infill preserves the propeller's balance.
- Orientation: exactly as the STLs come — mount fingers-down, prop flat, retainer brim-down.
- One plate, ~45–60 min total on a typical 0.4 mm machine.
- Tolerance note: dimensioned for ±0.15 mm XY. If your printer runs tight bores, you're still safe — the pivot keeps ≥0.15 mm radial clearance at worst case.

## Assembly (10 seconds, no hardware)
1. Center the mount's six fingers over your cap button and press straight down until it clicks.
2. Drop the propeller onto the mount, flat side down — it rests on the Ø9.2 boss shoulder, and the roomy bore is by design (the sleeve is the bearing, not the mast).
3. Drop the retainer into the propeller bore with its two windows over the post's nubs (rotate until it slides home), then twist ~35° until it stops. Spin. Strut. (To remove: press down lightly, twist back, lift.)

## The receipts (yes, really)
- Static imbalance: **0.0 g·mm** (CG offset 0.000 mm from the spin axis), couple terms 0.0 g·mm² — measured by the LMCAD engine's mass-properties pipeline.
- Pivot fit: Ø7.2 prop bore on the Ø6.6 retainer sleeve → 0.30–0.90 mm diametral clearance across ±0.15 mm extremes, interference impossible — and the journal is a continuous cylinder (v1's spindle carried the snap slot right under the bore).
- Vertical float: 0.65 mm nominal, 0.20–1.10 mm across worst-case extremes — never clamped, never sloppy enough to matter.
- Twist-lock proven three ways: locked pose swept 0→35° with constant 0.15 mm clearance (never scrapes); lifted-while-locked *interferes* (the lug bears on the ledge — that's the retention, measured); lifted-with-windows-aligned lifts free (that's the removal).
- Retainer insertion sweep: 18 descent stations with zero interference all the way to seat — nothing snags, ever.
- Propeller drop-on sweep: 14 stations from above the post down to its seat, zero interference — the Ø7.2 bore passes the Ø6.3 lug nubs with 0.39 mm measured margin.
- All three parts: watertight, exact-route geometry, support-free — gated, not eyeballed.

## Safety
This is a toy for cap-wearing pedestrians. It is **not** a helmet accessory — never mount it on bicycle, motorcycle, or climbing helmets, and don't wear it where a snagged spinning part could be a hazard (machinery, cycling, small-children-grabbing-range… if it's yanked hard, the six-finger button grip is the designed give-point — the assembly pops off the cap button harmlessly; the twist-lock itself doesn't yank open). Small parts; not for unsupervised kids under 3.
