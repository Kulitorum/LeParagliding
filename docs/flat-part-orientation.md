# Fabric orientation of flat parts — an unsolved input

Status: **known gap, deliberately not solved.** The nester currently assumes
the exported orientation is correct. It is not. This note records why, what
breaks, and what the C++ port of the core has to provide to fix it properly.

## The problem

`lep-2d-parts.json` gives every piece an outline in its own local frame, and
the Print tab's nester rotates pieces to pack them. Rotation is restricted to
quarter turns by default so that "the grain is preserved".

That restriction is only meaningful if the exported orientation *is* the grain
orientation. **It is not.** The translated Fortran core never records fabric
orientation at all. Each part is drawn wherever its plotter layout puts it —
`sepx/sepy` for ribs, `psep/psey` plus `seppix[i]` for panels, `xrsep/yrsep`
for V-ribs — and those positions exist to fill the plan's 1260 x 890.95 drawing
boxes legibly, not to say anything about the weave.

So "quarter turns preserve grain" is currently shorthand for "quarter turns
preserve *whatever orientation the plan happened to draw*". For a part whose
drawn orientation already matches its intended grain, that is right. For any
other part it is preserving the wrong thing.

The export marks this honestly: every piece carries
`"grainSource": "assumed-from-plan-layout"` alongside its `grainAngleDeg`.
Treat the angle as unknown, not as data. Any consumer that starts relying on it
should check `grainSource` first.

## Why it matters

Ripstop is orthotropic. Along warp and along weft it is stiff and behaves
similarly; **on the bias — any direction off the two thread axes — it stretches
substantially.** A panel cut at 45° to the weave will elongate under load in a
way the design never accounted for, and the wing will not hold its designed
shape in flight. This is not a finish-quality issue, it is an aerodynamic one.

Two consequences for the packer:

- Free rotation is only ever valid for **paper templates**, which the builder
  repositions on the fabric by eye. It must never be used to drive a cutter
  working on fabric. The Print tab labels it accordingly.
- Quarter-turn packing is *safe against bias* regardless of the above, because
  0/90/180/270 keep the weave square to the part either way. What it cannot do
  is guarantee the part is oriented the way the designer intended relative to
  the load path. Bias is avoided; correctness is not established.

## What the C++ core must provide

A per-piece grain direction as a first-class output, not an afterthought:

- **A vector or angle in the piece's own local frame**, emitted next to the
  outline. The frame is already well defined (part-local millimetres, y-up,
  origin at the piece's bottom-left).
- **Stable across regeneration.** Nesting results and any user-supplied
  overrides are keyed by piece id, and ids like `extrados-panel-3-2` shift when
  the cell count changes. Either ids become stable, or orientation data is
  keyed by something that survives a resample.
- **Mirrored correctly.** The plan exports one half of the wing. When the
  builder cuts the mirrored half, the grain direction mirrors with the part; a
  naive copy gets it wrong on every panel of one side.
- **Independent of the plan layout.** The whole point is that the drawing
  position must stop being load-bearing information.

### Where the direction could come from, cheapest first

1. **Construction geometry (recommended).** Every panel is generated from a
   parametric surface; the u/v directions are known at the moment the flat
   pattern is computed and simply thrown away. Emitting the chordwise (or
   spanwise, per the design's convention) parameter direction as the grain
   vector is nearly free and is almost certainly what the designer means. Ribs
   are similar: the chord line is the natural axis. Straps and rod pockets:
   their long axis.
2. **Geometric heuristic.** Principal axis of the outline, or a straight
   skeleton, to give large parts an axis-aligned orientation. No engineering
   input, but better than the plan layout and needs no core changes — this is
   the fallback if (1) proves awkward.
3. **Stress-guided.** Take the principal stress direction per panel from the
   Playground's XPBD membrane solve and align the warp to it. The most
   defensible answer and the most work; also circular in a sense, since the
   simulation assumes a shape the fabric orientation helps determine. Worth
   revisiting once the port makes the panel generation accessible.

## Proposed GUI: draw the fibre direction

Independent of the core work, and useful even after it lands as an override:

- Show the source shapes on screen (the Print tab's preview already draws
  exactly these outlines).
- The user clicks inside a piece and drags a line. That vector is the fibre
  direction for that piece.
- Store per piece id in the design's Studio trailer, alongside the B-spline
  definitions already kept there.
- Feed it to the packer: the allowed rotations become those mapping the stored
  vector onto a machine axis. Because the weave is orthogonal, that is
  0/90/180/270 relative to the stored vector rather than relative to the drawn
  orientation — which is the whole fix.
- Offer "apply to all pieces in this category", since a whole category usually
  shares one convention.

Worth doing even with only a handful of pieces annotated: an unannotated piece
falls back to today's behaviour, so the feature degrades cleanly.

## Gotchas for whoever picks this up

- **Do not scale seam allowances.** Unrelated to grain but adjacent in the same
  code: allowances are a fixed 15 mm whatever the wing's size. See the Print
  tab's allowance mode.
- **`grainAngleDeg` currently reads 90 for every piece.** That is a placeholder,
  not a measurement. Do not "fix" a bug by trusting it.
- **The nester's `rotationStepDeg` is about bias, not about grain correctness.**
  Setting it to 90 prevents bias. It does not make the orientation right.
- **Rotation is recorded per placement** (`Placement::rotationDeg`), so once a
  real grain vector exists, validating an existing layout is straightforward:
  the mapped grain vector must land on an axis.
- **The printed part needs a grain arrow** once this is real. Currently nothing
  is drawn, because there is nothing trustworthy to draw.
