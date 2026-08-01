# Playground shape analysis

The Playground began as a toy: inflate the wing, pull the brakes, watch it
move. The inflation and the force visualisation earned their keep; the
free-flight layer did not — a paraglider simulated at this fidelity flies
like a self-collapsing plastic bag, and watching it fall over tells a
designer nothing. What a designer actually wants from a live soft-body wing
is a different question entirely:

> **Does the wing hold the shape it was designed to have, under the loads
> it was designed to carry — and if not, where does it give first?**

That is a question this model *can* answer usefully, because it is a
relative one. The XPBD cloth is not certified aerodynamics, but the mesh it
is built from samples the design's exact ballooning law — the rest pose IS
the design shape — so every departure of the settled, loaded wing from its
rest pose is a structural signal: fabric going slack, a nose denting
inward, a section washing out, a line row shedding load. The Playground is
therefore now a **wind tunnel with instruments** rather than a flight game.

## The wind tunnel

Pinned mode is the tunnel: the carabiners are fixed to the world (the
tether), the airflow is set by the two sliders — dynamic pressure q (with
its equivalent airspeed shown) and angle of attack α — and the pressure
field shapes the fabric exactly as before.

New is the **Flight load** toggle: with it on, the wing-level polar force
pass is imposed in pinned mode too (spread over the skin as pressure,
resultant anchored on the hang line — see `applyAerodynamicForces`).
Without it, the tunnel wing carries only the pressure field's own
resultant, which under-reads lift badly (d'Alembert), so the lines see a
fraction of flight load and every line-load number is fiction. With it,
the canopy hangs in its lines against a realistic ~1.5 kN resultant,
which is the condition under which "does it hold its shape" is worth
asking.

### Why the tunnel load is open-loop

Free flight closes the aerodynamic loop: the wing's own measured, filtered
angle of attack drives CL and CD, and the whole calibrated stability stack
exists to keep that loop from diverging. The tunnel deliberately does
NOT. Its polar is evaluated at the **prescribed** angle — the rigged rest
angle, shifted degree-for-degree with the slider — making the load a dead
load along the current airflow. Every closed-loop tunnel variant tried
found a failure: the 0.25 s filter pumped a ~10 s surge cycle on the
tether's one-sided cables; a 3 s filter stopped the pump but lost the
static restoring race and let the wing slide nose-down onto taut A-lines
over fifteen quiet seconds; an instant anchor with slow magnitude slid
the same way. Open-loop, the bridle geometry alone is statically stable
(a nose-down excursion slackens the C rows and the still-taut A rows
restore it, and vice versa) — and a measurement instrument *wants* the
load prescribed: the wing's actual attitude under it is an output (the
HUD's α, the twist distribution), not an input.

Three supporting choices, all tunnel-only, all measured in: a **soft
start** ramps q over the first two seconds so the wing loads up at its
rigged attitude instead of getting snapped onto its cables (the hard
start could bounce it into a parked nose-down state — a real state, a
front-tucked wing, but not the one a measurement is about); a **brake
gap** of 20 cm, exactly as real wings rig one, because the loaded canopy
pitches into its tether cone and fixed-handle brake cables sized to the
rest pose otherwise go spuriously taut (80 N on one side at zero input,
and the asymmetric snatch wound the wing up); and **heavier damping**
(8/s absolute vs the sandbox's 3/s), because a tunnel mount is allowed to
be damped — the tunnel measures statics, and the dynamics it would
distort are free flight's job.

### Per-cell air

The interior side of the stamp used to be one blanket assumption — every
cell sits at its section's ram pressure, always. That assumption cannot
lose pressure, so it also cannot re-inflate: a folded region kept being
stamped exactly like a healthy one, the fold carried no strain energy,
and the field pinned it shut forever (measured: after a −6° excursion the
wing parked at −21% volume with one side's A row unloaded and never came
back). `SimCell` replaces the assumption with one gauge-pressure state
per bay between adjacent ribs, advanced once per frame in
`applyPressure` by three effects — all positional and all restoring, so
none re-opens the velocity loops the stability stack exists to avoid:

- **Intake**: the cell relaxes toward its section's ram target, gated by
  how much of its Vent-tagged skin still projects into the oncoming air
  (normalised so the designed pose counts as fully open). A tucked nose
  seals its own intake instead of being force-fed.
- **Cross-ports**: neighbouring cells exchange pressure through the rib
  hole area the design actually declares (`SimMesh::ribHoles`, exported
  from the Section 4 hole groups). This is the re-inflation path — and a
  design without holes honestly does not get it.
- **Squeeze**: below 90% of the rest section area the cell answers with
  extra pressure (up to 3× its state). Per-cell-uniform pressure times
  the live area vectors is the gradient of enclosed volume, so this term
  pushes toward re-opening even where a fold has inverted faces, which
  the chordwise-shaped part of the stamp cannot.

On a healthy wing the state converges to exactly the ram target and the
squeeze deadband adds nothing, so the stamped field — and with it the
whole `--shape` calibration — is unchanged (verified identical flags,
loads and settle time on gnuC2, and `SimControls::cellPressureModel =
false` reproduces the old stamp bit for bit; the bench takes
`--no-cells`). Measured A/B on gnuC2 with the bench's `--dive` and
`--tuck` experiments: moderate collapses (−4° excursion, grab yanks to
~3 m) now recover cleanly where recovery is expected; after a violent
−6° excursion the old model parks permanently deflated while the cell
model re-pressurises every section back to rest volume.

### What the wing meets, and what a brake does

Three corrections to the free-flight force model, all measured against
sessions where a wing folded and never came back.

**Intakes are fed by their own motion.** Each vent face's inflow is the
flux of the relative wind through that face — `Σ (v_air − v_fabric)·A⃗`,
normalised against the scoop the designed mouth makes, so the rest pose
counts as fully open. It replaces a gate that projected the mouth onto
one bulk wind direction for the whole wing: a wing that had pitched,
rolled or swung then read as sealed everywhere at once, and since it
could still empty, it never came back. Exhaust is gated on the mouth's
live *aperture* (`|Σ A⃗|` over its rest value) rather than its direction,
so a folded mouth cannot dump air it can no longer take back.

**Deformed fabric has drag.** Half the sum of `|A⃗·ŵ|` over a closed
surface is its frontal area along ŵ; the live skin's, minus 1.25× the
designed skin's at the same attitude, is the bluff-body area the
deformation created — zero on a wing holding its shape (a loaded canopy
balloons 7–20% over the drawing, hence the deadband), square metres once
it is a bag. Directed downwind, so it is pure dissipation. Without it a
folded canopy made almost no force at all: the risers carried 321 N of a
927 N system, the whole machine fell at two thirds of g, and in a fall
that steep the pilot has no apparent weight left to tension the lines
with — so nothing pulled the wing back into shape. Two bounds keep it
honest: the live frontal area is capped at the planform (a fold stacks
layers the area sum counts and the air does not — 9.1 m² on a 15 m²
wing), and the force at the impulse that would null the relative motion
within one frame, without which it pushed a collapsed wing *upward*.

**Each rib meets its own wind, and a brake is not a pitch input.** The
per-rib relative wind now includes the canopy's rigid-body spin (`I·ω =
L` over the canopy nodes about their centroid), which is where roll and
yaw damping come from — a rolling wing has one tip descending into the
air and the other rising out of it. The fit is rigid on purpose: a rigid
body has no breathing mode, so none of the fabric's own motion reaches
the pressure field through it. And the wing-level angle of attack is
measured from the leading edge to a **40%-chord extrados node**
(`RibChord::referenceNode`), rotated back onto the chord by a rest-pose
offset (`SimBody::attitudeOffsetRadians`) — that calibration is not
optional, since the reference node rides tens of degrees above the chord
on the aerofoil's own thickness. Measured off the full chord, a brake
pull rotated the LE→TE line and read as the whole wing pitching up; the
polar answered with more lift, more induced drag, less airspeed and
therefore a still higher angle. With a hand held still at 20 cm, α ran
20.9 → 23.1 → 29.4 → 76°. It now goes 13.2 → 15.5 → 16.4°.

The polar is evaluated **per half-span**, each side at its own brake. The
pull enters as an effective camber angle (8° at full travel) rather than
a bare lift increment, so the braked half also reaches the stall blend
first — a hard pull dropping its own side is then a consequence rather
than a rule. The wing-level pair stays the mean of the two, so a
symmetric pull is bit-for-bit what it was. Absolute tunnel loads moved
with the α reference (gnuC2: 1406 → 1278 N lift, L/D 7.79 → 7.66); settle
time, span, area, volume and flags did not.

**The turning moment from that split is NOT solved**, and two attempts
are recorded in the code so a third does not repeat them. Adding a roll
row to the force-distribution solve wrecked the *symmetric* glide
(airspeed 9 → 14.5 m/s, sink −1.3 → −3.2, before any brake was pulled):
forcing the increment's own roll moment to a prescribed value is not a
no-op, it rebuilds the whole increment field. Layering the couple on
after the solve is symmetric-safe but folded the wing at 4 s against a
9 s baseline (span 8.4 → 5.4 m in one second), because a spanwise-linear
pressure gradient loads the tips hardest — exactly where this fabric is
weakest. It sits behind `LEP_AERO_BRAKE_ROLL`, off by default.

Both attempts share one flaw: they bolt a couple onto a wing-level
resultant. A brake's moment has to arrive where the brake acts — the aft
fabric of its own half — which means running the whole force-and-
distribution pass per half-span, each with its own α, brake, anchor and
4×4 solve. That is the next attempt. Until then a one-sided pull still
departs after several seconds (8 s, against 9 s before the split).

### Fabric contact

Without contact, folded fabric passes freely through itself and through
the lines — which cuts both ways: impossible geometry, but also an
unphysical escape hatch that lets a tangle "un-knot" by ghosting. The
**Fabric contact** checkbox (Solver section; `SimControls::
fabricContact`, bench `--contact`) adds the Playground's own thin-cloth
pass: candidates found once per frame by a spatial-hash sweep (skin
nodes vs skin triangles and vs suspension-line segments, with rest-pose
pairs excluded so designed-adjacent fabric never fights), then plain
PBD position projections after every substep with an inelastic normal
velocity fix, a remembered approach side so a fast crossing is pushed
BACK through, and a 1 mm/substep correction cap so deep stacks resolve
gently. It is deliberately NOT the engine's certified contact pipeline,
which enumerates O(V·T + E²) feature pairs serially ninety times a
frame — five orders of magnitude over budget — and cannot be switched
off once registered.

Two calibration lessons, both measured on gnuC2: the separation must
stay at 1 mm — at 2 mm the wrinkle fields of a healthy loaded wing
(~23% slack fabric at sub-millimetre spacing) light up as thousands of
false contacts that stiffen the skin and snag collapse recovery — and
capture margins must be taken relative to the canopy-mean velocity,
or bulk motion makes the whole upper and lower skin candidates of each
other. Cost with the box ticked: ~12 ms/frame on gnuC2 at native
resolution (detection once per frame, thirty projection passes), about
32 fps against 52 without. Off is the identical old code path.

With contact on, a −4° excursion recovers to a clean wing (32 mm
residual asymmetry); a −6° one recovers to ~81% span with a genuine
tip cravat still working itself out — physically locked fabric now, not
geometry that ghosted apart. Line-line contact (riser twists) is not
modelled.

## The instruments

`playground_metrics.{h,cpp}` compares the live wing against a
`ShapeBaseline` captured at build time. Everything is derived from the
positions, constraints and accumulated constraint impulses of the live
body — no new physics, only measurement.

Per rib (each compared to its rest section after a rigid best-fit, so trim
rotation and translation do not count as error):

- **Section RMS / max deviation** — how far the section's nodes sit from
  the rest section shape. In-plane distortion and out-of-plane buckling
  both land here.
- **Twist change** — the section's pitch relative to the wing, live vs
  rest: the live washout distribution. This is the number that shows a
  brake pull or an overload actually twisting the wing.
- **Chord ratio** — chordwise stretch/compression of the section.
- **Leading-edge dent** — inward normal displacement of the nose nodes
  (chord fraction < 0.10). The front-tuck precursor.

Wing-level:

- **Span / area / volume ratios** vs rest (projected span, projected
  planform area, enclosed skin volume).
- **Slack-fabric fraction** — fraction of skin edges under compression.
  Fabric cannot push; a compressed edge is a wrinkle.
- **Asymmetry** — mirror error between left and right rib pairs under
  symmetric input. A symmetric wing developing asymmetry is the model
  telling you something is unstable.
- **Agitation** — RMS node velocity relative to the bulk: flutter and
  non-settling detection.

Lines:

- **Per-segment tension** read from the XPBD accumulated multiplier
  (force = λ/h², exact for the solved state), summed into **row loads**
  (A/B/C/D…, left/right separately), with slack-segment counts. Row
  grouping uses the engine's own per-line row plans; a mesh exported
  before those tags existed reports no row table at all — an empty table
  is honest, a mis-grouped one is not. Re-run the engine to regenerate
  the mesh with tags.

## The verdicts

A measurement pass emits **flags** when a metric crosses a heuristic
threshold — "anything weird" made explicit: `FrontTuckRisk`,
`ProfileDistortion`, `WashoutChange`, `SlackFabric`, `SpanLoss`,
`UnderInflated`, `Asymmetry`, `SlackRow`, `Unsettled`. Thresholds are
constants in `playground_metrics.h`, documented as heuristics; they are
tripwires for a designer's attention, not pass/fail engineering criteria.

They were calibrated by sweeping three dissimilar wings (gnuC2, gnuA7,
Swoop) and placing each threshold above the healthy working band and
below the collapse states, with every dimensional threshold expressed
relative to the wing's own scale — chord fractions, span fractions,
airspeed fractions — so the calibration carries across sizes. Two
model truths the thresholds encode: a healthy loaded wing stands at
~25% slack skin edges and ~2-5% of chord in nose travel (the unloaded
stagnation-region fabric relaxing inward from its designed ballooning —
the reason real wings grew nose rods), so those levels are the baseline,
not a warning; and the stubby tip/stabilo sections are floppy while
perfectly healthy, so the per-rib flags judge them against no less than
half the mean chord. "Settled" likewise means *the measurement has
converged* — agitation and resultant stationary — not "the fabric is
motionless": wings differ in how loudly they flutter at their standing
state (a wing whose C row hangs slack breathes at several percent of
airspeed forever).

## Ways of looking

- **Colour by** — the skin heatmap has four sources: *Stress* (edge
  strain), *Shape deviation* (per-node distance from the aligned rest
  shape, mm), *Slack fabric* (compression strain — the wrinkle map)
  and *Pressure* (the pressure difference across each cell face, Pa —
  per FACE and unsmoothed on purpose, because the cell-by-cell
  structure of the inflation load is the thing being examined; each
  cell carries its own internal gauge pressure state — fed through its
  leading-edge intake while that intake faces the airflow, exchanged
  with its neighbours through the rib cross-port holes the design
  declares, and boosted when the section is squeezed below its rest
  area — so a tucked cell visibly loses its ram feed and a collapsed
  side is re-fed by the inflated one; see "Per-cell air" below). One
  combo box, one shared scale slider. All three tint per
  VERTEX from per-node fields (edge strains scattered to their
  endpoints), so the skin shades smoothly instead of rendering as
  facets. A calibrated legend paints INSIDE the viewport (top right)
  whenever a colouring is active: the exact ramp the skin is tinted
  with, the quantity and unit, numeric ticks, and a live peak marker —
  a peak past full scale parks at the top of the bar with its true
  value printed, so a saturated display reads as saturated rather than
  lying. Line-tension colouring gets its own bar below. (The overlay
  needs a stencil buffer; the app's shared GL format requests one.)
- **The page layout** puts every control in a panel on the LEFT and
  gives the viewport the full window height — the wing on screen is
  roughly 1:1 while a typical window is 2:1, so chrome above the
  picture was the wrong place for it. Under the viewport sit the same
  navigation buttons as the Design tab's 3D view (Fit, Iso, Front,
  Back, Left, Right, Top, Bottom); a single status line runs across
  the bottom.
- **Settle** — steps the live wing at the Accurate solver setting (60
  substeps × 4 iterations) as fast as the machine allows, unpaced by
  the 16 ms frame clock, until the measurement converges — IN the
  view, so the convergence is watched happening under whatever heatmap
  is active, with the status line counting simulated seconds and the
  agitation falling toward its quiet target. On convergence it pauses
  for review. The live view is a compromise between frame rate and
  accuracy; the settled pose is what the wing's numbers should be
  quoted from.
- **Live shape HUD** — one line under the solver readout: span %, volume
  %, worst section deviation and where, slack %, LE dent, row loads. On
  while the tunnel runs, so slider changes answer in real time.
- **α sweep** — the Analyse button runs the tunnel across an
  angle-of-attack range on a worker thread (fresh body per point, settle
  to quiescence, then measure), and opens a report: metric-vs-α plots,
  the full table, every flag with the α it first appeared at, CSV export.
  This is the "polar" a shape designer wants: not just L/D vs α, but
  *shape integrity* vs α.
- **Grab tool** — click any line junction in the tunnel and pull. A
  kinematic anchor with a cable follows the cursor; the HUD reports pull
  distance and force. Pull an A-branch and watch the front tuck develop,
  with every instrument live. This is deliberate sabotage as a design
  probe: how hard is it to fold, and how does it recover?

## Headless

`softwing-bench --shape [seconds]` settles the tunnel at the current
controls and prints a full report; `--shape-sweep from:to:step` emits the
sweep as CSV. The GUI and the bench share `settleAndMeasure()`, so a
number in a report can always be reproduced without a GUI.

## The turning couple, and where it had to go

A one-sided brake used to produce no turning moment at all. The polar was
evaluated per half at each half's own brake (commit `02edc34`), but the
two halves' coefficients were then averaged into a single wing-level
pair, so the difference between them never reached the wing.

Three ways of getting it there were tried, and the first three failed:

1. **A fifth row in the force-distribution solve** — a spanwise pressure
   gradient `ν` constrained to produce the roll moment. It wrecked the
   *symmetric* glide: airspeed 9 → 14.5 m/s, sink −1.3 → −3.2, with zero
   brake. Constraining the increment's own roll moment is not a no-op; the
   extra row rebuilds the entire increment field.
2. **The same gradient layered on after the solve.** Symmetric-safe, but
   it folded the wing at 4 s against a 9 s baseline (span 8.4 → 5.4 m in
   one second). A spanwise-linear pressure gradient loads the tips
   hardest, and the tips are where this fabric is weakest.
3. **The whole force pass run per half-span** — each half cancelling its
   own pressure resultant, on its own anchor, in its own 4×4. This is the
   obvious generalisation and it takes the arc's lateral bracing out of
   the fabric: an arced canopy's two halves lean on each other hard, each
   half's pressure resultant carries a large spanwise component, and the
   pair cancels. Asking each half's retrim to cancel its own cost the
   tunnel wing 8% of its span and 18% of its volume, and it never settled.
   Sharing the spanwise row out by area instead fixed that but left a
   twist flag at a tip: giving each half its own `v` still puts a lateral
   body force on each half that the single solve never had.
4. **What is in the tree.** The shared pass is left exactly as it was —
   the wing-level resultant, the wing anchor, one solve over all the skin
   — and a *differential* pass adds half the coefficient difference to one
   half and takes it off the other, through the same machinery, about each
   half's own anchor on its own mean chord. The two anchors are offset
   spanwise, so the couple has the wing's real lever arm and costs nothing
   in pitch (a spanwise arm crossed with any force has no span-axis
   component). With equal brakes and equal angles the difference is
   *exactly* the zero vector and the pass does not run, so the tunnel
   calibration is bit-for-bit unchanged — verified by CSV comparison, not
   by inspection.

### Roll and yaw damping, which had to come first

Before any of that would read, the wing had to stop turning on its own. A
hands-up symmetric launch drifted 47° of heading in ten seconds and folded
at 17–18 s: the wing-level polar gave a rolling or yawing wing no
restoring force whatever, because both halves saw one angle and one
dynamic pressure.

Both now come from the canopy's **rigid-body spin** (`canopySpinOf`), the
same fit the per-rib pressure wind uses and for the same reason — a rigid
fit has no breathing mode, so no fabric motion reaches a kilonewton-scale
force through it. Each half gets the wind at its own quarter-chord
station, which yields a low-passed angle departure and a dynamic-pressure
ratio; both are 0 and 1 on a wing that is not rotating.

Measuring each half's own **chord line** instead was tried first and is
wrong twice over: a rigid roll does not move the chords relative to each
other, so it damped nothing, and what it did measure — differential twist
— is positive feedback, since a half that has twisted nose-up is handed
more lift and twists further. It took the glide's sideslip from 5 to 9 m/s
and brought the departure forward from 17 s to 13.

With the kinematic form, the symmetric glide's heading drift over the
first ten seconds falls from 47° to 11°, sideslip stays inside ±1 m/s to
12 s, and bank holds inside ±5° to 12 s against ±8° and growing.

### What the brake does now, and what it still does not

A one-sided pull produces a real, mirror-symmetric turn **toward the
braked side**: 8 cm ramped over 6 s gives −11 °/s for the left brake and
+7 to +27 °/s for the right, against +3 to +11 °/s hands-up.

The **bank is inverted** — the wing skids, banked out of the turn it is
yawing into. This is honest to the brake model rather than a bug in the
couple: `polarFor` adds the brake as an effective camber angle, so the
braked half always makes *more* lift at the wing's operating angle (12° +
8° of camber at full pull is still short of the 20° stall knee), and a
real wing banks into the turn because the braked half **slows**. The
speed loop is present — brake drag yaws the wing, the braked half becomes
the retreating one, its dynamic pressure falls — but at these turn rates
it is worth about the same as the camber's lift increment and arrives a
second later, so the camber wins. Closing that means re-calibrating
`kBrakeCamberRadians` against a flap-deflected stall angle, which needs
data this project does not have.

Adding the flap's centre-of-pressure travel (feeding the camber into each
half's anchor fraction, which is real aerodynamics) halves the wrong-way
bank without flipping its sign, and brings the departure under a one-sided
pull forward by 1.5 s. It is not in the tree for that reason.

## How a cell loses its air

`advanceCellPressures` relaxes each cell toward its ribs' ram pressure,
and `ribPressure` is ½ρv² from the rib's own relative wind **whether that
rib is flying or buried inside a fold**. A collapsed cell was therefore
force-fed exactly as hard as a healthy one, both sat at target, and there
was no gradient across the rib holes for the cross-ports to work on — so
the Neighbour-reinflation slider was inert by construction. Ten times a
zero gradient is still zero. Measured through the standing `--dive -6`
case on gnuC2, the cell states went *up*, 66 → 74–80 Pa, while the wing
folded.

A folded bay is now **vented toward ambient**, at a rate rising with how
far it has collapsed. Three details are load-bearing:

- It is a **leak, not a reduced target.** The intake keeps its own ram
  target and its own rate, so a folded bay whose mouth still meets the
  airflow rams itself back open exactly as before. Reducing the target —
  the obvious reading — would take that away and make a collapse
  permanent by construction, which is the failure the cell model exists
  to escape.
- The signal is **bay volume, not section area.** Section area is the
  natural guess and it never fires: through `--dive -6` the wing lost 31%
  of its enclosed volume and half its span with no rib loop ever below
  67% of its own rest section. This canopy does not collapse by
  flattening its sections, it collapses by concertinaing its ribs
  together. Bays go to 5–9% of their rest *volume* there, against 97–98%
  on a healthy settled wing on both reference meshes.
- It is **deadbanded** on that separation, so a wing holding its shape
  sees exactly nothing.

`--dive -6` on gnuC2 then ends at −1.4% of its settled volume and 94% of
its rest span, against −31% and 67% before; mirror error falls from
2165 mm to 183 mm, the `UnderInflated` flag goes away, and the cell field
carries a real 53–77 Pa gradient where it used to be a flat 74–80. The
`--tuck 250` guard on the Swoop at 60×4 is untouched — no bay there ever
crosses the threshold.

## Limits, stated

Same boundary as always, now with a sharper edge: the polar is classical
lifting-line with heuristic constants, the pressure distribution is a
shaped guess, and the fabric is a mass-spring cloth, so **absolute**
forces and angles carry model error. What the instruments measure is the
**relative, structural** response of one design — where fabric goes
slack, which row unloads, at what α the nose dents — and comparisons of
those between design revisions. That is the claim, and all of it.
