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
  grouping uses the engine's own line metadata where the mesh carries it,
  else clustering by attachment chord fraction.

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

- **Colour by** — the skin heatmap now has three sources: *Stress*
  (edge strain, as before), *Shape deviation* (per-node distance from the
  aligned rest shape, mm) and *Slack fabric* (compression strain — the
  wrinkle map). One combo box, one shared scale slider, legend follows.
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

## Limits, stated

Same boundary as always, now with a sharper edge: the polar is classical
lifting-line with heuristic constants, the pressure distribution is a
shaped guess, and the fabric is a mass-spring cloth, so **absolute**
forces and angles carry model error. What the instruments measure is the
**relative, structural** response of one design — where fabric goes
slack, which row unloads, at what α the nose dents — and comparisons of
those between design revisions. That is the claim, and all of it.
