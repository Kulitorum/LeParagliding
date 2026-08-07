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
back). `SimCell` now stores finite air mass per bay. Its raw gauge pressure is
derived from `p = mRT/V - p_atmosphere`, where `V` is the live closed-skin
volume measured from that bay's skin triangles plus virtual rib caps.

- **Moving intake**: each Vent-tagged mouth is a bidirectional orifice to a
  reservoir whose pressure comes from the mouth's own motion through the
  air. Its vector-area aperture distinguishes an open mouth from a pinched
  one. Moving boundaries also sweep ambient-density air through an open
  mouth, which is the low-Mach control-volume term needed to avoid treating
  ordinary ballooning as expansion of a sealed pressure vessel.
- **Cross-ports**: neighbouring cells exchange equal-and-opposite air mass
  through the actual rib-hole area (`SimMesh::ribHoles`). Internal transfer
  is conservative and cannot overdraw a donor or cross pressure equilibrium
  within a pneumatic step.
- **Volume response**: a sealed squeeze raises pressure naturally; a sealed
  expansion lowers it. There is no synthetic squeeze boost and no
  visual-collapse leak that deletes pressure. An open or leaky cell loses
  pressure only through an explicit flow path.

Pressure and volume are refreshed between XPBD substeps, because gas
stiffness applied one frame late is unstable. `cellPressureModel = false`
still selects the old blanket ram-pressure stamp for comparison.

The raw gas state is deliberately distinct from the pressure applied to the
cloth. In a healthy bay the resolved load retains the calibrated ram field as
a prior, avoiding a stiff gas/cloth feedback driven by ordinary fabric
breathing. As true volume, mouth opening or ram recovery is lost, authority
shifts continuously to the finite-mass state. The raw value remains available
in `cellRawPressure` for diagnostics; the `Cell resolved p` heatmap shows the
pressure that actually loads the skin.

### Air state: the wing moves through the atmosphere

`SimControls::ambientAirVelocityWorld` is the surrounding air's velocity in
world coordinates. The single sign convention everywhere is
`v_relative = v_air − v_surface`: ribs, vent faces, the wing polar, pilot drag,
pressure summaries, air-mote travel and flight telemetry all derive from that
state. In the tunnel, the q-derived reference flow at the selected angle of
attack is added to ambient. In free flight, ambient is the entire atmosphere;
the pressure slider remains only a reference dynamic pressure/load cap and
trimmed-launch speed. It is not injected again as a moving air mass.

The build still calibrates and pre-inflates the rest shape at the reference
apparent flow. TrimmedGlide then gives all components the common ground
velocity `ambient − rotate(referenceFlow, span, glideAngle)`; DropFromRest
resets ground velocity to zero while retaining that pre-inflation. Shifting
ambient and every node velocity by the same vector leaves sample, pressure,
cell flow, force and a solver step invariant in the focused regression.

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

The finite-wing polar's missing viscous drag has a separate bounded-free-flight
path. Only `max(0, q*S*Cd_polar - D_pressure)` is spread by current skin area
along the relative wind; it supplies no lift, is zero at q=0, and its measured
air-relative power is non-positive. The excess-frontal/fabric heuristic above
is deliberately excluded from that traction. Reinjecting its residual turned
ordinary 2-5 m2 breathing estimates into hundreds of newtons of new shear and
a positive deformation/drag feedback loop. Pinned shape analysis remains
pressure-only, and the legacy oracle receives no traction.

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

Two failed historical turning-moment attempts are recorded so they are not
repeated. Adding a roll
row to the force-distribution solve wrecked the *symmetric* glide
(airspeed 9 → 14.5 m/s, sink −1.3 → −3.2, before any brake was pulled):
forcing the increment's own roll moment to a prescribed value is not a
no-op, it rebuilds the whole increment field. Layering the couple on
after the solve is symmetric-safe but folded the wing at 4 s against a
9 s baseline (span 8.4 → 5.4 m in one second), because a spanwise-linear
pressure gradient loads the tips hardest — exactly where this fabric is
weakest. It sat behind the removed `LEP_AERO_BRAKE_ROLL` experiment.

The production resolution is the bounded final-Cp hierarchy documented
below: one shared global field first, followed by a lower-priority L-R
lift/drag target. It preserves the arc's common bracing and reports the
differential authority instead of bolting an unconstrained couple on later.

### Fabric contact

Without contact, folded fabric passes freely through itself and through
the lines — which cuts both ways: impossible geometry, but also an
unphysical escape hatch that lets a tangle "un-knot" by ghosting. The
**Fabric contact** checkbox (Solver section; `SimControls::
fabricContact`, bench `--contact`) adds the Playground's own thin-cloth
pass in the Qt-free `playground_contact` module. It covers skin
vertex/triangle, skin edge/edge, and authored suspension
segment/triangle features. Harness ties and generated brake-control
cables are not authored suspension and are not contact colliders.
Line/line contact and friction remain unsupported.

Exclusions now come only from topology: incident and one-ring skin
features, plus triangles adjacent to an authored line attachment. The
old rule that permanently excluded every pair close in the rest pose
was wrong — two unrelated panels designed 0.5 mm apart must still
collide. Candidates are deterministic and persist while their swept
substep envelope remains valid. The envelope covers 1.5 times the
velocity-predicted substep travel plus a mesh-scaled 3--10 mm allowance;
if XPBD motion escapes it, the retained pre-substep candidates are
projected first and a narrow envelope is rebuilt for the next substep.
That preserves the approach side through a crossing without inflating
every candidate over a full violent frame.

The broad phase is a complete 3-D grid for ordinary features. A feature
covering more than 64 cells goes on an explicit oversized list and is
tested against the full compatible set; dense cells use local
sweep-and-prune rather than being skipped. Work and candidate budgets
remain hard bounds. Hitting either sets `coverageComplete=false` and is
reported by the bench; partial coverage is never presented as complete.
Projection is mass-weighted across every feature node, retains the
captured separating side, and removes only closing normal velocity.
Same-side deep overlaps retain the gentle 1 mm/substep cap, while a
signed crossing is returned fully to its captured side in one pass.
It is deliberately NOT the generic registered SoftBody contact pipeline,
whose exhaustive per-iteration enumeration is unsuitable here.

The fabric separation remains 1 mm — at 2 mm the wrinkle fields of a healthy loaded wing
(~23% slack fabric at sub-millimetre spacing) light up as thousands of
false contacts that stiffen the skin and snag collapse recovery — and
capture margins must be taken relative to the canopy-mean velocity,
or bulk motion makes the whole upper and lower skin candidates of each
other. The complete feature set is still expensive: a bounded gnuC2
native-resolution startup frame at 30x2 measured 550.6 ms with contact
versus 24.3 ms with contact off, with 1,474 vertex/triangle,
3,968 edge/edge and 573 line/triangle candidates, eight envelope
refreshes and complete coverage. That cost is why contact remains opt-in.
Off never enters the module and preserves the pinned tunnel path exactly.

The earlier collapse-recovery claims were measured with the incomplete
vertex-only pass and do not carry forward as validation of this feature
set. The current guarantees are the deterministic feature and integration
tests plus honest real-mesh coverage diagnostics, not a calibrated cravat
or riser-twist model.

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
  (force = λ/h², exact for the solved state). Exact/reversed legacy JSON
  duplicates are discarded before constraints are built, keyed by quantized
  endpoints plus authored plan/brake semantics. **Riser and row loads** are
  vector reactions across the authored suspension cut immediately above the
  carabiners; pilot harness and synthesized brake-control drawing segments are
  excluded so the same load path is not counted twice. Row loads remain split
  A/B/C/D… and left/right, with slack-segment counts. Row
  grouping uses the engine's own per-line row plans; a mesh exported
  before those tags existed reports no row table at all — an empty table
  is honest, a mis-grouped one is not. Re-run the engine to regenerate
  the mesh with tags.

### Free-flight payload and suspension mass

The experimental free-flight mode takes pilot plus harness/instruments as an
explicit mass (90 kg default); it no longer solves backward from its own polar
to choose the payload that makes the current pressure setting balance. The
payload is presently one dynamic point mass. Its bilateral XPBD ties to the
carabiners make a real translational gravity pendulum, but it has no body
orientation, rotational inertia, weight shift or separate harness geometry;
those claims wait for the generic rigid-payload suspension migration.

Until line material metadata carries measured mass per metre, authored lines
use a documented 1 g/m fallback. Half of each unique segment's mass is lumped
to each welded endpoint. A 0.1 g positive numerical floor per junction and per
generated brake-control node is reported separately, so solver conditioning is
not mistaken for physical line mass. Harness ties have no added mass because
the explicit payload includes the harness, and synthesized brake-control
cables do not duplicate the authored brake cascade's physical mass.

The pinned tunnel deliberately retains 50 g per authored junction as
**nonphysical static-relaxation ballast**, reported separately in the session
log and bench output. Tunnel gravity is zero, so the inertial conditioning does
not change its static weight or equilibrium; it only lets the light branched
cable graph settle at the standard 30x2 budget. Free flight never receives
this ballast: it uses physical length-lumped line mass plus the 0.1 g floor,
and `simulatedMassKilograms()` therefore remains the physical flying mass.

Two launch conditions are intentionally distinct. **Trimmed glide** retains
the estimated flight-path direction, but sizes speed/q from the bounded field's
achieved vertical wing force and total dynamic-node weight. It runs eight
bounded-only co-moving quasi-static gravity/aero/cable frames, resets every
wake/control/cell/Cp transient state, and recalibrates q on the loaded geometry
before release. Three cable reverse/forward sweep pairs propagate the payload
reaction through the deep suspension graph; zero remains the exact historical
and tunnel path. **Drop from rest** performs none of that and releases the
pre-inflated system with zero initial velocity. It is a transient/pendulum
experiment, not a ground inflation or trim solution.

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

## Bounded final exterior-Cp projection

The production retrim does not solve an unbounded pressure increment and then
clip its answer. `applyPressure` records each face's interior pressure, local
dynamic pressure and stamped exterior-Cp prior. The Qt-free
`playground_pressure` solver chooses the final Cp directly inside `[-3, 1]`,
then the face load is reconstructed exactly as `p_inside - q*Cp`. Cp=1 is the
stagnation ceiling; q≈0 faces are fixed because Cp cannot change their load.

The objective prior is the same-frame field produced by the calibrated legacy
load path (global force/pitch 4x4, then two half-wing 4x4 passes), but each
intermediate pressure proposal is clamped through that face's physical Cp
interval `[p_inside-q, p_inside+3q]`. This matters during the pre-inflated soft
start, where applying the old absolute pressure clamp directly can present
`std::clamp` with an inverted interval. The objective is an area integral with
equal Cp mobility per unit area. Each active face is additionally restricted
to +/-0.05 Cp around that preferred topology, intersected with the global
`[-3,1]` envelope. This local trust region prevents an exact global equality
from moving load to a structurally different chordwise location.

Stages are global force (three rows), pitch about the live anchor, then L-R
lift and drag together. A later stage freezes earlier achieved equalities and
gets one reported authority in [0,1]; saturation is therefore a physical
limit rather than a hidden post-clamp residual. Exact pure/offline solves find
the maximum on the requested target ray by bisection. Production instead
caches the last verified authority and dual multipliers: it reprojects the
cached value on the live geometry, halves it until feasible if necessary, and
tries one +0.02 probe. Force starts at zero; pitch and differential start at
one so their normally feasible full-target fast path is not delayed. Every
accepted probe is still an exact bounded projection, and the zero-authority
baseline is the current hierarchical value itself. The L-R target starts from
the prior field and adds the polar's side request with the explicit factor two
required by +D/-D half loading. This retains the arced canopy's common lateral
bracing.

The semismooth dual Newton solve normalises rows internally but reports force
and moment residuals in N and N.m. Rank, condition estimate, authority hint,
probe/backoff state, active bounds, Cp range, iteration cost and numerical
failure are carried through the body, metrics, HUD/session log and
`softwing-bench`. Production telemetry
uses the achieved bounded pressure force; requested polar lift/drag remain
separate diagnostics. `softwing-bench --pressure-acceptance` compares fresh
bounded and legacy bodies and gates finite settling, flags, Cp bounds,
leading-edge dent and washout. The old 4x4 increment and clamp is available
only as `--legacy-pressure` for the bit-comparable regression oracle.

Partial force authority also changes how to read pinned L/D: it is the ratio of
the pressure force that survived the bounds, not a claim that the requested
finite-wing polar was achieved. Free flight adds only the separately diagnosed
polar-drag deficit above. With the deep-cable passes, achieved-load launch,
eight-frame relaxation and corrected polar-only traction, the capped gnuC2
five-second probe stays flying: alpha <=16.39 degrees, L/D >=3.09, span >=9.62
m and volume >=-2.6%. At 5 s alpha is 14.77 degrees, sink -1.64 m/s, span 9.95
m and volume -1.3%. The slow load/phugoid oscillation and 97/190 slack lines at
that sample preserve the experimental claim boundary; this is not a certified
or converged absolute glide model.

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
4. **What is in the tree.** One bounded final-Cp field serves the complete
   wing. Global force and pitch are frozen before the two L-R lift/drag rows
   are introduced with a common authority. Their target is the prior L-R
   field plus the polar's +D/-D request, so the common solve retains the arc's
   lateral bracing and does not independently cancel each half. Equal brakes
   and equal incidence request no new differential. The former shared 4x4
   plus two half 4x4 passes remain only under `--legacy-pressure` for the
   historical bit-comparable row.

### Roll and yaw damping, which had to come first

Before any of that would read, the wing had to stop turning on its own. A
hands-up symmetric launch drifted 47° of heading in ten seconds and folded
at 17–18 s: the wing-level polar gave a rolling or yawing wing no
restoring force whatever, because both halves saw one angle and one
dynamic pressure.

Both now use the canopy's **rigid-body spin** (`canopySpinOf`), the same fit
the per-rib pressure flow uses and for the same reason — a rigid fit has no
breathing mode, so no fabric motion reaches a kilonewton-scale force through
it. Each half gets the flow at its own quarter-chord station. In addition, each
rest rib section normal is mapped into the live rigid span/chord/up frame, and
the half flow is projected into that section plane. Sideslip therefore meets
the mirrored arc at opposite incidences and creates a weathercock moment
through the existing differential pressure pass. Per-rib no-beta incidence
and the exact planform-weighted common mode are removed, so a non-rotating,
no-sideslip wing remains 0/1 and common trim is unchanged even for asymmetric
section geometry.

The helper tests verify mirrored-beta incidence and a near-odd restoring yaw
pressure moment for both signs, a flat arc's zero response, no-beta invariance,
and the existing ±10° clamp under an extreme beta input. This is directional
authority, not proof by itself. After the separate cable-load, launch and
polar-only-drag repairs, the capped five-second gnuC2 probe stays within alpha
16.39 degrees, span above 9.62 m and volume above -2.6%; its remaining slow
load oscillation preserves the experimental claim boundary.

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

A one-sided pull produces a coordinated turn **toward the braked side**. On
Swoop Original, a sustained 30 cm right pull reaches +362 degrees accumulated
air-relative course at 30 simulated seconds while holding 8.61 m material
span, -0.6% volume, 7.5 degrees alpha and 915 N achieved vertical pressure
support. In the established turn nose leads course by about 5-7 degrees,
bank is +9-11 degrees and course rate is about +14 degrees/s.

The old opposite conclusion came from mixing mesh and physical directions.
The mesh chord and relative-air vector point downstream; physical forward and
travel point upstream. The old bank metric was also reversed: with span +X and
downstream chord +Y, lowering the +X tip tilts lift toward +X and is the
positive coordinated bank. `FlightFrameSample` now owns these conversions and
reports nose heading, course, beta and bank separately.

Two live-frame bugs made the failure physical rather than merely diagnostic.
The per-rib pressure pass projected against a rest/world section normal and
measured incidence from the brake-moved full chord. It now carries every rest
section plane into the live wing frame and measures the brake-immune
LE-to-40%-extrados attitude line, rotated by a per-rib rest offset back onto
the true chord. A rigidly yawed wing therefore retains the same section
incidences, and moving only a trailing edge cannot impersonate rigid pitch.
The live wing span also keeps the low-tip-to-high-tip material sign instead of
being flipped toward fixed world X; that old hemisphere test inverted lift as
a valid turn crossed 90 degrees and made a 360 impossible.

In free flight, the cable/fabric pressure field is the sole brake aerodynamic
input. The removed free-flight `polarFor` camber and explicit brake drag
counted the same live flap a second time: in the reproduced 30 cm failure the
nose reached +29 degrees while course was only +5 degrees, creating 23 degrees
of crossflow before the four-second fold. The half polar still supplies
rigid-spin and sideslip damping, but free flight gives it no independent brake
coefficient. The pinned/tunnel path retains its prescribed brake polar for
compatibility.

A sustained 40 cm pull still enters a deep asymmetric departure on this
reduced-order Swoop at about five seconds. That is an explicit remaining
deep-control/material-model boundary, not a reason to rotate the atmosphere,
silently cap the input, or add a force shortcut. The demonstrated regression
is the reported 30 cm case through a complete circle.

## How a cell loses its air

Ordinary air flow uses explicit openings. The moving intake is
bidirectional: a mouth with positive ram recovery fills the cell, while an
open mouth with lower reservoir pressure lets an over-pressured cell exhaust.
A pinched mouth is closed in both directions. Cross-ports transfer mass to a
neighbour but cannot change the canopy's total finite air mass.

The reduced-order model cannot resolve mouth-lip flutter, acoustic waves or
seam/fabric porosity. A cell with a live open intake therefore also has a
documented out-of-envelope pressure-relief path: it exchanges mass with the
atmosphere only when a geometric impulse would otherwise trap pressure outside
the low-pressure ram-air envelope, with its authority scaled continuously by
the live opening. A pinched mouth, or a test cell with no atmosphere boundary,
is genuinely sealed and follows the gas law without that guard.

A visually folded bay is not assigned an arbitrary pressure loss. If its
mouth is sealed and it is squeezed, pressure rises. If the fold leaves an
opening but destroys ram recovery, it exhausts toward ambient. If later data
justify fabric or seam porosity, that must be represented as a calibrated
atmosphere port rather than another collapse threshold.

The old rib-area × leading-edge-spacing proxy remains only as a malformed-
topology fallback. Normal operation measures each bay from its actual skin,
so a mid-cell cave changes volume even when both bounding rib loops stay
unchanged. The bench reports section ratio, closed volume, pressure/ram and
mouth opening together; a constant pressure is therefore distinguishable
from missing geometry coupling.

## Selectable fabric formulation

The calibrated/default skin remains the legacy bilateral distance truss. The
prototype selector replaces only those skin stretch/shear springs with
orthotropic three-component membrane elements; the ribs, straps, seams,
suspension, pressure/cell model and opt-in contact path remain the same. No
parallel truss is left under the membrane skin, so comparisons do not silently
double its stiffness.

Each ordered source quad defines one material direction field: `q0 -> q3` is
chord/warp and `q0 -> q1` is span/weft. Both triangles receive a winding-safe,
isometric tangent chart transported from that common direction, leaving the
designed pose at exactly zero Green strain even on a mildly non-planar quad.
Degenerate charts are skipped and counted. Interior manifold edges receive a
signed four-node dihedral XPBD hinge; boundaries are free, while degenerate,
non-manifold or inconsistently wound incidences are skipped and diagnosed.

The prototype defaults are warp/weft/coupling/shear
`8000/5000/1000/1500 N/m`, compression ratio `0.05`, zero material damping and
dihedral compliance `5e-4`. Compression uses an SPD-preserving `D*K*D`
stiffness reduction. A ratio of exactly one takes the original bilateral
matrix path byte-for-byte. Shear keeps full stiffness in tension, is partially
softened when one material direction compresses, and receives the full
retained ratio when both do (the shear scale is the geometric mean of the two
normal scales).

Slack and strain heatmaps use the membrane's warp/weft Green strains in this
mode instead of reconstructing edge strain. Material values are prototype
controls, not measured cloth certification. Nonzero constraint-space membrane
damping is additionally diagnosed as experimental: it is not yet accepted in
the full mixed membrane/rib/line network, so the prototype default is off.

At native gnuC2 resolution the mode builds 15,840 membrane triangles and
23,727 hinges, retaining 6,949 non-skin distance/cable constraints. A bounded
80 Pa hard-load probe for ten frames at 30x2 remained finite (volume +4.7%,
span +1.7%), but ran at about 267 ms/frame: roughly 135 ms in membrane solves
and 98 ms in the deliberately serial hinges. That is a correctness probe, not
a settled/calibrated shape claim or an interactive-performance claim.

## Limits, stated

Same boundary as always, now with a sharper edge: the polar is classical
lifting-line with heuristic constants, the pressure distribution is a
shaped guess, and neither the calibrated distance truss nor the experimental
orthotropic material is measured coupon data, so **absolute**
forces and angles carry model error. What the instruments measure is the
**relative, structural** response of one design — where fabric goes
slack, which row unloads, at what α the nose dents — and comparisons of
those between design revisions. That is the claim, and all of it.
