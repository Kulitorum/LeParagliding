# Playground free flight — handover

The P0-P5 Playground repair pass is implemented and measured. The former
pitch-retrim defect is no longer open: production now solves the final
exterior-Cp field inside explicit physical bounds and reports any missing
authority instead of promising an equality and clipping it afterwards.

---

## Orientation

The Playground is a soft-body paraglider sandbox: XPBD cloth + cable
constraints, a stamped pressure field, and a wing-level aerodynamic
polar. It lives in:

| | |
|---|---|
| `src/gui/playground_sim.{h,cpp}` | the wing: mesh → body, pressure, polar, step |
| `src/gui/playground_pressure_solve.{h,cpp}` | Qt-free bounded final-Cp hierarchy |
| `src/gui/playground_metrics.{h,cpp}` | the instruments (shape report, collapse diagnostics) |
| `src/gui/playground_page.{h,cpp}` | the tab: GL view, controls, session log |
| `tools/softwing_bench.cpp` | headless driver — same body the GUI builds |
| `tests/playground_cells_test.cpp` | unit tests for the per-cell air model |
| `docs/playground-shape-analysis.md` | the design record; keep it current |

Build and test (cmake/ctest are **not** on PATH):

```powershell
$env:PATH = "C:\Qt\Tools\CMake_64\bin;$env:PATH"
cmake --build build --config Release --target softwing-bench playground-pressure-solve-test playground-cells-test LEparagliding
ctest --preset release          # 21 tests, all must pass
```

Meshes for benching exist at `build/aero/{gnuC2,Swoop,gnuLAB4,gnuA1}/lep-sim.json`.
Regenerate with `leparagliding-engine --preview
resources/presets/<name>/leparagliding.txt <outdir>`.

**A running LEparagliding.exe locks the link target.** Do not kill an
instance you did not start — rename the exe aside and relink, or ask.

### The guards that must not move

```powershell
# 1. Tunnel calibration. Bit-for-bit: compare the CSV row, not the prose.
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --shape --csv --legacy-pressure
# expect 1280.0 N lift, L/D 7.66, no flags,
# line reaction 1146.8 N; rows A 356.1/354.5, B 206.1/205.9,
# C 53.3/54.0 (unique physical lines, vector reaction at the riser cut)

# 2. Production bounded-Cp shape versus the legacy calibration.
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --pressure-acceptance
# expect PASS: finite/settled/no flags, Cp within [-3,1], LE dent no more
# than legacy +10 mm, and |washout| no more than legacy +0.5 degrees

# 3. Corrected symmetric free-glide guard, first 5 s.
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --glide 300
# expect a finite run with launch support 940 N against 938 N weight;
# alpha <= 16.39 deg, L/D >= 3.09, span >= 9.62 m, volume >= -2.6%.
# This bounds the former collapse but is not a converged-trim acceptance test.

# 4. Collapse recovery.
.\build\bin\Release\softwing-bench.exe build\aero\Swoop\lep-sim.json --tuck 250 --substeps 60 --iterations 4
# expect "recovered: no flags", and 0 bays ever vented
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --dive -6
```

**`--dive` is CHAOTIC — never read one run of it as a regression.** Swept
across -5/-6/-7 the outcome is not even monotonic in the disturbance: the
milder -5 ends with 5.52 m of span against -6's 6.89 m. A reordering that
changed no physics at all has also moved it substantially. If you need a
verdict from it, sweep the angle and compare distributions, or use guard 1,
which is deterministic and bit-comparable.

Guard 1 is the explicit legacy oracle. `--legacy-pressure` selects the old
unbounded increment plus post-clamp retrim; without it the bounded-Cp path is
the production/default simulation and is expected to change the shape row.

### Useful bench flags

`--brake-left CM` / `--brake-right CM` (asymmetric), `--ramp SECONDS`,
`--release SECONDS`, `--tuck [PULL_CM]`, `--dive [DEGREES]`,
`--no-cells`, `--contact`, `--substeps N --iterations N`, `--pressure PA`,
`--line-sweeps N` (free-flight cable-only reverse/forward pairs),
`--legacy-pressure` (regression only), `--pressure-acceptance` (bounded versus
legacy production-shape gate).

The `--glide` rows now also carry **bank, heading, turn rate and
sideslip**. Heading is measured from the wing's travel THROUGH THE AIR,
not its ground track. Forward and sink telemetry likewise subtract the
ambient-air velocity. The first version of this instrument used the ground
track and reported the brake turning the wing the wrong way.

The `--tuck` / `--dive` tables carry `sec`, `vol` and `vnt`: the worst
bay's live/rest section, its live/rest volume, and how many bays the
collapse vent is acting on.

The GUI writes the same numbers to a session log, truncated on start and
on every Reset:

```
%LOCALAPPDATA%\Laboratori d'envol\LEparagliding\playground-session.log
```

Read that log first if the user reports something. Its first column is
SIMULATED seconds, and so is the `Time N.N s` now at the front of the
shape HUD — the two line up deliberately.

**Wall clock versus simulated time is a trap that has already bitten
once.** The wing keeps its own 60 Hz clock however long a frame takes to
compute, so at 30×2 on a real wing it runs 2–3× slower than the clock on
the wall. That is why the brake now has a hand-speed limiter (below), and
why the HUD shows simulated seconds.

---

## What was done, and what it is worth

### The brake's turning moment — done

A one-sided pull now produces a real, mirror-symmetric turn **toward the
braked side**: 8 cm ramped over 6 s gives −11 °/s left, +7 to +27 °/s
right, against +3 to +11 °/s hands-up on the Swoop.

The wing-level resultant and live anchor remain shared over the whole skin.
The bounded-Cp hierarchy freezes that force and its pitch result before adding
the halves' lift/drag difference. Its target retains the base L-R pressure
field, so the arc's lateral bracing is not rebuilt per half. Equal brakes and
equal angles request no new differential. Guard 1 is bit-for-bit only because
it selects the explicit legacy solver.

Roll and yaw damping had to come first, or nothing was measurable: the
hands-up glide drifted 47° of heading in ten seconds. Both now come from
the canopy's rigid-body spin (per-half angle departure and per-half
dynamic pressure). Heading drift over the first ten seconds fell 47° → 11°.

Three failed approaches are recorded in
`docs/playground-shape-analysis.md` with their measurements — a fifth row
in the solve, a spanwise gradient layered on after, and running the whole
force pass per half-span. Read them before proposing a fourth.

**Still wrong: the bank is inverted.** The wing skids — banked out of the
turn it is yawing into — because `polarFor` adds the brake as effective
camber, so the braked half always makes *more* lift, and the speed loop
that should drop it arrives a second later and is worth about the same.
Closing that means re-calibrating `kBrakeCamberRadians` against a
flap-deflected stall angle, which needs data this project does not have.

### A folded cell losing its air — done

A bay below 55% of its rest **volume** is vented toward ambient. Three
things are load-bearing and each was learned by measurement:

- It is a **leak, not a reduced target**, so a folded bay whose mouth
  still meets the airflow still rams itself open.
- The signal is **volume, not section area**. Section area never fires:
  through `--dive -6` gnuC2 lost 31% of its enclosed volume with no rib
  loop below 67% of its own rest section. This canopy collapses by
  concertinaing its ribs together.
- Healthy wings sit at 95–98% of rest bay volume on all four meshes, so
  the deadband is most of the range.

`--dive -6` on gnuC2 now ends at −1.4% volume and 94% span (was −31% and
67%), mirror error 2165 → 183 mm, and the cell field carries a real
53–77 Pa gradient where it used to be a flat 74–80. The ×10 slider is
finally a real experiment.

### Brake hand-speed limit and polar wake lag — done, NOT yet validated

`SimBody::brakeApplied` chases the control at `kBrakeHandSpeed` (0.6 m/s)
**in simulated time**, and `SimBody::brakeFilteredMetres` lags that by
`alphaFilterSeconds` before reaching the polar. The first fixes the
wall-clock/simulated-time mismatch above; the second makes the turning
couple internally consistent, since the rotation-derived halves of it are
already low-passed.

**Neither has been shown to fix the case that motivated them.** The
motivating report is in the next section and reproducing it needs the
GUI, not the bench.

---

## Bounded final-Cp retrim — fixed (P5)

### The report

On gnuA1 in free flight the leading edge dimples, the trailing edge flaps
"like there's no pressure inside at all", and the flapping oscillates
within each cell and often never settles. It is better at 60×4 than at
30×2.

### What the instruments say

```
LE dp   1..16 Pa      leading edge carries almost no load
TE dp -29..-33 Pa     trailing edge is SUCKED IN
cells  59..71 Pa      the air inside is at its 61 Pa target and uniform
pitch M 137..182 N.m  the pitch solve is NOT being achieved
```

### The diagnosis

`applyPressure` is correct and does what a pneumatic cell must: one
interior pressure per cell, all chordwise variation on the exterior via
`Cp(chordFraction)`.

`applyAerodynamicForces` then adds `δp = n̂·v + μ·s`, where `s` is the
**chordwise station**. That is a chordwise-varying addition to the net
difference across the skin — the front and back of one sealed cell end up
at different pressure, which no real cell can do.

And it passes a hard physical limit, which is the provable part. With the
interior at ~62 Pa and the net difference at −30 Pa, the exterior must be
at ~92 Pa gauge, i.e. **Cp ≈ 1.2–1.5. Above stagnation.**
`externalPressureCoefficient` caps Cp at 1 for exactly this reason and
says so in its comment; the retrim's floor is an arbitrary `−0.5·q` and
drives straight through it.

The non-zero pitch residual is the corroboration: 137–182 N·m means the
clamp is eating the solve, which only happens when the retrim is asking
for pressures the clamp refuses.

That the trailing edge is where it lands, and that more substeps help,
both follow: the trailing edge is the far end of the chordwise constraint
chain from the line attachments, so it is the least converged place for a
bogus load to act.

**Ruled out by measurement, do not re-investigate:** the cross-ports are
not stealing pressure (a cell has one pressure state, so the term has no
chordwise degree of freedom at all, and neighbours at equal pressure
exchange exactly nothing); the collapse vent is not firing (0 bays vented,
worst bay 95% of rest volume on a healthy gnuA1); the cell interiors are
at target throughout.

### The production fix

`applyPressure` now retains, per face, the cell interior pressure, local
dynamic pressure and physical exterior-Cp prior. `playground_pressure` is a
Qt-free deterministic bounded weighted-equality solver. Its variable is the
**final** exterior Cp, bounded to the prototype envelope `[-3, 1]`; a face is
always reconstructed as `p_difference = p_inside - q*Cp`. Faces with
effectively zero q are fixed to their prior, because they have no aerodynamic
authority.

The objective prior is the same-frame pressure topology produced by the
calibrated legacy load path: its global force/pitch 4x4 pass followed by the
two half-wing 4x4 passes. Every intermediate proposal is clamped through the
face's physical Cp pressure interval `[p_inside-q, p_inside+3q]` before it is
converted back to Cp. Do not substitute the legacy absolute pressure clamp:
during the pre-inflated soft start `p_inside` can be about 80 Pa while local q
is below 1 Pa, so that old clamp's nominal lower bound can exceed its upper
bound. Starting from the base section Cp was also measured and rejected: the
minimum bought the polar with new leading-edge suction and produced a 125 mm
dent.

The weighted objective is an area integral with equal Cp mobility per unit
area. Each active face has a local +/-0.05 Cp trust interval around that
calibrated proposal, intersected with the global `[-3,1]` envelope. That is
4 Pa at the standard q=80 Pa: enough to repair small clipped resultant and
pitch errors without allowing exact global equalities to rebuild the pressure
topology at the nose or trailing edge. Equalities enter in a strict hierarchy:
three global force rows, pitch about the live anchor, then the two L-R
lift/drag rows with one common authority. Each later stage freezes the physical
result achieved by the earlier stages. The differential target retains the
prior L-R pressure field and adds twice `sideDifference` (the latter is +D on
one half and -D on the other), preserving the arc's lateral bracing.

The solver uses a six-variable-or-smaller semismooth dual Newton projection,
with deterministic line-search and iteration limits. The exact pure/offline
mode bisects the requested target ray. On one undeformed full-q gnuC2 geometry
it found force authority 0.1608, but cost 78 projections, 595 Newton iterations
and 598 ms. That audit rejected an earlier first-bound shortcut: its answer was
zero merely because one preferred face already lay at a trust bound while
thousands of other faces could redistribute safely.

Production therefore caches a verified authority and the normalised dual
multipliers for each stage. Force starts at zero; pitch and differential start
at one so their normally feasible full targets retain the one-projection fast
path. Below one, a stage projects the cached value on the live geometry,
halves it deterministically if geometry made it infeasible, then tries one
+0.02 increase. The current hierarchical value is already the exact
zero-authority baseline, so it is not redundantly projected. Every value that
is accepted remains an exact bounded weighted projection, and later stages
freeze the actual earlier result. Physical saturation (`valid`, authority
below one) remains distinct from invalid/numerical failure. Diagnostics carry
the incoming hint, accepted probes and backoffs as well as physical N/N.m
residuals, rank/condition, active Cp bounds, Cp range,
projection/iteration counts and solve time through `SimBody`, shape metrics,
the HUD/session log and bench.

On the normal gnuC2 subdiv-1 body (15,840 faces), the production bounded run
settles with no flags at span 1.0144, volume 1.0192, 69.1 mm leading-edge dent
and -2.30 degree washout, versus the legacy oracle's 1.0150, 1.0228, 71.7 mm
and -2.36 degrees. A measured final frame reported
force/pitch/differential authority 0.2000/0.1975/0.1906, ranks 3/4/6, Cp range
-1.8641..1, and six projections / 23 Newton iterations / 16.8 ms for the Cp
pass. Its actual applied telemetry was 1177.0 N lift and 25.7 N drag; the
requested polar target remains separately visible in diagnostics. The apparent
L/D 45.74 is therefore an authority-limited applied-force ratio, not the
finite-wing polar's requested glide performance. The total
shape step measured 100 ms/frame in that run, but other processes were active;
use the isolated performance profile rather than treating one wall-clock
sample as a regression guard. `--pressure-acceptance` runs both fresh bodies
and enforces the shape/Cp gate above. `--legacy-pressure` is the only route to
the former 4x4 increment plus post-clamp implementation and exists solely for
the bit-comparable oracle.

That pressure-only L/D is intentionally still what the pinned shape instrument
reports. In bounded **free flight only**, any positive polar drag not realized
by the pressure field is now applied along the relative wind as a
current-face-area-weighted skin traction. It is diagnosed separately in N and
air-relative W, is exactly zero at q=0, never supplies missing lift, and is
absent from both the pinned pressure-shape gate and the legacy oracle. Its
constructed power is non-positive. Its target is exactly
`q * planformArea * polarCd`: the separate excess-frontal-area/fabric-drag
heuristic remains on its existing pressure request and is **not** reinjected
through this traction. Including that heuristic was measured creating a
self-amplifying loop: normal early cloth breathing reported 2-5 m2 of excess
area, which became hundreds of newtons of extra shear, more deformation and
still more reported area.

The final gnuC2 launch/load-path probe uses three reverse/forward cable-only
sweep pairs, achieved-load q calibration, and eight bounded-only co-moving
quasi-static relaxation frames. At 90 kg pilot / 95.6 kg system it selects
8.57 m/s and 45.0 Pa from effective vertical CL 0.893, with 940 N achieved
wing support against 938 N system weight (horizontal/vertical launch residual
3/+2 N, six scalar calibration solves). At 0.1 s it reports alpha 8.94
degrees, sink -1.38 m/s, span 10.74 m, volume -0.8%, Pz 801 N and risers
853 N; at 2.0 s alpha 10.79 degrees, sink -1.17 m/s, span 10.12 m, volume
-1.8%, Pz 924 N. The unrelaxed/no-cable propagation baseline at 2 s was a
collapsed span 5.47 m, volume -25.1%, alpha -49.1 degrees and sink -7.4 m/s.

A capped `--glide 300` now stays finite and structurally flying through 5 s.
Across its rows alpha remains below 16.39 degrees, L/D above 3.09, span above
9.62 m and volume above -2.6%. At 3 s it is at alpha 13.22 degrees, sink
-1.11 m/s, span 11.20 m and Pz 1477 N; at 5 s alpha 14.77 degrees, sink
-1.64 m/s, span 9.95 m, volume -1.3% and Pz 578 N. This is a large bounded
stability improvement, not a proof of converged trim: the wing still carries a
slow load/phugoid oscillation and 97/190 suspension segments are slack at the
5 s sample. Longer/distributional acceptance remains future work; do not infer
certified flight dynamics from this five-second guard.

---

## Directional stability: section-plane weathercock pass is now wired

### The original report

Pull one brake and the wing steers — correctly, toward the braked side —
then loses inflation and collapses. The pilot's description: "it feels
like the wing rotates but the local wind does not, so it's essentially
flying sideways, and then correctly collapses."

### It was not the weather

Free flight now states this explicitly. `SimControls::ambientAirVelocityWorld`
is the world velocity of the surrounding air, and every aerodynamic path uses
`airVelocityWorld − surfaceVelocity`. The pressure slider's q-derived speed is
added only in the tunnel; in free flight it is a load cap and trimmed-launch
reference, never a second moving atmosphere. A trimmed launch starts at
`ambient − rotate(referenceFlow, span, glideAngle)`, while DropFromRest starts
at zero ground velocity but keeps the build's reference-condition
pre-inflation. Rotating the atmosphere with the wing remains forbidden.

The focused Galilean regression shifts ambient air and every node velocity by
the same non-axis-aligned vector. Wing sample, cell/intake pressure, distributed
force and one complete solver step remain invariant in the relative frame.

### What was actually missing

`sampleWingAero` deletes the sideslip before it measures anything:

```cpp
const softwing::Vec3 windInPlane =
    relative - dot(relative, spanAxis) * spanAxis;
```

That span component IS the sideslip. After this line it survives only in
the lift and drag DIRECTIONS. Nothing in the model turns sideslip into a
yawing moment: the polar's drag acts through the anchor on the centreline
so it has no lever arm, the per-rib pressure winds carry rotation but not
sideslip, and the pilot's drag acts below the wing and gives roll.

So the wing had **no directional stability whatever**. Once it yawed,
nothing brings the nose back into the wind, the sideslip persists and
grows, and a canopy flying sideways collapses. It is visible hands-up
(sideslip walks out to several m/s on the Swoop with no input at all) and
the brake makes it worse, because the turning couple yaws the wing and yaw
without weathercock stability is pure sideslip generation. The better the
steering works, the faster the wing ends up flying sideways.

### What is implemented

A real canopy weathercocks from its arc: sideslip meets the two halves' tilted
section planes at different incidence. Each rest rib normal is now mapped from
the rest span/chord/up frame into the live rigid span/chord/up frame. The common
chord and each half's rigid-spin-relative flow are projected into that mapped
section plane, then averaged with projected rib planform area. The no-beta
section incidence is subtracted per rib and the exact planform-weighted common
mode is removed, so asymmetric section geometry cannot move common trim.

The result enters only the existing low-passed/clamped half differential; no
freestream rotation, per-node velocity feedback or new force channel was
added. Focused tests prove flat-arc zero response, no-beta invariance on an
asymmetric section setup, mirrored beta sign, near-odd restoring yaw-pressure
moment parity, and bounded extreme-beta state. The deterministic tunnel CSV
guard remains exactly: lift 1280.0 N, L/D 7.66, line reaction 1146.8 N and the
same row loads/shape fields documented above.

Directional stability alone did not fix the original 1.7 s departure. The
subsequent cable-load, launch and polar-only-drag repairs now keep the capped
five-second gnuC2 probe within alpha 16.39 degrees, span above 9.62 m and
volume above -2.6%; see the final P5 measurements above. That still does not
make the helper-level restoring sign or the full model a certified glide.

### Also true, and probably related

gnuA1 is under-inflated **in the tunnel**, before free flight is involved
at all: `--shape --pressure 61` gives volume −8.3% and an `UnderInflated`
flag, with row B carrying 18 N against row A's 375. And in the bench's
free-flight launch it dives from the rest pose with no brake at all —
alpha decays to −0.7° and airspeed runs to 18 m/s by 5 s. Whether that is
the same bug or a separate trim problem on a low-aspect-ratio wing
(AR 3.45) is not established.

---

## Invariants — the expensive lessons

Breaking any of these has cost days before. They are in
`docs/playground-shape-analysis.md` in more detail.

1. **System-level forces enter the fabric as pressure.** Point loads at
   line attachments dent the intrados; area-spread body forces lean the
   canopy; distributed couples crush the nose. All measured failing.
2. **The polar cancels the pressure field's resultant in full.** Anything
   added per-face is cancelled along with it and reaches the trajectory
   as *nothing*. A new force must go into `wingForce`.
3. **No per-node velocity feedback into the pressure field.** Pressure
   accelerates fabric, moving fabric sees less wind, less wind means less
   pressure — the canopy talks itself flat (span 10.4 → 5.2 m, measured).
   Anything rotational must come from the canopy's **rigid-body** fit,
   which has no breathing mode.
4. **α's dynamic sign must be verified**: sinking must *raise* α.
5. **Damping is relative** to the bulk velocity, or it becomes a fake
   drag ~5× the real budget and sets the trim speed itself.
6. **Drag decelerates, never propels.**
7. The solver's "left" brake is at **negative mesh x = the viewer's
   right**. The crossover lives in `setBrakePull` alone. `SimBody::ribHalf`
   and the bench's `--brake-left` use the solver's sense.
8. **A control default that is only pushed to the view on `valueChanged`
   is never applied.** The air-mote slider read 100 while the field stayed
   off, because `setValue` in the constructor emits nothing. `ensureView`
   now pushes it like the other controls; check that list when adding one.

## Suspension graph and load telemetry — fixed

The simulation writer used to mirror `capturedLines_` even though the legacy
drawing pass had already captured both wing halves. Every physical segment was
therefore emitted twice (gnuC2: 380 records for 190 unique lines). The writer
now emits each quantized endpoint/plan/brake identity once in deterministic
first-seen order. The parser and body builder also reject exact/reversed
duplicates defensively, so saved meshes from before this fix still produce one
XPBD cable per physical line. Coincident segments with different authored
plan/brake roles remain distinct.

Line diagnostics now distinguish authored suspension from the drawable pilot
harness and synthesized brake-control cables. Riser load is the magnitude of
the vector reaction through the authored canopy-side cut above the carabiners,
not a scalar sum of both sides of the same load path. Row sides are vector sums
for the same reason. The free-flight bench's system weight now comes from every
dynamic solver-node mass, including rib fabric, line junctions and controls,
instead of `pilot + planform area * skin density`.

The remaining instantaneous-λ question is diagnostic: a single last-substep
multiplier is not a time-averaged load instrument.

## Bounded cloth contact — fixed, still opt-in (P3)

The old rest-distance exclusion and vertex-only pass has been replaced by the
Qt-free `playground_contact` module. It supports skin vertex/triangle, skin
edge/edge and authored suspension segment/triangle contact. Incident/one-ring
skin topology and authored line attachments are excluded explicitly; unrelated
fabric merely close in the designed pose still collides. Harness and synthetic
brake-control segments are filtered by `LineSegment::suspension`.

The 3-D grid does not truncate large primitives or skip dense cells. Oversized
features are tested explicitly, dense cells use local sweep-and-prune, and any
broad-phase or candidate budget hit marks `coverageComplete=false`. Candidates
are deterministic and persist across substeps until a velocity-predicted,
mesh-scaled envelope is escaped. Projection is mass-weighted, remembers the
approach side through crossings, and removes closing normal velocity. Bench
diagnostics expose candidates by feature, active contacts, refreshes, budgets,
coverage and worst penetration before/after.

Contact remains off by default. A deliberately bounded gnuC2 startup frame at
native resolution/30x2 completed coverage with 1,474 vertex/triangle, 3,968
edge/edge and 573 line/triangle candidates and eight refreshes, but cost about
550.6 ms versus 24.3 ms with contact off. That is correct enough to expose
fold topology, not interactive enough to enable globally. Friction and
line/line contact remain unsupported. The focused Qt-free test covers every
feature and failure diagnostic; a separate `stepSimulation` integration test
guards enabled blocking and the unchanged disabled ghosting path.

## Explicit payload, line mass and launch — done (P1)

Free flight no longer sizes the pilot from the aerodynamic polar that is meant
to carry him. `SimControls::pilotMassKg` is an explicit pilot-plus-harness input
(90 kg default, clamped to 30–250 kg); the GUI and `softwing-bench
--pilot-mass KG` rebuild the body with that mass, and the system-mass readout is
the sum of the actual dynamic solver nodes.

Authored suspension mass is now `length * 0.001 kg/m`, half lumped to each end
of every unique physical segment. The 1 g/m fallback is explicit because the
mesh does not yet carry measured material g/m. In free flight each welded
junction adds only a 0.1 g positive solver floor. Physical line mass, total
junction floor and the generated brake-control-node floor are reported
separately. Harness ties add no separate mass (the payload input already
includes the harness), and synthesized brake-control cables add no physical
line mass (the authored brake cascade already owns it).

The pinned, zero-gravity tunnel retains the historical 50 g per junction only
as explicitly diagnosed **nonphysical static-relaxation ballast**. It cannot
change tunnel weight or static equilibrium because gravity is off, but keeps
the light branched cable graph conditioned at the standard 30x2 budget. Free
flight never receives it: its nodes contain physical length-lumped line mass
plus the 0.1 g floor, and its reported system mass remains physical.

`LaunchMode::TrimmedGlide` remains the default and preserves the common
flight-path direction, but its speed is now weight-aware. On an untouched
rest geometry, then again after build relaxation, a short scalar loop measures
the bounded field's achieved vertical wing force and scales q toward the total
dynamic solver-node weight. The final verified authority/dual cache is retained
for release. Tests cover 30/90/250 kg, monotonic q/speed (with explicit 4x-q
cap saturation) and achieved support within 5% when uncapped.

Before release, bounded TrimmedGlide alone runs eight co-moving quasi-static
frames with gravity, aerodynamics and the real suspension graph. Every node's
velocity is reset to the common apparent-flow velocity each frame, deliberately
erasing relative vibration while positions find a load-compatible shape.
Travel, wake/half-q/brake filters, cell state and Cp warm starts are then reset
and the final q is recalibrated. This costs roughly one second on the full
gnuC2 body but raises the measured 0.1 s riser reaction from 619 N to 853 N and
prevents the release transient from outrunning the light line graph.
`DropFromRest` leaves every node at zero velocity under the already stamped
pressure field and performs none of this: it is explicitly a pre-inflated
release transient, not a ground-inflation model. The P1 payload remains one
dynamic translational point joined to the carabiners by solved bilateral XPBD
constraints. A focused gravity test fixes the carabiner side, releases that
same payload off-bottom, and verifies that it accelerates and swings toward
bottom; orientation, harness inertia and weight shift remain for the later
rigid-payload migration.

The split is intentional. Giving physical gram-scale inertia to the pinned
tunnel exposed a severe 30x2 limit cycle, so the pinned zero-gravity instrument
keeps its diagnosed ballast. Free flight instead sets
`StepSettings::cableConstraintSweepPairs=3`: six cheap serial cable-only passes,
reverse then forward, are distributed before the normal structural sweeps so a
payload reaction traverses the seven-level suspension graph while cloth/ribs
still react in the same iteration. Zero is an exact historical bypass and is
what the tunnel uses. On gnuC2 at 2 s, zero pairs collapsed to span 5.47 m,
volume -25.1% and alpha -49.1 degrees; three pairs held span 10.33 m, volume
+4.6% and alpha +8.73 degrees before the later launch/drag repairs. A focused
seven-level 90 kg chain test verifies the pass count, determinism at zero and
reduced worst cable extension. This is targeted load-path conditioning, not a
reason to put tunnel ballast into free flight.

## Selectable orthotropic fabric and fold hinges — done, prototype (P4)

The Playground now has a clear legacy/material selector. Legacy remains the
default and keeps its skin topology and arithmetic unchanged. The material
mode removes the skin distance truss entirely, then builds one orthotropic
membrane element per valid skin triangle. Ordered source quads define
`q0 -> q3` as chord/warp and `q0 -> q1` as span/weft; each triangle gets an
isometric tangent chart transported from that common direction, so the rest
pose begins at exactly zero Green strain. Ribs, straps, seams, suspension,
cells and the opt-in contact path are retained.

Compression softening is SPD-safe `D*K*D`. Ratio `1.0` takes an exact bypass
through the historical bilateral matrix; the prototype uses `0.05`. The shear
scale is the geometric mean of the normal direction scales, so one compressed
direction partially softens shear and two give the full retained ratio. The
default prototype stiffnesses are warp/weft/coupling/shear
8000/5000/1000/1500 N/m. These are exploratory controls, not measured fabric
data.

Every valid interior two-triangle edge receives a signed four-node XPBD
dihedral hinge. It resists only fold, not in-plane shear, is mass weighted and
solved in deterministic insertion order after the membrane sweep. Boundary
edges stay free; degenerate, non-manifold and inconsistently wound incidence
is counted and skipped. Hinge state participates in rollback and persistence.
Finite-difference tests cover the signed gradient at a generic nonzero hinge
and prove that one projection reduces both residual signs; mirrored winding,
degeneracy, fixed masses and state round-trip are also guarded.

The gnuC2 material inventory is 15,840 membranes, 23,727 hinges, zero skipped
elements/hinges and 6,949 retained non-skin distance/cable constraints. The
bounded acceptance probe

```powershell
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --membrane --frames 10 --warmup 0
```

stayed finite at the full 80 Pa hard load: volume +4.7%, span +1.7%. It is not
interactive yet: about 267 ms/frame at 30x2, split roughly 135 ms membrane and
98 ms serial hinges. This is deliberately a bounded correctness probe, not a
settled shape or calibration result.

One real limitation was found and left explicit. The generic nonzero membrane
constraint damping is stable on isolated coupons but destabilises this mixed
membrane/rib/line network at the high substep ratio. It is not the compression
branch or hinge path: bilateral compression with nearly disabled bending shows
the same failure, while zero damping remains finite. The Playground prototype
therefore defaults damping to zero and warns on an explicit nonzero value. A
future fix needs a coupled damping design, not material-constant tuning.

`softwing_material` and `playground_material` cover constitutive SPD/exact
bypass, topology/model separation, ordered chart directions, membrane metrics,
hinges, degeneracies/non-manifold incidence and deterministic finite stepping.
The established gnuC2 legacy `--shape --csv --legacy-pressure` row remains
bit-for-bit unchanged.
