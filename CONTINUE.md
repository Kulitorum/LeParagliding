# Playground free flight — handover

Two changes are queued on the Playground's free-flight model. Both are
diagnosed, both have a measured failure the fix has to move, and one of
them has two recorded failed attempts you should not repeat.

Everything here is at `02edc34` on `main`, pushed, working tree clean.

---

## Orientation

The Playground is a soft-body paraglider sandbox: XPBD cloth + cable
constraints, a stamped pressure field, and a wing-level aerodynamic
polar. It lives in:

| | |
|---|---|
| `src/gui/playground_sim.{h,cpp}` | the wing: mesh → body, pressure, polar, step |
| `src/gui/playground_metrics.{h,cpp}` | the instruments (shape report, collapse diagnostics) |
| `src/gui/playground_page.{h,cpp}` | the tab: GL view, controls, session log |
| `tools/softwing_bench.cpp` | headless driver — same body the GUI builds |
| `tests/playground_cells_test.cpp` | unit tests for the per-cell air model |
| `docs/playground-shape-analysis.md` | the design record; keep it current |

Build and test (cmake/ctest are **not** on PATH):

```powershell
$env:PATH = "C:\Qt\Tools\CMake_64\bin;$env:PATH"
cmake --build build --config Release --target softwing-bench playground-cells-test LEparagliding
ctest --preset release          # 21 tests, all must pass
```

Meshes for benching already exist at `build/aero/gnuC2/lep-sim.json` and
`build/aero/Swoop/lep-sim.json`. Regenerate with
`leparagliding-engine --preview resources/presets/<name>/leparagliding.txt <outdir>`.

### The two guards that must not move

Run these before and after **any** change to the force model.

```powershell
# 1. Tunnel calibration. Expect: settled 3.2 s, 1278 N lift, L/D 7.66, no flags.
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --shape

# 2. Symmetric free glide. Expect: alpha 11-12 deg, L/D ~6.5, volume within +3%.
.\build\bin\Release\softwing-bench.exe build\aero\Swoop\lep-sim.json --glide 600
```

A change that improves an asymmetric case while moving either of these
has not worked. Attempt (a) below was caught exactly this way.

### Useful bench flags

`--brake-left CM` / `--brake-right CM` (asymmetric), `--ramp SECONDS` (a
step to 30 cm tumbles a wing that survives the same pull ramped over 6 s
— never test a brake as a step), `--release SECONDS`, `--tuck [PULL_CM]`,
`--dive [DEGREES]`, `--no-cells`, `--contact`, `--substeps N
--iterations N`.

The `--glide` rows carry riser load, slack-segment count, weakest cell,
sharpest leading-edge kink and fabric drag. The GUI writes the same
numbers to a session log, truncated on start and on every Reset:

```
%LOCALAPPDATA%\Laboratori d'envol\LEparagliding\playground-session.log
```

Read that log first if the user reports something — it carries their
control inputs interleaved with the wing's response.

**Wall-clock matters when reading a user's report.** The Swoop runs at
~27 fps at 30×2 and ~8 fps at 60×4, against a 60 Hz simulated clock — so
the user watches at 0.46× or 0.13× real time. Twenty seconds of watching
is 2–9 seconds of flight.

---

## Task 1 — the brake's turning moment (per-half-span force pass)

### Where it stands

`applyAerodynamicForces` (playground_sim.cpp:2507) evaluates a polar for
each half of the wing at its own brake — `polarFor` at :2590, called at
:2621–2622. The pull enters as an effective camber angle
(`kBrakeCamberRadians`, 8° at full travel) rather than a bare lift
increment, so the braked half also reaches the stall blend first.

That part works and is symmetric-safe: the wing-level pair is the mean of
the two, so equal brakes reproduce the old numbers exactly.

**What is missing is the moment.** The two halves' coefficients differ,
but that difference never reaches the wing, so a one-sided pull still
produces no turn — it just decelerates and departs at ~8 s.

### Two attempts that failed. Do not repeat them.

**(a) A fifth row in the force-distribution solve.** The correction is
distributed as a per-face pressure increment `δp = n̂·v + μ·s`, with
`(v, μ)` from a 4×4 solve (assembly at :2884, elimination at :2910) that
makes the increment's resultant equal `correction` and its pitch moment
about the anchor cancel the pressure field's. Adding a fifth unknown (a
spanwise gradient `ν`) and a fifth row (roll moment about the wind axis)
**wrecked the symmetric glide**: airspeed 9 → 14.5 m/s, sink −1.3 → −3.2,
with zero brake. Constraining the increment's own roll moment to a
prescribed value is not a no-op — it rebuilds the entire increment field.

**(b) Layering the couple on after the solve.** Symmetric-safe (a
symmetric wing gets exactly zero, and the glide was verified restored),
but it **folded the wing at 4 s against a 9 s baseline**, span 8.4 → 5.4 m
in one second. A spanwise-linear pressure gradient loads the tips
hardest, which is where this fabric is weakest.

Attempt (b) is still in the tree at :2971, behind
`LEP_AERO_BRAKE_ROLL` (off by default), with `rollTarget` computed at
:2858. Read it before designing the replacement, then delete it — it is
kept as a record, not as a fallback.

### What to do instead

Both attempts bolt a couple onto a wing-level resultant. A brake's moment
has to arrive **where the brake acts** — the aft fabric of its own half —
not as a gradient smeared across the span.

Run the whole force-and-distribution pass **per half-span**: split the
skin faces by `dot(faceCentre − anchor, spanAxis)`, and for each half
compute its own α (from that half's ribs), its own brake, its own polar,
its own anchor on its own mean chord, and its own 4×4 solve against that
half's faces. The roll and yaw moments then emerge from two correctly
placed resultants rather than being imposed.

Points to settle while designing it:

- **α per half.** `sampleWingAero` (:2383) currently means over all ribs.
  It must be measured from `RibChord::referenceNode` — the 40%-chord
  extrados node, brake-immune — and rotated back onto the chord by
  `SimBody::attitudeOffsetRadians`. Measuring off the full chord makes a
  brake read as whole-wing pitch-up; that bug ran α to 76° with the
  pilot's hand held still. The offset calibration is **not optional**:
  the reference node rides tens of degrees above the chord on aerofoil
  thickness, and skipping it gave α swinging ±120°.
- **The halves must not fight over pitch.** Each half cancelling its own
  pitch moment about its own anchor is probably right, but verify the
  pair still trims where the single pass did.
- **Fabric drag and the α filter are wing-level** and should stay that
  way; only the polar and its placement split.
- **Symmetric input must stay bit-for-bit.** Two identical halves summing
  to the old single pass is the cheapest correctness check you have.

---

## Task 2 — a folded cell must be able to lose its air

### The symptom

The user's report, confirmed in the session log: after a collapse the
wing will not re-inflate, and turning the **Neighbour reinflation**
slider (`SimControls::crossPortGain`, 1–10) to ×10 changes nothing.

### Why the slider is inert

The cross-port term drives neighbouring cells toward *equal* pressure.
Both the folded cell and its healthy neighbour sit at their ram target,
so there is no gradient — and multiplying zero by ten is zero. The user
worked this out himself before I measured it.

The reason both sit at target is `advanceCellPressures` (:1878): each
cell relaxes toward `target[cell]`, the mean of its two ribs' ram
pressures (:1901), and `ribPressure` (:2186) is `½ρv²` from the rib's own
relative wind — **regardless of whether that rib is in clean air or
buried inside a fold**. So a collapsed cell is force-fed as hard as a
flying one.

Measured through a departure: cell pressure went **up**, 28 Pa → 90–129
Pa, while the wing was folding. The cross-ports had nothing to do.

### What to do

Make an unloaded, folded section decay toward ambient, so there is a
gradient for the cross-ports to act on and the reinflation path becomes
real. The signal for "this section is not flying" needs to be positional
and restoring — no velocity feedback (see the invariants below). Two
candidates, both already computed nearby:

- the live/rest **section area ratio** (the squeeze term at :2034 already
  reads it, via the rib-loop *vector* areas so folded loops cancel), and
- the **line tension** into that bay — `constraintTensionNewtons` in
  playground_metrics, already used by the log and bench.

A section at a small fraction of its rest area, with slack lines, is a
bag: its target should fall toward zero rather than stay at ram. Get that
right and the ×10 slider becomes a real experiment rather than a no-op.

Guard it the way the rest of this model is guarded: a **deadband**, so a
healthy loaded wing (which sits between 100% and 106% of rest volume)
sees exactly nothing and the calibration does not move.

Verify with `--tuck 250` on the Swoop at 60×4, which currently recovers
cleanly with no flags — that must stay true — and with `--dive -6` on
gnuC2, which is the standing collapse-recovery case.

---

## Invariants — the expensive lessons

Breaking any of these has cost days before. They are in
`docs/playground-shape-analysis.md` in more detail.

1. **System-level forces enter the fabric as pressure.** Point loads at
   line attachments dent the intrados; area-spread body forces lean the
   canopy; distributed couples crush the nose. All measured failing.
2. **The polar cancels the pressure field's resultant in full**
   (`correction = wingForce − aerodynamicForce(sim)`, :2741). Anything
   added per-face is cancelled along with it and reaches the trajectory
   as *nothing*. A new force must go into `wingForce`.
3. **No per-node velocity feedback into the pressure field.** Pressure
   accelerates fabric, moving fabric sees less wind, less wind means less
   pressure — the canopy talks itself flat (span 10.4 → 5.2 m, measured).
   The per-rib wind uses the canopy's **rigid-body** spin for exactly
   this reason: a rigid fit has no breathing mode.
4. **α's dynamic sign must be verified**: sinking must *raise* α. The
   convention was inverted for a long time, was statically
   self-consistent, and made every free-flight attempt diverge.
5. **Damping is relative** to the bulk velocity, or it becomes a fake
   drag ~5× the real budget and sets the trim speed itself.
6. **Drag decelerates, never propels.** The fabric-drag term is bounded
   at the planform area and at the impulse that nulls the relative motion
   in one frame; unbounded it pushed a collapsed wing *upward*.
7. The solver's "left" brake is at **negative mesh x = the viewer's
   right**. The crossover lives in `setBrakePull` alone — do not add a
   second one.

## Also open, lower priority

About two-thirds of line segments read zero tension even in healthy
flight (258/378 on the Swoop, 237/386 on gnuC2), while the tunnel's own
per-row table looks sane (A 419/409 N, B 231/233 N, C 52/56 N, summing to
the weight). Some of it is cascade branches sharing unevenly plus an
instantaneous solver-λ read, but it has not been explained.
