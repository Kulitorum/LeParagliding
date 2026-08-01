# Playground free flight — handover

The two changes the previous handover queued are done and measured. What
is open now is one diagnosis, made with numbers, that the previous
handover did not know about: **the pitch retrim pays for its couple with
a pressure the air cannot exert**, and that is what makes a trailing edge
flap and a leading edge dimple.

Everything here is on `main`, pushed, working tree clean.

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

Meshes for benching exist at `build/aero/{gnuC2,Swoop,gnuLAB4,gnuA1}/lep-sim.json`.
Regenerate with `leparagliding-engine --preview
resources/presets/<name>/leparagliding.txt <outdir>`.

**A running LEparagliding.exe locks the link target.** Do not kill an
instance you did not start — rename the exe aside and relink, or ask.

### The guards that must not move

```powershell
# 1. Tunnel calibration. Bit-for-bit: compare the CSV row, not the prose.
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --shape --csv
# expect settled 3.2 s, 1278.2 N lift, L/D 7.66, no flags,
# rows A 376.8/367.0  B 212.1/211.2  C 49.1/51.9

# 2. Symmetric free glide, first 10 s.
.\build\bin\Release\softwing-bench.exe build\aero\Swoop\lep-sim.json --glide 600
# expect alpha 10-13, L/D 6.4-6.7, volume within +3%, span 8.4-8.7

# 3. Collapse recovery.
.\build\bin\Release\softwing-bench.exe build\aero\Swoop\lep-sim.json --tuck 250 --substeps 60 --iterations 4
# expect "recovered: no flags", and 0 bays ever vented
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --dive -6
# expect volume back to about -1%, span 94% of rest, cells spread 53..77 Pa
```

Guard 1 is genuinely bit-for-bit today — every change below was checked
against the CSV row and none of them moved a digit. Keep it that way; it
is the cheapest correctness signal in the project.

### Useful bench flags

`--brake-left CM` / `--brake-right CM` (asymmetric), `--ramp SECONDS`,
`--release SECONDS`, `--tuck [PULL_CM]`, `--dive [DEGREES]`,
`--no-cells`, `--contact`, `--substeps N --iterations N`, `--pressure PA`.

The `--glide` rows now also carry **bank, heading, turn rate and
sideslip**. Heading is measured from the wing's travel THROUGH THE AIR,
not its ground track: the model flies the wing in an air mass moving at
the airspeed the pressure slider sets, so the ground velocity is the
difference of two comparable vectors and its direction is nearly
meaningless — a 10° yaw can swing it 90°. The first version of this
instrument used the ground track and reported the brake turning the wing
the wrong way.

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

The shared force pass is **unchanged** — same wing-level resultant, same
anchor, one solve over all the skin. On top of it a *differential* pass
adds half the halves' coefficient difference to one half and takes it off
the other, through the same 4×4 machinery, about each half's own anchor
on its own mean chord. The two anchors are offset spanwise, so the couple
has the wing's real lever arm and costs nothing in pitch. With equal
brakes and equal angles the difference is exactly the zero vector and the
pass does not run — which is why guard 1 is bit-for-bit.

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

## OPEN, and the important one: the retrim exceeds stagnation pressure

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

### What to do

The floor should be the physical one — the exterior cannot exceed
stagnation, so the net difference cannot fall below `interior −
ribPressure` — rather than `−0.5·q`.

Be warned what that costs: on a healthy cell sitting at ram that floor is
about **zero**, so the retrim would lose the ability to pull any face
inward, and the pitch solve is *already* saturating. The couple would
then have to come from moving the centre of pressure — reshaping the
exterior Cp, which is legitimately chordwise-varying — instead of from a
gradient added to the net difference. That is a redesign of the
calibrated stability stack, which is the part of this model that has
historically cost days. Do it deliberately, on top of a commit, with
guard 1's CSV row open.

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

## Also open, lower priority

About two-thirds of line segments read zero tension even in healthy
flight (258/378 on the Swoop), while the tunnel's own per-row table looks
sane. Some of it is cascade branches sharing unevenly plus an
instantaneous solver-λ read, but it has not been explained. The riser sum
in the session log is affected — it reads roughly twice the system weight.
