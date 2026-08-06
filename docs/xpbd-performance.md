# Playground XPBD solver performance

Working notes on where the Playground tab's soft-body simulation spends its
time and what has been done about it. Measured with `softwing-bench`
(`tools/softwing_bench.cpp`), which builds the identical body the Playground
builds — same mesh, same refinement, same constraints, same step settings —
via the shared `src/gui/playground_sim.{h,cpp}` module, and reports the
core's own `StepPerformanceProfile` alongside wall clock.

```powershell
cmake --build build --config Release --target softwing-bench
build\bin\Release\softwing-bench.exe <lep-sim.json> --subdiv 2 --frames 30
```

Machine for every number below: AMD Ryzen 9 5900HX, 8 physical cores / 16
threads, Windows 11, MSVC Release.

## Baseline (2026-07-29, commit f4b1483)

Step settings are the Playground's own: 1/60 s frame, 4 substeps, 30
constraint iterations, simple ribs, single-threaded.

| Preset | Subdiv | Nodes | Constraints | Solves/frame | ms/frame | fps |
|---|---|---|---|---|---|---|
| Alp-Skin-19 | 1 | 3 208 | 12 584 | 1.51 M | 21.2 | 47 |
| Alp-Skin-19 | 2 | 11 707 | 46 799 | 5.62 M | 84.4 | 12 |
| gnuLAB4 | 1 | 5 869 | 24 563 | 2.95 M | 42.3 | 24 |
| gnuLAB4 | 2 | 22 630 | 93 267 | 11.19 M | 186.1 | 5 |
| gnuC2 | 1 | 8 208 | 34 894 | 4.19 M | 60.5 | 17 |
| gnuC2 | 2 | 31 999 | 132 336 | 15.88 M | 290.0 | 3 |

Where the time goes is the same everywhere, to within a rounding error:

```
  solver total                  60.266 ms/frame  100.0%    (gnuC2, subdiv 1)
    prediction + pressure        0.799 ms/frame    1.3%
    distance constraints        59.340 ms/frame   98.5%
    membrane constraints         0.003 ms/frame    0.0%
    membrane diagnostics         0.000 ms/frame    0.0%
    finalization                 0.113 ms/frame    0.2%
```

### What the baseline says

1. **It is one loop.** 97.8–98.5% of every frame is
   `SoftBody::solveConstraint` over the distance/cable constraint vector,
   run `substeps × iterations = 120` times per frame. Nothing else is worth
   looking at until that changes.

2. **Cost is linear and flat.** 13.7 ns per constraint solve at subdiv 1,
   17.8 ns at subdiv 2 — the drift is cache, not algorithm. Frame time is
   simply `120 × constraints × ~15 ns`, so it tracks constraint count
   exactly and the preset only matters through its size.

3. **The wing is already over budget at native resolution.** The view timer
   asks for 16 ms. The *smallest* preset costs 21 ms and the largest 60 ms,
   so the Playground is running at 3–17 fps rather than the 60 it is asking
   for, and the resolution control makes it roughly 4× worse per step.

4. **The core's existing parallelism does not apply here.** `softwing_core`
   has a `WorkerPool`, a graph colouring and two parallel membrane sweeps —
   but all of it is gated on `membraneElements_`, and the Playground builds
   none: the skin is a mass-spring cloth of `DistanceConstraint`s, not
   membrane elements. `SoftBody::poolFor` therefore returns `nullptr` on
   every step and `StepSettings::workerThreads` is a no-op for this body.
   The infrastructure is the right shape; the distance-constraint sweep just
   was never wired into it.

> Every threaded number below is best-of-N wall clock. Single runs on this
> machine swing by 20–30% when Windows Defender is scanning; taking the
> minimum of 3–5 runs is what made the scaling curve legible at all.

## Multi-threading the constraint sweep

Three changes, all in `src/softwing`:

**1. A coloured parallel sweep for distance constraints**
(`SoftBody::constraintColouring`, `solveConstraintsColoured`). Greedy
first-fit over the line graph: constraints sharing a colour touch no common
node, so a colour is one deterministic Gauss-Seidel phase and the whole sweep
runs through the existing `WorkerPool::forEachPhase`. Two things differ from
the membrane colouring it is modelled on:

- *The graph is not degree-bounded.* A simple rib web is a hub with a spoke
  to every loop node, so a hub node can carry over a hundred constraints and
  each spoke needs its own colour. The membrane code's single 64-bit
  per-node colour mask would throw; this one sizes a wide mask from the
  measured maximum degree.
- *There is a long tail.* gnuC2 colours into 38 colours of which 9 hold 93%
  of the constraints; the other 29 are hub spokes, a handful each. Colours
  below `kConstraintsPerBarrier` are run serially after the parallel ones,
  because a barrier costs more than they do.

**2. A chunk grain sized for constraints, not membrane elements.** This was
worth more than everything else combined. The membrane sweep claims work in
chunks of 8 elements; a distance solve is roughly a tenth of a membrane
element's work, so at grain 8 a chunk is ~100 ns and the cache line holding
the chunk cursor ping-pongs between cores faster than the constraints are
solved. gnuC2 at 6 threads went from **47.6 ms to 31.9 ms** on that alone —
before it, threading was worth 1.27×.

**3. A packed hot copy of node state** (`SoftBody::SolveNode`). The sweep
reads exactly `position` and `inverseMass`; a `Node` is 104 bytes. Packing
those two into 32 bytes for the duration of the iteration loop makes no
difference single-threaded, but a large one threaded — two nodes per cache
line instead of one node straddling two means far less cross-core
invalidation:

| gnuC2, subdiv 1 | serial | 4 threads | 8 threads |
|---|---|---|---|
| sweeping `Node` | 63.6 ms | 38.8 ms (1.64×) | 49.1 ms (1.29×) |
| packed | 65.5 ms | 33.6 ms (1.95×) | 37.3 ms (1.76×) |

Only usable when nothing else in the substep moves nodes — membrane, contact
and suspension all do — so it is guarded, and it is the same arithmetic on
the same values either way.

Alongside these, `projectDistanceConstraint` now does two divisions and one
square root where it did five and one: `alpha` takes a caller-hoisted
`1/dt²`, and the unit direction is never materialised (`difference /
currentLength` was three divisions). This measured as *no* change, which is
itself the finding — see below.

### Results

Best-of-3, Playground step settings, simple ribs.

| Preset | Subdiv | serial | 4 thr | 6 thr | 8 thr | best |
|---|---|---|---|---|---|---|
| Alp-Skin-19 | 1 | 22.4 ms | 13.1 | **11.7** | 12.2 | 1.91× |
| Alp-Skin-19 | 2 | 89.5 ms | 41.2 | 35.1 | **33.5** | 2.67× |
| gnuLAB4 | 1 | 43.2 ms | **21.6** | 21.9 | 28.0 | 2.00× |
| gnuLAB4 | 2 | 187.4 ms | 74.9 | **73.8** | 84.3 | 2.54× |
| gnuC2 | 1 | 62.5 ms | 31.1 | **29.9** | 32.3 | 2.09× |
| gnuC2 | 2 | 284.4 ms | 131.3 | **109.0** | 121.8 | 2.61× |

Roughly **2× at native resolution, 2.6× refined** — bigger bodies scale
better because there is more work per barrier. The optimum is at 6 workers
on this 8-core/16-thread part and it degrades badly beyond the physical core
count (12 workers: 55.9 ms, 16 workers: 62.8 ms on gnuC2 — no better than
serial). The Playground therefore asks for `physical cores − 2`
(`playgroundWorkerThreads()`), leaving room for the UI and the driver.

Confirmed in the app, not only in the harness: `LEparagliding.exe
--playground <lep-sim.json>` on gnuC2 at 9x resolution (30 310 nodes) runs
error-free and burns 37.5 CPU-seconds per 8 wall seconds — about 4.7x — where
before the change it could only ever have used one core. (`PrintWindow`
cannot capture the Playground's native GL child window, so the screenshot
taken shows the status line and controls but not the wing itself.)

### Why only 2×, and not 6×

Not Amdahl: the serial colour tail is 6.5% of constraints on gnuC2. The
sweep costs ~15 ns per constraint, about 50 cycles, and removing three of
its five divisions changed nothing — so it is not throughput-bound on the
arithmetic either. What is left is the dependency structure: consecutive
constraints in a Gauss-Seidel sweep share nodes, so each solve's
store feeds the next one's load, and the chain of load → subtract → dot →
sqrt → divide → store is close to 50 cycles all by itself. The colouring
breaks that chain, but it also scatters each colour's constraints across the
mesh, so the locality it costs roughly cancels the parallelism it buys until
several cores are working at once.

This is the honest ceiling for the CPU: the kernel is latency-bound on a
dependency chain, which is exactly the shape of problem a GPU answers by
running thousands of independent constraints at once rather than by making
any one of them faster.

## GPU prototype

`tools/softwing_gpu.cpp` — an OpenGL 4.3 compute-shader backend, run from
`softwing-bench --gpu`. Node state lives in shader storage buffers between
frames; face pressure, the nodal pressure gather, prediction, the constraint
sweep and velocity finalization are all kernels, so a frame costs no host
traffic beyond a 16-byte-per-anchor upload. It reuses the core's own
constraint colouring (`SoftBody::constraintColouringView`), which is what
makes the two backends comparable at all.

Two things about it are prototype-grade and reported rather than hidden.
It works in **float32** — consumer GeForce parts run double at a fraction of
float rate, so a double version would measure the wrong thing. And it needs
a **0.9 ms readback** to hand the pose back for rendering, which a real
integration inside the Playground would not pay (the view could draw
straight from the position buffer).

No CUDA toolkit is installed on this machine, so compute shaders it is; the
GPU is an RTX 3070 Laptop. Note that a switchable-graphics laptop hands
OpenGL to the *integrated* adapter unless the executable exports
`NvOptimusEnablement` — the first measurements were accidentally on a Radeon
iGPU and 66% slower.

### It is dispatch-bound, not compute-bound

A coloured sweep needs one dispatch and one memory barrier per colour, and a
wing colours into a lot of colours:

| Body | Colours | Dispatches/frame | ms/frame |
|---|---|---|---|
| Alp-Skin-19 subdiv 1 | 27 | 3 254 | 15.8 |
| gnuC2 subdiv 1 | 38 | 4 574 | 19.5 |
| gnuC2 subdiv 2 | 72 | 8 654 | 40.0 |

That is **4.6 µs per dispatch, flat**, and cost tracks dispatch count rather
than constraint count — gnuC2 at subdiv 2 has 3.8× the constraints of subdiv
1 but only 2.1× the frame time, matching its 1.9× dispatch count. The RTX
3070 is doing 4.19 M constraint solves per frame, which is nothing; it
spends the frame being told to start.

Most of those dispatches are near-empty. Of gnuC2's 38 colours, 9 hold 93%
of the constraints and the other 29 are rib-hub spokes — one spoke per rib
per colour, so ~60 constraints in a dispatch that costs the same 4.6 µs as
one with 4 000.

### Jacobi is fast and wrong

The obvious fix is a sweep that does not need a dispatch per colour:
`--gpu-jacobi` has every constraint propose a correction from the shared
state and every node average its incident proposals — 2 dispatches per
iteration regardless of topology, so 254 per frame instead of 4 574.

It is **2.84 ms/frame, 22× faster than the serial CPU** and 10× faster than
the threaded one. It is also unusable. The wing's fabric is nearly
inextensible (compliance 1e-8 m/N) and averaged Jacobi corrections are far
too soft to hold it:

| gnuC2, iterations | GPU volume | CPU volume | GPU ms |
|---|---|---|---|
| 30 (shipped) | 67.8 m³ | 8.91 m³ | 2.8 |
| 120 | 13.2 m³ | 8.31 m³ | 10.8 |
| 300 | 10.3 m³ | 8.18 m³ | 23.4 |
| 600 | 9.20 m³ | 8.16 m³ | 39.1 |

At the shipped iteration count the wing balloons to **7.6× its correct
enclosed volume**. Buying convergence back with iterations costs more than
the colouring did — at 600 iterations it is still 13% over and slower than
the coloured path. For this compliance regime Jacobi is a dead end, which is
worth knowing.

### Where the coloured GPU path lands

Best-of-N against the CPU numbers above; enclosed volume is the physical
check — the wing is a pressure vessel, so its volume is what says whether
the solver is holding it.

| Preset | Subdiv | CPU serial | CPU 6 thr | GPU | GPU vs 6 thr | GPU volume error |
|---|---|---|---|---|---|---|
| Alp-Skin-19 | 1 | 22.4 ms | 11.7 | 13.6 | 0.86× | +45% |
| Alp-Skin-19 | 2 | 89.5 ms | 35.1 | 21.0 | 1.67× | +41% |
| gnuLAB4 | 1 | 43.2 ms | 21.9 | 17.5 | 1.25× | +9% |
| gnuLAB4 | 2 | 187.4 ms | 73.8 | 31.2 | 2.37× | +15% |
| gnuC2 | 1 | 62.5 ms | 29.9 | 18.0 | 1.66× | +8% |
| gnuC2 | 2 | 284.4 ms | 109.0 | 37.6 | 2.90× | +13% |

**Verdict: not wired into the Playground, and it should not be yet.**

- The speed only shows up on the big refined bodies (2.4–2.9× over the
  threaded CPU at subdiv 2, 7.6× over serial), because that is where a
  dispatch finally has enough work in it. On the smallest wing it *loses* to
  six CPU threads.
- The accuracy is the blocker, not the speed. float32 costs 8–13% enclosed
  volume on the double-skin wings and **45% on Alp-Skin-19** — a single-skin
  wing with a much softer structure, where the error compounds. That is a
  visibly different wing, not a rounding difference. Isolating it against a
  CPU reference running the *same* coloured sweep (`--threads 6 --gpu`)
  leaves the same 7.4% on gnuC2, so it is precision, not sweep order.

Two things would change the verdict, in this order:

1. **Fix the precision.** Either doubles (fine on a compute card, ~1/64 rate
   on this GeForce) or carrying positions as an offset from a per-frame
   double reference, which is the usual trick and keeps float arithmetic.
   Without this the GPU path is not honest about the wing's shape.
2. **Cut the dispatches.** The rib-hub spokes are what generate the long
   colour tail; giving the simple rib web a different topology, or handling
   the tail colours in one dispatch with atomics, would remove most of
   4 574 dispatches and the coloured path stops being latency-bound.

## Spending the frame better

Everything above made the same work cheaper. This section is about doing
less of it, which turned out to be worth more than all of it.

### Substeps beat iterations

XPBD convergence is governed by `substeps × iterations`, but the two are not
worth the same: a substep re-linearises every constraint about a fresh
state, an iteration only re-solves the same linearisation. The Playground
shipped `4 × 30` = 120 sweeps per frame, hand-picked and never swept.
Sweeping the grid on gnuC2 against a heavily-converged reference (30 × 30):

| Equal budget: 120 sweeps | ms | volume error |
|---|---|---|
| 4 × 30 (shipped) | 28.6 | +8.5% |
| 8 × 15 | 37.1 | +7.1% |
| 15 × 8 | 35.8 | +5.1% |
| 30 × 4 | 35.9 | +3.0% |
| 60 × 2 | 44.4 | +1.5% |

At equal sweeps, substeps win on accuracy and cost more per sweep (each one
re-runs the pressure, prediction and finalization passes). The efficient
frontier is what matters, and on it `30 × 2` reaches **+5.5% error at
24.8 ms** where the shipped `4 × 30` sat at **+8.5% at 28.6 ms** — cheaper
*and* closer, which makes it a strict improvement over the status quo.

That shifted the profile: at 30 substeps the once-per-substep passes went
from 4% of the frame to 31%, so `accumulatePressureForces`,
`predictPositions`, `finalizeVelocities` and the pack/unpack are now
parallel too. The pressure pass had to change shape to get there — it
scattered a third of each face's load to its corners, which cannot be shared
out without atomics, so it now gathers per node from a `TriangleIncidence`
CSR. Each node sums its faces in ascending triangle index, the order the
scatter added them in, so the gather is bit-identical to it.

### The rib web

The simple rib was a centroid hub with a spoke to every loop node. That put
~100 constraints on one node, and since constraints meeting at a node cannot
be solved in parallel, that single hub forced ~100 colours: **29 of gnuC2's
38 colours were hub spokes**, a few dozen constraints each, costing a full
barrier apiece on the CPU and a full dispatch apiece on the GPU.

It is now the same cross-section ladder the detailed model uses, at one bay
deep with one station per outline segment. Holes are skipped for it: a
one-bay ladder tests each cell by its middle, which at one bay deep is the
centre of the section — exactly where an airfoil hole is — so honouring them
would delete most of the struts.

| gnuC2, subdiv 1 | colours | GPU dispatches | GPU ms |
|---|---|---|---|
| hub web | 38 | 4 574 | 19.5 |
| ladder web | 17 | 1 112 | **6.3** |

On the CPU it is a wash (the ladder adds ~5% more constraints and the
parallel colour count is unchanged at 9). On the GPU it is a 3× win, and for
the detailed rib model the serial colour tail drops from 7.4% to 1.0%.

**It is a real model change, and it shows.** The hub tied every loop node to
the section centroid at its rest distance, which is not a rib — it is a
shape-lock that pins the profile to its designed form. The ladder holds the
two skins apart, which is what a rib does, and lets the section otherwise
take its pressure shape. Wings therefore settle at a larger enclosed volume
than they used to, and single-skin wings much more so (Alp-Skin-19 at native
resolution roughly doubles). Whether that is better is a judgement about the
toy, not a measurement — but note the effect shrinks fast with mesh
resolution (Alp-Skin-19: 13.8 m³ at subdiv 1, 9.1 m³ at subdiv 2), which is
what you would expect if the hub was compensating for a coarse mesh.

Two things were tried and rejected: bracing each bay with a second diagonal
(moved the settled volume 0.5%, cost 680 constraints), and selecting the web
per rib by section depth (no discriminator — 32 of 34 Alp-Skin ribs and 59
of 61 gnuC2 ribs have ≥7% depth, so the ladder is geometrically fine on
both).

### Where it lands

Three solver budgets are now exposed in the Playground's **Solver** control
(`lep::playground::solverQualities`), defaulting to Balanced. Best-of-2,
6 workers, ladder ribs:

| Preset | Subdiv | Fast (15×2) | Balanced (30×2) | Accurate (60×4) |
|---|---|---|---|---|
| Alp-Skin-19 | 1 | 5.3 ms (189 fps) | 10.9 ms (92) | 36.0 ms (28) |
| Alp-Skin-19 | 2 | 17.0 ms (59) | 34.6 ms (29) | 112.2 ms (9) |
| gnuLAB4 | 1 | 8.0 ms (125) | 20.6 ms (49) | 59.3 ms (17) |
| gnuLAB4 | 2 | 41.2 ms (24) | 84.4 ms (12) | 255.9 ms (4) |
| gnuC2 | 1 | 10.6 ms (94) | 26.0 ms (39) | 79.6 ms (13) |
| gnuC2 | 2 | 79.4 ms (13) | 148.5 ms (7) | 402.1 ms (2) |

Against the 62.5 ms / 16 fps this started at on gnuC2 at native resolution:
**Fast is 5.9× faster, Balanced 2.4× faster and better converged than the
old default.** The wing is inside its 16 ms frame budget for the first time.

Note the line-tension legend reads the solver's multiplier and divides by
the substep squared to recover a force, so it now takes the substep length
from the live quality setting rather than a constant — otherwise switching
quality would silently rescale every line load.

## The load field

Not a performance change, but it shares the plumbing and it is what the
solver is now solving.

### There is no ambient pressure, and there does not need to be

A membrane only feels the *difference* across it, so the field has always
been a per-face Δp with ambient cancelled out. What was wrong was that Δp
was the **same on every face**. Reality is nowhere near uniform:

- the cell interior sits at ram (stagnation) pressure, so its gauge
  pressure is q = ½ρV²;
- the outside of any face is q·Cp;
- so Δp = q·(1 − Cp), and one control drives the whole field.

That single change fixes the thing that made the wing look wrong: the lower
surface used to be pushed outward at a full q, when in truth it sits near
stagnation (Cp → 1) and carries **almost no load at all**. It now runs 0 Pa
at the leading edge rising to q at the trailing edge, against 140–260 Pa on
the upper surface — so the intrados stays taut and slack instead of
ballooning downward like an airbed.

Cp is capped at 1 because nothing subsonic beats stagnation pressure; that
also guarantees Δp ≥ 0, so no face is ever sucked inward.

### The lift is no longer fake

The old model added a follower force on faces that pointed up, faded by
`max(0, normal.z)`. It existed because a pure follower load has no preferred
attitude and winds the wing around its span axis forever, and the fade was
what stopped that.

Now the load *is* the pressure field, and the attitude feedback is physical:

- Every rib keeps its leading and trailing **node indices**, so its chord is
  read from the live deformed pose, not the rest one.
- A fixed airflow direction is placed by rotating the wing's rest chord
  about its span axis by the angle of attack.
- Each section's local angle is measured between its live chord and that
  airflow, flattened into the section's own plane so sweep and arc do not
  contaminate it.
- `sectionLiftCoefficient` is thin-aerofoil (2π sinα cosα) with a 4° camber
  offset, rolled off past a 15° stall by a Gaussian. **The roll-off is the
  stability**: a section that pitches to a silly angle stops pulling instead
  of pulling harder.
- `externalPressureCoefficient` puts an upper-surface suction peak at 10%
  chord scaling with that CL, and a leading-edge stagnation bump on the
  lower surface.

The suction peak's position is not cosmetic. Putting it at 0% chord — where
the skin faces *forward* — dragged the wing along its own chord instead of
lifting it; moving it to 10% cut the spurious fore-aft pull by a third.

Per-face chord station and surface are baked at build time (nearest rib by
distance to its plane, which follows sweep and arc where a plain spanwise
coordinate would not). Meshes with no rib loops fall back to the old uniform
field.

### Does it produce a wing?

gnuC2 at q = 80 Pa, measured with `softwing-bench --aoa N`:

| AoA | lift | fore/aft | enclosed volume vs designed |
|---|---|---|---|
| 0° | 393 N | +301 N | −2.1% |
| 3° | 447 N | −358 N | +3.1% |
| 6° | 624 N | −437 N | +4.1% |
| 10° | 560 N | −349 N | +4.0% |
| 15° | 646 N | −323 N | +4.0% |

Right order of magnitude — a wing carrying pilot and kit needs about 1 kN,
and this makes 400–650 N at a trim-speed q. The settled shape stays within
4% of the designed one at every angle, so the load field is not inflating
the wing past its own pattern.

The fore/aft number is a forward pull, and that is expected rather than a
bug: integrating pressure alone over a lifting body recovers leading-edge
suction and no viscous drag, which is d'Alembert's paradox. Nothing here
models skin friction. It shows up as the wing hanging forward in its lines.

### On the 0.1 bar figure

Cell overpressure is ram pressure, q = ½ρV²: about **68 Pa at 38 km/h**,
143 Pa on bar, 231 Pa in a dive. That is 0.7–2.3 mbar. 0.1 bar is 10 000 Pa
and would need 461 km/h — the figure in circulation is almost certainly
0.1 **kPa** or ~1 mbar. The Pressure slider's 0–100 Pa range, default 80,
corresponds to 0–47 km/h with a default of 41 km/h, which is ordinary trim.

## Free flight

`SimControls::freeFlight` unpins everything: gravity on, a pilot mass slung
under the risers, brakes that shorten a real line instead of dragging a
synthetic handle, and the whole system translated back to the origin after
each step (`recentreSystem` — position and previous position move together,
so it is a change of origin and not a brake on the system). The Playground
exposes it as the **Free flight** checkbox on the Solver row, with a live
readout (airspeed, sink, glide ratio, angle of attack, pilot mass).

**It flies.** Production checks use `softwing-bench --glide` (and
`--brake N`, `--polar`) and require pressure support, span, enclosed volume
and suspension error to remain bounded together. Swoop 22 and both Hegala
fixtures now hold their span and volume through the ten-second hands-up
check instead of entering the pressure/slack/stress limit cycle.

### The bug that doomed everything before it

The angle-of-attack convention was inverted relative to physics: wind from
below the chord measured as *negative* α, and the slider compensated by
tilting the freestream the opposite way. Statically self-consistent — the
pinned tab never noticed — but dynamically it flipped the fundamental
feedback: a sinking wing LOST lift and a climbing wing GAINED it. Positive
feedback in both directions; no damping, moment anchoring, filtering or
launch trimming could stabilise it, and in hindsight nothing else was ever
going to work until this was fixed. The convention is now physical
(sinking → wind from below → more α → more lift → sink arrested).

### Architecture

The calibrated section-pressure field shapes the fabric and supplies the
free-flight lift at its natural centre of pressure. A classical finite-wing
polar supplements that field where it is useful without replacing it:

- C_L(α) with camber offset and stall roll-off (wing-level stall at ~20°,
  the section law's earlier roll-off stays for the local field), floored
  just under zero (a fabric wing pushed from above tucks, it does not fly
  inverted), blending to flat-plate normal force past stall so a stalled
  wing is a parachute, not a free-faller.
- C_D = parasitic + brake flap drag + induced (projected aspect ratio) +
  post-stall plate drag; bluff-body pilot drag acts at the pilot node
  against the pilot's own relative wind.
- In free flight the bounded final-Cp solve preserves the section field's
  total force and natural pitch moment. It uses the half-wing polar terms
  only for asymmetric steering, then adds any missing positive viscous drag
  as area-weighted, dissipative skin traction. Replacing the live pressure
  force and moment with a second polar target made the two feedback paths
  fight: pressure, stress and line slack oscillated in phase until collapse.
- The tunnel still uses the hang-line anchor and prescribed polar force/pitch
  retrim because it is a measurement instrument, not a freely moving coupled
  system. The explicit legacy increment-and-clamp mode remains a regression
  oracle only.
- Relative wind is measured against the **canopy's** own mean velocity —
  against the whole system's (pilot-dominated) mean, the canopy's pendulum
  swing was invisible to the air and undamped. Per-node feedback remains
  catastrophic (see below); the canopy-mean is the workable middle.
- `StepSettings::dampingReferenceVelocity`: free-flight damping decays node
  velocity toward the system's bulk velocity, not toward zero. Absolute
  damping at glide speed is a fake drag several times the real drag budget
  — it, not the aerodynamics, would set the trim speed.
- Pilot-plus-harness mass is explicit. TrimmedGlide keeps the estimated path
  direction but sizes q from achieved bounded wing support versus total
  dynamic-node weight, then runs eight co-moving quasi-static load frames and
  recalibrates on the retained geometry. DropFromRest remains an untouched
  pre-inflated zero-velocity release.
- The canopy carries a potential-flow planform estimate of aerodynamic added
  air mass as scalar solver inertia. Its artificial gravity is cancelled per
  node, so it changes acceleration without changing system weight, launch
  support or static line load. Omitting it left a few kilograms of cloth
  reacting against a 90 kg pilot and turned small pressure changes into large
  catch-and-release pulses.
- `StepSettings::cableConstraintSweepPairs` adds deterministic serial
  graph-depth reverse/forward passes over the complete suspension load path:
  unilateral line cables plus bilateral canopy, harness and brake-handle ties.
  Some passes are interleaved with cloth iterations and most close the path
  after the final cloth sweep. Playground free flight uses 96 pairs at 30x2;
  zero is the exact historical/tunnel bypass.

Three earlier lessons that still stand:

- **Never feed per-node velocity back into the load.** The canopy talks
  itself flat (gnuC2: 10.4 m of span down to 5.2 m).
- **Dynamic pressure scales with the full relative wind, not its in-plane
  component** — or the arced tips are undercharged.
- **Brake lines need a handle per side**, or their cables pull the tips
  together.

### Controls

`Pressure` is now q in pascals; `Lift` is replaced by `Angle`, the angle of
attack in degrees (0–15, default 6). The brake sliders take the VIEWER's
left and right (the solver's "left" cascade sits at negative mesh x, which
the camera shows on the viewer's right; the crossover lives in
`setBrakePull` alone). **Reset** rebuilds the wing at its rest pose from
the retained mesh; **Fly mode** turns the cursor over the view into the
brake input — top centre hands-up, straight down both brakes, toward a
side releases the opposite brake, Esc leaves. In free flight the Angle
slider is ignored — the wing finds its own trim, and tilting the oncoming air is a
hillside, not an angle-of-attack control; at the pinned default of 6° it
trimmed the flying wing into its stall. Note the sign-convention fix
changed the pinned numbers slightly too (the wind now genuinely comes from
below at positive angles): gnuC2 settles at 10.78 m span, +4.3% volume,
778 N up at the 6° default. The GPU prototype takes its face
pressures as a host upload rather than mirroring this model in a kernel — a
per-face constant is 63 KB a frame — so on that path the field is only as
fresh as the last readback. That does not affect what its timings measure.

The colour selector keeps the three pressure quantities separate:

- **Cell resolved p** is the spatially uniform gauge pressure applied inside
  each cell. The solver also retains the raw `mRT/V` gas pressure. In healthy
  flight the applied value uses the calibrated ram field as a prior; authority
  shifts continuously to the finite-mass value as the bay loses volume, mouth
  opening or ram recovery. Faces in one cell therefore have one colour
  spatially, while the value changes over time and from cell to cell. Its map
  is signed because a transient under-pressure is possible.
- **External Cp** is the signed, dimensionless outside aerodynamic pressure
  coefficient on a fixed physical -3..1 scale.
- **Fabric Δp** is the actual signed triangle load in pascals,
  `p_inside - q*Cp`. It varies around a cell because the outside Cp varies;
  a uniform internal pressure does not imply a uniform fabric load.

The signed maps are blue below zero, neutral at zero and red above zero.
Keeping these as separate fields avoids presenting the upper-surface suction
band as if it were a spanwise difference in cell inflation.

The Solver control sits on a row of its own. Both existing rows are already
full edge to edge at a near-maximised window — the filter row runs to the
legend, and the wing row's four sliders stretch until whatever follows them
is pushed off the window — so a widget added to either vanishes rather than
wraps. (That the wing row already overflows, leaving Right brake and Pause
off-screen at 1555 px, predates this work and is untouched.)
