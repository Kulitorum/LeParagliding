# softwing_core (vendored)

XPBD soft-body physics core vendored from the author's SoftWingLab project
(`C:\CODE\SoftWingLab`, commit `dd24fe05b8ae280b857d743f1ded95a8de477d4d`,
2026-07-25), relicensed into this repository under GPL-3.0 by its owner.

This is the dependency-free simulation subset only — soft body (XPBD
distance/cable constraints, per-face pressure), orthotropic membrane,
contact, lumped pneumatics, canopy container, and suspension line
cascades. The aerodynamics, flight-state, and test-fixture layers are
deliberately not vendored; `aerodynamics.h`, `flight_state_access.h`,
and `aerodynamic_test_access.h` ride along headers-only because the
vendored .cpp files define (never call) methods of their access
structs.

The Studio uses this core for the Playground tab's toy live-wing simulation
(inflation and line pulling); it makes no engineering claims — see
`wingDesignConfidenceBoundary` in SoftWingLab.

## No longer a verbatim copy

These files started as an unmodified copy so they could be re-synced from
SoftWingLab by copying over them. **That contract has been dropped**, by the
owner's decision, and `soft_body.{h,cpp}` has diverged: the Playground's wing
is a mass-spring cloth of distance constraints with no membrane elements, so
none of the vendored parallel machinery applied to it and 98% of every frame
sat in a single-threaded loop. See `docs/xpbd-performance.md` for the
measurements and the reasoning.

What changed, all of it modelled on the membrane paths already here and
carrying the same reproducibility contract (bit-identical at any worker
count; `workerThreads == 0` still selects the untouched serial sweep):

- `ConstraintColouring` / `solveConstraintsColoured` — a coloured parallel
  sweep for the distance/cable constraints.
- `SolveNode` and the packed sweep — the constraint iteration loop runs on a
  32-byte hot copy of node position and inverse mass when nothing else in the
  substep moves nodes.
- `projectDistanceConstraint` — the projection itself, shared by both sweeps,
  with three of its five divisions removed.
- `constraintColouringReport` / `constraintColouringView` — observation only,
  for the benchmark and for the GPU backend in `tools/`.

Re-syncing from SoftWingLab now means merging rather than copying. Anything
LEparagliding-specific still belongs in `src/gui` / `src/engine` instead.
