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

The Studio uses this core for the Playground tab's live-wing wind tunnel
(inflation, shape-fidelity instrumentation, line pulling — see
`docs/playground-shape-analysis.md`). The claim boundary is stated there:
relative, structural signals of one design and comparisons between design
revisions, never certified absolute aerodynamics — see
`wingDesignConfidenceBoundary` in SoftWingLab.

## No longer a verbatim copy

These files started as an unmodified copy so they could be re-synced from
SoftWingLab by copying over them. **That contract has been dropped**, by the
owner's decision, and `soft_body.{h,cpp}` has diverged. The Playground still
defaults to its calibrated mass-spring distance-truss skin, but can now build
the skin from the generic membrane elements as an explicitly experimental
material mode. See `docs/xpbd-performance.md` for the parallel-solver history
and `docs/playground-shape-analysis.md` for the material-mode boundary.

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
- `StepSettings::dampingReferenceVelocity` — what the velocity damping
  decays toward. Defaults to zero, which reduces bit-for-bit to the
  historical absolute damping; the Playground's free flight sets it to the
  system's bulk velocity so fabric ringing is damped without the damping
  acting as a fake drag on the glide.
- `StepSettings::cableConstraintSweepPairs` — optional serial reverse/forward
  suspension-load-path passes distributed before and after the general
  structural sweeps. They include unilateral cables and the bilateral canopy
  and harness ties, conditioning deep load propagation without paying for
  extra cloth iterations. Zero bypasses the path exactly and preserves
  historical arithmetic; Playground free flight currently requests 96 pairs.
- `OrthotropicMembraneMaterial::compressionStiffnessRatio` — an SPD-safe
  `D*K*D` compression reduction. The default `1.0` bypasses the new branch and
  preserves the original bilateral stiffness matrix exactly; the shear scale
  is the geometric mean of the two normal-direction scales.
- `DihedralBendingConstraint` — a true signed four-node hinge on two adjacent
  membrane triangles. Zero workers preserve serial insertion order; threaded
  callers use a deterministic node-disjoint four-node colouring. Hinge state
  is included in rollback/state persistence, and hinges are deliberately
  omitted at skin boundaries, degenerate faces and non-manifold/inconsistently
  wound edges.
- `StepSettings::updateMembraneSolverDiagnostics` — permits hosts to omit the
  post-solve stored residual/resultant estimate. It defaults on for core
  compatibility and changes no physics; Playground turns it off because its
  strain/energy diagnostics are evaluated directly from published geometry.

The generic membrane constraint-space damping field predates the Playground
material mode. Nonzero values are stable for the small isolated coupons it was
written for, but are not yet accepted for the mixed full-wing network where
ribs, lines and membranes repeatedly move shared nodes. The Playground
therefore defaults material damping to zero and diagnoses any explicit
nonzero setting as experimental; changing that requires a separate coupled
damping design, not a hidden material retune.

Re-syncing from SoftWingLab now means merging rather than copying. Anything
LEparagliding-specific still belongs in `src/gui` / `src/engine` instead.
