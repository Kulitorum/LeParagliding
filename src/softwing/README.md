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

Files are copied **unmodified** so they can be re-synced from SoftWingLab
by copying over them; keep local changes out of these files and put any
LEparagliding-specific glue in `src/gui` / `src/engine` instead. The
Studio uses this core for the Playground tab's toy live-wing simulation
(inflation and line pulling); it makes no engineering claims — see
`wingDesignConfidenceBoundary` in SoftWingLab.
