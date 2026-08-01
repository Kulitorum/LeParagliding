#ifndef LEP_PLAYGROUND_SIM_H
#define LEP_PLAYGROUND_SIM_H

#include <softwing/soft_body.h>

#include <QByteArray>
#include <QString>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

// The Playground's toy live-wing model, independent of the widget that
// displays it: mesh parsing, mesh refinement, SoftBody assembly and the
// per-frame step. Split out of playground_page.cpp so the headless
// benchmark (tools/softwing_bench.cpp) drives the exact same body the GUI
// does — a solver measurement is only worth anything if it is measuring the
// real wing.
namespace lep::playground {

inline constexpr double metresPerMillimetre = 0.001;
inline constexpr double fabricArealDensity = 0.045;   // kg/m^2
// Tuned on the Swoop harness: at 1e-8/1e-9 with 4 substeps and 25
// iterations the wing settles at +23% volume under 80 Pa instead of
// creeping past its own fabric.
inline constexpr double skinCompliance = 1.0e-8;      // XPBD, m/N
inline constexpr double lineCompliance = 1.0e-9;
// Heavy enough that ~1 kN of lift through the cascade keeps a workable
// mass ratio for the solver (5 g junctions let the wing creep skyward).
inline constexpr double lineJunctionMass = 0.05;      // kg
inline constexpr double anchorBandMetres = 0.3;
inline constexpr double gravityMetresPerSecondSquared = 9.80665;
// How far the pilot's mass hangs below the carabiners.
inline constexpr double pilotDropMetres = 0.45;
// Free-flight fabric/attitude damping, applied RELATIVE to the system's
// bulk velocity (see StepSettings::dampingReferenceVelocity), so it quiets
// fabric ringing and canopy pitch flapping without braking the glide
// itself. It can therefore be much stronger than an absolute damping could
// ever have been: at this value the canopy's pitch mode — which fed the
// polar force pass its angle of attack and got a whipping force back —
// stays put, while the pendulum (period of several seconds) is merely
// well-damped rather than dead.
inline constexpr double systemDampingPerSecond = 1.5;
// Time constant of the low-pass on the wing-level angle of attack that
// feeds the polar. Real unsteady aerodynamics lags geometry too (the wake
// needs time to adjust); here the lag is also what keeps the imposed force
// from following every fabric wobble with full authority.
inline constexpr double alphaFilterSeconds = 0.25;
inline constexpr double lineAttachRadiusMetres = 0.12;
// Cells across the section in each rib strut. One more per resolution step,
// so ribs get denser along with the skin. Kept small deliberately: every
// interior node is one more thing that can buckle out of the rib's plane.
inline constexpr int defaultRibLayers = 4;
// Extra chord stations per outline segment. Cells must come out smaller
// than an airfoil hole or dropping them by their middle misses the holes
// entirely — at one station per outline node only 1% of cells fell in a
// hole, against holes covering 22% of the rib.
inline constexpr int defaultRibStationSplit = 3;
inline constexpr double simulationTimeStep = 1.0 / 60.0;
inline constexpr double defaultAngleOfAttackDegrees = 6.0;

// How the frame's constraint-solving budget is spent. XPBD convergence is
// governed by substeps x iterations, but the two are not worth the same: a
// substep re-linearises every constraint about a fresh state, an iteration
// only re-solves the same linearisation. Measured on gnuC2 (see
// docs/xpbd-performance.md), 30x2 settles closer to a converged reference
// than 4x30 does, for less time — so the ladder below buys accuracy with
// substeps and keeps iterations low throughout.
struct SolverQuality
{
    const char *label;
    int substeps;
    int iterations;
};
inline constexpr SolverQuality solverQualities[]{
    {"Fast", 15, 2},
    {"Balanced", 30, 2},
    {"Accurate", 60, 4},
};
inline constexpr int defaultSolverQuality = 1;   // Balanced
inline constexpr int simulationSubsteps =
    solverQualities[defaultSolverQuality].substeps;
inline constexpr int simulationIterations =
    solverQualities[defaultSolverQuality].iterations;
inline constexpr double substepSeconds =
    simulationTimeStep / simulationSubsteps;
inline constexpr std::size_t noConstraint =
    std::numeric_limits<std::size_t>::max();

// Which skin a quad belongs to, matching the engine's surfaceNames order.
// Meshes written before the tag existed report everything as Extrados.
enum class SimSurface
{
    Extrados,
    Vent,
    Intrados,
    // Drawn from the constraint structure rather than from exported quads:
    // the rib webs (loop plus spokes to the rib centre) and the internal
    // V/H-rib sheets. They carry no pressure — they are the load path
    // between the skin and the lines, which is where the interesting
    // stress lives.
    Rib,
    Strap,
    Count,
};
inline constexpr int simSurfaceCount = static_cast<int>(SimSurface::Count);
// Only the three skin surfaces come from the mesh file's tags.
inline constexpr int simExportedSurfaceCount = 3;

struct SimLine
{
    softwing::Vec3 a;
    softwing::Vec3 b;
    bool brake = false;
    // Row plan from the engine's line labelling, 1..6 = A..F (brakes
    // report 6). 0 when the mesh predates the field.
    int plan = 0;
};

struct SimStrap
{
    std::vector<softwing::Vec3> a;
    std::vector<softwing::Vec3> b;
};

struct SimMesh
{
    std::vector<softwing::Vec3> nodes;
    std::vector<std::array<int, 4>> quads;
    // Parallel to quads.
    std::vector<SimSurface> quadSurfaces;
    std::vector<std::vector<int>> ribLoops;
    // Parallel to ribLoops: closed hole outlines in the rib's plane.
    std::vector<std::vector<std::vector<softwing::Vec3>>> ribHoles;
    std::vector<SimStrap> straps;
    std::vector<SimLine> lines;
};

struct LineSegment
{
    std::size_t a = 0;
    std::size_t b = 0;
    bool brake = false;
    std::size_t constraint = noConstraint;
    // Row plan carried over from the SimLine, 0 when unknown.
    int plan = 0;
};

// A brake line from the pilot to the top of one brake cascade. Pulling the
// brake shortens it, so the pull is a force between two masses rather than a
// node being dragged to a prescribed place.
struct BrakeLine
{
    std::size_t constraint = 0;
    double restLength = 0.0;
    bool left = false;
};

// One drawn triangle: the skin quads, plus the rib webs and V/H sheets
// that are otherwise constraint-only.
struct RenderFace
{
    std::array<std::size_t, 3> nodes{};
    SimSurface surface = SimSurface::Extrados;
    // Constraint index per side, noConstraint where the side is not
    // constrained (a rib fan's perimeter is, its rails are not).
    std::array<std::size_t, 3> edges{};
};

// A rib's chord, kept as node indices so it can be re-read from the live
// deformed pose rather than the rest one: the section's angle of attack is
// what drives its load, and that changes as the wing moves.
struct RibChord
{
    std::size_t leadingNode = 0;
    std::size_t trailingNode = 0;
    // A skin node at ~40% chord on the extrados: forward of the flap, so
    // no brake pull can move it. leadingNode->referenceNode is therefore
    // an attitude line the pilot's hands cannot rotate, which is what the
    // wing-level angle of attack is measured from. The full chord still
    // sets the chordwise pressure distribution and the section's own
    // shape — it is only the ANGLE that must not follow the flap.
    std::size_t referenceNode = 0;
    // Rib plane normal at rest, i.e. the local span direction. The section's
    // angle of attack is measured in the plane perpendicular to it.
    softwing::Vec3 spanAxis;
    double restChordLength = 0.0;
};

// Where a skin triangle sits on the wing, so a chordwise pressure
// distribution can be evaluated for it. Fixed at build time — the topology
// does not move, only the geometry does.
struct FaceAero
{
    std::uint32_t rib = 0;
    // Which pneumatic cell (bay between two adjacent ribs) the face
    // belongs to. Meaningless when SimBody::cells is empty.
    std::uint32_t cell = 0;
    // 0 at the leading edge, 1 at the trailing edge.
    float chordFraction = 0.0F;
    bool upperSurface = false;
};

// One pneumatic cell: the bay between two spanwise-adjacent ribs. The
// cells carry the wing's internal air as a per-cell gauge pressure state
// instead of the old blanket assumption that every cell sits at ram
// pressure: a cell is fed through its leading-edge intake only while that
// intake actually faces the airflow, neighbouring cells exchange air
// through the rib's cross-port holes, and a cell squeezed below its rest
// section reacts with extra pressure. Together those are what let a
// collapsed side re-inflate — the old open-loop stamp held a fold shut
// forever, because a flat cell was stamped exactly like an inflated one.
struct SimCell
{
    // The two bounding ribs, as indices into ribChords, in span order.
    std::array<std::size_t, 2> ribs{0, 0};
    // Skin triangles of this cell's leading-edge intake (Vent surface).
    std::vector<std::size_t> ventFaces;
    // Rest magnitudes the live signals are measured against.
    double restVentArea = 0.0;      // m², sum of vent face areas
    // The volume of air the designed mouth scoops per metre of travel, m²
    // — its rest-pose area vector projected on the build-time airflow. The
    // live scoop over this one is the speed air enters at, so the designed
    // pose counts as fully open and no mouth is charged for the cosine it
    // was drawn with.
    double restVentProjection = 0.0;
    // The mouth's rest-pose opening, m²: the LENGTH of its summed area
    // vector, which for any patch is the area of the flat opening its rim
    // bounds. Rotating the wing cannot change it and folding the mouth
    // shut takes it to zero, which is exactly the difference between "the
    // mouth points somewhere else now" and "the mouth is closed".
    double restVentAperture = 0.0;
    double restSectionArea = 0.0;   // m², mean of the two rib loop areas
    double restVolume = 0.0;        // m³, section area × rib spacing
    // Cross-port area through the rib shared with the NEXT cell in span
    // order, summed from that rib's hole outlines. Zero when the design
    // has no holes there — an unported rib genuinely blocks cross-flow.
    double portAreaToNext = 0.0;
};

struct SimBuildOptions
{
    bool detailedRibs = false;
    int ribLayers = defaultRibLayers;
    int ribStationSplit = defaultRibStationSplit;
};

// One fabric-contact candidate: a skin node against a skin triangle or a
// suspension-line segment, found by the once-per-frame detection pass and
// re-projected every substep (see applyFabricContact in the .cpp).
struct ContactCandidate
{
    std::uint32_t node = 0;
    // Skin triangle index, or an index into SimBody::lineSegments.
    std::uint32_t item = 0;
    bool line = false;
    // Which side of the triangle the node approached from at detection,
    // so a fast crossing inside one frame cannot flip the push direction.
    float side = 1.0F;
};

// Scratch for the fabric/line contact pass: sorted cell grids rebuilt per
// frame, the frame's candidate list, and the build-time preparations —
// the skin's unique node set, the grid cell size, and the pairs that sit
// inside the contact thickness in the REST pose (designed-adjacent
// fabric, e.g. around line attachments, which contact must never fight).
struct ContactScratch
{
    std::vector<std::pair<std::uint64_t, std::uint32_t>> triangleCells;
    std::vector<std::pair<std::uint64_t, std::uint32_t>> segmentCells;
    std::vector<ContactCandidate> candidates;
    std::vector<std::uint64_t> restExclusions;   // sorted pair keys
    std::vector<std::uint32_t> skinNodes;
    std::vector<float> itemSpeed;   // per-item scratch, m/s
    // Capture-inflated item AABBs (xyz min, xyz max), triangles first,
    // then segments — the cheap reject before any closest-point math.
    std::vector<std::array<float, 6>> itemBounds;
    std::vector<std::pair<std::uint64_t, std::uint32_t>> nodeCells;
    std::vector<std::uint64_t> pairScratch;
    double cellSize = 0.05;
    bool prepared = false;
};

// A built wing: the solver body plus everything the view and the controls
// index into it by.
struct SimBody
{
    std::unique_ptr<softwing::SoftBody> body;
    std::size_t skinTriangleCount = 0;
    // Parallel to the skin triangles first, then the rib and strap faces.
    std::vector<RenderFace> renderFaces;
    std::vector<std::size_t> topFaces;
    std::vector<LineSegment> lineSegments;
    std::vector<BrakeLine> brakeLines;
    // The pilot, or noConstraint when the mesh carried no suspension lines
    // to hang one from. Nothing in the body is pinned: wing and pilot both
    // fly, and the pair is translated back to the origin after each step so
    // the view keeps them (see recentreSystem).
    std::size_t pilotNode = noConstraint;
    double pilotMass = 0.0;
    // Nodes below this index belong to the canopy (skin, rib interiors);
    // line junctions, handles and the pilot come after. The aerodynamic
    // relative wind is measured against the canopy's own mean velocity,
    // not the whole system's: the system's is dominated by the pilot, and
    // against it the canopy's pendulum swing is invisible to the air —
    // which removed exactly the aerodynamic damping that keeps a real
    // canopy from whipping over on its lines. A mean over thousands of
    // nodes keeps single-node flutter out of the loop, which is what made
    // the earlier per-node feedback catastrophic.
    std::size_t canopyNodeCount = 0;
    // Parallel to the skin triangles. Empty when the mesh carries no rib
    // loops to hang a chord off, which falls the pressure field back to a
    // uniform one.
    std::vector<FaceAero> faceAero;
    std::vector<RibChord> ribChords;
    // Each rib's outline loop as body node indices, parallel to ribChords.
    // The shape instrumentation fits the rest section onto the live one
    // through these (see playground_metrics.h).
    std::vector<std::vector<std::size_t>> ribLoopNodes;
    // The pilot-end junctions: fixed to the world in the wind tunnel, tied
    // to the pilot in free flight. Line segments with an endpoint here are
    // the riser level, which is where per-row loads are read.
    std::vector<std::size_t> carabinerNodes;
    // Skin nodes the suspension lines tie into, deduplicated. The polar
    // correction and the pitch-trim couple are applied here: this is the
    // load path the canopy is built to carry point loads on, and pressing
    // system-level forces onto the fabric instead was measured denting the
    // nose and folding the tips. Empty when the mesh has no lines.
    std::vector<std::size_t> lineAttachmentNodes;
    // The designed skin's face area vectors, parallel to the first
    // skinTriangleCount triangles. Half the sum of |A.w| over a closed
    // surface is its frontal area along w, so these are the reference the
    // fabric-drag term measures the live shape's excess against.
    std::vector<softwing::Vec3> restFaceAreas;
    // The two ribs whose leading edges sit furthest out along the rest span
    // axis. The live span axis is read between them, so the wing-level angle
    // of attack follows the wing's real attitude rather than its rest one.
    std::array<std::size_t, 2> spanTipRibs{0, 0};
    // Rest-pose PROJECTED planform (the shadow the wing casts on the ground
    // plane) and the aspect ratio built from it. Projected, not wetted: the
    // induced-drag law and the lift reference area both want the planform,
    // and using the upper skin's wetted area under-read the aspect ratio by
    // a third and over-read the reference area by the arc's cosine losses.
    double planformArea = 0.0;
    double aspectRatio = 5.0;
    // Per-rib section lift coefficient from the most recent load stamp;
    // shapes the chordwise pressure distribution.
    std::vector<double> ribLiftCoefficient;
    // Which half-span each rib belongs to, fixed at build time from its
    // rest station along restSpanAxis: 0 is the low-span (negative mesh x)
    // half, which is the solver's LEFT and the side SimControls::brakeLeft
    // acts on. Skin faces inherit their nearest rib's half, so the
    // partition is a property of the DESIGN and never migrates: a face
    // jittering across the centreline would otherwise swap which brake's
    // polar loads it, several times a second.
    std::vector<std::uint8_t> ribHalf;
    // Rest-pose projected planform of each half, m². Its sum is
    // planformArea, so two half-passes at equal coefficients impose
    // exactly what the single wing-level pass imposed.
    std::array<double, 2> halfPlanformArea{0.0, 0.0};
    // Low-passed departure of each half's angle of attack from the
    // wing-level one, radians, and each half's dynamic pressure as a
    // ratio of the wing's. Both come from the canopy's rigid-body spin,
    // and between them they are a free-flying wing's whole roll and yaw
    // damping: a rolling wing has one half descending into the air and
    // the other rising out of it, a yawing one has a tip running forward
    // and a tip running back, and until the polar was split per half
    // nothing in the imposed force knew either. Both are 0 and 1 on a
    // wing that is not rotating, so a symmetric case is untouched.
    std::array<double, 2> alphaHalfDeviationRadians{0.0, 0.0};
    std::array<double, 2> halfDynamicPressureRatio{1.0, 1.0};
    // The pull the WING has, left then right, as against the pull
    // SimControls asks for. A hand has a finite speed, and the two are
    // not the same thing here for a reason worth spelling out: the
    // controls are sampled in WALL-CLOCK time and the wing lives in
    // SIMULATED time, and on a real wing at 30x2 those run two to three
    // times apart. A brake movement the pilot makes over three quarters
    // of a second therefore reaches the wing as a third of a second of
    // input — a snatch the pilot never made, and one that surges the
    // wing hard enough to collapse it. Rate-limiting the pull in
    // simulated time is what puts the two back on one clock.
    //
    // Deliberately in simulated time rather than off the measured frame
    // rate: a limiter that read the wall clock would make the physics
    // depend on how busy the machine is, and the headless bench would
    // stop reproducing the GUI.
    std::array<double, 2> brakeApplied{0.0, 0.0};
    // And the pull the POLAR sees, low-passed. The geometric side of a
    // brake is immediate — the line shortens, the trailing edge comes
    // down, the solver carries it — but the aerodynamic side is not: a
    // deflected trailing edge changes the section's circulation only as
    // fast as the wake can adjust, which is the same lag the wing-level
    // angle of attack already runs on. Without it the turning couple was
    // internally inconsistent, its camber difference following the
    // pilot's hand instantly while the rotation-derived differences it
    // is balanced against lagged by a quarter second — so a fast release
    // reversed the couple's sign for exactly that quarter second.
    std::array<double, 2> brakeFilteredMetres{0.0, 0.0};
    // The pneumatic cells in span order, and their internal gauge pressure
    // in pascals — the state applyPressure integrates each frame. Empty
    // until the first stamp initialises it (cells pre-inflated to their
    // ram target, so a fresh build behaves exactly like the old model).
    std::vector<SimCell> cells;
    std::vector<double> cellPressure;
    // Fabric/line contact working set; inert until the option is on.
    ContactScratch contact;
    // Chord fraction of the designed hang line: the chord station the
    // carabiners sit under at rest. The imposed aerodynamic resultant
    // acts here at trim and travels aft/forward of it as the angle of
    // attack rises/falls (see kAnchorTravelPerRadian), which is what
    // makes pitch statically stable at the designed trim angle.
    double resultantChordFraction = 0.28;
    // The wing-level angle of attack at rest with the build-time controls:
    // the angle the designed line geometry trims the wing to.
    double alphaTrimRadians = 0.1;
    // The slider angle the body was built at. The tunnel's prescribed
    // angle of attack is alphaTrimRadians shifted by however far the
    // slider has moved since — tilting the airflow tilts the rest pose's
    // angle with it, degree for degree.
    double builtAngleOfAttackDegrees = defaultAngleOfAttackDegrees;
    // The steady glide-path angle the polar predicts for this wing at its
    // in-flight trim, from the build-time fixed point. Used to launch the
    // system on the glide instead of at a dead stop.
    double glideAngleRadians = 0.0;
    // Slow-averaged angle of attack: the washout reference the pitch
    // anchor damps against. NaN until seeded.
    double alphaSlowRadians =
        std::numeric_limits<double>::quiet_NaN();
    // Low-passed wing-level angle of attack, the polar's actual input.
    // NaN until the first free-flight force pass seeds it.
    double alphaFilteredRadians =
        std::numeric_limits<double>::quiet_NaN();
    // Its rate of change, for the anchor's pitch-rate (Cmq) term.
    double alphaRateRadiansPerSecond = 0.0;
    // Retrim bookkeeping: how far the applied pressure field's resultant
    // and pitch moment landed from what the solve asked for (clamping can
    // eat into both). Diagnostics only.
    softwing::Vec3 lastForceResidual;
    double lastPitchResidual = 0.0;
    // Filled in by each free-flight force pass, for reporting and the HUD.
    softwing::Vec3 lastAeroForce;
    double lastLift = 0.0;
    double lastDrag = 0.0;
    // The fabric-drag resultant this frame, and the extra frontal area it
    // came from. Zero on a wing holding its designed shape; it is what a
    // folded canopy has that a flying one does not.
    double lastFabricDragNewtons = 0.0;
    double lastExcessFrontalArea = 0.0;
    double lastGlideRatio = 0.0;
    double lastAlphaDegrees = 0.0;
    double lastAirspeed = 0.0;
    // How far the AIR has travelled past the wing since the build, in the
    // wing's own frame. Free flight re-centres the whole system on the
    // origin every frame and the tunnel never moves at all, so neither
    // mode leaves any trace of travel in the node positions — this is the
    // only record that the wing is going anywhere, and it is what the air
    // motes are drawn against.
    softwing::Vec3 airTravel;
    // Rest-pose angle from the leading-edge-to-40%-extrados attitude line
    // to the true chord line, radians. The reference node sits above the
    // chord by the aerofoil's own thickness and camber, so the two lines
    // are tens of degrees apart; subtracting this makes the measured
    // angle of attack read exactly as it did off the full chord in the
    // rest pose, while still being immune to what a brake does aft of it.
    double attitudeOffsetRadians = 0.0;
    // Rest-pose mean chord and span directions, used to place the airflow.
    softwing::Vec3 restChordDirection{0.0, 1.0, 0.0};
    softwing::Vec3 restSpanAxis{1.0, 0.0, 0.0};
    softwing::Vec3 boundsLow;
    softwing::Vec3 boundsHigh;
    // The interactive grab: a kinematic anchor node tied to one line
    // junction by a soft cable, so the mouse can pull the cascade and the
    // pull is a readable force rather than a teleported node. noConstraint
    // when no grab has ever been made on this body.
    std::size_t grabAnchorNode = noConstraint;
    std::size_t grabConstraint = noConstraint;
    std::size_t grabbedNode = noConstraint;
};

// Live inputs to a step. Held apart from the body so the benchmark can
// drive the same wing with a different worker count or a profile attached
// without touching the build.
struct SimControls
{
    // Dynamic pressure q = ½ρV². It sets the whole load field, because in
    // this model both sides of the fabric are referred to it: the cell
    // interior is fed toward ram (stagnation) pressure — per cell through
    // its intake and cross-ports when the cell model is on, pinned at
    // exactly q when it is off — and the outside of any face is q·Cp.
    // 80 Pa is 41 km/h, an ordinary trim speed. A real canopy runs
    // 0.7–2.3 mbar this way, not the 0.1 bar that gets quoted — 0.1 bar
    // would need 460 km/h.
    double pressurePascal = 80.0;
    // Angle of the airflow to the wing's rest chord, in degrees. Replaces
    // the old fake follower "lift" force: the load now comes out of the
    // pressure field, and this is what tilts that field. In free flight it
    // acts as a trimmer: it shifts the angle the wing's pitch trim seeks.
    double angleOfAttackDegrees = defaultAngleOfAttackDegrees;
    // Let the whole system fly: gravity on, nothing pinned, the pilot's
    // mass free to swing under the canopy, and the pair translated back to
    // the origin after each step. EXPERIMENTAL and off by default — the
    // wing does not yet trim, it pitches over within a couple of seconds
    // (see docs/xpbd-performance.md). With it off the pilot is pinned, which
    // is the behaviour the tab has always had.
    bool freeFlight = false;
    // Wind-tunnel loading: impose the wing-level polar force pass in
    // pinned mode too, so the canopy hangs in its lines against a
    // realistic ~1 kN resultant instead of the pressure field's badly
    // under-read lift, and every line-load number means something. Off by
    // default so the bench's timing baselines and pose checksums are
    // untouched; the GUI turns it on. Ignored in free flight, which has
    // its own force pass.
    bool flightLoad = false;
    // Per-cell internal air model: intake gating, cross-port flow between
    // neighbouring cells, and a volume-deficit pressure response. On a
    // healthy wing the stamped field converges to exactly the old
    // uniform-ram one; it differs only in distress (a tucked cell seals
    // its own intake, a collapsed side is re-fed by its neighbours). Off
    // reproduces the old blanket ram-pressure stamp bit for bit.
    bool cellPressureModel = true;
    // Multiplier on the rib cross-port flow — the path by which an
    // inflated cell re-feeds a collapsed neighbour. 1 is the area the
    // design actually declares; higher is deliberately unphysical, a
    // hand on the one mechanism that re-inflates a sealed cell so its
    // effect can be seen rather than argued about.
    double crossPortGain = 1.0;
    // Fabric self-contact plus fabric-versus-line contact, as a runtime
    // option: folded fabric stops passing through itself and through the
    // suspension lines, which is what lets a cravat clear. This is the
    // Playground's own thin-cloth pass (once-per-frame detection, per-
    // substep projection), NOT the engine's certified contact machinery —
    // that pipeline re-enumerates every vertex-triangle and edge-edge
    // combination serially in every constraint iteration, which is five
    // orders of magnitude over the frame budget on a real wing, and once
    // a pair is registered it cannot be turned off. Off skips the pass
    // entirely and steps exactly as before.
    bool fabricContact = false;
    double brakeLeft = 0.0;
    double brakeRight = 0.0;
    int substeps = simulationSubsteps;
    int constraintIterations = simulationIterations;
    // 0 keeps the core's serial sweep. See playgroundWorkerThreads().
    unsigned workerThreads = 0;
    softwing::StepPerformanceProfile *performanceProfile = nullptr;
};

// Worker count for the Playground's step. The core never reads the machine's
// core count itself (its results are reproducible per worker count, so the
// count has to be an explicit input), which leaves the choice here.
//
// Physical cores, not logical: the sweep is a barrier every colour and SMT
// siblings only add scheduling jitter to that — measured, 12 and 16 workers
// are markedly slower than 6 on an 8-core/16-thread part. Two are held back
// for the UI and the driver, which is also where the measured optimum sat.
[[nodiscard]] unsigned playgroundWorkerThreads();

std::optional<SimMesh> parseSimMesh(const QByteArray &data, QString &error);

// Splits every skin quad into factor x factor sub-quads, bilinear on the
// quad's corners, and refines the rib loops to the same spacing so their
// webs keep matching the skin. The engine's mesh is a decimated sampling
// of the exact ballooning law, so this adds no shape detail — it buys the
// XPBD solver a finer cloth discretization (factor^2 the triangles) at
// factor^2 the cost per step.
SimMesh refineSimMesh(const SimMesh &mesh, int factor);

// Assembles the solver body. Leaves the pressure field stamped for the
// given controls, so the caller can step immediately.
SimBody buildSimBody(const SimMesh &mesh,
                     const SimBuildOptions &options,
                     const SimControls &controls);

// Restamps the whole per-face pressure field. Cheap enough to redo every
// frame, which it must be: the field follows the wing's live attitude.
void applyPressure(SimBody &sim, const SimControls &controls);

// Net force from the pressure field alone, in newtons, for the current
// pose. A sanity check with a number attached: a wing carrying pilot and
// kit has to make roughly 1 kN.
[[nodiscard]] softwing::Vec3 aerodynamicForce(const SimBody &sim);

// The wing-level polar sample the free-flight force pass imposes: the
// wing's live angle of attack, the finite-wing lift and drag coefficients
// at it, and the frame the force acts in. Valid only when the mesh carried
// rib chords and the relative wind is not degenerate.
struct WingAeroSample
{
    bool valid = false;
    double dynamicPressure = 0.0;    // from the live relative wind, capped
    double airspeed = 0.0;
    double alphaRadians = 0.0;
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;    // profile + lines + induced
    softwing::Vec3 windDirection;    // unit, pointing downstream
    softwing::Vec3 liftDirection;    // unit, normal to the wind
    softwing::Vec3 spanAxis;         // unit, live, oriented like the rest one
};
[[nodiscard]] WingAeroSample sampleWingAero(const SimBody &sim,
                                            const SimControls &controls);

// The free-flight force pass. Integrating an inviscid pressure field over a
// lifting body recovers full leading-edge suction and no viscous loss —
// d'Alembert's paradox — so its resultant is lift-poor and thrust-rich, and
// no suction tuning fixes it (lift and spurious thrust scale together). So
// the pressure field is not asked for the system force at all: its net
// resultant is cancelled and a classical finite-wing polar (C_L(α) with
// stall roll-off, C_D0 + C_L²/(π·AR·e)) is imposed in its place, spread
// over the skin by area, plus bluff-body pilot drag on the pilot node. The
// pressure field keeps the job it is good at — shaping the fabric — and
// the polar sets the trim and the glide. Clears the body's external forces
// and replaces them, so it owns that channel; call it once per frame,
// immediately before stepping.
void applyAerodynamicForces(SimBody &sim, const SimControls &controls);

// Lift and drag resolved along the airflow, and the glide ratio they imply.
// In free flight these are the imposed polar's numbers (recorded by
// applyAerodynamicForces); pinned, they are the raw pressure resultant,
// which carries no drag model and no glide ratio worth reading.
struct AeroSummary
{
    softwing::Vec3 force;   // pressure + drag, newtons
    double lift = 0.0;      // across the airflow
    double drag = 0.0;      // along it
    double glideRatio = 0.0;
};
[[nodiscard]] AeroSummary aerodynamicSummary(const SimBody &sim,
                                             const SimControls &controls);

// One frame: brake lengths, load field, the XPBD step, then recentring.
// Propagates the solver's exception on failure.
void stepSimulation(SimBody &sim, const SimControls &controls);

// The interactive grab. Soft enough that a pull is a spring the fabric can
// answer, stiff enough that dragging feels direct; the softness is also
// what makes the force readout well-conditioned.
inline constexpr double grabCompliance = 1.0e-7;   // m/N
// Ties the grab cable to the given junction node (an endpoint of some
// LineSegment), creating or re-aiming the kinematic anchor at the node's
// current position. Returns false if the node index is out of range.
// Re-grabbing a different junction adds a fresh cable and slackens the old
// one — constraints cannot be removed once added.
bool beginGrab(SimBody &sim, std::size_t junctionNode);
// Moves the kinematic anchor; the cable does the pulling.
void moveGrab(SimBody &sim, const softwing::Vec3 &target);
// Slackens the cable so it carries nothing; the anchor stays for reuse.
void endGrab(SimBody &sim);
[[nodiscard]] bool grabActive(const SimBody &sim);
// Current pull in newtons, from the grab cable's accumulated multiplier.
[[nodiscard]] double grabForceNewtons(const SimBody &sim,
                                      const SimControls &controls);

// Translates the whole system so its centre of mass sits at the origin.
// Position and previous position move together, so this is a pure change of
// origin: velocities, and therefore the physics, are untouched. Without it a
// glider that is flying would simply leave the viewport.
void recentreSystem(SimBody &sim);

}  // namespace lep::playground

#endif  // LEP_PLAYGROUND_SIM_H
