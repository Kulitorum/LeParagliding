#ifndef LEP_PLAYGROUND_SIM_H
#define LEP_PLAYGROUND_SIM_H

#include <softwing/soft_body.h>

#include <QByteArray>
#include <QString>

#include <array>
#include <cstddef>
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
// Stands in for every drag we do not model. It has to damp fabric ringing
// without damping the pendulum: the swing period is several seconds, so
// anything much above this eats the very motion the pilot mass exists to
// produce.
inline constexpr double systemDampingPerSecond = 0.5;
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
    // 0 at the leading edge, 1 at the trailing edge.
    float chordFraction = 0.0F;
    bool upperSurface = false;
};

struct SimBuildOptions
{
    bool detailedRibs = false;
    int ribLayers = defaultRibLayers;
    int ribStationSplit = defaultRibStationSplit;
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
    // Parallel to the skin triangles. Empty when the mesh carries no rib
    // loops to hang a chord off, which falls the pressure field back to a
    // uniform one.
    std::vector<FaceAero> faceAero;
    std::vector<RibChord> ribChords;
    // Rest-pose planform, for the induced-drag term: a long thin wing pays
    // far less of it than a short fat one, and that is most of why one
    // glides better than the other.
    double planformArea = 0.0;
    double aspectRatio = 5.0;
    // Per-rib section lift coefficient from the most recent load stamp.
    // The drag pass reads it back: induced drag goes as CL squared, so it
    // has to know what each section was actually working at.
    std::vector<double> ribLiftCoefficient;
    // Filled in by each step's drag pass, for reporting.
    softwing::Vec3 lastDragForce;
    // Rest-pose mean chord and span directions, used to place the airflow.
    softwing::Vec3 restChordDirection{0.0, 1.0, 0.0};
    softwing::Vec3 restSpanAxis{1.0, 0.0, 0.0};
    softwing::Vec3 boundsLow;
    softwing::Vec3 boundsHigh;
};

// Live inputs to a step. Held apart from the body so the benchmark can
// drive the same wing with a different worker count or a profile attached
// without touching the build.
struct SimControls
{
    // Dynamic pressure q = ½ρV². It sets the whole load field, because in
    // this model both sides of the fabric are referred to it: the cell
    // interior sits at ram (stagnation) pressure, so its gauge pressure IS
    // q, and the outside of any face is q·Cp. 80 Pa is 41 km/h, an ordinary
    // trim speed. A real canopy runs 0.7–2.3 mbar this way, not the 0.1 bar
    // that gets quoted — 0.1 bar would need 460 km/h.
    double pressurePascal = 80.0;
    // Angle of the airflow to the wing's rest chord, in degrees. Replaces
    // the old fake follower "lift" force: the load now comes out of the
    // pressure field, and this is what tilts that field.
    double angleOfAttackDegrees = 6.0;
    // Let the whole system fly: gravity on, nothing pinned, the pilot's
    // mass free to swing under the canopy, and the pair translated back to
    // the origin after each step. EXPERIMENTAL and off by default — the
    // wing does not yet trim, it pitches over within a couple of seconds
    // (see docs/xpbd-performance.md). With it off the pilot is pinned, which
    // is the behaviour the tab has always had.
    bool freeFlight = false;
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

// Adds the tangential drag the pressure field cannot produce. Integrating
// pressure over a lifting body recovers leading-edge suction and no viscous
// loss — d'Alembert's paradox — so without this the model makes *thrust*,
// nothing sets a trim speed, and a glide ratio computed from it comes out
// negative. Clears the body's external forces and replaces them, so it owns
// that channel; call it once per frame, immediately before stepping.
void applyAerodynamicDrag(SimBody &sim, const SimControls &controls);

// Lift and drag resolved along the airflow, and the glide ratio they imply.
// Only meaningful once applyAerodynamicDrag has run for this pose.
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

// Translates the whole system so its centre of mass sits at the origin.
// Position and previous position move together, so this is a pure change of
// origin: velocities, and therefore the physics, are untouched. Without it a
// glider that is flying would simply leave the viewport.
void recentreSystem(SimBody &sim);

}  // namespace lep::playground

#endif  // LEP_PLAYGROUND_SIM_H
