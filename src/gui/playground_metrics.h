#ifndef LEP_PLAYGROUND_METRICS_H
#define LEP_PLAYGROUND_METRICS_H

#include "playground_sim.h"

#include <QChar>
#include <QString>

#include <atomic>
#include <cstddef>
#include <functional>
#include <vector>

// Shape-fidelity instrumentation for the Playground's wind-tunnel mode.
//
// The engine's mesh samples the design's exact ballooning law, so the rest
// pose IS the design shape, and every departure of the settled, loaded wing
// from it is a structural signal: fabric going slack, a nose denting
// inward, a section washing out, a line row shedding load. Everything here
// is measurement over the live body — no new physics. See
// docs/playground-shape-analysis.md for what each number means and the
// limits of the claim.
namespace lep::playground {

// --- Heuristic thresholds for the "anything weird" flags. Tripwires for a
// designer's attention, not pass/fail engineering criteria; each is
// documented where it is checked.
// Calibrated against gnuC2 swept -4..+24 deg at 80 Pa: each threshold
// sits above the clean working band (-2..+14 deg) and below the collapse
// states, so a flag means "off the healthy band", not "the model's normal
// operating point". Every dimensional threshold is expressed relative to
// the wing's own scale (chord, span, airspeed), so the calibration
// carries to a mini wing or a tandem instead of being one wing's numbers.
//
// Leading-edge dent, as a fraction of the rib's rest chord, floored at a
// fraction of the wing's MEAN chord so the stubby tip and stabilo
// sections — genuinely floppy on real wings, standing at ~100 mm of nose
// travel while perfectly healthy — cannot drive a wing-level flag. A
// healthy loaded wing stands at 2-5% of chord here (the unloaded
// mid-cell nose relaxes inward from its designed ballooning — the reason
// real wings grew nose rods); a real fold reads at metre scale, 10x the
// worst healthy tip.
inline constexpr double flagLeadingEdgeDentChordFraction = 0.08;
inline constexpr double flagLeadingEdgeDentMeanChordFloor = 0.12;
// Section RMS against the larger of the rib's own chord and half the
// mean chord: the same tip-section exemption.
inline constexpr double flagSectionRmsChordFraction = 0.030;
inline constexpr double flagTwistChangeDegrees = 4.0;
// A quarter of the skin's edges are slack at a healthy trim — the barely
// loaded intrados flaps on real wings too. Abnormal starts near a third.
inline constexpr double flagSlackFabricFraction = 0.30;
inline constexpr double flagSpanLossRatio = 0.95;
// Under the shaped Cp field a healthy wing settles at 100-106% of its
// design volume (the +23% figure belongs to the old uniform-pressure
// sandbox), so under-inflation means falling clearly below rest.
inline constexpr double flagUnderInflatedVolumeRatio = 0.95;
// Mirror error as a fraction of the rest span: a symmetric wing under
// symmetric input measures ~0.05% of span; instability onset reads 0.4%+.
inline constexpr double flagAsymmetrySpanFraction = 0.003;
// SlackRow: one side of a row carrying less than this fraction of its
// mirror side's force while the mirror is loaded.
inline constexpr double flagSlackRowFraction = 0.25;
// An edge shorter than its rest length by more than this strain is a
// wrinkle: fabric cannot push, so compression is slack cloth.
inline constexpr double slackStrainThreshold = -0.002;
// Quiescence: RMS canopy node velocity relative to the canopy's bulk
// motion below this fraction of the tunnel airspeed counts as settled.
// Not lower: a loaded wing carries a standing micro-flutter floor of
// roughly 1% of airspeed in its slack panels (a quarter of the skin's
// edges are slack at trim — fabric there has no stiffness and never goes
// fully quiet), and a threshold under that floor reports an unsettled
// wing forever. The floor keeps a zero-wind measurement from demanding
// perfect stillness.
//
// Wings differ in how loud that flutter floor is (a wing whose C row
// hangs slack breathes at several percent of airspeed while perfectly
// stationary), so settleAndMeasure also accepts STATIONARITY: agitation
// and the aerodynamic resultant holding steady across a two-second
// window. Settled means "the measurement has converged", not "the
// fabric is motionless".
inline constexpr double settleAgitationAirspeedFraction = 0.013;
inline constexpr double settleAgitationFloorMetresPerSecond = 0.02;
inline constexpr double settleStationarySpread = 0.15;
inline constexpr double settleStationaryForceSpread = 0.02;

enum class ShapeFlag
{
    FrontTuckRisk,
    ProfileDistortion,
    WashoutChange,
    SlackFabric,
    SpanLoss,
    UnderInflated,
    Asymmetry,
    SlackRow,
    Unsettled,
};
[[nodiscard]] QString shapeFlagName(ShapeFlag flag);

// Everything the instruments compare the live wing against, captured once
// from a freshly built body BEFORE any step (buildSimBody leaves the
// positions at the mesh's rest pose; the free-flight launch only sets
// velocities).
struct ShapeBaseline
{
    // Full node table at build. Canopy nodes come first (see
    // SimBody::canopyNodeCount); junctions and pilot follow.
    std::vector<softwing::Vec3> restPositions;
    std::size_t canopyNodeCount = 0;
    // Area-weighted outward rest normal per node; zero for nodes that
    // carry no skin triangle (rib interiors, junctions). Deviation along
    // this is signed: negative = inward, the direction a dent goes.
    std::vector<softwing::Vec3> restNormals;
    // Rest positions of each rib's outline loop, parallel to
    // SimBody::ribChords / ribLoopNodes.
    std::vector<std::vector<softwing::Vec3>> restRibLoops;
    // Rib pairing across the mirror plane (x = 0 in mesh convention):
    // pairs[i] = the rib whose rest span station is -station(i); i itself
    // where no partner exists (the centre rib).
    std::vector<std::size_t> mirrorRib;
    double restVolume = 0.0;
    double restSpan = 0.0;
    double restArea = 0.0;
};
[[nodiscard]] ShapeBaseline captureShapeBaseline(const SimBody &sim);

struct RibShape
{
    // Residual after a rigid best-fit of the rest section onto the live
    // one, so trim rotation and translation do not count as error.
    double rmsMetres = 0.0;
    double maxMetres = 0.0;
    // Live chord length over rest chord length.
    double chordRatio = 1.0;
    // Section pitch relative to the wing, live vs rest, positive nose-up:
    // the live washout distribution.
    double twistDegrees = 0.0;
    // Worst inward normal displacement of the nose nodes (chord fraction
    // < 0.10), positive = dented inward. The front-tuck precursor.
    double leadingEdgeDentMetres = 0.0;
};

struct RowLoad
{
    QChar row;               // 'A'..'F'; brake cascade reports as 'F'
    bool brake = false;
    double leftNewtons = 0.0;   // mesh -x side
    double rightNewtons = 0.0;  // mesh +x side
    int segments = 0;           // riser-level segments in this row
    int slackSegments = 0;
};

struct ShapeFlagInfo
{
    ShapeFlag flag;
    QString detail;
};

struct ShapeReport
{
    // Echo of the conditions the measurement was taken under.
    double alphaDegrees = 0.0;
    double dynamicPressurePascal = 0.0;

    double spanRatio = 1.0;
    double areaRatio = 1.0;
    double volumeRatio = 1.0;
    double slackFraction = 0.0;
    double asymmetryMetres = 0.0;
    double agitationMetresPerSecond = 0.0;

    double worstDeviationMetres = 0.0;
    std::size_t worstDeviationRib = 0;
    double worstLeadingEdgeDentMetres = 0.0;
    std::size_t worstLeadingEdgeDentRib = 0;
    double worstTwistDegrees = 0.0;
    std::size_t worstTwistRib = 0;

    // Sum of riser-level line tensions, and how many riser segments hang
    // slack.
    double lineLoadNewtons = 0.0;
    int slackRiserSegments = 0;

    // The imposed polar's numbers when the flight-load pass ran (copied
    // from SimBody::last*); zero otherwise.
    double liftNewtons = 0.0;
    double dragNewtons = 0.0;
    double glideRatio = 0.0;

    std::vector<RibShape> ribs;
    std::vector<RowLoad> rows;
    std::vector<ShapeFlagInfo> flags;
};

// One full measurement pass over the live body. Cheap enough for a few
// hertz, not for every frame.
[[nodiscard]] ShapeReport measureShape(const SimBody &sim,
                                       const SimControls &controls,
                                       const ShapeBaseline &baseline);

// Per-node deviation from the globally aligned rest shape, metres, sized
// like the body's node table (zero beyond the canopy). Feeds the "Shape
// deviation" heatmap.
void nodeDeviationField(const SimBody &sim,
                        const ShapeBaseline &baseline,
                        std::vector<float> &metresOut);

// Per-render-face worst compression strain (negative numbers; 0 where
// taut), skin surfaces only. Feeds the "Slack fabric" heatmap.
void faceSlackField(const SimBody &sim, std::vector<float> &strainOut);

// Per-NODE strain extremes over the constrained edges touching each
// node: worst tension into tensileOut (>= 0), worst compression into
// slackOut (<= 0), both sized like the body's node table. A face
// carries one value in the per-face fields above and renders faceted;
// scattered to nodes, the same data shades smoothly across the skin.
// Rib-web faces join in only when detailedRibs built them as real
// sheets — a hub-and-spoke rib's edge strain is spoke tension dressed
// up as fabric stress.
void nodeStrainFields(const SimBody &sim,
                      bool detailedRibs,
                      std::vector<float> &tensileOut,
                      std::vector<float> &slackOut);

// Tension of one constraint in newtons from the XPBD accumulated
// multiplier of the last substep: F = -accumulatedLambda / h^2, positive
// when taut, 0 for slack cables. Valid immediately after a step.
[[nodiscard]] double constraintTensionNewtons(const SimBody &sim,
                                              const SimControls &controls,
                                              std::size_t constraint);

// Step until the measurement converges (quiescent, or stationary in
// agitation and resultant) or maxSeconds of simulated time has passed,
// then measure. Shared by the GUI sweep and the bench so any reported
// number is reproducible headless. A caller running this on a worker
// thread passes `cancelled`; it is polled every frame so a cancel (a
// closing dialog, an exiting application) returns within milliseconds
// instead of blocking a full settle.
struct SettleResult
{
    ShapeReport report;
    double simulatedSeconds = 0.0;
    bool settled = false;
};
// `progress`, when given, is called at every quiescence probe (each
// quarter second of simulated time) with the simulated time so far and
// the current agitation — the number that trends toward the quiescence
// target, which is the honest thing to show as progress. Called on the
// stepping thread.
[[nodiscard]] SettleResult settleAndMeasure(
    SimBody &sim,
    const SimControls &controls,
    const ShapeBaseline &baseline,
    double maxSeconds = 6.0,
    const std::atomic<bool> *cancelled = nullptr,
    const std::function<void(double simulatedSeconds,
                             double agitationMetresPerSecond)> *progress =
        nullptr);
// The agitation the settle loop counts as quiet at the given dynamic
// pressure, for callers displaying progress against it.
[[nodiscard]] double settleQuiescenceTarget(double pressurePascal);

// CSV serialisation for the sweep exports; header first, then one row per
// report. Row-load columns are emitted for rows A..F unconditionally so
// every row of the file has the same shape.
[[nodiscard]] QString shapeReportCsvHeader();
[[nodiscard]] QString shapeReportCsvRow(const ShapeReport &report);

}  // namespace lep::playground

#endif  // LEP_PLAYGROUND_METRICS_H
