#include <softwing/soft_body.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool finite(const softwing::Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
           && std::isfinite(value.z);
}

softwing::OrthotropicMembraneMaterial material(double compression = 1.0) {
    softwing::OrthotropicMembraneMaterial result;
    result.warpStiffness = 800.0;
    result.weftStiffness = 500.0;
    result.couplingStiffness = 100.0;
    result.shearStiffness = 180.0;
    result.dampingTime = 0.02;
    result.compressionStiffnessRatio = compression;
    return result;
}

void testCompressionScaling() {
    const auto bilateral = material();
    const softwing::SymmetricMatrix3 original = bilateral.stiffnessMatrix();
    const softwing::SymmetricMatrix3 exact =
        softwing::effectiveMembraneStiffness(
            bilateral, {-0.2, -0.1, 0.15});
    check(exact.xx == original.xx && exact.yy == original.yy
              && exact.zz == original.zz && exact.xy == original.xy
              && exact.xz == original.xz && exact.yz == original.yz,
          "compression: ratio one is exact historical stiffness");

    const auto softened = material(0.04);
    const softwing::Vec3 compression{-0.1, 0.0, 0.0};
    const softwing::Vec3 tension{0.1, 0.0, 0.0};
    const softwing::Vec3 oldCompression = original * compression;
    const softwing::Vec3 newCompression =
        softwing::effectiveMembraneStiffness(softened, compression)
        * compression;
    check(std::abs(newCompression.x) < std::abs(oldCompression.x),
          "compression: normal compressive resultant is reduced");
    const softwing::SymmetricMatrix3 tensileMatrix =
        softwing::effectiveMembraneStiffness(softened, tension);
    check(tensileMatrix.xx == original.xx
              && tensileMatrix.yy == original.yy
              && tensileMatrix.zz == original.zz
              && tensileMatrix.xy == original.xy,
          "compression: pure tension remains unchanged");
    check(softwing::isPositiveDefinite(
              softwing::effectiveMembraneStiffness(
                  softened, {-0.1, -0.2, 0.3})),
          "compression: D*K*D remains positive definite");
}

void testCompressedMembraneSolve() {
    softwing::SoftBody body;
    body.addFixedNode({0.0, 0.0, 0.0});
    body.addNode({0.9, 0.0, 0.0}, 1.0);
    body.addNode({0.0, 1.0, 0.0}, 1.0);
    const std::size_t triangle = body.addTriangle(0, 1, 2);
    const softwing::MembraneElementDefinition definition{
        triangle,
        {{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}},
        material(0.05),
        softwing::MaterialRole::Bulk};
    static_cast<void>(body.addMembraneElements(
        std::span<const softwing::MembraneElementDefinition>{&definition, 1}));
    softwing::StepSettings settings;
    settings.timeStep = 1.0 / 60.0;
    settings.substeps = 2;
    settings.constraintIterations = 4;
    settings.gravity = {};
    settings.velocityDampingPerSecond = 0.0;
    body.step(settings);
    for (const softwing::Node& node : body.nodes()) {
        check(finite(node.position) && finite(node.velocity),
              "compression: softened XPBD solve stays finite");
    }
    const softwing::MembraneElementDiagnostics diagnostics =
        body.membraneDiagnostics(0);
    check(std::isfinite(diagnostics.elasticEnergy)
              && diagnostics.elasticEnergy >= 0.0,
          "compression: softened membrane energy is finite and nonnegative");
}

struct HingeBody {
    softwing::SoftBody body;
    std::array<softwing::Vec3, 4> rest;
};

HingeBody hinge(bool mirrored, bool fixFirstThree = true) {
    const double side = mirrored ? -1.0 : 1.0;
    HingeBody result;
    result.rest = {{{0.0, 0.0, 0.0},
                    {1.0, 0.0, 0.0},
                    {0.0, side, 0.0},
                    {1.0, -side, 0.0}}};
    for (std::size_t i = 0; i < result.rest.size(); ++i) {
        if (fixFirstThree && i < 3) {
            result.body.addFixedNode(result.rest[i]);
        } else {
            result.body.addNode(result.rest[i], 1.0);
        }
    }
    result.body.addDihedralBendingConstraint(0, 1, 2, 3, 0.0, 0.0);
    return result;
}

softwing::StepSettings hingeSettings() {
    softwing::StepSettings settings;
    settings.timeStep = 1.0 / 60.0;
    settings.substeps = 1;
    settings.constraintIterations = 1;
    settings.gravity = {};
    settings.velocityDampingPerSecond = 0.0;
    return settings;
}

void stampPositions(softwing::SoftBody& body) {
    for (softwing::Node& node : body.nodes()) {
        node.previousPosition = node.position;
        node.velocity = {};
    }
}

void testDihedralFlatAndFolded() {
    HingeBody flat = hinge(false);
    const auto before = flat.body.nodes();
    flat.body.step(hingeSettings());
    for (std::size_t i = 0; i < before.size(); ++i) {
        check(length(flat.body.nodes()[i].position - before[i].position)
                  < 1.0e-15,
              "hinge: flat rest pose does not move");
    }

    HingeBody folded = hinge(false);
    folded.body.nodes()[3].position.z = 0.5;
    stampPositions(folded.body);
    const double beforeFold = folded.body.nodes()[3].position.z;
    folded.body.step(hingeSettings());
    check(std::abs(folded.body.nodes()[3].position.z) < std::abs(beforeFold),
          "hinge: folded free corner restores toward its rest plane");
    for (std::size_t i = 0; i < 3; ++i) {
        check(length(folded.body.nodes()[i].position - folded.rest[i]) == 0.0,
              "hinge: fixed nodes remain fixed under mass weighting");
    }
}

void testDihedralRigidInPlaneMirrorAndDegenerate() {
    HingeBody inPlane = hinge(false);
    inPlane.body.nodes()[2].position = {-0.25, 1.3, 0.0};
    inPlane.body.nodes()[3].position = {1.4, -0.7, 0.0};
    stampPositions(inPlane.body);
    const auto before = inPlane.body.nodes();
    inPlane.body.step(hingeSettings());
    for (std::size_t i = 0; i < before.size(); ++i) {
        check(length(inPlane.body.nodes()[i].position - before[i].position)
                  < 1.0e-15,
              "hinge: in-plane shear creates no false bending");
    }

    HingeBody right = hinge(false);
    HingeBody left = hinge(true);
    right.body.nodes()[3].position.z = 0.4;
    left.body.nodes()[3].position.z = 0.4;
    stampPositions(right.body);
    stampPositions(left.body);
    right.body.step(hingeSettings());
    left.body.step(hingeSettings());
    check(std::abs(right.body.nodes()[3].position.z
                   - left.body.nodes()[3].position.z)
              < 1.0e-12,
          "hinge: mirrored winding has mirrored restoring magnitude");

    HingeBody degenerate = hinge(false);
    degenerate.body.nodes()[3].position = {0.5, 0.0, 0.0};
    stampPositions(degenerate.body);
    degenerate.body.step(hingeSettings());
    for (const softwing::Node& node : degenerate.body.nodes()) {
        check(finite(node.position) && finite(node.velocity),
              "hinge: degenerate live face is a finite no-op");
    }
}

void testDihedralStateRoundTrip() {
    HingeBody source = hinge(false);
    const softwing::DihedralBendingConstraint& value =
        source.body.dihedralConstraints().front();
    std::ostringstream encoded;
    encoded.imbue(std::locale::classic());
    encoded << std::setprecision(std::numeric_limits<double>::max_digits10)
            << value.a << ' ' << value.b << ' ' << value.c << ' ' << value.d
            << ' ' << value.restAngleRadians << ' ' << value.compliance;
    std::istringstream decoded(encoded.str());
    decoded.imbue(std::locale::classic());
    std::size_t a = 0, b = 0, c = 0, d = 0;
    double rest = 0.0, compliance = 0.0;
    check(static_cast<bool>(decoded >> a >> b >> c >> d >> rest >> compliance),
          "hinge: canonical state parses");
    HingeBody restored;
    restored.rest = source.rest;
    for (const softwing::Vec3& point : restored.rest) {
        restored.body.addNode(point, 1.0);
    }
    restored.body.addDihedralBendingConstraint(
        a, b, c, d, rest, compliance);
    const auto& roundTrip = restored.body.dihedralConstraints().front();
    check(roundTrip.a == value.a && roundTrip.b == value.b
              && roundTrip.c == value.c && roundTrip.d == value.d
              && roundTrip.restAngleRadians == value.restAngleRadians
              && roundTrip.compliance == value.compliance,
          "hinge: state round trip preserves oriented edge and rest sign");
}

softwing::SoftBody genericHinge(double restAngle) {
    softwing::SoftBody body;
    body.addNode({-0.2, 0.1, 0.3}, 0.8);
    body.addNode({1.1, -0.15, 0.25}, 1.1);
    body.addNode({0.15, 1.0, -0.2}, 0.9);
    body.addNode({0.75, -0.8, 0.65}, 1.3);
    body.addDihedralBendingConstraint(0, 1, 2, 3, restAngle, 0.0);
    return body;
}

void testDihedralGradientAndProjectionSigns() {
    softwing::SoftBody gradientBody = genericHinge(0.0);
    const softwing::DihedralBendingDiagnostics analytic =
        gradientBody.dihedralDiagnostics(0);
    check(analytic.valid, "hinge: generic signed-angle diagnostic is valid");
    constexpr double epsilon = 1.0e-6;
    double maximumError = 0.0;
    for (std::size_t node = 0; node < 4; ++node) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            double* coordinate = axis == 0
                ? &gradientBody.nodes()[node].position.x
                : (axis == 1 ? &gradientBody.nodes()[node].position.y
                             : &gradientBody.nodes()[node].position.z);
            *coordinate += epsilon;
            const double plus =
                gradientBody.dihedralDiagnostics(0).angleRadians;
            *coordinate -= 2.0 * epsilon;
            const double minus =
                gradientBody.dihedralDiagnostics(0).angleRadians;
            *coordinate += epsilon;
            const double numerical =
                std::remainder(plus - minus,
                               6.283185307179586476925286766559)
                / (2.0 * epsilon);
            const softwing::Vec3 &value = analytic.angleGradient[node];
            const double expected =
                axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
            maximumError = std::max(maximumError,
                                    std::abs(numerical - expected));
        }
    }
    check(maximumError < 2.0e-6,
          "hinge: analytic signed-angle gradient matches finite difference");

    const double restAngle = analytic.angleRadians;
    for (const double direction : {-1.0, 1.0}) {
        softwing::SoftBody projected = genericHinge(restAngle);
        projected.nodes()[3].position.z += direction * 0.12;
        stampPositions(projected);
        const double before = std::abs(std::remainder(
            projected.dihedralDiagnostics(0).angleRadians - restAngle,
            6.283185307179586476925286766559));
        projected.step(hingeSettings());
        const double after = std::abs(std::remainder(
            projected.dihedralDiagnostics(0).angleRadians - restAngle,
            6.283185307179586476925286766559));
        check(after < before,
              "hinge: one XPBD projection reduces either signed residual");
    }
}

softwing::SoftBody cableCascade() {
    softwing::SoftBody body;
    body.addFixedNode({0.0, 0.0, 0.0});
    for (int level = 1; level <= 7; ++level) {
        body.addNode({0.0, 0.0, -static_cast<double>(level)},
                     level == 7 ? 90.0 : 0.05);
    }
    // Match the Playground's intentionally non-topological build order:
    // authored cables first, then the canopy attachment and harness ties.
    for (int level = 1; level < 6; ++level) {
        body.addCableConstraint(
            static_cast<std::size_t>(level),
            static_cast<std::size_t>(level + 1), 1.0, 0.0);
    }
    body.addSuspensionTieConstraint(0, 1, 1.0, 0.0);
    body.addSuspensionTieConstraint(6, 7, 1.0, 0.0);
    body.addForce(7, {0.0, 0.0, -9000.0});
    return body;
}

double worstCableExtension(const softwing::SoftBody& body) {
    double worst = 0.0;
    for (const softwing::DistanceConstraint& cable : body.constraints()) {
        worst = std::max(
            worst,
            length(body.nodes()[cable.b].position
                   - body.nodes()[cable.a].position)
                - cable.restLength);
    }
    return worst;
}

void testCableCascadeSweeps() {
    softwing::StepSettings settings;
    settings.timeStep = 1.0 / 60.0;
    settings.substeps = 1;
    settings.constraintIterations = 2;
    settings.gravity = {};
    settings.velocityDampingPerSecond = 0.0;

    softwing::SoftBody plain = cableCascade();
    plain.step(settings);
    const double plainExtension = worstCableExtension(plain);

    softwing::SoftBody again = cableCascade();
    again.step(settings);
    check(again.nodes()[7].position.z == plain.nodes()[7].position.z,
          "load-path sweeps: zero pairs preserve deterministic legacy path");

    softwing::StepPerformanceProfile profile;
    settings.cableConstraintSweepPairs = 3;
    settings.performanceProfile = &profile;
    softwing::SoftBody conditioned = cableCascade();
    conditioned.step(settings);
    const double conditionedExtension = worstCableExtension(conditioned);
    check(conditionedExtension < 0.85 * plainExtension,
          "load-path sweeps: reverse/forward passes condition seven levels");
    check(profile.cableConstraintVisits == 42,
          "load-path sweeps: visits count cables and suspension ties");
    for (const softwing::Node& node : conditioned.nodes()) {
        check(finite(node.position) && finite(node.velocity),
              "load-path sweeps: conditioned cascade stays finite");
    }
}

} // namespace

int main() {
    testCompressionScaling();
    testCompressedMembraneSolve();
    testDihedralFlatAndFolded();
    testDihedralRigidInPlaneMirrorAndDegenerate();
    testDihedralStateRoundTrip();
    testDihedralGradientAndProjectionSigns();
    testCableCascadeSweeps();
    if (failures != 0) {
        std::fprintf(stderr, "%d softwing material check(s) failed\n", failures);
        return 1;
    }
    std::printf("all softwing material checks passed\n");
    return 0;
}
