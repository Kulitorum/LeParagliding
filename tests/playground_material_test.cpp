#include "playground_metrics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace pg = lep::playground;

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

pg::SimMesh openQuad()
{
    pg::SimMesh mesh;
    // Ordered chart contract: q0->q1 is span/weft (+x), q0->q3 is
    // chord/warp (+y).
    mesh.nodes = {{0.0, 0.0, 0.0},
                  {1.0, 0.0, 0.0},
                  {1.0, 1.0, 0.0},
                  {0.0, 1.0, 0.0}};
    mesh.quads = {{0, 1, 2, 3}};
    mesh.quadSurfaces = {pg::SimSurface::Extrados};
    return mesh;
}

pg::SimControls controls(pg::SkinModel model)
{
    pg::SimControls result;
    result.skinModel = model;
    result.pressurePascal = 0.0;
    result.flightLoad = false;
    result.freeFlight = false;
    result.fabricContact = false;
    return result;
}

softwing::StepSettings oneProjection()
{
    softwing::StepSettings settings;
    settings.timeStep = pg::simulationTimeStep;
    settings.substeps = 1;
    settings.constraintIterations = 1;
    settings.gravity = {};
    settings.velocityDampingPerSecond = 0.0;
    return settings;
}

void stamp(pg::SimBody &sim)
{
    for (softwing::Node &node : sim.body->nodes()) {
        node.previousPosition = node.position;
        node.velocity = {};
    }
}

double membraneEnergy(const pg::SimBody &sim)
{
    double energy = 0.0;
    for (std::size_t element = 0;
         element < sim.body->membraneElements().size(); ++element) {
        energy += sim.body->membraneDiagnostics(element).elasticEnergy;
    }
    return energy;
}

void testInventoryAndRest()
{
    const pg::SimMesh mesh = openQuad();
    pg::SimBody legacy = pg::buildSimBody(
        mesh, {}, controls(pg::SkinModel::LegacyDistanceTruss));
    pg::SimControls membraneControls =
        controls(pg::SkinModel::OrthotropicMembrane);
    pg::SimBody membrane = pg::buildSimBody(mesh, {}, membraneControls);

    check(legacy.body->membraneElements().empty()
              && legacy.body->dihedralConstraints().empty(),
          "inventory: legacy body has no material elements or hinges");
    check(legacy.body->constraints().size() == 6,
          "inventory: legacy quad has perimeter and both diagonals");
    check(membrane.body->constraints().empty(),
          "inventory: membrane quad has no duplicate skin truss");
    check(membrane.body->membraneElements().size() == 2,
          "inventory: one membrane per skin triangle");
    check(membrane.body->dihedralConstraints().size() == 1,
          "inventory: one true hinge on the interior diagonal");
    check(membrane.skippedMembraneElements == 0
              && membrane.skippedDihedralHinges == 0,
          "inventory: regular quad has no skipped material feature");
    check(std::all_of(membrane.renderFaces.begin(),
                      membrane.renderFaces.end(),
                      [](const pg::RenderFace &face) {
                          return face.membraneElement.has_value()
                                 && std::all_of(
                                     face.edges.begin(), face.edges.end(),
                                     [](std::size_t edge) {
                                         return edge == pg::noConstraint;
                                     });
                      }),
          "inventory: render faces map membranes and no skin springs");

    const auto before = membrane.body->nodes();
    for (int frame = 0; frame < 3; ++frame) {
        pg::stepSimulation(membrane, membraneControls);
    }
    for (std::size_t node = 0; node < before.size(); ++node) {
        check(length(membrane.body->nodes()[node].position
                     - before[node].position)
                  < 1.0e-12,
              "rest: membrane chart and hinge leave rest pose invariant");
    }
}

void testWarpWeftAndCompression()
{
    pg::SimControls materialControls =
        controls(pg::SkinModel::OrthotropicMembrane);
    materialControls.warpStiffness = 8000.0;
    materialControls.weftStiffness = 2000.0;
    materialControls.couplingStiffness = 0.0;
    materialControls.shearStiffness = 1000.0;
    materialControls.compressionStiffnessRatio = 1.0;
    pg::SimBody warp = pg::buildSimBody(openQuad(), {}, materialControls);
    pg::SimBody weft = pg::buildSimBody(openQuad(), {}, materialControls);
    for (softwing::Node &node : warp.body->nodes()) {
        node.position.y *= 1.05;
    }
    for (softwing::Node &node : weft.body->nodes()) {
        node.position.x *= 1.05;
    }
    check(membraneEnergy(warp) > 2.5 * membraneEnergy(weft),
          "material: ordered quad distinguishes stiff warp from soft weft");
    std::vector<float> tensile;
    std::vector<float> slack;
    pg::nodeStrainFields(warp, false, tensile, slack);
    check(*std::max_element(tensile.begin(), tensile.end()) > 0.0F,
          "metrics: membrane tension reaches the node stress field");

    pg::SimControls legacyControls =
        controls(pg::SkinModel::LegacyDistanceTruss);
    pg::SimControls softControls =
        controls(pg::SkinModel::OrthotropicMembrane);
    softControls.compressionStiffnessRatio = 1.0e-4;
    pg::SimBody legacy = pg::buildSimBody(openQuad(), {}, legacyControls);
    pg::SimBody soft = pg::buildSimBody(openQuad(), {}, softControls);
    for (pg::SimBody *sim : {&legacy, &soft}) {
        for (softwing::Node &node : sim->body->nodes()) {
            if (node.position.x > 0.5) {
                node.position.x = 0.9;
            }
        }
        stamp(*sim);
        sim->body->step(oneProjection());
    }
    const auto width = [](const pg::SimBody &sim) {
        const auto &nodes = sim.body->nodes();
        return 0.5 * ((nodes[1].position.x - nodes[0].position.x)
                      + (nodes[2].position.x - nodes[3].position.x));
    };
    check(width(legacy) > width(soft) + 1.0e-4,
          "compression: softened membrane wrinkles instead of bilateral truss response");
    std::vector<float> faceSlack;
    pg::faceSlackField(soft, faceSlack);
    check(*std::min_element(faceSlack.begin(), faceSlack.end()) < 0.0F,
          "metrics: membrane compression reaches the face wrinkle field");
}

void testBendingAndDeterminism()
{
    pg::SimControls stiffControls =
        controls(pg::SkinModel::OrthotropicMembrane);
    stiffControls.bendCompliance = 0.0;
    pg::SimControls softControls = stiffControls;
    softControls.bendCompliance = 1.0;
    pg::SimBody stiff = pg::buildSimBody(openQuad(), {}, stiffControls);
    pg::SimBody soft = pg::buildSimBody(openQuad(), {}, softControls);
    for (pg::SimBody *sim : {&stiff, &soft}) {
        for (std::size_t node = 0; node < 3; ++node) {
            sim->body->fixNode(node);
        }
        sim->body->nodes()[3].position.z = 0.4;
        stamp(*sim);
        sim->body->step(oneProjection());
    }
    check(std::abs(stiff.body->nodes()[3].position.z)
              < std::abs(soft.body->nodes()[3].position.z),
          "bending: stiff true hinge restores more than compliant hinge");

    pg::SimBody first = pg::buildSimBody(openQuad(), {}, stiffControls);
    pg::SimBody second = pg::buildSimBody(openQuad(), {}, stiffControls);
    first.body->nodes()[3].position.z = 0.1;
    second.body->nodes()[3].position.z = 0.1;
    stamp(first);
    stamp(second);
    for (int frame = 0; frame < 10; ++frame) {
        pg::stepSimulation(first, stiffControls);
        pg::stepSimulation(second, stiffControls);
    }
    for (std::size_t node = 0; node < first.body->nodes().size(); ++node) {
        const auto &a = first.body->nodes()[node];
        const auto &b = second.body->nodes()[node];
        check(std::isfinite(a.position.x) && std::isfinite(a.position.y)
                  && std::isfinite(a.position.z),
              "determinism: short membrane run remains finite");
        check(a.position.x == b.position.x && a.position.y == b.position.y
                  && a.position.z == b.position.z,
              "determinism: repeated short membrane run is bit-identical");
    }
}

void testDegenerateAndNonManifoldDiagnostics()
{
    pg::SimMesh degenerate = openQuad();
    degenerate.nodes[2] = {2.0, 0.0, 0.0};
    degenerate.nodes[3] = {3.0, 0.0, 0.0};
    const pg::SimBody rejected = pg::buildSimBody(
        degenerate, {}, controls(pg::SkinModel::OrthotropicMembrane));
    check(rejected.body->membraneElements().empty()
              && rejected.skippedMembraneElements == 2,
          "diagnostics: degenerate charts are skipped rather than NaN");

    pg::SimMesh nonManifold;
    nonManifold.nodes = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                         {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
                         {1.0, 0.0, 1.0}, {0.0, 0.0, 1.0},
                         {1.0, -1.0, 0.0}, {0.0, -1.0, 0.0}};
    nonManifold.quads = {{0, 1, 2, 3},
                         {0, 1, 4, 5},
                         {0, 1, 6, 7}};
    nonManifold.quadSurfaces.assign(3, pg::SimSurface::Extrados);
    const pg::SimBody body = pg::buildSimBody(
        nonManifold, {}, controls(pg::SkinModel::OrthotropicMembrane));
    check(body.skippedDihedralHinges >= 1,
          "diagnostics: non-manifold edge is reported as skipped");
    const bool badHinge = std::any_of(
        body.body->dihedralConstraints().begin(),
        body.body->dihedralConstraints().end(),
        [](const softwing::DihedralBendingConstraint &hinge) {
            return std::min(hinge.a, hinge.b) == 0
                   && std::max(hinge.a, hinge.b) == 1;
        });
    check(!badHinge,
          "diagnostics: no hinge is retained on a non-manifold edge");
}

}  // namespace

int main()
{
    testInventoryAndRest();
    testWarpWeftAndCompression();
    testBendingAndDeterminism();
    testDegenerateAndNonManifoldDiagnostics();
    if (failures != 0) {
        std::fprintf(stderr, "%d playground material check(s) failed\n",
                     failures);
        return 1;
    }
    std::printf("all playground material checks passed\n");
    return 0;
}
