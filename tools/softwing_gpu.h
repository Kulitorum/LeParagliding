#ifndef LEP_SOFTWING_GPU_H
#define LEP_SOFTWING_GPU_H

#include "../src/gui/playground_sim.h"

#include <QString>

#include <memory>
#include <vector>

// An OpenGL 4.3 compute-shader XPBD backend for the Playground's wing,
// written to be measured against the CPU solver rather than to replace it.
// It runs the whole substep loop — face pressure, prediction, the coloured
// constraint sweep, velocity finalization — on the GPU, with node state
// resident in shader storage buffers between frames and a readback only when
// the caller asks for the pose.
//
// It is a prototype, and differs from the core solver in two ways that
// matter and are reported rather than hidden:
//
//   * It works in float32. Consumer GeForce parts run double at a small
//     fraction of float rate, so a double implementation would measure the
//     wrong thing entirely. See GpuSoftBody::maximumDeviation.
//   * It reuses the core's constraint colouring verbatim, so the sweep order
//     matches SoftBody's parallel path exactly — which is what makes the two
//     comparable at all.
namespace lep::playground {

// How the constraint sweep is scheduled on the device. The two are different
// solvers, not two schedules of one solver, and they converge differently —
// which is the point of measuring both.
enum class GpuSolveMode
{
    // The core's colouring, one dispatch per colour. Same sweep as
    // SoftBody's parallel CPU path, so the poses are directly comparable —
    // but a wing colours into dozens of colours and the dispatches dominate.
    ColouredGaussSeidel,
    // Every constraint proposes a correction from the same state; each node
    // then averages the corrections of its incident constraints. Two
    // dispatches per iteration regardless of topology.
    Jacobi,
};

class GpuSoftBody
{
public:
    GpuSoftBody();
    ~GpuSoftBody();
    GpuSoftBody(const GpuSoftBody &) = delete;
    GpuSoftBody &operator=(const GpuSoftBody &) = delete;

    // Creates an offscreen 4.3 core context, compiles the kernels and
    // uploads the body. False on failure, with the reason in `error`.
    bool initialize(const SimBody &sim, GpuSolveMode mode, QString &error);

    [[nodiscard]] QString rendererDescription() const;
    // Colours dispatched per constraint iteration — the prototype's main
    // cost driver, so the caller reports it.
    [[nodiscard]] std::size_t colourCount() const;
    [[nodiscard]] std::size_t dispatchesPerFrame(const SimControls &controls) const;

    // One frame: the same anchors, lift stamp and substep loop the CPU path
    // runs. Blocks until the GPU is idle, so the caller's wall clock is the
    // frame's real cost.
    void step(SimBody &sim, const SimControls &controls);

    // Copies the GPU pose back into the body's nodes.
    void readback(SimBody &sim);
    // Largest per-node distance between the GPU pose and the body's current
    // CPU pose, in metres. Meaningful only when both were stepped the same
    // way from the same start.
    [[nodiscard]] double maximumDeviation(const SimBody &sim) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lep::playground

#endif  // LEP_SOFTWING_GPU_H
