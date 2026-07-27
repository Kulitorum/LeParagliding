#ifndef SOFTWING_PARALLEL_H
#define SOFTWING_PARALLEL_H

#include <cstddef>
#include <exception>
#include <functional>
#include <memory>

namespace softwing {

// Fixed-size worker pool for the solver's coloured sweeps.
//
// Deliberately minimal, and deliberately not a general task scheduler: the
// simulation core is byte-reproducible by contract, so nothing here may let
// thread scheduling reach a result. Two properties carry that guarantee:
//
//   * Work is split into contiguous index ranges decided solely by
//     (count, workerCount). Which worker runs which range, and in what order
//     they finish, cannot change the partition.
//   * A range body must write only to state owned by its own indices. The
//     membrane colouring is what makes that true for the XPBD sweep; see
//     SoftBody::membraneColouring.
//
// Together those mean a run is bit-identical for a given workerCount, and the
// worker count is an explicit input (StepSettings::workerThreads), never
// sampled from the machine. Exceptions thrown by a range are captured and
// rethrown on the calling thread from the lowest range index, so a failing
// step reports the same error whichever worker happened to reach it first.
class WorkerPool {
public:
    using RangeBody = std::function<void(std::size_t begin, std::size_t end)>;

    // workerCount is clamped to at least 1. A pool of 1 runs inline on the
    // calling thread and spawns nothing.
    explicit WorkerPool(unsigned workerCount);
    ~WorkerPool();

    // Neither copyable nor movable. The shutdown handshake lives in the
    // destructor, so a move-assignment would tear down the target's Impl while
    // its workers were still running on it -- destroying joinable threads and
    // leaving the running ones reading freed memory. Hold one by pointer (see
    // WorkerPoolSlot) rather than relocating it.
    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;
    WorkerPool(WorkerPool&&) = delete;
    WorkerPool& operator=(WorkerPool&&) = delete;

    [[nodiscard]] unsigned workerCount() const noexcept;

    // Invokes body over a partition of [0, count) and blocks until every
    // range has completed. Rethrows the lowest-index range's exception, if
    // any, after all ranges have finished.
    void forEachRange(std::size_t count, const RangeBody& body);

    // Runs `phaseCount` phases back to back in a SINGLE dispatch, with an
    // internal barrier between them: phase p+1 starts only once every worker
    // has finished phase p, and workers stay hot in between.
    //
    // This exists because the colour sweep is many tiny phases. Waking and
    // re-synchronising a couple of dozen threads costs microseconds, which is
    // the same order as one colour's work -- so dispatching per colour spends
    // more on scheduling than on physics. Fusing them amortises that over the
    // whole sweep.
    //
    // `size(p)` gives phase p's element count and must be a pure function of
    // p; every worker evaluates it independently and they must agree.
    using PhaseSize = std::function<std::size_t(std::size_t phase)>;
    // `grain(p)` gives the number of phase elements in one dynamic claim.
    // Like size, it must be a pure function of p and at least one.
    using PhaseGrain = std::function<std::size_t(std::size_t phase)>;
    using PhaseBody = std::function<
        void(std::size_t phase, std::size_t begin, std::size_t end)>;
    void forEachPhase(std::size_t phaseCount,
                      const PhaseSize& size,
                      const PhaseGrain& grain,
                      const PhaseBody& body);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Holds a pool for an object that must stay copyable. A pool is a scratch
// resource rather than simulation state, so every assignment leaves the
// target's own pool untouched instead of transferring workers -- two copies of
// a body are equal as physics regardless of which threads either is using.
//
// Move behaves like copy on purpose. The aerodynamics failure path restores a
// body with `*owner_ = std::move(backup)`, and a defaulted move would overwrite
// the live pool with the backup's empty one -- joining every worker and forcing
// a full respawn on the next substep, on the error path of all places.
class WorkerPoolSlot {
public:
    WorkerPoolSlot() = default;
    WorkerPoolSlot(const WorkerPoolSlot&) noexcept {}
    WorkerPoolSlot& operator=(const WorkerPoolSlot&) noexcept { return *this; }
    WorkerPoolSlot(WorkerPoolSlot&&) noexcept {}
    WorkerPoolSlot& operator=(WorkerPoolSlot&&) noexcept { return *this; }
    ~WorkerPoolSlot() = default;

    // Rebuilds only when the count changes. A count of 1 still yields a pool
    // (which runs inline) so that callers can keep "which sweep" and "how many
    // workers" as independent decisions.
    [[nodiscard]] WorkerPool* get(unsigned workerCount);

private:
    std::unique_ptr<WorkerPool> pool_;
    unsigned size_ = 0;
};

// Worker count the machine can sustain, for callers that want a default.
// Never consulted by the core itself -- see the reproducibility note above.
[[nodiscard]] unsigned hardwareWorkerCount() noexcept;
// Physical execution cores when the platform exposes that topology. This
// avoids treating SMT siblings as extra cores for synchronization-heavy
// solver work; falls back to hardwareWorkerCount() when unavailable.
[[nodiscard]] unsigned hardwarePhysicalCoreCount() noexcept;

}  // namespace softwing

#endif  // SOFTWING_PARALLEL_H
