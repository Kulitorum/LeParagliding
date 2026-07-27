#include "softwing/parallel.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace softwing {
namespace {

// A colour sweep is tens of microseconds, so between the sweeps of one step a
// worker should stay hot rather than pay to be woken. Between *steps* it may
// idle indefinitely -- an interactive host renders far slower than it steps,
// and a pool that spun through that would burn every core it owns and starve
// the UI thread. So: spin, then yield, then sleep properly.
constexpr int kSpinsBeforeYield = 8192;
constexpr int kYieldsBeforeSleep = 64;

// Elements per claimable chunk. A colour sweep is split into fixed chunks that
// workers claim dynamically, rather than one range each: on a hybrid CPU the
// E-core workers are several times slower than the P-core ones, and an equal
// split makes every barrier wait on the slowest. Claiming lets fast workers
// take more chunks.
//
// This does NOT cost reproducibility. The chunk boundaries are a function of
// (count, grain) alone, and the elements of one colour touch disjoint nodes --
// so which worker executes which chunk, and in what order, cannot be observed
// in the result. Only the boundaries need to be fixed, and they are.
//
// The value trades scheduling slack against claim overhead: too small and the
// claim atomic dominates, too large and the tail of a colour straggles again.
// Swept over 8..48 across 8/16/24 workers on a 7340-element canopy; flat from
// 8 to 16, clearly worse by 24.
constexpr std::size_t kChunkGrain = 8;

std::size_t chunkGrain() { return kChunkGrain; }

template <typename Predicate>
void spinUntil(Predicate ready) {
    int spins = 0;
    const int budget = kSpinsBeforeYield;
    while (!ready()) {
        if (++spins < budget) {
            continue;
        }
        spins = 0;
        std::this_thread::yield();
    }
}

}  // namespace

struct WorkerPool::Impl {
    unsigned workerCount = 1;
    std::vector<std::thread> threads;

    // Bumped once per forEachRange call; workers wake when it changes.
    std::atomic<unsigned> generation{0};
    std::atomic<unsigned> pending{0};
    std::atomic<bool> stopping{false};

    // Published before `generation` is bumped and read-only for the duration
    // of a sweep, so the release/acquire pair on `generation` is what makes
    // these visible to workers.
    const RangeBody* body = nullptr;
    std::size_t count = 0;

    std::mutex failureMutex;
    std::size_t failedRange = std::numeric_limits<std::size_t>::max();
    std::exception_ptr failure;

    std::mutex sleepMutex;
    std::condition_variable wake;
    std::atomic<int> sleepers{0};

    std::atomic<std::size_t> nextChunk{0};
    std::size_t chunkCount = 0;

    // Fixed chunk boundaries -- a function of (count, kChunkGrain) alone,
    // never of thread identity, arrival order, or even workerCount.
    [[nodiscard]] std::pair<std::size_t, std::size_t> chunk(
        std::size_t index) const noexcept {
        const std::size_t begin = index * chunkGrain();
        return {begin, std::min(begin + chunkGrain(), count)};
    }

    // Claims chunks until the sweep is exhausted. Every worker runs this, so a
    // worker that finishes early keeps helping instead of idling at a barrier.
    void drain() noexcept {
        while (true) {
            const std::size_t index =
                nextChunk.fetch_add(1, std::memory_order_relaxed);
            if (index >= chunkCount) {
                return;
            }
            const auto [begin, end] = chunk(index);
            try {
                (*body)(begin, end);
            } catch (...) {
                const std::lock_guard<std::mutex> lock(failureMutex);
                // Lowest chunk wins, so the reported failure does not depend
                // on which worker happened to fault first.
                if (index < failedRange) {
                    failedRange = index;
                    failure = std::current_exception();
                }
            }
        }
    }

    const PhaseSize* phaseSize = nullptr;
    const PhaseGrain* phaseGrain = nullptr;
    const PhaseBody* phaseBody = nullptr;
    std::size_t phaseCount = 0;

    std::atomic<unsigned> barrierArrived{0};
    std::atomic<unsigned> barrierGeneration{0};

    // Sense-reversing barrier across all workers. The last to arrive resets
    // the chunk cursor for the next phase and releases everyone; the rest
    // spin, since by construction they are only microseconds from release.
    void barrierWait() noexcept {
        const unsigned generationSeen =
            barrierGeneration.load(std::memory_order_acquire);
        if (barrierArrived.fetch_add(1, std::memory_order_acq_rel) ==
            workerCount - 1) {
            barrierArrived.store(0, std::memory_order_relaxed);
            nextChunk.store(0, std::memory_order_relaxed);
            barrierGeneration.fetch_add(1, std::memory_order_release);
            return;
        }
        spinUntil([&] {
            return barrierGeneration.load(std::memory_order_acquire) !=
                   generationSeen;
        });
    }

    void drainPhase(std::size_t phase) noexcept {
        const std::size_t phaseElements = (*phaseSize)(phase);
        const std::size_t grain = std::max<std::size_t>(1, (*phaseGrain)(phase));
        const std::size_t chunks = (phaseElements + grain - 1) / grain;
        while (true) {
            const std::size_t index =
                nextChunk.fetch_add(1, std::memory_order_relaxed);
            if (index >= chunks) {
                return;
            }
            const std::size_t begin = index * grain;
            const std::size_t end = std::min(begin + grain, phaseElements);
            try {
                (*phaseBody)(phase, begin, end);
            } catch (...) {
                const std::lock_guard<std::mutex> lock(failureMutex);
                // Ordered by (phase, chunk) so the reported failure is the
                // earliest one in sweep order, not the first to be raised.
                const std::size_t ordinal = phase * chunks + index;
                if (ordinal < failedRange) {
                    failedRange = ordinal;
                    failure = std::current_exception();
                }
            }
        }
    }

    void runPhases() noexcept {
        // Every worker walks the same phase sequence and computes the same
        // sizes, so no phase index needs publishing.
        for (std::size_t phase = 0; phase < phaseCount; ++phase) {
            drainPhase(phase);
            barrierWait();
        }
    }

    // seq_cst, not acquire: publishGeneration bumps `generation` and then
    // reads `sleepers` to decide whether to notify. That is a store-load
    // (Dekker) pair, and it only holds if both sides sit in the single total
    // order -- with an acquire load here, a weakly-ordered target could let
    // the bump and the sleepers++ pass each other, the notify be skipped, and
    // this worker sleep forever on work that was already published.
    [[nodiscard]] bool awake(unsigned seen) const noexcept {
        return generation.load(std::memory_order_seq_cst) != seen ||
               stopping.load(std::memory_order_seq_cst);
    }

    void waitForWork(unsigned seen) {
        for (int attempt = 0; attempt < kYieldsBeforeSleep; ++attempt) {
            for (int spin = 0; spin < kSpinsBeforeYield; ++spin) {
                if (awake(seen)) {
                    return;
                }
            }
            std::this_thread::yield();
        }
        // Still nothing: the host is between steps, so give the core back.
        // Registering as a sleeper before re-testing under the lock is what
        // closes the race with forEachRange's sleepers check -- if it missed
        // us, it had already published the new generation, and the predicate
        // below sees it.
        std::unique_lock<std::mutex> lock(sleepMutex);
        sleepers.fetch_add(1);
        wake.wait(lock, [&] { return awake(seen); });
        sleepers.fetch_sub(1);
    }

    void runDispatch() noexcept {
        if (phaseBody != nullptr) {
            runPhases();
        } else {
            drain();
        }
    }

    void workerLoop() {
        unsigned seen = 0;
        while (true) {
            waitForWork(seen);
            if (stopping.load(std::memory_order_acquire)) {
                return;
            }
            seen = generation.load(std::memory_order_acquire);
            runDispatch();
            pending.fetch_sub(1, std::memory_order_release);
        }
    }

    void publishGeneration() {
        generation.fetch_add(1, std::memory_order_seq_cst);
        if (sleepers.load(std::memory_order_seq_cst) > 0) {
            const std::lock_guard<std::mutex> lock(sleepMutex);
            wake.notify_all();
        }
    }
};

WorkerPool::WorkerPool(unsigned workerCount)
    : impl_(std::make_unique<Impl>()) {
    impl_->workerCount = std::max(1u, workerCount);
    // Range 0 runs on the calling thread, so only workerCount - 1 are spawned.
    impl_->threads.reserve(impl_->workerCount - 1);
    for (unsigned index = 1; index < impl_->workerCount; ++index) {
        impl_->threads.emplace_back(
            [impl = impl_.get()] { impl->workerLoop(); });
    }
}

WorkerPool::~WorkerPool() {
    {
        // Under the lock so a worker mid-way into wait() cannot miss this and
        // sleep forever, which would hang the join below.
        const std::lock_guard<std::mutex> lock(impl_->sleepMutex);
        impl_->stopping.store(true, std::memory_order_seq_cst);
        impl_->wake.notify_all();
    }
    for (std::thread& thread : impl_->threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

unsigned WorkerPool::workerCount() const noexcept {
    return impl_->workerCount;
}

void WorkerPool::forEachRange(std::size_t count, const RangeBody& body) {
    if (count == 0) {
        return;
    }
    // One worker, or too little work to cover a wakeup: run inline.
    if (impl_->workerCount == 1 || count == 1) {
        body(0, count);
        return;
    }

    impl_->body = &body;
    impl_->count = count;
    impl_->chunkCount = (count + chunkGrain() - 1) / chunkGrain();
    impl_->nextChunk.store(0, std::memory_order_relaxed);
    impl_->failedRange = std::numeric_limits<std::size_t>::max();
    impl_->failure = nullptr;
    impl_->pending.store(impl_->workerCount - 1, std::memory_order_relaxed);
    impl_->publishGeneration();

    impl_->drain();
    spinUntil([&] {
        return impl_->pending.load(std::memory_order_acquire) == 0;
    });

    impl_->body = nullptr;
    if (impl_->failure) {
        std::rethrow_exception(impl_->failure);
    }
}

void WorkerPool::forEachPhase(std::size_t phaseCount,
                              const PhaseSize& size,
                              const PhaseGrain& grain,
                              const PhaseBody& body) {
    if (phaseCount == 0) {
        return;
    }
    if (impl_->workerCount == 1) {
        for (std::size_t phase = 0; phase < phaseCount; ++phase) {
            const std::size_t elements = size(phase);
            if (elements > 0) {
                body(phase, 0, elements);
            }
        }
        return;
    }

    impl_->phaseSize = &size;
    impl_->phaseGrain = &grain;
    impl_->phaseBody = &body;
    impl_->phaseCount = phaseCount;
    impl_->nextChunk.store(0, std::memory_order_relaxed);
    impl_->barrierArrived.store(0, std::memory_order_relaxed);
    impl_->failedRange = std::numeric_limits<std::size_t>::max();
    impl_->failure = nullptr;
    impl_->pending.store(impl_->workerCount - 1, std::memory_order_relaxed);
    impl_->publishGeneration();

    impl_->runPhases();
    spinUntil([&] {
        return impl_->pending.load(std::memory_order_acquire) == 0;
    });

    impl_->phaseBody = nullptr;
    impl_->phaseGrain = nullptr;
    impl_->phaseSize = nullptr;
    if (impl_->failure) {
        std::rethrow_exception(impl_->failure);
    }
}

WorkerPool* WorkerPoolSlot::get(unsigned workerCount) {
    if (pool_ == nullptr || size_ != workerCount) {
        pool_ = std::make_unique<WorkerPool>(workerCount);
        size_ = workerCount;
    }
    return pool_.get();
}

unsigned hardwareWorkerCount() noexcept {
    const unsigned reported = std::thread::hardware_concurrency();
    return reported == 0 ? 1u : reported;
}

unsigned hardwarePhysicalCoreCount() noexcept {
#ifdef _WIN32
    try {
        DWORD bytes = 0;
        if (GetLogicalProcessorInformationEx(
                RelationProcessorCore, nullptr, &bytes) == FALSE &&
            GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
            return hardwareWorkerCount();
        }
        if (bytes == 0) return hardwareWorkerCount();

        std::vector<unsigned char> buffer(bytes);
        auto* const information =
            reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(
                buffer.data());
        if (GetLogicalProcessorInformationEx(
                RelationProcessorCore, information, &bytes) == FALSE) {
            return hardwareWorkerCount();
        }

        unsigned cores = 0;
        DWORD offset = 0;
        while (offset < bytes) {
            const auto* const entry =
                reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(
                    buffer.data() + offset);
            if (entry->Size == 0 || entry->Size > bytes - offset) {
                return hardwareWorkerCount();
            }
            if (entry->Relationship == RelationProcessorCore) ++cores;
            offset += entry->Size;
        }
        return cores == 0 ? hardwareWorkerCount() : cores;
    } catch (...) {
        return hardwareWorkerCount();
    }
#else
    return hardwareWorkerCount();
#endif
}

}  // namespace softwing
