#include "nest_worker.h"

#include <QElapsedTimer>
#include <QMetaObject>

namespace {

// Minimum gap between published layouts. Fast enough to look live, slow enough
// that repainting does not starve the search.
constexpr qint64 publishIntervalMs = 150;

} // namespace

NestWorker::NestWorker(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<flatparts::NestResult>("flatparts::NestResult");
}

NestWorker::~NestWorker()
{
    cancel();
}

bool NestWorker::isRunning() const
{
    return thread_ != nullptr && thread_->isRunning();
}

void NestWorker::cancel()
{
    cancelled_ = true;
    if (thread_ != nullptr) {
        thread_->quit();
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
    }
}

void NestWorker::start(const flatparts::FlatPartSet &parts,
                       const QVector<int> &indices,
                       const flatparts::NestOptions &options)
{
    cancel();
    cancelled_ = false;
    const int generation = ++generation_;

    // The parts set is copied into the worker deliberately: the GUI can rebuild
    // it from a new engine run while a pack is still winding down, and a
    // reference would then dangle.
    thread_ = QThread::create([this, parts, indices, options, generation] {
        run(parts, indices, options, generation);
    });
    thread_->start();
}

void NestWorker::run(flatparts::FlatPartSet parts,
                     QVector<int> indices,
                     flatparts::NestOptions options,
                     int generation)
{
    QElapsedTimer sincePublish;
    sincePublish.start();
    bool published = false;

    flatparts::NestCallbacks callbacks;
    callbacks.improved = [&](const flatparts::NestResult &snapshot) {
        // Throttled, but the last improvement of a run must not be swallowed by
        // the throttle — nest() returns it too, and finished() carries it.
        if (published && sincePublish.elapsed() < publishIntervalMs) {
            return;
        }
        published = true;
        sincePublish.restart();
        emit progress(snapshot, generation);
    };
    callbacks.stopRequested = [this] { return cancelled_.load(); };

    const flatparts::NestResult result =
        flatparts::nest(parts, indices, options, callbacks);

    // Always report the final layout, cancelled or not. A run stopped by the
    // user still produced the best layout found so far, and that is exactly the
    // one they want to keep.
    emit finished(result, generation);
}
