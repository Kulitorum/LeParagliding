#pragma once

#include <QObject>
#include <QThread>

#include <atomic>

#include "flat_parts.h"
#include "nesting.h"

// Runs a pack off the GUI thread and publishes each improvement as it lands.
//
// Packing a full wing takes seconds, and the search finds its best layouts
// early — so showing them as they arrive is both cheap and the only way the
// user can tell a pack is progressing rather than hung. Progress is throttled
// on this side rather than in the nester: the search can improve many times a
// second in the small-part phase, and repainting a canvas of that size for
// every one would make the packer slower than the thing it is optimising.
class NestWorker : public QObject
{
    Q_OBJECT

public:
    explicit NestWorker(QObject *parent = nullptr);
    ~NestWorker() override;

    // Starts a pack, cancelling any run already in flight. Safe to call again
    // immediately; the previous thread is torn down first.
    void start(const flatparts::FlatPartSet &parts,
               const QVector<int> &indices,
               const flatparts::NestOptions &options);
    void cancel();
    bool isRunning() const;

    // Identifies the run a result belongs to. A cancelled run's final result is
    // already queued to the GUI thread by the time the next run starts, so
    // without this it would arrive late and overwrite the new run's display.
    int generation() const { return generation_; }

signals:
    void progress(const flatparts::NestResult &result, int generation);
    void finished(const flatparts::NestResult &result, int generation);

private:
    void run(flatparts::FlatPartSet parts,
             QVector<int> indices,
             flatparts::NestOptions options,
             int generation);

    QThread *thread_ = nullptr;
    // Read by the worker between iterations; written by the GUI thread.
    std::atomic_bool cancelled_{false};
    std::atomic_int generation_{0};
};
