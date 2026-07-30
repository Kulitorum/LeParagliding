#include "playground_analysis.h"

#include <QCloseEvent>
#include <QColor>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QFontMetricsF>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QPolygonF>
#include <QProgressBar>
#include <QPushButton>
#include <QSplitter>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QThread>
#include <QVBoxLayout>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace {

// Fixed series palette, validated for the app's dark surfaces (lightness
// band, chroma floor and CVD pair separation). Assigned in declaration
// order, never cycled, and the riser rows map fixedly A..F onto it so a
// row keeps its colour across sweeps and designs — colour follows the
// entity, not its rank in whatever happens to be plotted.
constexpr QRgb kSeriesRgb[]{
    0x3987e5,   // blue
    0xd95926,   // orange
    0x199e70,   // aqua
    0xc98500,   // yellow
    0xd55181,   // magenta
    0x008300,   // green
};
constexpr int kSeriesColourCount =
    static_cast<int>(sizeof(kSeriesRgb) / sizeof(kSeriesRgb[0]));

QColor seriesColour(int index)
{
    return QColor::fromRgb(
        kSeriesRgb[std::clamp(index, 0, kSeriesColourCount - 1)]);
}

}  // namespace

struct SweepPlotSeries
{
    QString name;
    QColor colour;
    // x is the angle of attack in degrees.
    std::vector<QPointF> points;
};

// One small line-plot widget, used four times. Deliberate chart
// discipline, shared by all four: ONE y-axis (two measures of different
// scale get two plots, never two scales in one), thin 2 px polylines with
// no point markers except the hover highlight, a recessive grid of ~3
// horizontal lines plus the baseline, tick labels in the palette's
// disabled text colour, and legend text in the NORMAL text colour — the
// colour chip beside it carries identity, text never wears the series
// colour. All chrome comes from the widget palette so the plots follow
// the app theme; only the series hexes are fixed.
class PlaygroundSweepPlot : public QWidget
{
public:
    explicit PlaygroundSweepPlot(const QString &title,
                                 const QString &unit,
                                 QWidget *parent = nullptr)
        : QWidget(parent),
          title_(title),
          unit_(unit)
    {
        setMouseTracking(true);
        setMinimumSize(240, 170);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setData(std::vector<SweepPlotSeries> series,
                 std::vector<double> flaggedAlphas)
    {
        series_ = std::move(series);
        flaggedAlphas_ = std::move(flaggedAlphas);
        alphas_.clear();
        for (const SweepPlotSeries &line : series_) {
            for (const QPointF &point : line.points) {
                alphas_.push_back(point.x());
            }
        }
        std::sort(alphas_.begin(), alphas_.end());
        alphas_.erase(std::unique(alphas_.begin(),
                                  alphas_.end(),
                                  [](double a, double b) {
                                      return std::abs(a - b) < 1.0e-9;
                                  }),
                      alphas_.end());
        computeRanges();
        hoverAlpha_.reset();
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), palette().color(QPalette::Base));

        const QColor ink = palette().color(QPalette::WindowText);
        QColor grid = ink;
        grid.setAlphaF(0.15F);
        const QColor tickInk =
            palette().color(QPalette::Disabled, QPalette::WindowText);

        QFont titleFont = font();
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(ink);
        painter.drawText(QRectF(8.0, 2.0, width() - 16.0, 18.0),
                         Qt::AlignLeft | Qt::AlignVCenter,
                         title_);

        if (alphas_.empty()) {
            return;
        }
        const QRectF area = plotArea();
        if (area.width() < 40.0 || area.height() < 40.0) {
            return;
        }

        QFont tickFont = font();
        tickFont.setBold(false);
        tickFont.setPointSize(9);
        painter.setFont(tickFont);
        const QFontMetricsF metrics(tickFont);

        // Horizontal gridlines at round values, baseline included; no
        // vertical lines — the hover hairline does that job on demand.
        // Label precision follows the step (a 2.5 step needs the .5).
        const int decimals =
            yStep_ >= 1.0 ? (std::floor(yStep_) == yStep_ ? 0 : 1)
                          : (yStep_ >= 0.1 ? 1 : 2);
        for (double value = yMin_; value <= yMax_ + yStep_ * 0.25;
             value += yStep_) {
            const double y = yPixel(value, area);
            painter.setPen(QPen(grid, 1.0));
            painter.drawLine(QPointF(area.left(), y),
                             QPointF(area.right(), y));
            painter.setPen(tickInk);
            painter.drawText(QRectF(0.0,
                                    y - metrics.height() * 0.5,
                                    area.left() - 5.0,
                                    metrics.height()),
                             Qt::AlignRight | Qt::AlignVCenter,
                             QString::number(value, 'f', decimals));
        }

        double xTick = 20.0;
        for (const double candidate : {0.5, 1.0, 2.0, 4.0, 5.0, 10.0}) {
            if ((xMax_ - xMin_) / candidate <= 8.0) {
                xTick = candidate;
                break;
            }
        }
        painter.setPen(tickInk);
        for (double value = std::ceil(xMin_ / xTick) * xTick;
             value <= xMax_ + xTick * 0.25;
             value += xTick) {
            painter.drawText(QRectF(xPixel(value, area) - 30.0,
                                    area.bottom() + 3.0,
                                    60.0,
                                    metrics.height()),
                             Qt::AlignHCenter | Qt::AlignTop,
                             QString::number(value, 'f', xTick < 1.0 ? 1 : 0));
        }

        painter.save();
        painter.setClipRect(area.adjusted(-2.0, -2.0, 2.0, 2.0));
        for (const SweepPlotSeries &line : series_) {
            QPolygonF polyline;
            polyline.reserve(static_cast<int>(line.points.size()));
            for (const QPointF &point : line.points) {
                polyline.append(QPointF(xPixel(point.x(), area),
                                        yPixel(point.y(), area)));
            }
            painter.setPen(QPen(line.colour, 2.0));
            painter.setBrush(Qt::NoBrush);
            if (polyline.size() >= 2) {
                painter.drawPolyline(polyline);
            } else if (polyline.size() == 1) {
                // A one-point sweep has no line to draw.
                painter.drawEllipse(polyline.first(), 2.0, 2.0);
            }
        }
        painter.restore();

        // Alpha points whose report tripped a flag: a hollow triangle
        // sitting on the x baseline, in the text colour so it reads in
        // both themes without borrowing a series colour.
        painter.setPen(QPen(ink, 1.0));
        painter.setBrush(Qt::NoBrush);
        for (const double alpha : flaggedAlphas_) {
            const double x = xPixel(alpha, area);
            const QPointF triangle[3]{QPointF(x, area.bottom() - 7.0),
                                      QPointF(x - 4.0, area.bottom()),
                                      QPointF(x + 4.0, area.bottom())};
            painter.drawPolygon(triangle, 3);
        }

        // No legend for a single series — the title already names it.
        if (series_.size() >= 2) {
            const double rowHeight = metrics.height() + 2.0;
            double y = area.top() + 4.0;
            for (const SweepPlotSeries &line : series_) {
                const double textWidth = metrics.horizontalAdvance(line.name);
                const double x = area.right() - 6.0 - textWidth;
                painter.setPen(Qt::NoPen);
                painter.setBrush(line.colour);
                painter.drawRect(
                    QRectF(x - 12.0, y + rowHeight * 0.5 - 4.0, 8.0, 8.0));
                painter.setPen(ink);
                painter.setBrush(Qt::NoBrush);
                painter.drawText(QRectF(x, y, textWidth + 2.0, rowHeight),
                                 Qt::AlignLeft | Qt::AlignVCenter,
                                 line.name);
                y += rowHeight;
            }
        }

        if (hoverAlpha_) {
            const double alpha = *hoverAlpha_;
            const double x = xPixel(alpha, area);
            QColor hairline = ink;
            hairline.setAlphaF(0.35F);
            painter.setPen(QPen(hairline, 1.0));
            painter.drawLine(QPointF(x, area.top()),
                             QPointF(x, area.bottom()));

            QString readout =
                QStringLiteral("\u03b1 %1\u00b0").arg(alpha, 0, 'f', 1);
            for (const SweepPlotSeries &line : series_) {
                const auto found = std::find_if(
                    line.points.begin(),
                    line.points.end(),
                    [alpha](const QPointF &point) {
                        return std::abs(point.x() - alpha) < 1.0e-6;
                    });
                if (found == line.points.end()) {
                    continue;
                }
                painter.setPen(Qt::NoPen);
                painter.setBrush(line.colour);
                painter.drawEllipse(QPointF(x, yPixel(found->y(), area)),
                                    4.0,
                                    4.0);
                const double value = found->y();
                readout += QStringLiteral("  %1 %2%3")
                               .arg(line.name)
                               .arg(value,
                                    0,
                                    'f',
                                    std::abs(value) >= 100.0 ? 0 : 1)
                               .arg(unit_);
            }
            painter.setPen(ink);
            painter.setBrush(Qt::NoBrush);
            painter.drawText(QRectF(area.left() + 4.0,
                                    area.top() + 2.0,
                                    area.width() - 8.0,
                                    metrics.height()),
                             Qt::AlignLeft | Qt::AlignTop,
                             readout);
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (alphas_.empty()) {
            return;
        }
        const QRectF area = plotArea();
        if (area.width() <= 0.0) {
            return;
        }
        const double alpha = xMin_
                             + (event->position().x() - area.left())
                                   / area.width() * (xMax_ - xMin_);
        const auto nearest = std::min_element(
            alphas_.begin(),
            alphas_.end(),
            [alpha](double a, double b) {
                return std::abs(a - alpha) < std::abs(b - alpha);
            });
        if (!hoverAlpha_ || *hoverAlpha_ != *nearest) {
            hoverAlpha_ = *nearest;
            update();
        }
    }

    void leaveEvent(QEvent *) override
    {
        if (hoverAlpha_) {
            hoverAlpha_.reset();
            update();
        }
    }

private:
    [[nodiscard]] QRectF plotArea() const
    {
        return QRectF(46.0,
                      24.0,
                      std::max(0.0, width() - 46.0 - 8.0),
                      std::max(0.0, height() - 24.0 - 18.0));
    }
    [[nodiscard]] double xPixel(double x, const QRectF &area) const
    {
        return area.left() + (x - xMin_) / (xMax_ - xMin_) * area.width();
    }
    [[nodiscard]] double yPixel(double y, const QRectF &area) const
    {
        return area.bottom() - (y - yMin_) / (yMax_ - yMin_) * area.height();
    }

    // Y bounds snap to round multiples of a round step so the gridlines
    // label as honest numbers; ~4 intervals matches the "3 gridlines plus
    // baseline" the plots are meant to carry.
    void computeRanges()
    {
        if (alphas_.empty()) {
            return;
        }
        xMin_ = alphas_.front();
        xMax_ = alphas_.back();
        if (xMax_ - xMin_ < 1.0e-9) {
            xMin_ -= 1.0;
            xMax_ += 1.0;
        }
        double low = std::numeric_limits<double>::max();
        double high = std::numeric_limits<double>::lowest();
        for (const SweepPlotSeries &line : series_) {
            for (const QPointF &point : line.points) {
                low = std::min(low, point.y());
                high = std::max(high, point.y());
            }
        }
        if (low > high) {
            low = 0.0;
            high = 1.0;
        }
        if (high - low < 1.0e-9) {
            low -= 1.0;
            high += 1.0;
        }
        const double rough = (high - low) / 4.0;
        const double magnitude =
            std::pow(10.0, std::floor(std::log10(rough)));
        double step = magnitude * 10.0;
        for (const double candidate : {1.0, 2.0, 2.5, 5.0}) {
            if (magnitude * candidate >= rough) {
                step = magnitude * candidate;
                break;
            }
        }
        yStep_ = step;
        yMin_ = std::floor(low / step) * step;
        yMax_ = std::ceil(high / step) * step;
        if (yMax_ - yMin_ < step * 0.5) {
            yMax_ = yMin_ + step;
        }
    }

    QString title_;
    // Appended to hover values ("%", " mm", " N"); empty for L/D.
    QString unit_;
    std::vector<SweepPlotSeries> series_;
    std::vector<double> flaggedAlphas_;
    // Sorted union of the series' x values: the hover snaps to these.
    std::vector<double> alphas_;
    std::optional<double> hoverAlpha_;
    double xMin_ = 0.0;
    double xMax_ = 1.0;
    double yMin_ = 0.0;
    double yMax_ = 1.0;
    double yStep_ = 1.0;
};

// The sweep itself, off the GUI thread. The mesh is parsed and refined
// once; each alpha point then gets a FRESH body settled from the rest
// pose — settling one body through a sweep would let each point inherit
// the previous point's deformation history, and hysteresis is exactly
// what this instrument must not fabricate. Results live in the worker
// and are read on the GUI thread only after finished() has been
// delivered: QThread emits finished() after run() returns, so no lock
// and no metatype registration for the report type is needed.
class PlaygroundSweepWorker : public QThread
{
    Q_OBJECT

public:
    PlaygroundSweepWorker(QByteArray meshData,
                          int subdivision,
                          lep::playground::SimBuildOptions options,
                          lep::playground::SimControls controls,
                          std::vector<double> alphaDegrees,
                          double settleSeconds,
                          QObject *parent)
        : QThread(parent),
          meshData_(std::move(meshData)),
          subdivision_(subdivision),
          options_(options),
          controls_(controls),
          alphaDegrees_(std::move(alphaDegrees)),
          settleSeconds_(settleSeconds)
    {
    }

    // Honoured every settle frame (the flag rides into settleAndMeasure),
    // so a cancel returns within milliseconds; the points already
    // measured stay valid.
    void requestCancel() { cancel_.store(true, std::memory_order_relaxed); }
    [[nodiscard]] bool cancelled() const
    {
        return cancel_.load(std::memory_order_relaxed);
    }

    // GUI thread, after finished() only.
    [[nodiscard]] const std::vector<lep::playground::SettleResult> &
    results() const
    {
        return results_;
    }
    [[nodiscard]] const QString &error() const { return error_; }

signals:
    void progress(int done, int total);

protected:
    void run() override
    {
        using namespace lep::playground;
        QString parseError;
        const std::optional<SimMesh> parsed =
            parseSimMesh(meshData_, parseError);
        if (!parsed) {
            error_ = parseError;
            return;
        }
        const SimMesh mesh = refineSimMesh(*parsed, subdivision_);
        const int total = static_cast<int>(alphaDegrees_.size());
        for (int index = 0; index < total; ++index) {
            if (cancelled()) {
                return;
            }
            const double alpha = alphaDegrees_[index];
            SimControls controls = controls_;
            controls.angleOfAttackDegrees = alpha;
            // Always the tunnel with the flight load on: the sweep's
            // whole point is line loads under a realistic resultant, and
            // free flight would make alpha an outcome, not an input.
            controls.freeFlight = false;
            controls.flightLoad = true;
            controls.workerThreads = playgroundWorkerThreads();
            try {
                SimBody sim = buildSimBody(mesh, options_, controls);
                const ShapeBaseline baseline = captureShapeBaseline(sim);
                // The cancel flag goes all the way into the settle loop:
                // a closing dialog waits milliseconds, not a full settle
                // point.
                SettleResult settled = settleAndMeasure(
                    sim, controls, baseline, settleSeconds_, &cancel_);
                if (cancelled()) {
                    return;
                }
                results_.push_back(std::move(settled));
            } catch (const std::exception &failure) {
                error_ = QStringLiteral(
                             "Solver failed at \u03b1=%1\u00b0: %2")
                             .arg(alpha, 0, 'f', 1)
                             .arg(QString::fromUtf8(failure.what()));
                return;
            }
            emit progress(index + 1, total);
        }
    }

private:
    QByteArray meshData_;
    int subdivision_;
    lep::playground::SimBuildOptions options_;
    lep::playground::SimControls controls_;
    std::vector<double> alphaDegrees_;
    double settleSeconds_;
    std::vector<lep::playground::SettleResult> results_;
    QString error_;
    std::atomic<bool> cancel_{false};
};

PlaygroundAnalysisDialog::PlaygroundAnalysisDialog(
    const QByteArray &meshData,
    int subdivision,
    const lep::playground::SimBuildOptions &options,
    const lep::playground::SimControls &controls,
    QWidget *parent)
    : QDialog(parent),
      meshData_(meshData),
      subdivision_(subdivision),
      buildOptions_(options),
      baseControls_(controls)
{
    // The page's profile pointer belongs to the GUI thread's own step
    // loop; the worker must never write through it.
    baseControls_.performanceProfile = nullptr;

    setWindowTitle(tr("Shape analysis"));
    setWindowFlags(Qt::Tool);
    resize(980, 760);

    auto *rootLayout = new QVBoxLayout(this);

    auto *controlsRow = new QHBoxLayout;
    const auto addSpin = [&](const QString &label,
                             double minimum,
                             double maximum,
                             double value,
                             double singleStep,
                             const QString &suffix) {
        controlsRow->addWidget(new QLabel(label, this));
        auto *spin = new QDoubleSpinBox(this);
        spin->setRange(minimum, maximum);
        spin->setDecimals(1);
        spin->setSingleStep(singleStep);
        spin->setValue(value);
        spin->setSuffix(suffix);
        controlsRow->addWidget(spin);
        return spin;
    };
    const QString degree = QStringLiteral("\u00b0");
    alphaFrom_ = addSpin(QStringLiteral("\u03b1 from"),
                         -30.0, 90.0, -4.0, 1.0, degree);
    alphaTo_ = addSpin(tr("to"), -30.0, 90.0, 24.0, 1.0, degree);
    alphaStep_ = addSpin(tr("step"), 0.5, 15.0, 2.0, 0.5, degree);
    settleSeconds_ = addSpin(tr("settle"),
                             2.0, 20.0, 6.0, 1.0, QStringLiteral(" s"));

    runButton_ = new QPushButton(tr("Run"), this);
    controlsRow->addWidget(runButton_);
    progress_ = new QProgressBar(this);
    progress_->setRange(0, 1);
    progress_->setValue(0);
    controlsRow->addWidget(progress_, 1);
    exportButton_ = new QPushButton(tr("Export CSV..."), this);
    exportButton_->setEnabled(false);
    controlsRow->addWidget(exportButton_);
    rootLayout->addLayout(controlsRow);

    auto *splitter = new QSplitter(Qt::Vertical, this);

    auto *plots = new QWidget(splitter);
    auto *grid = new QGridLayout(plots);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(8);
    retentionPlot_ = new PlaygroundSweepPlot(tr("Shape retention (%)"),
                                             QStringLiteral("%"),
                                             plots);
    deviationPlot_ = new PlaygroundSweepPlot(tr("Deviation (mm)"),
                                             QStringLiteral(" mm"),
                                             plots);
    glidePlot_ = new PlaygroundSweepPlot(tr("Glide L/D"), QString(), plots);
    rowLoadPlot_ = new PlaygroundSweepPlot(tr("Riser row loads (N)"),
                                           QStringLiteral(" N"),
                                           plots);
    grid->addWidget(retentionPlot_, 0, 0);
    grid->addWidget(deviationPlot_, 0, 1);
    grid->addWidget(glidePlot_, 1, 0);
    grid->addWidget(rowLoadPlot_, 1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    splitter->addWidget(plots);

    auto *lower = new QWidget(splitter);
    auto *lowerLayout = new QVBoxLayout(lower);
    lowerLayout->setContentsMargins(0, 0, 0, 0);
    table_ = new QTableWidget(lower);
    table_->setColumnCount(11);
    table_->setHorizontalHeaderLabels(
        {QStringLiteral("\u03b1 (\u00b0)"),
         tr("Span %"),
         tr("Vol %"),
         tr("Slack %"),
         tr("Worst dev (mm)"),
         tr("LE dent (mm)"),
         QStringLiteral("Twist (\u00b0)"),
         tr("Line load (N)"),
         tr("L/D"),
         tr("Settled"),
         tr("Flags")});
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setStretchLastSection(true);
    lowerLayout->addWidget(table_);
    status_ = new QLabel(lower);
    lowerLayout->addWidget(status_);
    splitter->addWidget(lower);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    rootLayout->addWidget(splitter, 1);

    connect(runButton_, &QPushButton::clicked,
            this, &PlaygroundAnalysisDialog::toggleRun);
    connect(exportButton_, &QPushButton::clicked,
            this, &PlaygroundAnalysisDialog::exportCsv);
}

PlaygroundAnalysisDialog::~PlaygroundAnalysisDialog()
{
    stopWorker();
}

void PlaygroundAnalysisDialog::closeEvent(QCloseEvent *event)
{
    stopWorker();
    QDialog::closeEvent(event);
}

void PlaygroundAnalysisDialog::toggleRun()
{
    if (worker_ != nullptr) {
        // Second press while running is the cancel; the run state clears
        // when the worker's finished() lands.
        worker_->requestCancel();
        return;
    }
    startSweep();
}

void PlaygroundAnalysisDialog::startSweep()
{
    const double from = alphaFrom_->value();
    const double to = alphaTo_->value();
    const double step = alphaStep_->value();
    int count = 1;
    if (to > from && step > 0.0) {
        count = static_cast<int>(std::floor((to - from) / step + 1.0e-6))
                + 1;
    }
    std::vector<double> alphas;
    alphas.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        alphas.push_back(from + step * index);
    }

    results_.clear();
    exportButton_->setEnabled(false);
    progress_->setRange(0, count);
    progress_->setValue(0);
    status_->setText(tr("sweep 0/%1...").arg(count));

    worker_ = new PlaygroundSweepWorker(meshData_,
                                        subdivision_,
                                        buildOptions_,
                                        baseControls_,
                                        std::move(alphas),
                                        settleSeconds_->value(),
                                        this);
    connect(worker_, &PlaygroundSweepWorker::progress,
            this, &PlaygroundAnalysisDialog::sweepProgress);
    connect(worker_, &QThread::finished,
            this, &PlaygroundAnalysisDialog::sweepFinished);
    setRunning(true);
    worker_->start();
}

void PlaygroundAnalysisDialog::stopWorker()
{
    if (worker_ == nullptr) {
        return;
    }
    worker_->requestCancel();
    // Blocks until run() returns; the cancel flag is honoured between
    // alpha points, so this is at most one settle. Disconnecting first
    // keeps the already-queued finished() from re-entering the normal
    // finish path against a worker this method is about to discard.
    worker_->disconnect(this);
    worker_->wait();
    worker_->deleteLater();
    worker_ = nullptr;
    setRunning(false);
}

void PlaygroundAnalysisDialog::sweepProgress(int done, int total)
{
    progress_->setValue(done);
    status_->setText(tr("sweep %1/%2...").arg(done).arg(total));
}

void PlaygroundAnalysisDialog::sweepFinished()
{
    PlaygroundSweepWorker *worker = worker_;
    if (worker == nullptr) {
        return;
    }
    worker_ = nullptr;
    results_ = worker->results();
    const QString error = worker->error();
    const bool cancelled = worker->cancelled();
    worker->deleteLater();
    setRunning(false);

    exportButton_->setEnabled(!results_.empty());
    populateResults();

    if (!error.isEmpty()) {
        status_->setText(error);
        return;
    }
    QString summary = cancelled
                          ? tr("cancelled, %1 points").arg(results_.size())
                          : tr("%1 points").arg(results_.size());
    bool flagged = false;
    for (const lep::playground::SettleResult &result : results_) {
        if (result.report.flags.empty()) {
            continue;
        }
        summary +=
            QStringLiteral(", first flag at \u03b1=%1\u00b0 (%2)")
                .arg(result.report.alphaDegrees, 0, 'f', 1)
                .arg(lep::playground::shapeFlagName(
                    result.report.flags.front().flag));
        flagged = true;
        break;
    }
    if (!flagged && !results_.empty()) {
        summary += tr(", no flags");
    }
    status_->setText(summary);
}

void PlaygroundAnalysisDialog::populateResults()
{
    using lep::playground::RowLoad;
    using lep::playground::SettleResult;
    using lep::playground::ShapeReport;

    std::vector<double> flagged;
    for (const SettleResult &result : results_) {
        if (!result.report.flags.empty()) {
            flagged.push_back(result.report.alphaDegrees);
        }
    }

    const auto makeSeries = [this](const QString &name,
                                   int colourIndex,
                                   auto value) {
        SweepPlotSeries line;
        line.name = name;
        line.colour = seriesColour(colourIndex);
        line.points.reserve(results_.size());
        for (const SettleResult &result : results_) {
            line.points.push_back(QPointF(result.report.alphaDegrees,
                                          value(result.report)));
        }
        return line;
    };

    retentionPlot_->setData(
        {makeSeries(tr("span"), 0,
                    [](const ShapeReport &report) {
                        return report.spanRatio * 100.0;
                    }),
         makeSeries(tr("vol"), 1,
                    [](const ShapeReport &report) {
                        return report.volumeRatio * 100.0;
                    })},
        flagged);
    deviationPlot_->setData(
        {makeSeries(tr("worst dev"), 0,
                    [](const ShapeReport &report) {
                        return report.worstDeviationMetres * 1000.0;
                    }),
         makeSeries(tr("LE dent"), 1,
                    [](const ShapeReport &report) {
                        return report.worstLeadingEdgeDentMetres * 1000.0;
                    })},
        flagged);
    glidePlot_->setData(
        {makeSeries(tr("L/D"), 0,
                    [](const ShapeReport &report) {
                        return report.glideRatio;
                    })},
        flagged);

    // One series per row letter seen anywhere in the sweep, left+right
    // summed. A row absent from one point's report simply carried no
    // riser segments there and plots as zero.
    std::vector<QChar> rowLetters;
    for (const SettleResult &result : results_) {
        for (const RowLoad &row : result.report.rows) {
            if (std::find(rowLetters.begin(), rowLetters.end(), row.row)
                == rowLetters.end()) {
                rowLetters.push_back(row.row);
            }
        }
    }
    std::sort(rowLetters.begin(), rowLetters.end());
    std::vector<SweepPlotSeries> rowSeries;
    rowSeries.reserve(rowLetters.size());
    for (const QChar letter : rowLetters) {
        SweepPlotSeries line;
        line.name = QString(letter);
        line.colour = seriesColour(letter.toLatin1() - 'A');
        line.points.reserve(results_.size());
        for (const SettleResult &result : results_) {
            double load = 0.0;
            for (const RowLoad &row : result.report.rows) {
                if (row.row == letter) {
                    load += row.leftNewtons + row.rightNewtons;
                }
            }
            line.points.push_back(
                QPointF(result.report.alphaDegrees, load));
        }
        rowSeries.push_back(std::move(line));
    }
    rowLoadPlot_->setData(std::move(rowSeries), flagged);

    table_->setRowCount(static_cast<int>(results_.size()));
    const auto cell = [this](int row, int column, const QString &text) {
        auto *item = new QTableWidgetItem(text);
        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        table_->setItem(row, column, item);
        return item;
    };
    for (int row = 0; row < static_cast<int>(results_.size()); ++row) {
        const SettleResult &result =
            results_[static_cast<std::size_t>(row)];
        const ShapeReport &report = result.report;
        cell(row, 0, QString::number(report.alphaDegrees, 'f', 1));
        cell(row, 1, QString::number(report.spanRatio * 100.0, 'f', 1));
        cell(row, 2, QString::number(report.volumeRatio * 100.0, 'f', 1));
        cell(row, 3, QString::number(report.slackFraction * 100.0, 'f', 1));
        cell(row, 4,
             QStringLiteral("%1 (rib %2)")
                 .arg(report.worstDeviationMetres * 1000.0, 0, 'f', 1)
                 .arg(report.worstDeviationRib));
        cell(row, 5,
             QStringLiteral("%1 (rib %2)")
                 .arg(report.worstLeadingEdgeDentMetres * 1000.0, 0, 'f', 1)
                 .arg(report.worstLeadingEdgeDentRib));
        cell(row, 6,
             QStringLiteral("%1 (rib %2)")
                 .arg(report.worstTwistDegrees, 0, 'f', 1)
                 .arg(report.worstTwistRib));
        cell(row, 7, QString::number(report.lineLoadNewtons, 'f', 0));
        cell(row, 8, QString::number(report.glideRatio, 'f', 2));
        cell(row, 9,
             result.settled
                 ? QStringLiteral("yes (%1 s)")
                       .arg(result.simulatedSeconds, 0, 'f', 1)
                 : tr("no"));
        QStringList names;
        for (const lep::playground::ShapeFlagInfo &info : report.flags) {
            names << lep::playground::shapeFlagName(info.flag);
        }
        // The warning reads through the prefix glyph, not through row
        // colouring: whole tinted rows drown the numbers they sit on.
        auto *flags = cell(row, 10,
                           names.isEmpty()
                               ? QString()
                               : QStringLiteral("\u26a0 ")
                                     + names.join(QStringLiteral(", ")));
        flags->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }
    table_->resizeColumnsToContents();
}

void PlaygroundAnalysisDialog::exportCsv()
{
    if (results_.empty()) {
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this,
        tr("Export sweep"),
        QStringLiteral("shape-sweep.csv"),
        tr("CSV files (*.csv)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate
                   | QIODevice::Text)) {
        status_->setText(
            tr("Export failed: %1").arg(file.errorString()));
        return;
    }
    QTextStream out(&file);
    out << lep::playground::shapeReportCsvHeader() << '\n';
    for (const lep::playground::SettleResult &result : results_) {
        out << lep::playground::shapeReportCsvRow(result.report) << '\n';
    }
    out.flush();
    if (out.status() != QTextStream::Ok
        || file.error() != QFileDevice::NoError) {
        status_->setText(
            tr("Export failed: %1").arg(file.errorString()));
        return;
    }
    status_->setText(tr("Exported %1 rows to %2")
                         .arg(results_.size())
                         .arg(path));
}

void PlaygroundAnalysisDialog::setRunning(bool running)
{
    if (running_ == running) {
        return;
    }
    running_ = running;
    runButton_->setText(running ? tr("Cancel") : tr("Run"));
    for (QDoubleSpinBox *spin :
         {alphaFrom_, alphaTo_, alphaStep_, settleSeconds_}) {
        spin->setEnabled(!running);
    }
    emit sweepRunning(running);
}

#include "playground_analysis.moc"
