#include "airfoil_panel.h"

#include "section_grid.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSlider>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace {

const QColor kBackground(0x0e, 0x17, 0x26);
const QColor kBorder(0x2a, 0x3a, 0x50);
const QColor kAxisInk(0x93, 0xa4, 0xba);
const QColor kBrightInk(0xe6, 0xed, 0xf7);
const QColor kBubbleBackground(0x1b, 0x29, 0x3c);
const QColor kSegmentColors[3] = {
    QColor(0x39, 0x87, 0xe5), // extrados: blue
    QColor(0xd9, 0x59, 0x26), // air intake: orange
    QColor(0x19, 0x9e, 0x70), // intrados: aqua
};
const QString kOkColor = QStringLiteral("#93a4ba");
const QString kWarningColor = QStringLiteral("#fab219");
const QString kErrorColor = QStringLiteral("#e66767");
constexpr double kHandleHitRadius = 12.0;

double sliderToPercent(int position)
{
    return std::pow(10.0, position / 100.0 - 2.0);
}

int percentToSlider(double percent)
{
    const double clamped = std::clamp(percent, 0.01, 10.0);
    return static_cast<int>(std::lround((std::log10(clamped) + 2.0) * 100.0));
}

QJsonObject segmentToJson(const lep::BSpline2D &spline,
                          const std::vector<double> &stations)
{
    QJsonObject object;
    object.insert(QStringLiteral("degree"), spline.degree);
    QJsonArray knots;
    for (const double knot : spline.knots)
        knots.append(knot);
    object.insert(QStringLiteral("knots"), knots);
    QJsonArray controlX;
    for (const double value : spline.controlX)
        controlX.append(value);
    object.insert(QStringLiteral("controlX"), controlX);
    QJsonArray controlY;
    for (const double value : spline.controlY)
        controlY.append(value);
    object.insert(QStringLiteral("controlY"), controlY);
    QJsonArray stationArray;
    for (const double value : stations)
        stationArray.append(value);
    object.insert(QStringLiteral("stations"), stationArray);
    return object;
}

void segmentFromJson(const QJsonObject &object, lep::BSpline2D *spline,
                     std::vector<double> *stations)
{
    *spline = lep::BSpline2D();
    stations->clear();
    spline->degree = object.value(QStringLiteral("degree")).toInt();
    for (const QJsonValue &value :
         object.value(QStringLiteral("knots")).toArray())
        spline->knots.push_back(value.toDouble());
    for (const QJsonValue &value :
         object.value(QStringLiteral("controlX")).toArray())
        spline->controlX.push_back(value.toDouble());
    for (const QJsonValue &value :
         object.value(QStringLiteral("controlY")).toArray())
        spline->controlY.push_back(value.toDouble());
    for (const QJsonValue &value :
         object.value(QStringLiteral("stations")).toArray())
        stations->push_back(value.toDouble());
}

QVector<QPointF> sampleSpline(const lep::BSpline2D &spline, int count)
{
    QVector<QPointF> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i) {
        const double u = static_cast<double>(i) / (count - 1);
        double x = 0.0;
        double y = 0.0;
        spline.evaluate(u, &x, &y);
        result.append(QPointF(x, y));
    }
    return result;
}

} // namespace

// ---------------------------------------------------------------- view --

AirfoilView::AirfoilView(QWidget *parent) : QWidget(parent)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::ClickFocus);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void AirfoilView::setSegments(const QVector<SegmentDisplay> &segments)
{
    segments_ = segments;
    dragging_ = false;
    hoverSegment_ = hoverIndex_ = -1;
    if (activeSegment_ >= segments_.size()
        || (activeSegment_ >= 0
            && activeIndex_ >= segments_.at(activeSegment_).handles.size()))
        activeSegment_ = activeIndex_ = -1;
    update();
}

void AirfoilView::setSegmentSmooth(int segment, const QVector<QPointF> &smooth)
{
    if (segment < 0 || segment >= segments_.size())
        return;
    segments_[segment].smooth = smooth;
    update();
}

void AirfoilView::setHandlePosition(int segment, int index,
                                    const QPointF &position)
{
    if (segment < 0 || segment >= segments_.size())
        return;
    if (index < 0 || index >= segments_.at(segment).handles.size())
        return;
    segments_[segment].handles[index] = position;
    update();
}

QPointF AirfoilView::handlePosition(int segment, int index) const
{
    if (segment < 0 || segment >= segments_.size())
        return QPointF();
    if (index < 0 || index >= segments_.at(segment).handles.size())
        return QPointF();
    return segments_.at(segment).handles.at(index);
}

void AirfoilView::setMessage(const QString &message)
{
    message_ = message;
    update();
}

AirfoilView::Mapping AirfoilView::mapping() const
{
    double minX = 0.0, maxX = 1.0, minY = -0.1, maxY = 0.1;
    bool first = true;
    for (const SegmentDisplay &segment : segments_) {
        for (const QVector<QPointF> *set :
             {&segment.points, &segment.smooth, &segment.handles}) {
            for (const QPointF &point : *set) {
                if (first) {
                    minX = maxX = point.x();
                    minY = maxY = point.y();
                    first = false;
                } else {
                    minX = std::min(minX, point.x());
                    maxX = std::max(maxX, point.x());
                    minY = std::min(minY, point.y());
                    maxY = std::max(maxY, point.y());
                }
            }
        }
    }
    const double margin = 26.0;
    const double dx = std::max(maxX - minX, 1e-6);
    const double dy = std::max(maxY - minY, 1e-6);
    Mapping m;
    m.scale = std::min((width() - 2.0 * margin) / dx,
                       (height() - 2.0 * margin) / dy);
    m.offsetX = (width() - dx * m.scale) / 2.0 - minX * m.scale;
    m.offsetY = (height() + dy * m.scale) / 2.0 + minY * m.scale;
    return m;
}

QPointF AirfoilView::toPixel(const QPointF &point, const Mapping &m) const
{
    return QPointF(point.x() * m.scale + m.offsetX,
                   m.offsetY - point.y() * m.scale);
}

QPointF AirfoilView::fromPixel(const QPointF &pixel, const Mapping &m) const
{
    return QPointF((pixel.x() - m.offsetX) / m.scale,
                   (m.offsetY - pixel.y()) / m.scale);
}

bool AirfoilView::findHandle(const QPointF &pixel, int *segment,
                             int *index) const
{
    const Mapping m = mapping();
    double best = kHandleHitRadius;
    *segment = -1;
    *index = -1;
    for (int s = 0; s < segments_.size(); ++s) {
        const QVector<QPointF> &handles = segments_.at(s).handles;
        for (int i = 0; i < handles.size(); ++i) {
            const QPointF p = toPixel(handles.at(i), m);
            const double distance =
                std::hypot(p.x() - pixel.x(), p.y() - pixel.y());
            if (distance <= best) {
                best = distance;
                *segment = s;
                *index = i;
            }
        }
    }
    return *segment >= 0;
}

void AirfoilView::updateHover(const QPointF &pixel)
{
    int segment = -1;
    int index = -1;
    findHandle(pixel, &segment, &index);
    if (segment != hoverSegment_ || index != hoverIndex_) {
        hoverSegment_ = segment;
        hoverIndex_ = index;
        update();
    }
    if (segment >= 0)
        setCursor(Qt::SizeAllCursor);
    else
        unsetCursor();
}

void AirfoilView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(kBorder, 1.0));
    painter.setBrush(kBackground);
    painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                            6.0, 6.0);

    const QFontMetrics metrics(font());
    if (!message_.isEmpty() || segments_.isEmpty()) {
        painter.setPen(kAxisInk);
        painter.drawText(rect().adjusted(16, 16, -16, -16),
                         Qt::AlignCenter | Qt::TextWordWrap,
                         message_.isEmpty() ? QStringLiteral("No airfoil")
                                            : message_);
        return;
    }

    const Mapping m = mapping();

    // Chord baseline for orientation.
    painter.setPen(QPen(QColor(0x1b, 0x28, 0x40), 1.0));
    const QPointF chord0 = toPixel(QPointF(0.0, 0.0), m);
    const QPointF chord1 = toPixel(QPointF(1.0, 0.0), m);
    painter.drawLine(chord0, chord1);

    // Legend, one label per segment.
    double legendX = 12.0;
    for (const SegmentDisplay &segment : segments_) {
        painter.setPen(segment.color);
        painter.drawText(QPointF(legendX, 10.0 + metrics.ascent()),
                         segment.label);
        legendX += metrics.horizontalAdvance(segment.label) + 16.0;
    }

    for (int s = 0; s < segments_.size(); ++s) {
        const SegmentDisplay &segment = segments_.at(s);
        const bool splineMode = !segment.handles.isEmpty();

        QPolygonF polyline;
        const QVector<QPointF> &source =
            segment.smooth.isEmpty() ? segment.points : segment.smooth;
        polyline.reserve(source.size());
        for (const QPointF &point : source)
            polyline.append(toPixel(point, m));
        QPen pen(segment.color, 2.0);
        pen.setCapStyle(Qt::RoundCap);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        painter.drawPolyline(polyline);

        if (splineMode) {
            // Passive dots: the sampled points written to the file.
            painter.setPen(Qt::NoPen);
            QColor dot = segment.color;
            dot.setAlpha(180);
            painter.setBrush(dot);
            for (const QPointF &point : segment.points)
                painter.drawEllipse(toPixel(point, m), 1.8, 1.8);

            // Control polygon and square handles.
            QPolygonF controls;
            controls.reserve(segment.handles.size());
            for (const QPointF &handle : segment.handles)
                controls.append(toPixel(handle, m));
            QColor polygonColor = segment.color;
            polygonColor.setAlpha(140);
            painter.setPen(QPen(polygonColor, 1.0, Qt::DashLine));
            painter.drawPolyline(controls);
            for (int i = 0; i < controls.size(); ++i) {
                const bool emphasized =
                    (s == hoverSegment_ && i == hoverIndex_)
                    || (s == activeSegment_ && i == activeIndex_);
                const double radius = emphasized ? 5.5 : 3.8;
                painter.setPen(QPen(kBackground, 1.6));
                painter.setBrush(segment.color);
                painter.drawRect(QRectF(controls.at(i).x() - radius,
                                        controls.at(i).y() - radius,
                                        radius * 2.0, radius * 2.0));
            }
        } else {
            painter.setPen(Qt::NoPen);
            QColor dot = segment.color;
            dot.setAlpha(200);
            painter.setBrush(dot);
            for (const QPointF &pixel : polyline)
                painter.drawEllipse(pixel, 1.6, 1.6);
        }
    }

    // Coordinate bubble for the hovered/active handle.
    const int bubbleSegment = dragging_ ? activeSegment_ : hoverSegment_;
    const int bubbleIndex = dragging_ ? activeIndex_ : hoverIndex_;
    if (bubbleSegment >= 0 && bubbleSegment < segments_.size()
        && bubbleIndex >= 0
        && bubbleIndex < segments_.at(bubbleSegment).handles.size()) {
        const QPointF handle =
            segments_.at(bubbleSegment).handles.at(bubbleIndex);
        const QPointF anchor = toPixel(handle, m);
        const QString text = QStringLiteral("x %1 · y %2")
                                 .arg(QString::number(handle.x(), 'f', 4))
                                 .arg(QString::number(handle.y(), 'f', 4));
        const QSizeF size(metrics.horizontalAdvance(text) + 14.0,
                          metrics.height() + 8.0);
        QPointF corner(anchor.x() + 12.0, anchor.y() - size.height() - 8.0);
        if (corner.x() + size.width() > width() - 6.0)
            corner.setX(anchor.x() - size.width() - 12.0);
        if (corner.y() < 6.0)
            corner.setY(anchor.y() + 12.0);
        const QRectF bubble(corner, size);
        painter.setPen(QPen(segments_.at(bubbleSegment).color, 1.0));
        painter.setBrush(kBubbleBackground);
        painter.drawRoundedRect(bubble, 4.0, 4.0);
        painter.setPen(kBrightInk);
        painter.drawText(bubble, Qt::AlignCenter, text);
    }
}

void AirfoilView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }
    int segment = -1;
    int index = -1;
    if (findHandle(event->position(), &segment, &index)) {
        activeSegment_ = segment;
        activeIndex_ = index;
        dragging_ = true;
        setCursor(Qt::SizeAllCursor);
        update();
    }
}

void AirfoilView::mouseMoveEvent(QMouseEvent *event)
{
    if (!dragging_) {
        updateHover(event->position());
        return;
    }
    const QPointF position = fromPixel(event->position(), mapping());
    segments_[activeSegment_].handles[activeIndex_] = position;
    update();
    emit handleMoved(activeSegment_, activeIndex_, position);
}

void AirfoilView::mouseReleaseEvent(QMouseEvent *event)
{
    if (!dragging_ || event->button() != Qt::LeftButton) {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    dragging_ = false;
    update();
    emit editCommitted(); // may re-enter setSegments(); keep this last
}

void AirfoilView::keyPressEvent(QKeyEvent *event)
{
    if (event->matches(QKeySequence::Undo)) {
        emit undoRequested();
        return;
    }
    if (event->matches(QKeySequence::Redo)) {
        emit redoRequested();
        return;
    }
    if (activeSegment_ < 0 || activeSegment_ >= segments_.size()
        || activeIndex_ < 0
        || activeIndex_ >= segments_.at(activeSegment_).handles.size()) {
        QWidget::keyPressEvent(event);
        return;
    }
    double dx = 0.0;
    double dy = 0.0;
    switch (event->key()) {
    case Qt::Key_Left:
        dx = -1.0;
        break;
    case Qt::Key_Right:
        dx = 1.0;
        break;
    case Qt::Key_Up:
        dy = 1.0;
        break;
    case Qt::Key_Down:
        dy = -1.0;
        break;
    default:
        QWidget::keyPressEvent(event);
        return;
    }
    double step = 0.0005;
    if (event->modifiers().testFlag(Qt::ShiftModifier))
        step *= 10.0;
    QPointF position = segments_.at(activeSegment_).handles.at(activeIndex_);
    position += QPointF(dx * step, dy * step);
    segments_[activeSegment_].handles[activeIndex_] = position;
    update();
    const int segment = activeSegment_;
    const int index = activeIndex_;
    emit handleMoved(segment, index, position);
    emit editCommitted();
}

void AirfoilView::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);
    hoverSegment_ = hoverIndex_ = -1;
    update();
}

// --------------------------------------------------------------- panel --

AirfoilPanel::AirfoilPanel(
    QPlainTextEdit *editor, std::function<QString()> designDirectory,
    std::function<QJsonObject()> loadSplines,
    std::function<void(const QJsonObject &)> storeSplines, QWidget *parent)
    : QWidget(parent), editor_(editor),
      designDirectory_(std::move(designDirectory)),
      loadSplines_(std::move(loadSplines)),
      storeSplines_(std::move(storeSplines))
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto *header = new QHBoxLayout;
    header->setSpacing(6);
    auto *fileLabel = new QLabel(QStringLiteral("Airfoil file"), this);
    fileLabel->setObjectName(QStringLiteral("hint"));
    header->addWidget(fileLabel);
    fileCombo_ = new QComboBox(this);
    fileCombo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    fileCombo_->setMinimumContentsLength(20);
    header->addWidget(fileCombo_, 0);
    header->addStretch();
    layout->addLayout(header);

    view_ = new AirfoilView(this);
    layout->addWidget(view_, 1);

    description_ = new QLabel(this);
    description_->setObjectName(QStringLiteral("hint"));
    description_->setWordWrap(true);
    layout->addWidget(description_);

    auto *statusRow = new QHBoxLayout;
    statusRow->setSpacing(6);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    statusRow->addWidget(status_, 1);
    refitButton_ = new QPushButton(QStringLiteral("Re-fit spline"), this);
    refitButton_->setObjectName(QStringLiteral("quietButton"));
    statusRow->addWidget(refitButton_);
    applyButton_ = new QPushButton(QStringLiteral("Apply spline"), this);
    applyButton_->setObjectName(QStringLiteral("quietButton"));
    applyButton_->setToolTip(QStringLiteral(
        "Rewrite the airfoil file from the stored B-splines"));
    statusRow->addWidget(applyButton_);
    convertButton_ =
        new QPushButton(QStringLiteral("Convert to B-splines…"), this);
    convertButton_->setObjectName(QStringLiteral("quietButton"));
    convertButton_->setToolTip(QStringLiteral(
        "Fit the three contour segments with B-splines. A regenerated "
        "\"<name>-spline.txt\" is written beside the design and the rows "
        "point at it; the original file is never modified."));
    statusRow->addWidget(convertButton_);
    removeButton_ = new QPushButton(QStringLiteral("Remove splines"), this);
    removeButton_->setObjectName(QStringLiteral("quietButton"));
    statusRow->addWidget(removeButton_);
    layout->addLayout(statusRow);

    connect(fileCombo_, &QComboBox::currentIndexChanged, this,
            [this](int) { rebuildView(); });
    connect(convertButton_, &QPushButton::clicked, this,
            [this] { showConvertDialog(); });
    connect(refitButton_, &QPushButton::clicked, this,
            [this] { showConvertDialog(); });
    connect(removeButton_, &QPushButton::clicked, this,
            [this] { removeSplines(); });
    connect(applyButton_, &QPushButton::clicked, this,
            [this] { applySplineToFile(); });
    connect(editor_, &QPlainTextEdit::textChanged, this, [this] {
        if (!applyingEdit_)
            syncFromText();
    });
    connect(view_, &AirfoilView::handleMoved, this,
            [this](int segment, int index, const QPointF &position) {
                LoadedAirfoil *airfoil = currentAirfoil();
                if (previewActive_ || airfoil == nullptr
                    || airfoil->splines.empty())
                    return;
                // Weld the boundary handle shared with the neighbour, so
                // the contour stays connected at the segment joins.
                const int lastOfSegment =
                    static_cast<int>(
                        airfoil->splines[static_cast<size_t>(segment)]
                            .controlX.size())
                    - 1;
                if (index == 0 && segment > 0) {
                    view_->setHandlePosition(
                        segment - 1,
                        static_cast<int>(
                            airfoil->splines[static_cast<size_t>(segment)
                                             - 1]
                                .controlX.size())
                            - 1,
                        position);
                } else if (index == lastOfSegment
                           && segment + 1 < static_cast<int>(
                                  airfoil->splines.size())) {
                    view_->setHandlePosition(segment + 1, 0, position);
                }
                // Morph the neighbourhood live, pulling every handle from
                // the view — the single source of truth mid-drag.
                for (int s = std::max(0, segment - 1);
                     s <= std::min(
                         static_cast<int>(airfoil->splines.size()) - 1,
                         segment + 1);
                     ++s) {
                    lep::BSpline2D moved =
                        airfoil->splines[static_cast<size_t>(s)];
                    for (size_t c = 0; c < moved.controlX.size(); ++c) {
                        const QPointF handle = view_->handlePosition(
                            s, static_cast<int>(c));
                        moved.controlX[c] = handle.x();
                        moved.controlY[c] = handle.y();
                    }
                    view_->setSegmentSmooth(s, sampleSpline(moved, 120));
                }
            });
    connect(view_, &AirfoilView::editCommitted, this, [this] {
        if (!previewActive_)
            commitHandleEdit();
    });
    connect(view_, &AirfoilView::undoRequested, this, [this] {
        if (const UndoState *state = undo_.undo())
            restoreState(*state);
    });
    connect(view_, &AirfoilView::redoRequested, this, [this] {
        if (const UndoState *state = undo_.redo())
            restoreState(*state);
    });

    loadSplinesFromDocument();
    syncFromText();
}

AirfoilPanel::UndoState AirfoilPanel::captureState() const
{
    UndoState state;
    state.text = editor_->toPlainText();
    state.splines = loadSplines_ ? loadSplines_() : QJsonObject();
    for (const LoadedAirfoil &airfoil : airfoils_) {
        QFile file(airfoil.absolutePath);
        if (file.open(QIODevice::ReadOnly))
            state.files.insert(airfoil.absolutePath, file.readAll());
    }
    return state;
}

void AirfoilPanel::pushUndo(UndoState before)
{
    UndoState after = captureState();
    if (before.text == after.text && before.splines == after.splines
        && before.files == after.files)
        return;
    undo_.push(std::move(before), std::move(after));
}

void AirfoilPanel::restoreState(const UndoState &state)
{
    for (auto it = state.files.constBegin(); it != state.files.constEnd();
         ++it) {
        QFile current(it.key());
        if (current.open(QIODevice::ReadOnly)
            && current.readAll() == it.value())
            continue;
        current.close();
        QSaveFile file(it.key());
        if (file.open(QIODevice::WriteOnly)) {
            file.write(it.value());
            file.commit();
        }
    }
    if (storeSplines_)
        storeSplines_(state.splines);
    loadSplinesFromDocument();
    if (editor_->toPlainText() != state.text) {
        applyingEdit_ = true;
        QTextCursor cursor(editor_->document());
        cursor.select(QTextCursor::Document);
        cursor.insertText(state.text);
        applyingEdit_ = false;
    }
    syncFromText();
}

void AirfoilPanel::loadSplinesFromDocument()
{
    tolerancePercent_ = loadSplines_
                            ? loadSplines_()
                                  .value(QStringLiteral("airfoils"))
                                  .toObject()
                                  .value(QStringLiteral("tolerancePercent"))
                                  .toDouble(0.2)
                            : 0.2;
}

AirfoilPanel::LoadedAirfoil *AirfoilPanel::currentAirfoil()
{
    const int index = fileCombo_->currentIndex();
    if (index < 0 || index >= static_cast<int>(airfoils_.size()))
        return nullptr;
    return &airfoils_[static_cast<size_t>(index)];
}

void AirfoilPanel::syncFromText()
{
    const QString previousSelection = fileCombo_->currentText();
    airfoils_.clear();

    const lep::SectionGrid grid =
        lep::parseSectionGrid(editor_->toPlainText().toStdString());
    QStringList names;
    for (const lep::GridRow &row : grid.rows) {
        if (row.tokens.size() < 2)
            continue;
        const QString name = QString::fromStdString(row.tokens[1].text);
        if (!names.contains(name))
            names.append(name);
    }

    const QString directory = designDirectory_ ? designDirectory_()
                                               : QString();
    const QJsonObject airfoilRoot =
        loadSplines_ ? loadSplines_()
                           .value(QStringLiteral("airfoils"))
                           .toObject()
                           .value(QStringLiteral("files"))
                           .toObject()
                     : QJsonObject();

    for (const QString &name : names) {
        LoadedAirfoil loaded;
        loaded.referencedName = name;
        loaded.absolutePath = QDir(directory).filePath(name);
        if (!name.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive)) {
            loaded.problem = QStringLiteral(
                "Only .txt airfoil files can be shown here.");
        } else {
            QFile file(loaded.absolutePath);
            if (!file.open(QIODevice::ReadOnly)) {
                loaded.problem =
                    QStringLiteral("The file was not found next to the "
                                   "design (%1).")
                        .arg(loaded.absolutePath);
            } else {
                std::string parseError;
                if (!lep::parseAirfoilFile(
                        QString::fromUtf8(file.readAll()).toStdString(),
                        &loaded.file, &parseError)) {
                    loaded.problem = QString::fromStdString(parseError);
                } else if (!parseError.empty()) {
                    loaded.problem = QString::fromStdString(parseError);
                }
            }
        }

        // Stored splines for this file, one per contour segment, with the
        // fixed sampling stations captured at fit time.
        if (loaded.problem.isEmpty()) {
            const QJsonArray segments = airfoilRoot.value(name)
                                            .toObject()
                                            .value(QStringLiteral("segments"))
                                            .toArray();
            const auto fileSegments = loaded.file.segments();
            if (static_cast<size_t>(segments.size())
                == fileSegments.size()) {
                bool usable = true;
                for (qsizetype s = 0; s < segments.size(); ++s) {
                    lep::BSpline2D spline;
                    std::vector<double> stations;
                    segmentFromJson(segments.at(s).toObject(), &spline,
                                    &stations);
                    const int pointCount =
                        fileSegments[static_cast<size_t>(s)].last
                        - fileSegments[static_cast<size_t>(s)].first + 1;
                    if (!spline.isValid()
                        || static_cast<int>(stations.size())
                               != pointCount) {
                        usable = false;
                        break;
                    }
                    loaded.splines.push_back(spline);
                    loaded.stations.push_back(std::move(stations));
                }
                if (!usable) {
                    loaded.splines.clear();
                    loaded.stations.clear();
                }
            }
        }

        // Staleness: the file must equal the splines sampled at the
        // stored stations, give or take the file's 6-decimal rounding.
        if (!loaded.splines.empty()) {
            const auto fileSegments = loaded.file.segments();
            for (size_t s = 0;
                 s < fileSegments.size() && !loaded.stale; ++s) {
                for (size_t i = 0; i < loaded.stations[s].size(); ++i) {
                    double x = 0.0;
                    double y = 0.0;
                    loaded.splines[s].evaluate(loaded.stations[s][i], &x,
                                               &y);
                    const size_t index =
                        static_cast<size_t>(fileSegments[s].first) + i;
                    if (std::hypot(x - loaded.file.xs[index],
                                   y - loaded.file.ys[index])
                        > 2e-6) {
                        loaded.stale = true;
                        break;
                    }
                }
            }
        }
        airfoils_.push_back(std::move(loaded));
    }

    fileCombo_->blockSignals(true);
    fileCombo_->clear();
    for (const LoadedAirfoil &airfoil : airfoils_)
        fileCombo_->addItem(airfoil.referencedName);
    const int restored = fileCombo_->findText(previousSelection);
    fileCombo_->setCurrentIndex(std::max(0, restored));
    fileCombo_->blockSignals(false);
    fileCombo_->setEnabled(airfoils_.size() > 1);

    rebuildView();
}

void AirfoilPanel::rebuildView()
{
    LoadedAirfoil *airfoil = currentAirfoil();
    if (airfoil == nullptr) {
        view_->setSegments({});
        view_->setMessage(QStringLiteral(
            "No airfoil rows found — each Section 2 record names the "
            "profile file for one rib."));
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
        status_->clear();
        description_->clear();
        convertButton_->setVisible(false);
        removeButton_->setVisible(false);
        refitButton_->setVisible(false);
        applyButton_->setVisible(false);
        return;
    }
    if (!airfoil->problem.isEmpty() && airfoil->file.totalPoints() == 0) {
        view_->setSegments({});
        view_->setMessage(airfoil->problem);
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(airfoil->problem);
        description_->clear();
        convertButton_->setVisible(false);
        removeButton_->setVisible(false);
        refitButton_->setVisible(false);
        applyButton_->setVisible(false);
        return;
    }

    const auto fileSegments = airfoil->file.segments();
    const char *labels3[3] = {"Extrados", "Air intake", "Intrados"};
    const std::vector<lep::BSpline2D> &splines =
        previewActive_ ? previewSplines_ : airfoil->splines;

    QVector<AirfoilView::SegmentDisplay> display;
    for (size_t s = 0; s < fileSegments.size(); ++s) {
        AirfoilView::SegmentDisplay segment;
        segment.color = kSegmentColors[s % 3];
        segment.label = fileSegments.size() == 3
                            ? QString::fromLatin1(labels3[s])
                            : QStringLiteral("Contour");
        for (int i = fileSegments[s].first; i <= fileSegments[s].last; ++i)
            segment.points.append(
                QPointF(airfoil->file.xs[static_cast<size_t>(i)],
                        airfoil->file.ys[static_cast<size_t>(i)]));
        if (s < splines.size() && splines[s].isValid()) {
            segment.smooth = sampleSpline(splines[s], 120);
            for (size_t c = 0; c < splines[s].controlX.size(); ++c)
                segment.handles.append(QPointF(splines[s].controlX[c],
                                               splines[s].controlY[c]));
        }
        display.append(segment);
    }
    view_->setMessage(QString());
    view_->setSegments(display);

    description_->setText(
        QStringLiteral("<b>%1</b> · %2 points (extrados %3 · intake %4 · "
                       "intrados %5) · one file can serve many ribs, so "
                       "edits affect every rib that references it")
            .arg(airfoil->referencedName)
            .arg(airfoil->file.totalPoints())
            .arg(airfoil->file.extradosPoints)
            .arg(airfoil->file.intakePoints)
            .arg(airfoil->file.intradosPoints));

    const bool splineBacked = !airfoil->splines.empty();
    convertButton_->setVisible(!previewActive_);
    convertButton_->setText(splineBacked
                                ? QStringLiteral("Re-fit B-splines…")
                                : QStringLiteral("Convert to B-splines…"));
    removeButton_->setVisible(splineBacked && !previewActive_);
    refitButton_->setVisible(false);
    applyButton_->setVisible(airfoil->stale && !previewActive_);

    if (previewActive_) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral(
            "Previewing B-spline conversion — Convert applies it, Cancel "
            "restores the current shape."));
    } else if (airfoil->stale) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral(
            "⚠ The file no longer matches its B-splines (edited outside "
            "the spline editor?) — Apply spline rewrites the file, or "
            "re-fit from the file's current shape."));
        refitButton_->setVisible(true);
    } else if (!airfoil->problem.isEmpty()) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral("⚠ ") + airfoil->problem);
    } else if (splineBacked) {
        int controls = 0;
        for (const lep::BSpline2D &spline : airfoil->splines)
            controls += static_cast<int>(spline.controlX.size());
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
        status_->setText(
            QStringLiteral("B-spline backed · %1 control points · drag "
                           "squares to reshape (arrows nudge, Shift = ×10, "
                           "Ctrl+Z undoes) · the file regenerates on every "
                           "edit · press Enter in the text to rebuild the "
                           "3D preview")
                .arg(controls));
    } else {
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
        status_->setText(QStringLiteral(
            "Point-by-point file · Convert to B-splines to edit the shape "
            "with control handles. The original file stays untouched; a "
            "regenerated copy takes its place in the rows."));
    }
}

lep::AirfoilFile AirfoilPanel::sampledFromSplines(
    const lep::AirfoilFile &reference,
    const std::vector<lep::BSpline2D> &splines,
    const std::vector<std::vector<double>> &stations) const
{
    lep::AirfoilFile result = reference;
    const auto fileSegments = reference.segments();
    for (size_t s = 0; s < fileSegments.size() && s < splines.size(); ++s) {
        const size_t pointCount =
            static_cast<size_t>(fileSegments[s].last - fileSegments[s].first)
            + 1;
        // The stored stations are the sampling policy; fall back to the
        // reference's own chord parameters only when they are absent.
        std::vector<double> params;
        if (s < stations.size() && stations[s].size() == pointCount) {
            params = stations[s];
        } else {
            std::vector<double> xs, ys;
            for (int i = fileSegments[s].first; i <= fileSegments[s].last;
                 ++i) {
                xs.push_back(reference.xs[static_cast<size_t>(i)]);
                ys.push_back(reference.ys[static_cast<size_t>(i)]);
            }
            params = lep::chordParameters(xs, ys);
        }
        for (size_t j = 0; j < params.size(); ++j) {
            double x = 0.0;
            double y = 0.0;
            splines[s].evaluate(params[j], &x, &y);
            result.xs[static_cast<size_t>(fileSegments[s].first) + j] = x;
            result.ys[static_cast<size_t>(fileSegments[s].first) + j] = y;
        }
    }
    return result;
}

bool AirfoilPanel::writeAirfoil(const LoadedAirfoil &airfoil,
                                const QString &path,
                                const std::vector<lep::BSpline2D> &splines,
                                const std::vector<std::vector<double>> &stations,
                                QString *error)
{
    const lep::AirfoilFile sampled =
        sampledFromSplines(airfoil.file, splines, stations);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        *error = file.errorString();
        return false;
    }
    const std::string text = lep::formatAirfoilFile(sampled);
    if (file.write(text.data(), static_cast<qint64>(text.size()))
            != static_cast<qint64>(text.size())
        || !file.commit()) {
        *error = file.errorString();
        return false;
    }
    return true;
}

void AirfoilPanel::persistSplines()
{
    if (!storeSplines_)
        return;
    QJsonObject root = loadSplines_ ? loadSplines_() : QJsonObject();
    QJsonObject files;
    for (const LoadedAirfoil &airfoil : airfoils_) {
        if (airfoil.splines.empty())
            continue;
        QJsonArray segments;
        for (size_t s = 0; s < airfoil.splines.size(); ++s)
            segments.append(segmentToJson(
                airfoil.splines[s],
                s < airfoil.stations.size() ? airfoil.stations[s]
                                            : std::vector<double>()));
        QJsonObject entry;
        entry.insert(QStringLiteral("segments"), segments);
        files.insert(airfoil.referencedName, entry);
    }
    if (files.isEmpty()) {
        root.remove(QStringLiteral("airfoils"));
    } else {
        QJsonObject airfoilsRoot;
        airfoilsRoot.insert(QStringLiteral("version"), 1);
        airfoilsRoot.insert(QStringLiteral("tolerancePercent"),
                            tolerancePercent_);
        airfoilsRoot.insert(QStringLiteral("files"), files);
        root.insert(QStringLiteral("airfoils"), airfoilsRoot);
    }
    storeSplines_(root);
}

void AirfoilPanel::commitHandleEdit()
{
    LoadedAirfoil *airfoil = currentAirfoil();
    if (airfoil == nullptr || airfoil->splines.empty())
        return;
    UndoState before = captureState();
    // Handles in the view are the authoritative new control points.
    for (size_t s = 0; s < airfoil->splines.size(); ++s) {
        const int handleCount =
            static_cast<int>(airfoil->splines[s].controlX.size());
        for (int c = 0; c < handleCount; ++c) {
            const QPointF handle =
                view_->handlePosition(static_cast<int>(s), c);
            airfoil->splines[s].controlX[static_cast<size_t>(c)] =
                handle.x();
            airfoil->splines[s].controlY[static_cast<size_t>(c)] =
                handle.y();
        }
    }
    QString error;
    if (!writeAirfoil(*airfoil, airfoil->absolutePath, airfoil->splines,
                      airfoil->stations, &error)) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(
            QStringLiteral("The airfoil file could not be written: %1")
                .arg(error));
        return;
    }
    airfoil->file = sampledFromSplines(airfoil->file, airfoil->splines,
                                       airfoil->stations);
    persistSplines();
    rebuildView();
    pushUndo(std::move(before));
}

void AirfoilPanel::applySplineToFile()
{
    LoadedAirfoil *airfoil = currentAirfoil();
    if (airfoil == nullptr || airfoil->splines.empty())
        return;
    UndoState before = captureState();
    QString error;
    if (!writeAirfoil(*airfoil, airfoil->absolutePath, airfoil->splines,
                      airfoil->stations, &error)) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(
            QStringLiteral("The airfoil file could not be written: %1")
                .arg(error));
        return;
    }
    syncFromText();
    pushUndo(std::move(before));
}

void AirfoilPanel::removeSplines()
{
    LoadedAirfoil *airfoil = currentAirfoil();
    if (airfoil == nullptr || airfoil->splines.empty())
        return;
    UndoState before = captureState();
    airfoil->splines.clear();
    airfoil->stations.clear();
    persistSplines();
    status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
    syncFromText();
    pushUndo(std::move(before));
}

void AirfoilPanel::showConvertDialog()
{
    LoadedAirfoil *airfoil = currentAirfoil();
    if (airfoil == nullptr || airfoil->file.totalPoints() < 4)
        return;

    QDialog dialog(this);
    dialog.setWindowTitle(
        QStringLiteral("Convert %1 to B-splines").arg(
            airfoil->referencedName));
    auto *layout = new QVBoxLayout(&dialog);
    auto *intro = new QLabel(
        QStringLiteral(
            "The three contour segments are fitted with parametric "
            "B-splines within the tolerance. The original file is not "
            "modified: a regenerated \"-spline\" copy is written beside "
            "the design and the Section 2 rows point at it (one undo step "
            "in the text restores the old reference)."),
        &dialog);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto *sliderRow = new QHBoxLayout;
    sliderRow->addWidget(new QLabel(QStringLiteral("Tolerance"), &dialog));
    auto *slider = new QSlider(Qt::Horizontal, &dialog);
    slider->setRange(0, 300);
    slider->setValue(percentToSlider(tolerancePercent_));
    sliderRow->addWidget(slider, 1);
    auto *sliderLabel = new QLabel(&dialog);
    sliderLabel->setMinimumWidth(150);
    sliderRow->addWidget(sliderLabel);
    layout->addLayout(sliderRow);

    auto *summary = new QLabel(&dialog);
    summary->setWordWrap(true);
    summary->setObjectName(QStringLiteral("hint"));
    layout->addWidget(summary);

    auto *buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("Convert"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    const auto fileSegments = airfoil->file.segments();
    double chord = 1.0;
    {
        const auto [minIt, maxIt] = std::minmax_element(
            airfoil->file.xs.begin(), airfoil->file.xs.end());
        chord = std::max(*maxIt - *minIt, 1e-6);
    }

    const auto updatePreview = [&] {
        const double percent = sliderToPercent(slider->value());
        sliderLabel->setText(QStringLiteral("%1 % of chord")
                                 .arg(QString::number(percent, 'f', 2)));
        previewSplines_.clear();
        double worst = 0.0;
        QStringList counts;
        for (const auto &segment : fileSegments) {
            std::vector<double> xs, ys;
            for (int i = segment.first; i <= segment.last; ++i) {
                xs.push_back(airfoil->file.xs[static_cast<size_t>(i)]);
                ys.push_back(airfoil->file.ys[static_cast<size_t>(i)]);
            }
            double maxError = 0.0;
            const lep::BSpline2D fitted = lep::fitBSpline2D(
                xs, ys, percent / 100.0 * chord, &maxError);
            if (fitted.isValid()) {
                previewSplines_.push_back(fitted);
                counts << QString::number(fitted.controlX.size());
                worst = std::max(worst, maxError);
            }
        }
        const bool complete =
            previewSplines_.size() == fileSegments.size();
        buttons->button(QDialogButtonBox::Ok)->setEnabled(complete);
        summary->setText(
            complete
                ? QStringLiteral("Control points per segment: %1 · worst "
                                 "deviation %2 (%3 % of chord)")
                      .arg(counts.join(QStringLiteral(" / ")))
                      .arg(QString::number(worst, 'f', 5))
                      .arg(QString::number(worst / chord * 100.0, 'f', 3))
                : QStringLiteral("The contour could not be fitted."));
        previewActive_ = true;
        rebuildView();
    };
    connect(slider, &QSlider::valueChanged, &dialog,
            [&updatePreview](int) { updatePreview(); });
    updatePreview();

    const int result = dialog.exec();
    previewActive_ = false;
    if (result != QDialog::Accepted
        || previewSplines_.size() != fileSegments.size()) {
        previewSplines_.clear();
        rebuildView();
        return;
    }

    tolerancePercent_ = sliderToPercent(slider->value());
    const QString oldName = airfoil->referencedName;
    QString newName = oldName;
    if (!oldName.endsWith(QStringLiteral("-spline.txt"),
                          Qt::CaseInsensitive)) {
        newName = oldName;
        newName.replace(QRegularExpression(QStringLiteral("\\.txt$"),
                                           QRegularExpression::
                                               CaseInsensitiveOption),
                        QString());
        newName += QStringLiteral("-spline.txt");
    }

    UndoState undoBefore = captureState();
    // The fit-time chord parameters become the permanent sampling
    // stations for this airfoil: the file is these splines evaluated at
    // exactly these stations from now on.
    std::vector<std::vector<double>> stations;
    for (const auto &segment : fileSegments) {
        std::vector<double> xs, ys;
        for (int i = segment.first; i <= segment.last; ++i) {
            xs.push_back(airfoil->file.xs[static_cast<size_t>(i)]);
            ys.push_back(airfoil->file.ys[static_cast<size_t>(i)]);
        }
        stations.push_back(lep::chordParameters(xs, ys));
    }

    QString error;
    const QString newPath =
        QDir(designDirectory_ ? designDirectory_() : QString())
            .filePath(newName);
    if (!writeAirfoil(*airfoil, newPath, previewSplines_, stations,
                      &error)) {
        previewSplines_.clear();
        rebuildView();
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(
            QStringLiteral("The regenerated airfoil could not be written: "
                           "%1")
                .arg(error));
        return;
    }

    // Adopt the splines under the (possibly new) reference name, persist,
    // then repoint the section rows — the text change triggers the final
    // resync that reloads everything consistently.
    airfoil->referencedName = newName;
    airfoil->absolutePath = newPath;
    airfoil->splines = previewSplines_;
    airfoil->stations = stations;
    previewSplines_.clear();
    persistSplines();

    if (newName != oldName) {
        const lep::SectionGrid grid = lep::parseSectionGrid(
            editor_->toPlainText().toStdString());
        QTextDocument *document = editor_->document();
        applyingEdit_ = true;
        QTextCursor cursor(document);
        cursor.beginEditBlock();
        for (const lep::GridRow &row : grid.rows) {
            if (row.tokens.size() < 2
                || QString::fromStdString(row.tokens[1].text) != oldName)
                continue;
            const QTextBlock block =
                document->findBlockByNumber(row.lineIndex);
            if (!block.isValid())
                continue;
            cursor.setPosition(block.position());
            cursor.movePosition(QTextCursor::EndOfBlock,
                                QTextCursor::KeepAnchor);
            cursor.insertText(QString::fromStdString(
                lep::lineWithReplacedToken(block.text().toStdString(),
                                           row.tokens[1],
                                           newName.toStdString())));
        }
        cursor.endEditBlock();
        applyingEdit_ = false;
    }
    syncFromText();
    pushUndo(std::move(undoBefore));
}
