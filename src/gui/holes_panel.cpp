#include "holes_panel.h"

#include "airfoil_file.h"
#include "section_grid.h"

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

const QColor kBackground(0x0e, 0x17, 0x26);
const QColor kBorder(0x2a, 0x3a, 0x50);
const QColor kOutlineInk(0x64, 0x78, 0x92);
const QColor kAxisInk(0x93, 0xa4, 0xba);
const QColor kBrightInk(0xe6, 0xed, 0xf7);
const QColor kBubbleBackground(0x1b, 0x29, 0x3c);
const QColor kHoleColor(0x39, 0x87, 0xe5);
const QColor kHoleSelected(0x67, 0xd3, 0xff);
const QString kOkColor = QStringLiteral("#93a4ba");
const QString kWarningColor = QStringLiteral("#fab219");
const QString kErrorColor = QStringLiteral("#e66767");
constexpr double kGripRadius = 11.0;
constexpr double kPi = 3.14159265358979323846;

bool parseNumber(const std::string &token, double *value)
{
    char *end = nullptr;
    *value = std::strtod(token.c_str(), &end);
    return end && *end == '\0' && end != token.c_str();
}

// The hole's outline in airfoil coordinates (percent of chord), matching
// the engine's drawing: ellipses rotate by the raw value (radians),
// triangles and rectangles by degrees.
QPolygonF holeOutline(const HoleShape &hole)
{
    QPolygonF polygon;
    if (hole.type == 1.0) {
        const double rotation = hole.angle;
        for (int i = 0; i <= 40; ++i) {
            const double t = 2.0 * kPi * i / 40.0;
            polygon.append(QPointF(
                hole.x + hole.a * std::cos(t) * std::cos(rotation)
                    - hole.b * std::sin(t) * std::sin(rotation),
                hole.y + hole.a * std::cos(t) * std::sin(rotation)
                    + hole.b * std::sin(t) * std::cos(rotation)));
        }
    } else if (hole.type == 3.0) {
        // Triangle with legs a (signed, along the rotated base) and b
        // (vertical), like the engine's construction without the rounded
        // corners.
        const double rotation = hole.angle * kPi / 180.0;
        const double side = hole.a >= 0.0 ? 1.0 : -1.0;
        const double a = std::abs(hole.a);
        polygon.append(QPointF(hole.x, hole.y));
        polygon.append(QPointF(hole.x + side * a * std::cos(rotation),
                               hole.y + a * std::sin(rotation)));
        polygon.append(QPointF(hole.x, hole.y + hole.b));
        polygon.append(polygon.first());
    } else if (hole.type == 4.0) {
        const double rotation = hole.angle * kPi / 180.0;
        const double c = std::cos(rotation);
        const double s = std::sin(rotation);
        const QPointF corners[4] = {
            {-hole.a, -hole.b}, {hole.a, -hole.b},
            {hole.a, hole.b},   {-hole.a, hole.b}};
        for (const QPointF &corner : corners)
            polygon.append(QPointF(
                hole.x + corner.x() * c - corner.y() * s,
                hole.y + corner.x() * s + corner.y() * c));
        polygon.append(polygon.first());
    }
    return polygon;
}

QString holeTypeName(double type)
{
    if (type == 1.0)
        return QStringLiteral("ellipse");
    if (type == 3.0)
        return QStringLiteral("triangle");
    if (type == 4.0)
        return QStringLiteral("rectangle");
    if (type == 11.0)
        return QStringLiteral("unloaded-rib marker");
    return QStringLiteral("type %1").arg(type);
}

} // namespace

// ---------------------------------------------------------------- view --

HolesView::HolesView(QWidget *parent) : QWidget(parent)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::ClickFocus);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void HolesView::setScene(const QVector<QPointF> &outline,
                         const QVector<HoleShape> &holes)
{
    outline_ = outline;
    holes_ = holes;
    dragging_ = false;
    dragGrip_ = Grip::None;
    hoverHole_ = -1;
    hoverGrip_ = Grip::None;
    if (selected_ >= holes_.size())
        selected_ = -1;
    update();
}

void HolesView::setMessage(const QString &message)
{
    message_ = message;
    update();
}

HolesView::Mapping HolesView::mapping() const
{
    double minX = 0.0, maxX = 100.0, minY = -10.0, maxY = 10.0;
    bool first = true;
    const auto include = [&](const QPointF &point) {
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
    };
    for (const QPointF &point : outline_)
        include(point);
    for (const HoleShape &hole : holes_)
        include(QPointF(hole.x, hole.y));
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

QPointF HolesView::toPixel(const QPointF &point, const Mapping &m) const
{
    return QPointF(point.x() * m.scale + m.offsetX,
                   m.offsetY - point.y() * m.scale);
}

QPointF HolesView::fromPixel(const QPointF &pixel, const Mapping &m) const
{
    return QPointF((pixel.x() - m.offsetX) / m.scale,
                   (m.offsetY - pixel.y()) / m.scale);
}

QPointF HolesView::sizeHandle(const HoleShape &hole) const
{
    const double rotation =
        hole.type == 1.0 ? hole.angle : hole.angle * kPi / 180.0;
    const double c = std::cos(rotation);
    const double s = std::sin(rotation);
    return QPointF(hole.x + hole.a * c - hole.b * s,
                   hole.y + hole.a * s + hole.b * c);
}

HolesView::Grip HolesView::gripAt(const QPointF &pixel, int *hole) const
{
    const Mapping m = mapping();
    *hole = -1;
    Grip grip = Grip::None;
    double best = kGripRadius;
    for (int i = 0; i < holes_.size(); ++i) {
        if (holes_.at(i).type == 11.0)
            continue; // marker rows carry no geometry
        const QPointF centre = toPixel(QPointF(holes_.at(i).x,
                                               holes_.at(i).y),
                                       m);
        const double centreDistance =
            std::hypot(centre.x() - pixel.x(), centre.y() - pixel.y());
        if (centreDistance <= best) {
            best = centreDistance;
            *hole = i;
            grip = Grip::Centre;
        }
        if (holes_.at(i).type == 1.0 || holes_.at(i).type == 4.0) {
            const QPointF handle = toPixel(sizeHandle(holes_.at(i)), m);
            const double handleDistance =
                std::hypot(handle.x() - pixel.x(), handle.y() - pixel.y());
            if (handleDistance <= best) {
                best = handleDistance;
                *hole = i;
                grip = Grip::Size;
            }
        }
    }
    return grip;
}

void HolesView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(kBorder, 1.0));
    painter.setBrush(kBackground);
    painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                            6.0, 6.0);
    const QFontMetrics metrics(font());
    if (!message_.isEmpty() || outline_.isEmpty()) {
        painter.setPen(kAxisInk);
        painter.drawText(rect().adjusted(16, 16, -16, -16),
                         Qt::AlignCenter | Qt::TextWordWrap,
                         message_.isEmpty() ? QStringLiteral("No holes")
                                            : message_);
        return;
    }

    const Mapping m = mapping();
    QPolygonF outlinePixels;
    outlinePixels.reserve(outline_.size());
    for (const QPointF &point : outline_)
        outlinePixels.append(toPixel(point, m));
    painter.setPen(QPen(kOutlineInk, 1.4));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolyline(outlinePixels);

    for (int i = 0; i < holes_.size(); ++i) {
        const HoleShape &hole = holes_.at(i);
        const bool isSelected = i == selected_;
        const QColor color = isSelected ? kHoleSelected : kHoleColor;
        if (hole.type == 11.0)
            continue;
        const QPolygonF shape = holeOutline(hole);
        if (!shape.isEmpty()) {
            QPolygonF pixels;
            pixels.reserve(shape.size());
            for (const QPointF &point : shape)
                pixels.append(toPixel(point, m));
            painter.setPen(QPen(color, isSelected ? 2.2 : 1.6));
            painter.drawPolyline(pixels);
        }
        // Centre grip.
        const QPointF centre = toPixel(QPointF(hole.x, hole.y), m);
        const bool centreHot =
            (i == hoverHole_ && hoverGrip_ == Grip::Centre)
            || (dragging_ && i == selected_ && dragGrip_ == Grip::Centre);
        const double radius = centreHot ? 5.5 : 3.8;
        painter.setPen(QPen(kBackground, 1.6));
        painter.setBrush(color);
        painter.drawRect(QRectF(centre.x() - radius, centre.y() - radius,
                                radius * 2.0, radius * 2.0));
        // Size grip (diamond) for ellipses and rectangles.
        if (hole.type == 1.0 || hole.type == 4.0) {
            const QPointF handle = toPixel(sizeHandle(hole), m);
            const bool handleHot =
                (i == hoverHole_ && hoverGrip_ == Grip::Size)
                || (dragging_ && i == selected_ && dragGrip_ == Grip::Size);
            const double r = handleHot ? 6.0 : 4.2;
            QPolygonF diamond;
            diamond << QPointF(handle.x(), handle.y() - r)
                    << QPointF(handle.x() + r, handle.y())
                    << QPointF(handle.x(), handle.y() + r)
                    << QPointF(handle.x() - r, handle.y());
            painter.drawPolygon(diamond);
        }
    }

    // Coordinate bubble for the hovered/dragged grip.
    const int bubbleHole =
        dragging_ ? selected_ : (hoverGrip_ != Grip::None ? hoverHole_ : -1);
    if (bubbleHole >= 0 && bubbleHole < holes_.size()) {
        const HoleShape &hole = holes_.at(bubbleHole);
        const Grip grip = dragging_ ? dragGrip_ : hoverGrip_;
        const QString text =
            grip == Grip::Size
                ? QStringLiteral("a %1 · b %2 %chord")
                      .arg(QString::number(hole.a, 'f', 2))
                      .arg(QString::number(hole.b, 'f', 2))
                : QStringLiteral("x %1 · y %2 %chord")
                      .arg(QString::number(hole.x, 'f', 2))
                      .arg(QString::number(hole.y, 'f', 2));
        const QPointF anchor =
            toPixel(grip == Grip::Size ? sizeHandle(hole)
                                       : QPointF(hole.x, hole.y),
                    m);
        const QSizeF size(metrics.horizontalAdvance(text) + 14.0,
                          metrics.height() + 8.0);
        QPointF corner(anchor.x() + 12.0, anchor.y() - size.height() - 8.0);
        if (corner.x() + size.width() > width() - 6.0)
            corner.setX(anchor.x() - size.width() - 12.0);
        if (corner.y() < 6.0)
            corner.setY(anchor.y() + 12.0);
        const QRectF bubble(corner, size);
        painter.setPen(QPen(kHoleSelected, 1.0));
        painter.setBrush(kBubbleBackground);
        painter.drawRoundedRect(bubble, 4.0, 4.0);
        painter.setPen(kBrightInk);
        painter.drawText(bubble, Qt::AlignCenter, text);
    }
}

void HolesView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }
    int hole = -1;
    const Grip grip = gripAt(event->position(), &hole);
    if (grip != Grip::None) {
        if (selected_ != hole) {
            selected_ = hole;
            emit selectionChanged(hole);
        }
        dragging_ = true;
        dragGrip_ = grip;
        update();
    }
}

void HolesView::mouseMoveEvent(QMouseEvent *event)
{
    if (!dragging_) {
        int hole = -1;
        const Grip grip = gripAt(event->position(), &hole);
        if (hole != hoverHole_ || grip != hoverGrip_) {
            hoverHole_ = hole;
            hoverGrip_ = grip;
            update();
        }
        if (grip != Grip::None)
            setCursor(Qt::SizeAllCursor);
        else
            unsetCursor();
        return;
    }
    HoleShape &hole = holes_[selected_];
    const QPointF position = fromPixel(event->position(), mapping());
    if (dragGrip_ == Grip::Centre) {
        hole.x = position.x();
        hole.y = position.y();
    } else {
        // Back-rotate the offset into the hole's own frame; sizes stay
        // positive.
        const double rotation = hole.type == 1.0
                                    ? hole.angle
                                    : hole.angle * kPi / 180.0;
        const double dx = position.x() - hole.x;
        const double dy = position.y() - hole.y;
        hole.a = std::max(
            0.05, dx * std::cos(rotation) + dy * std::sin(rotation));
        hole.b = std::max(
            0.05, -dx * std::sin(rotation) + dy * std::cos(rotation));
    }
    update();
    emit holeEdited(selected_);
}

void HolesView::mouseReleaseEvent(QMouseEvent *event)
{
    if (!dragging_ || event->button() != Qt::LeftButton) {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    dragging_ = false;
    dragGrip_ = Grip::None;
    update();
    emit editCommitted(); // may re-enter setScene(); keep this last
}

void HolesView::keyPressEvent(QKeyEvent *event)
{
    if (event->matches(QKeySequence::Undo)) {
        emit undoRequested();
        return;
    }
    if (event->matches(QKeySequence::Redo)) {
        emit redoRequested();
        return;
    }
    if (selected_ < 0 || selected_ >= holes_.size()) {
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
    const double step =
        event->modifiers().testFlag(Qt::ShiftModifier) ? 0.5 : 0.05;
    holes_[selected_].x += dx * step;
    holes_[selected_].y += dy * step;
    update();
    emit holeEdited(selected_);
    emit editCommitted();
}

void HolesView::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);
    hoverHole_ = -1;
    hoverGrip_ = Grip::None;
    update();
}

// --------------------------------------------------------------- panel --

HolesPanel::HolesPanel(QPlainTextEdit *editor,
                       std::function<QString()> designDirectory,
                       std::function<QString()> section2Text,
                       QWidget *parent)
    : QWidget(parent), editor_(editor),
      designDirectory_(std::move(designDirectory)),
      section2Text_(std::move(section2Text))
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto *header = new QHBoxLayout;
    header->setSpacing(6);
    auto *groupLabel = new QLabel(QStringLiteral("Hole group"), this);
    groupLabel->setObjectName(QStringLiteral("hint"));
    header->addWidget(groupLabel);
    groupCombo_ = new QComboBox(this);
    groupCombo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    header->addWidget(groupCombo_, 0);
    header->addStretch();
    layout->addLayout(header);

    view_ = new HolesView(this);
    layout->addWidget(view_, 1);

    description_ = new QLabel(this);
    description_->setObjectName(QStringLiteral("hint"));
    description_->setWordWrap(true);
    layout->addWidget(description_);

    status_ = new QLabel(this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    connect(groupCombo_, &QComboBox::currentIndexChanged, this,
            [this](int) { rebuildView(); });
    connect(editor_, &QPlainTextEdit::textChanged, this, [this] {
        if (!applyingEdit_)
            syncFromText();
    });
    connect(view_, &HolesView::selectionChanged, this,
            [this](int) { rebuildView(); });
    connect(view_, &HolesView::editCommitted, this,
            [this] { commitHoleEdit(); });
    connect(view_, &HolesView::undoRequested, this, [this] {
        if (const QString *text = undo_.undo())
            restoreText(*text);
    });
    connect(view_, &HolesView::redoRequested, this, [this] {
        if (const QString *text = undo_.redo())
            restoreText(*text);
    });

    syncFromText();
}

QString HolesPanel::airfoilNameForRib(int rib) const
{
    if (!section2Text_)
        return QString();
    const lep::SectionGrid grid =
        lep::parseSectionGrid(section2Text_().toStdString());
    for (const lep::GridRow &row : grid.rows) {
        if (row.tokens.size() < 2)
            continue;
        double value = 0.0;
        if (parseNumber(row.tokens[0].text, &value)
            && static_cast<int>(std::lround(value)) == rib)
            return QString::fromStdString(row.tokens[1].text);
    }
    return QString();
}

void HolesPanel::syncFromText()
{
    groups_.clear();
    problems_.clear();

    const lep::SectionGrid grid =
        lep::parseSectionGrid(editor_->toPlainText().toStdString());
    const auto intAt = [&grid](int rowIndex, int *value) {
        if (rowIndex >= static_cast<int>(grid.rows.size()))
            return false;
        const lep::GridRow &row = grid.rows[static_cast<size_t>(rowIndex)];
        double parsed = 0.0;
        if (row.tokens.empty()
            || !parseNumber(row.tokens[0].text, &parsed))
            return false;
        *value = static_cast<int>(std::lround(parsed));
        return true;
    };

    int cursor = 0;
    int groupCount = 0;
    if (!intAt(cursor++, &groupCount)) {
        problems_ << QStringLiteral(
            "The first record must be the number of hole groups.");
    }
    for (int g = 0; g < groupCount && problems_.isEmpty(); ++g) {
        Group group;
        int holeCount = 0;
        if (!intAt(cursor++, &group.firstRib)
            || !intAt(cursor++, &group.lastRib)
            || !intAt(cursor++, &holeCount)) {
            problems_ << QStringLiteral(
                           "Group %1: expected first rib, last rib and "
                           "hole count, each on its own record.")
                             .arg(g + 1);
            break;
        }
        for (int h = 0; h < holeCount; ++h, ++cursor) {
            if (cursor >= static_cast<int>(grid.rows.size())) {
                problems_ << QStringLiteral(
                               "Group %1 declares %2 holes but the "
                               "section ends after %3.")
                                 .arg(g + 1)
                                 .arg(holeCount)
                                 .arg(h);
                break;
            }
            const lep::GridRow &row =
                grid.rows[static_cast<size_t>(cursor)];
            if (row.tokens.size() < 8) {
                problems_ << QStringLiteral(
                               "Group %1, hole %2: a hole row needs at "
                               "least 8 values (type, x, y, a, b, angle, "
                               "radius, reserved).")
                                 .arg(g + 1)
                                 .arg(h + 1);
                break;
            }
            HoleShape hole;
            hole.lineIndex = row.lineIndex;
            double values[7] = {0, 0, 0, 0, 0, 0, 0};
            bool numeric = parseNumber(row.tokens[0].text, &hole.type);
            for (int t = 1; t <= 7 && numeric; ++t)
                numeric = parseNumber(row.tokens[static_cast<size_t>(t)]
                                          .text,
                                      &values[t - 1]);
            if (!numeric) {
                problems_ << QStringLiteral(
                               "Group %1, hole %2: a value is not a "
                               "number.")
                                 .arg(g + 1)
                                 .arg(h + 1);
                break;
            }
            hole.x = values[0];
            hole.y = values[1];
            hole.a = values[2];
            hole.b = values[3];
            hole.angle = values[4];
            hole.radius = values[5];
            group.holes.append(hole);
        }
        if (!problems_.isEmpty())
            break;
        groups_.append(group);
    }

    const QString previous = groupCombo_->currentText();
    groupCombo_->blockSignals(true);
    groupCombo_->clear();
    for (const Group &group : groups_) {
        const QString airfoil = airfoilNameForRib(group.firstRib);
        groupCombo_->addItem(
            QStringLiteral("Ribs %1–%2 · %3 hole%4%5")
                .arg(group.firstRib)
                .arg(group.lastRib)
                .arg(group.holes.size())
                .arg(group.holes.size() == 1 ? QString()
                                             : QStringLiteral("s"))
                .arg(airfoil.isEmpty()
                         ? QString()
                         : QStringLiteral(" · %1").arg(airfoil)));
    }
    const int restored = groupCombo_->findText(previous);
    groupCombo_->setCurrentIndex(std::max(0, restored));
    groupCombo_->blockSignals(false);
    groupCombo_->setEnabled(groups_.size() > 1);

    rebuildView();
}

void HolesPanel::rebuildView()
{
    const int index = groupCombo_->currentIndex();
    if (!problems_.isEmpty() || index < 0 || index >= groups_.size()) {
        view_->setScene({}, {});
        view_->setMessage(problems_.isEmpty()
                              ? QStringLiteral("No hole groups defined — "
                                               "the first record is the "
                                               "group count.")
                              : problems_.join(QLatin1Char('\n')));
        status_->setStyleSheet(QStringLiteral("color: %1;")
                                   .arg(problems_.isEmpty() ? kOkColor
                                                            : kErrorColor));
        status_->setText(problems_.join(QLatin1Char(' ')));
        description_->clear();
        return;
    }

    const Group &group = groups_.at(index);

    // Outline from the airfoil of the group's first rib (percent of
    // chord, like the hole coordinates).
    QVector<QPointF> outline;
    QString outlineNote;
    const QString airfoilName = airfoilNameForRib(group.firstRib);
    if (!airfoilName.isEmpty() && designDirectory_) {
        QFile file(QDir(designDirectory_()).filePath(airfoilName));
        if (file.open(QIODevice::ReadOnly)) {
            lep::AirfoilFile airfoil;
            std::string error;
            if (lep::parseAirfoilFile(
                    QString::fromUtf8(file.readAll()).toStdString(),
                    &airfoil, &error)) {
                outline.reserve(airfoil.totalPoints());
                for (int i = 0; i < airfoil.totalPoints(); ++i)
                    outline.append(
                        QPointF(airfoil.xs[static_cast<size_t>(i)] * 100.0,
                                airfoil.ys[static_cast<size_t>(i)]
                                    * 100.0));
            }
        }
        if (outline.isEmpty())
            outlineNote = QStringLiteral(
                              " · the airfoil outline (%1) could not be "
                              "loaded")
                              .arg(airfoilName);
    }

    view_->setMessage(QString());
    view_->setScene(outline, group.holes);

    const int selected = view_->selectedHole();
    if (selected >= 0 && selected < group.holes.size()) {
        const HoleShape &hole = group.holes.at(selected);
        description_->setText(
            QStringLiteral("<b>Hole %1</b> · %2 — centre x %3, y %4 (%chord "
                           "above the chord line) · a %5 · b %6 · drag the "
                           "square to move, the diamond to resize")
                .arg(selected + 1)
                .arg(holeTypeName(hole.type))
                .arg(QString::number(hole.x, 'f', 2))
                .arg(QString::number(hole.y, 'f', 2))
                .arg(QString::number(hole.a, 'f', 2))
                .arg(QString::number(hole.b, 'f', 2)));
    } else {
        description_->setText(QStringLiteral(
            "Lightening holes for one rib range, drawn inside its airfoil. "
            "Values are %% of the rib chord: type (1 ellipse, 3 triangle, "
            "4 rectangle), centre x and y, the two half-sizes, rotation "
            "and corner radius."));
    }

    status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
    status_->setText(
        QStringLiteral("%1 hole%2 on ribs %3–%4%5 · drag to edit · arrows "
                       "nudge (Shift = ×10) · Ctrl+Z here undoes · counts "
                       "and new holes are edited in the text above")
            .arg(group.holes.size())
            .arg(group.holes.size() == 1 ? QString() : QStringLiteral("s"))
            .arg(group.firstRib)
            .arg(group.lastRib)
            .arg(outlineNote));
}

void HolesPanel::commitHoleEdit()
{
    const int index = groupCombo_->currentIndex();
    if (index < 0 || index >= groups_.size())
        return;
    Group &group = groups_[index];
    const QString before = editor_->toPlainText();

    QTextDocument *document = editor_->document();
    applyingEdit_ = true;
    QTextCursor cursor(document);
    cursor.beginEditBlock();
    for (int h = 0; h < group.holes.size()
                    && h < view_->holes().size();
         ++h) {
        const HoleShape &edited = view_->holes().at(h);
        HoleShape &stored = group.holes[h];
        if (edited.x == stored.x && edited.y == stored.y
            && edited.a == stored.a && edited.b == stored.b)
            continue;
        stored = edited;
        const QTextBlock block =
            document->findBlockByNumber(edited.lineIndex);
        if (!block.isValid())
            continue;
        const lep::SectionGrid lineGrid = lep::parseSectionGrid(
            "* x\n" + block.text().toStdString() + "\n");
        if (lineGrid.rows.empty()
            || lineGrid.rows.front().tokens.size() < 5)
            continue;
        std::string line = block.text().toStdString();
        // Replace tokens 2-5 (x, y, a, b) back to front so the recorded
        // offsets stay valid.
        const auto &tokens = lineGrid.rows.front().tokens;
        const double values[4] = {edited.x, edited.y, edited.a, edited.b};
        for (int t = 4; t >= 1; --t) {
            char buffer[32];
            std::snprintf(buffer, sizeof buffer, "%.2f",
                          values[t - 1]);
            line = lep::lineWithReplacedToken(
                line, tokens[static_cast<size_t>(t)], buffer);
        }
        cursor.setPosition(block.position());
        cursor.movePosition(QTextCursor::EndOfBlock,
                            QTextCursor::KeepAnchor);
        cursor.insertText(QString::fromStdString(line));
    }
    cursor.endEditBlock();
    applyingEdit_ = false;
    syncFromText();
    const QString after = editor_->toPlainText();
    if (after != before)
        undo_.push(before, after);
}

void HolesPanel::restoreText(const QString &text)
{
    if (editor_->toPlainText() == text)
        return;
    applyingEdit_ = true;
    QTextCursor cursor(editor_->document());
    cursor.select(QTextCursor::Document);
    cursor.insertText(text);
    applyingEdit_ = false;
    syncFromText();
}
