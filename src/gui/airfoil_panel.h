#pragma once

#include "airfoil_file.h"
#include "panel_undo.h"
#include "spline_fit.h"

#include <QHash>
#include <QJsonObject>
#include <QPointF>
#include <QVector>
#include <QWidget>

#include <functional>
#include <vector>

class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;

// Aspect-true display of one airfoil contour with optional B-spline
// overlays per segment (extrados / intake / intrados). Control handles
// drag freely in 2D; boundary handles shared by two segments are welded
// by the host.
class AirfoilView final : public QWidget
{
    Q_OBJECT
public:
    struct SegmentDisplay
    {
        QColor color;
        QString label;
        QVector<QPointF> points;  // raw file points
        QVector<QPointF> smooth;  // sampled spline; empty = polyline mode
        QVector<QPointF> handles; // 2D control points; empty = not editable
    };

    explicit AirfoilView(QWidget *parent = nullptr);
    void setSegments(const QVector<SegmentDisplay> &segments);
    void setSegmentSmooth(int segment, const QVector<QPointF> &smooth);
    void setHandlePosition(int segment, int index, const QPointF &position);
    QPointF handlePosition(int segment, int index) const;
    void setMessage(const QString &message);
    QSize minimumSizeHint() const override { return {380, 180}; }

signals:
    void handleMoved(int segment, int index, const QPointF &position);
    void editCommitted();
    // Ctrl+Z / Ctrl+Y pressed with the view focused; the panel owns the
    // undo stack for graphical edits.
    void undoRequested();
    void redoRequested();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void leaveEvent(QEvent *event) override;

private:
    struct Mapping
    {
        double scale = 1.0;
        double offsetX = 0.0;
        double offsetY = 0.0;
    };
    Mapping mapping() const;
    QPointF toPixel(const QPointF &point, const Mapping &m) const;
    QPointF fromPixel(const QPointF &pixel, const Mapping &m) const;
    bool findHandle(const QPointF &pixel, int *segment, int *index) const;
    void updateHover(const QPointF &pixel);

    QVector<SegmentDisplay> segments_;
    QString message_;
    int hoverSegment_ = -1;
    int hoverIndex_ = -1;
    int activeSegment_ = -1;
    int activeIndex_ = -1;
    bool dragging_ = false;
};

// Section 2 companion panel: shows the airfoil files referenced by the
// section rows and converts them to B-spline truth, mirroring Section 1.
// Conversion writes a regenerated "<name>-spline.txt" beside the design
// and repoints the section rows at it (one undoable text edit); the
// original file is never modified. Control-point edits rewrite only that
// generated file. Spline definitions persist in the design's Studio
// trailer under "airfoils", keyed by the generated file name.
class AirfoilPanel final : public QWidget
{
    Q_OBJECT
public:
    AirfoilPanel(QPlainTextEdit *editor,
                 std::function<QString()> designDirectory,
                 std::function<QJsonObject()> loadSplines,
                 std::function<void(const QJsonObject &)> storeSplines,
                 QWidget *parent = nullptr);

private:
    struct LoadedAirfoil
    {
        QString referencedName; // as written in the section rows
        QString absolutePath;
        lep::AirfoilFile file;
        QString problem;                     // load/parse issue, if any
        std::vector<lep::BSpline2D> splines; // one per segment when backed
        // Fixed sampling stations per segment, captured at fit time; the
        // file is always the splines evaluated at exactly these stations
        // (re-deriving them from the moving samples would drift).
        std::vector<std::vector<double>> stations;
        bool stale = false;
    };

    // Everything a graphical edit can touch: the section text (file
    // references), the trailer JSON, and the referenced files on disk.
    struct UndoState
    {
        QString text;
        QJsonObject splines;
        QHash<QString, QByteArray> files; // absolute path -> content
    };
    UndoState captureState() const;
    void restoreState(const UndoState &state);
    void pushUndo(UndoState before);

    void syncFromText();
    void rebuildView();
    void showConvertDialog();
    void removeSplines();
    void applySplineToFile();
    void commitHandleEdit();
    LoadedAirfoil *currentAirfoil();
    void persistSplines();
    void loadSplinesFromDocument();
    bool writeAirfoil(const LoadedAirfoil &airfoil, const QString &path,
                      const std::vector<lep::BSpline2D> &splines,
                      const std::vector<std::vector<double>> &stations,
                      QString *error);
    lep::AirfoilFile sampledFromSplines(
        const lep::AirfoilFile &reference,
        const std::vector<lep::BSpline2D> &splines,
        const std::vector<std::vector<double>> &stations) const;

    QPlainTextEdit *editor_;
    std::function<QString()> designDirectory_;
    std::function<QJsonObject()> loadSplines_;
    std::function<void(const QJsonObject &)> storeSplines_;
    QComboBox *fileCombo_;
    AirfoilView *view_;
    QLabel *description_;
    QLabel *status_;
    QPushButton *convertButton_;
    QPushButton *removeButton_;
    QPushButton *refitButton_;
    QPushButton *applyButton_;
    std::vector<LoadedAirfoil> airfoils_;
    double tolerancePercent_ = 0.2;
    bool applyingEdit_ = false;
    bool previewActive_ = false;
    std::vector<lep::BSpline2D> previewSplines_;
    PanelUndoStack<UndoState> undo_;
};
