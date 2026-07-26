#pragma once

#include "panel_undo.h"

#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <functional>

class QComboBox;
class QLabel;
class QPlainTextEdit;

// One hole of a Section 4 group, in the airfoil's own coordinates
// (percent of chord; y positive towards the upper surface).
struct HoleShape
{
    int lineIndex = -1; // line within the section text
    double type = 1.0;  // 1 ellipse · 3 triangle · 4 rectangle · 11 marker
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;     // semi-axis / first side
    double b = 0.0;     // semi-axis / second side
    double angle = 0.0; // rotation (radians for ellipses, degrees for 3/4)
    double radius = 0.0; // corner radius (types 3/4)
};

// Aspect-true display of one hole group inside its airfoil outline. The
// centre square moves a hole, the diamond handle resizes it (types 1/4).
class HolesView final : public QWidget
{
    Q_OBJECT
public:
    explicit HolesView(QWidget *parent = nullptr);
    void setScene(const QVector<QPointF> &outline,
                  const QVector<HoleShape> &holes);
    void setMessage(const QString &message);
    const QVector<HoleShape> &holes() const { return holes_; }
    int selectedHole() const { return selected_; }
    QSize minimumSizeHint() const override { return {380, 180}; }

signals:
    void selectionChanged(int hole);
    void holeEdited(int hole); // live during a drag
    void editCommitted();
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
    enum class Grip
    {
        None,
        Centre,
        Size,
    };
    Mapping mapping() const;
    QPointF toPixel(const QPointF &point, const Mapping &m) const;
    QPointF fromPixel(const QPointF &pixel, const Mapping &m) const;
    QPointF sizeHandle(const HoleShape &hole) const;
    Grip gripAt(const QPointF &pixel, int *hole) const;

    QVector<QPointF> outline_;
    QVector<HoleShape> holes_;
    QString message_;
    int selected_ = -1;
    int hoverHole_ = -1;
    Grip hoverGrip_ = Grip::None;
    Grip dragGrip_ = Grip::None;
    bool dragging_ = false;
};

// Section 4 companion panel: shows each hole group drawn inside the
// airfoil of its first rib (resolved through the Section 2 rows), with
// draggable centre/size handles writing straight back into the hole rows.
// Ctrl+Z / Ctrl+Y in the view undo graphical edits via the panel stack.
class HolesPanel final : public QWidget
{
    Q_OBJECT
public:
    HolesPanel(QPlainTextEdit *editor,
               std::function<QString()> designDirectory,
               std::function<QString()> section2Text,
               QWidget *parent = nullptr);

private:
    struct Group
    {
        int firstRib = 0;
        int lastRib = 0;
        QVector<HoleShape> holes;
    };

    void syncFromText();
    void rebuildView();
    void commitHoleEdit();
    void restoreText(const QString &text);
    QString airfoilNameForRib(int rib) const;

    QPlainTextEdit *editor_;
    std::function<QString()> designDirectory_;
    std::function<QString()> section2Text_;
    QComboBox *groupCombo_;
    HolesView *view_;
    QLabel *description_;
    QLabel *status_;
    QVector<Group> groups_;
    QStringList problems_;
    bool applyingEdit_ = false;
    PanelUndoStack<QString> undo_;
};
