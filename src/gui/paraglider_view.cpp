#include "paraglider_view.h"

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_GradientFillMethod.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Graphic3d_Camera.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_RenderingParams.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#ifdef Q_OS_WIN
#include <WNT_Window.hxx>
#else
#include <Aspect_NeutralWindow.hxx>
#endif

#include <QFileInfo>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double centimetresPerMillimetre = 0.1;

QString occFailureMessage(const Standard_Failure &failure)
{
    return failure.GetMessageString() != nullptr
               ? QString::fromUtf8(failure.GetMessageString())
               : QStringLiteral("Unknown Open CASCADE error");
}

double meshDeflectionMillimetres(double diagonalMillimetres, double deflectionScale)
{
    const double base = std::clamp(diagonalMillimetres * 0.00035, 0.025, 1.0);
    return std::clamp(base * deflectionScale, 0.005, 50.0);
}

bool triangulateShape(const TopoDS_Shape &shape, double deflectionMillimetres)
{
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh mesher(
        shape,
        deflectionMillimetres,
        false,
        0.20,
        true);
    return mesher.IsDone();
}

int countTriangles(const TopoDS_Shape &shape)
{
    int triangles = 0;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE);
         explorer.More();
         explorer.Next()) {
        TopLoc_Location location;
        const occ::handle<Poly_Triangulation> triangulation =
            BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location);
        if (!triangulation.IsNull()) {
            triangles += triangulation->NbTriangles();
        }
    }
    return triangles;
}

} // namespace

class ParagliderView::Impl
{
public:
    Impl()
    {
        displayConnection = new Aspect_DisplayConnection;
        graphicDriver = new OpenGl_GraphicDriver(displayConnection);
        viewer = new V3d_Viewer(graphicDriver);
        viewer->SetDefaultLights();
        viewer->SetLightOn();

        context = new AIS_InteractiveContext(viewer);
        // The explicit BRepMesh pass owns the render mesh; the presentation
        // must not re-triangulate behind the resolution preference. Computed
        // (hidden-line) view mode stays off — it re-runs CPU HLR over every
        // NURBS face on each redraw, which takes seconds per frame.
        context->DefaultDrawer()->SetAutoTriangulation(false);
        view = viewer->CreateView();
        view->SetBgGradientColors(
            Quantity_Color(0.063, 0.106, 0.169, Quantity_TOC_RGB),
            Quantity_Color(0.027, 0.055, 0.094, Quantity_TOC_RGB),
            Aspect_GradientFillMethod_Vertical,
            false);
        view->TriedronDisplay(
            Aspect_TOTP_LEFT_LOWER,
            Quantity_NOC_WHITE,
            0.075,
            V3d_ZBUFFER);
        view->ChangeRenderingParams().NbMsaaSamples = 4;
        view->Camera()->SetProjectionType(
            Graphic3d_Camera::Projection_Perspective);
        view->SetProj(V3d_TypeOfOrientation_Zup_AxoRight);
    }

    ~Impl()
    {
        if (!context.IsNull()) {
            context->RemoveAll(false);
        }
        presentation.Nullify();
        context.Nullify();
        view.Nullify();
        viewer.Nullify();
        graphicDriver.Nullify();
        displayConnection.Nullify();
    }

    occ::handle<Aspect_DisplayConnection> displayConnection;
    occ::handle<OpenGl_GraphicDriver> graphicDriver;
    occ::handle<V3d_Viewer> viewer;
    occ::handle<AIS_InteractiveContext> context;
    occ::handle<V3d_View> view;
    occ::handle<AIS_Shape> presentation;
    TopoDS_Shape shape;

    QString modelPath;
    int surfaces = 0;
    int rationalSurfaces = 0;
    int shells = 0;
    int splines = 0;
    int triangles = 0;
    double widthMillimetres = 0.0;
    double depthMillimetres = 0.0;
    double heightMillimetres = 0.0;
    double diagonalMillimetres = 0.0;
    double resolutionScale = 1.0;
    bool perspective = true;
};

ParagliderView::ParagliderView(QWidget *parent)
    : QWidget(parent)
    , impl_(std::make_unique<Impl>())
{
    setObjectName(QStringLiteral("paragliderViewport"));
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(420, 320);
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_PaintOnScreen);
}

ParagliderView::~ParagliderView() = default;

bool ParagliderView::loadStep(const QString &path, QString *errorMessage)
{
    const QFileInfo fileInfo(path);
    if (!fileInfo.isFile()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("STEP file does not exist: %1")
                                .arg(fileInfo.absoluteFilePath());
        }
        return false;
    }

    try {
        STEPControl_Reader reader;
        const QByteArray encodedPath =
            fileInfo.absoluteFilePath().toUtf8();
        if (reader.ReadFile(encodedPath.constData()) != IFSelect_RetDone) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("OCCT could not read the STEP file.");
            }
            return false;
        }
        if (reader.TransferRoots() <= 0) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("The STEP file contains no transferable model roots.");
            }
            return false;
        }

        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("The STEP file contains no readable shape.");
            }
            return false;
        }

        Bnd_Box bounds;
        BRepBndLib::Add(shape, bounds, false);
        if (bounds.IsVoid()) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("The STEP model has no finite bounds.");
            }
            return false;
        }

        double xMin = 0.0;
        double yMin = 0.0;
        double zMin = 0.0;
        double xMax = 0.0;
        double yMax = 0.0;
        double zMax = 0.0;
        bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double diagonal = std::sqrt(
            std::pow(xMax - xMin, 2.0)
            + std::pow(yMax - yMin, 2.0)
            + std::pow(zMax - zMin, 2.0));

        // OCCT owns the render mesh. No application-side polygonization or
        // triangulation is used by the viewport.
        if (!triangulateShape(
                shape,
                meshDeflectionMillimetres(diagonal, impl_->resolutionScale))) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("OCCT could not triangulate the NURBS model.");
            }
            return false;
        }

        int surfaceCount = 0;
        int rationalSurfaceCount = 0;
        int shellCount = 0;
        int splineCount = 0;
        const int triangleCount = countTriangles(shape);

        for (TopExp_Explorer explorer(shape, TopAbs_FACE);
             explorer.More();
             explorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            const occ::handle<Geom_Surface> surface =
                BRep_Tool::Surface(face);
            const occ::handle<Geom_BSplineSurface> nurbs =
                occ::handle<Geom_BSplineSurface>::DownCast(surface);
            if (!nurbs.IsNull()) {
                ++surfaceCount;
                if (nurbs->IsURational() || nurbs->IsVRational()) {
                    ++rationalSurfaceCount;
                }
            }
        }
        for (TopExp_Explorer explorer(shape, TopAbs_EDGE);
             explorer.More();
             explorer.Next()) {
            double first = 0.0;
            double last = 0.0;
            const occ::handle<Geom_Curve> curve =
                BRep_Tool::Curve(
                    TopoDS::Edge(explorer.Current()),
                    first,
                    last);
            if (!occ::handle<Geom_BSplineCurve>::DownCast(curve).IsNull()) {
                ++splineCount;
            }
        }
        for (TopExp_Explorer explorer(shape, TopAbs_SHELL);
             explorer.More();
             explorer.Next()) {
            ++shellCount;
        }

        if (surfaceCount == 0
            || shellCount == 0
            || triangleCount == 0) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral(
                        "The STEP file is not a triangulatable NURBS surface model.");
            }
            return false;
        }

        impl_->context->RemoveAll(false);
        impl_->shape = shape;
        impl_->presentation = new AIS_Shape(shape);
        impl_->presentation->SetColor(
            Quantity_Color(0.20, 0.57, 0.88, Quantity_TOC_RGB));
        impl_->presentation->SetMaterial(
            Graphic3d_NameOfMaterial_Satin);
        impl_->presentation->Attributes()->SetFaceBoundaryDraw(true);
        impl_->presentation->Attributes()->SetFaceBoundaryAspect(
            new Prs3d_LineAspect(
                Quantity_Color(0.37, 0.82, 1.0, Quantity_TOC_RGB),
                Aspect_TOL_SOLID,
                1.0));
        impl_->context->Display(
            impl_->presentation,
            AIS_Shaded,
            0,
            false);

        impl_->modelPath = fileInfo.absoluteFilePath();
        impl_->surfaces = surfaceCount;
        impl_->rationalSurfaces = rationalSurfaceCount;
        impl_->shells = shellCount;
        impl_->splines = splineCount;
        impl_->triangles = triangleCount;
        impl_->widthMillimetres = xMax - xMin;
        impl_->depthMillimetres = yMax - yMin;
        impl_->heightMillimetres = zMax - zMin;
        impl_->diagonalMillimetres = diagonal;

        setView(ViewPreset::Isometric);
        fitAll();
        redraw();
        return true;
    } catch (const Standard_Failure &failure) {
        if (errorMessage != nullptr) {
            *errorMessage =
                QStringLiteral("OCCT model load failed: %1")
                    .arg(occFailureMessage(failure));
        }
        return false;
    }
}

void ParagliderView::clearModel()
{
    impl_->context->RemoveAll(false);
    impl_->presentation.Nullify();
    impl_->shape.Nullify();
    impl_->modelPath.clear();
    impl_->surfaces = 0;
    impl_->rationalSurfaces = 0;
    impl_->shells = 0;
    impl_->splines = 0;
    impl_->triangles = 0;
    impl_->widthMillimetres = 0.0;
    impl_->depthMillimetres = 0.0;
    impl_->heightMillimetres = 0.0;
    impl_->diagonalMillimetres = 0.0;
    redraw();
}

void ParagliderView::setTriangulationResolution(double deflectionScale)
{
    const double scale = std::clamp(deflectionScale, 0.05, 32.0);
    if (qFuzzyCompare(scale, impl_->resolutionScale)) {
        return;
    }
    impl_->resolutionScale = scale;
    if (!hasModel()) {
        return;
    }

    triangulateShape(
        impl_->shape,
        meshDeflectionMillimetres(impl_->diagonalMillimetres, scale));
    impl_->triangles = countTriangles(impl_->shape);
    if (!impl_->presentation.IsNull()) {
        impl_->presentation->SetToUpdate();
        impl_->context->Redisplay(impl_->presentation, false);
    }
    redraw();
}

double ParagliderView::triangulationResolution() const
{
    return impl_->resolutionScale;
}

void ParagliderView::fitAll()
{
    if (!hasModel()) {
        return;
    }
    impl_->view->FitAll(0.04, false);
    impl_->view->ZFitAll();
    redraw();
}

void ParagliderView::setView(ViewPreset preset)
{
    V3d_TypeOfOrientation orientation =
        V3d_TypeOfOrientation_Zup_AxoRight;
    switch (preset) {
    case ViewPreset::Isometric:
        orientation = V3d_TypeOfOrientation_Zup_AxoRight;
        break;
    case ViewPreset::Front:
        orientation = V3d_TypeOfOrientation_Zup_Front;
        break;
    case ViewPreset::Back:
        orientation = V3d_TypeOfOrientation_Zup_Back;
        break;
    case ViewPreset::Left:
        orientation = V3d_TypeOfOrientation_Zup_Left;
        break;
    case ViewPreset::Right:
        orientation = V3d_TypeOfOrientation_Zup_Right;
        break;
    case ViewPreset::Top:
        orientation = V3d_TypeOfOrientation_Zup_Top;
        break;
    case ViewPreset::Bottom:
        orientation = V3d_TypeOfOrientation_Zup_Bottom;
        break;
    }
    impl_->view->SetProj(orientation);
    redraw();
}

void ParagliderView::setPerspective(bool enabled)
{
    impl_->perspective = enabled;
    impl_->view->Camera()->SetProjectionType(
        enabled
            ? Graphic3d_Camera::Projection_Perspective
            : Graphic3d_Camera::Projection_Orthographic);
    impl_->view->ZFitAll();
    redraw();
}

void ParagliderView::toggleProjection()
{
    setPerspective(!impl_->perspective);
}

bool ParagliderView::isPerspective() const
{
    return impl_->perspective;
}

bool ParagliderView::hasModel() const
{
    return !impl_->shape.IsNull();
}

qsizetype ParagliderView::surfaceCount() const
{
    return impl_->surfaces;
}

qsizetype ParagliderView::rationalSurfaceCount() const
{
    return impl_->rationalSurfaces;
}

qsizetype ParagliderView::shellCount() const
{
    return impl_->shells;
}

qsizetype ParagliderView::splineCount() const
{
    return impl_->splines;
}

qsizetype ParagliderView::triangleCount() const
{
    return impl_->triangles;
}

QString ParagliderView::modelSummary() const
{
    if (!hasModel()) {
        return QStringLiteral("No model loaded");
    }
    return QStringLiteral(
               "%1 NURBS surfaces (%2 rational) in %3 shells · "
               "%4 splines · %5 OCCT triangles · %6 × %7 × %8 cm")
        .arg(impl_->surfaces)
        .arg(impl_->rationalSurfaces)
        .arg(impl_->shells)
        .arg(impl_->splines)
        .arg(impl_->triangles)
        .arg(impl_->widthMillimetres * centimetresPerMillimetre, 0, 'f', 1)
        .arg(impl_->depthMillimetres * centimetresPerMillimetre, 0, 'f', 1)
        .arg(impl_->heightMillimetres * centimetresPerMillimetre, 0, 'f', 1);
}

QSize ParagliderView::sizeHint() const
{
    return {760, 620};
}

QPaintEngine *ParagliderView::paintEngine() const
{
    return nullptr;
}

void ParagliderView::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    ensureNativeWindow();
    redraw();
}

void ParagliderView::paintEvent(QPaintEvent *)
{
    redraw();
}

void ParagliderView::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    if (!impl_->view->Window().IsNull()) {
        impl_->view->MustBeResized();
        redraw();
    }
}

void ParagliderView::mousePressEvent(QMouseEvent *event)
{
    setFocus(Qt::MouseFocusReason);
    previousMousePosition_ = event->position().toPoint();
    dragButton_ = event->button();
    shiftPan_ = event->modifiers().testFlag(Qt::ShiftModifier);
    if (dragButton_ == Qt::LeftButton && !shiftPan_) {
        impl_->view->StartRotation(
            previousMousePosition_.x(),
            previousMousePosition_.y());
    }
    updateCursor();
    event->accept();
}

void ParagliderView::mouseMoveEvent(QMouseEvent *event)
{
    if (dragButton_ == Qt::NoButton) {
        return;
    }

    const QPoint position = event->position().toPoint();
    const QPoint delta = position - previousMousePosition_;
    if (dragButton_ == Qt::RightButton
        || dragButton_ == Qt::MiddleButton
        || (dragButton_ == Qt::LeftButton && shiftPan_)) {
        impl_->view->Pan(delta.x(), -delta.y());
    } else if (dragButton_ == Qt::LeftButton) {
        impl_->view->Rotation(position.x(), position.y());
    }
    previousMousePosition_ = position;
    redraw();
    event->accept();
}

void ParagliderView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == dragButton_) {
        dragButton_ = Qt::NoButton;
        shiftPan_ = false;
        updateCursor();
    }
    event->accept();
}

void ParagliderView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        fitAll();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void ParagliderView::wheelEvent(QWheelEvent *event)
{
    const double steps =
        static_cast<double>(event->angleDelta().y()) / 120.0;
    if (std::abs(steps) > std::numeric_limits<double>::epsilon()) {
        impl_->view->SetZoom(std::pow(1.15, steps), true);
        redraw();
    }
    event->accept();
}

void ParagliderView::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_F:
        fitAll();
        break;
    case Qt::Key_0:
        setView(ViewPreset::Isometric);
        fitAll();
        break;
    case Qt::Key_1:
        setView(ViewPreset::Front);
        fitAll();
        break;
    case Qt::Key_2:
        setView(ViewPreset::Back);
        fitAll();
        break;
    case Qt::Key_3:
        setView(ViewPreset::Left);
        fitAll();
        break;
    case Qt::Key_4:
        setView(ViewPreset::Right);
        fitAll();
        break;
    case Qt::Key_5:
        setView(ViewPreset::Top);
        fitAll();
        break;
    case Qt::Key_6:
        setView(ViewPreset::Bottom);
        fitAll();
        break;
    case Qt::Key_P:
        toggleProjection();
        break;
    default:
        QWidget::keyPressEvent(event);
        return;
    }
    event->accept();
}

void ParagliderView::ensureNativeWindow()
{
    if (!impl_->view->Window().IsNull()) {
        return;
    }
#ifdef Q_OS_WIN
    const occ::handle<WNT_Window> window =
        new WNT_Window(
            reinterpret_cast<Aspect_Handle>(winId()),
            Quantity_NOC_BLACK);
#else
    const occ::handle<Aspect_NeutralWindow> window =
        new Aspect_NeutralWindow;
    window->SetNativeHandle(
        static_cast<Aspect_Drawable>(winId()));
    window->SetSize(width(), height());
#endif
    impl_->view->SetWindow(window);
    if (!window->IsMapped()) {
        window->Map();
    }
    impl_->view->MustBeResized();
}

void ParagliderView::redraw()
{
    if (!impl_->view->Window().IsNull()) {
        impl_->view->Redraw();
    } else {
        update();
    }
}

void ParagliderView::updateCursor()
{
    if (dragButton_ == Qt::NoButton) {
        unsetCursor();
    } else if (dragButton_ == Qt::LeftButton && !shiftPan_) {
        setCursor(Qt::ClosedHandCursor);
    } else {
        setCursor(Qt::SizeAllCursor);
    }
}
