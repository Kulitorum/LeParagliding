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

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_TextLabel.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <Aspect_GradientFillMethod.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Graphic3d_Camera.hxx>
#include <Graphic3d_ClipPlane.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_RenderingParams.hxx>
#include <NCollection_Sequence.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Prs3d_TypeOfHighlight.hxx>
#include <Quantity_Color.hxx>
#include <BinXCAFDrivers.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#ifdef Q_OS_WIN
#include <WNT_Window.hxx>
#else
#include <Aspect_NeutralWindow.hxx>
#endif

#include <QElapsedTimer>
#include <QFileInfo>
#include <QHash>
#include <QKeyEvent>
#include <QLoggingCategory>
#include <QTimer>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <QStringList>
#include <QWheelEvent>

#include <functional>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

// Interaction diagnostics; enable with
// QT_LOGGING_RULES="lep.viewport.debug=true".
Q_LOGGING_CATEGORY(lepViewport, "lep.viewport", QtWarningMsg)

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

Quantity_Color toQuantity(const QColor &color)
{
    return {color.redF(), color.greenF(), color.blueF(), Quantity_TOC_RGB};
}

QString labelName(const TDF_Label &label)
{
    occ::handle<TDataStd_Name> name;
    if (!label.FindAttribute(TDataStd_Name::GetID(), name)) {
        return {};
    }
    const TCollection_ExtendedString &text = name->Get();
    return QString::fromUtf16(
        reinterpret_cast<const char16_t *>(text.ToExtString()),
        text.Length());
}

// Maps the stable part names written by the engine's STEP assembly to
// display-colour roles; anything unrecognized is styled as OtherParts.
ParagliderView::ColorRole roleForPath(const QStringList &path)
{
    using ColorRole = ParagliderView::ColorRole;
    for (const QString &name : path) {
        if (name == QStringLiteral("Extrados")) {
            return ColorRole::Extrados;
        }
        if (name == QStringLiteral("Intrados")) {
            return ColorRole::Intrados;
        }
        if (name == QStringLiteral("Vents")) {
            return ColorRole::Vents;
        }
        if (name == QStringLiteral("Extrados curves")
            || name == QStringLiteral("Intrados curves")
            || name == QStringLiteral("Vent curves")) {
            return ColorRole::SurfaceWireframe;
        }
        if (name == QStringLiteral("Ribs")) {
            return ColorRole::Ribs;
        }
        if (name == QStringLiteral("Brake lines")) {
            return ColorRole::BrakeLines;
        }
        if (name == QStringLiteral("Diagonals")) {
            return ColorRole::Diagonals;
        }
        if (name.size() == 6 && name.startsWith(QStringLiteral("Plan "))) {
            const QChar plan = name.at(5);
            if (plan >= QLatin1Char('A') && plan <= QLatin1Char('F')) {
                return static_cast<ColorRole>(
                    static_cast<int>(ColorRole::PlanA)
                    + (plan.unicode() - u'A'));
            }
        }
    }
    return ColorRole::OtherParts;
}

// Maps a viewport position onto the arcball sphere (Shoemake mapping with a
// hyperbolic sheet outside the ball, so dragging beyond the ball spins the
// view around the camera axis).
gp_Vec mapToArcballSphere(const QPoint &position, int width, int height)
{
    const double radius =
        0.45 * static_cast<double>(std::min(width, height));
    const double x =
        (static_cast<double>(position.x()) - 0.5 * width) / radius;
    const double y =
        (0.5 * height - static_cast<double>(position.y())) / radius;
    const double lengthSquared = x * x + y * y;
    const double z = lengthSquared <= 0.5
                         ? std::sqrt(1.0 - lengthSquared)
                         : 0.5 / std::sqrt(lengthSquared);
    gp_Vec vector(x, y, z);
    vector.Normalize();
    return vector;
}

} // namespace

class ParagliderView::Impl
{
public:
    struct Part
    {
        PartInfo info;
        TopoDS_Shape shape;
        occ::handle<AIS_Shape> object;
        bool hasFaces = false;
        ColorRole surfaceRole = ColorRole::OtherParts;
        ColorRole curveRole = ColorRole::OtherParts;
    };

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

        const occ::handle<Prs3d_Drawer> &hoverStyle =
            context->HighlightStyle(Prs3d_TypeOfHighlight_Dynamic);
        hoverStyle->SetColor(Quantity_Color(1.0, 0.85, 0.30, Quantity_TOC_RGB));
        hoverStyle->SetDisplayMode(AIS_Shaded);
        const occ::handle<Prs3d_Drawer> &selectionStyle =
            context->HighlightStyle(Prs3d_TypeOfHighlight_Selected);
        selectionStyle->SetColor(
            Quantity_Color(1.0, 0.55, 0.10, Quantity_TOC_RGB));
        selectionStyle->SetDisplayMode(AIS_Shaded);

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

        for (int role = 0; role < colorRoleCount; ++role) {
            colors[role] = defaultColor(static_cast<ColorRole>(role));
        }
    }

    ~Impl()
    {
        if (!context.IsNull()) {
            context->RemoveAll(false);
        }
        parts.clear();
        objectToPart.clear();
        context.Nullify();
        view.Nullify();
        viewer.Nullify();
        graphicDriver.Nullify();
        displayConnection.Nullify();
    }

    void clearParts()
    {
        context->RemoveAll(false);
        parts.clear();
        objectToPart.clear();
        rootShape.Nullify();
        hoveredPart = -1;
        measureMarkers.Nullify();
        measureLabel.Nullify();
        measureCount = 0;
        pivotMarker.Nullify();
    }

    bool isAncestorOf(int ancestorId, int nodeId) const
    {
        if (nodeId < 0 || nodeId >= parts.size()) {
            return false;
        }
        int parent = parts.at(nodeId).info.parentId;
        while (parent >= 0) {
            if (parent == ancestorId) {
                return true;
            }
            parent = parts.at(parent).info.parentId;
        }
        return false;
    }

    void updateClipPlane()
    {
        if (!clipPlane.IsNull()) {
            view->RemoveClipPlane(clipPlane);
            clipPlane.Nullify();
        }
        if (clipAxis == ClipAxis::None || rootShape.IsNull()) {
            return;
        }

        gp_XYZ direction(1.0, 0.0, 0.0);
        double axisMin = xMinMillimetres;
        double axisMax = xMaxMillimetres;
        gp_Pnt point(
            0.5 * (xMinMillimetres + xMaxMillimetres),
            0.5 * (yMinMillimetres + yMaxMillimetres),
            0.5 * (zMinMillimetres + zMaxMillimetres));
        switch (clipAxis) {
        case ClipAxis::X:
            break;
        case ClipAxis::Y:
            direction = gp_XYZ(0.0, 1.0, 0.0);
            axisMin = yMinMillimetres;
            axisMax = yMaxMillimetres;
            break;
        case ClipAxis::Z:
            direction = gp_XYZ(0.0, 0.0, 1.0);
            axisMin = zMinMillimetres;
            axisMax = zMaxMillimetres;
            break;
        case ClipAxis::None:
            return;
        }
        const double coordinate =
            axisMin + clipPosition * (axisMax - axisMin);
        point.SetXYZ(
            gp_XYZ(
                clipAxis == ClipAxis::X ? coordinate : point.X(),
                clipAxis == ClipAxis::Y ? coordinate : point.Y(),
                clipAxis == ClipAxis::Z ? coordinate : point.Z()));
        if (clipFlipped) {
            direction.Reverse();
        }

        clipPlane = new Graphic3d_ClipPlane(gp_Pln(point, gp_Dir(direction)));
        clipPlane->SetCapping(true);
        clipPlane->SetCappingColor(
            Quantity_Color(0.35, 0.44, 0.55, Quantity_TOC_RGB));
        clipPlane->SetOn(true);
        view->AddClipPlane(clipPlane);
    }

    // Stops a running camera glide. jumpToEnd is used by programmatic
    // camera operations so they compose; user input keeps the mid-flight
    // state for a seamless takeover.
    void cancelCameraAnimation(bool jumpToEnd)
    {
        if (animationTimer != nullptr && animationTimer->isActive()) {
            animationTimer->stop();
            if (jumpToEnd && !animationEnd.IsNull()) {
                view->Camera()->Copy(animationEnd);
            }
        }
        animationStart.Nullify();
        animationEnd.Nullify();
    }

    bool stepCameraAnimation()
    {
        constexpr double durationMilliseconds = 220.0;
        const double linear = std::clamp(
            static_cast<double>(animationClock.elapsed())
                / durationMilliseconds,
            0.0,
            1.0);
        const double eased = linear * linear * (3.0 - 2.0 * linear);
        if (!animationStart.IsNull() && !animationEnd.IsNull()) {
            // Interpolate mutates the camera the handle points to, so the
            // local copy still drives the view's active camera.
            occ::handle<Graphic3d_Camera> camera = view->Camera();
            Graphic3d_Camera::Interpolate(
                animationStart,
                animationEnd,
                eased,
                camera);
        }
        return linear >= 1.0;
    }

    void showPivotMarker()
    {
        hidePivotMarker();
        if (rootShape.IsNull()) {
            return;
        }
        pivotMarker = new AIS_Shape(
            BRepBuilderAPI_MakeVertex(orbitPivot).Vertex());
        const Quantity_Color markerColor(0.93, 0.96, 1.0, Quantity_TOC_RGB);
        pivotMarker->SetColor(markerColor);
        pivotMarker->Attributes()->SetPointAspect(
            new Prs3d_PointAspect(Aspect_TOM_O_PLUS, markerColor, 4.0));
        context->Display(pivotMarker, 0, -1, false);
    }

    void hidePivotMarker()
    {
        if (!pivotMarker.IsNull()) {
            context->Remove(pivotMarker, false);
            pivotMarker.Nullify();
        }
    }

    // World size of one native pixel at the given depth in front of the
    // camera; the anchor of depth-true panning and dollying.
    double worldPerPixel(const gp_Pnt &at, int viewportHeightPixels) const
    {
        const occ::handle<Graphic3d_Camera> camera = view->Camera();
        double heightWorld = 0.0;
        if (camera->IsOrthographic()) {
            heightWorld = camera->ViewDimensions().Y();
        } else {
            const double depth = std::max(
                gp_Vec(camera->Eye(), at).Dot(gp_Vec(camera->Direction())),
                1.0e-6);
            heightWorld =
                2.0 * depth
                * std::tan(
                    camera->FOVy() * std::numbers::pi / 360.0);
        }
        return heightWorld
               / static_cast<double>(std::max(1, viewportHeightPixels));
    }

    void removeMeasureVisuals()
    {
        if (!measureMarkers.IsNull()) {
            context->Remove(measureMarkers, false);
            measureMarkers.Nullify();
        }
        if (!measureLabel.IsNull()) {
            context->Remove(measureLabel, false);
            measureLabel.Nullify();
        }
    }

    QString addMeasurePoint(const gp_Pnt &point)
    {
        if (measureCount == 2) {
            measureCount = 0;
        }
        measurePoints[measureCount] = point;
        ++measureCount;
        removeMeasureVisuals();

        BRep_Builder builder;
        TopoDS_Compound markers;
        builder.MakeCompound(markers);
        for (int index = 0; index < measureCount; ++index) {
            builder.Add(
                markers,
                BRepBuilderAPI_MakeVertex(measurePoints[index]).Vertex());
        }
        QString status;
        const Quantity_Color accent(0.98, 0.87, 0.26, Quantity_TOC_RGB);
        if (measureCount == 2) {
            const gp_Pnt &first = measurePoints[0];
            const gp_Pnt &second = measurePoints[1];
            if (first.Distance(second) > gp::Resolution()) {
                BRepBuilderAPI_MakeEdge makeEdge(first, second);
                if (makeEdge.IsDone()) {
                    builder.Add(markers, makeEdge.Edge());
                }
            }
            const double distanceCm =
                first.Distance(second) * centimetresPerMillimetre;
            const QString labelText =
                QStringLiteral("%1 cm").arg(distanceCm, 0, 'f', 1);
            measureLabel = new AIS_TextLabel;
            measureLabel->SetPosition(
                gp_Pnt(
                    0.5 * (first.XYZ() + second.XYZ())));
            measureLabel->SetText(
                TCollection_ExtendedString(
                    labelText.toUtf8().constData(),
                    true));
            measureLabel->SetColor(accent);
            measureLabel->SetHeight(16.0);
            context->Display(measureLabel, 0, -1, false);
            status = QStringLiteral(
                         "Distance %1 cm · ΔX %2 · ΔY %3 · ΔZ %4 cm")
                         .arg(distanceCm, 0, 'f', 1)
                         .arg((second.X() - first.X())
                                  * centimetresPerMillimetre, 0, 'f', 1)
                         .arg((second.Y() - first.Y())
                                  * centimetresPerMillimetre, 0, 'f', 1)
                         .arg((second.Z() - first.Z())
                                  * centimetresPerMillimetre, 0, 'f', 1);
        } else {
            status = QStringLiteral("Measure: click the second point");
        }

        measureMarkers = new AIS_Shape(markers);
        measureMarkers->SetColor(accent);
        measureMarkers->SetWidth(2.5);
        measureMarkers->Attributes()->SetPointAspect(
            new Prs3d_PointAspect(Aspect_TOM_O, accent, 4.0));
        // Selection mode -1: visible but never pickable, so measurement
        // marks do not interfere with further picking.
        context->Display(measureMarkers, 0, -1, false);
        return status;
    }

    int partIdOf(const occ::handle<AIS_InteractiveObject> &object) const
    {
        return objectToPart.value(object.get(), -1);
    }

    void collectLeaves(int id, QVector<int> *leaves) const
    {
        if (id < 0 || id >= parts.size()) {
            return;
        }
        if (!parts.at(id).info.isGroup) {
            leaves->append(id);
            return;
        }
        for (int index = id + 1; index < parts.size(); ++index) {
            // The parts vector is pre-order, so descendants form a
            // contiguous block; walk parents to test ancestry.
            int parent = parts.at(index).info.parentId;
            while (parent >= 0 && parent != id) {
                parent = parts.at(parent).info.parentId;
            }
            if (parent == id && !parts.at(index).info.isGroup) {
                leaves->append(index);
            }
        }
    }

    void applyPartStyle(Part &part)
    {
        if (part.object.IsNull()) {
            return;
        }
        const Quantity_Color surface =
            toQuantity(colors[static_cast<int>(part.surfaceRole)]);
        const Quantity_Color curve =
            toQuantity(colors[static_cast<int>(part.curveRole)]);
        if (part.hasFaces) {
            const occ::handle<Prs3d_Drawer> drawer = part.object->Attributes();
            drawer->SetupOwnShadingAspect();
            drawer->ShadingAspect()->SetMaterial(
                Graphic3d_NameOfMaterial_Satin);
            drawer->ShadingAspect()->SetColor(surface);
            drawer->SetFaceBoundaryDraw(true);
            drawer->SetFaceBoundaryAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
            drawer->SetWireAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
            drawer->SetLineAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
            drawer->SetFreeBoundaryAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
            drawer->SetUnFreeBoundaryAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
            drawer->SetSeenLineAspect(
                new Prs3d_LineAspect(curve, Aspect_TOL_SOLID, 1.0));
        } else {
            part.object->SetColor(curve);
            part.object->SetWidth(
                part.info.role == ColorRole::Ribs ? 1.6 : 2.2);
        }
    }

    occ::handle<Aspect_DisplayConnection> displayConnection;
    occ::handle<OpenGl_GraphicDriver> graphicDriver;
    occ::handle<V3d_Viewer> viewer;
    occ::handle<AIS_InteractiveContext> context;
    occ::handle<V3d_View> view;

    QVector<Part> parts;
    QHash<const AIS_InteractiveObject *, int> objectToPart;
    TopoDS_Shape rootShape;
    QColor colors[colorRoleCount];

    QString modelPath;
    int surfaces = 0;
    int rationalSurfaces = 0;
    int splines = 0;
    int triangles = 0;
    double widthMillimetres = 0.0;
    double depthMillimetres = 0.0;
    double heightMillimetres = 0.0;
    double diagonalMillimetres = 0.0;
    double xMinMillimetres = 0.0;
    double xMaxMillimetres = 0.0;
    double yMinMillimetres = 0.0;
    double yMaxMillimetres = 0.0;
    double zMinMillimetres = 0.0;
    double zMaxMillimetres = 0.0;
    double resolutionScale = 1.0;
    bool perspective = true;
    bool cameraInitialized = false;
    double transparency = 0.0;
    ClipAxis clipAxis = ClipAxis::None;
    bool clipFlipped = false;
    double clipPosition = 0.5;
    occ::handle<Graphic3d_ClipPlane> clipPlane;
    bool measureMode = false;
    int measureCount = 0;
    gp_Pnt measurePoints[2];
    occ::handle<AIS_Shape> measureMarkers;
    occ::handle<AIS_TextLabel> measureLabel;
    int hoveredPart = -1;
    gp_Vec arcballVector;
    bool arcballActive = false;
    gp_Pnt orbitPivot;
    bool orbitPivotValid = false;
    gp_Pnt grabPoint;
    occ::handle<AIS_Shape> pivotMarker;
    QTimer *animationTimer = nullptr;
    QElapsedTimer animationClock;
    occ::handle<Graphic3d_Camera> animationStart;
    occ::handle<Graphic3d_Camera> animationEnd;
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

    impl_->animationTimer = new QTimer(this);
    impl_->animationTimer->setInterval(16);
    connect(impl_->animationTimer, &QTimer::timeout, this, [this] {
        if (impl_->stepCameraAnimation()) {
            impl_->animationTimer->stop();
            impl_->animationStart.Nullify();
            impl_->animationEnd.Nullify();
        }
        redraw();
    });
}

ParagliderView::~ParagliderView() = default;

bool ParagliderView::loadStep(const QString &path, QString *errorMessage)
{
    const QFileInfo fileInfo(path);
    if (!fileInfo.isFile()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Model file does not exist: %1")
                                .arg(fileInfo.absoluteFilePath());
        }
        return false;
    }
    const bool binaryXcaf =
        fileInfo.suffix().compare(QStringLiteral("xbf"), Qt::CaseInsensitive)
        == 0;

    try {
        occ::handle<TDocStd_Document> document;
        const QByteArray encodedPath =
            fileInfo.absoluteFilePath().toUtf8();
        if (binaryXcaf) {
            // Binary XCAF from a preview engine run: open the document
            // directly, no STEP entity translation involved.
            const occ::handle<TDocStd_Application> application =
                XCAFApp_Application::GetApplication();
            BinXCAFDrivers::DefineFormat(application);
            if (application->Open(
                    TCollection_ExtendedString(encodedPath.constData(), true),
                    document)
                    != PCDM_RS_OK
                || document.IsNull()) {
                if (errorMessage != nullptr) {
                    *errorMessage = QStringLiteral(
                        "OCCT could not read the binary XCAF model.");
                }
                return false;
            }
        } else {
            STEPCAFControl_Reader reader;
            reader.SetNameMode(true);
            reader.SetColorMode(true);
            if (reader.ReadFile(encodedPath.constData()) != IFSelect_RetDone) {
                if (errorMessage != nullptr) {
                    *errorMessage =
                        QStringLiteral("OCCT could not read the STEP file.");
                }
                return false;
            }

            XCAFApp_Application::GetApplication()->NewDocument(
                "MDTV-XCAF",
                document);
            if (!reader.Transfer(document)) {
                if (errorMessage != nullptr) {
                    *errorMessage =
                        QStringLiteral("The STEP file contains no transferable model roots.");
                }
                return false;
            }
        }

        const occ::handle<XCAFDoc_ShapeTool> shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());

        QVector<Impl::Part> parts;
        const std::function<void(const TDF_Label &,
                                 const TopLoc_Location &,
                                 int,
                                 QStringList)>
            traverse = [&](const TDF_Label &label,
                           const TopLoc_Location &location,
                           int parentId,
                           QStringList path) {
                QString name = labelName(label);
                if (name.isEmpty()) {
                    name = QStringLiteral("Part %1").arg(parts.size() + 1);
                }
                path.append(name);

                Impl::Part part;
                part.info.id = static_cast<int>(parts.size());
                part.info.parentId = parentId;
                part.info.name = name;
                part.info.role = roleForPath(path);

                if (shapeTool->IsAssembly(label)) {
                    part.info.isGroup = true;
                    const int groupId = part.info.id;
                    parts.append(part);

                    NCollection_Sequence<TDF_Label> components;
                    XCAFDoc_ShapeTool::GetComponents(label, components);
                    for (int index = 1; index <= components.Length(); ++index) {
                        const TDF_Label &component = components.Value(index);
                        TDF_Label referred;
                        if (!XCAFDoc_ShapeTool::GetReferredShape(
                                component,
                                referred)) {
                            continue;
                        }
                        traverse(
                            referred,
                            location.Multiplied(
                                XCAFDoc_ShapeTool::GetLocation(component)),
                            groupId,
                            path);
                    }
                    return;
                }

                TopoDS_Shape shape = shapeTool->GetShape(label);
                if (shape.IsNull()) {
                    return;
                }
                part.shape = shape.Moved(location);
                part.hasFaces =
                    TopExp_Explorer(part.shape, TopAbs_FACE).More();
                part.surfaceRole = part.info.role;
                part.curveRole =
                    part.hasFaces
                        ? ColorRole::SurfaceWireframe
                        : part.info.role;
                parts.append(part);
            };

        NCollection_Sequence<TDF_Label> freeShapes;
        shapeTool->GetFreeShapes(freeShapes);
        if (freeShapes.Length() == 1
            && shapeTool->IsAssembly(freeShapes.Value(1))) {
            // Hoist the single "Wing" root so its sections become the
            // top-level tree entries.
            NCollection_Sequence<TDF_Label> components;
            XCAFDoc_ShapeTool::GetComponents(freeShapes.Value(1), components);
            for (int index = 1; index <= components.Length(); ++index) {
                TDF_Label referred;
                if (XCAFDoc_ShapeTool::GetReferredShape(
                        components.Value(index),
                        referred)) {
                    traverse(
                        referred,
                        XCAFDoc_ShapeTool::GetLocation(
                            components.Value(index)),
                        -1,
                        {});
                }
            }
        } else {
            for (int index = 1; index <= freeShapes.Length(); ++index) {
                traverse(freeShapes.Value(index), TopLoc_Location(), -1, {});
            }
        }

        // The shapes own their geometry independently of the document;
        // close it or every preview rebuild leaks a session document.
        XCAFApp_Application::GetApplication()->Close(document);

        bool anyLeaf = false;
        for (const Impl::Part &part : parts) {
            if (!part.info.isGroup) {
                anyLeaf = true;
                break;
            }
        }
        if (!anyLeaf) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("The STEP file contains no readable shape.");
            }
            return false;
        }

        BRep_Builder builder;
        TopoDS_Compound rootShape;
        builder.MakeCompound(rootShape);
        for (const Impl::Part &part : parts) {
            if (!part.info.isGroup) {
                builder.Add(rootShape, part.shape);
            }
        }

        Bnd_Box bounds;
        BRepBndLib::Add(rootShape, bounds, false);
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
                rootShape,
                meshDeflectionMillimetres(diagonal, impl_->resolutionScale))) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral("OCCT could not triangulate the NURBS model.");
            }
            return false;
        }

        int surfaceCount = 0;
        int rationalSurfaceCount = 0;
        int splineCount = 0;
        const int triangleCount = countTriangles(rootShape);

        for (TopExp_Explorer explorer(rootShape, TopAbs_FACE);
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
        for (TopExp_Explorer explorer(rootShape, TopAbs_EDGE);
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

        if (surfaceCount > 0 && triangleCount == 0) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    QStringLiteral(
                        "The STEP file is not a triangulatable NURBS surface model.");
            }
            return false;
        }

        impl_->clearParts();
        impl_->parts = std::move(parts);
        impl_->rootShape = rootShape;
        for (Impl::Part &part : impl_->parts) {
            if (part.info.isGroup) {
                continue;
            }
            part.object = new AIS_Shape(part.shape);
            impl_->applyPartStyle(part);
            if (part.hasFaces && impl_->transparency > 0.0) {
                part.object->SetTransparency(impl_->transparency);
            }
            impl_->objectToPart.insert(part.object.get(), part.info.id);
            impl_->context->Display(part.object, AIS_Shaded, 0, false);
        }

        impl_->modelPath = fileInfo.absoluteFilePath();
        impl_->surfaces = surfaceCount;
        impl_->rationalSurfaces = rationalSurfaceCount;
        impl_->splines = splineCount;
        impl_->triangles = triangleCount;
        impl_->widthMillimetres = xMax - xMin;
        impl_->depthMillimetres = yMax - yMin;
        impl_->heightMillimetres = zMax - zMin;
        impl_->diagonalMillimetres = diagonal;
        impl_->xMinMillimetres = xMin;
        impl_->xMaxMillimetres = xMax;
        impl_->yMinMillimetres = yMin;
        impl_->yMaxMillimetres = yMax;
        impl_->zMinMillimetres = zMin;
        impl_->zMaxMillimetres = zMax;

        impl_->updateClipPlane();
        if (!impl_->cameraInitialized) {
            setView(ViewPreset::Isometric);
            fitAll();
            impl_->cameraInitialized = true;
        } else {
            // Preview rebuild of the same design: keep the user's camera,
            // only refresh the depth range for the new geometry.
            impl_->view->ZFitAll();
        }
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
    impl_->clearParts();
    impl_->modelPath.clear();
    impl_->surfaces = 0;
    impl_->rationalSurfaces = 0;
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
        impl_->rootShape,
        meshDeflectionMillimetres(impl_->diagonalMillimetres, scale));
    impl_->triangles = countTriangles(impl_->rootShape);
    for (Impl::Part &part : impl_->parts) {
        if (!part.object.IsNull()) {
            part.object->SetToUpdate();
            impl_->context->Redisplay(part.object, false);
        }
    }
    redraw();
}

double ParagliderView::triangulationResolution() const
{
    return impl_->resolutionScale;
}

void ParagliderView::runCameraOperation(
    const std::function<void()> &operation,
    bool animate)
{
    impl_->cancelCameraAnimation(true);
    const bool canAnimate =
        animate
        && impl_->cameraInitialized
        && isVisible()
        && !impl_->view->Window().IsNull();
    if (!canAnimate) {
        operation();
        impl_->orbitPivot = impl_->view->Camera()->Center();
        impl_->orbitPivotValid = true;
        redraw();
        return;
    }

    const occ::handle<Graphic3d_Camera> start =
        new Graphic3d_Camera(impl_->view->Camera());
    operation();
    impl_->animationEnd = new Graphic3d_Camera(impl_->view->Camera());
    impl_->orbitPivot = impl_->view->Camera()->Center();
    impl_->orbitPivotValid = true;
    impl_->view->Camera()->Copy(start);
    impl_->animationStart = start;
    impl_->animationClock.start();
    impl_->animationTimer->start();
}

void ParagliderView::fitAll()
{
    if (!hasModel()) {
        return;
    }
    runCameraOperation(
        [this] {
            impl_->view->FitAll(0.04, false);
            impl_->view->ZFitAll();
        },
        true);
}

void ParagliderView::fitSelection()
{
    if (!hasModel()) {
        return;
    }
    Bnd_Box bounds;
    for (impl_->context->InitSelected();
         impl_->context->MoreSelected();
         impl_->context->NextSelected()) {
        const int id =
            impl_->partIdOf(impl_->context->SelectedInteractive());
        if (id >= 0) {
            BRepBndLib::Add(impl_->parts.at(id).shape, bounds, false);
        }
    }
    if (bounds.IsVoid()) {
        fitAll();
        return;
    }
    bounds.Enlarge(impl_->diagonalMillimetres * 0.01);
    runCameraOperation(
        [this, bounds] {
            impl_->view->FitAll(bounds, 0.10, false);
            impl_->view->ZFitAll();
        },
        true);
}

void ParagliderView::setView(ViewPreset preset, bool fit)
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
    runCameraOperation(
        [this, orientation, fit] {
            impl_->view->SetProj(orientation);
            if (fit && hasModel()) {
                impl_->view->FitAll(0.04, false);
            }
            impl_->view->ZFitAll();
        },
        true);
}

void ParagliderView::resetCamera()
{
    impl_->perspective = true;
    runCameraOperation(
        [this] {
            impl_->view->Camera()->SetProjectionType(
                Graphic3d_Camera::Projection_Perspective);
            // SetProj also resets the camera up vector, clearing any
            // arcball roll.
            impl_->view->SetProj(V3d_TypeOfOrientation_Zup_AxoRight);
            if (hasModel()) {
                impl_->view->FitAll(0.04, false);
            }
            impl_->view->ZFitAll();
        },
        true);
}

void ParagliderView::resetCameraOnNextLoad()
{
    impl_->cameraInitialized = false;
}

void ParagliderView::setSurfaceTransparency(double transparency01)
{
    const double transparency = std::clamp(transparency01, 0.0, 0.9);
    if (qFuzzyCompare(1.0 + transparency, 1.0 + impl_->transparency)) {
        return;
    }
    impl_->transparency = transparency;
    for (Impl::Part &part : impl_->parts) {
        if (!part.hasFaces || part.object.IsNull()) {
            continue;
        }
        if (transparency > 0.0) {
            part.object->SetTransparency(transparency);
        } else {
            part.object->UnsetTransparency();
        }
        part.object->SynchronizeAspects();
    }
    redraw();
}

double ParagliderView::surfaceTransparency() const
{
    return impl_->transparency;
}

void ParagliderView::setClipPlane(
    ClipAxis axis,
    bool flipped,
    double position01)
{
    impl_->clipAxis = axis;
    impl_->clipFlipped = flipped;
    impl_->clipPosition = std::clamp(position01, 0.0, 1.0);
    impl_->updateClipPlane();
    redraw();
}

void ParagliderView::setMeasureMode(bool enabled)
{
    if (enabled == impl_->measureMode) {
        return;
    }
    impl_->measureMode = enabled;
    if (enabled) {
        setCursor(Qt::CrossCursor);
        if (measurementChanged) {
            measurementChanged(
                QStringLiteral("Measure: click the first point"));
        }
    } else {
        unsetCursor();
        impl_->removeMeasureVisuals();
        impl_->measureCount = 0;
        redraw();
        if (measurementChanged) {
            measurementChanged(QString());
        }
    }
    if (measureModeChanged) {
        measureModeChanged(enabled);
    }
}

bool ParagliderView::isMeasureMode() const
{
    return impl_->measureMode;
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
    return !impl_->rootShape.IsNull();
}

qsizetype ParagliderView::partCount() const
{
    qsizetype count = 0;
    for (const Impl::Part &part : impl_->parts) {
        if (!part.info.isGroup) {
            ++count;
        }
    }
    return count;
}

qsizetype ParagliderView::surfaceCount() const
{
    return impl_->surfaces;
}

qsizetype ParagliderView::rationalSurfaceCount() const
{
    return impl_->rationalSurfaces;
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
               "%1 parts · %2 NURBS surfaces (%3 rational) · "
               "%4 splines · %5 OCCT triangles · %6 × %7 × %8 cm")
        .arg(partCount())
        .arg(impl_->surfaces)
        .arg(impl_->rationalSurfaces)
        .arg(impl_->splines)
        .arg(impl_->triangles)
        .arg(impl_->widthMillimetres * centimetresPerMillimetre, 0, 'f', 1)
        .arg(impl_->depthMillimetres * centimetresPerMillimetre, 0, 'f', 1)
        .arg(impl_->heightMillimetres * centimetresPerMillimetre, 0, 'f', 1);
}

QVector<ParagliderView::PartInfo> ParagliderView::partTree() const
{
    QVector<PartInfo> tree;
    tree.reserve(impl_->parts.size());
    for (const Impl::Part &part : impl_->parts) {
        tree.append(part.info);
    }
    return tree;
}

QString ParagliderView::partPath(int id) const
{
    QStringList names;
    while (id >= 0 && id < impl_->parts.size()) {
        names.prepend(impl_->parts.at(id).info.name);
        id = impl_->parts.at(id).info.parentId;
    }
    return names.join(QStringLiteral(" / "));
}

void ParagliderView::setPartVisible(int id, bool visible)
{
    QVector<int> leaves;
    impl_->collectLeaves(id, &leaves);
    if (id >= 0 && id < impl_->parts.size()) {
        impl_->parts[id].info.visible = visible;
    }
    for (const int leaf : leaves) {
        Impl::Part &part = impl_->parts[leaf];
        part.info.visible = visible;
        if (part.object.IsNull()) {
            continue;
        }
        if (visible) {
            impl_->context->Display(part.object, AIS_Shaded, 0, false);
        } else {
            impl_->context->Erase(part.object, false);
        }
    }
    redraw();
}

void ParagliderView::showOnlyPart(int id)
{
    if (id < 0 || id >= impl_->parts.size()) {
        return;
    }
    for (Impl::Part &part : impl_->parts) {
        const bool keep =
            part.info.id == id
            || impl_->isAncestorOf(id, part.info.id)
            || impl_->isAncestorOf(part.info.id, id);
        part.info.visible = keep;
        if (part.info.isGroup || part.object.IsNull()) {
            continue;
        }
        if (keep) {
            impl_->context->Display(part.object, AIS_Shaded, 0, false);
        } else {
            impl_->context->Erase(part.object, false);
        }
    }
    redraw();
}

void ParagliderView::showAllParts()
{
    for (Impl::Part &part : impl_->parts) {
        part.info.visible = true;
        if (!part.info.isGroup && !part.object.IsNull()) {
            impl_->context->Display(part.object, AIS_Shaded, 0, false);
        }
    }
    redraw();
}

void ParagliderView::selectPart(int id)
{
    impl_->context->ClearSelected(false);
    QVector<int> leaves;
    impl_->collectLeaves(id, &leaves);
    for (const int leaf : leaves) {
        const Impl::Part &part = impl_->parts.at(leaf);
        if (!part.object.IsNull() && part.info.visible) {
            impl_->context->AddOrRemoveSelected(part.object, false);
        }
    }
    redraw();
}

void ParagliderView::clearSelection()
{
    impl_->context->ClearSelected(false);
    redraw();
}

void ParagliderView::zoomToPart(int id)
{
    QVector<int> leaves;
    impl_->collectLeaves(id, &leaves);
    Bnd_Box bounds;
    for (const int leaf : leaves) {
        BRepBndLib::Add(impl_->parts.at(leaf).shape, bounds, false);
    }
    if (bounds.IsVoid()) {
        return;
    }
    bounds.Enlarge(impl_->diagonalMillimetres * 0.01);
    runCameraOperation(
        [this, bounds] {
            impl_->view->FitAll(bounds, 0.10, false);
            impl_->view->ZFitAll();
        },
        true);
}

QString ParagliderView::colorRoleLabel(ColorRole role)
{
    switch (role) {
    case ColorRole::Extrados:
        return QStringLiteral("Extrados (top surface)");
    case ColorRole::Intrados:
        return QStringLiteral("Intrados (bottom surface)");
    case ColorRole::Vents:
        return QStringLiteral("Vents");
    case ColorRole::SurfaceWireframe:
        return QStringLiteral("Surface wireframe");
    case ColorRole::Ribs:
        return QStringLiteral("Ribs");
    case ColorRole::PlanA:
        return QStringLiteral("Lines · Plan A");
    case ColorRole::PlanB:
        return QStringLiteral("Lines · Plan B");
    case ColorRole::PlanC:
        return QStringLiteral("Lines · Plan C");
    case ColorRole::PlanD:
        return QStringLiteral("Lines · Plan D");
    case ColorRole::PlanE:
        return QStringLiteral("Lines · Plan E");
    case ColorRole::PlanF:
        return QStringLiteral("Lines · Plan F");
    case ColorRole::BrakeLines:
        return QStringLiteral("Brake lines");
    case ColorRole::OtherParts:
        return QStringLiteral("Other parts");
    case ColorRole::Diagonals:
        return QStringLiteral("Diagonals (H/V ribs)");
    }
    return {};
}

QColor ParagliderView::defaultColor(ColorRole role)
{
    // Kept in sync with the default colours the engine embeds in the STEP.
    switch (role) {
    case ColorRole::Extrados:
        return QColor::fromRgbF(0.20, 0.57, 0.88);
    case ColorRole::Intrados:
        return QColor::fromRgbF(0.45, 0.69, 0.90);
    case ColorRole::Vents:
        return QColor::fromRgbF(0.93, 0.60, 0.23);
    case ColorRole::SurfaceWireframe:
        return QColor::fromRgbF(0.37, 0.82, 1.0);
    case ColorRole::Ribs:
        return QColor::fromRgbF(0.78, 0.80, 0.84);
    case ColorRole::PlanA:
        return QColor::fromRgbF(0.89, 0.29, 0.29);
    case ColorRole::PlanB:
        return QColor::fromRgbF(0.95, 0.62, 0.19);
    case ColorRole::PlanC:
        return QColor::fromRgbF(0.35, 0.79, 0.42);
    case ColorRole::PlanD:
        return QColor::fromRgbF(0.29, 0.74, 0.86);
    case ColorRole::PlanE:
        return QColor::fromRgbF(0.72, 0.47, 0.90);
    case ColorRole::PlanF:
        return QColor::fromRgbF(0.62, 0.66, 0.72);
    case ColorRole::BrakeLines:
        return QColor::fromRgbF(0.95, 0.83, 0.28);
    case ColorRole::OtherParts:
        return QColor::fromRgbF(0.62, 0.66, 0.72);
    case ColorRole::Diagonals:
        return QColor::fromRgbF(0.83, 0.45, 0.74);
    }
    return QColor(Qt::gray);
}

QColor ParagliderView::color(ColorRole role) const
{
    return impl_->colors[static_cast<int>(role)];
}

void ParagliderView::setColor(ColorRole role, const QColor &color)
{
    if (!color.isValid()
        || impl_->colors[static_cast<int>(role)] == color) {
        return;
    }
    impl_->colors[static_cast<int>(role)] = color;
    bool changed = false;
    for (Impl::Part &part : impl_->parts) {
        if (part.object.IsNull()
            || (part.surfaceRole != role && part.curveRole != role)) {
            continue;
        }
        impl_->applyPartStyle(part);
        impl_->context->Redisplay(part.object, false);
        changed = true;
    }
    if (changed) {
        redraw();
    }
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
    impl_->cancelCameraAnimation(false);
    previousMousePosition_ = event->position().toPoint();
    pressPosition_ = previousMousePosition_;
    dragButton_ = event->button();
    dragMoved_ = false;
    shiftPan_ = event->modifiers().testFlag(Qt::ShiftModifier);

    gp_Pnt picked;
    bool havePicked = false;
    if (hasModel() && !impl_->view->Window().IsNull()) {
        const QPoint native = nativePixel(previousMousePosition_);
        impl_->context->MoveTo(
            native.x(),
            native.y(),
            impl_->view,
            false);
        if (impl_->context->MainSelector()->NbPicked() > 0) {
            picked = impl_->context->MainSelector()->PickedPoint(1);
            havePicked = true;
        }
    }

    const bool panButton =
        dragButton_ == Qt::RightButton
        || dragButton_ == Qt::MiddleButton
        || (dragButton_ == Qt::LeftButton && shiftPan_);
    if (dragButton_ == Qt::LeftButton && !shiftPan_) {
        // The point under the cursor becomes the orbit pivot, so a
        // zoomed-in detail rotates around itself instead of swinging
        // around the far-away wing centre.
        if (havePicked) {
            impl_->orbitPivot = picked;
            impl_->orbitPivotValid = true;
        } else if (!impl_->orbitPivotValid) {
            impl_->orbitPivot = impl_->view->Camera()->Center();
            impl_->orbitPivotValid = true;
        }
        impl_->arcballVector =
            mapToArcballSphere(previousMousePosition_, width(), height());
        impl_->arcballActive = true;
    } else if (panButton && hasModel()) {
        if (havePicked) {
            impl_->grabPoint = picked;
        } else {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            const QPoint native = nativePixel(previousMousePosition_);
            impl_->view->Convert(native.x(), native.y(), x, y, z);
            impl_->grabPoint = gp_Pnt(x, y, z);
        }
    }
    updateCursor();
    event->accept();
}

void ParagliderView::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint position = event->position().toPoint();
    if (dragButton_ == Qt::NoButton) {
        updateHover(position);
        return;
    }

    if ((position - pressPosition_).manhattanLength() > 3) {
        dragMoved_ = true;
    }

    const QPoint delta =
        nativePixel(position) - nativePixel(previousMousePosition_);
    if (dragButton_ == Qt::RightButton
        || dragButton_ == Qt::MiddleButton
        || (dragButton_ == Qt::LeftButton && shiftPan_)) {
        // Depth-true "grab" pan: the point that was pressed stays glued to
        // the cursor at any zoom level.
        const occ::handle<Graphic3d_Camera> camera = impl_->view->Camera();
        const int viewportHeightPixels =
            std::max(1, qRound(height() * devicePixelRatioF()));
        const double scale =
            impl_->worldPerPixel(impl_->grabPoint, viewportHeightPixels);
        const gp_Dir direction = camera->Direction();
        gp_Vec right = gp_Vec(direction).Crossed(gp_Vec(camera->Up()));
        if (right.Magnitude() > gp::Resolution()) {
            right.Normalize();
            const gp_Vec upOrtho = right.Crossed(gp_Vec(direction));
            const gp_Vec translation =
                right.Multiplied(-delta.x() * scale)
                + upOrtho.Multiplied(delta.y() * scale);
            camera->SetEye(camera->Eye().Translated(translation));
            camera->SetCenter(camera->Center().Translated(translation));
        }
    } else if (dragButton_ == Qt::LeftButton && impl_->arcballActive) {
        // Arcball rotation about the picked pivot: the drag maps to a
        // rotation on a virtual sphere; outside the sphere the hyperbolic
        // mapping degenerates to a roll around the view axis.
        if (dragMoved_ && impl_->pivotMarker.IsNull()) {
            impl_->showPivotMarker();
        }
        const gp_Vec current =
            mapToArcballSphere(position, width(), height());
        const gp_Vec previous = impl_->arcballVector;
        gp_Vec axis = previous.Crossed(current);
        const double angle =
            std::atan2(axis.Magnitude(), previous.Dot(current));
        if (axis.Magnitude() > gp::Resolution()
            && std::abs(angle) > 1.0e-6) {
            axis.Normalize();
            const occ::handle<Graphic3d_Camera> camera =
                impl_->view->Camera();
            const gp_Dir direction = camera->Direction();
            const gp_Dir up = camera->Up();
            gp_Vec right = gp_Vec(direction).Crossed(gp_Vec(up));
            if (right.Magnitude() > gp::Resolution()) {
                right.Normalize();
                gp_Vec worldAxis =
                    right.Multiplied(axis.X())
                    + gp_Vec(up).Multiplied(axis.Y())
                    - gp_Vec(direction).Multiplied(axis.Z());
                if (worldAxis.Magnitude() > gp::Resolution()) {
                    gp_Trsf rotation;
                    rotation.SetRotation(
                        gp_Ax1(
                            impl_->orbitPivotValid
                                ? impl_->orbitPivot
                                : camera->Center(),
                            gp_Dir(worldAxis)),
                        -angle);
                    camera->SetCenter(
                        camera->Center().Transformed(rotation));
                    camera->SetEye(
                        camera->Eye().Transformed(rotation));
                    camera->SetUp(up.Transformed(rotation));
                    impl_->view->Invalidate();
                }
            }
        }
        impl_->arcballVector = current;
    }
    previousMousePosition_ = position;
    if (dragButton_ != Qt::NoButton) {
        redraw();
    }
    event->accept();
}

void ParagliderView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == dragButton_) {
        const bool clicked = !dragMoved_;
        const bool leftButton = dragButton_ == Qt::LeftButton;
        dragButton_ = Qt::NoButton;
        shiftPan_ = false;
        impl_->arcballActive = false;
        if (!impl_->pivotMarker.IsNull()) {
            impl_->hidePivotMarker();
            redraw();
        }
        updateCursor();
        if (clicked && leftButton) {
            pickAt(
                event->position().toPoint(),
                event->modifiers().testFlag(Qt::ControlModifier));
        }
    }
    event->accept();
}

void ParagliderView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (!impl_->measureMode) {
            // Double-click a part: glide the camera target onto that point;
            // double-click empty space: frame the whole model again.
            pickAt(event->position().toPoint(), true);
            if (impl_->context->MainSelector()->NbPicked() == 0) {
                fitAll();
            }
        }
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void ParagliderView::wheelEvent(QWheelEvent *event)
{
    const double steps =
        static_cast<double>(event->angleDelta().y()) / 120.0;
    if (std::abs(steps) > std::numeric_limits<double>::epsilon()
        && hasModel()) {
        impl_->cancelCameraAnimation(false);
        const bool precise =
            event->modifiers().testFlag(Qt::ShiftModifier);
        const double factor = std::pow(precise ? 1.03 : 1.15, steps);
        const QPoint position =
            nativePixel(event->position().toPoint());

        // The 3D anchor under the cursor: the model point when there is
        // one, otherwise the matching point on the camera-target plane.
        impl_->context->MoveTo(
            position.x(),
            position.y(),
            impl_->view,
            false);
        gp_Pnt anchor;
        if (impl_->context->MainSelector()->NbPicked() > 0) {
            anchor = impl_->context->MainSelector()->PickedPoint(1);
        } else {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            impl_->view->Convert(position.x(), position.y(), x, y, z);
            anchor = gp_Pnt(x, y, z);
        }

        const occ::handle<Graphic3d_Camera> camera = impl_->view->Camera();
        if (camera->IsOrthographic()) {
            // Scale about the anchor: zoom, then pan its projection back
            // under the cursor. SetZoom's second argument means "start a
            // fresh camera operation", not "update the view".
            impl_->view->SetZoom(factor, true);
            int projectedX = 0;
            int projectedY = 0;
            impl_->view->Convert(
                anchor.X(),
                anchor.Y(),
                anchor.Z(),
                projectedX,
                projectedY);
            impl_->view->Pan(
                position.x() - projectedX,
                projectedY - position.y());
        } else {
            // Perspective dolly: step a fraction of the remaining distance
            // toward the anchor, so approach decelerates near the surface
            // and zooming back out retraces the same path.
            double fraction = 1.0 - 1.0 / factor;
            gp_Vec toAnchor(camera->Eye(), anchor);
            const double distance = toAnchor.Magnitude();
            const double minimumDistance =
                std::max(impl_->diagonalMillimetres * 1.0e-4, 0.2);
            if (distance > gp::Resolution()) {
                if (fraction > 0.0
                    && distance * (1.0 - fraction) < minimumDistance) {
                    fraction =
                        std::max(0.0, 1.0 - minimumDistance / distance);
                }
                const gp_Vec move = toAnchor.Multiplied(fraction);
                camera->SetEye(camera->Eye().Translated(move));
                camera->SetCenter(camera->Center().Translated(move));
            }
        }
        // What you zoom toward is what you meant to inspect: make it the
        // orbit pivot too.
        impl_->orbitPivot = anchor;
        impl_->orbitPivotValid = true;
        qCDebug(lepViewport)
            << "wheel" << steps << "factor" << factor
            << "anchor" << anchor.X() << anchor.Y() << anchor.Z()
            << "eye distance" << camera->Distance();
        redraw();
    }
    event->accept();
}

void ParagliderView::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_F:
        fitSelection();
        break;
    case Qt::Key_R:
    case Qt::Key_Home:
        resetCamera();
        break;
    case Qt::Key_0:
        setView(ViewPreset::Isometric, true);
        break;
    case Qt::Key_1:
        setView(ViewPreset::Front, true);
        break;
    case Qt::Key_2:
        setView(ViewPreset::Back, true);
        break;
    case Qt::Key_3:
        setView(ViewPreset::Left, true);
        break;
    case Qt::Key_4:
        setView(ViewPreset::Right, true);
        break;
    case Qt::Key_5:
        setView(ViewPreset::Top, true);
        break;
    case Qt::Key_6:
        setView(ViewPreset::Bottom, true);
        break;
    case Qt::Key_P:
        toggleProjection();
        break;
    case Qt::Key_M:
        setMeasureMode(!isMeasureMode());
        break;
    case Qt::Key_Escape:
        if (!isMeasureMode()) {
            QWidget::keyPressEvent(event);
            return;
        }
        setMeasureMode(false);
        break;
    default:
        QWidget::keyPressEvent(event);
        return;
    }
    event->accept();
}

void ParagliderView::leaveEvent(QEvent *event)
{
    if (impl_->hoveredPart != -1) {
        impl_->hoveredPart = -1;
        impl_->context->ClearDetected(false);
        redraw();
        if (partHovered) {
            partHovered(-1);
        }
    }
    QWidget::leaveEvent(event);
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
        if (impl_->measureMode) {
            setCursor(Qt::CrossCursor);
        } else {
            unsetCursor();
        }
    } else if (dragButton_ == Qt::LeftButton && !shiftPan_) {
        setCursor(Qt::ClosedHandCursor);
    } else {
        setCursor(Qt::SizeAllCursor);
    }
}

void ParagliderView::updateHover(const QPoint &position)
{
    if (!hasModel() || impl_->view->Window().IsNull()) {
        return;
    }
    const QPoint native = nativePixel(position);
    impl_->context->MoveTo(
        native.x(),
        native.y(),
        impl_->view,
        true);
    int detected = -1;
    if (impl_->context->HasDetected()) {
        detected = impl_->partIdOf(impl_->context->DetectedInteractive());
    }
    if (detected != impl_->hoveredPart) {
        impl_->hoveredPart = detected;
        if (partHovered) {
            partHovered(detected);
        }
    }
}

void ParagliderView::pickAt(const QPoint &position, bool retargetCamera)
{
    if (!hasModel() || impl_->view->Window().IsNull()) {
        return;
    }
    const QPoint native = nativePixel(position);
    impl_->context->MoveTo(
        native.x(),
        native.y(),
        impl_->view,
        false);

    if (impl_->measureMode) {
        if (impl_->context->MainSelector()->NbPicked() > 0) {
            const QString status = impl_->addMeasurePoint(
                impl_->context->MainSelector()->PickedPoint(1));
            qCDebug(lepViewport) << "measure point at" << native << status;
            redraw();
            if (measurementChanged) {
                measurementChanged(status);
            }
        }
        return;
    }

    if (retargetCamera) {
        // Make the picked model point the camera target (and the arcball
        // pivot) while preserving the viewing direction, gliding there.
        if (impl_->context->MainSelector()->NbPicked() > 0) {
            const gp_Pnt picked =
                impl_->context->MainSelector()->PickedPoint(1);
            runCameraOperation(
                [this, picked] {
                    const occ::handle<Graphic3d_Camera> camera =
                        impl_->view->Camera();
                    const gp_Dir direction = camera->Direction();
                    double distance =
                        gp_Vec(camera->Eye(), picked)
                            .Dot(gp_Vec(direction));
                    if (distance <= impl_->diagonalMillimetres * 1.0e-4) {
                        distance = camera->Distance();
                    }
                    camera->SetCenter(picked);
                    camera->SetEye(
                        gp_Pnt(
                            picked.XYZ()
                            - direction.XYZ().Multiplied(distance)));
                    impl_->view->ZFitAll();
                },
                true);
            qCDebug(lepViewport)
                << "retarget at" << native
                << "picked point" << picked.X() << picked.Y() << picked.Z();
        } else {
            qCDebug(lepViewport)
                << "retarget at" << native << "hit nothing";
        }
        return;
    }

    int picked = -1;
    if (impl_->context->HasDetected()) {
        picked = impl_->partIdOf(impl_->context->DetectedInteractive());
    }
    qCDebug(lepViewport)
        << "click pick at" << native << "part" << picked
        << (picked >= 0 ? partPath(picked) : QStringLiteral("(none)"));
    if (picked >= 0) {
        impl_->context->SetSelected(
            impl_->parts.at(picked).object,
            false);
    } else {
        impl_->context->ClearSelected(false);
    }
    redraw();
    if (partPicked) {
        partPicked(picked);
    }
}

QPoint ParagliderView::nativePixel(const QPoint &logicalPosition) const
{
    // Qt reports positions in logical (DPI-scaled) pixels while the OCCT
    // view window measures physical pixels; on a scaled Windows desktop the
    // two differ and picking would land beside the cursor.
    const qreal scale = devicePixelRatioF();
    return {
        qRound(logicalPosition.x() * scale),
        qRound(logicalPosition.y() * scale),
    };
}
