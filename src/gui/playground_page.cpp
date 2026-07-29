#include "playground_page.h"

#include "playground_sim.h"

#include "softwing/soft_body.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

// The wing itself — mesh parsing, refinement, body assembly and the step —
// lives in playground_sim.{h,cpp} so the headless solver benchmark can drive
// exactly the same simulation. What is left here is the view: camera,
// shading, colour ramps and the control panel.
using lep::playground::LineSegment;
using lep::playground::RenderFace;
using lep::playground::SimBody;
using lep::playground::SimBuildOptions;
using lep::playground::SimControls;
using lep::playground::SimMesh;
using lep::playground::SimSurface;
using lep::playground::buildSimBody;
using lep::playground::defaultRibLayers;
using lep::playground::defaultRibStationSplit;
using lep::playground::noConstraint;
using lep::playground::parseSimMesh;
using lep::playground::refineSimMesh;
using lep::playground::simSurfaceCount;

namespace {

constexpr float cameraFieldOfViewDegrees = 40.0F;
constexpr float degreesToRadians = 3.14159265358979323846F / 180.0F;
constexpr double maximumBrakeTravelMetres = 0.6;
// Stretch that saturates the stress ramp, adjustable so slack fabric and
// hard-loaded seams can each be examined at a useful contrast. The legend
// reports the live peak next to it.
constexpr double defaultStressFullScaleStrain = 0.01;   // 1% of rest length
constexpr double maximumStressFullScaleStrain = 0.05;   // slider top, 5%
// Suspension lines are nearly inextensible (see lineCompliance), so stretch
// tells you nothing about them: their load is read from the solver's
// multiplier instead, which is a force. Newtons per line.
constexpr double defaultLineFullScaleNewtons = 100.0;
constexpr double maximumLineFullScaleNewtons = 500.0;
// Grey for faces that are drawn while stress colouring is on but carry no
// meaningful stress of their own — the simple rib web.
const QVector3D uncolouredTint(0.58F, 0.60F, 0.63F);
} // namespace

// A minimal orbit-camera OpenGL view running the XPBD body on a timer.
// No signals or slots of its own, so no Q_OBJECT / moc involvement.
class PlaygroundView : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit PlaygroundView(QWidget *parent = nullptr)
        : QOpenGLWidget(parent)
    {
        // Render into an own native window instead of joining Qt's
        // whole-window GL composition: the main window mixes raster
        // widgets with the OCCT viewport's native swapchain, and letting
        // this widget flip the window into GL composition both blacked
        // out the other GL tabs and left a ghost copy of this view at its
        // pre-layout geometry.
        setAttribute(Qt::WA_NativeWindow, true);
        controls_.workerThreads = lep::playground::playgroundWorkerThreads();
        timer_ = new QTimer(this);
        timer_->setInterval(16);
        connect(timer_, &QTimer::timeout, this, [this] {
            stepSimulation();
            update();
        });
    }

    QString buildFromMesh(const SimMesh &mesh)
    {
        sim_ = buildSimBody(mesh, buildOptions_, controls_);

        const softwing::Vec3 low = sim_.boundsLow;
        const softwing::Vec3 high = sim_.boundsHigh;
        target_ = QVector3D(static_cast<float>((low.x + high.x) / 2),
                            static_cast<float>((low.y + high.y) / 2),
                            static_cast<float>((low.z + high.z) / 2));
        distance_ = static_cast<float>(2.0 * length(high - low));

        setRunning(true);
        return {};
    }

    void setPressurePascal(double pressure)
    {
        controls_.pressurePascal = pressure;
        applyPressure();
    }

    // Degrees between the airflow and the wing's rest chord. The load falls
    // out of the pressure field now, so this is the only handle the
    // aerodynamics needs — it replaces a slider that dialled in a fake force.
    void setAngleOfAttack(double degrees)
    {
        controls_.angleOfAttackDegrees = degrees;
        applyPressure();
    }

    void setBrakePull(double leftMetres, double rightMetres)
    {
        controls_.brakeLeft = leftMetres;
        controls_.brakeRight = rightMetres;
    }

    void setSurfaceVisible(SimSurface surface, bool visible)
    {
        surfaceVisible_[static_cast<std::size_t>(surface)] = visible;
        update();
    }

    void setLinesVisible(bool visible)
    {
        linesVisible_ = visible;
        update();
    }

    // Index into lep::playground::solverQualities. Applies from the next
    // frame; the body is untouched, only how hard it is solved.
    void setSolverQuality(int index)
    {
        const auto &table = lep::playground::solverQualities;
        const int count = static_cast<int>(std::size(table));
        const lep::playground::SolverQuality &chosen =
            table[std::clamp(index, 0, count - 1)];
        controls_.substeps = chosen.substeps;
        controls_.constraintIterations = chosen.iterations;
    }

    void setStressColoring(bool enabled)
    {
        stressColoring_ = enabled;
        update();
    }

    // Stretch at which the ramp saturates, as a fraction of rest length.
    void setStressFullScale(double strain)
    {
        stressFullScale_ =
            std::clamp(strain, 1.0e-4, maximumStressFullScaleStrain);
        update();
    }

    // Takes effect on the next build; the page rebuilds when it changes.
    void setDetailedRibs(bool enabled, int layers, int stationSplit)
    {
        buildOptions_.detailedRibs = enabled;
        buildOptions_.ribLayers = std::max(1, layers);
        buildOptions_.ribStationSplit = std::max(1, stationSplit);
    }

    bool detailedRibs() const { return buildOptions_.detailedRibs; }

    void setLineTensionColoring(bool enabled)
    {
        lineTensionColoring_ = enabled;
        update();
    }

    void setLineFullScale(double newtons)
    {
        lineFullScaleNewtons_ =
            std::clamp(newtons, 1.0, maximumLineFullScaleNewtons);
        update();
    }

    // Peak stretch across the wing right now, for the legend.
    // Highest cable load in the wing right now, for the legend.
    double peakLineTension() const
    {
        if (!sim_.body) {
            return 0.0;
        }
        const auto &constraints = sim_.body->constraints();
        double peak = 0.0;
        for (const LineSegment &segment : sim_.lineSegments) {
            peak = std::max(peak, lineTension(segment, constraints));
        }
        return peak;
    }

    double peakStrain() const
    {
        if (!sim_.body) {
            return 0.0;
        }
        const auto &nodes = sim_.body->nodes();
        const auto &constraints = sim_.body->constraints();
        double peak = 0.0;
        for (const RenderFace &face : sim_.renderFaces) {
            if (surfaceVisible_[static_cast<std::size_t>(face.surface)]
                && colourable(face)) {
                peak = std::max(peak, faceStrain(face, nodes, constraints));
            }
        }
        return peak;
    }

    void setRunning(bool running)
    {
        if (running && sim_.body) {
            timer_->start();
        } else {
            timer_->stop();
        }
    }

    bool isRunning() const { return timer_->isActive(); }
    bool hasBody() const { return sim_.body != nullptr; }
    QString lastSimError() const { return simError_; }
    QString lastGlError() const { return glError_; }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.10F, 0.11F, 0.13F, 1.0F);

        // The application's default surface format is XFLR5's, which is a
        // 3.3 core profile on most machines: core GLSL syntax and a bound
        // VAO are mandatory there, while older configurations fall back
        // to a compatibility context and 1.10 syntax.
        const bool coreProfile =
            context()->format().profile() == QSurfaceFormat::CoreProfile;
        const QString vertexSource =
            coreProfile
                ? QStringLiteral(
                      "#version 330 core\n"
                      "in vec3 position;\n"
                      "in vec3 normal;\n"
                      "in vec3 tint;\n"
                      "uniform mat4 mvp;\n"
                      "out vec3 vNormal;\n"
                      "out vec3 vTint;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    vTint = tint;\n"
                      "    gl_Position = mvp * vec4(position, 1.0);\n"
                      "}\n")
                : QStringLiteral(
                      "attribute vec3 position;\n"
                      "attribute vec3 normal;\n"
                      "attribute vec3 tint;\n"
                      "uniform mat4 mvp;\n"
                      "varying vec3 vNormal;\n"
                      "varying vec3 vTint;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    vTint = tint;\n"
                      "    gl_Position = mvp * vec4(position, 1.0);\n"
                      "}\n");
        const QString shading =
            QStringLiteral(
                "    float shade = lit\n"
                "        ? 0.25 + 0.75 * abs(dot(normalize(vNormal),\n"
                "                                normalize(vec3(0.3, -0.5, "
                "0.8))))\n"
                "        : 1.0;\n");
        const QString fragmentSource =
            coreProfile
                ? QStringLiteral(
                      "#version 330 core\n"
                      "uniform vec4 color;\n"
                      "uniform bool lit;\n"
                      "uniform bool useTint;\n"
                      "in vec3 vNormal;\n"
                      "in vec3 vTint;\n"
                      "out vec4 fragColor;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    vec3 base = useTint ? vTint : color.rgb;\n"
                          "    fragColor = vec4(base * shade, color.a);\n"
                          "}\n")
                : QStringLiteral(
                      "uniform vec4 color;\n"
                      "uniform bool lit;\n"
                      "uniform bool useTint;\n"
                      "varying vec3 vNormal;\n"
                      "varying vec3 vTint;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    vec3 base = useTint ? vTint : color.rgb;\n"
                          "    gl_FragColor = vec4(base * shade, color.a);\n"
                          "}\n");

        program_ = new QOpenGLShaderProgram(this);
        if (!program_->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                               vertexSource)
            || !program_->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                                  fragmentSource)
            || !program_->link()) {
            glError_ = QStringLiteral("OpenGL shader error: %1")
                           .arg(program_->log().trimmed());
            return;
        }
        vao_.create();
        buffer_.create();
    }

    void resizeGL(int, int) override {}

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (!sim_.body || !glError_.isEmpty()) {
            return;
        }

        QMatrix4x4 projection;
        projection.perspective(
            cameraFieldOfViewDegrees,
            width() > 0 ? float(width()) / float(std::max(height(), 1))
                        : 1.0F,
            0.02F,
            500.0F);
        QMatrix4x4 view;
        view.translate(0, 0, -distance_);
        view.rotate(pitch_, 1, 0, 0);
        view.rotate(yaw_, 0, 0, 1);
        view.translate(-target_);
        const QMatrix4x4 mvp = projection * view;

        const auto &nodes = sim_.body->nodes();

        vertexScratch_.clear();
        vertexScratch_.reserve(sim_.renderFaces.size() * 27);
        const auto &constraints = sim_.body->constraints();
        for (const RenderFace &face : sim_.renderFaces) {
            if (!surfaceVisible_[static_cast<std::size_t>(face.surface)]) {
                continue;
            }
            const softwing::Vec3 &a = nodes[face.nodes[0]].position;
            const softwing::Vec3 &b = nodes[face.nodes[1]].position;
            const softwing::Vec3 &c = nodes[face.nodes[2]].position;
            const softwing::Vec3 normal = normalized(cross(b - a, c - a));
            // useTint is a per-draw uniform while colourability is per
            // face, so an uncoloured face still needs a colour of its own
            // here — leaving it zeroed painted the simple ribs black.
            QVector3D tint = uncolouredTint;
            if (stressColoring_ && colourable(face)) {
                tint = stressTint(faceStrain(face, nodes, constraints));
            }
            for (const softwing::Vec3 *point : {&a, &b, &c}) {
                vertexScratch_.push_back(static_cast<float>(point->x));
                vertexScratch_.push_back(static_cast<float>(point->y));
                vertexScratch_.push_back(static_cast<float>(point->z));
                vertexScratch_.push_back(static_cast<float>(normal.x));
                vertexScratch_.push_back(static_cast<float>(normal.y));
                vertexScratch_.push_back(static_cast<float>(normal.z));
                vertexScratch_.push_back(tint.x());
                vertexScratch_.push_back(tint.y());
                vertexScratch_.push_back(tint.z());
            }
        }
        const int skinFloats = static_cast<int>(vertexScratch_.size());
        if (linesVisible_) {
            for (const LineSegment &segment : sim_.lineSegments) {
                QVector3D tint;
                if (lineTensionColoring_) {
                    tint = rampTint(
                        lineTension(segment, constraints)
                        / std::max(lineFullScaleNewtons_, 1.0e-6));
                }
                for (const std::size_t node : {segment.a, segment.b}) {
                    const softwing::Vec3 &point = nodes[node].position;
                    vertexScratch_.push_back(static_cast<float>(point.x));
                    vertexScratch_.push_back(static_cast<float>(point.y));
                    vertexScratch_.push_back(static_cast<float>(point.z));
                    vertexScratch_.push_back(segment.brake ? 1.0F : 0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(tint.x());
                    vertexScratch_.push_back(tint.y());
                    vertexScratch_.push_back(tint.z());
                }
            }
        }

        program_->bind();
        QOpenGLVertexArrayObject::Binder vaoBinder(&vao_);
        program_->setUniformValue("mvp", mvp);
        buffer_.bind();
        buffer_.allocate(
            vertexScratch_.data(),
            static_cast<int>(vertexScratch_.size() * sizeof(float)));
        constexpr int stride = 9 * sizeof(float);
        program_->enableAttributeArray("position");
        program_->setAttributeBuffer("position", GL_FLOAT, 0, 3, stride);
        program_->enableAttributeArray("normal");
        program_->setAttributeBuffer(
            "normal", GL_FLOAT, 3 * sizeof(float), 3, stride);
        program_->enableAttributeArray("tint");
        program_->setAttributeBuffer(
            "tint", GL_FLOAT, 6 * sizeof(float), 3, stride);

        program_->setUniformValue("lit", true);
        program_->setUniformValue("useTint", stressColoring_);
        program_->setUniformValue(
            "color", QVector4D(0.72F, 0.78F, 0.88F, 1.0F));
        glDrawArrays(GL_TRIANGLES, 0, skinFloats / 9);

        program_->setUniformValue("lit", false);
        program_->setUniformValue("useTint", lineTensionColoring_);
        program_->setUniformValue(
            "color", QVector4D(0.55F, 0.62F, 0.55F, 1.0F));
        glDrawArrays(GL_LINES,
                     skinFloats / 9,
                     (static_cast<int>(vertexScratch_.size()) - skinFloats)
                         / 9);
        buffer_.release();
        program_->release();
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        lastMouse_ = event->position();
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        const QPointF delta = event->position() - lastMouse_;
        lastMouse_ = event->position();
        if (event->buttons() & Qt::LeftButton) {
            yaw_ += static_cast<float>(delta.x()) * 0.4F;
            pitch_ += static_cast<float>(delta.y()) * 0.4F;
            pitch_ = std::clamp(pitch_, -90.0F, 90.0F);
            update();
        } else if (event->buttons() & Qt::RightButton) {
            // Slide the orbit target across the camera's screen plane. The
            // view matrix is a pure rotation about the target followed by a
            // pull-back, so the camera's world-space right/up axes are the
            // first two rows of that rotation. Scaling by the view height at
            // the target's depth makes the wing track the cursor exactly.
            QMatrix4x4 rotation;
            rotation.rotate(pitch_, 1, 0, 0);
            rotation.rotate(yaw_, 0, 0, 1);
            const QVector3D right = rotation.row(0).toVector3D();
            const QVector3D up = rotation.row(1).toVector3D();
            const float metresPerPixel =
                2.0F * distance_
                * std::tan(cameraFieldOfViewDegrees * 0.5F * degreesToRadians)
                / static_cast<float>(std::max(height(), 1));
            target_ -= right * static_cast<float>(delta.x()) * metresPerPixel;
            target_ += up * static_cast<float>(delta.y()) * metresPerPixel;
            update();
        }
    }

    void wheelEvent(QWheelEvent *event) override
    {
        distance_ *=
            event->angleDelta().y() > 0 ? 1.0F / 1.15F : 1.15F;
        update();
    }

private:
    // The skin is a mass-spring cloth, not a membrane element, so there is
    // no stress tensor to read: the honest per-face measure is how far its
    // three sides are stretched past their rest length. Because every skin
    // edge shares one compliance, tension is proportional to this, so the
    // picture reads the same as a tension plot.
    // Without the detailed model a rib is a hub and spokes, so its colour
    // would be spoke tension dressed up as rib stress. Leave those faces
    // plain rather than show a number that means nothing.
    bool colourable(const RenderFace &face) const
    {
        return face.surface != SimSurface::Rib || buildOptions_.detailedRibs;
    }

    // Tensile only: slack fabric carries no load, so edges shorter than
    // their rest length read as zero rather than as compression.
    static double faceStrain(
        const RenderFace &face,
        const std::vector<softwing::Node> &nodes,
        const std::vector<softwing::DistanceConstraint> &constraints)
    {
        double peak = 0.0;
        for (const std::size_t side : face.edges) {
            if (side == noConstraint || side >= constraints.size()) {
                continue;
            }
            const softwing::DistanceConstraint &constraint =
                constraints[side];
            if (constraint.restLength <= 0.0) {
                continue;
            }
            const double current =
                length(nodes[constraint.b].position
                       - nodes[constraint.a].position);
            peak = std::max(
                peak,
                (current - constraint.restLength) / constraint.restLength);
        }
        return peak;
    }

    // Cable tension in newtons. XPBD's multiplier is an impulse over the
    // substep, so dividing by the substep squared recovers the force; the
    // sign convention makes a tensile multiplier negative, hence the
    // magnitude. A slack cable holds zero, which is what we want to show.
    // The substep length is whatever the solver quality currently asks for,
    // so it is read from the live controls rather than from a constant --
    // otherwise switching quality would silently rescale every line load.
    double lineTension(
        const LineSegment &segment,
        const std::vector<softwing::DistanceConstraint> &constraints) const
    {
        if (segment.constraint == noConstraint
            || segment.constraint >= constraints.size()) {
            return 0.0;
        }
        const double substep =
            lep::playground::simulationTimeStep
            / std::max(1, controls_.substeps);
        return std::abs(constraints[segment.constraint].accumulatedLambda)
               / (substep * substep);
    }

    // Unloaded blue -> teal -> green -> amber -> red at full scale.
    static QVector3D rampTint(double loadFraction)
    {
        static const std::array<QVector3D, 5> ramp{
            QVector3D(0.16F, 0.29F, 0.62F),
            QVector3D(0.16F, 0.60F, 0.62F),
            QVector3D(0.30F, 0.68F, 0.33F),
            QVector3D(0.90F, 0.68F, 0.20F),
            QVector3D(0.83F, 0.24F, 0.20F)};
        const double position =
            std::clamp(loadFraction, 0.0, 1.0) * (ramp.size() - 1);
        const auto stop =
            std::min(static_cast<std::size_t>(position), ramp.size() - 2);
        const float blend =
            static_cast<float>(position - static_cast<double>(stop));
        return ramp[stop] * (1.0F - blend) + ramp[stop + 1] * blend;
    }

    QVector3D stressTint(double strain) const
    {
        return rampTint(strain / std::max(stressFullScale_, 1.0e-6));
    }

    void applyPressure()
    {
        lep::playground::applyPressure(sim_, controls_);
    }

    void stepSimulation()
    {
        try {
            lep::playground::stepSimulation(sim_, controls_);
        } catch (const std::exception &exception) {
            simError_ = QString::fromUtf8(exception.what());
            setRunning(false);
        }
    }

    SimBody sim_;
    SimControls controls_;
    SimBuildOptions buildOptions_;
    std::array<bool, simSurfaceCount> surfaceVisible_{
        true, true, true, true, true};
    bool linesVisible_ = true;
    bool stressColoring_ = false;
    double stressFullScale_ = defaultStressFullScaleStrain;
    bool lineTensionColoring_ = false;
    double lineFullScaleNewtons_ = defaultLineFullScaleNewtons;
    std::vector<float> vertexScratch_;
    QString simError_;
    QString glError_;

    QTimer *timer_ = nullptr;
    QOpenGLShaderProgram *program_ = nullptr;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};
    QVector3D target_;
    float distance_ = 10.0F;
    float yaw_ = 30.0F;
    float pitch_ = -60.0F;
    QPointF lastMouse_;
};

PlaygroundPage::PlaygroundPage(QWidget *parent)
    : QWidget(parent)
{
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        qWarning() << "PlaygroundPage constructed"
                   << static_cast<void *>(this);
    }
    status_ = new QLabel(
        QStringLiteral("Run a preview or export first — the Playground "
                       "loads the calculated wing."),
        this);
    status_->setWordWrap(true);

    const auto makeSlider = [this](int maximum, int value) {
        auto *slider = new QSlider(Qt::Horizontal, this);
        slider->setRange(0, maximum);
        slider->setValue(value);
        return slider;
    };
    // Capped at a third of what the slider used to reach: past roughly this
    // the skin inflates further than the fabric would really allow and the
    // wing stops looking like one, so the extra travel was only misleading.
    pressure_ = makeSlider(100, 80);
    lift_ = makeSlider(15, 6);
    leftBrake_ = makeSlider(100, 0);
    rightBrake_ = makeSlider(100, 0);
    runButton_ = new QPushButton(QStringLiteral("Pause"), this);
    runButton_->setCheckable(true);

    // Display filters: the solver always sees the whole wing, these only
    // decide what is drawn, so hiding the extrados looks inside a wing that
    // is still inflating normally.
    const auto makeCheck = [this](const QString &text, bool checked) {
        auto *check = new QCheckBox(text, this);
        check->setChecked(checked);
        return check;
    };
    showExtrados_ = makeCheck(QStringLiteral("Top"), true);
    showVent_ = makeCheck(QStringLiteral("Vent"), true);
    showIntrados_ = makeCheck(QStringLiteral("Bottom"), true);
    showRibs_ = makeCheck(QStringLiteral("Ribs"), true);
    showStraps_ = makeCheck(QStringLiteral("V/H ribs"), true);
    showLines_ = makeCheck(QStringLiteral("Lines"), true);
    showStress_ = makeCheck(QStringLiteral("Colour by stress"), false);

    // Full-scale stretch for the ramp, in hundredths of a percent so the
    // low end (where fabric actually works) still has resolution.
    stressScale_ = new QSlider(Qt::Horizontal, this);
    stressScale_->setRange(
        10, static_cast<int>(maximumStressFullScaleStrain * 10000.0));
    stressScale_->setValue(
        static_cast<int>(defaultStressFullScaleStrain * 10000.0));
    stressScale_->setMaximumWidth(140);
    stressScale_->setVisible(false);
    showLineTension_ =
        makeCheck(QStringLiteral("Colour lines by tension"), false);
    lineScale_ = new QSlider(Qt::Horizontal, this);
    lineScale_->setRange(
        5, static_cast<int>(maximumLineFullScaleNewtons));
    lineScale_->setValue(static_cast<int>(defaultLineFullScaleNewtons));
    lineScale_->setMaximumWidth(140);
    lineScale_->setVisible(false);

    stressLegend_ = new QLabel(this);
    stressLegend_->setVisible(false);

    // How much solving each frame gets. This is a sandbox, so the default
    // is the setting that keeps a mid-sized wing interactive; the higher
    // ones are there for when the shape matters more than the frame rate.
    // Takes effect on the next frame — no rebuild.
    quality_ = new QComboBox(this);
    for (const lep::playground::SolverQuality &entry :
         lep::playground::solverQualities) {
        quality_->addItem(
            QStringLiteral("%1 (%2x%3)")
                .arg(QString::fromLatin1(entry.label))
                .arg(entry.substeps)
                .arg(entry.iterations));
    }
    quality_->setCurrentIndex(lep::playground::defaultSolverQuality);
    quality_->setToolTip(QStringLiteral(
        "Substeps x iterations per frame. More substeps hold the fabric "
        "closer to its designed length; fewer keep the wing interactive."));

    auto *view = new QHBoxLayout;
    view->addWidget(new QLabel(QStringLiteral("Show"), this));
    view->addWidget(showExtrados_);
    view->addWidget(showVent_);
    view->addWidget(showIntrados_);
    view->addWidget(showRibs_);
    view->addWidget(showStraps_);
    view->addWidget(showLines_);
    view->addSpacing(16);
    view->addWidget(showStress_);
    view->addWidget(stressScale_);
    view->addWidget(showLineTension_);
    view->addWidget(lineScale_);
    view->addWidget(stressLegend_, 1);
    view->addStretch();

    // Its own row. The other two are already full edge to edge — the filter
    // row runs to the legend and the wing row's four stretching sliders will
    // grow until whatever follows them is pushed off the window — so a
    // widget added to either disappears rather than wraps.
    auto *solver = new QHBoxLayout;
    solver->addWidget(new QLabel(QStringLiteral("Solver"), this));
    solver->addWidget(quality_);
    solver->addStretch();

    auto *controls = new QHBoxLayout;
    controls->addWidget(new QLabel(QStringLiteral("Pressure"), this));
    controls->addWidget(pressure_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Angle"), this));
    controls->addWidget(lift_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Left brake"), this));
    controls->addWidget(leftBrake_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Right brake"), this));
    controls->addWidget(rightBrake_, 1);
    controls->addWidget(runButton_);

    layout_ = new QVBoxLayout(this);
    layout_->addWidget(status_);
    layout_->addLayout(view);
    layout_->addLayout(solver);
    layout_->addLayout(controls);

    const auto bindSurface = [this](QCheckBox *check, SimSurface surface) {
        connect(check, &QCheckBox::toggled, this,
                [this, surface](bool visible) {
                    if (view_ != nullptr) {
                        view_->setSurfaceVisible(surface, visible);
                    }
                });
    };
    bindSurface(showExtrados_, SimSurface::Extrados);
    bindSurface(showVent_, SimSurface::Vent);
    bindSurface(showIntrados_, SimSurface::Intrados);
    bindSurface(showRibs_, SimSurface::Rib);
    bindSurface(showStraps_, SimSurface::Strap);
    connect(showLines_, &QCheckBox::toggled, this, [this](bool visible) {
        if (view_ != nullptr) {
            view_->setLinesVisible(visible);
        }
    });
    connect(quality_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (view_ != nullptr) {
            view_->setSolverQuality(index);
        }
    });

    // The legend needs the live peak stretch, which only exists while the
    // solver runs; polling a few times a second keeps it readable instead
    // of flickering with every frame.
    legendTimer_ = new QTimer(this);
    legendTimer_->setInterval(250);
    connect(legendTimer_, &QTimer::timeout, this, [this] {
        if (view_ == nullptr) {
            return;
        }
        QStringList parts;
        if (showStress_->isChecked()) {
            parts << QStringLiteral("fabric slack → %1% stretch (peak %2%)")
                         .arg(stressScale_->value() / 100.0, 0, 'f', 2)
                         .arg(view_->peakStrain() * 100.0, 0, 'f', 2);
        }
        if (showLineTension_->isChecked()) {
            parts << QStringLiteral("lines 0 → %1 N (peak %2 N)")
                         .arg(lineScale_->value())
                         .arg(view_->peakLineTension(), 0, 'f', 1);
        }
        stressLegend_->setText(parts.join(QStringLiteral("  ·  ")));
    });
    connect(stressScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setStressFullScale(value / 10000.0);
        }
    });
    // One legend serves both modes, so it lives as long as either is on.
    const auto refreshLegendTimer = [this] {
        const bool wanted = showStress_->isChecked()
                            || showLineTension_->isChecked();
        stressLegend_->setVisible(wanted);
        if (wanted) {
            legendTimer_->start();
        } else {
            legendTimer_->stop();
        }
    };
    connect(showStress_, &QCheckBox::toggled, this,
            [this, refreshLegendTimer](bool enabled) {
                if (view_ != nullptr) {
                    view_->setStressColoring(enabled);
                }
                stressScale_->setVisible(enabled);
                refreshLegendTimer();
            });
    connect(lineScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setLineFullScale(static_cast<double>(value));
        }
    });
    connect(showLineTension_, &QCheckBox::toggled, this,
            [this, refreshLegendTimer](bool enabled) {
                if (view_ != nullptr) {
                    view_->setLineTensionColoring(enabled);
                }
                lineScale_->setVisible(enabled);
                refreshLegendTimer();
            });

    connect(pressure_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setPressurePascal(static_cast<double>(value));
        }
    });
    connect(lift_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setAngleOfAttack(static_cast<double>(value));
        }
    });
    const auto pushBrakes = [this] {
        if (view_ != nullptr) {
            view_->setBrakePull(
                leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
                rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
        }
    };
    connect(leftBrake_, &QSlider::valueChanged, this, pushBrakes);
    connect(rightBrake_, &QSlider::valueChanged, this, pushBrakes);
    connect(runButton_, &QPushButton::toggled, this, [this](bool paused) {
        if (view_ != nullptr) {
            view_->setRunning(!paused);
        }
        runButton_->setText(paused ? QStringLiteral("Run")
                                   : QStringLiteral("Pause"));
    });
}

void PlaygroundPage::ensureView()
{
    if (view_ != nullptr || creatingView_) {
        return;
    }
    creatingView_ = true;
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        qWarning() << "ensureView creating view on page"
                   << static_cast<void *>(this);
    }
    view_ = new PlaygroundView(this);
    layout_->addWidget(view_, 1);
    view_->setPressurePascal(static_cast<double>(pressure_->value()));
    view_->setAngleOfAttack(static_cast<double>(lift_->value()));
    view_->setBrakePull(
        leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
        rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
    view_->setSurfaceVisible(SimSurface::Extrados,
                             showExtrados_->isChecked());
    view_->setSurfaceVisible(SimSurface::Vent, showVent_->isChecked());
    view_->setSurfaceVisible(SimSurface::Intrados,
                             showIntrados_->isChecked());
    view_->setSurfaceVisible(SimSurface::Rib, showRibs_->isChecked());
    view_->setSurfaceVisible(SimSurface::Strap, showStraps_->isChecked());
    view_->setLinesVisible(showLines_->isChecked());
    view_->setStressFullScale(stressScale_->value() / 10000.0);
    view_->setStressColoring(showStress_->isChecked());
    view_->setLineFullScale(static_cast<double>(lineScale_->value()));
    view_->setLineTensionColoring(showLineTension_->isChecked());
    view_->show();
    creatingView_ = false;
}

void PlaygroundPage::setSimMeshPath(const QString &path)
{
    // Previews run the engine in a temporary directory that is removed
    // right after the model is handed over, so the mesh must be read here
    // and now — by the time the tab is first opened the file is gone.
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        status_->setText(
            QStringLiteral("Could not read the simulation mesh %1")
                .arg(path));
        return;
    }
    pendingData_ = file.readAll();
    meshData_ = pendingData_;
    if (isVisible()) {
        loadIfPending();
    }
}

void PlaygroundPage::setMeshSubdivision(int factor)
{
    const int clamped = std::clamp(factor, 1, maximumMeshSubdivision);
    if (clamped == subdivision_) {
        return;
    }
    subdivision_ = clamped;
    rebuildSimulation();
}

void PlaygroundPage::setDetailedRibs(bool enabled)
{
    if (enabled == detailedRibs_) {
        return;
    }
    detailedRibs_ = enabled;
    rebuildSimulation();
}

void PlaygroundPage::rebuildSimulation()
{
    if (meshData_.isEmpty()) {
        return;
    }
    pendingData_ = meshData_;
    if (view_ != nullptr && isVisible()) {
        loadIfPending();
    }
}

void PlaygroundPage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    ensureView();
    loadIfPending();
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        QTimer::singleShot(4000, this, [this] {
            qWarning() << "PlaygroundPage geometry" << geometry();
            const QList<QWidget *> children =
                findChildren<QWidget *>(Qt::FindDirectChildrenOnly);
            for (QWidget *child : children) {
                qWarning() << " child" << child->metaObject()->className()
                           << child->geometry() << "visible"
                           << child->isVisible();
            }
            qWarning() << " view_" << static_cast<void *>(view_)
                       << (view_ != nullptr ? view_->geometry() : QRect());
        });
    }
}

void PlaygroundPage::loadIfPending()
{
    if (view_ == nullptr || pendingData_.isEmpty()) {
        return;
    }
    QString error;
    const std::optional<SimMesh> mesh = parseSimMesh(pendingData_, error);
    if (!mesh) {
        status_->setText(error);
        pendingData_.clear();
        return;
    }
    // Refining is quadratic in the factor and can take a moment at 4x.
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    // Ribs gain a ring per resolution step so they densify with the skin.
    view_->setDetailedRibs(detailedRibs_,
                           defaultRibLayers + 2 * (subdivision_ - 1),
                           defaultRibStationSplit + subdivision_ - 1);
    const SimMesh simulated = refineSimMesh(*mesh, subdivision_);
    const QString buildError = view_->buildFromMesh(simulated);
    QGuiApplication::restoreOverrideCursor();
    if (!buildError.isEmpty()) {
        status_->setText(buildError);
        pendingData_.clear();
        return;
    }
    pendingData_.clear();
    QString resolution;
    if (subdivision_ > 1) {
        resolution = QStringLiteral(" · %1x resolution")
                         .arg(subdivision_ * subdivision_);
    }
    status_->setText(
        QStringLiteral("Toy simulation · %1 nodes, %2 skin quads, %3 line "
                       "segments%4 · drag to orbit, right-drag to pan, "
                       "wheel to zoom. Not engineering — a sandbox.")
            .arg(simulated.nodes.size())
            .arg(simulated.quads.size())
            .arg(simulated.lines.size())
            .arg(resolution));
    // Shader problems only surface once the first frame renders; a silent
    // black view is undiagnosable, so report them here.
    QTimer::singleShot(500, this, [this] {
        if (!view_->lastGlError().isEmpty()) {
            status_->setText(view_->lastGlError());
        }
    });
}
