#include "softwing_gpu.h"

#ifdef __APPLE__

// macOS caps OpenGL at 4.1: compute shaders and shader storage buffers
// do not exist there, and the 4.3 headers this backend needs stopped
// compiling on newer SDK/Qt combinations. The backend cannot RUN on the
// platform either way, so the honest macOS build is a stub whose
// initialize() says why — the bench itself still builds and every CPU
// mode works.
namespace lep::playground {

struct GpuSoftBody::Impl
{
};

GpuSoftBody::GpuSoftBody() = default;
GpuSoftBody::~GpuSoftBody() = default;

bool GpuSoftBody::initialize(const SimBody &, GpuSolveMode, QString &error)
{
    error = QStringLiteral(
        "the GPU backend needs OpenGL 4.3 compute shaders, and macOS "
        "caps OpenGL at 4.1");
    return false;
}

QString GpuSoftBody::rendererDescription() const
{
    return {};
}

std::size_t GpuSoftBody::colourCount() const
{
    return 0;
}

std::size_t GpuSoftBody::dispatchesPerFrame(const SimControls &) const
{
    return 0;
}

void GpuSoftBody::step(SimBody &, const SimControls &)
{
}

void GpuSoftBody::readback(SimBody &)
{
}

double GpuSoftBody::maximumDeviation(const SimBody &) const
{
    return 0.0;
}

}  // namespace lep::playground

#else

#include <softwing/parallel.h>

#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLVersionFunctionsFactory>
#include <QSurfaceFormat>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lep::playground {
namespace {

constexpr int kWorkgroupSize = 256;

const char *const kCommonPrologue = R"(#version 430 core
layout(local_size_x = 256) in;

struct Constraint {
    uint  a;
    uint  b;
    float restLength;
    float compliance;
    float lambda;
    uint  kind;          // 0 distance, 1 cable
    uint  pad0;
    uint  pad1;
};

// position.w carries the inverse mass: the constraint sweep wants exactly
// those two things per node, and packing them puts a whole node in one
// 16-byte fetch.
layout(std430, binding = 0) buffer Positions   { vec4 position[]; };
layout(std430, binding = 1) buffer Previous    { vec4 previous[]; };
layout(std430, binding = 2) buffer Velocities  { vec4 velocity[]; };
layout(std430, binding = 3) buffer Forces      { vec4 force[]; };
layout(std430, binding = 4) buffer Constraints { Constraint constraints[]; };
layout(std430, binding = 5) buffer Triangles   { uvec4 triangles[]; };
layout(std430, binding = 6) buffer FacePress   { float facePressure[]; };
layout(std430, binding = 7) buffer NodeTriOff  { uint nodeTriangleOffset[]; };
layout(std430, binding = 8) buffer NodeTriIdx  { uint nodeTriangleIndex[]; };
layout(std430, binding = 9) buffer AnchorNode  { uint anchorNode[]; };
layout(std430, binding = 10) buffer AnchorPos  { vec4 anchorPosition[]; };
// Jacobi only: proposed correction per constraint endpoint (2 per
// constraint), and the node -> incident endpoint list that reduces them.
layout(std430, binding = 11) buffer Deltas     { vec4 constraintDelta[]; };
layout(std430, binding = 12) buffer NodeConOff { uint nodeConstraintOffset[]; };
layout(std430, binding = 13) buffer NodeConIdx { uint nodeConstraintSlot[]; };
)";

// Nodal pressure force. The CPU walks triangles and scatters a third of each
// face's load to its corners; a GPU cannot do that without atomics, so this
// gathers instead — each node sums over its incident faces from a CSR list.
// The face area vector is recomputed once per corner rather than once per
// face, which is three times the arithmetic and none of the contention.
const char *const kPressureSource = R"(
uniform uint uNodeCount;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uNodeCount) return;
    vec3 total = vec3(0.0);
    uint begin = nodeTriangleOffset[i];
    uint end   = nodeTriangleOffset[i + 1u];
    for (uint slot = begin; slot < end; ++slot) {
        uint t = nodeTriangleIndex[slot];
        uvec4 tri = triangles[t];
        vec3 a = position[tri.x].xyz;
        vec3 b = position[tri.y].xyz;
        vec3 c = position[tri.z].xyz;
        vec3 areaVector = 0.5 * cross(b - a, c - a);
        total += facePressure[t] * areaVector / 3.0;
    }
    force[i].xyz = total;
}
)";

const char *const kPredictSource = R"(
uniform uint  uNodeCount;
uniform float uDt;
uniform float uDamping;
uniform vec3  uGravity;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uNodeCount) return;
    vec4 p = position[i];
    previous[i].xyz = p.xyz;
    if (p.w == 0.0) {
        velocity[i].xyz = vec3(0.0);
        return;
    }
    vec3 acceleration = uGravity + force[i].xyz * p.w;
    vec3 v = (velocity[i].xyz + acceleration * uDt) * uDamping;
    velocity[i].xyz = v;
    position[i].xyz = p.xyz + v * uDt;
}
)";

const char *const kResetLambdaSource = R"(
uniform uint uConstraintCount;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uConstraintCount) return;
    constraints[i].lambda = 0.0;
}
)";

// One colour of the coloured sweep. Constraints are uploaded colour-major,
// so a colour is the contiguous range [uBase, uBase + uCount) and no two
// invocations in a dispatch touch the same node.
const char *const kSolveSource = R"(
uniform uint  uBase;
uniform uint  uCount;
uniform float uInverseTimeStepSquared;

void main() {
    uint k = gl_GlobalInvocationID.x;
    if (k >= uCount) return;
    uint i = uBase + k;

    uint indexA = constraints[i].a;
    uint indexB = constraints[i].b;
    vec4 pa = position[indexA];
    vec4 pb = position[indexB];
    vec3 difference = pb.xyz - pa.xyz;
    float currentLength = length(difference);
    if (currentLength <= 1.0e-12) return;

    float constraintValue = currentLength - constraints[i].restLength;
    float lambda = constraints[i].lambda;
    bool cable = constraints[i].kind == 1u;
    if (cable && constraintValue <= 0.0 && lambda == 0.0) return;

    float inverseMassSum = pa.w + pb.w;
    float alpha = constraints[i].compliance * uInverseTimeStepSquared;
    if (inverseMassSum + alpha <= 0.0) return;

    float deltaLambda =
        (-constraintValue - alpha * lambda) / (inverseMassSum + alpha);
    float applied = deltaLambda;
    if (cable) {
        float updated = min(0.0, lambda + deltaLambda);
        applied = updated - lambda;
        constraints[i].lambda = updated;
    } else {
        constraints[i].lambda = lambda + deltaLambda;
    }

    float scale = applied / currentLength;
    position[indexA].xyz = pa.xyz - (pa.w * scale) * difference;
    position[indexB].xyz = pb.xyz + (pb.w * scale) * difference;
}
)";

// Jacobi pass one: every constraint proposes a correction for both of its
// nodes from the shared current state. Nothing is written to a position, so
// all constraints run in one dispatch whatever the topology.
const char *const kJacobiDeltaSource = R"(
uniform uint  uConstraintCount;
uniform float uInverseTimeStepSquared;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uConstraintCount) return;
    constraintDelta[2u * i]      = vec4(0.0);
    constraintDelta[2u * i + 1u] = vec4(0.0);

    uint indexA = constraints[i].a;
    uint indexB = constraints[i].b;
    vec4 pa = position[indexA];
    vec4 pb = position[indexB];
    vec3 difference = pb.xyz - pa.xyz;
    float currentLength = length(difference);
    if (currentLength <= 1.0e-12) return;

    float constraintValue = currentLength - constraints[i].restLength;
    float lambda = constraints[i].lambda;
    bool cable = constraints[i].kind == 1u;
    if (cable && constraintValue <= 0.0 && lambda == 0.0) return;

    float inverseMassSum = pa.w + pb.w;
    float alpha = constraints[i].compliance * uInverseTimeStepSquared;
    if (inverseMassSum + alpha <= 0.0) return;

    float deltaLambda =
        (-constraintValue - alpha * lambda) / (inverseMassSum + alpha);
    float applied = deltaLambda;
    if (cable) {
        float updated = min(0.0, lambda + deltaLambda);
        applied = updated - lambda;
        constraints[i].lambda = updated;
    } else {
        constraints[i].lambda = lambda + deltaLambda;
    }

    float scale = applied / currentLength;
    constraintDelta[2u * i]      = vec4(-(pa.w * scale) * difference, 0.0);
    constraintDelta[2u * i + 1u] = vec4( (pb.w * scale) * difference, 0.0);
}
)";

// Jacobi pass two: each node owns its own output and averages the
// corrections of its incident constraints, so no atomics and no ordering.
const char *const kJacobiGatherSource = R"(
uniform uint uNodeCount;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uNodeCount) return;
    uint begin = nodeConstraintOffset[i];
    uint end   = nodeConstraintOffset[i + 1u];
    if (end == begin) return;
    vec3 total = vec3(0.0);
    for (uint slot = begin; slot < end; ++slot) {
        total += constraintDelta[nodeConstraintSlot[slot]].xyz;
    }
    position[i].xyz += total / float(end - begin);
}
)";

const char *const kFinalizeSource = R"(
uniform uint  uNodeCount;
uniform float uInverseDt;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uNodeCount) return;
    vec4 p = position[i];
    velocity[i].xyz =
        p.w == 0.0 ? vec3(0.0) : (p.xyz - previous[i].xyz) * uInverseDt;
}
)";

// Anchors are pinned every frame on the CPU path too: the carabiners hold
// station and the brake handles are hauled down by the sliders.
const char *const kAnchorSource = R"(
uniform uint uAnchorCount;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uAnchorCount) return;
    uint node = anchorNode[i];
    vec3 place = anchorPosition[i].xyz;
    position[node].xyz = place;
    previous[node].xyz = place;
    velocity[node].xyz = vec3(0.0);
}
)";

GLuint dispatchGroups(std::size_t count)
{
    return static_cast<GLuint>((count + kWorkgroupSize - 1) / kWorkgroupSize);
}

}  // namespace

struct GpuSoftBody::Impl
{
    QOffscreenSurface surface;
    QOpenGLContext context;
    QOpenGLFunctions_4_3_Core *gl = nullptr;
    QString renderer;

    GLuint pressure = 0;
    GLuint predict = 0;
    GLuint resetLambda = 0;
    GLuint solve = 0;
    GLuint jacobiDelta = 0;
    GLuint jacobiGather = 0;
    GLuint finalize = 0;
    GLuint anchor = 0;
    GpuSolveMode mode = GpuSolveMode::ColouredGaussSeidel;

    enum Buffer {
        Positions,
        Previous,
        Velocities,
        Forces,
        Constraints,
        TriangleNodes,
        FacePressures,
        NodeTriangleOffsets,
        NodeTriangleIndices,
        AnchorNodes,
        AnchorPositions,
        ConstraintDeltas,
        NodeConstraintOffsets,
        NodeConstraintSlots,
        BufferCount,
    };
    GLuint buffers[BufferCount]{};

    std::size_t nodeCount = 0;
    std::size_t triangleCount = 0;
    std::size_t constraintCount = 0;
    std::size_t anchorCount = 0;
    // Colour-major ranges into the uploaded constraint buffer.
    std::vector<std::size_t> colourOffsets;
    std::vector<float> anchorScratch;
    std::vector<float> facePressureScratch;
    std::vector<float> readbackScratch;

    [[nodiscard]] GLuint compile(const char *body, QString &error) const;
    void bindAll() const;
    void setUint(GLuint program, const char *name, GLuint value) const
    {
        gl->glUniform1ui(gl->glGetUniformLocation(program, name), value);
    }
    void setFloat(GLuint program, const char *name, float value) const
    {
        gl->glUniform1f(gl->glGetUniformLocation(program, name), value);
    }
};

GLuint GpuSoftBody::Impl::compile(const char *body, QString &error) const
{
    const QByteArray source = QByteArray(kCommonPrologue) + body;
    const GLuint shader = gl->glCreateShader(GL_COMPUTE_SHADER);
    const char *const text = source.constData();
    gl->glShaderSource(shader, 1, &text, nullptr);
    gl->glCompileShader(shader);
    GLint status = 0;
    gl->glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        GLint length = 0;
        gl->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        QByteArray log(std::max(length, 1), '\0');
        gl->glGetShaderInfoLog(shader, log.size(), nullptr, log.data());
        error = QStringLiteral("compute shader failed to compile: %1")
                    .arg(QString::fromUtf8(log).trimmed());
        gl->glDeleteShader(shader);
        return 0;
    }
    const GLuint program = gl->glCreateProgram();
    gl->glAttachShader(program, shader);
    gl->glLinkProgram(program);
    gl->glDeleteShader(shader);
    gl->glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        GLint length = 0;
        gl->glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        QByteArray log(std::max(length, 1), '\0');
        gl->glGetProgramInfoLog(program, log.size(), nullptr, log.data());
        error = QStringLiteral("compute program failed to link: %1")
                    .arg(QString::fromUtf8(log).trimmed());
        gl->glDeleteProgram(program);
        return 0;
    }
    return program;
}

void GpuSoftBody::Impl::bindAll() const
{
    for (int index = 0; index < BufferCount; ++index) {
        gl->glBindBufferBase(
            GL_SHADER_STORAGE_BUFFER, index, buffers[index]);
    }
}

GpuSoftBody::GpuSoftBody() : impl_(std::make_unique<Impl>()) {}

GpuSoftBody::~GpuSoftBody()
{
    if (impl_->gl == nullptr) {
        return;
    }
    impl_->context.makeCurrent(&impl_->surface);
    impl_->gl->glDeleteBuffers(Impl::BufferCount, impl_->buffers);
    for (const GLuint program : {impl_->pressure,
                                 impl_->predict,
                                 impl_->resetLambda,
                                 impl_->solve,
                                 impl_->jacobiDelta,
                                 impl_->jacobiGather,
                                 impl_->finalize,
                                 impl_->anchor}) {
        if (program != 0) impl_->gl->glDeleteProgram(program);
    }
    impl_->context.doneCurrent();
}

bool GpuSoftBody::initialize(const SimBody &sim,
                             GpuSolveMode mode,
                             QString &error)
{
    Impl &self = *impl_;
    self.mode = mode;

    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(4, 3);
    self.surface.setFormat(format);
    self.surface.create();
    self.context.setFormat(format);
    if (!self.context.create()) {
        error = QStringLiteral("could not create an OpenGL 4.3 core context");
        return false;
    }
    if (!self.context.makeCurrent(&self.surface)) {
        error = QStringLiteral("could not make the offscreen context current");
        return false;
    }
    self.gl = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_4_3_Core>(
        &self.context);
    if (self.gl == nullptr || !self.gl->initializeOpenGLFunctions()) {
        error = QStringLiteral(
            "OpenGL 4.3 core functions are unavailable (compute shaders "
            "need 4.3; this context reports %1.%2)")
                    .arg(self.context.format().majorVersion())
                    .arg(self.context.format().minorVersion());
        return false;
    }
    self.renderer =
        QString::fromUtf8(reinterpret_cast<const char *>(
                              self.gl->glGetString(GL_RENDERER)))
        + QStringLiteral(" / ")
        + QString::fromUtf8(reinterpret_cast<const char *>(
              self.gl->glGetString(GL_VERSION)));

    struct {
        GLuint *slot;
        const char *source;
    } const programs[]{
        {&self.pressure, kPressureSource},
        {&self.predict, kPredictSource},
        {&self.resetLambda, kResetLambdaSource},
        {&self.solve, kSolveSource},
        {&self.jacobiDelta, kJacobiDeltaSource},
        {&self.jacobiGather, kJacobiGatherSource},
        {&self.finalize, kFinalizeSource},
        {&self.anchor, kAnchorSource},
    };
    for (const auto &entry : programs) {
        *entry.slot = self.compile(entry.source, error);
        if (*entry.slot == 0) {
            return false;
        }
    }

    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();
    const auto &constraints = sim.body->constraints();
    self.nodeCount = nodes.size();
    self.triangleCount = triangles.size();
    self.constraintCount = constraints.size();
    // Nothing is pinned any more -- the wing and its pilot both fly -- so
    // the anchor buffers stay empty and the kernel never runs. The prologue
    // still declares them so every kernel shares one binding table.
    self.anchorCount = 0;

    std::vector<float> positions(4 * self.nodeCount, 0.0F);
    for (std::size_t index = 0; index < self.nodeCount; ++index) {
        positions[4 * index + 0] =
            static_cast<float>(nodes[index].position.x);
        positions[4 * index + 1] =
            static_cast<float>(nodes[index].position.y);
        positions[4 * index + 2] =
            static_cast<float>(nodes[index].position.z);
        positions[4 * index + 3] =
            static_cast<float>(nodes[index].inverseMass);
    }

    // Top faces are flagged in the triangle's spare component rather than in
    // a buffer of their own; the stamp kernel is the only reader.
    std::vector<std::uint32_t> triangleNodes(4 * self.triangleCount, 0U);
    for (std::size_t index = 0; index < self.triangleCount; ++index) {
        triangleNodes[4 * index + 0] =
            static_cast<std::uint32_t>(triangles[index].a);
        triangleNodes[4 * index + 1] =
            static_cast<std::uint32_t>(triangles[index].b);
        triangleNodes[4 * index + 2] =
            static_cast<std::uint32_t>(triangles[index].c);
    }
    for (const std::size_t face : sim.topFaces) {
        triangleNodes[4 * face + 3] = 1U;
    }

    // Node -> incident triangles, as CSR.
    std::vector<std::uint32_t> offsets(self.nodeCount + 1, 0U);
    for (const softwing::Triangle &triangle : triangles) {
        ++offsets[triangle.a + 1];
        ++offsets[triangle.b + 1];
        ++offsets[triangle.c + 1];
    }
    for (std::size_t index = 0; index < self.nodeCount; ++index) {
        offsets[index + 1] += offsets[index];
    }
    std::vector<std::uint32_t> incidence(3 * self.triangleCount, 0U);
    std::vector<std::uint32_t> cursor(offsets.begin(), offsets.end() - 1);
    for (std::size_t index = 0; index < self.triangleCount; ++index) {
        const softwing::Triangle &triangle = triangles[index];
        for (const std::size_t node : {triangle.a, triangle.b, triangle.c}) {
            incidence[cursor[node]++] = static_cast<std::uint32_t>(index);
        }
    }

    // The core's own colouring, uploaded colour-major so a colour is a
    // contiguous dispatch range.
    struct GpuConstraint
    {
        std::uint32_t a;
        std::uint32_t b;
        float restLength;
        float compliance;
        float lambda;
        std::uint32_t kind;
        std::uint32_t pad0;
        std::uint32_t pad1;
    };
    const auto colouring = sim.body->constraintColouringView();
    std::vector<GpuConstraint> ordered;
    ordered.reserve(self.constraintCount);
    for (const std::size_t index : colouring.order) {
        const softwing::DistanceConstraint &constraint = constraints[index];
        ordered.push_back(
            {static_cast<std::uint32_t>(constraint.a),
             static_cast<std::uint32_t>(constraint.b),
             static_cast<float>(constraint.restLength),
             static_cast<float>(constraint.compliance),
             0.0F,
             constraint.kind == softwing::ConstraintKind::Cable ? 1U : 0U,
             0U,
             0U});
    }
    self.colourOffsets.assign(colouring.colourOffsets.begin(),
                              colouring.colourOffsets.end());

    // Node -> incident constraint endpoints, as CSR over the *reordered*
    // constraint array, for the Jacobi reduction. A slot is 2i or 2i+1: the
    // delta this constraint proposes for its first or second node.
    std::vector<std::uint32_t> constraintOffsets(self.nodeCount + 1, 0U);
    for (const GpuConstraint &constraint : ordered) {
        ++constraintOffsets[constraint.a + 1];
        ++constraintOffsets[constraint.b + 1];
    }
    for (std::size_t index = 0; index < self.nodeCount; ++index) {
        constraintOffsets[index + 1] += constraintOffsets[index];
    }
    std::vector<std::uint32_t> constraintSlots(2 * ordered.size(), 0U);
    std::vector<std::uint32_t> constraintCursor(constraintOffsets.begin(),
                                                constraintOffsets.end() - 1);
    for (std::size_t index = 0; index < ordered.size(); ++index) {
        constraintSlots[constraintCursor[ordered[index].a]++] =
            static_cast<std::uint32_t>(2 * index);
        constraintSlots[constraintCursor[ordered[index].b]++] =
            static_cast<std::uint32_t>(2 * index + 1);
    }

    const std::vector<std::uint32_t> anchorNodes(1, 0U);
    self.anchorScratch.assign(4, 0.0F);

    self.gl->glGenBuffers(Impl::BufferCount, self.buffers);
    const auto upload = [&](Impl::Buffer slot,
                            const void *data,
                            std::size_t bytes) {
        self.gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.buffers[slot]);
        self.gl->glBufferData(GL_SHADER_STORAGE_BUFFER,
                              static_cast<GLsizeiptr>(std::max<std::size_t>(
                                  bytes, sizeof(float) * 4)),
                              data,
                              GL_DYNAMIC_DRAW);
    };
    const std::size_t nodeBytes = sizeof(float) * 4 * self.nodeCount;
    upload(Impl::Positions, positions.data(), nodeBytes);
    upload(Impl::Previous, positions.data(), nodeBytes);
    std::vector<float> zeroes(4 * self.nodeCount, 0.0F);
    upload(Impl::Velocities, zeroes.data(), nodeBytes);
    upload(Impl::Forces, zeroes.data(), nodeBytes);
    upload(Impl::Constraints,
           ordered.data(),
           sizeof(GpuConstraint) * ordered.size());
    upload(Impl::TriangleNodes,
           triangleNodes.data(),
           sizeof(std::uint32_t) * triangleNodes.size());
    const std::vector<float> facePressures(self.triangleCount, 0.0F);
    upload(Impl::FacePressures,
           facePressures.data(),
           sizeof(float) * facePressures.size());
    upload(Impl::NodeTriangleOffsets,
           offsets.data(),
           sizeof(std::uint32_t) * offsets.size());
    upload(Impl::NodeTriangleIndices,
           incidence.data(),
           sizeof(std::uint32_t) * incidence.size());
    upload(Impl::AnchorNodes,
           anchorNodes.data(),
           sizeof(std::uint32_t) * anchorNodes.size());
    upload(Impl::AnchorPositions,
           self.anchorScratch.data(),
           sizeof(float) * self.anchorScratch.size());
    const std::vector<float> deltaSeed(8 * ordered.size(), 0.0F);
    upload(Impl::ConstraintDeltas,
           deltaSeed.data(),
           sizeof(float) * deltaSeed.size());
    upload(Impl::NodeConstraintOffsets,
           constraintOffsets.data(),
           sizeof(std::uint32_t) * constraintOffsets.size());
    upload(Impl::NodeConstraintSlots,
           constraintSlots.data(),
           sizeof(std::uint32_t) * constraintSlots.size());
    self.bindAll();
    self.readbackScratch.assign(4 * self.nodeCount, 0.0F);
    return true;
}

QString GpuSoftBody::rendererDescription() const
{
    return impl_->renderer;
}

std::size_t GpuSoftBody::colourCount() const
{
    return impl_->colourOffsets.empty() ? 0
                                        : impl_->colourOffsets.size() - 1;
}

std::size_t GpuSoftBody::dispatchesPerFrame(const SimControls &controls) const
{
    const std::size_t perIteration =
        impl_->mode == GpuSolveMode::Jacobi ? 2 : colourCount();
    const std::size_t perSubstep =
        3 + static_cast<std::size_t>(controls.constraintIterations) *
                perIteration;
    return 2 + static_cast<std::size_t>(controls.substeps) * perSubstep;
}

void GpuSoftBody::step(SimBody &sim, const SimControls &controls)
{
    Impl &self = *impl_;
    QOpenGLFunctions_4_3_Core *const gl = self.gl;
    const auto barrier = [gl] {
        gl->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    };


    // The load field is a per-face constant, so rather than mirror the CPU
    // aerodynamic model in a kernel it is computed once on the host and
    // uploaded -- 63 KB a frame, and both backends stay on identical
    // aerodynamics by construction.
    //
    // Caveat for the prototype: applyPressure reads the CPU body's pose, and
    // on this path that pose is only as fresh as the last readback. A real
    // integration would either read back each frame or port the model to a
    // kernel; neither changes what these timings measure, which is the
    // solver.
    applyPressure(sim, controls);
    const auto &triangles = sim.body->triangles();
    self.facePressureScratch.resize(self.triangleCount);
    for (std::size_t face = 0; face < self.triangleCount; ++face) {
        self.facePressureScratch[face] =
            static_cast<float>(triangles[face].pressureDifference);
    }
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER,
                     self.buffers[Impl::FacePressures]);
    gl->glBufferSubData(
        GL_SHADER_STORAGE_BUFFER,
        0,
        static_cast<GLsizeiptr>(sizeof(float) * self.triangleCount),
        self.facePressureScratch.data());

    const double substepTime =
        simulationTimeStep / std::max(1, controls.substeps);
    const float dt = static_cast<float>(substepTime);
    const float damping = static_cast<float>(std::exp(-3.0 * substepTime));
    const float inverseTimeStepSquared =
        static_cast<float>(1.0 / (substepTime * substepTime));
    const GLuint nodeGroups = dispatchGroups(self.nodeCount);

    for (int substep = 0; substep < controls.substeps; ++substep) {
        gl->glUseProgram(self.pressure);
        self.setUint(self.pressure, "uNodeCount",
                     static_cast<GLuint>(self.nodeCount));
        gl->glDispatchCompute(nodeGroups, 1, 1);
        barrier();

        gl->glUseProgram(self.predict);
        self.setUint(self.predict, "uNodeCount",
                     static_cast<GLuint>(self.nodeCount));
        self.setFloat(self.predict, "uDt", dt);
        self.setFloat(self.predict, "uDamping", damping);
        gl->glUniform3f(
            gl->glGetUniformLocation(self.predict, "uGravity"), 0.0F, 0.0F, 0.0F);
        gl->glDispatchCompute(nodeGroups, 1, 1);
        barrier();

        gl->glUseProgram(self.resetLambda);
        self.setUint(self.resetLambda, "uConstraintCount",
                     static_cast<GLuint>(self.constraintCount));
        gl->glDispatchCompute(dispatchGroups(self.constraintCount), 1, 1);
        barrier();

        if (self.mode == GpuSolveMode::Jacobi) {
            const GLuint constraintGroups =
                dispatchGroups(self.constraintCount);
            for (int iteration = 0;
                 iteration < controls.constraintIterations;
                 ++iteration) {
                gl->glUseProgram(self.jacobiDelta);
                self.setUint(self.jacobiDelta, "uConstraintCount",
                             static_cast<GLuint>(self.constraintCount));
                self.setFloat(self.jacobiDelta, "uInverseTimeStepSquared",
                              inverseTimeStepSquared);
                gl->glDispatchCompute(constraintGroups, 1, 1);
                barrier();

                gl->glUseProgram(self.jacobiGather);
                self.setUint(self.jacobiGather, "uNodeCount",
                             static_cast<GLuint>(self.nodeCount));
                gl->glDispatchCompute(nodeGroups, 1, 1);
                barrier();
            }
            continue;
        }

        gl->glUseProgram(self.solve);
        self.setFloat(self.solve, "uInverseTimeStepSquared",
                      inverseTimeStepSquared);
        const GLint baseLocation =
            gl->glGetUniformLocation(self.solve, "uBase");
        const GLint countLocation =
            gl->glGetUniformLocation(self.solve, "uCount");
        for (int iteration = 0; iteration < controls.constraintIterations;
             ++iteration) {
            for (std::size_t colour = 0; colour + 1 < self.colourOffsets.size();
                 ++colour) {
                const std::size_t begin = self.colourOffsets[colour];
                const std::size_t count =
                    self.colourOffsets[colour + 1] - begin;
                if (count == 0) continue;
                gl->glUniform1ui(baseLocation, static_cast<GLuint>(begin));
                gl->glUniform1ui(countLocation, static_cast<GLuint>(count));
                gl->glDispatchCompute(dispatchGroups(count), 1, 1);
                barrier();
            }
        }
    }

    gl->glUseProgram(self.finalize);
    self.setUint(self.finalize, "uNodeCount",
                 static_cast<GLuint>(self.nodeCount));
    self.setFloat(self.finalize, "uInverseDt", 1.0F / dt);
    gl->glDispatchCompute(nodeGroups, 1, 1);
    barrier();

    gl->glFinish();
}

void GpuSoftBody::readback(SimBody &sim)
{
    Impl &self = *impl_;
    self.gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER,
                          self.buffers[Impl::Positions]);
    self.gl->glGetBufferSubData(
        GL_SHADER_STORAGE_BUFFER,
        0,
        static_cast<GLsizeiptr>(sizeof(float) * self.readbackScratch.size()),
        self.readbackScratch.data());
    auto &nodes = sim.body->nodes();
    for (std::size_t index = 0; index < self.nodeCount; ++index) {
        nodes[index].position = {self.readbackScratch[4 * index + 0],
                                 self.readbackScratch[4 * index + 1],
                                 self.readbackScratch[4 * index + 2]};
    }
}

double GpuSoftBody::maximumDeviation(const SimBody &sim) const
{
    Impl &self = *impl_;
    self.gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER,
                          self.buffers[Impl::Positions]);
    self.gl->glGetBufferSubData(
        GL_SHADER_STORAGE_BUFFER,
        0,
        static_cast<GLsizeiptr>(sizeof(float) * self.readbackScratch.size()),
        self.readbackScratch.data());
    const auto &nodes = sim.body->nodes();
    double worst = 0.0;
    for (std::size_t index = 0; index < self.nodeCount; ++index) {
        const softwing::Vec3 difference{
            self.readbackScratch[4 * index + 0] - nodes[index].position.x,
            self.readbackScratch[4 * index + 1] - nodes[index].position.y,
            self.readbackScratch[4 * index + 2] - nodes[index].position.z};
        worst = std::max(worst, length(difference));
    }
    return worst;
}

}  // namespace lep::playground

#endif  // __APPLE__
