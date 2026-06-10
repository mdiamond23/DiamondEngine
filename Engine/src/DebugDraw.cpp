#include "DebugDraw.h"
#include <glad/gl.h>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>

namespace DebugDraw {
namespace {

// ---------------------------------------------------------------------------
// GPU state
// ---------------------------------------------------------------------------
struct Vert { glm::vec3 pos, color; };

static GLuint            s_VAO  = 0;
static GLuint            s_VBO  = 0;
static GLuint            s_Prog = 0;
static std::vector<Vert> s_Verts;

static const char* kVS = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uVP;
out vec3 vColor;
void main() {
    vColor      = aColor;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)glsl";

static const char* kFS = R"glsl(
#version 330 core
in  vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)glsl";

static void EnsureInit() {
    if (s_VAO) return;

    auto compile = [](GLenum type, const char* src) {
        GLuint sh = glCreateShader(type);
        glShaderSource(sh, 1, &src, nullptr);
        glCompileShader(sh);
        return sh;
    };
    GLuint vs = compile(GL_VERTEX_SHADER,   kVS);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFS);
    s_Prog = glCreateProgram();
    glAttachShader(s_Prog, vs);
    glAttachShader(s_Prog, fs);
    glLinkProgram(s_Prog);
    glDeleteShader(vs);
    glDeleteShader(fs);

    glGenVertexArrays(1, &s_VAO);
    glGenBuffers(1, &s_VBO);
    glBindVertexArray(s_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vert),
                          (void*)offsetof(Vert, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vert),
                          (void*)offsetof(Vert, color));
    glBindVertexArray(0);
}

static constexpr float kPi = 3.14159265358979f;

// One full circle perpendicular to `normal`, centred at `c`.
static void AddCircle(glm::vec3 c, glm::vec3 n, float r, glm::vec3 col, int segs = 32) {
    glm::vec3 up   = (fabsf(n.y) < 0.99f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
    glm::vec3 tang = glm::normalize(glm::cross(n, up));
    glm::vec3 btan = glm::cross(n, tang);
    for (int i = 0; i < segs; ++i) {
        float a0 = i       * 2.f * kPi / segs;
        float a1 = (i + 1) * 2.f * kPi / segs;
        s_Verts.push_back({ c + r * (cosf(a0) * tang + sinf(a0) * btan), col });
        s_Verts.push_back({ c + r * (cosf(a1) * tang + sinf(a1) * btan), col });
    }
}

// Arc from aStart to aEnd in the plane spanned by ax1 and ax2, centred at c.
static void AddArc(glm::vec3 c, glm::vec3 ax1, glm::vec3 ax2, float r,
                   float aStart, float aEnd, glm::vec3 col, int segs = 16) {
    for (int i = 0; i < segs; ++i) {
        float a0 = aStart + (aEnd - aStart) * i       / segs;
        float a1 = aStart + (aEnd - aStart) * (i + 1) / segs;
        s_Verts.push_back({ c + r * (cosf(a0) * ax1 + sinf(a0) * ax2), col });
        s_Verts.push_back({ c + r * (cosf(a1) * ax1 + sinf(a1) * ax2), col });
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void Line(glm::vec3 a, glm::vec3 b, glm::vec3 color) {
    s_Verts.push_back({a, color});
    s_Verts.push_back({b, color});
}

void Box(glm::vec3 center, glm::quat rot, glm::vec3 he, glm::vec3 color) {
    auto w = [&](glm::vec3 local) { return center + rot * local; };

    // 8 corners (winding: front face = z-, back face = z+)
    glm::vec3 c[8] = {
        w({-he.x, -he.y, -he.z}), w({+he.x, -he.y, -he.z}),
        w({+he.x, +he.y, -he.z}), w({-he.x, +he.y, -he.z}),
        w({-he.x, -he.y, +he.z}), w({+he.x, -he.y, +he.z}),
        w({+he.x, +he.y, +he.z}), w({-he.x, +he.y, +he.z}),
    };

    // Front and back face quads
    for (int f = 0; f < 2; ++f) {
        int base = f * 4;
        Line(c[base+0], c[base+1], color);
        Line(c[base+1], c[base+2], color);
        Line(c[base+2], c[base+3], color);
        Line(c[base+3], c[base+0], color);
    }
    // Connecting edges
    for (int i = 0; i < 4; ++i) Line(c[i], c[i + 4], color);
}

void Sphere(glm::vec3 center, float radius, glm::vec3 color) {
    AddCircle(center, {1, 0, 0}, radius, color);
    AddCircle(center, {0, 1, 0}, radius, color);
    AddCircle(center, {0, 0, 1}, radius, color);
}

void Capsule(glm::vec3 center, glm::quat rot, float halfHeight, float radius, glm::vec3 color) {
    glm::vec3 up    = rot * glm::vec3(0, 1, 0);
    glm::vec3 right = rot * glm::vec3(1, 0, 0);
    glm::vec3 fwd   = rot * glm::vec3(0, 0, 1);
    glm::vec3 top    = center + up * halfHeight;
    glm::vec3 bottom = center - up * halfHeight;

    // Cylinder body
    AddCircle(top,    up, radius, color);
    AddCircle(bottom, up, radius, color);
    for (const glm::vec3& side : { right, -right, fwd, -fwd })
        Line(top + side * radius, bottom + side * radius, color);

    // Hemisphere caps — two arcs per cap (XY and ZY planes of the capsule)
    AddArc(top,    right, up,  radius, 0.f, kPi, color); // top cap, right/up plane
    AddArc(top,    fwd,   up,  radius, 0.f, kPi, color); // top cap, fwd/up plane
    AddArc(bottom, right, -up, radius, 0.f, kPi, color); // bottom cap
    AddArc(bottom, fwd,   -up, radius, 0.f, kPi, color);
}

void Flush(const glm::mat4& vp) {
    if (s_Verts.empty()) return;
    EnsureInit();

    GLboolean depthMask; glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    glDepthMask(GL_FALSE); // draw on top without polluting depth buffer

    glUseProgram(s_Prog);
    glUniformMatrix4fv(glGetUniformLocation(s_Prog, "uVP"), 1, GL_FALSE,
                       glm::value_ptr(vp));

    glBindVertexArray(s_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(s_Verts.size() * sizeof(Vert)),
                 s_Verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, (GLsizei)s_Verts.size());

    glBindVertexArray(0);
    glUseProgram(0);
    glDepthMask(depthMask);

    s_Verts.clear();
}

} // namespace DebugDraw
