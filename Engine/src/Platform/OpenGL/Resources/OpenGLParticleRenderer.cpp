#include "Platform/OpenGL/Resources/OpenGLParticleRenderer.h"
#include "Renderer/TextureData.h"
#include "Profiling/GLRendererStats.h"

#include <glad/gl.h>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

namespace Diamond {

namespace {

constexpr const char* kVS = R"glsl(
#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
uniform mat4 uViewProj;
out vec2 vUV;
out vec4 vColor;
void main() {
    vUV    = aUV;
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)glsl";

constexpr const char* kFS = R"glsl(
#version 410 core
in  vec2 vUV;
in  vec4 vColor;
out vec4 FragColor;
uniform sampler2D uTex;
void main() {
    FragColor = vColor * texture(uTex, vUV);
}
)glsl";

uint32_t CompileProgram()
{
    auto compile = [](GLenum type, const char* src, const char* label) -> uint32_t {
        uint32_t sh = glCreateShader(type);
        glShaderSource(sh, 1, &src, nullptr);
        glCompileShader(sh);
        int ok = 0;
        glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
            spdlog::error("OpenGLParticleRenderer: {} shader compile error: {}", label, log);
        }
        return sh;
    };

    uint32_t vs = compile(GL_VERTEX_SHADER,   kVS, "vertex");
    uint32_t fs = compile(GL_FRAGMENT_SHADER, kFS, "fragment");
    uint32_t prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    int linked = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &linked);
    if (!linked) {
        char log[512];
        glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        spdlog::error("OpenGLParticleRenderer: program link error: {}", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

} // namespace

OpenGLParticleRenderer::OpenGLParticleRenderer()
{
    m_Program = CompileProgram();

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));

    glBindVertexArray(0);
}

OpenGLParticleRenderer::~OpenGLParticleRenderer()
{
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
    glDeleteProgram(m_Program);
}

void OpenGLParticleRenderer::Begin(const glm::mat4& view, const glm::mat4& proj)
{
    m_ViewProj = proj * view;

    // World-space camera basis = rows of the view rotation. In glm's column-major
    // storage that is view[col][row], so row 0 (right) and row 1 (up) read across.
    m_CamRight = glm::vec3(view[0][0], view[1][0], view[2][0]);
    m_CamUp    = glm::vec3(view[0][1], view[1][1], view[2][1]);

    // Save the state we touch so the pass leaves the pipeline as it found it.
    m_SavedDepthTest = glIsEnabled(GL_DEPTH_TEST);
    m_SavedBlend     = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_DEPTH_WRITEMASK,   &m_SavedDepthMask);
    glGetIntegerv(GL_BLEND_SRC_ALPHA,   &m_SavedBlendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA,   &m_SavedBlendDst);

    // Depth-test against the scene, but don't write depth: transparent particles
    // must not occlude each other, and the back-to-front/additive blend handles
    // their ordering.
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
}

void OpenGLParticleRenderer::Draw(const std::vector<RenderParticle>& particles,
                                  const Texture& texture, ParticleBlend blend)
{
    if (particles.empty()) return;

    if (blend == ParticleBlend::Additive)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    else
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Expand each particle into two camera-facing triangles on the CPU.
    m_Verts.clear();
    m_Verts.reserve(particles.size() * 6);
    for (const RenderParticle& p : particles) {
        const float c = std::cos(p.rotation);
        const float s = std::sin(p.rotation);
        // Spin the camera basis in the view plane for screen-facing rotation.
        const glm::vec3 r = (m_CamRight * c + m_CamUp * s) * (p.size * 0.5f);
        const glm::vec3 u = (m_CamUp * c - m_CamRight * s) * (p.size * 0.5f);

        const glm::vec3 bl = p.position - r - u;
        const glm::vec3 br = p.position + r - u;
        const glm::vec3 tr = p.position + r + u;
        const glm::vec3 tl = p.position - r + u;

        // UVs: top-left origin (matches the engine's texture convention).
        m_Verts.push_back({ bl, {0.0f, 1.0f}, p.color });
        m_Verts.push_back({ br, {1.0f, 1.0f}, p.color });
        m_Verts.push_back({ tr, {1.0f, 0.0f}, p.color });
        m_Verts.push_back({ bl, {0.0f, 1.0f}, p.color });
        m_Verts.push_back({ tr, {1.0f, 0.0f}, p.color });
        m_Verts.push_back({ tl, {0.0f, 0.0f}, p.color });
    }

    glUseProgram(m_Program);
    glUniformMatrix4fv(glGetUniformLocation(m_Program, "uViewProj"), 1, GL_FALSE,
                       glm::value_ptr(m_ViewProj));
    glUniform1i(glGetUniformLocation(m_Program, "uTex"), 0);
    texture.Bind(0);

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(m_Verts.size() * sizeof(Vertex)),
                 m_Verts.data(), GL_DYNAMIC_DRAW);
    GLStats::RecordBufferUpload();
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(m_Verts.size()));
    GLStats::RecordDraw(m_Verts.size() / 3);

    glBindVertexArray(0);
    glUseProgram(0);
}

void OpenGLParticleRenderer::End()
{
    if (m_SavedDepthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (m_SavedBlend)     glEnable(GL_BLEND);      else glDisable(GL_BLEND);
    glDepthMask(m_SavedDepthMask ? GL_TRUE : GL_FALSE);
    glBlendFunc(m_SavedBlendSrc, m_SavedBlendDst);
}

} // namespace Diamond
