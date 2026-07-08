#include "Platform/OpenGL/Resources/OpenGLRenderer2D.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"
#include "Renderer/Font.h"
#include "Profiling/GLRendererStats.h"

#include <glad/gl.h>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

namespace Diamond {

namespace {

constexpr const char* kVS = R"glsl(
#version 410 core
layout(location = 0) in vec2  aPos;
layout(location = 1) in vec2  aUV;
layout(location = 2) in vec4  aColor;
layout(location = 3) in float aMode;
uniform mat4 uProj;
out vec2  vUV;
out vec4  vColor;
out float vMode;
void main() {
    vUV     = aUV;
    vColor  = aColor;
    vMode   = aMode;
    gl_Position = uProj * vec4(aPos, 0.0, 1.0);
}
)glsl";

constexpr const char* kFS = R"glsl(
#version 410 core
in  vec2  vUV;
in  vec4  vColor;
in  float vMode;
out vec4  FragColor;
uniform sampler2D uTex;
void main() {
    vec4 s = texture(uTex, vUV);
    // mode 0: modulate RGBA (white tex -> solid color, sprites -> tinted texel).
    // mode 1: text — the R channel is glyph coverage, used as alpha.
    FragColor = (vMode > 0.5)
        ? vec4(vColor.rgb, vColor.a * s.r)
        : vColor * s;
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
            spdlog::error("OpenGLRenderer2D: {} shader compile error: {}", label, log);
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
        spdlog::error("OpenGLRenderer2D: program link error: {}", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// Native GL handle behind an abstract Texture (OpenGL backend only).
uint32_t NativeID(const Texture& tex)
{
    if (const auto* gl = dynamic_cast<const OpenGLTexture*>(&tex))
        return gl->GetRendererID();
    return 0;
}

} // namespace

OpenGLRenderer2D::OpenGLRenderer2D()
{
    m_Program = CompileProgram();

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, mode));

    glBindVertexArray(0);

    // 1x1 white texture so solid-color quads share the textured path.
    glGenTextures(1, &m_WhiteTex);
    glBindTexture(GL_TEXTURE_2D, m_WhiteTex);
    const uint8_t white[4] = { 255, 255, 255, 255 };
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

OpenGLRenderer2D::~OpenGLRenderer2D()
{
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
    glDeleteTextures(1, &m_WhiteTex);
    glDeleteProgram(m_Program);
}

void OpenGLRenderer2D::Begin(const glm::mat4& projection)
{
    m_Projection = projection;
    m_Verts.clear();
    m_CurrentTex = 0;
    m_InFrame = true;

    // Save state we change so we can leave the pipeline as we found it.
    m_SavedDepth = glIsEnabled(GL_DEPTH_TEST);
    m_SavedBlend = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &m_SavedBlendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &m_SavedBlendDst);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void OpenGLRenderer2D::End()
{
    Flush();

    if (m_SavedDepth) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (m_SavedBlend) glEnable(GL_BLEND);      else glDisable(GL_BLEND);
    glBlendFunc(m_SavedBlendSrc, m_SavedBlendDst);

    m_InFrame = false;
}

void OpenGLRenderer2D::SetTexture(uint32_t texID)
{
    if (texID != m_CurrentTex) {
        Flush();               // draw what we have with the previous texture
        m_CurrentTex = texID;
    }
}

void OpenGLRenderer2D::PushQuad(const glm::vec2& pos, const glm::vec2& size,
                                const glm::vec2& uvMin, const glm::vec2& uvMax,
                                const glm::vec4& color, float mode)
{
    const glm::vec2 tl = pos;
    const glm::vec2 tr = { pos.x + size.x, pos.y };
    const glm::vec2 br = { pos.x + size.x, pos.y + size.y };
    const glm::vec2 bl = { pos.x,          pos.y + size.y };

    const glm::vec2 uvTL = uvMin;
    const glm::vec2 uvTR = { uvMax.x, uvMin.y };
    const glm::vec2 uvBR = uvMax;
    const glm::vec2 uvBL = { uvMin.x, uvMax.y };

    // Two triangles: (TL, BR, TR) and (TL, BL, BR).
    m_Verts.push_back({ tl, uvTL, color, mode });
    m_Verts.push_back({ br, uvBR, color, mode });
    m_Verts.push_back({ tr, uvTR, color, mode });
    m_Verts.push_back({ tl, uvTL, color, mode });
    m_Verts.push_back({ bl, uvBL, color, mode });
    m_Verts.push_back({ br, uvBR, color, mode });
}

void OpenGLRenderer2D::DrawQuad(const glm::vec2& pos, const glm::vec2& size,
                                const glm::vec4& color)
{
    SetTexture(m_WhiteTex);
    PushQuad(pos, size, glm::vec2(0.0f), glm::vec2(1.0f), color, 0.0f);
}

void OpenGLRenderer2D::DrawTexturedQuad(const glm::vec2& pos, const glm::vec2& size,
                                        const Texture& texture, const glm::vec4& tint,
                                        const glm::vec2& uvMin, const glm::vec2& uvMax)
{
    uint32_t id = NativeID(texture);
    if (id == 0) return;
    SetTexture(id);
    PushQuad(pos, size, uvMin, uvMax, tint, 0.0f);
}

float OpenGLRenderer2D::DrawText(const Font& font, const std::string& text,
                                 const glm::vec2& pos, const glm::vec4& color, float scale)
{
    const auto& atlas = font.Atlas();
    if (!atlas) return pos.x;
    SetTexture(NativeID(*atlas));

    float penX      = pos.x;
    float baselineY = pos.y + font.Ascent() * scale;

    for (char ch : text) {
        if (ch == '\n') {
            penX = pos.x;
            baselineY += font.LineHeight() * scale;
            continue;
        }
        const Glyph* g = font.GetGlyph(static_cast<uint32_t>(static_cast<unsigned char>(ch)));
        if (!g) continue;
        if (g->size.x > 0.0f && g->size.y > 0.0f) {
            const glm::vec2 qpos  = { penX + g->offset.x * scale, baselineY + g->offset.y * scale };
            const glm::vec2 qsize = g->size * scale;
            PushQuad(qpos, qsize, g->uvMin, g->uvMax, color, 1.0f);
        }
        penX += g->advance * scale;
    }
    return penX - pos.x;
}

void OpenGLRenderer2D::Flush()
{
    if (m_Verts.empty() || m_CurrentTex == 0) {
        m_Verts.clear();
        return;
    }

    glUseProgram(m_Program);
    glUniformMatrix4fv(glGetUniformLocation(m_Program, "uProj"), 1, GL_FALSE,
                       glm::value_ptr(m_Projection));
    glUniform1i(glGetUniformLocation(m_Program, "uTex"), 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_CurrentTex);

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

    m_Verts.clear();
}

} // namespace Diamond
