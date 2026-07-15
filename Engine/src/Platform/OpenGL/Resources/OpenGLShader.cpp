#include "Platform/OpenGL/Resources/OpenGLShader.h"

#include <glad/gl.h>
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

namespace Diamond {

static std::string ReadFile(const std::string& path)
{
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        file.open(path);
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    } catch (const std::ifstream::failure&) {
        spdlog::error("OpenGLShader: failed to read '{}'", path);
        return {};
    }
}

// Compiles one stage; returns the shader object, or 0 on read/compile failure.
static uint32_t CompileStage(GLenum type, const std::string& path, const char* label)
{
    std::string source = ReadFile(path);
    if (source.empty()) return 0;
    const char* src = source.c_str();

    uint32_t shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, log);
        spdlog::error("Shader compile error [{}] in '{}': {}", label, path, log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

// Builds a linked program from the given stages (gsPath empty = no geometry
// stage). Returns 0 if any stage fails to read/compile or the program fails to
// link — leaving the caller's existing program untouched. This is what makes
// hot-reload safe: a broken edit never replaces a working program.
static uint32_t BuildProgram(const std::string& vsPath, const std::string& gsPath, const std::string& fsPath)
{
    bool hasGeom = !gsPath.empty();

    uint32_t vert = CompileStage(GL_VERTEX_SHADER, vsPath, "VERTEX");
    uint32_t geom = hasGeom ? CompileStage(GL_GEOMETRY_SHADER, gsPath, "GEOMETRY") : 0;
    uint32_t frag = CompileStage(GL_FRAGMENT_SHADER, fsPath, "FRAGMENT");

    uint32_t program = 0;
    if (vert && frag && (!hasGeom || geom)) {
        program = glCreateProgram();
        glAttachShader(program, vert);
        if (hasGeom) glAttachShader(program, geom);
        glAttachShader(program, frag);
        glLinkProgram(program);

        int linked = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (!linked) {
            char log[1024];
            glGetProgramInfoLog(program, 1024, nullptr, log);
            spdlog::error("Shader link error ('{}'): {}", fsPath, log);
            glDeleteProgram(program);
            program = 0;
        }
    }

    if (vert) glDeleteShader(vert);
    if (geom) glDeleteShader(geom);
    if (frag) glDeleteShader(frag);
    return program;
}

OpenGLShader::OpenGLShader(const std::string& vsPath, const std::string& fsPath)
    : m_VsPath(vsPath), m_FsPath(fsPath)
{
    m_RendererID = BuildProgram(m_VsPath, m_GsPath, m_FsPath);
}

OpenGLShader::OpenGLShader(const std::string& vsPath, const std::string& gsPath, const std::string& fsPath)
    : m_VsPath(vsPath), m_GsPath(gsPath), m_FsPath(fsPath)
{
    m_RendererID = BuildProgram(m_VsPath, m_GsPath, m_FsPath);
}

bool OpenGLShader::Reload()
{
    uint32_t program = BuildProgram(m_VsPath, m_GsPath, m_FsPath);
    if (!program) {
        spdlog::warn("OpenGLShader: reload failed for '{}' — keeping previous program", m_FsPath);
        return false;
    }
    if (m_RendererID) glDeleteProgram(m_RendererID);
    m_RendererID = program;
    spdlog::info("OpenGLShader: reloaded '{}'", m_FsPath);
    return true;
}

OpenGLShader::~OpenGLShader()
{
    glDeleteProgram(m_RendererID);
}

void OpenGLShader::Bind()   const { glUseProgram(m_RendererID); }
void OpenGLShader::Unbind() const { glUseProgram(0); }

static int GetLocation(uint32_t program, const std::string& name)
{
    int loc = glGetUniformLocation(program, name.c_str());
    if (loc == -1)
        spdlog::warn("OpenGLShader: uniform '{}' not found in program {}", name, program);
    return loc;
}

void OpenGLShader::SetBool(const std::string& name, bool value) const
    { glUniform1i(GetLocation(m_RendererID, name), (int)value); }

void OpenGLShader::SetInt(const std::string& name, int value) const
    { glUniform1i(GetLocation(m_RendererID, name), value); }

void OpenGLShader::SetFloat(const std::string& name, float value) const
    { glUniform1f(GetLocation(m_RendererID, name), value); }

void OpenGLShader::SetVec2(const std::string& name, const glm::vec2& v) const
    { glUniform2fv(GetLocation(m_RendererID, name), 1, &v[0]); }

void OpenGLShader::SetVec3(const std::string& name, const glm::vec3& v) const
    { glUniform3fv(GetLocation(m_RendererID, name), 1, &v[0]); }

void OpenGLShader::SetVec4(const std::string& name, const glm::vec4& v) const
    { glUniform4fv(GetLocation(m_RendererID, name), 1, &v[0]); }

void OpenGLShader::SetMat3(const std::string& name, const glm::mat3& mat) const
    { glUniformMatrix3fv(GetLocation(m_RendererID, name), 1, GL_FALSE, &mat[0][0]); }

void OpenGLShader::SetMat4(const std::string& name, const glm::mat4& mat) const
    { glUniformMatrix4fv(GetLocation(m_RendererID, name), 1, GL_FALSE, &mat[0][0]); }

void OpenGLShader::SetMat4Array(const std::string& name, const glm::mat4* mats, int count) const
    { glUniformMatrix4fv(GetLocation(m_RendererID, name), count, GL_FALSE, &mats[0][0][0]); }

} // namespace Diamond