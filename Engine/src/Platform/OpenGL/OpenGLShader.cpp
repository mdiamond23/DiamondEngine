#include "OpenGLShader.h"

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

OpenGLShader::OpenGLShader(const std::string& vsPath, const std::string& fsPath)
{
    std::string vsSource = ReadFile(vsPath);
    std::string fsSource = ReadFile(fsPath);
    const char* vsSrc = vsSource.c_str();
    const char* fsSrc = fsSource.c_str();

    uint32_t vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vsSrc, nullptr);
    glCompileShader(vert);
    CheckCompileErrors(vert, "VERTEX");

    uint32_t frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &fsSrc, nullptr);
    glCompileShader(frag);
    CheckCompileErrors(frag, "FRAGMENT");

    m_RendererID = glCreateProgram();
    glAttachShader(m_RendererID, vert);
    glAttachShader(m_RendererID, frag);
    glLinkProgram(m_RendererID);
    CheckCompileErrors(m_RendererID, "PROGRAM");

    glDeleteShader(vert);
    glDeleteShader(frag);
}

OpenGLShader::~OpenGLShader()
{
    glDeleteProgram(m_RendererID);
}

void OpenGLShader::Bind()   const { glUseProgram(m_RendererID); }
void OpenGLShader::Unbind() const { glUseProgram(0); }

void OpenGLShader::SetBool(const std::string& name, bool value) const
    { glUniform1i(glGetUniformLocation(m_RendererID, name.c_str()), (int)value); }

void OpenGLShader::SetInt(const std::string& name, int value) const
    { glUniform1i(glGetUniformLocation(m_RendererID, name.c_str()), value); }

void OpenGLShader::SetFloat(const std::string& name, float value) const
    { glUniform1f(glGetUniformLocation(m_RendererID, name.c_str()), value); }

void OpenGLShader::SetVec2(const std::string& name, const glm::vec2& v) const
    { glUniform2fv(glGetUniformLocation(m_RendererID, name.c_str()), 1, &v[0]); }

void OpenGLShader::SetVec3(const std::string& name, const glm::vec3& v) const
    { glUniform3fv(glGetUniformLocation(m_RendererID, name.c_str()), 1, &v[0]); }

void OpenGLShader::SetMat3(const std::string& name, const glm::mat3& mat) const
    { glUniformMatrix3fv(glGetUniformLocation(m_RendererID, name.c_str()), 1, GL_FALSE, &mat[0][0]); }

void OpenGLShader::SetMat4(const std::string& name, const glm::mat4& mat) const
    { glUniformMatrix4fv(glGetUniformLocation(m_RendererID, name.c_str()), 1, GL_FALSE, &mat[0][0]); }

void OpenGLShader::CheckCompileErrors(uint32_t shader, const std::string& type)
{
    int  success;
    char log[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, nullptr, log);
            spdlog::error("Shader compile error [{}]: {}", type, log);
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, nullptr, log);
            spdlog::error("Shader link error: {}", log);
        }
    }
}

} // namespace Diamond