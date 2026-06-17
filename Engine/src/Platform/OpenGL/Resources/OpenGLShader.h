#pragma once

#include "Renderer/Shader.h"
#include <cstdint>

namespace Diamond {

class OpenGLShader : public Shader {
public:
    OpenGLShader(const std::string& vsPath, const std::string& fsPath);
    OpenGLShader(const std::string& vsPath, const std::string& gsPath, const std::string& fsPath);
    ~OpenGLShader() override;

    void Bind()   const override;
    void Unbind() const override;

    void SetBool (const std::string& name, bool value)           const override;
    void SetInt  (const std::string& name, int value)            const override;
    void SetFloat(const std::string& name, float value)          const override;
    void SetVec2 (const std::string& name, const glm::vec2& v)   const override;
    void SetVec3 (const std::string& name, const glm::vec3& v)   const override;
    void SetMat3 (const std::string& name, const glm::mat3& mat) const override;
    void SetMat4 (const std::string& name, const glm::mat4& mat) const override;
    void SetMat4Array(const std::string& name, const glm::mat4* mats, int count) const override;

    // Recompiles from the stored source paths and swaps the GL program in place
    // (so existing references stay valid — the basis for hot-reload). On compile/
    // link failure the previous program is kept and the error logged; returns
    // whether the reload succeeded.
    bool Reload();

    // Source paths, exposed so a shader library can watch them for changes.
    const std::string& GetVertexPath()   const { return m_VsPath; }
    const std::string& GetGeometryPath() const { return m_GsPath; }   // empty if none
    const std::string& GetFragmentPath() const { return m_FsPath; }

private:
    uint32_t    m_RendererID = 0;
    std::string m_VsPath;
    std::string m_GsPath;   // empty when the shader has no geometry stage
    std::string m_FsPath;
};

} // namespace Diamond
