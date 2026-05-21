#pragma once

#include "Renderer/Shader.h"
#include <cstdint>

namespace Diamond {

class OpenGLShader : public Shader {
public:
    OpenGLShader(const std::string& vsPath, const std::string& fsPath);
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

private:
    uint32_t m_RendererID = 0;
    void CheckCompileErrors(uint32_t shader, const std::string& type);
};

} // namespace Diamond
