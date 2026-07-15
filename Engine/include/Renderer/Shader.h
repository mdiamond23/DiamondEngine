#pragma once

#include <string>
#include <memory>
#include <glm/glm.hpp>

namespace Diamond {

// Abstract shader program. Backend-specific instances are created via Shader::Create().
class Shader {
public:
    virtual ~Shader() = default;

    virtual void Bind()   const = 0;
    virtual void Unbind() const = 0;

    virtual void SetBool (const std::string& name, bool value)            const = 0;
    virtual void SetInt  (const std::string& name, int value)             const = 0;
    virtual void SetFloat(const std::string& name, float value)           const = 0;
    virtual void SetVec2 (const std::string& name, const glm::vec2& v)    const = 0;
    virtual void SetVec3 (const std::string& name, const glm::vec3& v)    const = 0;
    virtual void SetVec4 (const std::string& name, const glm::vec4& v)    const = 0;
    virtual void SetMat3 (const std::string& name, const glm::mat3& mat)  const = 0;
    virtual void SetMat4 (const std::string& name, const glm::mat4& mat)  const = 0;
    virtual void SetMat4Array(const std::string& name, const glm::mat4* mats, int count) const = 0;

    static std::shared_ptr<Shader> Create(const std::string& vsPath, const std::string& fsPath);
};

} // namespace Diamond