#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace Diamond {

class OpenGLTransparencyPass {
public:
    void Render(const OpenGLShader&          shader,
                std::vector<DrawCall>        draws,      // by value — sorted internally
                uint32_t                     albedoTex,
                uint32_t                     gBufferFBO,
                uint32_t                     hdrFBO,
                const glm::mat4&             view,
                const glm::mat4&             projection,
                const glm::vec3&             cameraPos,
                int viewportW, int viewportH);
};

} // namespace Diamond
