#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include "Renderer/Material.h"
#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace Diamond {

class OpenGLGBufferPass {
public:
    // Renders all draw calls into the G-buffer FBO.
    // fbo must already have gViewPos/gViewNormal/gAlbedo/gMaterial attached
    // (COLOR_ATTACHMENT 0-3) and a depth renderbuffer.
    // Each draw picks `skinnedShader` (and uploads its bone palette) when it has
    // one, otherwise `staticShader`; both share the same gbuffer fragment stage.
    void Render(const OpenGLShader&          staticShader,
                const OpenGLShader&          skinnedShader,
                const std::vector<DrawCall>& draws,
                const PBRMaterial&           material,
                const glm::mat4&             view,
                const glm::mat4&             projection,
                uint32_t                     fbo,
                int viewportW, int viewportH);
};

} // namespace Diamond
