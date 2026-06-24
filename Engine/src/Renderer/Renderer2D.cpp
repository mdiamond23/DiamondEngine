#include "Renderer/Renderer2D.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLRenderer2D.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Diamond {

glm::mat4 Renderer2D::OrthoProjection(float width, float height)
{
    // Top-left origin, +Y down: map x:[0,width] and y:[0,height] to clip space.
    return glm::ortho(0.0f, width, height, 0.0f, -1.0f, 1.0f);
}

std::unique_ptr<Renderer2D> Renderer2D::Create()
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_unique<OpenGLRenderer2D>();
        default:                       return nullptr;
    }
}

} // namespace Diamond
