#include "Platform/OpenGL/Resources/OpenGLRendererAPI.h"

#include <glad/gl.h>

namespace Diamond {

void OpenGLRendererAPI::Init()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
}

void OpenGLRendererAPI::SetClearColor(const glm::vec4& color)
{
    glClearColor(color.r, color.g, color.b, color.a);
}

void OpenGLRendererAPI::Clear()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRendererAPI::SetViewport(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
{
    glViewport(static_cast<int>(x), static_cast<int>(y),
               static_cast<int>(width), static_cast<int>(height));
}

} // namespace Diamond