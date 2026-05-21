#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/OpenGLRendererAPI.h"

namespace Diamond {

RendererAPI::API RendererAPI::s_API = RendererAPI::API::OpenGL;

std::unique_ptr<RendererAPI> RendererAPI::Create()
{
    switch (s_API) {
        case API::OpenGL: return std::make_unique<OpenGLRendererAPI>();
        default:          return nullptr;
    }
}

} // namespace Diamond
