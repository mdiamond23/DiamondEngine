#include "Renderer/ParticleRenderer.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLParticleRenderer.h"

namespace Diamond {

std::unique_ptr<ParticleRenderer> ParticleRenderer::Create()
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_unique<OpenGLParticleRenderer>();
        default:                       return nullptr;
    }
}

} // namespace Diamond
