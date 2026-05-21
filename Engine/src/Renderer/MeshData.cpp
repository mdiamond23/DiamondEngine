#include "Renderer/MeshData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/OpenGLMesh.h"

namespace Diamond {

std::shared_ptr<Mesh> Mesh::Create(const MeshData& data)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLMesh>(data);
        default:                       return nullptr;
    }
}

} // namespace Diamond