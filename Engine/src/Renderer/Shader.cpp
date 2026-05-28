#include "Renderer/Shader.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLShader.h"

namespace Diamond {

std::shared_ptr<Shader> Shader::Create(const std::string& vsPath, const std::string& fsPath)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLShader>(vsPath, fsPath);
        default:                       return nullptr;
    }
}

} // namespace Diamond
