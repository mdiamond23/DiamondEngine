#pragma once
#include <Renderer/MeshData.h>
#include <Renderer/Shader.h>
#include <Renderer/Material.h>
#include <memory>
#include <cstdint>
#include <vector>

class MeshThumbnailRenderer {
public:
    static constexpr int kSize = 128;

    MeshThumbnailRenderer();
    ~MeshThumbnailRenderer();

    // Renders all sub-meshes into a new GL texture and returns its ID.
    // Returns 0 on failure. The caller owns the texture and must glDeleteTextures it when done.
    uint32_t Render(const std::vector<Diamond::MeshData>& meshes);

    // Renders a UV sphere shaded with `mat` (lightweight PBR, no IBL) — the
    // material-ball preview. Same ownership contract as Render().
    uint32_t RenderMaterial(const Diamond::PBRMaterial& mat);

    bool IsReady() const { return m_Shader != nullptr; }

private:
    uint32_t m_FBO   = 0;
    uint32_t m_Depth = 0;
    std::shared_ptr<Diamond::Shader> m_Shader;
    std::shared_ptr<Diamond::Shader> m_MaterialShader;
    std::shared_ptr<Diamond::Mesh>   m_Sphere;   // lazily built unit sphere for previews
};
