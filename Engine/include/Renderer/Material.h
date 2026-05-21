#pragma once

#include <memory>
#include "TextureData.h"
#include "Shader.h"

namespace Diamond {

// PBR material using the metallic-roughness workflow.
// Call Bind() before drawing a mesh to bind all texture slots and set sampler uniforms.
struct PBRMaterial {
    // Surface maps (bind slots 0–4)
    std::shared_ptr<Texture> Albedo;
    std::shared_ptr<Texture> Normal;
    std::shared_ptr<Texture> Metallic;
    std::shared_ptr<Texture> Roughness;
    std::shared_ptr<Texture> AO;

    // IBL maps (bind slots 5–7, set after environment pre-computation)
    std::shared_ptr<Texture> IrradianceMap;
    std::shared_ptr<Texture> PrefilterMap;
    std::shared_ptr<Texture> BrdfLUT;

    void Bind(const Shader& shader) const;
};

} // namespace Diamond