#pragma once

#include <memory>
#include <string>
#include "TextureData.h"
#include "Shader.h"

namespace Diamond {

// PBR material using the metallic-roughness workflow.
// Call Bind() before drawing a mesh to bind all texture slots and set sampler uniforms.
// Path strings mirror each texture slot — populated when a texture is assigned so the
// material can be serialized back to disk without losing its source asset references.
struct PBRMaterial {
    // Surface maps (bind slots 0–4)
    std::shared_ptr<Texture> Albedo;
    std::shared_ptr<Texture> Normal;
    std::shared_ptr<Texture> Metallic;
    std::shared_ptr<Texture> Roughness;
    std::shared_ptr<Texture> AO;

    // Emissive (bind slot 5)
    std::shared_ptr<Texture> Emissive;
    float EmissiveStrength = 0.0f;

    float UVScale = 1.0f;

    // Source paths — set these alongside the texture pointers so the material
    // can be saved and reloaded without re-scanning the asset directory.
    std::string AlbedoPath;
    std::string NormalPath;
    std::string MetallicPath;
    std::string RoughnessPath;
    std::string AOPath;
    std::string EmissivePath;

    // IBL maps (bind slots 6–8, set after environment pre-computation)
    // These are scene-level and not serialized per material.
    std::shared_ptr<Texture> IrradianceMap;
    std::shared_ptr<Texture> PrefilterMap;
    std::shared_ptr<Texture> BrdfLUT;

    void Bind(const Shader& shader) const;
};

} // namespace Diamond