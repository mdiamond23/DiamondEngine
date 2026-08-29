#pragma once

#include <memory>
#include <string>
#include <glm/glm.hpp>
#include "TextureData.h"
#include "Shader.h"

namespace Diamond {

// glTF alpha handling. Opaque and Mask render through the deferred G-buffer
// (Mask alpha-tests against AlphaCutoff there); only Blend routes to the
// forward transparency pass, since it's the only mode that needs sorting.
enum class AlphaMode : uint8_t { Opaque, Mask, Blend };

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

    // Constant base color, multiplied into the albedo sample (or standing alone
    // when Albedo is unassigned, against the default-white fallback). glTF's
    // pbrMetallicRoughness.baseColorFactor — linear, not sRGB.
    glm::vec4 BaseColorFactor { 1.0f };

    // Constant metallic/roughness, used when the corresponding MAP IS ABSENT.
    // A material that has the map ignores these and takes the texture as-is —
    // unlike glTF, where the factor always multiplies. The distinction is
    // deliberate: it means these can default to the values an untextured
    // material has always rendered with, so adding them changed nothing about
    // any existing material while still giving an untextured one a real knob.
    float MetallicFactor  = 0.0f;
    float RoughnessFactor = 0.5f;

    // Cutoff only applies when Mode == Mask: fragments whose base-color alpha
    // (texture.a * BaseColorFactor.a) falls below it are discarded in the
    // G-buffer pass.
    AlphaMode Mode        = AlphaMode::Opaque;
    float     AlphaCutoff = 0.5f;

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